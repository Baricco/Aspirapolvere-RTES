/// Lo stato dell'app con Riverpod (docs/app.md, sezione 10.1): ogni dato è un provider che si attiva quando una
/// schermata lo guarda e si ferma quando nessuno lo guarda più.
library;

import 'dart:async';

import 'package:dio/dio.dart';
import 'package:flutter_riverpod/flutter_riverpod.dart';

import '../config/app_config.dart';
import '../map/rtm.dart';
import '../map/trail.dart';
import '../models/log_entry.dart';
import '../models/map_info.dart';
import '../models/robot_state.dart';
import '../models/schedule.dart';
import 'api_client.dart';

final apiClientProvider = Provider<ApiClient>((ref) => ApiClient());

// ------------------------------------------------------------------------------------------------ sessione

enum SessionStatus { unknown, loggedOut, loggedIn }

class Session {
  const Session(this.status, [this.username]);
  final SessionStatus status;
  final String? username;
}

class SessionNotifier extends Notifier<Session> {
  @override
  Session build() {
    final api = ref.watch(apiClientProvider);
    api.onUnauthenticated = () => state = const Session(SessionStatus.loggedOut);
    Future.microtask(_open);
    return const Session(SessionStatus.unknown);
  }

  /// All'apertura: rinnova il JWT; se la sessione è scaduta (o non c'è) si va al login.
  Future<void> _open() async {
    final api = ref.read(apiClientProvider);
    await api.loadToken();
    if (!api.hasToken) {
      state = const Session(SessionStatus.loggedOut);
      return;
    }
    try {
      await api.refresh();
      state = Session(SessionStatus.loggedIn, await api.me());
    } on ApiException catch (error) {
      if (error.isUnauthenticated) {
        state = const Session(SessionStatus.loggedOut);
      } else {
        // il server non risponde: si riprova, senza buttare la sessione
        await Future<void>.delayed(retryDelay);
        if (state.status == SessionStatus.unknown) return _open();
      }
    }
  }

  Future<void> login(String username, String password) async {
    final api = ref.read(apiClientProvider);
    await api.login(username, password);
    state = Session(SessionStatus.loggedIn, await api.me());
  }

  Future<void> logout() async {
    await ref.read(apiClientProvider).logout();
    state = const Session(SessionStatus.loggedOut);
  }
}

final sessionProvider = NotifierProvider<SessionNotifier, Session>(SessionNotifier.new);

// --------------------------------------------------------------------------------------------- notifiche

/// La notifica in alto, in qualsiasi schermata: i rifiuti dei comandi con il loro message.
class AppNotification {
  AppNotification(this.title, this.message) : id = _nextId++;
  static int _nextId = 0;
  final int id;
  final String title;
  final String message;
}

class NotificationNotifier extends Notifier<AppNotification?> {
  Timer? _timer;

  @override
  AppNotification? build() {
    ref.onDispose(() => _timer?.cancel());
    return null;
  }

  void show(String title, String message) {
    _timer?.cancel();
    state = AppNotification(title, message);
    _timer = Timer(notificationDuration, () => state = null);
  }

  void hide() {
    _timer?.cancel();
    state = null;
  }
}

final notificationProvider = NotifierProvider<NotificationNotifier, AppNotification?>(NotificationNotifier.new);

/// Esegue una richiesta che può essere rifiutata: il rifiuto diventa la notifica in alto. Ritorna true se è riuscita.
Future<bool> runCommand(WidgetRef ref, Future<void> Function(ApiClient api) command,
    {String title = 'Comando rifiutato'}) async {
  try {
    await command(ref.read(apiClientProvider));
    return true;
  } on ApiException catch (error) {
    if (!error.isUnauthenticated) {
      ref.read(notificationProvider.notifier).show(
            error.status == 409 ? title : 'Comando non riuscito',
            error.message.isNotEmpty ? error.message : error.reason,
          );
    }
    return false;
  }
}

// -------------------------------------------------------------------------------------- stato del robot

/// L'app è in primo piano: in background non si fa nessuna richiesta.
class AppActiveNotifier extends Notifier<bool> {
  @override
  bool build() => true;
  void set(bool active) => state = active;
}

final appActiveProvider = NotifierProvider<AppActiveNotifier, bool>(AppActiveNotifier.new);

/// Intervallo minimo fra due stati, scelto dalla schermata aperta: 100 ms con la mappa, 1000 ms nelle altre.
class StateIntervalNotifier extends Notifier<int> {
  @override
  int build() => stateIntervalOtherMs;
  void set(int milliseconds) => state = milliseconds;
}

final stateIntervalProvider = NotifierProvider<StateIntervalNotifier, int>(StateIntervalNotifier.new);

/// Lo stato del robot: una richiesta lunga continua (docs/app.md, sezione 4.4), ferma in background.
class RobotStateNotifier extends Notifier<RobotState?> {
  CancelToken? _cancelToken;
  bool _disposed = false;

  @override
  RobotState? build() {
    final active = ref.watch(appActiveProvider);
    final loggedIn = ref.watch(sessionProvider.select((session) => session.status == SessionStatus.loggedIn));
    _disposed = false;
    ref.onDispose(() {
      _disposed = true;
      _cancelToken?.cancel();
    });
    if (active && loggedIn) Future.microtask(_loop);
    return stateOrNull;
  }

  Future<void> _loop() async {
    final api = ref.read(apiClientProvider);
    final cancelToken = _cancelToken = CancelToken();
    int? after;
    while (!_disposed && !cancelToken.isCancelled) {
      try {
        final json = await api.getState(
            after: after, minIntervalMs: ref.read(stateIntervalProvider), cancelToken: cancelToken);
        if (_disposed || cancelToken.isCancelled) return;
        // 304: non è cambiato niente, si rimanda subito la stessa richiesta
        if (json != null) {
          final robotState = RobotState(json);
          after = robotState.stateNumber;
          state = robotState;
        }
      } on ApiException catch (error) {
        if (_disposed || cancelToken.isCancelled || error.isUnauthenticated) return;
        await Future<void>.delayed(retryDelay);
      }
    }
  }
}

final robotStateProvider = NotifierProvider.autoDispose<RobotStateNotifier, RobotState?>(RobotStateNotifier.new);

// ----------------------------------------------------------------------------------- mappa dal vivo e scia

/// Una fotografia della mappa dal vivo: la griglia è sempre la stessa, cambia il numero quando viene aggiornata.
class LiveMapSnapshot {
  LiveMapSnapshot(this.map, this.revision);
  final LiveMap map;
  final int revision;
}

/// La mappa dal vivo: all'apertura quella intera, poi solo le differenze quando map_version cambia.
class LiveMapNotifier extends Notifier<LiveMapSnapshot?> {
  final LiveMap _map = LiveMap();
  bool _busy = false;
  int _revision = 0;

  @override
  LiveMapSnapshot? build() {
    ref.listen(robotStateProvider.select((robotState) => robotState?.mapVersion), (previous, next) => _update(),
        fireImmediately: true);
    return stateOrNull;
  }

  Future<void> _update() async {
    final wanted = ref.read(robotStateProvider)?.mapVersion;
    if (_busy || wanted == null || wanted == _map.version) return;
    _busy = true;
    try {
      final message = await ref.read(apiClientProvider).getBytes('/api/robot/map',
          query: {if (_map.version != null) 'after': _map.version});
      _map.applyMessage(message);
      if (!ref.mounted) return;
      state = LiveMapSnapshot(_map, ++_revision);
    } on ApiException {
      // si riprova al prossimo cambio di versione
    } on MapFormatException {
      _map.version = null; // la prossima volta si richiede la mappa intera
    } finally {
      _busy = false;
    }
    // nel frattempo può essere arrivata un'altra versione
    if (ref.mounted && ref.read(robotStateProvider)?.mapVersion != _map.version) unawaited(_update());
  }
}

final liveMapProvider = NotifierProvider.autoDispose<LiveMapNotifier, LiveMapSnapshot?>(LiveMapNotifier.new);

class TrailSnapshot {
  TrailSnapshot(this.trail, this.revision);
  final Trail trail;
  final int revision;
}

/// La scia dell'ultima missione: tutta all'apertura, poi i punti nuovi mentre il robot si muove.
class TrailNotifier extends Notifier<TrailSnapshot?> {
  final Trail _trail = Trail();
  Timer? _timer;
  bool _busy = false;
  int _revision = 0;
  int? _lastStateNumber;

  @override
  TrailSnapshot? build() {
    _timer = Timer.periodic(trailRefreshInterval, (_) => _update());
    ref.onDispose(() => _timer?.cancel());
    Future.microtask(_update);
    return stateOrNull;
  }

  Future<void> _update() async {
    final robotState = ref.read(robotStateProvider);
    // senza stati nuovi il robot è fermo: la scia non è cresciuta
    if (_busy || robotState == null || (robotState.stateNumber == _lastStateNumber && _trail.trailId != null)) return;
    _busy = true;
    _lastStateNumber = robotState.stateNumber;
    try {
      final message = await ref.read(apiClientProvider).getJson('/api/robot/trail',
          query: {if (_trail.trailId != null) 'after': _trail.points.length});
      final before = _trail.points.length;
      final beforeId = _trail.trailId;
      _trail.applyMessage(message as Map<String, dynamic>);
      if (ref.mounted && (_trail.points.length != before || _trail.trailId != beforeId)) {
        state = TrailSnapshot(_trail, ++_revision);
      }
    } on ApiException {
      // si riprova al prossimo giro
    } finally {
      _busy = false;
    }
  }
}

final trailProvider = NotifierProvider.autoDispose<TrailNotifier, TrailSnapshot?>(TrailNotifier.new);

// ------------------------------------------------------------------------------- mappe, orari, registro

/// Le mappe salvate sul server. Si ricaricano quando il file sul robot cambia (un salvataggio può creare una mappa).
final mapsProvider = FutureProvider.autoDispose<List<MapInfo>>((ref) async {
  ref.watch(robotStateProvider.select((robotState) => robotState?.mapFileCrc));
  final list = await ref.watch(apiClientProvider).getJson('/api/robot/maps') as List;
  return [for (final map in list) MapInfo.fromJson(map as Map<String, dynamic>)];
});

final schedulesProvider = FutureProvider.autoDispose<List<Schedule>>((ref) async {
  final list = await ref.watch(apiClientProvider).getJson('/api/robot/schedules') as List;
  return [for (final schedule in list) Schedule.fromJson(schedule as Map<String, dynamic>)];
});

/// Dopo un cambio di mappa il robot si riavvia: fino a quando l'app mostra "riavvio in corso".
class RestartNotifier extends Notifier<bool> {
  Timer? _timer;

  @override
  bool build() {
    ref.onDispose(() => _timer?.cancel());
    // il robot è ripartito quando il suo tempo dall'avvio torna indietro
    ref.listen(robotStateProvider.select((robotState) => robotState?.uptimeS), (previous, next) {
      if (state && previous != null && next != null && next < previous) _finish();
    });
    return false;
  }

  void start() {
    _timer?.cancel();
    state = true;
    _timer = Timer(restartNoticeDuration, _finish);
  }

  void _finish() {
    _timer?.cancel();
    state = false;
  }
}

final restartProvider = NotifierProvider<RestartNotifier, bool>(RestartNotifier.new);

Future<List<LogEntry>> fetchLog(ApiClient api, {int? before}) async {
  final list = await api.getJson('/api/robot/log', query: {'limit': logPageSize, 'before': ?before})
      as List;
  return [for (final entry in list) LogEntry.fromJson(entry as Map<String, dynamic>)];
}
