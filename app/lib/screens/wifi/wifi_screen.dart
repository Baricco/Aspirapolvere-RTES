/// Rete Wi-Fi del robot (docs/wifi.md, sezione 10): reti vicine viste dal robot, scelta della rete con la password,
/// esito del cambio. Solo nell'APK (nella versione web il pulsante non c'è).
///
/// Con chi parla: robot Online → backend; robot Offline → il robot sulla sua rete (192.168.5.1), se il telefono ci è
/// collegato; altrimenti la schermata spiega come collegarsi a Maurice-WiFi.
library;

import 'dart:async';

import 'package:flutter/material.dart';
import 'package:flutter_riverpod/flutter_riverpod.dart';

import '../../api/api_client.dart';
import '../../api/providers.dart';
import '../../api/wifi_api.dart';
import '../../config/app_colors.dart';
import '../../config/app_config.dart';
import '../../models/robot_state.dart';
import '../../models/wifi.dart';
import '../../widgets/glass_panel.dart';
import '../../widgets/robot_badges.dart';
import '../../widgets/state_interval.dart';

/// Strada delle richieste.
enum _Route { checking, backend, local, none }

/// Il cambio di rete chiesto da questa schermata.
class _Change {
  _Change(this.ssid, this.number, this.isFromRobotNetwork);

  final String ssid;
  final int number;
  final bool isFromRobotNetwork; // chiesto con il telefono sulla rete del robot
  String? result; // esito di rtes-wifi, null finché non arriva
  bool isTimedOut = false; // il robot non è tornato online entro wifiChangeWait
}

class WifiScreen extends ConsumerStatefulWidget {
  const WifiScreen({super.key});

  @override
  ConsumerState<WifiScreen> createState() => _WifiScreenState();
}

class _WifiScreenState extends ConsumerState<WifiScreen> with StateIntervalMixin {
  @override
  int get stateIntervalMs => stateIntervalOtherMs;

  final LocalWifiApi _local = LocalWifiApi();
  late final PendingWifiChangeNotifier _pending = ref.read(pendingWifiChangeProvider.notifier);

  _Route _route = _Route.checking;
  String? _wifiKey; // modo e rete dell'ultimo stato visto: quando cambiano si ricarica la lista
  WifiStatus? _localStatus; // ultimo stato avuto dal robot sulla sua rete
  int _localMisses = 0; // controlli di fila senza risposta dal robot sulla sua rete
  WifiScan? _scan;
  bool _isScanning = false;
  bool _isRescanWanted = false; // un'altra ricarica chiesta mentre ce n'era già una in corso
  String? _scanError;
  _Change? _change;
  Timer? _checkTimer, _timeoutTimer;
  Future<void>? _check; // controllo della strada in corso
  bool _isRetrying = false;
  String? _retryError;

  WifiApi get _api => _route == _Route.local ? _local : BackendWifiApi(ref.read(apiClientProvider));

  @override
  void initState() {
    super.initState();
    Future.microtask(_checkRoute);
    // la schermata segue sempre lo stato vero: con chi parlare, la rete di adesso, l'esito di un cambio
    _checkTimer = Timer.periodic(wifiRouteCheckInterval, (_) => _checkRoute());
  }

  @override
  void dispose() {
    _checkTimer?.cancel();
    _timeoutTimer?.cancel();
    // un cambio ancora senza esito: lo mostrerà la schermata principale quando arriva
    if (_change != null && _change!.result == null) {
      _pending.screenClosed();
    } else {
      _pending.clear();
    }
    WifiNetworkBinding.unbind();
    super.dispose();
  }

  // --------------------------------------------------------------------------------------------- strada

  /// Decide con chi parlare: robot Online → backend; altrimenti il robot sulla sua rete, se il telefono ci è
  /// collegato; altrimenti nessuno. Si ripete ogni wifiRouteCheckInterval e a ogni cambio di Online. Con un controllo
  /// già in corso ritorna quello.
  Future<void> _checkRoute() => _check ??= _runCheck().whenComplete(() => _check = null);

  Future<void> _runCheck() async {
    if (!mounted) return;
    final robot = ref.read(robotStateProvider);
    if (robot?.online == true) {
      if (_route != _Route.backend) {
        await WifiNetworkBinding.unbind();
        _setRoute(_Route.backend);
      }
      _noteWifi(robot!.wifi);
      return;
    }
    final status = await _local.status();
    if (!mounted) return;
    // nel frattempo il robot potrebbe essere tornato online
    if (ref.read(robotStateProvider)?.online == true) return;
    if (status == null) {
      setState(() => _localMisses++);
      // sulla rete del robot la schermata resta com'è finché i controlli senza risposta non sono wifiLocalMisses
      if (_route != _Route.local || _localMisses >= wifiLocalMisses) _setRoute(_Route.none);
      return;
    }
    setState(() {
      _localMisses = 0;
      _localStatus = status;
      _retryError = null;
    });
    _setRoute(_Route.local);
    _finishChange(status);
    _noteWifi(status);
  }

  /// "Riprova": controllo della strada con la rotellina, e un testo se il robot non risponde.
  Future<void> _retry() async {
    setState(() {
      _isRetrying = true;
      _retryError = null;
    });
    await _checkRoute();
    if (!mounted) return;
    setState(() {
      _isRetrying = false;
      if (_route == _Route.none) {
        _retryError = 'Maurice non risponde su ${robotLocalUrl.replaceFirst('http://', '')}: il telefono è collegato '
            'a «$robotWifiSsid»?';
      }
    });
  }

  void _setRoute(_Route route) {
    if (!mounted || route == _route) return;
    setState(() {
      _route = route;
      _scan = null;
      _scanError = null;
      _wifiKey = null;
    });
    if (route == _Route.backend || route == _Route.local) _loadScan();
  }

  /// Modo e rete del Wi-Fi: se cambiano (cambio di rete, rete persa, rete del robot accesa o spenta) la lista si
  /// ricarica, così la rete "In uso" è sempre quella vera.
  void _noteWifi(WifiStatus? status) {
    if (status == null) return;
    final key = '${status.mode}|${status.ssid}';
    if (key == _wifiKey) return;
    final isFirst = _wifiKey == null;
    _wifiKey = key;
    if (!isFirst) _loadScan();
  }

  // ------------------------------------------------------------------------------------------- scansione

  Future<void> _loadScan() async {
    if (_isScanning) {
      _isRescanWanted = true;
      return;
    }
    final route = _route;
    setState(() {
      _isScanning = true;
      _scanError = null;
    });
    try {
      for (var attempt = 0;; attempt++) {
        try {
          final scan = await _api.scan();
          if (mounted && _route == route) setState(() => _scan = scan);
          return;
        } on ApiException catch (error) {
          // subito dopo un cambio di rete il robot può non rispondere ancora: si riprova
          final isTemporary = error.status == null || error.status == 503 || error.status == 504;
          if (!mounted || _route != route) return;
          if (!isTemporary || attempt + 1 >= wifiScanAttempts) {
            setState(() => _scanError = error.message.isNotEmpty ? error.message : 'Ricerca delle reti non riuscita.');
            return;
          }
          await Future<void>.delayed(wifiScanRetryDelay);
          if (!mounted || _route != route) return;
        }
      }
    } finally {
      if (mounted) setState(() => _isScanning = false);
      if (mounted && _isRescanWanted) {
        _isRescanWanted = false;
        unawaited(_loadScan());
      }
    }
  }

  // ---------------------------------------------------------------------------------------------- cambio

  Future<void> _connect(String ssid, bool isProtected) async {
    // il cambio va alla strada che c'era al tocco
    final route = _route;
    final api = _api;
    final password = isProtected ? await _askPassword(context, ssid) : '';
    if (password == null || !mounted) return;
    if (_route != route) {
      ref.read(notificationProvider.notifier).show('Cambio non partito', 'Maurice non risponde più da qui: riprova.');
      return;
    }
    final isFromRobotNetwork = route == _Route.local;
    try {
      final number = await api.connect(ssid, isProtected, password);
      if (!mounted) return;
      setState(() => _change = _Change(ssid, number, isFromRobotNetwork));
      _pending.start(number, ssid);
      _timeoutTimer?.cancel();
      _timeoutTimer = Timer(wifiChangeWait, () {
        if (mounted && _change?.number == number && _change?.result == null) setState(() => _change!.isTimedOut = true);
      });
    } on ApiException catch (error) {
      if (!error.isUnauthenticated) {
        ref.read(notificationProvider.notifier).show(
            error.status == 409 ? 'Cambio rifiutato' : 'Cambio non riuscito',
            error.message.isNotEmpty ? error.message : error.reason);
      }
    }
  }

  /// Esito arrivato (dallo stato del robot o da rtes-wifi sulla sua rete). La lista si ricarica comunque da
  /// _noteWifi, perché un cambio riuscito cambia la rete in uso.
  void _finishChange(WifiStatus status) {
    final change = _change;
    if (change == null || change.result != null || status.change.number != change.number || !status.change.isDone) return;
    setState(() => change.result = status.change.result);
    _pending.clear();
    _timeoutTimer?.cancel();
  }

  // ---------------------------------------------------------------------------------------------- disegno

  @override
  Widget build(BuildContext context) {
    final robot = ref.watch(robotStateProvider);
    ref.listen<RobotState?>(robotStateProvider, (previous, next) {
      if (next?.online == true) {
        if (next!.wifi != null) _finishChange(next.wifi!);
        if (_route == _Route.backend) _noteWifi(next.wifi);
      }
      // online o offline cambiato: si ricontrolla subito con chi parlare
      if ((previous?.online == true) != (next?.online == true)) _checkRoute();
    });

    final canRefresh = !_isScanning && (_route == _Route.backend || _route == _Route.local);
    final isResting = _route != _Route.backend || robot?.navigationState == 'sleep';
    final isBusy = _change != null && _change!.result == null && !_change!.isTimedOut;
    // durante un cambio, finché Maurice non si fa sentire da nessuna strada si aspetta
    final isWaitingRobot = isBusy && (_route == _Route.none || _route == _Route.checking);

    return Scaffold(
      appBar: ScreenHeader(title: 'Rete Wi-Fi', trailing: [
        IconButton(
          icon: const Icon(Icons.refresh_rounded),
          tooltip: 'Aggiorna',
          onPressed: canRefresh ? _loadScan : null,
        ),
        OnlineBadge(state: robot),
      ]),
      body: ListView(padding: const EdgeInsets.fromLTRB(screenPadding, 4, screenPadding, 24), children: [
        _StatusPanel(
          route: isWaitingRobot ? _Route.checking : _route,
          robot: robot,
          localStatus: _localStatus,
          isLocalSilent: _route == _Route.local && _localMisses > 0,
          isRetrying: _isRetrying,
          retryError: _retryError,
          onRetry: _retry,
        ),
        if (_change != null) ...[
          const SizedBox(height: 12),
          _ChangePanel(change: _change!, onClose: () => setState(() => _change = null)),
        ],
        if (!isWaitingRobot && (_route == _Route.backend || _route == _Route.local)) ...[
          const SizedBox(height: 12),
          _NetworksPanel(
            scan: _scan,
            isScanning: _isScanning,
            error: _scanError,
            disabledReason: !isResting
                ? 'Maurice deve essere a riposo per cambiare rete.'
                : isBusy
                    ? 'Aspetta la fine del cambio di rete.'
                    : null,
            onSelect: _connect,
            onOther: () async {
              final network = await _askOtherNetwork(context);
              if (network != null) await _connect(network.$1, network.$2);
            },
          ),
        ],
      ]),
    );
  }
}

/// In alto: con chi parla l'app e la rete di adesso, oppure come collegarsi alla rete del robot.
class _StatusPanel extends StatelessWidget {
  const _StatusPanel({
    required this.route,
    required this.robot,
    required this.localStatus,
    required this.isLocalSilent,
    required this.isRetrying,
    required this.retryError,
    required this.onRetry,
  });

  final _Route route;
  final RobotState? robot;
  final WifiStatus? localStatus; // stato del robot sulla sua rete
  final bool isLocalSilent; // sulla rete del robot l'ultimo controllo non ha avuto risposta
  final bool isRetrying;
  final String? retryError;
  final VoidCallback onRetry;

  /// Cosa fa il collegamento del robot, visto dalla sua rete.
  static String _localText(WifiStatus? status) {
    if (status == null) return 'Scegli la rete Wi-Fi a cui Maurice deve collegarsi.';
    if (status.mode == 'testing') return 'Maurice sta provando una rete nuova.';
    if (status.mode == 'station' && status.ssid != null) return 'Maurice è collegato a «${status.ssid}».';
    if (status.savedSsid == null) return 'Nessuna rete salvata: scegli la rete a cui Maurice deve collegarsi.';
    return 'Maurice cerca «${status.savedSsid}». Puoi sceglierne un\'altra qui sotto.';
  }

  @override
  Widget build(BuildContext context) {
    final palette = AppPalette.of(context);
    final secondary = TextStyle(color: palette.textSecondary, fontSize: 14);

    switch (route) {
      case _Route.checking:
        return const GlassPanel(
          child: Row(children: [
            SizedBox(width: 22, height: 22, child: CircularProgressIndicator(strokeWidth: 2.5)),
            SizedBox(width: 14),
            Expanded(child: Text('Aspetto Maurice…', style: TextStyle(fontSize: 16))),
          ]),
        );
      case _Route.backend:
        final wifi = robot?.wifi;
        return GlassPanel(
          child: Row(children: [
            Icon(wifi?.signalDbm != null ? _signalIcon(wifi!.signalDbm!) : Icons.wifi_rounded, color: palette.primary, size: 30),
            const SizedBox(width: 14),
            Expanded(
              child: Column(crossAxisAlignment: CrossAxisAlignment.start, children: [
                Text(wifi?.ssid != null ? 'Collegato a «${wifi!.ssid}»' : 'Maurice è online',
                    style: const TextStyle(fontSize: 17, fontWeight: FontWeight.w700)),
                Text(
                    wifi == null
                        ? 'Il programma del Wi-Fi del robot non manda il suo stato.'
                        : [if (wifi.signalDbm != null) '${wifi.signalDbm} dBm', if (wifi.ip != null) wifi.ip!].join(' · '),
                    style: secondary),
                if (wifi != null && wifi.isApOn)
                  Text('Rete di Maurice accesa · ${wifi.apClients} ${wifi.apClients == 1 ? 'telefono collegato' : 'telefoni collegati'}',
                      style: secondary),
              ]),
            ),
          ]),
        );
      case _Route.local:
        return GlassPanel(
          child: Row(children: [
            Icon(Icons.router_rounded, color: palette.primary, size: 30),
            const SizedBox(width: 14),
            Expanded(
              child: Column(crossAxisAlignment: CrossAxisAlignment.start, children: [
                const Text('Sei sulla rete di Maurice', style: TextStyle(fontSize: 17, fontWeight: FontWeight.w700)),
                Text(_localText(localStatus), style: secondary),
                if (isLocalSilent)
                  Text('Maurice non risponde, riprovo…', style: TextStyle(color: palette.warning, fontSize: 14)),
              ]),
            ),
          ]),
        );
      case _Route.none:
        return GlassPanel(
          child: Column(crossAxisAlignment: CrossAxisAlignment.start, children: [
            Row(children: [
              Icon(Icons.wifi_off_rounded, color: palette.warning, size: 30),
              const SizedBox(width: 14),
              const Expanded(child: Text('Maurice è offline', style: TextStyle(fontSize: 17, fontWeight: FontWeight.w700))),
            ]),
            const SizedBox(height: 10),
            const Text('Collega il telefono alla rete di Maurice e torna qui:', style: TextStyle(fontSize: 15)),
            const SizedBox(height: 8),
            _CopyRow(label: 'Rete', value: robotWifiSsid),
            if (robotWifiPassword.isNotEmpty) _CopyRow(label: 'Password', value: robotWifiPassword),
            const SizedBox(height: 8),
            Text(
                'Maurice accende la sua rete quando non trova la rete salvata per 30 secondi, e la tiene accesa finché '
                'c\'è un telefono collegato o finché non raggiunge il server. Se Android avvisa che la rete non ha '
                'internet, scegli di restare collegato.',
                style: secondary),
            if (retryError != null) ...[
              const SizedBox(height: 10),
              Text(retryError!, style: TextStyle(color: palette.fault, fontSize: 14, fontWeight: FontWeight.w600)),
            ],
            const SizedBox(height: 12),
            Align(
              alignment: Alignment.centerRight,
              child: FilledButton.icon(
                onPressed: isRetrying ? null : onRetry,
                icon: isRetrying
                    ? const SizedBox(width: 18, height: 18, child: CircularProgressIndicator(strokeWidth: 2))
                    : const Icon(Icons.refresh_rounded),
                label: const Text('Riprova'),
              ),
            ),
          ]),
        );
    }
  }
}

/// Nome e valore da copiare (nome e password della rete del robot).
class _CopyRow extends StatelessWidget {
  const _CopyRow({required this.label, required this.value});

  final String label;
  final String value;

  @override
  Widget build(BuildContext context) {
    final palette = AppPalette.of(context);
    return Padding(
      padding: const EdgeInsets.symmetric(vertical: 2),
      child: Row(children: [
        SizedBox(width: 82, child: Text(label, style: TextStyle(color: palette.textSecondary, fontSize: 14))),
        Expanded(child: SelectableText(value, style: const TextStyle(fontSize: 16, fontWeight: FontWeight.w600))),
      ]),
    );
  }
}

/// Il cambio in corso o il suo esito.
class _ChangePanel extends StatelessWidget {
  const _ChangePanel({required this.change, required this.onClose});

  final _Change change;
  final VoidCallback onClose;

  @override
  Widget build(BuildContext context) {
    final palette = AppPalette.of(context);
    final result = change.result;

    final Widget icon;
    String text;
    if (result != null) {
      final color = result == 'ok' ? palette.online : isWifiChangeKept(result) ? palette.warning : palette.fault;
      icon = Icon(result == 'ok' ? Icons.check_circle_rounded : Icons.error_rounded, color: color, size: 30);
      text = wifiResultMessage(result, change.ssid);
      if (change.isFromRobotNetwork) {
        text += result == 'ok'
            ? ' Puoi tornare sulla tua rete: Maurice spegne la sua quando non c\'è più nessuno collegato.'
            : isWifiChangeKept(result)
                ? ' La rete di Maurice resta accesa finché Maurice non raggiunge il server.'
                : ' La rete di Maurice resta accesa: puoi riprovare.';
      }
    } else if (change.isTimedOut) {
      icon = Icon(Icons.schedule_rounded, color: palette.warning, size: 30);
      text = wifiTimeoutMessage(change.ssid);
    } else {
      icon = const SizedBox(width: 30, height: 30, child: Padding(padding: EdgeInsets.all(4), child: CircularProgressIndicator(strokeWidth: 3)));
      text = change.isFromRobotNetwork
          ? 'Maurice si sta collegando a «${change.ssid}». Resta sulla rete di Maurice: l\'esito arriva qui.'
          : 'Maurice si sta collegando a «${change.ssid}»: per qualche secondo risulterà Offline.';
    }

    return GlassPanel(
      child: Column(crossAxisAlignment: CrossAxisAlignment.end, children: [
        Row(crossAxisAlignment: CrossAxisAlignment.start, children: [
          icon,
          const SizedBox(width: 14),
          Expanded(child: Text(text, style: const TextStyle(fontSize: 15))),
        ]),
        if (result != null || change.isTimedOut) TextButton(onPressed: onClose, child: const Text('Chiudi')),
      ]),
    );
  }
}

/// La lista delle reti vicine, con "Altra rete…" in fondo.
class _NetworksPanel extends StatelessWidget {
  const _NetworksPanel({
    required this.scan,
    required this.isScanning,
    required this.error,
    required this.disabledReason,
    required this.onSelect,
    required this.onOther,
  });

  final WifiScan? scan;
  final bool isScanning;
  final String? error;
  final String? disabledReason; // non null: le reti non si possono scegliere, e si dice perché
  final void Function(String ssid, bool isProtected) onSelect;
  final VoidCallback onOther;

  @override
  Widget build(BuildContext context) {
    final palette = AppPalette.of(context);
    final secondary = TextStyle(color: palette.textSecondary, fontSize: 13);
    final isEnabled = disabledReason == null;

    return GlassPanel(
      padding: const EdgeInsets.symmetric(vertical: 12),
      child: Column(crossAxisAlignment: CrossAxisAlignment.start, children: [
        Padding(
          padding: const EdgeInsets.symmetric(horizontal: 16),
          child: Row(children: [
            const Expanded(child: Text('Reti vicine', style: TextStyle(fontSize: 17, fontWeight: FontWeight.w700))),
            if (isScanning) const SizedBox(width: 18, height: 18, child: CircularProgressIndicator(strokeWidth: 2)),
          ]),
        ),
        if (disabledReason != null)
          Padding(
            padding: const EdgeInsets.fromLTRB(16, 6, 16, 0),
            child: Text(disabledReason!, style: TextStyle(color: palette.warning, fontSize: 14, fontWeight: FontWeight.w600)),
          ),
        if (error != null)
          Padding(
            padding: const EdgeInsets.fromLTRB(16, 8, 16, 4),
            child: Text(error!, style: TextStyle(color: palette.fault, fontSize: 14)),
          ),
        if (scan != null && scan!.networks.isEmpty && !isScanning)
          Padding(
            padding: const EdgeInsets.fromLTRB(16, 8, 16, 4),
            child: Text('Maurice non vede nessuna rete.', style: TextStyle(color: palette.textSecondary, fontSize: 15)),
          ),
        const SizedBox(height: 4),
        for (final network in scan?.networks ?? const <WifiNetwork>[])
          ListTile(
            enabled: isEnabled && !network.inUse,
            leading: Icon(_signalIcon(network.signalDbm), color: palette.primary),
            title: Text(network.ssid, overflow: TextOverflow.ellipsis),
            subtitle: Text([
              if (network.inUse) 'In uso',
              network.isProtected ? 'Con password' : 'Aperta',
              '${network.signalDbm} dBm',
            ].join(' · ')),
            trailing: network.inUse
                ? Icon(Icons.check_rounded, color: palette.online)
                : network.isProtected
                    ? Icon(Icons.lock_outline_rounded, color: palette.textSecondary, size: 20)
                    : null,
            onTap: () => onSelect(network.ssid, network.isProtected),
          ),
        ListTile(
          enabled: isEnabled,
          leading: const Icon(Icons.add_rounded),
          title: const Text('Altra rete…'),
          subtitle: const Text('Scrivi il nome di una rete che non compare'),
          onTap: onOther,
        ),
        Padding(
          padding: const EdgeInsets.fromLTRB(16, 4, 16, 0),
          child: Text('Maurice vede solo le reti a 2,4 GHz.', style: secondary),
        ),
      ]),
    );
  }
}

IconData _signalIcon(int dbm) => switch (signalBars(dbm)) {
      4 => Icons.signal_wifi_4_bar_rounded,
      3 => Icons.network_wifi_3_bar_rounded,
      2 => Icons.network_wifi_2_bar_rounded,
      _ => Icons.network_wifi_1_bar_rounded,
    };

/// Password con l'occhio per mostrarla. Ritorna null se l'utente annulla.
Future<String?> _askPassword(BuildContext context, String ssid) async {
  final controller = TextEditingController();
  var isVisible = false;
  final password = await showDialog<String>(
    context: context,
    builder: (context) => StatefulBuilder(
      builder: (context, setDialogState) {
        final length = controller.text.length;
        final isValid = length >= 8 && length <= 63;
        return AlertDialog(
          title: Text('Password di «$ssid»'),
          content: TextField(
            controller: controller,
            autofocus: true,
            obscureText: !isVisible,
            autocorrect: false,
            enableSuggestions: false,
            maxLength: 63,
            decoration: InputDecoration(
              labelText: 'Password',
              helperText: 'Da 8 a 63 caratteri',
              suffixIcon: IconButton(
                icon: Icon(isVisible ? Icons.visibility_off_outlined : Icons.visibility_outlined),
                tooltip: isVisible ? 'Nascondi' : 'Mostra',
                onPressed: () => setDialogState(() => isVisible = !isVisible),
              ),
            ),
            onChanged: (_) => setDialogState(() {}),
            onSubmitted: (_) {
              if (isValid) Navigator.of(context).pop(controller.text);
            },
          ),
          actions: [
            TextButton(onPressed: () => Navigator.of(context).pop(), child: const Text('Annulla')),
            FilledButton(
              onPressed: isValid ? () => Navigator.of(context).pop(controller.text) : null,
              child: const Text('Collega'),
            ),
          ],
        );
      },
    ),
  );
  controller.dispose();
  return password;
}

/// "Altra rete…": nome scritto a mano e tipo. Ritorna (nome, con password) o null se l'utente annulla.
Future<(String, bool)?> _askOtherNetwork(BuildContext context) async {
  final controller = TextEditingController();
  var isProtected = true;
  final network = await showDialog<(String, bool)>(
    context: context,
    builder: (context) => StatefulBuilder(
      builder: (context, setDialogState) {
        final name = controller.text.trim();
        return AlertDialog(
          title: const Text('Altra rete'),
          content: Column(mainAxisSize: MainAxisSize.min, children: [
            TextField(
              controller: controller,
              autofocus: true,
              autocorrect: false,
              decoration: const InputDecoration(labelText: 'Nome della rete'),
              onChanged: (_) => setDialogState(() {}),
            ),
            const SizedBox(height: 8),
            SwitchListTile(
              contentPadding: EdgeInsets.zero,
              title: const Text('Con password'),
              subtitle: const Text('WPA o WPA2'),
              value: isProtected,
              onChanged: (value) => setDialogState(() => isProtected = value),
            ),
          ]),
          actions: [
            TextButton(onPressed: () => Navigator.of(context).pop(), child: const Text('Annulla')),
            FilledButton(
              onPressed: name.isEmpty ? null : () => Navigator.of(context).pop((controller.text, isProtected)),
              child: const Text('Avanti'),
            ),
          ],
        );
      },
    ),
  );
  controller.dispose();
  return network;
}
