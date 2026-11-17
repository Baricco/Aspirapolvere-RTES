/// Schermata principale (docs/app.md, sezione 10.4), stile Vetro: la mappa riempie lo schermo e i comandi stanno
/// sopra in pannelli traslucidi.
library;

import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:flutter_riverpod/flutter_riverpod.dart';

import '../../api/providers.dart';
import '../../api/wifi_api.dart';
import '../../config/app_colors.dart';
import '../../config/app_config.dart';
import '../../map/map_painter.dart';
import '../../map/map_view.dart';
import '../../models/map_info.dart';
import '../../models/robot_state.dart';
import '../../models/wifi.dart';
import '../../widgets/alerts_box.dart';
import '../../widgets/confirm_dialog.dart';
import '../../widgets/glass_panel.dart';
import '../../widgets/main_button.dart';
import '../../widgets/robot_badges.dart';
import '../../widgets/state_interval.dart';
import '../diagnostics/diagnostics_screen.dart';
import '../log/log_screen.dart';
import '../maps/maps_screen.dart';
import '../remote/remote_screen.dart';
import '../schedules/schedules_screen.dart';
import '../wifi/wifi_screen.dart';

/// Spazio che i pannelli in alto e in basso tolgono alla mappa, per inquadrarla nella parte libera.
const double _topPanelsHeight = 150;
const double _bottomPanelHeight = 170;

class MainScreen extends ConsumerStatefulWidget {
  const MainScreen({super.key});

  @override
  ConsumerState<MainScreen> createState() => _MainScreenState();
}

class _MainScreenState extends ConsumerState<MainScreen> with StateIntervalMixin {
  final MapViewController _mapController = MapViewController();
  final GlobalKey<ScaffoldState> _scaffoldKey = GlobalKey();
  bool _commandBusy = false;

  @override
  int get stateIntervalMs => stateIntervalMapMs;

  Future<void> _mission(String name) async {
    if (_commandBusy) return;
    setState(() => _commandBusy = true);
    await runCommand(ref, (api) => api.send('POST', '/api/robot/mission/$name'));
    if (mounted) setState(() => _commandBusy = false);
  }

  void _open(Widget screen) {
    Navigator.of(context).push(MaterialPageRoute<void>(builder: (_) => screen));
  }

  /// Scelta dal menu delle mappe: una mappa del server, oppure "Nessuna mappa" (id null).
  Future<void> _selectMap(int? mapId) async {
    final ok = await runCommand(
      ref,
      (api) => mapId == null
          ? api.send('POST', '/api/robot/map/reset')
          : api.send('PUT', '/api/robot/maps/selected', {'id': mapId}),
    );
    // il robot carica la mappa nuova senza riavviarsi: l'esito arriva a mappa già caricata
    if (ok) ref.invalidate(mapsProvider);
  }

  /// Riposiziona base (docs/app.md, sezione 7.9): conferma, poi il comando. Il robot risponde subito se accetta;
  /// l'esito arriva nello stato (map.reposition) e lo mostra il listener in build.
  Future<void> _repositionBase() async {
    final confirmed = await confirmWithWarning(context,
        title: 'Riposiziona base',
        warning: 'Il robot, fermo sulla base, cerca dove si trova nella mappa e la sposta insieme alle zone proibite e ai muri. '
            'Sul server resta anche la mappa di prima.',
        confirmLabel: 'Riposiziona');
    if (!confirmed) return;
    final ok = await runCommand(ref, (api) => api.send('POST', '/api/robot/base/reposition'));
    if (ok) {
      ref.read(notificationProvider.notifier).show('Ricerca della base', 'Il robot cerca dove si trova nella mappa.');
    }
  }

  /// Esito di Riposiziona base, quando lo stato smette di essere "running".
  void _showRepositionResult(String result) {
    final notifications = ref.read(notificationProvider.notifier);
    switch (result) {
      case 'done':
        notifications.show('Base riposizionata', 'Mappa, zone e muri spostati: il robot usa già la mappa nuova.');
        ref.invalidate(mapsProvider);
      case 'not_found':
        notifications.show('Base non trovata', 'Il robot non riconosce la stanza dalla base: la mappa non è cambiata.');
      case 'rotated':
        notifications.show('Base non riposizionata', 'La base è girata di un angolo che non si può correggere: rifai la mappa.');
      default:
        notifications.show('Base non riposizionata', 'Qualcosa non ha funzionato: la mappa non è cambiata.');
    }
  }

  @override
  Widget build(BuildContext context) {
    ref.listen(robotStateProvider.select((robotState) => robotState?.mapReposition), (previous, next) {
      if (previous == 'running' && next != null && next != 'running') _showRepositionResult(next);
    });
    ref.listen(robotStateProvider.select((robotState) => robotState?.targetState), (previous, next) {
      if (previous != 'going') return;
      if (next == 'reached') ref.read(notificationProvider.notifier).show('Obiettivo raggiunto', 'Il robot è arrivato.');
      if (next == 'failed') {
        ref.read(notificationProvider.notifier).show('Obiettivo non raggiunto', 'Il robot non riesce ad arrivarci: si è messo in pausa.');
      }
    });
    // esito di un cambio di rete arrivato con la schermata del Wi-Fi chiusa, anche molto dopo (docs/wifi.md, sezione 13)
    ref.listen(robotStateProvider.select((robotState) => robotState?.online == true ? robotState?.wifi?.change : null),
        (previous, change) {
      final pending = ref.read(pendingWifiChangeProvider);
      if (pending == null || pending.isScreenOpen || change == null) return;
      if (change.number != pending.number || !change.isDone) return;
      ref.read(notificationProvider.notifier).show('Rete Wi-Fi', wifiResultMessage(change.result, pending.ssid));
      ref.read(pendingWifiChangeProvider.notifier).clear();
    });
    final state = ref.watch(robotStateProvider);
    final liveMap = ref.watch(liveMapProvider);
    final trail = ref.watch(trailProvider);
    final session = ref.watch(sessionProvider);
    final restarting = ref.watch(restartProvider);
    final schedules = ref.watch(schedulesProvider).value ?? const [];
    final maps = ref.watch(mapsProvider).value ?? const <MapInfo>[];
    final palette = AppPalette.of(context);
    final safe = MediaQuery.paddingOf(context);
    final hasData = state != null && state.hasRobotData;
    final alerts = buildAlerts(state, username: session.username, restarting: restarting, schedules: schedules);

    return Scaffold(
      key: _scaffoldKey,
      drawer: _MainMenu(onOpen: _open, onRepositionBase: _repositionBase),
      body: Stack(children: [
        // la mappa a tutto schermo: selezionata o dal vivo, con scia, zone, base e robot
        Positioned.fill(
          child: MapView(
            map: liveMap?.map,
            mapRevision: liveMap?.revision ?? 0,
            trail: trail?.trail.points ?? const [],
            trailRevision: trail?.revision ?? 0,
            zones: state?.userZones ?? const [],
            walls: state?.virtualWalls ?? const [],
            target: state?.targetState == 'going' && state?.target != null ? Offset(state!.target!.x, state.target!.y) : null,
            onLongPress: hasData && state.online ? (point) => confirmGoTo(context, ref, point) : null,
            showBase: state?.startedOnBase ?? false,
            robot: hasData && state.positionValid ? RobotPose(state.x, state.y, state.thetaDeg) : null,
            controller: _mapController,
            topInset: safe.top + _topPanelsHeight,
            bottomInset: safe.bottom + _bottomPanelHeight,
          ),
        ),

        // in alto: menu, nome, Online/Offline con l'ultimo contatto, batteria; sotto il menu delle mappe
        Positioned(
          left: screenPadding,
          right: screenPadding,
          top: safe.top + 8,
          child: Column(crossAxisAlignment: CrossAxisAlignment.end, children: [
            GlassPanel(
              padding: const EdgeInsets.fromLTRB(4, 4, 14, 4),
              child: Row(children: [
                IconButton(
                  icon: const Icon(Icons.menu_rounded),
                  tooltip: 'Menu',
                  onPressed: () => _scaffoldKey.currentState?.openDrawer(),
                ),
                const SizedBox(width: 2),
                const Expanded(
                  child: Text(appName,
                      maxLines: 1, overflow: TextOverflow.ellipsis, style: TextStyle(fontSize: 20, fontWeight: FontWeight.w700)),
                ),
                OnlineBadge(state: state),
                const SizedBox(width: 12),
                BatteryBadge(state: state),
              ]),
            ),
            const SizedBox(height: 10),
            Row(children: [
              Expanded(
                child: _MapMenu(
                  maps: maps,
                  canChange: state?.restingOnBase == true && state!.online && !restarting,
                  hasUnsavedMap: hasData && state.online && state.mapFileCrc == null,
                  onSelected: _selectMap,
                ),
              ),
              const SizedBox(width: 10),
              GlassIconButton(icon: Icons.layers_outlined, tooltip: 'Gestione mappe', onPressed: () => _open(const MapsScreen())),
            ]),
            const SizedBox(height: 10),
            GlassIconButton(
              icon: Icons.my_location_rounded,
              tooltip: 'Ricentra sul robot',
              size: 46,
              onPressed: _mapController.recenterOnRobot,
            ),
          ]),
        ),

        // in basso: riga di stato, avvisi, i tre pulsanti
        Positioned(
          left: screenPadding,
          right: screenPadding,
          bottom: safe.bottom + 12,
          child: GlassPanel(
            child: Column(crossAxisAlignment: CrossAxisAlignment.stretch, mainAxisSize: MainAxisSize.min, children: [
              _StatusLine(state: state),
              const SizedBox(height: 12),
              AlertsBox(alerts: alerts),
              Row(children: [
                Expanded(flex: 6, child: _mainButton(state)),
                const SizedBox(width: 8),
                Expanded(
                  flex: 5,
                  child: ActionButton(
                    label: 'Casa',
                    icon: Icons.home_outlined,
                    style: ActionButtonStyle.tinted,
                    onPressed: _commandBusy ? null : () => _mission('home'),
                  ),
                ),
                const SizedBox(width: 8),
                Expanded(
                  flex: 8,
                  child: ActionButton(
                    label: 'Telecomando',
                    icon: Icons.sports_esports_outlined,
                    onPressed: () => _open(const RemoteScreen()),
                  ),
                ),
              ]),
            ]),
          ),
        ),

        if (state == null) Positioned.fill(child: ColoredBox(color: palette.background.withValues(alpha: 0.4))),
      ]),
    );
  }

  /// Il pulsante principale: Avvia, Pausa o Riprendi secondo lo stato.
  Widget _mainButton(RobotState? state) {
    final action = state?.mainAction ?? MainAction.start;
    final (label, icon, name) = switch (action) {
      MainAction.start => ('Avvia', Icons.play_arrow_rounded, 'start'),
      MainAction.pause => ('Pausa', Icons.pause_rounded, 'pause'),
      MainAction.resume => ('Riprendi', Icons.play_arrow_rounded, 'resume'),
    };
    return ActionButton(
      label: label,
      icon: icon,
      style: ActionButtonStyle.primary,
      onPressed: _commandBusy ? null : () => _mission(name),
    );
  }
}

/// La riga di stato, in inglese: missione · stato a sinistra, durata a destra.
class _StatusLine extends StatelessWidget {
  const _StatusLine({required this.state});

  final RobotState? state;

  @override
  Widget build(BuildContext context) {
    final palette = AppPalette.of(context);
    const style = TextStyle(fontSize: 18, fontWeight: FontWeight.w700);
    if (state == null || !state!.hasRobotData) {
      return Text(state == null ? 'Connecting…' : 'No data from the robot yet', style: style.copyWith(color: palette.textSecondary));
    }
    // missione · stato a sinistra (si accorciano se non c'è posto), durata sempre a destra
    return Row(children: [
      Expanded(
        child: Text.rich(
          TextSpan(children: [
            TextSpan(text: state!.missionLabel),
            TextSpan(text: '  ·  ', style: TextStyle(color: palette.textSecondary)),
            TextSpan(text: state!.stateLabel),
          ]),
          maxLines: 1,
          overflow: TextOverflow.ellipsis,
          style: style,
        ),
      ),
      // la durata solo con una missione in corso: a riposo quella dell'ultima missione non dice niente
      if (state!.hasMission) ...[
        const SizedBox(width: 12),
        Text(state!.durationLabel, style: style.copyWith(color: palette.textSecondary, fontWeight: FontWeight.w600)),
      ],
    ]);
  }
}

/// Il menu delle mappe: le mappe del server più "Nessuna mappa". Attivo solo con il robot a riposo sulla base,
/// altrimenti spiega perché.
class _MapMenu extends ConsumerWidget {
  const _MapMenu({required this.maps, required this.canChange, required this.hasUnsavedMap, required this.onSelected});

  final List<MapInfo> maps;
  final bool canChange;
  final bool hasUnsavedMap;
  final void Function(int? mapId) onSelected;

  static const int _noMap = -1; // valore della voce "Nessuna mappa"

  @override
  Widget build(BuildContext context, WidgetRef ref) {
    final palette = AppPalette.of(context);
    final selected = maps.where((map) => map.selected).firstOrNull;
    final title = selected?.name ?? (hasUnsavedMap ? 'Mappa nuova' : 'Nessuna mappa');
    final subtitle = selected != null ? 'Mappa selezionata' : (hasUnsavedMap ? 'Non ancora salvata' : 'Il robot esplorerà');
    final content = Row(children: [
      Expanded(
        child: Column(crossAxisAlignment: CrossAxisAlignment.start, mainAxisSize: MainAxisSize.min, children: [
          Text(title, maxLines: 1, overflow: TextOverflow.ellipsis, style: const TextStyle(fontSize: 17, fontWeight: FontWeight.w600)),
          Text(subtitle, style: TextStyle(fontSize: 13.5, color: palette.textSecondary)),
        ]),
      ),
      Icon(Icons.keyboard_arrow_down_rounded, color: canChange ? palette.text : palette.textSecondary),
    ]);
    return GlassPanel(
      padding: EdgeInsets.zero,
      radius: controlRadius + 2,
      child: canChange
          ? PopupMenuButton<int>(
              tooltip: 'Scegli la mappa',
              position: PopupMenuPosition.under,
              onSelected: (value) => onSelected(value == _noMap ? null : value),
              itemBuilder: (context) => [
                for (final map in maps)
                  PopupMenuItem(
                    value: map.id,
                    child: Row(children: [
                      Expanded(child: Text(map.name)),
                      if (map.selected) Icon(Icons.check_rounded, size: 18, color: palette.primary),
                    ]),
                  ),
                const PopupMenuDivider(),
                const PopupMenuItem(value: _noMap, child: Text('Nessuna mappa')),
              ],
              child: Padding(padding: const EdgeInsets.fromLTRB(16, 8, 10, 8), child: content),
            )
          : InkWell(
              onTap: () => ref.read(notificationProvider.notifier).show(
                  'Mappa non modificabile adesso', 'Per cambiare mappa il robot deve essere a riposo sulla base.'),
              child: Padding(padding: const EdgeInsets.fromLTRB(16, 8, 10, 8), child: content),
            ),
    );
  }
}

/// Il menu laterale: Programmazione, Registro dei comandi, Avanzate (Azzera guasto, Riposiziona base, Rete Wi-Fi solo
/// nell'APK, Diagnostica), Esci.
class _MainMenu extends ConsumerWidget {
  const _MainMenu({required this.onOpen, required this.onRepositionBase});

  final void Function(Widget screen) onOpen;
  final Future<void> Function() onRepositionBase;

  @override
  Widget build(BuildContext context, WidgetRef ref) {
    final palette = AppPalette.of(context);
    final username = ref.watch(sessionProvider).username;
    void open(Widget screen) {
      Navigator.of(context).pop();
      onOpen(screen);
    }

    return Drawer(
      backgroundColor: palette.background,
      child: SafeArea(
        child: ListView(padding: const EdgeInsets.symmetric(vertical: 12), children: [
          Padding(
            padding: const EdgeInsets.fromLTRB(20, 12, 20, 18),
            child: Column(crossAxisAlignment: CrossAxisAlignment.start, children: [
              const Text(appName, style: TextStyle(fontSize: 26, fontWeight: FontWeight.w800)),
              if (username != null) Text(username, style: TextStyle(color: palette.textSecondary)),
            ]),
          ),
          ListTile(
            leading: const Icon(Icons.schedule_rounded),
            title: const Text('Programmazione'),
            onTap: () => open(const SchedulesScreen()),
          ),
          ListTile(
            leading: const Icon(Icons.receipt_long_outlined),
            title: const Text('Registro dei comandi'),
            onTap: () => open(const LogScreen()),
          ),
          ExpansionTile(
            leading: const Icon(Icons.tune_rounded),
            title: const Text('Avanzate'),
            shape: const Border(),
            children: [
              ListTile(
                contentPadding: const EdgeInsets.only(left: 56, right: 16),
                leading: const Icon(Icons.restart_alt_rounded),
                title: const Text('Azzera guasto'),
                onTap: () async {
                  Navigator.of(context).pop();
                  final ok = await runCommand(ref, (api) => api.send('POST', '/api/robot/fault/reset'));
                  if (ok) ref.read(notificationProvider.notifier).show('Guasto azzerato', 'Il robot è di nuovo pronto.');
                },
              ),
              ListTile(
                contentPadding: const EdgeInsets.only(left: 56, right: 16),
                leading: const Icon(Icons.my_location_rounded),
                title: const Text('Riposiziona base'),
                onTap: () {
                  Navigator.of(context).pop();
                  onRepositionBase();
                },
              ),
              // la rete del robot (192.168.5.1, senza internet) si raggiunge solo dall'app installata (docs/wifi.md, 10.1)
              if (!kIsWeb)
                ListTile(
                  contentPadding: const EdgeInsets.only(left: 56, right: 16),
                  leading: const Icon(Icons.wifi_rounded),
                  title: const Text('Rete Wi-Fi'),
                  onTap: () => open(const WifiScreen()),
                ),
              ListTile(
                contentPadding: const EdgeInsets.only(left: 56, right: 16),
                leading: const Icon(Icons.monitor_heart_outlined),
                title: const Text('Diagnostica'),
                onTap: () => open(const DiagnosticsScreen()),
              ),
            ],
          ),
          const Divider(height: 28),
          ListTile(
            leading: const Icon(Icons.logout_rounded),
            title: const Text('Esci'),
            onTap: () => ref.read(sessionProvider.notifier).logout(),
          ),
        ]),
      ),
    );
  }
}
