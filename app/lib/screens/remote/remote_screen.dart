/// Telecomando (docs/app.md, sezioni 6 e 10.4): prende il telecomando all'apertura e lo rilascia alla chiusura, con un
/// segno di vita ogni 10 s. Mappa piccola in alto, sotto a destra il joystick, a sinistra Casa e Spazzole e
/// aspirazione, sotto Azzera guasto.
library;

import 'dart:async';

import 'package:flutter/material.dart';
import 'package:flutter_riverpod/flutter_riverpod.dart';

import '../../api/api_client.dart';
import '../../api/providers.dart';
import '../../config/app_colors.dart';
import '../../config/app_config.dart';
import '../../map/map_painter.dart';
import '../../map/map_view.dart';
import '../../widgets/alerts_box.dart';
import '../../widgets/glass_panel.dart';
import '../../widgets/joystick.dart';
import '../../widgets/main_button.dart';
import '../../widgets/robot_badges.dart';
import '../../widgets/state_interval.dart';

class RemoteScreen extends ConsumerStatefulWidget {
  const RemoteScreen({super.key});

  @override
  ConsumerState<RemoteScreen> createState() => _RemoteScreenState();
}

class _RemoteScreenState extends ConsumerState<RemoteScreen> with StateIntervalMixin {
  final MapViewController _mapController = MapViewController();
  late final ApiClient _api = ref.read(apiClientProvider);
  late final NotificationNotifier _notifications = ref.read(notificationProvider.notifier);
  Timer? _keepalive;

  // joystick: quello che vuole il dito, e l'unico remote_move in viaggio
  double _wantedV = 0, _wantedOmega = 0;
  double _sentV = 0, _sentOmega = 0;
  bool _moveInFlight = false;
  bool _refused = false; // l'ultimo movimento è stato rifiutato: si riprova solo dopo che il joystick è tornato al centro
  bool _closed = false;
  bool _cleaningBusy = false;

  @override
  int get stateIntervalMs => stateIntervalMapMs;

  @override
  void initState() {
    super.initState();
    // prende il telecomando all'apertura; poi un segno di vita ogni 10 s (il robot lo rilascia dopo 30 s di silenzio)
    Future.microtask(() => _send('take'));
    _keepalive = Timer.periodic(remoteKeepaliveInterval, (_) => _send('keepalive', quiet: true));
  }

  @override
  void dispose() {
    _closed = true;
    _keepalive?.cancel();
    // lo rilascia alla chiusura: senza aspettare la risposta, la schermata non c'è più
    _api.send('POST', '/api/robot/remote/release').catchError((_) => null);
    super.dispose();
  }

  /// Un comando del telecomando. Ritorna true se è riuscito; il rifiuto diventa la notifica in alto.
  Future<bool> _send(String name, {Object? body, bool quiet = false}) async {
    try {
      await _api.send('POST', '/api/robot/remote/$name', body);
      return true;
    } on ApiException catch (error) {
      if (!quiet && !_closed && !error.isUnauthenticated) {
        _notifications.show(error.status == 409 ? 'Comando rifiutato' : 'Comando non riuscito', error.message);
      }
      return false;
    }
  }

  void _joystickChanged(double v, double omega) {
    setState(() {
      _wantedV = v;
      _wantedOmega = omega;
    });
    if (v == 0 && omega == 0) _refused = false;
    _pumpMoves();
  }

  /// Un remote_move alla volta: il successivo parte solo quando arriva la risposta del precedente. Finché il joystick
  /// è spinto il comando si ripete (sul robot vale 0,5 s), lo stesso valore al massimo ogni remoteMoveRepeatInterval;
  /// tornato al centro parte v = 0 e omega = 0, una volta.
  Future<void> _pumpMoves() async {
    if (_moveInFlight) return;
    _moveInFlight = true;
    final lastSent = Stopwatch();
    try {
      while (!_closed && !_refused) {
        var v = _wantedV, omega = _wantedOmega;
        final isStop = v == 0 && omega == 0;
        if (isStop && _sentV == 0 && _sentOmega == 0) break;
        // lo stesso comando di prima: si aspetta, a meno che nel frattempo il dito non si sposti
        if (v == _sentV && omega == _sentOmega && lastSent.isRunning && lastSent.elapsed < remoteMoveRepeatInterval) {
          await Future<void>.delayed(remoteMoveRepeatInterval - lastSent.elapsed);
          continue;
        }
        v = _wantedV;
        omega = _wantedOmega;
        lastSent
          ..reset()
          ..start();
        final ok = await _send('move', body: {'v': v, 'omega': omega});
        if (ok) {
          _sentV = v;
          _sentOmega = omega;
        } else {
          // rifiutato (posizione persa, telecomando di un altro...) o rete giù: il robot si ferma da solo dopo 0,5 s
          _sentV = _sentOmega = 0;
          _refused = !isStop;
          if (isStop) break;
        }
      }
    } finally {
      _moveInFlight = false;
    }
  }

  Future<void> _toggleCleaning(bool on) async {
    if (_cleaningBusy) return;
    setState(() => _cleaningBusy = true);
    await _send('cleaning', body: {'brushes': on, 'suction': on});
    if (mounted) setState(() => _cleaningBusy = false);
  }

  @override
  Widget build(BuildContext context) {
    final state = ref.watch(robotStateProvider);
    final liveMap = ref.watch(liveMapProvider);
    final trail = ref.watch(trailProvider);
    final session = ref.watch(sessionProvider);
    final palette = AppPalette.of(context);
    final hasData = state != null && state.hasRobotData;
    final cleaningOn = hasData && (state.brushesOn || state.suctionOn);
    final inFault = hasData && state.inFault;
    // gli avvisi in tempo reale: guasto, contatti, batteria scarica, posizione persa
    final alerts = buildAlerts(state, username: session.username, inRemoteScreen: true);

    return Scaffold(
      appBar: ScreenHeader(title: 'Telecomando', trailing: [
        OnlineBadge(state: state),
        const SizedBox(width: 12),
        BatteryBadge(state: state),
      ]),
      body: SafeArea(
        top: false,
        child: Padding(
          padding: const EdgeInsets.fromLTRB(screenPadding, 4, screenPadding, 12),
          child: Column(children: [
            // mappa piccola nella parte medio-alta, con il robot in tempo reale
            Expanded(
              flex: 5,
              child: ClipRRect(
                borderRadius: BorderRadius.circular(panelRadius),
                child: DecoratedBox(
                  decoration: BoxDecoration(border: Border.all(color: palette.panelBorder), borderRadius: BorderRadius.circular(panelRadius)),
                  child: Stack(children: [
                    Positioned.fill(
                      child: MapView(
                        map: liveMap?.map,
                        mapRevision: liveMap?.revision ?? 0,
                        trail: trail?.trail.points ?? const [],
                        trailRevision: trail?.revision ?? 0,
                        zones: state?.userZones ?? const [],
                        showBase: state?.startedOnBase ?? false,
                        robot: hasData && state.positionValid ? RobotPose(state.x, state.y, state.thetaDeg) : null,
                        controller: _mapController,
                      ),
                    ),
                    Positioned(
                      right: 10,
                      top: 10,
                      child: GlassIconButton(
                          icon: Icons.my_location_rounded, tooltip: 'Ricentra sul robot', size: 44, onPressed: _mapController.recenterOnRobot),
                    ),
                  ]),
                ),
              ),
            ),
            const SizedBox(height: 12),
            Expanded(
              flex: 6,
              child: GlassPanel(
                child: Column(crossAxisAlignment: CrossAxisAlignment.stretch, children: [
                  Expanded(
                    child: SingleChildScrollView(
                      child: Column(crossAxisAlignment: CrossAxisAlignment.start, children: [
                        Wrap(spacing: 8, runSpacing: 8, children: [
                          _Chip(
                            label: cleaningOn ? 'Spazzole e aspirazione accese' : 'Spazzole e aspirazione spente',
                            color: cleaningOn ? palette.primary : palette.textSecondary,
                          ),
                          if (alerts.isEmpty) _Chip(label: 'Nessun guasto', color: palette.textSecondary),
                        ]),
                        const SizedBox(height: 10),
                        AlertsBox(alerts: alerts),
                      ]),
                    ),
                  ),
                  Row(crossAxisAlignment: CrossAxisAlignment.end, children: [
                    // a sinistra: Casa e Spazzole e aspirazione affiancati, sotto Azzera guasto
                    Expanded(
                      child: Column(mainAxisSize: MainAxisSize.min, crossAxisAlignment: CrossAxisAlignment.stretch, children: [
                        Row(children: [
                          Expanded(
                            flex: 4,
                            child: ActionButton(
                              label: 'Casa',
                              icon: Icons.home_outlined,
                              style: ActionButtonStyle.tinted,
                              vertical: true,
                              height: 108,
                              // Casa di chi tiene il telecomando vale come rilascio e rientro: la schermata si chiude
                              onPressed: () async {
                                final navigator = Navigator.of(context);
                                if (await runCommand(ref, (api) => api.send('POST', '/api/robot/mission/home'))) navigator.maybePop();
                              },
                            ),
                          ),
                          const SizedBox(width: 10),
                          Expanded(
                            flex: 5,
                            child: ActionButton(
                              label: 'Spazzole e\naspirazione',
                              icon: Icons.wb_sunny_outlined,
                              style: cleaningOn ? ActionButtonStyle.primary : ActionButtonStyle.neutral,
                              vertical: true,
                              height: 108,
                              onPressed: _cleaningBusy ? null : () => _toggleCleaning(!cleaningOn),
                            ),
                          ),
                        ]),
                        const SizedBox(height: 10),
                        // sempre visibile, attivo solo con un guasto
                        ActionButton(
                          label: 'Azzera guasto',
                          icon: Icons.restart_alt_rounded,
                          style: ActionButtonStyle.danger,
                          onPressed: inFault ? () => runCommand(ref, (api) => api.send('POST', '/api/robot/fault/reset')) : null,
                        ),
                      ]),
                    ),
                    const SizedBox(width: 8),
                    // a destra: il joystick, con sotto le velocità chieste
                    Column(mainAxisSize: MainAxisSize.min, children: [
                      Joystick(onChanged: _joystickChanged, enabled: state?.online ?? false),
                      Text(
                        '${_wantedV.toStringAsFixed(2).replaceAll('.', ',')} m/s · ${_wantedOmega.toStringAsFixed(1).replaceAll('.', ',')} rad/s',
                        style: TextStyle(color: palette.textSecondary, fontSize: 14, fontFeatures: const [FontFeature.tabularFigures()]),
                      ),
                    ]),
                  ]),
                ]),
              ),
            ),
          ]),
        ),
      ),
    );
  }
}

class _Chip extends StatelessWidget {
  const _Chip({required this.label, required this.color});

  final String label;
  final Color color;

  @override
  Widget build(BuildContext context) {
    return Container(
      padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 6),
      decoration: BoxDecoration(color: color.withValues(alpha: 0.16), borderRadius: BorderRadius.circular(20)),
      child: Text(label, style: TextStyle(color: color, fontWeight: FontWeight.w600, fontSize: 14.5)),
    );
  }
}
