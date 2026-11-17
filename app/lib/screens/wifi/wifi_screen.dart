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
  WifiScan? _scan;
  bool _isScanning = false;
  String? _scanError;
  _Change? _change;
  Timer? _pollTimer, _timeoutTimer;
  bool _isPolling = false;

  WifiApi get _api => _route == _Route.local ? _local : BackendWifiApi(ref.read(apiClientProvider));

  @override
  void initState() {
    super.initState();
    Future.microtask(_chooseRoute);
  }

  @override
  void dispose() {
    _pollTimer?.cancel();
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

  Future<void> _chooseRoute() async {
    if (!mounted) return;
    setState(() => _route = _Route.checking);
    if (ref.read(robotStateProvider)?.online == true) {
      await WifiNetworkBinding.unbind();
      _setRoute(_Route.backend);
      return;
    }
    final status = await _local.status();
    if (!mounted) return;
    _setRoute(status != null ? _Route.local : _Route.none);
  }

  void _setRoute(_Route route) {
    if (!mounted) return;
    setState(() {
      _route = route;
      _scan = null;
      _scanError = null;
    });
    if (route == _Route.backend || route == _Route.local) _loadScan();
  }

  // ------------------------------------------------------------------------------------------- scansione

  Future<void> _loadScan({bool refresh = false}) async {
    setState(() {
      _isScanning = true;
      _scanError = null;
    });
    try {
      final scan = await _api.scan(refresh: refresh);
      if (mounted) setState(() => _scan = scan);
    } on ApiException catch (error) {
      if (mounted) setState(() => _scanError = error.message.isNotEmpty ? error.message : 'Ricerca delle reti non riuscita.');
    } finally {
      if (mounted) setState(() => _isScanning = false);
    }
  }

  // ---------------------------------------------------------------------------------------------- cambio

  Future<void> _connect(String ssid, bool isProtected) async {
    final password = isProtected ? await _askPassword(context, ssid) : '';
    if (password == null || !mounted) return;
    final isFromRobotNetwork = _route == _Route.local;
    try {
      final number = await _api.connect(ssid, isProtected, password);
      if (!mounted) return;
      setState(() => _change = _Change(ssid, number, isFromRobotNetwork));
      _pending.start(number, ssid);
      _timeoutTimer?.cancel();
      _timeoutTimer = Timer(wifiChangeWait, () {
        if (mounted && _change?.number == number && _change?.result == null) setState(() => _change!.isTimedOut = true);
      });
      // dalla rete del robot il telefono la perde: si cerca il robot di nuovo, sulla sua rete o dal backend
      if (isFromRobotNetwork) {
        _pollTimer?.cancel();
        _pollTimer = Timer.periodic(wifiChangePollInterval, (_) => _pollChange());
      }
    } on ApiException catch (error) {
      if (!error.isUnauthenticated) {
        ref.read(notificationProvider.notifier).show(
            error.status == 409 ? 'Cambio rifiutato' : 'Cambio non riuscito',
            error.message.isNotEmpty ? error.message : error.reason);
      }
    }
  }

  /// Esito arrivato (dallo stato del robot o da rtes-wifi sulla sua rete).
  void _finishChange(WifiStatus status) {
    final change = _change;
    if (change == null || change.result != null || status.change.number != change.number || !status.change.isDone) return;
    setState(() => change.result = status.change.result);
    _pending.clear();
    _pollTimer?.cancel();
    _timeoutTimer?.cancel();
  }

  /// Durante un cambio chiesto dalla rete del robot: il backend (se il cambio è riuscito) o il robot sulla sua rete
  /// (se è fallito e l'ha riaccesa).
  Future<void> _pollChange() async {
    if (_isPolling || _change == null || _change!.result != null) return;
    _isPolling = true;
    try {
      final robot = ref.read(robotStateProvider);
      final status = robot?.online == true ? robot!.wifi : await _local.status();
      if (!mounted || status == null) return;
      _finishChange(status);
      if (_change?.result != null) {
        await WifiNetworkBinding.unbind();
        _setRoute(robot?.online == true ? _Route.backend : _Route.local);
      }
    } finally {
      _isPolling = false;
    }
  }

  // ---------------------------------------------------------------------------------------------- disegno

  @override
  Widget build(BuildContext context) {
    final robot = ref.watch(robotStateProvider);
    ref.listen<RobotState?>(robotStateProvider, (previous, next) {
      final status = next?.wifi;
      if (next?.online == true && status != null) _finishChange(status);
      // il robot è tornato online mentre la schermata non trovava nessuno
      if (next?.online == true && _route == _Route.none) _setRoute(_Route.backend);
    });

    final canRefresh = !_isScanning && (_route == _Route.backend || _route == _Route.local);
    final isResting = _route != _Route.backend || robot?.navigationState == 'sleep';
    final isBusy = _change != null && _change!.result == null && !_change!.isTimedOut;

    return Scaffold(
      appBar: ScreenHeader(title: 'Rete Wi-Fi', trailing: [
        IconButton(
          icon: const Icon(Icons.refresh_rounded),
          tooltip: 'Aggiorna',
          onPressed: canRefresh ? () => _loadScan(refresh: true) : null,
        ),
        OnlineBadge(state: robot),
      ]),
      body: ListView(padding: const EdgeInsets.fromLTRB(screenPadding, 4, screenPadding, 24), children: [
        _StatusPanel(route: _route, robot: robot, onRetry: _chooseRoute),
        if (_change != null) ...[
          const SizedBox(height: 12),
          _ChangePanel(change: _change!, onClose: () => setState(() => _change = null)),
        ],
        if (_route == _Route.backend || _route == _Route.local) ...[
          const SizedBox(height: 12),
          _NetworksPanel(
            scan: _scan,
            isScanning: _isScanning,
            error: _scanError,
            isFromRobotNetwork: _route == _Route.local,
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
  const _StatusPanel({required this.route, required this.robot, required this.onRetry});

  final _Route route;
  final RobotState? robot;
  final VoidCallback onRetry;

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
            Text('Cerco Maurice…', style: TextStyle(fontSize: 16)),
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
                Text('Scegli la rete Wi-Fi a cui Maurice deve collegarsi.', style: secondary),
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
                'Maurice accende la sua rete quando non riesce a collegarsi alla rete salvata per 2 minuti, e ogni '
                '5 minuti la spegne per 1 minuto per riprovare. Se Android avvisa che la rete non ha internet, scegli di '
                'restare collegato.',
                style: secondary),
            const SizedBox(height: 12),
            Align(
              alignment: Alignment.centerRight,
              child: FilledButton.icon(onPressed: onRetry, icon: const Icon(Icons.refresh_rounded), label: const Text('Riprova')),
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
    final String text;
    if (result != null) {
      final color = result == 'ok' ? palette.online : isWifiChangeKept(result) ? palette.warning : palette.fault;
      icon = Icon(result == 'ok' ? Icons.check_circle_rounded : Icons.error_rounded, color: color, size: 30);
      text = wifiResultMessage(result, change.ssid);
    } else if (change.isTimedOut) {
      icon = Icon(Icons.schedule_rounded, color: palette.warning, size: 30);
      text = wifiTimeoutMessage(change.ssid);
    } else {
      icon = const SizedBox(width: 30, height: 30, child: Padding(padding: EdgeInsets.all(4), child: CircularProgressIndicator(strokeWidth: 3)));
      text = change.isFromRobotNetwork
          ? 'Maurice si sta collegando a «${change.ssid}»: il telefono perde la rete di Maurice e torna sulla sua. '
              'L\'esito arriva appena Maurice si fa sentire.'
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
    required this.isFromRobotNetwork,
    required this.disabledReason,
    required this.onSelect,
    required this.onOther,
  });

  final WifiScan? scan;
  final bool isScanning;
  final String? error;
  final bool isFromRobotNetwork;
  final String? disabledReason; // non null: le reti non si possono scegliere, e si dice perché
  final void Function(String ssid, bool isProtected) onSelect;
  final VoidCallback onOther;

  @override
  Widget build(BuildContext context) {
    final palette = AppPalette.of(context);
    final secondary = TextStyle(color: palette.textSecondary, fontSize: 13);
    final scannedAt = scan?.scannedAt;
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
        if (isFromRobotNetwork && scannedAt != null)
          Padding(
            padding: const EdgeInsets.fromLTRB(16, 2, 16, 0),
            child: Text('Lista delle ${_time(scannedAt)}: per aggiornarla Maurice spegne la sua rete per qualche secondo.',
                style: secondary),
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

String _time(DateTime time) =>
    '${time.hour.toString().padLeft(2, '0')}:${time.minute.toString().padLeft(2, '0')}';

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
