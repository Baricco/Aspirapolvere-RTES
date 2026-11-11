/// Gestione mappe (docs/app.md, sezioni 7 e 10.4): menu delle mappe aperto sulla selezionata, la mappa con le sue zone,
/// Rinomina, Seleziona sul robot, Cancella, e la modifica delle zone con Conferma e Annulla.
library;

import 'package:flutter/material.dart';
import 'package:flutter_riverpod/flutter_riverpod.dart';

import '../../api/api_client.dart';
import '../../api/providers.dart';
import '../../config/app_colors.dart';
import '../../config/app_config.dart';
import '../../map/map_view.dart';
import '../../map/rtm.dart';
import '../../map/zone_editor.dart';
import '../../models/map_info.dart';
import '../../models/robot_state.dart';
import '../../widgets/confirm_dialog.dart';
import '../../widgets/glass_panel.dart';
import '../../widgets/main_button.dart';
import '../../widgets/state_interval.dart';

/// Lunghezza massima del nome di una mappa (come sul server).
const int mapNameMaxLength = 40;

class MapsScreen extends ConsumerStatefulWidget {
  const MapsScreen({super.key});

  @override
  ConsumerState<MapsScreen> createState() => _MapsScreenState();
}

class _MapsScreenState extends ConsumerState<MapsScreen> with StateIntervalMixin {
  final MapViewController _mapController = MapViewController();
  int? _mapId; // la mappa mostrata
  LiveMap? _map; // le sue celle
  List<Zone> _zones = const []; // le sue zone, in metri
  int _revision = 0;
  ZoneEditor? _editor; // non null mentre si modificano le zone
  bool _busy = false;
  String? _loadError;

  @override
  int get stateIntervalMs => stateIntervalOtherMs;

  /// Scarica il file della mappa e ne legge celle e zone.
  Future<void> _load(int mapId) async {
    setState(() {
      _mapId = mapId;
      _map = null;
      _zones = const [];
      _editor = null;
      _loadError = null;
    });
    try {
      final file = decodeRtm(await ref.read(apiClientProvider).getBytes('/api/robot/maps/$mapId'));
      if (!mounted || _mapId != mapId) return;
      setState(() {
        _map = LiveMap()..paste(file);
        _zones = [
          for (final zone in file.zones)
            Zone(cellToMeters(zone[0]), cellToMeters(zone[1]), zone[2] * fineMapCellSize, zone[3] * fineMapCellSize),
        ];
        _revision++;
      });
      _mapController.fitMap();
    } on ApiException catch (error) {
      if (mounted) setState(() => _loadError = error.message);
    } on MapFormatException {
      if (mounted) setState(() => _loadError = 'Il file della mappa non è valido.');
    }
  }

  Future<void> _run(Future<void> Function(ApiClient api) command, {VoidCallback? onDone}) async {
    if (_busy) return;
    setState(() => _busy = true);
    final ok = await runCommand(ref, command, title: 'Non riuscito');
    if (!mounted) return;
    setState(() => _busy = false);
    if (ok) {
      ref.invalidate(mapsProvider);
      onDone?.call();
    }
  }

  Future<void> _rename(MapInfo map) async {
    final controller = TextEditingController(text: map.name);
    final name = await showDialog<String>(
      context: context,
      builder: (context) => AlertDialog(
        title: const Text('Rinomina mappa'),
        content: TextField(
          controller: controller,
          autofocus: true,
          maxLength: mapNameMaxLength,
          decoration: const InputDecoration(labelText: 'Nome'),
          onSubmitted: (value) => Navigator.of(context).pop(value),
        ),
        actions: [
          TextButton(onPressed: () => Navigator.of(context).pop(), child: const Text('Annulla')),
          FilledButton(onPressed: () => Navigator.of(context).pop(controller.text), child: const Text('Rinomina')),
        ],
      ),
    );
    if (name == null || name.trim().isEmpty || name.trim() == map.name) return;
    await _run((api) => api.send('PATCH', '/api/robot/maps/${map.id}', {'name': name.trim()}));
  }

  Future<void> _select(MapInfo map) async {
    await _run((api) => api.send('PUT', '/api/robot/maps/selected', {'id': map.id}),
        onDone: () => ref.read(restartProvider.notifier).start());
  }

  /// Cancellazione, sempre con conferma. Per la mappa selezionata prima "Nessuna mappa" sul robot e, solo se va a buon
  /// fine, la cancellazione sul server: se il robot non è a riposo sulla base o non è raggiungibile non si cancella niente.
  Future<void> _delete(MapInfo map) async {
    final confirmed = await confirmWithWarning(
      context,
      title: 'Cancellare "${map.name}"?',
      warning: map.selected
          ? 'È la mappa selezionata: il robot resterà senza mappa e alla prossima pulizia esplorerà da capo. '
              'La mappa viene cancellata anche dal server e non si può recuperare.'
          : 'La mappa viene cancellata dal server e non si può recuperare.',
    );
    if (!confirmed) return;
    await _run((api) async {
      if (map.selected) {
        await api.send('POST', '/api/robot/map/reset');
        ref.read(restartProvider.notifier).start();
      }
      await api.send('DELETE', '/api/robot/maps/${map.id}');
    }, onDone: () => setState(() => _mapId = null));
  }

  /// Conferma manda la lista intera delle zone: per la selezionata le applica il robot, per le altre il server.
  Future<void> _confirmZones(MapInfo map) async {
    final zones = List<Zone>.of(_editor!.zones);
    await _run((api) => api.send('PUT', '/api/robot/maps/${map.id}/zones', [for (final zone in zones) zone.toJson()]),
        onDone: () => setState(() {
              _zones = zones;
              _editor = null;
              _revision++;
            }));
  }

  @override
  Widget build(BuildContext context) {
    final maps = ref.watch(mapsProvider);
    final state = ref.watch(robotStateProvider);
    final palette = AppPalette.of(context);
    final list = maps.value ?? const <MapInfo>[];

    // il menu si apre sulla selezionata; se la mappa mostrata non c'è più si passa a un'altra
    if (list.isNotEmpty && !list.any((map) => map.id == _mapId)) {
      final first = list.where((map) => map.selected).firstOrNull ?? list.first;
      WidgetsBinding.instance.addPostFrameCallback((_) => _load(first.id));
    }
    final current = list.where((map) => map.id == _mapId).firstOrNull;
    final canSelect = state?.restingOnBase == true && state!.online;

    return Scaffold(
      appBar: const ScreenHeader(title: 'Gestione mappe'),
      body: SafeArea(
        top: false,
        child: Padding(
          padding: const EdgeInsets.fromLTRB(screenPadding, 4, screenPadding, 12),
          child: maps.isLoading && list.isEmpty
              ? const Center(child: CircularProgressIndicator())
              : list.isEmpty
                  ? Center(
                      child: Text(
                        'Non ci sono mappe sul server.\nLa prima si salva alla fine della prima esplorazione.',
                        textAlign: TextAlign.center,
                        style: TextStyle(color: palette.textSecondary, fontSize: 16),
                      ),
                    )
                  : Column(crossAxisAlignment: CrossAxisAlignment.stretch, children: [
                      _MapDropdown(maps: list, current: current, enabled: _editor == null, onChanged: _load),
                      const SizedBox(height: 12),
                      Expanded(
                        child: ClipRRect(
                          borderRadius: BorderRadius.circular(panelRadius),
                          child: DecoratedBox(
                            decoration:
                                BoxDecoration(border: Border.all(color: palette.panelBorder), borderRadius: BorderRadius.circular(panelRadius)),
                            child: _loadError != null
                                ? Center(child: Text(_loadError!, style: TextStyle(color: palette.fault)))
                                : MapView(
                                    map: _map,
                                    mapRevision: _revision,
                                    zones: _zones,
                                    showBase: true,
                                    controller: _mapController,
                                    editor: _editor,
                                  ),
                          ),
                        ),
                      ),
                      const SizedBox(height: 12),
                      if (current != null && _editor != null) ...[
                        Text(
                          _editor!.canAdd
                              ? 'Trascina per disegnare una zona; spostala, ridimensionala dagli angoli, cancellala con la x. Due dita per spostare la mappa.'
                              : 'Hai già $maxUserZones zone: è il massimo.',
                          style: TextStyle(color: palette.textSecondary, fontSize: 14),
                        ),
                        const SizedBox(height: 10),
                        Row(children: [
                          Expanded(
                            child: ActionButton(
                                label: 'Annulla', icon: Icons.close_rounded, onPressed: _busy ? null : () => setState(() => _editor = null)),
                          ),
                          const SizedBox(width: 10),
                          Expanded(
                            child: ActionButton(
                              label: 'Conferma',
                              icon: Icons.check_rounded,
                              style: ActionButtonStyle.primary,
                              onPressed: _busy ? null : () => _confirmZones(current),
                            ),
                          ),
                        ]),
                      ] else if (current != null) ...[
                        Row(children: [
                          Expanded(
                            child: ActionButton(
                              label: 'Modifica zone',
                              icon: Icons.crop_free_rounded,
                              style: ActionButtonStyle.tinted,
                              onPressed: _map == null || _busy ? null : () => setState(() => _editor = ZoneEditor(_zones)),
                            ),
                          ),
                          const SizedBox(width: 10),
                          Expanded(
                            child: ActionButton(
                                label: 'Rinomina', icon: Icons.edit_outlined, onPressed: _busy ? null : () => _rename(current)),
                          ),
                        ]),
                        const SizedBox(height: 10),
                        Row(children: [
                          Expanded(
                            child: ActionButton(
                              label: current.selected ? 'Selezionata' : 'Seleziona sul robot',
                              icon: current.selected ? Icons.check_circle_outline_rounded : Icons.download_done_rounded,
                              style: current.selected ? ActionButtonStyle.neutral : ActionButtonStyle.primary,
                              onPressed: current.selected || _busy
                                  ? null
                                  : canSelect
                                      ? () => _select(current)
                                      : () => ref.read(notificationProvider.notifier).show('Mappa non modificabile adesso',
                                          'Per cambiare mappa il robot deve essere a riposo sulla base.'),
                            ),
                          ),
                          const SizedBox(width: 10),
                          Expanded(
                            child: ActionButton(
                              label: 'Cancella',
                              icon: Icons.delete_outline_rounded,
                              style: ActionButtonStyle.danger,
                              onPressed: _busy ? null : () => _delete(current),
                            ),
                          ),
                        ]),
                      ],
                    ]),
        ),
      ),
    );
  }
}

/// Il menu delle mappe del server, con il segno su quella selezionata sul robot.
class _MapDropdown extends StatelessWidget {
  const _MapDropdown({required this.maps, required this.current, required this.enabled, required this.onChanged});

  final List<MapInfo> maps;
  final MapInfo? current;
  final bool enabled;
  final void Function(int mapId) onChanged;

  @override
  Widget build(BuildContext context) {
    final palette = AppPalette.of(context);
    return GlassPanel(
      padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 4),
      radius: controlRadius + 2,
      child: DropdownButtonHideUnderline(
        child: DropdownButton<int>(
          value: current?.id,
          isExpanded: true,
          borderRadius: BorderRadius.circular(controlRadius),
          icon: const Icon(Icons.keyboard_arrow_down_rounded),
          onChanged: enabled ? (mapId) => mapId == null ? null : onChanged(mapId) : null,
          items: [
            for (final map in maps)
              DropdownMenuItem(
                value: map.id,
                child: Row(children: [
                  Flexible(
                      child: Text(map.name,
                          overflow: TextOverflow.ellipsis, style: const TextStyle(fontSize: 17, fontWeight: FontWeight.w600))),
                  if (map.selected) ...[
                    const SizedBox(width: 10),
                    Text('selezionata', style: TextStyle(color: palette.primary, fontSize: 13.5, fontWeight: FontWeight.w600)),
                  ],
                ]),
              ),
          ],
        ),
      ),
    );
  }
}
