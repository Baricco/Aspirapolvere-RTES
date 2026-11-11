/// Diagnostica (docs/app.md, sezione 10.4): tutto lo stato del robot (sensori, correnti, lidar, driver, limiti,
/// sicurezza) e le scadenze mancate di ogni task, dal vivo.
library;

import 'package:flutter/material.dart';
import 'package:flutter_riverpod/flutter_riverpod.dart';

import '../../api/providers.dart';
import '../../config/app_colors.dart';
import '../../config/app_config.dart';
import '../../models/robot_state.dart';
import '../../widgets/glass_panel.dart';
import '../../widgets/robot_badges.dart';
import '../../widgets/state_interval.dart';

/// Le sezioni dello stato nell'ordine in cui si mostrano, con il loro titolo.
const List<(String, String)> _sections = [
  ('navigation', 'Navigazione'),
  ('objective', 'Obiettivo'),
  ('position', 'Posizione'),
  ('map', 'Mappa'),
  ('safety', 'Sicurezza'),
  ('lidar', 'Lidar'),
  ('battery', 'Batteria'),
  ('motors', 'Motori della pulizia'),
  ('driver', 'Driver'),
  ('sensors', 'Sensori'),
];

class DiagnosticsScreen extends ConsumerStatefulWidget {
  const DiagnosticsScreen({super.key});

  @override
  ConsumerState<DiagnosticsScreen> createState() => _DiagnosticsScreenState();
}

class _DiagnosticsScreenState extends ConsumerState<DiagnosticsScreen> with StateIntervalMixin {
  @override
  int get stateIntervalMs => stateIntervalOtherMs;

  @override
  Widget build(BuildContext context) {
    final state = ref.watch(robotStateProvider);
    final palette = AppPalette.of(context);
    return Scaffold(
      appBar: ScreenHeader(title: 'Diagnostica', trailing: [OnlineBadge(state: state)]),
      body: state == null || !state.hasRobotData
          ? Center(child: Text('Il robot non ha ancora mandato dati.', style: TextStyle(color: palette.textSecondary, fontSize: 16)))
          : ListView(padding: const EdgeInsets.fromLTRB(screenPadding, 4, screenPadding, 24), children: [
              _TasksPanel(tasks: state.tasks),
              const SizedBox(height: 12),
              _Panel(title: 'Robot', rows: [
                ('uptime_s', '${state.raw['uptime_s']}'),
                ('robot_time', '${state.raw['robot_time']}'),
                ('remote_control', '${state.raw['remote_control']}'),
                ('remote_control_user', '${state.raw['remote_control_user']}'),
                ('last_contact', timeAgo(state.lastContact)),
                ('map_version', '${state.mapVersion}'),
              ]),
              for (final (key, title) in _sections)
                if (state.raw[key] is Map<String, dynamic>) ...[
                  const SizedBox(height: 12),
                  _Panel(title: title, rows: _flatten(state.raw[key] as Map<String, dynamic>)),
                ],
            ]),
    );
  }
}

/// Appiattisce un oggetto JSON in righe "nome → valore": gli oggetti dentro diventano "padre.figlio".
List<(String, String)> _flatten(Map<String, dynamic> object, [String prefix = '']) {
  final rows = <(String, String)>[];
  for (final entry in object.entries) {
    final name = '$prefix${entry.key}';
    final value = entry.value;
    if (value is Map<String, dynamic>) {
      rows.addAll(_flatten(value, '$name.'));
    } else if (value is List && value.isNotEmpty && value.first is Map<String, dynamic>) {
      for (var i = 0; i < value.length; i++) {
        rows.add(('$name[$i]', (value[i] as Map<String, dynamic>).entries.map((e) => '${e.key} ${e.value}').join(', ')));
      }
    } else if (value is List) {
      rows.add((name, value.isEmpty ? '—' : value.join(', ')));
    } else {
      rows.add((name, value == null ? '—' : '$value'));
    }
  }
  return rows;
}

class _Panel extends StatelessWidget {
  const _Panel({required this.title, required this.rows});

  final String title;
  final List<(String, String)> rows;

  @override
  Widget build(BuildContext context) {
    final palette = AppPalette.of(context);
    return GlassPanel(
      child: Column(crossAxisAlignment: CrossAxisAlignment.start, children: [
        Text(title, style: const TextStyle(fontSize: 17, fontWeight: FontWeight.w700)),
        const SizedBox(height: 8),
        for (final (name, value) in rows)
          Padding(
            padding: const EdgeInsets.symmetric(vertical: 2.5),
            child: Row(crossAxisAlignment: CrossAxisAlignment.start, children: [
              Expanded(flex: 5, child: Text(name, style: TextStyle(color: palette.textSecondary, fontSize: 14))),
              const SizedBox(width: 10),
              Expanded(
                flex: 6,
                child: Text(value,
                    textAlign: TextAlign.right, style: const TextStyle(fontSize: 14, fontFeatures: [FontFeature.tabularFigures()])),
              ),
            ]),
          ),
      ]),
    );
  }
}

/// Le scadenze mancate e le attivazioni perse di ogni task: se non sono zero si vedono in rosso.
class _TasksPanel extends StatelessWidget {
  const _TasksPanel({required this.tasks});

  final List<TaskStats> tasks;

  @override
  Widget build(BuildContext context) {
    final palette = AppPalette.of(context);
    final anyMiss = tasks.any((task) => task.deadlineMisses > 0 || task.missedActivations > 0);
    Widget cell(String text, {Color? color, bool bold = false, TextAlign align = TextAlign.right}) => Text(text,
        textAlign: align,
        style: TextStyle(fontSize: 14.5, color: color, fontWeight: bold ? FontWeight.w700 : FontWeight.w500));
    return GlassPanel(
      child: Column(crossAxisAlignment: CrossAxisAlignment.start, children: [
        Row(children: [
          const Expanded(child: Text('Task real-time', style: TextStyle(fontSize: 17, fontWeight: FontWeight.w700))),
          Text(anyMiss ? 'scadenze mancate!' : 'nessuna scadenza mancata',
              style: TextStyle(color: anyMiss ? palette.fault : palette.online, fontWeight: FontWeight.w700)),
        ]),
        const SizedBox(height: 10),
        Row(children: [
          Expanded(flex: 4, child: cell('task', color: palette.textSecondary, align: TextAlign.left)),
          Expanded(flex: 4, child: cell('scadenze\nmancate', color: palette.textSecondary)),
          Expanded(flex: 4, child: cell('attivazioni\nperse', color: palette.textSecondary)),
        ]),
        for (final task in tasks)
          Padding(
            padding: const EdgeInsets.only(top: 6),
            child: Row(children: [
              Expanded(flex: 4, child: cell(task.name, align: TextAlign.left, bold: true)),
              Expanded(flex: 4, child: cell('${task.deadlineMisses}', color: task.deadlineMisses > 0 ? palette.fault : null, bold: task.deadlineMisses > 0)),
              Expanded(flex: 4, child: cell('${task.missedActivations}', color: task.missedActivations > 0 ? palette.fault : null, bold: task.missedActivations > 0)),
            ]),
          ),
      ]),
    );
  }
}
