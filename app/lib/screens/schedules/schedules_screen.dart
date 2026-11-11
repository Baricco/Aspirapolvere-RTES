/// Programmazione delle pulizie (docs/app.md, sezioni 8.7 e 10.4): un riquadro per orario con i 7 quadratini dei
/// giorni, l'ora in grande, l'interruttore e l'ultimo esito. Un solo interruttore acceso alla volta.
library;

import 'package:flutter/material.dart';
import 'package:flutter_riverpod/flutter_riverpod.dart';

import '../../api/providers.dart';
import '../../config/app_colors.dart';
import '../../config/app_config.dart';
import '../../models/schedule.dart';
import '../../widgets/alerts_box.dart';
import '../../widgets/confirm_dialog.dart';
import '../../widgets/dates.dart';
import '../../widgets/glass_panel.dart';
import '../../widgets/main_button.dart';
import '../../widgets/state_interval.dart';

/// Opacità dei quadratini dei giorni: 1 se attivi, 0,5 se no.
const double dayActiveOpacity = 1;
const double dayInactiveOpacity = 0.5;

/// Lato dei quadratini dei giorni nei riquadri: i 7 giorni stanno su una riga accanto all'ora, come nel mockup.
const double scheduleCardDaySize = 20;

class SchedulesScreen extends ConsumerStatefulWidget {
  const SchedulesScreen({super.key});

  @override
  ConsumerState<SchedulesScreen> createState() => _SchedulesScreenState();
}

class _SchedulesScreenState extends ConsumerState<SchedulesScreen> with StateIntervalMixin {
  @override
  int get stateIntervalMs => stateIntervalOtherMs;

  Future<void> _change(Future<void> Function() request) async {
    await runCommand(ref, (_) => request(), title: 'Non riuscito');
    ref.invalidate(schedulesProvider);
  }

  /// Accendere un orario spegne l'altro (lo fa il server nella stessa operazione); spegnere quello acceso disattiva
  /// la programmazione.
  Future<void> _setEnabled(Schedule schedule, bool enabled) => _change(() => ref
      .read(apiClientProvider)
      .send('PUT', '/api/robot/schedules/${schedule.id}', Schedule.body(enabled: enabled)));

  Future<void> _edit(Schedule? schedule) async {
    final edited = await showDialog<_EditResult>(context: context, builder: (context) => _EditDialog(schedule: schedule));
    if (edited == null || !mounted) return;
    final api = ref.read(apiClientProvider);
    if (edited.delete) {
      final confirmed = await confirmWithWarning(context,
          title: 'Cancellare l\'orario delle ${schedule!.time}?', warning: 'L\'orario viene tolto dalla programmazione.');
      if (confirmed) await _change(() => api.send('DELETE', '/api/robot/schedules/${schedule.id}'));
    } else if (schedule == null) {
      await _change(() => api.send('POST', '/api/robot/schedules', Schedule.body(days: edited.days, time: edited.time, enabled: false)));
    } else {
      await _change(() => api.send('PUT', '/api/robot/schedules/${schedule.id}', Schedule.body(days: edited.days, time: edited.time)));
    }
  }

  @override
  Widget build(BuildContext context) {
    final schedules = ref.watch(schedulesProvider);
    final palette = AppPalette.of(context);
    final list = schedules.value ?? const <Schedule>[];
    return Scaffold(
      appBar: const ScreenHeader(title: 'Programmazione'),
      floatingActionButton: SizedBox(
        width: 164,
        child: ActionButton(label: 'Aggiungi', icon: Icons.add_rounded, style: ActionButtonStyle.primary, onPressed: () => _edit(null)),
      ),
      body: schedules.isLoading && list.isEmpty
          ? const Center(child: CircularProgressIndicator())
          : list.isEmpty
              ? Center(
                  child: Text('Nessun orario.\nAggiungine uno per far partire la pulizia da sola.',
                      textAlign: TextAlign.center, style: TextStyle(color: palette.textSecondary, fontSize: 16)))
              : ListView.separated(
                  padding: const EdgeInsets.fromLTRB(screenPadding, 6, screenPadding, 96),
                  itemCount: list.length,
                  separatorBuilder: (_, _) => const SizedBox(height: 12),
                  itemBuilder: (context, index) {
                    final schedule = list[index];
                    return _ScheduleCard(
                      schedule: schedule,
                      onTap: () => _edit(schedule),
                      onEnabled: (enabled) => _setEnabled(schedule, enabled),
                    );
                  },
                ),
    );
  }
}

/// L'ultimo esito di un orario, detto per esteso.
String lastResultText(Schedule schedule) {
  if (schedule.lastResult == null || schedule.lastRunAt == null) return 'Mai eseguita';
  if (schedule.lastResult == Schedule.resultOk) return 'Ultima: avviata ${longDate(schedule.lastRunAt!)}';
  if (schedule.lastResult == Schedule.resultRetrying) return 'Ultima: non ancora partita, nuovo tentativo fra un minuto';
  return 'Ultima: non partita, ${scheduleFailureText(schedule.lastResult!)} dopo 10 tentativi';
}

class _ScheduleCard extends StatelessWidget {
  const _ScheduleCard({required this.schedule, required this.onTap, required this.onEnabled});

  final Schedule schedule;
  final VoidCallback onTap;
  final void Function(bool enabled) onEnabled;

  @override
  Widget build(BuildContext context) {
    final palette = AppPalette.of(context);
    return GlassPanel(
      padding: EdgeInsets.zero,
      child: InkWell(
        onTap: onTap,
        child: Padding(
          padding: const EdgeInsets.fromLTRB(18, 14, 14, 14),
          child: Column(crossAxisAlignment: CrossAxisAlignment.start, children: [
            Row(children: [
              // a sinistra i 7 quadratini dei giorni, a destra l'ora in grande e l'interruttore
              Expanded(child: DaySquares(days: schedule.days, size: scheduleCardDaySize)),
              const SizedBox(width: 8),
              Text(schedule.time,
                  style: const TextStyle(fontSize: 38, fontWeight: FontWeight.w700, height: 1, fontFeatures: [FontFeature.tabularFigures()])),
              const SizedBox(width: 10),
              Switch(value: schedule.enabled, onChanged: onEnabled),
            ]),
            const SizedBox(height: 10),
            Text(lastResultText(schedule),
                style: TextStyle(fontSize: 14.5, color: schedule.lastFailed ? palette.fault : palette.textSecondary)),
          ]),
        ),
      ),
    );
  }
}

/// I 7 quadratini dei giorni, da lunedì a domenica. Con onToggle si possono premere (nella modifica).
class DaySquares extends StatelessWidget {
  const DaySquares({super.key, required this.days, this.onToggle, this.size = 26});

  final List<bool> days;
  final void Function(int day)? onToggle;
  final double size;

  @override
  Widget build(BuildContext context) {
    final palette = AppPalette.of(context);
    return Wrap(spacing: size * 0.16, runSpacing: 4, children: [
      for (var day = 0; day < 7; day++)
        Opacity(
          opacity: days[day] ? dayActiveOpacity : dayInactiveOpacity,
          child: GestureDetector(
            onTap: onToggle == null ? null : () => onToggle!(day),
            child: Container(
              width: size,
              height: size + 4,
              alignment: Alignment.center,
              decoration: BoxDecoration(color: palette.primary, borderRadius: BorderRadius.circular(size * 0.24)),
              child: Text(weekdayLetters[day],
                  style: TextStyle(color: palette.onPrimary, fontWeight: FontWeight.w800, fontSize: size * 0.5)),
            ),
          ),
        ),
    ]);
  }
}

class _EditResult {
  _EditResult({this.days = const [], this.time = '', this.delete = false});
  final List<bool> days;
  final String time;
  final bool delete;
}

/// Aggiunta o modifica di un orario: giorni e ora; nella modifica anche la cancellazione.
class _EditDialog extends StatefulWidget {
  const _EditDialog({required this.schedule});

  final Schedule? schedule;

  @override
  State<_EditDialog> createState() => _EditDialogState();
}

class _EditDialogState extends State<_EditDialog> {
  late List<bool> _days = List.of(widget.schedule?.days ?? List.filled(7, true));
  late TimeOfDay _time = widget.schedule == null
      ? const TimeOfDay(hour: 9, minute: 0)
      : TimeOfDay(hour: int.parse(widget.schedule!.time.split(':')[0]), minute: int.parse(widget.schedule!.time.split(':')[1]));

  String get _timeText => '${_time.hour.toString().padLeft(2, '0')}:${_time.minute.toString().padLeft(2, '0')}';

  @override
  Widget build(BuildContext context) {
    final palette = AppPalette.of(context);
    return AlertDialog(
      title: Text(widget.schedule == null ? 'Nuovo orario' : 'Modifica orario'),
      content: Column(mainAxisSize: MainAxisSize.min, crossAxisAlignment: CrossAxisAlignment.start, children: [
        Text('Giorni', style: TextStyle(color: palette.textSecondary)),
        const SizedBox(height: 8),
        DaySquares(days: _days, size: 28, onToggle: (day) => setState(() => _days = [..._days]..[day] = !_days[day])),
        const SizedBox(height: 18),
        Text('Ora (del server, ora italiana)', style: TextStyle(color: palette.textSecondary)),
        const SizedBox(height: 4),
        InkWell(
          borderRadius: BorderRadius.circular(controlRadius),
          onTap: () async {
            final picked = await showTimePicker(
              context: context,
              initialTime: _time,
              builder: (context, child) =>
                  MediaQuery(data: MediaQuery.of(context).copyWith(alwaysUse24HourFormat: true), child: child!),
            );
            if (picked != null) setState(() => _time = picked);
          },
          child: Padding(
            padding: const EdgeInsets.symmetric(vertical: 6),
            child: Row(mainAxisSize: MainAxisSize.min, children: [
              Text(_timeText, style: const TextStyle(fontSize: 40, fontWeight: FontWeight.w700)),
              const SizedBox(width: 10),
              Icon(Icons.edit_outlined, color: palette.textSecondary),
            ]),
          ),
        ),
      ]),
      actionsAlignment: MainAxisAlignment.spaceBetween,
      actions: [
        if (widget.schedule != null)
          TextButton(
            style: TextButton.styleFrom(foregroundColor: palette.fault),
            onPressed: () => Navigator.of(context).pop(_EditResult(delete: true)),
            child: const Text('Cancella'),
          )
        else
          const SizedBox.shrink(),
        Row(mainAxisSize: MainAxisSize.min, children: [
          TextButton(onPressed: () => Navigator.of(context).pop(), child: const Text('Annulla')),
          FilledButton(
            onPressed: _days.contains(true) ? () => Navigator.of(context).pop(_EditResult(days: _days, time: _timeText)) : null,
            child: const Text('Salva'),
          ),
        ]),
      ],
    );
  }
}
