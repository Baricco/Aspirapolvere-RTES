/// Gli avvisi (docs/app.md, sezione 10.3): un riquadro sotto la mappa, alto 0 quando non ce ne sono.
library;

import 'package:flutter/material.dart';

import '../config/app_colors.dart';
import '../models/robot_state.dart';
import '../models/schedule.dart';

class AppAlert {
  const AppAlert(this.title, {this.detail, this.isFault = false});

  final String title;
  final String? detail;
  final bool isFault; // rosso: guasto o errore; altrimenti ambra
}

/// Testi in italiano dei guasti chiesti dai task (drv_task_fault_t in driver/driver.h), con cosa fare.
const Map<int, (String, String)> _faultTexts = {
  1: ('Robot incastrato', 'Liberalo, poi premi {main_button}.'),
  2: ('Robot chiuso fra gli ostacoli', 'Spostalo in un punto libero, poi premi {main_button}.'),
  3: ('Uscita dalla base non riuscita', 'Togli gli ostacoli davanti alla base, poi premi {main_button}.'),
  4: ('Nessuna strada per la base', 'Libera il passaggio o riporta il robot sulla base.'),
  5: ('Aggancio alla base non riuscito', 'Controlla la base, poi premi Casa o rimetti il robot sulla base.'),
  6: ('Aggancio alla base troppo lungo', 'Controlla la base, poi premi Casa o rimetti il robot sulla base.'),
  7: ('Errore interno della navigazione', 'Azzera il guasto; se torna, riavvia il robot.'),
  8: ('Errore interno del movimento', 'Azzera il guasto; se torna, riavvia il robot.'),
  9: ('Posizione non ritrovata', 'Rimetti il robot sulla base.'),
  10: ('Spazzola centrale bloccata', 'Toglila e puliscila, poi premi {main_button}.'),
};

/// Titolo e consiglio di un guasto. Per i guasti del driver (codice 0) resta il testo del robot. Il consiglio nomina il
/// pulsante principale che c'è davvero: Riprendi se c'era una missione, Avvia se no.
(String, String?) faultText(RobotState state) {
  final known = _faultTexts[state.faultCode];
  if (known == null) return ('Guasto', state.faultText);
  final mainButton = state.hasMission ? 'Riprendi' : 'Avvia';
  return (known.$1, known.$2.replaceAll('{main_button}', mainButton));
}

/// Motivi per cui una pulizia programmata non parte, detti per esteso.
String scheduleFailureText(String reason) => switch (reason) {
      'robot_unreachable' => 'robot non raggiungibile',
      'no_result' => 'il robot non ha risposto',
      'remote_in_use' => 'telecomando in uso',
      'cause_still_present' => 'robot in guasto',
      'position_lost' => 'il robot non sa dove si trova',
      _ => reason,
    };

/// Gli avvisi da mostrare adesso. inRemoteScreen: nella schermata del telecomando servono anche contatti e batteria.
List<AppAlert> buildAlerts(
  RobotState? state, {
  String? username,
  bool restarting = false,
  List<Schedule> schedules = const [],
  bool inRemoteScreen = false,
}) {
  final alerts = <AppAlert>[];
  if (restarting) alerts.add(const AppAlert('Riavvio in corso', detail: 'Il robot riparte con la mappa scelta.'));
  if (state == null || !state.hasRobotData) return alerts;

  if (state.inFault) {
    final (title, detail) = faultText(state);
    alerts.add(AppAlert(title, detail: detail, isFault: true));
  }
  if (state.positionLost) {
    alerts.add(const AppAlert('Posizione persa', detail: 'Il robot non sa dove si trova: solo Casa lo riporta alla base.'));
  }
  if (state.remoteControlUser != null && state.remoteControlUser != username) {
    alerts.add(AppAlert('Telecomando in uso', detail: '${state.remoteControlUser} sta guidando il robot.'));
  }
  if (state.remoteControl && state.batteryLow) {
    alerts.add(const AppAlert('Batteria scarica',
        detail: 'In telecomando il robot non torna da solo alla base: riportalo tu, o premi Casa.'));
  }
  if (inRemoteScreen) {
    if (state.bumperLeft || state.bumperRight) alerts.add(const AppAlert('Paraurti premuto', detail: 'In avanti è bloccato: vai indietro o ruota.'));
    if (state.anyCliff) alerts.add(const AppAlert('Vuoto sotto il robot', detail: 'In avanti è bloccato: vai indietro o ruota.'));
    if (state.lifted) alerts.add(const AppAlert('Robot sollevato'));
  }
  if (state.mapSaveFailed) alerts.add(const AppAlert('Salvataggio della mappa non riuscito', isFault: true));
  if (state.online && state.mapFileCrc == null && !restarting) {
    alerts.add(const AppAlert('Mappa nuova, non ancora salvata', detail: 'Si salva alla fine dell\'esplorazione.'));
  }
  for (final schedule in schedules) {
    if (schedule.enabled && schedule.lastFailed) {
      alerts.add(AppAlert('Pulizia programmata delle ${schedule.time} non partita',
          detail: scheduleFailureText(schedule.lastResult!), isFault: true));
    }
  }
  return alerts;
}

class AlertsBox extends StatelessWidget {
  const AlertsBox({super.key, required this.alerts});

  final List<AppAlert> alerts;

  @override
  Widget build(BuildContext context) {
    final palette = AppPalette.of(context);
    // alto 0 quando non ci sono avvisi
    return AnimatedSize(
      duration: const Duration(milliseconds: 180),
      alignment: Alignment.topCenter,
      child: Column(mainAxisSize: MainAxisSize.min, children: [
        for (final alert in alerts)
          Padding(
            padding: const EdgeInsets.only(bottom: 10),
            child: _AlertTile(alert: alert, color: alert.isFault ? palette.fault : palette.warning),
          ),
      ]),
    );
  }
}

class _AlertTile extends StatelessWidget {
  const _AlertTile({required this.alert, required this.color});

  final AppAlert alert;
  final Color color;

  @override
  Widget build(BuildContext context) {
    return Container(
      width: double.infinity,
      padding: const EdgeInsets.symmetric(horizontal: 14, vertical: 10),
      decoration: BoxDecoration(color: color.withValues(alpha: 0.16), borderRadius: BorderRadius.circular(controlRadius)),
      child: Row(crossAxisAlignment: CrossAxisAlignment.start, children: [
        Icon(Icons.warning_amber_rounded, color: color, size: 22),
        const SizedBox(width: 10),
        Expanded(
          child: Column(crossAxisAlignment: CrossAxisAlignment.start, children: [
            Text(alert.title, style: TextStyle(color: color, fontWeight: FontWeight.w700, fontSize: 15)),
            if (alert.detail != null && alert.detail!.isNotEmpty)
              Text(alert.detail!, style: TextStyle(color: color.withValues(alpha: 0.9), fontSize: 14)),
          ]),
        ),
      ]),
    );
  }
}
