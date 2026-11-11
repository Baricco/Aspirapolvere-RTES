// Prove dei modelli: classi Dart dai JSON del server (stato del robot, mappe, orari, registro) e modifica delle zone.
import 'dart:convert';

import 'package:flutter_test/flutter_test.dart';
import 'package:maurice/map/zone_editor.dart';
import 'package:maurice/models/log_entry.dart';
import 'package:maurice/models/map_info.dart';
import 'package:maurice/models/robot_state.dart';
import 'package:maurice/models/schedule.dart';
import 'package:maurice/widgets/alerts_box.dart';

// lo stato di esempio di docs/app.md (sezione 4.1), con i campi aggiunti dal server
const String stateJson = '''
{"format":1,"robot_time":1762700000,"uptime_s":1834,"remote_control":false,
 "navigation":{"state":"follow_path","mission":"clean","interrupted_mission":"none","mission_number":3,"mission_duration_s":312},
 "objective":{"type":"move","x":1.50,"y":-0.30,"theta_deg":0.0},
 "position":{"x":1.23,"y":-0.40,"theta_deg":12.3,"v":0.21,"omega":0.05,"valid":true},
 "map":{"state":"normal","save_failed":false,"started_on_base":true,"file_crc":"9c41e2a0",
        "forbidden_zones":[{"x":-0.365,"y":-0.22,"w":0.26,"h":0.44},{"x":1.0,"y":1.0,"w":0.5,"h":0.5}]},
 "safety":{"errors":["obstacle"],"max_v_forward":0.30,"max_v_backward":0.50},
 "lidar":{"valid":true,"sector_distances_mm":[0,0,0,0,0,0,0,0,0,0,0,0],"corridor_front_mm":640,"corridor_back_mm":1200},
 "battery":{"voltage":15.1,"percent":78,"level":"ok","charging":false,"charge_complete":false,"temperature_c":31},
 "motors":{"side_brush":60,"main_brush":60,"suction_rpm":12000},
 "driver":{"phase":"ready","fault":null,"limits":[],"on_base":false,"lifted":false,
           "docking":{"result":"idle","phase":"none","attempt":0},"lidar":{"ok":true,"rotation_hz":6.0}},
 "sensors":{"bumper_left":false,"bumper_right":false,"cliff":[false,false,false,false],"dustbin":true,
            "currents_ma":{"wheel_left":180,"wheel_right":175,"side_brush":90,"main_brush":220,"suction":600}},
 "tasks":[{"name":"movement","deadline_misses":0,"missed_activations":2}],
 "online":true,"last_contact":1762700001.5,"remote_control_user":null,"map_version":1794313279287,"state_number":42}
''';

RobotState stateWith(void Function(Map<String, dynamic> json) change) {
  final json = jsonDecode(stateJson) as Map<String, dynamic>;
  change(json);
  return RobotState(json);
}

void main() {
  group('stato del robot', () {
    final state = RobotState(jsonDecode(stateJson) as Map<String, dynamic>);

    test('campi del robot e del server', () {
      expect(state.online, true);
      expect(state.stateNumber, 42);
      expect(state.mapVersion, 1794313279287);
      expect(state.remoteControlUser, null);
      expect(state.navigationState, 'follow_path');
      expect(state.mission, 'clean');
      expect(state.missionNumber, 3);
      expect([state.x, state.y, state.thetaDeg], [1.23, -0.40, 12.3]);
      expect(state.positionValid, true);
      expect(state.batteryPercent, 78);
      expect(state.mapFileCrc, '9c41e2a0');
      expect(state.inFault, false);
      expect(state.brushesOn && state.suctionOn, true);
      expect(state.tasks.single.name, 'movement');
      expect(state.tasks.single.missedActivations, 2);
    });

    test('riga di stato in inglese: missione, stato, durata', () {
      expect(state.statusLine, 'Cleaning · Following path · 5:12');
      expect(stateWith((json) => json['navigation'] = {'state': 'sleep', 'mission': 'none', 'mission_duration_s': 0}).statusLine,
          'Idle · Sleeping · 0:00');
    });

    test('zone: la prima è quella della base', () {
      expect(state.forbiddenZones.length, 2);
      expect(state.userZones, [const Zone(1.0, 1.0, 0.5, 0.5)]);
      // non partito dalla base: la zona della base non c'è
      final offBase = stateWith((json) => (json['map'] as Map)['started_on_base'] = false);
      expect(offBase.userZones.length, 2);
    });

    test('il pulsante principale segue lo stato', () {
      MainAction action(String navigationState, String mission) =>
          stateWith((json) => json['navigation'] = {'state': navigationState, 'mission': mission}).mainAction;
      expect(action('sleep', 'none'), MainAction.start);
      expect(action('follow_path', 'clean'), MainAction.pause);
      expect(action('return_home', 'return_home'), MainAction.pause);
      expect(action('pause', 'clean'), MainAction.resume);
      expect(action('failure', 'clean'), MainAction.resume); // guasto a metà pulizia: Riprendi la continua
      expect(action('failure', 'none'), MainAction.start);
      expect(action('remote', 'clean'), MainAction.resume);
      expect(action('remote', 'none'), MainAction.start);
    });

    test('guasto con codice e testo', () {
      final fault = stateWith((json) {
        (json['driver'] as Map)['phase'] = 'fault';
        (json['driver'] as Map)['fault'] = {'code': 10, 'text': 'task fault 10: main brush blocked'};
      });
      expect(fault.inFault, true);
      expect(fault.faultCode, 10);
      expect(faultText(fault), ('Spazzola centrale bloccata', 'Toglila e puliscila, poi premi Riprendi.'));
      final faultAtRest = stateWith((json) {
        json['navigation'] = {'state': 'failure', 'mission': 'none'};
        (json['driver'] as Map)['phase'] = 'fault';
        (json['driver'] as Map)['fault'] = {'code': 1, 'text': 'task fault 1'};
      });
      expect(faultText(faultAtRest), ('Robot incastrato', 'Liberalo, poi premi Avvia.'));
      // guasto del driver, senza codice di un task: resta il testo del robot
      final driverFault = stateWith((json) {
        (json['driver'] as Map)['phase'] = 'fault';
        (json['driver'] as Map)['fault'] = {'code': 0, 'text': 'sensori muti'};
      });
      expect(faultText(driverFault), ('Guasto', 'sensori muti'));
    });

    test('stato senza dati del robot (server appena installato)', () {
      final empty = RobotState({'online': false, 'last_contact': null, 'remote_control_user': null, 'map_version': 5, 'state_number': 1});
      expect(empty.hasRobotData, false);
      expect(empty.online, false);
      expect(empty.navigationState, 'sleep');
      expect(empty.forbiddenZones, isEmpty);
      expect(empty.batteryPercent, null);
      expect(empty.tasks, isEmpty);
      expect(buildAlerts(empty), isEmpty);
    });

    test('avvisi', () {
      List<String> titles(RobotState robotState, {String? username, bool remote = false, List<Schedule> schedules = const []}) => [
            for (final alert in buildAlerts(robotState, username: username, inRemoteScreen: remote, schedules: schedules)) alert.title
          ];
      expect(titles(state, username: 'anna'), isEmpty); // nessun avviso: il riquadro è alto 0
      expect(titles(stateWith((json) => (json['map'] as Map)['state'] = 'lost')), ['Posizione persa']);
      expect(titles(stateWith((json) => json['remote_control_user'] = 'bruno'), username: 'anna'), ['Telecomando in uso']);
      expect(titles(stateWith((json) => json['remote_control_user'] = 'anna'), username: 'anna'), isEmpty);
      expect(titles(stateWith((json) => (json['map'] as Map).remove('file_crc'))), ['Mappa nuova, non ancora salvata']);
      expect(
          titles(stateWith((json) {
            json['remote_control'] = true;
            (json['battery'] as Map)['level'] = 'low';
          })),
          ['Batteria scarica']);
      expect(titles(stateWith((json) => (json['sensors'] as Map)['bumper_left'] = true), remote: true), ['Paraurti premuto']);
      const failed = Schedule(id: 1, days: [true, true, true, true, true, true, true], time: '08:30', enabled: true,
          lastRunAt: 1, lastResult: 'robot_unreachable');
      expect(titles(state, schedules: [failed]), ['Pulizia programmata delle 08:30 non partita']);
      expect(buildAlerts(null, restarting: true).single.title, 'Riavvio in corso');
    });
  });

  group('mappe, orari, registro', () {
    test('mappa', () {
      final map = MapInfo.fromJson(
          jsonDecode('{"id":4,"name":"Piano terra","selected":true,"saved_at":1762700000,"size_bytes":812}') as Map<String, dynamic>);
      expect([map.id, map.name, map.selected, map.savedAt, map.sizeBytes], [4, 'Piano terra', true, 1762700000, 812]);
    });

    test('orario', () {
      final schedule = Schedule.fromJson(jsonDecode(
              '{"id":2,"days":[true,false,true,false,true,false,false],"time":"08:30","enabled":true,"last_run_at":1762700000,"last_result":"ok"}')
          as Map<String, dynamic>);
      expect(schedule.days, [true, false, true, false, true, false, false]);
      expect([schedule.time, schedule.enabled, schedule.lastFailed], ['08:30', true, false]);
      Schedule withResult(String? result) => Schedule(id: 1, days: schedule.days, time: '10:00', enabled: false, lastResult: result);
      expect(withResult(null).lastFailed, false);
      expect(withResult('retrying').lastFailed, false);
      expect(withResult('robot_unreachable').lastFailed, true);
      expect(Schedule.body(enabled: true), {'enabled': true});
      expect(Schedule.body(days: schedule.days, time: '09:15'), {'days': schedule.days, 'time': '09:15'});
    });

    test('riga del registro', () {
      final entry = LogEntry.fromJson(jsonDecode(
              '{"id":9,"time":1762700000,"user":null,"scheduled":true,"type":"start","params":null,"status":"robot_unreachable","reason":null}')
          as Map<String, dynamic>);
      expect([entry.scheduled, entry.user, entry.ok, entry.type], [true, null, false, 'start']);
      final move = LogEntry.fromJson(jsonDecode(
              '{"id":10,"time":1762700001,"user":"anna","scheduled":false,"type":"remote_move","params":{"v":0.2,"omega":0.0},"status":"ok","reason":null}')
          as Map<String, dynamic>);
      expect(move.ok, true);
      expect(move.params!['v'], 0.2);
    });
  });

  group('modifica delle zone', () {
    test('disegnare, spostare, ridimensionare, cancellare', () {
      final editor = ZoneEditor(const []);
      // trascinando su un punto vuoto nasce una zona
      editor.startDrag(const Offset(1, 1), 0.1);
      editor.updateDrag(const Offset(1.01, 1.01)); // troppo poco: ancora niente
      expect(editor.zones, isEmpty);
      editor.updateDrag(const Offset(1.5, 1.4));
      editor.endDrag();
      expect(editor.zones, [const Zone(1, 1, 0.5, 0.4)]);
      // trascinando dentro si sposta
      editor.startDrag(const Offset(1.2, 1.2), 0.1);
      editor.updateDrag(const Offset(1.7, 1.0));
      editor.endDrag();
      expect(editor.zones, [const Zone(1.5, 0.8, 0.5, 0.4)]);
      // trascinando un angolo si ridimensiona: l'angolo opposto resta fermo
      editor.startDrag(const Offset(2.0, 1.2), 0.1);
      editor.updateDrag(const Offset(2.5, 1.6));
      editor.endDrag();
      expect(editor.zones, [const Zone(1.5, 0.8, 1.0, 0.8)]);
      // la x sta in alto a destra
      expect(editor.deleteHit(const Offset(2.5, 1.6), 0.05), 0);
      expect(editor.deleteHit(const Offset(1.5, 0.8), 0.05), -1);
      editor.removeAt(0);
      expect(editor.zones, isEmpty);
    });

    test('al massimo 15 zone, e dentro la mappa', () {
      final editor = ZoneEditor([for (var i = 0; i < 15; i++) Zone(i.toDouble(), 20, 0.5, 0.5)]);
      expect(editor.canAdd, false);
      editor.startDrag(const Offset(-5, -5), 0.1);
      editor.updateDrag(const Offset(-4, -4));
      editor.endDrag();
      expect(editor.zones.length, 15);
      // una zona spinta fuori resta dentro la griglia (±25,6 m)
      final edge = ZoneEditor(const [Zone(24, 24, 1, 1)]);
      edge.startDrag(const Offset(24.5, 24.5), 0.01);
      edge.updateDrag(const Offset(30, 30));
      edge.endDrag();
      expect(edge.zones.single.x + edge.zones.single.w, lessThanOrEqualTo(25.6 + 1e-9));
      expect(edge.zones.single.y + edge.zones.single.h, lessThanOrEqualTo(25.6 + 1e-9));
    });
  });
}
