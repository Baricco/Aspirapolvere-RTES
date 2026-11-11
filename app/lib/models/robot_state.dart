/// Stato del robot come arriva da GET /api/robot/state (docs/app.md, sezione 4): il JSON del robot più i campi del server.
library;

/// Una zona proibita, in metri sulla mappa: angolo con x e y minime, larghezza e altezza.
class Zone {
  const Zone(this.x, this.y, this.w, this.h);

  final double x, y, w, h;

  factory Zone.fromJson(Map<String, dynamic> json) => Zone(
        (json['x'] as num).toDouble(),
        (json['y'] as num).toDouble(),
        (json['w'] as num).toDouble(),
        (json['h'] as num).toDouble(),
      );

  Map<String, dynamic> toJson() => {'x': x, 'y': y, 'w': w, 'h': h};

  @override
  bool operator ==(Object other) =>
      other is Zone && other.x == x && other.y == y && other.w == w && other.h == h;

  @override
  int get hashCode => Object.hash(x, y, w, h);
}

/// Scadenze mancate e attivazioni perse di un task real-time.
class TaskStats {
  const TaskStats(this.name, this.deadlineMisses, this.missedActivations);

  final String name;
  final int deadlineMisses;
  final int missedActivations;
}

/// Cosa fa il pulsante principale secondo lo stato.
enum MainAction { start, pause, resume }

Map<String, dynamic> _object(dynamic value) => value is Map<String, dynamic> ? value : const {};

double _number(dynamic value) => value is num ? value.toDouble() : 0;

class RobotState {
  RobotState(this.raw);

  /// Il JSON intero, per la schermata di diagnostica.
  final Map<String, dynamic> raw;

  // --- campi aggiunti dal server ---
  bool get online => raw['online'] == true;
  double? get lastContact => (raw['last_contact'] as num?)?.toDouble();
  String? get remoteControlUser => raw['remote_control_user'] as String?;
  int get mapVersion => (raw['map_version'] as num?)?.toInt() ?? 0;
  int get stateNumber => (raw['state_number'] as num?)?.toInt() ?? 0;

  /// False finché il robot non ha mai mandato uno stato al server.
  bool get hasRobotData => raw.containsKey('navigation');

  // --- navigazione ---
  Map<String, dynamic> get _navigation => _object(raw['navigation']);
  String get navigationState => _navigation['state'] as String? ?? 'sleep';
  String get mission => _navigation['mission'] as String? ?? 'none';
  int get missionNumber => (_navigation['mission_number'] as num?)?.toInt() ?? 0;
  int get missionDurationS => (_navigation['mission_duration_s'] as num?)?.toInt() ?? 0;
  bool get hasMission => mission != 'none';
  bool get remoteControl => raw['remote_control'] == true;
  int get uptimeS => (raw['uptime_s'] as num?)?.toInt() ?? 0;

  // --- posizione ---
  Map<String, dynamic> get _position => _object(raw['position']);
  double get x => _number(_position['x']);
  double get y => _number(_position['y']);
  double get thetaDeg => _number(_position['theta_deg']);
  double get v => _number(_position['v']);
  double get omega => _number(_position['omega']);
  bool get positionValid => _position['valid'] == true;

  // --- mappa ---
  Map<String, dynamic> get _map => _object(raw['map']);
  String get mapState => _map['state'] as String? ?? 'normal';
  bool get mapSaveFailed => _map['save_failed'] == true;
  bool get startedOnBase => _map['started_on_base'] == true;
  String? get mapFileCrc => _map['file_crc'] as String?;

  /// Con la posizione persa solo Casa fa qualcosa.
  bool get positionLost => mapState == 'lost' || mapState == 'not_found';

  /// Tutte le zone proibite: la prima è quella della base, se il robot è partito dalla base.
  List<Zone> get forbiddenZones => [
        for (final zone in (_map['forbidden_zones'] as List? ?? const []))
          if (zone is Map<String, dynamic>) Zone.fromJson(zone),
      ];

  /// Le zone dell'utente, senza quella della base.
  List<Zone> get userZones {
    final zones = forbiddenZones;
    return startedOnBase && zones.isNotEmpty ? zones.sublist(1) : zones;
  }

  // --- batteria ---
  Map<String, dynamic> get _battery => _object(raw['battery']);
  int? get batteryPercent => (_battery['percent'] as num?)?.toInt();
  String get batteryLevel => _battery['level'] as String? ?? 'ok';
  bool get charging => _battery['charging'] == true;
  bool get batteryLow => batteryLevel != 'ok';

  // --- driver ---
  Map<String, dynamic> get _driver => _object(raw['driver']);
  bool get onBase => _driver['on_base'] == true;
  bool get lifted => _driver['lifted'] == true;
  bool get inFault => _driver['phase'] == 'fault' || navigationState == 'failure';
  String? get faultText => _object(_driver['fault'])['text'] as String?;
  int? get faultCode => (_object(_driver['fault'])['code'] as num?)?.toInt();
  List<String> get driverLimits => [for (final limit in (_driver['limits'] as List? ?? const [])) '$limit'];

  // --- motori della pulizia ---
  Map<String, dynamic> get _motors => _object(raw['motors']);
  bool get brushesOn => _number(_motors['main_brush']) != 0 || _number(_motors['side_brush']) != 0;
  bool get suctionOn => _number(_motors['suction_rpm']) != 0;

  // --- sensori ---
  Map<String, dynamic> get _sensors => _object(raw['sensors']);
  bool get bumperLeft => _sensors['bumper_left'] == true;
  bool get bumperRight => _sensors['bumper_right'] == true;
  bool get dustbin => _sensors['dustbin'] != false;
  bool get anyCliff => (_sensors['cliff'] as List? ?? const []).contains(true);

  List<TaskStats> get tasks => [
        for (final task in (raw['tasks'] as List? ?? const []))
          if (task is Map<String, dynamic>)
            TaskStats(
              task['name'] as String? ?? '',
              (task['deadline_misses'] as num?)?.toInt() ?? 0,
              (task['missed_activations'] as num?)?.toInt() ?? 0,
            ),
      ];

  /// A riposo sulla base: solo così si può cambiare mappa.
  bool get restingOnBase => navigationState == 'sleep' && onBase;

  /// Il pulsante principale: Avvia, Pausa o Riprendi secondo lo stato.
  MainAction get mainAction {
    switch (navigationState) {
      case 'sleep':
        return MainAction.start;
      case 'pause':
        return MainAction.resume;
      case 'failure':
      case 'remote':
        // dopo il guasto o il telecomando una missione in corso si riprende, altrimenti se ne avvia una nuova
        return hasMission ? MainAction.resume : MainAction.start;
      default:
        return MainAction.pause;
    }
  }

  /// La riga di stato, in inglese: missione · stato · durata, per esempio "Cleaning · Following path · 5:12".
  String get statusLine => '$missionLabel · $stateLabel · $durationLabel';

  String get missionLabel => _missionLabels[mission] ?? mission;
  String get stateLabel => _stateLabels[navigationState] ?? navigationState;

  String get durationLabel {
    final seconds = missionDurationS;
    return '${seconds ~/ 60}:${(seconds % 60).toString().padLeft(2, '0')}';
  }

  static const Map<String, String> _missionLabels = {
    'none': 'Idle',
    'explore': 'Exploring',
    'follow_borders': 'Cleaning edges',
    'clean': 'Cleaning',
    'return_home': 'Returning home',
  };

  static const Map<String, String> _stateLabels = {
    'wait': 'Waiting',
    'explore': 'Planning exploration',
    'clean': 'Planning strips',
    'follow_borders': 'Planning edges',
    'return_home': 'Planning return',
    'hook_up': 'Docking',
    'follow_path': 'Following path',
    'unlock': 'Unlocking',
    'sleep': 'Sleeping',
    'align': 'Aligning',
    'failure': 'Fault',
    'border_lap': 'Following edge',
    'leave_base': 'Leaving base',
    'cross': 'Crossing obstacle',
    'pause': 'Paused',
    'remote': 'Remote control',
  };
}
