/// Un orario della programmazione delle pulizie (GET /api/robot/schedules).
library;

class Schedule {
  const Schedule({
    required this.id,
    required this.days,
    required this.time,
    required this.enabled,
    this.lastRunAt,
    this.lastResult,
  });

  final int id;
  final List<bool> days; // 7 valori, da lunedì a domenica
  final String time; // "HH:MM", nell'ora locale del server
  final bool enabled;
  final int? lastRunAt; // secondi dal 1970
  final String? lastResult; // "ok", "retrying", oppure il motivo del fallimento

  static const String resultOk = 'ok';
  static const String resultRetrying = 'retrying';

  /// L'ultima pulizia programmata non è partita: l'app lo mostra in evidenza.
  bool get lastFailed => lastResult != null && lastResult != resultOk && lastResult != resultRetrying;

  factory Schedule.fromJson(Map<String, dynamic> json) => Schedule(
        id: (json['id'] as num).toInt(),
        days: [for (final day in json['days'] as List) day == true],
        time: json['time'] as String,
        enabled: json['enabled'] == true,
        lastRunAt: (json['last_run_at'] as num?)?.toInt(),
        lastResult: json['last_result'] as String?,
      );

  static Map<String, dynamic> body({List<bool>? days, String? time, bool? enabled}) => {
        'days': ?days,
        'time': ?time,
        'enabled': ?enabled,
      };
}
