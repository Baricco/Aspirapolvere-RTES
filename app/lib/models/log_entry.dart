/// Una riga del registro dei comandi (GET /api/robot/log).
library;

class LogEntry {
  const LogEntry({
    required this.id,
    required this.time,
    required this.user,
    required this.scheduled,
    required this.type,
    required this.params,
    required this.status,
    required this.reason,
  });

  final int id;
  final int time; // secondi dal 1970
  final String? user; // vuoto: programmazione, oppure utente tolto
  final bool scheduled; // comando mandato dalla programmazione delle pulizie
  final String type;
  final Map<String, dynamic>? params;
  final String status; // ok, rejected, robot_unreachable, no_result
  final String? reason;

  bool get ok => status == 'ok';

  factory LogEntry.fromJson(Map<String, dynamic> json) => LogEntry(
        id: (json['id'] as num).toInt(),
        time: (json['time'] as num).toInt(),
        user: json['user'] as String?,
        scheduled: json['scheduled'] == true,
        type: json['type'] as String,
        params: json['params'] is Map<String, dynamic> ? json['params'] as Map<String, dynamic> : null,
        status: json['status'] as String,
        reason: json['reason'] as String?,
      );
}
