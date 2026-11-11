/// Date e ore scritte in italiano, senza librerie.
library;

const List<String> weekdayNames = ['lunedì', 'martedì', 'mercoledì', 'giovedì', 'venerdì', 'sabato', 'domenica'];
const List<String> weekdayLetters = ['L', 'M', 'M', 'G', 'V', 'S', 'D'];
const List<String> monthNames = [
  'gennaio', 'febbraio', 'marzo', 'aprile', 'maggio', 'giugno',
  'luglio', 'agosto', 'settembre', 'ottobre', 'novembre', 'dicembre',
];

String _two(int value) => value.toString().padLeft(2, '0');

/// "lunedì 9 novembre alle 08:30", nell'ora del telefono.
String longDate(int epochSeconds) {
  final date = DateTime.fromMillisecondsSinceEpoch(epochSeconds * 1000);
  return '${weekdayNames[date.weekday - 1]} ${date.day} ${monthNames[date.month - 1]} alle ${_two(date.hour)}:${_two(date.minute)}';
}

/// "9 nov 08:30:12".
String shortDate(int epochSeconds) {
  final date = DateTime.fromMillisecondsSinceEpoch(epochSeconds * 1000);
  return '${date.day} ${monthNames[date.month - 1].substring(0, 3)} ${_two(date.hour)}:${_two(date.minute)}:${_two(date.second)}';
}
