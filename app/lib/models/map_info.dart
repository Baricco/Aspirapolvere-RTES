/// Una mappa salvata sul server (GET /api/robot/maps).
library;

class MapInfo {
  const MapInfo({required this.id, required this.name, required this.selected, required this.savedAt, required this.sizeBytes});

  final int id;
  final String name;
  final bool selected; // è quella sul robot
  final int savedAt; // secondi dal 1970
  final int sizeBytes;

  factory MapInfo.fromJson(Map<String, dynamic> json) => MapInfo(
        id: (json['id'] as num).toInt(),
        name: json['name'] as String,
        selected: json['selected'] == true,
        savedAt: (json['saved_at'] as num?)?.toInt() ?? 0,
        sizeBytes: (json['size_bytes'] as num?)?.toInt() ?? 0,
      );
}
