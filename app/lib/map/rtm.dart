/// Decodifica del formato RTM1 delle mappe (docs/formato-mappa.md) e mappa dal vivo con le differenze
/// (docs/app.md, sezione 7.3).
library;

import 'dart:typed_data';

const int fineMapCellNumber = 1024; // celle per lato della griglia fine
const double fineMapCellSize = 0.05; // lato di una cella, in metri
const int fineMapOrigin = fineMapCellNumber ~/ 2; // la cella dell'origine (la base di ricarica)
const int maxUserZones = 15;

const int mapCatUnknown = 0;
const int mapCatFree = 1;
const int mapCatOccupied = 2;
const int mapCatCliff = 3; // vuoto degli anti-caduta

const int _headerLength = 13; // numero magico, rettangolo (4 uint16), numero di zone
const int _zoneLength = 8;
const int _crcLength = 4;
const int _runCountMask = 0x3F;
const int _runCategoryShift = 6;

/// Il file non supera i controlli del formato.
class MapFormatException implements Exception {
  MapFormatException(this.message);
  final String message;
  @override
  String toString() => 'MapFormatException: $message';
}

/// Un rettangolo di celle fini a categorie, con le zone proibite dell'utente (in celle fini).
class RtmMap {
  RtmMap(this.x, this.y, this.width, this.height, this.zones, this.cells);

  final int x, y, width, height;
  final List<List<int>> zones; // ogni zona: x, y, larghezza, altezza
  final Uint8List cells; // larghezza * altezza categorie, riga dopo riga
}

final Uint32List _crcTable = _buildCrcTable();

Uint32List _buildCrcTable() {
  final table = Uint32List(256);
  for (var i = 0; i < 256; i++) {
    var value = i;
    for (var bit = 0; bit < 8; bit++) {
      value = (value & 1) != 0 ? 0xEDB88320 ^ (value >>> 1) : value >>> 1;
    }
    table[i] = value;
  }
  return table;
}

/// CRC-32 standard (lo stesso di zlib): polinomio riflesso 0xEDB88320.
int crc32(Uint8List data, [int start = 0, int? end]) {
  var crc = 0xFFFFFFFF;
  for (var i = start; i < (end ?? data.length); i++) {
    crc = _crcTable[(crc ^ data[i]) & 0xFF] ^ (crc >>> 8);
  }
  return (crc ^ 0xFFFFFFFF) & 0xFFFFFFFF;
}

int _u16(Uint8List data, int offset) => data[offset] | data[offset + 1] << 8;

int _u32(Uint8List data, int offset) =>
    (data[offset] | data[offset + 1] << 8 | data[offset + 2] << 16 | data[offset + 3] << 24) & 0xFFFFFFFF;

/// Decodifica un file RTM1 con tutti i controlli di docs/formato-mappa.md.
RtmMap decodeRtm(Uint8List data) {
  if (data.length < _headerLength + _crcLength) throw MapFormatException('file troppo corto');
  if (data[0] != 0x52 || data[1] != 0x54 || data[2] != 0x4D || data[3] != 0x31) {
    throw MapFormatException('numero magico sbagliato');
  }
  final x = _u16(data, 4), y = _u16(data, 6), width = _u16(data, 8), height = _u16(data, 10);
  final zoneCount = data[12];
  if (width == 0 || height == 0 || x + width > fineMapCellNumber || y + height > fineMapCellNumber) {
    throw MapFormatException('il rettangolo esce dalla griglia');
  }
  if (zoneCount > maxUserZones) throw MapFormatException('troppe zone');
  final end = data.length - _crcLength;
  var offset = _headerLength;
  if (offset + zoneCount * _zoneLength > end) throw MapFormatException('file troppo corto');
  final zones = <List<int>>[];
  for (var i = 0; i < zoneCount; i++, offset += _zoneLength) {
    final zone = [_u16(data, offset), _u16(data, offset + 2), _u16(data, offset + 4), _u16(data, offset + 6)];
    if (zone[0] + zone[2] > fineMapCellNumber || zone[1] + zone[3] > fineMapCellNumber) {
      throw MapFormatException('una zona esce dalla griglia');
    }
    zones.add(zone);
  }
  final total = width * height;
  final cells = Uint8List(total);
  var done = 0;
  while (done < total) {
    if (offset >= end) throw MapFormatException('le sequenze finiscono prima delle celle');
    final run = data[offset++];
    final count = run & _runCountMask;
    if (count == 0 || count > total - done) throw MapFormatException('sequenza non valida');
    cells.fillRange(done, done + count, run >> _runCategoryShift);
    done += count;
  }
  if (offset != end) throw MapFormatException('byte in più dopo le celle');
  if (_u32(data, end) != crc32(data, 0, end)) throw MapFormatException('CRC sbagliato');
  return RtmMap(x, y, width, height, zones, cells);
}

const int liveMessageFull = 0; // la mappa intera, che sostituisce quella dell'app
const int liveMessagePatches = 1; // i rettangoli cambiati, da incollare in ordine
const int _livePartHeaderLength = 12; // versione uint64, lunghezza uint32

/// La mappa dal vivo: tutta la griglia fine a categorie, aggiornata con i messaggi di GET /api/robot/map.
class LiveMap {
  LiveMap() : grid = Uint8List(fineMapCellNumber * fineMapCellNumber);

  final Uint8List grid;

  /// Versione della mappa sul server a cui è aggiornata questa copia; null finché non è arrivata la prima.
  int? version;

  /// Il rettangolo delle celle non sconosciute (in celle fini), null se la mappa è vuota.
  int? minX, minY, maxX, maxY;

  bool get isEmpty => minX == null;

  /// Applica un messaggio: la mappa intera sostituisce tutto, le differenze si incollano sopra.
  void applyMessage(Uint8List message) {
    if (message.isEmpty) throw MapFormatException('messaggio vuoto');
    final kind = message[0];
    if (kind != liveMessageFull && kind != liveMessagePatches) throw MapFormatException('tipo di messaggio sconosciuto');
    final parts = <(int, RtmMap)>[];
    var offset = 1;
    while (offset < message.length) {
      if (offset + _livePartHeaderLength > message.length) throw MapFormatException('messaggio troncato');
      // la versione è un uint64: in JavaScript gli interi arrivano a 2^53, più che abbastanza per i millisecondi
      final partVersion = _u32(message, offset) + _u32(message, offset + 4) * 0x100000000;
      final length = _u32(message, offset + 8);
      offset += _livePartHeaderLength;
      if (offset + length > message.length) throw MapFormatException('messaggio troncato');
      parts.add((partVersion, decodeRtm(Uint8List.sublistView(message, offset, offset + length))));
      offset += length;
    }
    if (kind == liveMessageFull) {
      if (parts.length != 1) throw MapFormatException('la mappa intera ha una parte sola');
      grid.fillRange(0, grid.length, mapCatUnknown);
      minX = minY = maxX = maxY = null;
    }
    for (final (partVersion, rectangle) in parts) {
      paste(rectangle);
      version = partVersion;
    }
  }

  /// Incolla un rettangolo sulla griglia e allarga il rettangolo delle celle viste.
  void paste(RtmMap rectangle) {
    for (var row = 0; row < rectangle.height; row++) {
      final start = (rectangle.y + row) * fineMapCellNumber + rectangle.x;
      grid.setRange(start, start + rectangle.width, rectangle.cells, row * rectangle.width);
    }
    for (var row = 0; row < rectangle.height; row++) {
      for (var column = 0; column < rectangle.width; column++) {
        if (rectangle.cells[row * rectangle.width + column] == mapCatUnknown) continue;
        final cellX = rectangle.x + column, cellY = rectangle.y + row;
        minX = minX == null || cellX < minX! ? cellX : minX;
        maxX = maxX == null || cellX > maxX! ? cellX : maxX;
        minY = minY == null || cellY < minY! ? cellY : minY;
        maxY = maxY == null || cellY > maxY! ? cellY : maxY;
      }
    }
  }
}

/// Da metri sulla mappa a celle fini, arrotondando come fa il robot quando salva le zone.
int metersToCell(double meters) => (meters / fineMapCellSize + 0.5).floor() + fineMapOrigin;

/// Da celle fini a metri sulla mappa (il bordo con la coordinata più bassa della cella).
double cellToMeters(int cell) => (cell - fineMapOrigin) * fineMapCellSize;
