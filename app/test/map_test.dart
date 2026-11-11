// Prove della decodifica RTM1 e della mappa dal vivo con le differenze. I dati di prova sono stati generati dal
// backend (backend/maps.py): così l'app e il server sono confrontati sullo stesso formato.
import 'dart:typed_data';

import 'package:flutter_test/flutter_test.dart';
import 'package:maurice/map/rtm.dart';
import 'package:maurice/map/trail.dart';

// una stanza 20 x 10 celle dalla cella (500, 505): bordo occupato, dentro libera, una zona (505, 508, 5, 9)
const String roomRtmHex = '52544d31f401f90114000a0001f901fc01050009009552825282528252825282528252825295268896d4';
// messaggio "mappa intera" con la versione 1794313279287 e la stanza
const String fullMessageHex =
    '0037d767c5a10100002a00000052544d31f401f90114000a0001f901fc01050009009552825282528252825282528252825295268896d4';
// messaggio "differenze": versione ...288 con il rettangolo 4 x 2 dalla cella (518, 507) (vuoto, vuoto, occupata,
// occupata / libera, libera, sconosciuta, sconosciuta), poi versione ...289 con una cella sconosciuta nell'origine
const String patchMessageHex =
    '0138d767c5a10100001500000052544d310602fb010400020000c28242021a9ab99a39d767c5a10100001200000052544d31000200020100010000012bb44b24';

Uint8List fromHex(String hex) =>
    Uint8List.fromList([for (var i = 0; i < hex.length; i += 2) int.parse(hex.substring(i, i + 2), radix: 16)]);

void main() {
  group('RTM1', () {
    test('decodifica una mappa del backend', () {
      final map = decodeRtm(fromHex(roomRtmHex));
      expect([map.x, map.y, map.width, map.height], [500, 505, 20, 10]);
      expect(map.zones, [
        [505, 508, 5, 9]
      ]);
      expect(map.cells.length, 200);
      expect(map.cells.sublist(0, 20), List.filled(20, mapCatOccupied)); // la prima riga è il muro
      expect(map.cells.sublist(20, 40), [mapCatOccupied, ...List.filled(18, mapCatFree), mapCatOccupied]);
      expect(map.cells.sublist(180, 200), List.filled(20, mapCatOccupied));
    });

    test('CRC-32 standard', () {
      expect(crc32(Uint8List.fromList('123456789'.codeUnits)), 0xCBF43926);
      final data = fromHex(roomRtmHex);
      expect(crc32(data, 0, data.length - 4), 0xd4968826);
    });

    test('rifiuta i file rovinati', () {
      final good = fromHex(roomRtmHex);
      Uint8List changed(void Function(Uint8List data) change) {
        final data = Uint8List.fromList(good);
        change(data);
        return data;
      }

      expect(() => decodeRtm(Uint8List.sublistView(good, 0, 10)), throwsA(isA<MapFormatException>())); // troppo corto
      expect(() => decodeRtm(changed((data) => data[3] = 0x32)), throwsA(isA<MapFormatException>())); // numero magico
      expect(() => decodeRtm(changed((data) => data[20] ^= 1)), throwsA(isA<MapFormatException>())); // CRC
      expect(() => decodeRtm(changed((data) => data[data.length - 1] ^= 1)), throwsA(isA<MapFormatException>()));
      expect(() => decodeRtm(Uint8List.sublistView(good, 0, good.length - 1)), throwsA(isA<MapFormatException>()));
      expect(() => decodeRtm(Uint8List(0)), throwsA(isA<MapFormatException>()));
    });

    test('metri e celle', () {
      expect(metersToCell(-0.365), 505); // la zona della base di docs/app.md
      expect(metersToCell(-0.22), 508);
      expect(cellToMeters(532), closeTo(1.0, 1e-9));
      expect(cellToMeters(fineMapOrigin), 0);
    });
  });

  group('mappa dal vivo', () {
    test('la mappa intera sostituisce tutto', () {
      final map = LiveMap();
      expect(map.isEmpty, true);
      expect(map.version, null);
      map.grid[5] = mapCatOccupied; // una cella vecchia che deve sparire
      map.applyMessage(fromHex(fullMessageHex));
      expect(map.version, 1794313279287);
      expect(map.grid[5], mapCatUnknown);
      expect([map.minX, map.minY, map.maxX, map.maxY], [500, 505, 519, 514]);
      expect(map.grid[505 * fineMapCellNumber + 500], mapCatOccupied);
      expect(map.grid[506 * fineMapCellNumber + 501], mapCatFree);
      expect(map.grid[504 * fineMapCellNumber + 500], mapCatUnknown);
    });

    test('le differenze si incollano in ordine e portano la versione', () {
      final map = LiveMap()..applyMessage(fromHex(fullMessageHex));
      map.applyMessage(fromHex(patchMessageHex));
      expect(map.version, 1794313279289);
      // il rettangolo 4 x 2 dalla cella (518, 507): riga 507 vuoto, vuoto, occupata, occupata
      expect([for (var x = 518; x < 522; x++) map.grid[507 * fineMapCellNumber + x]],
          [mapCatCliff, mapCatCliff, mapCatOccupied, mapCatOccupied]);
      // riga 508: libera, libera, sconosciuta, sconosciuta (una differenza può anche riportare una cella a sconosciuta)
      expect([for (var x = 518; x < 522; x++) map.grid[508 * fineMapCellNumber + x]],
          [mapCatFree, mapCatFree, mapCatUnknown, mapCatUnknown]);
      // le celle fuori dai rettangoli restano com'erano, e il rettangolo delle celle viste si allarga
      expect(map.grid[506 * fineMapCellNumber + 501], mapCatFree);
      expect(map.maxX, 521);
      expect(map.minX, 500);
    });

    test('differenze senza parti: non cambia niente', () {
      final map = LiveMap()..applyMessage(fromHex(fullMessageHex));
      map.applyMessage(Uint8List.fromList([liveMessagePatches]));
      expect(map.version, 1794313279287);
    });

    test('messaggi rovinati', () {
      final map = LiveMap();
      final full = fromHex(fullMessageHex);
      expect(() => map.applyMessage(Uint8List(0)), throwsA(isA<MapFormatException>()));
      expect(() => map.applyMessage(Uint8List.fromList([7])), throwsA(isA<MapFormatException>()));
      expect(() => map.applyMessage(Uint8List.sublistView(full, 0, full.length - 3)), throwsA(isA<MapFormatException>()));
      expect(() => map.applyMessage(Uint8List.sublistView(full, 0, 6)), throwsA(isA<MapFormatException>()));
    });
  });

  group('scia', () {
    Map<String, dynamic> message(int id, int offset, List<List<double>> points) =>
        {'trail_id': id, 'offset': offset, 'points': points};

    test('tutta all\'apertura, poi solo i punti nuovi', () {
      final trail = Trail();
      trail.applyMessage(message(7, 0, [
        [0, 0],
        [0.1, 0]
      ]));
      expect(trail.points.length, 2);
      trail.applyMessage(message(7, 2, [
        [0.2, 0.5]
      ]));
      expect(trail.points.length, 3);
      expect(trail.points.last.dy, 0.5);
      trail.applyMessage(message(7, 3, []));
      expect(trail.points.length, 3);
    });

    test('missione nuova: la scia vecchia si butta', () {
      final trail = Trail()
        ..applyMessage(message(7, 0, [
          [0, 0],
          [0.1, 0]
        ]));
      trail.applyMessage(message(8, 0, [
        [1, 1]
      ]));
      expect(trail.trailId, 8);
      expect(trail.points.length, 1);
      expect(trail.points.first.dx, 1);
    });

    test('scia cambiata ma risposta a metà: alla prossima si richiede tutto', () {
      final trail = Trail()
        ..applyMessage(message(7, 0, [
          [0, 0],
          [0.1, 0]
        ]));
      trail.applyMessage(message(8, 2, [
        [9, 9]
      ]));
      expect(trail.points, isEmpty);
      expect(trail.trailId, null);
    });
  });
}
