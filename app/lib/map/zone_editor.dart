/// Modifica delle zone proibite (docs/app.md, sezione 10.4): rettangoli disegnati trascinando, spostati, ridimensionati
/// dagli angoli e cancellati con la x in alto a destra. Qui c'è solo la logica, in metri sulla mappa; il disegno e le
/// dita stanno in map_view.dart.
library;

import 'dart:math' as math;
import 'dart:ui';

import '../config/map_colors.dart';
import '../models/robot_state.dart';
import 'rtm.dart';

/// Cosa sta facendo il dito.
enum ZoneDragKind { create, move, resize }

class ZoneEditor {
  ZoneEditor(List<Zone> zones) : zones = List.of(zones);

  /// La lista intera che Conferma manderà al server.
  final List<Zone> zones;

  ZoneDragKind? _kind;
  int _index = -1;
  Offset _start = Offset.zero; // dove è partito il dito
  Offset _anchor = Offset.zero; // angolo opposto a quello trascinato (ridimensionamento e creazione)
  Zone? _original; // la zona com'era all'inizio dello spostamento

  bool get canAdd => zones.length < maxUserZones;

  /// Il punto in alto a destra di una zona: lì sta la x che la cancella.
  static Offset deletePoint(Zone zone) => Offset(zone.x + zone.w, zone.y + zone.h);

  static List<Offset> corners(Zone zone) => [
        Offset(zone.x, zone.y),
        Offset(zone.x + zone.w, zone.y),
        Offset(zone.x, zone.y + zone.h),
        Offset(zone.x + zone.w, zone.y + zone.h),
      ];

  /// Indice della zona la cui x è entro radius dal punto, oppure -1.
  int deleteHit(Offset point, double radius) {
    for (var i = zones.length - 1; i >= 0; i--) {
      if ((deletePoint(zones[i]) - point).distance <= radius) return i;
    }
    return -1;
  }

  void removeAt(int index) => zones.removeAt(index);

  /// Il dito si appoggia: su un angolo ridimensiona, dentro una zona la sposta, altrove ne disegna una nuova.
  void startDrag(Offset point, double handleRadius) {
    _start = point;
    for (var i = zones.length - 1; i >= 0; i--) {
      final zoneCorners = corners(zones[i]);
      for (var corner = 0; corner < 4; corner++) {
        if ((zoneCorners[corner] - point).distance <= handleRadius) {
          _kind = ZoneDragKind.resize;
          _index = i;
          _anchor = zoneCorners[3 - corner]; // l'angolo opposto resta fermo
          return;
        }
      }
    }
    for (var i = zones.length - 1; i >= 0; i--) {
      final zone = zones[i];
      if (point.dx >= zone.x && point.dx <= zone.x + zone.w && point.dy >= zone.y && point.dy <= zone.y + zone.h) {
        _kind = ZoneDragKind.move;
        _index = i;
        _original = zone;
        return;
      }
    }
    if (!canAdd) {
      _kind = null;
      return;
    }
    _kind = ZoneDragKind.create;
    _anchor = point;
    _index = -1;
  }

  /// Il dito si muove.
  void updateDrag(Offset point) {
    switch (_kind) {
      case ZoneDragKind.move:
        final original = _original!;
        zones[_index] = _clamp(Zone(original.x + point.dx - _start.dx, original.y + point.dy - _start.dy, original.w, original.h));
      case ZoneDragKind.resize:
        zones[_index] = _fromCorners(_anchor, point);
      case ZoneDragKind.create:
        // la zona nasce quando il dito si è spostato abbastanza da farne una valida
        if ((point.dx - _anchor.dx).abs() < zoneMinSide || (point.dy - _anchor.dy).abs() < zoneMinSide) return;
        if (_index < 0) {
          zones.add(_fromCorners(_anchor, point));
          _index = zones.length - 1;
        } else {
          zones[_index] = _fromCorners(_anchor, point);
        }
      case null:
        break;
    }
  }

  void endDrag() {
    _kind = null;
    _index = -1;
  }

  bool get isDragging => _kind != null;

  static Zone _fromCorners(Offset a, Offset b) {
    final x = math.min(a.dx, b.dx), y = math.min(a.dy, b.dy);
    return _clamp(Zone(x, y, math.max((a.dx - b.dx).abs(), zoneMinSide), math.max((a.dy - b.dy).abs(), zoneMinSide)));
  }

  /// Tiene la zona dentro la griglia della mappa e arrotonda al centimetro.
  static Zone _clamp(Zone zone) {
    const half = fineMapOrigin * fineMapCellSize;
    final w = math.min(zone.w, 2 * half), h = math.min(zone.h, 2 * half);
    final x = zone.x.clamp(-half, half - w), y = zone.y.clamp(-half, half - h);
    double round(double value) => (value * 100).roundToDouble() / 100;
    return Zone(round(x), round(y), round(w), round(h));
  }
}
