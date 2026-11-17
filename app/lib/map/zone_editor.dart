/// Modifica delle zone proibite e dei muri virtuali (docs/app.md, sezione 10.4). Zone: rettangoli disegnati trascinando,
/// spostati, ridimensionati dagli angoli e cancellati con la x in alto a destra. Muri: segmenti disegnati trascinando,
/// spostati, allungati o girati dagli estremi e cancellati con la x a metà. Un dito su un punto vuoto disegna quello che
/// dice [ZoneEditor.target]. Qui c'è solo la logica, in metri sulla mappa; il disegno e le dita stanno in map_view.dart.
library;

import 'dart:math' as math;
import 'dart:ui';

import '../config/map_colors.dart';
import '../models/robot_state.dart';
import 'rtm.dart';

/// Cosa disegna un dito appoggiato su un punto vuoto.
enum EditTarget { zones, walls }

/// Cosa sta facendo il dito.
enum ZoneDragKind { create, move, resize, createWall, moveWall, moveWallEnd }

class ZoneEditor {
  ZoneEditor(List<Zone> zones, [List<VirtualWall> walls = const []])
      : zones = List.of(zones),
        walls = List.of(walls);

  /// Le liste intere che Conferma manderà al server.
  final List<Zone> zones;
  final List<VirtualWall> walls;

  /// Cosa si disegna su un punto vuoto: zone o muri (l'interruttore della schermata).
  EditTarget target = EditTarget.zones;

  ZoneDragKind? _kind;
  int _index = -1;
  bool _isFirstEnd = true; // l'estremo del muro che si sta trascinando
  Offset _start = Offset.zero; // dove è partito il dito
  Offset _anchor = Offset.zero; // angolo opposto a quello trascinato (ridimensionamento e creazione)
  Zone? _original; // la zona com'era all'inizio dello spostamento
  VirtualWall? _originalWall; // il muro com'era all'inizio dello spostamento

  bool get canAdd => target == EditTarget.zones ? zones.length < maxUserZones : walls.length < maxVirtualWalls;

  /// Il punto in alto a destra di una zona: lì sta la x che la cancella.
  static Offset deletePoint(Zone zone) => Offset(zone.x + zone.w, zone.y + zone.h);

  /// Il punto a metà di un muro: lì sta la x che lo cancella.
  static Offset wallDeletePoint(VirtualWall wall) => Offset((wall.x1 + wall.x2) / 2, (wall.y1 + wall.y2) / 2);

  static List<Offset> corners(Zone zone) => [
        Offset(zone.x, zone.y),
        Offset(zone.x + zone.w, zone.y),
        Offset(zone.x, zone.y + zone.h),
        Offset(zone.x + zone.w, zone.y + zone.h),
      ];

  /// Distanza fra un punto e un muro, in metri.
  static double wallDistance(VirtualWall wall, Offset point) {
    final a = Offset(wall.x1, wall.y1), b = Offset(wall.x2, wall.y2);
    final segment = b - a;
    final length2 = segment.dx * segment.dx + segment.dy * segment.dy;
    final t = length2 == 0 ? 0.0 : (((point - a).dx * segment.dx + (point - a).dy * segment.dy) / length2).clamp(0.0, 1.0);
    return (point - (a + segment * t)).distance;
  }

  /// Cancella la zona o il muro la cui x è entro radius dal punto. Ritorna true se ha cancellato qualcosa.
  bool deleteAt(Offset point, double radius) {
    for (var i = walls.length - 1; i >= 0; i--) {
      if ((wallDeletePoint(walls[i]) - point).distance <= radius) {
        walls.removeAt(i);
        return true;
      }
    }
    for (var i = zones.length - 1; i >= 0; i--) {
      if ((deletePoint(zones[i]) - point).distance <= radius) {
        zones.removeAt(i);
        return true;
      }
    }
    return false;
  }

  /// Il dito si appoggia: sull'estremo di un muro lo sposta, sull'angolo di una zona la ridimensiona, sopra un muro o
  /// dentro una zona li sposta, altrove disegna una zona o un muro nuovo (secondo [target]).
  void startDrag(Offset point, double handleRadius) {
    _start = point;
    for (var i = walls.length - 1; i >= 0; i--) {
      final wall = walls[i];
      for (final isFirst in [true, false]) {
        final end = isFirst ? Offset(wall.x1, wall.y1) : Offset(wall.x2, wall.y2);
        if ((end - point).distance <= handleRadius) {
          _kind = ZoneDragKind.moveWallEnd;
          _index = i;
          _isFirstEnd = isFirst;
          return;
        }
      }
    }
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
    for (var i = walls.length - 1; i >= 0; i--) {
      if (wallDistance(walls[i], point) <= handleRadius) {
        _kind = ZoneDragKind.moveWall;
        _index = i;
        _originalWall = walls[i];
        return;
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
    _kind = target == EditTarget.zones ? ZoneDragKind.create : ZoneDragKind.createWall;
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
      case ZoneDragKind.createWall:
        // il muro nasce quando è lungo almeno una cella
        if ((point - _anchor).distance < wallMinLength) return;
        final wall = _clampWall(VirtualWall(_anchor.dx, _anchor.dy, point.dx, point.dy));
        if (_index < 0) {
          walls.add(wall);
          _index = walls.length - 1;
        } else {
          walls[_index] = wall;
        }
      case ZoneDragKind.moveWall:
        final original = _originalWall!;
        final dx = point.dx - _start.dx, dy = point.dy - _start.dy;
        walls[_index] = _clampWall(VirtualWall(original.x1 + dx, original.y1 + dy, original.x2 + dx, original.y2 + dy));
      case ZoneDragKind.moveWallEnd:
        final wall = walls[_index];
        final moved = _isFirstEnd ? VirtualWall(point.dx, point.dy, wall.x2, wall.y2) : VirtualWall(wall.x1, wall.y1, point.dx, point.dy);
        // un estremo non può finire sopra l'altro: il muro resta lungo almeno una cella
        if (math.sqrt(math.pow(moved.x2 - moved.x1, 2) + math.pow(moved.y2 - moved.y1, 2)) >= wallMinLength) {
          walls[_index] = _clampWall(moved);
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

  static double _round(double value) => (value * 100).roundToDouble() / 100;

  /// Tiene la zona dentro la griglia della mappa e arrotonda al centimetro.
  static Zone _clamp(Zone zone) {
    const half = fineMapOrigin * fineMapCellSize;
    final w = math.min(zone.w, 2 * half), h = math.min(zone.h, 2 * half);
    final x = zone.x.clamp(-half, half - w), y = zone.y.clamp(-half, half - h);
    return Zone(_round(x), _round(y), _round(w), _round(h));
  }

  /// Tiene gli estremi del muro dentro la griglia della mappa e arrotonda al centimetro.
  static VirtualWall _clampWall(VirtualWall wall) {
    const half = fineMapOrigin * fineMapCellSize;
    double clamp(double value) => _round(value.clamp(-half, half));
    return VirtualWall(clamp(wall.x1), clamp(wall.y1), clamp(wall.x2), clamp(wall.y2));
  }
}
