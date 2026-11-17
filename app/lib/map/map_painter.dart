/// Disegno della mappa (docs/app.md, sezione 10.4): celle, scia, zone proibite, muri virtuali, base e robot.
library;

import 'dart:math' as math;
import 'dart:typed_data';
import 'dart:ui' as ui;

import 'package:flutter/material.dart';

import '../config/map_colors.dart';
import '../models/robot_state.dart';
import 'rtm.dart';
import 'trail.dart';
import 'zone_editor.dart';

/// Punti dello schermo per cella fine con lo zoom a 1.
const double pixelsPerCell = 2;

/// Lato, in punti, di tutta la griglia con lo zoom a 1.
const double mapCanvasSide = fineMapCellNumber * pixelsPerCell;

/// Raggio, in punti dello schermo, della x che cancella una zona e delle maniglie agli angoli.
const double zoneDeleteRadius = 13;
const double zoneHandleRadius = 7;

/// Lunghezza, in punti dello schermo, dei tratti del bordo tratteggiato delle zone.
const double zoneDashLength = 7;

/// Da metri sulla mappa a punti della tela: x verso destra, y verso l'alto.
Offset worldToCanvas(double x, double y) => Offset(
      (x / fineMapCellSize + fineMapOrigin) * pixelsPerCell,
      (fineMapOrigin - y / fineMapCellSize) * pixelsPerCell,
    );

/// L'operazione inversa di worldToCanvas.
Offset canvasToWorld(Offset point) => Offset(
      (point.dx / pixelsPerCell - fineMapOrigin) * fineMapCellSize,
      (fineMapOrigin - point.dy / pixelsPerCell) * fineMapCellSize,
    );

/// Dove sta e dove guarda il robot.
class RobotPose {
  const RobotPose(this.x, this.y, this.thetaDeg);
  final double x, y, thetaDeg;
}

class MapPainter extends CustomPainter {
  MapPainter({
    required this.colors,
    required this.image,
    required this.imageRect,
    required this.trail,
    required this.zones,
    this.walls = const [],
    this.target,
    required this.showBase,
    required this.robot,
    required this.editing,
    required this.scale,
    required this.revision,
  });

  final MapColors colors;
  final ui.Image? image; // le celle viste, una per pixel
  final Rect? imageRect; // dove va l'immagine sulla tela
  final List<Offset> trail; // in metri
  final List<Zone> zones; // zone dell'utente
  final List<VirtualWall> walls; // muri virtuali
  final Offset? target; // obiettivo dato dall'app, in metri
  final bool showBase;
  final RobotPose? robot;
  final bool editing; // si stanno modificando le zone: maniglie e x
  final double scale; // zoom corrente: le cose a misura di dito restano grandi uguali
  final int revision; // cambia quando c'è qualcosa da ridisegnare

  @override
  void paint(Canvas canvas, Size size) {
    if (image != null && imageRect != null) {
      canvas.drawImageRect(
        image!,
        Rect.fromLTWH(0, 0, image!.width.toDouble(), image!.height.toDouble()),
        imageRect!,
        Paint()..filterQuality = FilterQuality.none,
      );
    }
    _paintTrail(canvas);
    if (showBase) _paintBase(canvas);
    for (final zone in zones) {
      _paintZone(canvas, zone);
    }
    for (final wall in walls) {
      _paintWall(canvas, wall);
    }
    if (target != null) _paintTarget(canvas, target!);
    if (robot != null) _paintRobot(canvas, robot!);
  }

  double _meters(double meters) => meters / fineMapCellSize * pixelsPerCell;

  void _paintTrail(Canvas canvas) {
    if (trail.length < 2) return;
    // un tratto per ogni pezzo fra due interruzioni (dove il robot non aspirava)
    final path = Path();
    var isNewStretch = true;
    for (final point in trail) {
      if (isTrailBreak(point)) {
        isNewStretch = true;
        continue;
      }
      final canvasPoint = worldToCanvas(point.dx, point.dy);
      if (isNewStretch) {
        path.moveTo(canvasPoint.dx, canvasPoint.dy);
        isNewStretch = false;
      } else {
        path.lineTo(canvasPoint.dx, canvasPoint.dy);
      }
    }
    canvas.drawPath(
      path,
      Paint()
        ..color = colors.trail
        ..style = PaintingStyle.stroke
        ..strokeWidth = _meters(trailWidth)
        ..strokeCap = StrokeCap.round
        ..strokeJoin = StrokeJoin.round,
    );
  }

  void _paintBase(Canvas canvas) {
    final topLeft = worldToCanvas(baseFaceX - baseDepth, baseWidth / 2);
    final rect = Rect.fromLTWH(topLeft.dx, topLeft.dy, _meters(baseDepth), _meters(baseWidth));
    canvas.drawRRect(RRect.fromRectAndRadius(rect, Radius.circular(_meters(0.02))), Paint()..color = colors.base);
  }

  void _paintZone(Canvas canvas, Zone zone) {
    final topLeft = worldToCanvas(zone.x, zone.y + zone.h);
    final rect = Rect.fromLTWH(topLeft.dx, topLeft.dy, _meters(zone.w), _meters(zone.h));
    canvas.drawRect(rect, Paint()..color = colors.forbiddenZoneFill);

    final border = Paint()
      ..color = colors.forbiddenZoneBorder
      ..style = PaintingStyle.stroke
      ..strokeWidth = 2 / scale;
    final dash = zoneDashLength / scale;
    void dashedLine(Offset from, Offset to) {
      final length = (to - from).distance;
      final direction = (to - from) / length;
      for (var position = 0.0; position < length; position += 2 * dash) {
        canvas.drawLine(from + direction * position, from + direction * math.min(position + dash, length), border);
      }
    }

    dashedLine(rect.topLeft, rect.topRight);
    dashedLine(rect.topRight, rect.bottomRight);
    dashedLine(rect.bottomRight, rect.bottomLeft);
    dashedLine(rect.bottomLeft, rect.topLeft);

    if (!editing) return;

    // maniglie agli angoli e x in alto a destra
    final handle = Paint()..color = colors.forbiddenZoneBorder;
    for (final corner in ZoneEditor.corners(zone)) {
      canvas.drawCircle(worldToCanvas(corner.dx, corner.dy), zoneHandleRadius / scale, handle);
    }
    _paintDelete(canvas, rect.topRight, handle);
  }

  /// Un muro virtuale: una linea piena; in modifica le maniglie agli estremi e la x a metà.
  void _paintWall(Canvas canvas, VirtualWall wall) {
    final paint = Paint()
      ..color = colors.virtualWall
      ..strokeWidth = math.max(_meters(virtualWallWidth), 3 / scale)
      ..strokeCap = StrokeCap.round;
    canvas.drawLine(worldToCanvas(wall.x1, wall.y1), worldToCanvas(wall.x2, wall.y2), paint);
    if (!editing) return;
    final handle = Paint()..color = colors.virtualWall;
    canvas.drawCircle(worldToCanvas(wall.x1, wall.y1), zoneHandleRadius / scale, handle);
    canvas.drawCircle(worldToCanvas(wall.x2, wall.y2), zoneHandleRadius / scale, handle);
    final middle = ZoneEditor.wallDeletePoint(wall);
    _paintDelete(canvas, worldToCanvas(middle.dx, middle.dy), handle);
  }

  /// L'obiettivo dato dall'app: un anello con il punto al centro.
  void _paintTarget(Canvas canvas, Offset point) {
    final center = worldToCanvas(point.dx, point.dy);
    final paint = Paint()
      ..color = colors.target
      ..style = PaintingStyle.stroke
      ..strokeWidth = math.max(_meters(0.03), 2 / scale);
    canvas.drawCircle(center, _meters(targetRadius), paint);
    canvas.drawCircle(center, _meters(targetRadius) * 0.3, Paint()..color = colors.target);
  }

  /// La x che cancella una zona o un muro.
  void _paintDelete(Canvas canvas, Offset deleteCenter, Paint handle) {
    final radius = zoneDeleteRadius / scale;
    canvas.drawCircle(deleteCenter, radius, handle);
    final cross = Paint()
      ..color = Colors.white
      ..strokeWidth = 2.4 / scale
      ..strokeCap = StrokeCap.round;
    final arm = radius * 0.42;
    canvas.drawLine(deleteCenter + Offset(-arm, -arm), deleteCenter + Offset(arm, arm), cross);
    canvas.drawLine(deleteCenter + Offset(-arm, arm), deleteCenter + Offset(arm, -arm), cross);
  }

  void _paintRobot(Canvas canvas, RobotPose pose) {
    final center = worldToCanvas(pose.x, pose.y);
    final radius = _meters(robotDiameter / 2);
    canvas.drawCircle(center, radius, Paint()..color = colors.robot);
    // la tacca dal centro al bordo dice dove guarda (sullo schermo la y va in giù)
    final angle = -pose.thetaDeg * math.pi / 180;
    canvas.drawLine(
      center,
      center + Offset(math.cos(angle), math.sin(angle)) * radius,
      Paint()
        ..color = colors.robotMark
        ..strokeWidth = radius * 0.32
        ..strokeCap = StrokeCap.round,
    );
  }

  @override
  bool shouldRepaint(MapPainter oldDelegate) =>
      oldDelegate.revision != revision ||
      oldDelegate.image != image ||
      oldDelegate.scale != scale ||
      oldDelegate.colors != colors ||
      oldDelegate.editing != editing;
}

/// Costruisce l'immagine delle celle viste: un pixel per cella, le sconosciute trasparenti.
/// Ritorna anche il rettangolo sulla tela dove va disegnata.
Future<(ui.Image, Rect)?> buildMapImage(LiveMap map, MapColors colors) async {
  if (map.isEmpty) return null;
  final width = map.maxX! - map.minX! + 1, height = map.maxY! - map.minY! + 1;
  final pixels = Uint8List(width * height * 4);
  final palette = [null, colors.free, colors.occupied, colors.cliff];
  for (var row = 0; row < height; row++) {
    // la riga più in alto dell'immagine è quella con la y più grande
    final cellY = map.maxY! - row;
    for (var column = 0; column < width; column++) {
      final color = palette[map.grid[cellY * fineMapCellNumber + map.minX! + column] & 3];
      if (color == null) continue;
      final offset = (row * width + column) * 4;
      pixels[offset] = (color.r * 255).round();
      pixels[offset + 1] = (color.g * 255).round();
      pixels[offset + 2] = (color.b * 255).round();
      pixels[offset + 3] = 255;
    }
  }
  final buffer = await ui.ImmutableBuffer.fromUint8List(pixels);
  final descriptor = ui.ImageDescriptor.raw(buffer, width: width, height: height, pixelFormat: ui.PixelFormat.rgba8888);
  final codec = await descriptor.instantiateCodec();
  final frame = await codec.getNextFrame();
  final rect = Rect.fromLTWH(
    map.minX! * pixelsPerCell,
    (fineMapCellNumber - 1 - map.maxY!) * pixelsPerCell,
    width * pixelsPerCell,
    height * pixelsPerCell,
  );
  return (frame.image, rect);
}
