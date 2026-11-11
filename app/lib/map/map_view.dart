/// La mappa sullo schermo: zoom e spostamento con le dita (InteractiveViewer), disegno con CustomPainter, e la
/// modifica delle zone quando serve. Nessuna libreria per le mappe (docs/app.md, sezione 10.1).
library;

import 'dart:ui' as ui;

import 'package:flutter/material.dart';

import '../config/map_colors.dart';
import '../models/robot_state.dart';
import 'map_painter.dart';
import 'rtm.dart';
import 'zone_editor.dart';

/// Zoom minimo e massimo della mappa.
const double mapMinScale = 0.2;
const double mapMaxScale = 24;

/// Zoom quando si ricentra sul robot: circa 6 metri di mappa nella larghezza dello schermo.
const double recenterVisibleMeters = 6;

/// Margine, in punti, attorno alla mappa quando la si inquadra tutta.
const double fitPadding = 28;

/// Lato minimo, in punti della tela (4 m), della zona inquadrata: una mappa appena nata non si ingrandisce a dismisura.
const double fitMinimumSide = 4 / fineMapCellSize * pixelsPerCell;

/// Permette alla schermata di ricentrare la mappa sul robot.
class MapViewController {
  _MapViewState? _state;

  void recenterOnRobot() => _state?._recenterOnRobot();
  void fitMap() => _state?._fit(force: true);
}

class MapView extends StatefulWidget {
  const MapView({
    super.key,
    required this.map,
    required this.mapRevision,
    this.trail = const [],
    this.trailRevision = 0,
    this.zones = const [],
    this.showBase = true,
    this.robot,
    this.controller,
    this.editor,
    this.onZonesChanged,
    this.topInset = 0,
    this.bottomInset = 0,
  });

  final LiveMap? map;
  final int mapRevision; // cambia quando la mappa viene aggiornata
  final List<Offset> trail;
  final int trailRevision;
  final List<Zone> zones; // ignorate quando c'è l'editor: si disegnano le sue
  final bool showBase;
  final RobotPose? robot;
  final MapViewController? controller;

  /// Se c'è, le zone si modificano con le dita (e la mappa si sposta solo con due dita).
  final ZoneEditor? editor;
  final VoidCallback? onZonesChanged;

  /// Spazio coperto dai pannelli in alto e in basso: la mappa si inquadra nella parte libera.
  final double topInset, bottomInset;

  @override
  State<MapView> createState() => _MapViewState();
}

class _MapViewState extends State<MapView> {
  final TransformationController _transformation = TransformationController();
  ui.Image? _image;
  Rect? _imageRect;
  int _imageRevision = -1;
  MapColors? _imageColors;
  Size _viewport = Size.zero;
  bool _userMoved = false; // l'utente ha spostato o ingrandito la mappa: da qui non la si reinquadra più da soli
  int _pointers = 0;
  int _editRevision = 0;

  @override
  void initState() {
    super.initState();
    widget.controller?._state = this;
    _transformation.addListener(() => setState(() {}));
  }

  @override
  void didUpdateWidget(MapView oldWidget) {
    super.didUpdateWidget(oldWidget);
    widget.controller?._state = this;
  }

  @override
  void dispose() {
    widget.controller?._state = null;
    _transformation.dispose();
    super.dispose();
  }

  double get _scale => _transformation.value.getMaxScaleOnAxis();

  /// Ricostruisce l'immagine delle celle quando la mappa o i colori cambiano.
  Future<void> _refreshImage(MapColors colors) async {
    final map = widget.map;
    if (map == null || (_imageRevision == widget.mapRevision && _imageColors == colors)) return;
    final revision = widget.mapRevision;
    _imageRevision = revision;
    _imageColors = colors;
    final built = await buildMapImage(map, colors);
    if (!mounted || revision != _imageRevision) return;
    setState(() {
      _image = built?.$1;
      _imageRect = built?.$2;
    });
    _fit();
  }

  void _show(Rect canvasRect) {
    final free = Size(_viewport.width, _viewport.height - widget.topInset - widget.bottomInset);
    if (free.width <= 0 || free.height <= 0) return;
    final horizontal = (free.width - 2 * fitPadding) / canvasRect.width;
    final vertical = (free.height - 2 * fitPadding) / canvasRect.height;
    final scale = (horizontal < vertical ? horizontal : vertical).clamp(mapMinScale, mapMaxScale).toDouble();
    final center = canvasRect.center;
    _transformation.value = Matrix4.identity()
      ..translateByDouble(free.width / 2 - center.dx * scale, widget.topInset + free.height / 2 - center.dy * scale, 0, 1)
      ..scaleByDouble(scale, scale, 1, 1);
  }

  /// Inquadra tutta la mappa: da sola finché l'utente non la tocca (la mappa cresce mentre il robot esplora), poi solo
  /// se chiesto.
  void _fit({bool force = false}) {
    if ((_userMoved && !force) || _viewport == Size.zero) return;
    if (force) _userMoved = false;
    final rect = _imageRect;
    // mappa vuota o ancora piccola: si guarda almeno la zona attorno alla base
    final minimum = Rect.fromCenter(center: worldToCanvas(0, 0), width: fitMinimumSide, height: fitMinimumSide);
    _show(rect == null ? minimum : rect.expandToInclude(Rect.fromCenter(center: rect.center, width: fitMinimumSide, height: fitMinimumSide)));
  }

  void _recenterOnRobot() {
    final robot = widget.robot;
    if (robot == null) return _fit(force: true);
    _userMoved = true;
    final side = recenterVisibleMeters / fineMapCellSize * pixelsPerCell;
    _show(Rect.fromCenter(center: worldToCanvas(robot.x, robot.y), width: side, height: side));
  }

  // --- modifica delle zone: eventi grezzi delle dita, così non litigano con zoom e spostamento ---

  void _pointerDown(PointerDownEvent event) {
    _pointers++;
    final editor = widget.editor;
    if (editor == null) return;
    if (_pointers > 1) {
      editor.endDrag(); // due dita: è uno zoom
      return;
    }
    final point = canvasToWorld(event.localPosition);
    final metersPerPoint = fineMapCellSize / pixelsPerCell / _scale;
    final deleted = editor.deleteHit(point, zoneDeleteRadius * 1.3 * metersPerPoint);
    if (deleted >= 0) {
      editor.removeAt(deleted);
      _zonesChanged();
      return;
    }
    editor.startDrag(point, zoneHandleRadius * 2.4 * metersPerPoint);
  }

  void _pointerMove(PointerMoveEvent event) {
    final editor = widget.editor;
    if (editor == null || _pointers != 1 || !editor.isDragging) return;
    editor.updateDrag(canvasToWorld(event.localPosition));
    _zonesChanged();
  }

  void _pointerUp(PointerEvent event) {
    _pointers = _pointers > 0 ? _pointers - 1 : 0;
    widget.editor?.endDrag();
  }

  void _zonesChanged() {
    setState(() => _editRevision++);
    widget.onZonesChanged?.call();
  }

  @override
  Widget build(BuildContext context) {
    final colors = MapColors.of(context);
    _refreshImage(colors);
    final editor = widget.editor;
    return LayoutBuilder(builder: (context, constraints) {
      final viewport = constraints.biggest;
      if (viewport != _viewport) {
        _viewport = viewport;
        WidgetsBinding.instance.addPostFrameCallback((_) => _fit());
      }
      return ColoredBox(
        color: colors.unknown,
        child: InteractiveViewer(
          transformationController: _transformation,
          constrained: false,
          minScale: mapMinScale,
          maxScale: mapMaxScale,
          boundaryMargin: const EdgeInsets.all(double.infinity),
          onInteractionStart: (_) => _userMoved = true,
          // con le zone in modifica un dito disegna: la mappa si sposta e si ingrandisce con due dita
          panEnabled: editor == null,
          child: Listener(
            onPointerDown: _pointerDown,
            onPointerMove: _pointerMove,
            onPointerUp: _pointerUp,
            onPointerCancel: _pointerUp,
            child: CustomPaint(
              size: const Size(mapCanvasSide, mapCanvasSide),
              painter: MapPainter(
                colors: colors,
                image: _image,
                imageRect: _imageRect,
                trail: widget.trail,
                zones: editor?.zones ?? widget.zones,
                showBase: widget.showBase,
                robot: widget.robot,
                editing: editor != null,
                scale: _scale,
                revision: Object.hash(widget.mapRevision, widget.trailRevision, _editRevision, widget.robot?.x,
                    widget.robot?.y, widget.robot?.thetaDeg, Object.hashAll(widget.zones), widget.showBase),
              ),
            ),
          ),
        ),
      );
    });
  }
}
