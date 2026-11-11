/// Colori della mappa, tutti in questo file (docs/app.md, sezione 10.5).
library;

import 'package:flutter/material.dart';

class MapColors {
  const MapColors({
    required this.unknown,
    required this.free,
    required this.occupied,
    required this.cliff,
    required this.forbiddenZoneFill,
    required this.forbiddenZoneBorder,
    required this.base,
    required this.trail,
    required this.robot,
    required this.robotMark,
  });

  final Color unknown; // cella mai vista: è lo sfondo
  final Color free; // cella libera
  final Color occupied; // muri e ostacoli
  final Color cliff; // vuoto degli anti-caduta (scale, gradini)
  final Color forbiddenZoneFill;
  final Color forbiddenZoneBorder;
  final Color base; // base di ricarica
  final Color trail; // scia dell'ultima missione
  final Color robot;
  final Color robotMark; // la tacca che dice dove guarda il robot

  static const MapColors light = MapColors(
    unknown: Color(0xFFEEF2F5),
    free: Color(0xFFFFFFFF),
    occupied: Color(0xFF6B7782),
    cliff: Color(0xFFE2703A),
    forbiddenZoneFill: Color(0x33D64545),
    forbiddenZoneBorder: Color(0xFFD64545),
    base: Color(0xFF9AA7B2),
    trail: Color(0x66808B95),
    robot: Color(0xFF1D232B),
    robotMark: Color(0xFFFFFFFF),
  );

  static const MapColors dark = MapColors(
    unknown: Color(0xFF12181D),
    free: Color(0xFF2B3338),
    occupied: Color(0xFFB9C2C9),
    cliff: Color(0xFFD9541E),
    forbiddenZoneFill: Color(0x40FF7B7B),
    forbiddenZoneBorder: Color(0xFFFF7B7B),
    base: Color(0xFFC9D1D8),
    trail: Color(0x80808B95),
    robot: Color(0xFFE6EAF0),
    robotMark: Color(0xFF1D232B),
  );

  static MapColors of(BuildContext context) =>
      Theme.of(context).brightness == Brightness.dark ? dark : light;
}

// --- misure del disegno, in metri sulla mappa ---

/// Diametro del robot.
const double robotDiameter = 0.349;

/// Larghezza della scia: quanto la spazzola centrale.
const double trailWidth = 0.16;

/// La base di ricarica: larga 30 cm e profonda 12, con la faccia a un raggio di robot dietro l'origine.
const double baseWidth = 0.30;
const double baseDepth = 0.12;
const double baseFaceX = -0.1745;

/// Lato minimo di una zona proibita disegnata: una cella della mappa.
const double zoneMinSide = 0.05;
