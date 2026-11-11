/// Colori dell'interfaccia (docs/app.md, sezione 10.5): palette "Petrolio principale, grafite accento", tema chiaro e
/// scuro che seguono il telefono. I colori della mappa stanno in map_colors.dart.
library;

import 'package:flutter/material.dart';

class AppPalette {
  const AppPalette({
    required this.brightness,
    required this.primary,
    required this.onPrimary,
    required this.accent,
    required this.background,
    required this.panel,
    required this.panelBorder,
    required this.control,
    required this.text,
    required this.textSecondary,
    required this.online,
    required this.warning,
    required this.fault,
  });

  final Brightness brightness;
  final Color primary; // petrolio: pulsante principale, cose accese
  final Color onPrimary; // testo sopra il petrolio
  final Color accent; // grafite: robot sulla mappa, manopola del joystick
  final Color background;
  final Color panel; // pannelli traslucidi sopra la mappa
  final Color panelBorder;
  final Color control; // pulsanti secondari e campi
  final Color text;
  final Color textSecondary;
  final Color online; // verde
  final Color warning; // ambra
  final Color fault; // rosso

  /// Tinta leggera del petrolio: Casa e pulsanti secondari.
  Color get primaryTint => primary.withValues(alpha: brightness == Brightness.dark ? 0.28 : 0.16);

  static const AppPalette light = AppPalette(
    brightness: Brightness.light,
    primary: Color(0xFF0E7C78),
    onPrimary: Color(0xFFFFFFFF),
    accent: Color(0xFF1D232B),
    background: Color(0xFFEEF2F5),
    panel: Color(0xD9FFFFFF),
    panelBorder: Color(0x1F1D232B),
    control: Color(0xFFE2E8ED),
    text: Color(0xFF151B21),
    textSecondary: Color(0xFF5A6772),
    online: Color(0xFF1F9D55),
    warning: Color(0xFFB7791F),
    fault: Color(0xFFD64545),
  );

  static const AppPalette dark = AppPalette(
    brightness: Brightness.dark,
    primary: Color(0xFF3CC2B9),
    onPrimary: Color(0xFF06211F),
    accent: Color(0xFFE6EAF0),
    background: Color(0xFF12181D),
    panel: Color(0xCC161D23),
    panelBorder: Color(0x24E6EAF0),
    control: Color(0xFF242C33),
    text: Color(0xFFF4F7FA),
    textSecondary: Color(0xFF9AA7B2),
    online: Color(0xFF4ADE80),
    warning: Color(0xFFF2B84B),
    fault: Color(0xFFFF7B7B),
  );

  static AppPalette of(BuildContext context) =>
      Theme.of(context).brightness == Brightness.dark ? dark : light;
}

// --- misure comuni dello stile Vetro ---

/// Raggio degli angoli dei pannelli.
const double panelRadius = 22;

/// Raggio degli angoli di pulsanti e riquadri dentro i pannelli.
const double controlRadius = 16;

/// Sfocatura dietro i pannelli traslucidi.
const double panelBlur = 18;

/// Margine fra i pannelli e il bordo dello schermo.
const double screenPadding = 14;

ThemeData buildTheme(AppPalette palette) {
  final base = ThemeData(
    brightness: palette.brightness,
    useMaterial3: true,
    fontFamily: 'Manrope',
    scaffoldBackgroundColor: palette.background,
    colorScheme: ColorScheme.fromSeed(
      seedColor: palette.primary,
      brightness: palette.brightness,
      primary: palette.primary,
      onPrimary: palette.onPrimary,
      surface: palette.background,
      onSurface: palette.text,
      error: palette.fault,
    ),
  );
  return base.copyWith(
    textTheme: base.textTheme.apply(bodyColor: palette.text, displayColor: palette.text),
    appBarTheme: AppBarTheme(
      backgroundColor: Colors.transparent,
      foregroundColor: palette.text,
      elevation: 0,
      scrolledUnderElevation: 0,
      titleTextStyle: TextStyle(
        fontFamily: 'Manrope',
        fontSize: 22,
        fontWeight: FontWeight.w700,
        color: palette.text,
      ),
    ),
    switchTheme: SwitchThemeData(
      thumbColor: WidgetStateProperty.resolveWith(
        (states) => states.contains(WidgetState.selected) ? palette.onPrimary : palette.textSecondary,
      ),
      trackColor: WidgetStateProperty.resolveWith(
        (states) => states.contains(WidgetState.selected) ? palette.primary : palette.control,
      ),
      trackOutlineColor: const WidgetStatePropertyAll(Colors.transparent),
    ),
    dialogTheme: DialogThemeData(
      backgroundColor: palette.brightness == Brightness.dark ? const Color(0xFF1B2329) : Colors.white,
      shape: RoundedRectangleBorder(borderRadius: BorderRadius.circular(panelRadius)),
    ),
    inputDecorationTheme: InputDecorationTheme(
      filled: true,
      fillColor: palette.control,
      border: OutlineInputBorder(
        borderRadius: BorderRadius.circular(controlRadius),
        borderSide: BorderSide.none,
      ),
    ),
  );
}
