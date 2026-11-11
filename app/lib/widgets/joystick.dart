/// Il joystick del telecomando (docs/app.md, sezione 10.4): su e giù danno v avanti e indietro, destra e sinistra
/// danno omega, combinabili. Anello piccolo con quattro frecce discrete; la manopola esce dall'anello quando è spinta.
library;

import 'package:flutter/material.dart';

import '../config/app_colors.dart';
import '../config/app_config.dart';

/// Diametro dell'anello, in punti.
const double joystickRingDiameter = 140;

/// Diametro della manopola.
const double joystickKnobDiameter = 60;

/// Corsa della manopola dal centro: più del raggio libero dell'anello, così spinta in fondo ne esce.
const double joystickTravel = 64;

class Joystick extends StatefulWidget {
  const Joystick({super.key, required this.onChanged, this.enabled = true});

  /// Chiamata a ogni spostamento con v in m/s e omega in rad/s; (0, 0) quando il joystick torna al centro.
  final void Function(double v, double omega) onChanged;
  final bool enabled;

  @override
  State<Joystick> createState() => _JoystickState();
}

class _JoystickState extends State<Joystick> {
  Offset _knob = Offset.zero; // spostamento della manopola dal centro, in punti

  void _move(Offset localPosition) {
    if (!widget.enabled) return;
    const center = Offset(joystickTravel + joystickKnobDiameter / 2, joystickTravel + joystickKnobDiameter / 2);
    var offset = localPosition - center;
    if (offset.distance > joystickTravel) offset = offset / offset.distance * joystickTravel;
    setState(() => _knob = offset);

    // normalizza fra -1 e 1, con la zona morta al centro
    double axis(double value) {
      final fraction = value / joystickTravel;
      if (fraction.abs() < joystickDeadZone) return 0;
      return (fraction.abs() - joystickDeadZone) / (1 - joystickDeadZone) * fraction.sign;
    }

    final forward = -axis(offset.dy); // su = avanti
    final turn = -axis(offset.dx); // destra = rotazione oraria, cioè omega negativa
    // arrotondate al passo, e poi alle cifre del passo: niente 0.5700000000000001
    double step(double value) => double.parse(((value / joystickSpeedStep).roundToDouble() * joystickSpeedStep).toStringAsFixed(3)) + 0.0;
    widget.onChanged(step(forward * (forward >= 0 ? joystickMaxV : joystickMaxVBack)), step(turn * joystickMaxOmega));
  }

  void _release() {
    setState(() => _knob = Offset.zero);
    widget.onChanged(0, 0);
  }

  @override
  Widget build(BuildContext context) {
    final palette = AppPalette.of(context);
    const side = 2 * joystickTravel + joystickKnobDiameter;
    final arrow = palette.textSecondary;
    return Opacity(
      opacity: widget.enabled ? 1 : 0.45,
      child: GestureDetector(
        onPanStart: (details) => _move(details.localPosition),
        onPanUpdate: (details) => _move(details.localPosition),
        onPanEnd: (_) => _release(),
        onPanCancel: _release,
        child: SizedBox(
          width: side,
          height: side,
          child: Stack(alignment: Alignment.center, children: [
            Container(
              width: joystickRingDiameter,
              height: joystickRingDiameter,
              decoration: BoxDecoration(shape: BoxShape.circle, border: Border.all(color: palette.panelBorder, width: 1.5)),
            ),
            for (final (alignment, icon) in const [
              (Alignment(0, -0.62), Icons.keyboard_arrow_up_rounded),
              (Alignment(0, 0.62), Icons.keyboard_arrow_down_rounded),
              (Alignment(-0.62, 0), Icons.keyboard_arrow_left_rounded),
              (Alignment(0.62, 0), Icons.keyboard_arrow_right_rounded),
            ])
              Align(alignment: alignment, child: Icon(icon, size: 20, color: arrow)),
            Transform.translate(
              offset: _knob,
              child: Container(
                width: joystickKnobDiameter,
                height: joystickKnobDiameter,
                decoration: BoxDecoration(
                  shape: BoxShape.circle,
                  color: palette.accent,
                  boxShadow: [BoxShadow(color: Colors.black.withValues(alpha: 0.25), blurRadius: 10, offset: const Offset(0, 3))],
                ),
              ),
            ),
          ]),
        ),
      ),
    );
  }
}

