/// Pannello dello stile Vetro (docs/app.md, sezione 10.5): traslucido, sfocato, con gli angoli arrotondati.
library;

import 'dart:ui';

import 'package:flutter/material.dart';

import '../config/app_colors.dart';

class GlassPanel extends StatelessWidget {
  const GlassPanel({super.key, required this.child, this.padding = const EdgeInsets.all(16), this.radius = panelRadius});

  final Widget child;
  final EdgeInsetsGeometry padding;
  final double radius;

  @override
  Widget build(BuildContext context) {
    final palette = AppPalette.of(context);
    return ClipRRect(
      borderRadius: BorderRadius.circular(radius),
      child: BackdropFilter(
        filter: ImageFilter.blur(sigmaX: panelBlur, sigmaY: panelBlur),
        child: DecoratedBox(
          decoration: BoxDecoration(
            color: palette.panel,
            borderRadius: BorderRadius.circular(radius),
            border: Border.all(color: palette.panelBorder),
          ),
          // un Material trasparente qui dentro: le evidenziazioni dei pulsanti si disegnano dentro il pannello arrotondato
          child: Material(type: MaterialType.transparency, child: Padding(padding: padding, child: child)),
        ),
      ),
    );
  }
}

/// Pulsante quadrato con un'icona, dentro un pannello di vetro (menu, ricentra, gestione mappe).
class GlassIconButton extends StatelessWidget {
  const GlassIconButton({super.key, required this.icon, required this.onPressed, this.tooltip, this.size = 52});

  final IconData icon;
  final VoidCallback? onPressed;
  final String? tooltip;
  final double size;

  @override
  Widget build(BuildContext context) {
    final palette = AppPalette.of(context);
    return SizedBox(
      width: size,
      height: size,
      child: GlassPanel(
        padding: EdgeInsets.zero,
        radius: size >= 52 ? controlRadius + 2 : size / 2,
        child: IconButton(
          icon: Icon(icon, size: 22),
          color: onPressed == null ? palette.textSecondary : palette.text,
          tooltip: tooltip,
          onPressed: onPressed,
        ),
      ),
    );
  }
}
