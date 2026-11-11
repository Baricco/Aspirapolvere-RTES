/// I pulsanti dei comandi (docs/app.md, sezioni 5.4 e 10.5): il principale è pieno di petrolio, Casa e i secondari
/// hanno la tinta leggera, gli altri sono neutri.
library;

import 'package:flutter/material.dart';

import '../config/app_colors.dart';

enum ActionButtonStyle { primary, tinted, neutral, danger }

class ActionButton extends StatelessWidget {
  const ActionButton({
    super.key,
    required this.label,
    required this.icon,
    required this.onPressed,
    this.style = ActionButtonStyle.neutral,
    this.vertical = false,
    this.height = 56,
  });

  final String label;
  final IconData icon;
  final VoidCallback? onPressed; // null: pulsante spento
  final ActionButtonStyle style;
  final bool vertical; // icona sopra il testo, per i pulsanti quadrati del telecomando
  final double height;

  @override
  Widget build(BuildContext context) {
    final palette = AppPalette.of(context);
    final enabled = onPressed != null;
    final (background, foreground, border) = switch (style) {
      ActionButtonStyle.primary => (palette.primary, palette.onPrimary, null),
      ActionButtonStyle.tinted => (palette.primaryTint, palette.primary, null),
      ActionButtonStyle.neutral => (palette.control, palette.text, null),
      ActionButtonStyle.danger => (palette.fault.withValues(alpha: 0.10), palette.fault, palette.fault),
    };
    // in verticale le righe sono quelle dell'etichetta (con \n): se non c'entrano il contenuto rimpicciolisce, senza
    // spezzare le parole
    final content = vertical
        ? FittedBox(
            fit: BoxFit.scaleDown,
            child: Column(mainAxisSize: MainAxisSize.min, children: [
              Icon(icon, size: 24),
              const SizedBox(height: 6),
              Text(label,
                  textAlign: TextAlign.center,
                  softWrap: false,
                  style: const TextStyle(fontSize: 14.5, fontWeight: FontWeight.w600, height: 1.15)),
            ]),
          )
        // se lo spazio non basta il pulsante rimpicciolisce il contenuto invece di tagliare il testo
        : FittedBox(
            fit: BoxFit.scaleDown,
            child: Row(mainAxisSize: MainAxisSize.min, children: [
              Icon(icon, size: 20),
              const SizedBox(width: 7),
              Text(label, maxLines: 1, style: const TextStyle(fontSize: 16, fontWeight: FontWeight.w600)),
            ]),
          );
    return Opacity(
      opacity: enabled ? 1 : 0.45,
      child: Material(
        color: background,
        shape: RoundedRectangleBorder(
          borderRadius: BorderRadius.circular(controlRadius),
          side: border == null ? BorderSide.none : BorderSide(color: border),
        ),
        clipBehavior: Clip.antiAlias,
        child: InkWell(
          onTap: onPressed,
          child: SizedBox(
            height: height,
            child: Padding(
              padding: EdgeInsets.symmetric(horizontal: vertical ? 6 : 10),
              child: IconTheme.merge(
                data: IconThemeData(color: foreground),
                child: DefaultTextStyle.merge(style: TextStyle(color: foreground), child: Center(child: content)),
              ),
            ),
          ),
        ),
      ),
    );
  }
}
