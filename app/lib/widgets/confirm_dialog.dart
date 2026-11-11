/// Conferma con avviso: ogni cancellazione la chiede (docs/app.md, sezione 10.3).
library;

import 'package:flutter/material.dart';

import '../config/app_colors.dart';

/// Mostra l'avviso e ritorna true solo se l'utente conferma.
Future<bool> confirmWithWarning(BuildContext context,
    {required String title, required String warning, String confirmLabel = 'Cancella'}) async {
  final palette = AppPalette.of(context);
  final confirmed = await showDialog<bool>(
    context: context,
    builder: (context) => AlertDialog(
      icon: Icon(Icons.warning_amber_rounded, color: palette.fault, size: 36),
      title: Text(title),
      content: Text(warning),
      actions: [
        TextButton(onPressed: () => Navigator.of(context).pop(false), child: const Text('Annulla')),
        FilledButton(
          style: FilledButton.styleFrom(backgroundColor: palette.fault, foregroundColor: Colors.white),
          onPressed: () => Navigator.of(context).pop(true),
          child: Text(confirmLabel),
        ),
      ],
    ),
  );
  return confirmed == true;
}
