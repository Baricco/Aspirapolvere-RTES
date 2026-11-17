/// Conferma con avviso: ogni cancellazione la chiede (docs/app.md, sezione 10.3).
library;

import 'package:flutter/material.dart';
import 'package:flutter_riverpod/flutter_riverpod.dart';

import '../api/providers.dart';

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

/// Obiettivo dalla mappa (pressione lunga, nella schermata principale e nel telecomando): conferma, poi il comando. Il
/// robot ci va e poi torna a quello che faceva (con il telecomando attivo lo rilascia); se non ci arriva si mette in
/// pausa e l'esito arriva nello stato (navigation.target).
Future<void> confirmGoTo(BuildContext context, WidgetRef ref, Offset point) async {
  final confirmed = await showDialog<bool>(
    context: context,
    builder: (context) => AlertDialog(
      title: const Text('Mandare il robot qui?'),
      content: const Text('Il robot va in questo punto, poi torna a quello che stava facendo. '
          'Se non riesce ad arrivarci si mette in pausa.'),
      actions: [
        TextButton(onPressed: () => Navigator.of(context).pop(false), child: const Text('Annulla')),
        FilledButton(onPressed: () => Navigator.of(context).pop(true), child: const Text('Vai')),
      ],
    ),
  );
  if (confirmed != true) return;
  final x = (point.dx * 100).roundToDouble() / 100, y = (point.dy * 100).roundToDouble() / 100;
  await runCommand(ref, (api) => api.send('POST', '/api/robot/goto', {'x': x, 'y': y}));
}
