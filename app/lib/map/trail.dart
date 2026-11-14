/// La scia dell'ultima missione (docs/app.md, sezione 7.4): la registra il server, l'app chiede solo i punti nuovi.
/// Il server registra solo mentre il robot aspira: dove l'aspirazione si è spenta la scia ha un'interruzione (null nel
/// messaggio), che qui diventa [trailBreak]. Così la scia disegnata è la zona pulita.
library;

import 'dart:ui';

/// Interruzione della scia: fra il punto prima e quello dopo non si disegna niente.
const Offset trailBreak = Offset(double.nan, double.nan);

/// true se il punto della scia è un'interruzione.
bool isTrailBreak(Offset point) => point.dx.isNaN;

class Trail {
  /// Identifica la scia sul server: quando cambia è cominciata una missione nuova.
  int? trailId;

  /// I punti, in metri sulla mappa.
  final List<Offset> points = [];

  /// Applica la risposta di GET /api/robot/trail?after=N. Se la scia è un'altra, o il server riparte dall'inizio,
  /// quella vecchia si butta.
  void applyMessage(Map<String, dynamic> message) {
    final id = (message['trail_id'] as num).toInt();
    final offset = (message['offset'] as num).toInt();
    if (id != trailId || offset == 0 || offset > points.length) {
      points.clear();
      trailId = id;
    } else if (offset < points.length) {
      points.removeRange(offset, points.length);
    }
    // se la scia è cambiata ma il server ha risposto dal punto chiesto, mancano i punti prima: alla prossima si richiede tutto
    if (offset != points.length) {
      trailId = null;
      return;
    }
    for (final point in message['points'] as List) {
      if (point == null) {
        points.add(trailBreak);
      } else {
        points.add(Offset(((point as List)[0] as num).toDouble(), (point[1] as num).toDouble()));
      }
    }
  }
}
