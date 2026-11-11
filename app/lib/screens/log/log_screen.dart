/// Registro dei comandi (docs/app.md, sezione 10.4): chi, cosa, quando, esito; dal più recente.
library;

import 'package:flutter/material.dart';
import 'package:flutter_riverpod/flutter_riverpod.dart';

import '../../api/api_client.dart';
import '../../api/providers.dart';
import '../../config/app_colors.dart';
import '../../config/app_config.dart';
import '../../models/log_entry.dart';
import '../../widgets/dates.dart';
import '../../widgets/state_interval.dart';

/// I comandi detti in italiano.
const Map<String, String> commandNames = {
  'start': 'Avvia',
  'pause': 'Pausa',
  'resume': 'Riprendi',
  'home': 'Casa',
  'fault_reset': 'Azzera guasto',
  'remote_take': 'Telecomando: preso',
  'remote_move': 'Telecomando: movimento',
  'remote_cleaning': 'Telecomando: spazzole e aspirazione',
  'remote_keepalive': 'Telecomando: segno di vita',
  'remote_release': 'Telecomando: rilasciato',
  'zones': 'Zone proibite',
  'map_select': 'Cambio di mappa',
  'map_reset': 'Nessuna mappa',
};

/// Gli esiti detti in italiano.
String statusText(LogEntry entry) => switch (entry.status) {
      'ok' => 'eseguito',
      'rejected' => 'rifiutato${entry.reason == null ? '' : ' (${entry.reason})'}',
      'robot_unreachable' => 'robot non raggiungibile',
      'no_result' => 'nessun esito dal robot',
      _ => entry.status,
    };

/// Un numero con la virgola e le cifre decimali date, senza il segno davanti allo zero.
String decimal(dynamic value, int digits) {
  final number = value is num ? value.toDouble() : 0.0;
  final text = (number.abs() < 0.5 / 1000 ? 0.0 : number).toStringAsFixed(digits);
  return text.replaceAll('.', ',');
}

/// I parametri che vale la pena mostrare.
String? paramsText(LogEntry entry) {
  final params = entry.params;
  if (params == null) return null;
  return switch (entry.type) {
    'remote_move' => 'v ${decimal(params['v'], 2)} m/s, ω ${decimal(params['omega'], 2)} rad/s',
    'remote_cleaning' => 'spazzole ${params['brushes'] == true ? 'sì' : 'no'}, aspirazione ${params['suction'] == true ? 'sì' : 'no'}',
    'zones' => '${(params['zones'] as List?)?.length ?? 0} zone',
    'map_select' => 'mappa ${params['map_id']}',
    _ => null,
  };
}

class LogScreen extends ConsumerStatefulWidget {
  const LogScreen({super.key});

  @override
  ConsumerState<LogScreen> createState() => _LogScreenState();
}

class _LogScreenState extends ConsumerState<LogScreen> with StateIntervalMixin {
  final List<LogEntry> _entries = [];
  bool _loading = false;
  bool _finished = false; // non ci sono voci più vecchie
  String? _error;

  @override
  int get stateIntervalMs => stateIntervalOtherMs;

  @override
  void initState() {
    super.initState();
    _loadMore();
  }

  Future<void> _loadMore({bool restart = false}) async {
    if (_loading) return;
    setState(() {
      _loading = true;
      _error = null;
    });
    try {
      final page = await fetchLog(ref.read(apiClientProvider), before: restart || _entries.isEmpty ? null : _entries.last.id);
      if (!mounted) return;
      setState(() {
        if (restart) _entries.clear();
        _entries.addAll(page);
        _finished = page.length < logPageSize;
      });
    } on ApiException catch (error) {
      if (mounted) setState(() => _error = error.message);
    } finally {
      if (mounted) setState(() => _loading = false);
    }
  }

  @override
  Widget build(BuildContext context) {
    final palette = AppPalette.of(context);
    return Scaffold(
      appBar: ScreenHeader(title: 'Registro dei comandi', trailing: [
        IconButton(icon: const Icon(Icons.refresh_rounded), tooltip: 'Aggiorna', onPressed: () => _loadMore(restart: true)),
      ]),
      body: _entries.isEmpty
          ? Center(
              child: _loading
                  ? const CircularProgressIndicator()
                  : Text(_error ?? 'Nessun comando nel registro.', style: TextStyle(color: palette.textSecondary, fontSize: 16)))
          : ListView.separated(
              padding: const EdgeInsets.fromLTRB(screenPadding, 4, screenPadding, 24),
              itemCount: _entries.length + 1,
              separatorBuilder: (_, _) => Divider(height: 1, color: palette.panelBorder),
              itemBuilder: (context, index) {
                if (index == _entries.length) {
                  return Padding(
                    padding: const EdgeInsets.symmetric(vertical: 14),
                    child: Center(
                      child: _finished
                          ? Text('Il registro tiene gli ultimi 30 giorni.', style: TextStyle(color: palette.textSecondary))
                          : TextButton(onPressed: _loading ? null : _loadMore, child: Text(_loading ? 'Carico...' : 'Mostra i precedenti')),
                    ),
                  );
                }
                final entry = _entries[index];
                final params = paramsText(entry);
                return Padding(
                  padding: const EdgeInsets.symmetric(vertical: 10),
                  child: Row(crossAxisAlignment: CrossAxisAlignment.start, children: [
                    Padding(
                      padding: const EdgeInsets.only(top: 3, right: 12),
                      child: Icon(entry.ok ? Icons.check_circle_outline_rounded : Icons.error_outline_rounded,
                          size: 20, color: entry.ok ? palette.online : palette.fault),
                    ),
                    Expanded(
                      child: Column(crossAxisAlignment: CrossAxisAlignment.start, children: [
                        Text(commandNames[entry.type] ?? entry.type, style: const TextStyle(fontSize: 16, fontWeight: FontWeight.w600)),
                        if (params != null) Text(params, style: TextStyle(color: palette.textSecondary, fontSize: 14)),
                        Text(statusText(entry), style: TextStyle(color: entry.ok ? palette.textSecondary : palette.fault, fontSize: 14)),
                      ]),
                    ),
                    Column(crossAxisAlignment: CrossAxisAlignment.end, children: [
                      Text(entry.scheduled ? 'programmazione' : (entry.user ?? 'utente tolto'),
                          style: const TextStyle(fontSize: 14, fontWeight: FontWeight.w600)),
                      Text(shortDate(entry.time), style: TextStyle(color: palette.textSecondary, fontSize: 13)),
                    ]),
                  ]),
                );
              },
            ),
    );
  }
}
