/// Ogni schermata sceglie l'intervallo minimo fra due stati (docs/app.md, sezione 4.4): 100 ms con la mappa, 1000 ms
/// nelle altre. Quando la schermata si chiude torna quello della schermata sotto.
library;

import 'package:flutter/material.dart';
import 'package:flutter_riverpod/flutter_riverpod.dart';

import '../api/providers.dart';

mixin StateIntervalMixin<T extends ConsumerStatefulWidget> on ConsumerState<T> {
  /// L'intervallo voluto da questa schermata, in millisecondi.
  int get stateIntervalMs;

  int? _previousInterval;
  late final StateIntervalNotifier _notifier = ref.read(stateIntervalProvider.notifier);

  @override
  void initState() {
    super.initState();
    // i provider non si possono cambiare mentre l'albero dei widget si costruisce
    Future.microtask(() {
      if (!mounted) return;
      _previousInterval = ref.read(stateIntervalProvider);
      _notifier.set(stateIntervalMs);
    });
  }

  @override
  void dispose() {
    final previous = _previousInterval;
    if (previous != null) Future.microtask(() => _notifier.set(previous));
    super.dispose();
  }
}

/// Barra in alto delle schermate secondarie: indietro, titolo, e a destra quello che serve.
class ScreenHeader extends StatelessWidget implements PreferredSizeWidget {
  const ScreenHeader({super.key, required this.title, this.trailing = const []});

  final String title;
  final List<Widget> trailing;

  @override
  Size get preferredSize => const Size.fromHeight(64);

  @override
  Widget build(BuildContext context) {
    return AppBar(
      title: Text(title),
      titleSpacing: 4,
      leading: IconButton(
        icon: const Icon(Icons.arrow_back_ios_new_rounded, size: 20),
        tooltip: 'Indietro',
        onPressed: () => Navigator.of(context).maybePop(),
      ),
      actions: [...trailing, const SizedBox(width: 12)],
    );
  }
}
