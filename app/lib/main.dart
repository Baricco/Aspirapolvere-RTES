/// App di Maurice, il robot aspirapolvere (docs/app.md, sezione 10).
library;

import 'package:flutter/material.dart';
import 'package:flutter_riverpod/flutter_riverpod.dart';

import 'api/providers.dart';
import 'config/app_colors.dart';
import 'config/app_config.dart';
import 'screens/login/login_screen.dart';
import 'screens/main/main_screen.dart';
import 'widgets/top_notification.dart';

void main() {
  runApp(const ProviderScope(child: MauriceApp()));
}

class MauriceApp extends ConsumerStatefulWidget {
  const MauriceApp({super.key});

  @override
  ConsumerState<MauriceApp> createState() => _MauriceAppState();
}

class _MauriceAppState extends ConsumerState<MauriceApp> with WidgetsBindingObserver {
  @override
  void initState() {
    super.initState();
    WidgetsBinding.instance.addObserver(this);
  }

  @override
  void dispose() {
    WidgetsBinding.instance.removeObserver(this);
    super.dispose();
  }

  /// Con l'app in background non si fa nessuna richiesta: la richiesta lunga dello stato si ferma.
  @override
  void didChangeAppLifecycleState(AppLifecycleState state) {
    ref.read(appActiveProvider.notifier).set(state == AppLifecycleState.resumed);
  }

  @override
  Widget build(BuildContext context) {
    final session = ref.watch(sessionProvider);
    return MaterialApp(
      title: appName,
      debugShowCheckedModeBanner: false,
      theme: buildTheme(AppPalette.light),
      darkTheme: buildTheme(AppPalette.dark),
      themeMode: ThemeMode.system, // il tema segue il telefono
      builder: (context, child) => TopNotificationOverlay(child: child ?? const SizedBox.shrink()),
      // la chiave cambia con l'accesso: al logout tutte le schermate aperte spariscono
      home: KeyedSubtree(
        key: ValueKey(session.status),
        child: switch (session.status) {
          SessionStatus.unknown => const _OpeningScreen(),
          SessionStatus.loggedOut => const LoginScreen(),
          SessionStatus.loggedIn => const MainScreen(),
        },
      ),
    );
  }
}

class _OpeningScreen extends StatelessWidget {
  const _OpeningScreen();

  @override
  Widget build(BuildContext context) {
    return const Scaffold(body: Center(child: CircularProgressIndicator()));
  }
}
