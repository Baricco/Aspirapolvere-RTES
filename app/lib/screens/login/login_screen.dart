/// Accesso: utente e password. Nessuna registrazione: gli utenti li crea lo script di amministrazione.
library;

import 'package:flutter/material.dart';
import 'package:flutter_riverpod/flutter_riverpod.dart';

import '../../api/api_client.dart';
import '../../api/providers.dart';
import '../../config/app_colors.dart';
import '../../config/app_config.dart';
import '../../widgets/main_button.dart';

class LoginScreen extends ConsumerStatefulWidget {
  const LoginScreen({super.key});

  @override
  ConsumerState<LoginScreen> createState() => _LoginScreenState();
}

class _LoginScreenState extends ConsumerState<LoginScreen> {
  final _username = TextEditingController();
  final _password = TextEditingController();
  bool _busy = false;
  String? _error;

  @override
  void dispose() {
    _username.dispose();
    _password.dispose();
    super.dispose();
  }

  Future<void> _login() async {
    if (_busy || _username.text.trim().isEmpty || _password.text.isEmpty) return;
    setState(() {
      _busy = true;
      _error = null;
    });
    try {
      await ref.read(sessionProvider.notifier).login(_username.text.trim(), _password.text);
    } on ApiException catch (error) {
      if (mounted) setState(() => _error = error.message);
    } finally {
      if (mounted) setState(() => _busy = false);
    }
  }

  @override
  Widget build(BuildContext context) {
    final palette = AppPalette.of(context);
    return Scaffold(
      body: SafeArea(
        child: Center(
          child: SingleChildScrollView(
            padding: const EdgeInsets.all(28),
            child: ConstrainedBox(
              constraints: const BoxConstraints(maxWidth: 380),
              child: AutofillGroup(
                child: Column(crossAxisAlignment: CrossAxisAlignment.stretch, children: [
                  Text(appName, textAlign: TextAlign.center, style: const TextStyle(fontSize: 40, fontWeight: FontWeight.w800)),
                  const SizedBox(height: 6),
                  Text('Robot aspirapolvere', textAlign: TextAlign.center, style: TextStyle(color: palette.textSecondary, fontSize: 16)),
                  const SizedBox(height: 36),
                  TextField(
                    controller: _username,
                    autofillHints: const [AutofillHints.username],
                    textInputAction: TextInputAction.next,
                    decoration: const InputDecoration(labelText: 'Utente'),
                  ),
                  const SizedBox(height: 14),
                  TextField(
                    controller: _password,
                    obscureText: true,
                    autofillHints: const [AutofillHints.password],
                    onSubmitted: (_) => _login(),
                    decoration: const InputDecoration(labelText: 'Password'),
                  ),
                  SizedBox(
                    height: 44,
                    child: Center(
                      child: _error == null
                          ? null
                          : Text(_error!, textAlign: TextAlign.center, style: TextStyle(color: palette.fault, fontSize: 14)),
                    ),
                  ),
                  ActionButton(
                    label: _busy ? 'Accesso...' : 'Accedi',
                    icon: Icons.login_rounded,
                    style: ActionButtonStyle.primary,
                    onPressed: _busy ? null : _login,
                  ),
                ]),
              ),
            ),
          ),
        ),
      ),
    );
  }
}
