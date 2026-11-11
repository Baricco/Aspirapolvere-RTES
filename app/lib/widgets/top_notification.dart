/// La notifica in alto, sopra qualsiasi schermata: i rifiuti dei comandi con il loro messaggio (docs/app.md, 10.3).
library;

import 'package:flutter/material.dart';
import 'package:flutter_riverpod/flutter_riverpod.dart';

import '../api/providers.dart';
import '../config/app_colors.dart';

class TopNotificationOverlay extends ConsumerWidget {
  const TopNotificationOverlay({super.key, required this.child});

  final Widget child;

  @override
  Widget build(BuildContext context, WidgetRef ref) {
    final notification = ref.watch(notificationProvider);
    final palette = AppPalette.of(context);
    return Stack(children: [
      child,
      Positioned(
        left: screenPadding,
        right: screenPadding,
        top: MediaQuery.paddingOf(context).top + 8,
        child: IgnorePointer(
          ignoring: notification == null,
          child: AnimatedSlide(
            duration: const Duration(milliseconds: 220),
            curve: Curves.easeOut,
            offset: notification == null ? const Offset(0, -1.6) : Offset.zero,
            child: AnimatedOpacity(
              duration: const Duration(milliseconds: 220),
              opacity: notification == null ? 0 : 1,
              child: Align(
                alignment: Alignment.topCenter,
                child: ConstrainedBox(
                  constraints: const BoxConstraints(maxWidth: 520),
                  child: Material(
                    // chiara sul tema scuro e scura sul chiaro, come nel mockup: si deve notare
                    color: palette.accent,
                    elevation: 8,
                    borderRadius: BorderRadius.circular(controlRadius + 2),
                    child: InkWell(
                      borderRadius: BorderRadius.circular(controlRadius + 2),
                      onTap: () => ref.read(notificationProvider.notifier).hide(),
                      child: Padding(
                        padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 12),
                        child: Row(crossAxisAlignment: CrossAxisAlignment.start, children: [
                          Icon(Icons.info_outline_rounded, color: palette.warning, size: 22),
                          const SizedBox(width: 12),
                          Expanded(
                            child: Column(crossAxisAlignment: CrossAxisAlignment.start, children: [
                              Text(notification?.title ?? '',
                                  style: TextStyle(color: palette.background, fontWeight: FontWeight.w700, fontSize: 16)),
                              Text(notification?.message ?? '',
                                  style: TextStyle(color: palette.background.withValues(alpha: 0.86), fontSize: 15)),
                            ]),
                          ),
                        ]),
                      ),
                    ),
                  ),
                ),
              ),
            ),
          ),
        ),
      ),
    ]);
  }
}
