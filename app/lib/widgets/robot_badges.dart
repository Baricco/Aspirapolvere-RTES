/// "Online"/"Offline" (in inglese) con l'ultimo contatto, e la batteria: in alto nella principale e nel telecomando.
library;

import 'package:flutter/material.dart';

import '../config/app_colors.dart';
import '../models/robot_state.dart';

/// Da quanto tempo, detto in breve: "adesso", "3 min fa", "2 h fa", "5 g fa".
String timeAgo(double? epochSeconds) {
  if (epochSeconds == null) return 'mai visto';
  final seconds = DateTime.now().millisecondsSinceEpoch / 1000 - epochSeconds;
  if (seconds < 60) return 'adesso';
  if (seconds < 3600) return '${seconds ~/ 60} min fa';
  if (seconds < 86400) return '${seconds ~/ 3600} h fa';
  return '${seconds ~/ 86400} g fa';
}

class OnlineBadge extends StatelessWidget {
  const OnlineBadge({super.key, required this.state});

  final RobotState? state;

  @override
  Widget build(BuildContext context) {
    final palette = AppPalette.of(context);
    final online = state?.online ?? false;
    final color = online ? palette.online : palette.textSecondary;
    return Container(
      padding: const EdgeInsets.symmetric(horizontal: 12, vertical: 6),
      decoration: BoxDecoration(color: palette.control, borderRadius: BorderRadius.circular(20)),
      child: Row(mainAxisSize: MainAxisSize.min, children: [
        Container(width: 9, height: 9, decoration: BoxDecoration(color: color, shape: BoxShape.circle)),
        const SizedBox(width: 7),
        Text(online ? 'Online' : 'Offline', style: const TextStyle(fontWeight: FontWeight.w600, fontSize: 15)),
        if (!online && state != null) ...[
          const SizedBox(width: 6),
          Text(timeAgo(state!.lastContact), style: TextStyle(color: palette.textSecondary, fontSize: 13)),
        ],
      ]),
    );
  }
}

class BatteryBadge extends StatelessWidget {
  const BatteryBadge({super.key, required this.state});

  final RobotState? state;

  @override
  Widget build(BuildContext context) {
    final palette = AppPalette.of(context);
    final percent = state?.batteryPercent;
    if (percent == null) return const SizedBox.shrink();
    final low = state!.batteryLow;
    return Row(mainAxisSize: MainAxisSize.min, children: [
      if (state!.charging) Icon(Icons.bolt_rounded, size: 18, color: palette.primary),
      Text('$percent%',
          style: TextStyle(color: low ? palette.fault : palette.textSecondary, fontWeight: FontWeight.w600, fontSize: 15)),
    ]);
  }
}
