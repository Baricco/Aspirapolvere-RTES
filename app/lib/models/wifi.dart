/// Rete Wi-Fi del robot (docs/wifi.md, sezione 7): reti vicine, stato del Wi-Fi nello stato del robot, esiti dei cambi.
library;

import '../config/app_config.dart';

/// Una rete vista dal robot.
class WifiNetwork {
  const WifiNetwork(this.ssid, this.signalDbm, this.isProtected, this.inUse);

  final String ssid;
  final int signalDbm;
  final bool isProtected; // WPA o WPA2 con password; false = rete aperta
  final bool inUse;

  factory WifiNetwork.fromJson(Map<String, dynamic> json) => WifiNetwork(
        json['ssid'] as String? ?? '',
        (json['signal_dbm'] as num?)?.toInt() ?? -100,
        json['security'] == 'wpa',
        json['in_use'] == true,
      );
}

/// Risultato di una scansione: le reti e quando sono state cercate (con la rete del robot accesa può essere vecchia).
class WifiScan {
  const WifiScan(this.networks, this.scannedAt);

  final List<WifiNetwork> networks;
  final DateTime? scannedAt;

  factory WifiScan.fromJson(Map<String, dynamic> json) => WifiScan(
        [for (final network in (json['networks'] as List? ?? const [])) WifiNetwork.fromJson(network as Map<String, dynamic>)],
        json['scanned_at'] is num
            ? DateTime.fromMillisecondsSinceEpoch(((json['scanned_at'] as num) * 1000).toInt())
            : null,
      );
}

/// L'ultimo cambio di rete, come lo racconta rtes-wifi.
class WifiChange {
  const WifiChange(this.number, this.ssid, this.state, this.result);

  final int number; // 0 = nessun cambio da quando rtes-wifi è partito
  final String ssid;
  final String state; // none, testing, done
  final String result; // none, ok, backend_unreachable, internet_unreachable, no_address, wrong_password, not_found

  bool get isDone => state == 'done';

  @override
  bool operator ==(Object other) =>
      other is WifiChange && other.number == number && other.ssid == ssid && other.state == state && other.result == result;

  @override
  int get hashCode => Object.hash(number, ssid, state, result);
}

/// Il campo wifi dello stato del robot, uguale alla risposta di GET /api/wifi/status sulla rete del robot.
class WifiStatus {
  const WifiStatus(this.mode, this.ssid, this.signalDbm, this.ip, this.change);

  final String mode; // connecting, station, testing, ap
  final String? ssid;
  final int? signalDbm;
  final String? ip;
  final WifiChange change;

  factory WifiStatus.fromJson(Map<String, dynamic> json) {
    final change = json['change'] is Map<String, dynamic> ? json['change'] as Map<String, dynamic> : const {};
    return WifiStatus(
      json['mode'] as String? ?? 'connecting',
      json['ssid'] as String?,
      (json['signal_dbm'] as num?)?.toInt(),
      json['ip'] as String?,
      WifiChange(
        (change['number'] as num?)?.toInt() ?? 0,
        change['ssid'] as String? ?? '',
        change['state'] as String? ?? 'none',
        change['result'] as String? ?? 'none',
      ),
    );
  }
}

/// Tacche del segnale, da 1 a 4.
int signalBars(int dbm) {
  var bars = 4;
  for (final step in wifiSignalSteps) {
    if (dbm >= step) return bars;
    bars--;
  }
  return 1;
}

/// true se con questo esito il robot è rimasto sulla rete nuova.
bool isWifiChangeKept(String result) => result == 'ok' || result == 'backend_unreachable';

/// Il testo per l'utente di un esito (docs/wifi.md, sezione 5).
String wifiResultMessage(String result, String ssid) {
  switch (result) {
    case 'ok':
      return 'Maurice è collegato a «$ssid».';
    case 'backend_unreachable':
      return 'Maurice è su «$ssid» e ha internet, ma non raggiunge il server.';
    case 'internet_unreachable':
      return '«$ssid» non porta a internet: Maurice torna alla rete di prima.';
    case 'no_address':
      return 'Il router di «$ssid» non ha dato un indirizzo a Maurice: torna alla rete di prima.';
    case 'wrong_password':
      return 'Password sbagliata per «$ssid»: Maurice torna alla rete di prima.';
    case 'not_found':
      return 'Maurice non trova «$ssid» o non riesce a collegarsi: torna alla rete di prima.';
    default:
      return 'Cambio di rete verso «$ssid» finito.';
  }
}

/// Il robot non è tornato online entro wifiChangeWait.
String wifiTimeoutMessage(String ssid) =>
    'Maurice non è tornato online: può essere il server irraggiungibile dalla rete «$ssid». '
    'Quando lo raggiungerà vedrai l\'esito vero.';
