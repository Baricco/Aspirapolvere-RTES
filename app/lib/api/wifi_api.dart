/// Richieste del Wi-Fi del robot (docs/wifi.md, sezioni 7.2, 7.3 e 10.2): le stesse dal backend (robot online) e dal
/// robot sulla sua rete (telefono collegato a Maurice-WiFi). Cambia solo l'indirizzo.
library;

import 'package:dio/dio.dart';
import 'package:flutter/foundation.dart';
import 'package:flutter/services.dart';
import 'package:flutter_riverpod/flutter_riverpod.dart';

import '../config/app_config.dart';
import '../models/wifi.dart';
import 'api_client.dart';

abstract class WifiApi {
  /// Reti vicine.
  Future<WifiScan> scan();

  /// Cambio di rete: ritorna il numero del cambio; l'esito arriva poi nello stato del Wi-Fi.
  Future<int> connect(String ssid, bool isProtected, String password);
}

Map<String, dynamic> _connectBody(String ssid, bool isProtected, String password) =>
    {'ssid': ssid, 'security': isProtected ? 'wpa' : 'open', if (isProtected) 'password': password};

/// Dal backend, con la sessione dell'utente.
class BackendWifiApi implements WifiApi {
  BackendWifiApi(this.api);

  final ApiClient api;

  @override
  Future<WifiScan> scan() async =>
      WifiScan.fromJson(await api.getJson('/api/robot/wifi/networks') as Map<String, dynamic>);

  @override
  Future<int> connect(String ssid, bool isProtected, String password) async {
    final body = await api.send('POST', '/api/robot/wifi/connect', _connectBody(ssid, isProtected, password));
    return ((body as Map<String, dynamic>)['change_number'] as num).toInt();
  }
}

/// Su Android le richieste vanno forzate sulla rete Wi-Fi: la rete del robot non ha internet, e senza questo il
/// telefono le manda sui dati mobili (canale "maurice/wifi_network" in MainActivity.kt). Sugli altri sistemi non fa niente.
class WifiNetworkBinding {
  static const MethodChannel _channel = MethodChannel('maurice/wifi_network');

  static bool get _isAndroid => !kIsWeb && defaultTargetPlatform == TargetPlatform.android;

  /// Lega le richieste dell'app alla rete Wi-Fi di adesso. Ritorna false se il telefono non è su una rete Wi-Fi.
  static Future<bool> bindToWifi() async {
    if (!_isAndroid) return true;
    try {
      return await _channel.invokeMethod<bool>('bindToWifi') ?? false;
    } on PlatformException {
      return false;
    } on MissingPluginException {
      return false;
    }
  }

  /// Le richieste tornano sulla rete scelta dal sistema (quella con internet).
  static Future<void> unbind() async {
    if (!_isAndroid) return;
    try {
      await _channel.invokeMethod<void>('unbind');
    } on PlatformException {
      // niente da fare
    } on MissingPluginException {
      // niente da fare
    }
  }
}

/// Dal robot sulla sua rete: rtes-wifi su http://192.168.5.1, senza sessione (protegge la password della rete).
class LocalWifiApi implements WifiApi {
  final Dio _dio = Dio(BaseOptions(
    baseUrl: robotLocalUrl,
    connectTimeout: robotLocalProbeTimeout,
    receiveTimeout: wifiScanTimeout,
  ));

  ApiException _toApiException(DioException error) {
    final body = error.response?.data;
    if (body is Map<String, dynamic> && body['reason'] is String) {
      return ApiException(error.response?.statusCode, body['reason'] as String, body['message'] as String? ?? '', body);
    }
    return ApiException(null, 'robot_unreachable', 'Maurice non risponde sulla sua rete.');
  }

  Future<dynamic> _request(String method, String path, {Object? body, Duration? timeout}) async {
    await WifiNetworkBinding.bindToWifi();
    try {
      final response = await _dio.request<dynamic>(path,
          data: body, options: Options(method: method, receiveTimeout: timeout));
      return response.data;
    } on DioException catch (error) {
      throw _toApiException(error);
    }
  }

  /// Stato del Wi-Fi del robot, oppure null se il telefono non è sulla rete del robot.
  Future<WifiStatus?> status() async {
    try {
      final body = await _request('GET', '/api/wifi/status', timeout: robotLocalProbeTimeout);
      return body is Map<String, dynamic> ? WifiStatus.fromJson(body) : null;
    } on ApiException {
      await WifiNetworkBinding.unbind();
      return null;
    }
  }

  @override
  Future<WifiScan> scan() async =>
      WifiScan.fromJson(await _request('GET', '/api/wifi/networks') as Map<String, dynamic>);

  @override
  Future<int> connect(String ssid, bool isProtected, String password) async {
    final body = await _request('POST', '/api/wifi/connect', body: _connectBody(ssid, isProtected, password));
    return ((body as Map<String, dynamic>)['change_number'] as num).toInt();
  }
}

/// Un cambio di rete chiesto da questa app e non ancora finito: se la schermata del Wi-Fi è chiusa quando arriva
/// l'esito (anche molto dopo, docs/wifi.md sezione 13), la schermata principale lo mostra come notifica.
class PendingWifiChange {
  const PendingWifiChange(this.number, this.ssid, {this.isScreenOpen = true});

  final int number;
  final String ssid;
  final bool isScreenOpen;
}

class PendingWifiChangeNotifier extends Notifier<PendingWifiChange?> {
  @override
  PendingWifiChange? build() => null;

  void start(int number, String ssid) => state = PendingWifiChange(number, ssid);

  void screenClosed() {
    final change = state;
    if (change != null) state = PendingWifiChange(change.number, change.ssid, isScreenOpen: false);
  }

  void clear() => state = null;
}

final pendingWifiChangeProvider =
    NotifierProvider<PendingWifiChangeNotifier, PendingWifiChange?>(PendingWifiChangeNotifier.new);
