/// Client delle richieste al server (docs/app.md, sezioni 8.4 e 8.5): token e refresh in un punto solo, errori nella
/// forma {reason, message}, richiesta lunga dello stato.
library;


import 'package:dio/dio.dart';
import 'package:flutter/foundation.dart';
import 'package:flutter_secure_storage/flutter_secure_storage.dart';

import '../config/app_config.dart';
import 'http_adapter_io.dart' if (dart.library.js_interop) 'http_adapter_web.dart';

/// Un errore del server, o della rete: reason è il motivo per il codice, message il testo per l'utente.
class ApiException implements Exception {
  ApiException(this.status, this.reason, this.message, [this.details = const {}]);

  final int? status; // null: il server non ha risposto
  final String reason;
  final String message;
  final Map<String, dynamic> details;

  bool get isUnauthenticated => status == 401;

  @override
  String toString() => 'ApiException($status, $reason)';
}

const String _tokenStorageKey = 'maurice_token';

class ApiClient {
  ApiClient({Dio? dio, this.onUnauthenticated})
      : dio = dio ??
            Dio(BaseOptions(
              baseUrl: serverUrl,
              connectTimeout: requestTimeout,
              receiveTimeout: requestTimeout,
              // 304 è la risposta normale della richiesta lunga quando non cambia niente
              validateStatus: (status) => status != null && (status < 300 || status == 304),
            )) {
    configureHttpAdapter(this.dio);
  }

  final Dio dio;

  /// Chiamata quando il server risponde 401: la sessione è scaduta e si torna al login.
  void Function()? onUnauthenticated;

  /// Sul telefono il JWT sta nella memoria sicura del sistema; sul web non si salva, c'è il cookie.
  final FlutterSecureStorage _storage = const FlutterSecureStorage();
  String? _token;

  Future<void> loadToken() async {
    if (kIsWeb) return;
    _token = await _storage.read(key: _tokenStorageKey);
  }

  Future<void> _saveToken(String? token) async {
    _token = token;
    if (kIsWeb) return;
    if (token == null) {
      await _storage.delete(key: _tokenStorageKey);
    } else {
      await _storage.write(key: _tokenStorageKey, value: token);
    }
  }

  bool get hasToken => kIsWeb || _token != null;

  Options _options({ResponseType? responseType, Duration? timeout}) => Options(
        headers: {if (_token != null) 'Authorization': 'Bearer $_token'},
        responseType: responseType,
        receiveTimeout: timeout,
      );

  /// Trasforma un errore di dio in ApiException.
  ApiException _toApiException(DioException error) {
    final response = error.response;
    if (response == null) {
      return ApiException(null, 'server_unreachable', 'Il server non risponde.');
    }
    final body = response.data;
    if (body is Map<String, dynamic> && body['reason'] is String) {
      return ApiException(response.statusCode, body['reason'] as String, body['message'] as String? ?? '', body);
    }
    return ApiException(response.statusCode, 'error', 'Errore ${response.statusCode}.');
  }

  Future<Response<T>> _send<T>(
    String method,
    String path, {
    Object? body,
    Map<String, dynamic>? query,
    ResponseType? responseType,
    Duration? timeout,
    CancelToken? cancelToken,
    bool isLogin = false,
  }) async {
    try {
      final options = _options(responseType: responseType, timeout: timeout);
      options.method = method;
      return await dio.request<T>(path, data: body, queryParameters: query, options: options, cancelToken: cancelToken);
    } on DioException catch (error) {
      final exception = _toApiException(error);
      if (exception.isUnauthenticated && !isLogin) {
        await _saveToken(null);
        onUnauthenticated?.call();
      }
      throw exception;
    }
  }

  // ------------------------------------------------------------------------------------------- accesso

  Future<void> login(String username, String password) async {
    final response = await _send<Map<String, dynamic>>('POST', '/api/auth/login',
        body: {'username': username, 'password': password, 'use_cookie': kIsWeb}, isLogin: true);
    if (!kIsWeb) await _saveToken(response.data!['token'] as String);
  }

  /// A ogni apertura: JWT nuovo per la stessa sessione. Solleva ApiException con 401 se la sessione è scaduta.
  Future<void> refresh() async {
    final response = await _send<Map<String, dynamic>>('POST', '/api/auth/refresh');
    if (!kIsWeb) await _saveToken(response.data!['token'] as String);
  }

  Future<void> logout() async {
    try {
      await _send<dynamic>('POST', '/api/auth/logout');
    } on ApiException {
      // la sessione si chiude comunque da questa parte
    }
    await _saveToken(null);
  }

  Future<String> me() async {
    final response = await _send<Map<String, dynamic>>('GET', '/api/auth/me');
    return response.data!['username'] as String;
  }

  // ----------------------------------------------------------------------------------------- richieste

  /// Richiesta lunga dello stato. Senza after risponde subito; con after aspetta uno stato più nuovo e ritorna null
  /// se in 25 s non cambia niente (304).
  Future<Map<String, dynamic>?> getState({int? after, int minIntervalMs = 0, CancelToken? cancelToken}) async {
    final response = await _send<dynamic>('GET', '/api/robot/state',
        query: {'after': ?after, if (after != null) 'min_interval_ms': minIntervalMs},
        timeout: stateRequestTimeout,
        cancelToken: cancelToken);
    if (response.statusCode == 304) return null;
    return response.data as Map<String, dynamic>;
  }

  Future<dynamic> getJson(String path, {Map<String, dynamic>? query}) async =>
      (await _send<dynamic>('GET', path, query: query)).data;

  Future<Uint8List> getBytes(String path, {Map<String, dynamic>? query}) async {
    final response = await _send<List<int>>('GET', path, query: query, responseType: ResponseType.bytes);
    return Uint8List.fromList(response.data!);
  }

  /// Un comando o una modifica: POST, PUT, PATCH o DELETE. I rifiuti arrivano come ApiException (409, 503, 504...).
  Future<dynamic> send(String method, String path, [Object? body]) async =>
      (await _send<dynamic>(method, path, body: body)).data;
}
