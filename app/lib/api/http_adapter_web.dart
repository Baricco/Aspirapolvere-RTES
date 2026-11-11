/// Sul web il JWT sta in un cookie HttpOnly che il JavaScript non vede: il browser lo manda da solo, anche al backend
/// di sviluppo su un'altra origine, solo se le richieste hanno withCredentials.
library;

import 'package:dio/browser.dart';
import 'package:dio/dio.dart';

void configureHttpAdapter(Dio dio) {
  dio.httpClientAdapter = BrowserHttpClientAdapter(withCredentials: true);
}
