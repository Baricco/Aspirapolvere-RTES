/// Configurazione dell'app (docs/app.md, sezione 10.2). I valori privati arrivano dal file config.json, che git ignora,
/// letto alla compilazione con `flutter run --dart-define-from-file=config.json` (esempio: config.example.json).
library;

/// Nome mostrato nell'app.
const String appName = String.fromEnvironment('APP_NAME', defaultValue: 'Maurice');

/// Indirizzo del server, senza la barra in fondo. Vuoto: lo stesso indirizzo da cui è servita l'app web.
const String serverUrl = String.fromEnvironment('SERVER_URL');

// --- tempi ---

/// Intervallo minimo fra due stati nelle schermate con la mappa (principale e telecomando), in millisecondi.
const int stateIntervalMapMs = 100;

/// Intervallo minimo fra due stati nelle altre schermate, in millisecondi.
const int stateIntervalOtherMs = 1000;

/// Tempo massimo della richiesta lunga dello stato: il server risponde entro 25 s.
const Duration stateRequestTimeout = Duration(seconds: 35);

/// Tempo massimo delle altre richieste: un comando aspetta l'esito del robot fino a 3 s.
const Duration requestTimeout = Duration(seconds: 10);

/// Attesa prima di riprovare quando il server non risponde.
const Duration retryDelay = Duration(seconds: 2);

/// Ogni quanto la schermata del telecomando manda il segno di vita.
const Duration remoteKeepaliveInterval = Duration(seconds: 10);

/// Ogni quanto si chiedono i punti nuovi della scia mentre il robot si muove.
const Duration trailRefreshInterval = Duration(seconds: 1);

/// Per quanto resta visibile la notifica in alto.
const Duration notificationDuration = Duration(seconds: 4);

/// Dopo un cambio di mappa il robot si riavvia: per quanto al massimo si mostra "riavvio in corso".
const Duration restartNoticeDuration = Duration(seconds: 40);

/// Voci del registro dei comandi chieste per pagina.
const int logPageSize = 50;

// --- joystick del telecomando: gli stessi massimi che il robot applica comunque (REMOTE_MAX_* in navigation.h) ---

/// Velocità in avanti con il joystick tutto su, in m/s.
const double joystickMaxV = 0.35;

/// Velocità all'indietro con il joystick tutto giù, in m/s.
const double joystickMaxVBack = 0.35;

/// Velocità di rotazione con il joystick tutto di lato, in rad/s.
const double joystickMaxOmega = 1.2;

/// Il joystick fermo in una posizione ripete lo stesso comando al massimo ogni tanto: sul robot un movimento vale 0,5 s
/// (REMOTE_MOVE_TIMEOUT), quindi il robot non si ferma. Un valore diverso parte appena arriva la risposta del precedente.
const Duration remoteMoveRepeatInterval = Duration(milliseconds: 200);

/// Sotto questa frazione della corsa il joystick vale zero.
const double joystickDeadZone = 0.12;

/// Passi in cui si arrotondano v e omega: un dito fermo non manda comandi sempre diversi.
const double joystickSpeedStep = 0.01;
