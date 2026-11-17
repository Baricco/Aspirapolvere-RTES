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

// --- rete Wi-Fi del robot (docs/wifi.md, sezione 10) ---

/// Nome della rete che il robot accende quando non trova la sua (WIFI_AP_SSID in tasks/vacuum/cloud/config.h).
const String robotWifiSsid = String.fromEnvironment('ROBOT_WIFI_SSID', defaultValue: 'Maurice-WiFi');

/// Password di quella rete: privata, solo in config.json (WIFI_AP_PASSWORD sul robot).
const String robotWifiPassword = String.fromEnvironment('ROBOT_WIFI_PASSWORD');

/// Indirizzo del robot sulla sua rete: lì risponde rtes-wifi, in HTTP semplice (la rete è protetta da WPA2).
const String robotLocalUrl = 'http://192.168.5.1';

/// Attesa massima della risposta del robot sulla sua rete quando si controlla se il telefono ci è collegato.
const Duration robotLocalProbeTimeout = Duration(seconds: 2);

/// Attesa massima di una scansione: dal backend fino a 10 s, sulla rete del robot con "Aggiorna" anche di più.
const Duration wifiScanTimeout = Duration(seconds: 40);

/// Dopo un cambio di rete, se il robot non torna online entro questo tempo l'app lo dice (docs/wifi.md, sezione 13).
const Duration wifiChangeWait = Duration(minutes: 3);

/// Soglie delle tacche del segnale, in dBm: da 4 tacche (sopra la prima) a 1.
const List<int> wifiSignalSteps = [-55, -67, -78];

/// La schermata del Wi-Fi ricontrolla ogni tanto con chi parlare (backend o robot sulla sua rete) e lo stato del robot.
const Duration wifiRouteCheckInterval = Duration(seconds: 3);

/// Tentativi di una ricarica della lista quando il robot non risponde (subito dopo un cambio di rete), e attesa fra due.
const int wifiScanAttempts = 4;
const Duration wifiScanRetryDelay = Duration(seconds: 5);
