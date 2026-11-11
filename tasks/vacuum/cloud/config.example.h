#ifndef CLOUD_CONFIG_H
#define CLOUD_CONFIG_H

// Tutta la configurazione dell'interfaccia robot-server dell'app (docs/app.md, sezione 9.2), con valori finti.
// Questo file è l'esempio versionato: va copiato in tasks/vacuum/cloud/config.h (che git ignora) mettendo i valori veri.
// config.h non va mai versionato né pubblicato.

// --- Indirizzo del server e file del token ---
#define SERVER_URL              "https://<server>"              // indirizzo del server, senza la barra in fondo. Solo HTTPS
#define SERVER_URL_ENV          "RTES_SERVER_URL"               // sul PC (simulatore): variabile d'ambiente con l'indirizzo del server
#define SERVER_CA_ENV           "RTES_SERVER_CA"                // sul PC: variabile d'ambiente con il certificato autofirmato del server di prova, al posto delle radici di server_roots.h
#define TOKEN_FILE              "/mnt/UDISK/rt/robot-token"     // file con il token del robot (permessi 0600), mai nel progetto
#define TOKEN_FILE_ENV          "RTES_TOKEN_FILE"               // sul PC: variabile d'ambiente con il file del token
#define TOKEN_MAX_LENGTH        128                             // lunghezza massima del token, in caratteri

// --- Richieste del robot (docs/app.md, sezione 8.6) ---
#define PATH_COMMANDS           "/api/device/commands"
#define PATH_COMMAND_RESULT     "/api/device/commands/%u/result"
#define PATH_STATE              "/api/device/state"
#define PATH_LIVE_MAP           "/api/device/map?version=%u"
#define PATH_LIVE_MAP_PATCH     "/api/device/map/patch?base_version=%u&version=%u"
#define PATH_SAVED_MAP          "/api/device/map/saved?reason=%s"
#define PATH_MAP_DOWNLOAD       "/api/device/maps/%d"
#define SAVED_REASON_STARTUP    "startup"                       // la mappa salvata che il robot ha all'avvio
#define SAVED_REASON_SAVE       "save"                          // la mappa appena salvata

// --- Tempi delle richieste ---
#define COMMANDS_REQUEST_TIMEOUT_S  40                          // tempo massimo della richiesta lunga dei comandi (il server risponde entro 25 s)
#define REQUEST_TIMEOUT_S           10                          // tempo massimo delle altre richieste
#define CONNECT_TIMEOUT_S           10                          // tempo massimo per collegarsi al server
#define RECONNECT_DELAYS_S          { 1, 2, 5, 10 }             // attese fra un tentativo e l'altro quando la rete cade, espresse in secondi: l'ultima si ripete

// --- Thread dello stato (docs/app.md, sezioni 4.2 e 7.3) ---
#define STATE_CYCLE                 RT_MS(100)                  // ciclo del thread dello stato
#define STATE_IDLE_INTERVAL         RT_S(10)                    // a riposo lo stato parte ogni tanto anche se non cambia niente
#define LIVE_MAP_SEND_INTERVAL      RT_MS(500)                  // ogni quanto partono le differenze della mappa dal vivo
#define LIVE_MAP_MARGIN_CELLS       4                           // celle fini di margine attorno al rettangolo letto per la mappa dal vivo
#define STARTUP_BASE_WAIT           RT_S(3)                     // all'avvio, attesa massima per sapere se il robot è partito dalla base

// --- Interprete dei comandi ---
#define COMMAND_RESULT_TIMEOUT      RT_S(1)                     // attesa massima dell'esito di un comando dalla navigazione (un job ogni 100 ms)
#define COMMAND_RESULT_POLL         RT_MS(10)                   // attesa fra due controlli dell'esito
#define ZONES_RESULT_TIMEOUT        RT_S(4)                     // attesa massima delle zone applicate dalla mappatura
#define MAP_FILE_RESULT_TIMEOUT     RT_S(5)                     // attesa massima della sostituzione o cancellazione del file della mappa
#define MAP_DOWNLOAD_MAX_BYTES      (2 * 1024 * 1024)           // dimensione massima di una mappa scaricata

// --- Thread ---
#define NETWORK_NICE                5                           // nice dei thread di rete: il task aperiodico, sullo stesso core, ha la precedenza
#define NETWORK_STACK               (256 * 1024)                // stack dei thread di rete: con mlockall ogni byte di stack resta in RAM
#define COMMANDS_FIFO_STACK         (128 * 1024)                // stack del thread che legge la FIFO dei comandi

// --- FIFO dei comandi ---
#define COMMANDS_PATH_DEFAULT       "/tmp/rtes/vacuum-commands" // file speciale dei comandi sul robot: ogni riga è un comando JSON
#define COMMANDS_PATH_ENV           "RTES_COMMANDS"             // sul PC: variabile d'ambiente con il percorso del file dei comandi
#define COMMAND_LINE_MAX            2048                        // lunghezza massima di una riga di comando

#endif
