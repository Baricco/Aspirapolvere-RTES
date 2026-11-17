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
#define COMMANDS_PATH_ENV           "RTES_COMMANDS"             // variabile d'ambiente con il percorso del file speciale dei comandi (solo per le prove: senza, niente file e niente thread)
#define COMMANDS_FIFO_MODE          0600                        // permessi del file speciale: legge e scrive solo il proprietario
#define COMMAND_LINE_MAX            2048                        // lunghezza massima di una riga di comando

// --- Wi-Fi del robot: rtes-wifi (docs/wifi.md) ---
#define PATH_DEVICE_PING            "/api/device/ping"              // prova del backend dopo un cambio di rete
#define WIFI_AP_SSID                "Maurice-WiFi"                  // rete del robot
#define WIFI_AP_PASSWORD            "<password>"                  // password della rete del robot (WPA2, 8-63 caratteri)
#define WIFI_AP_ADDRESS             "192.168.5.1"                   // indirizzo del robot sulla sua rete
#define WIFI_AP_NETMASK             "255.255.255.0"
#define WIFI_AP_DHCP_FIRST          "192.168.5.10"                  // indirizzi dati ai telefoni
#define WIFI_AP_DHCP_LAST           "192.168.5.50"
#define WIFI_AP_CHANNEL             6
#define WIFI_HTTP_PORT              80                              // server HTTP locale, solo con la rete del robot accesa
#define WIFI_INTERFACE              "wlan0"
#define WIFI_SUPPLICANT_CONF        "/etc/wifi/wpa_supplicant.conf" // la rete salvata
#define WIFI_SUPPLICANT_CTRL_DIR    "/etc/wifi/sockets"             // socket di controllo di wpa_supplicant
#define WIFI_RUN_DIR                "/tmp/rtes-wifi"                // file di lavoro di rtes-wifi (configurazioni generate, socket)
#define WIFI_SOCKET_PATH            "/tmp/rtes-wifi.sock"           // socket fra il programma dei task e rtes-wifi
#define WIFI_SOCKET_ENV             "RTES_WIFI_SOCKET"              // sul PC: variabile d'ambiente con il percorso del socket
#define WIFI_SUPPLICANT_BIN         "/usr/sbin/wpa_supplicant"
#define WIFI_UDHCPC_BIN             "/sbin/udhcpc"
#define WIFI_HOSTAPD_BIN            "/usr/sbin/hostapd"
#define WIFI_DNSMASQ_BIN            "/usr/sbin/dnsmasq"
#define WIFI_IFCONFIG_BIN           "/sbin/ifconfig"

// tempi dei modi (docs/wifi.md, sezione 4)
#define WIFI_CONNECT_TIMEOUT_S      120     // all'avvio (e dopo un cambio fallito) senza indirizzo per tanto: rete del robot
#define WIFI_LOST_TIMEOUT_S         120     // rete persa durante il funzionamento per tanto: rete del robot
#define WIFI_AP_DURATION_S          300     // rete del robot accesa per tanto, poi si riprova la rete salvata
#define WIFI_AP_RETRY_S             60      // tentativo sulla rete salvata fra due periodi della rete del robot
#define WIFI_AP_SCAN_S              5       // con la rete del robot accesa, "Aggiorna" la spegne al massimo per tanto per cercare le reti

// tempi del cambio di rete e delle scansioni (docs/wifi.md, sezioni 5 e 6)
#define WIFI_SCAN_TIMEOUT_S         8       // attesa massima dei risultati di una scansione
#define WIFI_ASSOC_TIMEOUT_S        30      // attesa massima del collegamento alla rete nuova
#define WIFI_DHCP_TIMEOUT_S         20      // attesa massima dell'indirizzo dalla rete nuova
#define WIFI_BACKEND_TRIES          2       // tentativi della prova del backend
#define WIFI_BACKEND_TIMEOUT_S      10      // tempo massimo di ogni tentativo
#define WIFI_PING_ADDRESS           "8.8.8.8"
#define WIFI_PING_TRIES             3
#define WIFI_PING_TIMEOUT_S         2
#define WIFI_SCAN_MAX_NETWORKS      32      // reti al massimo nella lista
#define WIFI_RESTING_STALE_S        30      // senza notizie dal programma dei task da tanto, il robot conta come a riposo
#define WIFI_SCAN_REPLY_WAIT_S      30      // attesa massima della risposta a una scansione dentro rtes-wifi
#define WIFI_CONNECT_REPLY_WAIT_S   5       // attesa massima della risposta (accettato o no) a un cambio

// lato del programma dei task (cloud/wifi_link.c, docs/wifi.md sezione 7)
#define WIFI_SCAN_COMMAND_TIMEOUT_MS    9000            // attesa massima di una scansione (il backend aspetta l'esito 10 s)
#define WIFI_CONNECT_COMMAND_TIMEOUT_MS 6000            // attesa massima della risposta a un cambio (accettato o no)
#define WIFI_STATUS_INTERVAL            RT_S(2)         // ogni quanto il thread dello stato chiede lo stato del Wi-Fi
#define WIFI_STATUS_TIMEOUT_MS          300             // attesa massima della risposta allo stato del Wi-Fi
#define WIFI_RESTING_INTERVAL           RT_S(10)        // il riposo si manda a rtes-wifi a ogni cambio e almeno ogni tanto

#endif
