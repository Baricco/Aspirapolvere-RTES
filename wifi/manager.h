#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

// Gestore dei modi del Wi-Fi (docs/wifi.md, sezioni 4-6): un thread solo, l'unico che tocca la radio. Gli altri
// thread (server locale e HTTP) leggono lo stato pubblicato e gli passano le richieste con le funzioni qui sotto,
// che aspettano la sua risposta.

#include "radio.h"
#include "wifi.h"

#define WIFI_REPLY_MESSAGE_MAX 160

// stato pubblicato dal gestore
typedef struct {
    wifi_mode_t mode;
    wifi_link_t link;
    unsigned change_number;                 // numero dell'ultimo cambio di rete (0 = nessuno)
    char change_ssid[WIFI_SSID_MAX + 1];
    wifi_change_state_t change_state;
    wifi_result_t change_result;
    int64_t change_time;                    // fine dell'ultimo cambio, in secondi dal 1970 (0 se non è finito)
} wifi_status_t;

// risposta a una richiesta
typedef struct {
    bool ok;
    int http_code;                          // 200, 400 parametri sbagliati, 409 rifiutata, 503 radio che non risponde
    const char *reason;                     // motivo del rifiuto, NULL se ok
    char message[WIFI_REPLY_MESSAGE_MAX];   // testo per l'utente, vuoto se ok
    unsigned change_number;                 // connect: numero del cambio accettato
    int network_count;                      // scan: reti in networks
    int64_t scanned_at;                     // scan: ora della scansione, in secondi dal 1970
} wifi_reply_t;

// gira nel thread principale finché non arriva manager_stop
void manager_run(const radio_t *radio);
void manager_stop(void);

// copia dello stato pubblicato
void manager_get_status(wifi_status_t *status);

// reti vicine (refresh: con la rete del robot accesa la spegne per cercare di nuovo). networks deve avere posto per
// WIFI_SCAN_MAX_NETWORKS reti
void manager_scan(bool is_refresh, wifi_network_t *networks, wifi_reply_t *reply);

// cambio di rete: controlla i parametri e il riposo, poi risponde subito; l'esito arriva nello stato
void manager_connect(const wifi_connect_request_t *request, wifi_reply_t *reply);

// il programma dei task dice se il robot è a riposo
void manager_set_resting(bool is_resting);

// true se la rete del robot è accesa (il server HTTP locale ascolta solo allora)
bool manager_is_ap(void);

#endif
