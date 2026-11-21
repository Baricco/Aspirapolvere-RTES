#ifndef WIFI_RADIO_H
#define WIFI_RADIO_H

// Le operazioni sul Wi-Fi vero che servono al gestore (manager.c). Due versioni con le stesse funzioni:
//   - radio_real.c: wpa_supplicant e udhcpc su wlan0, hostapd e dnsmasq su wlan1, sul robot;
//   - radio_sim.c:  reti e telefoni finti sul PC (--simulate), per provare regole, cambio e richieste senza il robot.
// Le chiama solo il thread del gestore: possono bloccare (al massimo per il tempo che ricevono).

#include <stddef.h>

#include "wifi.h"

typedef struct {
    // prepara la radio all'avvio (chiude i processi rimasti da un avvio precedente). Ritorna false se non si può
    bool (*init)(void);

    // avvia il collegamento alla rete salvata, che si cerca da sola finché non c'è; lo ferma e toglie l'indirizzo
    bool (*start_station)(void);
    void (*stop_station)(void);

    // accende e spegne la rete del robot
    bool (*start_ap)(void);
    void (*stop_ap)(void);

    // lavoro di ogni giro del gestore: eventi, processi figli chiusi, udhcpc avviato a collegamento fatto, canale
    // della rete del robot uguale a quello della rete che wlan0 usa o cerca
    void (*poll)(void);

    // collegamento di adesso. Ritorna true se wlan0 ha un indirizzo IPv4
    bool (*get_link)(wifi_link_t *link);

    // telefoni collegati alla rete del robot
    int (*get_ap_clients)(void);

    // rete salvata in ssid (vuoto se non c'è). Ritorna true se c'è
    bool (*get_saved_ssid)(char *ssid, size_t size);

    // scansione: riempie list (al massimo max reti, già filtrate e senza doppioni) e ritorna quante, oppure -1
    int (*scan)(wifi_network_t *list, int max, int64_t timeout_ms);

    // cambio di rete: aggiunge la rete nuova senza salvarla e la sceglie; aspetta il collegamento; poi la salva al
    // posto della vecchia (commit) oppure la toglie e torna alla vecchia (abort)
    bool (*try_network_begin)(const wifi_connect_request_t *request);
    wifi_assoc_t (*try_network_wait)(int64_t timeout_ms);
    void (*try_network_commit)(void);
    void (*try_network_abort)(void);

    // prova del backend (GET PATH_DEVICE_PING con il token) e ping a WIFI_PING_ADDRESS
    bool (*check_backend)(void);
    bool (*ping_internet)(void);
} radio_t;

extern const radio_t radio_real;
extern const radio_t radio_sim;

#endif
