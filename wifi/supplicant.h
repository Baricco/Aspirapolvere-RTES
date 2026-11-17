#ifndef WIFI_SUPPLICANT_H
#define WIFI_SUPPLICANT_H

// Socket di controllo di wpa_supplicant (lo stesso protocollo di wpa_cli): comandi di testo come "STATUS" o
// "ADD_NETWORK" su un socket Unix a datagrammi in WIFI_SUPPLICANT_CTRL_DIR/<interfaccia>. Due collegamenti:
//   - comandi: richiesta e risposta;
//   - eventi:  dopo "ATTACH" wpa_supplicant ci manda gli eventi ("<3>CTRL-EVENT-CONNECTED ...").

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct {
    int fd;                         // -1 se chiuso
    char local_path[108];           // socket nostro, da cancellare alla chiusura
} supplicant_link_t;

#define SUPPLICANT_LINK_CLOSED { .fd = -1 }

// apre il collegamento (is_events: anche ATTACH). Ritorna false se wpa_supplicant non c'è ancora
bool supplicant_open(supplicant_link_t *link, bool is_events);
void supplicant_close(supplicant_link_t *link);

// manda un comando e scrive la risposta in reply (terminata da '\0'). Ritorna false se non risponde
bool supplicant_command(supplicant_link_t *link, const char *command, char *reply, size_t size);

// come supplicant_command, ma ritorna true solo se la risposta è "OK"
bool supplicant_ok(supplicant_link_t *link, const char *command);

// legge un evento se arriva entro timeout_ms (0 = solo se c'è già). Ritorna false se non ne arriva nessuno
bool supplicant_event(supplicant_link_t *link, char *event, size_t size, int64_t timeout_ms);

// cerca "chiave=valore" nelle righe di una risposta (STATUS, SIGNAL_POLL) e copia il valore. Ritorna false se manca
bool supplicant_value(const char *reply, const char *key, char *value, size_t size);

#endif
