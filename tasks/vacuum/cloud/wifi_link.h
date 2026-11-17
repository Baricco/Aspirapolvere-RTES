#ifndef CLOUD_WIFI_LINK_H
#define CLOUD_WIFI_LINK_H

#include <stdbool.h>

// Collegamento con rtes-wifi, il programma che gestisce il Wi-Fi del robot (docs/wifi.md, sezione 7.1): un socket
// Unix con una richiesta JSON per riga e una risposta per riga. Lo usano il thread dei comandi (wifi_scan,
// wifi_connect) e il thread dello stato (stato del Wi-Fi, riposo). Mai i task.

// manda una richiesta (oggetto JSON su una riga, senza a capo) e aspetta la risposta per al massimo timeout_ms.
// Ritorna la risposta (da liberare con free), oppure NULL se rtes-wifi non risponde
char *wifi_request(const char *request, int timeout_ms);

#endif
