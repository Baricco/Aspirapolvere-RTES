#ifndef WIFI_CHECK_H
#define WIFI_CHECK_H

// Prove del collegamento dopo un cambio di rete (docs/wifi.md, sezione 5): il backend dell'app e internet.

#include <stdbool.h>

// legge indirizzo del server, token e certificato come il programma dei task (cloud/config.h, sezione 9.2 di
// docs/app.md). Ritorna false se manca il token: senza, la prova del backend fallisce sempre
bool init_backend_check(void);

// GET PATH_DEVICE_PING con il token, in HTTPS con le radici di server_roots.h. Ritorna true se risponde 200
bool check_backend(void);

// un ping (ICMP echo) a WIFI_PING_ADDRESS. Ritorna true se arriva la risposta entro WIFI_PING_TIMEOUT_S. Serve root
bool ping_internet(void);

#endif
