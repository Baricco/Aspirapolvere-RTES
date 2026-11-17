#ifndef WIFI_SERVER_H
#define WIFI_SERVER_H

// Le due porte di rtes-wifi (docs/wifi.md, sezione 7):
//   - socket Unix per il programma dei task: una richiesta JSON per riga, una risposta per riga;
//   - HTTP sulla rete del robot (solo quando è accesa): le stesse richieste del backend, per l'app.
// Un thread ascolta tutte e due; ogni connessione ha un suo thread breve, così una scansione lunga non ferma lo stato.

#include <stdbool.h>

// avvia il thread. http_address e http_port: dove ascolta il server HTTP con la rete del robot accesa
bool start_server(const char *socket_path, const char *http_address, int http_port);

#endif
