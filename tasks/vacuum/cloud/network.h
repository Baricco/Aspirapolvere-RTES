#ifndef CLOUD_NETWORK_H
#define CLOUD_NETWORK_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Collegamento del robot al server dell'app (docs/app.md, sezione 9.3): due thread normali (SCHED_OTHER) sul core 3,
// quello del task aperiodico, con nice più alto così il task aperiodico ha la precedenza. Il robot non riceve mai
// connessioni: è lui a collegarsi al server, solo in HTTPS e verificandone il certificato.
//   - thread dei comandi: richiesta lunga dei comandi, li passa all'interprete e manda l'esito di ognuno;
//   - thread dello stato: stato del robot, mappa dal vivo, mappa salvata.
// Non condividono nessun mutex con i task: leggono con le funzioni read_* e scrivono con le funzioni di richiesta.

#define RESTART_EXIT_CODE 3     // codice di uscita del programma quando chiede di essere rilanciato (cambio di mappa)

// legge il token e avvia i due thread. Ritorna 0, oppure -1 se la rete non parte (per esempio senza il file del token):
// il programma va avanti senza app, come prima
int start_network(void);

// chiede il riavvio del programma (dopo un cambio di mappa): il main ferma i task ed esce con RESTART_EXIT_CODE
void request_restart(void);
bool is_restart_requested(void);

// un buffer di byte che cresce: il corpo di una risposta, o una mappa codificata da mandare
typedef struct {
    uint8_t *data;
    size_t size;
    size_t capacity;
} byte_buffer_t;

// aggiunge dei byte al buffer (ha la forma di map_write_function_t: context è un byte_buffer_t). Ritorna false se la memoria non basta
bool append_bytes(void *context, const uint8_t *data, size_t size);
void free_bytes(byte_buffer_t *buffer);

// una connessione al server, tenuta aperta fra una richiesta e l'altra. Ogni thread ha la sua
typedef struct http_connection http_connection_t;

#define HTTP_FAILED (-1)        // la richiesta non è arrivata al server, o la risposta non è tornata

// manda una richiesta al server e ritorna il codice HTTP della risposta, oppure HTTP_FAILED. body può essere NULL;
// is_json dice se il corpo è JSON (altrimenti sono byte, per le mappe); response, se non è NULL, riceve il corpo della risposta
long send_http_request(http_connection_t *connection, const char *method, const char *path, const uint8_t *body,
                       size_t body_size, bool is_json, byte_buffer_t *response, long timeout_s);

#endif
