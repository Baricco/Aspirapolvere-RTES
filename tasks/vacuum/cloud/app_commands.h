#ifndef CLOUD_APP_COMMANDS_H
#define CLOUD_APP_COMMANDS_H

#include <stdbool.h>

// L'unico interprete dei comandi JSON dell'app (docs/app.md, sezione 9.4). Lo usano il thread dei comandi della rete e
// la FIFO: i due si alternano con un mutex fra loro soli, mai con i task. Non fa lavoro vero: traduce i comandi in
// chiamate alle funzioni di richiesta (navigazione, task aperiodico, driver) e ne aspetta l'esito.
//
// Un comando è un oggetto JSON con il campo "type" e i parametri accanto, per esempio:
//   {"type":"start"}   {"type":"remote_move","v":0.2,"omega":-0.5}   {"type":"zones","zones":[{"x":1,"y":1,"w":0.5,"h":0.5}]}

#define APP_COMMAND_MESSAGE_MAX 160     // lunghezza massima del testo dell'esito

// Esito di un comando: quello che torna all'app
typedef struct {
    bool ok;                                    // true = eseguito
    const char *reason;                         // motivo del rifiuto (per esempio "busy"), NULL se eseguito
    char message[APP_COMMAND_MESSAGE_MAX];      // testo per l'utente, vuoto se eseguito
    bool is_restart_needed;                     // true dopo un cambio di mappa: mandato l'esito, il programma va riavviato
} app_command_result_t;

// funzione che scarica la mappa map_id dal server nel file path. Ritorna true se ci riesce.
// La dà chi chiama l'interprete: il thread di rete passa la sua, la FIFO non ne ha (NULL)
typedef bool (*app_map_download_t)(int map_id, const char *path);

// esegue un comando JSON e ne scrive l'esito in result. Aspetta l'esito prima di tornare, così nessun comando ne
// sovrascrive un altro
void run_app_command(const char *json, app_map_download_t download_map, app_command_result_t *result);

#endif
