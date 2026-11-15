#ifndef TASK_APERIODIC_H
#define TASK_APERIODIC_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "rt/rt.h"
#include "mapping.h"

// Task aperiodico: sta da solo sul core 3, con lo scheduling normale.
// Esegue le richieste dei task e dell'app. Ogni richiesta passa da una funzione di richiesta qui sotto,
// che scrive il suo doppio buffer e sveglia il task: chi chiede fa una sola chiamata e non aspetta mai.

#define APERIODIC_CPU 3                 // core del task aperiodico

// caratteristiche del task aperiodico
extern rt_aperiodic_t aperiodic_task;

void init_aperiodic(void);
void job_aperiodic(void *arg);

// richiesta di salvataggio della mappa (la chiama la navigazione)
void request_map_save(void);

// richiesta di ricerca della posizione dopo un sollevamento (la chiama la mappatura, che ha già preparato i punti):
// restituisce il numero della richiesta, che torna nel risultato
uint32_t request_position_search(void);

// richiesta di nuove zone proibite dell'utente (la chiama l'interprete dei comandi dell'app): la lista intera voluta,
// senza la zona della base. Il task aperiodico la controlla, la passa alla mappatura e salva la mappa. Ritorna il numero
// della richiesta: quando read_applied_user_zones_version() lo raggiunge, la mappatura ha applicato le zone
uint32_t request_user_zones(const mapping_forbidden_zone_t *zones, int num_zones);

// Riposiziona base (docs/app.md, sezione 7.9): stato dell'operazione, letto dallo stato dell'app e dalla navigazione.
// Lo scrivono la navigazione (comando accettato), la mappatura (ricerca fallita) e il task aperiodico (file spostato o no)
typedef enum {
    REPOSITION_IDLE,        // nessun riposizionamento dall'avvio del programma
    REPOSITION_RUNNING,     // comando accettato: lidar acceso, ricerca della posizione, spostamento del file
    REPOSITION_DONE,        // file della mappa spostato: il programma si riavvia con la mappa nuova
    REPOSITION_NOT_FOUND,   // la ricerca non ha trovato una posizione abbastanza sicura: la mappa non cambia
    REPOSITION_ROTATED,     // la base è girata di un angolo lontano da un multiplo di 90 gradi: la mappa non cambia
    REPOSITION_FAILED,      // mappa vuota, nessuna scansione buona o errore sul file: la mappa non cambia
} reposition_status_t;

#define REPOSITION_RESTART_DELAY    RT_S(10)    // Riposiziona base riuscito: il programma si riavvia appena il server ha l'esito, o comunque dopo questo tempo

void set_reposition_status(reposition_status_t status);
reposition_status_t read_reposition_status(void);

// richiesta di spostare il file della mappa (la chiama la mappatura, dopo la ricerca): ogni cella e ogni zona si sposta
// di (dx, dy) celle fini e poi ruota di quarter_turns quarti di giro in senso antiorario attorno all'origine. Il task
// aperiodico riscrive map.rtm, smette di salvare e scrive REPOSITION_DONE (o REPOSITION_FAILED)
uint32_t request_map_reposition(int dx, int dy, int quarter_turns);

// richieste sul file della mappa (le chiama l'interprete dei comandi dell'app, con il robot a riposo sulla base):
// sostituire map.rtm con il file scaricato dal server (map.rtm + MAP_DOWNLOAD_EXT) oppure cancellarlo. Da quel momento
// il task aperiodico non salva più: il programma va riavviato. Ritornano il numero della richiesta
uint32_t request_map_replace(void);
uint32_t request_map_delete(void);

// esito di una richiesta sul file della mappa: ritorna false finché il task aperiodico non l'ha eseguita, poi true con l'esito in ok
bool read_map_file_request_result(uint32_t request_num, bool *ok);

// true se l'ultimo salvataggio della mappa è fallito (lo legge l'app per avvisare l'utente)
bool read_map_save_failed(void);

// numero di salvataggi della mappa riusciti: quando cambia, il thread dello stato dell'app manda il file al server
uint32_t read_map_save_count(void);

#define MAP_DOWNLOAD_EXT    ".download"     // estensione aggiunta a map.rtm per il file scaricato dal server, prima della sostituzione

// percorso del file della mappa (map.rtm nella cartella delle mappe) con in fondo suffix (per esempio ".tmp" per il
// file temporaneo, "" per quello vero). Ritorna 0, oppure -1 se il percorso è troppo lungo
int map_file_path(char *path, size_t size, const char *suffix);

// funzione helper che restituisce la categoria della cella fine (x, y) da una griglia a categorie a 2 bit per cella (quella di copy_map_categories)
static inline uint8_t get_map_category(const uint8_t *categories, int x, int y) {
    uint32_t index = (uint32_t)y * FINE_MAP_CELL_NUMBER + (uint32_t)x;
    return (categories[index >> 2] >> (2 * (index & 3))) & 3;
}

// chi riceve i byte della codifica RTM1, un pezzo alla volta (un file, un buffer in memoria). Ritorna false se non riesce
typedef bool (*map_write_function_t)(void *context, const uint8_t *data, size_t size);

// codifica in RTM1 (docs/formato-mappa.md) il rettangolo di celle fini che parte da (x, y), largo width e alto height,
// di una griglia a categorie, con le zone proibite date (in metri) e il CRC in fondo. La usano il salvataggio della
// mappa e la mappa dal vivo dell'app. Ritorna false se chi scrive non riesce
bool encode_map_rectangle(const uint8_t *categories, int x, int y, int width, int height,
                          const mapping_forbidden_zone_t *zones, int num_zones, map_write_function_t write, void *context);

// il rettangolo più piccolo che contiene tutte le celle non sconosciute di una griglia a categorie.
// Ritorna false se la griglia è vuota
bool find_map_rectangle(const uint8_t *categories, int *x, int *y, int *width, int *height);

// CRC-32 standard: si parte da crc = 0, si chiama con i byte uno dopo l'altro, il risultato è il CRC
uint32_t map_crc32(uint32_t crc, const uint8_t *data, size_t size);

#endif
