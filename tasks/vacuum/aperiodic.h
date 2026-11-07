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
extern rt_aperiodico_t aperiodic_task;

void init_aperiodic(void);
void job_aperiodic(void *arg);

// richiesta di salvataggio della mappa (la chiama la navigazione)
void request_map_save(void);

// richiesta di ricerca della posizione dopo un sollevamento (la chiama la mappatura, che ha già preparato i punti):
// restituisce il numero della richiesta, che torna nel risultato
uint32_t request_position_search(void);

// richiesta di nuove zone proibite dell'utente (la chiama il thread dei comandi, più avanti quello dell'app): la lista
// intera voluta, senza la zona della base. Il task aperiodico la controlla, la passa alla mappatura e salva la mappa
void request_user_zones(const mapping_forbidden_zone_t *zones, int num_zones);

// true se l'ultimo salvataggio della mappa è fallito (lo leggerà l'app per avvisare l'utente)
bool read_map_save_failed(void);

// percorso del file della mappa attiva (cartella delle mappe, numero letto dal file della mappa attiva) con in fondo
// suffix (per esempio ".tmp" per il file temporaneo, "" per quello vero). Ritorna 0, oppure -1 se il percorso non esiste
int map_file_path(char *path, size_t size, const char *suffix);

// CRC-32 standard: si parte da crc = 0, si chiama con i byte uno dopo l'altro, il risultato è il CRC
uint32_t map_crc32(uint32_t crc, const uint8_t *data, size_t size);

#endif
