#ifndef CLOUD_LIVE_MAP_H
#define CLOUD_LIVE_MAP_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "network.h"

// Mappa dal vivo per l'app (docs/app.md, sezione 7.3), nel thread dello stato. Il thread tiene una copia privata della
// griglia fine a categorie: l'ultima confermata dal server. Ogni LIVE_MAP_SEND_INTERVAL legge dalla mappatura solo il
// rettangolo attorno alle posizioni non ancora mandate (copy_map_categories: la mappatura non aspetta mai), lo
// confronta con la copia privata e manda in RTM1 il rettangolo delle celle cambiate. A programma avviato non si legge
// mai tutta la mappa. Qui stanno anche l'invio della mappa salvata e le funzioni sul file map.rtm.

#define MAP_CRC_TEXT_LEN 9      // il CRC di una mappa come testo: 8 cifre esadecimali e il terminatore

// decodifica un file RTM1 (docs/formato-mappa.md) con tutti i suoi controlli. Se categories non è NULL (griglia a
// categorie grande MAP_CATEGORIES_BYTES, già azzerata) ci scrive le celle. Ritorna false se il file non è valido
bool decode_map_data(const uint8_t *data, size_t size, uint8_t *categories);

// il CRC-32 scritto in fondo a un file RTM1, come testo esadecimale. Ritorna false se il file non c'è o è troppo corto
bool read_map_file_crc(const char *path, char crc_text[MAP_CRC_TEXT_LEN]);

// controlla che il file sia una mappa RTM1 valida con il CRC atteso (quello mandato dal server con map_select)
bool check_map_file(const char *path, const char *expected_crc_text);

// cerca, dentro la finestra che parte da (window_x, window_y) larga window_width e alta window_height, il rettangolo più
// piccolo che contiene tutte le celle diverse fra le due griglie a categorie. Ritorna false se sono uguali
bool find_changed_rectangle(const uint8_t *old_categories, const uint8_t *new_categories, int window_x, int window_y,
                            int window_width, int window_height, int *x, int *y, int *width, int *height);

// copia un rettangolo di celle da una griglia a categorie a un'altra
void copy_category_rectangle(uint8_t *destination, const uint8_t *source, int x, int y, int width, int height);

// all'avvio: se il robot è partito dalla base e map.rtm è valida, la copia privata parte da quel file; altrimenti è
// vuota. Da qui la mappa salvata e la mappa dal vivo intera sono da mandare
void init_live_map(void);

// a ogni ciclo del thread dello stato: se la posizione è valida allarga il rettangolo delle posizioni non ancora mandate
void track_live_map_position(void);

// ogni LIVE_MAP_SEND_INTERVAL: manda quello che c'è da mandare (mappa salvata, mappa intera, differenze).
// Ritorna false se una richiesta non è arrivata al server (la rete è giù): si riprova più tardi
bool send_live_map(http_connection_t *connection);

// la versione della mappa dal vivo che il server dice di avere (nella risposta a ogni stato; negativa se non ne ha
// nessuna). Se non è quella della copia privata, per esempio dopo un riavvio del server, alla prossima send_live_map
// parte la mappa intera, anche se sul robot non è cambiato niente
void check_server_map_version(long server_version);

// il CRC del file map.rtm sul robot come testo, vuoto se il file non c'è (per lo stato: map.file_crc)
const char *read_live_map_file_crc(void);

#endif
