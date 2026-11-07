#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/scambio.h"

#include "aperiodic.h"
#include "mapping.h"

#define MAPS_DIR_DEFAULT    "/mnt/UDISK/rt/maps"    // cartella delle mappe sul robot (sul PC si sceglie con RTES_MAPS)
#define MAPS_DIR_ENV        "RTES_MAPS"             // variabile d'ambiente con la cartella delle mappe
#define ACTIVE_MAP_FILE     "attiva"                // file, nella cartella delle mappe, con il numero della mappa attiva
#define ACTIVE_MAP_DEFAULT  0                       // mappa attiva se il file non c'è
#define MAP_FILE_EXT        ".rtm"                  // estensione dei file delle mappe
#define MAP_TMP_EXT         ".tmp"                  // estensione aggiunta al file temporaneo, rinominato alla fine

// richiesta di salvataggio della mappa: la scrive solo la navigazione
typedef struct {
    uint32_t num;           // numero progressivo della richiesta, 0 = nessuna richiesta
} aperiodic_save_request_t;

// stato interno del task aperiodico
typedef struct {
    uint32_t last_save_num;                         // numero dell'ultima richiesta di salvataggio eseguita
    uint8_t categories[MAP_CATEGORIES_BYTES];       // copia della griglia fine a categorie, 2 bit per cella
} aperiodic_state_t;

static aperiodic_state_t aper_state;

// doppio buffer della richiesta di salvataggio e numero dell'ultima richiesta scritta (solo la navigazione lo usa)
static scambio_t save_request_buffer;
static aperiodic_save_request_t save_buffer1, save_buffer2;
static uint32_t save_request_num;

// true se l'ultimo salvataggio è fallito
static _Atomic bool save_failed;

// caratteristiche del task
rt_aperiodico_t aperiodic_task;

void init_aperiodic(void) {
    scambio_init(&save_request_buffer, &save_buffer1, &save_buffer2, sizeof(aperiodic_save_request_t));
    atomic_store(&save_failed, false);
}

void request_map_save(void) {

    // scrivi la richiesta con un numero nuovo, poi sveglia il task
    aperiodic_save_request_t request = { .num = ++save_request_num };
    scambio_scrivi(&save_request_buffer, &request);
    rt_aperiodico_sveglia(&aperiodic_task);
}

bool read_map_save_failed(void) {
    return atomic_load(&save_failed);
}

uint32_t map_crc32(uint32_t crc, const uint8_t *data, size_t size) {
    crc = ~crc;
    for (size_t i = 0; i < size; i++) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; bit++) crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1)));
    }
    return ~crc;
}

int map_file_path(char *path, size_t size, const char *suffix) {

    const char *dir = getenv(MAPS_DIR_ENV) ? getenv(MAPS_DIR_ENV) : MAPS_DIR_DEFAULT;

    // numero della mappa attiva: dal file, se c'è
    int active = ACTIVE_MAP_DEFAULT;
    char active_path[256];
    snprintf(active_path, sizeof active_path, "%s/%s", dir, ACTIVE_MAP_FILE);
    FILE *f = fopen(active_path, "r");
    if (f) {
        if (fscanf(f, "%d", &active) != 1 || active < 0) active = ACTIVE_MAP_DEFAULT;
        fclose(f);
    }

    int n = snprintf(path, size, "%s/%d%s%s", dir, active, MAP_FILE_EXT, suffix);
    return (n < 0 || (size_t)n >= size) ? -1 : 0;
}

// funzione helper che restituisce la categoria della cella fine (x, y) dalla copia a 2 bit
static inline uint8_t get_category(int x, int y) {
    uint32_t index = (uint32_t)y * FINE_MAP_CELL_NUMBER + (uint32_t)x;
    return (aper_state.categories[index >> 2] >> (2 * (index & 3))) & 3;
}

// funzione helper che scrive dei byte nel file e ne aggiorna il CRC. Ritorna false se la scrittura fallisce
static bool write_bytes(FILE *f, uint32_t *crc, const uint8_t *data, size_t size) {
    *crc = map_crc32(*crc, data, size);
    return fwrite(data, 1, size, f) == size;
}

// funzione helper che scrive un uint16 little endian
static bool write_u16(FILE *f, uint32_t *crc, uint16_t value) {
    uint8_t bytes[2] = { (uint8_t)value, (uint8_t)(value >> 8) };
    return write_bytes(f, crc, bytes, 2);
}

// funzione helper che porta una coordinata della zona (in metri) in celle fini
static inline int zone_coord_to_cells(double coord) {
    return (int)floor(coord / FINE_MAP_CELL_SIZE + 0.5) + FINE_MAP_ORIGIN;
}

// funzione helper che porta una dimensione della zona (in metri) in celle fini
static inline int zone_size_to_cells(double size) {
    return (int)floor(size / FINE_MAP_CELL_SIZE + 0.5);
}

// salva la mappa nel file della mappa attiva. Ritorna true se ci riesce
static bool save_map(void) {

    // copia la griglia a categorie, fra un job e l'altro della mappatura
    if (!copy_map_categories(aper_state.categories)) {
        fprintf(stderr, "aperiodico: mappa non salvata, la mappatura non lascia la finestra libera per la copia\n");
        return false;
    }

    // rettangolo delle celle viste (non sconosciute)
    int x_min = FINE_MAP_CELL_NUMBER, y_min = FINE_MAP_CELL_NUMBER, x_max = -1, y_max = -1;
    for (int y = 0; y < FINE_MAP_CELL_NUMBER; y++) {
        for (int x = 0; x < FINE_MAP_CELL_NUMBER; x++) {
            if (get_category(x, y) == MAP_CAT_UNKNOWN) continue;
            if (x < x_min) x_min = x;
            if (x > x_max) x_max = x;
            if (y < y_min) y_min = y;
            if (y > y_max) y_max = y;
        }
    }

    // mappa vuota: niente da salvare
    if (x_max < 0) {
        fprintf(stderr, "aperiodico: mappa non salvata, è vuota\n");
        return false;
    }

    // zone proibite dell'utente
    mapping_forbidden_zone_t zones[MAX_FORBIDDEN_ZONES];
    int num_zones = read_user_zones(zones);

    char path[256], tmp_path[256];
    if (map_file_path(path, sizeof path, "") != 0 || map_file_path(tmp_path, sizeof tmp_path, MAP_TMP_EXT) != 0) {
        fprintf(stderr, "aperiodico: mappa non salvata, percorso troppo lungo\n");
        return false;
    }

    FILE *f = fopen(tmp_path, "wb");
    if (!f) {
        fprintf(stderr, "aperiodico: mappa non salvata, non riesco a creare %s\n", tmp_path);
        return false;
    }

    uint32_t crc = 0;
    bool ok = write_bytes(f, &crc, (const uint8_t *)MAP_FILE_MAGIC, MAP_FILE_MAGIC_LEN);

    // rettangolo e numero di zone
    ok = ok && write_u16(f, &crc, (uint16_t)x_min) && write_u16(f, &crc, (uint16_t)y_min);
    ok = ok && write_u16(f, &crc, (uint16_t)(x_max - x_min + 1)) && write_u16(f, &crc, (uint16_t)(y_max - y_min + 1));
    uint8_t zones_byte = (uint8_t)num_zones;
    ok = ok && write_bytes(f, &crc, &zones_byte, 1);

    // zone, in celle fini
    for (int i = 0; ok && i < num_zones; i++) {
        ok = write_u16(f, &crc, (uint16_t)zone_coord_to_cells(zones[i].x)) &&
             write_u16(f, &crc, (uint16_t)zone_coord_to_cells(zones[i].y)) &&
             write_u16(f, &crc, (uint16_t)zone_size_to_cells(zones[i].w)) &&
             write_u16(f, &crc, (uint16_t)zone_size_to_cells(zones[i].h));
    }

    // celle a sequenze: riga dopo riga, una sequenza può continuare nella riga dopo, al massimo MAP_RUN_MAX celle
    uint8_t run_category = get_category(x_min, y_min);
    int run_count = 0;
    for (int y = y_min; ok && y <= y_max; y++) {
        for (int x = x_min; ok && x <= x_max; x++) {
            uint8_t category = get_category(x, y);
            if (category != run_category || run_count == MAP_RUN_MAX) {
                uint8_t run = (uint8_t)(run_category << MAP_RUN_CATEGORY_SHIFT | run_count);
                ok = write_bytes(f, &crc, &run, 1);
                run_category = category;
                run_count = 0;
            }
            run_count++;
        }
    }
    if (ok) {
        uint8_t run = (uint8_t)(run_category << MAP_RUN_CATEGORY_SHIFT | run_count);
        ok = write_bytes(f, &crc, &run, 1);
    }

    // CRC di tutto quello che precede, little endian (non entra nel CRC)
    uint8_t crc_bytes[MAP_FILE_CRC_LEN] = { (uint8_t)crc, (uint8_t)(crc >> 8), (uint8_t)(crc >> 16), (uint8_t)(crc >> 24) };
    ok = ok && fwrite(crc_bytes, 1, MAP_FILE_CRC_LEN, f) == MAP_FILE_CRC_LEN;

    // chiudi e rinomina sopra il vecchio: fino a questo momento il file vecchio resta intatto
    if (fclose(f) != 0) ok = false;
    if (!ok || rename(tmp_path, path) != 0) {
        remove(tmp_path);
        fprintf(stderr, "aperiodico: mappa non salvata, errore di scrittura su %s\n", tmp_path);
        return false;
    }

    fprintf(stderr, "aperiodico: mappa salvata in %s (%d x %d celle, %d zone)\n", path, x_max - x_min + 1, y_max - y_min + 1, num_zones);
    return true;
}

// job del task: esegue le richieste nuove
void job_aperiodic(void *arg) {

    (void)arg;

    aperiodic_save_request_t save_request = {0};
    scambio_leggi(&save_request_buffer, &save_request);

    if (save_request.num != aper_state.last_save_num) {
        aper_state.last_save_num = save_request.num;
        atomic_store(&save_failed, !save_map());
    }
}
