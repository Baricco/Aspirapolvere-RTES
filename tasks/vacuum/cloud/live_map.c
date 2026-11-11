#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "config_check.h"
#include "live_map.h"
#include "../aperiodic.h"
#include "../mapping.h"
#include "../movement.h"

#define HTTP_OK         200
#define HTTP_CONFLICT   409     // il server non ha la versione di partenza della differenza

// copia privata della griglia fine a categorie: l'ultima confermata dal server
static uint8_t confirmed_categories[MAP_CATEGORIES_BYTES];

// griglia in cui si legge dalla mappatura il rettangolo attorno al robot (fuori dal rettangolo contiene letture vecchie)
static uint8_t current_categories[MAP_CATEGORIES_BYTES];

// stato della mappa dal vivo: lo usa solo il thread dello stato
typedef struct {
    uint32_t version;                   // versione della copia privata, cresce a ogni differenza confermata dal server
    bool is_full_map_pending;           // true se va mandata la copia privata intera (all'avvio, o dopo un 409)
    const char *saved_map_pending;      // motivo con cui va mandato il file map.rtm (SAVED_REASON_*), NULL se non va mandato
    uint32_t last_save_count;           // ultimo read_map_save_count() visto: quando cambia c'è un file nuovo da mandare
    char file_crc_text[MAP_CRC_TEXT_LEN];   // CRC del file map.rtm, vuoto se il file non c'è

    // rettangolo, in celle fini, delle posizioni del robot non ancora mandate
    bool is_positions_empty;
    int positions_x_min, positions_y_min, positions_x_max, positions_y_max;
} live_map_state_t;

static live_map_state_t live_state;

// funzione helper che legge un uint16 little endian
static inline uint16_t read_u16(const uint8_t *data) {
    return (uint16_t)(data[0] | data[1] << 8);
}

// funzione helper che legge un uint32 little endian
static inline uint32_t read_u32(const uint8_t *data) {
    return (uint32_t)data[0] | (uint32_t)data[1] << 8 | (uint32_t)data[2] << 16 | (uint32_t)data[3] << 24;
}

// funzione helper che scrive la categoria della cella fine (x, y) in una griglia a categorie a 2 bit per cella
static inline void set_map_category(uint8_t *categories, int x, int y, uint8_t category) {
    uint32_t index = (uint32_t)y * FINE_MAP_CELL_NUMBER + (uint32_t)x;
    uint8_t shift = 2 * (index & 3);
    categories[index >> 2] = (uint8_t)((categories[index >> 2] & ~(3u << shift)) | (uint32_t)category << shift);
}

bool decode_map_data(const uint8_t *data, size_t size, uint8_t *categories) {

    // header: numero magico, rettangolo dentro la griglia, numero di zone
    const size_t header_len = MAP_FILE_HEADER_LEN - MAP_FILE_CRC_LEN;
    if (size < MAP_FILE_HEADER_LEN || memcmp(data, MAP_FILE_MAGIC, MAP_FILE_MAGIC_LEN) != 0) return false;

    int x = read_u16(data + 4), y = read_u16(data + 6), width = read_u16(data + 8), height = read_u16(data + 10);
    int num_zones = data[12];
    if (width == 0 || height == 0 || x + width > FINE_MAP_CELL_NUMBER || y + height > FINE_MAP_CELL_NUMBER) return false;
    if (num_zones > MAX_FORBIDDEN_ZONES - 1) return false;

    // zone: devono stare dentro la griglia
    size_t offset = header_len;
    const size_t end = size - MAP_FILE_CRC_LEN;
    if (offset + (size_t)num_zones * MAP_FILE_ZONE_LEN > end) return false;
    for (int i = 0; i < num_zones; i++, offset += MAP_FILE_ZONE_LEN) {
        if (read_u16(data + offset) + read_u16(data + offset + 4) > FINE_MAP_CELL_NUMBER ||
            read_u16(data + offset + 2) + read_u16(data + offset + 6) > FINE_MAP_CELL_NUMBER) return false;
    }

    // il CRC si controlla prima di scrivere le celle: un file rovinato non tocca la griglia
    if (read_u32(data + end) != map_crc32(0, data, end)) return false;

    // celle a sequenze, riga dopo riga: prima un giro di soli controlli, poi la scrittura
    for (int pass = 0; pass < 2; pass++) {
        const uint32_t total = (uint32_t)width * (uint32_t)height;
        uint32_t done = 0;
        size_t run_offset = offset;
        while (done < total) {
            if (run_offset >= end) return false;
            uint8_t run = data[run_offset++];
            uint32_t count = run & MAP_RUN_COUNT_MASK;
            uint8_t category = run >> MAP_RUN_CATEGORY_SHIFT;
            if (count == 0 || count > total - done) return false;
            for (uint32_t k = 0; pass == 1 && k < count; k++) {
                set_map_category(categories, x + (int)((done + k) % (uint32_t)width), y + (int)((done + k) / (uint32_t)width), category);
            }
            done += count;
        }

        // finite le celle devono restare solo i 4 byte del CRC
        if (run_offset != end) return false;
        if (categories == NULL) break;
    }

    return true;
}

// funzione helper che legge tutto un file in un buffer. Ritorna false se il file non c'è
static bool read_file(const char *path, byte_buffer_t *buffer) {

    FILE *f = fopen(path, "rb");
    if (!f) return false;

    uint8_t chunk[4096];
    size_t n;
    bool ok = true;
    while (ok && (n = fread(chunk, 1, sizeof chunk, f)) > 0) ok = append_bytes(buffer, chunk, n);

    fclose(f);
    return ok;
}

bool read_map_file_crc(const char *path, char crc_text[MAP_CRC_TEXT_LEN]) {

    crc_text[0] = '\0';

    FILE *f = fopen(path, "rb");
    if (!f) return false;

    uint8_t crc_bytes[MAP_FILE_CRC_LEN];
    bool ok = fseek(f, -MAP_FILE_CRC_LEN, SEEK_END) == 0 && fread(crc_bytes, 1, MAP_FILE_CRC_LEN, f) == MAP_FILE_CRC_LEN;
    fclose(f);

    if (ok) snprintf(crc_text, MAP_CRC_TEXT_LEN, "%08x", (unsigned)read_u32(crc_bytes));
    return ok;
}

bool check_map_file(const char *path, const char *expected_crc_text) {

    byte_buffer_t file = {0};
    char crc_text[MAP_CRC_TEXT_LEN];

    bool ok = read_file(path, &file) && decode_map_data(file.data, file.size, NULL) &&
              read_map_file_crc(path, crc_text) && strcmp(crc_text, expected_crc_text) == 0;

    free_bytes(&file);
    return ok;
}

bool find_changed_rectangle(const uint8_t *old_categories, const uint8_t *new_categories, int window_x, int window_y,
                            int window_width, int window_height, int *x, int *y, int *width, int *height) {

    int x_min = FINE_MAP_CELL_NUMBER, y_min = FINE_MAP_CELL_NUMBER, x_max = -1, y_max = -1;

    for (int cell_y = window_y; cell_y < window_y + window_height; cell_y++) {
        for (int cell_x = window_x; cell_x < window_x + window_width; cell_x++) {
            if (get_map_category(old_categories, cell_x, cell_y) == get_map_category(new_categories, cell_x, cell_y)) continue;
            if (cell_x < x_min) x_min = cell_x;
            if (cell_x > x_max) x_max = cell_x;
            if (cell_y < y_min) y_min = cell_y;
            if (cell_y > y_max) y_max = cell_y;
        }
    }

    // nessuna cella cambiata
    if (x_max < 0) return false;

    *x = x_min;
    *y = y_min;
    *width = x_max - x_min + 1;
    *height = y_max - y_min + 1;
    return true;
}

void copy_category_rectangle(uint8_t *destination, const uint8_t *source, int x, int y, int width, int height) {
    for (int cell_y = y; cell_y < y + height; cell_y++) {
        for (int cell_x = x; cell_x < x + width; cell_x++) {
            set_map_category(destination, cell_x, cell_y, get_map_category(source, cell_x, cell_y));
        }
    }
}

// funzione helper che rilegge il CRC del file map.rtm (dopo un salvataggio, o all'avvio)
static void refresh_file_crc(void) {
    char path[256];
    if (map_file_path(path, sizeof path, "") != 0 || !read_map_file_crc(path, live_state.file_crc_text)) live_state.file_crc_text[0] = '\0';
}

void check_server_map_version(long server_version) {
    if (server_version != (long)live_state.version) live_state.is_full_map_pending = true;
}

const char *read_live_map_file_crc(void) {
    return live_state.file_crc_text;
}

void init_live_map(void) {

    // il controllo della partenza dalla base dura circa un secondo dopo che il driver è pronto
    int64_t limit = rt_adesso() + STARTUP_BASE_WAIT;
    while (!read_started_on_base() && rt_adesso() < limit) rt_dormi_fino_a(rt_adesso() + STATE_CYCLE);

    memset(confirmed_categories, 0, sizeof confirmed_categories);

    // la mappatura ha caricato map.rtm solo se il robot è partito dalla base: la copia privata parte dallo stesso file
    if (read_started_on_base()) {
        char path[256];
        byte_buffer_t file = {0};
        if (map_file_path(path, sizeof path, "") == 0 && read_file(path, &file) && !decode_map_data(file.data, file.size, confirmed_categories)) {
            memset(confirmed_categories, 0, sizeof confirmed_categories);
        }
        free_bytes(&file);
    }

    live_state.version = 0;
    live_state.is_full_map_pending = true;
    live_state.saved_map_pending = SAVED_REASON_STARTUP;
    live_state.last_save_count = read_map_save_count();
    live_state.is_positions_empty = true;
    refresh_file_crc();
}

void track_live_map_position(void) {

    movement_position_data_t position = read_movement();
    if (!position.valid) return;

    uint32_t cell_index = coord_to_fine_index(position.x, position.y);
    int cell_x = cell_index & 0x3FF, cell_y = cell_index >> 10;

    if (live_state.is_positions_empty) {
        live_state.is_positions_empty = false;
        live_state.positions_x_min = live_state.positions_x_max = cell_x;
        live_state.positions_y_min = live_state.positions_y_max = cell_y;
        return;
    }

    if (cell_x < live_state.positions_x_min) live_state.positions_x_min = cell_x;
    if (cell_x > live_state.positions_x_max) live_state.positions_x_max = cell_x;
    if (cell_y < live_state.positions_y_min) live_state.positions_y_min = cell_y;
    if (cell_y > live_state.positions_y_max) live_state.positions_y_max = cell_y;
}

// funzione helper che manda il file map.rtm (corpo vuoto se il robot non ha mappa). Ritorna false se la rete è giù
static bool send_saved_map(http_connection_t *connection) {

    char path[256], request_path[128];
    byte_buffer_t file = {0};

    if (map_file_path(path, sizeof path, "") == 0) read_file(path, &file);

    snprintf(request_path, sizeof request_path, PATH_SAVED_MAP, live_state.saved_map_pending);
    long code = send_http_request(connection, "PUT", request_path, file.data, file.size, false, NULL, REQUEST_TIMEOUT_S);
    free_bytes(&file);

    if (code == HTTP_FAILED) return false;

    // se il server la rifiuta non serve rimandarla uguale: si aspetta il prossimo salvataggio
    if (code != HTTP_OK) fprintf(stderr, "rete: il server ha rifiutato la mappa salvata (%ld)\n", code);
    live_state.saved_map_pending = NULL;
    return true;
}

// funzione helper che manda la copia privata intera, dalla memoria. Ritorna false se la rete è giù
static bool send_full_map(http_connection_t *connection) {

    byte_buffer_t encoded = {0};
    char request_path[128];
    int x, y, width, height;

    // mappa vuota: una sola cella sconosciuta nell'origine
    if (!find_map_rectangle(confirmed_categories, &x, &y, &width, &height)) {
        x = y = FINE_MAP_ORIGIN;
        width = height = 1;
    }

    bool ok = encode_map_rectangle(confirmed_categories, x, y, width, height, NULL, 0, append_bytes, &encoded);
    long code = HTTP_FAILED;
    if (ok) {
        snprintf(request_path, sizeof request_path, PATH_LIVE_MAP, (unsigned)live_state.version);
        code = send_http_request(connection, "PUT", request_path, encoded.data, encoded.size, false, NULL, REQUEST_TIMEOUT_S);
    }
    free_bytes(&encoded);

    if (code == HTTP_OK) live_state.is_full_map_pending = false;
    return code != HTTP_FAILED;
}

// funzione helper che legge il rettangolo attorno alle posizioni non ancora mandate, lo confronta con la copia privata
// e manda le celle cambiate. Ritorna false se la rete è giù
static bool send_map_changes(http_connection_t *connection) {

    // il rettangolo delle posizioni allargato della portata del lidar più un margine: fuori da lì la mappatura non scrive
    const int reach = (int)(LIDAR_MAPPING_MAX_RANGE / FINE_MAP_CELL_SIZE) + LIVE_MAP_MARGIN_CELLS;
    int window_x = live_state.positions_x_min - reach, window_y = live_state.positions_y_min - reach;
    int window_x_max = live_state.positions_x_max + reach, window_y_max = live_state.positions_y_max + reach;
    if (window_x < 0) window_x = 0;
    if (window_y < 0) window_y = 0;
    if (window_x_max > FINE_MAP_CELL_NUMBER - 1) window_x_max = FINE_MAP_CELL_NUMBER - 1;
    if (window_y_max > FINE_MAP_CELL_NUMBER - 1) window_y_max = FINE_MAP_CELL_NUMBER - 1;
    int window_width = window_x_max - window_x + 1, window_height = window_y_max - window_y + 1;

    // chi legge rifà la copia se un job della mappatura ci è passato sopra: la mappatura non aspetta mai.
    // Se non ci riesce si riprova al prossimo giro, il rettangolo delle posizioni resta
    if (!copy_map_categories(current_categories, window_x, window_y, window_width, window_height, NULL, NULL)) return true;

    int x, y, width, height;
    if (!find_changed_rectangle(confirmed_categories, current_categories, window_x, window_y, window_width, window_height, &x, &y, &width, &height)) {
        live_state.is_positions_empty = true;
        return true;
    }

    byte_buffer_t encoded = {0};
    char request_path[128];
    long code = HTTP_FAILED;
    if (encode_map_rectangle(current_categories, x, y, width, height, NULL, 0, append_bytes, &encoded)) {
        snprintf(request_path, sizeof request_path, PATH_LIVE_MAP_PATCH, (unsigned)live_state.version, (unsigned)(live_state.version + 1));
        code = send_http_request(connection, "POST", request_path, encoded.data, encoded.size, false, NULL, REQUEST_TIMEOUT_S);
    }
    free_bytes(&encoded);

    if (code == HTTP_OK) {

        // conferma: la copia privata si aggiorna e il rettangolo delle posizioni si svuota
        copy_category_rectangle(confirmed_categories, current_categories, x, y, width, height);
        live_state.version++;
        live_state.is_positions_empty = true;

    } else if (code == HTTP_CONFLICT) {

        // il server non ha la versione di partenza (per esempio dopo un suo riavvio): si rimanda la copia privata
        // intera, dalla memoria, e al giro dopo le differenze in sospeso
        live_state.is_full_map_pending = true;
    }

    // invio fallito: il rettangolo resta e si allarga, le celle perse partono all'invio dopo
    return code != HTTP_FAILED;
}

bool send_live_map(http_connection_t *connection) {

    // un salvataggio nuovo: il file va mandato al server (se quello dell'avvio non è ancora partito resta lui: è
    // il server a decidere se il file è una mappa nuova)
    uint32_t save_count = read_map_save_count();
    if (save_count != live_state.last_save_count) {
        live_state.last_save_count = save_count;
        if (live_state.saved_map_pending == NULL) live_state.saved_map_pending = SAVED_REASON_SAVE;
        refresh_file_crc();
    }

    if (live_state.saved_map_pending != NULL && !send_saved_map(connection)) return false;

    if (live_state.is_full_map_pending) return send_full_map(connection);

    if (live_state.is_positions_empty) return true;

    return send_map_changes(connection);
}
