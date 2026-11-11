#include <errno.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/scambio.h"

#include "aperiodic.h"
#include "mapping.h"

#define MAPS_DIR_DEFAULT    "/mnt/UDISK/rt/maps"    // cartella delle mappe sul robot (sul PC si sceglie con RTES_MAPS)
#define MAPS_DIR_ENV        "RTES_MAPS"             // variabile d'ambiente con la cartella delle mappe
#define MAP_FILE_NAME       "map.rtm"               // l'unica mappa sul robot: quella selezionata (le altre stanno sul server dell'app)
#define MAP_TMP_EXT         ".tmp"                  // estensione aggiunta al file temporaneo, rinominato alla fine
#define MAP_ENCODE_CHUNK    256                     // la codifica passa i byte a chi li scrive a pezzi di questa dimensione
#define ZONES_APPLY_TIMEOUT RT_S(3)                 // tempo massimo per vedere applicate le zone nuove dalla mappatura (un job ogni 200 ms)
#define ZONES_APPLY_WAIT    RT_MS(20)               // attesa fra due controlli

// richiesta di salvataggio della mappa: la scrive solo la navigazione
typedef struct {
    uint32_t num;           // numero progressivo della richiesta, 0 = nessuna richiesta
} aperiodic_save_request_t;

// stato interno del task aperiodico
typedef struct {
    uint32_t last_save_num;                         // numero dell'ultima richiesta di salvataggio eseguita
    uint32_t last_search_num;                       // numero dell'ultima richiesta di ricerca della posizione eseguita
    uint32_t last_zones_num;                        // numero dell'ultima richiesta di zone proibite eseguita
    uint32_t last_file_num;                         // numero dell'ultima richiesta sul file della mappa (sostituzione o cancellazione) eseguita
    bool is_saving_disabled;                        // true dopo una sostituzione o una cancellazione del file: da lì non si salva più, il programma sta per riavviarsi
    uint8_t categories[MAP_CATEGORIES_BYTES];       // copia della griglia fine a categorie, 2 bit per cella
} aperiodic_state_t;

static aperiodic_state_t aper_state;

// doppio buffer della richiesta di salvataggio e numero dell'ultima richiesta scritta (solo la navigazione lo usa)
static scambio_t save_request_buffer;
static aperiodic_save_request_t save_buffer1, save_buffer2;
static uint32_t save_request_num;

// richiesta di ricerca della posizione: la scrive solo la mappatura
typedef struct {
    uint32_t num;           // numero progressivo della richiesta, 0 = nessuna richiesta
} aperiodic_search_request_t;

static scambio_t search_request_buffer;
static aperiodic_search_request_t search_buffer1, search_buffer2;
static uint32_t search_request_num;

// richiesta di zone proibite: la scrive solo il thread dei comandi
typedef struct {
    uint32_t num;                                           // numero progressivo della richiesta, 0 = nessuna richiesta
    int num_zones;
    mapping_forbidden_zone_t zones[MAX_FORBIDDEN_ZONES - 1];
} aperiodic_zones_request_t;

static scambio_t zones_request_buffer;
static aperiodic_zones_request_t zones_buffer1, zones_buffer2;
static uint32_t zones_request_num;

// richiesta sul file della mappa: sostituirlo con quello scaricato dal server, oppure cancellarlo. La scrive solo
// l'interprete dei comandi dell'app
typedef struct {
    uint32_t num;           // numero progressivo della richiesta, 0 = nessuna richiesta
    bool is_delete;         // true = cancella map.rtm, false = rinomina il file scaricato sopra map.rtm
} aperiodic_file_request_t;

static scambio_t file_request_buffer;
static aperiodic_file_request_t file_buffer1, file_buffer2;
static uint32_t file_request_num;

// esito dell'ultima richiesta sul file eseguita: prima l'esito, poi il numero (chi aspetta guarda il numero)
static _Atomic bool file_request_ok;
static _Atomic uint32_t file_request_done_num;

// true se l'ultimo salvataggio è fallito
static _Atomic bool save_failed;

// numero di salvataggi riusciti: quando cambia, il thread dello stato dell'app manda il file al server
static _Atomic uint32_t save_count;

// caratteristiche del task
rt_aperiodico_t aperiodic_task;

void init_aperiodic(void) {
    scambio_init(&save_request_buffer, &save_buffer1, &save_buffer2, sizeof(aperiodic_save_request_t));
    scambio_init(&search_request_buffer, &search_buffer1, &search_buffer2, sizeof(aperiodic_search_request_t));
    scambio_init(&zones_request_buffer, &zones_buffer1, &zones_buffer2, sizeof(aperiodic_zones_request_t));
    scambio_init(&file_request_buffer, &file_buffer1, &file_buffer2, sizeof(aperiodic_file_request_t));
    atomic_store(&save_failed, false);
}

void request_map_save(void) {

    // scrivi la richiesta con un numero nuovo, poi sveglia il task
    aperiodic_save_request_t request = { .num = ++save_request_num };
    scambio_scrivi(&save_request_buffer, &request);
    rt_aperiodico_sveglia(&aperiodic_task);
}

uint32_t request_position_search(void) {

    // scrivi la richiesta con un numero nuovo, poi sveglia il task
    aperiodic_search_request_t request = { .num = ++search_request_num };
    scambio_scrivi(&search_request_buffer, &request);
    rt_aperiodico_sveglia(&aperiodic_task);
    return request.num;
}

uint32_t request_user_zones(const mapping_forbidden_zone_t *zones, int num_zones) {

    // scrivi la richiesta con un numero nuovo, poi sveglia il task
    aperiodic_zones_request_t request = { .num = ++zones_request_num, .num_zones = num_zones };
    for (int i = 0; i < num_zones && i < MAX_FORBIDDEN_ZONES - 1; i++) request.zones[i] = zones[i];
    scambio_scrivi(&zones_request_buffer, &request);
    rt_aperiodico_sveglia(&aperiodic_task);
    return request.num;
}

// funzione helper comune alle due richieste sul file della mappa
static uint32_t request_map_file_change(bool is_delete) {

    // scrivi la richiesta con un numero nuovo, poi sveglia il task
    aperiodic_file_request_t request = { .num = ++file_request_num, .is_delete = is_delete };
    scambio_scrivi(&file_request_buffer, &request);
    rt_aperiodico_sveglia(&aperiodic_task);
    return request.num;
}

uint32_t request_map_replace(void) {
    return request_map_file_change(false);
}

uint32_t request_map_delete(void) {
    return request_map_file_change(true);
}

bool read_map_file_request_result(uint32_t request_num, bool *ok) {
    if (atomic_load(&file_request_done_num) != request_num) return false;
    *ok = atomic_load(&file_request_ok);
    return true;
}

bool read_map_save_failed(void) {
    return atomic_load(&save_failed);
}

uint32_t read_map_save_count(void) {
    return atomic_load(&save_count);
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

    int n = snprintf(path, size, "%s/%s%s", dir, MAP_FILE_NAME, suffix);
    return (n < 0 || (size_t)n >= size) ? -1 : 0;
}

// funzione helper che porta una coordinata della zona (in metri) in celle fini
static inline int zone_coord_to_cells(double coord) {
    return (int)floor(coord / FINE_MAP_CELL_SIZE + 0.5) + FINE_MAP_ORIGIN;
}

// funzione helper che porta una dimensione della zona (in metri) in celle fini
static inline int zone_size_to_cells(double size) {
    return (int)floor(size / FINE_MAP_CELL_SIZE + 0.5);
}

// stato della codifica: i byte pronti si accumulano in un pezzo, che passa a chi scrive quando è pieno
typedef struct {
    map_write_function_t write;
    void *context;
    uint32_t crc;
    uint8_t chunk[MAP_ENCODE_CHUNK];
    size_t chunk_size;
    bool ok;
} map_encoder_t;

// funzione helper che passa a chi scrive il pezzo accumulato
static void encoder_flush(map_encoder_t *encoder) {
    if (encoder->ok && encoder->chunk_size > 0) encoder->ok = encoder->write(encoder->context, encoder->chunk, encoder->chunk_size);
    encoder->chunk_size = 0;
}

// funzione helper che aggiunge un byte alla codifica e ne aggiorna il CRC
static void encoder_put(map_encoder_t *encoder, uint8_t byte) {
    encoder->crc = map_crc32(encoder->crc, &byte, 1);
    encoder->chunk[encoder->chunk_size++] = byte;
    if (encoder->chunk_size == MAP_ENCODE_CHUNK) encoder_flush(encoder);
}

// funzione helper che aggiunge un uint16 little endian
static void encoder_put_u16(map_encoder_t *encoder, uint16_t value) {
    encoder_put(encoder, (uint8_t)value);
    encoder_put(encoder, (uint8_t)(value >> 8));
}

bool encode_map_rectangle(const uint8_t *categories, int x, int y, int width, int height,
                          const mapping_forbidden_zone_t *zones, int num_zones, map_write_function_t write, void *context) {

    map_encoder_t encoder = { .write = write, .context = context, .ok = true };

    // numero magico, rettangolo e numero di zone
    for (int i = 0; i < MAP_FILE_MAGIC_LEN; i++) encoder_put(&encoder, (uint8_t)MAP_FILE_MAGIC[i]);
    encoder_put_u16(&encoder, (uint16_t)x);
    encoder_put_u16(&encoder, (uint16_t)y);
    encoder_put_u16(&encoder, (uint16_t)width);
    encoder_put_u16(&encoder, (uint16_t)height);
    encoder_put(&encoder, (uint8_t)num_zones);

    // zone, in celle fini
    for (int i = 0; i < num_zones; i++) {
        encoder_put_u16(&encoder, (uint16_t)zone_coord_to_cells(zones[i].x));
        encoder_put_u16(&encoder, (uint16_t)zone_coord_to_cells(zones[i].y));
        encoder_put_u16(&encoder, (uint16_t)zone_size_to_cells(zones[i].w));
        encoder_put_u16(&encoder, (uint16_t)zone_size_to_cells(zones[i].h));
    }

    // celle a sequenze: riga dopo riga, una sequenza può continuare nella riga dopo, al massimo MAP_RUN_MAX celle
    uint8_t run_category = get_map_category(categories, x, y);
    int run_count = 0;
    for (int cell_y = y; encoder.ok && cell_y < y + height; cell_y++) {
        for (int cell_x = x; cell_x < x + width; cell_x++) {
            uint8_t category = get_map_category(categories, cell_x, cell_y);
            if (category != run_category || run_count == MAP_RUN_MAX) {
                encoder_put(&encoder, (uint8_t)(run_category << MAP_RUN_CATEGORY_SHIFT | run_count));
                run_category = category;
                run_count = 0;
            }
            run_count++;
        }
    }
    encoder_put(&encoder, (uint8_t)(run_category << MAP_RUN_CATEGORY_SHIFT | run_count));

    // CRC di tutto quello che precede, little endian (non entra nel CRC)
    uint32_t crc = encoder.crc;
    for (int i = 0; i < MAP_FILE_CRC_LEN; i++) encoder_put(&encoder, (uint8_t)(crc >> (8 * i)));

    encoder_flush(&encoder);
    return encoder.ok;
}

bool find_map_rectangle(const uint8_t *categories, int *x, int *y, int *width, int *height) {

    int x_min = FINE_MAP_CELL_NUMBER, y_min = FINE_MAP_CELL_NUMBER, x_max = -1, y_max = -1;
    for (int cell_y = 0; cell_y < FINE_MAP_CELL_NUMBER; cell_y++) {
        for (int cell_x = 0; cell_x < FINE_MAP_CELL_NUMBER; cell_x++) {
            if (get_map_category(categories, cell_x, cell_y) == MAP_CAT_UNKNOWN) continue;
            if (cell_x < x_min) x_min = cell_x;
            if (cell_x > x_max) x_max = cell_x;
            if (cell_y < y_min) y_min = cell_y;
            if (cell_y > y_max) y_max = cell_y;
        }
    }

    // mappa vuota
    if (x_max < 0) return false;

    *x = x_min;
    *y = y_min;
    *width = x_max - x_min + 1;
    *height = y_max - y_min + 1;
    return true;
}

// funzione helper che scrive nel file i byte della codifica
static bool write_to_file(void *context, const uint8_t *data, size_t size) {
    return fwrite(data, 1, size, (FILE *)context) == size;
}

// salva la mappa nel file map.rtm. Ritorna true se ci riesce
static bool save_map(void) {

    // copia la griglia a categorie e le zone dell'utente, fra un job e l'altro della mappatura
    mapping_forbidden_zone_t zones[MAX_FORBIDDEN_ZONES - 1];
    int num_zones = 0;
    if (!copy_map_categories(aper_state.categories, 0, 0, FINE_MAP_CELL_NUMBER, FINE_MAP_CELL_NUMBER, zones, &num_zones)) {
        fprintf(stderr, "aperiodico: mappa non salvata, la mappatura non lascia la finestra libera per la copia\n");
        return false;
    }

    // rettangolo delle celle viste (non sconosciute)
    int x, y, width, height;
    if (!find_map_rectangle(aper_state.categories, &x, &y, &width, &height)) {
        fprintf(stderr, "aperiodico: mappa non salvata, è vuota\n");
        return false;
    }

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

    // le sequenze vanno dirette nel file temporaneo, con il CRC calcolato man mano
    bool ok = encode_map_rectangle(aper_state.categories, x, y, width, height, zones, num_zones, write_to_file, f);

    // chiudi e rinomina sopra il vecchio: fino a questo momento il file vecchio resta intatto
    if (fclose(f) != 0) ok = false;
    if (!ok || rename(tmp_path, path) != 0) {
        remove(tmp_path);
        fprintf(stderr, "aperiodico: mappa non salvata, errore di scrittura su %s\n", tmp_path);
        return false;
    }

    fprintf(stderr, "aperiodico: mappa salvata in %s (%d x %d celle, %d zone)\n", path, width, height, num_zones);
    return true;
}

// funzione helper che salva la mappa e ne pubblica l'esito. Dopo una sostituzione o una cancellazione del file non si
// salva più: il programma sta per riavviarsi con il file nuovo, e un salvataggio lo sovrascriverebbe
static void save_map_and_publish(void) {

    if (aper_state.is_saving_disabled) return;

    bool ok = save_map();
    atomic_store(&save_failed, !ok);
    if (ok) atomic_fetch_add(&save_count, 1);
}

// funzione helper che esegue una richiesta sul file della mappa: la rinomina del file scaricato sopra map.rtm (la
// rinomina sostituisce in un colpo solo: dopo un'interruzione di corrente c'è o la mappa vecchia intera o la nuova
// intera) oppure la cancellazione. La fa il task aperiodico perché è lui che salva: un salvataggio non può incrociarsi
static void apply_map_file_request(const aperiodic_file_request_t *request) {

    char path[256], download_path[256];
    bool ok = map_file_path(path, sizeof path, "") == 0 && map_file_path(download_path, sizeof download_path, MAP_DOWNLOAD_EXT) == 0;

    if (ok && request->is_delete) {
        // se il file non c'è non c'è niente da cancellare: va bene lo stesso
        ok = remove(path) == 0 || errno == ENOENT;
    } else if (ok) {
        ok = rename(download_path, path) == 0;
    }

    if (ok) aper_state.is_saving_disabled = true;

    fprintf(stderr, "aperiodico: mappa %s%s\n", request->is_delete ? "cancellata" : "sostituita", ok ? "" : ": NON RIUSCITO");

    atomic_store(&file_request_ok, ok);
    atomic_store(&file_request_done_num, request->num);
}

// funzione helper che controlla le zone volute, le passa alla mappatura e, appena applicate, salva la mappa (se un file
// della mappa c'è già: altrimenti la mappa non è ancora completa, e le zone si salveranno con il primo salvataggio)
static void apply_user_zones_request(const aperiodic_zones_request_t *request) {

    // ogni zona deve stare dentro la griglia ed essere larga e alta almeno una cella
    const double half_side = FINE_MAP_ORIGIN * FINE_MAP_CELL_SIZE;
    if (request->num_zones < 0 || request->num_zones > MAX_FORBIDDEN_ZONES - 1) {
        fprintf(stderr, "aperiodico: zone rifiutate, al massimo %d\n", MAX_FORBIDDEN_ZONES - 1);
        return;
    }
    for (int i = 0; i < request->num_zones; i++) {
        const mapping_forbidden_zone_t *z = &request->zones[i];
        if (z->w < FINE_MAP_CELL_SIZE || z->h < FINE_MAP_CELL_SIZE || z->x < -half_side || z->y < -half_side ||
            z->x + z->w > half_side || z->y + z->h > half_side) {
            fprintf(stderr, "aperiodico: zone rifiutate, la zona %d esce dalla griglia o è vuota\n", i);
            return;
        }
    }

    // passa le zone alla mappatura, con la versione della richiesta
    mapping_user_zones_t user_zones = { .version = request->num, .num_zones = request->num_zones };
    for (int i = 0; i < request->num_zones; i++) user_zones.zones[i] = request->zones[i];
    write_user_zones(&user_zones);

    // aspetta che la mappatura le applichi (con la posizione persa non le applica: si salveranno più avanti)
    int64_t limit = rt_adesso() + ZONES_APPLY_TIMEOUT;
    while (read_applied_user_zones_version() != request->num) {
        if (rt_adesso() > limit) {
            fprintf(stderr, "aperiodico: zone non ancora applicate dalla mappatura, si salveranno con la mappa\n");
            return;
        }
        rt_dormi_fino_a(rt_adesso() + ZONES_APPLY_WAIT);
    }
    fprintf(stderr, "aperiodico: %d zone proibite applicate\n", request->num_zones);

    // salva subito, se esiste già un file della mappa
    char path[256];
    FILE *f = map_file_path(path, sizeof path, "") == 0 ? fopen(path, "rb") : NULL;
    if (f) {
        fclose(f);
        save_map_and_publish();
    }
}

// job del task: esegue le richieste nuove
void job_aperiodic(void *arg) {

    (void)arg;

    // Richiesta: Ricerca della posizione
    aperiodic_search_request_t search_request = {0};
    scambio_leggi(&search_request_buffer, &search_request);

    if (search_request.num != aper_state.last_search_num) {
        aper_state.last_search_num = search_request.num;
        run_position_search(search_request.num);
    }

    // Richiesta: Salvataggio della Mappa
    aperiodic_save_request_t save_request = {0};
    scambio_leggi(&save_request_buffer, &save_request);

    if (save_request.num != aper_state.last_save_num) {
        aper_state.last_save_num = save_request.num;
        save_map_and_publish();
    }

    // Richiesta: Scrittura delle Zone Proibite
    aperiodic_zones_request_t zones_request = {0};
    scambio_leggi(&zones_request_buffer, &zones_request);

    if (zones_request.num != aper_state.last_zones_num) {
        aper_state.last_zones_num = zones_request.num;
        apply_user_zones_request(&zones_request);
    }

    // Richiesta: Sostituzione o Cancellazione del file della mappa (dall'app)
    aperiodic_file_request_t file_request = {0};
    scambio_leggi(&file_request_buffer, &file_request);

    if (file_request.num != aper_state.last_file_num) {
        aper_state.last_file_num = file_request.num;
        apply_map_file_request(&file_request);
    }
}
