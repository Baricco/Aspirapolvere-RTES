#include <stdatomic.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>

#include "driver/driver.h"
#include "driver/scambio.h"

#include "mapping.h"
#include "lidar.h"
#include "movement.h"
#include "navigation.h"


#define FINE_GRID_MAX_SCORE         100         // massimo punteggio di una cella della griglia fine
#define FINE_GRID_MIN_SCORE         (-100)      // minimo punteggio di una cella della griglia fine
#define LIDAR_HIT_SCORE             30          // punteggio aggiunto ad una cella se il lidar rileva che c'è un'ostacolo
#define LIDAR_PASS_SCORE            (-10)       // punteggio tolto da una cella se il lidar non vede ostacoli
#define LIDAR_MAPPING_MAX_RANGE     6           // distanza massima a cui il lidar può misurare un punto, prima che il rumore sia troppo alto, espresso in m
#define INFLATION_MARGIN_MM         25          // margine usato per calcolare un margine di distanza tra il robot e gli ostacoli nella griglia di navigazione, espresso in mm
#define FINE_CELL_INFLATION_RADIUS  (int)(ceil(((ROBOT_RADIUS + INFLATION_MARGIN_MM) / 1000) / FINE_MAP_CELL_SIZE)) // indica di quanto vengono ingrossate le celle con ostacoli nella mappa di navigazione, il margine serve per essere certi che il robot non sbatta contro gli ostacoli, considerando anche le dimensioni del robot e un piccolo margine d'errore
#define FINE_CELL_OCCUPATION_THRESHOLD 50        // soglia sopra al quale una cella è considerata occupata
#define UNKNOWN_CELL_THRESHOLD      8           // numero minimo di celle fini da osservare affinchè la cella di navigazione risulti osservata
#define ROTATION_SPEED_MAX_THRESHOLD 1.5        // massima velocità angolare che può avere il robot prima che la scansione del lidar venga scartata, oltre questa velocità la scansione è deformata. espressa in rad/s
#define VOID_OBSTACLE                -128       // punteggio speciale da assegnare ad una cella che viene rilevata come "dirupo" dai sensori anti-caduta, una volta che una cella viene messa a questo valore, non può essere modificata dal ciclo di aggiornamento lidar-based
#define LOW_OBSTACLE                 127        // punteggio speciale da assegnare ad una cella che viene rilevata come ostacolo dal bumper, ma non rilevato dal lidar, una volta che una cella viene messa a questo valore, non può essere modificata dal ciclo di aggiornamento lidar-based

#define CELL_COV_RADIUS (ROBOT_RADIUS / 1000.0 / FINE_MAP_CELL_SIZE - M_SQRT2)    // raggio delle celle coperte certamente dal robot al suo passaggio

// Posizione dei sensori di contatto rispetto al centro del robot (x in avanti, y a sinistra, angoli positivi a sinistra).
// Valori presi da sim/robot.h, dove sono segnati come stime: da verificare sul robot.
#define CLIFF_SENSOR_DISTANCE        0.15       // distanza degli anti-caduta dal centro del robot, espressa in m
// angolo dell'anti-caduta k rispetto al davanti del robot, nello stesso ordine di cliff[] e di CONTATTO_VUOTO_*
// (0 sinistro laterale +50°, 1 sinistro anteriore +18°, 2 destro anteriore -18°, 3 destro laterale -50°), espresso in radianti
#define CLIFF_SENSOR_ANGLE(k)        (((k) == 0 ? 50 : (k) == 1 ? 18 : (k) == 2 ? -18 : -50) * M_PI / 180)

#define BUMPER_DISTANCE              ((ROBOT_RADIUS + 20) / 1000)   // dove si segna l'urto: 2 cm oltre il bordo del robot (come il software originale), espresso in m
#define BUMPER_SX_ANGLE              (45 * M_PI / 180)              // direzione dell'urto per il bumper sinistro, espressa in radianti
#define BUMPER_DX_ANGLE              (-45 * M_PI / 180)             // direzione dell'urto per il bumper destro, espressa in radianti


// canale di scambio del doppio buffer
static scambio_t double_buffer;
// i due buffer
static mapping_forbidden_zone_list_t buffer1, buffer2;

// caratteristiche del task
rt_task_t mapping_task;

// Struttura che contiene i dati da condividere con il task di navigazione e
// il loro mutex che li protegge
typedef struct {

    // Griglia di Navigazione come Array piatto
    // Questa mappa dovrebbe essere di 65536 celle int da 1 Byte, 
    // quindi una coordinata è un numero di 2 Byte 
    // (trick di ottimizzazione della memoria, per questo le dimensioni sono così)
    uint8_t navigation_grid[NAV_MAP_CELL_NUMBER*NAV_MAP_CELL_NUMBER];

    // Numero di versione della mappa, quando cambia, la navigazione lo vede
    _Atomic uint32_t version_number;

    // Mutex per condividere la mappa tra task di mappatura e di navigazione
    pthread_mutex_t mutex;


} navigation_map_data_t;

static navigation_map_data_t navigation_map_data;


// Definizione dello stato interno del task di mappatura
typedef struct {

    // Mappa fine usata internamente dal task di mappatura
    // Anche questa mappa è indicizzata come la griglia di navigazione
    // Ogni cella di questa griglia ha un punteggio da FINE_GRID_MIN_SCORE
    // a FINE_GRID_MAX_SCORE che ne indica la probabilità che sia occupata
    int8_t fine_grid[FINE_MAP_CELL_NUMBER * FINE_MAP_CELL_NUMBER];

    // numero di celle fini occupate entro il raggio di ingrossamento per ogni cella della griglia di navigazione
    uint8_t num_occupied_fine_cells[NAV_MAP_CELL_NUMBER * NAV_MAP_CELL_NUMBER];

    // numero di celle fini già viste per ogni cella della griglia di navigazione
    uint8_t num_observed_fine_cells[NAV_MAP_CELL_NUMBER * NAV_MAP_CELL_NUMBER];

    // numero dell'ultima scansione del lidar (restituito da drv_giro)
    uint32_t last_scan_num;

    // ultima scansione del lidar
    lidar_giro_t last_scan;

    // numero dell'ultimo evento dei sensori applicato
    uint32_t last_event;

    // numero dell'ultima cella chiusa aggiornata dal task di navigazione
    uint32_t last_closed_num;

} mapping_state_t;

// stato del task di mappatura
static mapping_state_t map_state;

uint32_t read_nav_map(uint8_t *navigation_map) {
    
    uint32_t cur_version = 0;

    // acquisisci il mutex per copiare la mappa
    pthread_mutex_lock(&navigation_map_data.mutex);

    // esegui la copia della mappa
    memcpy(navigation_map, navigation_map_data.navigation_grid, sizeof(navigation_map_data.navigation_grid));

    // copia anche il numero di versone della mappa
    cur_version = navigation_map_data.version_number;

    // rilascia il mutex
    pthread_mutex_unlock(&navigation_map_data.mutex);

    return cur_version;
}

uint32_t read_nav_map_version_number(void) {

    return navigation_map_data.version_number;

}

// funzione di inizializzazione del task
// ! Questa funzione restituisce un intero che indica se ci sono stati problemi
// ! Nella creazione del mutex
int init_mapping(void) {

    // inizializza il doppio buffer passando i due buffer da usare
    // e la loro dimensione 
    scambio_init(
        &double_buffer,
        &buffer1,
        &buffer2, 
        sizeof(mapping_forbidden_zone_list_t)
    );

    // Inizializza il mutex, salva il valore di ritorno per restituirlo alla fine
    int error_num = rt_mutex_init(&navigation_map_data.mutex);

    // Imposta tutte le celle ai bordi della navigation grid a blocked, 
    // in questo modo nel calcolo del percorso del task di navigazione
    // non ci sono problemi ai bordi della mappa
    for (int i = 0; i < NAV_MAP_CELL_NUMBER; i++) {
        navigation_map_data.navigation_grid[i] = NAV_CELL_BLOCKED;
        navigation_map_data.navigation_grid[((NAV_MAP_CELL_NUMBER - 1) << 8) | i] = NAV_CELL_BLOCKED;
        navigation_map_data.navigation_grid[i << 8] = NAV_CELL_BLOCKED;
        navigation_map_data.navigation_grid[(i << 8) | (NAV_MAP_CELL_NUMBER - 1)] = NAV_CELL_BLOCKED;
    }

    // aumenta il numero di versione, per segnalare che sono state fatte delle modifiche
    navigation_map_data.version_number++;

    return error_num;

}

// funzione di lettura delle zone proibite
mapping_forbidden_zone_list_t read_forbidden_zone_list(void) {

    // Crea e Inizializza a 0 la struttura dati forbidden_zones
    mapping_forbidden_zone_list_t forbidden_zones = {0};

    // Copia il buffer pubblicato in forbidden_zones
    scambio_leggi(&double_buffer, &forbidden_zones);

    // ritorna forbidden_zones
    return forbidden_zones;
}


// funzione di scrittura dei dati del task (le zone proibite)
void write_forbidden_zone_list(const mapping_forbidden_zone_list_t *forbidden_zones) {

    // Copia scrivi le zone proibite nel doppio buffer
    scambio_scrivi(&double_buffer, forbidden_zones);

    return;
}

// Funzione helper che serve a calcolare le variabili x_min, x_max, y_min e y_max
// per scorrere le celle attorno alla cella cell_x, cell_y entro un certo range
// Osserva che questa funzione prende coordinate sulla griglia fine e 
// restituisce coordinate sulla griglia di navigazione  
static inline void get_x_y_min_max_area_around_cell(int cell_x, int cell_y, int range, int *x_min, int *x_max, int *y_min, int *y_max) {
    
    // calcola le celle attorno a quella corrente (fine_x, fine_y)
    *x_min = (cell_x - range) / 4;
    *x_max = (cell_x + range) / 4;

    *y_min = (cell_y - range) / 4;
    *y_max = (cell_y + range) / 4;

    // clampa i valori delle celle attorno per non contare i bordi 
    if (*x_min < 1) *x_min = 1;
    if (*y_min < 1) *y_min = 1;
    if (*x_max > 254) *x_max = 254;
    if (*y_max > 254) *y_max = 254;
}

// Funzione helper per aggiornare il numero di celle occupate, sia per la cella
// cell_x cell_y, sia per le 9 celle attorno
static void update_occupied_cells(const int fine_x, const int fine_y, const int value) {
    
    // calcola le celle attorno a quella corrente (fine_x, fine_y)
    int nav_x_min = 0;
    int nav_x_max = 0;

    int nav_y_min = 0;
    int nav_y_max = 0;

    get_x_y_min_max_area_around_cell(fine_x, fine_y, FINE_CELL_INFLATION_RADIUS, &nav_x_min, &nav_x_max, &nav_y_min, &nav_y_max);

    // calcola le coordinate in metri della cella corrente
    double fine_x_mt, fine_y_mt;
    fine_index_to_coord((fine_y << 10) | fine_x, &fine_x_mt, &fine_y_mt);

    // Scorri le 9 celle attorno alla cella alle coordinate fine_x e fine_y
    for (int nav_y = nav_y_min; nav_y <= nav_y_max; nav_y++) {
        for (int nav_x = nav_x_min; nav_x <= nav_x_max; nav_x++) {

            // calcola il navigation index, una sola volta
            uint16_t nav_index = (nav_y << 8) | nav_x;

            // calcola il centro della cella corrente in metri
            double cur_x_mt, cur_y_mt;
            nav_index_to_coord(nav_index, &cur_x_mt, &cur_y_mt);

            // calcola la distanza tra la cella di riferimento e quella corrente
            double dist = hypot(fine_x_mt - cur_x_mt, fine_y_mt - cur_y_mt);

            // se la distanza è <= FINE_CELL_INFLATION_RADIUS * FINE_MAP_CELL_SIZE
            //  aumenta il numero di celle occupate della cella corrente
            if (dist <= FINE_CELL_INFLATION_RADIUS * FINE_MAP_CELL_SIZE) 
                map_state.num_occupied_fine_cells[nav_index] += value;
        }
    }
}

// Funzione Helper che imposta una cella ad un valore speciale
static void set_fine_cell_special_value(const uint32_t cell_index, const int special_value) {

    // Salva il vecchio punteggio della griglia fine
    int old_score = map_state.fine_grid[cell_index];

    // se old_score è uno dei valori speciali, esci
    if (old_score == VOID_OBSTACLE || old_score == LOW_OBSTACLE) return;

    // calcola x e y della griglia fine partendo da cell_index
    int x_fine = cell_index & 0x3FF;
    int y_fine = cell_index >> 10;

    // Ricava l'indice della cella della griglia di navigazione
    // che corrisponde alla cella della griglia fine all'indice cell_index
    int nav_index = (x_fine >> 2) | ((y_fine >> 2) << 8);

    // Aggiorna il numero di celle osservate, analogo a add_score_to_fine_cell
    if (old_score == 0) map_state.num_observed_fine_cells[nav_index]++;   // se una cella che prima era sconosciuta, ora è stata vista, segnala come osservata

    // Aggiorna il numero di celle occupate, analogo a add_score_to_fine_cell
    if (old_score <= FINE_CELL_OCCUPATION_THRESHOLD) update_occupied_cells(x_fine, y_fine, 1);       // prima non era occupata, ora risulta occupata

    // Scrivi il valore speciale nella griglia
    map_state.fine_grid[cell_index] = special_value;
}

// Funzione helper per applicare gli eventi alla mappa, restituisce true se ha modificato la mappa
static bool apply_queued_events(void) {
        
    // Variabile che conterrà gli ultimi 32 eventi registrati dal driver
    drv_contatti_t last_events = {0};
    
    // Leggi gli ultimi eventi
    drv_contatti(&last_events);
    
    bool hasChanged = false;

    for (uint32_t i = map_state.last_event + 1; i <= last_events.ultimo; i++) {

        // Salva l'evento corrente 
        drv_contatto_t cur_event = last_events.ev[i % CONTATTI_N];

        // l'evento che stiamo guardando è diverso da i, significa che sono avvenuti più di 32
        // eventi nell'utimo giro, saltiamo questo evento
        if (cur_event.numero != i) continue;

        // Se l'evento corrente non ha una posizione valida, saltalo
        if (!cur_event.posa_valida) continue;

        // Se l'evento ha una posizione vecchia (>100ms), saltalo
        if (cur_event.t_ns - cur_event.posa.t_ns > RT_MS(100)) continue;

        double dist = 0, angle = 0;
        int8_t event_value = 0;

        // Ricava di che evento si tratta
        if (cur_event.sensore <= CONTATTO_VUOTO_DX_LAT) {       // se il sensore è uno di quelli di caduta
            dist = CLIFF_SENSOR_DISTANCE;
            angle = CLIFF_SENSOR_ANGLE(cur_event.sensore);
            event_value = VOID_OBSTACLE;
        } else if (cur_event.sensore == CONTATTO_INCASTRO) {                      // se il sensore è quello degli incastri
            dist = BUMPER_DISTANCE;
            angle = 0;
            event_value = LOW_OBSTACLE;
        } else {                            // se il sensore è il bumper
            dist = BUMPER_DISTANCE;
            angle = cur_event.sensore == CONTATTO_BUMPER_DX ? BUMPER_DX_ANGLE : BUMPER_SX_ANGLE;
            event_value = LOW_OBSTACLE;
        }

        // calcola le coordinate della cella in cui è avvenuto l'evento
        double x = cur_event.posa.x + dist * cos(cur_event.posa.theta + angle);
        double y = cur_event.posa.y + dist * sin(cur_event.posa.theta + angle);

        set_fine_cell_special_value(coord_to_fine_index(x, y), event_value);

        hasChanged = true;
    }

    map_state.last_event = last_events.ultimo;

    return hasChanged;

}

// Funzione helper per sommare uno score ad una cella della fine map
static void add_score_to_fine_cell(const uint32_t cell_index, const int score) {

    // Salva il vecchio punteggio della griglia fine
    int old_score = map_state.fine_grid[cell_index];

    // se old_score è uno dei valori speciali, esci
    if (old_score == VOID_OBSTACLE || old_score == LOW_OBSTACLE) return;

    // salva il punteggio in una variabile temporanea per controllare overflow e underflow
    int tot_score = old_score + score;

    // Controlla se fa overflow o underflow e clampa il valore
    if (tot_score > FINE_GRID_MAX_SCORE) tot_score = FINE_GRID_MAX_SCORE;
    if (tot_score < FINE_GRID_MIN_SCORE) tot_score = FINE_GRID_MIN_SCORE;

    // calcola x e y della griglia fine partendo da cell_index
    int x_fine = cell_index & 0x3FF;
    int y_fine = cell_index >> 10;

    // Ricava l'indice della cella della griglia di navigazione
    // che corrisponde alla cella della griglia fine all'indice cell_index
    int nav_index = (x_fine >> 2) | ((y_fine >> 2) << 8);

    // Aggiorna il contatore del numero di celle fini osservate
    // Che corrispondono alla cella di navigazione in nav_index
    if (old_score != 0 && tot_score == 0) map_state.num_observed_fine_cells[nav_index]--;   // se una cella che prima era stata osservata, ora non è più sicura, segnala come sconosciuta
    if (old_score == 0 && tot_score != 0) map_state.num_observed_fine_cells[nav_index]++;   // se una cella che prima era sconosciuta, ora è stata vista, segnala come osservata

    // Aggiorna il contatore del numero di celle fini occupate
    // Che corrispondono alla cella di navigazione in nav_index
    if (old_score <= FINE_CELL_OCCUPATION_THRESHOLD && tot_score > FINE_CELL_OCCUPATION_THRESHOLD) update_occupied_cells(x_fine, y_fine, 1);       // prima non era occupata, ora risulta occupata
    if (old_score > FINE_CELL_OCCUPATION_THRESHOLD && tot_score <= FINE_CELL_OCCUPATION_THRESHOLD) update_occupied_cells(x_fine, y_fine, -1);       // prima era occupata, ora risulta libera


    // assegna lo score clampato alla cella
    map_state.fine_grid[cell_index] = tot_score;

}

// Funzione Helper che esegue l'algoritmo di Bresenham
// Questa funzione trova le celle della griglia fine per cui è passato
// il raggio del lidar e ad ognuna somma lo score corrispondente
static void exec_bresenham_algorithm(const int x_coord_start, const int y_coord_start, const int x_coord_end, const int y_coord_end) {

    // calcola la distanza tra i due punti nell'asse x
    int dx = abs(x_coord_end - x_coord_start);

    // calcola il verso in cui si muove la x
    int sx = (x_coord_start < x_coord_end) ? 1 : -1;

    // calcola la distanza tra i due punti nell'asse y
    // il segno meno serve per non dover invertire i confronti e le somme di dy
    // rispetto a dx
    int dy = -abs(y_coord_end - y_coord_start);

    // calcola il verso in cui si muove la y
    int sy = (y_coord_start < y_coord_end) ? 1 : -1;

    // calcola quanto la cella corrente si trova distante rispetto alla retta
    int err = dx + dy;

    // Inzializza le coordinate della cella corrente alla cella iniziale
    int cur_cell_x = x_coord_start, cur_cell_y = y_coord_start;

    // calcola la portata massima del lidar una volta sola prima del ciclo
    int max_portata = lround(LIDAR_MAPPING_MAX_RANGE / FINE_MAP_CELL_SIZE);


    // Esegui l'algoritmo finchè non arrivi alla cella finale
    for(int step = 0; step <= dx - dy; step++ ) {
        // Se siamo arrivati all'ultima cella
        if (cur_cell_x == x_coord_end && cur_cell_y == y_coord_end) {
            // Aggiungi l'hit_score e esci dal ciclo
            add_score_to_fine_cell(cur_cell_y << 10 | cur_cell_x, LIDAR_HIT_SCORE);
            break;
        } else {    // Se non siamo ancora arrivati alla fine

            // Aggiungi il pass_score
            add_score_to_fine_cell(cur_cell_y << 10 | cur_cell_x, LIDAR_PASS_SCORE);

            // raddoppia errore e salvatelo in una variabile per fare i confronti
            int e2 = 2 * err;
            
            // avanza di un passo nella direzione corrente in base all'errore
            if (e2 >= dy) { err += dy; cur_cell_x += sx; }
            if (e2 <= dx) { err += dx; cur_cell_y += sy; }

            // controlla se la distanza percorsa è uguale alla portata, se sì, esci
            // da qui in poi l'errore del lidar è troppo grande
            int delta_x = cur_cell_x - x_coord_start;
            int delta_y = cur_cell_y - y_coord_start;
            int cur_dist = delta_x * delta_x + delta_y * delta_y;
            if (cur_dist >= max_portata * max_portata) break;
        }
    } 

}

// funzione helper che data la posizione attuale del robot,
// segna le celle sopra alle quali passa come libere
static void mark_cells_under_robot(double pos_x, double pos_y) {

    uint32_t idx = coord_to_fine_index(pos_x, pos_y);
    
    // calcola la cella che si trova al centro del robot
    uint16_t robot_y = idx >> 10, robot_x = idx & 0x3FF;

    int radius = CELL_COV_RADIUS;

    // se il robot si trova sul bordo della mappa, non aggiornare le celle sotto di lui (esci)
    if (robot_x < radius || robot_y < radius || robot_x > 1023 - radius || robot_y > 1023 - radius) return;

    // scorri tutte le celle sotto al robot
    for (int dy = -radius; dy <= radius; dy++) {
        for (int dx = -radius; dx <= radius; dx++) {
            // se la cella corrente si trova effettivamente sotto il robot
            if (dx*dx + dy*dy <= CELL_COV_RADIUS*CELL_COV_RADIUS) {
                // sottrai 100 dalla cella, in modo che sia segnata come libera
                add_score_to_fine_cell(((robot_y + dy) << 10) | (robot_x + dx), FINE_GRID_MIN_SCORE);
            }
        }
    }

}

// job del task
void job_mapping(void *arg) {
    
    (void)arg;


    // Leggi l'ultiima scansione del lidar
    uint32_t cur_scan_num = drv_giro(&map_state.last_scan);

    // Salvati se il numero di scansione è uguale all'ultimo giro
    bool is_same_scan = cur_scan_num == map_state.last_scan_num;

    // Se è una nuova scansione, aggiorna il numero di scansione
    if (!is_same_scan) map_state.last_scan_num = cur_scan_num;

    // Leggi la posizione attuale del robot, dal task movement
    movement_position_data_t cur_pos = read_movement();

    // Se la posizione attuale non è valida (il robot è sollevato), esci
    if (!cur_pos.valid) return;

    // Applica gli eventi speciali (sensori di caduta, incastro e bumper anteriore) sulla mappa
    bool has_applied_events = apply_queued_events();

    // Aggiorna i valori delle celle sotto al robot
    mark_cells_under_robot(cur_pos.x, cur_pos.y);

    // se la velocità di rotazione del robot al momento della scansione del lidar
    // supera la soglia o la scansione è vecchia, la scansione non è valida
    
    bool use_scan = false;

    if (fabs(cur_pos.omega) < ROTATION_SPEED_MAX_THRESHOLD && !is_same_scan) use_scan = true;

    // se gli eventi non hanno applicato nulla e la scansione non è valida, esci
    if (!use_scan && !has_applied_events) return;

    // Definisci le coordinate del robot come celle della fine map, serviranno
    // sia nell'algoritmo di Bresenham, che nel calcolo della navigation map
    int robot_cell_y = cur_pos.y, robot_cell_x = cur_pos.x;


    {   // queste variabili mi servono solo per calcolare robot_cell_xy quindi le metto in un blocco separato

        uint32_t robot_cell_index = coord_to_fine_index(cur_pos.x, cur_pos.y);

        robot_cell_y = robot_cell_index >> 10;
        robot_cell_x = robot_cell_index & 0x3FF;
    }

    // se la nuova scansione va utilizzata
    if (use_scan) {
        // Calcola inizio e durata della scansione, insieme alla velocità del robot serve 
        // per compensare la deformazione della scansione del lidar
        int64_t scan_start_time = map_state.last_scan.t_inizio_ns;
        int64_t scan_duration = map_state.last_scan.t_fine_ns - scan_start_time;

        // Scorri tutti i punti della scansione del lidar e se sono != 0
        // Calcola la posizione del punto rispetto al robot nella mappa
        for (int i = 0; i < map_state.last_scan.n; i++) {
            
            // se un punto ha distanza 0, non considerarlo
            if (map_state.last_scan.p[i].distanza_mm == 0) continue;
            
            // calcola l'istante in cui è stato campionato questo punto
            int64_t cur_point_time = scan_start_time + (scan_duration * i) / map_state.last_scan.n;

            // calcola il tempo trascorso tra la scansione del punto e l'acquisizione dell'ultimo
            // frame dai sensori. espresso in secondi
            double dt = (cur_point_time - cur_pos.frame_acq_t) / 1e9;

            // calcola l'angolo del robot nell'istante di acquisizione del punto
            double cur_theta = cur_pos.theta + cur_pos.omega * dt;

            // calcola seno e coseno dell'angolo del robot
            double cos_theta = cos(cur_theta);
            double sin_theta = sin(cur_theta);

            // calcola la posizione del robot nell'istante di acquisizione del punto
            double cur_robot_x = cur_pos.x + cur_pos.v * cos_theta * dt;
            double cur_robot_y = cur_pos.y + cur_pos.v * sin_theta * dt;

            // calcola l'indice della cella della fine map dove si trova il robot all'istante
            // di acquisizione del punto 
            uint32_t robot_cell_index = coord_to_fine_index(cur_robot_x, cur_robot_y);

            // dall'indice ricava le coordinate del robot
            // serviranno per l'algoritmo di Bresenham e anche fuori dal for
            robot_cell_y = robot_cell_index >> 10;
            robot_cell_x = robot_cell_index & 0x3FF;


            // inizializza le coordinate del punto nella mappa fine a 0, relative
            // rispetto alla posizione del robot
            double x_coord_rel = 0, y_coord_rel = 0;

            // calcola le coordinate x, y del punto scansionato rispetto al robot
            drv_punto_xy(&map_state.last_scan.p[i], &x_coord_rel, &y_coord_rel);

            // per ottenere le coordinate assolute, ruota i punti della rotazione del robot
            // e somma le coordinate del robot a x/y_coord
            double x_coord_abs = cur_robot_x + (x_coord_rel * cos_theta - y_coord_rel * sin_theta);
            double y_coord_abs = cur_robot_y + (x_coord_rel * sin_theta + y_coord_rel * cos_theta);

            
            // calcola l'indice della cella della mappa fine del punto corrente
            uint32_t cur_point_cell_index = coord_to_fine_index(x_coord_abs, y_coord_abs);

            // dall'indice ricava le coordinate del punto
            // serviranno per l'algoritmo di Bresenham
            int cur_point_cell_y = cur_point_cell_index >> 10;
            int cur_point_cell_x = cur_point_cell_index & 0x3FF;


            // Adesso facciamo l'algoritmo di Bresenham per ottenere
            // le celle della fine map per cui passa il raggio del lidar
            // e aggiornare sia i punti per cui è passato il raggio
            // e non ha trovato ostacoli, sia l'ultimo punto dove il raggio si è infranto
            exec_bresenham_algorithm(robot_cell_x, robot_cell_y, cur_point_cell_x, cur_point_cell_y);

        }
    }

    // Calcola i valori x, y massimi e minimi entro il quale si trova l'area appena aggiornata
    int nav_x_min, nav_x_max, nav_y_min, nav_y_max;
    get_x_y_min_max_area_around_cell(
        robot_cell_x,
        robot_cell_y,
        (LIDAR_MAPPING_MAX_RANGE / FINE_MAP_CELL_SIZE) + (FINE_CELL_INFLATION_RADIUS * 2),
        &nav_x_min,
        &nav_x_max, 
        &nav_y_min, 
        &nav_y_max
    );


    // leggi se ci sono state modifiche fatte dal task di navigazione
    navigation_closed_cells_t closed_cells = read_closed_cells();

    // Prendi il mutex
    pthread_mutex_lock(&navigation_map_data.mutex);

    bool hasChanged = false;

    // scorri le modifiche proposte dal task di navigazione e applicale alla mappa
    for (uint32_t i = map_state.last_closed_num; i < closed_cells.num; i++) {
        navigation_map_data.navigation_grid[closed_cells.cells[i % CLOSED_CELLS_MAX]] = NAV_CELL_TOO_NARROW;
        hasChanged = true;
    }

    // metti l'indice dell'ultima cella letta all'ultimo indice letto dopo il for, in questo
    // modo last_closed_num si comporta come un buffer circolare
    map_state.last_closed_num = closed_cells.num;

    // Scorri le celle all'interno dell'area aggiornata
    for (int cur_nav_y = nav_y_min; cur_nav_y <= nav_y_max; cur_nav_y++) {
        for (int cur_nav_x = nav_x_min; cur_nav_x <= nav_x_max; cur_nav_x++) {

            // ricava l'indice della cella attuale
            int cur_nav_index = cur_nav_y << 8 | cur_nav_x;

            // ignora le celle appartenenti a zone proibite
            if (navigation_map_data.navigation_grid[cur_nav_index] == NAV_CELL_FORBIDDEN || navigation_map_data.navigation_grid[cur_nav_index] == NAV_CELL_TOO_NARROW) continue;

            // prossimo stato da assegnare alla cella corrente
            mapping_nav_cell_state next_state;

            // se c'è anche solo una cella fine occupata, imposta tutta la cella di navigazione 
            // che la contiene come occupata
            if (map_state.num_occupied_fine_cells[cur_nav_index] > 0) next_state = NAV_CELL_BLOCKED;
            // altrimenti, se almeno UNKNOWN_CELL_THRESHOLD sono state osservate, segnala come libera
            else if (map_state.num_observed_fine_cells[cur_nav_index] >= UNKNOWN_CELL_THRESHOLD) next_state = NAV_CELL_FREE;
            // altrimenti segnala come sconosciuta
            else next_state = NAV_CELL_UNKNOWN;
            

            // Se il prossimo stato è diverso da quello attuale, imposta hasChanged a true
            // e assegna il nuovo stato alla cella della griglia di navigazione
            if (next_state != navigation_map_data.navigation_grid[cur_nav_index]) {
                navigation_map_data.navigation_grid[cur_nav_index] = next_state;
                hasChanged = true;
            }
            
        }
    }

    // Se è cambiato qualcosa, aumenta il numero di versione per segnalare al task
    // di navigazione che deve prendersi la mappa aggiornata
    if (hasChanged) navigation_map_data.version_number++;

    // Rilascia il mutex
    pthread_mutex_unlock(&navigation_map_data.mutex);
}  