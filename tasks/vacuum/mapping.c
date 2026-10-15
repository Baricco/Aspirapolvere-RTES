#include <stdatomic.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>

#include "driver/driver.h"
#include "driver/scambio.h"

#include "mapping.h"
#include "lidar.h"
#include "movement.h"


#define FINE_GRID_MAX_SCORE         100         // massimo punteggio di una cella della griglia fine
#define FINE_GRID_MIN_SCORE         (-100)      // minimo punteggio di una cella della griglia fine
#define LIDAR_HIT_SCORE             30          // punteggio aggiunto ad una cella se il lidar rileva che c'è un'ostacolo
#define LIDAR_PASS_SCORE            (-10)       // punteggio tolto da una cella se il lidar non vede ostacoli
#define LIDAR_MAPPING_MAX_RANGE     6           // distanza massima a cui il lidar può misurare un punto, prima che il rumore sia troppo alto, espresso in m
#define INFLATION_MARGIN_MM         25          // margine usato per calcolare un margine di distanza tra il robot e gli ostacoli nella griglia di navigazione, espresso in mm
#define FINE_CELL_INFLATION_RADIUS  (int)(ceil(((ROBOT_RADIUS + INFLATION_MARGIN_MM) / 1000) / FINE_MAP_CELL_SIZE)) // indica di quanto vengono ingrossate le celle con ostacoli nella mappa di navigazione, il margine serve per essere certi che il robot non sbatta contro gli ostacoli, considerando anche le dimensioni del robot e un piccolo margine d'errore
#define FINE_CELL_OCCUPATION_THRESHOLD 50        // soglia sopra al quale una cella è considerata occupata
#define UNKNOWN_CELL_THRESHOLD      8           // numero minimo di celle fini da osservare affinchè la cella di navigazione risulti osservata


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

    // numero di celle fini già viste entro il raggio di ingrossamento per ogni cella della griglia di navigazione
    uint8_t num_observed_fine_cells[NAV_MAP_CELL_NUMBER * NAV_MAP_CELL_NUMBER];

    // numero dell'ultima scansione del lidar (restituito da drv_giro)
    uint32_t last_scan_num;

    // ultima scansione del lidar
    lidar_giro_t last_scan;

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

// Funzione helper per sommare uno score ad una cella della fine map
static void add_score_to_fine_cell(const uint32_t cell_index, const int score) {
    
    // salva il punteggio in una variabile temporanea per controllare overflow e underflow
    int tot_score = map_state.fine_grid[cell_index] + score;

    // Controlla se fa overflow o underflow e clampa il valore
    if (tot_score > FINE_GRID_MAX_SCORE) tot_score = FINE_GRID_MAX_SCORE;
    if (tot_score < FINE_GRID_MIN_SCORE) tot_score = FINE_GRID_MIN_SCORE;

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
    while(true) {
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

// job del task
void job_mapping(void *arg) {
    
    (void)arg;


    // Leggi l'ultiima scansione del lidar
    uint32_t cur_scan_num = drv_giro(&map_state.last_scan);

    // Se il numero di scansione è 0, o è uguale all'ultimo giro, esci 
    if (cur_scan_num == map_state.last_scan_num) return;

    map_state.last_scan_num = cur_scan_num;

    // Leggi la posizione attuale del robot, dal task movement
    movement_position_data_t cur_pos = read_movement();

    // Se la posizione attuale non è valida (il robot è sollevato), esci
    if (!cur_pos.valid) return;

    // Per non ricalcolare seno e coseno di theta per ogni punto lo salviamo
    // in una variabile fuori dal for
    double cos_theta = cos(cur_pos.theta);
    double sin_theta = sin(cur_pos.theta);

    // calcola l'indice della cella della fine map dove si trova il robot,
    // anche questo lo facciamo fuori dal for perchè è sempre uguale
    uint32_t robot_cell_index = coord_to_fine_index(cur_pos.x, cur_pos.y);

    // dall'indice ricava le coordinate del robot
    // serviranno per l'algoritmo di Bresenham
    int robot_cell_y = robot_cell_index >> 10;
    int robot_cell_x = robot_cell_index & 0x3FF;


    // Scorri tutti i punti della scansione del lidar e se sono != 0
    // Calcola la posizione del punto rispetto al robot nella mappa
    for (int i = 0; i < map_state.last_scan.n; i++) {
        
        // se un punto ha distanza 0, non considerarlo
        if (map_state.last_scan.p[i].distanza_mm == 0) continue;
        
        // inizializza le coordinate del punto nella mappa fine a 0, relative
        // rispetto alla posizione del robot
        double x_coord_rel = 0, y_coord_rel = 0;

        // calcola le coordinate x, y del punto scansionato rispetto al robot
        drv_punto_xy(&map_state.last_scan.p[i], &x_coord_rel, &y_coord_rel);

        // per ottenere le coordinate assolute, ruota i punti della rotazione del robot
        // e somma le coordinate del robot a x/y_coord
        double x_coord_abs = cur_pos.x + (x_coord_rel * cos_theta - y_coord_rel * sin_theta);
        double y_coord_abs = cur_pos.y + (x_coord_rel * sin_theta + y_coord_rel * cos_theta);

        
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