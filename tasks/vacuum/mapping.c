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
#define INFLATION_MARGIN_MM         46          // margine usato per calcolare un margine di distanza tra il robot e gli ostacoli nella griglia di navigazione, espresso in mm
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

#define ALIGN_DAMPING 1e8                   // costante per non avere divisioni per 0 nel gradient descent
#define ALIGN_STEPS         5               // numero di passi della discesa del gradiente per ogni scansione
#define ALIGN_MAX_STEP_M    0.05            // massimo spostamento in un passo della discesa del gradiente, lungo x e lungo y, espresso in m

#define STUCK_WINDOW_SCANS     7        // numero di scansioni ferme prima che scatti l'allarme "robot incastrato"
#define STUCK_MIN_ODOM_MOVE_M  0.05     // distanza minima che il robot deve percorrere prima che si consideri in movimento
#define STUCK_MAX_MAP_MOVE_M   0.02     // distanza massima da percorrere perchè si consideri fermo

#define QUAD_CROSSED_FLAG 128           // se questo flag è attivo dentro al contatore num_occupied_fine_cells, significa che quella cella fine è attraversabile dal robot
#define QUAD_CROSSED_DISTANCE 0.02      // quanto il centro del robot deve passare vicino al centro del quadrante per attivarne il flag
#define QUAD_CROSSED_MAX_STEP 0.15      // se il robot si sposta di una distanza superiore a questa tra due job, non si considerano le celle in mezzo come attraversate

// canale di scambio del doppio buffer delle zone proibite
static scambio_t double_buffer;
// i due buffer
static mapping_forbidden_zone_list_t buffer1, buffer2;

// canale di scambio del doppio buffer per l'allineamento
static scambio_t double_buffer_align;
// i due buffer dell'allineamento 
static mapping_correction_t buffer3, buffer4;

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


// Un punto del lidar pronto per l'allineamento: coordinate in metri rispetto al robot nella posizione di riferimento
typedef struct {
    double x, y;
} align_point_t;

// Elemento della lista degli ultimi valori dell'allineamento
typedef struct { 
    double odom_x, odom_y, map_x, map_y;
} stuck_history_entry_t;

// Definizione dello stato interno del task di mappatura
typedef struct {

    // Mappa fine usata internamente dal task di mappatura
    // Anche questa mappa è indicizzata come la griglia di navigazione
    // Ogni cella di questa griglia ha un punteggio da FINE_GRID_MIN_SCORE
    // a FINE_GRID_MAX_SCORE che ne indica la probabilità che sia occupata
    int8_t fine_grid[FINE_MAP_CELL_NUMBER * FINE_MAP_CELL_NUMBER];

    // numero di celle fini occupate entro il raggio di ingrossamento per ogni quadrante di una cella della griglia di navigazione
    // 1 contatore per ogni cella fine all'interno del quadrante della cella di navigazione
    uint8_t num_occupied_fine_cells[4][NAV_MAP_CELL_NUMBER * NAV_MAP_CELL_NUMBER];

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

    // stima allineata del robot
    double aligned_x, aligned_y, aligned_theta;

    // posizione data dall'odometria dopo ultimo allineamento
    double last_odom_x, last_odom_y, last_odom_theta;

    // indica se la stima è già inizializzata o meno
    bool aligned_valid;

    // punti misurati dal lidar da usare per l'allineamento 
    align_point_t lidar_point[LIDAR_GIRO_MAX_PUNTI];

    // storico delle pose dell'odometria, letto dal task movement a ogni job
    movement_odom_history_t odom_history;

    // versione dell'ultima correzione pubblicata
    uint32_t correction_version;

    // Scansioni passate, usate per verificare se il robot è incastrato    
    stuck_history_entry_t stuck_history[STUCK_WINDOW_SCANS];

    uint32_t stuck_history_count;     // numero delle scansioni registrate finora

    // Struct che contiene l'ultima posizione registrata del robot
    struct {                            
        double x, y;                // coordinate
        bool is_valid;              // indica se esiste un ultima posizione
    } last_mark;

} mapping_state_t;

// stato del task di mappatura
static mapping_state_t map_state;



// funzione helper che prende l'indice di una cella della mappa di navigazione
// e restituisce la bit mask con i quadranti bloccati e lo stato a 0
static uint8_t calc_nav_cell_bitmask(uint16_t index) {
    
    // Inizializza la bitmask a 0
    uint8_t bitmask = 0;
    
    // scorri tutti i quadranti
    for (int q = 0; q < 4; q++) {
        // se un quadrante ha una delle celle fini bloccate al suo interno 
        // se è 0 non ha celle fini bloccate, l'8° bit serve come flag per il passaggio del robot
        if (map_state.num_occupied_fine_cells[q][index] >= 1 && map_state.num_occupied_fine_cells[q][index] <= 127) {
            // metti a 1 il bit corrispondente nella bitmask
            bitmask |= NAV_CELL_QUAD[q];
        }
    }

    // shift di 4 a dx per fare il trick qui sotto
    bitmask >>= 4;

    // Questo è un modo assolutamente intuitivo e facile da leggere per indicare
    // che sono bloccate due celle in diagonale (9 = 1001; 6 = 0110)
    if (bitmask == 9 || bitmask == 6) {
        // Se sono bloccate solo due celle in diagonale, blocca anche una terza cella
        // in questo modo il robot non vede come libera la strada che connette i due quadranti passando per il centro
        bitmask |= 3;       // altro metodo simpatico sempre sulla stessa linea per bloccare le due celle in alto (quindi forzando il blocco di una terza cella)   
    }

    // rimettiamo a posto la bitmask
    bitmask <<= 4;

    return bitmask;
}

// funzione che prende un punto p(x, y), cioè il centro di un quadrante
// e un segmento s(x1, y1, x2, y2), cioè il tragitto percorso dal robot tra due job di mappatura
// e restituisce la distanza tra il punto e il segmento
static double calc_segment_distance(double p_x, double p_y, double s_x1, double s_y1, double s_x2, double s_y2) {
    
    // calcola il vettore del segmento e la sua lunghezza al quadrato
    double d_x = s_x2 - s_x1;
    double d_y = s_y2 - s_y1;
    double len2 = d_x * d_x + d_y * d_y;

    // se il segmento ha lunghezza 0 (robot fermo), la distanza è quella dal suo estremo
    if (len2 == 0) return hypot(p_x - s_x1, p_y - s_y1);

    // calcola a che punto del segmento viene proiettato il punto: 0 sul primo estremo, 1 sul secondo
    double t = ((p_x - s_x1) * d_x + (p_y - s_y1) * d_y) / len2;

    // se il punto cade oltre uno dei due estremi, il punto più vicino è quest'ultimo
    t = fmax(0, fmin(t, 1));

    // restituisci la distanza dal punto del segmento più vicino
    return hypot(p_x - (s_x1 + t * d_x), p_y - (s_y1 + t * d_y));
}

// funzione che legge la mappa interpolando con interpolazione bilineare. prende la posizione nella mappa x, y espressa in metri
// e restituisce il punteggio interpolato da 0 a 100 e quanto cambia il punteggio per ogni metro lungo x e lungo y
static void read_fine_grid_bilinear(const double x, const double y, double *interpolated_score, double *grad_x, double *grad_y) {

    // passa da metri a celle nella mappa fine, non usiamo coord_to_fine_index perchè c'è bisogno di ottenere i quattro centri attorno ad ogni cella
    double cell_x = x / FINE_MAP_CELL_SIZE + FINE_MAP_ORIGIN - 0.5;
    double cell_y = y / FINE_MAP_CELL_SIZE + FINE_MAP_ORIGIN - 0.5;

    // calcoliamo la cella il cui centro si trova a sinistra del punto (x, y) e la cella il cui centro si trova sotto al punto (x, y)
    int prev_cell_x = floor(cell_x);
    int prev_cell_y = floor(cell_y);

    // calcoliamo la distanza tra il punto (x, y) e la posizione del punto (cell_x, cell_y), x_deviation e y_deviation sono valori 
    // da 0 a 1 e indicano la posizione del punto (x, y) rispetto ai centri delle quattro celle trovate 
    double x_deviation = cell_x - prev_cell_x;
    double y_deviation = cell_y - prev_cell_y;

    // Se le celle si trovano ai bordi della mappa, metti i valori di output a 0 ed esci
    if (prev_cell_x < 0 || prev_cell_x > FINE_MAP_CELL_NUMBER - 2 || prev_cell_y < 0 || prev_cell_y > FINE_MAP_CELL_NUMBER - 2) {
        *interpolated_score = 0;
        *grad_x = 0;
        *grad_y = 0;
        return;
    }


    // Leggi i punteggi delle quattro celle individuate (li mettiamo in un array per semplificare le operazioni successive)
    int8_t score[4];

    score[0] = map_state.fine_grid[prev_cell_y << 10 | prev_cell_x];
    score[1] = map_state.fine_grid[prev_cell_y << 10 | (prev_cell_x + 1)];
    score[2] = map_state.fine_grid[(prev_cell_y + 1) << 10 | prev_cell_x];
    score[3] = map_state.fine_grid[(prev_cell_y + 1) << 10 | (prev_cell_x + 1)];

    // verifichiamo che ogni score sia compreso tra FINE_GRID_MIN_SCORE e FINE_GRID_MAX_SCORE, altrimenti lo mettiamo a 0
    // questo serve a rimuovere i valori speciali come VOID_OBSTACLE e LOW_OBSTACLE
    for (int i = 0; i < 4; i++) {
        if (score[i] <= 0 || score[i] > FINE_GRID_MAX_SCORE) score[i] = 0;
    }

    // calcola la media degli score nell'asse delle x (ottenendo una media superiore e una inferiore)
    double bottom_mean = (1 - x_deviation) * score[0] + x_deviation * score[1];
    double top_mean = (1 - x_deviation) * score[2] + x_deviation * score[3];

    // fai la media delle due interpolazioni parziali (ora capisco perchè si chiama BI-lineare)
    *interpolated_score = (1 - y_deviation) * bottom_mean + y_deviation * top_mean;

    // calcola i gradienti nelle x e nelle y
    *grad_y = (top_mean - bottom_mean) / FINE_MAP_CELL_SIZE; 
    *grad_x = ((1 - y_deviation) * (score[1] - score[0]) + y_deviation * (score[3] - score[2])) / FINE_MAP_CELL_SIZE;

}

// funzione che esegue un passo della discesa del gradiente. prende la posizione creduta del robot e i punti del lidar
// e sposta il robot per far cadere i punti sugli ostacoli. Corregge solo x e y: l'angolo resta quello misurato dall'IMU,
// che sbaglia pochissimo, mentre correggerlo con la mappa la farebbe ruotare un po' alla volta
static void align_scan_step(const align_point_t *points, int n, double *x, double *y, double theta) {

    // Ogni punto del lidar dice "per stare sul muro, il robot dovrebbe spostarsi così". Queste somme raccolgono i voti di tutti
    // i punti: sono i coefficienti di un sistema di 2 equazioni in 2 incognite (spostamento in x, spostamento in y).
    // gx e gy sono i gradienti del punteggio in x e y; err è quanto manca al punto per stare in pieno su un ostacolo.
    // ALIGN_DAMPING sulla diagonale evita il det nullo e, nelle direzioni senza informazioni (lungo un corridoio), tiene il passo quasi a zero.
    double sum_gx_gx = ALIGN_DAMPING, sum_gy_gy = ALIGN_DAMPING, sum_gx_gy = 0;
    double sum_gx_err = 0, sum_gy_err = 0;

    // seno e coseno dell'angolo, una volta sola per tutti i punti
    double cos_theta = cos(theta);
    double sin_theta = sin(theta);

    for (int i = 0; i < n; i++) {

        // ruota il punto dell'angolo del robot: ora è il vettore che va dal robot al punto, negli assi della mappa
        double rotated_x = points[i].x * cos_theta - points[i].y * sin_theta;
        double rotated_y = points[i].x * sin_theta + points[i].y * cos_theta;

        // leggi la mappa nel punto in cui cadrebbe il punto del lidar, con la posizione creduta
        double score, grad_x, grad_y;
        read_fine_grid_bilinear(*x + rotated_x, *y + rotated_y, &score, &grad_x, &grad_y);

        // quanto manca al punto per stare in pieno su un ostacolo
        double score_error = FINE_GRID_MAX_SCORE - score;

        // aggiungi il voto di questo punto alle somme
        sum_gx_gx += grad_x * grad_x;
        sum_gx_gy += grad_x * grad_y;
        sum_gy_gy += grad_y * grad_y;
        sum_gx_err += grad_x * score_error;
        sum_gy_err += grad_y * score_error;
    }

    // Risolvi il sistema con la regola di Cramer:
    //   | sum_gx_gx  sum_gx_gy |   | step_x |   | sum_gx_err |
    //   | sum_gx_gy  sum_gy_gy | * | step_y | = | sum_gy_err |
    double det = sum_gx_gx * sum_gy_gy - sum_gx_gy * sum_gx_gy;

    // se il sistema non si può risolvere, non muovere nulla
    if (fabs(det) < 1e-9) return;

    double step_x = (sum_gy_gy * sum_gx_err - sum_gx_gy * sum_gy_err) / det;
    double step_y = (sum_gx_gx * sum_gy_err - sum_gx_gy * sum_gx_err) / det;

    // limita il passo: un solo passo non deve poter portare il robot troppo lontano
    if (step_x > ALIGN_MAX_STEP_M) step_x = ALIGN_MAX_STEP_M;
    if (step_x < -ALIGN_MAX_STEP_M) step_x = -ALIGN_MAX_STEP_M;
    if (step_y > ALIGN_MAX_STEP_M) step_y = ALIGN_MAX_STEP_M;
    if (step_y < -ALIGN_MAX_STEP_M) step_y = -ALIGN_MAX_STEP_M;

    // applica il passo alla posizione creduta
    *x += step_x;
    *y += step_y;
}

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

    // inizializza il doppio buffer dell'allineamento passando i due buffer da usare
    // e la loro dimensione 
    scambio_init(
        &double_buffer_align,
        &buffer3,
        &buffer4, 
        sizeof(mapping_correction_t)
    );


    // Inizializza il mutex, salva il valore di ritorno per restituirlo alla fine
    int error_num = rt_mutex_init(&navigation_map_data.mutex);

    // Imposta tutte le celle ai bordi della navigation grid a blocked, 
    // in questo modo nel calcolo del percorso del task di navigazione
    // non ci sono problemi ai bordi della mappa
    for (int i = 0; i < NAV_MAP_CELL_NUMBER; i++) {
        navigation_map_data.navigation_grid[i] = NAV_CELL_BLOCKED | NAV_CELL_QUAD_MASK;
        navigation_map_data.navigation_grid[((NAV_MAP_CELL_NUMBER - 1) << 8) | i] = NAV_CELL_BLOCKED | NAV_CELL_QUAD_MASK;
        navigation_map_data.navigation_grid[i << 8] = NAV_CELL_BLOCKED | NAV_CELL_QUAD_MASK;
        navigation_map_data.navigation_grid[(i << 8) | (NAV_MAP_CELL_NUMBER - 1)] = NAV_CELL_BLOCKED | NAV_CELL_QUAD_MASK;
    }

    // aumenta il numero di versione, per segnalare che sono state fatte delle modifiche
    navigation_map_data.version_number++;

    // inizializza l'allineamento a false
    map_state.aligned_valid = false;

    // inizializza la versione della correzione
    map_state.correction_version = 0;

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

// funzione di lettura delle correzione della posizione del robot rispetto alla mappa
mapping_correction_t read_mapping_correction(void) {

    // Crea e Inizializza a 0 la struttura dati mapping_correction
    mapping_correction_t mapping_correction = {0};

    // Copia il buffer pubblicato in mapping_correction
    scambio_leggi(&double_buffer_align, &mapping_correction);

    // ritorna mapping_correction
    return mapping_correction;

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

            // scorri i 4 quadranti della cella corrente, nell'ordine di NAV_CELL_QUAD
            for (int quad = 0; quad < 4; quad++) {

                // calcola il centro del quadrante corrente in metri
                double quad_x_mt, quad_y_mt;
                quad_to_coord(nav_index, NAV_CELL_QUAD[quad], &quad_x_mt, &quad_y_mt);

                // calcola la distanza tra la cella di riferimento e il centro del quadrante
                double dist = hypot(fine_x_mt - quad_x_mt, fine_y_mt - quad_y_mt);

                // se la distanza è <= (ROBOT_RADIUS + INFLATION_MARGIN_MM) / 1000
                //  aumenta il numero di celle occupate del quadrante corrente
                if (dist <= (ROBOT_RADIUS + INFLATION_MARGIN_MM) / 1000)
                    map_state.num_occupied_fine_cells[quad][nav_index] += value;
            }
        }
    }
}

// Funzione helper che toglie il segno di quadrante attraversato dai quadranti attorno a una cella della griglia fine:
// il robot sarà anche passato di lì, ma un contatto con il vuoto o con un ostacolo dimostra che non deve tornarci
static void clear_crossed_quads(const int fine_x, const int fine_y) {

    // calcola le celle attorno a quella corrente (fine_x, fine_y)
    int nav_x_min = 0;
    int nav_x_max = 0;

    int nav_y_min = 0;
    int nav_y_max = 0;

    get_x_y_min_max_area_around_cell(fine_x, fine_y, FINE_CELL_INFLATION_RADIUS, &nav_x_min, &nav_x_max, &nav_y_min, &nav_y_max);

    // calcola le coordinate in metri della cella corrente
    double fine_x_mt, fine_y_mt;
    fine_index_to_coord((fine_y << 10) | fine_x, &fine_x_mt, &fine_y_mt);

    // scorri le celle attorno alla cella alle coordinate fine_x e fine_y
    for (int nav_y = nav_y_min; nav_y <= nav_y_max; nav_y++) {
        for (int nav_x = nav_x_min; nav_x <= nav_x_max; nav_x++) {

            uint16_t nav_index = (nav_y << 8) | nav_x;

            // scorri i 4 quadranti della cella corrente, nell'ordine di NAV_CELL_QUAD
            for (int quad = 0; quad < 4; quad++) {

                // calcola il centro del quadrante corrente in metri
                double quad_x_mt, quad_y_mt;
                quad_to_coord(nav_index, NAV_CELL_QUAD[quad], &quad_x_mt, &quad_y_mt);

                // se il quadrante è entro la stessa distanza usata per i contatori, togli il segno
                if (hypot(fine_x_mt - quad_x_mt, fine_y_mt - quad_y_mt) <= (ROBOT_RADIUS + INFLATION_MARGIN_MM) / 1000) {
                    map_state.num_occupied_fine_cells[quad][nav_index] &= ~QUAD_CROSSED_FLAG;
                }
            }
        }
    }
}

// Funzione Helper che imposta una cella ad un valore speciale
static void set_fine_cell_special_value(const uint32_t cell_index, const int special_value) {

    // calcola x e y della griglia fine partendo da cell_index
    int x_fine = cell_index & 0x3FF;
    int y_fine = cell_index >> 10;

    // togli il segno di quadrante attraversato, va fatto anche se la cella ha già un valore speciale:
    // se il robot torna a vedere il buco nella stessa cella, il segno potrebbe essere stato rimesso
    clear_crossed_quads(x_fine, y_fine);

    // Salva il vecchio punteggio della griglia fine
    int old_score = map_state.fine_grid[cell_index];

    // se old_score è uno dei valori speciali, esci
    if (old_score == VOID_OBSTACLE || old_score == LOW_OBSTACLE) return;

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

// Funzione helper che porta una posizione data dall'odometria nel sistema della mappa, usando l'ultimo allineamento.
// Tra stima e odometria c'è uno spostamento più una rotazione: all'ultimo allineamento l'odometria valeva last_odom e la
// stima valeva aligned, quindi basta ripartire da aligned con lo spostamento fatto da allora, ruotato di offset.
// Finché non c'è stato nessun allineamento la mappa e l'odometria coincidono, e la posizione resta com'è.
static void odom_to_map(double *x, double *y, double *theta) {

    if (!map_state.aligned_valid) return;

    double offset = map_state.aligned_theta - map_state.last_odom_theta;
    double dx = *x - map_state.last_odom_x;
    double dy = *y - map_state.last_odom_y;

    *x = map_state.aligned_x + dx * cos(offset) - dy * sin(offset);
    *y = map_state.aligned_y + dx * sin(offset) + dy * cos(offset);
    *theta += offset;
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
        // la posizione dell'evento arriva da drv_posa, quindi è già nel sistema della mappa
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

// funzione helper che prende la posizione attuale del robot e segna le celle che attraversa come attraversabili
// in questo modo, se il robot è passato una volta per una cella, potrà passarci durante tutta la sessione
static void mark_crossed_quads(double pos_x, double pos_y) {

    // se lo spostamento non è valido o supera QUAD_CROSSED_MAX_STEP, salva la posizione ed esci
    if (!map_state.last_mark.is_valid ||
        hypot(pos_x - map_state.last_mark.x, pos_y - map_state.last_mark.y) > QUAD_CROSSED_MAX_STEP) {
    
            map_state.last_mark.x = pos_x;
            map_state.last_mark.y = pos_y;
            map_state.last_mark.is_valid = true;
            return;
    }

    // Altrimenti, controlla i quadranti delle 3x3 celle attorno al robot
    uint16_t robot_cell = coord_to_nav_index(pos_x, pos_y);

    for (int dy = -1; dy <= 1; dy++) {
        for (int dx = -1; dx <= 1; dx++) {

            uint16_t cell = robot_cell + dy * NAV_MAP_CELL_NUMBER + dx;

            // scorri i 4 quadranti della cella, nell'ordine di NAV_CELL_QUAD
            for (int quad = 0; quad < 4; quad++) {

                // calcola il centro del quadrante
                double quad_x, quad_y;
                quad_to_coord(cell, NAV_CELL_QUAD[quad], &quad_x, &quad_y);

                // se il tratto percorso dal robot passa abbastanza vicino al centro del quadrante, segnalo come attraversato
                if (calc_segment_distance(quad_x, quad_y, map_state.last_mark.x, map_state.last_mark.y, pos_x, pos_y) <= QUAD_CROSSED_DISTANCE) {
                    map_state.num_occupied_fine_cells[quad][cell] |= QUAD_CROSSED_FLAG;
                }
            }
        }
    }

    // Salva la nuova posizione attuale
    map_state.last_mark.x = pos_x;
    map_state.last_mark.y = pos_y;

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

    mark_crossed_quads(pos_x, pos_y);
    
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

// funzione helper che riporta un angolo tra -M_PI e M_PI
static double wrap_angle(double angle) {
    while (angle > M_PI) angle -= 2 * M_PI;
    while (angle < -M_PI) angle += 2 * M_PI;
    return angle;
}

// funzione helper che calcola la posizione del robot in un certo istante secondo l'odometria grezza, usando lo storico delle
// pose pubblicato dal task movement. Cerca le due pose attorno all'istante richiesto e fa la media pesata in base al tempo.
// Non usa le velocità: da incastrato le ruote dichiarano ancora la velocità piena, e ne verrebbe uno spostamento inventato.
// Se l'istante è fuori dallo storico restituisce la posa più vicina
static void pose_at_time(int64_t time_ns, double *x, double *y, double *theta) {

    const movement_odom_history_t *history = &map_state.odom_history;

    // se lo storico è vuoto la posizione è quella di partenza
    if (history->count == 0) {
        *x = 0;
        *y = 0;
        *theta = 0;
        return;
    }

    // scorri le pose dalla più recente alla più vecchia: la più recente è quella prima di next_index
    uint32_t newest = (history->next_index + MOVEMENT_ODOM_HISTORY_SIZE - 1) % MOVEMENT_ODOM_HISTORY_SIZE;
    movement_odom_pose_t after = history->pose[newest];
    movement_odom_pose_t before = after;

    for (uint32_t i = 1; i < history->count && before.t_ns > time_ns; i++) {
        after = before;
        before = history->pose[(newest + MOVEMENT_ODOM_HISTORY_SIZE - i) % MOVEMENT_ODOM_HISTORY_SIZE];
    }

    // calcola quanto l'istante richiesto è vicino alla posa successiva, da 0 a 1 (fuori dallo storico resta agli estremi)
    double weight = 0;
    if (after.t_ns > before.t_ns) weight = (double)(time_ns - before.t_ns) / (double)(after.t_ns - before.t_ns);
    if (weight < 0) weight = 0;
    if (weight > 1) weight = 1;

    // fai la media pesata delle due pose. per l'angolo si media la differenza, che può passare da +PI a -PI
    *x = before.x + (after.x - before.x) * weight;
    *y = before.y + (after.y - before.y) * weight;
    *theta = before.theta + wrap_angle(after.theta - before.theta) * weight;
}

// funzione helper che prepara i punti dell'ultima scansione per l'allineamento e ne restituisce il numero.
// i punti di una scansione sono misurati in istanti diversi: ogni punto viene messo sulla mappa con la posizione
// che il robot aveva nel suo istante, poi viene espresso rispetto a un'unica posizione, ref
static int prepare_lidar_points(double ref_x, double ref_y, double ref_theta) {

    const lidar_giro_t *scan = &map_state.last_scan;
    int64_t scan_duration = scan->t_fine_ns - scan->t_inizio_ns;

    // seno e coseno dell'angolo di riferimento
    double cos_ref = cos(ref_theta);
    double sin_ref = sin(ref_theta);

    int num_points = 0;

    for (int i = 0; i < scan->n; i++) {

        // scarta i punti senza eco (che sono a 0) e quelli oltre LIDAR_MAPPING_MAX_RANGE
        if (scan->p[i].distanza_mm == 0 || scan->p[i].distanza_mm > LIDAR_MAPPING_MAX_RANGE * 1000) continue;

        // calcola la posizione del robot nell'istante in cui è stato misurato questo punto
        int64_t point_time = scan->t_inizio_ns + (scan_duration * i) / scan->n;
        double robot_x, robot_y, robot_theta;
        pose_at_time(point_time, &robot_x, &robot_y, &robot_theta);

        // calcola le coordinate del punto rispetto al robot
        double rel_x, rel_y;
        drv_punto_xy(&scan->p[i], &rel_x, &rel_y);

        // calcola le coordinate del punto sulla mappa
        double cos_robot = cos(robot_theta);
        double sin_robot = sin(robot_theta);
        double abs_x = robot_x + rel_x * cos_robot - rel_y * sin_robot;
        double abs_y = robot_y + rel_x * sin_robot + rel_y * cos_robot;

        // esprimi il punto rispetto alla posizione di riferimento: è la rotazione inversa di quella sopra
        double dx = abs_x - ref_x;
        double dy = abs_y - ref_y;
        map_state.lidar_point[num_points].x = dx * cos_ref + dy * sin_ref;
        map_state.lidar_point[num_points].y = -dx * sin_ref + dy * cos_ref;
        num_points++;
    }

    return num_points;
}

// funzione helper che aggiorna la stima allineata del robot con l'ultima scansione.
// ref_x, ref_y e ref_theta sono la posizione data dall'odometria alla fine della scansione.
static void update_aligned_pose(double ref_x, double ref_y, double ref_theta) {

    // Alla prima scansione la mappa è vuota e non c'è niente con cui allineare, la stima parte dall'odometria
    if (!map_state.aligned_valid) {
        map_state.aligned_x = ref_x;
        map_state.aligned_y = ref_y;
        map_state.aligned_theta = ref_theta;
        map_state.aligned_valid = true;
    } else {
        // la stima precedente più lo spostamento dell'odometria dall'ultima scansione.
        // la stima e l'odometria possono avere angoli diversi, quindi prima di sommarlo lo spostamento
        // va ruotato negli assi della stima, ed è l'angolo fra i due che dice di quanto ruotarlo
        double odom_dx = ref_x - map_state.last_odom_x;
        double odom_dy = ref_y - map_state.last_odom_y;
        double odom_dtheta = wrap_angle(ref_theta - map_state.last_odom_theta);
        double offset = map_state.aligned_theta - map_state.last_odom_theta;

        map_state.aligned_x += odom_dx * cos(offset) - odom_dy * sin(offset);
        map_state.aligned_y += odom_dx * sin(offset) + odom_dy * cos(offset);
        map_state.aligned_theta = wrap_angle(map_state.aligned_theta + odom_dtheta);

        // ogni passo avvicina la stima alla posizione in cui i punti cadono sugli ostacoli
        int num_points = prepare_lidar_points(ref_x, ref_y, ref_theta);

        for (int step = 0; step < ALIGN_STEPS; step++) {
            align_scan_step(map_state.lidar_point, num_points, &map_state.aligned_x, &map_state.aligned_y, map_state.aligned_theta);
        }

        map_state.aligned_theta = wrap_angle(map_state.aligned_theta);
    }

    // Salva l'odometria di questa scansione, servirà per calcolare lo spostamento alla prossima
    map_state.last_odom_x = ref_x;
    map_state.last_odom_y = ref_y;
    map_state.last_odom_theta = ref_theta;

    uint8_t idx = map_state.stuck_history_count % STUCK_WINDOW_SCANS;
    stuck_history_entry_t last_stuck_entry = map_state.stuck_history[idx];      // la scansione di 7 giri fa

    bool is_stuck = false;

    if(map_state.stuck_history_count >= STUCK_WINDOW_SCANS) {
        double odom_moved = hypot(ref_x - last_stuck_entry.odom_x, ref_y - last_stuck_entry.odom_y);
        double map_moved  = hypot(map_state.aligned_x - last_stuck_entry.map_x, map_state.aligned_y - last_stuck_entry.map_y);
        is_stuck = odom_moved > STUCK_MIN_ODOM_MOVE_M && map_moved < STUCK_MAX_MAP_MOVE_M;
    }
    
    // salva la scansione attuale nello storico, sovrascrivendo la più vecchia
    map_state.stuck_history[idx] = (stuck_history_entry_t){
        .odom_x = ref_x,
        .odom_y = ref_y,
        .map_x = map_state.aligned_x,
        .map_y = map_state.aligned_y,
    };
    map_state.stuck_history_count++;

    // Pubblica la correzione, il task movement la userà per portare la sua odometria nel sistema della mappa
    mapping_correction_t mapping_correction = {
        .version = ++map_state.correction_version,
        .odom_x = map_state.last_odom_x,
        .odom_y = map_state.last_odom_y,
        .odom_theta = map_state.last_odom_theta,
        .map_x = map_state.aligned_x,
        .map_y = map_state.aligned_y,
        .map_theta = map_state.aligned_theta,
        .is_stuck = is_stuck,
    };

    scambio_scrivi(&double_buffer_align, &mapping_correction);
}

// funzione helper che serve alla navigazione per rilevare se una cella di navigazione contiene al suo interno almeno una 
// cella fine che sia un LOW_OBSTACLE o VOID_OBSTACLE, entrambi invisibili per il lidar
bool is_nav_cell_invisible_obstacle(uint16_t cell) {
    
    // calcola la prima cella fine della cella di navigazione
    const int first_fine_x = (cell & 0xFF) * FINE_CELL_IN_NAV_CELL;
    const int first_fine_y = (cell >> 8) * FINE_CELL_IN_NAV_CELL;

    // scorri le celle fini della cella di navigazione
    for (int fine_y = first_fine_y; fine_y < first_fine_y + FINE_CELL_IN_NAV_CELL; fine_y++) {
        for (int fine_x = first_fine_x; fine_x < first_fine_x + FINE_CELL_IN_NAV_CELL; fine_x++) {

            const int score = map_state.fine_grid[(fine_y << 10) | fine_x];

            if (score == VOID_OBSTACLE || score == LOW_OBSTACLE) return true;
        }
    }

    return false;
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

    // Leggi lo storico delle pose dell'odometria: serve a sapere dov'era il robot quando è stato misurato ogni punto
    read_odom_history(&map_state.odom_history);

    // Applica gli eventi speciali (sensori di caduta, incastro e bumper anteriore) sulla mappa
    bool has_applied_events = apply_queued_events();

    // Porta la posizione attuale del robot nel sistema della mappa
    double map_x = cur_pos.odom_x, map_y = cur_pos.odom_y, map_theta = cur_pos.odom_theta;
    odom_to_map(&map_x, &map_y, &map_theta);

    // Aggiorna i valori delle celle sotto al robot
    mark_cells_under_robot(map_x, map_y);

    // se la velocità di rotazione del robot al momento della scansione del lidar
    // supera la soglia o la scansione è vecchia, la scansione non è valida
    
    bool use_scan = false;

    if (fabs(cur_pos.omega) < ROTATION_SPEED_MAX_THRESHOLD && !is_same_scan) use_scan = true;

    // se gli eventi non hanno applicato nulla e la scansione non è valida, esci
    if (!use_scan && !has_applied_events) return;

    // Definisci le coordinate del robot come celle della fine map, serviranno
    // sia nell'algoritmo di Bresenham, che nel calcolo della navigation map
    int robot_cell_y = map_y, robot_cell_x = map_x;


    {   // queste variabili mi servono solo per calcolare robot_cell_xy quindi le metto in un blocco separato

        uint32_t robot_cell_index = coord_to_fine_index(map_x, map_y);

        robot_cell_y = robot_cell_index >> 10;
        robot_cell_x = robot_cell_index & 0x3FF;
    }

    // se la nuova scansione va utilizzata
    if (use_scan) {

        // Calcola inizio e durata della scansione, insieme alla velocità del robot serve 
        // per compensare la deformazione della scansione del lidar
        int64_t scan_start_time = map_state.last_scan.t_inizio_ns;
        int64_t scan_duration = map_state.last_scan.t_fine_ns - scan_start_time;

        // calcola la posizione di riferimento dell'allineamento: dove si trovava il robot alla fine della scansione
        double ref_x, ref_y, ref_theta;
        pose_at_time(map_state.last_scan.t_fine_ns, &ref_x, &ref_y, &ref_theta);

        // aggiorna la stima allineata, prima di scrivere la scansione nella mappa
        update_aligned_pose(ref_x, ref_y, ref_theta);

        
        // Scorri tutti i punti della scansione del lidar e se sono != 0
        // Calcola la posizione del punto rispetto al robot nella mappa
        for (int i = 0; i < map_state.last_scan.n; i++) {
            
            // se un punto ha distanza 0, non considerarlo
            if (map_state.last_scan.p[i].distanza_mm == 0) continue;
            
            // calcola l'istante in cui è stato campionato questo punto
            int64_t cur_point_time = scan_start_time + (scan_duration * i) / map_state.last_scan.n;

            // calcola la posizione del robot nell'istante di acquisizione del punto
            double cur_robot_x, cur_robot_y, cur_theta;
            pose_at_time(cur_point_time, &cur_robot_x, &cur_robot_y, &cur_theta);

            // portala nel sistema della mappa: la scansione si scrive con la posizione corretta
            odom_to_map(&cur_robot_x, &cur_robot_y, &cur_theta);

            // calcola seno e coseno dell'angolo del robot
            double cos_theta = cos(cur_theta);
            double sin_theta = sin(cur_theta);

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
        uint16_t cur_idx = closed_cells.cells[i % CLOSED_CELLS_MAX];
        navigation_map_data.navigation_grid[cur_idx] = set_nav_cell_state(navigation_map_data.navigation_grid[cur_idx], NAV_CELL_TOO_NARROW);
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

            // Salva lo stato della cella corrente
            uint8_t cur_cell_state = get_nav_cell_state(navigation_map_data.navigation_grid[cur_nav_index]);

            // Salva la bitmask della cella corrente
            uint8_t cur_cell_mask = calc_nav_cell_bitmask(cur_nav_index);

            // prossimo stato da assegnare alla cella corrente
            mapping_nav_cell_state next_state;

            // Se la cella è proibita o troppo stretta, next_state è lo stato attuale
            if (cur_cell_state == NAV_CELL_FORBIDDEN || cur_cell_state == NAV_CELL_TOO_NARROW) next_state = cur_cell_state;
            // Se la cella ha la bitmask tutta a 1, allora next_state è NAV_CELL_BLOCKED
            else if (cur_cell_mask == NAV_CELL_QUAD_MASK) next_state = NAV_CELL_BLOCKED;
            // altrimenti, se almeno UNKNOWN_CELL_THRESHOLD sono state osservate, segnala come libera
            else if (map_state.num_observed_fine_cells[cur_nav_index] >= UNKNOWN_CELL_THRESHOLD) next_state = NAV_CELL_FREE;
            // altrimenti segnala come sconosciuta
            else next_state = NAV_CELL_UNKNOWN;
            

            // Se il prossimo stato è diverso da quello attuale, o se la bitmask
            // è cambiata, allora imposta hasChanged a true
            // e assegna il nuovo stato alla cella della griglia di navigazione
            if ((next_state | cur_cell_mask) != navigation_map_data.navigation_grid[cur_nav_index]) {
                navigation_map_data.navigation_grid[cur_nav_index] = next_state | cur_cell_mask;
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