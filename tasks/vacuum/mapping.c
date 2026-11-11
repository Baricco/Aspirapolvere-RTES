#include <stdatomic.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>

#include "driver/driver.h"
#include "driver/scambio.h"

#include "mapping.h"
#include "lidar.h"
#include "movement.h"
#include "navigation.h"
#include "aperiodic.h"


#define FINE_GRID_MAX_SCORE         100         // massimo punteggio di una cella della griglia fine
#define FINE_GRID_MIN_SCORE         (-100)      // minimo punteggio di una cella della griglia fine
#define LIDAR_HIT_SCORE             30          // punteggio aggiunto ad una cella se il lidar rileva che c'è un'ostacolo
#define LIDAR_PASS_SCORE            (-10)       // punteggio tolto da una cella se il lidar non vede ostacoli
#define INFLATION_MARGIN_MM         46          // margine usato per calcolare un margine di distanza tra il robot e gli ostacoli nella griglia di navigazione, espresso in mm
#define FINE_CELL_INFLATION_RADIUS  (int)(ceil(((ROBOT_RADIUS + INFLATION_MARGIN_MM) / 1000) / FINE_MAP_CELL_SIZE)) // indica di quanto vengono ingrossate le celle con ostacoli nella mappa di navigazione, il margine serve per essere certi che il robot non sbatta contro gli ostacoli, considerando anche le dimensioni del robot e un piccolo margine d'errore
#define FINE_CELL_OCCUPATION_THRESHOLD 50        // soglia sopra al quale una cella è considerata occupata
#define UNKNOWN_CELL_THRESHOLD      8           // numero minimo di celle fini da osservare affinchè la cella di navigazione risulti osservata
#define NEAR_OBSTACLE_NO_WRITE_M    0.5         // se entro questa distanza dal robot c'è un ostacolo della mappa, la scansione serve solo per allinearsi e non si scrive, espressa in m
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

// Costanti della correzione dell'angolo con la direzione dei muri: la base sta contro un muro, quindi nella mappa i muri
// sono a 0 gradi a meno di multipli di 90. L'IMU sbaglia di qualche grado in pochi minuti
#define WALL_NEAR_POINTS_M  0.06            // due punti consecutivi del giro più vicini di così formano un tratto di muro, espresso in m
#define WALL_MIN_SEGMENT_M  0.005           // tratti più corti di così si scartano (punti doppi), espresso in m
#define WALL_MAX_RANGE_M    4.0             // punti più lontani di così non si usano (oltre il rumore del lidar cresce), espresso in m
#define WALL_MIN_LENGTH_M   0.5             // lunghezza totale minima dei tratti per usare il giro, espressa in m
#define WALL_MIN_STRENGTH   0.35            // quanto deve essere netta la direzione dei muri: 0 = nessuna, 1 = tutti i tratti paralleli o perpendicolari
#define WALL_MAX_OMEGA      0.6             // giri fatti ruotando più veloce di così non correggono l'angolo, espressa in rad/s
#define WALL_MAX_ERROR      (10 * M_PI / 180)   // errori più grandi di così non si correggono (muro storto o mobile in diagonale), espresso in radianti
#define WALL_GAIN           0.02            // frazione dell'errore corretta a ogni giro: il rumore di un giro solo non sposta l'angolo 
#define WALL_MIN_MOVE_M     0.05            // strada minima fra due correzioni: da fermo la vista è sempre la stessa e un suo piccolo difetto si sommerebbe giro dopo giro (da fermo l'IMU non deriva), espressa in m

// Costanti della mappa salvata
#define LOAD_SCORE_FREE         (-100)      // punteggio con cui si salva una cella libera caricata dal file
#define LOAD_SCORE_OCCUPIED     80          // punteggio con cui si salva una cella occupata caricata dal file
#define LOAD_BASE_WAIT          RT_S(3)     // all'avvio, tempo massimo di attesa del driver pronto per sapere se il robot è sulla base (la mappa si carica solo lì)
#define MAP_COPY_MAX_ATTEMPTS   100         // tentativi della copia fra un job e l'altro prima di rinunciare
#define MAP_COPY_RETRY_WAIT     RT_MS(2)    // attesa fra due tentativi della copia, se la mappatura sta lavorando

#define STUCK_WINDOW_SCANS     7        // numero di scansioni ferme prima che scatti l'allarme "robot incastrato"
#define STUCK_MIN_ODOM_MOVE_M  0.05     // distanza minima che il robot deve percorrere prima che si consideri in movimento
#define STUCK_MAX_MAP_MOVE_M   0.02     // distanza massima da percorrere perchè si consideri fermo

#define QUAD_CROSSED_FLAG 128           // se questo flag è attivo dentro al contatore num_occupied_fine_cells, significa che quella cella fine è attraversabile dal robot
#define QUAD_CROSSED_DISTANCE 0.02      // quanto il centro del robot deve passare vicino al centro del quadrante per attivarne il flag
#define BASE_CHECK_JOBS 5               // numero di job in cui si controlla se il robot è sulla base all'avvio, corrispondono a circa 1 s
#define QUAD_CROSSED_MAX_STEP 0.15      // se il robot si sposta di una distanza superiore a questa tra due job, non si considerano le celle in mezzo come attraversate

// Costanti riguardanti la routine di ricerca della posizione attuale del robot dopo il sollevamento
#define SEARCH_POINT_SPACING        0.15        // distanza minima fra due punti della scansione usati nella ricerca, espressa in m
#define SEARCH_MAX_RANGE            4.0         // i punti più lontani di così non si usano nella ricerca, espressa in m
#define SEARCH_MAX_POINTS           128         // numero massimo di punti usati nella ricerca
#define SEARCH_MIN_POINTS           30          // con meno punti di così la scansione non basta per cercare la posizione
#define SEARCH_NUM_ANGLES           ((int)(2 * M_PI * SEARCH_MAX_RANGE / FINE_MAP_CELL_SIZE) + 1)  // numero di angoli provati (503): fra due angoli vicini un punto a SEARCH_MAX_RANGE si sposta al massimo di una cella fine
#define SEARCH_LEVELS               6           // livello più alto della griglia della ricerca: blocchi da 2^6 = 64 celle fini per lato (3,2 m)
#define SEARCH_MAX_CELLS            400         // lato massimo, in celle fini, della zona in cui si cerca (20 m)
#define SEARCH_MIN_SCORE_PCT        70          // punteggio minimo per accettare una posizione, in percentuale del massimo (2 per punto).
#define SEARCH_LOOKUPS_PER_JOB      300000      // letture della griglia della ricerca fra due controlli dell'annullamento
#define SEARCH_BUILD_CELLS_PER_JOB  20000       // celle della griglia della ricerca preparate fra due controlli dell'annullamento
#define SEARCH_STACK_SIZE           128         // blocchi che possono aspettare nella pila per un angolo (al massimo 49 radici più 3 per livello)
#define SEARCH_MAX_ATTEMPTS         3           // ricerche o conferme fallite dopo un sollevamento prima di arrendersi
#define SEARCH_REFINE_ANGLE         (0.4 * M_PI / 180)  // la rifinitura prova gli angoli entro questa distanza da quello trovato, espressa in radianti
#define SEARCH_REFINE_ANGLE_STEP    (0.1 * M_PI / 180)  // passo degli angoli provati dalla rifinitura, espresso in radianti
#define CONFIRM_DISTANCE            1.0         // strada da percorrere con la posizione che regge, prima di confermarla, espressa in m
#define CONFIRM_MAX_BAD_SCANS       5           // scansioni di fila sotto SEARCH_MIN_SCORE_PCT che fanno scartare la posizione trovata
#define CONFIRM_MAX_OMEGA           0.5         // durante la conferma le scansioni fatte ruotando più veloce di così non si controllano (sono deformate), espressa in rad/s

// Ogni cella della griglia della ricerca è un uint16_t con 3 numeri da 3 bit: il primo livello a cui il blocco che parte dalla cella
// contiene la vicina di un ostacolo, un ostacolo, una cella libera. Salendo di livello il blocco contiene quello del livello prima,
// quindi basta sapere a che livello le cose cambiano. Il punteggio di un punto al livello h è
// (h >= livello vicina) + (h >= livello ostacolo): 2 su un ostacolo, 1 accanto, 0 lontano
#define SEARCH_NEAR_SHIFT           0           // bit 0-2: primo livello con la vicina di un ostacolo (cella occupata o una delle 8 attorno)
#define SEARCH_OCC_SHIFT            3           // bit 3-5: primo livello con un ostacolo
#define SEARCH_FREE_SHIFT           6           // bit 6-8: primo livello con una cella libera (dove può stare il centro del robot)
#define SEARCH_LEVEL_NEVER          7           // il blocco non lo contiene a nessun livello

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

// Fasi della ricerca della posizione dopo un sollevamento
typedef enum {
    SEARCH_WAIT_SCAN,       // aspetta una scansione iniziata dopo il riappoggio
    SEARCH_WAIT_RESULT,     // la ricerca gira nel task aperiodico (la mappa è prestata): aspetta il risultato
    SEARCH_CANCELLED,       // ricerca annullata da un tasto: aspetta che il robot sia sulla base (aggancio alla cieca)
} search_phase_t;

// esito della ricerca fatta dal task aperiodico
typedef enum {
    SEARCH_FOUND,           // posizione trovata (e rifinita)
    SEARCH_NOT_FOUND,       // nessuna posizione sopra la soglia
    SEARCH_NO_AREA,         // la mappa è vuota: non c'è niente in cui cercare
    SEARCH_ABORTED,         // annullata (tasto premuto o robot sollevato di nuovo)
} search_outcome_t;

// risultato della ricerca: lo scrive solo il task aperiodico, lo legge la mappatura
typedef struct {
    uint32_t request_num;   // numero della richiesta a cui risponde
    search_outcome_t outcome;
    double x, y, theta;     // posizione trovata, sulla mappa (solo con SEARCH_FOUND)
} search_result_t;

// Struttura che contiene un possibile blocco di posizioni del centro del robot ancora da guardare
typedef struct {
    int16_t x, y;           // coordinate dell'angolo in basso a sinistra del blocco di posizioni
    uint8_t level;          // livello della griglia di ricerca alla quale si riferisce il blocco
    int16_t bound;          // limite superiore del punteggio di tutte le posizioni del blocco
} search_node_t;

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

    // posizione della stima all'ultima correzione dell'angolo con i muri
    double wall_last_x, wall_last_y;

    // true finché l'angolo non è stato preso dai muri al primo giro buono (l'aggancio alla base può essere ruotato)
    bool is_initial_theta_pending;

    // zone proibite dell'utente caricate con la mappa (la zona della base si aggiunge quando il robot parte dalla base)
    mapping_forbidden_zone_t user_zones[MAX_FORBIDDEN_ZONES - 1];
    int num_user_zones;
    uint32_t user_zones_version;    // versione delle zone volute già applicata

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

    uint32_t base_check_jobs;       // numero di job in cui è stato controllato se il robot è sulla base
    bool base_seen;                 // indica se durante il controllo il robot è risultato sulla base

    uint32_t last_mission_num;      // numero dell'ultima pulizia vista dalla mappatura

    mapping_map_state state;        // stato della posizione del robot sulla mappa, viene pubblicato insieme alla correzione

    // Ricerca della posizione dopo un sollevamento e conferma della posizione trovata
    struct {
        search_phase_t phase;                       // fase della ricerca
        uint32_t request_num;                       // numero dell'ultima ricerca chiesta al task aperiodico
        bool is_cancel_requested;                   // l'annullamento della ricerca in corso è stato chiesto (il risultato non vale)
        bool is_stopped_by_user;                    // l'annullamento viene da un tasto (si aspetta la base) e non da un sollevamento
        uint32_t last_stop_num;                     // ultimo stop_search_num della navigazione già visto
        uint8_t attempts;                           // numero di ricerche o conferme fallite dall'ultimo sollevamento
        int64_t put_down_t;                         // istante del riappoggio, valgono solo le scansioni del lidar iniziate dopo
        bool grid_ready;                            // indica se la griglia corrisponde alla mappa
        int x0, y0;                                 // cella fine corrispondente alla cella (0, 0) della griglia della ricerca
        int width, height;                          // dimensioni della zona della ricerca, in celle fini
        int build_level, build_row;                 // a che punto è la preparazione della griglia
        uint16_t grid[SEARCH_MAX_CELLS * SEARCH_MAX_CELLS];    // griglia della ricerca
        align_point_t point[SEARCH_MAX_POINTS];     // punti della scansione usati, rispetto al robot
        int num_points;                             // numero di punti usati
        double ref_x, ref_y, ref_theta;             // odometria alla fine della scansione usata
        int angle;                                  // indice dell'angolo che si sta guardando, da 0 a SEARCH_NUM_ANGLES - 1
        int16_t offset_x[SEARCH_MAX_POINTS], offset_y[SEARCH_MAX_POINTS];  // punti ruotati dell'angolo corrente, in celle fini
        search_node_t stack[SEARCH_STACK_SIZE];     // blocchi ancora da guardare per l'angolo corrente, il migliore in cima
        int stack_len;                              // numero di blocchi nella pila
        int best_score;                             // miglior punteggio trovato finora (all'inizio la soglia meno 1)
        int best_angle, best_x, best_y;             // posizione col miglior punteggio, best_angle inizializzato a -1
        double confirm_distance;                    // strada percorsa durante la conferma, con la posizione che regge
        double confirm_last_x, confirm_last_y;      // odometria all'ultimo controllo della conferma
        int bad_scans;                              // scansioni di fila sotto soglia durante la conferma
    } search;

} mapping_state_t;

// stato del task di mappatura
static mapping_state_t map_state;

// indica se il robot è partito dalla base di ricarica, è atomico perché lo legge il task di navigazione
static _Atomic bool started_on_base;

// risultato della ricerca della posizione (doppio buffer: scrive il task aperiodico, legge la mappatura)
static scambio_t double_buffer_search;
static search_result_t search_buffer1, search_buffer2;

// zone volute dall'utente (doppio buffer: scrive il task aperiodico, legge la mappatura) e versione applicata
static scambio_t double_buffer_zones;
static mapping_user_zones_t zones_buffer1, zones_buffer2;
static _Atomic uint32_t applied_zones_version;

// numero della ricerca da annullare: lo scrive la mappatura, lo controlla la ricerca nel task aperiodico mentre gira
static _Atomic uint32_t search_cancel_num;

// contatore dei job, per la lettura della mappa fra un job e l'altro (copy_map_categories): sale di 1 all'inizio e alla
// fine di ogni job, quindi è dispari mentre un job è in corso. REGOLA: la mappatura scrive la mappa (griglia fine,
// contatori, griglia di navigazione) SOLO dentro il suo job, o in init_mapping prima che partano i task
static _Atomic uint32_t map_job_counter;



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

    // due quadranti liberi in diagonale restano liberi tutti e due: il passo in diagonale è sempre permesso
    // (deciso con l'utente il 2026-11-08, guasto di p5; prima qui si bloccava anche un terzo quadrante)

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
static void load_map_if_on_base(void);

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


    // inizializza il doppio buffer delle zone volute dall'utente
    scambio_init(&double_buffer_zones, &zones_buffer1, &zones_buffer2, sizeof(mapping_user_zones_t));

    // inizializza il doppio buffer del risultato della ricerca della posizione
    scambio_init(&double_buffer_search, &search_buffer1, &search_buffer2, sizeof(search_result_t));

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

    // al primo giro buono del lidar l'angolo si prende dai muri, senza tetto: l'aggancio alla base può essere ruotato
    map_state.is_initial_theta_pending = true;

    // se il robot è sulla base carica la mappa attiva (prima che partano i task, quindi senza scadenze)
    if (error_num == 0) load_map_if_on_base();

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


// funzione di lettura del risultato del controllo sulla partenza dalla base
bool read_started_on_base(void) {
    return atomic_load(&started_on_base);
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

// Funzione helper che dice se entro NEAR_OBSTACLE_NO_WRITE_M dalla posizione (x, y) c'è una cella fine occupata.
// Le celle segnate dal bumper (LOW_OBSTACLE) non contano: un urto vicino non deve impedire di scrivere la scansione
static bool is_near_obstacle(const double x, const double y) {

    // cella fine della posizione e raggio del cerchio da controllare, in celle fini
    uint32_t center_index = coord_to_fine_index(x, y);
    int center_x = center_index & 0x3FF;
    int center_y = center_index >> 10;
    int radius = (int)(NEAR_OBSTACLE_NO_WRITE_M / FINE_MAP_CELL_SIZE);

    for (int cell_y = center_y - radius; cell_y <= center_y + radius; cell_y++) {
        for (int cell_x = center_x - radius; cell_x <= center_x + radius; cell_x++) {

            // solo le celle dentro il cerchio
            if ((cell_x - center_x) * (cell_x - center_x) + (cell_y - center_y) * (cell_y - center_y) > radius * radius) continue;

            int score = map_state.fine_grid[cell_y << 10 | cell_x];
            if (score > FINE_CELL_OCCUPATION_THRESHOLD && score != LOW_OBSTACLE) return true;
        }
    }

    return false;
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

            // Aggiungi il pass_score, escludendo le ultime due celle perchè se no
            // ci sono problemi con l'allineamento per ostacoli che si trovano
            // a cavallo tra due celle
            if (abs(x_coord_end - cur_cell_x) + abs(y_coord_end - cur_cell_y) > 2) add_score_to_fine_cell(cur_cell_y << 10 | cur_cell_x, LIDAR_PASS_SCORE);

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

// funzione helper che pubblica la correzione: l'ultima stima allineata, l'odometria in quell'istante e lo stato della posizione.
// se la stima non è ancora inizializzata sono tutti 0, quindi la correzione non sposta niente (posizione sulla mappa = odometria)
static void publish_correction(bool is_stuck) {

    mapping_correction_t mapping_correction = {
        .version = ++map_state.correction_version,
        .odom_x = map_state.last_odom_x,
        .odom_y = map_state.last_odom_y,
        .odom_theta = map_state.last_odom_theta,
        .map_x = map_state.aligned_x,
        .map_y = map_state.aligned_y,
        .map_theta = map_state.aligned_theta,
        .is_stuck = is_stuck,
        .state = map_state.state,
    };

    scambio_scrivi(&double_buffer_align, &mapping_correction);
}

// funzione helper che corregge l'angolo della stima allineata con la direzione dei muri. I punti sono quelli preparati da
// prepare_lidar_points, rispetto al robot alla fine della scansione e già in ordine di angolo, quindi due punti consecutivi
// vicini sono un pezzo dello stesso muro. Per la direzione a meno di 90 gradi si fa la media di 4 volte l'angolo dei tratti
// (così un tratto a 0, 90, 180 o 270 gradi conta allo stesso modo), pesata con la loro lunghezza. 4 volte l'angolo si
// ottiene senza funzioni trigonometriche: il versore del tratto è il numero complesso (c + i s), e la sua quarta potenza
// è il quadrato del quadrato
static void correct_theta_with_walls(int num_points) {

    // se il robot non si è spostato abbastanza dall'ultima correzione vede la stessa scena: niente correzione
    // (non vale per la prima, quella che prende l'angolo dai muri all'avvio)
    if (!map_state.is_initial_theta_pending &&
        hypot(map_state.aligned_x - map_state.wall_last_x, map_state.aligned_y - map_state.wall_last_y) < WALL_MIN_MOVE_M) return;

    // se il robot ruota troppo veloce i punti del giro sono deformati: niente correzione
    const lidar_giro_t *scan = &map_state.last_scan;
    double scan_seconds = (scan->t_fine_ns - scan->t_inizio_ns) / 1e9;
    double start_x, start_y, start_theta, end_x, end_y, end_theta;
    pose_at_time(scan->t_inizio_ns, &start_x, &start_y, &start_theta);
    pose_at_time(scan->t_fine_ns, &end_x, &end_y, &end_theta);
    if (scan_seconds <= 0 || fabs(wrap_angle(end_theta - start_theta)) / scan_seconds > WALL_MAX_OMEGA) return;

    // somme pesate di 4 volte l'angolo dei tratti (parte reale e immaginaria) e lunghezza totale dei tratti
    double sum_c = 0, sum_s = 0, sum_length = 0;

    for (int i = 1; i < num_points; i++) {

        const align_point_t *a = &map_state.lidar_point[i - 1];
        const align_point_t *b = &map_state.lidar_point[i];

        // scarta i punti lontani, dove il rumore del lidar è più alto
        if (hypot(a->x, a->y) > WALL_MAX_RANGE_M || hypot(b->x, b->y) > WALL_MAX_RANGE_M) continue;

        // il tratto fra i due punti: se sono troppo lontani non sono lo stesso muro, se sono troppo vicini è un punto doppio
        double dx = b->x - a->x, dy = b->y - a->y;
        double length = hypot(dx, dy);
        if (length < WALL_MIN_SEGMENT_M || length > WALL_NEAR_POINTS_M) continue;

        // versore del tratto, poi il suo quadrato (2 volte l'angolo) e il quadrato del quadrato (4 volte l'angolo)
        double c = dx / length, s = dy / length;
        double c2 = c * c - s * s, s2 = 2 * c * s;
        double c4 = c2 * c2 - s2 * s2, s4 = 2 * c2 * s2;

        sum_c += length * c4;
        sum_s += length * s4;
        sum_length += length;
    }

    // troppi pochi tratti, oppure muri senza una direzione netta (stanza piena di oggetti storti): niente correzione
    if (sum_length < WALL_MIN_LENGTH_M || hypot(sum_c, sum_s) / sum_length < WALL_MIN_STRENGTH) return;

    // direzione dei muri rispetto al robot, a meno di 90 gradi, poi nella mappa: deve essere 0 a meno di 90 gradi.
    // wrap_angle di 4 volte l'angolo riporta l'errore fra -45 e +45 gradi
    double walls_in_robot = atan2(sum_s, sum_c) / 4;
    double error = wrap_angle(4 * (walls_in_robot + map_state.aligned_theta)) / 4;

    // la prima volta l'errore si toglie tutto: l'aggancio alla base può essere ruotato anche di 20 gradi, e
    // fino a 45 non c'è ambiguità. La base sta contro un muro, quindi i muri si vedono sempre
    if (map_state.is_initial_theta_pending) {
        map_state.aligned_theta = wrap_angle(map_state.aligned_theta - error);
        map_state.is_initial_theta_pending = false;
        map_state.wall_last_x = map_state.aligned_x;
        map_state.wall_last_y = map_state.aligned_y;
        return;
    }

    // un errore troppo grande è più probabilmente un muro storto che l'IMU: niente correzione
    if (fabs(error) > WALL_MAX_ERROR) return;

    // correggi una piccola parte dell'errore, e ricorda dove è stata fatta la correzione
    map_state.aligned_theta = wrap_angle(map_state.aligned_theta - WALL_GAIN * error);
    map_state.wall_last_x = map_state.aligned_x;
    map_state.wall_last_y = map_state.aligned_y;
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

        // prima l'angolo, con la direzione dei muri, così i passi su x e y usano già quello corretto
        correct_theta_with_walls(num_points);

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
    stuck_history_entry_t last_stuck_entry = map_state.stuck_history[idx];      // la scansione di STUCK_WINDOW_SCANS giri fa

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
    publish_correction(is_stuck);
}

// funzione helper che calcola lo stato di una cella di navigazione dalle celle fini, senza guardare gli stati che si
// conservano (proibita, troppo stretta): bloccata se tutti e 4 i quadranti sono bloccati, libera se abbastanza celle fini
// sono state osservate, altrimenti sconosciuta
static mapping_nav_cell_state calc_nav_cell_state(int nav_index, uint8_t cell_mask) {

    if (cell_mask == NAV_CELL_QUAD_MASK) return NAV_CELL_BLOCKED;

    if (map_state.num_observed_fine_cells[nav_index] >= UNKNOWN_CELL_THRESHOLD) return NAV_CELL_FREE;

    return NAV_CELL_UNKNOWN;
}

// funzione helper che riporta una cella fine con un valore speciale (vuoto o ostacolo basso) a sconosciuta: è l'inverso
// di set_fine_cell_special_value, che l'aveva contata fra le osservate e fra le occupate
static void clear_fine_cell_special_value(const uint32_t cell_index) {

    int x_fine = cell_index & 0x3FF;
    int y_fine = cell_index >> 10;
    int nav_index = (x_fine >> 2) | ((y_fine >> 2) << 8);

    map_state.fine_grid[cell_index] = 0;
    map_state.num_observed_fine_cells[nav_index]--;
    update_occupied_cells(x_fine, y_fine, -1);
}

// funzione helper che riparte da una posizione nuova dopo un sollevamento: la posizione sulla mappa (map_x, map_y, map_theta)
// corrisponde all'odometria (odom_x, odom_y, odom_theta)
static void restart_from_pose(double odom_x, double odom_y, double odom_theta, double map_x, double map_y, double map_theta, mapping_map_state next_state) {

    // la stima allineata è la posizione nuova, e l'ultima odometria è quella in cui la posizione vale
    // così lo spostamento e la rotazione misurati in aria non contano
    map_state.aligned_x = map_x;
    map_state.aligned_y = map_y;
    map_state.aligned_theta = map_theta;
    map_state.aligned_valid = true;
    map_state.last_odom_x = odom_x;
    map_state.last_odom_y = odom_y;
    map_state.last_odom_theta = odom_theta;

    // azzera lo storico dell'incastro
    map_state.stuck_history_count = 0;

    // le celle fra il punto del sollevamento e quello del riappoggio non sono state attraversate
    map_state.last_mark.is_valid = false;

    // gli eventi dei sensori arrivati mentre il robot era in aria non valgono, salta direttamente all'ultimo
    drv_contatti_t last_events = {0};
    drv_contatti(&last_events);
    map_state.last_event = last_events.ultimo;

    // da normale la mappa ricomincia a cambiare, quindi la griglia della ricerca andrà rifatta
    if (next_state == MAP_NORMAL) map_state.search.grid_ready = false;

    map_state.state = next_state;

    publish_correction(false);
}

// Funzione helper che lancia il guasto nel caso in cui la routine di ricerca non trovi la nuova posizione del robot nella mappa
static void give_up_search(void) {

    map_state.state = MAP_NOT_FOUND;

    publish_correction(false);

    drv_segnala_guasto(DRV_FAULT_POSITION_NOT_FOUND);
}

// funzione helper che conta un tentativo fallito (ricerca senza risultato o conferma non riuscita): se ne restano,
// torna a cercare con una scansione nuova, altrimenti si arrende
static void count_failed_attempt(void) {

    map_state.search.attempts++;

    if (map_state.search.attempts >= SEARCH_MAX_ATTEMPTS) {
        give_up_search();
        return;
    }

    map_state.state = MAP_LOST;
    map_state.search.phase = SEARCH_WAIT_SCAN;
    map_state.search.put_down_t = rt_adesso();

    publish_correction(false);
}

// funzione helper che legge uno dei tre livelli di una cella della griglia della ricerca (shift è uno dei SEARCH_*_SHIFT)
static inline int get_search_level(uint16_t cell, int shift) {
    return (cell >> shift) & 7;
}

// funzione helper che calcola il livello 0 di una cella della griglia della ricerca partendo dalla cella fine (fine_x, fine_y)
static uint16_t calc_search_cell(int fine_x, int fine_y) {

    int near = SEARCH_LEVEL_NEVER, occupied = SEARCH_LEVEL_NEVER, free = SEARCH_LEVEL_NEVER;

    int8_t score = map_state.fine_grid[fine_y << 10 | fine_x];

    // un ostacolo visto dal lidar
    if (score >= 1 && score <= FINE_GRID_MAX_SCORE) occupied = 0;

    // una cella libera: il lidar ci è passato attraverso (ad eccezione del vuoto perchè il robot non ci può stare)
    if (score < 0 && score != VOID_OBSTACLE) free = 0;

    // la vicina di un ostacolo: la cella o una delle 8 attorno è un ostacolo. Serve a tollerare il rumore del lidar
    for (int dy = -1; dy <= 1; dy++) {
        for (int dx = -1; dx <= 1; dx++) {
            int x = fine_x + dx, y = fine_y + dy;
            if (x < 0 || y < 0 || x >= FINE_MAP_CELL_NUMBER || y >= FINE_MAP_CELL_NUMBER) continue;
            int8_t near_score = map_state.fine_grid[y << 10 | x];
            if (near_score >= 1 && near_score <= FINE_GRID_MAX_SCORE) near = 0;
        }
    }

    return (uint16_t)(near << SEARCH_NEAR_SHIFT | occupied << SEARCH_OCC_SHIFT | free << SEARCH_FREE_SHIFT);
}

// funzione helper che sceglie la zona della ricerca: tutte le celle di navigazione già osservate, con una cella fine di margine,
// al massimo SEARCH_MAX_CELLS per lato attorno all'ultima posizione nota. Restituisce false se la mappa è vuota
static bool set_search_area(void) {

    int x_min = FINE_MAP_CELL_NUMBER, x_max = -1, y_min = FINE_MAP_CELL_NUMBER, y_max = -1;

    // scorri le celle di navigazione
    for (int i = 0; i < NAV_MAP_CELL_NUMBER * NAV_MAP_CELL_NUMBER; i++) {

        if (map_state.num_observed_fine_cells[i] == 0) continue;

        int x = (i & 0xFF) * FINE_CELL_IN_NAV_CELL, y = (i >> 8) * FINE_CELL_IN_NAV_CELL;

        if (x < x_min) x_min = x;
        if (y < y_min) y_min = y;
        if (x + FINE_CELL_IN_NAV_CELL - 1 > x_max) x_max = x + FINE_CELL_IN_NAV_CELL - 1;
        if (y + FINE_CELL_IN_NAV_CELL - 1 > y_max) y_max = y + FINE_CELL_IN_NAV_CELL - 1;
    }

    if (x_max < 0) return false;

    // una cella di margine, senza uscire dalla griglia fine
    x_min = x_min > 0 ? x_min - 1 : 0;
    y_min = y_min > 0 ? y_min - 1 : 0;
    x_max = x_max < FINE_MAP_CELL_NUMBER - 1 ? x_max + 1 : x_max;
    y_max = y_max < FINE_MAP_CELL_NUMBER - 1 ? y_max + 1 : y_max;

    // se la zona è troppo grande, tieni il pezzo attorno all'ultima posizione nota
    uint32_t last_cell = coord_to_fine_index(map_state.aligned_x, map_state.aligned_y);
    int last_x = last_cell & 0x3FF, last_y = last_cell >> 10;

    if (x_max - x_min + 1 > SEARCH_MAX_CELLS) {
        x_min = last_x - SEARCH_MAX_CELLS / 2;
        if (x_min < 0) x_min = 0;
        if (x_min > FINE_MAP_CELL_NUMBER - SEARCH_MAX_CELLS) x_min = FINE_MAP_CELL_NUMBER - SEARCH_MAX_CELLS;
        x_max = x_min + SEARCH_MAX_CELLS - 1;
    }
    if (y_max - y_min + 1 > SEARCH_MAX_CELLS) {
        y_min = last_y - SEARCH_MAX_CELLS / 2;
        if (y_min < 0) y_min = 0;
        if (y_min > FINE_MAP_CELL_NUMBER - SEARCH_MAX_CELLS) y_min = FINE_MAP_CELL_NUMBER - SEARCH_MAX_CELLS;
        y_max = y_min + SEARCH_MAX_CELLS - 1;
    }

    map_state.search.x0 = x_min;
    map_state.search.y0 = y_min;
    map_state.search.width = x_max - x_min + 1;
    map_state.search.height = y_max - y_min + 1;

    return true;
}

// FUNZIONE 1 della ricerca: prepara la griglia della ricerca, al massimo SEARCH_BUILD_CELLS_PER_JOB celle per job.
// Restituisce true quando ha finito. Il livello 0 viene dalla griglia fine; il livello h di una cella si ricava dal livello h - 1
// delle 4 celle (x, y), (x + s, y), (x, y + s), (x + s, y + s) con s = 2^(h-1), perché i loro blocchi messi insieme formano il
// blocco da 2^h. Si può fare sulla stessa griglia: in questo passaggio si scrive solo il valore h, che non cambia la lettura "< h"
static bool build_search_grid(void) {

    int processed_cells = 0;

    while (map_state.search.build_level <= SEARCH_LEVELS) {

        int level = map_state.search.build_level;
        int step = 1 << (level > 0 ? level - 1 : 0);

        for (; map_state.search.build_row < map_state.search.height; map_state.search.build_row++) {

            // finito il budget di questo job, si continua dalla stessa riga al prossimo
            if (processed_cells >= SEARCH_BUILD_CELLS_PER_JOB) return false;

            int y = map_state.search.build_row;

            for (int x = 0; x < map_state.search.width; x++) {

                uint16_t *cell = &map_state.search.grid[y * map_state.search.width + x];

                if (level == 0) {
                    *cell = calc_search_cell(map_state.search.x0 + x, map_state.search.y0 + y);
                    continue;
                }

                // le 3 celle che con questa formano il blocco del livello h (quelle fuori dalla zona non contano)
                int near_x[3] = { x + step, x, x + step };
                int near_y[3] = { y, y + step, y + step };

                const int shifts[3] = { SEARCH_NEAR_SHIFT, SEARCH_OCC_SHIFT, SEARCH_FREE_SHIFT };

                for (int k = 0; k < 3; k++) {

                    // già vero a un livello più basso: resta così
                    if (get_search_level(*cell, shifts[k]) < level) continue;

                    for (int j = 0; j < 3; j++) {

                        if (near_x[j] >= map_state.search.width || near_y[j] >= map_state.search.height) continue;

                        uint16_t near_cell = map_state.search.grid[near_y[j] * map_state.search.width + near_x[j]];

                        // se una delle 3 lo contiene al livello h - 1, il blocco del livello h lo contiene
                        if (get_search_level(near_cell, shifts[k]) < level) {
                            *cell = (uint16_t)((*cell & ~(7u << shifts[k])) | (unsigned)level << shifts[k]);
                            break;
                        }
                    }
                }
            }

            processed_cells += map_state.search.width;
        }

        // livello finito, passa al prossimo
        map_state.search.build_level++;
        map_state.search.build_row = 0;
    }

    return true;
}

// FUNZIONE 2 della ricerca: sceglie i punti della scansione da usare fra i num_lidar_points di map_state.lidar_point (già
// rispetto al robot): solo quelli entro SEARCH_MAX_RANGE, e uno solo ogni SEARCH_POINT_SPACING (i punti sono in ordine di angolo,
// quindi basta la distanza dall'ultimo tenuto). Meno punti vuol dire ricerca più veloce, con la stessa precisione
static void prepare_search_points(int num_lidar_points) {

    map_state.search.num_points = 0;

    for (int i = 0; i < num_lidar_points && map_state.search.num_points < SEARCH_MAX_POINTS; i++) {

        align_point_t p = map_state.lidar_point[i];

        if (hypot(p.x, p.y) > SEARCH_MAX_RANGE) continue;

        if (map_state.search.num_points > 0) {
            align_point_t last = map_state.search.point[map_state.search.num_points - 1];
            if (hypot(p.x - last.x, p.y - last.y) < SEARCH_POINT_SPACING) continue;
        }

        map_state.search.point[map_state.search.num_points++] = p;
    }
}

// funzione helper che calcola il limite superiore del punteggio di un blocco di posizioni (x, y, level) per l'angolo corrente:
// ogni punto legge, al livello del blocco, la cella in cui cade quando il centro del robot è nella cella (x, y). Il valore di
// quella cella al livello h è il massimo su tutto il blocco che parte da lì, cioè su tutte le posizioni del blocco del robot
static int calc_search_bound(int x, int y, int level) {

    int size = 1 << level;
    int bound = 0;

    for (int i = 0; i < map_state.search.num_points; i++) {

        int cell_x = x + map_state.search.offset_x[i];
        int cell_y = y + map_state.search.offset_y[i];

        // se il blocco del punto inizia prima della zona ma ci entra, il suo pezzo dentro la zona sta nel blocco della cella 0
        if (cell_x < 0) {
            if (cell_x + size <= 0) continue;
            cell_x = 0;
        }
        if (cell_y < 0) {
            if (cell_y + size <= 0) continue;
            cell_y = 0;
        }

        if (cell_x >= map_state.search.width || cell_y >= map_state.search.height) continue;

        uint16_t cell = map_state.search.grid[cell_y * map_state.search.width + cell_x];

        bound += (level >= get_search_level(cell, SEARCH_NEAR_SHIFT)) + (level >= get_search_level(cell, SEARCH_OCC_SHIFT));
    }

    return bound;
}

// funzione helper che mette nella pila i blocchi dati, il migliore in cima. Scarta quelli che non possono battere il migliore
// e quelli senza nemmeno una cella libera (il centro del robot non ci può stare). Restituisce le letture fatte
static int push_search_nodes(const int *x, const int *y, int count, int level) {

    search_node_t nodes[64];
    int num_nodes = 0;

    for (int i = 0; i < count; i++) {

        if (x[i] >= map_state.search.width || y[i] >= map_state.search.height) continue;

        if (get_search_level(map_state.search.grid[y[i] * map_state.search.width + x[i]], SEARCH_FREE_SHIFT) > level) continue;

        int bound = calc_search_bound(x[i], y[i], level);

        if (bound <= map_state.search.best_score) continue;

        // ordinamento per inserimento, dal limite più alto al più basso (sono al massimo 64)
        int j = num_nodes++;
        while (j > 0 && nodes[j - 1].bound < bound) {
            nodes[j] = nodes[j - 1];
            j--;
        }
        nodes[j] = (search_node_t){ .x = (int16_t)x[i], .y = (int16_t)y[i], .level = (uint8_t)level, .bound = (int16_t)bound };
    }

    // si mettono dal peggiore al migliore, così il migliore resta in cima. La pila non si può riempire (vedi SEARCH_STACK_SIZE)
    for (int i = num_nodes - 1; i >= 0 && map_state.search.stack_len < SEARCH_STACK_SIZE; i--) {
        map_state.search.stack[map_state.search.stack_len++] = nodes[i];
    }

    return count * map_state.search.num_points;
}

// funzione helper che passa all'angolo successivo: ruota i punti e mette nella pila i blocchi più grandi (le radici)
static int start_search_angle(void) {

    double theta = map_state.search.angle * 2 * M_PI / SEARCH_NUM_ANGLES;
    double cos_theta = cos(theta), sin_theta = sin(theta);

    // con il centro del robot al centro di una cella, un punto cade nella cella spostata di round(coordinata / lato della cella)
    for (int i = 0; i < map_state.search.num_points; i++) {
        double x = map_state.search.point[i].x * cos_theta - map_state.search.point[i].y * sin_theta;
        double y = map_state.search.point[i].x * sin_theta + map_state.search.point[i].y * cos_theta;
        map_state.search.offset_x[i] = (int16_t)floor(x / FINE_MAP_CELL_SIZE + 0.5);
        map_state.search.offset_y[i] = (int16_t)floor(y / FINE_MAP_CELL_SIZE + 0.5);
    }

    int root_x[64], root_y[64], num_roots = 0;
    int size = 1 << SEARCH_LEVELS;

    for (int y = 0; y < map_state.search.height && num_roots < 64; y += size) {
        for (int x = 0; x < map_state.search.width && num_roots < 64; x += size) {
            root_x[num_roots] = x;
            root_y[num_roots] = y;
            num_roots++;
        }
    }

    return push_search_nodes(root_x, root_y, num_roots, SEARCH_LEVELS);
}

// FUNZIONE 3 della ricerca: ramificazione e potatura, un angolo alla volta, al massimo SEARCH_LOOKUPS_PER_JOB letture per job.
// Restituisce true quando ha guardato tutti gli angoli. Un blocco si divide in 4 solo se il suo limite supera il miglior punteggio
// trovato: così trova esattamente la posizione migliore fra tutte quelle provate, come se le guardasse tutte
static bool search_step(void) {

    int lookups = 0;

    while (lookups < SEARCH_LOOKUPS_PER_JOB) {

        // finito l'angolo corrente, passa al prossimo
        if (map_state.search.stack_len == 0) {

            map_state.search.angle++;

            if (map_state.search.angle >= SEARCH_NUM_ANGLES) return true;

            lookups += start_search_angle();
            continue;
        }

        search_node_t node = map_state.search.stack[--map_state.search.stack_len];

        // nel frattempo si è trovato di meglio
        if (node.bound <= map_state.search.best_score) continue;

        // una posizione sola: il limite è il suo punteggio, ed è il migliore finora
        if (node.level == 0) {
            map_state.search.best_score = node.bound;
            map_state.search.best_angle = map_state.search.angle;
            map_state.search.best_x = node.x;
            map_state.search.best_y = node.y;
            continue;
        }

        // dividi il blocco nei suoi 4 quarti
        int half = 1 << (node.level - 1);
        int child_x[4] = { node.x, node.x + half, node.x, node.x + half };
        int child_y[4] = { node.y, node.y, node.y + half, node.y + half };

        lookups += push_search_nodes(child_x, child_y, 4, node.level - 1);
    }

    return false;
}

// funzione helper che somma i punteggi della griglia fine (interpolati, come l'allineamento) nei punti della ricerca,
// con il robot in (x, y, theta)
static double calc_bilinear_score(double x, double y, double theta) {

    double cos_theta = cos(theta), sin_theta = sin(theta), total = 0;

    for (int i = 0; i < map_state.search.num_points; i++) {
        double score, grad_x, grad_y;
        double px = x + map_state.search.point[i].x * cos_theta - map_state.search.point[i].y * sin_theta;
        double py = y + map_state.search.point[i].x * sin_theta + map_state.search.point[i].y * cos_theta;
        read_fine_grid_bilinear(px, py, &score, &grad_x, &grad_y);
        total += score;
    }

    return total;
}

// FUNZIONE 4 della ricerca: rifinisce la posizione trovata, che ha l'errore di mezza cella e mezzo passo d'angolo.
// Per ogni angolo vicino fa i passi dell'allineamento su x e y, e tiene la posizione col punteggio più alto
static void refine_search_result(double *x, double *y, double *theta) {

    double best_score = -1, best_x = *x, best_y = *y, best_theta = *theta;

    for (double delta = -SEARCH_REFINE_ANGLE; delta <= SEARCH_REFINE_ANGLE + 1e-9; delta += SEARCH_REFINE_ANGLE_STEP) {

        double cur_x = *x, cur_y = *y, cur_theta = *theta + delta;

        for (int step = 0; step < ALIGN_STEPS; step++) {
            align_scan_step(map_state.search.point, map_state.search.num_points, &cur_x, &cur_y, cur_theta);
        }

        double score = calc_bilinear_score(cur_x, cur_y, cur_theta);

        if (score > best_score) {
            best_score = score;
            best_x = cur_x;
            best_y = cur_y;
            best_theta = cur_theta;
        }
    }

    *x = best_x;
    *y = best_y;
    *theta = wrap_angle(best_theta);
}

// funzione helper che calcola il punteggio della ricerca (2 su un ostacolo, 1 accanto) con il robot in (x, y, theta) sulla mappa.
// Serve alla conferma: è lo stesso punteggio con cui la ricerca ha scelto la posizione
static int calc_search_score(double x, double y, double theta) {

    double cos_theta = cos(theta), sin_theta = sin(theta);
    int score = 0;

    for (int i = 0; i < map_state.search.num_points; i++) {

        double px = x + map_state.search.point[i].x * cos_theta - map_state.search.point[i].y * sin_theta;
        double py = y + map_state.search.point[i].x * sin_theta + map_state.search.point[i].y * cos_theta;

        uint32_t index = coord_to_fine_index(px, py);
        int cell_x = (int)(index & 0x3FF) - map_state.search.x0;
        int cell_y = (int)(index >> 10) - map_state.search.y0;

        if (cell_x < 0 || cell_y < 0 || cell_x >= map_state.search.width || cell_y >= map_state.search.height) continue;

        uint16_t cell = map_state.search.grid[cell_y * map_state.search.width + cell_x];

        score += (get_search_level(cell, SEARCH_NEAR_SHIFT) == 0) + (get_search_level(cell, SEARCH_OCC_SHIFT) == 0);
    }

    return score;
}

// funzione helper che dice se un punteggio della ricerca supera la soglia SEARCH_MIN_SCORE_PCT (il massimo è 2 per punto)
static bool is_search_score_enough(int score) {
    return map_state.search.num_points >= SEARCH_MIN_POINTS && score * 100 >= SEARCH_MIN_SCORE_PCT * 2 * map_state.search.num_points;
}

// funzione chiamata dal task aperiodico (gira nel suo thread, sul core 3): fa tutta la ricerca della posizione
// e ne pubblica il risultato. PRESTITO: dalla richiesta al risultato, la mappa (griglia fine) e i dati della ricerca
// (map_state.search: punti, posizione di riferimento, griglia della ricerca) li usa solo questa funzione. La mappatura non
// li tocca: con la posizione persa non scrive sulla mappa (il job esce subito dopo relocate), e aspetta il risultato
// prima di preparare una ricerca nuova. Fra un pezzo e l'altro controlla se la ricerca va annullata (search_cancel_num)
void run_position_search(uint32_t request_num) {

    search_result_t result = { .request_num = request_num, .outcome = SEARCH_NOT_FOUND };

    // annullata prima ancora di partire
    if (atomic_load(&search_cancel_num) == request_num) {
        result.outcome = SEARCH_ABORTED;
        scambio_scrivi(&double_buffer_search, &result);
        return;
    }

    // la griglia della ricerca si rifà solo se la mappa è cambiata dall'ultima ricerca
    if (!map_state.search.grid_ready) {

        // mappa vuota: non c'è niente in cui cercare
        if (!set_search_area()) {
            result.outcome = SEARCH_NO_AREA;
            scambio_scrivi(&double_buffer_search, &result);
            return;
        }

        map_state.search.build_level = 0;
        map_state.search.build_row = 0;
        while (!build_search_grid()) {
            if (atomic_load(&search_cancel_num) == request_num) {
                result.outcome = SEARCH_ABORTED;
                scambio_scrivi(&double_buffer_search, &result);
                return;
            }
        }
        map_state.search.grid_ready = true;
    }

    // prepara la ricerca: si parte dall'angolo -1, così il primo passo apre l'angolo 0
    map_state.search.angle = -1;
    map_state.search.stack_len = 0;
    map_state.search.best_score = (SEARCH_MIN_SCORE_PCT * 2 * map_state.search.num_points + 99) / 100 - 1;
    map_state.search.best_angle = -1;

    // ramificazione e potatura, un pezzo alla volta
    while (!search_step()) {
        if (atomic_load(&search_cancel_num) == request_num) {
            result.outcome = SEARCH_ABORTED;
            scambio_scrivi(&double_buffer_search, &result);
            return;
        }
    }

    // posizione trovata: il centro della cella e l'angolo provato, poi la rifinitura
    if (map_state.search.best_angle >= 0) {
        result.x = (map_state.search.x0 + map_state.search.best_x - FINE_MAP_ORIGIN + 0.5) * FINE_MAP_CELL_SIZE;
        result.y = (map_state.search.y0 + map_state.search.best_y - FINE_MAP_ORIGIN + 0.5) * FINE_MAP_CELL_SIZE;
        result.theta = wrap_angle(map_state.search.best_angle * 2 * M_PI / SEARCH_NUM_ANGLES);
        refine_search_result(&result.x, &result.y, &result.theta);
        result.outcome = SEARCH_FOUND;
    }

    // la mappa torna alla mappatura
    scambio_scrivi(&double_buffer_search, &result);
}

// funzione che cerca la posizione del robot dopo il riappoggio (stato MAP_LOST): prepara i punti, chiede la ricerca al
// task aperiodico e ne usa il risultato
static void relocate(const movement_position_data_t *cur_pos, const drv_stato_t *cur_drv_state, bool is_same_scan) {

    // un tasto premuto con la posizione persa (la navigazione fa l'aggancio alla cieca): la ricerca si annulla e si aspetta
    // che il robot sia sulla base. Se la ricerca sta girando, la mappa si riprende solo quando arriva il suo risultato
    navigation_closed_cells_t closed_cells = read_closed_cells();
    if (closed_cells.stop_search_num != map_state.search.last_stop_num) {
        map_state.search.last_stop_num = closed_cells.stop_search_num;
        if (map_state.search.phase == SEARCH_WAIT_RESULT) {
            map_state.search.is_cancel_requested = true;
            map_state.search.is_stopped_by_user = true;
            atomic_store(&search_cancel_num, map_state.search.request_num);
        } else {
            map_state.search.phase = SEARCH_CANCELLED;
        }
    }

    switch (map_state.search.phase) {

        case SEARCH_WAIT_SCAN: {

            // riappoggiato sulla base: è lì per forza, e la base è l'origine della mappa
            if (read_started_on_base() && cur_drv_state->sulla_base) {
                restart_from_pose(cur_pos->odom_x, cur_pos->odom_y, cur_pos->odom_theta, 0, 0, 0, MAP_NORMAL);
                return;
            }

            // mappa ancora vuota: non c'è niente da ritrovare, si riparte da dove si è
            if (!map_state.aligned_valid) {
                restart_from_pose(cur_pos->odom_x, cur_pos->odom_y, cur_pos->odom_theta, cur_pos->odom_x, cur_pos->odom_y, cur_pos->odom_theta, MAP_NORMAL);
                return;
            }

            // serve una scansione nuova, iniziata dopo il riappoggio (da sollevato il lidar è spento)
            if (is_same_scan || map_state.last_scan.t_inizio_ns <= map_state.search.put_down_t) return;

            // la posizione dell'odometria alla fine della scansione, e i punti rispetto a quella posizione
            read_odom_history(&map_state.odom_history);
            pose_at_time(map_state.last_scan.t_fine_ns, &map_state.search.ref_x, &map_state.search.ref_y, &map_state.search.ref_theta);
            prepare_search_points(prepare_lidar_points(map_state.search.ref_x, map_state.search.ref_y, map_state.search.ref_theta));

            // pochi punti (per esempio il robot è contro un mobile): aspetta la prossima scansione
            if (map_state.search.num_points < SEARCH_MIN_POINTS) return;

            // la ricerca la fa il task aperiodico: da qui la mappa e i dati della ricerca gli sono prestati
            map_state.search.is_cancel_requested = false;
            map_state.search.is_stopped_by_user = false;
            map_state.search.request_num = request_position_search();
            map_state.search.phase = SEARCH_WAIT_RESULT;
            break;
        }
        case SEARCH_WAIT_RESULT: {

            // il risultato di questa ricerca non è ancora arrivato: la mappa è ancora prestata
            search_result_t result = {0};
            scambio_leggi(&double_buffer_search, &result);
            if (result.request_num != map_state.search.request_num) break;

            // ricerca annullata (anche se ha fatto in tempo a finire): da un tasto si aspetta la base, da un sollevamento
            // si cerca di nuovo dopo il riappoggio
            if (map_state.search.is_cancel_requested || result.outcome == SEARCH_ABORTED) {
                map_state.search.phase = map_state.search.is_stopped_by_user ? SEARCH_CANCELLED : SEARCH_WAIT_SCAN;
                break;
            }

            // mappa vuota: si riparte da dove si è
            if (result.outcome == SEARCH_NO_AREA) {
                restart_from_pose(cur_pos->odom_x, cur_pos->odom_y, cur_pos->odom_theta, cur_pos->odom_x, cur_pos->odom_y, cur_pos->odom_theta, MAP_NORMAL);
                break;
            }

            // nessuna posizione sopra la soglia
            if (result.outcome == SEARCH_NOT_FOUND) {
                count_failed_attempt();
                break;
            }

            // il robot riprende a lavorare, ma finché la posizione non è confermata la mappa non si scrive
            map_state.search.confirm_distance = 0;
            map_state.search.confirm_last_x = map_state.search.ref_x;
            map_state.search.confirm_last_y = map_state.search.ref_y;
            map_state.search.bad_scans = 0;

            restart_from_pose(map_state.search.ref_x, map_state.search.ref_y, map_state.search.ref_theta, result.x, result.y, result.theta, MAP_UNCONFIRMED);
            break;
        }
        case SEARCH_CANCELLED: {

            // la navigazione porta il robot alla base con l'aggancio a infrarossi: sulla base la posizione è l'origine
            if (read_started_on_base() && cur_drv_state->sulla_base) {
                restart_from_pose(cur_pos->odom_x, cur_pos->odom_y, cur_pos->odom_theta, 0, 0, 0, MAP_NORMAL);
            }
            break;
        }
    }
}

// funzione che conferma la posizione trovata dalla ricerca (stato MAP_UNCONFIRMED): a ogni scansione allinea come sempre e
// controlla che la scansione regga sulla mappa. Se regge per CONFIRM_DISTANCE di strada la posizione è confermata,
// se non regge per CONFIRM_MAX_BAD_SCANS scansioni di fila è sbagliata. Non scrive niente sulla mappa
static void confirm_position(const movement_position_data_t *cur_pos, bool is_same_scan) {

    // stesse condizioni della mappatura normale per usare una scansione
    if (is_same_scan || fabs(cur_pos->omega) >= ROTATION_SPEED_MAX_THRESHOLD) return;

    // allinea come sempre (pubblica anche la correzione)
    double ref_x, ref_y, ref_theta;
    pose_at_time(map_state.last_scan.t_fine_ns, &ref_x, &ref_y, &ref_theta);
    update_aligned_pose(ref_x, ref_y, ref_theta);

    // controlla la scansione con il punteggio della ricerca, nella posizione appena allineata. Se il robot ruota veloce la
    // scansione è deformata e non si controlla (l'allineamento sopra invece si fa comunque)
    if (fabs(cur_pos->omega) > CONFIRM_MAX_OMEGA) return;

    prepare_search_points(prepare_lidar_points(ref_x, ref_y, ref_theta));

    if (is_search_score_enough(calc_search_score(map_state.aligned_x, map_state.aligned_y, map_state.aligned_theta))) map_state.search.bad_scans = 0;
    else map_state.search.bad_scans++;

    // la posizione non regge: era sbagliata
    if (map_state.search.bad_scans >= CONFIRM_MAX_BAD_SCANS) {
        count_failed_attempt();
        return;
    }

    // somma la strada fatta dall'ultimo controllo
    map_state.search.confirm_distance += hypot(ref_x - map_state.search.confirm_last_x, ref_y - map_state.search.confirm_last_y);
    map_state.search.confirm_last_x = ref_x;
    map_state.search.confirm_last_y = ref_y;

    // ha retto per tutta la strada: posizione confermata, da qui si torna a scrivere la mappa
    if (map_state.search.confirm_distance >= CONFIRM_DISTANCE) {
        restart_from_pose(map_state.last_odom_x, map_state.last_odom_y, map_state.last_odom_theta,
                          map_state.aligned_x, map_state.aligned_y, map_state.aligned_theta, MAP_NORMAL);
    }
}

// funzione che, all'inizio di una nuova pulizia nella stessa accensione, azzera quello che durante una pulizia non si
// aggiorna più (le condizioni della stanza potrebbero essere cambiate): celle troppo strette, quadranti attraversati,
// celle del vuoto e degli ostacoli bassi. Ostacoli e celle libere restano: il lidar li aggiorna comunque.
// Lavora solo sulla zona conosciuta, la stessa della ricerca della posizione
static void reset_session_data(void) {

    // le celle chiuse la navigazione le ricomincia a contare da zero
    map_state.last_closed_num = 0;
    map_state.last_mark.is_valid = false;

    // la mappa cambia: la griglia della ricerca andrà rifatta
    map_state.search.grid_ready = false;

    if (!set_search_area()) return;

    // celle fini degli ostacoli bassi. Il vuoto invece resta
    for (int y = map_state.search.y0; y < map_state.search.y0 + map_state.search.height; y++) {
        for (int x = map_state.search.x0; x < map_state.search.x0 + map_state.search.width; x++) {
            int8_t score = map_state.fine_grid[y << 10 | x];
            if (score == LOW_OBSTACLE) clear_fine_cell_special_value(y << 10 | x);
        }
    }

    // celle di navigazione della zona (più una di margine per l'ingrossamento degli ostacoli)
    int nav_x_min = map_state.search.x0 / FINE_CELL_IN_NAV_CELL - 1, nav_x_max = (map_state.search.x0 + map_state.search.width) / FINE_CELL_IN_NAV_CELL + 1;
    int nav_y_min = map_state.search.y0 / FINE_CELL_IN_NAV_CELL - 1, nav_y_max = (map_state.search.y0 + map_state.search.height) / FINE_CELL_IN_NAV_CELL + 1;
    if (nav_x_min < 1) nav_x_min = 1;
    if (nav_y_min < 1) nav_y_min = 1;
    if (nav_x_max > NAV_MAP_CELL_NUMBER - 2) nav_x_max = NAV_MAP_CELL_NUMBER - 2;
    if (nav_y_max > NAV_MAP_CELL_NUMBER - 2) nav_y_max = NAV_MAP_CELL_NUMBER - 2;

    pthread_mutex_lock(&navigation_map_data.mutex);

    for (int nav_y = nav_y_min; nav_y <= nav_y_max; nav_y++) {
        for (int nav_x = nav_x_min; nav_x <= nav_x_max; nav_x++) {

            int nav_index = nav_y << 8 | nav_x;

            // togli il segno di quadrante attraversato
            for (int quad = 0; quad < 4; quad++) map_state.num_occupied_fine_cells[quad][nav_index] &= ~QUAD_CROSSED_FLAG;

            // ricalcola quadranti e stato: la proibita resta, la troppo stretta torna normale
            uint8_t cell_mask = calc_nav_cell_bitmask(nav_index);
            uint8_t cur_state = get_nav_cell_state(navigation_map_data.navigation_grid[nav_index]);
            uint8_t next_state = cur_state == NAV_CELL_FORBIDDEN ? NAV_CELL_FORBIDDEN : calc_nav_cell_state(nav_index, cell_mask);

            navigation_map_data.navigation_grid[nav_index] = next_state | cell_mask;
        }
    }

    navigation_map_data.version_number++;

    pthread_mutex_unlock(&navigation_map_data.mutex);
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

// Funzione helper che segna come proibite le celle di navigazione in cui il centro del robot
// farebbe entrare il corpo nella zona: quelle che toccano la zona allargata del raggio del robot.
// Lo stato proibito poi resta da solo, perché quando ricalcola gli stati la mappatura lo conserva
static void mark_forbidden_cells(const mapping_forbidden_zone_t zone) {

    const double radius = ROBOT_RADIUS / 1000.0;

    // calcola le celle dei due vertici opposti della zona allargata
    uint16_t cell_min = coord_to_nav_index(zone.x - radius, zone.y - radius);
    uint16_t cell_max = coord_to_nav_index(zone.x + zone.w + radius, zone.y + zone.h + radius);

    pthread_mutex_lock(&navigation_map_data.mutex);

    // scorri il rettangolo di celle: l'indice ha la y nel byte alto e la x in quello basso
    for (int cur_y = cell_min >> 8; cur_y <= cell_max >> 8; cur_y++) {
        for (int cur_x = cell_min & 0xFF; cur_x <= (cell_max & 0xFF); cur_x++) {

            int cur_index = cur_y << 8 | cur_x;

            // cambia solo lo stato, i bit dei quadranti restano quelli di prima
            navigation_map_data.navigation_grid[cur_index] = set_nav_cell_state(navigation_map_data.navigation_grid[cur_index], NAV_CELL_FORBIDDEN);
        }
    }

    // la navigazione deve prendere la mappa nuova
    navigation_map_data.version_number++;

    pthread_mutex_unlock(&navigation_map_data.mutex);
}

// funzione helper inversa di mark_forbidden_cells: le celle proibite della zona (allargata del raggio del robot) tornano
// allo stato calcolato dalla mappa. Le zone che restano vanno segnate di nuovo dopo, perché possono toccare le stesse celle
static void unmark_forbidden_cells(const mapping_forbidden_zone_t zone) {

    const double radius = ROBOT_RADIUS / 1000.0;

    uint16_t cell_min = coord_to_nav_index(zone.x - radius, zone.y - radius);
    uint16_t cell_max = coord_to_nav_index(zone.x + zone.w + radius, zone.y + zone.h + radius);

    pthread_mutex_lock(&navigation_map_data.mutex);

    for (int cur_y = cell_min >> 8; cur_y <= cell_max >> 8; cur_y++) {
        for (int cur_x = cell_min & 0xFF; cur_x <= (cell_max & 0xFF); cur_x++) {

            int cur_index = cur_y << 8 | cur_x;
            if (get_nav_cell_state(navigation_map_data.navigation_grid[cur_index]) != NAV_CELL_FORBIDDEN) continue;

            uint8_t cell_mask = calc_nav_cell_bitmask(cur_index);
            navigation_map_data.navigation_grid[cur_index] = calc_nav_cell_state(cur_index, cell_mask) | cell_mask;
        }
    }

    navigation_map_data.version_number++;

    pthread_mutex_unlock(&navigation_map_data.mutex);
}

// Funzione helper che inserisce la zona proibita della base di ricarica nella lista
// delle zone proibite per il task di sicurezza e nelle celle della mappa per il task di navigazione
static void publish_base_zone(void) {

    mapping_forbidden_zone_list_t zone_list = {0};

    zone_list.forbidden_zones[0].x = RECHARGING_STATION_ZONE_X;
    zone_list.forbidden_zones[0].y = RECHARGING_STATION_ZONE_Y;
    zone_list.forbidden_zones[0].w = RECHARGING_STATION_ZONE_W;
    zone_list.forbidden_zones[0].h = RECHARGING_STATION_ZONE_H;
    zone_list.num_zones = 1;

    // dopo la base, le zone dell'utente caricate con la mappa
    for (int i = 0; i < map_state.num_user_zones; i++) zone_list.forbidden_zones[zone_list.num_zones++] = map_state.user_zones[i];

    write_forbidden_zone_list(&zone_list);

    for (int i = 0; i < zone_list.num_zones; i++) mark_forbidden_cells(zone_list.forbidden_zones[i]);
}

// funzione helper che applica le zone volute dall'utente (scritte dal task aperiodico): toglie le celle proibite delle
// zone vecchie, poi ripubblica la lista (la base, poi le zone nuove) e segna tutte le zone. Le zone hanno senso solo se
// la sessione è partita dalla base (la mappa ha l'origine lì): finché il controllo non è finito si aspetta
static void apply_user_zones(void) {

    mapping_user_zones_t wanted = {0};
    scambio_leggi(&double_buffer_zones, &wanted);

    if (wanted.version == map_state.user_zones_version || !read_started_on_base()) return;

    for (int i = 0; i < map_state.num_user_zones; i++) unmark_forbidden_cells(map_state.user_zones[i]);

    map_state.num_user_zones = wanted.num_zones;
    for (int i = 0; i < wanted.num_zones; i++) map_state.user_zones[i] = wanted.zones[i];

    publish_base_zone();

    map_state.user_zones_version = wanted.version;
    atomic_store(&applied_zones_version, wanted.version);
}

// Funzione helper che nei primi job controlla se il robot è partito dalla base di ricarica.
static void check_start_on_base(void) {

    // se il controllo è già finito esci
    if (map_state.base_check_jobs >= BASE_CHECK_JOBS) return;

    drv_stato_t cur_drv_state = {0};
    drv_stato(&cur_drv_state);

    // finché il driver non è pronto sulla_base non vale ancora niente
    if (cur_drv_state.fase != DRV_PRONTO) return;

    if (cur_drv_state.sulla_base) map_state.base_seen = true;

    map_state.base_check_jobs++;

    // se il controllo è finito e il robot è partito dalla base inserisci la zona proibita
    // il risultato va pubblicato dopo, così chi legge true trova la zona già pronta
    if (map_state.base_check_jobs == BASE_CHECK_JOBS && map_state.base_seen) {
        publish_base_zone();
        atomic_store(&started_on_base, true);
    }
}

// funzione helper che restituisce la categoria di una cella fine per il file della mappa (docs/formato-mappa.md)
static inline uint8_t fine_cell_category(int8_t score) {
    if (score == 0) return MAP_CAT_UNKNOWN;
    if (score == VOID_OBSTACLE) return MAP_CAT_VOID;
    if (score == LOW_OBSTACLE) return MAP_CAT_FREE;     // il lidar lì vede libero: alla sessione dopo lo ritrova il paraurti
    if (score > FINE_CELL_OCCUPATION_THRESHOLD) return MAP_CAT_OCCUPIED;
    return MAP_CAT_FREE;
}

// funzione chiamata dal task aperiodico che copia la griglia fine a categorie mentre la
// mappatura non sta lavorando. Il contatore dei job è dispari durante un job: se è dispari si aspetta, se cambia durante
// la copia la copia mescola due versioni e si rifà. La mappatura non aspetta mai
bool copy_map_categories(uint8_t *categories, int rect_x, int rect_y, int rect_width, int rect_height,
                         mapping_forbidden_zone_t *user_zones, int *num_user_zones) {

    // il rettangolo, in byte lungo x (4 celle per byte) e in righe lungo y
    const uint32_t bytes_per_row = FINE_MAP_CELL_NUMBER / 4;
    const uint32_t first_byte_x = (uint32_t)rect_x / 4;
    const uint32_t last_byte_x = (uint32_t)(rect_x + rect_width - 1) / 4;

    for (int attempt = 0; attempt < MAP_COPY_MAX_ATTEMPTS; attempt++) {

        uint32_t before = atomic_load(&map_job_counter);

        // un job è in corso: aspetta un po' e riprova
        if (before & 1) {
            rt_dormi_fino_a(rt_adesso() + MAP_COPY_RETRY_WAIT);
            continue;
        }

        // 4 celle per byte, la cella con indice più basso nei bit più bassi
        for (uint32_t y = (uint32_t)rect_y; y < (uint32_t)(rect_y + rect_height); y++) {
            for (uint32_t i = y * bytes_per_row + first_byte_x; i <= y * bytes_per_row + last_byte_x; i++) {
                const int8_t *cell = &map_state.fine_grid[i * 4];
                categories[i] = (uint8_t)(fine_cell_category(cell[0]) | fine_cell_category(cell[1]) << 2 |
                                          fine_cell_category(cell[2]) << 4 | fine_cell_category(cell[3]) << 6);
            }
        }

        // le zone dell'utente cambiano nel job della mappatura come la griglia: si copiano nella stessa finestra
        if (user_zones != NULL) {
            *num_user_zones = map_state.num_user_zones;
            for (int i = 0; i < map_state.num_user_zones; i++) user_zones[i] = map_state.user_zones[i];
        }

        // i dati letti vanno guardati prima di rileggere il contatore
        atomic_thread_fence(memory_order_acquire);
        if (atomic_load(&map_job_counter) == before) return true;
    }

    return false;
}

// funzione chiamata dal task aperiodico: scrive le zone volute dall'utente, che la mappatura applica nel suo job
void write_user_zones(const mapping_user_zones_t *user_zones) {
    scambio_scrivi(&double_buffer_zones, user_zones);
}

// funzione chiamata dal task aperiodico: versione delle zone volute che la mappatura ha già applicato
uint32_t read_applied_user_zones_version(void) {
    return atomic_load(&applied_zones_version);
}

// funzione helper che legge byte dal file della mappa e ne aggiorna il CRC. Ritorna false se il file finisce prima
static bool read_map_bytes(FILE *f, uint32_t *crc, uint8_t *data, size_t size) {
    if (fread(data, 1, size, f) != size) return false;
    *crc = map_crc32(*crc, data, size);
    return true;
}

// funzione helper che legge un uint16 little endian dal file della mappa
static bool read_map_u16(FILE *f, uint32_t *crc, uint16_t *value) {
    uint8_t bytes[2];
    if (!read_map_bytes(f, crc, bytes, 2)) return false;
    *value = (uint16_t)(bytes[0] | bytes[1] << 8);
    return true;
}

// funzione helper che legge il file della mappa e scrive le celle nella griglia fine. Fa tutti i
// controlli del formato: se uno fallisce la griglia torna vuota e ritorna false
static bool read_map_file(FILE *f, int *x0, int *y0, int *width, int *height) {

    uint32_t crc = 0;
    uint8_t magic[MAP_FILE_MAGIC_LEN];
    uint16_t x, y, w, h;
    uint8_t zones_byte;

    // header: magic number, rettangolo dentro la griglia, numero di zone
    bool ok = read_map_bytes(f, &crc, magic, MAP_FILE_MAGIC_LEN) && memcmp(magic, MAP_FILE_MAGIC, MAP_FILE_MAGIC_LEN) == 0;
    ok = ok && read_map_u16(f, &crc, &x) && read_map_u16(f, &crc, &y) && read_map_u16(f, &crc, &w) && read_map_u16(f, &crc, &h);
    ok = ok && w > 0 && h > 0 && x + w <= FINE_MAP_CELL_NUMBER && y + h <= FINE_MAP_CELL_NUMBER;
    ok = ok && read_map_bytes(f, &crc, &zones_byte, 1);

    // numero di zone: al massimo una in meno della lista, il primo posto è della base
    int num_zones = zones_byte;
    ok = ok && num_zones <= MAX_FORBIDDEN_ZONES - 1;

    // zone, da celle fini a metri
    mapping_forbidden_zone_t zones[MAX_FORBIDDEN_ZONES - 1];
    for (int i = 0; ok && i < num_zones; i++) {
        uint16_t zx = 0, zy = 0, zw = 0, zh = 0;
        ok = read_map_u16(f, &crc, &zx) && read_map_u16(f, &crc, &zy) && read_map_u16(f, &crc, &zw) && read_map_u16(f, &crc, &zh);
        ok = ok && zx + zw <= FINE_MAP_CELL_NUMBER && zy + zh <= FINE_MAP_CELL_NUMBER;
        zones[i].x = (zx - FINE_MAP_ORIGIN) * FINE_MAP_CELL_SIZE;
        zones[i].y = (zy - FINE_MAP_ORIGIN) * FINE_MAP_CELL_SIZE;
        zones[i].w = zw * FINE_MAP_CELL_SIZE;
        zones[i].h = zh * FINE_MAP_CELL_SIZE;
    }

    // celle a sequenze, riga dopo riga
    const uint32_t total = (uint32_t)w * h;
    uint32_t done = 0;
    while (ok && done < total) {
        uint8_t run;
        ok = read_map_bytes(f, &crc, &run, 1);
        uint32_t count = run & MAP_RUN_COUNT_MASK;
        uint8_t category = run >> MAP_RUN_CATEGORY_SHIFT;
        ok = ok && count > 0 && count <= total - done;
        for (uint32_t k = 0; ok && k < count; k++, done++) {
            int cell_x = x + (int)(done % w), cell_y = y + (int)(done / w);
            int8_t score = category == MAP_CAT_FREE ? LOAD_SCORE_FREE :
                           category == MAP_CAT_OCCUPIED ? LOAD_SCORE_OCCUPIED :
                           category == MAP_CAT_VOID ? VOID_OBSTACLE : 0;
            map_state.fine_grid[cell_y << 10 | cell_x] = score;
        }
    }

    // CRC, poi il file deve essere finito
    uint8_t crc_bytes[MAP_FILE_CRC_LEN];
    ok = ok && fread(crc_bytes, 1, MAP_FILE_CRC_LEN, f) == MAP_FILE_CRC_LEN;
    ok = ok && (uint32_t)(crc_bytes[0] | crc_bytes[1] << 8 | crc_bytes[2] << 16 | (uint32_t)crc_bytes[3] << 24) == crc;
    ok = ok && fgetc(f) == EOF;

    if (!ok) {
        memset(map_state.fine_grid, 0, sizeof map_state.fine_grid);
        return false;
    }

    for (int i = 0; i < num_zones; i++) map_state.user_zones[i] = zones[i];
    map_state.num_user_zones = num_zones;
    *x0 = x; *y0 = y; *width = w; *height = h;
    return true;
}

// funzione helper chiamata da init_mapping: se il robot è sulla base, carica la mappa attiva e ricalcola contatori e
// griglia di navigazione. La mappa ha l'origine nella base, quindi fuori dalla base non si carica la mappa ma si esplora come se la mappa non esistesse
static void load_map_if_on_base(void) {

    // aspetta il driver pronto, poi guarda per un po' se il robot è sulla base (il bit può essere intermittente)
    drv_stato_t drv_state = {0};
    int64_t limit = rt_adesso() + LOAD_BASE_WAIT;
    bool on_base = false;
    while (rt_adesso() < limit && !on_base) {
        drv_stato(&drv_state);
        if (drv_state.fase == DRV_PRONTO && drv_state.sulla_base) on_base = true;
        else rt_dormi_fino_a(rt_adesso() + RT_MS(50));
    }
    if (!on_base) {
        fprintf(stderr, "mappatura: il robot non è sulla base, la mappa non si carica\n");
        return;
    }

    char path[256];
    if (map_file_path(path, sizeof path, "") != 0) return;
    FILE *f = fopen(path, "rb");
    if (!f) {
        fprintf(stderr, "mappatura: nessuna mappa in %s, si esplora\n", path);
        return;
    }
    int x0, y0, width, height;
    bool ok = read_map_file(f, &x0, &y0, &width, &height);
    fclose(f);
    if (!ok) {
        fprintf(stderr, "mappatura: la mappa %s non supera i controlli, si esplora\n", path);
        return;
    }

    // contatori: celle osservate per cella di navigazione, celle occupate (anche il vuoto) attorno ai quadranti
    for (int y = y0; y < y0 + height; y++) {
        for (int x = x0; x < x0 + width; x++) {
            int8_t score = map_state.fine_grid[y << 10 | x];
            if (score == 0) continue;
            map_state.num_observed_fine_cells[(x >> 2) | ((y >> 2) << 8)]++;
            if (score > FINE_CELL_OCCUPATION_THRESHOLD || score == VOID_OBSTACLE) update_occupied_cells(x, y, 1);
        }
    }

    // griglia di navigazione del rettangolo, più una cella di margine per l'ingrossamento degli ostacoli (i bordi della griglia restano bloccati)
    int nav_x_min = x0 / FINE_CELL_IN_NAV_CELL - 1, nav_x_max = (x0 + width) / FINE_CELL_IN_NAV_CELL + 1;
    int nav_y_min = y0 / FINE_CELL_IN_NAV_CELL - 1, nav_y_max = (y0 + height) / FINE_CELL_IN_NAV_CELL + 1;
    if (nav_x_min < 1) nav_x_min = 1;
    if (nav_y_min < 1) nav_y_min = 1;
    if (nav_x_max > NAV_MAP_CELL_NUMBER - 2) nav_x_max = NAV_MAP_CELL_NUMBER - 2;
    if (nav_y_max > NAV_MAP_CELL_NUMBER - 2) nav_y_max = NAV_MAP_CELL_NUMBER - 2;

    pthread_mutex_lock(&navigation_map_data.mutex);
    for (int nav_y = nav_y_min; nav_y <= nav_y_max; nav_y++) {
        for (int nav_x = nav_x_min; nav_x <= nav_x_max; nav_x++) {
            int nav_index = nav_y << 8 | nav_x;
            uint8_t cell_mask = calc_nav_cell_bitmask(nav_index);
            navigation_map_data.navigation_grid[nav_index] = calc_nav_cell_state(nav_index, cell_mask) | cell_mask;
        }
    }
    navigation_map_data.version_number++;
    pthread_mutex_unlock(&navigation_map_data.mutex);

    fprintf(stderr, "mappatura: mappa caricata da %s (%d x %d celle, %d zone)\n", path, width, height, map_state.num_user_zones);
}

// lavoro di un job del task (lo chiama job_mapping, che ci mette attorno il contatore dei job)
static void job_mapping_body(void *arg) {
    
    (void)arg;

    // Leggi l'ultiima scansione del lidar
    uint32_t cur_scan_num = drv_giro(&map_state.last_scan);

    // Salvati se il numero di scansione è uguale all'ultimo giro
    bool is_same_scan = cur_scan_num == map_state.last_scan_num;

    // Se è una nuova scansione, aggiorna il numero di scansione
    if (!is_same_scan) map_state.last_scan_num = cur_scan_num;

    // Leggi la posizione attuale del robot, dal task movement
    movement_position_data_t cur_pos = read_movement();

    // Leggi lo stato del driver, serve a sapere se il robot è sollevato
    drv_stato_t cur_drv_state = {0};
    drv_stato(&cur_drv_state);

    // se il robot è sollevato la sua posizione è persa: pubblicalo una volta sola ed esci,
    // da sollevato non si scrive niente sulla mappa
    if (cur_drv_state.sollevato) {
        if (map_state.state != MAP_LOST) {
            map_state.state = MAP_LOST;
            map_state.search.attempts = 0;
            publish_correction(false);
        }

        // a ogni sollevamento la ricerca ricomincia. L'istante si aggiorna finché il robot è in aria,
        // così l'ultimo valore è quello del riappoggio. Se una ricerca sta girando nel task aperiodico si annulla, e si
        // ricomincia quando arriva il suo risultato (la mappa le è prestata fino ad allora). Se la ricerca era stata
        // annullata da un tasto il robot sta andando alla base, e resta così
        if (map_state.search.phase == SEARCH_WAIT_RESULT) {
            if (!map_state.search.is_cancel_requested) {
                map_state.search.is_cancel_requested = true;
                atomic_store(&search_cancel_num, map_state.search.request_num);
            }
        } else if (map_state.search.phase != SEARCH_CANCELLED) {
            map_state.search.phase = SEARCH_WAIT_SCAN;
        }
        map_state.search.put_down_t = rt_adesso();
        return;
    }

    // se il robot è stato riappoggiato, cerca la sua posizione
    if (map_state.state == MAP_LOST) {
        relocate(&cur_pos, &cur_drv_state, is_same_scan);
        return;
    }

    // posizione non ritrovata: il robot è in guasto e la mappa non si tocca. Quando il guasto viene azzerato (tasto premuto
    // o robot rimesso sulla base) si ricomincia a cercare da capo, e sulla base la posizione si ritrova subito
    if (map_state.state == MAP_NOT_FOUND) {
        if (cur_drv_state.fase != DRV_GUASTO) {
            map_state.state = MAP_LOST;
            map_state.search.attempts = 0;
            map_state.search.phase = SEARCH_WAIT_SCAN;
            map_state.search.put_down_t = rt_adesso();
            publish_correction(false);
        }
        return;
    }

    // Controlla se il robot è partito dalla base di ricarica
    check_start_on_base();

    // Leggi lo storico delle pose dell'odometria: serve a sapere dov'era il robot quando è stato misurato ogni punto
    read_odom_history(&map_state.odom_history);

    // posizione trovata dalla ricerca ma non ancora confermata: allinea e controlla, senza scrivere sulla mappa
    if (map_state.state == MAP_UNCONFIRMED) {
        confirm_position(&cur_pos, is_same_scan);
        return;
    }

    // se la navigazione ha fatto partire una nuova pulizia, azzera quello che in una pulizia non si aggiorna più
    {
        navigation_closed_cells_t closed_cells = read_closed_cells();
        if (closed_cells.mission_num != map_state.last_mission_num) {
            map_state.last_mission_num = closed_cells.mission_num;
            reset_session_data();
        }
    }

    // zone proibite volute dall'utente (dal task aperiodico)
    apply_user_zones();

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

        // vicino a un ostacolo la scansione serve solo per allinearsi
        bool write_scan = !is_near_obstacle(map_state.aligned_x, map_state.aligned_y);
        
        // Scorri tutti i punti della scansione del lidar e se sono != 0
        // Calcola la posizione del punto rispetto al robot nella mappa
        for (int i = 0; write_scan && i < map_state.last_scan.n; i++) {
            
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
            // altrimenti bloccata, libera o sconosciuta, dalle celle fini
            else next_state = calc_nav_cell_state(cur_nav_index, cur_cell_mask);
            

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

// job del task: il lavoro vero sta in job_mapping_body, qui attorno c'è il contatore dei job. Chiamarlo da una funzione
// a parte fa sì che nessun return anticipato del job possa saltare l'incremento finale
void job_mapping(void *arg) {

    // dispari: la mappa sta per cambiare
    atomic_fetch_add(&map_job_counter, 1);

    job_mapping_body(arg);

    // pari: la mappa non cambia fino al prossimo job
    atomic_fetch_add(&map_job_counter, 1);
}
