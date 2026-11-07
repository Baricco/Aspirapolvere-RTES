#include "driver/driver.h"
#include "driver/scambio.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>

#include "navigation.h"
#include "mapping.h"
#include "aperiodic.h"
#include "movement.h"
#include "safety.h"
#include "lidar.h"

#define DEG_TO_RAD(a) ((a) * M_PI / 180)

#define CELL_NOT_VISITED 0xFFFF
#define ESCAPE_MAX_CELLS 3
#define WAIT_TIME RT_MS(3000)
#define PASS_MAX_TURN DEG_TO_RAD(30)        // un punto del percorso si attraversa senza fermarsi se la svolta è sotto questo angolo
#define CROSS_ANGLE DEG_TO_RAD(30)          // attraversamento di tappeti e cavi: di quanto il robot si mette di traverso rispetto alla direzione di partenza, così una ruota sale prima dell'altra
#define CROSS_PUSH_TIME RT_MS(3000)         // attraversamento: per quanto il robot spinge dritto da ognuna delle due parti
#define CRUISING_SPEED 0.35                  // velocità di crociera del robot, m/s
#define LEAVE_BASE_SPEED 0.1                 // velocità con cui il robot esce dalla base, espressa in m/s
#define LEAVE_BASE_TIMEOUT RT_MS(15000)      // tempo massimo per uscire dalla base, scaduto il quale il robot va in guasto
#define MAX_ROTATION_SPEED  1.2              // massima velocità di rotazione, per stare sotto al soglia oltre il quale la mappatura scarta i giri (1,5) con un certo margine, rad/s
#define UNKNOWN_SPEED_FACTOR 0.5             // fattore da moltiplicare alla velocità quando il robot si muove in territorio sconosciuto


#define STALL_SPEED 0.02                // velocità massima in avanti sotto la quale la sicurezza sta trattenendo il robot, espressa in m/s
#define STALL_OMEGA 0.1                 // velocità angolare sotto la quale il robot non sta ruotando, espressa in rad/s
#define STALL_TIME RT_MS(1000)          // per quanto tempo deve durare lo stallo prima che venga considerato come tale


#define TRAIL_SIZE 8            // lunghezza del buffer delle ultime posizioni percorse, serve per permettere al robot di indietreggiare, nel caso di problemi
#define TRAIL_STEP 0.05         // distanza minima in metri tra le ultime posizioni salvate nel buffer        
#define UNLOCK_MAX_ATTEMPTS 10  // tentativi di sblocco prima di andare in guasto (ne mettiamo tanti, per essere sicuri che il robot tenti di uscire tante volte prima di fermarsi e arrendersi)

#define UNLOCK_PHASE_TIMEOUT RT_MS(5000)    // durata massima di ogni fase

// il task di navigazione ha celle 20x20cm l'idea è che per non avere strisce non pulite, ogni cella viene divisa in 
// due "corsie" durante la fase di pulizia, questo significa che ogni cella verrà divisa in due parti 10x20 
#define LANE_DEVIATION 0.05                 // scostamento di una corsia dal centro della cella, espresso in metri
#define LANE_CLEAN_DISTANCE_THRESHOLD 0.025 // la distanza entro cui una metà si segna come pulita, espresso in metri

// il rettangolo attorno al robot che si segna come pulito durante il giro dei bordi, espresso in metri
#define BORDER_CLEAN_LEFT 0.05              // a sinistra del centro: il lato verso la stanza, dove arriva il rullo
#define BORDER_CLEAN_RIGHT 0.1745           // a destra del centro: tutto il lato della spazzola laterale, fino al bordo del robot
#define BORDER_CLEAN_ALONG 0.10             // avanti e indietro rispetto al centro: i centri delle metà distano fino a 20 cm lungo le strisce
#define BORDER_DONE_DISTANCE 0.16           // distanza entro cui una metà pulita fa considerare pulito un quadrante, espressa in metri

#define BORDER_SPEED 0.15                   // velocità massima lungo il bordo, espressa in m/s
#define BORDER_ROTATION_GAIN 2.5            // guadagno usato per calcolare la velocità di rotazione dall'errore di direzione, espresso in 1/secondi
#define BORDER_MAX_OMEGA 1.0                // velocità massima di rotazione durante il giro dei bordi, espressa in rad/s
#define BORDER_SLOW_ANGLE 0.8               // errore di direzione da cui in su il robot non avanza e ruota soltanto, espresso in radianti
#define BORDER_HOOK_TOLERANCE 0.44          // errore entro cui l'ostacolo si considera a destra del robot (25°), espresso in radianti
#define BORDER_NO_NEW_DISTANCE 0.5          // strada senza mezze celle nuove dopo cui il giro finisce, espressa in metri
#define BORDER_LOST_TIME RT_MS(1000)        // tempo senza vedere l'ostacolo dopo cui il giro finisce
#define BORDER_LAP_TIMEOUT RT_MS(1800000)   // rete di sicurezza contro i giri infiniti (30 minuti, circa 225 m di contorno), non deve mai scattare
#define SCAN_HALF_TIME RT_MS(85)            // metà della durata di un giro del lidar

// le due estremità di ogni striscia si allungano fino al primo ostacolo, così il rullo pulisce la corsia da un capo all'altro
#define STRIP_END_FRONT_DISTANCE 30         // spazio da lasciare davanti al robot alle estremità della striscia, espresso in mm
#define STRIP_END_ZONE 0.22                 // distanza dal punto di estremità entro cui il robot può fermarsi senza proseguire la striscia, espressa in metri

#define NAV_PATH_NO_QUAD 0                  // valore da restituire se non si trova nessun quadrante nelle funzioni di ricerca    
#define NAV_PATH_CENTER 1                   // valore per indicare di raggiungere il centro di una cella, non un quadrante specifico

// costanti di velocità delle spazzole, espresse in percentuale
#define BRUSH_SPEED_OFF 0
#define BRUSH_SPEED_NORMAL 60
#define BRUSH_SPEED_MAX 100
#define BRUSH_SPEED_UNLOCK (-100)       // valore negativo per la spazzola centrale (velocità massima, verso opposto)

#define BRUSH_UNLOCK_TIME RT_MS(4500)           // durata dello sblocco della spazzola centrale, poi torna a girare normalmente
#define BRUSH_UNLOCK_MAX_ATTEMPTS 5             // numero massimo di sblocchi della spazzola centrale di fila: se si blocca ancora, il robot va in guasto
#define BRUSH_UNLOCK_RESET_TIME RT_MS(10000)    // tempo in cui, finito uno sblocco, la spazzola deve girare normalmente perchè lo sblocco sia riuscito

// costanti di potenza di aspirazione, espressa in giri al minuto
#define SUCTION_POWER_OFF 0
#define SUCTION_POWER_NORMAL 15500
#define SUCTION_POWER_MAX 19500

#define CHARGING_STATION_CELL coord_to_nav_index(AGG_PUNTO_X_M, 0)          // cella che si trova davanti alla base di ricarica 
#define NO_PATH_HOME_TIMEOUT RT_MS(30000)       // tempo per cui il ritorno alla base riprova a cercare il percorso (per esempio se una persona blocca il passaggio) prima che il robot vada in guasto
#define HOOK_UP_TIMEOUT RT_MS(30000)            // tempo massimo che può impiegare il robot ad agganciarsi alla base di ricarica prima che vada in stato di guasto

#define BATTERY_LOW_CONFIRM_TIME RT_S(20)       // la batteria deve restare bassa per tutto questo tempo prima del rientro, altrimenti un carico forte potrebbe farla scendere sotto la soglia per qualche istante, causando un rientro inutile
#define HOME_IR_DOCK_DIST 1.0                   // se la cella davanti alla base non si raggiunge, da questa distanza dal punto di aggancio (o meno) l'aggancio a infrarossi parte da dove si trova il robot, espressa in metri
#define BATTERY_RESUME_PERCENT 90               // percentuale di carica della batteria che deve raggiungere il robot prima che ricominci a pulire

#define ALIGN_TIMEOUT RT_MS(15000)              // tempo massimo che può durare l'allineamento al centro di una cella, prima che il robot vada in guasto

typedef enum {
    UNLOCK_PHASE_BACK,
    UNLOCK_PHASE_ROTATE,    
} unlock_phase_t;

// Un tentativo di sblocco: quanto indietreggiare sulla scia, di quanto ruotare 
// sul posto e a che velocità indietreggiare
typedef struct {
    double back_dist;       // distanza da percorrere all'indietro sulla scia, espressa in m
    double rot_angle;       // rotazione sul posto dopo l'indietreggiamento, positiva a sinistra (senso antiorario), espressa in rad
    double back_speed;      // velocità massima all'indietro, espressa in m/s
} unlock_step_t;

// Tabella dei tentativi di sblocco, ogni tentativo è diverso dagli altri
// così il robot non riprova sempre la stessa manovra
static const unlock_step_t UNLOCK_TABLE[UNLOCK_MAX_ATTEMPTS] = {
    { 0.05, DEG_TO_RAD(0),    0.10 },  
    { 0.05, DEG_TO_RAD(30),   0.10 },   
    { 0.10, DEG_TO_RAD(-45),  0.15 },   
    { 0.05, DEG_TO_RAD(90),   0.08 },   
    { 0.10, DEG_TO_RAD(-120), 0.15 },   
    { 0.15, DEG_TO_RAD(60),   0.10 },   
    { 0.05, DEG_TO_RAD(-90),  0.20 },   
    { 0.15, DEG_TO_RAD(150),  0.10 },   
    { 0.10, DEG_TO_RAD(180),  0.15 },   
    { 0.20, DEG_TO_RAD(120),  0.10 },   
};


// array costante con i valori da sommare all'indice per ottenere i vicini
static const int NEIGHBOR_SHIFT[4] = {1, -1, NAV_MAP_CELL_NUMBER, -NAV_MAP_CELL_NUMBER};

// Primo doppio buffer per comunicare con il task di movimento
// canale di scambio del doppio buffer
static scambio_t double_buffer;
// i due buffer
static navigation_objective_t buffer1, buffer2;

// Secondo doppio buffer per comunicare con la mappatura
static scambio_t double_buffer_2;
// i due buffer
static navigation_closed_cells_t buffer3, buffer4;  

// caratteristiche del task
rt_task_t navigation_task;

// Coda di ricerca, usata per la ricerca del percorso
typedef struct {
    uint16_t values[NAV_MAP_CELL_NUMBER * NAV_MAP_CELL_NUMBER];         // lista degli indici tra cui cercare, ogni indice può comparire al massimo una sola volta, quindi basta un'array di queste dimensioni
    uint32_t head, tail;        // testa e coda dell'array, servono per interfacciarcisi come se fosse una coda
} nav_search_queue_t;

// Percorso, struttura dati che contiene la lista di celle da percorrere e la lunghezza totale
typedef struct {

    uint16_t cells[NAV_MAP_CELL_NUMBER * NAV_MAP_CELL_NUMBER];       // lista di celle da percorere
    uint32_t length;                                                 // lunghezza del percorso
    uint8_t quads[NAV_MAP_CELL_NUMBER * NAV_MAP_CELL_NUMBER];        // per ogni cella in cells, contiene i valori dei quadranti
} nav_path_t;

// enum per le direzioni da usare in BA*
typedef enum {
    CLEAN_DIR_NORTH,
    CLEAN_DIR_SOUTH,
    CLEAN_DIR_EAST,
    CLEAN_DIR_WEST,
} clean_dir_t;


// Enum degli stati del task di navigazione
// Il task di navigazione è una macchina a stati 
typedef enum {
    NAV_STATE_WAIT,             // attendi che la mappa si riempia
    NAV_STATE_EXPLORE,          // cerca la frontiera più vicina
    NAV_STATE_CLEAN,            // prepara il prossimo tratto da pulire
    NAV_STATE_FOLLOW_BORDERS,   // gira lungo i bordi della stanza (muri, attorno agli ostacoli)
    NAV_STATE_RETURN_HOME,      // cerca il percorso per la base di ricarica
    NAV_STATE_HOOK_UP,          // passa il controllo al driver per agganciare il robot alla base
    NAV_STATE_FOLLOW_PATH,      // segui il percorso calcolato, un punto di svolta alla volta
    NAV_STATE_UNLOCK,           // dopo un allarme, ferma il robot, indietreggia leggermente e riparti
    NAV_STATE_SLEEP,            // robot fermo, missione completata
    NAV_STATE_ALIGN,            // robot in allineamento, dopo un arresto, porta il robot al centro della cella di navigazione
    NAV_STATE_FAILURE,          // robot fermo, qualcosa è andato storto
    NAV_STATE_BORDER_LAP,       // il robot segue un ostacolo tenendolo a destra, fino alla fine del giro
    NAV_STATE_LEAVE_BASE,       // il robot esce dalla base di ricarica
    NAV_STATE_CROSS,            // il robot incastrato su un tappeto o un cavo avanza piano ruotando a turno a sinistra e a destra
    NAV_STATE_PAUSE,            // pulizia in pausa (tasto di accensione): robot fermo, riprende alla prossima pressione
} nav_state_t;

// definizione dello stato interno del task di navigazione
typedef struct {
    
    uint8_t map[NAV_MAP_CELL_NUMBER * NAV_MAP_CELL_NUMBER];            // copia della mappa di navigazione, passata dal task di mappatura
    uint16_t came_from[NAV_MAP_CELL_NUMBER * NAV_MAP_CELL_NUMBER];     // array delle provenienze: ogni cella contiene l'indice della cella da cui il robot è venuto
    uint32_t map_version;                       // numero di versione della mappa di navigazione
    
    navigation_objective_t cur_obj;             // ultimo obiettivo mandato
    nav_state_t state;                          // stato corrente del task di navigazione (macchina a stati)    
    nav_state_t mission;                        // fase della missione: lo stato di pianificazione a cui tornare (esplorazione, pulizia, seguimuro, ritorno alla base)
    
    nav_search_queue_t s_q;                     // search_queue, coda di ricerca, serve per la ricerca del percorso
    
    nav_path_t path;                            // percorso che deve fare il robot
    uint16_t path_idx;                          // indice del path dell'ultimo punto a cui è stato mandato mandato il robot
    uint16_t raw_path[NAV_MAP_CELL_NUMBER * NAV_MAP_CELL_NUMBER]; // celle del percorso trovato con la ricerca prima di aggiungere i quadranti

    int64_t state_start;                        // istante di ingresso nello stato corrente, serve per gli stati temporizzati
    uint8_t unlock_attempt;                     // numero di tentativi di sblocco fatti di fila
    int64_t stall_start;                        // istante in cui è iniziato l'arresto, 0 se il robot non è in arresto
    bool center_to_center;                      // è true se il percorso attuale va seguito senza prendere scorciatoie (le scorciatoie potrebbero far finire il robot in posizioni diverse dal centro delle celle, rendendo possibili situazioni di stallo dove il task di sicurezza ferma il robot per via di ostacoli che la navigazione non vede)
    navigation_closed_cells_t closed_cells;     // lista di celle chiuse da segnalare al task di mappatura
    
    struct {                                    // struct con le ultime TRAIL_SIZE posizioni del robot
        double x[TRAIL_SIZE];                   // buffer delle x delle posizioni
        double y[TRAIL_SIZE];                   // buffer delle y delle posizioni
        uint8_t head;                     // testa del buffer circolare
        uint8_t len;                      // dimensione impegnata del buffer circolare
    } trail;

    unlock_phase_t unlock_phase;            // la fase corrente di sblocco
    int64_t phase_start;                    // istante di inizio della fase corrente di sblocco
    double back_left;                       // distanza da percorrere all'indietro per completare lo sblocco, espressa in metri

    uint8_t clean_cell_mask[NAV_MAP_CELL_NUMBER * NAV_MAP_CELL_NUMBER / 4];     // la maschera che indica quali celle sono state pulite: ogni coppia di bit della maschera corrisponde ad una cella della mappa, per inidirizzare si usa una funzione helper che prende due indici (uno che indica quale cella, l'altro indica quale metà della cella), per convenzione, nel resto del codice diremo: sx = metà 0 = verso la coordinata più bassa dell'asse laterale; dx = metà 1 = verso la più alta
    bool strip_axis_x;              // l'asse secondo il quale si sviluppano le strisce della pulizia (true -> x, false -> y), viene calcolato dinamicamente all'inzio della fase di pulizia, in modo tale da minimizzare il numero di strisce (riducendo i tempi di pulizia)
    double cur_lat_shift;           // scostamento laterale del percorso attuale, espresso in metri      
    bool is_axis;                   // true se strip_axis_x è già stato calcolato, false altrimenti
   
    bool is_border_hooked;                  // true se durante il giro dei bordi il robot ha finito di ruotare e ha l'ostacolo a destra
    double border_no_new_dist;              // strada percorsa nel giro dall'ultima mezza cella nuova, espressa in metri
    double border_last_x, border_last_y;    // ultima posizione del giro, serve per misurare la strada percorsa
    int64_t border_lost_start;              // istante in cui il robot ha smesso di vedere l'ostacolo, 0 se lo vede

    bool strip_has_back;                    // indica se nella striscia di pulizia attuale c'è un punto dietro all'inizio
    bool has_left_base;                     // true se il robot è già uscito dalla base di ricarica
    bool is_clean_started;                  // true se la pulizia a strisce è già iniziata, la prima striscia parte da un estremo della stanza
    bool strip_has_end;                     // indica se nella striscia di pulizia attuale c'è un punto oltre alla fine
    bool hook_has_started;                  // indica se sia già stato richiesto al driver di agganciarsi alla base di ricarica

    double cross_theta;                     // attraversamento degli ostacoli bassi: direzione del robot quando si è incastrato

    int64_t battery_low_since;              // istante da cui il driver dice batteria bassa (o critica), 0 se è normale
    nav_state_t interrupted_mission;        // la missione che aveva il robot prima che venisse interrotta per via della batteria scarica. se 0, significa che il robot non si è interrotto per la batteria scarica.

    uint32_t last_power_presses;            // pressioni del tasto di accensione già lette 
    uint32_t last_home_presses;             // pressioni del tasto casa già lette
    bool is_map_ready;                      // true se in questa accensione un'esplorazione è finita (la mappa è completa): da qui la mappa si può salvare

    int64_t brush_unlock_start;             // istante di inizio dell'ultimo sblocco della spazzola centrale, 0 se non ce n'è mai stato uno
    uint8_t brush_unlock_attempt;           // numero di sblocchi della spazzola centrale fatti di fila

} navigation_state_t;

// stato interno della navigazione
static navigation_state_t nav_state;

// funzione helper per pushare una nuova posizione nella trail
static void trail_push(double x, double y) {
       
    // Inserisci la x e la y nella rispettiva cella del buffer circolare
    nav_state.trail.x[nav_state.trail.head] = x;
    nav_state.trail.y[nav_state.trail.head] = y;

    // avanza la testa del buffer circolare
    nav_state.trail.head = (nav_state.trail.head + 1) % TRAIL_SIZE; 

    // aumenta la lunghezza del buffer
    if(nav_state.trail.len < TRAIL_SIZE) nav_state.trail.len++;

}

// funzione helper per poppare una posizione dal trail
static bool trail_pop(double *x, double *y) {
    
    // verifica se il buffer è vuoto
    if (nav_state.trail.len <= 0) return false;

    // indietreggia la testa del buffer circolare
    nav_state.trail.head = (nav_state.trail.head + TRAIL_SIZE - 1) % TRAIL_SIZE; 
    
    // Inserisci la rispettiva cella del buffer circolare in x e y
    *x = nav_state.trail.x[nav_state.trail.head];
    *y = nav_state.trail.y[nav_state.trail.head];


    // diminuisci la lunghezza del buffer
    nav_state.trail.len--;

    return true;

}

// funzione helper per ottenere l'indice della maschera di pulizia
// prende l'indice della cella e la metà (0 o 1)
static inline uint32_t clean_mask_bit_index(const uint16_t cell_index, const bool cell_half) {
    return 2 * cell_index + cell_half; 
} 

// funzione helper per ottenere byte e bit shift da usare per lavorare sulla clean mask
static inline void calc_byte_bit_shift(const uint16_t cell_index, const bool cell_half, uint16_t *byte_idx, uint8_t *bit_shift) {
    
    // calcola l'indice della clean mask
    uint32_t idx = clean_mask_bit_index(cell_index, cell_half);

    // dall'indice ricava il byte della clean mask in cui si trova la mezza cella...
    *byte_idx = idx / 8;
    
    // ...e l'indice del bit cercato all'interno del byte
    *bit_shift = idx % 8;
}

// funzione helper per verificare se una certa mezza cella sia pulita
static inline bool is_half_cell_clean(const uint16_t cell_index, const bool cell_half) {
    
    uint16_t byte_idx = 0;
    uint8_t bit_shift = 0;

    // calcola byte della clean mask e indice del bit all'interno del byte
    calc_byte_bit_shift(cell_index, cell_half, &byte_idx, &bit_shift);

    // ritorna il bit della maschera
    return nav_state.clean_cell_mask[byte_idx] & (1u << bit_shift);

}

// funzione helper per segnare una mezza cella nella clean mask come pulita
// restituisce true se la mezza cella era sporca, false se era già pulita
static inline bool set_half_cell_clean(const uint16_t cell_index, const bool cell_half) {

    uint16_t byte_idx = 0;
    uint8_t bit_shift = 0;

    // calcola byte della clean mask e indice del bit all'interno del byte
    calc_byte_bit_shift(cell_index, cell_half, &byte_idx, &bit_shift);

    // controlla se la mezza cella era sporca
    bool was_dirty = !(nav_state.clean_cell_mask[byte_idx] & (1u << bit_shift));

    // metti il bit corrispondente a 1
    nav_state.clean_cell_mask[byte_idx] |= (1u << bit_shift);

    return was_dirty;

}

// funzione helper che restituisce lo spostamento laterale, cioè di quanto cambia l'indice di una cella passando alla cella accanto
// dipende solo dal campo strip_axis nello stato di navigazione, espresso in numero di celle
static inline int calc_clean_lateral_shift(void) {
    return nav_state.strip_axis_x ? NAV_MAP_CELL_NUMBER : 1;
}

// funzione helper opposta a calc_clean_lateral_shift
static inline int calc_clean_along_shift(void) {
    return nav_state.strip_axis_x ? 1 : NAV_MAP_CELL_NUMBER;
}

// funzione helper che prende l'indice di una cella e restituisce true se è totalmente libera
// quindi se lo stato è libero e la bitmask è tutta a 0
static inline bool is_nav_cell_completely_free(const uint16_t cell) {
    return nav_state.map[cell] == NAV_CELL_FREE;
}

// funzione helper che prende l'indice di una cella e restituisce true se ha stato NAV_CELL_FREE
// questa funzione potrebbe restituire true anche se la cella è solo parzialmente libera guardando la bitmask
static inline bool is_nav_cell_free(const uint16_t cell) {
    return get_nav_cell_state(nav_state.map[cell]) == NAV_CELL_FREE;
}


// funzione helper che prende una mezza cella e restituisce true se è possibile attraversarla
static inline bool is_half_cell_free_walkable(const uint16_t cell, const bool cell_half) {

    // i due quadranti della corsia della metà: con le strisce lungo x la metà 0 è in basso e la 1 in alto,
    // con le strisce lungo y la metà 0 è a sinistra e la 1 a destra
    uint8_t lane_quads;
    if (nav_state.strip_axis_x) lane_quads = cell_half ? (NAV_CELL_TOP_LEFT | NAV_CELL_TOP_RIGHT) : (NAV_CELL_BOTTOM_LEFT | NAV_CELL_BOTTOM_RIGHT);
    else lane_quads = cell_half ? (NAV_CELL_TOP_RIGHT | NAV_CELL_BOTTOM_RIGHT) : (NAV_CELL_TOP_LEFT | NAV_CELL_BOTTOM_LEFT);

    // la metà è percorribile se la cella è libera e i due quadranti della corsia hanno il bit a 0
    return is_nav_cell_free(cell) && (nav_state.map[cell] & lane_quads) == 0;
}

static inline bool is_half_cell_free_dirty(const uint16_t cell, const bool cell_half) {
    return (is_half_cell_free_walkable(cell, cell_half) && !is_half_cell_clean(cell, cell_half));
}

// funzione helper per calcolare quanto la corsia in cui passa il robot si scosti dal centro della cella
// prende cell_index e cell_half e restituisce un valore in metri
static inline double calc_clean_lane_shift(const bool cell_half) {

    // strumentopolo misterioso che ci servirà più tardi
    // serve per la logica di: metà dx -> vicino a dx, metà sx -> vicino sulla sx
    int8_t sign_shift = (cell_half ? 1 : -1);
    
    return LANE_DEVIATION * sign_shift;
}


// funzione helper per verificare se la pulizia sia attiva o meno, serve per decidere se attivare le spazzole
static inline bool is_active_cleaning(void) {
    return (
        (nav_state.mission == NAV_STATE_CLEAN) &&
        (nav_state.is_axis) &&
        (
            nav_state.state != NAV_STATE_UNLOCK &&
            nav_state.state != NAV_STATE_FAILURE &&
            nav_state.state != NAV_STATE_WAIT &&
            nav_state.state != NAV_STATE_SLEEP &&
            nav_state.state != NAV_STATE_LEAVE_BASE &&
            nav_state.state != NAV_STATE_PAUSE
        )
    );
}

// funzione helper che prende cancola la distanza tra il centro della cella e il robot
static double calc_robot_lateral_offset(const movement_position_data_t robot_pos, uint16_t robot_cell) {
    
    // calcola il centro della cella
    double robot_cell_x, robot_cell_y;
    nav_index_to_coord(robot_cell, &robot_cell_x, &robot_cell_y);

    // calcola lo scostamento laterale del robot dal centro
    // nelle y se l'asse di riferimento sono le x e viceversa
    double dist;
    if (nav_state.strip_axis_x) {
        dist = robot_pos.y - robot_cell_y;
    } else {
        dist = robot_pos.x - robot_cell_x;
    }

    return dist;
}

// funzione helper che prende la posizione e la cella del robot e restituisce la metà della cella in cui si trova
static bool get_robot_half_cell(const movement_position_data_t robot_pos, uint16_t cell) {

    double dist = calc_robot_lateral_offset(robot_pos, cell);

    double dist_half_sx = fabs(dist - calc_clean_lane_shift(0));
    double dist_half_dx = fabs(dist - calc_clean_lane_shift(1));

    return dist_half_sx > dist_half_dx;
}

// funzione che durante il giro dei bordi segna come pulito quello che il robot ha sotto, restituisce
// true se ha segnato almeno una mezza cella nuova. L'asse delle strisce deve essere già stato calcolato
static bool set_border_band_clean(const movement_position_data_t robot_pos) {

    // calcola la cella in cui si trova il robot
    uint16_t robot_cell = coord_to_nav_index(robot_pos.x, robot_pos.y);

    // finché il robot non è agganciato segna solo la mezza cella sotto il centro, così ogni giro pulisce almeno il punto di arrivo.
    if (!nav_state.is_border_hooked) return set_half_cell_clean(robot_cell, get_robot_half_cell(robot_pos, robot_cell));

    bool new_clean = false;

    // calcola seno e coseno dell'angolo del robot
    double cos_theta = cos(robot_pos.theta);
    double sin_theta = sin(robot_pos.theta);

    // scorri le 3x3 celle attorno al robot
    for (int dy = -1; dy <= 1; dy++) {
        for (int dx = -1; dx <= 1; dx++) {

            uint16_t cell = robot_cell + dy * NAV_MAP_CELL_NUMBER + dx;

            // scorri le due mezze celle
            for (int half = 0; half < 2; half++) {

                // calcola il centro della mezza cella
                double half_x, half_y;
                nav_index_to_coord(cell, &half_x, &half_y);

                if (nav_state.strip_axis_x) {
                    half_y += calc_clean_lane_shift(half);
                } else {
                    half_x += calc_clean_lane_shift(half);
                }

                // calcola lo scostamento della mezza cella dal robot
                double shift_x = half_x - robot_pos.x;
                double shift_y = half_y - robot_pos.y;

                // porta lo scostamento nel sistema del robot: quanto è avanti e quanto è a sinistra (a destra è negativo)
                double along = shift_x * cos_theta + shift_y * sin_theta;
                double left = -shift_x * sin_theta + shift_y * cos_theta;

                // se il centro della mezza cella cade nel rettangolo, segnala come pulita
                if (left >= -BORDER_CLEAN_RIGHT && left <= BORDER_CLEAN_LEFT && fabs(along) <= BORDER_CLEAN_ALONG) {
                    if (set_half_cell_clean(cell, half)) new_clean = true;
                }
            }
        }
    }

    return new_clean;
}

// funzione helper che segna le due mezze celle che si trovano sotto il robot come pulite se lo scostamento 
// laterale del robot dal centro è inferiore a LANE_CLEAN_DISTANCE_THRESHOLD
static void set_robot_cell_clean(const movement_position_data_t robot_pos) {

    // calcola la cella in cui si trova il robot
    uint16_t robot_cell = coord_to_nav_index(robot_pos.x, robot_pos.y);

    double dist = calc_robot_lateral_offset(robot_pos, robot_cell);

    // controlla se la mezza cella sx sia sotto al robot, se sì, segnala come pulita
    if (fabs(dist - calc_clean_lane_shift(0)) < LANE_CLEAN_DISTANCE_THRESHOLD) {
        set_half_cell_clean(robot_cell, 0);
    }
    
    // controlla se la mezza cella dx sia sotto al robot, se sì, segnala come pulita
    if (fabs(dist - calc_clean_lane_shift(1)) < LANE_CLEAN_DISTANCE_THRESHOLD) {
        
        set_half_cell_clean(robot_cell, 1);
    }
}

// funzione che prende una mezza cella e la direzione e retituisce la mezza cella vicina in quella direzione
static void get_half_cell_neighbor(const uint16_t cell, const bool cell_half, const clean_dir_t dir, uint16_t *neigh_cell, bool *neigh_half) {

    // la cella vicina parte dalla cella attuale
    *neigh_cell = cell;
    *neigh_half = cell_half;

    switch (dir) {
    case CLEAN_DIR_NORTH:
        
        *neigh_cell += calc_clean_along_shift();

        break;
    case CLEAN_DIR_SOUTH:
        
        *neigh_cell -= calc_clean_along_shift();

        break;
    case CLEAN_DIR_EAST:
        
        if (cell_half) {
            *neigh_cell += calc_clean_lateral_shift();
        }

        *neigh_half = !cell_half;

        break;
    case CLEAN_DIR_WEST:
        
        if (!cell_half) {
            *neigh_cell -= calc_clean_lateral_shift();
        }

        *neigh_half = !cell_half;

        break;    
    default:        // se siamo qui c'è un errore, non facciamo niente
        break;
    }

}

// funzione helper per impostare lo stato
static void set_state(nav_state_t next_state) {
    
    if (next_state == NAV_STATE_EXPLORE ||
        next_state == NAV_STATE_CLEAN ||
        next_state == NAV_STATE_FOLLOW_BORDERS ||
        next_state == NAV_STATE_RETURN_HOME
    ) nav_state.mission = next_state;    // se è uno stato di pianificazione, aggiorna la fase della missione

    nav_state.state = next_state;       // imposta il prossimo stato
     
    nav_state.state_start = rt_adesso();    // imposta l'istante di inizio ad ora

    nav_state.stall_start = 0;              // azzera il tempo di arresto del robot (il robot non è in arresto)

}

// funzione helper che verifica se il robot debba tornare a caricare prima di terminare la missione attuale
static bool is_battery_dead() {

    // leggi lo stato attuale del driver
    drv_stato_t cur_state = {0};

    drv_stato(&cur_state);

    // da quanto tempo la batteria è bassa: un calo di qualche istante (carico forte) non conta
    if (cur_state.batteria == BAT_OK) nav_state.battery_low_since = 0;
    else if (nav_state.battery_low_since == 0) nav_state.battery_low_since = rt_adesso();

    // se la batteria è scarica da almeno BATTERY_LOW_CONFIRM_TIME e la missione attuale è di esplorazione o pulizia
    // e lo stato attuale non è sblocco, attesa, guasto, riposo, uscita o aggancio alla base
    // ritorna il segnale che la batteria è scarica e bisogna interrompere la missione per andare a ricaricarla
    return (
        nav_state.battery_low_since != 0 && rt_adesso() - nav_state.battery_low_since >= BATTERY_LOW_CONFIRM_TIME &&
        (nav_state.mission == NAV_STATE_EXPLORE || nav_state.mission == NAV_STATE_FOLLOW_BORDERS || nav_state.mission == NAV_STATE_CLEAN) &&
        (nav_state.state != NAV_STATE_UNLOCK && nav_state.state != NAV_STATE_WAIT && nav_state.state != NAV_STATE_FAILURE && nav_state.state != NAV_STATE_SLEEP && nav_state.state != NAV_STATE_LEAVE_BASE && nav_state.state != NAV_STATE_HOOK_UP && nav_state.state != NAV_STATE_PAUSE)
    );
}

// Funzione helper per impostare una cella della mappa come troppo stretta (NAV_CELL_TOO_NARROW)
// al task di mappatura, aggiornando anche la copia locale della mappa di navigazione
static void close_cell(uint16_t cell) {

    // la cella davanti alla base non si chiude mai: è quella a cui deve arrivare il ritorno alla base
    if (cell == CHARGING_STATION_CELL) return;
    
    // aggiungi la cella alla lista di celle da mandare al task di mappatura
    nav_state.closed_cells.cells[nav_state.closed_cells.num % CLOSED_CELLS_MAX] = cell;

    // aumenta il numero di celle nella lista
    nav_state.closed_cells.num++;

    // pubblica la nuova lista
    scambio_scrivi(&double_buffer_2, &nav_state.closed_cells);

    // aggiorna anche la copia locale della mappa
    nav_state.map[cell] = set_nav_cell_state(nav_state.map[cell], NAV_CELL_TOO_NARROW);
}

// funzione helper per calcolare il tempo passato da un certo istante ad adesso
static inline int64_t elapsed_time(const int64_t start) {
    return rt_adesso() - start;
}

// Funzione Helper che prende allarmi e posizione attuale ritorna true
// se il robot è in stallo da almeno STALL_TIME
static bool is_stalled(safety_alarm_t cur_alarm, movement_position_data_t cur_pos) {
    // controlla se la velocità e velocità angolare sono sopra le soglie di stallo
    if (cur_alarm.max_v_forw > STALL_SPEED || fabs(cur_pos.omega) > STALL_OMEGA) {
        // se non c'è stallo, azzera l'istante di stallo e ritorna false
        nav_state.stall_start = 0;
        return false;
    } else if (nav_state.stall_start == 0) {    // se è in stallo e stall_start è a 0
        // metti stall_start all'istante corrente
        nav_state.stall_start = rt_adesso();
    }

    // ritorna se è passato più di STALL_TIME dall'ultimo stallo
    return (elapsed_time(nav_state.stall_start) >= STALL_TIME);


}

// funzioni helper per la coda
static inline void search_queue_push(uint16_t next_elem) {
    nav_state.s_q.values[nav_state.s_q.tail++] = next_elem;
}

static inline bool is_search_queue_empty(void) {
    return nav_state.s_q.head >= nav_state.s_q.tail;
}

static inline uint16_t search_queue_pop(void) {
    if (is_search_queue_empty()) return CELL_NOT_VISITED; 
    return nav_state.s_q.values[nav_state.s_q.head++];
}

static inline void search_queue_flush(void) {
    nav_state.s_q.head = 0;
    nav_state.s_q.tail = 0;
}

// funzione helper che verifica se due celle vicine hanno due quadranti "di contatto" liberi
// riempiendo i due quadranti passati per puntatore con i quadranti liberi in contatto
static inline bool get_contact_quads(const uint16_t cellA, const uint16_t cellB, uint8_t *quadA, uint8_t *quadB) {

    // calcola la differenza tra l'indice della cella B e l'indice della cella A
    int diff = cellB - cellA;

    // le due coppie di quadranti affacciati, una per ogni metà del lato in comune fra A e B:
    // in ogni coppia il primo quadrante è di A e il secondo è di B
    uint8_t pairs[2][2];

    // se le celle sono adiacenti (se non è così chi ha chiamato la funzione non ha capito cosa fa)
    // le differenze possono essere solo +-1 o +-NAV_MAP_CELL_NUMBER, da cui si ricavano le 4 direzioni dell'una rispetto all'altra
    switch (diff) {
        case 1:                         // la cella B è alla destra di A
            // i quadranti di destra di A toccano quelli di sinistra di B alla stessa altezza
            pairs[0][0] = NAV_CELL_TOP_RIGHT;       pairs[0][1] = NAV_CELL_TOP_LEFT;
            pairs[1][0] = NAV_CELL_BOTTOM_RIGHT;    pairs[1][1] = NAV_CELL_BOTTOM_LEFT;
        break;

        case -1:                        // la cella B è alla sinistra di A
            // i quadranti di sinistra di A toccano quelli di destra di B alla stessa altezza
            pairs[0][0] = NAV_CELL_TOP_LEFT;        pairs[0][1] = NAV_CELL_TOP_RIGHT;
            pairs[1][0] = NAV_CELL_BOTTOM_LEFT;     pairs[1][1] = NAV_CELL_BOTTOM_RIGHT;
        break;

        case NAV_MAP_CELL_NUMBER:       // la cella B è sopra A
            // i quadranti in alto di A toccano quelli in basso di B dalla stessa parte
            pairs[0][0] = NAV_CELL_TOP_LEFT;        pairs[0][1] = NAV_CELL_BOTTOM_LEFT;
            pairs[1][0] = NAV_CELL_TOP_RIGHT;       pairs[1][1] = NAV_CELL_BOTTOM_RIGHT;
        break;

        case -NAV_MAP_CELL_NUMBER:      // la cella B è sotto A
            // i quadranti in basso di A toccano quelli in alto di B dalla stessa parte
            pairs[0][0] = NAV_CELL_BOTTOM_LEFT;     pairs[0][1] = NAV_CELL_TOP_LEFT;
            pairs[1][0] = NAV_CELL_BOTTOM_RIGHT;    pairs[1][1] = NAV_CELL_TOP_RIGHT;
        break;

    default:                            // A e B non sono adiacenti
        return false;
    }

    // cerca la prima coppia con tutti e due i quadranti liberi (bit del quadrante a 0 --> libero)
    for (int i = 0; i < 2; i++) {
        if ((nav_state.map[cellA] & pairs[i][0]) == 0 && (nav_state.map[cellB] & pairs[i][1]) == 0) {
            *quadA = pairs[i][0];
            *quadB = pairs[i][1];
            return true;
        }
    }

    // nessuna coppia affacciata libera: da A non si può passare a B
    return false;
}


// definizione dell'enum dei possibili obiettivi della funzione di ricerca
typedef enum {
    GOAL_BORDER,    // l'obiettivo della ricerca è trovare una cella al confine delle celle esplorate
    GOAL_DIRTY,     // l'obiettivo della ricerca è trovare una cella con almeno una metà non pulita
    GOAL_CELL,      // l'obiettivo della ricerca è trovare una cella precisa (indice della cella)
    GOAL_FREE,      // l'obiettivo della ricerca è trovare una cella libera, partendo da una bloccata
    GOAL_EDGE,      // l'obiettivo della ricerca è trovare una cella con un quadrante di bordo ancora non pulito
    GOAL_NEAR_HOME, // l'obiettivo della ricerca è trovare una cella libera entro HOME_IR_DOCK_DIST dal punto di aggancio
} nav_search_goal_t;



// funzione helper che prende due celle vicine e restituisce quali quadranti sono liberi per transitare da una cella all'altra
// restituisce NAV_PATH_NO_QUAD se non ci sono quadranti liberi, restituisce NAV_PATH_CENTER se i quadranti di cell1 sono tutti liberi
static uint8_t get_adiacent_quad(const uint16_t cell1, const uint16_t cell2) {

    uint8_t quad, neigh;

    if (!get_contact_quads(cell1, cell2, &quad, &neigh)) return NAV_PATH_NO_QUAD;

    // se la cella 1 è libera, il robot può andare direttamente al centro della cella
    if (is_nav_cell_completely_free(cell1)) return NAV_PATH_CENTER; 

    return quad;

}

// Funzione helper che prende delle coordinate in metri e calcola il punto del reticolo dei quadranti in cui cadono.
// il reticolo ha 512x512 punti distanti mezza cella, ogni punto è il centro di un quadrante di una cella:
// è come coord_to_nav_index, ma con mezza cella al posto della cella
static inline void coord_to_lattice(const double x, const double y, int *lattice_x, int *lattice_y) {
    *lattice_x = (int)floor(x / (NAV_MAP_CELL_SIZE / 2)) + 2 * NAV_MAP_ORIGIN;
    *lattice_y = (int)floor(y / (NAV_MAP_CELL_SIZE / 2)) + 2 * NAV_MAP_ORIGIN;
}

// Funzione helper che prende le coordinate di un punto in una cella di navigazione
// e restituisce true se quel punto si trova in una cella che genera un bordo, false altrimenti
static bool is_lattice_point_edge_source(const double x, const double y) {

    // calcola il punto del reticolo dei quadranti in cui cade il punto
    int lattice_x, lattice_y;
    coord_to_lattice(x, y, &lattice_x, &lattice_y);

    // se il punto è fuori dal reticolo non c'è nessun ostacolo noto
    if (lattice_x < 0 || lattice_y < 0 || lattice_x >= 2 * NAV_MAP_CELL_NUMBER || lattice_y >= 2 * NAV_MAP_CELL_NUMBER) return false;

    // ogni cella contiene 2x2 punti del reticolo, quindi la cella del punto è la metà delle coordinate
    uint16_t cell = (lattice_y / 2) << 8 | (lattice_x / 2);

    // il resto della divisione per 2 dice quale quadrante della cella è: x dispari --> destra, y dispari --> alto
    uint8_t quad;
    if (lattice_y % 2 == 1) quad = (lattice_x % 2 == 1) ? NAV_CELL_TOP_RIGHT : NAV_CELL_TOP_LEFT;
    else quad = (lattice_x % 2 == 1) ? NAV_CELL_BOTTOM_RIGHT : NAV_CELL_BOTTOM_LEFT;

    // se lo stato della cella è NAV_CELL_FORBIDDEN restituisci false perchè quella zona non genera bordi da pulire
    if (get_nav_cell_state(nav_state.map[cell]) == NAV_CELL_FORBIDDEN) return false;

    // conta solo il bit del quadrante, non lo stato della cella: il bit a 1 vuol dire che lì c'è un ostacolo vero,
    // quindi le celle troppo strette e quelle sconosciute da sole non generano bordi
    return (nav_state.map[cell] & quad) != 0;
}


// Funzione helper che prende una cella e uno dei suoi quadranti e restituisce true se il quadrante
// è di bordo, cioè se è libero e tocca con un lato un quadrante bloccato
static bool is_quad_border(const uint16_t cell, const uint8_t quad) {

    // se il bit del quadrante è a 1 il quadrante non è libero
    if (nav_state.map[cell] & quad) return false;

    // calcola il centro del quadrante
    double x, y;
    quad_to_coord(cell, quad, &x, &y);

    // i centri dei quadranti vicini distano mezza cella
    const double quad_size = NAV_MAP_CELL_SIZE / 2;

    // il quadrante è di bordo se almeno uno dei 4 vicini è bloccato, le diagonali non contano
    return (
        is_lattice_point_edge_source(x + quad_size, y) ||
        is_lattice_point_edge_source(x - quad_size, y) ||
        is_lattice_point_edge_source(x, y + quad_size) ||
        is_lattice_point_edge_source(x, y - quad_size)
    );
}

// Funzione helper che prende le coordinate di un punto (il centro di un quadrante) e restituisce true
// se c'è una mezza cella pulita a meno di BORDER_DONE_DISTANCE da quel punto.
// L'asse delle strisce deve essere già stato calcolato
static bool is_clean_half_near(const double x, const double y) {

    // calcola la cella in cui si trova il punto
    const uint16_t center_cell = coord_to_nav_index(x, y);

    // scorri le 3x3 celle attorno al punto
    for (int dy = -1; dy <= 1; dy++) {
        for (int dx = -1; dx <= 1; dx++) {

            const uint16_t cell = center_cell + dy * NAV_MAP_CELL_NUMBER + dx;

            // scorri le due mezze celle
            for (int half = 0; half < 2; half++) {

                // le mezze celle sporche non contano
                if (!is_half_cell_clean(cell, half)) continue;

                // calcola il centro della mezza cella, e metà dei suoi lati
                double half_x, half_y, side_x, side_y;
                nav_index_to_coord(cell, &half_x, &half_y);

                if (nav_state.strip_axis_x) {
                    half_y += calc_clean_lane_shift(half);
                    side_x = NAV_MAP_CELL_SIZE / 2;
                    side_y = NAV_MAP_CELL_SIZE / 4;
                } else {
                    half_x += calc_clean_lane_shift(half);
                    side_x = NAV_MAP_CELL_SIZE / 4;
                    side_y = NAV_MAP_CELL_SIZE / 2;
                }

                // calcola la distanza del punto dal rettangolo della mezza cella
                const double dist_x = fmax(fabs(x - half_x) - side_x, 0);
                const double dist_y = fmax(fabs(y - half_y) - side_y, 0);

                if (hypot(dist_x, dist_y) <= BORDER_DONE_DISTANCE) return true;
            }
        }
    }

    return false;
}

// Funzione helper per stabilire se la cella fornita corrisponde all'obiettivo della ricerca
// qui la ricerca viene fatta sempre sulle 4 celle adiacenti (su, giù, dx, sx), il percorso
// verrà poi tagliato usando l'algoritmo di bresenham per permettere percorsi in diagonale
static bool cell_is_goal(const nav_search_goal_t goal, const uint16_t cell, const uint16_t goal_cell) {
    
    switch (goal) {
        case GOAL_BORDER:           // se la cella cercata è una cella di frontiera
            if (is_nav_cell_free(cell)) {     // se la cella in cui ci troviamo è libera
                // controllo le 4 celle adiacenti alla cella corrente, se sono sconosciute,
                // significa che cell è una cella di frontiera
                for (int i = 0; i < 4; i++) {
                    if (
                        get_nav_cell_state(nav_state.map[cell + NEIGHBOR_SHIFT[i]]) == NAV_CELL_UNKNOWN && 
                        get_adiacent_quad(cell, cell + NEIGHBOR_SHIFT[i]) != NAV_PATH_NO_QUAD
                    ) { return true; }
                }
            }
            break;
        case GOAL_DIRTY:
            // restituisce true se la cella è libera e almeno una delle sue due metà è sporca
            return (is_half_cell_free_dirty(cell, 0) || is_half_cell_free_dirty(cell, 1));
            break;          
        case GOAL_CELL:     // il terzo parametro serve solo qui
            return cell == goal_cell;
            break;
        case GOAL_FREE:
            return is_nav_cell_free(cell);
            break;
        case GOAL_NEAR_HOME: {
            // cella libera con il centro abbastanza vicino al punto di aggancio
            if (!is_nav_cell_free(cell)) return false;
            double x, y;
            nav_index_to_coord(cell, &x, &y);
            return hypot(x - AGG_PUNTO_X_M, y) <= HOME_IR_DOCK_DIST;
        }
        case GOAL_EDGE:
           
            if (!is_nav_cell_free(cell)) return false;          // se la cella non è libera non è un bordo

            // scorri i 4 quadranti della cella
            for (int i = 0; i < 4; i++) {

                // se il quadrante non è di bordo passa al prossimo
                if (!is_quad_border(cell, NAV_CELL_QUAD[i])) continue;

                // calcola il centro del quadrante
                double x, y;
                quad_to_coord(cell, NAV_CELL_QUAD[i], &x, &y);

                // se non c'è una mezza cella pulita vicina, questo bordo è ancora da pulire
                if (!is_clean_half_near(x, y)) return true;
            }

            break;
        default:                // se entriamo in questo caso c'è un errore
            return false;       // ritorniamo false nel dubbio
            break;
    }

    return false;
    
}

// Funzione helper per stabilire se il robot può attraversare la cella passata (next_cell)
// di norma il controllo sarebbe solo se la cella è libera, ma se il robot dovesse
// trovarsi in una zona bloccata o proibita dobbiamo permettergli di attraversarla per 
// uscire facendo la strada più breve. 
// start_cell è la cella da cui parte la ricerca, posizione corrente del robot
// cur_cell è la cella presa dalla coda di ricerca, di cui stiamo guardando i vicini
// next_cell il vicino di cur_cell che stiamo valutando se mettere nella coda di ricerca o meno
static bool cell_can_enter(const uint16_t cur_cell, const uint16_t next_cell, const uint16_t start_cell) {

    // ricava x e y di next_cell dall'indice
    uint16_t next_cell_y = next_cell >> 8;
    uint16_t next_cell_x = next_cell & 0xFF;

    // se next_cell si trova sul bordo della mappa, non può entrare a prescindere
    if (next_cell_x == 0 || next_cell_x == NAV_MAP_CELL_NUMBER - 1) return false;   // bordo destro e sinistro
    if (next_cell_y == 0 || next_cell_y == NAV_MAP_CELL_NUMBER - 1) return false;   // bordo superiore e inferiore

    uint8_t quad_corridor1, quad_corridor2;

    // se next_cell è libera, può entrare
    if (is_nav_cell_free(next_cell) && (!is_nav_cell_free(cur_cell) || get_contact_quads(cur_cell, next_cell, &quad_corridor1, &quad_corridor2))) return true;

    // se siamo qui significa che next_cell è proibita, bloccata o sconosciuta
    // in questo caso si può passare solo se:
    // 1. next_cell non è sconosciuta
    // 2. cur_cell è bloccata o proibita (da una cella libera non puoi passare per una bloccata)
    // 3. se next_cell si trova entro ESCAPE_MAX_CELLS da start_cell
    // 4. se next_cell è proibita, anche start_cell dev'essere proibita (il robot è già dentro la zona e deve uscirne)
    if (
        get_nav_cell_state(nav_state.map[next_cell]) != NAV_CELL_UNKNOWN &&
        !is_nav_cell_free(cur_cell) &&
        get_nav_cell_state(nav_state.map[next_cell]) != NAV_CELL_TOO_NARROW &&
        (get_nav_cell_state(nav_state.map[next_cell]) != NAV_CELL_FORBIDDEN || get_nav_cell_state(nav_state.map[start_cell]) == NAV_CELL_FORBIDDEN)
    ) {
        
        // ricava x e y di start_cell dall'indice
        uint16_t start_cell_y = start_cell >> 8;
        uint16_t start_cell_x = start_cell & 0xFF;
        
        // calcola la distanza da start_cell a next_cell
        double dist = hypot(next_cell_x - start_cell_x, next_cell_y - start_cell_y);
        
        // se la distanza è inferiore a ESCAPE_MAX_CELLS, il robot può entrare in una cella 
        // bloccata o proibita
        if (dist <= ESCAPE_MAX_CELLS) return true; 

    }

    // in tutti gli altri casi, non può entrare
    return false;
}


// Funzione helper per calcolare la velocità alla quale deve andare il robot
// in base agli allarmi mandati dal task di sicurezza e la cella obiettivo
static inline double calc_speed(const double max_v) {

    // la velocità del robot si calcola come il minimo tra la CRUISING_SPEED e la
    // massima velocità passata per parametro (calcolata dal task di sicurezza)
    // se la cella obiettivo è una cella sconosciuta, si moltiplica la velocità
    // per UNKNOWN_SPEED_FACTOR, ottenendo la velocità finale

    return MIN(max_v, CRUISING_SPEED) * (!is_nav_cell_free(nav_state.path.cells[nav_state.path_idx]) ? UNKNOWN_SPEED_FACTOR : 1);

}

// Funzione che calcola il campo strip_axis_x dello stato. serve per calcolare l'asse di riferimento per cui
// disporre le strisce della pulizia. viene chiamata SOLO dopo la fine dell'esplorazione, visto che il suo 
// funzionamento si basa sul fatto che quando l'esplorazione termina, la nav_state_s_q è piena con le celle della mappa
static void calc_strip_axis() {

    // contatori del numero di strisce che salterebbero fuori sia con il primo, che con il secondo asse
    uint32_t num_strip_x = 0, num_strip_y = 0;
    
    // scorri tutte le celle all'interno della coda usata nell'esplorazione:
    // la coda DEVE contenere tutte le celle della mappa raggiungibili dal robot, affinchè l'algoritmo funzioni
    for (uint32_t i = 0; i < nav_state.s_q.tail; i++) {
        
        
        // Se la cella corrente non è libera, passa alla prossima
        if (nav_state.map[nav_state.s_q.values[i]] != NAV_CELL_FREE) continue;

        // Se la cella precedente lungo l'asse X non è libera, da questa cella parte una striscia
        if (nav_state.map[nav_state.s_q.values[i] - 1] != NAV_CELL_FREE) num_strip_x++;

        // Se la cella precedente lungo l'asse y non è libear, da questa cella parte una striscia
        if (nav_state.map[nav_state.s_q.values[i] - NAV_MAP_CELL_NUMBER] != NAV_CELL_FREE) num_strip_y++;
    }

    // strip_axis_x è vero se num_strip_x < num_strip_y
    nav_state.strip_axis_x = num_strip_x < num_strip_y;

    // segna come calcolato l'asse
    nav_state.is_axis = true;

}


// funzione helper per aggiungere una cella + quadrante in fondo al percorso
static void add_to_path(const uint16_t cell, const uint8_t quad) {
    
    // controlla se c'è spazio nell'array del percorso, se no esci
    if (nav_state.path.length >= NAV_MAP_CELL_NUMBER * NAV_MAP_CELL_NUMBER) return;

    // controlla se l'ultima cella nel percorso sia uguale a quella che stiamo aggiungendo, se sì esci
    if (nav_state.path.length > 0 && nav_state.path.cells[nav_state.path.length - 1] == cell && nav_state.path.quads[nav_state.path.length - 1] == quad) return;

    // aggiungi cella e quadrante al percorso e incrementa la lunghezza
    nav_state.path.cells[nav_state.path.length] = cell;

    nav_state.path.quads[nav_state.path.length] = quad;

    nav_state.path.length++;

}

// funzione helper che data una cella e due suoi quadranti restituisce il quadrante per cui si può passare per congiungerli
// ritorna NAV_PATH_NO_QUAD se non si può, altrimenti usa gli stessi valori e le stesse convenzioni della bitmask delle celle
// se i due quadranti sono già adiacenti restituisce NAV_PATH_NO_QUAD
static uint8_t get_middle_quad(const uint16_t cell, const uint8_t quad1, const uint8_t quad2) {

    // i due quadranti di mezzo, cioè quelli diversi da quad1 e quad2
    uint8_t middle1, middle2;

    // controlla se i due quadranti sono opposti e ricava i due quadranti di mezzo, se non sono opposti non serve passare in mezzo
    if ((quad1 | quad2) == (NAV_CELL_TOP_LEFT | NAV_CELL_BOTTOM_RIGHT)) {
        middle1 = NAV_CELL_TOP_RIGHT;
        middle2 = NAV_CELL_BOTTOM_LEFT;
    } else if ((quad1 | quad2) == (NAV_CELL_TOP_RIGHT | NAV_CELL_BOTTOM_LEFT)) {
        middle1 = NAV_CELL_TOP_LEFT;
        middle2 = NAV_CELL_BOTTOM_RIGHT;
    } else {
        return NAV_PATH_NO_QUAD;
    }

    // restituisci il primo quadrante di mezzo libero (bit del quadrante a 0)
    if ((nav_state.map[cell] & middle1) == 0) return middle1;
    if ((nav_state.map[cell] & middle2) == 0) return middle2;

    // nessuno dei due è libero: non dovrebbe succedere, la mappatura non lascia mai liberi solo due quadranti opposti
    return NAV_PATH_NO_QUAD;
}


// funzione helper che prende due celle consecutive del percorso e scrive nei due puntatori
// da quale quadrante il robot esce da cell1 (exit_quad) e in quale quadrante entra in cell2 (entry_quad).
// se il passo non ha bisogno di quadranti scrive NAV_PATH_NO_QUAD in tutti e due:
// se le due celle sono tutte libere il robot va da centro a centro
// se cell1 non è libera il robot sta uscendo da una zona bloccata e va da centro a centro
static void get_step_quads(const uint16_t cell1, const uint16_t cell2, uint8_t *exit_quad, uint8_t *entry_quad) {

    // all'inizio supponiamo che non servano quadranti
    *exit_quad = NAV_PATH_NO_QUAD;
    *entry_quad = NAV_PATH_NO_QUAD;

    // se una delle due celle non è libera non servono quadranti, esci
    if (!is_nav_cell_free(cell1) || !is_nav_cell_free(cell2)) return;

    // se le due celle sono tutte libere non servono quadranti, esci
    if (is_nav_cell_completely_free(cell1) && is_nav_cell_completely_free(cell2)) return;

    // altrimenti il robot deve passare per la porta tra le due celle: get_contact_quads scrive nei due
    // puntatori il quadrante di cell1 e quello di cell2 adiacenti e liberi.
    if (!get_contact_quads(cell1, cell2, exit_quad, entry_quad)) {
        *exit_quad = NAV_PATH_NO_QUAD;
        *entry_quad = NAV_PATH_NO_QUAD;
    }

}

// funzione helper che prende il numero di celle del percorso grezzo 
// e scrive in nav_state.path il percorso vero, con cella + quadrante per ogni punto da raggiungere.
// restituisce la lunghezza del percorso vero.
// per ogni cella del percorso grezzo aggiunge:
// se la cella non è libera: il centro
// se la cella è tutta libera: il quadrante di ingresso (se c'è), il centro, il quadrante di uscita (se c'è)
// se la cella è libera in parte: il quadrante di ingresso, il quadrante di mezzo (se serve), il quadrante di uscita;
// se non ci sono né ingresso né uscita, il primo quadrante libero
static uint32_t expand_path(const uint32_t raw_length) {

    // svuota il percorso vero, add_to_path lo riempie da capo
    nav_state.path.length = 0;

    // quadrante di ingresso nella cella corrente: viene calcolato al giro prima, insieme all'uscita dalla cella
    // precedente, perchè ogni passo (cella precedente -> cella corrente) si calcola una volta sola.
    // la prima cella del percorso non ha un ingresso
    uint8_t entry_quad = NAV_PATH_NO_QUAD;

    // scorri tutte le celle del percorso grezzo
    for (uint32_t i = 0; i < raw_length; i++) {

        // cella corrente del percorso grezzo
        uint16_t cur_cell = nav_state.raw_path[i];

        // quadrante di uscita dalla cella corrente e quadrante di ingresso nella prossima cella
        // l'ultima cella del percorso non ha un'uscita, quindi restano NAV_PATH_NO_QUAD
        uint8_t exit_quad = NAV_PATH_NO_QUAD;
        uint8_t next_entry_quad = NAV_PATH_NO_QUAD;

        // se c'è una prossima cella, calcola i quadranti del passo dalla cella corrente alla prossima
        if (i + 1 < raw_length) get_step_quads(cur_cell, nav_state.raw_path[i + 1], &exit_quad, &next_entry_quad);

        if (!is_nav_cell_free(cur_cell)) {      // se la cella corrente non è libera (il robot sta uscendo da una zona bloccata)

            // il robot passa per il centro della cella, come prima della mappa a due livelli
            add_to_path(cur_cell, NAV_PATH_CENTER);

        } else if (is_nav_cell_completely_free(cur_cell)) {     // se la cella corrente è tutta libera

            // se il robot entra da una porta, aggiungi il quadrante di ingresso
            if (entry_quad != NAV_PATH_NO_QUAD) add_to_path(cur_cell, entry_quad);

            // aggiungi il centro della cella
            add_to_path(cur_cell, NAV_PATH_CENTER);

            // se il robot esce da una porta, aggiungi il quadrante di uscita
            if (exit_quad != NAV_PATH_NO_QUAD) add_to_path(cur_cell, exit_quad);

        } else if (entry_quad == NAV_PATH_NO_QUAD && exit_quad == NAV_PATH_NO_QUAD) {     // se la cella è libera in parte, ma non ha né ingresso né uscita

            // succede se il percorso è di una cella sola, o se è l'ultima cella dopo una zona bloccata:
            // aggiungi il primo quadrante libero (bit del quadrante a 0) della cella
            for (int quad = 0; quad < 4; quad++) {
                if ((nav_state.map[cur_cell] & NAV_CELL_QUAD[quad]) == 0) {
                    add_to_path(cur_cell, NAV_CELL_QUAD[quad]);
                    break;
                }
            }

        } else {        // se la cella corrente è libera in parte, il robot non passa mai per il centro

            // se c'è un ingresso, aggiungi il quadrante di ingresso
            if (entry_quad != NAV_PATH_NO_QUAD) add_to_path(cur_cell, entry_quad);

            // se ingresso e uscita sono in angoli opposti, aggiungi il quadrante di mezzo libero tra i due.
            // se manca uno dei due, o se non sono opposti, get_middle_quad restituisce NAV_PATH_NO_QUAD
            uint8_t middle_quad = get_middle_quad(cur_cell, entry_quad, exit_quad);
            if (middle_quad != NAV_PATH_NO_QUAD) add_to_path(cur_cell, middle_quad);

            // se c'è un'uscita, aggiungi il quadrante di uscita
            if (exit_quad != NAV_PATH_NO_QUAD) add_to_path(cur_cell, exit_quad);

        }

        // l'ingresso nella prossima cella diventa l'ingresso della cella corrente al prossimo giro
        entry_quad = next_entry_quad;
    }

    // restituisci la lunghezza del percorso vero
    return nav_state.path.length;
}


// Funzione helper che scrive il percorso completo nello stato della navigazione
// riceve il punto di partenza e il punto di arrivo del percorso
static uint32_t reconstruct_path(const uint16_t start_cell, const uint16_t end_cell) {
    
    // Inizializza la lunghezza del percorso a 0. questa variabile viene incrementata ad ogni passo
    // del for successivo
    int length = 0;
    
    // Scorri il percorso nell'array came_from, partendo da end_cell e andando a ritroso
    // fermati a start_cell, l'array path verrà riempito al contrario
    for (int cur_cell = end_cell; cur_cell != start_cell; cur_cell = nav_state.came_from[cur_cell]) {
        
        // aggiungi cur_cell al percorso nello stato di navigazione
        nav_state.raw_path[length] = cur_cell;

        // aumenta la lunghezza del percorso
        length++;
    }
    
    // aggiungi start_cell alla fine, il for di prima non lo aggiungeva
    nav_state.raw_path[length] = start_cell;
    length++;

    // rovescia l'array del percorso
    for (int i = 0; i < length / 2; i++) {
        uint16_t tmp;
        tmp = nav_state.raw_path[i];
        nav_state.raw_path[i] = nav_state.raw_path[length - 1 - i];
        nav_state.raw_path[length - 1 - i] = tmp;
    }

    return length;
}


// Funzione che esegue la ricerca di una cella di un determinato tipo
// start_cell: la cella in cui si trova il robot
// goal: il tipo di cella che stiamo cercando (frontiera, non pulita, specifica)
// goal_cell: nel caso il tipo sia "cella specifica", contiene l'indice della cella, altrimenti non viene usata
// restituisce la lunghezza del percorso trovato (0 se non esiste) e salva in nav_state il percorso trovato
static uint32_t search_cell(const uint16_t start_cell, const nav_search_goal_t goal, const uint16_t goal_cell) {
    
    // Inzializza a CELL_NOT_VISITED la griglia di came_from
    memset(nav_state.came_from, CELL_NOT_VISITED, sizeof(nav_state.came_from));

    // svuota la coda di ricerca
    search_queue_flush();

    // imposta la cella start_cell in came_from a start_cell. siccome start_cell è la cella di
    // partenza mettiamo che viene da sè stessa
    nav_state.came_from[start_cell] = start_cell;

    // fai il push della cella di partenza nella search_queue
    search_queue_push(start_cell);

    // Ciclo di ricerca
    while (!is_search_queue_empty()) {
        
        // preleva il prossimo elemento da esplorare
        uint16_t cur_cell = search_queue_pop();

        // se corrisponde al goal, ricorstruisci il percorso completo e restituiscine la lunghezza
        if (cell_is_goal(goal, cur_cell, goal_cell)) return expand_path(reconstruct_path(start_cell, cur_cell));

        // se siamo qui significa che cur_cell non è il goal, quindi controlliamo le celle vicine
        for (int i = 0; i < 4; i++) {

            // calcola l'indice della cella vicina alla cella corrente
            uint16_t neigh_cell = cur_cell + NEIGHBOR_SHIFT[i];
            
            // Verifica se la vicina attuale potrebbe essere una candidata come prossima cella
            if (nav_state.came_from[neigh_cell] == CELL_NOT_VISITED && cell_can_enter(cur_cell, neigh_cell, start_cell)) {
                // se sì, salva cur_cell come cella di provenienza di neigh_cell
                nav_state.came_from[neigh_cell] = cur_cell;
                // e aggiungila alla coda di ricerca
                search_queue_push(neigh_cell);
            } 
        
        }
        
    }

    // Se non trovi nessun percorso, metti 0 come distanza del percorso e restituisci 0
    return nav_state.path.length = 0;
    
}



// Funzione che prende l'indice di un punto del percorso e calcola le sue coordinate in metri:
// se il punto è un quadrante, le coordinate sono il centro del quadrante
// se il punto è il centro della cella, le coordinate sono il centro della cella spostato dello scostamento
// laterale della corsia attuale (nav_state.cur_lat_shift), come per le strisce della pulizia
static void get_path_coord(const uint32_t path_idx, double *x, double *y) {

    // calcola le coordinate del punto: quad_to_coord con NAV_PATH_CENTER finisce nel default e restituisce il centro della cella
    quad_to_coord(nav_state.path.cells[path_idx], nav_state.path.quads[path_idx], x, y);

    // se il punto è il centro della cella, applica lo scostamento laterale alle x o alle y in base all'asse di riferimento per lo scostamento
    if (nav_state.path.quads[path_idx] == NAV_PATH_CENTER) {
        if (nav_state.strip_axis_x) *y += nav_state.cur_lat_shift; else *x += nav_state.cur_lat_shift;
    }

}

// Funzione helper che prende un punto del reticolo dei quadranti e restituisce true se il suo quadrante è libero
static bool is_lattice_point_free(const int lattice_x, const int lattice_y) {

    // se il punto è fuori dal reticolo non è libero
    if (lattice_x < 0 || lattice_y < 0 || lattice_x >= 2 * NAV_MAP_CELL_NUMBER || lattice_y >= 2 * NAV_MAP_CELL_NUMBER) return false;

    // ogni cella contiene 2x2 punti del reticolo, quindi la cella del punto è la metà delle coordinate
    uint16_t cell = (lattice_y / 2) << 8 | (lattice_x / 2);

    // il resto della divisione per 2 dice quale quadrante della cella è: x dispari = destra, y dispari = alto
    uint8_t quad;
    if (lattice_y % 2 == 1) quad = (lattice_x % 2 == 1) ? NAV_CELL_TOP_RIGHT : NAV_CELL_TOP_LEFT;
    else quad = (lattice_x % 2 == 1) ? NAV_CELL_BOTTOM_RIGHT : NAV_CELL_BOTTOM_LEFT;

    // il punto è libero se la cella è libera (le troppo strette e le proibite possono avere i bit a 0) e il bit del quadrante è 0
    return is_nav_cell_free(cell) && (nav_state.map[cell] & quad) == 0;
}



// Funzione helper che usa Bresenham per verificare se due punti del percorso cadono sulla stessa
// retta e non ci sono quadranti non liberi nel mezzo (sul reticolo dei quadranti, da 10 cm)
// Se fossi stato un buon programmatore avrei riutilizzato il codice
// per fare Bresenham di mapping.c ma non ho voglia e sono stanco
static bool line_of_sight(uint32_t start_idx, uint32_t end_idx) {

    // Ricava x e y base (il punto il punto di partenza) e da
    // tail (il punto di svolta), come punti del reticolo dei quadranti
    double start_x, start_y, end_x, end_y;
    get_path_coord(start_idx, &start_x, &start_y);
    get_path_coord(end_idx, &end_x, &end_y);

    int base_x, base_y, tail_x, tail_y;
    coord_to_lattice(start_x, start_y, &base_x, &base_y);
    coord_to_lattice(end_x, end_y, &tail_x, &tail_y);

    
    // calcola la distanza tra i due punti nell'asse x
    int dx = abs(tail_x - base_x);

    // calcola il verso in cui si muove la x
    int sx = (base_x < tail_x) ? 1 : -1;

    // calcola la distanza tra i due punti nell'asse y
    // il segno meno serve per non dover invertire i confronti e le somme di dy
    // rispetto a dx
    int dy = -abs(tail_y - base_y);

    // calcola il verso in cui si muove la y
    int sy = (base_y < tail_y) ? 1 : -1;

    // calcola quanto la cella corrente si trova distante rispetto alla retta
    int err = dx + dy;

    // Inzializza le coordinate della cella corrente alla cella iniziale
    int cur_cell_x = base_x, cur_cell_y = base_y;


    // Esegui l'algoritmo finchè non arrivi alla cella finale
    for (int step = 0; step <= dx - dy; step++) {
        // Se siamo arrivati all'ultima cella
        if (cur_cell_x == tail_x && cur_cell_y == tail_y) {
            return true;
        } else {    // Se non siamo ancora arrivati alla fine


            // raddoppia errore e salvatelo in una variabile per fare i confronti
            int e2 = 2 * err;
            
            bool vert_error = e2 <= dx;
            bool hor_error = e2 >= dy;


            // avanza di un passo nella direzione corrente in base all'errore

            if (vert_error && hor_error) {  
                
                // se uno dei due punti in diagonale non è libero, ritorna false
                if (!is_lattice_point_free(cur_cell_x, cur_cell_y + sy) || !is_lattice_point_free(cur_cell_x + sx, cur_cell_y)) return false;
            }
            if (hor_error) { err += dy; cur_cell_x += sx; }
            if (vert_error) { err += dx; cur_cell_y += sy; }

            if (!is_lattice_point_free(cur_cell_x, cur_cell_y)) return false;
        }
    } 

    return false;

}

// Funzione che dato nav_state.path_idx, che indica in quale punto del percorso
// si trova il robot, restituisce l'indice del prossimo punto di svolta per mandare il prossimo obiettivo.
static inline bool is_strip_path(void) {
    return nav_state.mission == NAV_STATE_CLEAN && nav_state.cur_lat_shift != 0;
}

static uint16_t next_turning_point(void) {

    // Inizializza gli indici all'inizio del path e alla fine del path
    uint32_t cur_idx = nav_state.path_idx;
    uint32_t last_idx = nav_state.path.length - 1;

    // Se il percorso ha lunghezza 0, restituisce l'indice di partenza
    if (nav_state.path.length == 0 || cur_idx >= last_idx) return cur_idx;

    // Controlla se la posizione attuale è in una cella non libera o se non bisogna fare tagli al percorso (center_to_center)
    if (!is_nav_cell_free(nav_state.path.cells[nav_state.path_idx]) || nav_state.center_to_center) {
        // se il robot si trova in una cella non libera, vai alla prossima cella nel percorso senza fare tagli
        return cur_idx + 1;
    }

    // Ricerca dicotomica dell'indice della prossima cella del percorso per cui si può tagliare
    // Definizione degli estremi
    int low = cur_idx + 1;      // l'estremo inferiore è la prossima cella, quella è sempre visibile, quindi non serve controllarla
    int high =  last_idx;       // l'estremo superiore, parte dall'ultima cella, è quello che cerchiamo

    // Controllo Rapido del Caso Migliore:
    // se si può tagliare direttamente tra low e high, fallo senza fare ricerca dicotomica
    if (line_of_sight(cur_idx, high)) return high;

    while (low < high) {        // ciclo di ricerca

        // calcola il punto medio tra low e high, l'arrotodamento per eccesso (+1)
        // serve a scongiurare il caso del ciclo infinito (high = low + 1)
        int mid = (low + high + 1) / 2;      

        // se dal robot si vede il punto mid, allora il punto di svolta è da mid in poi
        if (line_of_sight(cur_idx, mid)) {
            low = mid;
        } else {    // altrimenti il punto di svolta è prima
            high = mid - 1;
        }
    }

    // alla fine del ciclo, low contiene il punto di svolta
    return low;

}


// funzione di inizializzazione del task
void init_navigation(void) {

    // inizializza il primo doppio buffer passando i due buffer da usare
    // e la loro dimensione 
    scambio_init(
        &double_buffer,
        &buffer1,
        &buffer2, 
        sizeof(navigation_objective_t)
    );

    // inizializza il secondo doppio buffer passando i due buffer da usare
    // e la loro dimensione 
    scambio_init(
        &double_buffer_2,
        &buffer3,
        &buffer4, 
        sizeof(navigation_closed_cells_t)
    );


    nav_state.state_start = rt_adesso();

    // si parte in riposo: il robot aspetta un comando (tasto di accensione o l'app)
    nav_state.state = NAV_STATE_SLEEP;
    nav_state.mission = NAV_STATE_SLEEP;

    nav_state.is_axis = false;

    nav_state.is_clean_started = false;

}

// funzione di lettura dei dati del task (la chiamerà il task di movimento)
navigation_objective_t read_navigation(void) {

    // Crea e Inizializza a 0 la struttura dati navigation_data
    navigation_objective_t navigation_data = {0};

    // Copia il buffer pubblicato in navigation_data
    scambio_leggi(&double_buffer, &navigation_data);

    // ritorna navigation_data
    return navigation_data;
}


// funzione di lettura dei dati del task (la chiamerà il task di mappatura)
navigation_closed_cells_t read_closed_cells(void) {

    // Crea e Inizializza a 0 la struttura dati navigation_closed_cells
    navigation_closed_cells_t navigation_closed_cells = {0};

    // Copia il buffer pubblicato in navigation_closed_cells
    scambio_leggi(&double_buffer_2, &navigation_closed_cells);

    // ritorna navigation_closed_cells
    return navigation_closed_cells;
}



// funzione di scrittura dei dati del task (l'obiettivo)
void write_objective(const navigation_objective_t *next_objective) {

    // Copia scrivi il prossimo obiettivo nel doppio buffer
    scambio_scrivi(&double_buffer, next_objective);

    return;
}


// Funzione helper per creare l'obiettivo e pubblicarlo
void send_objective(
    const objective_type_t obj_type, 
    const double omega,
    const double theta,
    const double v,
    const double x,
    const double y) {

    const navigation_objective_t next_obj = {
        .num = nav_state.cur_obj.num + 1,
        .obj_type = obj_type,
        .omega = omega,
        .theta = theta,
        .v = v,
        .x = x,
        .y = y,
    };

    nav_state.cur_obj = next_obj;

    write_objective(&next_obj);

}

// funzione helper per fermare il robot e andare in stato di guasto. reason è il motivo, che il driver scrive nel registro
// e pubblica in drv_stato_t.task_fault: il driver ferma i motori, accende il LED del guasto e suona
static inline void stop_and_fail(drv_task_fault_t reason) {

    send_objective(OBJ_STOP, 0, 0, 0, 0, 0);

    drv_segnala_guasto(reason);

    set_state(NAV_STATE_FAILURE);

} 

// funzione helper per verificare se l'obiettivo corrente è stato raggiunto
static inline bool is_objective_achieved(const movement_position_data_t cur_pos) {
    return cur_pos.obj_achieved && cur_pos.num_obj == nav_state.cur_obj.num;
}



// funzione helper che aggiunge la cella vicina sconosciuta al percorso da fare
static bool append_unknown_neighbor(void) {

    // Prendi l'ultima cella del percorso, quella è la frontiera trovata dalla ricerca
    uint16_t cur_border = nav_state.path.cells[nav_state.path.length - 1];

    // Scorri i quattro vicini di cur_border
    for (int i = 0; i < 4; i++) {

        int cur_neighbor_idx = cur_border + NEIGHBOR_SHIFT[i];
        
        // quadrante della cella di frontiera da cui il robot esce verso il vicino
        // (NAV_PATH_CENTER se la cella di frontiera è tutta libera, NAV_PATH_NO_QUAD se non c'è un quadrante adiacente libero)
        uint8_t facing_quad = get_adiacent_quad(cur_border, cur_neighbor_idx);

        // Controlla se il vicino i-esimo è sconosciuto e raggiungibile
        if (get_nav_cell_state(nav_state.map[cur_neighbor_idx]) == NAV_CELL_UNKNOWN && facing_quad != NAV_PATH_NO_QUAD) {

            // se l'ultimo quadrante del percorso e quello di uscita sono in angoli opposti, passa per il quadrante di mezzo
            uint8_t middle_quad = get_middle_quad(cur_border, nav_state.path.quads[nav_state.path.length - 1], facing_quad);
            if (middle_quad != NAV_PATH_NO_QUAD) add_to_path(cur_border, middle_quad);

            // aggiungi il quadrante di uscita dalla cella di frontiera
            // (se è il centro ed era già l'ultimo punto del percorso, add_to_path non lo ripete)
            add_to_path(cur_border, facing_quad);

            // aggiungi il centro della cella sconosciuta
            add_to_path(cur_neighbor_idx, NAV_PATH_CENTER);

            return true;

        }
    }

    // Se nessuno dei vicini è sconosciuto restituisci false, non dovremmo mai finire qui:
    // se una cella è di frontiera, ha per definizione dei vicini sconosciuti
    return false;

}

// Dopo un arresto o uno sblocco, riporta il robot al centro della sua cella se è 
// libera o al quadrante libero più vicino, altrimenti ricalcola il percorso
static void resume_after_stop(safety_alarm_t cur_alarm, movement_position_data_t cur_pos) {
    // calcola la cella di navigazione in cui si trova il robot
    uint16_t robot_index = coord_to_nav_index(cur_pos.x, cur_pos.y);

    double x = 0, y = 0;


    // se la cella in cui si trova il robot non è libera
    if (!is_nav_cell_free(robot_index)) {

        // manda l'obiettivo di fermarsi al robot
        send_objective(OBJ_STOP, 0, 0, 0, 0, 0);

        // La mappa è cambiata sotto al robot
        // ricalcola il percorso
        set_state(nav_state.mission);

        return;

    } else if (is_nav_cell_completely_free(robot_index)){        // se il robot si trova in una cella libera
        
        // ricava le coordinate del centro della cella
        nav_index_to_coord(robot_index, &x, &y);
    
    } else {                                                    // se il robot è in una cella parzialmente libera

        // vai al quadrante libero più vicino

        // coordinate del quadrante libero più vicino al robot e la sua distanza dal robot
        // (la distanza parte da un valore enorme, così il primo quadrante libero la sostituisce sempre)
        double best_dist = INFINITY;

        // scorri i 4 quadranti della cella del robot
        for (int quad = 0; quad < 4; quad++) {

            // se il quadrante è bloccato (bit a 1) passa al prossimo
            if (nav_state.map[robot_index] & NAV_CELL_QUAD[quad]) continue;

            // calcola le coordinate del centro del quadrante e la sua distanza dal robot
            double quad_x, quad_y;
            quad_to_coord(robot_index, NAV_CELL_QUAD[quad], &quad_x, &quad_y);
            double dist = hypot(quad_x - cur_pos.x, quad_y - cur_pos.y);

            // se è il più vicino trovato finora, salvalo
            if (dist < best_dist) {
                x = quad_x;
                y = quad_y;
                best_dist = dist;
            }
        }

    }

    // manda al robot l'obiettivo di andare verso il centro della cella / quadrante libero trovato
    send_objective(OBJ_MOVE, MAX_ROTATION_SPEED, 0, MIN(cur_alarm.max_v_forw, CRUISING_SPEED), x, y);

    // passa allo stato di allineamento
    set_state(NAV_STATE_ALIGN);
}

// Funzione Helper per lo sblocco del robot, fa partire la rotazione del tentativo corrente
static void unlock_start_rotation(safety_alarm_t cur_alarm, movement_position_data_t cur_pos) {

    // salva la riga della tabella del tentativo in corso 
    const unlock_step_t cur_try = UNLOCK_TABLE[nav_state.unlock_attempt - 1];

    // se questo tentativo non prevede rotazione, riprendi subito
    if (cur_try.rot_angle == 0) {
        resume_after_stop(cur_alarm, cur_pos);
        return;
    }

    // ruota sul posto
    send_objective(OBJ_ROTATE, MAX_ROTATION_SPEED, cur_pos.theta + cur_try.rot_angle, 0, 0, 0);

    // passa alla fase di rotazione e segna quando è iniziata (per il timeout)
    nav_state.unlock_phase = UNLOCK_PHASE_ROTATE;
    nav_state.phase_start = rt_adesso();
}

// Funzione Helper per lo sblocco del robot, fa andare il robot al prossimo punto della scia
static void unlock_next_back_point(safety_alarm_t cur_alarm, movement_position_data_t cur_pos) {


    // Definizione delle coordinate del prossimo punto della scia
    double x_next, y_next;

    // se il robot non deve più indietreggiare o se non ci sono altri punti nella scia, ruota il robot per questo tentativo di sblocco
    if (nav_state.back_left <= 0 || !trail_pop(&x_next, &y_next)) unlock_start_rotation(cur_alarm, cur_pos);
    else {          // altrimenti 

        // togli dalla distanza da percorrere all'indietro la distanza tra il punto poppato e la posizione del robot
        nav_state.back_left -= hypot(x_next - cur_pos.x, y_next - cur_pos.y);

        // manda al robot l'obiettivo di andare nel punto poppato
        send_objective(
            OBJ_MOVE, 
            MAX_ROTATION_SPEED, 
            0, 
            -MIN(UNLOCK_TABLE[nav_state.unlock_attempt - 1].back_speed, cur_alarm.max_v_backw), 
            x_next, 
            y_next
        );

    }

}

// Funzione Helper per lo sblocco del robot, inizia il tentativo di sblocco
static void start_unlock_attempt(safety_alarm_t cur_alarm, movement_position_data_t cur_pos) {

    // se i tentativi fatti raggiungono il numero massimo, vai in guasto ed esci
    if (nav_state.unlock_attempt >= UNLOCK_MAX_ATTEMPTS) {

        stop_and_fail(DRV_FAULT_UNLOCK_ATTEMPTS_EXHAUSTED);

        return;
    }

    // altrimenti, imposta la distanza da indietreggiare pari alla distanza del tentativo corrente
    nav_state.back_left = UNLOCK_TABLE[nav_state.unlock_attempt].back_dist;

    // incrementa il numero di tentativi fatti
    nav_state.unlock_attempt++;

    // imposta la fase di sblocco all'indietreggiamento
    nav_state.unlock_phase = UNLOCK_PHASE_BACK;

    // imposta l'istante di inizio del tentativo corrente ad adesso
    nav_state.phase_start = rt_adesso();

    // inizia ad indietreggiare
    unlock_next_back_point(cur_alarm, cur_pos);

}

// funzione helper che riporta un angolo tra -M_PI e M_PI
static inline double wrap_angle(const double angle) {
    return atan2(sin(angle), cos(angle));
}


// Funzione helper che dice se il punto del percorso idx si può attraversare senza fermarsi: serve un punto dopo,
// e la svolta tra il tratto che arriva al punto e quello che riparte deve essere piccola
static bool can_pass_through(const uint32_t idx, const movement_position_data_t cur_pos) {

    // cerca il punto di svolta dopo idx, facendo finta di essere già lì
    uint32_t saved_idx = nav_state.path_idx;
    nav_state.path_idx = idx;
    uint32_t next_idx = next_turning_point();
    nav_state.path_idx = saved_idx;

    // l'ultimo punto del percorso è sempre un arrivo
    if (next_idx == idx) return false;

    double point_x, point_y, next_x, next_y;
    get_path_coord(idx, &point_x, &point_y);
    get_path_coord(next_idx, &next_x, &next_y);

    double turn = wrap_angle(atan2(next_y - point_y, next_x - point_x) - atan2(point_y - cur_pos.y, point_x - cur_pos.x));

    return fabs(turn) < PASS_MAX_TURN;
}

// Funzione helper: dopo un allarme segna la cella del robot come ostacolo e fa partire lo sblocco
static void start_unlock_after_alarm(const safety_alarm_t cur_alarm, const movement_position_data_t cur_pos) {

    // l'incastro diventa un ostacolo sulla mappa solo adesso: se l'attraversamento riesce, il bordo del tappeto resta percorribile
    if (cur_alarm.error_mask & SAFETY_ERROR_STUCK) drv_incastro(1);

    // se non c'è l'errore SAFETY_ERROR_CONTACT segna la cella del robot come ostacolo,
    // per non incappare nello stesso allarme successivamente 
    if (!(cur_alarm.error_mask & SAFETY_ERROR_CONTACT)) {
        close_cell(coord_to_nav_index(cur_pos.x, cur_pos.y));
    }

    // durante il giro dei bordi un tentativo fallito chiude il bordo: segna come pulita la mezza cella sotto il
    // robot, altrimenti la ricerca lo rimanderebbe sempre nello stesso punto
    if (nav_state.mission == NAV_STATE_FOLLOW_BORDERS && nav_state.is_axis) {
        uint16_t alarm_cell = coord_to_nav_index(cur_pos.x, cur_pos.y);
        set_half_cell_clean(alarm_cell, get_robot_half_cell(cur_pos, alarm_cell));
    }

    // imposta lo stato corrente a sblocco
    set_state(NAV_STATE_UNLOCK);

    // comincia a fare il primo sblocco
    start_unlock_attempt(cur_alarm, cur_pos);
}

// Funzione helper che dice se provare ad attraversare l'ostacolo basso che tiene incastrato il robot: solo se va
// avanti lungo un percorso o sta uscendo dalla base ed è già staccato dai contatti 
static bool should_start_cross(const uint8_t error_mask) {

    if (!(error_mask & SAFETY_ERROR_STUCK) || (error_mask & SAFETY_ERROR_CONTACT)) return false;

    // se c'è già stato uno sblocco in questo tratto di percorso, l'attraversamento è già stato provato
    if (nav_state.unlock_attempt != 0) return false;

    if (nav_state.state == NAV_STATE_FOLLOW_PATH) return nav_state.cur_obj.v > 0;

    if (nav_state.state == NAV_STATE_LEAVE_BASE) {
        drv_stato_t cur_drv_state = {0};
        drv_stato(&cur_drv_state);
        return !cur_drv_state.sulla_base;
    }

    return false;
}

// Funzione helper che fa ruotare il robot sul posto, per mettersi di traverso rispetto alla direzione di partenza
// questo perchè il robot riesce ad attraversare alcuni ostacoli se sale prima con una ruota e poi con l'altra
static void start_cross_rotation(const double angle) {

    send_objective(OBJ_ROTATE, MAX_ROTATION_SPEED, wrap_angle(nav_state.cross_theta + angle), 0, 0, 0);

    nav_state.phase_start = rt_adesso();
}

// Funzione helper per quando l'attraversamento non riesce: dalla base non si arretra, altrove si passa allo sblocco
static void end_cross_failed(const safety_alarm_t cur_alarm, const movement_position_data_t cur_pos) {

    if (read_started_on_base() && !nav_state.has_left_base) {

        // segna che l'attraversamento è stato provato: resta il tempo massimo dell'uscita dalla base
        nav_state.unlock_attempt = 1;

        set_state(NAV_STATE_LEAVE_BASE);

    } else start_unlock_after_alarm(cur_alarm, cur_pos);
}

// Funzione per far partire il robot seguendo il percorso che ha nello stato
static void start_following_path(const safety_alarm_t cur_alarm, const movement_position_data_t cur_pos, const double lat_shift, const bool is_first_point) {

    nav_state.path_idx = 0;

    nav_state.cur_lat_shift = lat_shift;

    set_state(NAV_STATE_FOLLOW_PATH);

    if (is_first_point) {

        double lane_x, lane_y;
        
        get_path_coord(0, &lane_x, &lane_y);

        // calcola la distanza tra il robot e la corsia
        double dist = hypot(cur_pos.x - lane_x, cur_pos.y - lane_y);

        // se il robot è troppo lontano dalla corsia, mandalo lì
        if (dist > MOVE_TOLERANCE_M) send_objective(OBJ_MOVE, MAX_ROTATION_SPEED, 0, calc_speed(cur_alarm.max_v_forw), lane_x, lane_y);

    }

}


// Funzione helper che fa partire l'uscita dalla base, questo p l'ultimo momento in cui il robot ha il permesso di muoversi con i contatti chiusi
static void start_leaving_base(void) {
    nav_state.has_left_base = false;
    drv_esci_dalla_base(1);
    set_state(NAV_STATE_LEAVE_BASE);
}

// Funzione helper che fa partire l'ultimo tratto del rientro: il percorso arriva solo al centro
// della cella, che è fuori dall'asse della base, quindi il robot deve andare proprio sul punto
static void start_hook_up(const safety_alarm_t cur_alarm) {

    // azzera questo flag, perchè il flag indica che la fase di aggancio è stata demandata al driver
    // mentre in questo momento dobbiamo ancora metterci nelle condizioni di chiedere al driver l'aggancio
    nav_state.hook_has_started = false;

    send_objective(OBJ_MOVE, MAX_ROTATION_SPEED, 0, MIN(cur_alarm.max_v_forw, CRUISING_SPEED), AGG_PUNTO_X_M, 0);

    set_state(NAV_STATE_HOOK_UP);
}

// funzione helper, autoesplicativa
static void go_to_nearest_free_cell(const safety_alarm_t cur_alarm, const movement_position_data_t cur_pos, const uint16_t robot_index) {

    // cerchiamo la cella libera più vicina
    if (search_cell(robot_index, GOAL_FREE, 0) == 0) {

        // se la ricerca ritorna 0, non ci sono celle libere vicine
        // raggiungibili entro ESCAPE_MAX_CELLS, quindi stop e guasto

        stop_and_fail(DRV_FAULT_NO_ESCAPE_CELL);

    } else {
        
        // inizia a seguire il percorso (passa allo stato segui percorso)
        start_following_path(cur_alarm, cur_pos, 0, false);
    }
}

// funzione helper che restituisce true se è possibile estendere la striscia attuale
static inline bool can_extend_strip(const uint16_t cell, const bool cell_half, const double lat_shift) {
    return is_half_cell_free_dirty(cell, cell_half) && calc_clean_lane_shift(cell_half) == lat_shift;
}

// funzione helper che parte da una mezza cella e avanza sulla corsia nella direzione passata finché la corsia è
// sporca e percorribile: restituisce quante celle ha fatto e salva l'ultima cella raggiunta in end_cell
static uint32_t find_lane_end(const uint16_t cell, const bool cell_half, const clean_dir_t dir, const double lat_shift, uint16_t *end_cell) {

    uint32_t count = 0;

    uint16_t next_cell;
    bool next_half;

    *end_cell = cell;

    get_half_cell_neighbor(cell, cell_half, dir, &next_cell, &next_half);

    while (can_extend_strip(next_cell, next_half, lat_shift)) {

        *end_cell = next_cell;
        count++;

        get_half_cell_neighbor(next_cell, next_half, dir, &next_cell, &next_half);
    }

    return count;
}

// funzione che prende una mezza cella della corsia da pulire e scrive la striscia da percorrere dentro nav_state.path, restituendo la lunghezza
static uint32_t plan_strip(const uint16_t lane_cell, const bool start_half) {

    double lat_shift = calc_clean_lane_shift(start_half);

    // trova i due capi della corsia sporca, così la striscia la percorre tutta e la corsia non si spezza
    uint16_t north_end, south_end;
    uint32_t north_count = find_lane_end(lane_cell, start_half, CLEAN_DIR_NORTH, lat_shift, &north_end);
    uint32_t south_count = find_lane_end(lane_cell, start_half, CLEAN_DIR_SOUTH, lat_shift, &south_end);

    // parti dal capo più vicino alla cella passata e vai verso l'altro
    const uint16_t start_cell = (north_count < south_count) ? north_end : south_end;
    const clean_dir_t dir = (north_count < south_count) ? CLEAN_DIR_SOUTH : CLEAN_DIR_NORTH;

    // calcola la prima cella dopo quella di inizio
    uint16_t next_cell;
    bool next_half;
    get_half_cell_neighbor(start_cell, start_half, dir, &next_cell, &next_half);

    // calcola la cella prima dell'inizio della striscia, sulla stessa corsia
    uint16_t back_cell;
    bool back_half;
    get_half_cell_neighbor(start_cell, start_half, dir == CLEAN_DIR_NORTH ? CLEAN_DIR_SOUTH : CLEAN_DIR_NORTH, &back_cell, &back_half);

    uint32_t length = 0;

    // se dietro all'inizio della striscia c'è un ostacolo, il percorso parte dal punto indietro:
    // lì il rullo non passerebbe mai, così la corsia viene pulita dall'estremità.
    // se invece la cella dietro è percorribile non serve, un'altra striscia ci passa comunque
    nav_state.strip_has_back = !is_half_cell_free_walkable(back_cell, back_half) && !is_nav_cell_invisible_obstacle(back_cell);

    if (nav_state.strip_has_back) {
        nav_state.path.cells[length] = back_cell;
        nav_state.path.quads[length] = NAV_PATH_CENTER;
        length++;
    }

    // la cella d'inizio della striscia
    nav_state.path.cells[length] = start_cell;
    nav_state.path.quads[length] = NAV_PATH_CENTER;
    length++;

    // allunga la striscia nella direzione scelta finché si può
    while (can_extend_strip(next_cell, next_half, lat_shift)) {

        // aggiungi la cella alla striscia
        nav_state.path.cells[length] = next_cell;
        nav_state.path.quads[length] = NAV_PATH_CENTER;
        length++;

        // passa alla vicina successiva nella stessa direzione
        get_half_cell_neighbor(next_cell, next_half, dir, &next_cell, &next_half);
    }

    // se dopo la fine della striscia c'è un ostacolo, l'ultimo punto è la cella successiva sulla stessa corsia, questo
    // serve per assicurarsi che non rimangano spazi sporchi tra la zona interna della stanza, e i bordi lungo gli ostacoli
    nav_state.strip_has_end = !is_half_cell_free_walkable(next_cell, next_half) && !is_nav_cell_invisible_obstacle(next_cell);

    if (nav_state.strip_has_end) {
        nav_state.path.cells[length] = next_cell;
        nav_state.path.quads[length] = NAV_PATH_CENTER;
        length++;
    }

    // salva la lunghezza della striscia nel percorso
    nav_state.path.length = length;

    return length;
}

// funzione helper che calcola la velocità di rotazione del giro dei bordi dall'errore di direzione
static inline double calc_border_omega(const double dir_error) {
    return MAX(-BORDER_MAX_OMEGA, MIN(BORDER_ROTATION_GAIN * dir_error, BORDER_MAX_OMEGA));
}

// funzione che restituisce di quanto ha ruotato il robot da quando il lidar ha letto la scansione, in radianti:
// i dati del lidar possono essere vecchi di 300 ms, se nel frattempo il robot ha girato bisogna tenerne conto
static double calc_turn_since_scan(const int64_t scan_end_t, const movement_position_data_t cur_pos) {

    // leggi lo storico delle ultime pose
    movement_odom_history_t history;
    read_odom_history(&history);

    // l'istante medio della scansione, i punti sono stati letti lungo tutto il giro
    const int64_t scan_t = scan_end_t - SCAN_HALF_TIME;

    // cerca la posa con l'istante più vicino
    double scan_theta = cur_pos.odom_theta;
    int64_t best_diff = INT64_MAX;

    for (uint32_t i = 0; i < history.count; i++) {

        int64_t diff = llabs(history.pose[i].t_ns - scan_t);

        if (diff < best_diff) {
            best_diff = diff;
            scan_theta = history.pose[i].theta;
        }
    }

    return wrap_angle(cur_pos.odom_theta - scan_theta);
}

// funzione che prepara un nuovo giro attorno a un ostacolo e passa allo stato del giro
static void start_border_lap(const movement_position_data_t cur_pos) {

    nav_state.is_border_hooked = false;
    nav_state.border_no_new_dist = 0;
    nav_state.border_last_x = cur_pos.x;
    nav_state.border_last_y = cur_pos.y;
    nav_state.border_lost_start = 0;

    set_state(NAV_STATE_BORDER_LAP);
}

// funzione che termina il giro attorno all'ostacolo
static void end_border_lap(void) {

    // manda il segnale di stop per fermare il robot
    send_objective(OBJ_STOP, 0, 0, 0, 0, 0);

    // imposta lo stato della navigazione allo stato precedente all'inizio del giro
    set_state(nav_state.mission);
}

// funzione helper che restituisce true se il robot sta andando verso un punto di estremità della striscia
// (il punto indietro o l'ultimo punto) e deve fermarsi lì: quei punti possono stare dentro un ostacolo, quindi il
// robot non ci arriva e fermarsi è il comportamento normale, non un problema da risolvere
static bool is_strip_end_reached(const safety_alarm_t cur_alarm, const movement_position_data_t cur_pos) {

    // controlla che il percorso sia una striscia e che il robot stia andando verso un punto di estremità
    if (
        !is_strip_path() ||
        !((nav_state.strip_has_back && nav_state.path_idx == 0) ||
        (nav_state.strip_has_end && nav_state.path_idx == nav_state.path.length - 1))
    ) return false;

    // calcola il punto di estremità
    double end_x, end_y;
    get_path_coord(nav_state.path_idx, &end_x, &end_y);

    // se la cella dopo la striscia è libera il robot taglia dritto e questo punto può essere ancora lontano:
    // uno stallo a metà striscia non è la fine, deve essere gestito come tutti gli altri stalli
    if (hypot(end_x - cur_pos.x, end_y - cur_pos.y) > STRIP_END_ZONE) return false;

    // il robot deve guardare verso il punto: mentre ruota, il corridoio davanti può passare su un muro laterale
    if (fabs(wrap_angle(atan2(end_y - cur_pos.y, end_x - cur_pos.x) - cur_pos.theta)) > ALIGNMENT_THRESHOLD) return false;

    // il robot si ferma se davanti c'è un ostacolo o se la sicurezza lo sta trattenendo
    lidar_scan_data_t scan = read_lidar();

    return (scan.valid && scan.corridor_front_distance <= STRIP_END_FRONT_DISTANCE) || is_stalled(cur_alarm, cur_pos);
}


// funzione che cerca da dove far partire la pulizia a strisce: tra tutte le mezze celle sporche raggiungibili dal robot
// prende quelle con la coordinata laterale più bassa e più alta, cioè i due estremi della stanza, e sceglie la più vicina al
// robot. restituisce true se ha trovato una mezza cella e ne salva la cella in start_cell
static bool find_clean_start_cell(const uint16_t robot_cell, const movement_position_data_t cur_pos, uint16_t *start_cell) {

    // una ricerca che non trova niente visita tutte le celle raggiungibili e le lascia nella coda
    search_cell(robot_cell, GOAL_CELL, CELL_NOT_VISITED);

    bool found = false;
    double min_lateral = 0, max_lateral = 0;
    uint16_t min_cell = 0, max_cell = 0;

    // scorri tutte le celle raggiungibili
    for (uint32_t i = 0; i < nav_state.s_q.tail; i++) {

        const uint16_t cell = nav_state.s_q.values[i];

        // calcola il centro della cella
        double cell_x, cell_y;
        nav_index_to_coord(cell, &cell_x, &cell_y);

        // scorri le due mezze celle
        for (int half = 0; half < 2; half++) {

            if (!is_half_cell_free_dirty(cell, half)) continue;

            // calcola la coordinata laterale della mezza cella
            double lateral = (nav_state.strip_axis_x ? cell_y : cell_x) + calc_clean_lane_shift(half);

            if (!found || lateral < min_lateral) { min_lateral = lateral; min_cell = cell; }
            if (!found || lateral > max_lateral) { max_lateral = lateral; max_cell = cell; }

            found = true;
        }
    }

    if (!found) return false;

    // scegli l'estremo più vicino al robot
    double robot_lateral = nav_state.strip_axis_x ? cur_pos.y : cur_pos.x;
    *start_cell = (fabs(robot_lateral - min_lateral) <= fabs(robot_lateral - max_lateral)) ? min_cell : max_cell;

    return true;
}


// funzione helper che restitusice true se bisogna attivare spazzole e aspirazione, false altrimenti
// l'aspirazione si attiva se stiamo pulendo (bordi o interno della stanza) e non siamo in stato di 
// guasto, sblocco, attesa o riposo
// funzione helper che legge i comandi arrivati dall'ultimo job: per ora le pressioni dei tasti contate dal driver,
// più avanti anche i comandi dell'app
static void read_commands(const drv_stato_t *cur_drv_state, bool *power_pressed, bool *home_pressed) {

    *power_pressed = cur_drv_state->pressioni_accensione != nav_state.last_power_presses;
    *home_pressed = cur_drv_state->pressioni_casa != nav_state.last_home_presses;

    nav_state.last_power_presses = cur_drv_state->pressioni_accensione;
    nav_state.last_home_presses = cur_drv_state->pressioni_casa;
}

// funzione helper che fa partire una nuova pulizia: azzera quello che riguarda la pulizia di prima e chiede alla mappatura
// di azzerare quello che in una pulizia non si aggiorna più (celle chiuse comprese). Se la mappa c'è già salta l'esplorazione.
// L'attesa poi fa uscire il robot dalla base, se ci si trova
static void start_new_mission(void) {

    memset(nav_state.clean_cell_mask, 0, sizeof nav_state.clean_cell_mask);
    nav_state.is_axis = false;
    nav_state.is_clean_started = false;
    nav_state.interrupted_mission = 0;
    nav_state.unlock_attempt = 0;
    nav_state.trail.len = 0;
    nav_state.has_left_base = false;

    // celle chiuse: la lista ricomincia, e il numero di pulizia nuovo dice alla mappatura di azzerare le sue
    nav_state.closed_cells.num = 0;
    nav_state.closed_cells.mission_num++;
    scambio_scrivi(&double_buffer_2, &nav_state.closed_cells);

    drv_led(DRV_LED_AZZURRO);

    set_state(NAV_STATE_WAIT);

    // la missione parte sempre dall'esplorazione: con una mappa completa non trova frontiere e passa subito ai bordi,
    // se la mappa ha zone nuove (per esempio dietro una porta aperta) esplora solo quelle. Dalla sua ultima ricerca
    // viene anche l'asse delle strisce
    nav_state.mission = NAV_STATE_EXPLORE;
}

// funzione helper che chiede al task aperiodico di salvare la mappa, se si può: sessione partita dalla base (la mappa
// ha l'origine nella base), esplorazione finita almeno una volta (una mappa a metà farebbe saltare zone alla volta
// dopo) e posizione sulla mappa normale (non persa né da confermare)
static void request_map_save_if_allowed(void) {
    if (read_started_on_base() && nav_state.is_map_ready && read_mapping_correction().state == MAP_NORMAL)
        request_map_save();
}

// funzione helper che mette in pausa: il robot si ferma dov'è, la missione resta quella di prima
static void start_pause(void) {

    send_objective(OBJ_STOP, 0, 0, 0, 0, 0);

    // non deve muoversi con i contatti chiusi, e se il driver stava facendo l'aggancio la manovra si annulla
    drv_esci_dalla_base(0);
    if (nav_state.state == NAV_STATE_HOOK_UP && nav_state.hook_has_started) drv_aggancia(0);
    nav_state.hook_has_started = false;

    drv_led(DRV_LED_AZZURRO_RESPIRA);

    set_state(NAV_STATE_PAUSE);
}

// funzione helper che manda il robot alla base subito: la missione finisce (alla base andrà in riposo)
static void go_home_now(void) {

    send_objective(OBJ_STOP, 0, 0, 0, 0, 0);

    drv_esci_dalla_base(0);
    nav_state.interrupted_mission = 0;

    drv_led(DRV_LED_AZZURRO);

    set_state(NAV_STATE_RETURN_HOME);
}

// funzione helper che, con la posizione persa (ricerca in corso o robot in aria), porta il robot alla base alla cieca:
// la missione finisce, la mappatura annulla la ricerca e aspetta la base, e il driver fa l'aggancio a infrarossi da dove
// si trova il robot (senza percorso: la posizione non c'è). Se l'aggancio non riesce il robot va in guasto
static void start_blind_hook_up(void) {

    send_objective(OBJ_STOP, 0, 0, 0, 0, 0);

    drv_esci_dalla_base(0);
    nav_state.interrupted_mission = 0;

    drv_led(DRV_LED_AZZURRO);

    // la mappatura annulla la ricerca (se gira) e aspetta che il robot sia sulla base
    nav_state.closed_cells.stop_search_num++;
    scambio_scrivi(&double_buffer_2, &nav_state.closed_cells);

    // aggancio a infrarossi: lo fa il driver, la navigazione aspetta l'esito
    drv_aggancia(1);
    nav_state.hook_has_started = true;
    set_state(NAV_STATE_HOOK_UP);
    nav_state.mission = NAV_STATE_RETURN_HOME;
}

// funzione che applica i comandi (tasto di accensione e tasto casa) in base allo stato. Il guasto lo azzera già il driver
// alla pressione: qui, se il guasto non c'è più, si passa al riposo e il tasto fa quello che farebbe in riposo
static void apply_commands(const drv_stato_t *cur_drv_state, bool power_pressed, bool home_pressed) {

    // guasto azzerato (da un tasto o perché il robot è stato rimesso sulla base): riposo
    if (nav_state.state == NAV_STATE_FAILURE) {
        if (cur_drv_state->fase == DRV_GUASTO) return;
        drv_led(DRV_LED_AZZURRO);
        set_state(NAV_STATE_SLEEP);
    }

    // posizione persa: qualsiasi tasto annulla la ricerca e porta il robot alla base alla cieca
    // Se l'aggancio alla cieca è già partito, i tasti non fanno niente
    if ((power_pressed || home_pressed) && read_mapping_correction().state == MAP_LOST &&
        !(nav_state.state == NAV_STATE_HOOK_UP && nav_state.hook_has_started)) {
        start_blind_hook_up();
        return;
    }

    switch (nav_state.state) {

        case NAV_STATE_SLEEP:
            // accensione: nuova pulizia. casa: sulla base niente, fuori rientra
            if (power_pressed) start_new_mission();
            else if (home_pressed && !cur_drv_state->sulla_base) go_home_now();
            break;

        case NAV_STATE_PAUSE:
            // accensione: riprende passando dall'attesa, che torna alla missione (o all'uscita dalla base). casa: rientra
            if (power_pressed) {
                drv_led(DRV_LED_AZZURRO);
                set_state(NAV_STATE_WAIT);
            } else if (home_pressed) go_home_now();
            break;

        case NAV_STATE_RETURN_HOME:
        case NAV_STATE_HOOK_UP:
            // sta già tornando alla base: casa non fa niente, accensione mette in pausa
            if (power_pressed) start_pause();
            break;

        default:
            // in missione: accensione mette in pausa, casa rientra. Se il robot non è ancora uscito dalla base, casa lo
            // lascia lì in riposo
            if (power_pressed) start_pause();
            else if (home_pressed) {
                if (cur_drv_state->sulla_base) {
                    send_objective(OBJ_STOP, 0, 0, 0, 0, 0);
                    drv_esci_dalla_base(0);
                    set_state(NAV_STATE_SLEEP);
                } else go_home_now();
            }
            break;
    }
}

static bool should_activate_brushes() {
    return (
        (nav_state.mission == NAV_STATE_CLEAN || nav_state.mission == NAV_STATE_FOLLOW_BORDERS) &&
        (
            nav_state.state != NAV_STATE_UNLOCK &&
            nav_state.state != NAV_STATE_FAILURE &&
            nav_state.state != NAV_STATE_WAIT &&
            nav_state.state != NAV_STATE_SLEEP &&
            nav_state.state != NAV_STATE_LEAVE_BASE &&
            nav_state.state != NAV_STATE_PAUSE
        )
    );
}

// funzione helper che dice se lo sblocco della spazzola centrale è in corso 
static inline bool is_brush_unlocking() {
    return nav_state.brush_unlock_start != 0 && elapsed_time(nav_state.brush_unlock_start) < BRUSH_UNLOCK_TIME;
}

// funzione helper che, se la sicurezza segnala la spazzola centrale bloccata, avvia il suo sblocco oppure manda il robot in guasto
static void check_brush_unlock(const safety_alarm_t cur_alarm) {

    // se la spazzola non è bloccata non c'è niente da fare
    if (!(cur_alarm.error_mask & SAFETY_ERROR_BRUSH)) return;

    // se la spazzola è bloccata anche mentre gira al contrario, lo sblocco non serve, quindi va in guasto
    if (is_brush_unlocking()) {
        stop_and_fail(DRV_FAULT_BRUSH_STUCK);
        return;
    }

    // se dopo l'ultimo sblocco la spazzola ha girato normalmente per BRUSH_UNLOCK_RESET_TIME, quello sblocco era riuscito:
    // questo è un blocco nuovo e i tentativi ripartono da zero
    if (nav_state.brush_unlock_start == 0 || elapsed_time(nav_state.brush_unlock_start) >= BRUSH_UNLOCK_TIME + BRUSH_UNLOCK_RESET_TIME) nav_state.brush_unlock_attempt = 0;

    // se ha già fatto BRUSH_UNLOCK_MAX_ATTEMPTS sblocchi di fila e la spazzola è ancora bloccata, quindi va in guasto
    if (nav_state.brush_unlock_attempt >= BRUSH_UNLOCK_MAX_ATTEMPTS) {
        stop_and_fail(DRV_FAULT_BRUSH_STUCK);
        return;
    }

    // avvia un nuovo sblocco
    nav_state.brush_unlock_attempt++;
    nav_state.brush_unlock_start = rt_adesso();

}

// job del task
void job_navigation(void *arg) {
    
    (void)arg;

    // Se la mappa è cambiata, salva la versione aggiornata e il numero di versione aggiornato
    if (read_nav_map_version_number() != nav_state.map_version) {
    
        nav_state.map_version = read_nav_map(nav_state.map);

        // rimetti nella copia le celle chiuse, che la mappatura potrebbe non aver ancora ricevuto
        for (uint32_t i = 0; i < MIN(nav_state.closed_cells.num, CLOSED_CELLS_MAX); i++) {
            nav_state.map[nav_state.closed_cells.cells[i]] = set_nav_cell_state(nav_state.map[nav_state.closed_cells.cells[i]], NAV_CELL_TOO_NARROW);
        }

    }

    // leggi la posizione corrente dal task di movimento
    movement_position_data_t cur_pos = read_movement();

    // leggi lo stato del driver: guasto, tasti, base
    drv_stato_t cur_drv_state = {0};
    drv_stato(&cur_drv_state);

    // se il driver è in guasto (chiesto da un task o suo) la navigazione si ferma in guasto
    if (cur_drv_state.fase == DRV_GUASTO && nav_state.state != NAV_STATE_FAILURE) {
        send_objective(OBJ_STOP, 0, 0, 0, 0, 0);
        set_state(NAV_STATE_FAILURE);
    }

    // leggi e applica i comandi: si fa prima di tutto il resto, anche con la posizione non valida
    bool power_pressed, home_pressed;
    read_commands(&cur_drv_state, &power_pressed, &home_pressed);
    if (power_pressed || home_pressed || nav_state.state == NAV_STATE_FAILURE) apply_commands(&cur_drv_state, power_pressed, home_pressed);

    // leggi gli allarmi mandati dal task safety, servono per lo sblocco della spazzola e per la velocità massima degli obiettivi
    safety_alarm_t cur_alarm = read_safety();

    // se la spazzola centrale è bloccata avvia il suo sblocco, o manda il robot in guasto
    if (should_activate_brushes()) check_brush_unlock(cur_alarm);

    // se bisogna attivare spazzole e aspirazione attivali, altrimenti disattivali
    if (should_activate_brushes()) {
        drv_aspirazione(SUCTION_POWER_NORMAL);

        // durante lo sblocco la spazzola centrale gira al contrario, la laterale continua normalmente
        if (is_brush_unlocking()) drv_spazzole(BRUSH_SPEED_NORMAL, BRUSH_SPEED_UNLOCK);
        else drv_spazzole(BRUSH_SPEED_NORMAL, BRUSH_SPEED_NORMAL);
    } else {
        drv_aspirazione(SUCTION_POWER_OFF);
        drv_spazzole(BRUSH_SPEED_OFF, BRUSH_SPEED_OFF);
    }

    // se la posizione corrente non è valida
    if (!cur_pos.valid) {

        // aggancio alla cieca in corso (tasto premuto con la posizione persa): si aspetta solo l'esito del driver
        if (nav_state.state == NAV_STATE_HOOK_UP && nav_state.hook_has_started) {
            if (cur_drv_state.aggancio == AGG_FATTO) set_state(NAV_STATE_SLEEP);
            else if (cur_drv_state.aggancio == AGG_FALLITO) stop_and_fail(DRV_FAULT_DOCKING_FAILED);
            return;
        }
        
        // se l'obiettivo corrente non è già OBJ_STOP, aggiungi l'obiettivo OBJ_STOP
        if (nav_state.cur_obj.obj_type != OBJ_STOP) send_objective(OBJ_STOP, 0, 0, 0, 0, 0);

        // se il robot stava uscendo dalla base ormai non ci è più: togli il permesso di muoversi con i contatti chiusi.
        // se viene riappoggiato sulla base, l'attesa fa ripartire l'uscita
        if (read_started_on_base() && !nav_state.has_left_base) {
            drv_esci_dalla_base(0);
            nav_state.has_left_base = true;
        }
        
        // metti il robot in attesa, a meno che non stia già aspettando un comando (riposo, pausa, guasto):
        // riappoggiato, deve restare lì e non ripartire da solo
        if (nav_state.state != NAV_STATE_SLEEP && nav_state.state != NAV_STATE_PAUSE && nav_state.state != NAV_STATE_FAILURE) set_state(NAV_STATE_WAIT);

        // azzera la scia delle posizioni
        nav_state.trail.len = 0;

        // esci
        return;
    }

    // calcola la distanza tra la posizione attuale del robot e l'ultima posizione
    // salvata nel buffer della scia delle posizioni.
    // se il buffer della scia delle posizioni è vuoto o la distanza dall'ultima
    // posizione è almeno TRAIL_STEP, aggiungi la posizione corrente al buffer
    if ((nav_state.trail.len <= 0 || hypot(nav_state.trail.x[(nav_state.trail.head + TRAIL_SIZE - 1) % TRAIL_SIZE] - cur_pos.x, nav_state.trail.y[(nav_state.trail.head + TRAIL_SIZE - 1) % TRAIL_SIZE] - cur_pos.y) >= TRAIL_STEP) && nav_state.state != NAV_STATE_UNLOCK) {       // btw spero che nessuno legga mai questo schifo di if
        trail_push(cur_pos.x, cur_pos.y);
    }

    // se la pulizia è in corso, segna come pulite le mezze celle su cui si trova il robot
    if (is_active_cleaning()) set_robot_cell_clean(cur_pos);

    // l'incastro si segnala alla mappa solo quando parte lo sblocco
    drv_incastro(0);

    // se il robot è incastrato su qualcosa di basso, prima dello sblocco prova ad attraversarlo
    if (should_start_cross(cur_alarm.error_mask)) {
        nav_state.cross_theta = cur_pos.theta;
        set_state(NAV_STATE_CROSS);
        start_cross_rotation(CROSS_ANGLE);
        return;
    }

    // se ci sono degli allarmi (stuck, slip o motor overload) e lo stato non è 
    // attesa, sblocco, riposo o guasto, vai in sblocco
    // il primo controllo dentro l'if verifica se uno o più dei 3 bit meno significativi siano a 1
    if ((cur_alarm.error_mask & (SAFETY_ERROR_STUCK | SAFETY_ERROR_SLIP | SAFETY_ERROR_MOT_OVRLD | SAFETY_ERROR_CONTACT)) && 
        nav_state.state != NAV_STATE_SLEEP &&
        nav_state.state != NAV_STATE_UNLOCK &&
        nav_state.state != NAV_STATE_WAIT &&
        nav_state.state != NAV_STATE_FAILURE &&
        nav_state.state != NAV_STATE_HOOK_UP &&
        nav_state.state != NAV_STATE_LEAVE_BASE &&
        nav_state.state != NAV_STATE_CROSS &&
        nav_state.state != NAV_STATE_PAUSE) {

        start_unlock_after_alarm(cur_alarm, cur_pos);

        return;
    }

    // controlla se la batteria è scarica
    if (is_battery_dead()) {
        
        // salva la missione attuale
        nav_state.interrupted_mission = nav_state.mission;

        // ferma il robot
        send_objective(OBJ_STOP, 0, 0, 0, 0, 0);

        // manda il robot alla base di ricarica
        set_state(NAV_STATE_RETURN_HOME);

        return;
    }

    // Switch della macchina a stati
    switch (nav_state.state) {
        case NAV_STATE_WAIT: {
            // se l'obiettivo non è di stare fermo
            if (nav_state.cur_obj.obj_type != OBJ_STOP) {
                // manda l'obiettivo di fermarsi
                send_objective(OBJ_STOP, 0, 0, 0, 0, 0);
                // esci per non fare l'if dopo
                return;
            }

            // se il robot è stato fermo per più di WAIT_TIME
            if (elapsed_time(nav_state.state_start)>= WAIT_TIME) {
                // leggi lo stato del driver, serve a sapere se il robot è sulla base

                // se il robot è partito dalla base e non ne è ancora uscito, esce per prima cosa.
                // lo stesso se è stato sollevato e riappoggiato sulla base
                if (read_started_on_base() && (!nav_state.has_left_base || cur_drv_state.sulla_base)) start_leaving_base();
                // altrimenti passa al prossimo stato
                else set_state(nav_state.mission);
            }
            
            break;
        }
        case NAV_STATE_EXPLORE: {
            
            // calcola la cella in cui si trova il robot
            uint16_t robot_index = coord_to_nav_index(cur_pos.x, cur_pos.y);

            // se la cella della mappa sulla quale si trova il robot è sconosciuta segnala come libera.
            // quest'operazione verrà fatta poi dal task di mappatura, ma siccome 
            // è possibile che il task di navigazione parta prima e riceva una mappa
            // non aggiornata, lo segnamo anche nella mappa privata di questo task
            if (get_nav_cell_state(nav_state.map[robot_index]) == NAV_CELL_UNKNOWN) nav_state.map[robot_index] = NAV_CELL_FREE;
            
            // se il robot si trova in una cella non libera
            if (!is_nav_cell_free(robot_index)) {

                go_to_nearest_free_cell(cur_alarm, cur_pos, robot_index);

                break;

            }
            
            
            
            // cerca i confini della mappa
            uint32_t dist = search_cell(robot_index, GOAL_BORDER, 0);

            if (dist == 0) {        // se dist == 0, l'esplorazione è finita
                
                // l'asse delle strisce si calcola adesso, finché la coda contiene le celle raggiungibili dell'ultima
                // ricerca (quella che non ha trovato frontiere), poi la mappa completa si salva
                calc_strip_axis();
                nav_state.is_map_ready = true;
                request_map_save_if_allowed();

                // passa al giro dei bordi
                set_state(NAV_STATE_FOLLOW_BORDERS); 

            } else { // Se esiste un percorso verso la frontiera

                // aggiungi la cella sconosciuta oltre la frontiera alla zona da visitare
                if (!append_unknown_neighbor()) {
                    
                    // Se entriamo qui dentro significa che una cella di frontiera non 
                    // ha una cella vicina sconosciuta (impossibile)

                    stop_and_fail(DRV_FAULT_INCONSISTENT_STATE);

                    break;
                }

                start_following_path(cur_alarm, cur_pos, 0, false);

            }

            break;
        }
        case NAV_STATE_CLEAN: {

            // per prima cosa calcoliamo l'asse se non è già stato calcolato
            if (!nav_state.is_axis) { 
                calc_strip_axis();
                break;
            }

            // calcola la cella del robot
            uint16_t robot_cell = coord_to_nav_index(cur_pos.x, cur_pos.y);

            // se la cella del robot non è libera, vai alla cella libera più vicina ed esci
            if (!is_nav_cell_free(robot_cell)) {
                go_to_nearest_free_cell(cur_alarm, cur_pos, robot_cell);
                break;
            }

            // la prima striscia parte da un estremo della stanza, le altre proseguono corsia dopo corsia
            if (!nav_state.is_clean_started) {

                nav_state.is_clean_started = true;

                uint16_t clean_start_cell;

                if (find_clean_start_cell(robot_cell, cur_pos, &clean_start_cell) && search_cell(robot_cell, GOAL_CELL, clean_start_cell) != 0) {
                    start_following_path(cur_alarm, cur_pos, 0, false);
                    break;
                }
            }

            // calcola la metà della cella in cui si trova il robot
            bool robot_half_cell = get_robot_half_cell(cur_pos, robot_cell);

            // la mezza cella di partenza è quella del robot se è libera e sporta
            if (is_half_cell_free_dirty(robot_cell, robot_half_cell)) {
                
                plan_strip(robot_cell, robot_half_cell);

                start_following_path(cur_alarm, cur_pos, calc_clean_lane_shift(robot_half_cell), true);

            } else {

                // scorri le 4 direzioni in cui bisogna controllare (N, S, E, W)
                for (clean_dir_t dir = 0; dir < 4; dir++) {

                    uint16_t start_cell;
                    bool start_half_cell;

                    // trova la mezza cella da cui il robot deve partire
                    get_half_cell_neighbor(robot_cell, robot_half_cell, dir, &start_cell, &start_half_cell);

                    // controlla se questo vicino è libero e sporco
                    if (is_half_cell_free_dirty(start_cell, start_half_cell)) {
                        
                        // se sì, pulisci lui
                        plan_strip(start_cell, start_half_cell);
                        start_following_path(cur_alarm, cur_pos, calc_clean_lane_shift(start_half_cell), true);
                        return;
                    } 

                }

                // Se arriviamo qui significa che non abbiamo trovato una cella vicina da pulire, quindi
                // siamo in un vicolo cieco e dobbiamo cercare una cella nel resto della mappa
                if (search_cell(robot_cell, GOAL_DIRTY, 0) == 0) {  // la funzione restituisce 0 se non trova niente
                    // quindi se entriamo qui dentro significa che la pulizia è terminata,
                    // facciamo tornare il robot alla base di ricarica
                    set_state(NAV_STATE_RETURN_HOME);
                    break;
                }

                // Se invece siamo qui significa che search_cell ha trovato una cella da pulire, quindi
                // andiamo a pulirla
                start_following_path(cur_alarm, cur_pos, 0, false); 

            }

            break;
        }
        case NAV_STATE_FOLLOW_BORDERS: {

            // per prima cosa calcoliamo l'asse se non è già stato calcolato, deve venire prima di
            // ogni altra ricerca perchè usa la coda lasciata dall'ultima ricerca dell'esplorazione
            if (!nav_state.is_axis) {
                calc_strip_axis();
                break;
            }

            // calcola la cella del robot
            uint16_t robot_cell = coord_to_nav_index(cur_pos.x, cur_pos.y);

            // se la cella del robot non è libera, vai alla cella libera più vicina ed esci
            if (!is_nav_cell_free(robot_cell)) {
                go_to_nearest_free_cell(cur_alarm, cur_pos, robot_cell);
                break;
            }

            // se il robot si trova già su un bordo da pulire, inizia il giro
            if (cell_is_goal(GOAL_EDGE, robot_cell, 0)) {
                start_border_lap(cur_pos);
                break;
            }

            // cerca il bordo da pulire più vicino
            if (search_cell(robot_cell, GOAL_EDGE, 0) == 0) {
                // se non ce ne sono più, il giro dei bordi è finito: passa alla pulizia
                set_state(NAV_STATE_CLEAN);
            } else {
                // altrimenti vai al bordo: alla fine del percorso si torna qui e parte il giro
                start_following_path(cur_alarm, cur_pos, 0, false);
            }

            break;
        }
        case NAV_STATE_BORDER_LAP: {

            // segna come pulito quello che il robot ha sotto, viene prima di tutto il resto
            // così il punto di arrivo risulta pulito anche se il giro finisce subito
            bool new_clean = set_border_band_clean(cur_pos);

            // aggiorna la strada percorsa
            nav_state.border_no_new_dist += hypot(cur_pos.x - nav_state.border_last_x, cur_pos.y - nav_state.border_last_y);
            nav_state.border_last_x = cur_pos.x;
            nav_state.border_last_y = cur_pos.y;

            // se il robot sta pulendo qualcosa di nuovo, azzera la strada e i tentativi di sblocco
            if (new_clean) {
                nav_state.border_no_new_dist = 0;
                nav_state.unlock_attempt = 0;
            }

            // rete di sicurezza contro i giri infiniti
            if (elapsed_time(nav_state.state_start) >= BORDER_LAP_TIMEOUT) {
                end_border_lap();
                break;
            }

            // leggi i dati del lidar e di quanto ha ruotato il robot da quella scansione
            lidar_scan_data_t scan = read_lidar();
            double turn = calc_turn_since_scan(scan.scan_end_t, cur_pos);

            // se il robot non è ancora agganciato, ruota sul posto finché ha l'ostacolo a destra
            if (!nav_state.is_border_hooked) {

                // se non c'è niente da seguire il giro finisce
                if (!scan.valid || !scan.nearest_border) {
                    end_border_lap();
                    break;
                }

                // quanto manca per avere l'ostacolo a destra (a -90 gradi)
                double hook_error = wrap_angle(scan.nearest_angle - turn + M_PI / 2);

                if (fabs(hook_error) < BORDER_HOOK_TOLERANCE) {
                    nav_state.is_border_hooked = true;
                } else {
                    send_objective(OBJ_CONST_SPEED, calc_border_omega(hook_error), 0, 0, 0, 0);
                }

                break;
            }

            // se il robot non vede più l'ostacolo si ferma, e dopo BORDER_LOST_TIME il giro finisce
            if (!scan.valid || !scan.border_found) {

                send_objective(OBJ_CONST_SPEED, 0, 0, 0, 0, 0);

                if (nav_state.border_lost_start == 0) nav_state.border_lost_start = rt_adesso();

                if (elapsed_time(nav_state.border_lost_start) >= BORDER_LOST_TIME) end_border_lap();

                break;
            }

            nav_state.border_lost_start = 0;

            // calcola l'errore tra la direzione in cui andare e quella attuale
            double dir_error = wrap_angle(scan.border_heading - turn);

            // più l'errore è grande meno il robot avanza, da BORDER_SLOW_ANGLE in su ruota soltanto
            double next_v = MAX(BORDER_SPEED * (1 - fabs(dir_error) / BORDER_SLOW_ANGLE), 0);

            // manda velocità e rotazione, senza superare la velocità massima decisa dalla sicurezza
            send_objective(OBJ_CONST_SPEED, calc_border_omega(dir_error), 0, MIN(next_v, cur_alarm.max_v_forw), 0, 0);

            // il giro finisce se il robot non pulisce più niente di nuovo, o se la sicurezza lo sta trattenendo
            if (nav_state.border_no_new_dist >= BORDER_NO_NEW_DISTANCE || is_stalled(cur_alarm, cur_pos)) {
                end_border_lap();
            }

            break;
        }
        case NAV_STATE_FOLLOW_PATH: {

            // aggiorna la velocità come la più piccola tra velocità di crociera e cur_alarm.max_v_forw
            nav_state.cur_obj.v = calc_speed(cur_alarm.max_v_forw);

            // pubblica la velocità aggiornata
            write_objective(&nav_state.cur_obj);

            // alle estremità di una striscia il robot può fermarsi prima di arrivare al punto
            bool strip_end_reached = is_strip_end_reached(cur_alarm, cur_pos);

            // Controlla se l'obiettivo è stato raggiunto
            if (is_objective_achieved(cur_pos) || strip_end_reached) {

                // Salva il prossimo punto di svolta
                int next_turn_pt = next_turning_point();

                // Controlla se il percorso è finito
                if (next_turn_pt == nav_state.path_idx) {

                    // Azzera i tentativi di sblocco
                    nav_state.unlock_attempt = 0;

                    // Azzera center_to_center, da adesso si può tagliare di nuovo il percorso
                    nav_state.center_to_center = false;

                    // se la striscia è finita prima dell'ultimo punto ferma il robot, l'obiettivo è ancora attivo
                    if (strip_end_reached) send_objective(OBJ_STOP, 0, 0, 0, 0, 0);

                    // torna all'esplorazione
                    set_state(nav_state.mission);

                    break;

                } else {    // Se il percorso da seguire non è ancora finito
                    
                    // imposta il prossimo punto di svolta come obiettivo
                    nav_state.path_idx = next_turn_pt;
                    
                    // Calcola le coordinate del centro della corsia
                    double next_x, next_y;
                    get_path_coord(nav_state.path_idx, &next_x, &next_y);

                    // imposta come prossimo obiettivo di andare al prossimo punto di svolta
                    send_objective(OBJ_MOVE, MAX_ROTATION_SPEED, 0, calc_speed(cur_alarm.max_v_forw), next_x, next_y);

                    // se la svolta dopo questo punto è piccola il robot lo attraversa senza fermarsi
                    if (!strip_end_reached && can_pass_through(nav_state.path_idx, cur_pos)) {
                        nav_state.cur_obj.pass_through = true;
                        write_objective(&nav_state.cur_obj);
                    }
                }
            } else { // se l'obiettivo non è stato raggiunto, rimani in questo stato
                if (is_stalled(cur_alarm, cur_pos)) {       // controlla se il robot è in stallo
                    if (nav_state.center_to_center) {   // se non possiamo tagliare il percorso (quindi siamo già entrati in stallo prima)
                        
                        // chiudi la cella verso cui sta andando perchè è troppo stretta
                        close_cell(nav_state.path.cells[nav_state.path_idx]);


                    } 

                    resume_after_stop(cur_alarm, cur_pos);

                    

                }

                break;
            }    

            
            break;
        }
        case NAV_STATE_CROSS: {

            // se è un ostacolo vero o è passato troppo tempo, l'attraversamento è fallito
            if ((cur_alarm.error_mask & SAFETY_ERROR_CONTACT) || elapsed_time(nav_state.state_start) >= 2 * (CROSS_PUSH_TIME + UNLOCK_PHASE_TIMEOUT)) {
                end_cross_failed(cur_alarm, cur_pos);
                break;
            }

            // il tipo dell'obiettivo corrente dice a che punto siamo
            if (nav_state.cur_obj.obj_type == OBJ_ROTATE) {

                // finita la rotazione spingi dritto
                if (is_objective_achieved(cur_pos) || elapsed_time(nav_state.phase_start) >= UNLOCK_PHASE_TIMEOUT) {
                    send_objective(OBJ_CONST_SPEED, 0, 0, MIN(CRUISING_SPEED, cur_alarm.max_v_forw), 0, 0);
                    nav_state.phase_start = rt_adesso();
                }

                break;
            }

            // aggiorna la velocità della spinta
            nav_state.cur_obj.v = MIN(CRUISING_SPEED, cur_alarm.max_v_forw);
            write_objective(&nav_state.cur_obj);

            // l'allarme di incastro guarda l'ultimo secondo e mezzo: prima di metà spinta non dice ancora niente.
            // se dopo è spento il robot si è liberato, e torna a quello che stava facendo
            if (elapsed_time(nav_state.phase_start) >= CROSS_PUSH_TIME / 2 && !(cur_alarm.error_mask & SAFETY_ERROR_STUCK)) {

                if (read_started_on_base() && !nav_state.has_left_base) set_state(NAV_STATE_LEAVE_BASE);
                else resume_after_stop(cur_alarm, cur_pos);

                break;
            }

            // finita la spinta ancora incastrato: prova dall'altra parte, se è già l'altra parte ha fallito
            if (elapsed_time(nav_state.phase_start) >= CROSS_PUSH_TIME) {

                if (wrap_angle(cur_pos.theta - nav_state.cross_theta) >= 0) start_cross_rotation(-CROSS_ANGLE);
                else end_cross_failed(cur_alarm, cur_pos);
            }

            break;
        }
        case NAV_STATE_LEAVE_BASE: {

            // se il robot è arrivato al punto davanti alla base. la posizione è quella data dal lidar:
            // se le ruote slittano l'odometria direbbe che il robot è uscito anche quando è fermo
            if (read_mapping_correction().map_x >= AGG_PUNTO_X_M) {

                send_objective(OBJ_STOP, 0, 0, 0, 0, 0);

                // da adesso il permesso di muoversi con i contatti chiusi non serve più, quindi lo revochiamo
                drv_esci_dalla_base(0);

                // segna che il robot è uscito dalla base
                nav_state.has_left_base = true;

                // svuota la scia, così uno sblocco subito dopo non arretra verso la base
                nav_state.trail.len = 0;

                // la missione non è cambiata
                set_state(nav_state.mission);

            } else if (elapsed_time(nav_state.state_start) >= LEAVE_BASE_TIMEOUT) {     // se è passato troppo tempo da quando ha provato a uscire dalla base

                // revoca il permesso di muoversi al robot
                drv_esci_dalla_base(0);

                stop_and_fail(DRV_FAULT_LEAVE_BASE_TIMEOUT);

            } else {

                // il movimento è sempre dritto, il robot non deve mai ruotare con i contatti chiusi
                // questo è importante per evitare che il robot sposti la base per sbaglio
                // se qualcosa lo blocca il driver tiene ferme le ruote, e riparte quando l'ostacolo sparisce
                send_objective(OBJ_CONST_SPEED, 0, 0, MIN(LEAVE_BASE_SPEED, cur_alarm.max_v_forw), 0, 0);
            }

            break;
        }
        case NAV_STATE_HOOK_UP: {

            // se il driver sta già facendo la manovra, aspetta l'esito
            if (nav_state.hook_has_started) {

                // agganciato: riposo sulla base, e la mappa si salva
                if (cur_drv_state.aggancio == AGG_FATTO) {
                    set_state(NAV_STATE_SLEEP);
                    request_map_save_if_allowed();
                }
                else if (cur_drv_state.aggancio == AGG_FALLITO) stop_and_fail(DRV_FAULT_DOCKING_FAILED);

                break;
            }

            // rete di sicurezza: qui gli sblocchi sono disattivati, il robot potrebbe restare bloccato
            if (elapsed_time(nav_state.state_start) >= HOOK_UP_TIMEOUT) {
                stop_and_fail(DRV_FAULT_DOCKING_TIMEOUT);
                break;
            }

            // il tipo dell'obiettivo corrente dice a che punto siamo
            if (nav_state.cur_obj.obj_type == OBJ_MOVE) {

                // aggiorna la velocità come la più piccola tra velocità di crociera e cur_alarm.max_v_forw
                nav_state.cur_obj.v = MIN(cur_alarm.max_v_forw, CRUISING_SPEED);

                // pubblica la velocità aggiornata
                write_objective(&nav_state.cur_obj);

                // arrivato sul punto, da qui in poi comanda il driver, che trova la base con gli infrarossi e si gira da solo
                if (is_objective_achieved(cur_pos)) {

                    send_objective(OBJ_STOP, 0, 0, 0, 0, 0);

                    // al contatto le ruote si devono fermare
                    drv_esci_dalla_base(0);

                    drv_aggancia(1);

                    nav_state.hook_has_started = true;
                }
            }

            break;
        }
        case NAV_STATE_RETURN_HOME: {

            // se il robot non è partito dalla base non sa dove si trovi, quindi si ferma dove si trova
            if (!read_started_on_base()) {
                send_objective(OBJ_STOP, 0, 0, 0, 0, 0);
                set_state(NAV_STATE_SLEEP);
                break;
            }

            // calcola la cella del robot
            uint16_t robot_cell = coord_to_nav_index(cur_pos.x, cur_pos.y);

            // se la cella del robot non è libera, vai alla cella libera più vicina ed esci
            if (!is_nav_cell_free(robot_cell)) {
                go_to_nearest_free_cell(cur_alarm, cur_pos, robot_cell);
                break;
            }

            // se il robot è già nella cella davanti alla base, inizia l'ultimo tratto
            if (robot_cell == CHARGING_STATION_CELL) {
                start_hook_up(cur_alarm);
                break;
            }

            // cerca il percorso, alla fine si torna qui
            if (search_cell(robot_cell, GOAL_CELL, CHARGING_STATION_CELL) == 0) {

                // la cella davanti alla base non si raggiunge: se il
                // robot è già vicino al punto di aggancio l'aggancio a infrarossi lo fa il driver da qui, altrimenti ci si
                // avvicina il più possibile e si torna qui
                // (mezza cella in più: il robot arrivato al centro della cella trovata con GOAL_NEAR_HOME deve rientrarci)
                if (hypot(cur_pos.x - AGG_PUNTO_X_M, cur_pos.y) <= HOME_IR_DOCK_DIST + NAV_MAP_CELL_SIZE / 2) {
                    send_objective(OBJ_STOP, 0, 0, 0, 0, 0);
                    drv_aggancia(1);
                    nav_state.hook_has_started = true;
                    set_state(NAV_STATE_HOOK_UP);
                    break;
                }
                if (search_cell(robot_cell, GOAL_NEAR_HOME, 0) != 0) {
                    start_following_path(cur_alarm, cur_pos, 0, false);
                    break;
                }

                // non c'è una strada per tornare alla base: può essere un ostacolo che si sposta (una persona davanti al
                // robot), quindi il robot resta fermo e riprova a ogni job. Resta in questo stato, quindi state_start dice
                // da quanto tempo non trova la strada: solo dopo NO_PATH_HOME_TIMEOUT va in guasto
                if (elapsed_time(nav_state.state_start) >= NO_PATH_HOME_TIMEOUT) stop_and_fail(DRV_FAULT_NO_PATH_HOME);
                else if (nav_state.cur_obj.obj_type != OBJ_STOP) send_objective(OBJ_STOP, 0, 0, 0, 0, 0);
            } else {
                start_following_path(cur_alarm, cur_pos, 0, false);
            }

            break;
        }
        case NAV_STATE_SLEEP: {

            // se l'obiettivo non è di stare fermo, manda l'obiettivo di fermarsi
            if (nav_state.cur_obj.obj_type != OBJ_STOP) send_objective(OBJ_STOP, 0, 0, 0, 0, 0);

            // leggi lo stato attuale del driver
            drv_stato_t cur_state = {0};

            drv_stato(&cur_state);

            // controlla se quando è andato a caricarsi era stata interrotta una missione
            // e adesso si trova sulla base di ricarica
            // e la batteria è a percentuale maggiore della soglia per ripartire
            if (nav_state.interrupted_mission != 0 && cur_state.sulla_base && cur_state.percentuale >= BATTERY_RESUME_PERCENT) {
                
                // imposta la missione alla missione precedente all'interruzione dovuta alla batteria
                nav_state.mission = nav_state.interrupted_mission;

                // azzera interrupted_mission
                nav_state.interrupted_mission = 0;

                nav_state.has_left_base = false;

                start_leaving_base();

            }

            break;
        }
        case NAV_STATE_UNLOCK: {
            
            // Gestione delle fasi di sblocco
            switch(nav_state.unlock_phase) {

                // fase di indietreggiamento
                case UNLOCK_PHASE_BACK:

                    // aggiorna la velocità
                    nav_state.cur_obj.v = -MIN(UNLOCK_TABLE[nav_state.unlock_attempt - 1].back_speed, cur_alarm.max_v_backw);

                    // pubblica la velocità aggiornata
                    write_objective(&nav_state.cur_obj);
                    
                    // Controlla se l'obiettivo è stato raggiunto
                    if (is_objective_achieved(cur_pos)) {
                        // se sì, passa al prossimo punto
                        unlock_next_back_point(cur_alarm, cur_pos);
                    } else if (elapsed_time(nav_state.phase_start) >= UNLOCK_PHASE_TIMEOUT) {   // altrimenti se il tentativo è durato più di UNLOCK_PHASE_TIMEOUT
                        // prova a far ruotare il robot
                        unlock_start_rotation(cur_alarm, cur_pos);
                    }
                    
                    break;

                // fase di rotazione 
                case UNLOCK_PHASE_ROTATE:
                    // Controlla se l'obiettivo è stato raggiunto
                    if (is_objective_achieved(cur_pos)) {
                        // se sì, riparti
                       resume_after_stop(cur_alarm, cur_pos);
                    } else if (elapsed_time(nav_state.phase_start) >= UNLOCK_PHASE_TIMEOUT) {   // altrimenti se il tentativo è durato più di UNLOCK_PHASE_TIMEOUT
                        // il tentativo è fallito, riprova 
                        start_unlock_attempt(cur_alarm, cur_pos);
                    }

                    break;
                
                // se entriamo qui andiamo in guasto
                default:
                    
                    stop_and_fail(DRV_FAULT_INCONSISTENT_STATE);
                    
                    break;

            }

            break;
        }
        case NAV_STATE_ALIGN: {
            
            // aggiorna la velocità come la più piccola tra velocità di crociera e cur_alarm.max_v_forw
            nav_state.cur_obj.v = MIN(cur_alarm.max_v_forw, CRUISING_SPEED);

            // pubblica la velocità aggiornata
            write_objective(&nav_state.cur_obj);

            // controlla se l'obiettivo corrente è stato raggiunto
            if (is_objective_achieved(cur_pos)) {

                // dì al robot di non tagliare i percorsi, per evitare altri problemi di allineamento
                nav_state.center_to_center = true;

                // rimetti l'obiettivo originale prima dell'allineamento
                set_state(nav_state.mission);

            } else if (is_stalled(cur_alarm, cur_pos) || elapsed_time(nav_state.state_start) >= ALIGN_TIMEOUT) {  // se l'obiettivo non è stato ancora raggiunto e il robot è in stallo, oppure se si supera l'ALIGN_TIMEOUT

                // imposta lo stato attuale di sblocco
                set_state(NAV_STATE_UNLOCK);
                
                // inizia la routine di sblocco
                start_unlock_attempt(cur_alarm, cur_pos);

            }

            
            break;
        }

        case NAV_STATE_FAILURE: {
            /* code */
            break;
        }
        case NAV_STATE_PAUSE: {
            // fermo finché non arriva un comando (apply_commands)
            if (nav_state.cur_obj.obj_type != OBJ_STOP) send_objective(OBJ_STOP, 0, 0, 0, 0, 0);
            break;
        }
        default:   {         // se entra qui dentro siamo in uno stato inconsistente
            
            stop_and_fail(DRV_FAULT_INCONSISTENT_STATE);

            break;
        }
    }

}  
