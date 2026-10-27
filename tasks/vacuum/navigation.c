#include "driver/driver.h"
#include "driver/scambio.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>

#include "navigation.h"
#include "mapping.h"
#include "movement.h"
#include "safety.h"

#define DEG_TO_RAD(a) ((a) * M_PI / 180)

#define CELL_NOT_VISITED 0xFFFF
#define ESCAPE_MAX_CELLS 3
#define WAIT_TIME RT_MS(3000)
#define CRUISING_SPEED 0.35                  // velocità di crociera del robot, m/s
#define MAX_ROTATION_SPEED  1.2             // massima velocità di rotazione, per stare sotto al soglia oltre il quale la mappatura scarta i giri (1,5) con un certo margine, rad/s
#define UNKNOWN_SPEED_FACTOR 0.5            // fattore da moltiplicare alla velocità quando il robot si muove in territorio sconosciuto


#define STALL_SPEED 0.02                // velocità massima in avanti sotto la quale la sicurezza sta trattenendo il robot, espressa in m/s
#define STALL_OMEGA 0.1                 // velocità angolare sotto la quale il robot non sta ruotando, espressa in rad/s
#define STALL_TIME RT_MS(1000)          // per quanto tempo deve durare lo stallo prima che venga considerato come tale


#define TRAIL_SIZE 8            // lunghezza del buffer delle ultime posizioni percorse, serve per permettere al robot di indietreggiare, nel caso di problemi
#define TRAIL_STEP 0.05         // distanza minima in metri tra le ultime posizioni salvate nel buffer        
#define UNLOCK_MAX_ATTEMPTS 10  // tentativi di sblocco prima di andare in guasto (ne mettiamo tanti, per essere sicuri che il robot tenti di uscire tante volte prima di fermarsi e arrendersi)

#define UNLOCK_PHASE_TIMEOUT RT_MS(5000)    // durata massima di ogni fase

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

} nav_path_t;


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

// Funzione helper per impostare una cella della mappa come troppo stretta (NAV_CELL_TOO_NARROW)
// al task di mappatura, aggiornando anche la copia locale della mappa di navigazione
static void close_cell(uint16_t cell) {
    
    // aggiungi la cella alla lista di celle da mandare al task di mappatura
    nav_state.closed_cells.cells[nav_state.closed_cells.num % CLOSED_CELLS_MAX] = cell;

    // aumenta il numero di celle nella lista
    nav_state.closed_cells.num++;

    // pubblica la nuova lista
    scambio_scrivi(&double_buffer_2, &nav_state.closed_cells);

    // aggiorna anche la copia locale della mappa
    nav_state.map[cell] = NAV_CELL_TOO_NARROW;
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


// definizione dell'enum dei possibili obiettivi della funzione di ricerca
typedef enum {
    GOAL_BORDER,    // l'obiettivo della ricerca è trovare una cella al confine delle celle esplorate
    GOAL_DIRTY,     // l'obiettivo della ricerca è trovare una cella non pulita
    GOAL_CELL,      // l'obiettivo della ricerca è trovare una cella precisa (indice della cella)
    GOAL_FREE,      // l'obiettivo della ricerca è trovare una cella libera, partendo da una bloccata
} nav_search_goal_t;

// Funzione helper per stabilire se la cella fornita corrisponde all'obiettivo della ricerca
// qui la ricerca viene fatta sempre sulle 4 celle adiacenti (su, giù, dx, sx), il percorso
// verrà poi tagliato usando l'algoritmo di bresenham per permettere percorsi in diagonale
static bool cell_is_goal(const nav_search_goal_t goal, const uint16_t cell, const uint16_t goal_cell) {
    
    switch (goal) {
        case GOAL_BORDER:           // se la cella cercata è una cella di frontiera
            if (nav_state.map[cell] == NAV_CELL_FREE) {     // se la cella in cui ci troviamo è libera
                // controllo le 4 celle adiacenti alla cella corrente, se sono sconosciute,
                // significa che cell è una cella di frontiera
                
                for (int i = 0; i < 4; i++) if (nav_state.map[cell + NEIGHBOR_SHIFT[i]] == NAV_CELL_UNKNOWN) return true;

            }
            break;
        case GOAL_DIRTY:
            /* code */
            break;          
        case GOAL_CELL:     // il terzo parametro serve solo qui
            return cell == goal_cell;
            break;
        case GOAL_FREE:
            return nav_state.map[cell] == NAV_CELL_FREE;
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

    // se next_cell è libera, può entrare
    if (nav_state.map[next_cell] == NAV_CELL_FREE) return true;

    // se siamo qui significa che next_cl, ferell è proibita, bloccata o sconosciuta
    // in questo caso si può passare solo se:
    // 1. next_cell non è sconosciuta
    // 2. cur_cell è bloccata o proibita (da una cella libera non puoi passare per una bloccata)
    // 3. se next_cell si trova entro ESCAPE_MAX_CELLS da start_cell
    if (nav_state.map[next_cell] != NAV_CELL_UNKNOWN && nav_state.map[cur_cell] != NAV_CELL_FREE && nav_state.map[next_cell] != NAV_CELL_TOO_NARROW) {
        
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

    return MIN(max_v, CRUISING_SPEED) * (nav_state.map[nav_state.path.cells[nav_state.path_idx]] != NAV_CELL_FREE ? UNKNOWN_SPEED_FACTOR : 1);

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
        nav_state.path.cells[length] = cur_cell;

        // aumenta la lunghezza del percorso
        length++;
    }
    
    // aggiungi start_cell alla fine, il for di prima non lo aggiungeva
    nav_state.path.cells[length] = start_cell;
    length++;

    // rovescia l'array del percorso
    for (int i = 0; i < length / 2; i++) {
        uint16_t tmp;
        tmp = nav_state.path.cells[i];
        nav_state.path.cells[i] = nav_state.path.cells[length - 1 - i];
        nav_state.path.cells[length - 1 - i] = tmp;
    }

    // salva la lunghezza totale del percorso nello stato della navigazione
    nav_state.path.length = length;

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
        if (cell_is_goal(goal, cur_cell, goal_cell)) return reconstruct_path(start_cell, cur_cell);

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

// Funzione helper che usa Bresenham per verificare se due punti cadono sulla stessa
// retta e non ci sono celle non libere nel mezzo
// Se fossi stato un buon programmatore avrei riutilizzato il codice
// per fare Bresenham di mapping.c ma non ho voglia e sono stanco
static bool line_of_sight(uint16_t start_cell, uint16_t end_cell) {
    
    // Ricava x e y base (il punto il punto di partenza) e da 
    // tail (il punto di svolta)
    int base_y = start_cell >> 8;
    int base_x = start_cell & 0xFF;  
    int tail_y = end_cell >> 8;
    int tail_x = end_cell & 0xFF;

    
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
                
                // calcola le due celle in diagonale 
                uint8_t fd_cur_cell = nav_state.map[(cur_cell_y + sy) << 8 | cur_cell_x];
                uint8_t sd_cur_cell = nav_state.map[cur_cell_y << 8 | (cur_cell_x + sx)];

                // se una delle due celle in diagonale non è libera, ritorna false
                if (fd_cur_cell != NAV_CELL_FREE || sd_cur_cell != NAV_CELL_FREE) return false;
            }
            if (hor_error) { err += dy; cur_cell_x += sx; }
            if (vert_error) { err += dx; cur_cell_y += sy; }

            if (nav_state.map[cur_cell_y << 8 | cur_cell_x] != NAV_CELL_FREE ) return false;
        }
    } 

    return false;

}

// Funzione che dato nav_state.path_idx, che indica in quale punto del percorso
// si trova il robot, restituisce l'indice del prossimo punto di svolta per mandare il 
// prossimo obiettivo.
static uint16_t next_turning_point(void) {

    // Inizializza gli indici all'inizio del path e alla fine del path
    uint32_t cur_idx = nav_state.path_idx;
    uint32_t last_idx = nav_state.path.length - 1;

    // Se il percorso ha lunghezza 0, restituisce l'indice di partenza
    if (nav_state.path.length == 0 || cur_idx >= last_idx) return cur_idx;

    // Controlla se la posizione attuale è in una cella non libera o se non bisogna fare tagli al percorso (center_to_center)
    if (nav_state.map[nav_state.path.cells[nav_state.path_idx]] != NAV_CELL_FREE || nav_state.center_to_center) {
        // se il robot si trova in una cella non libera, vai alla prossima cella
        // nel percorso, senza fare tagli
        return cur_idx + 1;
    }

    // Ricerca dicotomica dell'indice della prossima cella del percorso per cui si può tagliare
    // Definizione degli estremi
    int low = cur_idx + 1;      // l'estremo inferiore è la prossima cella, quella è sempre visibile, quindi non serve controllarla
    int high =  last_idx;       // l'estremo superiore, parte dall'ultima cella, è quello che cerchiamo

    // Controllo Rapido del Caso Migliore:
    // se si può tagliare direttamente tra low e high, fallo senza fare ricerca dicotomica
    if (line_of_sight(nav_state.path.cells[cur_idx], nav_state.path.cells[high])) return high;

    while (low < high) {        // ciclo di ricerca

        // calcola il punto medio tra low e high, l'arrotodamento per eccesso (+1)
        // serve a scongiurare il caso del ciclo infinito (high = low + 1)
        int mid = (low + high + 1) / 2;      

        // se dal robot si vede il punto mid, allora il punto di svolta è da mid in poi
        if (line_of_sight(nav_state.path.cells[cur_idx], nav_state.path.cells[mid])) {
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

    nav_state.mission = NAV_STATE_EXPLORE;      // si parte in attesa (stato azzerato), poi l'attesa porta all'esplorazione

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

// funzione helper per fermare il robot e andare in stato di guasto
static inline void stop_and_fail(void) {

    send_objective(OBJ_STOP, 0, 0, 0, 0, 0);

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
        
        // Controlla se il vicino i-esimo è sconosciuto
        if (nav_state.map[cur_neighbor_idx] == NAV_CELL_UNKNOWN) {
            
            // Se sì, aggiungilo al percorso
            nav_state.path.cells[nav_state.path.length++] = cur_neighbor_idx;

            return true;

        } 
    }

    // Se nessuno dei vicini è sconosciuto restituisci false, non dovremmo mai finire qui:
    // se una cella è di frontiera, ha per definizione dei vicini sconosciuti
    return false;

}

// Dopo un arresto o uno sblocco, riporta il robot al centro della sua cella se è 
// libera, altrimenti ricalcola il percorso
static void resume_after_stop(safety_alarm_t cur_alarm, movement_position_data_t cur_pos) {
    // calcola la cella di navigazione in cui si trova il robot
    uint16_t robot_index = coord_to_nav_index(cur_pos.x, cur_pos.y);

    // se la cella in cui si trova il robot non è libera
    if (nav_state.map[robot_index] != NAV_CELL_FREE) {

        // manda l'obiettivo di fermarsi al robot
        send_objective(OBJ_STOP, 0, 0, 0, 0, 0);

        // La mappa è cambiata sotto al robot
        // ricalcola il percorso
        set_state(nav_state.mission);

    } else {        // se il robot si trova in una cella libera
        
        // ricava le coordinate del centro della cella
        double x, y;
        nav_index_to_coord(robot_index, &x, &y);
    
        // manda al robot l'obiettivo di andare verso il centro della cella
        send_objective(OBJ_MOVE, MAX_ROTATION_SPEED, 0, MIN(cur_alarm.max_v_forw, CRUISING_SPEED), x, y);

        // passa allo stato di allineamento
        set_state(NAV_STATE_ALIGN);
    
    }
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

        stop_and_fail();

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

// job del task
void job_navigation(void *arg) {
    
    (void)arg;

    // Se la mappa è cambiata, salva la versione aggiornata e il numero di versione aggiornato
    if (read_nav_map_version_number() != nav_state.map_version) {
    
        nav_state.map_version = read_nav_map(nav_state.map);

        // rimetti nella copia le celle chiuse, che la mappatura potrebbe non aver ancora ricevuto
        for (uint32_t i = 0; i < MIN(nav_state.closed_cells.num, CLOSED_CELLS_MAX); i++) {
            nav_state.map[nav_state.closed_cells.cells[i]] = NAV_CELL_TOO_NARROW;
        }

    }

    // leggi la posizione corrente dal task di movimento
    movement_position_data_t cur_pos = read_movement();

    // se la posizione corrente non è valida
    if (!cur_pos.valid) {
        
        // se l'obiettivo corrente non è già OBJ_STOP, aggiungi l'obiettivo OBJ_STOP
        if (nav_state.cur_obj.obj_type != OBJ_STOP) send_objective(OBJ_STOP, 0, 0, 0, 0, 0);
        
        // metti il robot in attesa
        set_state(NAV_STATE_WAIT);

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


    // leggi gli allarmi mandati dal task safety, serve per la velocità massima degli obiettivi
    safety_alarm_t cur_alarm = read_safety();

    // se ci sono degli allarmi (stuck, slip o motor overload) e lo stato non è 
    // attesa, sblocco, riposo o guasto, vai in sblocco
    // il primo controllo dentro l'if verifica se uno o più dei 3 bit meno significativi siano a 1
    if ((cur_alarm.error_mask & (SAFETY_ERROR_STUCK | SAFETY_ERROR_SLIP | SAFETY_ERROR_MOT_OVRLD | SAFETY_ERROR_CONTACT)) && 
        nav_state.state != NAV_STATE_SLEEP &&
        nav_state.state != NAV_STATE_UNLOCK &&
        nav_state.state != NAV_STATE_WAIT &&
        nav_state.state != NAV_STATE_FAILURE) {
        
        // imposta lo stato corrente a sblocco
        set_state(NAV_STATE_UNLOCK);

        // comincia a fare il primo sblocco
        start_unlock_attempt(cur_alarm, cur_pos);

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
                // passa al prossimo stato
                set_state(nav_state.mission);
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
            if (nav_state.map[robot_index] == NAV_CELL_UNKNOWN) nav_state.map[robot_index] = NAV_CELL_FREE;
            
            // se il robot si trova in una cella non libera
            if (nav_state.map[robot_index] != NAV_CELL_FREE) {

                // cerchiamo la cella libera più vicina
                if (search_cell(robot_index, GOAL_FREE, 0) == 0) {

                    // se la ricerca ritorna 0, non ci sono celle libere vicine
                    // raggiungibili entro ESCAPE_MAX_CELLS, quindi stop e guasto

                    stop_and_fail();

                } else {
                    
                    // passa allo stato segui percorso
                    nav_state.path_idx = 0;

                    set_state(NAV_STATE_FOLLOW_PATH);


                }

                break;

            }
            
            
            
            // cerca i confini della mappa
            uint32_t dist = search_cell(robot_index, GOAL_BORDER, 0);

            if (dist == 0) {        // se dist == 0, l'esplorazione è finita
                
                // passa allo stato di pulizia
                set_state(NAV_STATE_CLEAN); 

            } else { // Se esiste un percorso verso la frontiera

                // aggiungi la cella sconosciuta oltre la frontiera alla zona da visitare
                if (!append_unknown_neighbor()) {
                    
                    // Se entriamo qui dentro significa che una cella di frontiera non 
                    // ha una cella vicina sconosciuta (impossibile)

                    stop_and_fail();

                    break;
                }

                nav_state.path_idx = 0;
                set_state(NAV_STATE_FOLLOW_PATH);

            }

            break;
        }
        case NAV_STATE_CLEAN: {

            
            /* code */
            break;
        }
        case NAV_STATE_FOLLOW_BORDERS: {
            /* code */
            break;
        }
        case NAV_STATE_FOLLOW_PATH: {

            // aggiorna la velocità come la più piccola tra velocità di crociera e cur_alarm.max_v_forw
            nav_state.cur_obj.v = calc_speed(cur_alarm.max_v_forw);

            // pubblica la velocità aggiornata
            write_objective(&nav_state.cur_obj);

            // Controlla se l'obiettivo è stato raggiunto
            if (is_objective_achieved(cur_pos)) {

                // Salva il prossimo punto di svolta
                int next_turn_pt = next_turning_point();

                // Controlla se il percorso è finito
                if (next_turn_pt == nav_state.path_idx) {
                    // Azzera i tentativi di sblocco
                    nav_state.unlock_attempt = 0;
                    // Azzera center_to_center, da adesso si può tagliare di nuovo il percorso
                    nav_state.center_to_center = false;
                    // torna all'esplorazione
                    set_state(nav_state.mission);
                    break;
                } else {    // Se il percorso da seguire non è ancora finito
                    
                    // imposta il prossimo punto di svolta come obiettivo
                    nav_state.path_idx = next_turn_pt;
                    
                    // Calcola le coordinate del centro della cella
                    double next_x, next_y;
                    nav_index_to_coord(nav_state.path.cells[nav_state.path_idx], &next_x, &next_y);

                    // imposta come prossimo obiettivo di andare al prossimo punto di svolta
                    send_objective(OBJ_MOVE, MAX_ROTATION_SPEED, 0, calc_speed(cur_alarm.max_v_forw), next_x, next_y);
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
        case NAV_STATE_HOOK_UP: {
            /* code */
            break;
        }
        case NAV_STATE_RETURN_HOME: {
            /* code */
            break;
        }
        case NAV_STATE_SLEEP: {
            /* code */
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
                    
                    stop_and_fail();
                    
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

            } else if (is_stalled(cur_alarm, cur_pos)) {  // se l'obiettivo non è stato ancora raggiunto e il robot è in stallo

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
        default:   {         // se entra qui dentro siamo in uno stato inconsistente
            
            stop_and_fail();

            break;
        }
    }

}  
