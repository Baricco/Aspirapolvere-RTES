#include "driver/driver.h"
#include "driver/scambio.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>

#include "navigation.h"
#include "mapping.h"
#include "movement.h"
#include "safety.h"

#define CELL_NOT_VISITED 0xFFFF
#define ESCAPE_MAX_CELLS 3

// canale di scambio del doppio buffer
static scambio_t double_buffer;
// i due buffer
static navigation_objective_t buffer1, buffer2;

// caratteristiche del task
rt_task_t navigation_task;

// Enum degli stati del task di navigazione
// Il task di navigazione è una macchina a stati 
typedef enum {
    NAV_STATE_WAIT,             // Stato di Attesa: il task di navigazione aspetta
} nav_state_t;


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


// definizione dello stato interno del task di navigazione
typedef struct {
    uint8_t map[NAV_MAP_CELL_NUMBER * NAV_MAP_CELL_NUMBER];            // copia della mappa di navigazione, passata dal task di mappatura
    uint16_t came_from[NAV_MAP_CELL_NUMBER * NAV_MAP_CELL_NUMBER];     // array delle provenienze: ogni cella contiene l'indice della cella da cui il robot è venuto
    uint32_t map_version;                       // numero di versione della mappa di navigazione
    navigation_objective_t last_obj;            // ultimo obiettivo mandato
    nav_state_t state;                          // stato corrente del task di navigazione (macchina a stati)    
    nav_search_queue_t s_q;                     // search_queue, coda di ricerca, serve per la ricerca del percorso
    nav_path_t path;                            // percorso che deve fare il robot
    uint16_t path_idx;                          // indice del path dell'ultimo punto a cui è stato mandato mandato il robot
} navigation_state_t;

// stato interno della navigazione
static navigation_state_t nav_state;

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
                if (nav_state.map[cell + 1] == NAV_CELL_UNKNOWN) return true;   // dx
                if (nav_state.map[cell - 1] == NAV_CELL_UNKNOWN) return true;   // sx
                if (nav_state.map[cell + NAV_MAP_CELL_NUMBER] == NAV_CELL_UNKNOWN) return true; // su
                if (nav_state.map[cell - NAV_MAP_CELL_NUMBER] == NAV_CELL_UNKNOWN) return true; // giù
            }
            break;
        case GOAL_DIRTY:
            /* code */
            break;          
        case GOAL_CELL:     // il terzo parametro serve solo qui
            return cell == goal_cell;
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

    // se siamo qui significa che next_cell è proibita, bloccata o sconosciuta
    // in questo caso si può passare solo se:
    // 1. next_cell non è sconosciuta
    // 2. cur_cell è bloccata o proibita (da una cella libera non puoi passare per una bloccata)
    // 3. se next_cell si trova entro ESCAPE_MAX_CELLS da start_cell
    if (nav_state.map[next_cell] != NAV_CELL_UNKNOWN &&  nav_state.map[cur_cell] != NAV_CELL_FREE) {
        
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

    // array costante con i valori da sommare all'indice per ottenere i vicini
    const int index_shift[4] = {1, -1, NAV_MAP_CELL_NUMBER, -NAV_MAP_CELL_NUMBER};

    // Ciclo di ricerca
    while (!is_search_queue_empty()) {
        
        // preleva il prossimo elemento da esplorare
        uint16_t cur_cell = search_queue_pop();

        // se corrisponde al goal, ricorstruisci il percorso completo e restituiscine la lunghezza
        if (cell_is_goal(goal, cur_cell, goal_cell)) return reconstruct_path(start_cell, cur_cell);

        // se siamo qui significa che cur_cell non è il goal, quindi controlliamo le celle vicine
        for (int i = 0; i < 4; i++) {

            // calcola l'indice della cella vicina alla cella corrente
            uint16_t neigh_cell = cur_cell + index_shift[i];
            
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

    // Controlla se la posizione attuale è in una cella non libera
    if (nav_state.map[nav_state.path.cells[nav_state.path_idx]] != NAV_CELL_FREE) {
        // se il robot si trova in una cella non libera, vai alla prossima cella
        // senza fare tagli
    } 

}


// funzione di inizializzazione del task
void init_navigation(void) {

    // inizializza il doppio buffer passando i due buffer da usare
    // e la loro dimensione 
    scambio_init(
        &double_buffer,
        &buffer1,
        &buffer2, 
        sizeof(navigation_objective_t)
    );

}

// funzione di lettura dei dati del task
navigation_objective_t read_navigation(void) {

    // Crea e Inizializza a 0 la struttura dati navigation_data
    navigation_objective_t navigation_data = {0};

    // Copia il buffer pubblicato in navigation_data
    scambio_leggi(&double_buffer, &navigation_data);

    // ritorna navigation_data
    return navigation_data;
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
        .num = nav_state.last_obj.num + 1,
        .obj_type = obj_type,
        .omega = omega,
        .theta = theta,
        .v = v,
        .x = x,
        .y = y,
    };

    nav_state.last_obj = next_obj;

    write_objective(&next_obj);

}

// job del task
void job_navigation(void *arg) {
    
    (void)arg;

    // Se la mappa è cambiata, salva la versione aggiornata e il numero di versione aggiornato
    if (read_nav_map_version_number() != nav_state.map_version) nav_state.map_version = read_nav_map(nav_state.map);

    // leggi la posizione corrente dal task di movimento
    movement_position_data_t cur_pos = read_movement();

    // se la posizione corrente non è valida
    if (!cur_pos.valid) {
        // se l'obiettivo corrente non è già OBJ_STOP, aggiungi l'obiettivo OBJ_STOP
        if (nav_state.last_obj.obj_type != OBJ_STOP) send_objective(OBJ_STOP, 0, 0, 0, 0, 0);
        // esci
        return;
    }

    // leggi gli allarmi mandati dal task safety, serve per la velocità massima degli obiettivi
    safety_alarm_t cur_alarm = read_safety();



}  