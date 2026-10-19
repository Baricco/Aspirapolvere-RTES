#include "driver/driver.h"
#include "driver/scambio.h"
#include <math.h>

#include "navigation.h"
#include "mapping.h"
#include "movement.h"
#include "safety.h"

#define CELL_NOT_VISITED 0xFFFF

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

// funzioni helper per la coda
static inline void search_queue_push(nav_search_queue_t *queue, uint16_t next_elem) {
    queue->values[queue->tail++] = next_elem;
}

static inline bool is_queue_empty(nav_search_queue_t *queue) {
    return queue->head >= queue->tail;
}

static inline uint16_t search_queue_pop(nav_search_queue_t *queue) {
    if (is_queue_empty(queue)) return CELL_NOT_VISITED; 
    return queue->values[queue->head++];
}

static inline void search_queue_flush(nav_search_queue_t *queue) {
    queue->head = 0;
    queue->tail = 0;
}

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

} navigation_state_t;

// stato interno della navigazione
static navigation_state_t nav_state;

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