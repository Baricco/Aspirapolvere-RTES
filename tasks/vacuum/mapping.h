#ifndef TASK_MAPPING_H
#define TASK_MAPPING_H

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "driver/driver.h"

// Numero massimo di zone proibite che si possono definire in una mappa
#define MAX_FORBIDDEN_ZONES 16

// Costanti legate alla mappa di navigazione (grossolana) e quella fine (di competenza di mapping e basta)
// ! FINE_MAP_CELL_NUMBER deve essere divisibile per FINE_CELL_IN_NAV_CELL
// ! NAV_MAP_CELL_NUMBER deve essere <= 256, per usare gli uint8_t come coordinate di navigazione
#define FINE_MAP_CELL_NUMBER    1024                                        // Numero di celle per lato nella Griglia Fine
#define FINE_MAP_CELL_SIZE      0.05                                        // Dimensione, espressa in metri, di una cella della griglia fine
#define FINE_CELL_IN_NAV_CELL   4                                           // Numero di celle fini per lato di una cella della mappa di navigazione, ogni cella di navigazione contiene 4x4 celle fini (16 celle in totale)
#define NAV_MAP_CELL_NUMBER (FINE_MAP_CELL_NUMBER / FINE_CELL_IN_NAV_CELL)  // Numero di celle per lato nella Griglia di Navigazione (256x256, 65536 celle totali)
#define NAV_MAP_CELL_SIZE   (FINE_MAP_CELL_SIZE * FINE_CELL_IN_NAV_CELL)    // Dimensione, espressa in metri, di una cella della griglia di navigazione 

#define FINE_MAP_ORIGIN     (FINE_MAP_CELL_NUMBER / 2)             // indice della mappa fine che contiene l'origine (la base di ricarica)
#define NAV_MAP_ORIGIN      (NAV_MAP_CELL_NUMBER / 2)              // indice della mappa di navigazione che contiene l'origine (la base di ricarica)


// Enum degli stati delle celle di navigazione
enum mapping_nav_cell_state{
    NAV_CELL_UNKNOWN,                    // Cella Sconosciuta: la cella non è ancora stata esplorata
    NAV_CELL_FREE,                       // Cella Libera: la cella è vuota e percorribile
    NAV_CELL_BLOCKED,                    // Cella Bloccata: la cella è coperta da un ostacolo
    NAV_CELL_FORBIDDEN                   // Cella Proibita: la cella è in una Zona Proibita, non è percorribile
};

// Funzione helper per calcolare l'indice della cella nella navigation map partendo dalle coordinate espresse in metri rispetto all'origine
static inline uint16_t coord_to_index(double x, double y) { 
    // prende le due coordinate (x,y) e ne calcola l'indice nella navigation map. l'indice è un numero a 16bit composto da 8bit che sono la y + 8bit che sono la x
    return (uint16_t)(floor(y / NAV_MAP_CELL_SIZE) + NAV_MAP_ORIGIN) << 8 | (uint16_t)(floor(x / NAV_MAP_CELL_SIZE) + NAV_MAP_ORIGIN);
}

// Funzione helper per calcolare le coordinate x,y del robot partendo da una cella della navigation map
static inline void index_to_coord(uint16_t index, double *x, double *y) {

    // esegue l'operazione inversa di coord_to_index per ottenere la y e la x dall'indice della cella.
    // siccome in coord_to_index tutti i punti in una cella cadono nello stesso indice, questa funzione
    // aggiunge 0.5 per prendere il punto che si trova nel centro geometrico della cella, l'errore è di 5cm al massimo
    *y = index >> 8;
    *y -= NAV_MAP_ORIGIN + 0.5;
    *y *= NAV_MAP_CELL_SIZE;

    *x = index & 0xFF;
    *x -= NAV_MAP_ORIGIN + 0.5;
    *x *= NAV_MAP_CELL_SIZE;
}

// Struttura dati che identifica una zona proibita (rettangolo)
typedef struct {    

    double x, y;        // coordinate del vertice con x e y minimi
    double w, h;        // dimensioni del rettangolo

} mapping_forbidden_zone_t;

// Struttura dati che contiene la lista di zone proibite nella mappa
typedef struct {

    mapping_forbidden_zone_t forbidden_zones[MAX_FORBIDDEN_ZONES];      // lista di zone proibite
    int8_t num_zones;                                                   // numero di zone presenti nell'array


} mapping_forbidden_zone_list_t;

// caratteristiche del task mapping, definita in mapping.c
extern rt_task_t mapping_task;

void init_mapping(void);
mapping_forbidden_zone_list_t read_forbidden_zone_list(void);
// funzione per pubblicare la lista di zone proibite (lo leggerà task navigation e safety)
void write_forbidden_zone_list(const mapping_forbidden_zone_list_t *new_forbidden_zones);
void job_mapping(void *arg);


#endif