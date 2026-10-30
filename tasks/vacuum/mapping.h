#ifndef TASK_MAPPING_H
#define TASK_MAPPING_H

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "driver/driver.h"

// Piccola spiegazione del funzionamento della mappa di navigazione:
// La mappa di navigazione è una griglia dove ogni cella (20x20 cm) è un byte
// ogni byte codifica due tipi di informazioni per la stessa cella:
// i 4 bit più significativi sono una bit mask che indica lo stato delle 4 celle
// da 10cm di lato ciascuna (1 --> Bloccata, 0 --> non Bloccata). 
// I 4 bit meno significativi indicano lo stato dell'intera cella 20x20
// gli stati possibili son definiti nell'enum qui sotto

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

// Simboli Mnemonici da usare nella bit mask nelle celle di navigazione (4 bit più significativi della cella)
#define NAV_CELL_TOP_LEFT       (1u << 4)       // XXX1 XXXX --> Cella in Alto a Sinistra
#define NAV_CELL_TOP_RIGHT      (1u << 5)       // XX1X XXXX --> Cella in Alto a Destra
#define NAV_CELL_BOTTOM_LEFT    (1u << 6)       // X1XX XXXX --> Cella in Basso a Sinistra
#define NAV_CELL_BOTTOM_RIGHT   (1u << 7)       // 1XXX XXXX --> Cella in Basso a Destra

#define NAV_CELL_STATE_MASK     15              // 0000 1111 --> Maschera per estrarre solamente lo stato da una cella
#define NAV_CELL_QUAD_MASK      240             // 1111 0000 --> Maschera per estrarre solamente i quadranti da una cella 

// Array che serve per scorrere i 4 quadranti di una cella della mappa di navigazione 
static const uint8_t NAV_CELL_QUAD[4] = { NAV_CELL_TOP_LEFT, NAV_CELL_TOP_RIGHT, NAV_CELL_BOTTOM_LEFT, NAV_CELL_BOTTOM_RIGHT };

// distanza di ogni quadrante dal centro di una cella della mappa di navigazione
#define NAV_CELL_QUAD_DIST_FROM_CENTER (NAV_MAP_CELL_SIZE / 4)

// Enum degli stati delle celle di navigazione
typedef enum {
    NAV_CELL_UNKNOWN,                    // Cella Sconosciuta: la cella non è ancora stata esplorata
    NAV_CELL_FREE,                       // Cella Libera: la cella è vuota e percorribile
    NAV_CELL_BLOCKED,                    // Cella Bloccata: la cella è coperta da un ostacolo
    NAV_CELL_FORBIDDEN,                   // Cella Proibita: la cella è in una Zona Proibita, non è percorribile
    NAV_CELL_TOO_NARROW,                 // Cella troppo Stretta: il passaggio è troppo stretto, nonostante questa cella sia libera, il robot non può comunque passare
} mapping_nav_cell_state;


// Funzione helper per calcolare l'indice della cella nella navigation map partendo dalle coordinate espresse in metri rispetto all'origine
static inline uint16_t coord_to_nav_index(double x, double y) { 
    // prende le due coordinate (x,y) e ne calcola l'indice nella navigation map. l'indice è un numero a 16bit composto da 8bit che sono la y + 8bit che sono la x
    return (uint16_t)(floor(y / NAV_MAP_CELL_SIZE) + NAV_MAP_ORIGIN) << 8 | (uint16_t)(floor(x / NAV_MAP_CELL_SIZE) + NAV_MAP_ORIGIN);
}

// Funzione helper per calcolare le coordinate x,y del robot partendo da una cella della navigation map
static inline void nav_index_to_coord(uint16_t index, double *x, double *y) {

    // esegue l'operazione inversa di coord_to_index per ottenere la y e la x dall'indice della cella.
    // siccome in coord_to_index tutti i punti in una cella cadono nello stesso indice, questa funzione
    // aggiunge 0.5 per prendere il punto che si trova nel centro geometrico della cella, l'errore è di 10cm al massimo
    *y = index >> 8;
    *y -= NAV_MAP_ORIGIN - 0.5;
    *y *= NAV_MAP_CELL_SIZE;

    *x = index & 0xFF;
    *x -= NAV_MAP_ORIGIN - 0.5;
    *x *= NAV_MAP_CELL_SIZE;
}

// Funzione helper per calcolare l'indice della cella nella fine map partendo dalle coordinate espresse in metri rispetto all'origine
static inline uint32_t coord_to_fine_index(double x, double y) { 
    // prende le due coordinate (x,y) e ne calcola l'indice nella fine map. l'indice è un numero a 20bit composto da 10bit che sono la y + 10bit che sono la x
    return (uint32_t)(floor(y / FINE_MAP_CELL_SIZE) + FINE_MAP_ORIGIN) << 10 | (uint16_t)(floor(x / FINE_MAP_CELL_SIZE) + FINE_MAP_ORIGIN);
}

// Funzione helper per calcolare le coordinate x,y del robot partendo da una cella della fine map
static inline void fine_index_to_coord(uint32_t index, double *x, double *y) {

    // esegue l'operazione inversa di coord_to_index per ottenere la y e la x dall'indice della cella.
    // siccome in coord_to_index tutti i punti in una cella cadono nello stesso indice, questa funzione
    // aggiunge 0.5 per prendere il punto che si trova nel centro geometrico della cella, l'errore è di 2,5cm al massimo
    *y = index >> 10;
    *y -= FINE_MAP_ORIGIN - 0.5;
    *y *= FINE_MAP_CELL_SIZE;

    *x = index & 0x3FF;
    *x -= FINE_MAP_ORIGIN - 0.5;
    *x *= FINE_MAP_CELL_SIZE;
}

// funzione helper che restituisce solo lo stato di una cella
static inline uint8_t get_nav_cell_state(uint8_t cell) {
    return cell & NAV_CELL_STATE_MASK;
}

// funzione helper che restituisce solo la maschera dei quadranti di una cella
static inline uint8_t get_nav_cell_quads(uint8_t cell) {
    return cell & NAV_CELL_QUAD_MASK;
}

// funzione helper che prende una cella e un nuovo stato da assegnarle e restituisce la 
// stessa cella con lo stato nuovo e la bit mask dei quadranti invariata
static inline uint8_t set_nav_cell_state(uint8_t cell, mapping_nav_cell_state state) {
    return ((cell >> 4) << 4) | state;      // trick simpatico con i bit shift 
}

// funzione helper che prende l'indice di una cella e uno dei 4 quadranti e
// restituisce le coordinate del centro di quel quadrante
static inline void quad_to_coord(uint16_t cell, uint8_t quad, double *x, double *y) {
        
    nav_index_to_coord(cell, x, y);

    switch (quad) {
    case NAV_CELL_TOP_LEFT:
        *x -= NAV_CELL_QUAD_DIST_FROM_CENTER;
        *y += NAV_CELL_QUAD_DIST_FROM_CENTER;
        break;
    case NAV_CELL_TOP_RIGHT:
        *x += NAV_CELL_QUAD_DIST_FROM_CENTER;
        *y += NAV_CELL_QUAD_DIST_FROM_CENTER;
        break;
    case NAV_CELL_BOTTOM_LEFT:
        *x -= NAV_CELL_QUAD_DIST_FROM_CENTER;
        *y -= NAV_CELL_QUAD_DIST_FROM_CENTER;
        break;
    case NAV_CELL_BOTTOM_RIGHT:
        *x += NAV_CELL_QUAD_DIST_FROM_CENTER;
        *y -= NAV_CELL_QUAD_DIST_FROM_CENTER;
        break;
    default:
        break;
    }
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

int init_mapping(void);
mapping_forbidden_zone_list_t read_forbidden_zone_list(void);
// funzione per pubblicare la lista di zone proibite (lo leggerà task navigation e safety)
void write_forbidden_zone_list(const mapping_forbidden_zone_list_t *new_forbidden_zones);
uint32_t read_nav_map_version_number(void);
uint32_t read_nav_map(uint8_t *navigation_map);
void job_mapping(void *arg);


#endif