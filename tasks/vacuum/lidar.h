#ifndef TASK_LIDAR_H
#define TASK_LIDAR_H

#include <stdbool.h>
#include <stdint.h>
#include "driver/driver.h"


#define DIRECTION_NUMBER 72         // direzioni in cui si calcola la strada libera, una ogni 360/DIRECTION_NUMBER gradi (5°)
#define DIRECTION_FRONT 0           // indice della direzione davanti al robot
#define DIRECTION_BACK (DIRECTION_NUMBER / 2)   // indice della direzione dietro al robot
#define RELIABILITY_FACTOR 0.25     // percentuale minima di punti validi affinchè la scansione sia valida
#define ROBOT_RADIUS 174.5          // raggio del robot in mm

#define NO_READ UINT16_MAX          // costante per segnalare che non è stata fatta nessuna lettura

// Costanti Specifiche per la pulizia dei bordi
#define BORDER_DISTANCE 0.03        // distanza che il robot deve mantenere dal bordo, espressa in metri
#define BORDER_LOOKAHEAD 0.10       // lunghezza del tratto davanti al robot su cui si controlla che una direzione sia libera, espressa in metri
#define BORDER_SCOPE 0.65           // oltre questa portata dal centro del robot, l'ostacolo non è considerato vicino
#define BORDER_MARGIN 0.003         // tolleranza sul rumore del lidar per i punti già più vicini di BORDER_DISTANCE, espressa in metri
#define SEARCH_STEP 0.035           // di quanto ruotare il robot ad ogni tentativo di ricerca del bordo, espresso in radianti (2°)
#define BORDER_MAX_ANGLE 0.52       // arco entro al quale si trova l'ostacolo che il robot sta seguendo al momento, espresso in radianti
#define MAX_STEP_NUMBER ((int)(2 * M_PI / SEARCH_STEP))         // numero di step necessari per fare un giro completo 



// struttura dati che verrà mandata al task di navigazione
typedef struct {

    uint32_t num;                   // numero della scansione
    int64_t scan_end_t;             // timestamp di fine scansione
    bool valid;                     // indica se la scansione ha abbastanza punti letti correttamente

    // la strada che il robot può fare dritto in ogni direzione prima di toccare un ostacolo, espressa in mm:
    // la direzione DIRECTION_FRONT è davanti, le altre in senso antiorario ogni 360/DIRECTION_NUMBER gradi,
    // DIRECTION_BACK è dietro
    uint16_t direction_free_distance[DIRECTION_NUMBER];

    // Campi Specifici per la pulizia dei bordi
    bool border_found;                          // indica se c'è un ostacolo da seguire entro BORDER_SCOPE dal centro, davanti o a destra del robot
    double border_heading;                      // direzione che il robot deve prendere per seguire il bordo, espresso in radianti
    double border_distance;                     // distanza dell'ostacolo dal bordo del robot, espressa in metri
    bool nearest_border;                        // indica se c'è un ostacolo entro BORDER_SCOPE dal centro, in qualsiasi direzione dal robot
    double nearest_angle;                       // angolo a cui si trova l'ostacolo rispetto al robot, espresso in radianti,

} lidar_scan_data_t;


// caratteristiche del task lidar, dichiarata in lidar.c
extern rt_task_t lidar_task;

void init_lidar(void);
lidar_scan_data_t read_lidar(void);
void job_lidar(void *arg);


#endif