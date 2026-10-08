#ifndef TASK_LIDAR_H
#define TASK_LIDAR_H

#include <stdbool.h>
#include <stdint.h>
#include "driver/driver.h"


#define SECTOR_NUMBER 12            // numero di settori in cui si divide la scansione del robot
#define RELIABILITY_FACTOR 0.25     // percentuale minima di punti validi affinchè la scansione sia valida
#define ROBOT_RADIUS 174.5          // raggio del robot in mm

#define NO_READ UINT16_MAX


// struttura dati che verrà mandata al task di navigazione
typedef struct {

    uint32_t num;                   // numero della scansione
    int64_t scan_end_t;             // timestamp di fine scansione
    bool valid;                     // indica se la scansione ha abbastanza punti letti correttamente
    
    // si prende la circonferenza tracciata dal lidar e la si scompone in 12 distanze
    uint16_t sector_distance[SECTOR_NUMBER];    // distanze in mm divisa per settori
    


} lidar_scan_data_t;


// caratteristiche del task lidar, dichiarata in lidar.c
extern rt_task_t lidar_task;

void init_lidar(void);
lidar_scan_data_t read_lidar(void);
void job_lidar(void *arg);


#endif