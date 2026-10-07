#ifndef TASK_MOVEMENT_H
#define TASK_MOVEMENT_H

#include <stdbool.h>
#include <stdint.h>
#include "driver/driver.h"




// struttura dati che verrà mandata al task di navigazione
typedef struct {

    u_int32_t num;              // numero del frame letto
    int64_t scan_end_t;         // timestamp di fine scansione
    int64_t 

} movement_position_data_t;


// caratteristiche del task movement, dichiarata in movement.c
extern rt_task_t movement_task;

void init_movement(void);
movement_position_data_t read_movement(void);
void job_movement(void *arg);


#endif