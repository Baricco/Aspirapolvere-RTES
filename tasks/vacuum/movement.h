#ifndef TASK_MOVEMENT_H
#define TASK_MOVEMENT_H

#include <stdbool.h>
#include <stdint.h>
#include "driver/driver.h"




// struttura dati che verrà mandata al task di navigazione
typedef struct {

    uint32_t num;               // numero del frame letto
    int64_t frame_acq_t;        // istante di arrivo del frame dai sensori
    double x, y;                // posizione x e y del robot rispetto alla posizione iniziale
    double theta;               // rotazione del robot rispetto alla rotazione iniziale, normalizzato tra -M_PI e +M_PI
    double v;                   // velocità misurata del robot 
    double omega;               // velocità angolare misurata del robot
    uint32_t num_obj;           // numero dell'obiettivo attualmente preso in carico (corrisponde al num della struttura dati che gli passa il task navigazione)
    bool obj_achieved;          // indica se l'obiettivo attualmente preso in carico sia stato raggiunto o meno
    bool valid;                 // se il robot viene sollevato o spostato a mano la sua posizione calcolata in movement risulta invalida


} movement_position_data_t;


// caratteristiche del task movement, definita in movement.c
extern rt_task_t movement_task;

void init_movement(void);
movement_position_data_t read_movement(void);
void job_movement(void *arg);


#endif