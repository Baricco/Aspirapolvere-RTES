#ifndef TASK_SAFETY_H
#define TASK_SAFETY_H

#include <stdbool.h>
#include <stdint.h>
#include "driver/driver.h"

// Macro Helper per calcolare il minimo tra due numeri
#define MIN(a, b) ((a) < (b) ? (a) : (b))

// Macro Helper per calcolare il massimo tra due numeri
#define MAX(a, b) ((a) > (b) ? (a) : (b))

// Simboli Mnemonici da usare nella error mask
#define SAFETY_ERROR_STUCK      (1u << 0)       // XXXXXXX1 --> Robot Incastrato
#define SAFETY_ERROR_SLIP       (1u << 1)       // XXXXXX1X --> Slittamento Ruote
#define SAFETY_ERROR_MOT_OVRLD  (1u << 2)       // XXXXX1XX --> Motore Sotto Sforzo
#define SAFETY_ERROR_OBSTACLE   (1u << 3)       // XXXX1XXX --> Ostacolo Vicino
#define SAFETY_ERROR_FORB_ZONE  (1u << 4)       // XXX1XXXX --> Zona Proibita Vicina


#define MAX_ABSOLUTE_SPEED      0.5         // velocità massima che il robot può raggiungere espressa in m/s
#define THRESHOLD_STUCK_MIN_V   0.02        // velocità minima che dev'essere comandata al robot perchè controlli se è incastrato, espressa in m/s
#define THRESHOLD_STUCK_MIN_W   0.1         // velocità angolare minima che dev'essere comandata al robot perchè controlli se è incastrato, espressa in rad/s
#define THRESHOLD_STUCK_RATIO_V 0.2         // frazione della velocità comandata sotto al quale la velocità effettiva è considerata 0
#define THRESHOLD_STUCK_RATIO_W 0.2         // frazione della velocità angolare comandata sotto al quale la velocità angolare effettiva è considerata 0
#define THRESHOLD_SLIP_W        0.3         // differenza massima tra velocità angolare sulle ruote e velocità angolare misurata dall'IMU sopra la quale il robot sta scivolando, espressa in rad/s
#define THRESHOLD_MAX_CURRENT   1000        // massima corrente a cui possono essere sottoposti i motori prima che scatti l'allarme, espressa in mA
#define THRESHOLD_STUCK_FRAME_COUNTER 38    // numero di frame consecutivi in cui il robot deve risultare bloccato, prima che si attivi l'allarme reale. corrispondono a circa 0,75s
#define THRESHOLD_SLIP_FRAME_COUNTER  25    // numero di frame consecutivi in cui il robot deve risultare in slittamento, prima che si attivi l'allarme reale. corrispondono a circa 0,50s
#define THRESHOLD_OVRLD_FRAME_COUNTER 25    // numero di frame consecutivi in cui il robot deve risultare in sovraccarico, prima che si attivi l'allarme reale. corrispondono a circa 0,50s

#define THRESHOLD_DISTANCE  0.25              // soglia di distanza per considerare vicino un ostacolo/zona proibita, espresso in metri
#define STOP_DISTANCE       0.01              // distanza dal bordo del robot alla quale la sicurezza porta la velocità in avanti a zero, espressa in metri
#define MOVEMENT_SAFE_GAIN  1.0               // guadagno usato nella formula per calcolare la velocità massima del robot in base agli ostacoli/zone proibite rilevate, espresso in 1/secondi
#define LIDAR_SCAN_MAX_AGE  RT_MS(500)        // età massima che può avere una scan del lidar prima di essere considerata invalida, espressa in nanosecondi

typedef struct {    

    uint32_t frame_num;                 // numero del frame letto
    int64_t frame_acq_t;                // istante di arrivo del frame dai sensori
    uint8_t  error_mask;                 // maschera di 8 bit (5 errori possibili), ogni bit corrisponde ad un tipo di errore
    double max_v_forw, max_v_backw;     // velocità massima in avanti e all'indietro

} safety_alarm_t;

// caratteristiche del task safety, definita in safety.c
extern rt_task_t safety_task;

void init_safety(void);
safety_alarm_t read_safety(void);
void job_safety(void *arg);


#endif