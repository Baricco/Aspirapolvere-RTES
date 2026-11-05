#ifndef TASK_MOVEMENT_H
#define TASK_MOVEMENT_H

#include <stdbool.h>
#include <stdint.h>
#include "driver/driver.h"

// Tolleranze per dichiarare raggiunto un obiettivo
#define ROTATE_TOLERANCE_RAD    0.0175  // OBJ_ROTATE: 1°;
#define MOVE_TOLERANCE_M        0.02    // OBJ_MOVE: 2 cm;
#define STOP_V_TOLERANCE        0.005   // OBJ_STOP: m/s misurati
#define STOP_OMEGA_TOLERANCE    0.02    // OBJ_STOP: rad/s misurati
#define PASS_TOLERANCE_M        0.10    // OBJ_MOVE: un punto da attraversare senza fermarsi conta come raggiunto a 10 cm
#define PASS_HOLD_TIME          RT_MS(300) // quanto il robot continua ad andare dopo un punto da attraversare, aspettando il prossimo obiettivo
#define MAX_ACCEL_V             0.5     // rampa: quanto può crescere la velocità, m/s^2
#define MAX_DECEL_V             1.0     // rampa: quanto può scendere la velocità, m/s^2
#define MAX_ACCEL_OMEGA         3.0     // rampa: quanto può cambiare la velocità di rotazione, rad/s^2
#define ALIGNMENT_THRESHOLD     0.50    // soglia di allineamneto in OBJ_MOVE: 0,17 rad, circa 10°
#define ALIGNMENT_THRESHOLD_NEAR 0.17   // come ALIGNMENT_THRESHOLD, ma vicino al punto da raggiungere, dove girando in corsa non ci si arriverebbe
#define ALIGNMENT_NEAR_DIST_M   0.15    // sotto questa distanza dal punto vale ALIGNMENT_THRESHOLD_NEAR

// Taratura dell'IMU: (l'IMU conta circa il 3% in meno), questa costante compensa l'errore
#define YAW_SCALE               1.0285

// Guadagno da usare quando si corregge l'errore
#define ROTATION_GAIN           2.5
#define MOVE_GAIN               3.0


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

    double odom_x, odom_y, odom_theta;  // posizione grezza del robot, serve alla mappatura per correggere x, y e theta

} movement_position_data_t;


// Numero di pose nello storico dell'odometria: a un frame ogni 20 ms sono circa 640 ms, più del ritardo
// massimo di una scansione del lidar (la sua durata, circa 170 ms, più il periodo della mappatura, 200 ms)
#define MOVEMENT_ODOM_HISTORY_SIZE 32

// Una posa dell'odometria grezza e l'istante in cui è stata calcolata
typedef struct {
    int64_t t_ns;
    double x, y, theta;
} movement_odom_pose_t;

// Storico delle ultime pose dell'odometria grezza: buffer circolare, serve alla mappatura
typedef struct {
    movement_odom_pose_t pose[MOVEMENT_ODOM_HISTORY_SIZE];
    uint32_t next_index;        // dove verrà scritta la prossima posa, quella precedente è la più recente
    uint32_t count;             // numero di pose valide
} movement_odom_history_t;

// caratteristiche del task movement, definita in movement.c
extern rt_task_t movement_task;

void init_movement(void);
movement_position_data_t read_movement(void);
void read_odom_history(movement_odom_history_t *history);
void job_movement(void *arg);


#endif