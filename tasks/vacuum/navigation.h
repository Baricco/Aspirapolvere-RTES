#ifndef TASK_NAVIGATION_H
#define TASK_NAVIGATION_H

#include <stdbool.h>
#include <stdint.h>
#include "driver/driver.h"


// in base al tipo di obiettivo, solo alcuni campi della struttura 
// avranno un'effettiva utilità
typedef enum {
    OBJ_STOP,           // nessun campo, questo dev'essere il primo valore dell'enum, così se la struttura è azzerata l'obiettivo è OBJ_STOP
    OBJ_ROTATE,         // solo i campi theta (fino a che angolo ruotare) e omega (velocità massima di rotazione espressa in rad/s)
    OBJ_MOVE,           // omega (velocità massima di rotazione, la rotazione la calcola da x e y); x e y (coordinate dell'obiettivo) e v (velocità massima del movimento espressa in m/s)
    OBJ_CONST_SPEED,    // v (velocità da mantenere) e omega (velocità di rotazione da mantenere)
} objective_type_t;

// struttura dati che verrà mandata al task di movimento
// Tutti i dati di questa struttura saranno in coordinate assolute
// dove l'origine delle coordinate si trova nel punto di partenza
// inziale del robot
typedef struct {

    uint32_t num;               // numero progressivo, id dell'obiettivo
    double v;                   // velocità massima alla quale il robot può andare verso l'obiettivo
    double omega;               // velocità massima alla quale il robot può ruotare 
    double x, y;                // coordinate dell'obiettivo rispetto all'origine
    double theta;               // angolo verso il quale il robot deve girarsi rispetto all'asse x nell'origine
    objective_type_t obj_type;  // obiettivo attuale, indica quali campi di questa struttura dati contano

} navigation_objective_t;


// caratteristiche del task navigation, definita in navigation.c
extern rt_task_t navigation_task;

void init_navigation(void);
navigation_objective_t read_navigation(void);
// funzione per pubblicare il prossimo obiettivo (lo leggerà task movement)
void write_objective(const navigation_objective_t *next_objective);
void job_navigation(void *arg);


#endif