#ifndef TASK_MAPPING_H
#define TASK_MAPPING_H

#include <stdbool.h>
#include <stdint.h>
#include "driver/driver.h"

// Numero massimo di zone proibite che si possono definire in una mappa
#define MAX_FORBIDDEN_ZONES 16

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