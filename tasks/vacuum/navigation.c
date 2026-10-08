#include "driver/driver.h"
#include "driver/scambio.h"
#include <math.h>

#include "navigation.h"

// canale di scambio del doppio buffer
static scambio_t double_buffer;
// i due buffer
static navigation_objective_t buffer1, buffer2;

// caratteristiche del task
rt_task_t navigation_task;

// funzione di inizializzazione del task
void init_navigation(void) {

    // inizializza il doppio buffer passando i due buffer da usare
    // e la loro dimensione 
    scambio_init(
        &double_buffer,
        &buffer1,
        &buffer2, 
        sizeof(navigation_objective_t)
    );

}

// funzione di lettura dei dati del task
navigation_objective_t read_navigation(void) {

    // Crea e Inizializza a 0 la struttura dati navigation_data
    navigation_objective_t navigation_data = {0};

    // Copia il buffer pubblicato in navigation_data
    scambio_leggi(&double_buffer, &navigation_data);

    // ritorna navigation_data
    return navigation_data;
}


// funzione di scrittura dei dati del task (l'obiettivo)
void write_objective(const navigation_objective_t *next_objective) {

    // Copia scrivi il prossimo obiettivo nel doppio buffer
    scambio_scrivi(&double_buffer, next_objective);

    return;
}

// job del task
void job_navigation(void *arg) {
    
    (void)arg;


}  