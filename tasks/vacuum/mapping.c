#include "driver/driver.h"
#include "driver/scambio.h"

#include "mapping.h"

// canale di scambio del doppio buffer
static scambio_t double_buffer;
// i due buffer
static mapping_forbidden_zone_list_t buffer1, buffer2;

// caratteristiche del task
rt_task_t mapping_task;



// Griglia di Navigazione come Array piatto
static uint8_t mapping_navigation_grid[NAV_MAP_CELL_NUMBER*NAV_MAP_CELL_NUMBER];       // Questa mappa dovrebbe essere di 65536 celle int da 1 Byte, quindi una coordinata è un numero di 2 Byte (trick di ottimizzazione della memoria per cui le dimensioni sono così)



// funzione di inizializzazione del task
void init_mapping(void) {

    // inizializza il doppio buffer passando i due buffer da usare
    // e la loro dimensione 
    scambio_init(
        &double_buffer,
        &buffer1,
        &buffer2, 
        sizeof(mapping_forbidden_zone_list_t)
    );

}

// funzione di lettura delle zone proibite
mapping_forbidden_zone_list_t read_forbidden_zone_list(void) {

    // Crea e Inizializza a 0 la struttura dati forbidden_zones
    mapping_forbidden_zone_list_t forbidden_zones = {0};

    // Copia il buffer pubblicato in forbidden_zones
    scambio_leggi(&double_buffer, &forbidden_zones);

    // ritorna forbidden_zones
    return forbidden_zones;
}


// funzione di scrittura dei dati del task (le zone proibite)
void write_forbidden_zone_list(const mapping_forbidden_zone_list_t *forbidden_zones) {

    // Copia scrivi le zone proibite nel doppio buffer
    scambio_scrivi(&double_buffer, forbidden_zones);

    return;
}

// job del task
void job_mapping(void *arg) {
    
    (void)arg;


}  