#include "driver/driver.h"
#include "driver/scambio.h"
#include <math.h>

#include "lidar.h"

// canale di scambio del doppio buffer
static scambio_t double_buffer;
// i due buffer
static lidar_scan_data_t buffer1, buffer2;

// caratteristiche del task
rt_task_t lidar_task;

// ultimo giro ricevuto dal driver
static uint32_t last_scan_num;

// funzione di inizializzazione del task
void init_lidar(void) {

    // inizializza il doppio buffer passando i due buffer da usare
    // e la loro dimensione 
    scambio_init(
        &double_buffer,
        &buffer1,
        &buffer2, 
        sizeof(lidar_scan_data_t)
    );

    // accendi il lidar alla velocità di rotazione di default
    drv_lidar(1, 0);
    
    // inizializza a 0 il numero di giri eseguiti
    last_scan_num = 0;
}

static void init_lidar_scan_data(lidar_scan_data_t *scan_data) {
    // Inizializza la struttura dati da ritornare a 0
    *scan_data = (lidar_scan_data_t){0};
    // Inizializza i settori a NO_READ
    for (int i = 0; i < SECTOR_NUMBER; i++) scan_data->sector_distance[i] = NO_READ;

    // inizializza le distanze davanti e dietro a NO_READ
    scan_data->corridor_front_distance = NO_READ;
    scan_data->corridor_back_distance = NO_READ;
    
}

// funzione di lettura dei dati del task
lidar_scan_data_t read_lidar(void) {

    // Crea e Inizializza a 0 la struttura dati scan_data
    lidar_scan_data_t scan_data;
    init_lidar_scan_data(&scan_data);

    // Copia il buffer pubblicato in scan_data
    scambio_leggi(&double_buffer, &scan_data);

    // ritorna scan_data
    return scan_data;
}

// job del task
void job_lidar(void *arg) {
    
    (void)arg;
    
    static lidar_giro_t cur_scan;
    
    // leggi la scansione lidar dal driver
    uint32_t scan_num = drv_giro(&cur_scan);

    // confronta il numero di scansione dall'ultimo numero letto
    if (scan_num <= last_scan_num) return;  // se già letto ritorna

    // creazione e inizializzazione a 0 della struttura scan_data
    lidar_scan_data_t cur_scan_data;
    init_lidar_scan_data(&cur_scan_data);

    // copia il numero di scan
    cur_scan_data.num = scan_num;

    // copia il timestamp di fine lettura
    cur_scan_data.scan_end_t = cur_scan.t_fine_ns;
    
    // contatore dei punti validi (!= 0) letti nella scansione
    uint16_t valid_points_counter = 0;

    // scorri tutti i punti letti nella scansione
    for (uint16_t i = 0; i < cur_scan.n; i++) {

        // scarta i punti a distanza 0mm
        if (cur_scan.p[i].distanza_mm == 0) continue;

        // se siamo arrivati qui il punto corrente è valido
        valid_points_counter++;

        // converti il punto del lidar in coordinate x,y
        // rispetto al centro del robot
        double x, y;
        drv_punto_xy(&cur_scan.p[i], &x, &y);
        
        // calcola l'angolo da (x, y)
        double angle = atan2(y, x);

        // shifta l'angolo di mezzo settore a sinistra
        angle += M_PI / SECTOR_NUMBER;

        // ottieni il numero di settore dall'angolo
        // dividendo l'angolo per la larghezza dei settori
        int sector_number = ((int)floor(angle / (M_PI * 2 / SECTOR_NUMBER)) + SECTOR_NUMBER) % SECTOR_NUMBER;
    
        // calcola la distanza del punto misurato dal bordo del robot
        int distance = (hypot(x, y) * 1000) - ROBOT_RADIUS;

        // rimuovi le distanze negative per evitare problemi di underflow
        if (distance < 0) distance = 0;

        // da qui in poi le coordinate servono in mm, quindi le moltiplico per 1000
        x *= 1000.0;
        y *= 1000.0;

        // se il punto misurato è il pù vicino in quel settore, aggiorna cur_scan_data
        if (cur_scan_data.sector_distance[sector_number] > distance) cur_scan_data.sector_distance[sector_number] = distance;
    
        // se la distanza dal punto misurato è 0 o il punto non si trova nella direzione di 
        // movimento del robot, esci
        if (distance == 0 || fabs(y) >= ROBOT_RADIUS) continue;

        // calcola la distanza del punto rispetto al bordo del robot
        double dist = fabs(x) - sqrt(ROBOT_RADIUS*ROBOT_RADIUS - y*y);

        // se la distanza è < 0 (errore dovuto al rumore del lidar), normalizza a 0
        if (dist < 0) dist = 0;

        // aggiorna la distanza degli ostacoli davanti o dietro al robot (in base alla direzione)
        if (x > 0) {
            if (cur_scan_data.corridor_front_distance > dist) cur_scan_data.corridor_front_distance = dist;
        } else if (cur_scan_data.corridor_back_distance > dist) cur_scan_data.corridor_back_distance = dist; 

        
    }

    // controlla che il numero di punti validi sia >25% dei punti totali (RELIABILITY FACTOR)
    // se non ci sono abbastanza punti validi, marca la scansione come invalida
    cur_scan_data.valid = (valid_points_counter > cur_scan.n * RELIABILITY_FACTOR) ; 

    // Pubblica la nuova lettura (scan_data)
    scambio_scrivi(&double_buffer, &cur_scan_data);

    // Aggiorna last_scan_num
    last_scan_num = scan_num;
}  