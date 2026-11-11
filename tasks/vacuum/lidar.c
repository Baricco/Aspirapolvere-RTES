#include "driver/driver.h"
#include "driver/exchange.h"
#include <math.h>

#include "lidar.h"

// canale di scambio del doppio buffer
static exchange_t double_buffer;
// i due buffer
static lidar_scan_data_t buffer1, buffer2;

// caratteristiche del task
rt_task_t lidar_task;

// ultimo giro ricevuto dal driver
static uint32_t last_scan_num;

// Array di struct di valori per ogni punto scansionato dal lidar
static struct {
    
    double raw_distance;                // distanza grezza misurata dal lidar del punto
    double angle;                       // angolo misurato dal lidar del punto
    double filtered_distance;           // distanza filtrata rimuovendo il rumore del lidar
    double filtered_x;                  // x del punto rimuovendo il rumore del lidar
    double filtered_y;                  // y del punto rimuovendo il rumore del lidar

} border_points[LIDAR_SCAN_MAX_POINTS];


// funzione di inizializzazione del task
void init_lidar(void) {

    // inizializza il doppio buffer passando i due buffer da usare
    // e la loro dimensione 
    exchange_init(
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
    exchange_read(&double_buffer, &scan_data);

    // ritorna scan_data
    return scan_data;
}

// funzione che popola l'array di border_points, estraendo i valori partendo dalle misurazioni grezze del lidar
static uint16_t collect_border_points(const lidar_scan_t *scan) {

    int point_num = 0;

    // Scorri tutti i punti della scansione
    for (int i = 0; i < scan->n; i++) {

        // scarta i punti senza eco
        if (scan->p[i].distance_mm == 0) continue;

        // converti il punto del lidar in coordinate x,y rispetto al centro del robot
        double x, y;
        drv_point_xy(&scan->p[i], &x, &y);

        // salva distanza dal centro e angolo
        border_points[point_num].raw_distance = hypot(x, y);
        border_points[point_num].angle = atan2(y, x);

        // incrementa il numero di punti validi
        point_num++;
    }

    // restituisci il numero di punti registrati nel'array
    return point_num;
}

// funzione che prende 5 valori e restituisce la mediana
static double median_of_five(const double values[5]) {

    // copia i valori in un array locale, per poterli ordinare
    double sorted[5];
    for (int i = 0; i < 5; i++) sorted[i] = values[i];

    // ordina l'array locale (insertion sort)
    for (int i = 1; i < 5; i++) {

        double cur_value = sorted[i];
        int j = i - 1;

        while (j >= 0 && sorted[j] > cur_value) {
            sorted[j + 1] = sorted[j];
            j--;
        }

        sorted[j + 1] = cur_value;
    }

    return sorted[2];
}

// funzione che toglie il rumore dalle distanze dei punti: ogni distanza viene sostituita dalla mediana
// tra lei e i suoi 4 vicini, poi si ricavano x e y del punto dalla distanza filtrata e dall'angolo
static void filter_border_points(const uint16_t point_num) {

    // scorri tutti i punti validi
    for (int i = 0; i < point_num; i++) {

        double neighbors[5];

        // prendi le distanze grezze del punto e dei suoi vicini, il giro è circolare
        for (int k = -2; k <= 2; k++) {
            neighbors[k + 2] = border_points[(i + k + point_num) % point_num].raw_distance;
        }

        border_points[i].filtered_distance = median_of_five(neighbors);
        border_points[i].filtered_x = border_points[i].filtered_distance * cos(border_points[i].angle);
        border_points[i].filtered_y = border_points[i].filtered_distance * sin(border_points[i].angle);
    }
}

// funzione che dice se andando per BORDER_LOOKAHEAD metri nella direzione passata (in radianti, rispetto al robot),
// il centro del robot resta abbastanza lontano da tutti i punti
static bool is_direction_clear(const double direction, const uint16_t point_num) {

    // distanza dal centro del robot alla quale deve stare un ostacolo, espressa in metri
    const double min_distance = ROBOT_RADIUS / 1000 + BORDER_DISTANCE;

    // calcola seno e coseno della direzione, non dipendono dal punto
    const double cos_dir = cos(direction);
    const double sin_dir = sin(direction);

    // scorri tutti i punti validi
    for (int i = 0; i < point_num; i++) {

        const double point_distance = border_points[i].filtered_distance;

        // i punti troppo lontani non possono bloccare la direzione, salta i calcoli
        if (point_distance > BORDER_LOOKAHEAD + min_distance + 0.05) continue;

        // scomponi la posizione del punto rispetto alla direzione: quanto si trova avanti e quanto di lato
        const double ahead = border_points[i].filtered_x * cos_dir + border_points[i].filtered_y * sin_dir;
        const double side = fabs(-border_points[i].filtered_x * sin_dir + border_points[i].filtered_y * cos_dir);

        // calcola la distanza del punto dal segmento lungo BORDER_LOOKAHEAD
        double segment_distance;
        if (ahead < 0) {                            // il punto è dietro al robot
            segment_distance = point_distance;
        } else if (ahead > BORDER_LOOKAHEAD) {      // il punto è oltre la fine del segmento
            segment_distance = hypot(ahead - BORDER_LOOKAHEAD, side);
        } else {                                    // il punto è a fianco del segmento
            segment_distance = side;
        }

        // la soglia è min_distance, ma se il punto è già più vicino basta non avvicinarsi ancora
        const double threshold = fmin(min_distance, point_distance - BORDER_MARGIN);

        // basta un solo punto troppo vicino per bloccare la direzione
        if (segment_distance < threshold) return false;
    }

    return true;
}

// funzione che calcola i dati per la pulizia dei bordi partendo dai punti della scansione
static void calc_border_data(const lidar_scan_t *scan, lidar_scan_data_t *scan_data) {

    // prendi i punti 
    const uint16_t point_num = collect_border_points(scan);

    // con troppi pochi punti la scansione non è utilizzabile
    if (point_num < 10) return;

    // filtra i punti 
    filter_border_points(point_num);

    // indici del punto più vicino in assoluto e del punto più vicino davanti o a destra, -1 se non esiste
    int nearest_idx = -1, followed_idx = -1;

    for (int i = 0; i < point_num; i++) {

        const double point_distance = border_points[i].filtered_distance;

        if (nearest_idx < 0 || point_distance < border_points[nearest_idx].filtered_distance) {
            nearest_idx = i;
        }

        if (border_points[i].angle <= BORDER_MAX_ANGLE && (followed_idx < 0 || point_distance < border_points[followed_idx].filtered_distance)) {
            followed_idx = i;
        }
    }

    // se l'ostacolo più vicino in assoluto è entro la portata, salva il suo angolo
    if (border_points[nearest_idx].filtered_distance <= BORDER_SCOPE) {
        scan_data->nearest_border = true;
        scan_data->nearest_angle = border_points[nearest_idx].angle;
    }

    // se non c'è un ostacolo da seguire davanti o a destra entro la portata, esci
    if (followed_idx < 0 || border_points[followed_idx].filtered_distance > BORDER_SCOPE) return;

    // parti dalla direzione dell'ostacolo e ruota verso sinistra, così l'ostacolo resta a destra,
    // fermati alla prima direzione libera
    for (int k = 0; k < MAX_STEP_NUMBER; k++) {

        const double direction = border_points[followed_idx].angle + k * SEARCH_STEP;

        if (is_direction_clear(direction, point_num)) {

            scan_data->border_found = true;

            // dopo le somme la direzione può superare M_PI, riportala tra -M_PI e M_PI
            scan_data->border_heading = atan2(sin(direction), cos(direction));

            // distanza dell'ostacolo dal bordo del robot, se esce negativa resta 0
            scan_data->border_distance = fmax(border_points[followed_idx].filtered_distance - ROBOT_RADIUS / 1000, 0);

            return;
        }
    }

    // se arriviamo qui il robot è circondato: border_found resta falso
}


// job del task
void job_lidar(void *arg) {
    
    (void)arg;
    
    static lidar_scan_t cur_scan;
    
    // leggi la scansione lidar dal driver
    uint32_t scan_num = drv_scan(&cur_scan);

    // confronta il numero di scansione dall'ultimo numero letto
    if (scan_num <= last_scan_num) return;  // se già letto ritorna

    // creazione e inizializzazione a 0 della struttura scan_data
    lidar_scan_data_t cur_scan_data;
    init_lidar_scan_data(&cur_scan_data);

    // copia il numero di scan
    cur_scan_data.num = scan_num;

    // copia il timestamp di fine lettura
    cur_scan_data.scan_end_t = cur_scan.t_end_ns;
    
    // contatore dei punti validi (!= 0) letti nella scansione
    uint16_t valid_points_counter = 0;

    // scorri tutti i punti letti nella scansione
    for (uint16_t i = 0; i < cur_scan.n; i++) {

        // scarta i punti a distanza 0mm
        if (cur_scan.p[i].distance_mm == 0) continue;

        // se siamo arrivati qui il punto corrente è valido
        valid_points_counter++;

        // converti il punto del lidar in coordinate x,y
        // rispetto al centro del robot
        double x, y;
        drv_point_xy(&cur_scan.p[i], &x, &y);
        
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

    // calcola i dati per seguire i bordi
    calc_border_data(&cur_scan, &cur_scan_data);

    // controlla che il numero di punti validi sia >25% dei punti totali (RELIABILITY FACTOR)
    // se non ci sono abbastanza punti validi, marca la scansione come invalida
    cur_scan_data.valid = (valid_points_counter > cur_scan.n * RELIABILITY_FACTOR) ; 

    // Pubblica la nuova lettura (scan_data)
    exchange_write(&double_buffer, &cur_scan_data);

    // Aggiorna last_scan_num
    last_scan_num = scan_num;
}  