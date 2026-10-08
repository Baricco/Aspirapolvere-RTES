#include "driver/driver.h"
#include "driver/scambio.h"
#include <math.h>

#include "movement.h"
#include "navigation.h"

// struttura dati interna che contiene lo stato interno del task
typedef struct {
    uint32_t last_frame_num;            // numero dell'ultimo frame ricevuto, se 0 nessun frame ricevuto
    int64_t last_frame_arrival_time;    // timestamp dell'arrivo dell'ultimo frame
    int32_t l_enc, r_enc;               // posizione della ruota sinistra e destra nel frame precedente
    int16_t last_yaw;                   // imbardata misurata nell'ultimo frame
    double x, y;                        // posizione x e y del robot rispetto alla posizione iniziale
    double theta;                       // rotazione del robot rispetto alla rotazione iniziale, normalizzato tra -M_PI e +M_PI
    double v;                           // velocità misurata del robot 
    double omega;                       // velocità angolare misurata del robot
    uint32_t cur_obj_num;               // numero dell'obiettivo in esecuzione al momento
    bool obj_achieved;                  // indica se l'obiettivo attuale è stato raggiunto

} movement_state_t;


// stato del task di movimento
static movement_state_t mov_state;

// canale di scambio del doppio buffer
static scambio_t double_buffer;
// i due buffer
static movement_position_data_t buffer1, buffer2;

// caratteristiche del task
rt_task_t movement_task;

// funzione di inizializzazione del task
void init_movement(void) {

    // inizializza il doppio buffer passando i due buffer da usare
    // e la loro dimensione 
    scambio_init(
        &double_buffer,
        &buffer1,
        &buffer2, 
        sizeof(movement_position_data_t)
    );

    // azzera la struttura dati dello stato del moviment
    mov_state.last_frame_num = 0;

}

// funzione di lettura dei dati del task
movement_position_data_t read_movement(void) {

    // Crea e Inizializza a 0 la struttura dati position_data
    movement_position_data_t position_data = {0};

    // Copia il buffer pubblicato in position_data
    scambio_leggi(&double_buffer, &position_data);

    // ritorna position_data
    return position_data;
}

// funzione helper che legge il frame dal driver
// e restituisce un nuovo mov_state aggiornato
// ! rempie solo i campi che legge dal driver,
// ! non sostituire mov_state con questo stato
// ! perchè azzererebbe gli altri campi
static movement_state_t get_cur_mov_state_from_driver(void) {

    // Inizializza il frame dei sensori a 0
    drv_sensori_t cur_frame = {0};

    // leggi il cur:frame dei sensori dal driver
    drv_sensori(&cur_frame);

    // Inzializza il movement_state a 0
    movement_state_t cur_mov_state = {0};

    // Aggiorna il last_frame_num con il numero del frame
    cur_mov_state.last_frame_num = cur_frame.numero;

    // Aggiorna il tempo di arrivo del frame
    cur_mov_state.last_frame_arrival_time = cur_frame.t_ns;

    // Aggiorna encoder sinistro e destro
    cur_mov_state.l_enc = cur_frame.s.encoder_sx;
    cur_mov_state.r_enc = cur_frame.s.encoder_dx;

    // Aggiorna imbardata
    cur_mov_state.last_yaw = cur_frame.s.angolo[0];

    // Aggiorna velocità
    cur_mov_state.v = cur_frame.s.vel_sx / GD32_SCALA_V;

    // Aggiorna velocità angolare
    cur_mov_state.omega = cur_frame.s.vel_dx  / GD32_SCALA_W;

    return cur_mov_state;
}

// Funzione helper che restituisce il movement_position_data_t da pubblicare
// agli altri task calcolato sul cur_state della posizione
static movement_position_data_t get_cur_pos_data_from_cur_state(const movement_state_t *cur_state) {
    

    // Crea la variabile di tipo movement_position_data_t da restituire
    movement_position_data_t next_mov_pos_data = {0};
    
    // Copia nella variabile tutti i valori necessari presi dallo stato attuale
    next_mov_pos_data.num = cur_state->last_frame_num;
    next_mov_pos_data.frame_acq_t = cur_state->last_frame_arrival_time;
    next_mov_pos_data.x = cur_state->x;
    next_mov_pos_data.y = cur_state->y;
    next_mov_pos_data.theta = cur_state->theta;
    next_mov_pos_data.v = cur_state->v;
    next_mov_pos_data.omega = cur_state->omega;
    
    next_mov_pos_data.valid = true; // Questa è da modificare per gestire il sollevamento del robot
    next_mov_pos_data.num_obj = cur_state->cur_obj_num;
    next_mov_pos_data.obj_achieved = false;

    // Ritorna il movement_position_data
    return next_mov_pos_data;

}

// Funzione helper per aggiornare mov_state con un nuovo stato senza
// azzerare gli altri campi
static void update_mov_state(movement_state_t *cur_state, const movement_state_t *next_state) {
    
    // Aggiorna i campi uno per uno, saltando quelli che sappiamo che 
    // non vengono calcolati qui dentro

    cur_state->last_frame_num = next_state->last_frame_num;
    cur_state->last_frame_arrival_time = next_state->last_frame_arrival_time;

    cur_state->l_enc = next_state->l_enc;
    cur_state->r_enc = next_state->r_enc;

    cur_state->last_yaw = next_state->last_yaw;

    cur_state->v = next_state->v;
    cur_state->omega = next_state->omega;

    cur_state->x = next_state->x;
    cur_state->y = next_state->y;
    
    cur_state->theta = next_state->theta;

}

// job del task
void job_movement(void *arg) {
    
    (void)arg;
    
    // questo job si comporta diversamente in base allo stato del driver
    // cioè se non è ancora disponibile un frame dai sensori o meno
    // Se non è disponibile ancora nessun frame dai sensori, verifica 
    // Se il driver sia pronto
    if (mov_state.last_frame_num == 0) {

        // Carica lo stato del driver
        drv_stato_t cur_drv_state = {0};
        drv_stato(&cur_drv_state);

        // Se il driver non è ancora pronto, ritorna
        if (cur_drv_state.fase != DRV_PRONTO) return;

        // Se ci troviamo qui significa che il driver è pronto, quindi
        // possiamo aggiornare lo stato in base al frame del driver
        mov_state = get_cur_mov_state_from_driver();
    }
    // Se il primo frame dei sensori è già arrivato
    else {

        // Salva lo stato del frame letto dai sensori
        // ! questo non è il vero stato del frame aggiornato, leggi
        // ! il commento nella dichiarazione della funzione 
        movement_state_t cur_state = get_cur_mov_state_from_driver();
    
        // Se il frame letto è lo stesso di prima, ritorna
        if (cur_state.last_frame_num == mov_state.last_frame_num) return;

        // Calcola la differenza tra la posizione precedente delle ruote
        // e la posizione attuale delle ruote
        double l_wheel_diff, r_wheel_diff;

        l_wheel_diff = (int32_t)((uint32_t)cur_state.l_enc - (uint32_t)mov_state.l_enc);
        r_wheel_diff = (int32_t)((uint32_t)cur_state.r_enc - (uint32_t)mov_state.r_enc);


        // Converti le differenze in distanze in metri
        l_wheel_diff /= GD32_SCALA_V;
        r_wheel_diff /= GD32_SCALA_V;

        // Il Centro del robot si sposta della media dei due spostamenti
        double mean_displacement = (l_wheel_diff + r_wheel_diff) / 2;


        // Adesso calcoliamo la direzione dello spostamento, 
        // Calcola la differenza tra l'imbardata attuale e quella precedente
        // Questa differenza è in centesimi di grado
        double yaw_diff = cur_state.last_yaw - mov_state.last_yaw;

        // Normalizza la differenza tra -18000 e 18000
        if (yaw_diff > 18000) yaw_diff -= 36000;
        else if (yaw_diff < -18000) yaw_diff += 3600018000;

        // Da questo punto in poi yaw_diff è in radianti
        yaw_diff *= M_PI / 18000;

        // La direzione del robot è quindi quella attuale + metà dell'imbardata 
        // appena calcolata, la salviamo in cur_state.theta
        double mean_direction = mov_state.theta + yaw_diff / 2;

        // Calcola  la posizione attuale usando seno e coseno
        // sulla direzione theta appena calcolata, salva in cur_state.x e cur_state.y
        cur_state.x = mov_state.x + cos(mean_direction) * mean_displacement;
        cur_state.y = mov_state.y + sin(mean_direction) * mean_displacement;

        // Calcola il theta finale
        cur_state.theta = mov_state.theta + yaw_diff;

        // Normalizza il theta finale tra -PI e PI
        if (cur_state.theta > M_PI) cur_state.theta -= M_PI * 2;
        else if (cur_state.theta < -M_PI) cur_state.theta += M_PI * 2;

        // Aggiorna lo stato del movimento
        update_mov_state(&mov_state, &cur_state);

        // Crea un oggetto movement_position_data_t e riempilo con la funzione helper
        movement_position_data_t cur_pos_data = get_cur_pos_data_from_cur_state(&mov_state);

        // Pubblica la nuova pos_data appena calcolata
        scambio_scrivi(&double_buffer, &cur_pos_data);

    }
    

}  