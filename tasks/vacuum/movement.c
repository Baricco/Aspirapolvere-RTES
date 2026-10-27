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
    next_mov_pos_data.obj_achieved = cur_state->obj_achieved;

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

// Funzione helper che normalizza un angolo tra due i valori -max_min_value e max_min_value
static void normalize_angle(const double max_min_value, double *to_normalize) {
    if (*to_normalize > max_min_value) *to_normalize -= max_min_value * 2;
    else if (*to_normalize < -max_min_value) *to_normalize += max_min_value * 2;
}

// Funzione helper che limita un valore tra -max_min e max_min, conservando il segno
static double saturate_value(const double max_min, double value) {
    if (fabs(value) > max_min) {
        // ritorna value saturato a max_min
        return max_min * (value < 0 ? -1 : 1);
    }
    return value;
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
        normalize_angle(18000, &yaw_diff);

        // Da questo punto in poi yaw_diff è in radianti
        yaw_diff *= M_PI / 18000;

        // Correggi l'errore di scala dell'IMU (vedi YAW_SCALE in movement.h)
        yaw_diff *= YAW_SCALE;

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
        normalize_angle(M_PI, &cur_state.theta);

        // Aggiorna lo stato del movimento
        update_mov_state(&mov_state, &cur_state);

        // Leggi l'obiettivo da raggiungere
        navigation_objective_t cur_obj = read_navigation();

        // Controlla se l'obiettivo è cambiato rispetto al precedente
        if (mov_state.cur_obj_num != cur_obj.num) {
            // se è cambiato segna il numero del nuovo obiettivo
            // nello stato e imposta obj_achieved a false

            mov_state.cur_obj_num = cur_obj.num;

            mov_state.obj_achieved = false;

        }

        // dichiara le prossime v e omega da impostare dopo
        // usando drv_velocità(...)
        double next_v = 0, next_omega = 0;

        // Controlla se l'obiettivo è stato raggiunto
        if (mov_state.obj_achieved) {
            // Se sì, ferma i motori
            next_v = 0;
            next_omega = 0;
        } else {
            
            // Altrimenti controlla il tipo di obiettivo da raggiungere
            switch (cur_obj.obj_type) {
                case OBJ_STOP: {        // se l'obiettivo è stare fermi
                    // ferma i motori
                    next_v = 0;
                    next_omega = 0;

                    // controlla se la velocità e la rotazione sono a 0
                    if (fabs(mov_state.v) <= STOP_V_TOLERANCE &&
                        fabs(mov_state.omega) <= STOP_OMEGA_TOLERANCE) {
                        // se sono entrambi a 0 (entro una certa tolleranza)
                        // imposta obj_achieved a true
                        mov_state.obj_achieved = true;    
                    }
                    
                    break;
                }
                
                case OBJ_ROTATE: {      // se l'obiettivo è ruotare
                    // calcola l'errore di rotazione attuale
                    double cur_error = cur_obj.theta - mov_state.theta;
                    normalize_angle(M_PI, &cur_error);
                    
                    // controlla se l'errore è <= della tolleranza
                    if (fabs(cur_error) <= ROTATE_TOLERANCE_RAD) {
                        // Se sì, imposta obj_achieved a true
                        mov_state.obj_achieved = true;
                        // esci dallo switch, l'obiettivo è già raggiunto
                        // quindi possiamo lasciare v e omega a 0
                        break;
                    }
                    

                    // se l'errore è > 0, gira a sinistra, altrimenti a destra
                    next_omega = cur_error * ROTATION_GAIN;
                    
                    // satura next_omega alla velocità massima di rotazione 
                    next_omega = saturate_value(cur_obj.omega, next_omega);

                    break;
                }

                case OBJ_MOVE: {        // se l'obiettivo è spostarsi

                    // calcola dx e dy del robot rispetto all'obiettivo
                    double dx = cur_obj.x - mov_state.x,
                           dy = cur_obj.y - mov_state.y;

                    // calcola la distanza tra il robot e l'obiettivo
                    double dist_error = hypot(dx, dy);

                    // verifica se la distanza è <= della tolleranza
                    if (dist_error <= MOVE_TOLERANCE_M) {
                        // se sì, imposta obj_achieved a true
                        mov_state.obj_achieved = true;

                        // esci dallo switch, l'obiettivo è già raggiunto
                        // quindi possiamo lasciare v e omega a 0
                        break;
                    }


                    // Calcola la direzione dell'obiettivo rispetto a quella del robot,
                    // se il robot ha velocità negativa, significa che deve andare indietro 
                    // quindi sommiamo PI alla dir_error
                    double dir_error = atan2(dy, dx) - mov_state.theta + (signbit(cur_obj.v) ? M_PI : 0);
                    normalize_angle(M_PI, &dir_error);

                    
                    // la velocità di rotazione sarà l'errore di direzione
                    // moltiplicato per il guadagno di rotazione
                    next_omega = dir_error * ROTATION_GAIN;

                    // satura next_omega alla velocità massima di rotazione 
                    next_omega = saturate_value(cur_obj.omega, next_omega);

                    // prima di modificare anche la velocità verifica
                    // che l'errore di direzione sia sotto
                    // una certa soglia
                    if (fabs(dir_error) > ALIGNMENT_THRESHOLD) break;

                    // la velocità di movimento sarà la distanza
                    // moltiplicata per il guadagno di spostamento
                    next_v = dist_error * MOVE_GAIN;

                    // satura next_v alla velocità massima di spostamento
                    next_v = saturate_value(fabs(cur_obj.v), next_v);

                    // moltiplica la velocità per il coseno della direzione
                    // in questo modo, la velocità sarà al massimo quando 
                    // la direzione è corretta, e sarà modulata dal coseno
                    // quando è sbagliata
                    next_v *= cos(dir_error);

                    if (signbit(cur_obj.v)) next_v *= -1;

                    break;
                }

                case OBJ_CONST_SPEED: {     // se l'obiettivo è mantenere velocità costante

                    // Questa è abbastanza banale, basta copiare dall'obiettivo
                    next_omega = cur_obj.omega;
                    next_v = cur_obj.v;
                    
                    break;
                }

                default: {
                    // Qualcosa è andato storto, per adesso facciamo che si ferma 
                    // e basta, in futuro deve anche andare in guasto
                    next_v = 0;
                    next_omega = 0;

                    break;
                }
            }

        }

        // Imposta la velocità di movimento e rotazione a next_v e next_omega
        drv_velocita(next_v, next_omega);

        // Crea un oggetto movement_position_data_t e riempilo con la funzione helper
        movement_position_data_t cur_pos_data = get_cur_pos_data_from_cur_state(&mov_state);

        // se la posizione è valida, manda la posizione al driver
        if (cur_pos_data.valid) drv_posa(cur_pos_data.x, cur_pos_data.y, cur_pos_data.theta);

        // Pubblica la nuova pos_data appena calcolata
        scambio_scrivi(&double_buffer, &cur_pos_data);

    }
    

}  