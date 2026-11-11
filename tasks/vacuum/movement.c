#include "driver/driver.h"
#include "driver/exchange.h"
#include <math.h>

#include "movement.h"
#include "navigation.h"
#include "mapping.h"

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

    double map_x, map_y, map_theta;     // posizione del robot sulla mappa

    double last_cmd_v, last_cmd_omega;  // ultime velocità comandate, servono per la rampa
    int64_t last_cmd_time;              // istante dell'ultimo comando di velocità
    int64_t pass_achieved_time;         // istante in cui è stato raggiunto un punto da attraversare, 0 se non c'è
    bool is_position_lost;              // true se la mappatura dice che la posizione è persa o non è stata ritrovata

} movement_state_t;


// stato del task di movimento
static movement_state_t mov_state;

// canale di scambio del doppio buffer
static exchange_t double_buffer;
// i due buffer
static movement_position_data_t buffer1, buffer2;

// canale di scambio dello storico delle pose dell'odometria
static exchange_t double_buffer_history;
// i due buffer
static movement_odom_history_t history_buffer1, history_buffer2;
// lo storico aggiornato da questo task
static movement_odom_history_t odom_history;

// caratteristiche del task
rt_task_t movement_task;

// funzione helper per la rampa: avvicina value a target senza superare max_up se il modulo cresce e max_down se cala
static double ramp_value(const double value, const double target, const double max_up, const double max_down) {

    // se il modulo cresce e il segno non cambia usa il limite di salita, altrimenti quello di discesa
    double max_step = (fabs(target) > fabs(value) && target * value >= 0) ? max_up : max_down;

    double step = target - value;

    if (step > max_step) step = max_step;
    if (step < -max_step) step = -max_step;

    return value + step;
}

// funzione di inizializzazione del task
void init_movement(void) {

    // inizializza il doppio buffer passando i due buffer da usare
    // e la loro dimensione 
    exchange_init(
        &double_buffer,
        &buffer1,
        &buffer2, 
        sizeof(movement_position_data_t)
    );

    // inizializza il doppio buffer dello storico delle pose
    exchange_init(
        &double_buffer_history,
        &history_buffer1,
        &history_buffer2,
        sizeof(movement_odom_history_t)
    );

    // azzera la struttura dati dello stato del moviment
    mov_state.last_frame_num = 0;

}

// funzione di lettura dei dati del task
movement_position_data_t read_movement(void) {

    // Crea e Inizializza a 0 la struttura dati position_data
    movement_position_data_t position_data = {0};

    // Copia il buffer pubblicato in position_data
    exchange_read(&double_buffer, &position_data);

    // ritorna position_data
    return position_data;
}

// funzione di lettura dello storico delle pose dell'odometria.
// la struttura è grande, quindi si passa il puntatore invece di restituirla
void read_odom_history(movement_odom_history_t *history) {

    // se non è ancora stato pubblicato niente, lo storico risulta vuoto
    if (exchange_read(&double_buffer_history, history) == 0) history->count = 0;
}

// funzione helper che legge il frame dal driver
// e restituisce un nuovo mov_state aggiornato
// ! rempie solo i campi che legge dal driver,
// ! non sostituire mov_state con questo stato
// ! perchè azzererebbe gli altri campi
static movement_state_t get_cur_mov_state_from_driver(void) {

    // Inizializza il frame dei sensori a 0
    drv_sensors_t cur_frame = {0};

    // leggi il cur:frame dei sensori dal driver
    drv_sensors(&cur_frame);

    // Inzializza il movement_state a 0
    movement_state_t cur_mov_state = {0};

    // Aggiorna il last_frame_num con il numero del frame
    cur_mov_state.last_frame_num = cur_frame.number;

    // Aggiorna il tempo di arrivo del frame
    cur_mov_state.last_frame_arrival_time = cur_frame.t_ns;

    // Aggiorna encoder sinistro e destro
    cur_mov_state.l_enc = cur_frame.s.encoder_left;
    cur_mov_state.r_enc = cur_frame.s.encoder_right;

    // Aggiorna imbardata
    cur_mov_state.last_yaw = cur_frame.s.angle[0];

    // Aggiorna velocità
    cur_mov_state.v = cur_frame.s.v_measured / GD32_SCALE_V;

    // Aggiorna velocità angolare
    cur_mov_state.omega = cur_frame.s.w_measured  / GD32_SCALE_W;

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
    // x, y e theta sono la posizione sulla mappa: quella che usano navigazione, sicurezza e driver
    next_mov_pos_data.x = cur_state->map_x;
    next_mov_pos_data.y = cur_state->map_y;
    next_mov_pos_data.theta = cur_state->map_theta;
    // odom_x, odom_y e odom_theta sono l'odometria grezza: la usa solo la mappatura per calcolare la correzione
    next_mov_pos_data.odom_x = cur_state->x;
    next_mov_pos_data.odom_y = cur_state->y;
    next_mov_pos_data.odom_theta = cur_state->theta;
    next_mov_pos_data.v = cur_state->v;
    next_mov_pos_data.omega = cur_state->omega;
    
    // la posizione non è valida solo quando la mappatura dice che è persa
    next_mov_pos_data.valid = !cur_state->is_position_lost;
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

// Funzione helper che calcola la posizione sulla mappa partendo dall'odometria grezza (cur_state->x, y e theta)
// e dall'ultima correzione pubblicata dalla mappatura. La correzione dice dove stava il robot sulla mappa quando l'odometria
// valeva odom_x, odom_y e odom_theta: basta ripartire da lì con lo spostamento fatto da allora, ruotato di offset
// ! l'odometria grezza non viene mai modificata, la correzione cambia solo map_x, map_y e map_theta
static void apply_map_correction(movement_state_t *cur_state, const mapping_correction_t *correction) {

    // Finché non è arrivata nessuna correzione, la posizione sulla mappa coincide con l'odometria
    if (correction->version == 0) {
        cur_state->map_x = cur_state->x;
        cur_state->map_y = cur_state->y;
        cur_state->map_theta = cur_state->theta;
        return;
    }

    // calcola l'angolo fra il sistema della mappa e quello dell'odometria
    double offset = correction->map_theta - correction->odom_theta;
    normalize_angle(M_PI, &offset);

    // calcola lo spostamento dell'odometria dall'ultimo allineamento
    double dx = cur_state->x - correction->odom_x;
    double dy = cur_state->y - correction->odom_y;

    // ruota lo spostamento negli assi della mappa e sommalo alla posizione allineata
    cur_state->map_x = correction->map_x + dx * cos(offset) - dy * sin(offset);
    cur_state->map_y = correction->map_y + dx * sin(offset) + dy * cos(offset);

    // calcola la rotazione sulla mappa e normalizzala tra -PI e PI
    cur_state->map_theta = cur_state->theta + offset;
    normalize_angle(M_PI, &cur_state->map_theta);
}

// Funzione helper che aggiunge allo storico la posa attuale dell'odometria grezza e lo pubblica
static void publish_odom_history(const movement_state_t *cur_state) {

    odom_history.pose[odom_history.next_index] = (movement_odom_pose_t){
        .t_ns = cur_state->last_frame_arrival_time,
        .x = cur_state->x,
        .y = cur_state->y,
        .theta = cur_state->theta,
    };

    // il buffer è circolare: arrivato in fondo riparte dall'inizio, sovrascrivendo la posa più vecchia
    odom_history.next_index = (odom_history.next_index + 1) % MOVEMENT_ODOM_HISTORY_SIZE;
    if (odom_history.count < MOVEMENT_ODOM_HISTORY_SIZE) odom_history.count++;

    exchange_write(&double_buffer_history, &odom_history);
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
        drv_status_t cur_drv_state = {0};
        drv_status(&cur_drv_state);

        // Se il driver non è ancora pronto, ritorna
        if (cur_drv_state.phase != DRV_READY) return;

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
        l_wheel_diff /= GD32_SCALE_V;
        r_wheel_diff /= GD32_SCALE_V;

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

        // Aggiungi la nuova posa allo storico dell'odometria e pubblicalo
        publish_odom_history(&mov_state);

        // Leggi la correzione della posizione proveniente dalla mappatura 
        mapping_correction_t map_correction = read_mapping_correction();

        // Calcola la posizione sulla mappa. Va fatto a ogni frame, anche se la correzione è sempre la stessa,
        // perché nel frattempo l'odometria è cambiata. Da qui in poi la guida usa solo la posizione sulla mappa
        apply_map_correction(&mov_state, &map_correction);

        // salva se la posizione è persa, servirà quando si pubblica la posizione
        mov_state.is_position_lost = map_correction.state == MAP_LOST || map_correction.state == MAP_NOT_FOUND;

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

            // se è un punto da attraversare continua ad andare, in attesa che la navigazione mandi il prossimo.
            // la navigazione gira ogni 100 ms: se tarda troppo il robot si ferma
            if (cur_obj.obj_type == OBJ_MOVE && cur_obj.pass_through) {
                if (mov_state.pass_achieved_time == 0) mov_state.pass_achieved_time = rt_now();
                if (rt_now() - mov_state.pass_achieved_time < PASS_HOLD_TIME) {
                    next_v = mov_state.last_cmd_v;
                    next_omega = mov_state.last_cmd_omega;
                }
            } else {
                // Se no, ferma i motori
                next_v = 0;
                next_omega = 0;
            }

        } else {
            mov_state.pass_achieved_time = 0;
            
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
                    double cur_error = cur_obj.theta - mov_state.map_theta;
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
                    double dx = cur_obj.x - mov_state.map_x,
                           dy = cur_obj.y - mov_state.map_y;

                    // calcola la distanza tra il robot e l'obiettivo
                    double dist_error = hypot(dx, dy);

                    // verifica se la distanza è <= della tolleranza (più larga per i punti da attraversare)
                    if (dist_error <= (cur_obj.pass_through ? PASS_TOLERANCE_M : MOVE_TOLERANCE_M)) {
                        // se sì, imposta obj_achieved a true
                        mov_state.obj_achieved = true;

                        // esci dallo switch, l'obiettivo è già raggiunto
                        // quindi possiamo lasciare v e omega a 0
                        break;
                    }


                    // Calcola la direzione dell'obiettivo rispetto a quella del robot,
                    // se il robot ha velocità negativa, significa che deve andare indietro 
                    // quindi sommiamo PI alla dir_error
                    double dir_error = atan2(dy, dx) - mov_state.map_theta + (signbit(cur_obj.v) ? M_PI : 0);
                    normalize_angle(M_PI, &dir_error);

                    
                    // la velocità di rotazione sarà l'errore di direzione
                    // moltiplicato per il guadagno di rotazione
                    next_omega = dir_error * ROTATION_GAIN;

                    // satura next_omega alla velocità massima di rotazione 
                    next_omega = saturate_value(cur_obj.omega, next_omega);

                    // prima di modificare anche la velocità verifica che l'errore di direzione sia sotto
                    // una certa soglia, più stretta vicino al punto: lì girando in corsa ci si girerebbe attorno
                    double align_threshold = (dist_error < ALIGNMENT_NEAR_DIST_M && !cur_obj.pass_through) ? ALIGNMENT_THRESHOLD_NEAR : ALIGNMENT_THRESHOLD;
                    if (fabs(dir_error) > align_threshold) break;

                    // la velocità di movimento sarà la distanza moltiplicata per il guadagno di spostamento:
                    // così il robot frena arrivando. i punti da attraversare non sono un arrivo, quindi lì non frena
                    next_v = cur_obj.pass_through ? fabs(cur_obj.v) : dist_error * MOVE_GAIN;

                    // satura next_v alla velocità massima di spostamento
                    next_v = saturate_value(fabs(cur_obj.v), next_v);

                    // moltiplica la velocità per il coseno della direzione
                    // in questo modo, la velocità sarà al massimo quando 
                    // la direzione è corretta, e sarà modulata dal coseno
                    // quando è sbagliata
                    next_v *= cos(dir_error);

                    // più l'errore di direzione è grande più il robot rallenta, fino a girare soltanto alla soglia
                    next_v *= 1 - fabs(dir_error) / align_threshold;

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
                    // Qualcosa è andato storto: tipo di obiettivo sconosciuto. Il robot si ferma e va in guasto
                    next_v = 0;
                    next_omega = 0;

                    drv_report_fault(DRV_FAULT_UNKNOWN_OBJECTIVE);

                    break;
                }
            }

        }

        if (cur_obj.obj_type != OBJ_STOP) {

            // calcola il tempo trascorso dall'ultimo comando
            double dt = (rt_now() - mov_state.last_cmd_time) / 1e9;
            if (dt > 0.05) dt = 0.05;

            next_v = ramp_value(mov_state.last_cmd_v, next_v, MAX_ACCEL_V * dt, MAX_DECEL_V * dt);
            next_omega = ramp_value(mov_state.last_cmd_omega, next_omega, MAX_ACCEL_OMEGA * dt, MAX_ACCEL_OMEGA * dt);
        }

        mov_state.last_cmd_v = next_v;
        mov_state.last_cmd_omega = next_omega;
        mov_state.last_cmd_time = rt_now();

        // Imposta la velocità di movimento e rotazione a next_v e next_omega
        drv_velocity(next_v, next_omega);

        // Crea un oggetto movement_position_data_t e riempilo con la funzione helper
        movement_position_data_t cur_pos_data = get_cur_pos_data_from_cur_state(&mov_state);

        // se la posizione è valida, manda la posizione al driver
        if (cur_pos_data.valid) drv_pose(cur_pos_data.x, cur_pos_data.y, cur_pos_data.theta);

        // Pubblica la nuova pos_data appena calcolata
        exchange_write(&double_buffer, &cur_pos_data);

    }
    

}  