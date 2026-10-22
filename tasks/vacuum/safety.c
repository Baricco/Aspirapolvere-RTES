#include "driver/driver.h"
#include "driver/scambio.h"
#include <math.h>

#include "safety.h"
#include "movement.h"
#include "lidar.h"
#include "mapping.h"


// struttura dati interna che contiene lo stato interno del task
typedef struct {
    
    uint32_t last_frame_num;            // numero dell'ultimo frame ricevuto, se 0 nessun frame ricevuto
    
    uint32_t stuck_frame_counter;       // conta per quanti frame di fila il robot è incastrato
    uint32_t slip_frame_counter;        // conta per quanti frame di fila le ruota slittano
    uint32_t overload_frame_counter;    // conta per quanti frame di file il motore è sotto sforzo

} safety_state_t;

// stato del task di sicurezza
static safety_state_t safe_state;

// canale di scambio del doppio buffer
static scambio_t double_buffer;

// i due buffer
static safety_alarm_t buffer1, buffer2;

// caratteristiche del task
rt_task_t safety_task;

// funzione di inizializzazione del task
void init_safety(void) {

    // inizializza il doppio buffer passando i due buffer da usare
    // e la loro dimensione 
    scambio_init(
        &double_buffer,
        &buffer1,
        &buffer2, 
        sizeof(safety_alarm_t)
    );

}

// funzione di lettura dei dati del task
safety_alarm_t read_safety(void) {

    // Crea e Inizializza a 0 la struttura dati safety_alarm
    safety_alarm_t safety_alarm = {0};

    // Copia il buffer pubblicato in safety_alarm
    scambio_leggi(&double_buffer, &safety_alarm);

    // ritorna safety_alarm
    return safety_alarm;
}



// Helper che calcola la velocità massima che il robot può raggiungere in sicurezza, data la distanza dell'ostacolo più vicino, limitata tra 0 e MAX_ABSOLUTE_SPEED
static double get_max_speed(double obstacle_distance) {
    
    // calcolo della velocità massima usando il guadagno e la distanza dell'ostacolo
    double max_speed = MOVEMENT_SAFE_GAIN * (obstacle_distance - STOP_DISTANCE);

    // clamp tra 0 e MAX_ABSOLUTE_SPEED
    if (max_speed <= 0) return 0;
    else if (max_speed >= MAX_ABSOLUTE_SPEED) return MAX_ABSOLUTE_SPEED;
    
    return max_speed;   

}

// job del task
void job_safety(void *arg) {
    
    (void)arg;

    // Leggi il frame dei sensori
    drv_sensori_t cur_frame = {0};
    drv_sensori(&cur_frame);

    // Se il frame dei sensori è lo stesso dello stato, o è 0 (driver non pronto), esci
    if (cur_frame.numero == safe_state.last_frame_num || cur_frame.numero == 0) return;

    // Leggi le velocità attuali del robot
    drv_stato_t cur_state = {0};
    drv_stato(&cur_state);

    // Ricava i dati sulle velocità di movimento e rotazione
    double expected_v = cur_state.v;     // Velocità impostata dal task movement
    double expected_w = cur_state.w;    // Velocità angolare impostata dal task movement

    // Velocità reali delle ruote misurate dal driver
    double encoder_v = cur_frame.s.vel_sx / GD32_SCALA_V;
    double encoder_w = cur_frame.s.vel_dx / GD32_SCALA_W;

    // Velocità angolare del robot misurata dall'IMU 
    double imu_w = cur_frame.s.vel_angolare[0] * (M_PI / 18000) * YAW_SCALE;  

    // Inizializza a zero la maschera degli errori
    uint8_t cur_error_mask = 0;
    
    // Controllo dell'incastro
    {       // Istanzio queste variabili in un blocco di codice separato perchè le devo usare solo qui

        // Calcola le 4 condizioni da verificare (v comandata, w comandata, v bloccata, w bloccata)
        bool is_v_expected = fabs(expected_v) >= THRESHOLD_STUCK_MIN_V;
        bool is_w_expected = fabs(expected_w) >= THRESHOLD_STUCK_MIN_W;
        bool is_v_stuck = fabs(encoder_v) <= THRESHOLD_STUCK_RATIO_V * fabs(expected_v);
        bool is_w_stuck = fabs(encoder_w) <= THRESHOLD_STUCK_RATIO_W * fabs(expected_w);
        
        // Verifica se il robot è bloccato (se il robot si dovrebbe muovere ma non lo fa, true)
        bool is_stuck = (is_v_expected || is_w_expected) &&
                        (!is_v_expected || is_v_stuck) &&
                        (!is_w_expected || is_w_stuck);


        if (is_stuck) {
            // Se il robot è bloccato, aumenta il contatore dei frame consecutivi in cui risulta bloccato
            safe_state.stuck_frame_counter++;
            if (safe_state.stuck_frame_counter >= THRESHOLD_STUCK_FRAME_COUNTER) {
                // se il robot è bloccato per THRESHOLD_STUCK_FRAME_COUNTER, attiva l'allarme
                cur_error_mask |= SAFETY_ERROR_STUCK;
            }
        } else safe_state.stuck_frame_counter = 0;      // se il robot non risulta bloccato azzera il contatore

    }

    // Controlla se il robot è incastrato ed eventualmente  segnala l'incastro
    drv_incastro((cur_error_mask & SAFETY_ERROR_STUCK) && expected_v >= THRESHOLD_STUCK_MIN_V);

    // Controllo dello slittamento
    {       // Istanzio queste variabili in un blocco di codice separato perchè le devo usare solo qui

        
        // Verifica se il robot sta slittando (se le ruote e l'IMU misurano due velocità angolari differenti true)
        bool is_slip = fabs(encoder_w - imu_w) >= THRESHOLD_SLIP_W;


        if (is_slip) {
            // Se il robot è in scivolata, aumenta il contatore dei frame consecutivi in cui risulta in scivolata
            safe_state.slip_frame_counter++;
            if (safe_state.slip_frame_counter >= THRESHOLD_SLIP_FRAME_COUNTER) {
                // se il robot è in scivolata per THRESHOLD_SLIP_FRAME_COUNTER, attiva l'allarme
                cur_error_mask |= SAFETY_ERROR_SLIP;
            }
        } else safe_state.slip_frame_counter = 0;      // se il robot non risulta in scivolata azzera il contatore

    }

    // Controllo del sovraccarico dei motori
    {       // Istanzio queste variabili in un blocco di codice separato perchè le devo usare solo qui

        
        // Verifica se i motori sono in sovraccarico (se una delle correnti è > della soglia max, true)
        bool is_overload =  (cur_frame.s.correnti[0] >= THRESHOLD_MAX_CURRENT) ||
                            (cur_frame.s.correnti[1] >= THRESHOLD_MAX_CURRENT);


        if (is_overload) {
            // Se uno dei motori è in sovraccarico, aumenta il contatore dei frame consecutivi in cui risulta in sovraccarico
            safe_state.overload_frame_counter++;
            if (safe_state.overload_frame_counter >= THRESHOLD_OVRLD_FRAME_COUNTER) {
                // se il robot è in sovraccarico per THRESHOLD_OVRLD_FRAME_COUNTER, attiva l'allarme
                cur_error_mask |= SAFETY_ERROR_MOT_OVRLD;
            }
        } else safe_state.overload_frame_counter = 0;      // se il robot non risulta in sovraccarico azzera il contatore

    }

    // Controllo degli ostacoli

    // Leggi i dati dello scan del lidar
    lidar_scan_data_t scan_data = read_lidar();


    // crea le variabili di distanza degli ostacoli davanti e dietro l'aspirapolvere e inizializzali alla distanza minima
    double obstacle_front_distance = THRESHOLD_DISTANCE;
    double obstacle_back_distance = THRESHOLD_DISTANCE;


    // Controlla che i dati dello scan del lidar siano validi
    if (scan_data.valid && scan_data.num != 0 && (cur_frame.t_ns - scan_data.scan_end_t) < LIDAR_SCAN_MAX_AGE) {

        // calcola la distanza minima davanti al robot (prendi i tre settori anteriori), convertiti in metri
        obstacle_front_distance = scan_data.corridor_front_distance / 1000.0;
        
        // prima c'era questo in obstacle_front_distance:
        // (MIN(MIN(scan_data.sector_distance[11], scan_data.sector_distance[1]), scan_data.sector_distance[0]) / 1000.0);

        // calcola la distanza minima dietro al robot (prendi i tre settori posteriori), convertiti in metri
        obstacle_back_distance = scan_data.corridor_back_distance / 1000.0;
        // prim c'era questo in obstacle_back_distance
        // (MIN(MIN(scan_data.sector_distance[5], scan_data.sector_distance[6]), scan_data.sector_distance[7])/ 1000.0);

    }

    // Attiva l'allarme dell'ostacolo se la distanza davanti o dietro è sotto la soglia di sicurezza
    if (obstacle_front_distance < THRESHOLD_DISTANCE || obstacle_back_distance < THRESHOLD_DISTANCE) {
        cur_error_mask |= SAFETY_ERROR_OBSTACLE;
    }

    // Calcola la velocità massima che il robot può raggiungere avanti e indietro, compresa tra 0 e MAX_ABSOLUTE_SPEED
    double obstacle_front_max_v = get_max_speed(obstacle_front_distance);
    double obstacle_back_max_v = get_max_speed(obstacle_back_distance);


    // Controllo delle Zone Proibite

    // crea le variabili di distanza delle zone proibite davanti e dietro l'aspirapolvere e inizializzali a infinito
    double zone_front_distance = INFINITY;
    double zone_back_distance = INFINITY;

    // Leggi la lista di zone proibite
    mapping_forbidden_zone_list_t cur_forb_zones = read_forbidden_zone_list();

    // Leggi la posizione attuale del robot
    movement_position_data_t cur_position = read_movement();

    // se la posizione attuale non è valida e ci sono delle zone proibite
    if (!cur_position.valid && cur_forb_zones.num_zones > 0) {

        // metti le distanze al minimo, per sicurezza
        zone_back_distance = THRESHOLD_DISTANCE;
        zone_front_distance = THRESHOLD_DISTANCE;
    
    } else {                    // Se la posizione è valida
        
        // Per ogni zona proibita

        // Pre calcola seno e coseno dell'angolo del robot, non dipende dalla zona
        double sin_theta = sin(cur_position.theta);
        double cos_theta = cos(cur_position.theta);

        for (int i = 0; i < cur_forb_zones.num_zones; i++) {
            
            mapping_forbidden_zone_t cur_forbidden_zone = cur_forb_zones.forbidden_zones[i];

            // Calcola la posizione del robot rispetto alla zona proibita
            double b_x = cur_forbidden_zone.x - cur_position.x;                             // positivo se il robot è prima della zona proibita nelle x
            double a_x = cur_position.x - (cur_forbidden_zone.x + cur_forbidden_zone.w);    // positivo se il robot è dopo la zona proibita nelle x
            double b_y = cur_forbidden_zone.y - cur_position.y;                             // positivo se il robot è prima della zona proibita nelle y
            double a_y = cur_position.y - (cur_forbidden_zone.y + cur_forbidden_zone.h);    // positivo se il robot è dopo la zona proibita nelle y

            // Calcola il vettore tra il robot e il punto più vicino alla zona proibita
            double vect_x = MAX(b_x, 0) - MAX(a_x, 0);     // componente x del vettore
            double vect_y = MAX(b_y, 0) - MAX(a_y, 0);     // componente y del vettore
            
            // Se entrambe le componenti del vettore sono 0, il robot si trova dentro la zona proibita
            if (vect_x == 0 && vect_y == 0) {
                // attiva l'allarme della forbidden zone
                cur_error_mask |= SAFETY_ERROR_FORB_ZONE;
                
                // procedi con la prossima zona, in modo che per uscire dalla zona il robot non entri
                // nelle eventuali zone proibite adiacenti
                continue;
            }

            // Calcola la distanza e la direzione del robot rispetto alla zona proibita
            double dist = hypot(vect_x, vect_y) - ROBOT_RADIUS / 1000.0;

            // in base alla direzione del robot rispetto alla zona proibita, aggiorna la front/back distance
            if (vect_x * cos_theta + vect_y * sin_theta >= 0) zone_front_distance = MIN(zone_front_distance, dist);
            else zone_back_distance = MIN(zone_back_distance, dist);

        }

        // se la distanza dalle zone proibite rispetto al robot è inferiore alla THRESHOLD_DISTANCE, attiva l'allarme
        if (zone_front_distance < THRESHOLD_DISTANCE || zone_back_distance < THRESHOLD_DISTANCE) {
            cur_error_mask |= SAFETY_ERROR_FORB_ZONE;
        }
    }

    // calcola le velocità massime per il robot rispetto alle zone proibite
    double zone_front_max_v = get_max_speed(zone_front_distance);
    double zone_back_max_v = get_max_speed(zone_back_distance);

    // Pubblicazione degli allarmi generati
    
    // inizializza a 0 la struttura allarme corrente
    safety_alarm_t cur_alarm = {0};

    // riempi l'allarme corrente con i dati appena ricavati
    cur_alarm.frame_num = cur_frame.numero;
    cur_alarm.frame_acq_t = cur_frame.t_ns;
    cur_alarm.error_mask = cur_error_mask;
    cur_alarm.max_v_forw = MIN(obstacle_front_max_v, zone_front_max_v);
    cur_alarm.max_v_backw = MIN(obstacle_back_max_v, zone_back_max_v);

    // pubblica il dato sul doppio buffer
    scambio_scrivi(&double_buffer, &cur_alarm);
    
    // aggiorna il numero dell'ultimo frame analizzato
    safe_state.last_frame_num = cur_frame.numero;

}  