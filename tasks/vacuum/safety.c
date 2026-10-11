#include "driver/driver.h"
#include "driver/scambio.h"
#include <math.h>

#include "safety.h"
#include "movement.h"
#include "lidar.h"

// Macro Helper per calcolare il minimo tra due numeri
#define MIN(a, b) ((a) < (b) ? (a) : (b))

// struttura dati interna che contiene lo stato interno del task
typedef struct {
    
    uint32_t last_frame_num;            // numero dell'ultimo frame ricevuto, se 0 nessun frame ricevuto
    int64_t last_frame_arrival_time;    // timestamp dell'arrivo dell'ultimo frame
    
    uint32_t stuck_frame_counter;       // conta per quanti frame di fila il robot è incastrato
    uint32_t slip_frame_counter;        // conta per quanti frame di fila le ruota slittano
    uint32_t overload_frame_counter;    // conta per quanti frame di file il motore è sotto sforzo

    uint32_t last_scan_num;             // numero dell'ultima scansione del lidar
    int64_t last_scan_end_t;            // timestamp di fine scansione del lidar

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

    // calcola la distanza minima davanti al robot (prendi i tre settori anteriori), convertiti in metri
    double front_distance = (MIN(MIN(scan_data.sector_distance[11], scan_data.sector_distance[1]), scan_data.sector_distance[0]) / 1000.0);

    // calcola la distanza minima dietro al robot (prendi i tre settori posteriori), convertiti in metri
    double back_distance = (MIN(MIN(scan_data.sector_distance[5], scan_data.sector_distance[6]), scan_data.sector_distance[7])/ 1000.0);
    
    


}  