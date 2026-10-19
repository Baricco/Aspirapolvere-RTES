#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>

#include "driver/driver.h"
#include "lidar.h"
#include "movement.h"
#include "navigation.h"
#include "safety.h"
#include "mapping.h"

// Gestione Graceful Exit dei segnali CTRL + C 
static volatile sig_atomic_t fine;
static void a_fine(int sig) { (void)sig; fine = 1; }

// stampa lo stato del driver con tutti i suoi campi
void printDrvState(drv_stato_t *drv_state) {
	printf("t_ns: %lld\n"
       "fase: %d\n"
       "guasto: %s\n"
       "versione_gd32: %s\n"
       "volt: %.2f\n"
       "percentuale: %d\n"
       "batteria: %d\n"
       "in_carica: %d\n"
       "sulla_base: %d\n"
       "temperatura_c: %d\n"
       "v: %+.3f\n"
       "w: %+.3f\n"
       "limiti: 0x%x\n"
       "spazzola_laterale: %d\n"
       "spazzola_centrale: %d\n"
       "aspirazione_giri: %d\n"
       "led: %d\n"
       "aspirazione_limiti: 0x%x\n"
       "lidar_acceso: %d\n"
       "lidar_potenza: %d\n"
       "lidar_ok: %d\n"
       "lidar_giri_s: %.2f\n"
       "lidar_giri: %u\n"
       "frame_sensori: %llu\n"
       "frame_errati: %llu\n"
       "byte_scartati: %llu\n"
       "frame_gd32_altri: %llu\n"
       "heartbeat_gd32: %llu\n"
       "keepalive: %llu\n"
       "heartbeat: %llu\n"
       "errori_scrittura: %llu\n"
       "sensori_hz: %.1f\n"
       "eta_sensori_ns: %lld\n"
       "tick_ritardo_max_ns: %lld\n"
       "intervallo_sensori_max_ns: %lld\n"
       "intervallo_sensori_min_ns: %lld\n"
       "lavoro_max_ns: %lld\n"
       "risvegli: %llu\n"
       "senza_rt: %d\n\n",
       (long long)drv_state->t_ns, drv_state->fase, drv_state->guasto, drv_state->versione_gd32,
       drv_state->volt, drv_state->percentuale, drv_state->batteria,
       drv_state->in_carica, drv_state->sulla_base, drv_state->temperatura_c,
       drv_state->v, drv_state->w, drv_state->limiti,
       drv_state->spazzola_laterale, drv_state->spazzola_centrale, drv_state->aspirazione_giri, drv_state->led,
       drv_state->aspirazione_limiti,
       drv_state->lidar_acceso, drv_state->lidar_potenza, drv_state->lidar_ok,
       drv_state->lidar_giri_s, (unsigned)drv_state->lidar_giri,
       (unsigned long long)drv_state->frame_sensori, (unsigned long long)drv_state->frame_errati,
       (unsigned long long)drv_state->byte_scartati, (unsigned long long)drv_state->frame_gd32_altri,
       (unsigned long long)drv_state->heartbeat_gd32, (unsigned long long)drv_state->keepalive,
       (unsigned long long)drv_state->heartbeat, (unsigned long long)drv_state->errori_scrittura,
       drv_state->sensori_hz, (long long)drv_state->eta_sensori_ns,
       (long long)drv_state->tick_ritardo_max_ns, (long long)drv_state->intervallo_sensori_max_ns,
       (long long)drv_state->intervallo_sensori_min_ns, (long long)drv_state->lavoro_max_ns,
       (unsigned long long)drv_state->risvegli, drv_state->senza_rt);
}

int main(void) {

	// fai il bind dei segnali di terminazione alla funzione per la terminazione graceful
	signal(SIGINT, a_fine);
	signal(SIGTERM, a_fine);

	/* 
	 * Il driver: sul robot usa /dev/ttyS3 e /dev/ttyS1, 
	 * nel simulatore le pseudo-seriali (./rtes.sh sim). 
	*/
	drv_config_t c;
	drv_config_default(&c);
	if (drv_avvia(&c)) return 1;

	rt_consenti_senza_privilegi(c.senza_rt);   /* sul PC senza permessi per SCHED_FIFO i task partono lo stesso */
	drv_esci_dalla_base(1);                    /* se parte sulla base può uscire (piano, in avanti) */


	// Avvia i task
	init_lidar();
       init_movement();
       init_navigation();
       init_safety();

       // se mapping fallisce nell'inizializzazione del mutex, 
       // chiudi il driver e ferma tutto
       if (init_mapping() != 0) {
              drv_ferma();
              return 1;
       }

       // Task del lidar
	rt_task_cfg_t task_cfg_lidar = {
              .nome = "task_lidar",              // nome del task
              .cpu = 1,                          // su quale core gira
              .sporadico = 0,                    // task periodico
              .periodo_ns = RT_MS(200),          // periodo in ms
              .job = job_lidar,                  // job da eseguire
       };

       // Task del movimento
       rt_task_cfg_t task_cfg_movement = {
              .nome = "task_movement",
              .cpu = 1,
              .sporadico = 1,
              .periodo_ns = RT_MS(15),
              .job = job_movement,
       };

       // Task della safety
       rt_task_cfg_t task_cfg_safety = {
              .nome = "task_safety",
              .cpu = 1,
              .sporadico = 1,
              .periodo_ns = RT_MS(15),
              .job = job_safety,
       };

       rt_task_cfg_t task_cfg_navigation = {
              .nome = "task_navigation",
              .cpu = 2,
              .sporadico = 0,
              .periodo_ns = RT_MS(100),
              .job = job_navigation,
       };
       
       // Task della mappatura
       rt_task_cfg_t task_cfg_mapping = {
              .nome = "task_mapping",
              .cpu = 2,
              .sporadico = 0,
              .periodo_ns = RT_MS(200),
              .job = job_mapping,
       };

       
       // Creiamo l'array di puntatori ai task_cfg per
       // poter calcolare le priorità dei task usando
       // RM a partire dai loro periodi
       rt_task_cfg_t *task_cfgs[] = { &task_cfg_lidar, &task_cfg_movement, &task_cfg_safety, &task_cfg_navigation, &task_cfg_mapping };

       // calcola il numero di task dentro l'array
       int task_number = sizeof(task_cfgs) / sizeof(task_cfgs[0]);

       // Assegna le priorità corrispondenti ai task in base al
       // periodo di ciascun task
       if (rt_priorita_rm(task_cfgs, task_number, RT_PRIO_TASK_MAX)) {
              // In caso di problemi (task_number < 0 o > 64)
              // ferma tutto e ritorna 1
              drv_ferma();
              return 1;
       }

       // salvati il timestamp attuale per poter far partire tutti 
       // i task contemporaneamente
       int64_t now_time = rt_adesso(); 

       // Dopo aver creato i task, avviali

       // Avvia il task lidar
	if (rt_task_avvia(&lidar_task, &task_cfg_lidar, now_time)) {
              // in caso di problemi (permessi mancanti o errore di creazione del thread)
              // ferma tutto e ritorna 1
              drv_ferma();
		return 1;
	}

       // Avvia il task movement
       if (rt_task_avvia(&movement_task, &task_cfg_movement, now_time)) {
              // in caso di problemi ferma tutto, inclusi i task
              // avviati prima e ritorna 1
              rt_task_ferma(&lidar_task);
              drv_ferma();
              return 1;
       }

       // Avvia il task safety
       if (rt_task_avvia(&safety_task, &task_cfg_safety, now_time)) {
              // in caso di problemi ferma tutto, inclusi i task
              // avviati prima e ritorna 1
              rt_task_ferma(&lidar_task);
              rt_task_ferma(&movement_task);
              drv_ferma();
              return 1;
       }

       // Avvia il task navigation
       if (rt_task_avvia(&navigation_task, &task_cfg_navigation, now_time)) {
              // in caso di problemi ferma tutto, inclusi i task
              // avviati prima e ritorna 1
              rt_task_ferma(&lidar_task);
              rt_task_ferma(&movement_task);
              rt_task_ferma(&safety_task);
              drv_ferma();
              return 1;
       }

       // Avvia il task mapping
       if (rt_task_avvia(&mapping_task, &task_cfg_mapping, now_time)) {
              // in caso di problemi ferma tutto, inclusi i task
              // avviati prima e ritorna 1
              rt_task_ferma(&lidar_task);
              rt_task_ferma(&movement_task);
              rt_task_ferma(&safety_task);
              rt_task_ferma(&navigation_task);
              drv_ferma();
              return 1;
       }

       // sveglia il task del moto con l'arrivo del frame dai sensori
       drv_sveglia_ai_sensori(&movement_task);

       // sveglia il task della safety con l'arrivo del frame dai sensori
       drv_sveglia_ai_sensori(&safety_task);


       // attiva il watchdog dei task, questa verifica che non 
       // missati più di 3 periodi
       drv_sorveglia(&lidar_task, 0, 0);
       drv_sorveglia(&movement_task, 1, 0);
       drv_sorveglia(&safety_task, 1, 0);
       drv_sorveglia(&navigation_task, 1, 0);
       drv_sorveglia(&mapping_task, 0, 0);


	// ciclo di esecuzione main loop
	// dopo aver avviato i task nei loro thread
	// il main aspetta solo il segnale per terminare
	// e stampa i dati eventualmente
       
	while (!fine) {

       }

       // Spegni il Watchdog dei task
       drv_dimentica(&lidar_task);               // lidar
       drv_dimentica(&movement_task);            // movimento
       drv_dimentica(&safety_task);              // safety
       drv_dimentica(&navigation_task);          // navigazione
       drv_dimentica(&mapping_task);             // mappatura


       // ferma i task
       rt_task_ferma(&lidar_task);               // lidar
       rt_task_ferma(&movement_task);            // movimento
       rt_task_ferma(&safety_task);              // safety
       rt_task_ferma(&navigation_task);          // navigazione
       rt_task_ferma(&mapping_task);             // mappatura

	// ferma il driver 
       drv_ferma();

       // Alla fine stampa le statistiche dei task
       // cioè job eseguiti, deadline miss e tempi di esecuzione
       rt_stats_stampa(&lidar_task);      // lidar
       rt_stats_stampa(&movement_task);   // movimento
       rt_stats_stampa(&safety_task);     // safety
       rt_stats_stampa(&navigation_task); // navigazione
       rt_stats_stampa(&mapping_task);    // mappatura



	return 0;
}