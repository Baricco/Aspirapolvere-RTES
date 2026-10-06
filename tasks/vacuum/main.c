#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "driver/driver.h"
#include "lidar.h"


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


rt_task_t lidar_task;

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

	rt_task_cfg_t task_cfg_lidar;
	task_cfg_lidar.nome = // da continuare dopo sono rotto di palle sinc



	// ciclo di esecuzione main loop
	// dopo aver avviato i task nei loro thread
	// il main aspetta solo il segnale per terminare
	// e stampa i dati eventualmente
	while (!fine) {

		sleep(1);
		
		// Leggi lo stato del driver
		drv_stato_t drv_state;
		drv_stato(&drv_state);

		// Stampa lo stato del driver
		printDrvState(&drv_state);
	
	}

	// Uscita Graceful
	drv_ferma();
	return 0;
}