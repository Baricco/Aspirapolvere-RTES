/*
 * primo.c - Il primo programma con un task, di prova (guida in docs/TASK.md).
 *
 * Il robot va avanti piano. Se tocca qualcosa (bumper) o vede un vuoto davanti (anti-caduta) torna indietro per
 * mezzo secondo, gira a sinistra di 90° (misurati con l'imbardata dell'IMU) e riparte. Il LED dice cosa sta facendo.
 *
 * Un solo task, "guida": SPORADICO, svegliato dal driver a ogni frame dei sensori (50 volte al secondo).
 * A ogni job legge l'ultimo frame, decide e manda il comando di velocità. Il driver fa comunque la sua parte di
 * sicurezza (con il bumper premuto non lascia andare avanti, anche se il task sbagliasse).
 *
 * Nel simulatore:   make build/host/primo && ./rtes.sh sim sim/stanze/soggiorno.txt build/host/primo
 * Sul robot:        tools/driver-robot.sh programma primo p1     (si ferma con tools/driver-robot.sh ferma)
 * Si ferma con Ctrl-C (o SIGTERM) e stampa le statistiche del task.
 */
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "driver/driver.h"

#define V_AVANTI      0.10     /* m/s */
#define V_INDIETRO   -0.05     /* m/s */
#define W_GIRA        0.8      /* rad/s, positivo = antiorario = a sinistra */
#define FRAME_INDIETRO 25      /* 25 frame da 20 ms = 0,5 s */
#define GRADI_GIRA    90
/* Il GD32 non si ferma di colpo: frena con una rampa di ~4 rad/s² (2214 unità/s², docs/hardware.md) e applica i
 * comandi ~29 ms dopo. Da 0,8 rad/s gira ancora 0,8² / (2 x 4) rad ≈ 4,6° più ~1,3° di ritardo: lo stop si dà 6° prima. */
#define ANTICIPO_GRADI 6

/* Il task: il thread, la sua configurazione e le sue statistiche. Globale perché lo usano main e il driver. */
static rt_task_t guida;

/* Stato del comportamento. Lo usa SOLO il job di guida, quindi non serve un mutex. */
enum { AVANTI, INDIETRO, GIRA };
static int fase = AVANTI;
static int frame_in_fase;      /* frame passati nella fase attuale */
static int imbardata_inizio;   /* imbardata all'inizio della rotazione, in centesimi di grado */

/* Differenza fra due angoli in centesimi di grado, riportata in (-180°, 180°]. */
static int differenza_angoli(int a, int b)
{
	int d = (a - b) % 36000;
	if (d > 18000)
		d -= 36000;
	if (d <= -18000)
		d += 36000;
	return d;
}

static void cambia_fase(int nuova, const drv_sensori_t *s)
{
	fase = nuova;
	frame_in_fase = 0;
	imbardata_inizio = s->s.angolo[0];
	drv_led(nuova == AVANTI ? 1 : 3);          /* azzurro mentre va avanti, giallo mentre si allontana */
}

/* Il job: gira a ogni frame dei sensori. Niente printf, malloc, sleep o file qui dentro (docs/TASK.md). */
static void job_guida(void *arg)
{
	(void)arg;
	drv_sensori_t s;
	if (drv_sensori(&s) == 0)
		return;                                /* nessun frame ancora */

	/* Sulla base gli anti-caduta leggono "vuoto" e i sensori sono spenti: lì non valgono. */
	int urto = s.s.bumper_sx || s.s.bumper_dx;
	int vuoto = s.s.cliff[0] || s.s.cliff[1] || s.s.cliff[2] || s.s.cliff[3];
	int ostacolo = !s.s.sulla_base && (urto || vuoto);

	frame_in_fase++;
	switch (fase) {
	case AVANTI:
		if (ostacolo)
			cambia_fase(INDIETRO, &s);
		else
			drv_velocita(V_AVANTI, 0);
		break;
	case INDIETRO:
		drv_velocita(V_INDIETRO, 0);
		if (frame_in_fase >= FRAME_INDIETRO)
			cambia_fase(GIRA, &s);
		break;
	case GIRA: {
		int girati = differenza_angoli(s.s.angolo[0], imbardata_inizio);
		if (abs(girati) >= (GRADI_GIRA - ANTICIPO_GRADI) * 100) {
			drv_velocita(0, 0);
			cambia_fase(AVANTI, &s);
		} else {
			drv_velocita(0, W_GIRA);
		}
		break;
	}
	}
}

static volatile sig_atomic_t fine;
static void a_fine(int sig) { (void)sig; fine = 1; }

int main(void)
{
	signal(SIGINT, a_fine);
	signal(SIGTERM, a_fine);

	/* 1. Il driver: sul robot usa /dev/ttyS3 e /dev/ttyS1, nel simulatore le pseudo-seriali (./rtes.sh sim). */
	drv_config_t c;
	drv_config_default(&c);
	if (drv_avvia(&c))
		return 1;
	rt_consenti_senza_privilegi(c.senza_rt);   /* sul PC senza permessi per SCHED_FIFO i task partono lo stesso */
	drv_esci_dalla_base(1);                    /* se parte sulla base può uscire (piano, in avanti) */

	/* 2. Il task: configurazione e avvio. */
	rt_task_cfg_t cfg = {
		.nome = "guida",
		.cpu = 1,                              /* CPU0 è del driver; i task stanno su CPU1-3 */
		.priorita = RT_PRIO_TASK_MAX,          /* con un solo task non serve rt_priorita_rm */
		.sporadico = 1,                        /* un job per ogni sveglia del driver */
		.periodo_ns = RT_MS(15),               /* intervallo MINIMO fra due sveglie (i frame arrivano fra ~15 e ~25 ms) */
		.job = job_guida,
	};
	if (rt_task_avvia(&guida, &cfg, rt_adesso())) {
		drv_ferma();
		return 1;
	}
	drv_sveglia_ai_sensori(&guida);            /* il driver lo sveglia a ogni frame dei sensori */
	drv_sorveglia(&guida, 1, RT_MS(100));      /* critico: se si blocca per 100 ms, motori fermi e stato di guasto */

	/* 3. Il main non è un task: qui si può stampare. Una riga al secondo finché non arriva Ctrl-C. */
	while (!fine) {
		sleep(1);
		drv_stato_t st;
		drv_stato(&st);
		printf("%s  %.2f V %3d %%  v %+.2f m/s  w %+.2f rad/s  limiti 0x%x  job %llu\n",
		       st.fase == DRV_GUASTO ? st.guasto : "ok", st.volt, st.percentuale, st.v, st.w, st.limiti,
		       (unsigned long long)guida.stats.job);
		fflush(stdout);
	}

	/* 4. Uscita in ordine: prima il driver smette di svegliare e sorvegliare il task, poi si ferma il task. */
	drv_dimentica(&guida);
	rt_task_ferma(&guida);
	drv_ferma();
	rt_stats_stampa(&guida);
	return 0;
}
