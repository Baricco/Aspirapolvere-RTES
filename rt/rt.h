/*
 * rt.h - Task periodici real-time su Linux (SCHED_FIFO), in stile FreeRTOS.
 *
 * Modello (quello classico della teoria RT):
 *   - un task periodico rilascia un "job" ogni `periodo_ns`;
 *   - il k-esimo job è rilasciato all'istante  istante_zero + fase + k*periodo;
 *   - deve terminare entro  rilascio + deadline  (deadline relativa).
 *
 * Il programmatore scrive solo la funzione `job`, cioè il lavoro da fare in un
 * periodo. Il ciclo "aspetta il rilascio -> esegui il job -> misura" lo gestisce
 * questo modulo. In FreeRTOS lo stesso schema si scrive a mano:
 *
 *     for (;;) { lavoro(); vTaskDelayUntil(&ultimo_risveglio, periodo); }
 *
 * Ogni task è un thread POSIX con:
 *   - politica SCHED_FIFO e priorità fissa 1..99 (99 = la più alta).
 *     Come in FreeRTOS, gira sempre il task pronto con priorità più alta, e
 *     a parità di priorità chi è arrivato prima;
 *   - affinità a un solo core (scheduling partizionato, vedi docs/PIANO.md).
 *
 * Uso tipico:
 *
 *     rt_init();                                  // una volta, all'avvio
 *     int64_t t0 = rt_adesso() + RT_MS(100);      // istante zero comune
 *     rt_task_cfg_t cfg = { .nome = "controllo", .priorita = 80, .cpu = 2,
 *                           .periodo_ns = RT_MS(20), .job = controllo_job };
 *     rt_task_t controllo;
 *     rt_task_avvia(&controllo, &cfg, t0);
 *     ...
 *     rt_task_ferma(&controllo);
 *     rt_stats_stampa(&controllo);
 */
#ifndef RT_H
#define RT_H

#include <pthread.h>
#include <semaphore.h>
#include <stdatomic.h>
#include <stddef.h>
#include <stdint.h>

#include "rt_tempo.h"

#define RT_PRIO_MIN 1
#define RT_PRIO_MAX 99
#define RT_CPU_QUALSIASI (-1)

/* Stack di ogni task, come lo usStackDepth di xTaskCreate in FreeRTOS.
 * Va dichiarato esplicitamente: il default di glibc è 8 MB per thread e,
 * con mlockall, ogni byte di stack resta bloccato in RAM. */
#define RT_STACK_DEFAULT (256 * 1024)

/* Una misura per ogni job (facoltativa, per i grafici e il calcolo del WCET). */
typedef struct {
	int64_t latenza_ns;     /* inizio effettivo - rilascio teorico */
	int64_t esecuzione_ns;  /* fine - inizio: quanto è durato il job */
} rt_campione_t;

typedef struct {
	const char *nome;
	int priorita;            /* SCHED_FIFO, RT_PRIO_MIN..RT_PRIO_MAX */
	int cpu;                 /* core a cui fissare il task, o RT_CPU_QUALSIASI */
	int64_t periodo_ns;      /* > 0 */
	int64_t deadline_ns;     /* deadline relativa; 0 = uguale al periodo */
	int64_t fase_ns;         /* ritardo del primo rilascio rispetto all'istante zero */
	void (*job)(void *arg);  /* il lavoro di un periodo */
	void *arg;               /* passato a job() */
	uint64_t max_job;        /* si ferma da solo dopo questi job; 0 = mai */
	size_t stack_byte;       /* dimensione dello stack; 0 = RT_STACK_DEFAULT */

	/* Facoltativo: buffer preallocato in cui salvare le misure di ogni job.
	 * Va allocato prima di avviare il task: nel ciclo RT non si alloca memoria. */
	rt_campione_t *campioni;
	size_t max_campioni;
} rt_task_cfg_t;

/* Statistiche aggiornate a ogni job. Vanno lette dopo rt_task_ferma():
 * mentre il task gira i valori potrebbero essere a metà di un aggiornamento. */
typedef struct {
	uint64_t job;               /* job completati */
	uint64_t deadline_mancate;  /* job finiti dopo rilascio + deadline */
	int64_t latenza_min_ns, latenza_max_ns, latenza_somma_ns;
	int64_t esec_min_ns, esec_max_ns, esec_somma_ns;
	int64_t risposta_max_ns;    /* max(fine - rilascio): il tempo di risposta peggiore osservato */
	size_t n_campioni;          /* campioni effettivamente salvati */
} rt_stats_t;

typedef struct {
	rt_task_cfg_t cfg;
	int64_t primo_rilascio_ns;
	pthread_t thread;
	atomic_int stop;
	sem_t avviato;      /* il thread segnala qui l'esito della sua configurazione */
	int errore_avvio;   /* 0 oppure un codice errno */
	int attivo;
	rt_stats_t stats;
} rt_task_t;

/*
 * Da chiamare una volta, all'inizio del main, prima di creare i task.
 * Blocca in RAM tutta la memoria del processo, presente e futura
 * (mlockall): un task RT non deve mai fermarsi per un page fault, che
 * costerebbe da microsecondi a millisecondi. In un RTOS la memoria è
 * sempre in RAM. Ritorna 0 oppure un codice errno.
 */
int rt_init(void);

/*
 * Crea il thread del task e lo avvia. Il primo job viene rilasciato a
 * istante_zero + cfg->fase_ns. Usare lo stesso istante_zero per tutti i task
 * li fa partire insieme (l'"istante critico" dell'analisi rate-monotonic).
 * Ritorna 0 oppure un codice errno (EPERM: servono i permessi di root o un
 * limite rtprio sufficiente; EINVAL: configurazione non valida).
 */
int rt_task_avvia(rt_task_t *t, const rt_task_cfg_t *cfg, int64_t istante_zero_ns);

/* Chiede al task di fermarsi dopo il job corrente e aspetta che finisca. */
void rt_task_ferma(rt_task_t *t);

/* Aspetta che il task finisca da solo (serve cfg.max_job > 0). */
void rt_task_attendi(rt_task_t *t);

/* Stampa le statistiche del task su stdout, in microsecondi. */
void rt_stats_stampa(const rt_task_t *t);

/*
 * Inizializza un mutex con priority inheritance: se un task a bassa
 * priorità tiene il mutex e uno ad alta priorità lo aspetta, il primo eredita
 * temporaneamente la priorità del secondo. Evita l'inversione di priorità
 * non limitata (il caso del Mars Pathfinder). È lo stesso comportamento dei
 * mutex di FreeRTOS. Il "priority ceiling" (PTHREAD_PRIO_PROTECT) non è
 * supportato da musl. Ritorna 0 oppure un codice errno.
 */
int rt_mutex_init(pthread_mutex_t *m);

#endif /* RT_H */
