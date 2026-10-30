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

/* Convenzione del progetto (docs/PIANO.md): il driver sta a 99, fuori dallo scheduling dei task; i task usano le
 * priorità da RT_PRIO_TASK_MAX in giù (rt_priorita_rm le assegna con Rate Monotonic). */
#define RT_PRIO_DRIVER   99
#define RT_PRIO_TASK_MAX 90

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
	/*
	 * Task periodico (sporadico = 0): un job ogni periodo_ns, a istanti fissi.
	 * Task sporadico (sporadico = 1): un job per ogni rt_task_sveglia() (es. a ogni frame dei sensori, vedi
	 * drv_sveglia_ai_sensori). periodo_ns è allora l'intervallo MINIMO fra due attivazioni: è il T da usare
	 * nell'analisi e nell'assegnazione delle priorità; il rilascio di ogni job è l'istante della sveglia.
	 */
	int sporadico;
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
	int senza_rt;       /* partito senza SCHED_FIFO (solo sul PC, vedi rt_consenti_senza_privilegi) */
	rt_stats_t stats;

	/* Task sporadici: la sveglia e l'istante dell'attivazione non ancora servita (0 = nessuna). */
	sem_t sveglia;
	_Atomic int64_t rilascio_pendente_ns;
	_Atomic uint64_t attivazioni_perse;     /* sveglie arrivate mentre la precedente non era ancora partita */

	/* Per il watchdog del driver (letti da un altro thread mentre il task gira, quindi atomici). */
	_Atomic int64_t battito_ns;             /* fine dell'ultimo job (all'avvio: il primo rilascio) */
	_Atomic int64_t job_iniziato_ns;        /* inizio del job in corso, 0 se il task aspetta */
	_Atomic uint64_t mancate_di_fila;       /* deadline mancate consecutive */
	_Atomic uint64_t mancate_totali;
} rt_task_t;

/*
 * Da chiamare una volta, all'inizio del main, prima di creare i task.
 * Blocca in RAM tutta la memoria del processo, presente e futura
 * (mlockall): un task RT non deve mai fermarsi per un page fault, che
 * costerebbe da microsecondi a millisecondi. In un RTOS la memoria è
 * sempre in RAM. Ritorna 0 oppure un codice errno.
 * Eccezione per il PC: se il processo non è root e la memoria bloccabile ha
 * un limite (sul PC 8 MB per i normali utenti), blocca solo la memoria di
 * adesso. Con anche quella futura gli stack dei thread creati dopo si
 * sommerebbero fino a passare il limite, e pthread_create fallirebbe. Sul
 * PC senza root manca anche SCHED_FIFO, quindi i tempi non valgono comunque.
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
 * Attiva un task sporadico: il suo prossimo job viene rilasciato all'istante `rilascio_ns` (di solito adesso).
 * Non si blocca mai (si può chiamare dal driver). Se l'attivazione precedente non è ancora partita, questa va
 * persa: si conta in attivazioni_perse e la funzione ritorna 0; altrimenti ritorna 1.
 */
int rt_task_sveglia(rt_task_t *t, int64_t rilascio_ns);

/*
 * Per il watchdog: 1 se il task sembra bloccato all'istante `adesso`, cioè se da più di `soglia_ns`
 *   - un suo job è iniziato e non è ancora finito, oppure
 *   - (sporadico) un'attivazione aspetta e il job non è partito, oppure
 *   - (periodico) non finisce un job (oltre al suo periodo).
 */
int rt_task_bloccato(rt_task_t *t, int64_t adesso, int64_t soglia_ns);

/*
 * Assegna le priorità con Rate Monotonic: periodo (o intervallo minimo) più corto = priorità più alta.
 * Il task più frequente riceve prio_max, gli altri scendono di 1 a ogni periodo diverso; periodi uguali hanno la
 * stessa priorità. Ritorna 0, oppure EINVAL se le priorità scenderebbero sotto RT_PRIO_MIN.
 * L'analisi di schedulabilità (fattore di utilizzo, tempi di risposta) resta a chi scrive i task.
 */
int rt_priorita_rm(rt_task_cfg_t *cfg[], int n, int prio_max);

/*
 * Sul PC, senza i permessi per SCHED_FIFO, rt_task_avvia fallirebbe con EPERM. Con si = 1 invece fa partire il
 * thread con lo scheduling normale (t->senza_rt = 1) e lo scrive una volta su stderr. Utile solo per provare la
 * LOGICA nel simulatore: i tempi non valgono niente. Sul robot non serve (si gira da root).
 */
void rt_consenti_senza_privilegi(int si);

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
