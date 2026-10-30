/*
 * rt.c - Implementazione dei task periodici real-time (vedi rt.h).
 */
#define _GNU_SOURCE   /* per CPU_SET e pthread_setaffinity_np */
#include "rt.h"

#include <errno.h>
#include <sched.h>
#include <stdio.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/resource.h>
#include <unistd.h>

int rt_init(void)
{
	struct rlimit l;
	if (geteuid() != 0 && getrlimit(RLIMIT_MEMLOCK, &l) == 0 && l.rlim_cur != RLIM_INFINITY) {
		fprintf(stderr, "rt: non root e memoria bloccabile limitata a %llu KB: blocco solo la memoria di adesso, "
		        "non quella futura (va bene solo sul PC)\n", (unsigned long long)l.rlim_cur / 1024);
		return mlockall(MCL_CURRENT) != 0 ? errno : 0;
	}
	if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0)
		return errno;
	return 0;
}

int rt_mutex_init(pthread_mutex_t *m)
{
	pthread_mutexattr_t a;
	int r = pthread_mutexattr_init(&a);
	if (r == 0)
		r = pthread_mutexattr_setprotocol(&a, PTHREAD_PRIO_INHERIT);
	if (r == 0)
		r = pthread_mutex_init(m, &a);
	pthread_mutexattr_destroy(&a);
	return r;
}

static void stats_azzera(rt_stats_t *s)
{
	memset(s, 0, sizeof(*s));
	s->latenza_min_ns = INT64_MAX;
	s->esec_min_ns = INT64_MAX;
}

static int consenti_senza_rt;

void rt_consenti_senza_privilegi(int si)
{
	consenti_senza_rt = si;
}

static void stats_aggiorna(rt_task_t *t, int64_t rilascio, int64_t inizio, int64_t fine)
{
	rt_stats_t *s = &t->stats;
	int64_t latenza = inizio - rilascio;
	int64_t esecuzione = fine - inizio;
	int64_t risposta = fine - rilascio;

	s->job++;
	if (risposta > t->cfg.deadline_ns) {
		s->deadline_mancate++;
		atomic_fetch_add(&t->mancate_di_fila, 1);
		atomic_fetch_add(&t->mancate_totali, 1);
	} else {
		atomic_store(&t->mancate_di_fila, 0);
	}

	if (latenza < s->latenza_min_ns) s->latenza_min_ns = latenza;
	if (latenza > s->latenza_max_ns) s->latenza_max_ns = latenza;
	s->latenza_somma_ns += latenza;

	if (esecuzione < s->esec_min_ns) s->esec_min_ns = esecuzione;
	if (esecuzione > s->esec_max_ns) s->esec_max_ns = esecuzione;
	s->esec_somma_ns += esecuzione;

	if (risposta > s->risposta_max_ns) s->risposta_max_ns = risposta;

	if (t->cfg.campioni && s->n_campioni < t->cfg.max_campioni) {
		t->cfg.campioni[s->n_campioni].latenza_ns = latenza;
		t->cfg.campioni[s->n_campioni].esecuzione_ns = esecuzione;
		s->n_campioni++;
	}
}

/* Fissa il thread chiamante al core indicato. */
static int fissa_al_core(int cpu)
{
	cpu_set_t set;
	CPU_ZERO(&set);
	CPU_SET(cpu, &set);
	return pthread_setaffinity_np(pthread_self(), sizeof(set), &set);
}

/* Il corpo di ogni task: configura il thread, poi esegue il ciclo periodico. */
static void *ciclo_task(void *arg)
{
	rt_task_t *t = arg;

	/* L'affinità si imposta dall'interno del thread, prima del primo job:
	 * musl non ha pthread_attr_setaffinity_np. Il thread nasce già SCHED_FIFO,
	 * quindi da qui in poi non può essere interrotto da task meno prioritari. */
	t->errore_avvio = (t->cfg.cpu == RT_CPU_QUALSIASI) ? 0 : fissa_al_core(t->cfg.cpu);
	sem_post(&t->avviato);
	if (t->errore_avvio != 0)
		return NULL;

	int64_t rilascio = t->primo_rilascio_ns;
	while (!atomic_load(&t->stop)) {
		if (t->cfg.sporadico) {
			/* aspetta la sveglia (rt_task_sveglia); il rilascio è l'istante in cui è arrivata */
			while (sem_wait(&t->sveglia) != 0)
				;
			if (atomic_load(&t->stop))
				break;
			rilascio = atomic_exchange(&t->rilascio_pendente_ns, 0);
			if (rilascio == 0)
				rilascio = rt_adesso();
		} else {
			rt_dormi_fino_a(rilascio);
		}
		int64_t inizio = rt_adesso();
		atomic_store(&t->job_iniziato_ns, inizio);
		t->cfg.job(t->cfg.arg);
		int64_t fine = rt_adesso();
		atomic_store(&t->job_iniziato_ns, 0);
		atomic_store(&t->battito_ns, fine);
		stats_aggiorna(t, rilascio, inizio, fine);

		if (t->cfg.max_job && t->stats.job >= t->cfg.max_job)
			break;
		if (t->cfg.sporadico)
			continue;
		/* Prossimo rilascio: sempre periodo dopo il precedente, non dopo
		 * "adesso". Se un job sfora il periodo, il successivo parte subito
		 * (come vTaskDelayUntil): i ritardi non si accumulano in deriva. */
		rilascio += t->cfg.periodo_ns;
	}
	return NULL;
}

int rt_task_avvia(rt_task_t *t, const rt_task_cfg_t *cfg, int64_t istante_zero_ns)
{
	if (!cfg->job || cfg->periodo_ns <= 0 || cfg->deadline_ns < 0 || cfg->fase_ns < 0 ||
	    cfg->priorita < RT_PRIO_MIN || cfg->priorita > RT_PRIO_MAX ||
	    (cfg->cpu != RT_CPU_QUALSIASI && (cfg->cpu < 0 || cfg->cpu >= CPU_SETSIZE)))
		return EINVAL;

	memset(t, 0, sizeof(*t));
	t->cfg = *cfg;
	if (t->cfg.deadline_ns == 0)
		t->cfg.deadline_ns = t->cfg.periodo_ns;
	t->primo_rilascio_ns = istante_zero_ns + t->cfg.fase_ns;
	atomic_init(&t->stop, 0);
	atomic_init(&t->rilascio_pendente_ns, 0);
	atomic_init(&t->attivazioni_perse, 0);
	atomic_init(&t->battito_ns, t->primo_rilascio_ns);
	atomic_init(&t->job_iniziato_ns, 0);
	atomic_init(&t->mancate_di_fila, 0);
	atomic_init(&t->mancate_totali, 0);
	stats_azzera(&t->stats);
	sem_init(&t->avviato, 0, 0);
	sem_init(&t->sveglia, 0, 0);

	/* Politica e priorità vanno dichiarate negli attributi con
	 * PTHREAD_EXPLICIT_SCHED: altrimenti il thread le eredita dal creatore
	 * (di solito SCHED_OTHER) e quelle negli attributi vengono ignorate. */
	pthread_attr_t a;
	struct sched_param p = { .sched_priority = cfg->priorita };
	int r = pthread_attr_init(&a);
	if (r == 0) r = pthread_attr_setinheritsched(&a, PTHREAD_EXPLICIT_SCHED);
	if (r == 0) r = pthread_attr_setschedpolicy(&a, SCHED_FIFO);
	if (r == 0) r = pthread_attr_setschedparam(&a, &p);
	if (r == 0) r = pthread_attr_setstacksize(&a, cfg->stack_byte ? cfg->stack_byte : RT_STACK_DEFAULT);
	if (r == 0) r = pthread_create(&t->thread, &a, ciclo_task, t);
	if (r == EPERM && consenti_senza_rt) {
		/* niente SCHED_FIFO (PC senza permessi): thread normale, solo per provare la logica */
		pthread_attr_setinheritsched(&a, PTHREAD_INHERIT_SCHED);
		r = pthread_create(&t->thread, &a, ciclo_task, t);
		if (r == 0) {
			static int detto;
			t->senza_rt = 1;
			if (!detto++)
				fprintf(stderr, "rt: niente permessi per SCHED_FIFO, i task girano con lo scheduling normale "
				                "(va bene solo per provare la logica, i tempi non valgono)\n");
		}
	}
	pthread_attr_destroy(&a);
	if (r != 0) {
		sem_destroy(&t->avviato);
		sem_destroy(&t->sveglia);
		return r;
	}

	/* Aspetta che il thread abbia impostato l'affinità, così un errore
	 * (es. core inesistente) viene riportato subito al chiamante. */
	sem_wait(&t->avviato);
	sem_destroy(&t->avviato);
	if (t->errore_avvio != 0) {
		pthread_join(t->thread, NULL);
		sem_destroy(&t->sveglia);
		return t->errore_avvio;
	}
	t->attivo = 1;
	return 0;
}

void rt_task_attendi(rt_task_t *t)
{
	if (!t->attivo)
		return;
	pthread_join(t->thread, NULL);
	sem_destroy(&t->sveglia);
	t->attivo = 0;
}

void rt_task_ferma(rt_task_t *t)
{
	atomic_store(&t->stop, 1);
	if (t->attivo && t->cfg.sporadico)
		sem_post(&t->sveglia);
	rt_task_attendi(t);
}

int rt_task_sveglia(rt_task_t *t, int64_t rilascio_ns)
{
	int64_t zero = 0;
	if (!atomic_compare_exchange_strong(&t->rilascio_pendente_ns, &zero, rilascio_ns ? rilascio_ns : 1)) {
		atomic_fetch_add(&t->attivazioni_perse, 1);
		return 0;
	}
	sem_post(&t->sveglia);
	return 1;
}

int rt_task_bloccato(rt_task_t *t, int64_t adesso, int64_t soglia_ns)
{
	int64_t iniziato = atomic_load(&t->job_iniziato_ns);
	if (iniziato && adesso - iniziato > soglia_ns)
		return 1;
	if (t->cfg.sporadico) {
		int64_t pendente = atomic_load(&t->rilascio_pendente_ns);
		return pendente && adesso - pendente > soglia_ns;
	}
	return adesso - atomic_load(&t->battito_ns) > t->cfg.periodo_ns + soglia_ns;
}

int rt_priorita_rm(rt_task_cfg_t *cfg[], int n, int prio_max)
{
	/* ordina per periodo crescente (insertion sort su una copia dei puntatori: n è piccolo) */
	rt_task_cfg_t *ord[64];
	if (n < 0 || n > 64)
		return EINVAL;
	for (int i = 0; i < n; i++) {
		int j = i;
		while (j > 0 && ord[j - 1]->periodo_ns > cfg[i]->periodo_ns) {
			ord[j] = ord[j - 1];
			j--;
		}
		ord[j] = cfg[i];
	}
	int prio = prio_max;
	for (int i = 0; i < n; i++) {
		if (i > 0 && ord[i]->periodo_ns != ord[i - 1]->periodo_ns)
			prio--;
		if (prio < RT_PRIO_MIN)
			return EINVAL;
		ord[i]->priorita = prio;
	}
	return 0;
}

void rt_stats_stampa(const rt_task_t *t)
{
	const rt_stats_t *s = &t->stats;
	if (s->job == 0) {
		printf("%s: nessun job eseguito\n", t->cfg.nome ? t->cfg.nome : "task");
		return;
	}
	printf("%s: prio %d, cpu %d, periodo %.1f us, deadline %.1f us\n",
	       t->cfg.nome ? t->cfg.nome : "task", t->cfg.priorita, t->cfg.cpu,
	       t->cfg.periodo_ns / 1e3, t->cfg.deadline_ns / 1e3);
	printf("  job %llu, deadline mancate %llu\n",
	       (unsigned long long)s->job, (unsigned long long)s->deadline_mancate);
	printf("  latenza    min %8.1f  media %8.1f  max %8.1f us\n",
	       s->latenza_min_ns / 1e3, (double)s->latenza_somma_ns / s->job / 1e3,
	       s->latenza_max_ns / 1e3);
	printf("  esecuzione min %8.1f  media %8.1f  max %8.1f us\n",
	       s->esec_min_ns / 1e3, (double)s->esec_somma_ns / s->job / 1e3,
	       s->esec_max_ns / 1e3);
	printf("  risposta max %.1f us\n", s->risposta_max_ns / 1e3);
}
