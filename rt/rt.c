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

static void stats_reset(rt_stats_t *s)
{
	memset(s, 0, sizeof(*s));
	s->latency_min_ns = INT64_MAX;
	s->exec_min_ns = INT64_MAX;
}

static int allow_without_rt;

void rt_allow_without_privileges(int enable)
{
	allow_without_rt = enable;
}

static int64_t thread_cpu_ns(void)
{
	struct timespec ts;
	clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ts);
	return ts.tv_sec * RT_NS_PER_S + ts.tv_nsec;
}

static void stats_update(rt_task_t *t, int64_t release, int64_t start, int64_t end, int64_t cpu)
{
	rt_stats_t *s = &t->stats;
	int64_t latency = start - release;
	int64_t execution = end - start;
	int64_t response = end - release;

	s->job++;
	if (response > t->cfg.deadline_ns) {
		s->missed_deadlines++;
		if (s->missed_deadlines == 1) {
			t->first_miss.job = s->job;
			t->first_miss.latency_ns = latency;
			t->first_miss.execution_ns = execution;
			t->first_miss.cpu_ns = cpu;
			t->first_miss.response_ns = response;
		}
		atomic_fetch_add(&t->consecutive_misses, 1);
		atomic_fetch_add(&t->total_misses, 1);
	} else {
		atomic_store(&t->consecutive_misses, 0);
	}

	if (latency < s->latency_min_ns) s->latency_min_ns = latency;
	if (latency > s->latency_max_ns) s->latency_max_ns = latency;
	s->latency_sum_ns += latency;

	if (execution < s->exec_min_ns) s->exec_min_ns = execution;
	if (execution > s->exec_max_ns) s->exec_max_ns = execution;
	s->exec_sum_ns += execution;

	if (response > s->response_max_ns) s->response_max_ns = response;
	if (cpu > s->cpu_max_ns) s->cpu_max_ns = cpu;

	if (t->cfg.samples && s->n_samples < t->cfg.max_samples) {
		t->cfg.samples[s->n_samples].latency_ns = latency;
		t->cfg.samples[s->n_samples].execution_ns = execution;
		s->n_samples++;
	}
}

/* Fissa il thread chiamante al core indicato. */
static int pin_to_core(int cpu)
{
	cpu_set_t set;
	CPU_ZERO(&set);
	CPU_SET(cpu, &set);
	return pthread_setaffinity_np(pthread_self(), sizeof(set), &set);
}

/* Il corpo di ogni task: configura il thread, poi esegue il ciclo periodico. */
static void *task_loop(void *arg)
{
	rt_task_t *t = arg;

	/* L'affinità si imposta dall'interno del thread, prima del primo job:
	 * musl non ha pthread_attr_setaffinity_np. Il thread nasce già SCHED_FIFO,
	 * quindi da qui in poi non può essere interrotto da task meno prioritari. */
	t->start_error = (t->cfg.cpu == RT_CPU_ANY) ? 0 : pin_to_core(t->cfg.cpu);
	sem_post(&t->started);
	if (t->start_error != 0)
		return NULL;

	int64_t release = t->first_release_ns;
	while (!atomic_load(&t->stop)) {
		if (t->cfg.sporadic) {
			/* aspetta la sveglia (rt_task_wake); il rilascio è l'istante in cui è arrivata */
			while (sem_wait(&t->wakeup) != 0)
				;
			if (atomic_load(&t->stop))
				break;
			release = atomic_exchange(&t->pending_release_ns, 0);
			if (release == 0)
				release = rt_now();
		} else {
			rt_sleep_until(release);
		}
		int64_t start = rt_now();
		atomic_store(&t->job_started_ns, start);
		int64_t cpu_start = thread_cpu_ns();
		t->cfg.job(t->cfg.arg);
		int64_t cpu = thread_cpu_ns() - cpu_start;
		int64_t end = rt_now();
		atomic_store(&t->job_started_ns, 0);
		atomic_store(&t->heartbeat_ns, end);
		stats_update(t, release, start, end, cpu);

		if (t->cfg.max_job && t->stats.job >= t->cfg.max_job)
			break;
		if (t->cfg.sporadic)
			continue;
		/* Prossimo rilascio: sempre periodo dopo il precedente, non dopo
		 * "adesso". Se un job sfora il periodo, il successivo parte subito
		 * (come vTaskDelayUntil): i ritardi non si accumulano in deriva. */
		release += t->cfg.period_ns;
	}
	return NULL;
}

int rt_task_start(rt_task_t *t, const rt_task_cfg_t *cfg, int64_t zero_time_ns)
{
	if (!cfg->job || cfg->period_ns <= 0 || cfg->deadline_ns < 0 || cfg->phase_ns < 0 ||
	    cfg->priority < RT_PRIO_MIN || cfg->priority > RT_PRIO_MAX ||
	    (cfg->cpu != RT_CPU_ANY && (cfg->cpu < 0 || cfg->cpu >= CPU_SETSIZE)))
		return EINVAL;

	memset(t, 0, sizeof(*t));
	t->cfg = *cfg;
	if (t->cfg.deadline_ns == 0)
		t->cfg.deadline_ns = t->cfg.period_ns;
	t->first_release_ns = zero_time_ns + t->cfg.phase_ns;
	atomic_init(&t->stop, 0);
	atomic_init(&t->pending_release_ns, 0);
	atomic_init(&t->lost_activations, 0);
	atomic_init(&t->heartbeat_ns, t->first_release_ns);
	atomic_init(&t->job_started_ns, 0);
	atomic_init(&t->consecutive_misses, 0);
	atomic_init(&t->total_misses, 0);
	stats_reset(&t->stats);
	sem_init(&t->started, 0, 0);
	sem_init(&t->wakeup, 0, 0);

	/* Politica e priorità vanno dichiarate negli attributi con
	 * PTHREAD_EXPLICIT_SCHED: altrimenti il thread le eredita dal creatore
	 * (di solito SCHED_OTHER) e quelle negli attributi vengono ignorate. */
	pthread_attr_t a;
	struct sched_param p = { .sched_priority = cfg->priority };
	int r = pthread_attr_init(&a);
	if (r == 0) r = pthread_attr_setinheritsched(&a, PTHREAD_EXPLICIT_SCHED);
	if (r == 0) r = pthread_attr_setschedpolicy(&a, SCHED_FIFO);
	if (r == 0) r = pthread_attr_setschedparam(&a, &p);
	if (r == 0) r = pthread_attr_setstacksize(&a, cfg->stack_bytes ? cfg->stack_bytes : RT_STACK_DEFAULT);
	if (r == 0) r = pthread_create(&t->thread, &a, task_loop, t);
	if (r == EPERM && allow_without_rt) {
		/* niente SCHED_FIFO (PC senza permessi): thread normale, solo per provare la logica */
		pthread_attr_setinheritsched(&a, PTHREAD_INHERIT_SCHED);
		r = pthread_create(&t->thread, &a, task_loop, t);
		if (r == 0) {
			static int told;
			t->without_rt = 1;
			if (!told++)
				fprintf(stderr, "rt: niente permessi per SCHED_FIFO, i task girano con lo scheduling normale "
				                "(va bene solo per provare la logica, i tempi non valgono)\n");
		}
	}
	pthread_attr_destroy(&a);
	if (r != 0) {
		sem_destroy(&t->started);
		sem_destroy(&t->wakeup);
		return r;
	}

	/* Aspetta che il thread abbia impostato l'affinità, così un errore
	 * (es. core inesistente) viene riportato subito al chiamante. */
	sem_wait(&t->started);
	sem_destroy(&t->started);
	if (t->start_error != 0) {
		pthread_join(t->thread, NULL);
		sem_destroy(&t->wakeup);
		return t->start_error;
	}
	t->active = 1;
	return 0;
}

void rt_task_join(rt_task_t *t)
{
	if (!t->active)
		return;
	pthread_join(t->thread, NULL);
	sem_destroy(&t->wakeup);
	t->active = 0;
}

void rt_task_stop(rt_task_t *t)
{
	atomic_store(&t->stop, 1);
	if (t->active && t->cfg.sporadic)
		sem_post(&t->wakeup);
	rt_task_join(t);
}

int rt_task_wake(rt_task_t *t, int64_t release_ns)
{
	int64_t zero = 0;
	if (!atomic_compare_exchange_strong(&t->pending_release_ns, &zero, release_ns ? release_ns : 1)) {
		atomic_fetch_add(&t->lost_activations, 1);
		return 0;
	}
	sem_post(&t->wakeup);
	return 1;
}

int rt_task_is_stuck(rt_task_t *t, int64_t now, int64_t threshold_ns)
{
	int64_t started_ns = atomic_load(&t->job_started_ns);
	if (started_ns && now - started_ns > threshold_ns)
		return 1;
	if (t->cfg.sporadic) {
		int64_t pending_ns = atomic_load(&t->pending_release_ns);
		return pending_ns && now - pending_ns > threshold_ns;
	}
	return now - atomic_load(&t->heartbeat_ns) > t->cfg.period_ns + threshold_ns;
}

int rt_priority_rm(rt_task_cfg_t *cfg[], int n, int prio_max)
{
	/* ordina per periodo crescente (insertion sort su una copia dei puntatori: n è piccolo) */
	rt_task_cfg_t *sorted[64];
	if (n < 0 || n > 64)
		return EINVAL;
	for (int i = 0; i < n; i++) {
		int j = i;
		while (j > 0 && sorted[j - 1]->period_ns > cfg[i]->period_ns) {
			sorted[j] = sorted[j - 1];
			j--;
		}
		sorted[j] = cfg[i];
	}
	int prio = prio_max;
	for (int i = 0; i < n; i++) {
		if (i > 0 && sorted[i]->period_ns != sorted[i - 1]->period_ns)
			prio--;
		if (prio < RT_PRIO_MIN)
			return EINVAL;
		sorted[i]->priority = prio;
	}
	return 0;
}

void rt_stats_print(const rt_task_t *t)
{
	const rt_stats_t *s = &t->stats;
	if (s->job == 0) {
		printf("%s: nessun job eseguito\n", t->cfg.name ? t->cfg.name : "task");
		return;
	}
	printf("%s: prio %d, cpu %d, periodo %.1f us, deadline %.1f us\n",
	       t->cfg.name ? t->cfg.name : "task", t->cfg.priority, t->cfg.cpu,
	       t->cfg.period_ns / 1e3, t->cfg.deadline_ns / 1e3);
	printf("  job %llu, deadline mancate %llu\n",
	       (unsigned long long)s->job, (unsigned long long)s->missed_deadlines);
	printf("  latenza    min %8.1f  media %8.1f  max %8.1f us\n",
	       s->latency_min_ns / 1e3, (double)s->latency_sum_ns / s->job / 1e3,
	       s->latency_max_ns / 1e3);
	printf("  esecuzione min %8.1f  media %8.1f  max %8.1f us\n",
	       s->exec_min_ns / 1e3, (double)s->exec_sum_ns / s->job / 1e3,
	       s->exec_max_ns / 1e3);
	printf("  risposta max %.1f us, CPU max di un job %.1f us\n", s->response_max_ns / 1e3, s->cpu_max_ns / 1e3);
}

/* ------------------------------------------------------------------------------------------ task aperiodico --- */

/* Il corpo del task aperiodico: aspetta una sveglia, esegue il job, misura. */
static void *aperiodic_loop(void *arg)
{
	rt_aperiodic_t *a = arg;

	a->start_error = (a->cfg.cpu == RT_CPU_ANY) ? 0 : pin_to_core(a->cfg.cpu);
	sem_post(&a->started);
	if (a->start_error != 0)
		return NULL;

	while (!atomic_load(&a->stop)) {
		while (sem_wait(&a->wakeup) != 0)
			;
		if (atomic_load(&a->stop))
			break;
		/* azzerare la richiesta PRIMA del job: una sveglia che arriva durante il job ne fa partire un altro dopo */
		int64_t request = atomic_exchange(&a->request_ns, 0);
		if (request == 0)
			continue;
		int64_t start = rt_now();
		int64_t cpu_start = thread_cpu_ns();
		a->cfg.job(a->cfg.arg);
		int64_t cpu = thread_cpu_ns() - cpu_start;
		int64_t end = rt_now();

		rt_aperiodic_stats_t *s = &a->stats;
		s->job++;
		s->exec_sum_ns += end - start;
		if (end - start > s->exec_max_ns) s->exec_max_ns = end - start;
		if (cpu > s->cpu_max_ns) s->cpu_max_ns = cpu;
		if (end - request > s->response_max_ns) s->response_max_ns = end - request;
	}
	return NULL;
}

int rt_aperiodic_start(rt_aperiodic_t *a, const rt_aperiodic_cfg_t *cfg)
{
	if (!cfg->job || (cfg->cpu != RT_CPU_ANY && (cfg->cpu < 0 || cfg->cpu >= CPU_SETSIZE)))
		return EINVAL;

	memset(a, 0, sizeof(*a));
	a->cfg = *cfg;
	atomic_init(&a->stop, 0);
	atomic_init(&a->request_ns, 0);
	atomic_init(&a->wakeups, 0);
	sem_init(&a->started, 0, 0);
	sem_init(&a->wakeup, 0, 0);

	/* scheduling normale dichiarato esplicitamente: chi lo crea (il main) potrebbe essere SCHED_FIFO */
	pthread_attr_t at;
	struct sched_param p = { .sched_priority = 0 };
	int r = pthread_attr_init(&at);
	if (r == 0) r = pthread_attr_setinheritsched(&at, PTHREAD_EXPLICIT_SCHED);
	if (r == 0) r = pthread_attr_setschedpolicy(&at, SCHED_OTHER);
	if (r == 0) r = pthread_attr_setschedparam(&at, &p);
	if (r == 0) r = pthread_attr_setstacksize(&at, cfg->stack_bytes ? cfg->stack_bytes : RT_STACK_DEFAULT);
	if (r == 0) r = pthread_create(&a->thread, &at, aperiodic_loop, a);
	pthread_attr_destroy(&at);
	if (r != 0) {
		sem_destroy(&a->started);
		sem_destroy(&a->wakeup);
		return r;
	}

	sem_wait(&a->started);
	sem_destroy(&a->started);
	if (a->start_error != 0) {
		pthread_join(a->thread, NULL);
		sem_destroy(&a->wakeup);
		return a->start_error;
	}
	a->active = 1;
	return 0;
}

void rt_aperiodic_wake(rt_aperiodic_t *a)
{
	atomic_fetch_add(&a->wakeups, 1);
	/* solo la prima sveglia non servita fa il sem_post: le altre si uniscono a lei */
	int64_t zero = 0;
	if (atomic_compare_exchange_strong(&a->request_ns, &zero, rt_now()))
		sem_post(&a->wakeup);
}

void rt_aperiodic_stop(rt_aperiodic_t *a)
{
	if (!a->active)
		return;
	atomic_store(&a->stop, 1);
	sem_post(&a->wakeup);
	pthread_join(a->thread, NULL);
	sem_destroy(&a->wakeup);
	a->stats.wakeups = atomic_load(&a->wakeups);
	a->active = 0;
}

void rt_aperiodic_print(const rt_aperiodic_t *a)
{
	const rt_aperiodic_stats_t *s = &a->stats;
	printf("%s: aperiodico, cpu %d, scheduling normale\n", a->cfg.name ? a->cfg.name : "task", a->cfg.cpu);
	if (s->job == 0) {
		printf("  nessun job eseguito (sveglie %llu)\n", (unsigned long long)s->wakeups);
		return;
	}
	printf("  job %llu, sveglie %llu\n", (unsigned long long)s->job, (unsigned long long)s->wakeups);
	printf("  esecuzione media %.1f  max %.1f us, CPU max di un job %.1f us\n",
	       (double)s->exec_sum_ns / s->job / 1e3, s->exec_max_ns / 1e3, s->cpu_max_ns / 1e3);
	printf("  risposta max %.1f us (dalla sveglia alla fine del job)\n", s->response_max_ns / 1e3);
}
