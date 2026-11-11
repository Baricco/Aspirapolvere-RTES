/*
 * Test dei task periodici (rt/rt.c).
 *
 * Servono i permessi per SCHED_FIFO: root, oppure un limite rtprio > 0
 * (`ulimit -r`). Senza permessi i test che creano task vengono saltati e
 * segnalati, non falliscono: dipende dalla macchina, non dal codice.
 */
#define _GNU_SOURCE   /* per sched_getcpu */
#include <errno.h>
#include <sched.h>
#include <stdatomic.h>
#include <string.h>

#include "test.h"
#include "../rt/rt.h"

static int contatore;
static int politica_vista, priorita_vista, cpu_vista;

static void job_conta(void *arg)
{
	(void)arg;
	contatore++;
}

static void job_osserva(void *arg)
{
	(void)arg;
	struct sched_param p;
	pthread_getschedparam(pthread_self(), &politica_vista, &p);
	priorita_vista = p.sched_priority;
	cpu_vista = sched_getcpu();
}

static void job_lento(void *arg)
{
	(void)arg;
	int64_t fine = rt_now() + RT_MS(3);
	while (rt_now() < fine)
		;
}

static int permessi_rt;

static void config_non_valide(void)
{
	rt_task_t t;
	rt_task_cfg_t base = { .priority = 50, .cpu = RT_CPU_ANY, .period_ns = RT_MS(1), .job = job_conta };
	rt_task_cfg_t c;

	c = base; c.job = NULL;          CONTROLLA(rt_task_start(&t, &c, rt_now()) == EINVAL);
	c = base; c.period_ns = 0;      CONTROLLA(rt_task_start(&t, &c, rt_now()) == EINVAL);
	c = base; c.priority = 0;        CONTROLLA(rt_task_start(&t, &c, rt_now()) == EINVAL);
	c = base; c.priority = 100;      CONTROLLA(rt_task_start(&t, &c, rt_now()) == EINVAL);
	c = base; c.cpu = -5;            CONTROLLA(rt_task_start(&t, &c, rt_now()) == EINVAL);
}

static void esegue_il_numero_giusto_di_job(void)
{
	contatore = 0;
	rt_task_cfg_t c = { .name = "conta", .priority = 50, .cpu = RT_CPU_ANY,
			    .period_ns = RT_MS(2), .job = job_conta, .max_job = 20 };
	rt_task_t t;
	CONTROLLA(rt_task_start(&t, &c, rt_now()) == 0);
	rt_task_join(&t);
	CONTROLLA(contatore == 20);
	CONTROLLA(t.stats.job == 20);
	CONTROLLA(t.stats.missed_deadlines == 0);
	CONTROLLA(t.cfg.deadline_ns == RT_MS(2));   /* deadline 0 => uguale al periodo */
}

static void politica_priorita_e_core(void)
{
	rt_task_cfg_t c = { .name = "osserva", .priority = 42, .cpu = 0,
			    .period_ns = RT_MS(1), .job = job_osserva, .max_job = 3 };
	rt_task_t t;
	CONTROLLA(rt_task_start(&t, &c, rt_now()) == 0);
	rt_task_join(&t);
	CONTROLLA(politica_vista == SCHED_FIFO);
	CONTROLLA(priorita_vista == 42);
	CONTROLLA(cpu_vista == 0);
}

static void periodo_rispettato_senza_deriva(void)
{
	/* 50 job da 2 ms: l'ultimo deve partire ~98 ms dopo il primo,
	 * qualunque sia il ritardo dei singoli risvegli. */
	rt_task_cfg_t c = { .name = "deriva", .priority = 50, .cpu = RT_CPU_ANY,
			    .period_ns = RT_MS(2), .job = job_conta, .max_job = 50 };
	rt_task_t t;
	int64_t t0 = rt_now() + RT_MS(10);
	CONTROLLA(rt_task_start(&t, &c, t0) == 0);
	rt_task_join(&t);
	int64_t durata = rt_now() - t0;
	CONTROLLA(durata >= RT_MS(98));
	CONTROLLA(durata < RT_MS(98) + RT_MS(5));
}

static void conta_le_deadline_mancate(void)
{
	/* job da 3 ms con deadline 1 ms: ogni job manca la deadline */
	rt_task_cfg_t c = { .name = "lento", .priority = 50, .cpu = RT_CPU_ANY,
			    .period_ns = RT_MS(10), .deadline_ns = RT_MS(1), .job = job_lento, .max_job = 5 };
	rt_task_t t;
	CONTROLLA(rt_task_start(&t, &c, rt_now()) == 0);
	rt_task_join(&t);
	CONTROLLA(t.stats.missed_deadlines == 5);
	CONTROLLA(t.stats.exec_min_ns >= RT_MS(3));
}

static void salva_i_campioni(void)
{
	rt_sample_t buf[4];
	rt_task_cfg_t c = { .name = "campioni", .priority = 50, .cpu = RT_CPU_ANY,
			    .period_ns = RT_MS(1), .job = job_conta, .max_job = 10,
			    .samples = buf, .max_samples = 4 };
	rt_task_t t;
	CONTROLLA(rt_task_start(&t, &c, rt_now()) == 0);
	rt_task_join(&t);
	CONTROLLA(t.stats.n_samples == 4);   /* non sfora il buffer */
	CONTROLLA(buf[0].latency_ns >= 0);
}

static void ferma_un_task_infinito(void)
{
	rt_task_cfg_t c = { .name = "infinito", .priority = 50, .cpu = RT_CPU_ANY,
			    .period_ns = RT_MS(1), .job = job_conta };
	rt_task_t t;
	CONTROLLA(rt_task_start(&t, &c, rt_now()) == 0);
	rt_sleep_until(rt_now() + RT_MS(20));
	rt_task_stop(&t);
	CONTROLLA(t.stats.job > 0);
}

static void mutex_priority_inheritance(void)
{
	pthread_mutex_t m;
	CONTROLLA(rt_mutex_init(&m) == 0);
	CONTROLLA(pthread_mutex_lock(&m) == 0);
	CONTROLLA(pthread_mutex_unlock(&m) == 0);
	pthread_mutex_destroy(&m);
}

/* ---- fase 3: task sporadici, watchdog, priorità Rate Monotonic ---- */

static atomic_int blocca_sporadico;

static void job_sporadico(void *arg)
{
	(void)arg;
	contatore++;
	while (atomic_load(&blocca_sporadico))
		rt_sleep_until(rt_now() + RT_MS(1));
}

static void task_sporadici(void)
{
	contatore = 0;
	atomic_store(&blocca_sporadico, 0);
	rt_task_cfg_t c = { .name = "sporadico", .priority = 50, .cpu = RT_CPU_ANY, .sporadic = 1,
	                    .period_ns = RT_MS(20), .job = job_sporadico };
	rt_task_t t;
	CONTROLLA(rt_task_start(&t, &c, rt_now()) == 0);
	rt_sleep_until(rt_now() + RT_MS(20));
	CONTROLLA(contatore == 0);                         /* senza sveglie non gira */
	for (int i = 0; i < 5; i++) {
		CONTROLLA(rt_task_wake(&t, rt_now()) == 1);
		rt_sleep_until(rt_now() + RT_MS(5));
	}
	CONTROLLA(contatore == 5);
	CONTROLLA(t.stats.job == 5 && t.stats.latency_max_ns < RT_MS(5));
	CONTROLLA(!rt_task_is_stuck(&t, rt_now(), RT_MS(10)));

	/* il job si blocca: la prima sveglia durante il job si accoda, la seconda si perde */
	atomic_store(&blocca_sporadico, 1);
	rt_task_wake(&t, rt_now());
	rt_sleep_until(rt_now() + RT_MS(5));         /* il job è partito ed è bloccato */
	CONTROLLA(rt_task_wake(&t, rt_now()) == 1);
	CONTROLLA(rt_task_wake(&t, rt_now()) == 0);
	CONTROLLA(atomic_load(&t.lost_activations) == 1);
	rt_sleep_until(rt_now() + RT_MS(30));
	CONTROLLA(rt_task_is_stuck(&t, rt_now(), RT_MS(20)));        /* il watchdog lo vede */
	atomic_store(&blocca_sporadico, 0);
	rt_sleep_until(rt_now() + RT_MS(10));
	CONTROLLA(contatore == 7);                         /* il job bloccato e quello accodato */
	CONTROLLA(atomic_load(&t.total_misses) >= 1);    /* il job bloccato ha sforato i 20 ms */
	CONTROLLA(!rt_task_is_stuck(&t, rt_now(), RT_MS(20)));
	rt_task_stop(&t);                                 /* deve svegliarlo anche mentre aspetta */
	CONTROLLA(!t.active);
}

static void bloccato_periodico(void)
{
	atomic_store(&blocca_sporadico, 0);
	rt_task_cfg_t c = { .name = "periodico", .priority = 50, .cpu = RT_CPU_ANY, .period_ns = RT_MS(10),
	                    .job = job_sporadico };
	rt_task_t t;
	CONTROLLA(rt_task_start(&t, &c, rt_now()) == 0);
	rt_sleep_until(rt_now() + RT_MS(35));
	CONTROLLA(!rt_task_is_stuck(&t, rt_now(), RT_MS(15)));
	atomic_store(&blocca_sporadico, 1);
	rt_sleep_until(rt_now() + RT_MS(50));
	CONTROLLA(rt_task_is_stuck(&t, rt_now(), RT_MS(15)));
	CONTROLLA(atomic_load(&t.consecutive_misses) == 0);  /* il job bloccato non è ancora finito: non è ancora contato */
	atomic_store(&blocca_sporadico, 0);
	rt_sleep_until(rt_now() + RT_MS(30));
	CONTROLLA(!rt_task_is_stuck(&t, rt_now(), RT_MS(15)));
	rt_task_stop(&t);
}

static void priorita_rate_monotonic(void)
{
	rt_task_cfg_t a = { .period_ns = RT_MS(20) }, b = { .period_ns = RT_MS(200) }, c = { .period_ns = RT_MS(20) },
	              d = { .period_ns = RT_MS(500) }, e = { .period_ns = RT_MS(100) };
	rt_task_cfg_t *v[] = { &b, &a, &d, &c, &e };
	CONTROLLA(rt_priority_rm(v, 5, RT_PRIO_TASK_MAX) == 0);
	CONTROLLA(a.priority == 90 && c.priority == 90);   /* periodi uguali: stessa priorità */
	CONTROLLA(e.priority == 89 && b.priority == 88 && d.priority == 87);
	CONTROLLA(rt_priority_rm(v, 5, 3) == EINVAL);      /* 4 periodi diversi non stanno in 3..1 */
	CONTROLLA(rt_priority_rm(v, 0, 90) == 0);
}

/* ------------------------------------------------------------------------------------------ task aperiodico --- */

static atomic_int aper_job;
static atomic_int aper_lento;          /* 1 = il job dura 50 ms */

static void job_aperiodico(void *arg)
{
	(void)arg;
	struct sched_param p;
	pthread_getschedparam(pthread_self(), &politica_vista, &p);
	priorita_vista = p.sched_priority;
	cpu_vista = sched_getcpu();
	if (atomic_load(&aper_lento)) {
		int64_t fine = rt_now() + RT_MS(50);
		while (rt_now() < fine)
			;
	}
	atomic_fetch_add(&aper_job, 1);
}

/* aspetta (al massimo 1 s) che i job eseguiti arrivino a n */
static void aspetta_job_aperiodici(int n)
{
	int64_t limite = rt_now() + RT_S(1);
	while (atomic_load(&aper_job) < n && rt_now() < limite)
		rt_sleep_until(rt_now() + RT_MS(1));
}

static void aperiodico_config_non_valide(void)
{
	rt_aperiodic_t a;
	rt_aperiodic_cfg_t c = { .cpu = RT_CPU_ANY, .job = NULL };
	CONTROLLA(rt_aperiodic_start(&a, &c) == EINVAL);
	c.job = job_aperiodico;
	c.cpu = -5;
	CONTROLLA(rt_aperiodic_start(&a, &c) == EINVAL);
}

static void aperiodico_un_job_per_sveglia(void)
{
	rt_aperiodic_t a;
	rt_aperiodic_cfg_t c = { .name = "prova", .cpu = 0, .job = job_aperiodico };
	atomic_store(&aper_job, 0);
	atomic_store(&aper_lento, 0);
	CONTROLLA(rt_aperiodic_start(&a, &c) == 0);
	/* senza sveglie non parte niente */
	rt_sleep_until(rt_now() + RT_MS(20));
	CONTROLLA(atomic_load(&aper_job) == 0);
	for (int i = 1; i <= 3; i++) {
		rt_aperiodic_wake(&a);
		aspetta_job_aperiodici(i);
	}
	rt_aperiodic_stop(&a);
	CONTROLLA(atomic_load(&aper_job) == 3);
	CONTROLLA(a.stats.job == 3);
	CONTROLLA(a.stats.wakeups == 3);
	CONTROLLA(politica_vista == SCHED_OTHER);    /* scheduling normale, nessuna priorità real-time */
	CONTROLLA(cpu_vista == 0);
	CONTROLLA(a.stats.response_max_ns >= a.stats.exec_max_ns);
}

static void aperiodico_sveglie_durante_un_job(void)
{
	rt_aperiodic_t a;
	rt_aperiodic_cfg_t c = { .name = "prova", .cpu = RT_CPU_ANY, .job = job_aperiodico };
	atomic_store(&aper_job, 0);
	atomic_store(&aper_lento, 1);
	CONTROLLA(rt_aperiodic_start(&a, &c) == 0);
	rt_aperiodic_wake(&a);
	rt_sleep_until(rt_now() + RT_MS(10));    /* il primo job è partito e dura 50 ms */
	for (int i = 0; i < 5; i++)                  /* 5 sveglie mentre lavora: valgono come una */
		rt_aperiodic_wake(&a);
	aspetta_job_aperiodici(2);
	rt_sleep_until(rt_now() + RT_MS(80));    /* nessun terzo job */
	rt_aperiodic_stop(&a);
	CONTROLLA(atomic_load(&aper_job) == 2);
	CONTROLLA(a.stats.wakeups == 6);
	CONTROLLA(a.stats.exec_max_ns >= RT_MS(50));
	atomic_store(&aper_lento, 0);
}

static void aperiodico_ferma_senza_sveglie(void)
{
	rt_aperiodic_t a;
	rt_aperiodic_cfg_t c = { .cpu = RT_CPU_ANY, .job = job_aperiodico };
	atomic_store(&aper_job, 0);
	CONTROLLA(rt_aperiodic_start(&a, &c) == 0);
	rt_aperiodic_stop(&a);
	CONTROLLA(atomic_load(&aper_job) == 0);
	CONTROLLA(a.stats.job == 0);
}

int main(void)
{
	/* Come nei programmi veri: memoria bloccata in RAM prima di creare i task.
	 * Così i test verificano anche che stack e buffer stiano nei limiti. */
	int ri = rt_init();
	if (ri != 0)
		printf("  attenzione: rt_init fallita (%s)\n", strerror(ri));

	ESEGUI(config_non_valide);
	ESEGUI(mutex_priority_inheritance);
	ESEGUI(priorita_rate_monotonic);
	/* il task aperiodico usa lo scheduling normale: non servono permessi */
	ESEGUI(aperiodico_config_non_valide);
	ESEGUI(aperiodico_un_job_per_sveglia);
	ESEGUI(aperiodico_sveglie_durante_un_job);
	ESEGUI(aperiodico_ferma_senza_sveglie);

	/* Prova a creare un task: se manca il permesso, salta il resto. */
	rt_task_cfg_t prova = { .priority = 1, .cpu = RT_CPU_ANY, .period_ns = RT_MS(1),
				.job = job_conta, .max_job = 1 };
	rt_task_t t;
	int r = rt_task_start(&t, &prova, rt_now());
	permessi_rt = (r == 0);
	if (permessi_rt) {
		rt_task_join(&t);
		ESEGUI(esegue_il_numero_giusto_di_job);
		ESEGUI(politica_priorita_e_core);
		ESEGUI(periodo_rispettato_senza_deriva);
		ESEGUI(conta_le_deadline_mancate);
		ESEGUI(salva_i_campioni);
		ESEGUI(ferma_un_task_infinito);
		ESEGUI(task_sporadici);
		ESEGUI(bloccato_periodico);
	} else {
		printf("  SALTATI i test con task SCHED_FIFO: %s (servono root o ulimit -r > 0)\n", strerror(r));
	}
	return TEST_FINE();
}
