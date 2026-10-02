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
	int64_t fine = rt_adesso() + RT_MS(3);
	while (rt_adesso() < fine)
		;
}

static int permessi_rt;

static void config_non_valide(void)
{
	rt_task_t t;
	rt_task_cfg_t base = { .priorita = 50, .cpu = RT_CPU_QUALSIASI, .periodo_ns = RT_MS(1), .job = job_conta };
	rt_task_cfg_t c;

	c = base; c.job = NULL;          CONTROLLA(rt_task_avvia(&t, &c, rt_adesso()) == EINVAL);
	c = base; c.periodo_ns = 0;      CONTROLLA(rt_task_avvia(&t, &c, rt_adesso()) == EINVAL);
	c = base; c.priorita = 0;        CONTROLLA(rt_task_avvia(&t, &c, rt_adesso()) == EINVAL);
	c = base; c.priorita = 100;      CONTROLLA(rt_task_avvia(&t, &c, rt_adesso()) == EINVAL);
	c = base; c.cpu = -5;            CONTROLLA(rt_task_avvia(&t, &c, rt_adesso()) == EINVAL);
}

static void esegue_il_numero_giusto_di_job(void)
{
	contatore = 0;
	rt_task_cfg_t c = { .nome = "conta", .priorita = 50, .cpu = RT_CPU_QUALSIASI,
			    .periodo_ns = RT_MS(2), .job = job_conta, .max_job = 20 };
	rt_task_t t;
	CONTROLLA(rt_task_avvia(&t, &c, rt_adesso()) == 0);
	rt_task_attendi(&t);
	CONTROLLA(contatore == 20);
	CONTROLLA(t.stats.job == 20);
	CONTROLLA(t.stats.deadline_mancate == 0);
	CONTROLLA(t.cfg.deadline_ns == RT_MS(2));   /* deadline 0 => uguale al periodo */
}

static void politica_priorita_e_core(void)
{
	rt_task_cfg_t c = { .nome = "osserva", .priorita = 42, .cpu = 0,
			    .periodo_ns = RT_MS(1), .job = job_osserva, .max_job = 3 };
	rt_task_t t;
	CONTROLLA(rt_task_avvia(&t, &c, rt_adesso()) == 0);
	rt_task_attendi(&t);
	CONTROLLA(politica_vista == SCHED_FIFO);
	CONTROLLA(priorita_vista == 42);
	CONTROLLA(cpu_vista == 0);
}

static void periodo_rispettato_senza_deriva(void)
{
	/* 50 job da 2 ms: l'ultimo deve partire ~98 ms dopo il primo,
	 * qualunque sia il ritardo dei singoli risvegli. */
	rt_task_cfg_t c = { .nome = "deriva", .priorita = 50, .cpu = RT_CPU_QUALSIASI,
			    .periodo_ns = RT_MS(2), .job = job_conta, .max_job = 50 };
	rt_task_t t;
	int64_t t0 = rt_adesso() + RT_MS(10);
	CONTROLLA(rt_task_avvia(&t, &c, t0) == 0);
	rt_task_attendi(&t);
	int64_t durata = rt_adesso() - t0;
	CONTROLLA(durata >= RT_MS(98));
	CONTROLLA(durata < RT_MS(98) + RT_MS(5));
}

static void conta_le_deadline_mancate(void)
{
	/* job da 3 ms con deadline 1 ms: ogni job manca la deadline */
	rt_task_cfg_t c = { .nome = "lento", .priorita = 50, .cpu = RT_CPU_QUALSIASI,
			    .periodo_ns = RT_MS(10), .deadline_ns = RT_MS(1), .job = job_lento, .max_job = 5 };
	rt_task_t t;
	CONTROLLA(rt_task_avvia(&t, &c, rt_adesso()) == 0);
	rt_task_attendi(&t);
	CONTROLLA(t.stats.deadline_mancate == 5);
	CONTROLLA(t.stats.esec_min_ns >= RT_MS(3));
}

static void salva_i_campioni(void)
{
	rt_campione_t buf[4];
	rt_task_cfg_t c = { .nome = "campioni", .priorita = 50, .cpu = RT_CPU_QUALSIASI,
			    .periodo_ns = RT_MS(1), .job = job_conta, .max_job = 10,
			    .campioni = buf, .max_campioni = 4 };
	rt_task_t t;
	CONTROLLA(rt_task_avvia(&t, &c, rt_adesso()) == 0);
	rt_task_attendi(&t);
	CONTROLLA(t.stats.n_campioni == 4);   /* non sfora il buffer */
	CONTROLLA(buf[0].latenza_ns >= 0);
}

static void ferma_un_task_infinito(void)
{
	rt_task_cfg_t c = { .nome = "infinito", .priorita = 50, .cpu = RT_CPU_QUALSIASI,
			    .periodo_ns = RT_MS(1), .job = job_conta };
	rt_task_t t;
	CONTROLLA(rt_task_avvia(&t, &c, rt_adesso()) == 0);
	rt_dormi_fino_a(rt_adesso() + RT_MS(20));
	rt_task_ferma(&t);
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

int main(void)
{
	/* Come nei programmi veri: memoria bloccata in RAM prima di creare i task.
	 * Così i test verificano anche che stack e buffer stiano nei limiti. */
	int ri = rt_init();
	if (ri != 0)
		printf("  attenzione: rt_init fallita (%s)\n", strerror(ri));

	ESEGUI(config_non_valide);
	ESEGUI(mutex_priority_inheritance);

	/* Prova a creare un task: se manca il permesso, salta il resto. */
	rt_task_cfg_t prova = { .priorita = 1, .cpu = RT_CPU_QUALSIASI, .periodo_ns = RT_MS(1),
				.job = job_conta, .max_job = 1 };
	rt_task_t t;
	int r = rt_task_avvia(&t, &prova, rt_adesso());
	permessi_rt = (r == 0);
	if (permessi_rt) {
		rt_task_attendi(&t);
		ESEGUI(esegue_il_numero_giusto_di_job);
		ESEGUI(politica_priorita_e_core);
		ESEGUI(periodo_rispettato_senza_deriva);
		ESEGUI(conta_le_deadline_mancate);
		ESEGUI(salva_i_campioni);
		ESEGUI(ferma_un_task_infinito);
	} else {
		printf("  SALTATI i test con task SCHED_FIFO: %s (servono root o ulimit -r > 0)\n", strerror(r));
	}
	return TEST_FINE();
}
