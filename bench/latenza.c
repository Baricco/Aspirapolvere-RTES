/*
 * latenza.c - Misura latenza di risveglio e jitter di un task periodico SCHED_FIFO.
 *
 * È la misura di base della fase 0. Un task periodico si sveglia a istanti
 * assoluti; la latenza è quanto si sveglia in ritardo rispetto all'istante
 * teorico. La latenza massima, insieme a quella di interrupt e kernel, è il
 * ritardo peggiore che ogni task real-time subirà su questo sistema.
 *
 * Esempi:
 *   ./latenza                          1 ms di periodo per 10 s, priorità 80, qualsiasi core
 *   ./latenza -c 3 -d 60 -o lat.csv    fissato al core 3, 60 s, campioni salvati in CSV
 *   ./latenza -e 200                   ogni job simula 200 us di calcolo
 *   ./latenza -d 3600 -s 30            misura di un'ora, una riga di avanzamento ogni 30 s
 */
#define _GNU_SOURCE
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/utsname.h>
#include <unistd.h>

#include "../rt/rt.h"

/* Calcolo fittizio: occupa la CPU per circa `us` microsecondi. */
static void job_carico(void *arg)
{
	int64_t durata = *(int64_t *)arg;
	if (durata <= 0)
		return;
	int64_t fine = rt_now() + durata;
	while (rt_now() < fine)
		;
}

static void uso(const char *prog)
{
	fprintf(stderr,
		"Uso: %s [-p periodo_us] [-d durata_s] [-P priorita] [-c cpu] [-e esecuzione_us] [-o file.csv] [-s secondi]\n"
		"  -p  periodo del task in microsecondi (default 1000)\n"
		"  -d  durata della misura in secondi (default 10)\n"
		"  -P  priorità SCHED_FIFO 1..99 (default 80)\n"
		"  -c  core a cui fissare il task (default: qualsiasi)\n"
		"  -e  calcolo simulato per ogni job, in microsecondi (default 0)\n"
		"  -o  salva un campione per job in CSV (latenza_ns,esecuzione_ns)\n"
		"  -s  stampa una riga di avanzamento ogni tot secondi (default: mai)\n", prog);
}

static void stampa_sistema(void)
{
	struct utsname u;
	if (uname(&u) == 0)
		printf("sistema: %s %s %s (%s)\n", u.sysname, u.release, u.version, u.machine);
	printf("core online: %ld\n", sysconf(_SC_NPROCESSORS_ONLN));
	FILE *f = fopen("/proc/sys/kernel/sched_rt_runtime_us", "r");
	if (f) {
		long v;
		if (fscanf(f, "%ld", &v) == 1)
			printf("sched_rt_runtime_us: %ld (tempo CPU per secondo concesso ai task RT)\n", v);
		fclose(f);
	}
}

int main(int argc, char **argv)
{
	int64_t periodo_us = 1000, esecuzione_us = 0;
	int durata_s = 10, priority = 80, cpu = RT_CPU_ANY, avanzamento_s = 0;
	const char *file_csv = NULL;

	int o;
	while ((o = getopt(argc, argv, "p:d:P:c:e:o:s:h")) != -1) {
		switch (o) {
		case 'p': periodo_us = atoll(optarg); break;
		case 'd': durata_s = atoi(optarg); break;
		case 'P': priority = atoi(optarg); break;
		case 'c': cpu = atoi(optarg); break;
		case 'e': esecuzione_us = atoll(optarg); break;
		case 'o': file_csv = optarg; break;
		case 's': avanzamento_s = atoi(optarg); break;
		default: uso(argv[0]); return 2;
		}
	}
	if (periodo_us <= 0 || durata_s <= 0 || avanzamento_s < 0) {
		uso(argv[0]);
		return 2;
	}

	stampa_sistema();

	int r = rt_init();
	if (r != 0)
		fprintf(stderr, "attenzione: mlockall fallita (%s), la misura può includere page fault\n",
			strerror(r));

	uint64_t n_job = (uint64_t)durata_s * 1000000 / periodo_us;
	int64_t execution_ns = RT_US(esecuzione_us);

	/* Il buffer dei campioni si alloca e si "tocca" prima di avviare il task:
	 * con mlockall le pagine restano in RAM e il ciclo RT non alloca niente. */
	rt_sample_t *samples = NULL;
	if (file_csv) {
		samples = calloc(n_job, sizeof(*samples));
		if (!samples) {
			fprintf(stderr, "memoria insufficiente per %llu campioni\n", (unsigned long long)n_job);
			return 1;
		}
	}

	rt_task_cfg_t cfg = {
		.name = "latenza",
		.priority = priority,
		.cpu = cpu,
		.period_ns = RT_US(periodo_us),
		.job = job_carico,
		.arg = &execution_ns,
		.max_job = n_job,
		.samples = samples,
		.max_samples = samples ? n_job : 0,
	};

	printf("misura: periodo %lld us, %llu job (~%d s), priorità %d, cpu %d, esecuzione %lld us\n",
	       (long long)periodo_us, (unsigned long long)n_job, durata_s, priority, cpu,
	       (long long)esecuzione_us);
	fflush(stdout);

	rt_task_t t;
	r = rt_task_start(&t, &cfg, rt_now() + RT_MS(100));
	if (r != 0) {
		fprintf(stderr, "impossibile avviare il task: %s%s\n", strerror(r),
			r == EPERM ? " (servono root o un limite rtprio: ulimit -r)" : "");
		return 1;
	}
	/* Avanzamento per le misure lunghe: lo stampa il thread principale
	 * (SCHED_OTHER), mai il task RT. Le statistiche si leggono mentre il task
	 * le aggiorna, quindi i valori sono indicativi (vedi rt.h); quelli esatti
	 * sono nel riepilogo finale. */
	for (int trascorsi = 0; avanzamento_s > 0 && trascorsi + avanzamento_s < durata_s;
	     trascorsi += avanzamento_s) {
		sleep(avanzamento_s);
		rt_stats_t s = t.stats;
		printf("avanzamento: %d s, job %llu/%llu, deadline mancate %llu, latenza max %.1f us\n",
		       trascorsi + avanzamento_s, (unsigned long long)s.job, (unsigned long long)n_job,
		       (unsigned long long)s.missed_deadlines, s.latency_max_ns / 1000.0);
		fflush(stdout);
	}
	rt_task_join(&t);
	rt_stats_print(&t);

	if (file_csv) {
		FILE *f = fopen(file_csv, "w");
		if (!f) {
			fprintf(stderr, "impossibile scrivere %s: %s\n", file_csv, strerror(errno));
			return 1;
		}
		fprintf(f, "latenza_ns,esecuzione_ns\n");
		for (size_t i = 0; i < t.stats.n_samples; i++)
			fprintf(f, "%lld,%lld\n", (long long)samples[i].latency_ns,
				(long long)samples[i].execution_ns);
		fclose(f);
		printf("campioni salvati in %s (%zu righe)\n", file_csv, t.stats.n_samples);
	}
	free(samples);
	return 0;
}
