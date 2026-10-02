/*
 * carico.c - Generatore di carico CPU per le misure di latenza.
 *
 * Avvia un thread che consuma CPU al 100% su ciascuno dei core indicati, per
 * la durata indicata. Di default i thread sono SCHED_OTHER (processi normali):
 * un task SCHED_FIFO sullo stesso core deve interromperli subito. Con -P si
 * possono rendere SCHED_FIFO, per studiare l'interferenza tra task RT.
 * Serve perché sul robot non c'è `taskset`.
 *
 * Esempi:
 *   ./carico -c 0,1,2 -d 60          core 0, 1 e 2 occupati per 60 s
 *   ./carico -c 3 -d 60 -P 10        core 3 occupato da un thread SCHED_FIFO a priorità 10
 */
#define _GNU_SOURCE
#include <pthread.h>
#include <sched.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "../rt/rt_tempo.h"

#define MAX_CORE 64

static int64_t fine_ns;
volatile uint64_t carico_contatore;   /* globale e volatile: il compilatore non può eliminare il ciclo */

static void *gira(void *arg)
{
	int cpu = (int)(long)arg;
	cpu_set_t set;
	CPU_ZERO(&set);
	CPU_SET(cpu, &set);
	int r = pthread_setaffinity_np(pthread_self(), sizeof(set), &set);
	if (r != 0) {
		fprintf(stderr, "core %d: affinità non impostata (%s)\n", cpu, strerror(r));
		return NULL;
	}
	while (rt_adesso() < fine_ns)
		for (int i = 0; i < 10000; i++)
			carico_contatore++;
	return NULL;
}

int main(int argc, char **argv)
{
	char *lista = NULL;
	int durata_s = 10, priorita = 0;
	int o;
	while ((o = getopt(argc, argv, "c:d:P:h")) != -1) {
		switch (o) {
		case 'c': lista = optarg; break;
		case 'd': durata_s = atoi(optarg); break;
		case 'P': priorita = atoi(optarg); break;
		default:
			fprintf(stderr, "Uso: %s -c core[,core...] [-d durata_s] [-P priorita_fifo]\n", argv[0]);
			return 2;
		}
	}
	if (!lista || durata_s <= 0) {
		fprintf(stderr, "Uso: %s -c core[,core...] [-d durata_s] [-P priorita_fifo]\n", argv[0]);
		return 2;
	}

	fine_ns = rt_adesso() + RT_S(durata_s);
	pthread_t th[MAX_CORE];
	int n = 0;
	for (char *tok = strtok(lista, ","); tok && n < MAX_CORE; tok = strtok(NULL, ",")) {
		pthread_attr_t a;
		pthread_attr_init(&a);
		if (priorita > 0) {
			struct sched_param p = { .sched_priority = priorita };
			pthread_attr_setinheritsched(&a, PTHREAD_EXPLICIT_SCHED);
			pthread_attr_setschedpolicy(&a, SCHED_FIFO);
			pthread_attr_setschedparam(&a, &p);
		}
		int r = pthread_create(&th[n], &a, gira, (void *)(long)atoi(tok));
		pthread_attr_destroy(&a);
		if (r != 0) {
			fprintf(stderr, "thread per il core %s non creato: %s\n", tok, strerror(r));
			continue;
		}
		n++;
	}
	printf("carico: %d thread %s per %d s\n", n, priorita > 0 ? "SCHED_FIFO" : "SCHED_OTHER", durata_s);
	fflush(stdout);
	for (int i = 0; i < n; i++)
		pthread_join(th[i], NULL);
	return 0;
}
