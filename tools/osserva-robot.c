/*
 * osserva-robot.c - Sul robot, legge da fuori lo stato della navigazione, la posizione e la mappa del programma dei task.
 *
 * Il programma NON viene modificato e non si accorge di niente: i tempi misurati restano quelli del codice vero. Come
 * sim/osserva.c (che è solo per il PC) legge la sua memoria da /proc/<pid>/mem, senza mutex: può capitare una mappa
 * aggiornata a metà, la successiva la corregge. Gira a scheduling normale, nice 10, sul core 3, che i task non usano.
 * Gli indirizzi non li cerca lui: l'eseguibile è statico, quindi fissi, e li ricava sul PC tools/driver-robot.sh missione.
 *
 * Uscita (i tempi sono secondi dall'avvio del programma):
 *   NOME.oss, testo, ogni 200 ms: "P t x y theta_gradi stato missione" (stato e missione: enum di navigation.c)
 *   NOME.map, binario little endian, ogni 10 s: double t; uint32 versione; uint32 strip_axis_x; griglia di navigazione
 *            da 65.536 byte; clean_cell_mask da 16.384 byte
 *
 * Uso: osserva-robot PID NOME ind_mappa ind_stato ind_missione ind_pulite ind_asse ind_sc_posa ind_buf_posa
 * (indirizzi in esadecimale). Esce quando il programma finisce.
 */
#define _GNU_SOURCE
#include <fcntl.h>
#include <sched.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/resource.h>
#include <time.h>
#include <unistd.h>

#define CELLE 65536
#define PULITE (CELLE / 4)
#define POSA_DIM 32        /* sizeof(drv_posa_t), driver/contatti.h */

static double adesso_s(void)
{
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (double)ts.tv_sec + ts.tv_nsec / 1e9;
}

/* L'istante di avvio del processo, in secondi sullo stesso orologio (campo 22 di /proc/pid/stat, in centesimi). */
static double avvio_processo(int pid)
{
	char p[64], b[1024];
	snprintf(p, sizeof p, "/proc/%d/stat", pid);
	FILE *f = fopen(p, "r");
	if (!f)
		return adesso_s();
	size_t n = fread(b, 1, sizeof b - 1, f);
	fclose(f);
	b[n] = 0;
	char *q = strrchr(b, ')');
	unsigned long long inizio = 0;
	if (!q || sscanf(q + 1, " %*c %*d %*d %*d %*d %*d %*u %*u %*u %*u %*u %*u %*u %*d %*d %*d %*d %*d %*d %llu", &inizio) != 1)
		return adesso_s();
	return (double)inizio / 100.0;
}

static uint8_t griglia[CELLE], pulite[PULITE];

int main(int argc, char **argv)
{
	if (argc != 10) {
		fprintf(stderr, "uso: osserva-robot PID NOME ind_mappa ind_stato ind_missione ind_pulite ind_asse ind_sc_posa ind_buf_posa\n");
		return 2;
	}
	int pid = atoi(argv[1]);
	uint32_t a[7];
	for (int i = 0; i < 7; i++)
		a[i] = (uint32_t)strtoul(argv[3 + i], NULL, 16);
	enum { MAPPA, STATO, MISSIONE, MASCHERA, ASSE, SC_POSA, BUF_POSA };

	cpu_set_t set;
	CPU_ZERO(&set);
	CPU_SET(3, &set);
	sched_setaffinity(0, sizeof set, &set);
	setpriority(PRIO_PROCESS, 0, 10);

	char p[300];
	snprintf(p, sizeof p, "/proc/%d/mem", pid);
	int fd = open(p, O_RDONLY);
	if (fd < 0) {
		perror(p);
		return 1;
	}
	snprintf(p, sizeof p, "%s.oss", argv[2]);
	FILE *oss = fopen(p, "w");
	snprintf(p, sizeof p, "%s.map", argv[2]);
	FILE *map = fopen(p, "wb");
	if (!oss || !map) {
		perror(p);
		return 1;
	}

	double t0 = avvio_processo(pid), prossima_mappa = 0;
	double x = 0, y = 0, th = 0;
	for (;;) {
		int32_t stato, missione;
		if (pread(fd, &stato, 4, a[STATO]) != 4 || pread(fd, &missione, 4, a[MISSIONE]) != 4)
			break;                        /* il programma è finito */
		uint32_t v, v2;
		uint8_t buf[POSA_DIM];
		if (pread(fd, &v, 4, a[SC_POSA]) == 4 && pread(fd, buf, sizeof buf, a[BUF_POSA] + (v & 1) * POSA_DIM) == (ssize_t)sizeof buf &&
		    pread(fd, &v2, 4, a[SC_POSA]) == 4 && v2 == v) {
			double d[3];
			memcpy(d, buf, sizeof d);
			x = d[0], y = d[1], th = d[2];
		}
		double t = adesso_s() - t0;
		fprintf(oss, "P %.2f %.3f %.3f %.1f %d %d\n", t, x, y, th * 57.29578, stato, missione);
		fflush(oss);
		if (t >= prossima_mappa) {
			uint32_t versione = 0, asse = 0;
			uint8_t b = 0;
			if (pread(fd, griglia, CELLE, a[MAPPA]) == CELLE && pread(fd, &versione, 4, a[MAPPA] + CELLE) == 4 &&
			    pread(fd, &b, 1, a[ASSE]) == 1 && pread(fd, pulite, PULITE, a[MASCHERA]) == PULITE) {
				asse = b;
				fwrite(&t, sizeof t, 1, map);
				fwrite(&versione, 4, 1, map);
				fwrite(&asse, 4, 1, map);
				fwrite(griglia, 1, CELLE, map);
				fwrite(pulite, 1, PULITE, map);
				fflush(map);
			}
			prossima_mappa = t + 10;
		}
		struct timespec ts = { 0, 200 * 1000000L };
		nanosleep(&ts, NULL);
	}
	fclose(oss);
	fclose(map);
	return 0;
}
