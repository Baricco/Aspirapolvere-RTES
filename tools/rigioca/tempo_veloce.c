/*
 * tempo_veloce.c - Libreria da caricare con LD_PRELOAD: il programma vede CLOCK_MONOTONIC scorrere RTES_VELOCITA volte
 * più in fretta del vero. Serve a tools/rigioca/rigioca.py per ripassare una registrazione più veloce del tempo reale:
 * i dati arrivano N volte più in fretta e il programma li vede con i tempi originali.
 * Tocca solo quello che usano i programmi del progetto: clock_gettime(CLOCK_MONOTONIC), clock_nanosleep su
 * CLOCK_MONOTONIC, ppoll, usleep. Il tempo di CPU dei thread resta vero.
 * Compilazione: gcc -O2 -shared -fPIC -o build/host/tempo_veloce.so tools/rigioca/tempo_veloce.c -ldl
 */
#define _GNU_SOURCE
#include <dlfcn.h>
#include <poll.h>
#include <stdlib.h>
#include <stdint.h>
#include <time.h>
#include <unistd.h>

static double k = 1;
static int64_t t0;                      /* istante vero (ns) in cui il tempo finto e quello vero coincidono */
static int (*vero_gettime)(clockid_t, struct timespec *);
static int (*vero_nanosleep)(clockid_t, int, const struct timespec *, struct timespec *);
static int (*vero_ppoll)(struct pollfd *, nfds_t, const struct timespec *, const sigset_t *);

static int64_t ns(const struct timespec *t) { return (int64_t)t->tv_sec * 1000000000LL + t->tv_nsec; }
static struct timespec ts(int64_t n) { struct timespec t = { n / 1000000000LL, n % 1000000000LL }; return t; }

__attribute__((constructor)) static void avvia(void)
{
	vero_gettime = dlsym(RTLD_NEXT, "clock_gettime");
	vero_nanosleep = dlsym(RTLD_NEXT, "clock_nanosleep");
	vero_ppoll = dlsym(RTLD_NEXT, "ppoll");
	const char *v = getenv("RTES_VELOCITA");
	if (v && atof(v) > 0) k = atof(v);
	struct timespec t;
	vero_gettime(CLOCK_MONOTONIC, &t);
	t0 = ns(&t);
}

static int64_t finto(int64_t vero) { return t0 + (int64_t)((vero - t0) * k); }
static int64_t vero(int64_t finto) { return t0 + (int64_t)((finto - t0) / k); }

int clock_gettime(clockid_t c, struct timespec *t)
{
	int r = vero_gettime(c, t);
	if (r == 0 && (c == CLOCK_MONOTONIC || c == CLOCK_MONOTONIC_RAW || c == CLOCK_BOOTTIME))
		*t = ts(finto(ns(t)));
	return r;
}

int clock_nanosleep(clockid_t c, int flags, const struct timespec *req, struct timespec *rem)
{
	if (c != CLOCK_MONOTONIC)
		return vero_nanosleep(c, flags, req, rem);
	struct timespec r = (flags & TIMER_ABSTIME) ? ts(vero(ns(req))) : ts((int64_t)(ns(req) / k));
	return vero_nanosleep(c, flags, &r, NULL);
}

int ppoll(struct pollfd *f, nfds_t n, const struct timespec *to, const sigset_t *m)
{
	if (!to)
		return vero_ppoll(f, n, NULL, m);
	struct timespec r = ts((int64_t)(ns(to) / k));
	return vero_ppoll(f, n, &r, m);
}

int usleep(useconds_t u)
{
	struct timespec r = ts((int64_t)(u * 1000.0 / k));
	return vero_nanosleep(CLOCK_MONOTONIC, 0, &r, NULL);
}
