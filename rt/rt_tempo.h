/*
 * rt_tempo.h - Gestione del tempo per i task real-time.
 *
 * Nel progetto il tempo si rappresenta come un intero a 64 bit di nanosecondi
 * (int64_t), misurato su CLOCK_MONOTONIC: un orologio che parte dall'accensione,
 * non torna mai indietro e non cambia se qualcuno modifica data e ora del
 * sistema (al contrario di CLOCK_REALTIME). È l'orologio giusto per periodi e
 * deadline. Un int64_t di nanosecondi copre circa 292 anni, quindi non va mai
 * in overflow.
 *
 * Le API POSIX usano invece struct timespec { tv_sec, tv_nsec }: le funzioni
 * qui sotto convertono tra le due rappresentazioni.
 */
#ifndef RT_TEMPO_H
#define RT_TEMPO_H

#include <errno.h>
#include <stdint.h>
#include <time.h>

#define RT_NS_PER_US  1000LL
#define RT_NS_PER_MS  1000000LL
#define RT_NS_PER_S   1000000000LL

/* Conversioni comode: RT_MS(20) = 20 ms espressi in ns. */
#define RT_US(x) ((int64_t)(x) * RT_NS_PER_US)
#define RT_MS(x) ((int64_t)(x) * RT_NS_PER_MS)
#define RT_S(x)  ((int64_t)(x) * RT_NS_PER_S)

static inline int64_t rt_timespec_a_ns(const struct timespec *ts)
{
	return (int64_t)ts->tv_sec * RT_NS_PER_S + ts->tv_nsec;
}

/* tv_nsec deve sempre stare in [0, 1e9): per i valori negativi la divisione
 * del C tronca verso zero, quindi il resto va riportato nell'intervallo. */
static inline struct timespec rt_ns_a_timespec(int64_t ns)
{
	struct timespec ts;
	ts.tv_sec = (time_t)(ns / RT_NS_PER_S);
	ts.tv_nsec = (long)(ns % RT_NS_PER_S);
	if (ts.tv_nsec < 0) {
		ts.tv_nsec += RT_NS_PER_S;
		ts.tv_sec -= 1;
	}
	return ts;
}

/* Istante attuale in ns su CLOCK_MONOTONIC. */
static inline int64_t rt_adesso(void)
{
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return rt_timespec_a_ns(&ts);
}

/*
 * Dorme fino all'istante assoluto `istante_ns` (non "per" un certo tempo).
 * È l'equivalente di vTaskDelayUntil() di FreeRTOS: usando istanti assoluti
 * l'errore non si accumula tra un periodo e l'altro (niente deriva).
 * Ritenta se il sonno viene interrotto da un segnale (EINTR).
 */
static inline void rt_dormi_fino_a(int64_t istante_ns)
{
	struct timespec ts = rt_ns_a_timespec(istante_ns);
	while (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL) == EINTR)
		;
}

#endif /* RT_TEMPO_H */
