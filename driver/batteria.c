/* batteria.c - Stato della batteria: vedi batteria.h. */
#include "batteria.h"

#include <string.h>

/*
 * Stima della percentuale ricopiata da everest-server (classe CBatteryCapacity3200, chiamata a ogni frame dei
 * sensori, 50 volte al secondo). Indirizzi nella versione estratta: batteryCapacity 0x66a2b0,
 * calculateIdleStateCapacity 0x668f28, calculateMoppingStateCapacity 0x669f70, calculateChargeStateCapacity 0x66a150.
 */
#define PASSO            0.01     /* 1 %: margine per "sale" / "scende", passo della salita in carica e del calo a tempo */
#define DISCESA          0.9998   /* verso la curva: a ogni frame resta il 99,98 % dello scarto (100 s) */
#define CARICA_PIENA_V   16.4     /* prima stima sulla base: sopra questa tensione, 97 % */

static void anello_metti(uint8_t *buf, int dim, int *n, int *i, long *somma, int x)
{
	if (*n == dim)
		*somma -= buf[*i];
	else
		(*n)++;
	buf[*i] = (uint8_t)x;
	*somma += x;
	*i = (*i + 1) % dim;
}

void batteria_init(batteria_t *b)
{
	memset(b, 0, sizeof(*b));
	b->stato = BAT_STATO_FERMO;          /* come l'originale all'avvio */
	b->stima = -1;
}

double batteria_volt(const batteria_t *b)
{
	return b->n ? (double)b->somma / b->n / 10.0 : 0;
}

int batteria_percentuale(const batteria_t *b)
{
	return b->stima < 0 ? 0 : (int)(b->stima * 100 + 0.5);
}

/* Tre tratti, con i confini a 13,3 V e 14,3 V; coefficienti del firmware. */
double batteria_curva(int stato, double v)
{
	if (stato == BAT_STATO_MOTO) {
		if (v >= 14.3)
			return 0.3355 * v - 4.3157;
		if (v >= 13.3)
			return 0.2039 * v * v - 5.3007 * v + 34.543;
		return 0.0618 * v * v - 1.5107 * v + 9.2473;
	}
	if (v >= 14.3)
		return 0.2845 * v - 3.7294;
	if (v >= 13.3)
		return 0.1107 * v * v - 2.8129 * v + 17.894;
	return 0.0375 * v * v - 0.9165 * v + 5.6079;
}

/* Lontano dalla base: la stima non sale mai. */
static double scarica(batteria_t *b, double v)
{
	double curva = batteria_curva(b->stato, v), s = b->stima;
	int limite = b->stato == BAT_STATO_MOTO ? 8500 : 16200;     /* frame: ~170 s e ~324 s */
	if (s < 0)
		return curva;
	if (curva - s > PASSO) {
		/* tensione risalita (carico tolto): la stima resta ferma e cala dell'1 % ogni `limite` frame */
		if (b->conta_salita++ > limite) {
			b->conta_salita = 0;
			s -= PASSO;
		}
	} else if (s - curva > PASSO) {
		s = s * DISCESA + curva * 0.0002;
	} else if (curva < s) {
		s = curva;
	}
	return s;
}

/* Sulla base: la tensione è alzata dal caricatore, quindi la curva serve solo per la prima stima. */
static double carica(batteria_t *b, double v, int carica_stabile)
{
	if (b->stima < 0) {
		if (carica_stabile || v > CARICA_PIENA_V)
			return 0.97;
		double x = v <= 16.0 ? v - 0.4 : v;
		return -0.0335 * x * x + 1.3959 * x - 12.942;
	}
	if (b->conta_carica++ > (v > 15.0 ? 5000 : 6000)) {          /* frame: ~100 s e ~120 s */
		b->conta_carica = 0;
		return b->stima + PASSO;
	}
	return b->stima;
}

static void aggiorna_stima(batteria_t *b, int tensione_dv, int in_carica, int sulla_base, int in_moto)
{
	/* bit "in carica" acceso per più dell'80 % degli ultimi 30 s; l'originale guarda la media prima di questo frame */
	int carica_stabile = b->n_c && 10 * b->somma_c > 8L * b->n_c;
	int stato = sulla_base || carica_stabile ? BAT_STATO_CARICA : in_moto ? BAT_STATO_MOTO : BAT_STATO_FERMO;
	if (stato != b->stato) {
		if (b->stato != BAT_STATO_CARICA && stato < b->stato) {
			b->n_t = b->i_t = 0;                 /* carico più forte: la media della tensione riparte */
			b->somma_t = 0;
		}
		b->conta_carica = b->conta_salita = 0;
		b->stato = stato;
	}
	anello_metti(b->tensioni, BAT_MEDIA_STIMA, &b->n_t, &b->i_t, &b->somma_t, tensione_dv);
	anello_metti(b->carica, BAT_MEDIA_CARICA, &b->n_c, &b->i_c, &b->somma_c, in_carica ? 1 : 0);

	double v = (double)b->somma_t / b->n_t / 10.0;
	double s = stato == BAT_STATO_CARICA ? carica(b, v, carica_stabile) : scarica(b, v);
	b->stima = s > 1 ? 1 : s < 0 ? 0 : s;
}

int batteria_aggiorna(batteria_t *b, int tensione_dv, int in_carica, int sulla_base, int in_moto)
{
	aggiorna_stima(b, tensione_dv, in_carica, sulla_base, in_moto);

	if (b->n == BAT_MEDIA_CAMPIONI)
		b->somma -= b->campioni[b->i];
	else
		b->n++;
	b->campioni[b->i] = tensione_dv;
	b->somma += tensione_dv;
	b->i = (b->i + 1) % BAT_MEDIA_CAMPIONI;

	/* soglie sulla media in decimi di volt: somma confrontata con soglia * n, senza divisioni */
	long s = b->somma, n = b->n;
	if (n < 10)
		return b->livello;              /* meno di 200 ms di misure: troppo poche per decidere */
	if (in_carica) {
		b->livello = BAT_OK;
	} else if (s < (long)BAT_SOGLIA_CRITICA_DV * n) {
		b->livello = BAT_CRITICA;
	} else if (b->livello == BAT_CRITICA) {
		if (s > (long)BAT_USCITA_CRITICA_DV * n)
			b->livello = BAT_BASSA;
	} else if (s < (long)BAT_SOGLIA_BASSA_DV * n) {
		b->livello = BAT_BASSA;
	} else if (b->livello == BAT_BASSA && s > (long)BAT_USCITA_BASSA_DV * n) {
		b->livello = BAT_OK;
	}
	return b->livello;
}
