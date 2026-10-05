/* batteria.c - Stato della batteria: vedi batteria.h. */
#include "batteria.h"

#include <string.h>

/*
 * Stima della percentuale ricopiata da everest-server (classe CBatteryCapacity3200, chiamata a ogni frame dei
 * sensori, 50 volte al secondo). Indirizzi nella versione estratta: batteryCapacity 0x66a2b0,
 * calculateIdleStateCapacity 0x668f28, calculateMoppingStateCapacity 0x669f70, calculateChargeStateCapacity 0x66a150,
 * calculate2600PaStateCapacity 0x669108 ... calculate600PaStateCapacity 0x669d90 (una per stato dell'aspirazione).
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

/* Le curve di ogni stato: tre tratti, con i confini a 13,3 V e 14,3 V (coefficienti del firmware, docs/hardware.md);
 * limite = frame del calo a tempo dell'1 %; nei tre stati più forti sopra 15,5 V la discesa verso la curva è 10 volte
 * più lenta, e da 15,6 V la curva vale 1,05. */
typedef struct {
	double a1, b1;               /* V >= 14,3: a1 V + b1 */
	double a2, b2, c2;           /* 13,3 <= V < 14,3: a2 V² + b2 V + c2 */
	double a3, b3, c3;           /* V < 13,3 */
	int limite;
	int forte;                   /* 1500, 2000, 2600 Pa */
} curva_t;

static const curva_t CURVE[11] = {
	[BAT_STATO_2600PA] = { 0.2673, -3.2609, -0.0957, 3.0044, -22.825, 0.1469, -3.5625, 21.608, 2250, 1 },
	[BAT_STATO_2000PA] = { 0.2512, -3.0092, -0.2143, 6.2383, -44.819, 0.103, -2.4044, 14.006, 2400, 1 },
	[BAT_STATO_1500PA] = { 0.2544, -3.0593, -0.0909, 2.8659, -21.825, 0.1228, -2.9405, 17.602, 2750, 1 },
	[BAT_STATO_1200PA] = { 0.2748, -3.4517, -0.0307, 1.1779, -10.069, 0.1167, -2.8292, 17.156, 3250, 0 },
	[BAT_STATO_1000PA] = { 0.2979, -3.749, -0.0749, 2.4066, -18.576, 0.1281, -3.0971, 18.716, 3500, 0 },
	[BAT_STATO_800PA]  = { 0.2964, -3.7618, 0.1327, -3.2947, 20.455, 0.0938, -2.3008, 14.107, 4050, 0 },
	[BAT_STATO_600PA]  = { 0.2956, -3.7757, 0.1436, -3.6098, 22.711, 0.1, -2.4568, 15.09, 4900, 0 },
	[BAT_STATO_MOTO]   = { 0.3355, -4.3157, 0.2039, -5.3007, 34.543, 0.0618, -1.5107, 9.2473, 8500, 0 },
	[BAT_STATO_FERMO]  = { 0.2845, -3.7294, 0.1107, -2.8129, 17.894, 0.0375, -0.9165, 5.6079, 16200, 0 },
};

static const curva_t *curva_di(int stato)
{
	return stato >= 1 && stato <= 10 && CURVE[stato].limite ? &CURVE[stato] : &CURVE[BAT_STATO_FERMO];
}

double batteria_curva(int stato, double v)
{
	const curva_t *c = curva_di(stato);
	if (c->forte && v >= 15.6)
		return 1.05;
	if (v >= 14.3)
		return c->a1 * v + c->b1;
	if (v >= 13.3)
		return (c->a2 * v + c->b2) * v + c->c2;
	return (c->a3 * v + c->b3) * v + c->c3;
}

int batteria_stato_carico(int in_moto, int giri)
{
	if (giri >= 20000) return BAT_STATO_2600PA;
	if (giri >= 19000) return BAT_STATO_2000PA;
	if (giri >= 17000) return BAT_STATO_1500PA;
	if (giri >= 15000) return BAT_STATO_1200PA;
	if (giri >= 14000) return BAT_STATO_1000PA;
	if (giri >= 12000) return BAT_STATO_800PA;
	if (giri > 1000) return BAT_STATO_600PA;
	return giri == 0 && in_moto ? BAT_STATO_MOTO : BAT_STATO_FERMO;
}

/* Lontano dalla base: la stima non sale mai. */
static double scarica(batteria_t *b, double v)
{
	const curva_t *c = curva_di(b->stato);
	double curva = batteria_curva(b->stato, v), s = b->stima;
	int limite = c->limite;                     /* frame: da ~324 s (fermo) a ~45 s (ventola a 2600 Pa) */
	double discesa = c->forte && v > 15.5 ? 0.99998 : DISCESA;
	if (s < 0)
		return curva;
	if (curva - s > PASSO) {
		/* tensione risalita (carico tolto): la stima resta ferma e cala dell'1 % ogni `limite` frame */
		if (b->conta_salita++ > limite) {
			b->conta_salita = 0;
			s -= PASSO;
		}
	} else if (s - curva > PASSO) {
		s = s * discesa + curva * (1 - discesa);
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

static void aggiorna_stima(batteria_t *b, int tensione_dv, int in_carica, int sulla_base, int in_moto, int giri)
{
	/* bit "in carica" acceso per più dell'80 % degli ultimi 30 s; l'originale guarda la media prima di questo frame */
	int carica_stabile = b->n_c && 10 * b->somma_c > 8L * b->n_c;
	int stato = sulla_base || carica_stabile ? BAT_STATO_CARICA : batteria_stato_carico(in_moto, giri);
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

int batteria_aggiorna(batteria_t *b, int tensione_dv, int in_carica, int sulla_base, int in_moto, int giri_ventola)
{
	aggiorna_stima(b, tensione_dv, in_carica, sulla_base, in_moto, giri_ventola);

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
