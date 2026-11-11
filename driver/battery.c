/* battery.c - Stato della batteria: vedi battery.h. */
#include "battery.h"

#include <string.h>

/*
 * Stima della percentuale ricopiata da everest-server (classe CBatteryCapacity3200, chiamata a ogni frame dei
 * sensori, 50 volte al secondo). Indirizzi nella versione estratta: batteryCapacity 0x66a2b0,
 * calculateIdleStateCapacity 0x668f28, calculateMoppingStateCapacity 0x669f70, calculateChargeStateCapacity 0x66a150,
 * calculate2600PaStateCapacity 0x669108 ... calculate600PaStateCapacity 0x669d90 (una per stato dell'aspirazione).
 */
#define STEP            0.01     /* 1 %: margine per "sale" / "scende", passo della salita in carica e del calo a tempo */
#define DECAY          0.9998   /* verso la curva: a ogni frame resta il 99,98 % dello scarto (100 s) */
#define FULL_CHARGE_V   16.4     /* prima stima sulla base: sopra questa tensione, 97 % */

static void ring_push(uint8_t *buf, int size, int *n, int *i, long *sum, int x)
{
	if (*n == size)
		*sum -= buf[*i];
	else
		(*n)++;
	buf[*i] = (uint8_t)x;
	*sum += x;
	*i = (*i + 1) % size;
}

void battery_init(battery_t *b)
{
	memset(b, 0, sizeof(*b));
	b->status = BAT_STATE_IDLE;          /* come l'originale all'avvio */
	b->estimate = -1;
}

double battery_volts(const battery_t *b)
{
	return b->n ? (double)b->sum / b->n / 10.0 : 0;
}

int battery_percentage(const battery_t *b)
{
	return b->estimate < 0 ? 0 : (int)(b->estimate * 100 + 0.5);
}

/* Le curve di ogni stato: tre tratti, con i confini a 13,3 V e 14,3 V (coefficienti del firmware, docs/hardware.md);
 * limite = frame del calo a tempo dell'1 %; nei tre stati più forti sopra 15,5 V la discesa verso la curva è 10 volte
 * più lenta, e da 15,6 V la curva vale 1,05. */
typedef struct {
	double a1, b1;               /* V >= 14,3: a1 V + b1 */
	double a2, b2, c2;           /* 13,3 <= V < 14,3: a2 V² + b2 V + c2 */
	double a3, b3, c3;           /* V < 13,3 */
	int limit;
	int strong;                   /* 1500, 2000, 2600 Pa */
} curve_t;

static const curve_t CURVES[11] = {
	[BAT_STATE_2600PA] = { 0.2673, -3.2609, -0.0957, 3.0044, -22.825, 0.1469, -3.5625, 21.608, 2250, 1 },
	[BAT_STATE_2000PA] = { 0.2512, -3.0092, -0.2143, 6.2383, -44.819, 0.103, -2.4044, 14.006, 2400, 1 },
	[BAT_STATE_1500PA] = { 0.2544, -3.0593, -0.0909, 2.8659, -21.825, 0.1228, -2.9405, 17.602, 2750, 1 },
	[BAT_STATE_1200PA] = { 0.2748, -3.4517, -0.0307, 1.1779, -10.069, 0.1167, -2.8292, 17.156, 3250, 0 },
	[BAT_STATE_1000PA] = { 0.2979, -3.749, -0.0749, 2.4066, -18.576, 0.1281, -3.0971, 18.716, 3500, 0 },
	[BAT_STATE_800PA]  = { 0.2964, -3.7618, 0.1327, -3.2947, 20.455, 0.0938, -2.3008, 14.107, 4050, 0 },
	[BAT_STATE_600PA]  = { 0.2956, -3.7757, 0.1436, -3.6098, 22.711, 0.1, -2.4568, 15.09, 4900, 0 },
	[BAT_STATE_MOVING]   = { 0.3355, -4.3157, 0.2039, -5.3007, 34.543, 0.0618, -1.5107, 9.2473, 8500, 0 },
	[BAT_STATE_IDLE]  = { 0.2845, -3.7294, 0.1107, -2.8129, 17.894, 0.0375, -0.9165, 5.6079, 16200, 0 },
};

static const curve_t *curve_of(int status)
{
	return status >= 1 && status <= 10 && CURVES[status].limit ? &CURVES[status] : &CURVES[BAT_STATE_IDLE];
}

double battery_curve(int status, double v)
{
	const curve_t *c = curve_of(status);
	if (c->strong && v >= 15.6)
		return 1.05;
	if (v >= 14.3)
		return c->a1 * v + c->b1;
	if (v >= 13.3)
		return (c->a2 * v + c->b2) * v + c->c2;
	return (c->a3 * v + c->b3) * v + c->c3;
}

int battery_load_state(int moving, int revolutions)
{
	if (revolutions >= 20000) return BAT_STATE_2600PA;
	if (revolutions >= 19000) return BAT_STATE_2000PA;
	if (revolutions >= 17000) return BAT_STATE_1500PA;
	if (revolutions >= 15000) return BAT_STATE_1200PA;
	if (revolutions >= 14000) return BAT_STATE_1000PA;
	if (revolutions >= 12000) return BAT_STATE_800PA;
	if (revolutions > 1000) return BAT_STATE_600PA;
	return revolutions == 0 && moving ? BAT_STATE_MOVING : BAT_STATE_IDLE;
}

/* Lontano dalla base: la stima non sale mai. */
static double discharge(battery_t *b, double v)
{
	const curve_t *c = curve_of(b->status);
	double curve = battery_curve(b->status, v), s = b->estimate;
	int limit = c->limit;                     /* frame: da ~324 s (fermo) a ~45 s (ventola a 2600 Pa) */
	double decay = c->strong && v > 15.5 ? 0.99998 : DECAY;
	if (s < 0)
		return curve;
	if (curve - s > STEP) {
		/* tensione risalita (carico tolto): la stima resta ferma e cala dell'1 % ogni `limite` frame */
		if (b->rise_count++ > limit) {
			b->rise_count = 0;
			s -= STEP;
		}
	} else if (s - curve > STEP) {
		s = s * decay + curve * (1 - decay);
	} else if (curve < s) {
		s = curve;
	}
	return s;
}

/* Sulla base: la tensione è alzata dal caricatore, quindi la curva serve solo per la prima stima. */
static double charge(battery_t *b, double v, int stable_charging)
{
	if (b->estimate < 0) {
		if (stable_charging || v > FULL_CHARGE_V)
			return 0.97;
		double x = v <= 16.0 ? v - 0.4 : v;
		return -0.0335 * x * x + 1.3959 * x - 12.942;
	}
	if (b->charge_count++ > (v > 15.0 ? 5000 : 6000)) {          /* frame: ~100 s e ~120 s */
		b->charge_count = 0;
		return b->estimate + STEP;
	}
	return b->estimate;
}

static void update_estimate(battery_t *b, int voltage_dv, int charging, int on_base, int moving, int revolutions)
{
	/* bit "in carica" acceso per più dell'80 % degli ultimi 30 s; l'originale guarda la media prima di questo frame */
	int stable_charging = b->n_c && 10 * b->sum_c > 8L * b->n_c;
	int status = on_base || stable_charging ? BAT_STATE_CHARGING : battery_load_state(moving, revolutions);
	if (status != b->status) {
		if (b->status != BAT_STATE_CHARGING && status < b->status) {
			b->n_t = b->i_t = 0;                 /* carico più forte: la media della tensione riparte */
			b->sum_t = 0;
		}
		b->charge_count = b->rise_count = 0;
		b->status = status;
	}
	ring_push(b->voltages, BAT_AVG_ESTIMATE, &b->n_t, &b->i_t, &b->sum_t, voltage_dv);
	ring_push(b->charge, BAT_AVG_CHARGING, &b->n_c, &b->i_c, &b->sum_c, charging ? 1 : 0);

	double v = (double)b->sum_t / b->n_t / 10.0;
	double s = status == BAT_STATE_CHARGING ? charge(b, v, stable_charging) : discharge(b, v);
	b->estimate = s > 1 ? 1 : s < 0 ? 0 : s;
}

int battery_update(battery_t *b, int voltage_dv, int charging, int on_base, int moving, int fan_rpm)
{
	update_estimate(b, voltage_dv, charging, on_base, moving, fan_rpm);

	if (b->n == BAT_AVG_SAMPLES)
		b->sum -= b->samples[b->i];
	else
		b->n++;
	b->samples[b->i] = voltage_dv;
	b->sum += voltage_dv;
	b->i = (b->i + 1) % BAT_AVG_SAMPLES;

	/* soglie sulla media in decimi di volt: somma confrontata con soglia * n, senza divisioni */
	long s = b->sum, n = b->n;
	if (n < 10)
		return b->level;              /* meno di 200 ms di misure: troppo poche per decidere */
	if (charging || on_base) {      /* sui contatti la carica passa anche con il bit "in carica" spento */
		b->level = BAT_OK;
	} else if (s < (long)BAT_CRITICAL_THRESHOLD_DV * n) {
		b->level = BAT_CRITICAL;
	} else if (b->level == BAT_CRITICAL) {
		if (s > (long)BAT_CRITICAL_EXIT_DV * n)
			b->level = BAT_LOW;
	} else if (s < (long)BAT_LOW_THRESHOLD_DV * n) {
		b->level = BAT_LOW;
	} else if (b->level == BAT_LOW && s > (long)BAT_LOW_EXIT_DV * n) {
		b->level = BAT_OK;
	}
	return b->level;
}
