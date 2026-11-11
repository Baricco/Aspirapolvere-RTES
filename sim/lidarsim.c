/* lidarsim.c - Il lidar simulato: vedi lidarsim.h. */
#include "lidarsim.h"

#include <math.h>
#include <string.h>

#include "gd32sim.h"
#include "parametri.h"
#include "robot.h"

#define MAX_PUNTI_SETTORE 29          /* 5 + 3 x 29 = 92 byte di parametri: il massimo che ci sta */
#define DIST_MIN 0.11

double lidarsim_giri_per_potenza(int power)
{
	if (power < 15)
		return 0;
	return par.lidar_a * power + par.lidar_b;
}

static void accoda(lidarsim_t *l, double t, const void *b, size_t n)
{
	if (l->quanti == LIDARSIM_CODA || n > sizeof l->coda[0].b) {
		l->persi++;
		return;
	}
	lidarsim_uscita_t *u = &l->coda[(l->testa + l->quanti) % LIDARSIM_CODA];
	double tc = t + (l->senza_ritardi ? 0 : parametri_ritardo_arrivo(sim_caso(&l->seme)));
	if (tc < l->ultima_consegna)
		tc = l->ultima_consegna;
	u->t = l->ultima_consegna = tc;
	u->n = (uint16_t)n;
	memcpy(u->b, b, n);
	l->quanti++;
}

static void manda(lidarsim_t *l, double t, uint8_t cmd, const void *param, size_t n)
{
	uint8_t p[LIDAR_MAX_PACKET];
	size_t k = lidar_build_packet(1, 0x61, cmd, param, n, p, sizeof p);
	if (k) {
		accoda(l, t, p, k);
		l->pacchetti++;
	}
}

const lidarsim_uscita_t *lidarsim_prossimo(lidarsim_t *l, double t)
{
	if (!l->quanti || l->coda[l->testa].t > t)
		return NULL;
	const lidarsim_uscita_t *u = &l->coda[l->testa];
	l->testa = (l->testa + 1) % LIDARSIM_CODA;
	l->quanti--;
	return u;
}

double lidarsim_prossima_consegna(const lidarsim_t *l)
{
	return l->quanti ? l->coda[l->testa].t : INFINITY;
}

void lidarsim_init(lidarsim_t *l, double t)
{
	double scarto = l->scarto_gradi;
	uint32_t seme = l->seme;
	int sr = l->senza_ritardi;
	memset(l, 0, sizeof(*l));
	l->scarto_gradi = scarto;
	l->seme = seme ? seme : 88172645u;
	l->senza_ritardi = sr;
	l->t_simulato = t;
	l->ultima_consegna = t;
}

/* Deviazione standard del rumore (m) alla distanza d, dalla tabella misurata per metro. */
static double rumore(double d)
{
	int k = (int)d;
	if (k > 6)
		k = 6;
	return par.lidar_rumore_mm[k] / 1000.0;
}

/* Un settore completo: n punti a partire dall'angolo `s` x 22,5°. */
static void settore(lidarsim_t *l, const mondo_t *m, double t, int s, double x, double y, double dir)
{
	int n = (int)lround(par.lidar_misure_s / (16 * l->rev_s));
	if (n > MAX_PUNTI_SETTORE) n = MAX_PUNTI_SETTORE;
	if (n < 1) n = 1;
	uint8_t p[5 + 3 * MAX_PUNTI_SETTORE];
	int vel = (int)lround(l->rev_s / 0.05);
	p[0] = (uint8_t)(vel > 255 ? 255 : vel);
	p[1] = 0;
	p[2] = 82;                                   /* il campo misterioso "0,82" del lidar vero */
	int inizio = s * 2250;
	p[3] = (uint8_t)(inizio >> 8);
	p[4] = (uint8_t)inizio;
	double lx = x + LIDAR_X * cos(dir) - LIDAR_Y * sin(dir), ly = y + LIDAR_X * sin(dir) + LIDAR_Y * cos(dir);
	for (int i = 0; i < n; i++) {
		double a_letto = (inizio + i * 2250.0 / n) / 100.0;           /* gradi, come li riporta il lidar */
		double ang = dir + M_PI - (a_letto - l->scarto_gradi) * M_PI / 180;   /* 0° del lidar = dietro, orario */
		/* triangolatore: il raggio misurato parte LIDAR_OTTICO a destra del centro di rotazione */
		double ox = lx + LIDAR_OTTICO * sin(ang), oy = ly - LIDAR_OTTICO * cos(ang);
		double d = mondo_raggio(m, ox, oy, ang, LIDAR_ALTEZZA, par.lidar_portata);
		int intensity = 0, quarti = 0;
		int eco = d >= DIST_MIN && d <= par.lidar_portata;
		if (eco && d < 2.0 && sim_caso(&l->seme) < par.lidar_buchi)
			eco = 0;                                                    /* punto isolato senza eco */
		if (eco) {
			d += rumore(d) * sim_gauss(&l->seme);
			quarti = (int)lround(d * 4000);
			intensity = (int)lround(par.lidar_intensita + par.lidar_intensita_dev * sim_gauss(&l->seme));
			if (intensity < 10) intensity = 10;
			if (intensity > 255) intensity = 255;
			if (l->n_corrente < LIDARSIM_MAX_PUNTI) {
				l->cor_x[l->n_corrente] = (float)(ox + d * cos(ang));
				l->cor_y[l->n_corrente] = (float)(oy + d * sin(ang));
				l->n_corrente++;
			}
		}
		p[5 + 3 * i] = (uint8_t)intensity;
		p[6 + 3 * i] = (uint8_t)(quarti >> 8);
		p[7 + 3 * i] = (uint8_t)quarti;
	}
	manda(l, t, LIDAR_SCAN_NEW, p, 5 + 3 * (size_t)n);
	if (s == 15) {
		memcpy(l->giro_x, l->cor_x, sizeof(float) * (size_t)l->n_corrente);
		memcpy(l->giro_y, l->cor_y, sizeof(float) * (size_t)l->n_corrente);
		l->n_giro = l->n_corrente;
		l->n_corrente = 0;
		l->revolutions++;
	}
}

void lidarsim_avanza(lidarsim_t *l, const mondo_t *m, double t, int alimentato, int power,
                     double x, double y, double dir)
{
	if (alimentato && !l->alimentato) {
		l->t_accensione = t;
		l->stato_avvio = 1;
	}
	if (!alimentato) {
		l->stato_avvio = 0;
		l->n_giro = 0;
	}
	l->alimentato = alimentato;
	double last = l->t_simulato;
	if (t < last)
		last = t;
	while (last < t - 1e-9) {
		double dt = fmin(0.001, t - last);
		last += dt;
		double target = alimentato ? lidarsim_giri_per_potenza(power) : 0;
		double tau = target <= 0 ? par.lidar_tau_spento : target > l->rev_s ? par.lidar_tau_su : par.lidar_tau_giu;
		l->rev_s += (target - l->rev_s) * (1 - exp(-dt / tau));
		if (!alimentato)
			continue;
		if (l->stato_avvio == 1 && last - l->t_accensione > 0.3) {
			static const char testo[] = "LDS3 lidar (simulato): avvio\r\n";
			accoda(l, last, testo, sizeof testo - 1);
			l->stato_avvio = 2;
		}
		if (l->stato_avvio == 2 && last - l->t_accensione > 0.5) {
			static const char version[] = "2030313958425713002F0024LDS3_NOR_1805041536";
			manda(l, last, LIDAR_VERSION, version, sizeof version - 1);
			l->stato_avvio = 3;
		}
		if (l->stato_avvio < 3)
			continue;
		if (l->rev_s < 4.0) {
			l->n_corrente = 0;
			if (last >= l->prossimo_ae) {
				uint8_t v = (uint8_t)lround(l->rev_s / 0.05);
				manda(l, last, LIDAR_SPEED, &v, 1);
				l->prossimo_ae = last + 0.1;
			}
			continue;
		}
		int prima = (int)(l->angle / 22.5);
		l->angle += l->rev_s * 360 * dt;
		if (l->angle >= 360)
			l->angle -= 360;
		int dopo = (int)(l->angle / 22.5);
		if (dopo != prima)
			settore(l, m, last, prima, x, y, dir);
	}
	l->t_simulato = last;
}
