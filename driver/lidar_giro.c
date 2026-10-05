/* lidar_giro.c - Giri completi del lidar e regolazione della velocità: vedi lidar_giro.h. */
#include "lidar_giro.h"

#include <string.h>

void lidar_assemblatore_init(lidar_assemblatore_t *a, int scarto_cdeg)
{
	memset(a, 0, sizeof(*a));
	a->ultimo_inizio = -1;
	a->scarto_cdeg = scarto_cdeg;
}

static uint16_t u16be(const uint8_t *b) { return (uint16_t)(b[0] << 8 | b[1]); }

int lidar_assembla(lidar_assemblatore_t *a, const lidar_pacchetto_t *p, int64_t t, lidar_giro_t *giro)
{
	if (p->cmd == LIDAR_VELOCITA && p->n_param >= 1) {
		a->giri_s = p->param[0] * 0.05;
		a->t_velocita = t;
		return 0;
	}
	if (p->cmd != LIDAR_SCANSIONE_NUOVA || p->n_param < 5)
		return 0;
	double vel = p->param[0] * 0.05;
	int inizio = u16be(p->param + 3);
	int n = (p->n_param - 5) / 3;
	a->giri_s = vel;
	a->t_velocita = t;
	a->settori++;

	int consegnato = 0;
	lidar_giro_t *c = &a->corrente;
	if (a->ultimo_inizio >= 0 && inizio <= a->ultimo_inizio) {
		/* l'angolo è tornato indietro: il giro precedente è finito */
		if (c->settori >= 12) {
			c->giri_s = c->settori ? a->somma_vel / c->settori : 0;
			c->numero = ++a->giri;
			memcpy(giro, c, sizeof(*c) - sizeof(c->p) + sizeof(c->p[0]) * c->n);
			consegnato = 1;
		} else {
			a->giri_scartati++;
		}
		c->n = 0;
		c->settori = 0;
		a->somma_vel = 0;
	}
	if (c->settori == 0)
		c->t_inizio_ns = t;
	c->t_fine_ns = t;
	c->settori++;
	a->somma_vel += vel;
	a->ultimo_inizio = inizio;
	for (int i = 0; i < n && c->n < LIDAR_GIRO_MAX_PUNTI; i++) {
		const uint8_t *q = p->param + 5 + 3 * i;
		int ang = inizio + (i * 2250) / n - a->scarto_cdeg;
		ang %= 36000;
		if (ang < 0)
			ang += 36000;
		lidar_punto_t *pt = &c->p[c->n++];
		pt->angolo_cdeg = (uint16_t)ang;
		pt->intensita = q[0];
		pt->distanza_mm = (uint16_t)((u16be(q + 1) + 2) / 4);
	}
	return consegnato;
}

void lidar_regolatore_init(lidar_regolatore_t *r, double obiettivo)
{
	memset(r, 0, sizeof(*r));
	r->obiettivo = obiettivo;
}

int lidar_regola(lidar_regolatore_t *r, double misurata)
{
	double e = r->obiettivo - misurata;
	r->integrale += e;
	if (r->integrale > 20)
		r->integrale = 20;
	if (r->integrale < -20)
		r->integrale = -20;
	double p = 60 + r->integrale + 10 * e + 10 * (e - r->errore_prec);
	r->errore_prec = e;
	if (r->obiettivo <= 0)
		p = 0;
	r->potenza = p < 0 ? 0 : p > 100 ? 100 : (int)(p + 0.5);
	return r->potenza;
}
