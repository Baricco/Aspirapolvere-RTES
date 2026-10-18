/* contatti.c - Registro degli scatti di anti-caduta e bumper (vedi contatti.h). */
#include <string.h>

#include "contatti.h"

void contatti_init(contatti_t *c)
{
	memset(c, 0, sizeof *c);
}

static void azzera(contatti_t *c)
{
	memset(c->conta, 0, sizeof c->conta);
	memset(c->segnato, 0, sizeof c->segnato);
}

int contatti_frame(contatti_t *c, const gd32_sensori_t *s, int64_t t, const drv_posa_t *posa)
{
	int accesi = (s->controllo & GD32_CONTROLLO_SENSORI_ACCESI) != 0;
	int tutti_vuoto = s->cliff[0] && s->cliff[1] && s->cliff[2] && s->cliff[3];
	if (!accesi || s->sulla_base || tutti_vuoto) {        /* i valori non significano niente: si riparte da zero */
		azzera(c);
		return 0;
	}
	int attivo[CONTATTO_SENSORI] = { s->cliff[0], s->cliff[1], s->cliff[2], s->cliff[3], s->bumper_sx, s->bumper_dx };
	int nuovi = 0;
	for (int k = 0; k < CONTATTO_SENSORI; k++) {
		if (!attivo[k]) {
			c->conta[k] = 0;
			c->segnato[k] = 0;
			continue;
		}
		if (c->conta[k]++ == 0) {
			c->t_inizio[k] = t;
			c->ha_posa[k] = posa != NULL;
			if (posa)
				c->posa_inizio[k] = *posa;
		}
		int soglia = k < CONTATTO_BUMPER_SX ? CONTATTI_FRAME_VUOTO : CONTATTI_FRAME_BUMPER;
		if (!c->segnato[k] && c->conta[k] >= soglia) {
			c->segnato[k] = 1;
			uint32_t n = ++c->pub.ultimo;
			drv_contatto_t e = { .numero = n, .sensore = (uint8_t)k, .t_ns = c->t_inizio[k], .posa_valida = c->ha_posa[k] };
			if (c->ha_posa[k])
				e.posa = c->posa_inizio[k];
			c->pub.ev[n % CONTATTI_N] = e;
			nuovi = 1;
		}
	}
	return nuovi;
}
