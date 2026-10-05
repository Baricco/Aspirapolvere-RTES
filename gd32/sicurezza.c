/* sicurezza.c - Blocco di sicurezza dei motori: vedi sicurezza.h. */
#include "sicurezza.h"

#include <string.h>

void sic_init(sicurezza_t *s, int sollevato, int64_t max_eta_ns)
{
	memset(s, 0, sizeof(*s));
	s->sollevato = sollevato;
	s->max_eta_ns = max_eta_ns;
	s->cliff_atteso = GD32_CLIFF_TUTTI_VUOTO;
}

void sic_cliff_atteso(sicurezza_t *s, uint8_t cliff)
{
	s->cliff_atteso = cliff & 0x0f;
}

int sic_e_motore(uint8_t cmd)
{
	switch (cmd) {
	case GD32_VELOCITA:
	case GD32_VELOCITA_RUOTE:
	case GD32_ASPIRAZIONE:
	case GD32_SPAZZOLA_LATERALE:
	case GD32_SPAZZOLA_CENTRALE:
	case GD32_ASPIRAZIONE_MODO:
	case GD32_ASPIRAZIONE_PWM:
	case GD32_ASPIRAZIONE_PWM_BAT:
		return 1;
	default:
		return 0;
	}
}

int sic_e_vietato(uint8_t cmd)
{
	return cmd == GD32_POMPA_ACQUA || cmd == GD32_ALIMENTAZIONE_SOC || cmd == GD32_RIAVVIA_SOC;
}

static int scatta(sicurezza_t *s, const char *motivo, int64_t adesso)
{
	if (s->scattato)
		return 0;
	s->scattato = 1;
	s->motivo = motivo;
	s->quando = adesso;
	return 1;
}

/* Il frame dice "sollevato": anti-caduta come atteso (di regola tutti su vuoto), sensori alimentati, robot non
 * sulla base. */
static int frame_sollevato(const sicurezza_t *s, const uint8_t *d)
{
	return (d[3] & 0x0f) == s->cliff_atteso && (d[64] & GD32_CONTROLLO_SENSORI_ACCESI) && (d[7] & 3) == 0;
}

int sic_ricevuto(sicurezza_t *s, const gd32_frame_t *f, int64_t adesso)
{
	if (f->cmd != GD32_SENSORI || f->n_dati < 96)
		return 0;
	s->ha_sensori = 1;
	s->cliff = f->dati[3];
	s->sollevato_ora = frame_sollevato(s, f->dati);
	s->ultimo_sensori = adesso;
	if (s->sollevato_ora) {
		s->visto_vuoto = 1;
		return 0;
	}
	if (s->visto_vuoto)
		return scatta(s, "il robot non risulta più sollevato (anti-caduta, sensori spenti o sulla base)", adesso);
	return 0;
}

int sic_controlla(sicurezza_t *s, int64_t adesso)
{
	if (s->visto_vuoto && adesso - s->ultimo_sensori > s->max_eta_ns)
		return scatta(s, "frame dei sensori assenti", adesso);
	return 0;
}

static int tutti_zero(const uint8_t *d, size_t n)
{
	for (size_t i = 0; i < n; i++)
		if (d[i])
			return 0;
	return 1;
}

sic_esito_t sic_permesso(sicurezza_t *s, uint8_t cmd, const uint8_t *dati, size_t n, int64_t adesso)
{
	if (sic_e_vietato(cmd)) {
		s->vietati++;
		return SIC_VIETATO;
	}
	if (!sic_e_motore(cmd) || tutti_zero(dati, n))
		return SIC_PERMESSO;
	/* motore diverso da zero: servono tutte le condizioni */
	sic_controlla(s, adesso);
	if (s->sollevato && !s->scattato && s->ha_sensori && s->sollevato_ora &&
	    adesso - s->ultimo_sensori <= s->max_eta_ns)
		return SIC_PERMESSO;
	s->rifiutati++;
	return SIC_RIFIUTATO;
}

size_t sic_frame_arresto(uint8_t *out, size_t dim)
{
	static const uint8_t zero[8];
	static const struct { uint8_t cmd, n; } arresto[] = {
		{ GD32_VELOCITA, 8 }, { GD32_VELOCITA_RUOTE, 8 }, { GD32_SPAZZOLA_LATERALE, 1 },
		{ GD32_SPAZZOLA_CENTRALE, 1 }, { GD32_ASPIRAZIONE, 2 }, { GD32_ASPIRAZIONE_PWM, 1 },
	};
	size_t tot = 0;
	for (size_t i = 0; i < sizeof arresto / sizeof arresto[0]; i++) {
		size_t k = gd32_componi(arresto[i].cmd, zero, arresto[i].n, out + tot, dim - tot);
		if (!k)
			return 0;
		tot += k;
	}
	return tot;
}

int sic_terra_limita(const gd32_sensori_t *s, int uscita_base, int32_t *v_tick, int32_t *w_tick)
{
	/* limiti per uscire dalla base: 0,1 m/s e 0,5 rad/s nelle unità di 0x66 */
	const int32_t V_BASE = 1656, W_BASE = 1000;
	int sensori = (s->controllo & GD32_CONTROLLO_SENSORI_ACCESI) != 0;
	int cliff = s->cliff[0] | s->cliff[1] | s->cliff[2] | s->cliff[3];
	int tutti_vuoto = s->cliff[0] & s->cliff[1] & s->cliff[2] & s->cliff[3];
	int motivi = 0;

	if (s->sulla_base) {
		motivi |= SIC_LIM_BASE;
		if (!uscita_base) {
			*v_tick = *w_tick = 0;
			return motivi;
		}
		if (*v_tick > V_BASE) *v_tick = V_BASE;
		if (*v_tick < -V_BASE) *v_tick = -V_BASE;
		if (*w_tick > W_BASE) *w_tick = W_BASE;
		if (*w_tick < -W_BASE) *w_tick = -W_BASE;
		return motivi;      /* sulla base anti-caduta e bumper non dicono niente (0f, sensori spenti) */
	}
	if (sensori && tutti_vuoto) {
		*v_tick = *w_tick = 0;
		return SIC_LIM_SOLLEVATO;
	}
	if (!sensori)
		motivi |= SIC_LIM_SENSORI;
	else {
		if (s->bumper_sx || s->bumper_dx)
			motivi |= SIC_LIM_BUMPER;
		if (cliff)
			motivi |= SIC_LIM_CLIFF;
	}
	if (motivi && *v_tick > 0)
		*v_tick = 0;
	return motivi;
}
