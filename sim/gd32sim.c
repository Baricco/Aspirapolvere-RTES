/* gd32sim.c - Il GD32 simulato: vedi gd32sim.h. */
#include "gd32sim.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "parametri.h"

#define TICK_M (1.0 / SCALA_V)                            /* metri per tick (~218 µm): 0x66 conta tick/s */
#define V_MAX_TICK (V_MAX * SCALA_V)
#define W_MAX_TICK (W_MAX * SCALA_W)

/* ---- numeri casuali (xorshift32: ripetibili con lo stesso seme) ---- */
double sim_caso(uint32_t *seme)
{
	uint32_t x = *seme ? *seme : 2463534242u;
	x ^= x << 13;
	x ^= x >> 17;
	x ^= x << 5;
	*seme = x;
	return (x >> 8) / 16777216.0;
}

double sim_gauss(uint32_t *seme)
{
	double s = 0;
	for (int i = 0; i < 6; i++)
		s += sim_caso(seme);
	return (s - 3) * 1.4142;
}

static double angolo_norm(double a)
{
	while (a > M_PI) a -= 2 * M_PI;
	while (a < -M_PI) a += 2 * M_PI;
	return a;
}

/* ---- batteria: curva di scarica misurata (fase 2, C8); soc 1 = 16,3 V, 0 = 13,8 V ---- */
static const double CURVA_SOC[] = { 0.00, 0.06, 0.20, 0.38, 0.59, 0.80, 0.94, 1.00 };
static const double CURVA_V[]   = { 13.80, 14.00, 14.20, 14.50, 15.00, 15.50, 15.90, 16.30 };

static double tensione_a_vuoto(double soc)
{
	if (soc < 0) {
		/* riserva sotto 13,8 V fino alla protezione (~12 V) */
		double riserva = (par.batteria_totale_ah - par.batteria_utile_ah) / par.batteria_utile_ah;
		return 13.8 + 1.8 * fmax(soc / riserva, -1.0);
	}
	int n = (int)(sizeof CURVA_SOC / sizeof CURVA_SOC[0]);
	for (int i = 1; i < n; i++)
		if (soc <= CURVA_SOC[i]) {
			double f = (soc - CURVA_SOC[i - 1]) / (CURVA_SOC[i] - CURVA_SOC[i - 1]);
			return CURVA_V[i - 1] + f * (CURVA_V[i] - CURVA_V[i - 1]);
		}
	return CURVA_V[n - 1];
}

/* ---- uscita: coda con l'istante di consegna (ritardo misurato) ---- */
static void manda(gd32sim_t *g, double t, uint8_t cmd, const void *data, size_t n)
{
	if (g->quanti == SIM_CODA) {
		g->persi++;
		return;
	}
	sim_uscita_t *u = &g->coda[(g->testa + g->quanti) % SIM_CODA];
	size_t k = gd32_build_frame(cmd, data, n, u->b, sizeof u->b);
	if (!k)
		return;
	double tc = t + (g->senza_ritardi ? 0 : parametri_ritardo_arrivo(sim_caso(&g->seme)));
	if (tc < g->ultima_consegna)
		tc = g->ultima_consegna;            /* la seriale non riordina i byte */
	u->t = g->ultima_consegna = tc;
	u->n = (uint16_t)k;
	g->quanti++;
}

const sim_uscita_t *gd32sim_prossimo(gd32sim_t *g, double t)
{
	if (!g->quanti || g->coda[g->testa].t > t)
		return NULL;
	const sim_uscita_t *u = &g->coda[g->testa];
	g->testa = (g->testa + 1) % SIM_CODA;
	g->quanti--;
	return u;
}

double gd32sim_prossima_consegna(const gd32sim_t *g)
{
	return g->quanti ? g->coda[g->testa].t : INFINITY;
}

static void avvisa(gd32sim_t *g, double t, const char *testo)
{
	snprintf(g->avviso, sizeof g->avviso, "%s", testo);
	g->t_avviso = t;
}

static void stato_firmware_iniziale(gd32sim_t *g)
{
	g->mode = 0;
	g->cmd_v = g->cmd_w = 0;
	g->cmd_ruote[0] = g->cmd_ruote[1] = 0;
	g->n_coda_cmd = 0;
	g->spazzola_lat = g->spazzola_cen = g->eco_lat = g->eco_cen = 0;
	g->fine_sblocco = 0;
	g->suction_rpm = g->aspirazione_pwm = 0;
	g->led = 12;                  /* azzurro che respira, come all'accensione */
	g->sensori_accesi = 1;
	g->cliff_retromarcia = 0;
	g->cliff_ir = 0;
	g->lidar_alimentato = 0;
	g->lidar_power = 0;
	g->sospeso = g->risveglio = 0;
	g->ultimo_hb = -100;
	g->motori_alimentati = 0;
	g->risposta_08 = 0;
	g->imbardata = 0;
}

void gd32sim_init(gd32sim_t *g, const mondo_t *m, double t)
{
	uint32_t seme = g->seme;
	double vb = g->velocita_batteria, tka = g->timeout_keepalive;
	int sr = g->senza_ritardi;
	memset(g, 0, sizeof(*g));
	g->seme = seme ? seme : 12345;
	g->velocita_batteria = vb > 0 ? vb : 1;
	g->timeout_keepalive = tka > 0 ? tka : 190;
	g->senza_ritardi = sr;
	g->x = m->robot_x;
	g->y = m->robot_y;
	g->dir = m->robot_dir;
	g->dist_sx = 27000;           /* gli encoder non partono da zero (valori come sul robot) */
	g->dist_dx = 26989;
	g->enc_sx = 27000;
	g->enc_dx = 26989;
	for (int i = 0; i < SIM_FINESTRA; i++) {
		g->storia_enc[i][0] = g->enc_sx;
		g->storia_enc[i][1] = g->enc_dx;
	}
	g->soc = 0.90;
	g->tensione = tensione_a_vuoto(g->soc);
	g->dustbin = 1;
	g->tasto_da[0] = g->tasto_da[1] = -1;
	g->t_contatto = -1;
	g->t_urto = g->t_incastro = -1;
	g->on = 1;
	if (!par.frame_riposo[88])
		parametri_frame_default();
	stato_firmware_iniziale(g);
	g->ultimo_ka = t;
	g->prossimo_sensori = t + par.gd32_periodo_ms / 1000;
	g->prossimo_hb_gd32 = t + 2.0;
	g->prossimo_enc = t;
	g->t_simulato = t;
	g->ultima_consegna = t;
}

void gd32sim_sposta(gd32sim_t *g, double x, double y, double dir)
{
	g->x = x;
	g->y = y;
	g->dir = angolo_norm(dir);
	g->caduto = 0;
	g->bloccato = g->incastrato = 0;
	g->t_urto = g->t_incastro = -1;
}

void gd32sim_velocita(const gd32sim_t *g, double *v, double *w)
{
	int fermo = g->lifted || g->caduto;
	*v = fermo ? 0 : (g->v_measured + g->w_measured) / 2 * TICK_M;
	*w = fermo ? 0 : (g->w_measured - g->v_measured) * TICK_M / CARREGGIATA;
}

/* ---- comandi dal SoC ---- */
static int32_t i32(const uint8_t *b) { return (int32_t)((uint32_t)b[0] | (uint32_t)b[1] << 8 | (uint32_t)b[2] << 16 | (uint32_t)b[3] << 24); }

static void storia(gd32sim_t *g, const gd32_frame_t *f, double t)
{
	if (f->cmd == GD32_HEARTBEAT || f->cmd == GD32_KEEPALIVE)
		return;
	if (f->cmd == GD32_VELOCITY && g->n_storia) {
		const sim_cmd_log_t *u = &g->storia[(g->i_storia + SIM_STORIA - 1) % SIM_STORIA];
		if (u->cmd == f->cmd && !memcmp(u->data, f->data, 8))
			return;
	}
	sim_cmd_log_t *s = &g->storia[g->i_storia];
	s->t = t;
	s->cmd = f->cmd;
	s->n = f->data_len;
	memcpy(s->data, f->data, f->data_len < sizeof s->data ? f->data_len : sizeof s->data);
	g->i_storia = (g->i_storia + 1) % SIM_STORIA;
	if (g->n_storia < SIM_STORIA)
		g->n_storia++;
}

/* I comandi dei motori valgono dopo il ritardo misurato (il GD32 li applica al suo ciclo di controllo). */
static void accoda_motore(gd32sim_t *g, double t, int mode, int32_t a, int32_t b)
{
	if (g->n_coda_cmd == SIM_COMANDI) {
		memmove(g->coda_cmd, g->coda_cmd + 1, sizeof g->coda_cmd[0] * (SIM_COMANDI - 1));
		g->n_coda_cmd--;
	}
	g->coda_cmd[g->n_coda_cmd++] = (sim_comando_t){ t + par.ruote_ritardo_ms / 1000, mode, a, b };
}

void gd32sim_ricevi(gd32sim_t *g, const gd32_frame_t *f, double t)
{
	const uint8_t *d = f->data;
	int n = f->data_len;
	g->frame_ricevuti++;
	g->rx_per_cmd[f->cmd]++;
	storia(g, f, t);
	if (!g->on)
		return;
	if (g->sospeso && f->cmd != GD32_WAKEUP_ACK && f->cmd != GD32_KEEPALIVE)
		return;
	switch (f->cmd) {
	case GD32_SLEEP:
		manda(g, t, GD32_SLEEP, NULL, 0);
		g->sospeso = 1;
		g->mode = 0;
		avvisa(g, t, "il GD32 è in sospensione (0x04): si sveglia con il tasto di accensione");
		break;
	case GD32_WAKEUP_ACK:
		if (g->sospeso || g->risveglio) {
			g->sospeso = g->risveglio = 0;
			g->prossimo_sensori = t + par.gd32_periodo_ms / 1000;
			avvisa(g, t, "risveglio confermato (0x05)");
		}
		break;
	case GD32_HEARTBEAT:
		g->ultimo_hb = t;
		break;
	case GD32_VERSION: {
		uint8_t v[32] = { 0 };
		memcpy(v, "2.0.1_19082727", 14);
		v[20] = 1;
		manda(g, t, GD32_VERSION, v, sizeof v);
		break;
	}
	case GD32_IMU_ZERO:
		g->imbardata = 0;
		g->risposta_08 = t + 0.2;
		break;
	case GD32_SOC_WATCHDOG:
		manda(g, t, GD32_SOC_WATCHDOG, d, n > 0 ? 1 : 0);
		break;
	case GD32_KEEPALIVE:
		g->ultimo_ka = t;
		break;
	case GD32_IMU_RAW: {
		uint8_t uno = 1;
		manda(g, t, GD32_IMU_RAW, &uno, 1);
		break;
	}
	case GD32_MACHINE_FUNCTIONS: {
		static const uint8_t r[8] = { 4, 0, 0, 0, 0x3f, 0, 0, 0 };
		manda(g, t, GD32_MACHINE_FUNCTIONS, r, sizeof r);
		break;
	}
	case GD32_CLIFF_BACKOFF:
		if (n >= 1) g->cliff_retromarcia = d[0] != 0;
		break;
	case GD32_CONTROL_TYPE:
		if (n >= 1) accoda_motore(g, t, d[0], 0, 0);
		break;
	case GD32_VELOCITY:
		if (n >= 8) accoda_motore(g, t, -1, i32(d), i32(d + 4));
		break;
	case GD32_WHEEL_SPEED:
		if (n >= 8) accoda_motore(g, t, -2, i32(d), i32(d + 4));
		break;
	case GD32_SUCTION:
		if (n >= 2) {
			g->suction_rpm = (int16_t)(d[0] | d[1] << 8) * 10;
			g->aspirazione_pwm = 0;
		}
		break;
	case GD32_SUCTION_PWM:
	case GD32_SUCTION_PWM_BATTERY:
		if (n >= 1) {
			g->aspirazione_pwm = d[0] > 100 ? 100 : d[0];
			g->suction_rpm = 0;
		}
		break;
	case GD32_SIDE_BRUSH:
		if (n >= 1) g->spazzola_lat = (int8_t)d[0];
		break;
	case GD32_MAIN_BRUSH:
		if (n >= 1) {
			g->spazzola_cen = (int8_t)d[0];
			if (g->spazzola_cen < 0)
				g->fine_sblocco = t + 4.2;
		}
		break;
	case GD32_WATER_PUMP:
		if (n >= 1 && d[0])
			avvisa(g, t, "ATTENZIONE: comando alla pompa dell'acqua (0x6b), che sul robot non c'è");
		break;
	case GD32_LIDAR_MOTOR:
		if (n >= 4) {
			int32_t p = i32(d);
			g->lidar_power = p < 0 ? 0 : p > 100 ? 100 : p;
		}
		break;
	case GD32_CLIFF_IR:
		if (n >= 1) g->cliff_ir = d[0] != 0;
		break;
	case GD32_BUTTON_LED:
		if (n >= 1) g->led = d[0];
		break;
	case GD32_LIDAR_POWER:
		if (n >= 1) g->lidar_alimentato = d[0] != 0;
		break;
	case GD32_SOC_POWER:
	case GD32_SOC_RESTART:
		avvisa(g, t, "PERICOLO: comando di alimentazione/riavvio del SoC (0x99/0x9a): sul robot vero è vietato");
		break;
	case GD32_SENSOR_POWER:
		if (n >= 1) g->sensori_accesi = d[0] != 0;
		break;
	case GD32_IMU_CALIBRATION_STATE: {
		uint8_t zero = 0;
		manda(g, t, GD32_IMU_CALIBRATION_STATE, &zero, 1);
		break;
	}
	default:
		break;
	}
}

/* ---- sensori ---- */
static void metti16(uint8_t *b, int v) { b[0] = (uint8_t)v; b[1] = (uint8_t)(v >> 8); }
static void metti32(uint8_t *b, int32_t v) { for (int i = 0; i < 4; i++) b[i] = (uint8_t)((uint32_t)v >> (8 * i)); }
static int satura16(double v) { return v > 32767 ? 32767 : v < -32768 ? -32768 : (int)lround(v); }

/* IR della base (byte 6) [misura del 6/10, prova ir1, docs/fase3.md; parziale]. La base manda 4 fasci codificati
 * (bit 1, 2, 4, 8); ogni ricevitore del robot riporta quelli che lo raggiungono (everest prende il bit più basso:
 * unpacketDockIR). fi = posizione del ricevitore vista dalla base, rispetto alla sua normale (+ = a destra del robot
 * che guarda la base), d = distanza dalla base:
 *   fascio 1: |fi - 5°| < 20°, da 0,5 a ~2,5 m (sull'asse si vede da solo a 1,6 m; vicinissimo quasi sparisce);
 *   fascio 2: |fi + 12°| < 20°, fino a ~1,3 m; sotto 0,7 m anche |fi| < 35° (vicino alla base domina lui);
 *   fascio 4: fi da +8° a +45°, da 0,7 a 2 m (visto a 0,2 m dall'asse, a 1,1 m; mai vicino alla base);
 *   fascio 8: fi da -60° a -25° [?: mai visto stanotte, per simmetria con il 4].
 * I due ricevitori sono davanti: nibble BASSO = sinistro, ALTO = destro. Ognuno vede la base solo se è nel suo campo:
 * il sinistro da -3° a +38°, il destro da -16° a +30° (angolo della base rispetto al davanti, + = a sinistra). */
static uint8_t fasci_base(const mondo_t *m, double x, double y)
{
	double dx = x - m->base_x, dy = y - m->base_y, d = hypot(dx, dy);
	double fi = angolo_norm(atan2(dy, dx) - m->base_dir) * 180 / M_PI;
	uint8_t f = 0;
	/* da vicino (misure r13 e r15, lidar a ~40 cm dalla faccia, cioè ricevitori a ~15 cm): sull'asse ogni ricevitore vede i
	   fasci 1 e 2 insieme; più lontano l'asse è il confine fra i due (a ~60 cm: 11 o 22, a volte 33) */
	if (d < 0.35 && fabs(fi) < 25) f |= 3;
	if ((fabs(fi - 5) < 20 && d >= 0.8 && d < 2.5) || (fi >= 0 && fi < 40 && d < 0.8)) f |= 1;
	if ((fabs(fi + 12) < 20 && d >= 0.8 && d < 1.3) || (fi < 0 && fi > -40 && d < 0.8)) f |= 2;
	if (fi > 8 && fi < 45 && d > 0.7 && d < 2.0) f |= 4;
	if (fi > -60 && fi < -25 && d < 2.0) f |= 8;
	return f;
}

static uint8_t calcola_ir_base(const gd32sim_t *g, const mondo_t *m)
{
	if (!m->ha_base || !g->sensori_accesi)
		return 0;
	double dx = g->x - m->base_x, dy = g->y - m->base_y, d = hypot(dx, dy);
	if (d > 3.0 || d < 0.05)
		return 0;
	double px = m->base_x + 0.01 * cos(m->base_dir), py = m->base_y + 0.01 * sin(m->base_dir);
	if (mondo_raggio(m, px, py, atan2(dy, dx), 0.05, d) < d - ROBOT_RAGGIO)
		return 0;                                   /* qualcosa fra la base e il robot */
	uint8_t nib[2] = { 0, 0 };
	for (int r = 0; r < 2; r++) {                   /* 0 = sinistro (nibble basso), 1 = destro (alto) */
		double ly = r == 0 ? 0.08 : -0.08;          /* ricevitori sul davanti, ~8 cm a lato [?] */
		double rx = g->x + 0.15 * cos(g->dir) - ly * sin(g->dir), ry = g->y + 0.15 * sin(g->dir) + ly * cos(g->dir);
		double guarda = angolo_norm(atan2(m->base_y - ry, m->base_x - rx) - g->dir) * 180 / M_PI;
		int vede = r == 0 ? (guarda > -3 && guarda < 38) : (guarda > -16 && guarda < 30);
		if (vede)
			nib[r] = fasci_base(m, rx, ry);
	}
	return (uint8_t)(nib[1] << 4 | nib[0]);
}

/* Velocità misurata dal GD32 (byte 32 e 36): dagli encoder, nella finestra di ~70 ms. */
static void velocita_misurata(const gd32sim_t *g, int32_t *v, int32_t *w)
{
	int k = (int)lround(par.ruote_finestra_ms / 10);
	if (k < 1) k = 1;
	if (k > SIM_FINESTRA - 1) k = SIM_FINESTRA - 1;
	int io = (g->i_enc + SIM_FINESTRA - 1) % SIM_FINESTRA, ip = (g->i_enc + SIM_FINESTRA - 1 - k) % SIM_FINESTRA;
	const int32_t *ora = g->storia_enc[io], *prima = g->storia_enc[ip];
	double dt = g->storia_t[io] - g->storia_t[ip];
	if (dt < 0.005)
		dt = k * 0.010;
	double dsx = ora[0] - prima[0], ddx = ora[1] - prima[1];
	*v = (int32_t)lround((dsx + ddx) / 2 / dt);
	*w = (int32_t)lround((ddx - dsx) / 2 / dt);
}

void gd32sim_dati_sensori(const gd32sim_t *g, double t, uint8_t d[96])
{
	uint32_t seme = g->seme ^ (uint32_t)(t * 1000);   /* rumore del frame: non tocca il generatore della fisica */
	memcpy(d, par.frame_riposo, 96);
	d[0] = 1;
	d[1] = g->sensori_accesi ? (uint8_t)((g->bumper_left ? 0x04 : 0) | (g->bumper_right ? 0x02 : 0)) : 0x06;
	d[3] = g->sensori_accesi ? g->cliff : 0;
	d[4] = (uint8_t)((d[4] & ~0x04) | (g->dustbin ? 0x04 : 0));
	d[6] = g->ir_base;
	d[7] = (uint8_t)((g->contatto_base ? 2 : 0) | (g->charging ? 1 : 0));
	double v10 = g->tensione * 10;
	d[8] = (uint8_t)(v10 < 0 ? 0 : v10 > 255 ? 255 : lround(v10));
	d[9] = (uint8_t)(int8_t)lround(par.temperature_c + fabs(g->current) * 0.5);
	metti32(d + 16, g->enc_sx);
	metti32(d + 24, g->enc_dx);
	int32_t vm = 0, wm = 0;
	if (g->mode == 2)
		velocita_misurata(g, &vm, &wm);
	metti32(d + 32, vm);
	metti32(d + 36, wm);
	int vel_imb = satura16(g->vel_imbardata * 180 / M_PI * 100 + par.imu_rumore_velocita * sim_gauss(&seme));
	int vel_bec = satura16(par.imu_rumore_velocita * sim_gauss(&seme));
	int vel_rol = satura16(par.imu_rumore_velocita * sim_gauss(&seme));
	metti16(d + 40, vel_imb);
	metti16(d + 42, satura16(angolo_norm(g->imbardata) * 180 / M_PI * 100));
	metti16(d + 44, vel_bec);
	metti16(d + 46, par.imu_beccheggio);
	metti16(d + 48, vel_rol);
	metti16(d + 50, par.imu_rollio);
	for (int k = 0; k < 2; k++) {
		int ms = 0;
		if (g->tasto_da[k] >= 0)
			ms = (int)fmin(65535, (t - g->tasto_da[k]) * 100);
		metti16(d + (k ? 62 : 58), ms);
	}
	d[64] = (uint8_t)(0x42 | (g->sensori_accesi ? 0x20 : 0) | (g->lidar_alimentato && g->on ? 0x80 : 0));
	d[65] = g->led;
	d[67] = (uint8_t)(int8_t)g->eco_lat;
	d[68] = (uint8_t)g->eco_cen;
	double vg = g->ventola_giri / 100;
	d[69] = (uint8_t)(vg > 255 ? 255 : lround(vg));
	/* correnti (mA): 74 ruota sx, 76 ruota dx, 78 spazzola laterale, 80 spazzola centrale, 82 aspirazione */
	double vr[2] = { g->v_measured, g->w_measured };
	for (int k = 0; k < 2; k++) {
		double i = 0;
		if (fabs(vr[k]) > 5) {
			i = par.corrente_ruota_base * (0.7 + 0.6 * sim_caso(&seme));
			if (sim_caso(&seme) < 0.10)
				i += (par.corrente_ruota_picchi - par.corrente_ruota_base) * sim_caso(&seme);
			if (t - g->t_avvio_ruote < 0.06)
				i = par.corrente_ruota_avvio;
			if ((g->t_urto >= 0 && t - g->t_urto < 0.1) || (g->t_incastro > 0 && t - g->t_incastro < 0.1))
				i = 700 + 0.2 * fabs(vr[k]);                 /* [?] motore sotto sforzo contro un ostacolo */
		}
		metti16(d + 74 + 2 * k, (int)i);
	}
	metti16(d + 78, par.corrente_riposo_laterale + (g->eco_lat ? 30 : 0));     /* [?] laterale non montata sul robot */
	double ic = par.corrente_riposo_centrale;
	if (g->eco_cen > 0) ic = par.corrente_centrale_accesa + 10 * sim_gauss(&seme);
	if (g->spazzola_cen < 0 && g->eco_cen) ic = par.corrente_centrale_sblocco;
	metti16(d + 80, (int)ic);
	double ia = par.corrente_riposo_aspirazione +
	            (par.aspirazione_corrente_max - par.corrente_riposo_aspirazione) *
	            pow(fmax(0, g->ventola_giri) / par.aspirazione_giri_max, par.aspirazione_esponente);
	metti16(d + 82, (int)ia);
	metti16(d + 84, par.byte84);
	metti16(d + 90, vel_imb);
	metti16(d + 92, vel_bec);
	metti16(d + 94, vel_rol);
}

/* ---- simulazione ---- */
static void reset_gd32(gd32sim_t *g, double t)
{
	g->reset++;
	g->on = 0;
	g->t_riaccensione = t + 5;
	stato_firmware_iniziale(g);
	avvisa(g, t, "RESET DEL GD32: troppo tempo senza keepalive 0x0d. Sul robot vero si riavvia TUTTO, anche Linux");
}

/* Avvicina att a obiettivo con le rampe misurate (su = accelerando, giù = rallentando o invertendo). */
static double rampa(double att, double target, double su, double giu, double dt)
{
	double r = (att * target >= 0 && fabs(target) > fabs(att)) ? su : giu;
	double diff = target - att, passo = r * dt;
	return fabs(diff) <= passo ? target : att + (diff > 0 ? passo : -passo);
}

static double satura(double v, double m) { return v > m ? m : v < -m ? -m : v; }

static void passo(gd32sim_t *g, const mondo_t *m, double t, double dt)
{
	/* firmware: reset, sorveglianze, comandi arrivati */
	if (!g->on && t >= g->t_riaccensione && g->soc > -0.2) {
		g->on = 1;
		g->ultimo_ka = t;
		g->prossimo_sensori = t + par.gd32_periodo_ms / 1000;
		g->prossimo_hb_gd32 = t + 2.0;
	}
	if (g->on && t - g->ultimo_ka > g->timeout_keepalive)
		reset_gd32(g, t);
	g->motori_alimentati = g->on && !g->sospeso && t - g->ultimo_hb < par.ruote_timeout_hb;
	if (g->spazzola_cen < 0 && t >= g->fine_sblocco)
		g->spazzola_cen = 0;
	while (g->n_coda_cmd && g->coda_cmd[0].t <= t) {
		sim_comando_t c = g->coda_cmd[0];
		memmove(g->coda_cmd, g->coda_cmd + 1, sizeof g->coda_cmd[0] * (size_t)(g->n_coda_cmd - 1));
		g->n_coda_cmd--;
		if (c.mode == -1) { g->cmd_v = c.a; g->cmd_w = c.b; }
		else if (c.mode == -2) { g->cmd_ruote[0] = c.a; g->cmd_ruote[1] = c.b; }
		else g->mode = c.mode;
	}

	/* anti-caduta e base sulla posizione attuale */
	uint8_t cliff = 0;
	for (int k = 0; k < 4; k++) {
		double a = g->dir + CLIFF_ANGOLI[k];
		if (g->lifted || g->caduto || mondo_vuoto(m, g->x + CLIFF_DISTANZA * cos(a), g->y + CLIFF_DISTANZA * sin(a)))
			cliff |= (uint8_t)(1 << k);
	}
	int contatto = 0;
	if (m->ha_base && !g->lifted) {
		/* contatti dietro: il punto più arretrato del robot sul lato dei contatti della base, robot che le dà le spalle */
		double dc = g->dir + (CONTATTI_DIETRO ? M_PI : 0);
		double cx = g->x + ROBOT_RAGGIO * cos(dc), cy = g->y + ROBOT_RAGGIO * sin(dc);
		contatto = hypot(cx - m->base_x, cy - m->base_y) < 0.03 &&
		           fabs(angolo_norm(dc - (m->base_dir + M_PI))) < 15 * M_PI / 180;
	}
	if (contatto && !g->contatto_base)
		g->t_contatto = t;
	g->contatto_base = contatto;
	if (contatto)
		cliff = g->sensori_accesi ? 0 : 0x0f;   /* spenti leggono 0f (fase 2, C5); accesi vedono il pavimento (prova o2) */
	g->cliff = cliff;
	g->ir_base = g->on ? calcola_ir_base(g, m) : 0;
	g->charging = contatto && g->on && t - g->t_contatto > 12 && g->soc < 1.0;

	/* ruote: obiettivi in tick/s per ruota, poi le rampe misurate */
	double tv = 0, tw = 0, tsx = 0, tdx = 0;
	int per_ruota = 0;
	if (g->motori_alimentati && g->mode == 2) {
		tv = satura(g->cmd_v, V_MAX_TICK);
		tw = satura(g->cmd_w, W_MAX_TICK);
		/* con i sensori spenti (0x9d 00, come sulla base) gli anti-caduta non contano: è così che everest esce dalla
		 * base, dove leggono 0f [?: da verificare sul robot] */
		if ((cliff & 0x06) && tv > 0 && g->sensori_accesi)
			tv = g->cliff_retromarcia ? -tv / 2 : 0;
	} else if (g->motori_alimentati && g->mode == 1) {
		per_ruota = 1;
		tsx = satura(g->cmd_ruote[0], V_MAX_TICK);
		tdx = satura(g->cmd_ruote[1], V_MAX_TICK);
	}
	double prima = fabs(g->v_measured) + fabs(g->w_measured);
	if (per_ruota) {
		g->v_measured = rampa(g->v_measured, tsx, par.rampa_v_su, par.rampa_v_giu, dt);
		g->w_measured = rampa(g->w_measured, tdx, par.rampa_v_su, par.rampa_v_giu, dt);
		g->v_cmd = (g->v_measured + g->w_measured) / 2;
		g->w_cmd = (g->w_measured - g->v_measured) / 2;
	} else if (g->motori_alimentati && g->mode == 2 && !g->lifted && !g->caduto) {
		/* a terra il GD32 regola v e omega con una risposta del primo ordine (val1, fase 3: dal 10 al 90% in
		 * ~220 ms per v e ~160 ms per omega, qualunque sia il gradino) */
		g->v_cmd += (tv - g->v_cmd) * (1 - exp(-dt / par.tau_v_terra));
		g->w_cmd += (tw - g->w_cmd) * (1 - exp(-dt / par.tau_w_terra));
		g->v_measured = g->v_cmd - g->w_cmd;
		g->w_measured = g->v_cmd + g->w_cmd;
	} else if (g->motori_alimentati && g->mode == 2) {
		/* sollevato: rampe ad accelerazione costante (c2, c2b) */
		g->v_cmd = rampa(g->v_cmd, tv, par.rampa_v_su, par.rampa_v_giu, dt);
		g->w_cmd = rampa(g->w_cmd, tw, par.rampa_w_su, par.rampa_w_giu, dt);
		g->v_measured = g->v_cmd - g->w_cmd;
		g->w_measured = g->v_cmd + g->w_cmd;
	} else {
		/* senza potenza (heartbeat mancante) o in modalità 00: le ruote si fermano da sole, per inerzia */
		g->v_cmd = rampa(g->v_cmd, 0, par.ruote_inerzia, par.ruote_inerzia, dt);
		g->w_cmd = rampa(g->w_cmd, 0, par.ruote_inerzia, par.ruote_inerzia, dt);
		g->v_measured = g->v_cmd - g->w_cmd;
		g->w_measured = g->v_cmd + g->w_cmd;
	}
	if (prima < 5 && fabs(g->v_measured) + fabs(g->w_measured) >= 5)
		g->t_avvio_ruote = t;

	/* movimento */
	double v = (g->v_measured + g->w_measured) / 2 * TICK_M, w = (g->w_measured - g->v_measured) * TICK_M / CARREGGIATA;
	double slitta = 1.0;
	if (!g->lifted && !g->caduto) {
		double dm = g->dir + w * dt / 2;
		double nx = g->x + v * cos(dm) * dt, ny = g->y + v * sin(dm) * dt, ang;
		double lx = nx + LIDAR_X * cos(g->dir) - LIDAR_Y * sin(g->dir), ly = ny + LIDAR_X * sin(g->dir) + LIDAR_Y * cos(g->dir);
		if (fabs(v) > 1e-6 && mondo_urto(m, nx, ny, ROBOT_RAGGIO, 0, ROBOT_ALTEZZA_CORPO, &ang) &&
		    (nx - g->x) * cos(ang) + (ny - g->y) * sin(ang) > 0) {
			/* il corpo spinge contro l'ostacolo */
			g->ang_urto = angolo_norm(ang - g->dir);
			g->t_urto = t;
			slitta = 0.3;
		} else if (fabs(v) > 1e-6 && mondo_urto(m, lx, ly, TORRETTA_RAGGIO, ROBOT_ALTEZZA_CORPO, ROBOT_ALTEZZA, &ang) &&
		           (nx - g->x) * cos(ang) + (ny - g->y) * sin(ang) > 0) {
			g->t_incastro = t;                  /* solo la torretta: fermo, ma nessun bumper */
			slitta = 0.3;
		} else {
			g->x = nx;
			g->y = ny;
		}
		g->dir = angolo_norm(g->dir + w * dt);
		if (mondo_vuoto(m, g->x, g->y)) {
			g->caduto = 1;
			avvisa(g, t, "il robot è CADUTO nel vuoto: spostalo a mano dalla pagina");
		}
	}
	g->dist_sx += g->v_measured * dt * slitta;
	g->dist_dx += g->w_measured * dt * slitta;
	g->enc_sx = (int32_t)floor(g->dist_sx);
	g->enc_dx = (int32_t)floor(g->dist_dx);
	g->bloccato = g->t_urto >= 0 && t - g->t_urto < 0.05;
	g->incastrato = g->t_incastro >= 0 && t - g->t_incastro < 0.05;
	if (t >= g->prossimo_enc - 1e-9) {
		g->storia_enc[g->i_enc][0] = g->enc_sx;
		g->storia_enc[g->i_enc][1] = g->enc_dx;
		g->storia_t[g->i_enc] = t;
		g->i_enc = (g->i_enc + 1) % SIM_FINESTRA;
		g->prossimo_enc += 0.010;
		if (g->prossimo_enc < t)
			g->prossimo_enc = t + 0.010;
	}

	/* bumper: urto del CORPO davanti negli ultimi 50 ms (la torretta non ha interruttori) */
	int urto = g->t_urto >= 0 && t - g->t_urto < 0.05 && fabs(g->ang_urto) < BUMPER_MEZZO_ARCO;
	g->bumper_left = urto && g->ang_urto > -BUMPER_CENTRO;
	g->bumper_right = urto && g->ang_urto < BUMPER_CENTRO;

	/* IMU: il GD32 integra la velocità angolare, con la deriva misurata */
	double w_vera = (g->lifted || g->caduto) ? 0 : w;
	g->vel_imbardata = w_vera * par.imu_scala;
	g->imbardata += (w_vera * par.imu_scala + par.imu_deriva_gradi_s * M_PI / 180) * dt;

	/* spazzole e aspirazione */
	g->eco_lat = g->motori_alimentati ? g->spazzola_lat : 0;
	g->eco_cen = !g->motori_alimentati ? 0 : g->spazzola_cen < 0 ? 100 : g->spazzola_cen;
	double revolutions = 0;
	if (g->motori_alimentati)
		revolutions = g->suction_rpm ? g->suction_rpm : g->aspirazione_pwm * par.aspirazione_giri_per_pwm;
	g->ventola_giri += (revolutions - g->ventola_giri) * (1 - exp(-dt / 0.5));   /* [?] costante di tempo della ventola */

	/* batteria: corrente stimata dai consumi */
	double i = 0.35                                            /* [?] SoC acceso */
	         + (g->lidar_alimentato ? 0.0015 * g->lidar_power : 0)
	         + (fabs(g->v_measured) + fabs(g->w_measured) > 5 ? 2 * par.corrente_ruota_base / 1000 : 0)
	         + (g->eco_cen ? par.corrente_centrale_accesa / 1000 : 0)
	         + (par.aspirazione_corrente_max / 1000) * pow(fmax(0, g->ventola_giri) / par.aspirazione_giri_max,
	                                                       par.aspirazione_esponente);
	if (g->charging)
		i -= 1.0;                                              /* [?] corrente di carica */
	g->current = i;
	g->soc -= i * dt / 3600 / par.batteria_utile_ah * g->velocita_batteria;
	if (g->soc > 1)
		g->soc = 1;
	g->tensione = tensione_a_vuoto(g->soc) - (i > 0 ? i * 0.12 : i * 0.05);
	if (g->on && g->tensione < 12.0) {
		g->on = 0;
		g->t_riaccensione = t + 10;
		stato_firmware_iniziale(g);
		avvisa(g, t, "batteria esaurita: protezione, il robot si spegne");
	}

	if (g->sospeso && !g->risveglio && g->tasto_da[0] >= 0) {
		g->risveglio = 1;
		g->prossimo_05 = t;
	}
}

void gd32sim_avanza(gd32sim_t *g, const mondo_t *m, double t)
{
	double last = g->t_simulato;
	if (t < last)
		last = t;
	for (; last < t - 1e-9; g->t_simulato = last) {
		double dt = fmin(0.002, t - last);
		last += dt;
		passo(g, m, last, dt);
		if (!g->on)
			continue;
		if (g->risposta_08 > 0 && last >= g->risposta_08) {
			uint8_t uno = 1;
			manda(g, g->risposta_08, GD32_IMU_ZERO, &uno, 1);
			g->risposta_08 = 0;
		}
		if (g->sospeso) {
			if (g->risveglio && last >= g->prossimo_05) {
				manda(g, g->prossimo_05, GD32_WAKEUP_ACK, NULL, 0);
				g->prossimo_05 = last + 0.1;
			}
			continue;
		}
		if (last >= g->prossimo_sensori) {
			uint8_t d[96];
			gd32sim_dati_sensori(g, g->prossimo_sensori, d);
			manda(g, g->prossimo_sensori, GD32_SENSORS, d, sizeof d);
			g->sensor_frames++;
			g->prossimo_sensori += par.gd32_periodo_ms / 1000;
			if (g->prossimo_sensori < last)
				g->prossimo_sensori = last + par.gd32_periodo_ms / 1000;
		}
		if (last >= g->prossimo_hb_gd32) {
			manda(g, g->prossimo_hb_gd32, GD32_HEARTBEAT, NULL, 0);
			g->prossimo_hb_gd32 += 2.0;
		}
	}
}
