/*
 * Test del simulatore: stanza a oggetti (lettura e scrittura, raggi su rettangoli ruotati e cerchi, quote, vuoto,
 * base), GD32 simulato (cinematica e dinamica tarata, velocità misurata, encoder, sorveglianze, anti-caduta, corpo e
 * torretta, contatti dietro, ritardi di consegna) e lidar simulato.
 */
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "test.h"
#include "../gd32/gd32.h"
#include "../lidar/lidar.h"
#include "../sim/gd32sim.h"
#include "../sim/lidarsim.h"
#include "../sim/mondo.h"
#include "../sim/parametri.h"
#include "../sim/robot.h"

static mondo_t m;   /* grande: statico */

static const char *STANZA =
	"# prova\n"
	"stanza nome=prova larghezza=3 profondita=3\n"
	"robot x=1.5 y=1.5 angolo=0\n"
	"base x=0.5 y=0.22 angolo=90\n"
	"muro da=0,0.05 a=3,0.05 spessore=0.1\n"
	"muro da=0,2.95 a=3,2.95 spessore=0.1\n"
	"muro da=0.05,0.1 a=0.05,2.9 spessore=0.1\n"
	"muro da=2.95,0.1 a=2.95,2.9 spessore=0.1\n"
	"rettangolo nome=diamante x=2 y=2.5 larghezza=0.2 profondita=0.2 angolo=45 quota_da=0 quota_a=1 gruppo=g\n"
	"cerchio nome=gamba x=0.6 y=2.5 raggio=0.05 quota_da=0 quota_a=0.7\n"
	"rettangolo nome=piano x=1 y=1 larghezza=0.4 profondita=0.4 angolo=0 quota_da=0.72 quota_a=0.75\n"
	"rettangolo nome=credenza x=1 y=0.6 larghezza=0.6 profondita=0.2 angolo=0 quota_da=0.09 quota_a=0.4\n"
	"vuoto nome=scala punti=2.2,0.5 2.8,0.5 2.8,1.2 2.2,1.2\n";

static void stanza_testo(void)
{
	char err[300];
	CONTROLLA(mondo_leggi_testo(&m, STANZA, err, sizeof err) == 0);
	CONTROLLA(m.n == 9 && fabs(m.larghezza - 3) < 1e-12 && !strcmp(m.nome, "prova"));
	CONTROLLA(m.ha_base && fabs(m.base_dir - M_PI / 2) < 1e-9 && fabs(m.robot_x - 1.5) < 1e-12);
	CONTROLLA(m.o[0].muro && fabs(m.o[0].larghezza - 3) < 1e-9 && fabs(m.o[0].quota_a - 2.5) < 1e-9);
	CONTROLLA(m.o[4].forma == OGG_RETTANGOLO && !strcmp(m.o[4].gruppo, "g") && fabs(m.o[4].angolo - M_PI / 4) < 1e-9);
	CONTROLLA(m.o[5].forma == OGG_CERCHIO && m.o[8].forma == OGG_VUOTO && m.o[8].n_punti == 4);
	/* andata e ritorno */
	char *t = mondo_testo(&m);
	static mondo_t m2;
	CONTROLLA(t && mondo_leggi_testo(&m2, t, err, sizeof err) == 0);
	CONTROLLA(m2.n == m.n);
	for (int i = 0; i < m.n && i < m2.n; i++)
		CONTROLLA(m2.o[i].forma == m.o[i].forma && fabs(m2.o[i].x - m.o[i].x) < 1e-4 && fabs(m2.o[i].y - m.o[i].y) < 1e-4 &&
		          fabs(m2.o[i].angolo - m.o[i].angolo) < 1e-4 && fabs(m2.o[i].quota_a - m.o[i].quota_a) < 1e-4 &&
		          !strcmp(m2.o[i].nome, m.o[i].nome) && !strcmp(m2.o[i].gruppo, m.o[i].gruppo));
	free(t);
	/* errori */
	CONTROLLA(mondo_leggi_testo(&m2, "robot x=1 y=1\n", err, sizeof err) != 0 && strstr(err, "stanza"));
	CONTROLLA(mondo_leggi_testo(&m2, "stanza larghezza=2 profondita=2\ncerchio x=1 y=1 raggio=0.1 colore=rosso\n", err,
	                            sizeof err) != 0 && strstr(err, "colore"));
	CONTROLLA(mondo_leggi_testo(&m2, "stanza larghezza=2 profondita=2\nvuoto punti=0,0 1,0\n", err, sizeof err) != 0);
	CONTROLLA(mondo_leggi_testo(&m2, "stanza larghezza=2 profondita=2\nrettangolo x=1 y=1 larghezza=1 profondita=1 "
	                                 "quota_da=0.5 quota_a=0.2\n", err, sizeof err) != 0);
}

static void stanza_geometria(void)
{
	char err[300];
	mondo_leggi_testo(&m, STANZA, err, sizeof err);
	/* raggi del lidar (piano a LIDAR_ALTEZZA) */
	CONTROLLA(fabs(mondo_raggio(&m, 1.5, 1.5, 0, LIDAR_ALTEZZA, 8) - 1.4) < 1e-9);          /* muro a x = 2,9 */
	CONTROLLA(fabs(mondo_raggio(&m, 1.5, 2.5, 0, LIDAR_ALTEZZA, 8) - (0.5 - 0.1 * sqrt(2))) < 1e-9);   /* spigolo */
	CONTROLLA(fabs(mondo_raggio(&m, 1.0, 2.5, M_PI, LIDAR_ALTEZZA, 8) - 0.35) < 1e-9);     /* cerchio */
	CONTROLLA(fabs(mondo_raggio(&m, 1.0, 1.5, -M_PI / 2, LIDAR_ALTEZZA, 8) - 1.4) < 1e-9);  /* credenza (da 9 cm) e piano
	                                                                                          non si vedono: il muro */
	CONTROLLA(mondo_raggio(&m, 1.5, 1.5, 0, LIDAR_ALTEZZA, 1.0) > 1.0);                     /* oltre la portata */
	/* il piano del tavolo (72 cm) e la credenza (da 9 cm) non tagliano il piano del lidar; la gamba sì */
	CONTROLLA(fabs(mondo_raggio(&m, 0.3, 1.0, 0, LIDAR_ALTEZZA, 8) - 2.6) < 1e-9);
	/* base: blocco di 0,30 x 0,12 dietro al lato dei contatti (y = 0,22) */
	CONTROLLA(mondo_in_base(&m, 0.5, 0.15) && !mondo_in_base(&m, 0.5, 0.25));
	CONTROLLA(fabs(mondo_raggio(&m, 0.5, 1.0, -M_PI / 2, 0.05, 8) - 0.78) < 1e-9);
	/* urti: corpo (0..7,5 cm) e torretta (7,5..9,5 cm) */
	double a;
	CONTROLLA(!mondo_urto(&m, 1.5, 1.5, ROBOT_RAGGIO, 0, ROBOT_ALTEZZA_CORPO, &a));
	CONTROLLA(mondo_urto(&m, 2.8, 1.5, ROBOT_RAGGIO, 0, ROBOT_ALTEZZA_CORPO, &a) && fabs(a) < 1e-6);
	CONTROLLA(!mondo_urto(&m, 1.0, 0.85, ROBOT_RAGGIO, 0, ROBOT_ALTEZZA_CORPO, &a));          /* sotto la credenza: corpo libero */
	CONTROLLA(mondo_urto(&m, 1.0, 0.85, ROBOT_RAGGIO, ROBOT_ALTEZZA_CORPO, ROBOT_ALTEZZA, &a)); /* ma la torretta tocca */
	CONTROLLA(!mondo_urto(&m, 1.0, 1.0, ROBOT_RAGGIO, 0, ROBOT_ALTEZZA, &a));                 /* sotto il tavolo si passa */
	/* vuoto e selezione */
	CONTROLLA(mondo_vuoto(&m, 2.5, 0.8) && !mondo_vuoto(&m, 2.0, 0.8));
	CONTROLLA(mondo_dentro(&m.o[4], 2.0, 2.5) && !mondo_dentro(&m.o[4], 2.09, 2.59));        /* rombo ruotato */
	/* stanza nuova: 4 muri */
	static mondo_t n;
	mondo_nuovo(&n, 4, 3);
	CONTROLLA(n.n == 4 && fabs(mondo_raggio(&n, 2, 1.5, M_PI / 2, LIDAR_ALTEZZA, 8) - 1.4) < 1e-9);
}

/* ---- GD32 simulato ---- */

static void manda(gd32sim_t *g, uint8_t cmd, const void *dati, int n, double t)
{
	gd32_frame_t f = { .cmd = cmd, .n_dati = (uint8_t)n };
	if (n)
		memcpy(f.dati, dati, (size_t)n);
	gd32sim_ricevi(g, &f, t);
}

static void velocita(gd32sim_t *g, double v, double w, double t)
{
	int32_t b[2] = { (int32_t)lround(v * SCALA_V), (int32_t)lround(w * SCALA_W) };
	manda(g, GD32_VELOCITA, b, 8, t);
}

/* Avanza a passi di 20 ms con heartbeat e keepalive (hb = 0: senza heartbeat); svuota la coda dei frame. */
static void avanza(gd32sim_t *g, const mondo_t *mm, double *t, double fino, int hb)
{
	while (*t < fino - 1e-9) {
		*t += 0.02;
		if (hb && fmod(*t + 1e-9, 0.2) < 0.02)
			manda(g, GD32_HEARTBEAT, NULL, 0, *t);
		if (fmod(*t + 1e-9, 3.0) < 0.02)
			manda(g, GD32_KEEPALIVE, NULL, 0, *t);
		gd32sim_avanza(g, mm, *t);
		while (gd32sim_prossimo(g, *t))
			;
	}
}

static gd32sim_t g;

static void nuovo_robot(double x, double y, double dir, int ritardi)
{
	m.robot_x = x, m.robot_y = y, m.robot_dir = dir;
	memset(&g, 0, sizeof g);
	g.senza_ritardi = !ritardi;
	gd32sim_init(&g, &m, 0);
	uint8_t due = 2;
	manda(&g, GD32_TIPO_CONTROLLO, &due, 1, 0);
	manda(&g, GD32_HEARTBEAT, NULL, 0, 0);
}

static void gd32_dinamica(void)
{
	mondo_nuovo(&m, 5, 3);
	nuovo_robot(1.0, 1.5, 0, 0);
	double t = 0;
	velocita(&g, 0.1, 0, t);
	avanza(&g, &m, &t, 2.0, 1);
	/* 2 s a 0,1 m/s: a terra la risposta è del primo ordine (costante di tempo tau) dopo il ritardo (29 ms), quindi
	   si perdono ritardo + tau x (1 - e^(-T/tau)) secondi di strada */
	double tr = 2.0 - par.ruote_ritardo_ms / 1000, tau = par.tau_v_terra;
	double atteso = 1.0 + 0.1 * (tr - tau * (1 - exp(-tr / tau)));
	printf("  dopo 2 s: x = %.4f (atteso %.4f)\n", g.x, atteso);
	CONTROLLA(fabs(g.x - atteso) < 0.004 && fabs(g.y - 1.5) < 1e-6);
	uint8_t d[96];
	gd32sim_dati_sensori(&g, t, d);
	int32_t enc = (int32_t)(d[16] | d[17] << 8 | d[18] << 16 | (uint32_t)d[19] << 24);
	CONTROLLA(abs(enc - 27000 - (int)lround((g.x - 1.0) * SCALA_V)) < 3);       /* 1 tick = 1/4580 m */
	int32_t vm = (int32_t)(d[32] | d[33] << 8 | d[34] << 16 | (uint32_t)d[35] << 24);
	CONTROLLA(abs(vm - (int)lround(0.1 * SCALA_V)) <= 8);                          /* velocità misurata in 70 ms */
	CONTROLLA(d[0] == 1 && d[64] == 0x62 && (d[4] & 4) && d[88] == 0x03 && d[89] == 0x19);  /* valori veri a riposo */

	/* rotazione di 90°: omega ha la rampa lenta, ma salita e discesa si compensano */
	velocita(&g, 0, 0, t);
	avanza(&g, &m, &t, t + 0.5, 1);
	double x0 = g.x;
	velocita(&g, 0, 0.5, t);
	avanza(&g, &m, &t, t + M_PI, 1);
	velocita(&g, 0, 0, t);
	avanza(&g, &m, &t, t + 1.0, 1);
	printf("  rotazione: %.1f° (attesi 90)\n", g.dir * 180 / M_PI);
	CONTROLLA(fabs(g.dir - M_PI / 2) < 0.04 && fabs(g.x - x0) < 1e-4);
	gd32sim_dati_sensori(&g, t, d);
	CONTROLLA(fabs((int16_t)(d[42] | d[43] << 8) / 100.0 - 90) < 2.5);          /* imbardata dell'IMU */

	/* senza heartbeat: dopo ~2 s motori senza potenza, le ruote si fermano per inerzia in ~0,5 s */
	velocita(&g, 1656 / SCALA_V, 0, t);                         /* 1656 tick/s (~0,36 m/s) */
	avanza(&g, &m, &t, t + 1.0, 1);
	CONTROLLA(g.motori_alimentati && g.vel_sx > 1000);
	double t_hb = t;
	avanza(&g, &m, &t, t_hb + par.ruote_timeout_hb - 0.1, 0);
	CONTROLLA(g.motori_alimentati && g.vel_sx > 1000);              /* ancora in moto */
	avanza(&g, &m, &t, t_hb + par.ruote_timeout_hb + 0.2, 0);
	CONTROLLA(!g.motori_alimentati && g.vel_sx > 1000 && g.vel_sx < 1500);  /* rallenta da solo: 1656 - 0,2 s x ~1880 */
	avanza(&g, &m, &t, t_hb + par.ruote_timeout_hb + 1.0, 0);
	CONTROLLA(fabs(g.vel_sx) < 1);
}

static void gd32_urti_e_base(void)
{
	char err[300];
	mondo_leggi_testo(&m, STANZA, err, sizeof err);
	/* contro il muro: corpo, bumper tutti e due */
	nuovo_robot(2.3, 1.5, 0, 0);
	double t = 0;
	int visto = 0;
	velocita(&g, 0.2, 0, t);
	for (int i = 0; i < 200; i++) {
		avanza(&g, &m, &t, t + 0.02, 1);
		visto |= g.bumper_sx && g.bumper_dx;
	}
	CONTROLLA(visto && fabs(g.x - (2.9 - ROBOT_RAGGIO)) < 0.01);
	/* sotto la credenza (da 9 cm): la torretta si incastra, NESSUN bumper */
	nuovo_robot(1.0, 1.3, -M_PI / 2, 0);
	t = 0;
	int bumper = 0, incastrato = 0;
	velocita(&g, 0.1, 0, t);
	for (int i = 0; i < 400; i++) {
		avanza(&g, &m, &t, t + 0.02, 1);
		bumper |= g.bumper_sx | g.bumper_dx;
		incastrato |= g.incastrato;
	}
	printf("  sotto la credenza: y = %.3f, incastrato %d, bumper %d\n", g.y, incastrato, bumper);
	/* la torretta è DIETRO il centro (LIDAR_X < 0): il corpo entra sotto la credenza finché la torretta tocca il bordo */
	CONTROLLA(incastrato && !bumper && fabs(g.y - (0.7 + TORRETTA_RAGGIO + LIDAR_X)) < 0.02);
	/* vuoto: anti-caduta, il GD32 non va avanti */
	nuovo_robot(1.8, 0.85, 0, 0);
	uint8_t zero = 0;
	manda(&g, GD32_CLIFF_RETROMARCIA, &zero, 1, 0);
	t = 0;
	velocita(&g, 0.1, 0, t);
	avanza(&g, &m, &t, 6.0, 1);
	CONTROLLA(!g.caduto && (g.cliff & 0x06) && g.x < 2.2);
	/* base con i contatti DIETRO: robot che le dà le spalle */
	nuovo_robot(0.5, 0.22 + ROBOT_RAGGIO + 0.001, M_PI / 2, 0);
	t = 0;
	avanza(&g, &m, &t, 13.0, 1);
	CONTROLLA(g.contatto_base && g.in_carica && g.cliff == 0x0f);
	nuovo_robot(0.5, 0.22 + ROBOT_RAGGIO + 0.001, -M_PI / 2, 0);      /* girato al contrario: niente contatto */
	t = 0;
	avanza(&g, &m, &t, 1.0, 1);
	CONTROLLA(!g.contatto_base);
	/* keepalive: senza 0x0d per più di 190 s il GD32 si resetta */
	t = 0;
	while (t < 195) {
		t += 0.1;
		gd32sim_avanza(&g, &m, t);
		while (gd32sim_prossimo(&g, t))
			;
	}
	CONTROLLA(g.reset == 1 && g.modo == 0);
}

typedef struct {
	int versione, funzioni, zero, sensori;
	double ultimo, min_iv, max_iv;
} risposte_t;

static void conta(const gd32_frame_t *fr, void *ctx)
{
	risposte_t *r = ctx;
	if (fr->cmd == GD32_VERSIONE && fr->n_dati == 32 && !memcmp(fr->dati, "2.0.1_19082727", 14)) r->versione = 1;
	if (fr->cmd == GD32_FUNZIONI_MACCHINA && fr->n_dati == 8 && fr->dati[4] == 0x3f) r->funzioni = 1;
	if (fr->cmd == GD32_IMU_ZERO && fr->n_dati == 1 && fr->dati[0] == 1) r->zero = 1;
	if (fr->cmd == GD32_SENSORI && fr->n_dati == 96) r->sensori++;
}

static void gd32_consegna(void)
{
	mondo_nuovo(&m, 3, 3);
	nuovo_robot(1.5, 1.5, 0, 1);
	manda(&g, GD32_VERSIONE, NULL, 0, 0);
	manda(&g, GD32_FUNZIONI_MACCHINA, NULL, 0, 0);
	manda(&g, GD32_IMU_ZERO, NULL, 0, 0);
	gd32_deframer_t d;
	gd32_deframer_init(&d);
	risposte_t r = { .min_iv = 1, .max_iv = 0 };
	double prec = -1;
	for (double t = 0.001; t < 120; t += 0.001) {
		gd32sim_avanza(&g, &m, t);
		const sim_uscita_t *u;
		while ((u = gd32sim_prossimo(&g, t))) {
			int prima = r.sensori;
			gd32_deframer_scrivi(&d, u->b, u->n, conta, &r);
			if (r.sensori > prima) {
				if (prec >= 0) {
					double iv = u->t - prec;
					if (iv < r.min_iv) r.min_iv = iv;
					if (iv > r.max_iv) r.max_iv = iv;
				}
				prec = u->t;
			}
		}
		if (fmod(t, 2.9) < 0.001)
			manda(&g, GD32_KEEPALIVE, NULL, 0, t);
	}
	printf("  %d frame dei sensori in 120 s, intervalli da %.2f a %.2f ms\n", r.sensori, r.min_iv * 1000, r.max_iv * 1000);
	CONTROLLA(r.versione && r.funzioni && r.zero && d.checksum_errati == 0);
	CONTROLLA(abs(r.sensori - 6000) <= 2);
	CONTROLLA(r.min_iv > 0.0145 && r.max_iv < 0.0252 && r.max_iv > 0.0203);       /* variano come sul robot */
}

/* ---- lidar ---- */
static int pacchetti_ad, punti, punti_davanti;
static double vel_letta, somma_d, somma_d2;

static void cb_lidar(const lidar_pacchetto_t *p, void *ctx)
{
	(void)ctx;
	if (p->cmd != LIDAR_SCANSIONE_NUOVA)
		return;
	pacchetti_ad++;
	vel_letta = p->param[0] * 0.05;
	int inizio = p->param[3] << 8 | p->param[4];
	int n = (p->n_param - 5) / 3;
	for (int i = 0; i < n; i++) {
		int q = p->param[6 + 3 * i] << 8 | p->param[7 + 3 * i];
		double ang = (inizio + i * 2250.0 / n) / 100.0 - 14.5;
		punti++;
		if (q && fabs(ang - 180.0) < 0.6) {
			double dd = q / 4000.0;
			punti_davanti++;
			somma_d += dd;
			somma_d2 += dd * dd;
		}
	}
}

static void lidar_simulato(void)
{
	mondo_nuovo(&m, 3, 3);
	static lidarsim_t l;
	memset(&l, 0, sizeof l);
	l.scarto_gradi = 14.5;
	lidarsim_init(&l, 0);
	lidar_deframer_t d;
	lidar_deframer_init(&d);
	for (double t = 0.01; t < 10.0; t += 0.01) {
		lidarsim_avanza(&l, &m, t, 1, 68, 1.5, 1.5, 0);
		const lidarsim_uscita_t *u;
		while ((u = lidarsim_prossimo(&l, t)))
			lidar_deframer_scrivi(&d, u->b, u->n, cb_lidar, NULL);
	}
	double media = somma_d / punti_davanti, dev = sqrt(somma_d2 / punti_davanti - media * media);
	printf("  %d pacchetti, %.2f giri/s, davanti %.4f m +- %.1f mm (%d punti)\n", pacchetti_ad, vel_letta, media,
	       dev * 1000, punti_davanti);
	CONTROLLA(fabs(vel_letta - (par.lidar_a * 68 + par.lidar_b)) < 0.1);
	CONTROLLA(fabs(media - (1.4 - LIDAR_X)) < 0.005 && dev > 0.003 && dev < 0.012);   /* rumore ~7 mm come il vero */
	CONTROLLA(d.checksum_errati == 0);
}

int main(void)
{
	parametri_frame_default();
	ESEGUI(stanza_testo);
	ESEGUI(stanza_geometria);
	ESEGUI(gd32_dinamica);
	ESEGUI(gd32_urti_e_base);
	ESEGUI(gd32_consegna);
	ESEGUI(lidar_simulato);
	return TEST_FINE();
}
