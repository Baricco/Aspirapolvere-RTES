/*
 * Test della logica del driver che non usa Linux: scambio senza mutex (anche con più thread), batteria,
 * giri del lidar (con un pacchetto vero del robot), regolatore della velocità del lidar, limiti "a terra",
 * registro dei contatti (anti-caduta e bumper).
 */
#include <math.h>
#include <pthread.h>
#include <stdatomic.h>
#include <string.h>
#include <unistd.h>

#include "test.h"
#include "../driver/batteria.h"
#include "../driver/contatti.h"
#include "../driver/lidar_giro.h"
#include "../driver/scambio.h"
#include "../gd32/gd32.h"
#include "../gd32/sicurezza.h"
#include "../rt/rt_tempo.h"

/* ---------------------------------------------------------------------------------------- scambio --- */

typedef struct {
	uint32_t a;
	uint32_t riempitivo[200];
	uint32_t b;                 /* sempre uguale ad a: se un lettore li vede diversi, ha letto una copia a metà */
} dato_t;

static dato_t c0, c1;
static scambio_t sc;
static atomic_int fine;
static atomic_int incoerenti, all_indietro;

static void *scrittore(void *arg)
{
	(void)arg;
	dato_t d;
	memset(&d, 0, sizeof d);
	for (uint32_t i = 1; !atomic_load(&fine); i++) {
		d.a = d.b = i;
		for (int k = 0; k < 200; k++)
			d.riempitivo[k] = i;
		scambio_scrivi(&sc, &d);
	}
	return NULL;
}

static void *lettore(void *arg)
{
	(void)arg;
	dato_t d;
	uint32_t ultima = 0;
	while (!atomic_load(&fine)) {
		uint32_t v = scambio_leggi(&sc, &d);
		if (v == 0)
			continue;
		if (d.a != d.b || d.riempitivo[100] != d.a)
			atomic_fetch_add(&incoerenti, 1);
		if (v < ultima || d.a != v)
			atomic_fetch_add(&all_indietro, 1);
		ultima = v;
	}
	return NULL;
}

static void scambio_base(void)
{
	scambio_init(&sc, &c0, &c1, sizeof(dato_t));
	dato_t d = { .a = 7, .b = 7 }, out;
	CONTROLLA(scambio_leggi(&sc, &out) == 0);           /* mai scritto */
	scambio_scrivi(&sc, &d);
	CONTROLLA(scambio_leggi(&sc, &out) == 1 && out.a == 7);
	d.a = d.b = 8;
	scambio_scrivi(&sc, &d);
	CONTROLLA(scambio_leggi(&sc, &out) == 2 && out.b == 8);
	CONTROLLA(scambio_versione(&sc) == 2);
}

static void scambio_con_thread(void)
{
	scambio_init(&sc, &c0, &c1, sizeof(dato_t));
	atomic_store(&fine, 0);
	pthread_t w, r[3];
	pthread_create(&w, NULL, scrittore, NULL);
	for (int i = 0; i < 3; i++)
		pthread_create(&r[i], NULL, lettore, NULL);
	usleep(300000);
	atomic_store(&fine, 1);
	pthread_join(w, NULL);
	for (int i = 0; i < 3; i++)
		pthread_join(r[i], NULL);
	printf("  %u versioni scritte, %llu riprove dei lettori\n", scambio_versione(&sc),
	       (unsigned long long)atomic_load(&sc.riprove));
	CONTROLLA(scambio_versione(&sc) > 1000);
	CONTROLLA(atomic_load(&incoerenti) == 0);           /* mai una copia mescolata */
	CONTROLLA(atomic_load(&all_indietro) == 0);         /* versioni sempre crescenti e giuste */
}

/* --------------------------------------------------------------------------------------- batteria --- */

static int riempi(batteria_t *b, int dv, int carica, int n)
{
	int l = 0;
	for (int i = 0; i < n; i++)
		l = batteria_aggiorna(b, dv, carica, carica, 0, 0);
	return l;
}

/* n frame lontano dalla base (fermo o in movimento); ritorna la percentuale */
static int scarica(batteria_t *b, int dv, int moto, int n)
{
	for (int i = 0; i < n; i++)
		batteria_aggiorna(b, dv, 0, 0, moto, 0);
	return batteria_percentuale(b);
}

static void batteria(void)
{
	/* curve dell'originale (valori calcolati a parte con le costanti del firmware) */
	CONTROLLA(fabs(batteria_curva(BAT_STATO_FERMO, 15.0) - 0.5381) < 1e-4);
	CONTROLLA(fabs(batteria_curva(BAT_STATO_FERMO, 13.8) - 0.1577) < 1e-3);
	CONTROLLA(fabs(batteria_curva(BAT_STATO_FERMO, 13.0) - 0.0313) < 1e-3);
	CONTROLLA(fabs(batteria_curva(BAT_STATO_MOTO, 15.0) - 0.7168) < 1e-4);
	CONTROLLA(fabs(batteria_curva(BAT_STATO_MOTO, 13.8) - 0.2241) < 1e-3);
	CONTROLLA(batteria_curva(BAT_STATO_FERMO, 14.6) < batteria_curva(BAT_STATO_FERMO, 14.7));

	/* aspirazione: lo stato si sceglie dai giri della ventola, come in batteryCapacity (0x66a2b0) */
	CONTROLLA(batteria_stato_carico(0, 20000) == BAT_STATO_2600PA && batteria_stato_carico(1, 19500) == BAT_STATO_2000PA);
	CONTROLLA(batteria_stato_carico(0, 17000) == BAT_STATO_1500PA && batteria_stato_carico(0, 16999) == BAT_STATO_1200PA);
	CONTROLLA(batteria_stato_carico(0, 14000) == BAT_STATO_1000PA && batteria_stato_carico(0, 12000) == BAT_STATO_800PA);
	CONTROLLA(batteria_stato_carico(1, 1001) == BAT_STATO_600PA && batteria_stato_carico(1, 1000) == BAT_STATO_FERMO);
	CONTROLLA(batteria_stato_carico(1, 0) == BAT_STATO_MOTO && batteria_stato_carico(0, 0) == BAT_STATO_FERMO);
	/* ogni curva salta al massimo del 5 % ai confini di 13,3 V e 14,3 V (le curve dell'originale "fermo" e "in
	   movimento" saltano fino al 4,4 %, quelle della ventola meno del 2 %): un segno sbagliato darebbe salti enormi */
	int continua = 1;
	for (int st = 1; st <= 10; st++) {
		if (st == 2)
			continue;
		for (int k = 0; k < 2; k++) {
			double v = k ? 14.3 : 13.3;
			if (fabs(batteria_curva(st, v) - batteria_curva(st, v - 1e-9)) > 0.05)
				continua = 0;
		}
	}
	CONTROLLA(continua);
	CONTROLLA(batteria_curva(BAT_STATO_1500PA, 15.6) == 1.05 && batteria_curva(BAT_STATO_1200PA, 15.6) < 1.05);
	CONTROLLA(fabs(batteria_curva(BAT_STATO_800PA, 14.0) - (0.1327 * 196 - 3.2947 * 14 + 20.455)) < 1e-9);

	/* soglie */
	batteria_t b;
	batteria_init(&b);
	CONTROLLA(riempi(&b, 120, 0, 5) == BAT_OK);         /* troppo pochi campioni per decidere */
	batteria_init(&b);
	CONTROLLA(riempi(&b, 160, 0, 50) == BAT_OK);
	CONTROLLA(fabs(batteria_volt(&b) - 16.0) < 1e-9);
	CONTROLLA(riempi(&b, 141, 0, 50) == BAT_BASSA);
	CONTROLLA(riempi(&b, 143, 0, 50) == BAT_BASSA);     /* isteresi: serve > 14,4 V */
	CONTROLLA(riempi(&b, 145, 0, 50) == BAT_OK);
	CONTROLLA(riempi(&b, 137, 0, 50) == BAT_CRITICA);
	CONTROLLA(riempi(&b, 139, 0, 50) == BAT_CRITICA);   /* isteresi: serve > 14,0 V */
	CONTROLLA(riempi(&b, 141, 0, 50) == BAT_BASSA);
	CONTROLLA(riempi(&b, 137, 1, 50) == BAT_OK);        /* in carica: sempre OK */
	/* un solo campione basso non basta (media di 1 s) */
	batteria_init(&b);
	riempi(&b, 160, 0, 50);
	CONTROLLA(riempi(&b, 100, 0, 1) == BAT_OK);

	/* percentuale lontano dalla base: la prima stima è la curva */
	batteria_init(&b);
	CONTROLLA(batteria_percentuale(&b) == 0);
	CONTROLLA(scarica(&b, 150, 0, 1) == 54);
	CONTROLLA(scarica(&b, 150, 0, 5000) == 54);
	/* la tensione scende: la stima la segue piano (media di 100 s, poi costante di tempo di 100 s), senza salti */
	int prec = 54, salti = 0, risalite = 0;
	for (int i = 0; i < 300; i++) {
		int p = scarica(&b, 145, 0, 50);
		salti += prec - p > 1;
		risalite += p > prec;
		prec = p;
	}
	CONTROLLA(salti == 0 && risalite == 0);
	CONTROLLA(prec >= 40 && prec <= 42);                /* curva a 14,5 V: 39,6 % */
	/* la tensione risale (carico tolto): la stima non sale */
	CONTROLLA(scarica(&b, 150, 0, 6000) == prec);
	/* e cala dell'1 % ogni 16202 frame in cui la curva dice di più (~324 s) */
	batteria_init(&b);
	scarica(&b, 145, 0, 1);
	double s0 = b.stima;
	scarica(&b, 150, 0, 16201);
	CONTROLLA(fabs(b.stima - s0) < 1e-12);
	scarica(&b, 150, 0, 1);
	CONTROLLA(fabs(b.stima - (s0 - 0.01)) < 1e-12);
	/* con la ventola a 18 000 giri/min (1500 Pa) il calo a tempo è ogni 2752 frame (~55 s) */
	batteria_init(&b);
	batteria_aggiorna(&b, 145, 0, 0, 0, 18000);
	s0 = b.stima;
	CONTROLLA(b.stato == BAT_STATO_1500PA && fabs(s0 - batteria_curva(BAT_STATO_1500PA, 14.5)) < 1e-12);
	for (int i = 0; i < 2751; i++)
		batteria_aggiorna(&b, 150, 0, 0, 0, 18000);
	CONTROLLA(fabs(b.stima - s0) < 1e-12);
	batteria_aggiorna(&b, 150, 0, 0, 0, 18000);
	CONTROLLA(fabs(b.stima - (s0 - 0.01)) < 1e-12);
	/* in movimento si usa l'altra curva */
	batteria_init(&b);
	CONTROLLA(scarica(&b, 150, 1, 1) == 72);
	batteria_init(&b);
	CONTROLLA(scarica(&b, 163, 1, 1) == 100);           /* la curva va oltre il 100 %: si limita */
	batteria_init(&b);
	CONTROLLA(scarica(&b, 120, 0, 1) == 1);

	/* sulla base: prima stima dalla curva di carica (tensione - 0,4 V), poi +1 % ogni 5002 frame sopra 15 V */
	batteria_init(&b);
	batteria_aggiorna(&b, 163, 1, 1, 0, 0);
	CONTROLLA(batteria_percentuale(&b) == 91);
	for (int i = 0; i < 5001; i++)
		batteria_aggiorna(&b, 163, 1, 1, 0, 0);
	CONTROLLA(batteria_percentuale(&b) == 91);
	batteria_aggiorna(&b, 163, 1, 1, 0, 0);
	CONTROLLA(batteria_percentuale(&b) == 92);
	batteria_init(&b);
	batteria_aggiorna(&b, 165, 1, 1, 0, 0);
	CONTROLLA(batteria_percentuale(&b) == 97);          /* sopra 16,4 V: 97 % */
	/* scendendo dalla base: per 6 s conta ancora come "in carica" (bit acceso per più dell'80 % degli ultimi 30 s),
	 * poi la stima parte dal valore raggiunto e scende piano verso la curva */
	batteria_init(&b);
	for (int i = 0; i < 40000; i++)
		batteria_aggiorna(&b, 163, 1, 1, 0, 0);
	CONTROLLA(batteria_percentuale(&b) == 98);
	scarica(&b, 162, 0, 300);
	CONTROLLA(b.stato == BAT_STATO_CARICA);
	scarica(&b, 162, 0, 1);
	CONTROLLA(b.stato == BAT_STATO_FERMO);
	CONTROLLA(scarica(&b, 162, 0, 50 * 600) == 88);     /* curva a 16,2 V: 87,9 % */
}

/* ---------------------------------------------------------------------------------------- lidar --- */

/* Un pacchetto 0xAD vero (fase 2, cattura c1): settore che parte da 0°, 5,3 giri/s, 24 punti. */
static const char *VERO = "aa00550161ad004d6a005200000000009501dc9501de9a01df9a01e58601e58e01e78001ea6801ef3e01fd00"
                          "00003a01c874021e68021a80024b44029c00000000000000000000000000000000000000000000000013a2";

static size_t esa(const char *s, uint8_t *out)
{
	size_t n = 0;
	for (; s[0] && s[1]; s += 2) {
		unsigned v;
		sscanf(s, "%2x", &v);
		out[n++] = (uint8_t)v;
	}
	return n;
}

static lidar_giro_t giro;
static int giri_visti;
static lidar_assemblatore_t ass;

static void raccogli(const lidar_pacchetto_t *p, void *ctx)
{
	int64_t t = *(int64_t *)ctx;
	if (lidar_assembla(&ass, p, t, &giro))
		giri_visti++;
}

/* Un settore costruito: n punti a distanza mm, velocità u8. */
static size_t settore(int s, int n, int mm, uint8_t *out)
{
	uint8_t p[5 + 3 * 29];
	p[0] = 120;                 /* 6 giri/s */
	p[1] = 0;
	p[2] = 82;
	p[3] = (uint8_t)((s * 2250) >> 8);
	p[4] = (uint8_t)(s * 2250);
	for (int i = 0; i < n; i++) {
		p[5 + 3 * i] = 100;
		p[6 + 3 * i] = (uint8_t)((mm * 4) >> 8);
		p[7 + 3 * i] = (uint8_t)(mm * 4);
	}
	return lidar_componi(1, 0x61, LIDAR_SCANSIONE_NUOVA, p, 5 + 3 * (size_t)n, out, 128);
}

static void lidar_giri(void)
{
	lidar_deframer_t d;
	lidar_deframer_init(&d);
	lidar_assemblatore_init(&ass, 800);
	giri_visti = 0;
	uint8_t b[256];
	int64_t t = RT_MS(1);
	size_t n = esa(VERO, b);
	CONTROLLA(lidar_deframer_scrivi(&d, b, n, raccogli, &t) == 1);   /* il pacchetto vero è valido */
	CONTROLLA(fabs(ass.giri_s - 5.3) < 1e-9);
	for (int s = 1; s < 16; s++) {
		t += RT_MS(10);
		n = settore(s, 21, 1000 + s, b);
		lidar_deframer_scrivi(&d, b, n, raccogli, &t);
	}
	CONTROLLA(giri_visti == 0);                         /* il giro si chiude solo quando l'angolo torna indietro */
	t += RT_MS(10);
	n = settore(0, 21, 500, b);
	lidar_deframer_scrivi(&d, b, n, raccogli, &t);
	CONTROLLA(giri_visti == 1);
	CONTROLLA(giro.settori == 16 && giro.n == 24 + 15 * 21);
	/* primo punto del pacchetto vero: intensità 0, distanza 0 (nessuna eco); il secondo 0x95, 0x01dc/4 = 119 mm */
	CONTROLLA(giro.p[0].distanza_mm == 0 && giro.p[1].intensita == 0x95 && giro.p[1].distanza_mm == 119);
	/* angoli: inizio 0 - scarto 8° = 352°; il punto 1 è 22,5°/24 più avanti */
	CONTROLLA(giro.p[0].angolo_cdeg == 35200);
	CONTROLLA(giro.p[1].angolo_cdeg == (2250 / 24) - 800 + 36000);
	/* settore 1, punto 0: 22,5° - 8° = 14,5° */
	CONTROLLA(giro.p[24].angolo_cdeg == 1450 && giro.p[24].distanza_mm == 1001);
	CONTROLLA(giro.t_inizio_ns == RT_MS(1) && giro.t_fine_ns == RT_MS(151));
	CONTROLLA(giro.numero == 1);

	/* un giro incompleto (accensione) non si consegna */
	lidar_assemblatore_init(&ass, 0);
	giri_visti = 0;
	for (int s = 10; s < 16; s++) {
		n = settore(s, 21, 700, b);
		lidar_deframer_scrivi(&d, b, n, raccogli, &t);
	}
	n = settore(0, 21, 700, b);
	lidar_deframer_scrivi(&d, b, n, raccogli, &t);
	CONTROLLA(giri_visti == 0 && ass.giri_scartati == 1);

	/* 0xAE: solo la velocità */
	uint8_t v = 60;
	n = lidar_componi(1, 0x61, LIDAR_VELOCITA, &v, 1, b, sizeof b);
	lidar_deframer_scrivi(&d, b, n, raccogli, &t);
	CONTROLLA(fabs(ass.giri_s - 3.0) < 1e-9);
}

static void lidar_regolatore(void)
{
	/* il motore del lidar simulato come nella fase 2: giri/s = 0,08575 x potenza + 0,16, con inerzia */
	lidar_regolatore_t r;
	lidar_regolatore_init(&r, 6.0);
	double vel = 0;
	int p = 0;
	for (int i = 0; i < 500; i++) {                     /* 10 s a passi di 20 ms */
		p = lidar_regola(&r, vel);
		double obiettivo = p < 15 ? 0 : 0.08575 * p + 0.16;
		vel += (obiettivo - vel) * (1 - exp(-0.02 / 0.6));
	}
	printf("  dopo 10 s: %.3f giri/s con potenza %d\n", vel, p);
	CONTROLLA(fabs(vel - 6.0) < 0.1);
	CONTROLLA(p >= 66 && p <= 70);
	CONTROLLA(r.integrale <= 20 && r.integrale >= -20);
	/* obiettivo zero: potenza zero */
	lidar_regolatore_init(&r, 0);
	CONTROLLA(lidar_regola(&r, 3) == 0);
	/* lidar fermo (nessuna misura): la potenza sale ma resta entro 100 */
	lidar_regolatore_init(&r, 6.0);
	for (int i = 0; i < 100; i++)
		p = lidar_regola(&r, 0);
	CONTROLLA(p == 100);
}

/* -------------------------------------------------------------------------------------- a terra --- */

static gd32_sensori_t sensori(void)
{
	gd32_sensori_t s;
	memset(&s, 0, sizeof s);
	s.controllo = 0x62;         /* sensori accesi */
	return s;
}

static void a_terra(void)
{
	gd32_sensori_t s = sensori();
	int32_t v = 1656, w = 400;
	CONTROLLA(sic_terra_limita(&s, 0, &v, &w) == 0 && v == 1656 && w == 400);

	s.bumper_sx = 1;
	v = 1656, w = 400;
	CONTROLLA(sic_terra_limita(&s, 0, &v, &w) == SIC_LIM_BUMPER && v == 0 && w == 400);   /* ruotare si può */
	v = -800, w = 0;
	CONTROLLA(sic_terra_limita(&s, 0, &v, &w) == SIC_LIM_BUMPER && v == -800);           /* indietro anche */

	s = sensori();
	s.cliff[1] = 1;
	v = 1656, w = 0;
	CONTROLLA(sic_terra_limita(&s, 0, &v, &w) == SIC_LIM_CLIFF && v == 0);

	s = sensori();
	s.cliff[0] = s.cliff[1] = s.cliff[2] = s.cliff[3] = 1;                              /* sollevato */
	v = -800, w = 400;
	CONTROLLA(sic_terra_limita(&s, 0, &v, &w) == SIC_LIM_SOLLEVATO && v == 0 && w == 0);

	s = sensori();
	s.sulla_base = 1;
	s.controllo = 0x42;                                                                  /* sensori spenti */
	s.cliff[0] = s.cliff[1] = s.cliff[2] = s.cliff[3] = 1;                              /* sulla base: 0f */
	s.bumper_sx = s.bumper_dx = 1;                                                       /* spenti: "premuti" */
	v = -800, w = 0;
	CONTROLLA(sic_terra_limita(&s, 0, &v, &w) == SIC_LIM_BASE && v == 0 && w == 0);
	v = -5000, w = 3000;
	CONTROLLA(sic_terra_limita(&s, 1, &v, &w) == SIC_LIM_BASE && v == (int32_t)(-0.1 * GD32_SCALA_V) &&
	          w == (int32_t)(0.5 * GD32_SCALA_W));                                    /* uscita piano: 0,1 m/s, 0,5 rad/s */

	s = sensori();
	s.controllo = 0x42;                                                                  /* spenti fuori dalla base */
	v = 1656, w = 0;
	CONTROLLA(sic_terra_limita(&s, 0, &v, &w) == SIC_LIM_SENSORI && v == 0);
}

/* -------------------------------------------------------------------------------------------- contatti --- */

#define MS(x) ((int64_t)(x) * 1000000)

static gd32_sensori_t sensori_a_terra(void)
{
	gd32_sensori_t s;
	memset(&s, 0, sizeof s);
	s.controllo = GD32_CONTROLLO_SENSORI_ACCESI;
	return s;
}

static void contatti(void)
{
	contatti_t c;
	contatti_init(&c);
	gd32_sensori_t s = sensori_a_terra();
	int64_t t = MS(1000);

	/* niente attivo: niente eventi */
	CONTROLLA(contatti_frame(&c, &s, t, NULL, 0) == 0 && c.pub.ultimo == 0);

	/* anti-caduta 1: 2 frame non bastano, al terzo l'evento, con l'istante del primo */
	s.cliff[1] = 1;
	CONTROLLA(contatti_frame(&c, &s, t += MS(20), NULL, 0) == 0);
	int64_t primo = t;
	CONTROLLA(contatti_frame(&c, &s, t += MS(20), NULL, 0) == 0);
	CONTROLLA(contatti_frame(&c, &s, t += MS(20), NULL, 0) == 1);
	CONTROLLA(c.pub.ultimo == 1 && c.pub.ev[1].numero == 1 && c.pub.ev[1].sensore == CONTATTO_VUOTO_SX_ANT);
	CONTROLLA(c.pub.ev[1].t_ns == primo);
	/* resta attivo: nessun altro evento */
	for (int i = 0; i < 10; i++)
		CONTROLLA(contatti_frame(&c, &s, t += MS(20), NULL, 0) == 0);
	CONTROLLA(c.pub.ultimo == 1);

	/* un lampo di 2 frame dopo essere tornato normale: niente (falso allarme filtrato) */
	s.cliff[1] = 0;
	contatti_frame(&c, &s, t += MS(20), NULL, 0);
	s.cliff[1] = 1;
	contatti_frame(&c, &s, t += MS(20), NULL, 0);
	contatti_frame(&c, &s, t += MS(20), NULL, 0);
	s.cliff[1] = 0;
	contatti_frame(&c, &s, t += MS(20), NULL, 0);
	CONTROLLA(c.pub.ultimo == 1);

	/* bumper destro: basta un frame */
	s.bumper_dx = 1;
	CONTROLLA(contatti_frame(&c, &s, t += MS(20), NULL, 0) == 1);
	CONTROLLA(c.pub.ultimo == 2 && c.pub.ev[2].sensore == CONTATTO_BUMPER_DX && c.pub.ev[2].t_ns == t);
	CONTROLLA(contatti_frame(&c, &s, t += MS(20), NULL, 0) == 0);
	s.bumper_dx = 0;
	contatti_frame(&c, &s, t += MS(20), NULL, 0);
	s.bumper_dx = 1;
	CONTROLLA(contatti_frame(&c, &s, t += MS(20), NULL, 0) == 1 && c.pub.ultimo == 3);   /* nuovo scatto, nuovo evento */
	s.bumper_dx = 0;

	/* robot sollevato (tutti e 4 nel vuoto): niente eventi, e i conteggi ripartono */
	s.cliff[0] = 1;
	contatti_frame(&c, &s, t += MS(20), NULL, 0);
	contatti_frame(&c, &s, t += MS(20), NULL, 0);
	s.cliff[1] = s.cliff[2] = s.cliff[3] = 1;
	for (int i = 0; i < 10; i++)
		CONTROLLA(contatti_frame(&c, &s, t += MS(20), NULL, 0) == 0);
	s.cliff[1] = s.cliff[2] = s.cliff[3] = 0;          /* posato con un solo sensore sul vuoto: si riconta da 1 */
	CONTROLLA(contatti_frame(&c, &s, t += MS(20), NULL, 0) == 0);
	CONTROLLA(c.pub.ultimo == 3);
	s.cliff[0] = 0;

	/* sensori spenti o robot sulla base: anti-caduta e bumper non significano niente */
	s.bumper_sx = 1;
	s.controllo = 0;
	CONTROLLA(contatti_frame(&c, &s, t += MS(20), NULL, 0) == 0);
	s.controllo = GD32_CONTROLLO_SENSORI_ACCESI;
	s.sulla_base = 1;
	CONTROLLA(contatti_frame(&c, &s, t += MS(20), NULL, 0) == 0);
	CONTROLLA(c.pub.ultimo == 3);
	s.sulla_base = 0;
	CONTROLLA(contatti_frame(&c, &s, t += MS(20), NULL, 0) == 1 && c.pub.ev[4].sensore == CONTATTO_BUMPER_SX);
	s.bumper_sx = 0;
	contatti_frame(&c, &s, t += MS(20), NULL, 0);

	/* lista circolare: dopo 40 eventi restano gli ultimi CONTATTI_N, ognuno al suo posto */
	for (int i = 0; i < 36; i++) {
		s.bumper_sx = 1;
		contatti_frame(&c, &s, t += MS(20), NULL, 0);
		s.bumper_sx = 0;
		contatti_frame(&c, &s, t += MS(20), NULL, 0);
	}
	CONTROLLA(c.pub.ultimo == 40);
	int ok = 1;
	for (uint32_t n = 40 - CONTATTI_N + 1; n <= 40; n++)
		ok &= c.pub.ev[n % CONTATTI_N].numero == n;
	CONTROLLA(ok);

	/* posizione: l'evento porta quella data dal moto; senza, posa_valida = 0 */
	CONTROLLA(c.pub.ev[40 % CONTATTI_N].posa_valida == 0);
	drv_posa_t posa = { .x = 1.25, .y = -0.5, .theta = 0.75, .t_ns = t };
	s.bumper_sx = 1;
	CONTROLLA(contatti_frame(&c, &s, t += MS(20), &posa, 0) == 1);
	drv_contatto_t e = c.pub.ev[c.pub.ultimo % CONTATTI_N];
	CONTROLLA(e.posa_valida && e.posa.x == 1.25 && e.posa.y == -0.5 && e.posa.theta == 0.75 && e.posa.t_ns == posa.t_ns);
	s.bumper_sx = 0;
	contatti_frame(&c, &s, t += MS(20), &posa, 0);
	/* anti-caduta: la posizione è quella del PRIMO frame attivo, non di quello in cui l'evento viene confermato */
	s.cliff[3] = 1;
	drv_posa_t p1 = { .x = 2, .t_ns = t }, p2 = { .x = 3, .t_ns = t + MS(20) }, p3 = { .x = 4, .t_ns = t + MS(40) };
	contatti_frame(&c, &s, t += MS(20), &p1, 0);
	contatti_frame(&c, &s, t += MS(20), &p2, 0);
	CONTROLLA(contatti_frame(&c, &s, t += MS(20), &p3, 0) == 1);
	e = c.pub.ev[c.pub.ultimo % CONTATTI_N];
	CONTROLLA(e.sensore == CONTATTO_VUOTO_DX_LAT && e.posa_valida && e.posa.x == 2);
	s.cliff[3] = 0;
	contatti_frame(&c, &s, t += MS(20), &p3, 0);

	/* incastro: evento al primo frame segnalato, con la posizione di quel frame, uno solo finché resta segnalato */
	uint32_t prima = c.pub.ultimo;
	drv_posa_t pi = { .x = 5, .y = 6, .theta = 1, .t_ns = t };
	CONTROLLA(contatti_frame(&c, &s, t += MS(20), &pi, 1) == 1);
	e = c.pub.ev[c.pub.ultimo % CONTATTI_N];
	CONTROLLA(c.pub.ultimo == prima + 1 && e.sensore == CONTATTO_INCASTRO && e.t_ns == t);
	CONTROLLA(e.posa_valida && e.posa.x == 5 && e.posa.y == 6 && e.posa.theta == 1);
	for (int i = 0; i < 10; i++)
		CONTROLLA(contatti_frame(&c, &s, t += MS(20), &p3, 1) == 0);
	/* breve (la sicurezza lo spegne dopo 100 ms): un evento lo stesso; tolto e ridato: un evento nuovo */
	contatti_frame(&c, &s, t += MS(20), &p3, 0);
	CONTROLLA(contatti_frame(&c, &s, t += MS(20), &p3, 1) == 1 && c.pub.ultimo == prima + 2);
	contatti_frame(&c, &s, t += MS(20), &p3, 0);
	/* robot sollevato o sulla base: niente incastri */
	s.cliff[0] = s.cliff[1] = s.cliff[2] = s.cliff[3] = 1;
	CONTROLLA(contatti_frame(&c, &s, t += MS(20), &p3, 1) == 0);
	s.cliff[0] = s.cliff[1] = s.cliff[2] = s.cliff[3] = 0;
	contatti_frame(&c, &s, t += MS(20), &p3, 0);
	s.sulla_base = 1;
	CONTROLLA(contatti_frame(&c, &s, t += MS(20), &p3, 1) == 0);
	s.sulla_base = 0;
	CONTROLLA(c.pub.ultimo == prima + 2);
	/* insieme al bumper: due eventi nello stesso frame */
	contatti_frame(&c, &s, t += MS(20), &p3, 0);
	s.bumper_dx = 1;
	CONTROLLA(contatti_frame(&c, &s, t += MS(20), &p3, 1) == 1 && c.pub.ultimo == prima + 4);
	CONTROLLA(c.pub.ev[(prima + 3) % CONTATTI_N].sensore == CONTATTO_BUMPER_DX);
	CONTROLLA(c.pub.ev[(prima + 4) % CONTATTI_N].sensore == CONTATTO_INCASTRO);
	s.bumper_dx = 0;
}

int main(void)
{
	ESEGUI(scambio_base);
	ESEGUI(scambio_con_thread);
	ESEGUI(batteria);
	ESEGUI(lidar_giri);
	ESEGUI(lidar_regolatore);
	ESEGUI(a_terra);
	ESEGUI(contatti);
	return TEST_FINE();
}
