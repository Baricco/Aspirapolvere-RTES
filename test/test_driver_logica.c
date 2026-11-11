/*
 * Test della logica del driver che non usa Linux: scambio senza mutex (anche con più thread), batteria,
 * giri del lidar (con un pacchetto vero del robot), regolatore della velocità del lidar, limiti "a terra",
 * registro dei contatti (anti-caduta e bumper), aggancio alla base.
 */
#include <math.h>
#include <pthread.h>
#include <stdatomic.h>
#include <string.h>
#include <unistd.h>

#include "test.h"
#include "../driver/docking.h"
#include "../driver/battery.h"
#include "../driver/contacts.h"
#include "../driver/lidar_scan.h"
#include "../driver/exchange.h"
#include "../gd32/gd32.h"
#include "../gd32/interlock.h"
#include "../rt/rt_time.h"

/* ---------------------------------------------------------------------------------------- scambio --- */

typedef struct {
	uint32_t a;
	uint32_t riempitivo[200];
	uint32_t b;                 /* sempre uguale ad a: se un lettore li vede diversi, ha letto una copia a metà */
} dato_t;

static dato_t c0, c1;
static exchange_t sc;
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
		exchange_write(&sc, &d);
	}
	return NULL;
}

static void *lettore(void *arg)
{
	(void)arg;
	dato_t d;
	uint32_t ultima = 0;
	while (!atomic_load(&fine)) {
		uint32_t v = exchange_read(&sc, &d);
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
	exchange_init(&sc, &c0, &c1, sizeof(dato_t));
	dato_t d = { .a = 7, .b = 7 }, out;
	CONTROLLA(exchange_read(&sc, &out) == 0);           /* mai scritto */
	exchange_write(&sc, &d);
	CONTROLLA(exchange_read(&sc, &out) == 1 && out.a == 7);
	d.a = d.b = 8;
	exchange_write(&sc, &d);
	CONTROLLA(exchange_read(&sc, &out) == 2 && out.b == 8);
	CONTROLLA(exchange_version(&sc) == 2);
}

static void scambio_con_thread(void)
{
	exchange_init(&sc, &c0, &c1, sizeof(dato_t));
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
	printf("  %u versioni scritte, %llu riprove dei lettori\n", exchange_version(&sc),
	       (unsigned long long)atomic_load(&sc.retries));
	CONTROLLA(exchange_version(&sc) > 1000);
	CONTROLLA(atomic_load(&incoerenti) == 0);           /* mai una copia mescolata */
	CONTROLLA(atomic_load(&all_indietro) == 0);         /* versioni sempre crescenti e giuste */
}

/* --------------------------------------------------------------------------------------- batteria --- */

static int riempi(battery_t *b, int dv, int charge, int n)
{
	int l = 0;
	for (int i = 0; i < n; i++)
		l = battery_update(b, dv, charge, charge, 0, 0);
	return l;
}

/* n frame lontano dalla base (fermo o in movimento); ritorna la percentuale */
static int scarica(battery_t *b, int dv, int moto, int n)
{
	for (int i = 0; i < n; i++)
		battery_update(b, dv, 0, 0, moto, 0);
	return battery_percentage(b);
}

static void battery(void)
{
	/* curve dell'originale (valori calcolati a parte con le costanti del firmware) */
	CONTROLLA(fabs(battery_curve(BAT_STATE_IDLE, 15.0) - 0.5381) < 1e-4);
	CONTROLLA(fabs(battery_curve(BAT_STATE_IDLE, 13.8) - 0.1577) < 1e-3);
	CONTROLLA(fabs(battery_curve(BAT_STATE_IDLE, 13.0) - 0.0313) < 1e-3);
	CONTROLLA(fabs(battery_curve(BAT_STATE_MOVING, 15.0) - 0.7168) < 1e-4);
	CONTROLLA(fabs(battery_curve(BAT_STATE_MOVING, 13.8) - 0.2241) < 1e-3);
	CONTROLLA(battery_curve(BAT_STATE_IDLE, 14.6) < battery_curve(BAT_STATE_IDLE, 14.7));

	/* aspirazione: lo stato si sceglie dai giri della ventola, come in batteryCapacity (0x66a2b0) */
	CONTROLLA(battery_load_state(0, 20000) == BAT_STATE_2600PA && battery_load_state(1, 19500) == BAT_STATE_2000PA);
	CONTROLLA(battery_load_state(0, 17000) == BAT_STATE_1500PA && battery_load_state(0, 16999) == BAT_STATE_1200PA);
	CONTROLLA(battery_load_state(0, 14000) == BAT_STATE_1000PA && battery_load_state(0, 12000) == BAT_STATE_800PA);
	CONTROLLA(battery_load_state(1, 1001) == BAT_STATE_600PA && battery_load_state(1, 1000) == BAT_STATE_IDLE);
	CONTROLLA(battery_load_state(1, 0) == BAT_STATE_MOVING && battery_load_state(0, 0) == BAT_STATE_IDLE);
	/* ogni curva salta al massimo del 5 % ai confini di 13,3 V e 14,3 V (le curve dell'originale "fermo" e "in
	   movimento" saltano fino al 4,4 %, quelle della ventola meno del 2 %): un segno sbagliato darebbe salti enormi */
	int continua = 1;
	for (int st = 1; st <= 10; st++) {
		if (st == 2)
			continue;
		for (int k = 0; k < 2; k++) {
			double v = k ? 14.3 : 13.3;
			if (fabs(battery_curve(st, v) - battery_curve(st, v - 1e-9)) > 0.05)
				continua = 0;
		}
	}
	CONTROLLA(continua);
	CONTROLLA(battery_curve(BAT_STATE_1500PA, 15.6) == 1.05 && battery_curve(BAT_STATE_1200PA, 15.6) < 1.05);
	CONTROLLA(fabs(battery_curve(BAT_STATE_800PA, 14.0) - (0.1327 * 196 - 3.2947 * 14 + 20.455)) < 1e-9);

	/* soglie */
	battery_t b;
	battery_init(&b);
	CONTROLLA(riempi(&b, 120, 0, 5) == BAT_OK);         /* troppo pochi campioni per decidere */
	battery_init(&b);
	CONTROLLA(riempi(&b, 160, 0, 50) == BAT_OK);
	CONTROLLA(fabs(battery_volts(&b) - 16.0) < 1e-9);
	CONTROLLA(riempi(&b, 141, 0, 50) == BAT_LOW);
	CONTROLLA(riempi(&b, 143, 0, 50) == BAT_LOW);     /* isteresi: serve > 14,4 V */
	CONTROLLA(riempi(&b, 145, 0, 50) == BAT_OK);
	CONTROLLA(riempi(&b, 137, 0, 50) == BAT_CRITICAL);
	CONTROLLA(riempi(&b, 139, 0, 50) == BAT_CRITICAL);   /* isteresi: serve > 14,0 V */
	CONTROLLA(riempi(&b, 141, 0, 50) == BAT_LOW);
	CONTROLLA(riempi(&b, 137, 1, 50) == BAT_OK);        /* in carica: sempre OK */
	/* un solo campione basso non basta (media di 1 s) */
	battery_init(&b);
	riempi(&b, 160, 0, 50);
	CONTROLLA(riempi(&b, 100, 0, 1) == BAT_OK);

	/* percentuale lontano dalla base: la prima stima è la curva */
	battery_init(&b);
	CONTROLLA(battery_percentage(&b) == 0);
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
	battery_init(&b);
	scarica(&b, 145, 0, 1);
	double s0 = b.estimate;
	scarica(&b, 150, 0, 16201);
	CONTROLLA(fabs(b.estimate - s0) < 1e-12);
	scarica(&b, 150, 0, 1);
	CONTROLLA(fabs(b.estimate - (s0 - 0.01)) < 1e-12);
	/* con la ventola a 18 000 giri/min (1500 Pa) il calo a tempo è ogni 2752 frame (~55 s) */
	battery_init(&b);
	battery_update(&b, 145, 0, 0, 0, 18000);
	s0 = b.estimate;
	CONTROLLA(b.status == BAT_STATE_1500PA && fabs(s0 - battery_curve(BAT_STATE_1500PA, 14.5)) < 1e-12);
	for (int i = 0; i < 2751; i++)
		battery_update(&b, 150, 0, 0, 0, 18000);
	CONTROLLA(fabs(b.estimate - s0) < 1e-12);
	battery_update(&b, 150, 0, 0, 0, 18000);
	CONTROLLA(fabs(b.estimate - (s0 - 0.01)) < 1e-12);
	/* in movimento si usa l'altra curva */
	battery_init(&b);
	CONTROLLA(scarica(&b, 150, 1, 1) == 72);
	battery_init(&b);
	CONTROLLA(scarica(&b, 163, 1, 1) == 100);           /* la curva va oltre il 100 %: si limita */
	battery_init(&b);
	CONTROLLA(scarica(&b, 120, 0, 1) == 1);

	/* sulla base: prima stima dalla curva di carica (tensione - 0,4 V), poi +1 % ogni 5002 frame sopra 15 V */
	battery_init(&b);
	battery_update(&b, 163, 1, 1, 0, 0);
	CONTROLLA(battery_percentage(&b) == 91);
	for (int i = 0; i < 5001; i++)
		battery_update(&b, 163, 1, 1, 0, 0);
	CONTROLLA(battery_percentage(&b) == 91);
	battery_update(&b, 163, 1, 1, 0, 0);
	CONTROLLA(battery_percentage(&b) == 92);
	battery_init(&b);
	battery_update(&b, 165, 1, 1, 0, 0);
	CONTROLLA(battery_percentage(&b) == 97);          /* sopra 16,4 V: 97 % */
	/* scendendo dalla base: per 6 s conta ancora come "in carica" (bit acceso per più dell'80 % degli ultimi 30 s),
	 * poi la stima parte dal valore raggiunto e scende piano verso la curva */
	battery_init(&b);
	for (int i = 0; i < 40000; i++)
		battery_update(&b, 163, 1, 1, 0, 0);
	CONTROLLA(battery_percentage(&b) == 98);
	scarica(&b, 162, 0, 300);
	CONTROLLA(b.status == BAT_STATE_CHARGING);
	scarica(&b, 162, 0, 1);
	CONTROLLA(b.status == BAT_STATE_IDLE);
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

static lidar_scan_t scan;
static int giri_visti;
static lidar_assembler_t ass;

static void raccogli(const lidar_packet_t *p, void *ctx)
{
	int64_t t = *(int64_t *)ctx;
	if (lidar_assemble(&ass, p, t, &scan))
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
	return lidar_build_packet(1, 0x61, LIDAR_SCAN_NEW, p, 5 + 3 * (size_t)n, out, 128);
}

static void lidar_revolutions(void)
{
	lidar_deframer_t d;
	lidar_deframer_init(&d);
	lidar_assembler_init(&ass, 800);
	giri_visti = 0;
	uint8_t b[256];
	int64_t t = RT_MS(1);
	size_t n = esa(VERO, b);
	CONTROLLA(lidar_deframer_feed(&d, b, n, raccogli, &t) == 1);   /* il pacchetto vero è valido */
	CONTROLLA(fabs(ass.rev_s - 5.3) < 1e-9);
	for (int s = 1; s < 16; s++) {
		t += RT_MS(10);
		n = settore(s, 21, 1000 + s, b);
		lidar_deframer_feed(&d, b, n, raccogli, &t);
	}
	CONTROLLA(giri_visti == 0);                         /* il giro si chiude solo quando l'angolo torna indietro */
	t += RT_MS(10);
	n = settore(0, 21, 500, b);
	lidar_deframer_feed(&d, b, n, raccogli, &t);
	CONTROLLA(giri_visti == 1);
	CONTROLLA(scan.sectors == 16 && scan.n == 24 + 15 * 21);
	/* primo punto del pacchetto vero: intensità 0, distanza 0 (nessuna eco); il secondo 0x95, 0x01dc/4 = 119 mm */
	CONTROLLA(scan.p[0].distance_mm == 0 && scan.p[1].intensity == 0x95 && scan.p[1].distance_mm == 119);
	/* angoli: inizio 0 - scarto 8° = 352°; il punto 1 è 22,5°/24 più avanti */
	CONTROLLA(scan.p[0].angle_cdeg == 35200);
	CONTROLLA(scan.p[1].angle_cdeg == (2250 / 24) - 800 + 36000);
	/* settore 1, punto 0: 22,5° - 8° = 14,5° */
	CONTROLLA(scan.p[24].angle_cdeg == 1450 && scan.p[24].distance_mm == 1001);
	CONTROLLA(scan.t_start_ns == RT_MS(1) && scan.t_end_ns == RT_MS(151));
	CONTROLLA(scan.number == 1);

	/* un giro incompleto (accensione) non si consegna */
	lidar_assembler_init(&ass, 0);
	giri_visti = 0;
	for (int s = 10; s < 16; s++) {
		n = settore(s, 21, 700, b);
		lidar_deframer_feed(&d, b, n, raccogli, &t);
	}
	n = settore(0, 21, 700, b);
	lidar_deframer_feed(&d, b, n, raccogli, &t);
	CONTROLLA(giri_visti == 0 && ass.discarded_scans == 1);

	/* 0xAE: solo la velocità */
	uint8_t v = 60;
	n = lidar_build_packet(1, 0x61, LIDAR_SPEED, &v, 1, b, sizeof b);
	lidar_deframer_feed(&d, b, n, raccogli, &t);
	CONTROLLA(fabs(ass.rev_s - 3.0) < 1e-9);
}

static void lidar_regolatore(void)
{
	/* il motore del lidar simulato come nella fase 2: giri/s = 0,08575 x potenza + 0,16, con inerzia */
	lidar_regulator_t r;
	lidar_regulator_init(&r, 6.0);
	double vel = 0;
	int p = 0;
	for (int i = 0; i < 500; i++) {                     /* 10 s a passi di 20 ms */
		p = lidar_regulate(&r, vel);
		double target = p < 15 ? 0 : 0.08575 * p + 0.16;
		vel += (target - vel) * (1 - exp(-0.02 / 0.6));
	}
	printf("  dopo 10 s: %.3f giri/s con potenza %d\n", vel, p);
	CONTROLLA(fabs(vel - 6.0) < 0.1);
	CONTROLLA(p >= 66 && p <= 70);
	CONTROLLA(r.integral <= 20 && r.integral >= -20);
	/* obiettivo zero: potenza zero */
	lidar_regulator_init(&r, 0);
	CONTROLLA(lidar_regulate(&r, 3) == 0);
	/* lidar fermo (nessuna misura): la potenza sale ma resta entro 100 */
	lidar_regulator_init(&r, 6.0);
	for (int i = 0; i < 100; i++)
		p = lidar_regulate(&r, 0);
	CONTROLLA(p == 100);
}

/* -------------------------------------------------------------------------------------- a terra --- */

static gd32_sensors_t sensori(void)
{
	gd32_sensors_t s;
	memset(&s, 0, sizeof s);
	s.control = 0x62;         /* sensori accesi */
	return s;
}

static void a_terra(void)
{
	gd32_sensors_t s = sensori();
	int32_t v = 1656, w = 400;
	CONTROLLA(interlock_ground_limit(&s, 0, &v, &w) == 0 && v == 1656 && w == 400);

	s.bumper_left = 1;
	v = 1656, w = 400;
	CONTROLLA(interlock_ground_limit(&s, 0, &v, &w) == INTERLOCK_LIM_BUMPER && v == 0 && w == 400);   /* ruotare si può */
	v = -800, w = 0;
	CONTROLLA(interlock_ground_limit(&s, 0, &v, &w) == INTERLOCK_LIM_BUMPER && v == -800);           /* indietro anche */

	s = sensori();
	s.cliff[1] = 1;
	v = 1656, w = 0;
	CONTROLLA(interlock_ground_limit(&s, 0, &v, &w) == INTERLOCK_LIM_CLIFF && v == 0);

	s = sensori();
	s.cliff[0] = s.cliff[1] = s.cliff[2] = s.cliff[3] = 1;                              /* sollevato */
	v = -800, w = 400;
	CONTROLLA(interlock_ground_limit(&s, 0, &v, &w) == INTERLOCK_LIM_LIFTED && v == 0 && w == 0);

	s = sensori();
	s.on_base = 1;
	s.control = 0x42;                                                                  /* sensori spenti */
	s.cliff[0] = s.cliff[1] = s.cliff[2] = s.cliff[3] = 1;                              /* sulla base: 0f */
	s.bumper_left = s.bumper_right = 1;                                                       /* spenti: "premuti" */
	v = -800, w = 0;
	CONTROLLA(interlock_ground_limit(&s, 0, &v, &w) == INTERLOCK_LIM_BASE && v == 0 && w == 0);
	v = -5000, w = 3000;
	CONTROLLA(interlock_ground_limit(&s, 1, &v, &w) == INTERLOCK_LIM_BASE && v == (int32_t)(-0.1 * GD32_SCALE_V) &&
	          w == (int32_t)(0.5 * GD32_SCALE_W));                                    /* uscita piano: 0,1 m/s, 0,5 rad/s */

	s = sensori();
	s.control = 0x42;                                                                  /* spenti fuori dalla base */
	v = 1656, w = 0;
	CONTROLLA(interlock_ground_limit(&s, 0, &v, &w) == INTERLOCK_LIM_SENSORS && v == 0);
}

/* -------------------------------------------------------------------------------------------- contatti --- */

#define MS(x) ((int64_t)(x) * 1000000)

static gd32_sensors_t sensori_a_terra(void)
{
	gd32_sensors_t s;
	memset(&s, 0, sizeof s);
	s.control = GD32_CONTROL_SENSORS_ON;
	return s;
}

/* robot inclinato (bordo di un tappeto): un "vuoto" non dà eventi, che si tratti di beccheggio o di rollio */
static void contatti_inclinato(void)
{
	contacts_t c;
	contacts_init(&c);
	gd32_sensors_t s = sensori_a_terra();
	int64_t t = MS(1000);

	/* rollio +6,5° (come nella prova p2): il laterale destro vede "vuoto" per 10 frame, nessun evento */
	s.angle[2] = 650;
	s.cliff[3] = 1;
	for (int i = 0; i < 10; i++)
		CONTROLLA(contacts_frame(&c, &s, t += MS(20), NULL, 0) == 0);
	CONTROLLA(c.published.last == 0);

	/* beccheggio -5,7°, rollio a posto: ancora nessun evento */
	s.angle[2] = 0;
	s.angle[1] = -570;
	for (int i = 0; i < 10; i++)
		CONTROLLA(contacts_frame(&c, &s, t += MS(20), NULL, 0) == 0);
	CONTROLLA(c.published.last == 0);

	/* torna in piano (entro la soglia) con il vuoto ancora lì: dopo 3 frame l'evento c'è */
	s.angle[1] = -150;
	CONTROLLA(contacts_frame(&c, &s, t += MS(20), NULL, 0) == 0);
	CONTROLLA(contacts_frame(&c, &s, t += MS(20), NULL, 0) == 0);
	CONTROLLA(contacts_frame(&c, &s, t += MS(20), NULL, 0) == 1);
	CONTROLLA(c.published.last == 1 && c.published.events[1].sensor == CONTACT_VOID_RIGHT_SIDE);

	/* il bumper invece conta anche da inclinato */
	s.angle[1] = -570;
	s.bumper_right = 1;
	CONTROLLA(contacts_frame(&c, &s, t += MS(20), NULL, 0) == 1);
	CONTROLLA(c.published.last == 2 && c.published.events[2].sensor == CONTACT_BUMPER_RIGHT);
}

static void contatti(void)
{
	contacts_t c;
	contacts_init(&c);
	gd32_sensors_t s = sensori_a_terra();
	int64_t t = MS(1000);

	/* niente attivo: niente eventi */
	CONTROLLA(contacts_frame(&c, &s, t, NULL, 0) == 0 && c.published.last == 0);

	/* anti-caduta 1: 2 frame non bastano, al terzo l'evento, con l'istante del primo */
	s.cliff[1] = 1;
	CONTROLLA(contacts_frame(&c, &s, t += MS(20), NULL, 0) == 0);
	int64_t primo = t;
	CONTROLLA(contacts_frame(&c, &s, t += MS(20), NULL, 0) == 0);
	CONTROLLA(contacts_frame(&c, &s, t += MS(20), NULL, 0) == 1);
	CONTROLLA(c.published.last == 1 && c.published.events[1].number == 1 && c.published.events[1].sensor == CONTACT_VOID_LEFT_FRONT);
	CONTROLLA(c.published.events[1].t_ns == primo);
	/* resta attivo: nessun altro evento */
	for (int i = 0; i < 10; i++)
		CONTROLLA(contacts_frame(&c, &s, t += MS(20), NULL, 0) == 0);
	CONTROLLA(c.published.last == 1);

	/* un lampo di 2 frame dopo essere tornato normale: niente (falso allarme filtrato) */
	s.cliff[1] = 0;
	contacts_frame(&c, &s, t += MS(20), NULL, 0);
	s.cliff[1] = 1;
	contacts_frame(&c, &s, t += MS(20), NULL, 0);
	contacts_frame(&c, &s, t += MS(20), NULL, 0);
	s.cliff[1] = 0;
	contacts_frame(&c, &s, t += MS(20), NULL, 0);
	CONTROLLA(c.published.last == 1);

	/* bumper destro: basta un frame */
	s.bumper_right = 1;
	CONTROLLA(contacts_frame(&c, &s, t += MS(20), NULL, 0) == 1);
	CONTROLLA(c.published.last == 2 && c.published.events[2].sensor == CONTACT_BUMPER_RIGHT && c.published.events[2].t_ns == t);
	CONTROLLA(contacts_frame(&c, &s, t += MS(20), NULL, 0) == 0);
	s.bumper_right = 0;
	contacts_frame(&c, &s, t += MS(20), NULL, 0);
	s.bumper_right = 1;
	CONTROLLA(contacts_frame(&c, &s, t += MS(20), NULL, 0) == 1 && c.published.last == 3);   /* nuovo scatto, nuovo evento */
	s.bumper_right = 0;

	/* robot sollevato (tutti e 4 nel vuoto): niente eventi, e i conteggi ripartono */
	s.cliff[0] = 1;
	contacts_frame(&c, &s, t += MS(20), NULL, 0);
	contacts_frame(&c, &s, t += MS(20), NULL, 0);
	s.cliff[1] = s.cliff[2] = s.cliff[3] = 1;
	for (int i = 0; i < 10; i++)
		CONTROLLA(contacts_frame(&c, &s, t += MS(20), NULL, 0) == 0);
	s.cliff[1] = s.cliff[2] = s.cliff[3] = 0;          /* posato con un solo sensore sul vuoto: si riconta da 1 */
	CONTROLLA(contacts_frame(&c, &s, t += MS(20), NULL, 0) == 0);
	CONTROLLA(c.published.last == 3);
	s.cliff[0] = 0;

	/* sensori spenti o robot sulla base: anti-caduta e bumper non significano niente */
	s.bumper_left = 1;
	s.control = 0;
	CONTROLLA(contacts_frame(&c, &s, t += MS(20), NULL, 0) == 0);
	s.control = GD32_CONTROL_SENSORS_ON;
	s.on_base = 1;
	CONTROLLA(contacts_frame(&c, &s, t += MS(20), NULL, 0) == 0);
	CONTROLLA(c.published.last == 3);
	s.on_base = 0;
	CONTROLLA(contacts_frame(&c, &s, t += MS(20), NULL, 0) == 1 && c.published.events[4].sensor == CONTACT_BUMPER_LEFT);
	s.bumper_left = 0;
	contacts_frame(&c, &s, t += MS(20), NULL, 0);

	/* lista circolare: dopo 40 eventi restano gli ultimi CONTACTS_N, ognuno al suo posto */
	for (int i = 0; i < 36; i++) {
		s.bumper_left = 1;
		contacts_frame(&c, &s, t += MS(20), NULL, 0);
		s.bumper_left = 0;
		contacts_frame(&c, &s, t += MS(20), NULL, 0);
	}
	CONTROLLA(c.published.last == 40);
	int ok = 1;
	for (uint32_t n = 40 - CONTACTS_N + 1; n <= 40; n++)
		ok &= c.published.events[n % CONTACTS_N].number == n;
	CONTROLLA(ok);

	/* posizione: l'evento porta quella data dal moto; senza, pose_valid = 0 */
	CONTROLLA(c.published.events[40 % CONTACTS_N].pose_valid == 0);
	drv_pose_t pose = { .x = 1.25, .y = -0.5, .theta = 0.75, .t_ns = t };
	s.bumper_left = 1;
	CONTROLLA(contacts_frame(&c, &s, t += MS(20), &pose, 0) == 1);
	drv_contact_t e = c.published.events[c.published.last % CONTACTS_N];
	CONTROLLA(e.pose_valid && e.pose.x == 1.25 && e.pose.y == -0.5 && e.pose.theta == 0.75 && e.pose.t_ns == pose.t_ns);
	s.bumper_left = 0;
	contacts_frame(&c, &s, t += MS(20), &pose, 0);
	/* anti-caduta: la posizione è quella del PRIMO frame attivo, non di quello in cui l'evento viene confermato */
	s.cliff[3] = 1;
	drv_pose_t p1 = { .x = 2, .t_ns = t }, p2 = { .x = 3, .t_ns = t + MS(20) }, p3 = { .x = 4, .t_ns = t + MS(40) };
	contacts_frame(&c, &s, t += MS(20), &p1, 0);
	contacts_frame(&c, &s, t += MS(20), &p2, 0);
	CONTROLLA(contacts_frame(&c, &s, t += MS(20), &p3, 0) == 1);
	e = c.published.events[c.published.last % CONTACTS_N];
	CONTROLLA(e.sensor == CONTACT_VOID_RIGHT_SIDE && e.pose_valid && e.pose.x == 2);
	s.cliff[3] = 0;
	contacts_frame(&c, &s, t += MS(20), &p3, 0);

	/* incastro: evento al primo frame segnalato, con la posizione di quel frame, uno solo finché resta segnalato */
	uint32_t prima = c.published.last;
	drv_pose_t pi = { .x = 5, .y = 6, .theta = 1, .t_ns = t };
	CONTROLLA(contacts_frame(&c, &s, t += MS(20), &pi, 1) == 1);
	e = c.published.events[c.published.last % CONTACTS_N];
	CONTROLLA(c.published.last == prima + 1 && e.sensor == CONTACT_STUCK && e.t_ns == t);
	CONTROLLA(e.pose_valid && e.pose.x == 5 && e.pose.y == 6 && e.pose.theta == 1);
	for (int i = 0; i < 10; i++)
		CONTROLLA(contacts_frame(&c, &s, t += MS(20), &p3, 1) == 0);
	/* breve (la sicurezza lo spegne dopo 100 ms): un evento lo stesso; tolto e ridato: un evento nuovo */
	contacts_frame(&c, &s, t += MS(20), &p3, 0);
	CONTROLLA(contacts_frame(&c, &s, t += MS(20), &p3, 1) == 1 && c.published.last == prima + 2);
	contacts_frame(&c, &s, t += MS(20), &p3, 0);
	/* robot sollevato o sulla base: niente incastri */
	s.cliff[0] = s.cliff[1] = s.cliff[2] = s.cliff[3] = 1;
	CONTROLLA(contacts_frame(&c, &s, t += MS(20), &p3, 1) == 0);
	s.cliff[0] = s.cliff[1] = s.cliff[2] = s.cliff[3] = 0;
	contacts_frame(&c, &s, t += MS(20), &p3, 0);
	s.on_base = 1;
	CONTROLLA(contacts_frame(&c, &s, t += MS(20), &p3, 1) == 0);
	s.on_base = 0;
	CONTROLLA(c.published.last == prima + 2);
	/* insieme al bumper: due eventi nello stesso frame */
	contacts_frame(&c, &s, t += MS(20), &p3, 0);
	s.bumper_right = 1;
	CONTROLLA(contacts_frame(&c, &s, t += MS(20), &p3, 1) == 1 && c.published.last == prima + 4);
	CONTROLLA(c.published.events[(prima + 3) % CONTACTS_N].sensor == CONTACT_BUMPER_RIGHT);
	CONTROLLA(c.published.events[(prima + 4) % CONTACTS_N].sensor == CONTACT_STUCK);
	s.bumper_right = 0;
}

/* --------------------------------------------------------------------------------------------- aggancio --- */

/* Modello: faccia della base su x = 0, robot in (x, y) con direzione th. I fasci hanno il confine sull'asse: un ricevitore a
 * y > 0 vede il fascio 1, a y < 0 il 2, e guardano solo davanti. I contatti si chiudono se arretra entro 8 cm dall'asse
 * [?: la larghezza vera non è misurata]: questo test prova la logica, non la precisione, che si vede solo sul robot. */
typedef struct {
	double x, y, th;
	int con_base;       /* 0 = la base non c'è */
	int charge;         /* 0 = contatto senza carica (batteria piena) */
	int asse_contatti;  /* 0 = i contatti non si chiudono mai */
	int spinte;         /* frame in cui ha spinto la base */
	int on_base;
	double t_contatto, t_fine;
} modello_agg_t;

static int ir_ricevitore(const modello_agg_t *m, double ly)
{
	if (!m->con_base)
		return 0;
	double rx = m->x + 0.15 * cos(m->th) - ly * sin(m->th), ry = m->y + 0.15 * sin(m->th) + ly * cos(m->th);
	double guarda = atan2(-ry, -rx) - m->th;
	while (guarda > M_PI) guarda -= 2 * M_PI;
	while (guarda < -M_PI) guarda += 2 * M_PI;
	if (fabs(guarda) > 0.45 || rx <= 0 || hypot(rx, ry) > 3)
		return 0;
	return ry >= 0 ? 1 : 2;
}

static void giro_modello(const modello_agg_t *m, lidar_scan_t *g)
{
	memset(g, 0, sizeof *g);
	if (!m->con_base || fabs(cos(m->th) + 1) > 0.2)   /* la faccia è davanti solo se la guarda */
		return;
	for (int a = 17000; a <= 19000; a += 100) {
		g->p[g->n].angle_cdeg = (uint16_t)a;
		g->p[g->n].distance_mm = (uint16_t)((m->x - 0.1745 + 0.1745 + 0.112) * 1000);
		g->n++;
	}
}

/* Ritorna l'esito, con t_max in s. */
static int simula_aggancio(modello_agg_t *m, double t_max)
{
	static docking_t a;
	static lidar_scan_t g;
	gd32_sensors_t s;
	memset(&s, 0, sizeof s);
	int64_t t = MS(1);
	double imb = 0, ex = 0;
	docking_start(&a, t);
	for (int i = 0; docking_active(&a) && t < MS((int64_t)(t_max * 1000)); i++) {
		t += MS(20);
		s.ir_dock_left = (uint8_t)ir_ricevitore(m, 0.08);   /* sinistro */
		s.ir_dock_right = (uint8_t)ir_ricevitore(m, -0.08);  /* destro */
		s.angle[0] = (int16_t)(((lround(imb) + 18000) % 36000 + 36000) % 36000 - 18000);   /* come il GD32: passa da +180° a -180° */
		s.encoder_left = s.encoder_right = (int32_t)lround(ex);
		s.on_base = m->on_base;
		s.charging = m->charge && m->on_base && t / 1e9 - m->t_contatto > 5;
		if (m->charge == 2)
			s.charging = m->on_base && t / 1e9 - m->t_contatto < 0.06;   /* 3 frame e poi solo contatto: la prova r2 */
		m->t_fine = t / 1e9 - m->t_contatto;
		s.bumper_left = cos(m->th) < 0 && m->x < 0.1745 + 0.005;
		if (i % 8 == 0) {
			giro_modello(m, &g);
			docking_scan(&a, &g, t);
		}
		docking_frame(&a, &s, t);
		double v = a.v, w = a.w;
		if (m->on_base && !docking_detach(&a))
			v = w = 0;
		if (s.bumper_left && v > 0)
			v = 0;
		if (cos(m->th) > 0 && m->x < 0.1745 && v < 0) {      /* di spalle, contro la faccia */
			m->spinte++;
			v = 0;
		}
		ex += (m->on_base && !docking_detach(&a) ? 0 : a.v) * 0.02 * GD32_SCALE_V;   /* contro la faccia le ruote slittano */
		m->x += v * 0.02 * cos(m->th);
		m->y += v * 0.02 * sin(m->th);
		m->th += w * 0.02;
		imb += w * 0.02 * 18000 / M_PI / 1.0285;
		int prima = m->on_base;
		m->on_base = m->asse_contatti && cos(m->th) > 0 && m->x < 0.1745 + 0.005 && fabs(m->y) < 0.08;
		if (m->on_base && !prima)
			m->t_contatto = t / 1e9;
	}
	return a.result;
}

static void aggancio_infrarossi(void)
{
	/* robot davanti alla base, a varie distanze, direzioni e scarti: si aggancia, sull'asse */
	static const struct { double x, y, th; } P[] = {
		{ 0.50, 0.00, M_PI }, { 0.50, 0.00, 0.0 }, { 0.50, 0.05, M_PI - 0.2 }, { 0.50, -0.05, M_PI + 0.2 },
		{ 1.00, 0.10, 1.0 }, { 1.50, -0.20, -2.0 }, { 0.40, 0.08, 0.5 },
	};
	for (size_t i = 0; i < sizeof P / sizeof P[0]; i++) {
		modello_agg_t m = { .x = P[i].x, .y = P[i].y, .th = P[i].th, .con_base = 1, .charge = 1, .asse_contatti = 1 };
		CONTROLLA(simula_aggancio(&m, 280) == DOCK_DONE);
		CONTROLLA(fabs(m.y) < 0.08);
		CONTROLLA(cos(m.th) > 0.9);                       /* di spalle */
	}
	/* senza base: un giro e poi rinuncia */
	modello_agg_t n = { .x = 1.0, .con_base = 0, .charge = 1, .asse_contatti = 1 };
	CONTROLLA(simula_aggancio(&n, 30) == DOCK_FAILED);
	/* contatto stabile senza il bit "in carica" (batteria scarica: il bit arriva solo a carica finita, prova carica1
	   dell'8/11): riesce dopo FRAME_CARICA frame di contatto, senza aspettare i 15 s */
	modello_agg_t p = { .x = 0.5, .th = M_PI, .con_base = 1, .charge = 0, .asse_contatti = 1 };
	CONTROLLA(simula_aggancio(&p, 280) == DOCK_DONE);
	CONTROLLA(p.t_fine < 2);
	/* bit "in carica" per 3 frame e poi solo contatto (la prova r2): conta il contatto, che è stabile */
	modello_agg_t c = { .x = 0.5, .th = M_PI, .con_base = 1, .charge = 2, .asse_contatti = 1 };
	CONTROLLA(simula_aggancio(&c, 280) == DOCK_DONE);
	CONTROLLA(c.t_fine < 2);
	/* contatti che non si chiudono: riprova DOCK_ATTEMPTS volte e rinuncia */
	modello_agg_t q = { .x = 0.5, .th = M_PI, .con_base = 1, .charge = 1, .asse_contatti = 0 };
	CONTROLLA(simula_aggancio(&q, 290) == DOCK_FAILED);
}

int main(void)
{
	ESEGUI(scambio_base);
	ESEGUI(scambio_con_thread);
	ESEGUI(battery);
	ESEGUI(lidar_revolutions);
	ESEGUI(lidar_regolatore);
	ESEGUI(a_terra);
	ESEGUI(contatti);
	ESEGUI(contatti_inclinato);
	ESEGUI(aggancio_infrarossi);
	return TEST_FINE();
}
