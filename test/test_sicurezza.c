/* Test del blocco di sicurezza dei motori (gd32/interlock.h). */
#include <string.h>

#include "test.h"
#include "../gd32/gd32.h"
#include "../gd32/interlock.h"

#define MS(x) ((int64_t)(x) * 1000000)

static const uint8_t V_AVANTI[8] = { 0x10, 0x27, 0, 0, 0, 0, 0, 0 };   /* 0x66: v = 10000, omega = 0 */
static const uint8_t ZERO8[8];
static const uint8_t SPAZZOLA[1] = { 50 };

static gd32_frame_t sensori(uint8_t cliff)
{
	gd32_frame_t f = { .cmd = GD32_SENSORS, .data_len = 96 };
	memset(f.data, 0, 96);
	f.data[3] = cliff;
	f.data[64] = 0x62;          /* sensori alimentati (bit 5), come nei frame veri fuori dalla base */
	return f;
}

/* Anti-caduta su "vuoto" ma sulla base, oppure con i sensori spenti: non vale come "sollevato". */
static void base_e_sensori_spenti(void)
{
	interlock_t s;
	interlock_init(&s, 1, MS(100));
	gd32_frame_t f = sensori(0x0f);
	f.data[7] = 0x03;           /* sulla base, in carica */
	CONTROLLA(interlock_received(&s, &f, MS(0)) == 0);
	CONTROLLA(interlock_permit(&s, GD32_VELOCITY, V_AVANTI, 8, MS(1)) == INTERLOCK_REFUSED);
	f = sensori(0x0f);
	f.data[64] = 0x42;          /* sensori spenti (0x9d 00) */
	interlock_received(&s, &f, MS(20));
	CONTROLLA(interlock_permit(&s, GD32_VELOCITY, V_AVANTI, 8, MS(21)) == INTERLOCK_REFUSED);
	CONTROLLA(!s.tripped);
	/* sollevato davvero, poi i sensori si spengono durante la prova: scatta */
	f = sensori(0x0f);
	interlock_received(&s, &f, MS(40));
	CONTROLLA(interlock_permit(&s, GD32_VELOCITY, V_AVANTI, 8, MS(41)) == INTERLOCK_ALLOWED);
	f.data[64] = 0x42;
	CONTROLLA(interlock_received(&s, &f, MS(60)) == 1);
	/* e lo stesso se lo si appoggia sulla base */
	interlock_init(&s, 1, MS(100));
	f = sensori(0x0f);
	interlock_received(&s, &f, MS(0));
	f.data[7] = 0x02;
	CONTROLLA(interlock_received(&s, &f, MS(20)) == 1);
	CONTROLLA(interlock_permit(&s, GD32_VELOCITY, V_AVANTI, 8, MS(21)) == INTERLOCK_REFUSED);
	/* frame corto: ignorato */
	interlock_init(&s, 1, MS(100));
	f = sensori(0x0f);
	f.data_len = 10;
	interlock_received(&s, &f, MS(0));
	CONTROLLA(!s.has_sensors);
}

static void senza_sollevato_niente_motori(void)
{
	interlock_t s;
	interlock_init(&s, 0, MS(100));
	gd32_frame_t f = sensori(0x0f);
	CONTROLLA(interlock_received(&s, &f, MS(0)) == 0);
	CONTROLLA(interlock_permit(&s, GD32_VELOCITY, V_AVANTI, 8, MS(1)) == INTERLOCK_REFUSED);
	CONTROLLA(interlock_permit(&s, GD32_SIDE_BRUSH, SPAZZOLA, 1, MS(1)) == INTERLOCK_REFUSED);
	CONTROLLA(s.refused == 2);
	/* fermare si può sempre, e gli altri comandi passano */
	CONTROLLA(interlock_permit(&s, GD32_VELOCITY, ZERO8, 8, MS(1)) == INTERLOCK_ALLOWED);
	CONTROLLA(interlock_permit(&s, GD32_KEEPALIVE, NULL, 0, MS(1)) == INTERLOCK_ALLOWED);
	uint8_t uno = 1;
	CONTROLLA(interlock_permit(&s, GD32_LIDAR_POWER, &uno, 1, MS(1)) == INTERLOCK_ALLOWED);
	CONTROLLA(interlock_permit(&s, GD32_LIDAR_MOTOR, V_AVANTI, 4, MS(1)) == INTERLOCK_ALLOWED);
}

static void senza_sensori_niente_motori(void)
{
	interlock_t s;
	interlock_init(&s, 1, MS(100));
	CONTROLLA(interlock_permit(&s, GD32_VELOCITY, V_AVANTI, 8, MS(0)) == INTERLOCK_REFUSED);
	/* frame diversi dai sensori non contano */
	gd32_frame_t hb = { .cmd = GD32_HEARTBEAT, .data_len = 0 };
	CONTROLLA(interlock_received(&s, &hb, MS(1)) == 0);
	CONTROLLA(interlock_permit(&s, GD32_VELOCITY, V_AVANTI, 8, MS(1)) == INTERLOCK_REFUSED);
	CONTROLLA(interlock_check(&s, MS(10000)) == 0);    /* mai visto 0x0f: non c'è niente da far scattare */
}

static void appoggiato_niente_motori(void)
{
	interlock_t s;
	interlock_init(&s, 1, MS(100));
	gd32_frame_t f = sensori(0x00);                  /* tutti gli anti-caduta vedono il pavimento */
	CONTROLLA(interlock_received(&s, &f, MS(0)) == 0);     /* non scatta: non è mai stato sollevato */
	CONTROLLA(interlock_permit(&s, GD32_VELOCITY, V_AVANTI, 8, MS(1)) == INTERLOCK_REFUSED);
	f = sensori(0x07);                               /* uno solo vede il pavimento */
	interlock_received(&s, &f, MS(20));
	CONTROLLA(interlock_permit(&s, GD32_VELOCITY, V_AVANTI, 8, MS(21)) == INTERLOCK_REFUSED);
	/* poi lo si solleva: da quel momento i motori sono permessi */
	f = sensori(0x0f);
	CONTROLLA(interlock_received(&s, &f, MS(40)) == 0);
	CONTROLLA(interlock_permit(&s, GD32_VELOCITY, V_AVANTI, 8, MS(41)) == INTERLOCK_ALLOWED);
	CONTROLLA(!s.tripped);
}

static void cambia_durante_la_prova(void)
{
	interlock_t s;
	interlock_init(&s, 1, MS(100));
	gd32_frame_t f = sensori(0x0f);
	interlock_received(&s, &f, MS(0));
	CONTROLLA(interlock_permit(&s, GD32_VELOCITY, V_AVANTI, 8, MS(1)) == INTERLOCK_ALLOWED);
	CONTROLLA(interlock_permit(&s, GD32_MAIN_BRUSH, SPAZZOLA, 1, MS(1)) == INTERLOCK_ALLOWED);
	/* un solo bit cambia: scatta subito, una volta sola */
	f = sensori(0x0e);
	CONTROLLA(interlock_received(&s, &f, MS(20)) == 1);
	CONTROLLA(s.tripped && s.reason != NULL && s.tripped_ns == MS(20));
	CONTROLLA(interlock_received(&s, &f, MS(40)) == 0);
	CONTROLLA(interlock_permit(&s, GD32_VELOCITY, V_AVANTI, 8, MS(41)) == INTERLOCK_REFUSED);
	/* anche se torna 0x0f, resta bloccato fino alla fine */
	f = sensori(0x0f);
	CONTROLLA(interlock_received(&s, &f, MS(60)) == 0);
	CONTROLLA(interlock_permit(&s, GD32_VELOCITY, V_AVANTI, 8, MS(61)) == INTERLOCK_REFUSED);
	CONTROLLA(interlock_permit(&s, GD32_SUCTION, V_AVANTI, 2, MS(61)) == INTERLOCK_REFUSED);
	/* fermare resta permesso */
	CONTROLLA(interlock_permit(&s, GD32_VELOCITY, ZERO8, 8, MS(61)) == INTERLOCK_ALLOWED);
}

static void sensori_vecchi(void)
{
	interlock_t s;
	interlock_init(&s, 1, MS(100));
	gd32_frame_t f = sensori(0x0f);
	interlock_received(&s, &f, MS(0));
	CONTROLLA(interlock_check(&s, MS(100)) == 0);      /* 100 ms esatti: ancora buono */
	CONTROLLA(interlock_permit(&s, GD32_VELOCITY, V_AVANTI, 8, MS(100)) == INTERLOCK_ALLOWED);
	CONTROLLA(interlock_check(&s, MS(101)) == 1);      /* poi scatta */
	CONTROLLA(interlock_check(&s, MS(200)) == 0);      /* una volta sola */
	interlock_received(&s, &f, MS(210));
	CONTROLLA(interlock_permit(&s, GD32_VELOCITY, V_AVANTI, 8, MS(211)) == INTERLOCK_REFUSED);

	/* se nessuno chiama interlock_check, ci pensa interlock_permit */
	interlock_init(&s, 1, MS(100));
	interlock_received(&s, &f, MS(0));
	CONTROLLA(interlock_permit(&s, GD32_VELOCITY, V_AVANTI, 8, MS(150)) == INTERLOCK_REFUSED);
	CONTROLLA(s.tripped);
}

static void forbidden(void)
{
	interlock_t s;
	interlock_init(&s, 1, MS(100));
	gd32_frame_t f = sensori(0x0f);
	interlock_received(&s, &f, MS(0));
	uint8_t zero = 0, uno = 1;
	CONTROLLA(interlock_permit(&s, GD32_WATER_PUMP, &zero, 1, MS(1)) == INTERLOCK_FORBIDDEN);   /* anche a zero */
	CONTROLLA(interlock_permit(&s, GD32_SOC_POWER, &uno, 1, MS(1)) == INTERLOCK_FORBIDDEN);
	CONTROLLA(interlock_permit(&s, GD32_SOC_RESTART, NULL, 0, MS(1)) == INTERLOCK_FORBIDDEN);
	CONTROLLA(s.forbidden == 3 && s.refused == 0);
	CONTROLLA(interlock_is_motor(GD32_SUCTION_PWM_BATTERY) && interlock_is_motor(GD32_WHEEL_SPEED));
	CONTROLLA(!interlock_is_motor(GD32_LIDAR_MOTOR) && !interlock_is_motor(GD32_BUTTON_LED));
}

/* I frame di arresto devono essere frame validi, uno per motore, con dati tutti a zero. */
static gd32_frame_t visti[16];
static size_t n_visti;
static void raccogli(const gd32_frame_t *f, void *ctx) { (void)ctx; if (n_visti < 16) visti[n_visti++] = *f; }

static void frame_arresto(void)
{
	uint8_t buf[256];
	size_t n = interlock_stop_frames(buf, sizeof buf);
	CONTROLLA(n > 0);
	gd32_deframer_t d;
	gd32_deframer_init(&d);
	n_visti = 0;
	gd32_deframer_feed(&d, buf, n, raccogli, NULL);
	CONTROLLA(n_visti == 6 && d.bad_checksums == 0 && d.discarded_bytes == 0);
	int velocita = 0, spazzole = 0, suction = 0;
	for (size_t i = 0; i < n_visti; i++) {
		CONTROLLA(interlock_is_motor(visti[i].cmd));
		for (int k = 0; k < visti[i].data_len; k++)
			CONTROLLA(visti[i].data[k] == 0);
		velocita += visti[i].cmd == GD32_VELOCITY && visti[i].data_len == 8;
		spazzole += visti[i].cmd == GD32_SIDE_BRUSH || visti[i].cmd == GD32_MAIN_BRUSH;
		suction += visti[i].cmd == GD32_SUCTION && visti[i].data_len == 2;
	}
	CONTROLLA(velocita == 1 && spazzole == 2 && suction == 1);
	CONTROLLA(interlock_stop_frames(buf, 10) == 0);      /* buffer troppo piccolo */
}

/* Eccezione del banco di prova (2026-10-05): scatole sotto il muso, anti-caduta 0x09 invece di 0x0f. */
static void expected_cliff(void)
{
	interlock_t s;
	interlock_init(&s, 1, MS(100));
	gd32_frame_t f = sensori(0x09);
	interlock_received(&s, &f, MS(0));
	CONTROLLA(interlock_permit(&s, GD32_VELOCITY, V_AVANTI, 8, MS(1)) == INTERLOCK_REFUSED);   /* di regola serve 0x0f */
	interlock_init(&s, 1, MS(100));
	interlock_set_expected_cliff(&s, 0x09);
	CONTROLLA(interlock_received(&s, &f, MS(0)) == 0);
	CONTROLLA(interlock_permit(&s, GD32_VELOCITY, V_AVANTI, 8, MS(1)) == INTERLOCK_ALLOWED);
	/* ora 0x0f (lo sollevano dalle scatole) è un cambiamento: scatta */
	f = sensori(0x0f);
	CONTROLLA(interlock_received(&s, &f, MS(20)) == 1);
	CONTROLLA(interlock_permit(&s, GD32_VELOCITY, V_AVANTI, 8, MS(21)) == INTERLOCK_REFUSED);
	/* e se lo appoggiano (0x00) idem */
	interlock_init(&s, 1, MS(100));
	interlock_set_expected_cliff(&s, 0x09);
	f = sensori(0x09);
	interlock_received(&s, &f, MS(0));
	f = sensori(0x00);
	CONTROLLA(interlock_received(&s, &f, MS(20)) == 1);
	/* sulla base resta vietato anche con lo schema giusto */
	interlock_init(&s, 1, MS(100));
	interlock_set_expected_cliff(&s, 0x09);
	f = sensori(0x09);
	f.data[7] = 0x02;
	interlock_received(&s, &f, MS(0));
	CONTROLLA(interlock_permit(&s, GD32_VELOCITY, V_AVANTI, 8, MS(1)) == INTERLOCK_REFUSED);
}

int main(void)
{
	ESEGUI(senza_sollevato_niente_motori);
	ESEGUI(senza_sensori_niente_motori);
	ESEGUI(appoggiato_niente_motori);
	ESEGUI(cambia_durante_la_prova);
	ESEGUI(sensori_vecchi);
	ESEGUI(base_e_sensori_spenti);
	ESEGUI(forbidden);
	ESEGUI(frame_arresto);
	ESEGUI(expected_cliff);
	return TEST_FINE();
}
