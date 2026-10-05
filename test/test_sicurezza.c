/* Test del blocco di sicurezza dei motori (gd32/sicurezza.h). */
#include <string.h>

#include "test.h"
#include "../gd32/gd32.h"
#include "../gd32/sicurezza.h"

#define MS(x) ((int64_t)(x) * 1000000)

static const uint8_t V_AVANTI[8] = { 0x10, 0x27, 0, 0, 0, 0, 0, 0 };   /* 0x66: v = 10000, omega = 0 */
static const uint8_t ZERO8[8];
static const uint8_t SPAZZOLA[1] = { 50 };

static gd32_frame_t sensori(uint8_t cliff)
{
	gd32_frame_t f = { .cmd = GD32_SENSORI, .n_dati = 96 };
	memset(f.dati, 0, 96);
	f.dati[3] = cliff;
	f.dati[64] = 0x62;          /* sensori alimentati (bit 5), come nei frame veri fuori dalla base */
	return f;
}

/* Anti-caduta su "vuoto" ma sulla base, oppure con i sensori spenti: non vale come "sollevato". */
static void base_e_sensori_spenti(void)
{
	sicurezza_t s;
	sic_init(&s, 1, MS(100));
	gd32_frame_t f = sensori(0x0f);
	f.dati[7] = 0x03;           /* sulla base, in carica */
	CONTROLLA(sic_ricevuto(&s, &f, MS(0)) == 0);
	CONTROLLA(sic_permesso(&s, GD32_VELOCITA, V_AVANTI, 8, MS(1)) == SIC_RIFIUTATO);
	f = sensori(0x0f);
	f.dati[64] = 0x42;          /* sensori spenti (0x9d 00) */
	sic_ricevuto(&s, &f, MS(20));
	CONTROLLA(sic_permesso(&s, GD32_VELOCITA, V_AVANTI, 8, MS(21)) == SIC_RIFIUTATO);
	CONTROLLA(!s.scattato);
	/* sollevato davvero, poi i sensori si spengono durante la prova: scatta */
	f = sensori(0x0f);
	sic_ricevuto(&s, &f, MS(40));
	CONTROLLA(sic_permesso(&s, GD32_VELOCITA, V_AVANTI, 8, MS(41)) == SIC_PERMESSO);
	f.dati[64] = 0x42;
	CONTROLLA(sic_ricevuto(&s, &f, MS(60)) == 1);
	/* e lo stesso se lo si appoggia sulla base */
	sic_init(&s, 1, MS(100));
	f = sensori(0x0f);
	sic_ricevuto(&s, &f, MS(0));
	f.dati[7] = 0x02;
	CONTROLLA(sic_ricevuto(&s, &f, MS(20)) == 1);
	CONTROLLA(sic_permesso(&s, GD32_VELOCITA, V_AVANTI, 8, MS(21)) == SIC_RIFIUTATO);
	/* frame corto: ignorato */
	sic_init(&s, 1, MS(100));
	f = sensori(0x0f);
	f.n_dati = 10;
	sic_ricevuto(&s, &f, MS(0));
	CONTROLLA(!s.ha_sensori);
}

static void senza_sollevato_niente_motori(void)
{
	sicurezza_t s;
	sic_init(&s, 0, MS(100));
	gd32_frame_t f = sensori(0x0f);
	CONTROLLA(sic_ricevuto(&s, &f, MS(0)) == 0);
	CONTROLLA(sic_permesso(&s, GD32_VELOCITA, V_AVANTI, 8, MS(1)) == SIC_RIFIUTATO);
	CONTROLLA(sic_permesso(&s, GD32_SPAZZOLA_LATERALE, SPAZZOLA, 1, MS(1)) == SIC_RIFIUTATO);
	CONTROLLA(s.rifiutati == 2);
	/* fermare si può sempre, e gli altri comandi passano */
	CONTROLLA(sic_permesso(&s, GD32_VELOCITA, ZERO8, 8, MS(1)) == SIC_PERMESSO);
	CONTROLLA(sic_permesso(&s, GD32_KEEPALIVE, NULL, 0, MS(1)) == SIC_PERMESSO);
	uint8_t uno = 1;
	CONTROLLA(sic_permesso(&s, GD32_ALIMENTAZIONE_LIDAR, &uno, 1, MS(1)) == SIC_PERMESSO);
	CONTROLLA(sic_permesso(&s, GD32_MOTORE_LIDAR, V_AVANTI, 4, MS(1)) == SIC_PERMESSO);
}

static void senza_sensori_niente_motori(void)
{
	sicurezza_t s;
	sic_init(&s, 1, MS(100));
	CONTROLLA(sic_permesso(&s, GD32_VELOCITA, V_AVANTI, 8, MS(0)) == SIC_RIFIUTATO);
	/* frame diversi dai sensori non contano */
	gd32_frame_t hb = { .cmd = GD32_HEARTBEAT, .n_dati = 0 };
	CONTROLLA(sic_ricevuto(&s, &hb, MS(1)) == 0);
	CONTROLLA(sic_permesso(&s, GD32_VELOCITA, V_AVANTI, 8, MS(1)) == SIC_RIFIUTATO);
	CONTROLLA(sic_controlla(&s, MS(10000)) == 0);    /* mai visto 0x0f: non c'è niente da far scattare */
}

static void appoggiato_niente_motori(void)
{
	sicurezza_t s;
	sic_init(&s, 1, MS(100));
	gd32_frame_t f = sensori(0x00);                  /* tutti gli anti-caduta vedono il pavimento */
	CONTROLLA(sic_ricevuto(&s, &f, MS(0)) == 0);     /* non scatta: non è mai stato sollevato */
	CONTROLLA(sic_permesso(&s, GD32_VELOCITA, V_AVANTI, 8, MS(1)) == SIC_RIFIUTATO);
	f = sensori(0x07);                               /* uno solo vede il pavimento */
	sic_ricevuto(&s, &f, MS(20));
	CONTROLLA(sic_permesso(&s, GD32_VELOCITA, V_AVANTI, 8, MS(21)) == SIC_RIFIUTATO);
	/* poi lo si solleva: da quel momento i motori sono permessi */
	f = sensori(0x0f);
	CONTROLLA(sic_ricevuto(&s, &f, MS(40)) == 0);
	CONTROLLA(sic_permesso(&s, GD32_VELOCITA, V_AVANTI, 8, MS(41)) == SIC_PERMESSO);
	CONTROLLA(!s.scattato);
}

static void cambia_durante_la_prova(void)
{
	sicurezza_t s;
	sic_init(&s, 1, MS(100));
	gd32_frame_t f = sensori(0x0f);
	sic_ricevuto(&s, &f, MS(0));
	CONTROLLA(sic_permesso(&s, GD32_VELOCITA, V_AVANTI, 8, MS(1)) == SIC_PERMESSO);
	CONTROLLA(sic_permesso(&s, GD32_SPAZZOLA_CENTRALE, SPAZZOLA, 1, MS(1)) == SIC_PERMESSO);
	/* un solo bit cambia: scatta subito, una volta sola */
	f = sensori(0x0e);
	CONTROLLA(sic_ricevuto(&s, &f, MS(20)) == 1);
	CONTROLLA(s.scattato && s.motivo != NULL && s.quando == MS(20));
	CONTROLLA(sic_ricevuto(&s, &f, MS(40)) == 0);
	CONTROLLA(sic_permesso(&s, GD32_VELOCITA, V_AVANTI, 8, MS(41)) == SIC_RIFIUTATO);
	/* anche se torna 0x0f, resta bloccato fino alla fine */
	f = sensori(0x0f);
	CONTROLLA(sic_ricevuto(&s, &f, MS(60)) == 0);
	CONTROLLA(sic_permesso(&s, GD32_VELOCITA, V_AVANTI, 8, MS(61)) == SIC_RIFIUTATO);
	CONTROLLA(sic_permesso(&s, GD32_ASPIRAZIONE, V_AVANTI, 2, MS(61)) == SIC_RIFIUTATO);
	/* fermare resta permesso */
	CONTROLLA(sic_permesso(&s, GD32_VELOCITA, ZERO8, 8, MS(61)) == SIC_PERMESSO);
}

static void sensori_vecchi(void)
{
	sicurezza_t s;
	sic_init(&s, 1, MS(100));
	gd32_frame_t f = sensori(0x0f);
	sic_ricevuto(&s, &f, MS(0));
	CONTROLLA(sic_controlla(&s, MS(100)) == 0);      /* 100 ms esatti: ancora buono */
	CONTROLLA(sic_permesso(&s, GD32_VELOCITA, V_AVANTI, 8, MS(100)) == SIC_PERMESSO);
	CONTROLLA(sic_controlla(&s, MS(101)) == 1);      /* poi scatta */
	CONTROLLA(sic_controlla(&s, MS(200)) == 0);      /* una volta sola */
	sic_ricevuto(&s, &f, MS(210));
	CONTROLLA(sic_permesso(&s, GD32_VELOCITA, V_AVANTI, 8, MS(211)) == SIC_RIFIUTATO);

	/* se nessuno chiama sic_controlla, ci pensa sic_permesso */
	sic_init(&s, 1, MS(100));
	sic_ricevuto(&s, &f, MS(0));
	CONTROLLA(sic_permesso(&s, GD32_VELOCITA, V_AVANTI, 8, MS(150)) == SIC_RIFIUTATO);
	CONTROLLA(s.scattato);
}

static void vietati(void)
{
	sicurezza_t s;
	sic_init(&s, 1, MS(100));
	gd32_frame_t f = sensori(0x0f);
	sic_ricevuto(&s, &f, MS(0));
	uint8_t zero = 0, uno = 1;
	CONTROLLA(sic_permesso(&s, GD32_POMPA_ACQUA, &zero, 1, MS(1)) == SIC_VIETATO);   /* anche a zero */
	CONTROLLA(sic_permesso(&s, GD32_ALIMENTAZIONE_SOC, &uno, 1, MS(1)) == SIC_VIETATO);
	CONTROLLA(sic_permesso(&s, GD32_RIAVVIA_SOC, NULL, 0, MS(1)) == SIC_VIETATO);
	CONTROLLA(s.vietati == 3 && s.rifiutati == 0);
	CONTROLLA(sic_e_motore(GD32_ASPIRAZIONE_PWM_BAT) && sic_e_motore(GD32_VELOCITA_RUOTE));
	CONTROLLA(!sic_e_motore(GD32_MOTORE_LIDAR) && !sic_e_motore(GD32_LED_TASTO));
}

/* I frame di arresto devono essere frame validi, uno per motore, con dati tutti a zero. */
static gd32_frame_t visti[16];
static size_t n_visti;
static void raccogli(const gd32_frame_t *f, void *ctx) { (void)ctx; if (n_visti < 16) visti[n_visti++] = *f; }

static void frame_arresto(void)
{
	uint8_t buf[256];
	size_t n = sic_frame_arresto(buf, sizeof buf);
	CONTROLLA(n > 0);
	gd32_deframer_t d;
	gd32_deframer_init(&d);
	n_visti = 0;
	gd32_deframer_scrivi(&d, buf, n, raccogli, NULL);
	CONTROLLA(n_visti == 6 && d.checksum_errati == 0 && d.byte_scartati == 0);
	int velocita = 0, spazzole = 0, aspirazione = 0;
	for (size_t i = 0; i < n_visti; i++) {
		CONTROLLA(sic_e_motore(visti[i].cmd));
		for (int k = 0; k < visti[i].n_dati; k++)
			CONTROLLA(visti[i].dati[k] == 0);
		velocita += visti[i].cmd == GD32_VELOCITA && visti[i].n_dati == 8;
		spazzole += visti[i].cmd == GD32_SPAZZOLA_LATERALE || visti[i].cmd == GD32_SPAZZOLA_CENTRALE;
		aspirazione += visti[i].cmd == GD32_ASPIRAZIONE && visti[i].n_dati == 2;
	}
	CONTROLLA(velocita == 1 && spazzole == 2 && aspirazione == 1);
	CONTROLLA(sic_frame_arresto(buf, 10) == 0);      /* buffer troppo piccolo */
}

/* Eccezione del banco di prova (2026-10-05): scatole sotto il muso, anti-caduta 0x09 invece di 0x0f. */
static void cliff_atteso(void)
{
	sicurezza_t s;
	sic_init(&s, 1, MS(100));
	gd32_frame_t f = sensori(0x09);
	sic_ricevuto(&s, &f, MS(0));
	CONTROLLA(sic_permesso(&s, GD32_VELOCITA, V_AVANTI, 8, MS(1)) == SIC_RIFIUTATO);   /* di regola serve 0x0f */
	sic_init(&s, 1, MS(100));
	sic_cliff_atteso(&s, 0x09);
	CONTROLLA(sic_ricevuto(&s, &f, MS(0)) == 0);
	CONTROLLA(sic_permesso(&s, GD32_VELOCITA, V_AVANTI, 8, MS(1)) == SIC_PERMESSO);
	/* ora 0x0f (lo sollevano dalle scatole) è un cambiamento: scatta */
	f = sensori(0x0f);
	CONTROLLA(sic_ricevuto(&s, &f, MS(20)) == 1);
	CONTROLLA(sic_permesso(&s, GD32_VELOCITA, V_AVANTI, 8, MS(21)) == SIC_RIFIUTATO);
	/* e se lo appoggiano (0x00) idem */
	sic_init(&s, 1, MS(100));
	sic_cliff_atteso(&s, 0x09);
	f = sensori(0x09);
	sic_ricevuto(&s, &f, MS(0));
	f = sensori(0x00);
	CONTROLLA(sic_ricevuto(&s, &f, MS(20)) == 1);
	/* sulla base resta vietato anche con lo schema giusto */
	sic_init(&s, 1, MS(100));
	sic_cliff_atteso(&s, 0x09);
	f = sensori(0x09);
	f.dati[7] = 0x02;
	sic_ricevuto(&s, &f, MS(0));
	CONTROLLA(sic_permesso(&s, GD32_VELOCITA, V_AVANTI, 8, MS(1)) == SIC_RIFIUTATO);
}

int main(void)
{
	ESEGUI(senza_sollevato_niente_motori);
	ESEGUI(senza_sensori_niente_motori);
	ESEGUI(appoggiato_niente_motori);
	ESEGUI(cambia_durante_la_prova);
	ESEGUI(sensori_vecchi);
	ESEGUI(base_e_sensori_spenti);
	ESEGUI(vietati);
	ESEGUI(frame_arresto);
	ESEGUI(cliff_atteso);
	return TEST_FINE();
}
