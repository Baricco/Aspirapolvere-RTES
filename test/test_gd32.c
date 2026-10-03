/* Test del protocollo GD32 (gd32/gd32.h), con frame veri presi dalle catture di AuxCtrl. */
#include <string.h>

#include "test.h"
#include "../gd32/gd32.h"

/* Dati di un frame 0x15 vero (cattura-ricarica, 2026-10-02): robot sulla base, tensione 0x7e = 12,6 V. */
static const uint8_t SENSORI_VERO[96] = {
	0x01, 0x06, 0x00, 0x0f, 0x00, 0x00, 0x00, 0x02, 0x7e, 0x1b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x78, 0x69, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x78, 0x69, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfe, 0xff, 0x1e, 0x00,
	0x00, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x42, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00,
	0x38, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x03, 0x19, 0xfd, 0xff, 0x00, 0x00, 0xfe, 0xff,
};

/* Raccoglie i frame restituiti dal deframer. */
typedef struct {
	gd32_frame_t f[64];
	size_t n;
} raccolta_t;

static void raccogli(const gd32_frame_t *f, void *ctx)
{
	raccolta_t *r = ctx;
	if (r->n < 64)
		r->f[r->n++] = *f;
}

static void checksum_frame_veri(void)
{
	/* heartbeat: FA FB 03 06 00 06 (un solo byte, dispari: 0 ^ 0x06) */
	uint8_t hb[] = { 0x06 };
	CONTROLLA(gd32_checksum(hb, 1) == 0x0006);
	/* richiesta di avvio 0x0c 01: parola 0x0c01 */
	uint8_t avvio[] = { 0x0c, 0x01 };
	CONTROLLA(gd32_checksum(avvio, 2) == 0x0c01);
	/* 0x17 01 80 cf f0: 0x1701 + 0x80cf, poi 0xf0 in XOR */
	uint8_t s17[] = { 0x17, 0x01, 0x80, 0xcf, 0xf0 };
	CONTROLLA(gd32_checksum(s17, 5) == (uint16_t)((0x1701 + 0x80cf) ^ 0xf0));
	CONTROLLA(gd32_checksum(NULL, 0) == 0);
}

static void componi_heartbeat_e_velocita(void)
{
	uint8_t out[GD32_MAX_FRAME];
	uint8_t atteso_hb[] = { 0xfa, 0xfb, 0x03, 0x06, 0x00, 0x06 };
	CONTROLLA(gd32_componi(GD32_HEARTBEAT, NULL, 0, out, sizeof out) == 6);
	CONTROLLA(memcmp(out, atteso_hb, 6) == 0);

	/* velocità zero: 8 byte a zero, checksum = 0x6600 (parola 66 00, poi 4 parole nulle, poi 00 in XOR) */
	uint8_t zeri[8] = { 0 };
	CONTROLLA(gd32_componi(GD32_VELOCITA, zeri, 8, out, sizeof out) == 14);
	CONTROLLA(out[2] == 11);
	CONTROLLA(out[12] == 0x66 && out[13] == 0x00);

	/* limiti */
	CONTROLLA(gd32_componi(0x01, zeri, GD32_MAX_DATI + 1, out, sizeof out) == 0);
	CONTROLLA(gd32_componi(0x01, zeri, 8, out, 13) == 0);
	uint8_t grande[GD32_MAX_DATI] = { 0 };
	CONTROLLA(gd32_componi(0x01, grande, GD32_MAX_DATI, out, sizeof out) == GD32_MAX_FRAME);
	CONTROLLA(out[2] == 255);
}

static void frame_intero(void)
{
	uint8_t fr[GD32_MAX_FRAME];
	size_t n = gd32_componi(GD32_SENSORI, SENSORI_VERO, 96, fr, sizeof fr);
	CONTROLLA(n == 102);
	gd32_deframer_t d;
	gd32_deframer_init(&d);
	raccolta_t r = { .n = 0 };
	CONTROLLA(gd32_deframer_scrivi(&d, fr, n, raccogli, &r) == 1);
	CONTROLLA(r.n == 1);
	CONTROLLA(r.f[0].cmd == GD32_SENSORI && r.f[0].n_dati == 96);
	CONTROLLA(memcmp(r.f[0].dati, SENSORI_VERO, 96) == 0);
	CONTROLLA(d.frame_ok == 1 && d.checksum_errati == 0 && d.byte_scartati == 0);

	gd32_sensori_t s;
	CONTROLLA(gd32_leggi_sensori(&r.f[0], &s) == 0);
	CONTROLLA(s.tensione_dv == 126 && s.sulla_base == 1 && s.in_carica == 0);
	gd32_frame_t hb = { .cmd = GD32_HEARTBEAT, .n_dati = 0 };
	CONTROLLA(gd32_leggi_sensori(&hb, &s) == -1);
	gd32_frame_t corto = { .cmd = GD32_SENSORI, .n_dati = 95 };
	CONTROLLA(gd32_leggi_sensori(&corto, &s) == -1);
}

/* Frame 0x15 vero (spia, 2026-10-03): robot sollevato fuori dalla base, 16,2 V, cassetto presente. */
static const uint8_t SENSORI_SOLLEVATO[96] = {
	0x01, 0x00, 0x00, 0x0f, 0x04, 0x00, 0x00, 0x00, 0xa2, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x78, 0x69, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x6d, 0x69, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfb, 0xff, 0x09, 0x00,
	0xff, 0xff, 0xda, 0xff, 0x00, 0x00, 0x00, 0x00, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x62, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00,
	0x39, 0x00, 0x0e, 0x00, 0x02, 0x00, 0x00, 0x00, 0x03, 0x19, 0xfe, 0xff, 0xff, 0xff, 0xfb, 0xff,
};

/* Decodifica completa dei due frame veri: ogni campo con il valore atteso. */
static void sensori_decodifica(void)
{
	gd32_frame_t f = { .cmd = GD32_SENSORI, .n_dati = 96 };
	gd32_sensori_t s;

	memcpy(f.dati, SENSORI_VERO, 96);           /* sulla base */
	CONTROLLA(gd32_leggi_sensori(&f, &s) == 0);
	CONTROLLA(s.stato == 1);
	CONTROLLA(s.bumper_sx == 1 && s.bumper_dx == 1 && s.bumper_bit == 0x06);
	CONTROLLA(s.cliff[0] && s.cliff[1] && s.cliff[2] && s.cliff[3]);
	CONTROLLA(s.cassetto == 0);
	CONTROLLA(s.in_carica == 0 && s.sulla_base == 1);
	CONTROLLA(s.temperatura_c == 27);
	CONTROLLA(s.encoder_sx == 0x6978 && s.encoder_dx == 0x6978);
	CONTROLLA(s.vel_angolare[1] == -2 && s.angolo[1] == 30 && s.angolo[2] == 5);
	CONTROLLA(s.gravita[0] == 0 && s.gravita[2] == 100);
	CONTROLLA(s.controllo == 0x42 && s.led == 0x08);
	CONTROLLA(s.correnti[2] == 3 && s.correnti[3] == 0x38 && s.correnti[5] == 3);
	CONTROLLA(s.byte88 == 0x1903 && s.magnetico[0] == -3 && s.magnetico[2] == -2);

	memcpy(f.dati, SENSORI_SOLLEVATO, 96);      /* sollevato, fuori dalla base */
	CONTROLLA(gd32_leggi_sensori(&f, &s) == 0);
	CONTROLLA(s.bumper_sx == 0 && s.bumper_dx == 0);
	CONTROLLA(f.dati[3] == GD32_CLIFF_TUTTI_VUOTO && s.cliff[0] && s.cliff[3]);
	CONTROLLA(s.cassetto == 1 && s.sulla_base == 0 && s.in_carica == 0);
	CONTROLLA(s.tensione_dv == 162 && s.temperatura_c == 31);
	CONTROLLA(s.encoder_dx == 0x696d);
	CONTROLLA(s.vel_angolare[1] == -5 && s.angolo[1] == 9 && s.vel_angolare[2] == -1 && s.angolo[2] == -38);
	CONTROLLA(s.controllo == 0x62 && s.led == 0x01);
	CONTROLLA(s.magnetico[1] == -1 && s.magnetico[2] == -5);

	/* campi con segno e scale, su un frame costruito apposta */
	uint8_t d[96] = { 0 };
	d[1] = 0x09;                /* solo i contatti "secondari": bit 3 (sx) e bit 0 (dx) */
	d[3] = 0x08;                /* solo il bit 3: anti-caduta destro laterale */
	d[6] = 0x4b;
	d[7] = 0x01;                /* in carica => anche sulla base */
	d[9] = 0xf6;                /* -10 °C */
	d[16] = 0xff; d[17] = 0xff; d[18] = 0xff; d[19] = 0xff;   /* encoder -1 */
	d[32] = 0x10; d[35] = 0x80; /* int32 negativo */
	d[58] = 0x2c; d[59] = 0x01; /* 300 x 10 ms */
	d[69] = 3;
	f.n_dati = 96;
	memcpy(f.dati, d, 96);
	CONTROLLA(gd32_leggi_sensori(&f, &s) == 0);
	CONTROLLA(s.bumper_sx == 1 && s.bumper_dx == 1);
	CONTROLLA(s.cliff[0] == 0 && s.cliff[1] == 0 && s.cliff[2] == 0 && s.cliff[3] == 1);
	CONTROLLA(s.ir_base_a == 4 && s.ir_base_b == 0x0b);
	CONTROLLA(s.in_carica == 1 && s.sulla_base == 1);
	CONTROLLA(s.temperatura_c == -10);
	CONTROLLA(s.encoder_sx == -1);
	CONTROLLA(s.vel_sx == (int32_t)0x80000010);
	CONTROLLA(s.tasto_accensione_ms == 3000);
	CONTROLLA(s.byte69_x100 == 300);
}

/* Lo stesso flusso di 3 frame, spezzato in blocchi di ogni dimensione da 1 a tutta la lunghezza. */
static void frame_spezzati_in_ogni_modo(void)
{
	uint8_t flusso[3 * GD32_MAX_FRAME];
	size_t n = 0;
	uint8_t v[8] = { 0xf9, 0xff, 0xff, 0xff, 0, 0, 0, 0 };
	n += gd32_componi(GD32_HEARTBEAT, NULL, 0, flusso + n, sizeof flusso - n);
	n += gd32_componi(GD32_SENSORI, SENSORI_VERO, 96, flusso + n, sizeof flusso - n);
	n += gd32_componi(GD32_VELOCITA, v, 8, flusso + n, sizeof flusso - n);

	int tutti_ok = 1;
	for (size_t blocco = 1; blocco <= n; blocco++) {
		gd32_deframer_t d;
		gd32_deframer_init(&d);
		raccolta_t r = { .n = 0 };
		for (size_t i = 0; i < n; i += blocco) {
			size_t k = n - i < blocco ? n - i : blocco;
			gd32_deframer_scrivi(&d, flusso + i, k, raccogli, &r);
		}
		if (r.n != 3 || r.f[0].cmd != GD32_HEARTBEAT || r.f[1].cmd != GD32_SENSORI ||
		    r.f[2].cmd != GD32_VELOCITA || memcmp(r.f[2].dati, v, 8) != 0 || d.byte_scartati != 0)
			tutti_ok = 0;
	}
	CONTROLLA(tutti_ok);
}

/*
 * Frame troncato come quelli di strace: dei 102 byte di un 0x15 ne arrivano solo 52, poi inizia il frame
 * successivo. Il deframer conta len = 99 byte, mangiando anche l'inizio del frame buono; il checksum non torna,
 * e rianalizzando deve ritrovare il frame buono.
 */
static void frame_troncato_e_risincronizzazione(void)
{
	uint8_t flusso[4 * GD32_MAX_FRAME];
	uint8_t fr[GD32_MAX_FRAME];
	size_t n = 0;
	size_t k = gd32_componi(GD32_SENSORI, SENSORI_VERO, 96, fr, sizeof fr);
	memcpy(flusso, fr, 52);
	n = 52;
	n += gd32_componi(GD32_SENSORI, SENSORI_VERO, 96, flusso + n, sizeof flusso - n);
	n += gd32_componi(GD32_HEARTBEAT, NULL, 0, flusso + n, sizeof flusso - n);
	CONTROLLA(k == 102);

	gd32_deframer_t d;
	gd32_deframer_init(&d);
	raccolta_t r = { .n = 0 };
	gd32_deframer_scrivi(&d, flusso, n, raccogli, &r);
	CONTROLLA(r.n == 2);
	CONTROLLA(r.n >= 1 && r.f[0].cmd == GD32_SENSORI && memcmp(r.f[0].dati, SENSORI_VERO, 96) == 0);
	CONTROLLA(r.n >= 2 && r.f[1].cmd == GD32_HEARTBEAT);
	CONTROLLA(d.checksum_errati >= 1);
	CONTROLLA(d.byte_scartati == 52);   /* esattamente i byte del frame troncato */
}

/* Rumore prima del frame, un FA isolato, un "FA FB" dentro i dati, len < 3. */
static void casi_limite(void)
{
	uint8_t dati_con_fafb[] = { 0x01, 0xfa, 0xfb, 0x02 };
	uint8_t fr[GD32_MAX_FRAME];
	uint8_t flusso[64];
	size_t n = 0;
	flusso[n++] = 0x00;
	flusso[n++] = 0xfa;                 /* FA non seguito da FB */
	flusso[n++] = 0x11;
	flusso[n++] = 0xfa;                 /* FA FB con len 2: non valido */
	flusso[n++] = 0xfb;
	flusso[n++] = 0x02;
	size_t k = gd32_componi(0x8d, dati_con_fafb, 4, fr, sizeof fr);
	memcpy(flusso + n, fr, k);
	n += k;

	gd32_deframer_t d;
	gd32_deframer_init(&d);
	raccolta_t r = { .n = 0 };
	gd32_deframer_scrivi(&d, flusso, n, raccogli, &r);
	CONTROLLA(r.n == 1);
	CONTROLLA(r.n == 1 && r.f[0].cmd == 0x8d && r.f[0].n_dati == 4 && memcmp(r.f[0].dati, dati_con_fafb, 4) == 0);
	CONTROLLA(d.len_errati == 1);
	CONTROLLA(d.byte_scartati == 6);
}

/* Molti frame rovinati di fila: la coda di rianalisi non deve mai traboccare. */
static void molti_errori_di_fila(void)
{
	uint8_t flusso[4096];
	size_t n = 0;
	/* 30 intestazioni con len massimo seguite da pochi byte: ognuna "mangia" la successiva */
	for (int i = 0; i < 30; i++) {
		flusso[n++] = 0xfa;
		flusso[n++] = 0xfb;
		flusso[n++] = 0xff;
		for (int j = 0; j < 40; j++)
			flusso[n++] = (uint8_t)(j * 7);
	}
	n += gd32_componi(GD32_HEARTBEAT, NULL, 0, flusso + n, sizeof flusso - n);
	/* riempimento finale: serve perché l'ultima intestazione con len 255 possa "chiudersi" e venire scartata */
	memset(flusso + n, 0, 300);
	n += 300;
	n += gd32_componi(GD32_HEARTBEAT, NULL, 0, flusso + n, sizeof flusso - n);

	gd32_deframer_t d;
	gd32_deframer_init(&d);
	raccolta_t r = { .n = 0 };
	gd32_deframer_scrivi(&d, flusso, n, raccogli, &r);
	CONTROLLA(r.n == 2);
	CONTROLLA(d.frame_ok == 2);
	CONTROLLA(d.attesa_n == 0 && d.n == 0);
}

/* Generatore pseudo-casuale semplice (xorshift32): stessa sequenza a ogni esecuzione. */
static uint32_t caso_stato = 12345;
static uint32_t caso(void)
{
	caso_stato ^= caso_stato << 13;
	caso_stato ^= caso_stato >> 17;
	caso_stato ^= caso_stato << 5;
	return caso_stato;
}

typedef struct {
	size_t n;
	int ordine_ok;
	uint8_t atteso_cmd[4096];
	size_t attesi;
} conta_t;

static void conta(const gd32_frame_t *f, void *ctx)
{
	conta_t *c = ctx;
	if (c->n >= c->attesi || f->cmd != c->atteso_cmd[c->n])
		c->ordine_ok = 0;
	c->n++;
}

/*
 * 3000 frame validi di lunghezza casuale, intervallati da rumore (senza byte FA) e da frame troncati,
 * consegnati in blocchi di dimensione casuale. Devono uscire tutti, nell'ordine giusto.
 */
static void flusso_casuale(void)
{
	static uint8_t flusso[3000 * (GD32_MAX_FRAME + 40)];
	static conta_t c;
	size_t n = 0;
	memset(&c, 0, sizeof c);
	c.ordine_ok = 1;
	for (int i = 0; i < 3000; i++) {
		uint8_t dati[GD32_MAX_DATI], fr[GD32_MAX_FRAME];
		size_t nd = caso() % 40;
		for (size_t j = 0; j < nd; j++)
			dati[j] = (uint8_t)caso();
		uint8_t cmd = (uint8_t)(caso() % 0xf0);
		size_t k = gd32_componi(cmd, dati, nd, fr, sizeof fr);
		if (caso() % 5 == 0 && k > 4) {
			/* frame troncato: si spedisce solo l'inizio, e non conta tra quelli attesi */
			size_t tronco = 3 + caso() % (k - 3);
			memcpy(flusso + n, fr, tronco);
			n += tronco;
		} else {
			memcpy(flusso + n, fr, k);
			n += k;
			if (c.attesi < sizeof c.atteso_cmd)
				c.atteso_cmd[c.attesi++] = cmd;
		}
		size_t rumore = caso() % 4;
		for (size_t j = 0; j < rumore; j++) {
			uint8_t b = (uint8_t)caso();
			flusso[n++] = b == 0xfa ? 0x00 : b;
		}
	}

	gd32_deframer_t d;
	gd32_deframer_init(&d);
	for (size_t i = 0; i < n;) {
		size_t k = 1 + caso() % 300;
		if (k > n - i)
			k = n - i;
		gd32_deframer_scrivi(&d, flusso + i, k, conta, &c);
		i += k;
	}
	/*
	 * Un frame troncato può, raramente, "inghiottire" un frame buono e produrre per caso un checksum
	 * giusto: con un checksum a 16 bit succede circa una volta su 65536. Su ~600 troncati non deve succedere.
	 */
	CONTROLLA(c.n == c.attesi);
	CONTROLLA(c.ordine_ok);
}

static void nomi(void)
{
	CONTROLLA(strcmp(gd32_nome_cmd(GD32_HEARTBEAT), "heartbeat") == 0);
	CONTROLLA(strcmp(gd32_nome_cmd(GD32_MOTORE_LIDAR), "motore_lidar") == 0);
	CONTROLLA(strcmp(gd32_nome_cmd(GD32_POMPA_ACQUA), "pompa_acqua") == 0);
	CONTROLLA(strcmp(gd32_nome_cmd(GD32_ALIMENTAZIONE_SENSORI), "alimentazione_sensori") == 0);
	CONTROLLA(gd32_nome_cmd(0xee) == NULL);
}

int main(void)
{
	ESEGUI(checksum_frame_veri);
	ESEGUI(componi_heartbeat_e_velocita);
	ESEGUI(frame_intero);
	ESEGUI(sensori_decodifica);
	ESEGUI(frame_spezzati_in_ogni_modo);
	ESEGUI(frame_troncato_e_risincronizzazione);
	ESEGUI(casi_limite);
	ESEGUI(molti_errori_di_fila);
	ESEGUI(flusso_casuale);
	ESEGUI(nomi);
	return TEST_FINE();
}
