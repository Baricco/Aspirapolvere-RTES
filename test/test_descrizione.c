/* Test della descrizione dei messaggi (gd32/descrizione.h): i campi devono coincidere con il decodificatore
 * gd32_leggi_sensori, non sovrapporsi e stare dentro i 96 byte. */
#include <string.h>

#include "test.h"
#include "../gd32/descrizione.h"
#include "../gd32/gd32.h"

static const campo_t *campo(const char *nome)
{
	for (int i = 0; i < N_CAMPI_SENSORI; i++)
		if (!strcmp(CAMPI_SENSORI[i].nome, nome))
			return &CAMPI_SENSORI[i];
	return NULL;
}

static void dentro_e_senza_sovrapposizioni(void)
{
	uint8_t usato[96][8];
	memset(usato, 0, sizeof usato);
	int ok = 1;
	for (int i = 0; i < N_CAMPI_SENSORI; i++) {
		const campo_t *c = &CAMPI_SENSORI[i];
		CONTROLLA(c->byte + c->n_byte <= 96);
		CONTROLLA(c->affidabilita == 'V' || c->affidabilita == 'C' || c->affidabilita == '?');
		for (int b = c->byte; b < c->byte + c->n_byte && b < 96; b++)
			for (int k = 0; k < 8; k++) {
				int mio = c->bit < 0 || (b == c->byte && k >= c->bit && k < c->bit + c->n_bit);
				if (!mio)
					continue;
				if (usato[b][k]) {
					printf("  sovrapposti al byte %d bit %d: %s\n", b, k, c->nome);
					ok = 0;
				}
				usato[b][k] = 1;
			}
	}
	CONTROLLA(ok);
}

static void uguale_al_decodificatore(void)
{
	gd32_frame_t f = { .cmd = GD32_SENSORI, .n_dati = 96 };
	for (int i = 0; i < 96; i++)
		f.dati[i] = (uint8_t)(i * 37 + 11);
	gd32_sensori_t s;
	CONTROLLA(gd32_leggi_sensori(&f, &s) == 0);
	const uint8_t *d = f.dati;
	CONTROLLA(descrizione_grezzo(campo("encoder_sx"), d) == s.encoder_sx);
	CONTROLLA(descrizione_grezzo(campo("encoder_dx"), d) == s.encoder_dx);
	CONTROLLA(descrizione_grezzo(campo("v_misurata"), d) == s.vel_sx);
	CONTROLLA(descrizione_grezzo(campo("w_misurata"), d) == s.vel_dx);
	CONTROLLA(descrizione_grezzo(campo("tensione"), d) == s.tensione_dv);
	CONTROLLA(descrizione_grezzo(campo("temperatura"), d) == s.temperatura_c);
	CONTROLLA(descrizione_grezzo(campo("vel_imbardata"), d) == s.vel_angolare[0]);
	CONTROLLA(descrizione_grezzo(campo("angolo_imbardata"), d) == s.angolo[0]);
	CONTROLLA(descrizione_grezzo(campo("angolo_rollio"), d) == s.angolo[2]);
	CONTROLLA(descrizione_grezzo(campo("gravita_z"), d) == s.gravita[2]);
	CONTROLLA(descrizione_grezzo(campo("tasto_casa"), d) * 10 == s.tasto_casa_ms);
	CONTROLLA(descrizione_grezzo(campo("led"), d) == s.led);
	CONTROLLA(descrizione_grezzo(campo("corrente_ruota_dx"), d) == s.correnti[1]);
	CONTROLLA(descrizione_grezzo(campo("corrente_aspirazione"), d) == s.correnti[4]);
	CONTROLLA(descrizione_grezzo(campo("eco_laterale"), d) == s.byte67);
	CONTROLLA(descrizione_grezzo(campo("ventola"), d) * 100 == s.byte69_x100);
	CONTROLLA(descrizione_grezzo(campo("copia_vel_beccheggio"), d) == s.magnetico[1]);
	/* campi a bit */
	CONTROLLA(descrizione_grezzo(campo("cassetto"), d) == s.cassetto);
	CONTROLLA(descrizione_grezzo(campo("in_carica"), d) == s.in_carica);
	CONTROLLA(descrizione_grezzo(campo("ir_base_a"), d) == s.ir_base_a);
	CONTROLLA(descrizione_grezzo(campo("ir_base_b"), d) == s.ir_base_b);
	CONTROLLA((descrizione_grezzo(campo("bumper_sx"), d) != 0) == s.bumper_sx);
	CONTROLLA((descrizione_grezzo(campo("bumper_dx"), d) != 0) == s.bumper_dx);
	CONTROLLA(descrizione_grezzo(campo("cliff_sx_laterale"), d) == s.cliff[0]);
	CONTROLLA(descrizione_grezzo(campo("cliff_dx_laterale"), d) == s.cliff[3]);
	CONTROLLA(descrizione_grezzo(campo("sensori_accesi"), d) == ((s.controllo >> 5) & 1));
}

static void messaggi(void)
{
	CONTROLLA(descrizione_messaggio(0x15, 0) && descrizione_messaggio(0x15, 0)->n_campi == N_CAMPI_SENSORI);
	const messaggio_t *v = descrizione_messaggio(0x66, 1);
	CONTROLLA(v && v->n_campi == 2);
	uint8_t d[8] = { 0x78, 0x06, 0, 0, 0x90, 0x01, 0, 0 };     /* 1656, 400 */
	CONTROLLA(descrizione_grezzo(&v->campi[0], d) == 1656 && descrizione_grezzo(&v->campi[1], d) == 400);
	CONTROLLA(descrizione_messaggio(0x66, 0) == NULL);
	for (int i = 0; i < N_MESSAGGI; i++)
		CONTROLLA(MESSAGGI[i].nome && MESSAGGI[i].spiegazione);
}

int main(void)
{
	ESEGUI(dentro_e_senza_sovrapposizioni);
	ESEGUI(uguale_al_decodificatore);
	ESEGUI(messaggi);
	return TEST_FINE();
}
