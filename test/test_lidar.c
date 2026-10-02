/* Test del protocollo del lidar (lidar/lidar.h), con i byte veri catturati all'accensione del lidar. */
#include <string.h>

#include "test.h"
#include "../lidar/lidar.h"

/*
 * I primi 140 byte ricevuti da AuxCtrl su /dev/ttyS1 (cattura-ricarica, 2026-10-02): 87 byte di testo di
 * avvio in GBK ("深圳市杉川机器人科技有限公司 / 软件作者：JLWan / 固件版本:LDS3_NOR...") e poi il pacchetto
 * di versione AA 00 33 01 61 AC 00 2B ... 0B B0 (indirizzo 1: checksum a somma).
 */
static const uint8_t AVVIO_VERO[] = {
	0x0c, 0x0e, 0x00, 0x0d, 0x0a, 0xc9, 0xee, 0xdb, 0xda, 0xca, 0xd0, 0xc9, 0xbc, 0xb4, 0xa8, 0xbb,
	0xfa, 0xc6, 0xf7, 0xc8, 0xcb, 0xbf, 0xc6, 0xbc, 0xbc, 0xd3, 0xd0, 0xcf, 0xde, 0xb9, 0xab, 0xcb,
	0xbe, 0x0d, 0x0a, 0xc8, 0xed, 0xbc, 0xfe, 0xd7, 0xf7, 0xd5, 0xdf, 0xa3, 0xba, 0x4a, 0x4c, 0x57,
	0x61, 0x6e, 0x0d, 0x0a, 0xb9, 0xcc, 0xbc, 0xfe, 0xb0, 0xe6, 0xb1, 0xbe, 0x3a, 0x4c, 0x44, 0x53,
	0x33, 0x5f, 0x4e, 0x4f, 0x52, 0x4d, 0x61, 0x79, 0x34, 0x32, 0x30, 0x31, 0x38, 0x31, 0x35, 0x3a,
	0x33, 0x36, 0x3a, 0x33, 0x34, 0x0d, 0x0a, 0xaa, 0x00, 0x33, 0x01, 0x61, 0xac, 0x00, 0x2b, 0x32,
	0x30, 0x33, 0x30, 0x33, 0x31, 0x33, 0x39, 0x35, 0x38, 0x34, 0x32, 0x35, 0x37, 0x31, 0x33, 0x30,
	0x30, 0x32, 0x46, 0x30, 0x30, 0x32, 0x34, 0x4c, 0x44, 0x53, 0x33, 0x5f, 0x4e, 0x4f, 0x52, 0x5f,
	0x31, 0x38, 0x30, 0x35, 0x30, 0x34, 0x31, 0x35, 0x33, 0x36, 0x0b, 0xb0,
};

typedef struct {
	lidar_pacchetto_t p[16];
	size_t n;
} raccolta_t;

static void raccogli(const lidar_pacchetto_t *p, void *ctx)
{
	raccolta_t *r = ctx;
	if (r->n < 16)
		r->p[r->n++] = *p;
}

static void crc_vettori_noti(void)
{
	/* valore di controllo standard di CRC-16/MODBUS */
	CONTROLLA(lidar_crc16((const uint8_t *)"123456789", 9) == 0x4B37);
	CONTROLLA(lidar_crc16(NULL, 0) == 0xFFFF);
	uint8_t b[] = { 0x01, 0x02, 0xff };
	CONTROLLA(lidar_somma(b, 3) == 0x102);
}

static void avvio_vero(void)
{
	lidar_deframer_t d;
	lidar_deframer_init(&d);
	raccolta_t r = { .n = 0 };
	lidar_deframer_scrivi(&d, AVVIO_VERO, sizeof AVVIO_VERO, raccogli, &r);
	CONTROLLA(r.n == 1);
	CONTROLLA(r.p[0].cmd == LIDAR_VERSIONE && r.p[0].indirizzo == 1 && r.p[0].tipo == 0x61);
	CONTROLLA(r.p[0].n_param == 43 && r.p[0].n_dichiarato == 43);
	CONTROLLA(memcmp(r.p[0].param + 24, "LDS3_NOR_1805041536", 19) == 0);
	CONTROLLA(d.byte_scartati == 87);     /* il testo di avvio */
	CONTROLLA(d.checksum_errati == 0);
}

static void componi_e_rileggi(void)
{
	uint8_t out[LIDAR_MAX_PACCHETTO], param[LIDAR_MAX_PARAMETRI];
	for (size_t i = 0; i < sizeof param; i++)
		param[i] = (uint8_t)(i * 31);
	/* il pacchetto vero ricomposto deve essere identico byte per byte */
	size_t k = lidar_componi(1, 0x61, LIDAR_VERSIONE, AVVIO_VERO + 87 + 8, 43, out, sizeof out);
	CONTROLLA(k == 53 && memcmp(out, AVVIO_VERO + 87, 53) == 0);

	/* indirizzo diverso da 1: CRC; tutte le lunghezze possibili, spedite un byte alla volta */
	int ok = 1;
	for (size_t n = 0; n <= LIDAR_MAX_PARAMETRI; n++) {
		k = lidar_componi(2, 0x61, LIDAR_SCANSIONE_NUOVA, param, n, out, sizeof out);
		lidar_deframer_t d;
		lidar_deframer_init(&d);
		raccolta_t r = { .n = 0 };
		for (size_t i = 0; i < k; i++)
			lidar_deframer_scrivi(&d, out + i, 1, raccogli, &r);
		if (k != n + 10 || r.n != 1 || r.p[0].n_param != n || memcmp(r.p[0].param, param, n) != 0)
			ok = 0;
	}
	CONTROLLA(ok);
	CONTROLLA(lidar_componi(1, 0x61, 0xAD, param, LIDAR_MAX_PARAMETRI + 1, out, sizeof out) == 0);
}

/* Pacchetto troncato seguito da uno buono; checksum rovinato; len fuori dai limiti. */
static void errori_e_risincronizzazione(void)
{
	uint8_t flusso[512], fr[LIDAR_MAX_PACCHETTO];
	size_t n = 0, k;
	memcpy(flusso, AVVIO_VERO + 87, 30);           /* versione troncata */
	n = 30;
	flusso[n++] = 0xaa; flusso[n++] = 0x00; flusso[n++] = 0x65;   /* len 101: troppo lungo */
	k = lidar_componi(1, 0x61, LIDAR_VELOCITA, "\x01\x02", 2, fr, sizeof fr);
	memcpy(flusso + n, fr, k);
	fr[k - 1] ^= 0xff;                               /* stesso pacchetto con checksum sbagliato */
	memcpy(flusso + n + k, fr, k);
	n += 2 * k;
	memcpy(flusso + n, AVVIO_VERO + 87, 53);
	n += 53;

	lidar_deframer_t d;
	lidar_deframer_init(&d);
	raccolta_t r = { .n = 0 };
	lidar_deframer_scrivi(&d, flusso, n, raccogli, &r);
	CONTROLLA(r.n == 2);
	CONTROLLA(r.n == 2 && r.p[0].cmd == LIDAR_VELOCITA && r.p[1].cmd == LIDAR_VERSIONE);
	CONTROLLA(d.len_errati >= 1);
	CONTROLLA(d.checksum_errati >= 2);
}

static void nomi(void)
{
	CONTROLLA(strcmp(lidar_nome_cmd(LIDAR_VERSIONE), "versione") == 0);
	CONTROLLA(lidar_nome_cmd(0x00) == NULL);
}

int main(void)
{
	ESEGUI(crc_vettori_noti);
	ESEGUI(avvio_vero);
	ESEGUI(componi_e_rileggi);
	ESEGUI(errori_e_risincronizzazione);
	ESEGUI(nomi);
	return TEST_FINE();
}
