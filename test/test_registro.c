/* Test del formato del registro della spia (spia/registro.h): scrittura e rilettura. */
#include <string.h>
#include <unistd.h>

#include "test.h"
#include "../spia/registro.h"

static void andata_e_ritorno(void)
{
	FILE *f = tmpfile();
	reg_intestazione_t h = { .n_canali = 2 };
	h.canali[0] = (reg_canale_t){ .id = 0, .protocollo = REG_PROTO_GD32, .nome = "gd32", .dispositivo = "/dev/ttyS3" };
	h.canali[1] = (reg_canale_t){ .id = 1, .protocollo = REG_PROTO_LIDAR, .nome = "lidar", .dispositivo = "/dev/ttyS1" };
	CONTROLLA(reg_scrivi_intestazione(f, &h) == 0);
	static uint8_t grande[REG_MAX_BLOCCO];
	for (size_t i = 0; i < sizeof grande; i++)
		grande[i] = (uint8_t)i;
	CONTROLLA(reg_scrivi_record(f, 123456789012345LL, 0, REG_VERSO_SOC, "\xfa\xfb\x03\x06\x00\x06", 6) == 0);
	CONTROLLA(reg_scrivi_record(f, -5, 1, REG_DAL_SOC, grande, REG_MAX_BLOCCO) == 0);
	CONTROLLA(reg_scrivi_record(f, 7, 0, REG_DAL_SOC, NULL, 0) == 0);
	rewind(f);

	reg_intestazione_t l;
	static reg_record_t r;
	CONTROLLA(reg_leggi_intestazione(f, &l) == 1);
	CONTROLLA(l.n_canali == 2 && l.canali[1].protocollo == REG_PROTO_LIDAR);
	CONTROLLA(!strcmp(l.canali[0].nome, "gd32") && !strcmp(l.canali[1].dispositivo, "/dev/ttyS1"));
	CONTROLLA(reg_leggi_record(f, &r) == 1);
	CONTROLLA(r.tempo_ns == 123456789012345LL && r.canale == 0 && r.direzione == REG_VERSO_SOC && r.n == 6);
	CONTROLLA(memcmp(r.dati, "\xfa\xfb\x03\x06\x00\x06", 6) == 0);
	CONTROLLA(reg_leggi_record(f, &r) == 1);
	CONTROLLA(r.tempo_ns == -5 && r.n == REG_MAX_BLOCCO && memcmp(r.dati, grande, REG_MAX_BLOCCO) == 0);
	CONTROLLA(reg_leggi_record(f, &r) == 1);
	CONTROLLA(r.n == 0 && r.direzione == REG_DAL_SOC);
	CONTROLLA(reg_leggi_record(f, &r) == 0);
	fclose(f);
}

static void file_troncato_e_sbagliato(void)
{
	FILE *f = tmpfile();
	reg_intestazione_t h = { .n_canali = 0 };
	reg_scrivi_intestazione(f, &h);
	reg_scrivi_record(f, 1, 0, 0, "abcdef", 6);
	fflush(f);
	long lung = ftell(f);
	CONTROLLA(ftruncate(fileno(f), lung - 2) == 0);   /* il record finisce a metà */
	rewind(f);
	static reg_record_t r;
	reg_intestazione_t l;
	CONTROLLA(reg_leggi_intestazione(f, &l) == 1);
	CONTROLLA(reg_leggi_record(f, &r) == -1);
	fclose(f);

	f = tmpfile();
	fputs("NON SONO UN REGISTRO", f);
	rewind(f);
	CONTROLLA(reg_leggi_intestazione(f, &l) == -1);
	fclose(f);
}

static void protocolli(void)
{
	CONTROLLA(reg_protocollo_da_nome("gd32") == REG_PROTO_GD32);
	CONTROLLA(reg_protocollo_da_nome("lidar") == REG_PROTO_LIDAR);
	CONTROLLA(reg_protocollo_da_nome("boh") == -1);
	CONTROLLA(!strcmp(reg_nome_protocollo(REG_PROTO_GREZZO), "grezzo"));
}

int main(void)
{
	ESEGUI(andata_e_ritorno);
	ESEGUI(file_troncato_e_sbagliato);
	ESEGUI(protocolli);
	return TEST_FINE();
}
