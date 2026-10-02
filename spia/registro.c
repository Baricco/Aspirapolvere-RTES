/* registro.c - File di registro della spia: vedi registro.h. */
#include "registro.h"

#include <string.h>

/* Il SoC (ARM) e il PC (x86) sono little-endian: si scrivono i numeri così come sono in memoria. */
_Static_assert(__BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__, "il formato del registro presuppone little-endian");

static int scrivi(FILE *f, const void *p, size_t n)
{
	return fwrite(p, 1, n, f) == n ? 0 : -1;
}

static int scrivi_stringa(FILE *f, const char *s)
{
	size_t n = strlen(s);
	uint8_t l = (uint8_t)(n > 255 ? 255 : n);
	return scrivi(f, &l, 1) || scrivi(f, s, l);
}

int reg_scrivi_intestazione(FILE *f, const reg_intestazione_t *h)
{
	if (scrivi(f, REG_MAGIA, strlen(REG_MAGIA)) || scrivi(f, &h->n_canali, 1))
		return -1;
	for (int i = 0; i < h->n_canali; i++) {
		const reg_canale_t *c = &h->canali[i];
		if (scrivi(f, &c->id, 1) || scrivi(f, &c->protocollo, 1) ||
		    scrivi_stringa(f, c->nome) || scrivi_stringa(f, c->dispositivo))
			return -1;
	}
	return 0;
}

int reg_scrivi_record(FILE *f, int64_t tempo_ns, uint8_t canale, uint8_t direzione, const void *dati, uint16_t n)
{
	uint8_t testa[12];
	memcpy(testa, &tempo_ns, 8);
	testa[8] = canale;
	testa[9] = direzione;
	memcpy(testa + 10, &n, 2);
	return scrivi(f, testa, sizeof testa) || scrivi(f, dati, n);
}

static int leggi(FILE *f, void *p, size_t n)
{
	return fread(p, 1, n, f) == n ? 0 : -1;
}

static int leggi_stringa(FILE *f, char *s, size_t dim)
{
	uint8_t l;
	char tmp[256];
	if (leggi(f, &l, 1) || leggi(f, tmp, l))
		return -1;
	size_t k = l < dim - 1 ? l : dim - 1;
	memcpy(s, tmp, k);
	s[k] = 0;
	return 0;
}

int reg_leggi_intestazione(FILE *f, reg_intestazione_t *h)
{
	char magia[sizeof REG_MAGIA - 1];
	memset(h, 0, sizeof(*h));
	if (leggi(f, magia, sizeof magia) || memcmp(magia, REG_MAGIA, sizeof magia) != 0)
		return -1;
	if (leggi(f, &h->n_canali, 1) || h->n_canali > REG_MAX_CANALI)
		return -1;
	for (int i = 0; i < h->n_canali; i++) {
		reg_canale_t *c = &h->canali[i];
		if (leggi(f, &c->id, 1) || leggi(f, &c->protocollo, 1) ||
		    leggi_stringa(f, c->nome, sizeof c->nome) ||
		    leggi_stringa(f, c->dispositivo, sizeof c->dispositivo))
			return -1;
	}
	return 1;
}

int reg_leggi_record(FILE *f, reg_record_t *r)
{
	uint8_t testa[12];
	size_t k = fread(testa, 1, sizeof testa, f);
	if (k == 0)
		return 0;
	if (k != sizeof testa)
		return -1;
	memcpy(&r->tempo_ns, testa, 8);
	r->canale = testa[8];
	r->direzione = testa[9];
	memcpy(&r->n, testa + 10, 2);
	return leggi(f, r->dati, r->n) ? -1 : 1;
}

const char *reg_nome_protocollo(uint8_t p)
{
	switch (p) {
	case REG_PROTO_GD32:  return "gd32";
	case REG_PROTO_LIDAR: return "lidar";
	case REG_PROTO_EVEREST: return "everest";
	default:              return "grezzo";
	}
}

int reg_protocollo_da_nome(const char *s)
{
	if (!strcmp(s, "gd32"))   return REG_PROTO_GD32;
	if (!strcmp(s, "lidar"))  return REG_PROTO_LIDAR;
	if (!strcmp(s, "everest")) return REG_PROTO_EVEREST;
	if (!strcmp(s, "grezzo")) return REG_PROTO_GREZZO;
	return -1;
}
