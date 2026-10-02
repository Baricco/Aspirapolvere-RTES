/* everest.c - Messaggi AuxCtrl <-> RobotApp: vedi everest.h. */
#include "everest.h"

#include <string.h>

void everest_deframer_init(everest_deframer_t *d)
{
	memset(d, 0, sizeof(*d));
}

static uint32_t le32(const uint8_t *p)
{
	return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24;
}

size_t everest_deframer_scrivi(everest_deframer_t *d, const uint8_t *b, size_t n, everest_msg_cb cb, void *ctx)
{
	size_t trovati = 0, i = 0;
	while (i < n) {
		if (d->n_testa < EVEREST_TESTA) {
			d->testa[d->n_testa++] = b[i++];
			/* risincronizzazione: i primi 4 byte devono essere la magia, altrimenti si scarta un byte */
			while (d->n_testa > 0 && d->n_testa <= 4 &&
			       memcmp(d->testa, "\xb0\xa9\x67\x58", d->n_testa) != 0) {
				memmove(d->testa, d->testa + 1, --d->n_testa);
				d->byte_scartati++;
			}
			if (d->n_testa < EVEREST_TESTA)
				continue;
			int32_t lung = (int32_t)le32(d->testa + 8);
			if (lung < 0 || lung > EVEREST_MAX_LUNG) {
				d->testate_errate++;
				memmove(d->testa, d->testa + 1, --d->n_testa);
				d->byte_scartati++;
				continue;
			}
			d->msg.comando = (int32_t)le32(d->testa + 4);
			d->msg.lunghezza = lung;
			d->msg.campo12 = (int32_t)le32(d->testa + 12);
			d->msg.tempo = (uint64_t)le32(d->testa + 16) | (uint64_t)le32(d->testa + 20) << 32;
			d->msg.n_dati = 0;
			d->ricevuti = 0;
		}
		size_t manca = (size_t)d->msg.lunghezza - d->ricevuti;
		size_t k = n - i < manca ? n - i : manca;
		for (size_t j = 0; j < k && d->msg.n_dati < EVEREST_MAX_DATI; j++)
			d->msg.dati[d->msg.n_dati++] = b[i + j];
		d->ricevuti += k;
		i += k;
		if (d->ricevuti == (size_t)d->msg.lunghezza) {
			d->messaggi_ok++;
			trovati++;
			if (cb)
				cb(&d->msg, ctx);
			d->n_testa = 0;
		}
	}
	return trovati;
}
