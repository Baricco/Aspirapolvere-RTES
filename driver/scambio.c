/* scambio.c - Doppio buffer con numero di versione: vedi scambio.h. */
#include "scambio.h"

#include <string.h>

void scambio_init(scambio_t *s, void *copia0, void *copia1, size_t dim)
{
	atomic_init(&s->versione, 0);
	atomic_init(&s->seq[0], 0);
	atomic_init(&s->seq[1], 0);
	atomic_init(&s->riprove, 0);
	s->ver_copia[0] = s->ver_copia[1] = 0;
	s->dim = dim;
	s->copia[0] = copia0;
	s->copia[1] = copia1;
}

void scambio_scrivi(scambio_t *s, const void *dati)
{
	uint32_t v = atomic_load_explicit(&s->versione, memory_order_relaxed) + 1;
	unsigned c = v & 1;
	uint32_t q = atomic_load_explicit(&s->seq[c], memory_order_relaxed);

	atomic_store_explicit(&s->seq[c], q + 1, memory_order_relaxed);     /* dispari: "sto scrivendo" */
	atomic_thread_fence(memory_order_release);                         /* ...prima di toccare i dati */
	memcpy(s->copia[c], dati, s->dim);
	s->ver_copia[c] = v;
	atomic_store_explicit(&s->seq[c], q + 2, memory_order_release);     /* pari: copia completa */
	atomic_store_explicit(&s->versione, v, memory_order_release);       /* annuncia la nuova versione */
}

uint32_t scambio_leggi(scambio_t *s, void *out)
{
	for (;;) {
		uint32_t v = atomic_load_explicit(&s->versione, memory_order_acquire);
		if (v == 0)
			return 0;
		unsigned c = v & 1;
		uint32_t q1 = atomic_load_explicit(&s->seq[c], memory_order_acquire);
		if (!(q1 & 1)) {
			memcpy(out, s->copia[c], s->dim);
			uint32_t vc = s->ver_copia[c];
			atomic_thread_fence(memory_order_acquire);                 /* i dati letti prima di ricontrollare */
			if (atomic_load_explicit(&s->seq[c], memory_order_relaxed) == q1)
				return vc;
		}
		atomic_fetch_add_explicit(&s->riprove, 1, memory_order_relaxed);
	}
}
