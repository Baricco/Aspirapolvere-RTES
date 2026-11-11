/* exchange.c - Doppio buffer con numero di versione: vedi exchange.h. */
#include "exchange.h"

#include <string.h>

void exchange_init(exchange_t *s, void *copy0, void *copy1, size_t size)
{
	atomic_init(&s->version, 0);
	atomic_init(&s->seq[0], 0);
	atomic_init(&s->seq[1], 0);
	atomic_init(&s->retries, 0);
	s->copy_version[0] = s->copy_version[1] = 0;
	s->size = size;
	s->copy[0] = copy0;
	s->copy[1] = copy1;
}

void exchange_write(exchange_t *s, const void *data)
{
	uint32_t v = atomic_load_explicit(&s->version, memory_order_relaxed) + 1;
	unsigned c = v & 1;
	uint32_t q = atomic_load_explicit(&s->seq[c], memory_order_relaxed);

	atomic_store_explicit(&s->seq[c], q + 1, memory_order_relaxed);     /* dispari: "sto scrivendo" */
	atomic_thread_fence(memory_order_release);                         /* ...prima di toccare i dati */
	memcpy(s->copy[c], data, s->size);
	s->copy_version[c] = v;
	atomic_store_explicit(&s->seq[c], q + 2, memory_order_release);     /* pari: copia completa */
	atomic_store_explicit(&s->version, v, memory_order_release);       /* annuncia la nuova versione */
}

uint32_t exchange_read(exchange_t *s, void *out)
{
	for (;;) {
		uint32_t v = atomic_load_explicit(&s->version, memory_order_acquire);
		if (v == 0)
			return 0;
		unsigned c = v & 1;
		uint32_t q1 = atomic_load_explicit(&s->seq[c], memory_order_acquire);
		if (!(q1 & 1)) {
			memcpy(out, s->copy[c], s->size);
			uint32_t vc = s->copy_version[c];
			atomic_thread_fence(memory_order_acquire);                 /* i dati letti prima di ricontrollare */
			if (atomic_load_explicit(&s->seq[c], memory_order_relaxed) == q1)
				return vc;
		}
		atomic_fetch_add_explicit(&s->retries, 1, memory_order_relaxed);
	}
}
