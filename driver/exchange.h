/*
 * exchange.h - Scambio di dati SENZA MUTEX fra il driver (un solo scrittore) e i task (tanti lettori).
 *
 * Problema: il driver pubblica ogni 20 ms i sensori, e i task li leggono quando vogliono. Con un mutex il driver,
 * che ha la priorità più alta di tutti, potrebbe dover aspettare un task che tiene il mutex (inversione di priorità).
 * Qui invece lo scrittore NON aspetta MAI: semmai è il lettore che ripete la lettura.
 *
 * Come funziona (doppio buffer con numero di versione, una variante del "seqlock" del kernel Linux):
 *   - ci sono due copie dei dati; lo scrittore scrive sempre in quella che NON contiene l'ultima versione pubblicata,
 *     poi annuncia la nuova versione. Mentre scrive, il contatore di quella copia è dispari;
 *   - il lettore prende la copia dell'ultima versione, legge il contatore, copia i dati e rilegge il contatore: se è
 *     cambiato (o era dispari) lo scrittore ci è passato sopra nel frattempo, e il lettore riprova.
 * Con due copie lo scrittore tocca la copia che si sta leggendo solo se scrive DUE volte durante una lettura (40 ms
 * per i sensori, contro pochi microsecondi per copiarli): in pratica il lettore non riprova quasi mai.
 *
 * Il lettore ottiene sempre una copia COERENTE (mai metà di un frame e metà del successivo) e il suo numero di
 * versione: chi legge due volte di fila la stessa versione sa che non è arrivato niente di nuovo.
 */
#ifndef EXCHANGE_H
#define EXCHANGE_H

#include <stdatomic.h>
#include <stddef.h>
#include <stdint.h>

typedef struct {
	_Atomic uint32_t version;       /* scritture completate (0 = mai scritto); l'ultima è nella copia versione & 1 */
	_Atomic uint32_t seq[2];         /* contatore di ogni copia: dispari mentre lo scrittore la sta scrivendo */
	uint32_t copy_version[2];           /* la versione contenuta in ogni copia */
	size_t size;
	void *copy[2];
	_Atomic uint64_t retries;        /* letture ripetute (statistica) */
} exchange_t;

/* Le due copie (di dim byte ciascuna) le fornisce chi chiama: niente allocazioni. */
void exchange_init(exchange_t *s, void *copy0, void *copy1, size_t size);

/* Solo lo scrittore (un solo thread). Non si blocca mai. */
void exchange_write(exchange_t *s, const void *data);

/* Qualsiasi thread. Copia in out l'ultima versione e ne ritorna il numero (0 = non è mai stato scritto niente). */
uint32_t exchange_read(exchange_t *s, void *out);

/* Numero dell'ultima versione pubblicata, senza copiare i dati. */
static inline uint32_t exchange_version(exchange_t *s)
{
	return atomic_load_explicit(&s->version, memory_order_acquire);
}

#endif /* EXCHANGE_H */
