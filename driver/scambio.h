/*
 * scambio.h - Scambio di dati SENZA MUTEX fra il driver (un solo scrittore) e i task (tanti lettori).
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
#ifndef SCAMBIO_H
#define SCAMBIO_H

#include <stdatomic.h>
#include <stddef.h>
#include <stdint.h>

typedef struct {
	_Atomic uint32_t versione;       /* scritture completate (0 = mai scritto); l'ultima è nella copia versione & 1 */
	_Atomic uint32_t seq[2];         /* contatore di ogni copia: dispari mentre lo scrittore la sta scrivendo */
	uint32_t ver_copia[2];           /* la versione contenuta in ogni copia */
	size_t dim;
	void *copia[2];
	_Atomic uint64_t riprove;        /* letture ripetute (statistica) */
} scambio_t;

/* Le due copie (di dim byte ciascuna) le fornisce chi chiama: niente allocazioni. */
void scambio_init(scambio_t *s, void *copia0, void *copia1, size_t dim);

/* Solo lo scrittore (un solo thread). Non si blocca mai. */
void scambio_scrivi(scambio_t *s, const void *dati);

/* Qualsiasi thread. Copia in out l'ultima versione e ne ritorna il numero (0 = non è mai stato scritto niente). */
uint32_t scambio_leggi(scambio_t *s, void *out);

/* Numero dell'ultima versione pubblicata, senza copiare i dati. */
static inline uint32_t scambio_versione(scambio_t *s)
{
	return atomic_load_explicit(&s->versione, memory_order_acquire);
}

#endif /* SCAMBIO_H */
