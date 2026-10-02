/*
 * test.h - Mini-framework per gli unit test.
 *
 * Ogni file test_*.c è un programma: le funzioni di test usano CONTROLLA()
 * e il main le chiama con ESEGUI(). Alla fine TEST_FINE() stampa il riepilogo
 * e restituisce un codice d'uscita diverso da 0 se qualche controllo è
 * fallito, così ./rtes.sh test sa che qualcosa non va.
 */
#ifndef TEST_H
#define TEST_H

#include <stdio.h>

static int test_controlli, test_falliti;

#define CONTROLLA(cond) do { \
	test_controlli++; \
	if (!(cond)) { \
		test_falliti++; \
		fprintf(stderr, "  FALLITO %s:%d: %s\n", __FILE__, __LINE__, #cond); \
	} \
} while (0)

#define ESEGUI(f) do { printf("- %s\n", #f); f(); } while (0)

#define TEST_FINE() ( \
	printf("%s: %d controlli, %d falliti\n", __FILE__, test_controlli, test_falliti), \
	test_falliti ? 1 : 0)

#endif /* TEST_H */
