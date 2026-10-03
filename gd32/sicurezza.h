/*
 * sicurezza.h - Blocco di sicurezza (interblocco) dei motori del robot.
 *
 * Decide, comando per comando, se un frame verso il GD32 può partire, e quando bisogna fermare tutti i motori.
 * È logica pura: non legge né scrive la seriale, non usa Linux e non ha stato globale. Il tempo lo passa
 * chi chiama (nanosecondi monotoni, come rt_adesso()). Lo usa "spia guida" (banco di prova della fase 2) e lo
 * userà il driver della fase 3.
 *
 * Regole (docs/fase2-piano.md, "Regole di sicurezza"):
 *   - comandi ai MOTORI (ruote, spazzole, aspirazione) diversi da zero solo se:
 *       1. chi usa il programma ha dichiarato che il robot è sollevato (--sollevato);
 *       2. l'ultimo frame dei sensori dice che il robot è sollevato: tutti e 4 gli anti-caduta vedono "vuoto"
 *          (byte 3 = 0x0f), i sensori sono alimentati (byte 64 bit 5) e il robot NON è sulla base (byte 7 = 0).
 *          Il solo 0x0f non basta: sulla base gli anti-caduta leggono 0x0f anche appoggiati, e con i sensori
 *          spenti (0x9d 00) i loro valori non significano niente (fase 2, prova c5c);
 *       3. quell'ultimo frame dei sensori è recente (max_eta_ns, di solito 100 ms: 5 frame persi);
 *   - un comando ai motori "a zero" è sempre permesso: fermare è sicuro;
 *   - dopo aver visto almeno una volta "sollevato", se una delle condizioni viene meno, oppure se i sensori
 *     smettono di arrivare, il blocco SCATTA: bisogna mandare subito i frame di arresto, e da quel momento
 *     ogni comando ai motori diverso da zero è rifiutato fino alla fine del programma (anche se torna a posto);
 *   - alcuni comandi sono sempre vietati: pompa dell'acqua (non c'è il serbatoio), alimentazione e riavvio
 *     del SoC.
 * Il motore del lidar (0x71, 0x97) non è considerato un motore "pericoloso": gira dentro la torretta.
 */
#ifndef GD32_SICUREZZA_H
#define GD32_SICUREZZA_H

#include <stddef.h>
#include <stdint.h>

#include "gd32.h"

typedef enum {
	SIC_PERMESSO = 0,    /* il frame può partire */
	SIC_RIFIUTATO,       /* motore diverso da zero ma le condizioni non ci sono (o il blocco è scattato) */
	SIC_VIETATO,         /* comando vietato sempre */
} sic_esito_t;

typedef struct {
	int sollevato;             /* dichiarato da chi usa il programma */
	int64_t max_eta_ns;        /* età massima dell'ultimo frame dei sensori */

	int ha_sensori;            /* è arrivato almeno un frame dei sensori */
	int visto_vuoto;           /* almeno una volta "sollevato" */
	int sollevato_ora;         /* l'ultimo frame dei sensori dice "sollevato" */
	uint8_t cliff;             /* byte 3 dell'ultimo frame dei sensori */
	int64_t ultimo_sensori;    /* quando è arrivato */

	int scattato;              /* il blocco è scattato: motori fermi fino alla fine */
	const char *motivo;        /* perché è scattato (testo fisso) */
	int64_t quando;            /* quando è scattato */

	uint64_t rifiutati, vietati;
} sicurezza_t;

void sic_init(sicurezza_t *s, int sollevato, int64_t max_eta_ns);

/* 1 se cmd comanda un motore pericoloso (ruote, spazzole, aspirazione), 0 altrimenti. */
int sic_e_motore(uint8_t cmd);

/* 1 se cmd è sempre vietato. */
int sic_e_vietato(uint8_t cmd);

/*
 * Da chiamare per ogni frame ricevuto dal GD32 (gli altri frame sono ignorati).
 * Ritorna 1 se il blocco è scattato ADESSO: chi chiama deve mandare subito i frame di arresto.
 */
int sic_ricevuto(sicurezza_t *s, const gd32_frame_t *f, int64_t adesso);

/*
 * Da chiamare spesso (ogni giro del ciclo): controlla che i sensori arrivino ancora.
 * Ritorna 1 se il blocco è scattato ADESSO.
 */
int sic_controlla(sicurezza_t *s, int64_t adesso);

/* Decide se il frame (cmd, dati) può partire. Aggiorna i contatori dei rifiutati e dei vietati. */
sic_esito_t sic_permesso(sicurezza_t *s, uint8_t cmd, const uint8_t *dati, size_t n, int64_t adesso);

/*
 * Scrive in out, uno dopo l'altro, i frame completi che fermano tutti i motori: velocità e velocità delle ruote
 * a zero, spazzole a zero, aspirazione a zero (anche in PWM). Ritorna i byte scritti (0 se dim non basta).
 */
size_t sic_frame_arresto(uint8_t *out, size_t dim);

#endif /* GD32_SICUREZZA_H */
