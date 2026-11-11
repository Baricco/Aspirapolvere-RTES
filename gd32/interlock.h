/*
 * interlock.h - Blocco di sicurezza (interblocco) dei motori del robot.
 *
 * Decide, comando per comando, se un frame verso il GD32 può partire, e quando bisogna fermare tutti i motori.
 * È logica pura: non legge né scrive la seriale, non usa Linux e non ha stato globale. Il tempo lo passa
 * chi chiama (nanosecondi monotoni, come rt_now()). Lo usa "spia guida" (banco di prova della fase 2) e lo
 * userà il driver della fase 3.
 *
 * Regole (docs/fase2-piano.md, "Regole di sicurezza"):
 *   - comandi ai MOTORI (ruote, spazzole, aspirazione) diversi da zero solo se:
 *       1. chi usa il programma ha dichiarato che il robot è sollevato (--sollevato);
 *       2. l'ultimo frame dei sensori dice che il robot è sollevato: tutti e 4 gli anti-caduta vedono "vuoto"
 *          (byte 3 = 0x0f), i sensori sono alimentati (byte 64 bit 5) e il robot NON è sulla base (byte 7 = 0).
 *          Il solo 0x0f non basta: sulla base gli anti-caduta leggono 0x0f anche appoggiati, e con i sensori
 *          spenti (0x9d 00) i loro valori non significano niente (fase 2, prova c5c);
 *       3. quell'ultimo frame dei sensori è recente (max_age_ns, di solito 100 ms: 5 frame persi);
 *   - un comando ai motori "a zero" è sempre permesso: fermare è sicuro;
 *   - dopo aver visto almeno una volta "sollevato", se una delle condizioni viene meno, oppure se i sensori
 *     smettono di arrivare, il blocco SCATTA: bisogna mandare subito i frame di arresto, e da quel momento
 *     ogni comando ai motori diverso da zero è rifiutato fino alla fine del programma (anche se torna a posto);
 *   - alcuni comandi sono sempre vietati: pompa dell'acqua (non c'è il serbatoio), alimentazione e riavvio
 *     del SoC.
 * Il motore del lidar (0x71, 0x97) non è considerato un motore "pericoloso": gira dentro la torretta.
 */
#ifndef GD32_INTERLOCK_H
#define GD32_INTERLOCK_H

#include <stddef.h>
#include <stdint.h>

#include "gd32.h"

typedef enum {
	INTERLOCK_ALLOWED = 0,    /* il frame può partire */
	INTERLOCK_REFUSED,       /* motore diverso da zero ma le condizioni non ci sono (o il blocco è scattato) */
	INTERLOCK_FORBIDDEN,         /* comando vietato sempre */
} interlock_result_t;

typedef struct {
	int lifted;             /* dichiarato da chi usa il programma */
	uint8_t expected_cliff;      /* byte 3 che vale "sollevato": GD32_CLIFF_ALL_VOID, salvo interlock_set_expected_cliff() */
	int64_t max_age_ns;        /* età massima dell'ultimo frame dei sensori */

	int has_sensors;            /* è arrivato almeno un frame dei sensori */
	int seen_lifted;           /* almeno una volta "sollevato" */
	int lifted_now;         /* l'ultimo frame dei sensori dice "sollevato" */
	uint8_t cliff;             /* byte 3 dell'ultimo frame dei sensori */
	int64_t last_sensors_ns;    /* quando è arrivato */

	int tripped;              /* il blocco è scattato: motori fermi fino alla fine */
	const char *reason;        /* perché è scattato (testo fisso) */
	int64_t tripped_ns;            /* quando è scattato */

	uint64_t refused, forbidden;
} interlock_t;

void interlock_init(interlock_t *s, int lifted, int64_t max_age_ns);

/*
 * ECCEZIONE, solo per i banchi di prova in cui il robot è sollevato ma alcuni anti-caduta sono coperti dal supporto
 * (2026-10-05: scatole sotto il muso, byte 3 = 0x09). Cambia lo schema degli anti-caduta che vale "sollevato" da
 * 0x0f (tutti nel vuoto, la regola normale) a `cliff`. Il resto non cambia: sensori alimentati, non sulla base,
 * frame recenti, e il blocco SCATTA se lo schema cambia (robot appoggiato o toccato).
 */
void interlock_set_expected_cliff(interlock_t *s, uint8_t cliff);

/* 1 se cmd comanda un motore pericoloso (ruote, spazzole, aspirazione), 0 altrimenti. */
int interlock_is_motor(uint8_t cmd);

/* 1 se cmd è sempre vietato. */
int interlock_is_forbidden(uint8_t cmd);

/*
 * Da chiamare per ogni frame ricevuto dal GD32 (gli altri frame sono ignorati).
 * Ritorna 1 se il blocco è scattato ADESSO: chi chiama deve mandare subito i frame di arresto.
 */
int interlock_received(interlock_t *s, const gd32_frame_t *f, int64_t now);

/*
 * Da chiamare spesso (ogni giro del ciclo): controlla che i sensori arrivino ancora.
 * Ritorna 1 se il blocco è scattato ADESSO.
 */
int interlock_check(interlock_t *s, int64_t now);

/* Decide se il frame (cmd, dati) può partire. Aggiorna i contatori dei rifiutati e dei vietati. */
interlock_result_t interlock_permit(interlock_t *s, uint8_t cmd, const uint8_t *data, size_t n, int64_t now);

/*
 * Scrive in out, uno dopo l'altro, i frame completi che fermano tutti i motori: velocità e velocità delle ruote
 * a zero, spazzole a zero, aspirazione a zero (anche in PWM). Ritorna i byte scritti (0 se dim non basta).
 */
size_t interlock_stop_frames(uint8_t *out, size_t size);

/*
 * Modalità "A TERRA" (il driver, uso normale; fase 3). Riduce il comando di velocità (v, omega in tick/s, le unità
 * di 0x66) in base all'ultimo frame dei sensori e ritorna i motivi (bit SIC_LIM_*), 0 se il comando passa intero:
 *   - INTERLOCK_LIM_LIFTED: tutti e 4 gli anti-caduta nel vuoto, sensori accesi e fuori dalla base -> il robot è stato
 *     sollevato (o è caduto): v = omega = 0;
 *   - INTERLOCK_LIM_BASE: sulla base (contatto o carica) -> v = omega = 0, salvo leaving_base = 1, che permette di
 *     muoversi piano (|v| <= 0,1 m/s, |omega| <= 0,5 rad/s) per uscire;
 *   - INTERLOCK_LIM_SENSORS: sensori spenti (0x9d 00) fuori dalla base -> niente moto in avanti (bumper e anti-caduta
 *     non sono affidabili);
 *   - INTERLOCK_LIM_BUMPER: un bumper premuto -> niente moto in avanti (indietro e rotazione sì, per allontanarsi);
 *   - INTERLOCK_LIM_CLIFF: un anti-caduta nel vuoto -> niente moto in avanti.
 * Le spazzole non sono limitate qui (il driver le ferma con il robot sollevato o sulla base).
 */
enum {
	INTERLOCK_LIM_LIFTED = 1, INTERLOCK_LIM_BASE = 2, INTERLOCK_LIM_SENSORS = 4, INTERLOCK_LIM_BUMPER = 8, INTERLOCK_LIM_CLIFF = 16,
};
int interlock_ground_limit(const gd32_sensors_t *s, int leaving_base, int32_t *v_tick, int32_t *w_tick);

#endif /* GD32_INTERLOCK_H */
