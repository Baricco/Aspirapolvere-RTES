/*
 * batteria.h - Stato della batteria per il driver: tensione media, percentuale stimata e soglie di sicurezza.
 *
 * Il GD32 dà solo la tensione in decimi di volt (byte 8) e la temperatura (byte 9), non la corrente.
 *
 * Percentuale: stessa stima del software originale (everest-server, classe CBatteryCapacity3200), così il numero
 * coincide con quello che mostrava l'app. In breve:
 *   - la tensione è la media degli ultimi 100 s (5000 frame);
 *   - la curva tensione -> carica dipende dal carico: robot fermo, ruote in movimento, sulla base;
 *   - lontano dalla base la percentuale non risale mai: se la curva dice meno, scende piano verso la curva
 *     (costante di tempo 100 s); se dice di più (tensione risalita a carico tolto) resta ferma e cala dell'1 % ogni
 *     ~324 s da fermo o ~170 s in movimento, come un consumo stimato;
 *   - sulla base sale dell'1 % ogni ~100 s (~120 s sotto 15 V).
 *   - con l'aspirazione accesa la curva dipende dai giri della ventola (7 curve, da 600 a 2600 Pa), con un calo a tempo
 *     più rapido quanto più la ventola è forte.
 * Dettagli, costanti e indirizzi nel firmware: docs/hardware.md, sezione "Batteria".
 *
 * Livelli (con isteresi, per non oscillare sul confine), sulla media dell'ultimo secondo:
 *   BAT_OK
 *   BAT_BASSA     sotto 14,2 V (si torna OK sopra 14,4 V): il driver accende il LED 6 (giallo, lampeggio lento)
 *   BAT_CRITICA   sotto 13,8 V (si torna BASSA sopra 14,0 V): il driver FERMA i motori e li rifiuta
 * Mentre è in carica il livello è sempre BAT_OK (la tensione sale comunque).
 * Le soglie sono più prudenti di quelle dell'originale, che non fa partire la pulizia sotto il 15 % (~13,8 V da
 * fermo) e la interrompe sotto l'8 % (~13,4 V da fermo, ~13,25 V in movimento); protezione a 12,0 V.
 *
 * Logica pura, senza Linux: si prova con gli unit test.
 */
#ifndef BATTERIA_H
#define BATTERIA_H

#include <stdint.h>

enum { BAT_OK = 0, BAT_BASSA = 1, BAT_CRITICA = 2 };

/* Stati di carico, con i numeri dell'originale: l'ordine conta (passando a un carico più forte, cioè a un numero più
 * basso, la media della tensione riparte da zero). Da 1 a 8 l'aspirazione, scelta dai giri della ventola (il 2 non
 * lo usa nessuno); 9 e 10 senza aspirazione. */
enum {
	BAT_STATO_CARICA = 0,
	BAT_STATO_2600PA = 1,        /* ventola >= 20 000 giri/min */
	BAT_STATO_2000PA = 3,        /* >= 19 000 */
	BAT_STATO_1500PA = 4,        /* >= 17 000 */
	BAT_STATO_1200PA = 5,        /* >= 15 000 */
	BAT_STATO_1000PA = 6,        /* >= 14 000 */
	BAT_STATO_800PA = 7,         /* >= 12 000 */
	BAT_STATO_600PA = 8,         /* > 1000 */
	BAT_STATO_MOTO = 9,          /* ventola ferma (0 giri) e ruote in movimento */
	BAT_STATO_FERMO = 10,        /* il resto (anche ventola fra 1 e 1000 giri/min) */
};

#define BAT_SOGLIA_BASSA_DV    142
#define BAT_USCITA_BASSA_DV    144
#define BAT_SOGLIA_CRITICA_DV  138
#define BAT_USCITA_CRITICA_DV  140
#define BAT_MEDIA_CAMPIONI     50         /* 1 s di frame dei sensori: per le soglie */
#define BAT_MEDIA_STIMA        5000       /* 100 s: media della tensione per la percentuale */
#define BAT_MEDIA_CARICA       1500       /* 30 s: quanto è stato acceso il bit "in carica" */

typedef struct {
	/* soglie */
	int campioni[BAT_MEDIA_CAMPIONI];
	int n, i, somma;
	int livello;
	/* percentuale */
	uint8_t tensioni[BAT_MEDIA_STIMA];
	int n_t, i_t;
	long somma_t;
	uint8_t carica[BAT_MEDIA_CARICA];
	int n_c, i_c;
	long somma_c;
	int stato;                   /* BAT_STATO_* */
	double stima;                /* 0..1; < 0 prima del primo frame */
	int conta_carica, conta_salita;
} batteria_t;

void batteria_init(batteria_t *b);

/*
 * Un frame dei sensori: tensione in decimi di volt (byte 8), in carica (byte 7 bit 0), sulla base (byte 7 bit 0 o 1),
 * ruote in movimento (velocità misurate, byte 32 e 36: l'originale guarda v + |ω| > 0, quindi la retromarcia dritta
 * conta come "fermo"), giri al minuto della ventola (byte 69 x 100). Ritorna il livello aggiornato.
 */
int batteria_aggiorna(batteria_t *b, int tensione_dv, int in_carica, int sulla_base, int in_moto, int giri_ventola);

/* Lo stato di carico che l'originale sceglierebbe (BAT_STATO_*), lontano dalla base. */
int batteria_stato_carico(int in_moto, int giri_ventola);

/* Tensione media dell'ultimo secondo, in volt. */
double batteria_volt(const batteria_t *b);

/* Percentuale stimata (0..100), come l'originale; 0 prima del primo frame. */
int batteria_percentuale(const batteria_t *b);

/* Curva dell'originale da tensione a carica (0..1, non limitata) per uno stato lontano dalla base (da 1 a 10). */
double batteria_curva(int stato, double volt);

#endif /* BATTERIA_H */
