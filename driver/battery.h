/*
 * battery.h - Stato della batteria per il driver: tensione media, percentuale stimata e soglie di sicurezza.
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
 *   BAT_LOW     sotto 14,2 V (si torna OK sopra 14,4 V): il driver accende il LED 6 (giallo, lampeggio lento)
 *   BAT_CRITICAL   sotto 13,8 V (si torna BASSA sopra 14,0 V): il driver FERMA i motori e li rifiuta
 * Mentre è in carica il livello è sempre BAT_OK (la tensione sale comunque).
 * Le soglie sono più prudenti di quelle dell'originale, che non fa partire la pulizia sotto il 15 % (~13,8 V da
 * fermo) e la interrompe sotto l'8 % (~13,4 V da fermo, ~13,25 V in movimento); protezione a 12,0 V.
 *
 * Logica pura, senza Linux: si prova con gli unit test.
 */
#ifndef BATTERY_H
#define BATTERY_H

#include <stdint.h>

enum { BAT_OK = 0, BAT_LOW = 1, BAT_CRITICAL = 2 };

/* Stati di carico, con i numeri dell'originale: l'ordine conta (passando a un carico più forte, cioè a un numero più
 * basso, la media della tensione riparte da zero). Da 1 a 8 l'aspirazione, scelta dai giri della ventola (il 2 non
 * lo usa nessuno); 9 e 10 senza aspirazione. */
enum {
	BAT_STATE_CHARGING = 0,
	BAT_STATE_2600PA = 1,        /* ventola >= 20 000 giri/min */
	BAT_STATE_2000PA = 3,        /* >= 19 000 */
	BAT_STATE_1500PA = 4,        /* >= 17 000 */
	BAT_STATE_1200PA = 5,        /* >= 15 000 */
	BAT_STATE_1000PA = 6,        /* >= 14 000 */
	BAT_STATE_800PA = 7,         /* >= 12 000 */
	BAT_STATE_600PA = 8,         /* > 1000 */
	BAT_STATE_MOVING = 9,          /* ventola ferma (0 giri) e ruote in movimento */
	BAT_STATE_IDLE = 10,        /* il resto (anche ventola fra 1 e 1000 giri/min) */
};

#define BAT_LOW_THRESHOLD_DV    142
#define BAT_LOW_EXIT_DV    144
#define BAT_CRITICAL_THRESHOLD_DV  138
#define BAT_CRITICAL_EXIT_DV  140
#define BAT_AVG_SAMPLES     50         /* 1 s di frame dei sensori: per le soglie */
#define BAT_AVG_ESTIMATE        5000       /* 100 s: media della tensione per la percentuale */
#define BAT_AVG_CHARGING       1500       /* 30 s: quanto è stato acceso il bit "in carica" */

typedef struct {
	/* soglie */
	int samples[BAT_AVG_SAMPLES];
	int n, i, sum;
	int level;
	/* percentuale */
	uint8_t voltages[BAT_AVG_ESTIMATE];
	int n_t, i_t;
	long sum_t;
	uint8_t charge[BAT_AVG_CHARGING];
	int n_c, i_c;
	long sum_c;
	int status;                   /* BAT_STATO_* */
	double estimate;                /* 0..1; < 0 prima del primo frame */
	int charge_count, rise_count;
} battery_t;

void battery_init(battery_t *b);

/*
 * Un frame dei sensori: tensione in decimi di volt (byte 8), in carica (byte 7 bit 0), sulla base (byte 7 bit 0 o 1),
 * ruote in movimento (velocità misurate, byte 32 e 36: l'originale guarda v + |ω| > 0, quindi la retromarcia dritta
 * conta come "fermo"), giri al minuto della ventola (byte 69 x 100). Ritorna il livello aggiornato.
 */
int battery_update(battery_t *b, int voltage_dv, int charging, int on_base, int moving, int fan_rpm);

/* Lo stato di carico che l'originale sceglierebbe (BAT_STATO_*), lontano dalla base. */
int battery_load_state(int moving, int fan_rpm);

/* Tensione media dell'ultimo secondo, in volt. */
double battery_volts(const battery_t *b);

/* Percentuale stimata (0..100), come l'originale; 0 prima del primo frame. */
int battery_percentage(const battery_t *b);

/* Curva dell'originale da tensione a carica (0..1, non limitata) per uno stato lontano dalla base (da 1 a 10). */
double battery_curve(int status, double volts);

#endif /* BATTERY_H */
