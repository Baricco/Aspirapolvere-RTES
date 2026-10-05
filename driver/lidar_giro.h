/*
 * lidar_giro.h - Dal flusso di pacchetti del lidar a giri completi, e regolazione della velocità del motore.
 *
 * Il lidar manda un pacchetto 0xAD per ogni settore di 22,5° (16 per giro, docs/fase2.md C1 e D4):
 *     param[0]      u8   velocità x 0,05 giri/s
 *     param[1..2]   i16  BE, sempre 82 (0,82) [?]
 *     param[3..4]   u16  BE, angolo di inizio del settore in centesimi di grado
 *     poi n punti da 3 byte: u8 intensità, u16 BE distanza in quarti di mm (0 = nessuna eco)
 * Il punto i del settore è all'angolo inizio + i x 22,5° / n. Angoli del LIDAR: 0° = dietro il robot, 90° = destra,
 * 180° = davanti, 270° = sinistra, crescenti in senso antiorario visto dall'alto. Sul robot vero c'è uno scarto di
 * circa +8° (letto = vero + 8°) [?]: il driver lo corregge con drv_config_t.scarto_lidar_cdeg.
 * Sotto ~4 giri/s il lidar manda solo pacchetti di velocità 0xAE (param[0] u8 x 0,05 giri/s).
 *
 * L'assemblatore raccoglie i settori; quando l'angolo di inizio torna indietro (nuovo giro) consegna il giro appena
 * finito, se ha almeno 12 settori su 16 (all'accensione il primo giro è incompleto).
 *
 * Il regolatore è quello di AuxCtrl (lidarSpeedControl, fase 2 C1), da chiamare ogni 20 ms:
 *     potenza = 60 + I + 10 e + 10 (e - e_prec),   e = obiettivo - velocità (giri/s),   I = somma degli e, in +-20
 * Logica pura, senza Linux.
 */
#ifndef LIDAR_GIRO_H
#define LIDAR_GIRO_H

#include <stdint.h>

#include "../lidar/lidar.h"

#define LIDAR_GIRO_MAX_PUNTI 800       /* 2050 misure/s a 4 giri/s = 513; abbondante */

typedef struct {
	uint16_t angolo_cdeg;    /* angolo del lidar corretto dello scarto, in centesimi di grado (0..35999) */
	uint16_t distanza_mm;    /* 0 = nessuna eco */
	uint8_t intensita;
} lidar_punto_t;

typedef struct {
	uint32_t numero;         /* progressivo dei giri consegnati */
	int64_t t_inizio_ns, t_fine_ns;   /* arrivo del primo e dell'ultimo pacchetto del giro */
	double giri_s;           /* velocità media riportata dal lidar */
	uint16_t settori;        /* settori ricevuti (16 se il giro è completo) */
	uint16_t n;              /* punti */
	lidar_punto_t p[LIDAR_GIRO_MAX_PUNTI];
} lidar_giro_t;

typedef struct {
	lidar_giro_t corrente;
	int ultimo_inizio;       /* angolo di inizio dell'ultimo settore, -1 = nessuno */
	double somma_vel;
	int scarto_cdeg;
	double giri_s;           /* ultima velocità letta (da 0xAD o 0xAE) */
	int64_t t_velocita;      /* quando è stata letta */
	uint32_t giri, giri_scartati, settori;
} lidar_assemblatore_t;

void lidar_assemblatore_init(lidar_assemblatore_t *a, int scarto_cdeg);

/*
 * Un pacchetto del lidar arrivato all'istante t. Ritorna 1 se ha completato un giro, che copia in *giro.
 * Aggiorna sempre a->giri_s con la velocità del pacchetto (0xAD o 0xAE).
 */
int lidar_assembla(lidar_assemblatore_t *a, const lidar_pacchetto_t *p, int64_t t, lidar_giro_t *giro);

typedef struct {
	double obiettivo;        /* giri/s */
	double integrale, errore_prec;
	int potenza;             /* ultima potenza calcolata, 0..100 */
} lidar_regolatore_t;

void lidar_regolatore_init(lidar_regolatore_t *r, double obiettivo);

/* Un passo del regolatore (ogni 20 ms) con la velocità misurata. Ritorna la potenza da mandare con 0x71. */
int lidar_regola(lidar_regolatore_t *r, double misurata);

#endif /* LIDAR_GIRO_H */
