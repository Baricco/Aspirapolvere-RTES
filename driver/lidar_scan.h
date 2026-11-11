/*
 * lidar_scan.h - Dal flusso di pacchetti del lidar a giri completi, e regolazione della velocità del motore.
 *
 * Il lidar manda un pacchetto 0xAD per ogni settore di 22,5° (16 per giro, docs/fase2.md C1 e D4):
 *     param[0]      u8   velocità x 0,05 giri/s
 *     param[1..2]   i16  BE, sempre 82 (0,82) [?]
 *     param[3..4]   u16  BE, angolo di inizio del settore in centesimi di grado
 *     poi n punti da 3 byte: u8 intensità, u16 BE distanza in quarti di mm (0 = nessuna eco)
 * Il punto i del settore è all'angolo inizio + i x 22,5° / n. Angoli del LIDAR: 0° = dietro il robot, 90° = SINISTRA,
 * 180° = davanti, 270° = DESTRA, crescenti in senso ORARIO visto dall'alto (il lidar è montato capovolto rispetto al
 * robot). Corretto il 5/10: fino ad allora si pensava antiorario, e il driver scambiava destra e sinistra (prova con
 * la rotazione: l'IMU segna +40,7°, la stanza vista dal lidar ruota di +42° invece che di -42°; conferma da
 * kaiaai/VacuumTiger, proscenic-m6pro.toml: scale = -1, offset = pi).
 * Lo 0 del lidar è spostato di LIDAR_OFFSET_CDEG (letto = vero + scarto): il driver lo toglie con
 * drv_config_t.lidar_offset_cdeg. Il lidar è un triangolatore: il punto misurato non sta sul raggio dal centro di
 * rotazione ma LIDAR_OTTICO più in là, a destra del raggio (drv_point_xy).
 * Sotto ~4 giri/s il lidar manda solo pacchetti di velocità 0xAE (param[0] u8 x 0,05 giri/s).
 *
 * L'assemblatore raccoglie i settori; quando l'angolo di inizio torna indietro (nuovo giro) consegna il giro appena
 * finito, se ha almeno 12 settori su 16 (all'accensione il primo giro è incompleto).
 *
 * Il regolatore è quello di AuxCtrl (lidarSpeedControl, fase 2 C1), da chiamare ogni 20 ms:
 *     potenza = 60 + I + 10 e + 10 (e - e_prec),   e = obiettivo - velocità (giri/s),   I = somma degli e, in +-20
 * Logica pura, senza Linux.
 */
#ifndef LIDAR_SCAN_H
#define LIDAR_SCAN_H

#include <stdint.h>

#include "../lidar/lidar.h"

#define LIDAR_SCAN_MAX_POINTS 800       /* 2050 misure/s a 4 giri/s = 513; abbondante */

/* Montaggio del lidar sul robot [misura del 5/10, docs/fase3.md; seconda fonte: kaiaai/VacuumTiger]:
 *   centro di rotazione 11,2 cm DIETRO il centro del robot (bumper contro un muro piatto: muro a 289 mm dal lidar e a
 *   174,5 mm dal centro; kaiaai: 110 mm), sull'asse;
 *   scarto dello 0: 14,5° (direzione della traslazione in avanti vista dal lidar, due prove: 180° +- 1,5°; kaiaai
 *   12,5°); con lo scostamento ottico spiega anche gli 8° visti in fase 2 con una scatola a 16 cm;
 *   scostamento ottico 2,5 cm (kaiaai: optical_offset; il segno dalle tre misure). */
#define LIDAR_X_M          (-0.112)
#define LIDAR_Y_M          0.0
#define LIDAR_OFFSET_CDEG  1450
#define LIDAR_OPTICAL_M     0.025

typedef struct {
	uint16_t angle_cdeg;    /* angolo del lidar corretto dello scarto, in centesimi di grado (0..35999) */
	uint16_t distance_mm;    /* 0 = nessuna eco */
	uint8_t intensity;
} lidar_point_t;

/* Punto nel sistema del robot: origine al CENTRO del robot, x in avanti, y a sinistra, in metri (è drv_point_xy). */
void lidar_point_xy(const lidar_point_t *p, double *x, double *y);

typedef struct {
	uint32_t number;         /* progressivo dei giri consegnati */
	int64_t t_start_ns, t_end_ns;   /* arrivo del primo e dell'ultimo pacchetto del giro */
	double rev_s;           /* velocità media riportata dal lidar */
	uint16_t sectors;        /* settori ricevuti (16 se il giro è completo) */
	uint16_t n;              /* punti */
	lidar_point_t p[LIDAR_SCAN_MAX_POINTS];
} lidar_scan_t;

typedef struct {
	lidar_scan_t current;
	int last_start;       /* angolo di inizio dell'ultimo settore, -1 = nessuno */
	double speed_sum;
	int offset_cdeg;
	double rev_s;           /* ultima velocità letta (da 0xAD o 0xAE) */
	int64_t t_speed;      /* quando è stata letta */
	uint32_t revolutions, discarded_scans, sectors;
} lidar_assembler_t;

void lidar_assembler_init(lidar_assembler_t *a, int offset_cdeg);

/*
 * Un pacchetto del lidar arrivato all'istante t. Ritorna 1 se ha completato un giro, che copia in *giro.
 * Aggiorna sempre a->rev_s con la velocità del pacchetto (0xAD o 0xAE).
 */
int lidar_assemble(lidar_assembler_t *a, const lidar_packet_t *p, int64_t t, lidar_scan_t *scan);

typedef struct {
	double target;        /* giri/s */
	double integral, prev_error;
	int power;             /* ultima potenza calcolata, 0..100 */
} lidar_regulator_t;

void lidar_regulator_init(lidar_regulator_t *r, double target);

/* Un passo del regolatore (ogni 20 ms) con la velocità misurata. Ritorna la potenza da mandare con 0x71. */
int lidar_regulate(lidar_regulator_t *r, double measured);

#endif /* LIDAR_SCAN_H */
