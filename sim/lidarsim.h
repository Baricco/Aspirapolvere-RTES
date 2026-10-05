/*
 * lidarsim.h - Il lidar 3irobotix LDS3 simulato, visto dalla seriale (/dev/ttyS1).
 *
 * Come quello vero (docs/fase2.md C1 e D4), con i parametri tarati di sim/parametri.txt:
 *   - si accende quando il GD32 lo alimenta (0x97 01): prima un testo, poi il pacchetto di versione 0xAC;
 *   - velocità = a x potenza (0x71) + b, raggiunta con le costanti di tempo misurate (salita ~0,18 s; da spento il rotore
 *     rallenta in ~0,5 s);
 *   - sotto ~4 giri/s manda solo pacchetti di velocità 0xAE (ogni 100 ms), sopra solo scansioni 0xAD;
 *   - ogni 0xAD è un settore di 22,5° (16 per giro) con n punti, n = ~2040 misure/s / (16 x giri/s), al massimo 29;
 *   - angoli del lidar: 0° = dietro, 90° = sinistra, 180° = davanti, 270° = destra (ORARIO visto dall'alto); scarto
 *     di 14,5° fra lo 0 del lidar e il robot e scostamento ottico di 2,5 cm come sul robot vero (`scarto_gradi`,
 *     driver/lidar_giro.h);
 *   - misura sul suo piano (LIDAR_ALTEZZA in sim/robot.h): vede solo gli oggetti che lo tagliano;
 *   - rumore sulla distanza dalla deviazione standard misurata (7 mm fino a 3 m, 37 mm oltre 6 m), qualche punto
 *     isolato senza eco, niente eco oltre la portata; intensità come quelle misurate.
 * I pacchetti vanno in una coda con l'istante di consegna, come per il GD32.
 */
#ifndef LIDARSIM_H
#define LIDARSIM_H

#include <stddef.h>
#include <stdint.h>

#include "../lidar/lidar.h"
#include "mondo.h"

#define LIDARSIM_CODA 256
#define LIDARSIM_MAX_PUNTI 1024

typedef struct {
	double t;
	uint16_t n;
	uint8_t b[LIDAR_MAX_PACCHETTO];
} lidarsim_uscita_t;

typedef struct {
	double scarto_gradi;
	int senza_ritardi;
	uint32_t seme;

	int alimentato;
	double t_accensione;
	int stato_avvio;              /* 0 spento, 1 aspetta il testo, 2 aspetta la versione, 3 pronto */
	double giri_s, angolo, prossimo_ae, t_simulato;

	/* ultimo giro completo, in coordinate della stanza, per la pagina */
	int n_giro, n_corrente;
	float giro_x[LIDARSIM_MAX_PUNTI], giro_y[LIDARSIM_MAX_PUNTI];
	float cor_x[LIDARSIM_MAX_PUNTI], cor_y[LIDARSIM_MAX_PUNTI];
	uint64_t pacchetti, giri;

	lidarsim_uscita_t coda[LIDARSIM_CODA];
	int testa, quanti;
	double ultima_consegna;
	uint64_t persi;
} lidarsim_t;

void lidarsim_init(lidarsim_t *l, double t);

/* Avanza fino a t. alimentato e potenza vengono dal GD32 simulato; x, y, dir è la posizione del robot. */
void lidarsim_avanza(lidarsim_t *l, const mondo_t *m, double t, int alimentato, int potenza,
                     double x, double y, double dir);

/* Il prossimo pacchetto (o testo) da consegnare se è ora, altrimenti NULL. */
const lidarsim_uscita_t *lidarsim_prossimo(lidarsim_t *l, double t);
double lidarsim_prossima_consegna(const lidarsim_t *l);

/* Velocità a regime per una potenza 0..100. */
double lidarsim_giri_per_potenza(int potenza);

#endif /* LIDARSIM_H */
