/*
 * parametri.h - Parametri dinamici del simulatore, ricavati dalle registrazioni del robot (tools/taratura/tara.py,
 * che scrive sim/parametri.txt). I default qui sono gli stessi valori tarati: il simulatore funziona anche senza file.
 * La geometria fissa del robot è invece in sim/robot.h.
 */
#ifndef PARAMETRI_H
#define PARAMETRI_H

#include <stdint.h>

#define PAR_MAX_QUANTILI 16

typedef struct {
	/* GD32 e consegna dei frame */
	double gd32_periodo_ms;                   /* 20,00056: il GD32 manda i sensori a questo periodo */
	int arrivo_n;                             /* ritardo con cui i frame arrivano al SoC: distribuzione per quantili */
	double arrivo_prob[PAR_MAX_QUANTILI], arrivo_ms[PAR_MAX_QUANTILI];

	/* ruote (unità di 0x66: tick/s per ruota) */
	double ruote_ritardo_ms;                  /* dal comando all'inizio del movimento */
	double rampa_v_su, rampa_v_giu;           /* tick/s² */
	double rampa_w_su, rampa_w_giu;
	double ruote_finestra_ms;                 /* finestra della velocità misurata (byte 32/36) */
	double ruote_timeout_hb;                  /* s senza heartbeat 0x06 prima che il GD32 tolga potenza */
	double ruote_inerzia;                     /* tick/s²: senza potenza le ruote si fermano da sole */
	double corrente_ruota_base, corrente_ruota_picchi, corrente_ruota_avvio;   /* mA */

	/* IMU */
	double imu_deriva_gradi_s, imu_rumore_velocita;   /* deriva dell'angolo di imbardata; rumore in 0,01 °/s */
	int imu_beccheggio, imu_rollio;                   /* 0,01° a riposo */

	/* valori a riposo e attuatori */
	int corrente_riposo_laterale, corrente_riposo_centrale, corrente_riposo_aspirazione, byte84;
	int temperatura_c;
	double corrente_centrale_accesa, corrente_centrale_sblocco;
	double aspirazione_giri_per_pwm, aspirazione_corrente_max, aspirazione_giri_max, aspirazione_esponente;
	uint8_t frame_riposo[96];                 /* un frame vero: i byte non calcolati dal simulatore vengono da qui */

	/* lidar */
	double lidar_a, lidar_b;                  /* giri/s = a x potenza + b */
	double lidar_tau_su, lidar_tau_giu, lidar_tau_spento;
	double lidar_misure_s;
	double lidar_rumore_mm[7];                /* deviazione standard per 0-1, 1-2, ... 6+ m */
	double lidar_buchi;                       /* probabilità di un punto senza eco isolato */
	double lidar_intensita, lidar_intensita_dev;
	double lidar_portata;

	/* batteria */
	double batteria_utile_ah, batteria_totale_ah;
} sim_parametri_t;

/* I parametri in uso (valori di default finché non si legge un file). */
extern sim_parametri_t par;

/* Legge un file "chiave = valore  # commento". Le chiavi sconosciute si ignorano. Ritorna 0, oppure -1 se il file
 * non si apre (restano i default). */
int parametri_leggi(const char *percorso);

/* Mette in par.frame_riposo il frame vero di default (quello di sim/parametri.txt al momento della taratura). */
void parametri_frame_default(void);

/* Un ritardo di consegna estratto dalla distribuzione (u uniforme in [0,1)), in secondi. */
double parametri_ritardo_arrivo(double u);

#endif /* PARAMETRI_H */
