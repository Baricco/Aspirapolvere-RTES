/*
 * gd32sim.h - Il GD32 simulato: fisica del robot e comportamento del firmware, visti dalla seriale.
 *
 * Riproduce quello che abbiamo misurato sul robot (docs/RIEPILOGO.md, docs/fase2.md, docs/fase3.md) con la geometria
 * di sim/robot.h e la dinamica tarata di sim/parametri.txt (tools/taratura/tara.py):
 *   - frame dei sensori 0x15 ogni 20,0006 ms, consegnato con il ritardo misurato (fino a qualche ms: gli intervalli
 *     visti dal SoC variano come sul robot); i byte non calcolati vengono da un frame vero a riposo;
 *   - risposte della sequenza di avvio; heartbeat del GD32 ogni 2 s;
 *   - motori: 0x65 modalità (00 spenti, 01 per ruota con 0x67, 02 v/omega con 0x66); il comando vale dopo ~29 ms; v e
 *     omega salgono e scendono con rampe diverse (in rotazione molto più lente); la velocità misurata (byte 32/36) è
 *     calcolata sugli encoder in una finestra di 70 ms (passi di ~7 unità, come il vero);
 *   - senza heartbeat 0x06 per 1,9 s il GD32 toglie potenza ai motori; senza 0x66 mantiene l'ultima velocità;
 *   - con gli anti-caduta anteriori sul vuoto non va avanti in modalità 02 (con 0x19 01 retromarcia a metà velocità);
 *   - urti: il CORPO preme il bumper (se l'urto è davanti), la TORRETTA del lidar ferma il robot senza bumper;
 *   - spazzola centrale negativa = sblocco per ~4,2 s, poi si ferma da sola;
 *   - senza keepalive 0x0d per ~190 s il GD32 RESETTA il robot (qui: 5 s di silenzio e stato azzerato);
 *   - sospensione 0x04 / risveglio 0x05 con il tasto di accensione;
 *   - carica: contatti DIETRO; sui contatti della base parte da sola dopo ~12 s e si ferma a 16,3 V;
 *   - 0x9d 00 (sensori spenti): byte 64 bit 5 a 0, bumper letti "premuti"; byte 64 bit 7 = lidar alimentato;
 *   - batteria: curva di scarica misurata, capacità tarata, caduta sotto carico; correnti dei motori tarate; IMU con la
 *     deriva e l'inclinazione misurate.
 * Le cose NON misurate sono [?] in sim/robot.h e in docs/fase3.md.
 *
 * Tempo: secondi (double) dall'avvio del simulatore. Il modulo non usa Linux: i frame da mandare vanno in una coda
 * con l'istante di consegna (gd32sim_prossimo), che chi lo usa scrive sulla pseudo-seriale.
 */
#ifndef GD32SIM_H
#define GD32SIM_H

#include <stddef.h>
#include <stdint.h>

#include "../gd32/gd32.h"
#include "mondo.h"
#include "robot.h"

#define SIM_CODA      512
#define SIM_STORIA    64
#define SIM_COMANDI   32
#define SIM_FINESTRA  16          /* campioni degli encoder (passi da 10 ms) per la velocità misurata */

typedef struct {
	double t;
	uint8_t cmd, n;
	uint8_t dati[12];
} sim_cmd_log_t;

typedef struct {
	double t;                     /* istante di consegna al SoC */
	uint16_t n;
	uint8_t b[GD32_MAX_FRAME];
} sim_uscita_t;

typedef struct {
	double t;                     /* da quando vale */
	int modo;                     /* -1 = 0x66, -2 = 0x67, altrimenti nuovo valore di 0x65 */
	int32_t a, b;
} sim_comando_t;

typedef struct {
	/* configurazione */
	double velocita_batteria;     /* moltiplica consumo e carica (1 = tempo reale) */
	double timeout_keepalive;     /* secondi senza 0x0d prima del reset (190 come il robot) */
	int senza_ritardi;            /* 1 = consegna i frame senza il ritardo misurato (test) */
	uint32_t seme;

	/* fisica */
	double x, y, dir;             /* posizione (m) e direzione (rad) */
	double v_cmd, w_cmd;          /* velocità richieste dopo le rampe, in unità di 0x66 (tick/s per ruota) */
	double vel_sx, vel_dx;        /* velocità delle ruote (tick/s) */
	double dist_sx, dist_dx;      /* tick percorsi da ogni ruota (continui) */
	int32_t enc_sx, enc_dx;
	int32_t storia_enc[SIM_FINESTRA][2];
	double storia_t[SIM_FINESTRA];
	int i_enc;
	double prossimo_enc;
	int sollevato, caduto;
	int bloccato, incastrato;     /* urto del corpo / della sola torretta nell'ultimo passo */
	double ang_urto, t_urto;      /* urto del corpo: direzione (rispetto al robot) e istante */
	double t_incastro;            /* ultimo urto della sola torretta */
	int bumper_sx, bumper_dx;
	uint8_t cliff;                /* bit 0 sx laterale ... bit 3 dx laterale */
	int contatto_base, in_carica;
	double t_contatto;
	uint8_t ir_base;
	double soc;                   /* 1 = 16,3 V, 0 = 13,8 V, negativo = riserva fino alla protezione */
	double corrente, tensione;
	double imbardata, vel_imbardata;     /* rad, rad/s (angolo integrato dal GD32: deriva) */
	int cassetto;
	double tasto_da[2];
	double t_avvio_ruote;         /* per il picco di corrente alla partenza */

	/* firmware */
	double t_simulato;
	int acceso;
	double t_riaccensione;
	int modo;
	int32_t cmd_v, cmd_w, cmd_ruote[2];  /* comandi in vigore (dopo il ritardo) */
	sim_comando_t coda_cmd[SIM_COMANDI];
	int n_coda_cmd;
	int spazzola_lat, spazzola_cen, eco_lat, eco_cen;
	double fine_sblocco;
	int aspirazione_giri, aspirazione_pwm;
	double ventola_giri;
	uint8_t led;
	int sensori_accesi, cliff_retromarcia, cliff_ir, lidar_alimentato, lidar_potenza;
	int sospeso, risveglio;
	double ultimo_hb, ultimo_ka;
	int motori_alimentati;
	double prossimo_sensori, prossimo_hb_gd32, prossimo_05, risposta_08;

	/* contatori e storia */
	uint64_t frame_ricevuti, frame_sensori, reset, rx_per_cmd[256];
	int n_storia, i_storia;
	sim_cmd_log_t storia[SIM_STORIA];
	char avviso[160];
	double t_avviso;

	/* frame da consegnare (coda circolare, in ordine di consegna) */
	sim_uscita_t coda[SIM_CODA];
	int testa, quanti;
	double ultima_consegna;
	uint64_t persi;
} gd32sim_t;

void gd32sim_init(gd32sim_t *g, const mondo_t *m, double t);
void gd32sim_ricevi(gd32sim_t *g, const gd32_frame_t *f, double t);
void gd32sim_avanza(gd32sim_t *g, const mondo_t *m, double t);

/* Il prossimo frame da consegnare se il suo istante è <= t (lo toglie dalla coda), altrimenti NULL. */
const sim_uscita_t *gd32sim_prossimo(gd32sim_t *g, double t);
/* Istante di consegna del prossimo frame in coda (INFINITY se vuota). */
double gd32sim_prossima_consegna(const gd32sim_t *g);

void gd32sim_sposta(gd32sim_t *g, double x, double y, double dir);
void gd32sim_velocita(const gd32sim_t *g, double *v, double *w);   /* m/s, rad/s reali del robot */
void gd32sim_dati_sensori(const gd32sim_t *g, double t, uint8_t d[96]);

/* Generatore di numeri (xorshift) usato anche dal lidar simulato. */
double sim_caso(uint32_t *seme);
double sim_gauss(uint32_t *seme);

#endif /* GD32SIM_H */
