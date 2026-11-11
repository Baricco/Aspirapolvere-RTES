/*
 * docking.h - Messa in carica: guidata dagli infrarossi della base, con lo stesso approccio del firmware originale
 * (everest-server, CActionTransferMotionGoDock e CActionDockAlignState; non una copia di tutti i suoi stati).
 *
 * I ricevitori infrarossi sono davanti e i contatti dietro. Quindi il robot arriva DI FRONTE alla base guidato dagli
 * infrarossi, si ferma col muso a pochi centimetri, si gira di 180° lì e arretra alla cieca solo per quei pochi
 * centimetri. La guida non dipende da come appare la base al lidar: una base nera, o una scatola lì vicino, non la
 * ingannano. Il lidar dice solo quanto spazio c'è davanti al muso; se non vede la base ci si ferma col bumper.
 *
 * Fasi: cerca (ruota finché trova il segnale e lo perde) -> centra (torna verso il centro dell'arco) -> avvicina
 * (avanti seguendo il confine fra i fasci 1 e 2, che è l'asse della base) -> rifinisci (a pochi cm, oscillando piano) ->
 * gira (180° con l'IMU) -> ultimi centimetri -> contatto: fermo, aspetta la carica. Se non riesce avanza, e riprova.
 * Lasciato fuori dall'originale perché non è servito finora: lo spostamento laterale ad archi (fatto quando è vicino alla
 * base lontano dall'asse) e la seconda manovra col lidar (CActionTransferMotionCharge).
 *
 * Da chiamare da un punto qualsiasi da cui si veda la base, entro qualche metro: non serve essere già allineati.
 * Da quel momento il driver ignora le velocità dei task e guida da solo, con le sue protezioni ancora attive
 * (gd32/interlock.h). La zona attorno alla base resta proibita per i task: ci entra solo questo codice.
 *
 * È logica pura, senza Linux e senza stato globale: la provano i test con un modello del robot (test/test_driver_logica.c).
 * Dentro il driver la protegge d.mx.
 */
#ifndef DOCKING_H
#define DOCKING_H

#include <stdint.h>

#include "../gd32/gd32.h"
#include "lidar_scan.h"

/* Il punto a cui la navigazione porta il robot prima di chiedere l'aggancio, e fino a cui esce dalla base. */
#define DOCK_POINT_X_M       0.30

#define DOCK_ATTEMPTS       3

/* Esito, in drv_status_t.docking */
enum { DOCK_IDLE = 0, DOCK_RUNNING, DOCK_DONE, DOCK_FAILED };

/* Fasi interne, in drv_status_t.docking_phase (per i registri e le prove) */
typedef enum {
	DOCK_P_NONE = 0,
	DOCK_P_SEARCH,      /* ruota finché trova il segnale e lo perde */
	DOCK_P_CENTER,     /* torna verso il centro dell'arco in cui lo vedeva */
	DOCK_P_APPROACH,   /* avanti di fronte alla base, seguendo il confine fra i fasci */
	DOCK_P_FINE_TUNE,  /* col muso a pochi cm: oscilla piano e si mette a metà */
	DOCK_P_DETACH,     /* bumper: indietro di poco */
	DOCK_P_BACK_OFF,   /* arrivato lontano dall'asse: indietro di 20 cm, poi si ricomincia */
	DOCK_P_TURN,       /* 180° con l'IMU */
	DOCK_P_LAST_CM,     /* in retromarcia verso i contatti */
	DOCK_P_CONTACT,   /* fermo sui contatti: aspetta la carica */
	DOCK_P_RETREAT,     /* avanti, per riprovare */
	DOCK_P_PRESS,      /* sul primo contatto: ancora pochi mm in retromarcia, finché la carica è stabile */
} dock_phase_t;

typedef struct {
	int result;           /* DOCK_IDLE, DOCK_RUNNING, DOCK_DONE, DOCK_FAILED */
	dock_phase_t phase;
	int attempt;       /* da 1 */
	const char *reason;  /* perché è fallito o si è ritirato (testo fisso) */

	/* comando verso le ruote, m/s e rad/s */
	double v, w;

	/* dall'IMU e dagli encoder */
	int has_prev;
	int16_t prev_yaw_raw;
	int32_t prev_enc_left, prev_enc_right;
	double yaw;          /* radianti, da quando è partito l'aggancio */
	double distance;       /* metri nella fase corrente, avanti = positiva */
	double arc;         /* radianti nella fase corrente */

	/* dall'ultimo giro del lidar */
	double front_dist; /* metri dal lidar a ciò che ha davanti (media); 1,5 se non vede niente */
	int near_points;    /* punti davanti col muso a meno di 6 cm */
	int64_t t_scan;

	/* ricerca e direzione */
	int seen, starts_inside, turn_dir;
	int axis_frames, axis_retries;
	double yaw_first, yaw_center, direction;
	double yaw_a, yaw_b;
	int has_a, has_b;
	int prev_side;
	dock_phase_t after_detach;

	int frame, frame2;
	int64_t t_start, t_phase, t_signal;

	/* sui contatti */
	int contact_frames, total_frames;
	int64_t t_no_contact;
} docking_t;

void docking_init(docking_t *a);

/* Parte da capo. */
void docking_start(docking_t *a, int64_t t);

/* Si ferma subito, qualunque cosa stia facendo: esito DOCK_IDLE. */
void docking_cancel(docking_t *a);

/* Si ferma con esito DOCK_FAILED: per chi aspetta l'esito (per esempio se il robot viene sollevato). */
void docking_abort(docking_t *a, const char *reason);

/* 1 se in questo momento il comando alle ruote lo decide l'aggancio. */
int docking_active(const docking_t *a);

/* 1 se serve il permesso di muoversi con i contatti chiusi (per premere sul primo contatto, o per staccarsi dalla base). */
int docking_detach(const docking_t *a);

/* Da chiamare a ogni giro completo del lidar. */
void docking_scan(docking_t *a, const lidar_scan_t *g, int64_t t);

/* Da chiamare a ogni frame dei sensori: aggiorna a->v, a->w. */
void docking_frame(docking_t *a, const gd32_sensors_t *s, int64_t t);

#endif /* DOCKING_H */
