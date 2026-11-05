/*
 * aggancio.h - Messa in carica: guidata dagli infrarossi della base, con lo stesso approccio del firmware originale
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
 * (gd32/sicurezza.h). La zona attorno alla base resta proibita per i task: ci entra solo questo codice.
 *
 * È logica pura, senza Linux e senza stato globale: la provano i test con un modello del robot (test/test_driver_logica.c).
 * Dentro il driver la protegge d.mx.
 */
#ifndef AGGANCIO_H
#define AGGANCIO_H

#include <stdint.h>

#include "../gd32/gd32.h"
#include "lidar_giro.h"

/* Il punto a cui la navigazione porta il robot prima di chiedere l'aggancio, e fino a cui esce dalla base. */
#define AGG_PUNTO_X_M       0.30

#define AGG_TENTATIVI       3

/* Esito, in drv_stato_t.aggancio */
enum { AGG_FERMO = 0, AGG_IN_CORSO, AGG_FATTO, AGG_FALLITO };

/* Fasi interne, in drv_stato_t.aggancio_fase (per i registri e le prove) */
typedef enum {
	AGG_F_NIENTE = 0,
	AGG_F_CERCA,      /* ruota finché trova il segnale e lo perde */
	AGG_F_CENTRA,     /* torna verso il centro dell'arco in cui lo vedeva */
	AGG_F_AVVICINA,   /* avanti di fronte alla base, seguendo il confine fra i fasci */
	AGG_F_RIFINISCI,  /* col muso a pochi cm: oscilla piano e si mette a metà */
	AGG_F_STACCA,     /* bumper: indietro di poco */
	AGG_F_INDIETRO,   /* arrivato lontano dall'asse: indietro di 20 cm, poi si ricomincia */
	AGG_F_GIRA,       /* 180° con l'IMU */
	AGG_F_ULTIMI,     /* in retromarcia verso i contatti */
	AGG_F_CONTATTO,   /* fermo sui contatti: aspetta la carica */
	AGG_F_RITIRA,     /* avanti, per riprovare */
	AGG_F_PREMI,      /* sul primo contatto: ancora pochi mm in retromarcia, finché la carica è stabile */
} agg_fase_t;

typedef struct {
	int esito;           /* AGG_FERMO, AGG_IN_CORSO, AGG_FATTO, AGG_FALLITO */
	agg_fase_t fase;
	int tentativo;       /* da 1 */
	const char *motivo;  /* perché è fallito o si è ritirato (testo fisso) */

	/* comando verso le ruote, m/s e rad/s */
	double v, w;

	/* dall'IMU e dagli encoder */
	int ha_prec;
	int16_t imbardata_prec;
	int32_t enc_sx_prec, enc_dx_prec;
	double yaw;          /* radianti, da quando è partito l'aggancio */
	double strada;       /* metri nella fase corrente, avanti = positiva */
	double arco;         /* radianti nella fase corrente */

	/* dall'ultimo giro del lidar */
	double dist_davanti; /* metri dal lidar a ciò che ha davanti (media); 1,5 se non vede niente */
	int punti_vicini;    /* punti davanti col muso a meno di 6 cm */
	int64_t t_giro;

	/* ricerca e direzione */
	int visto, parte_dentro, verso;
	int frame_asse, riprove_asse;
	double yaw_primo, yaw_centro, direzione;
	double yaw_a, yaw_b;
	int ha_a, ha_b;
	int lato_prec;
	agg_fase_t dopo_stacco;

	int frame, frame2;
	int64_t t_inizio, t_fase, t_segnale;

	/* sui contatti */
	int frame_contatto, frame_totali;
	int64_t t_senza_contatto;
} aggancio_t;

void aggancio_init(aggancio_t *a);

/* Parte da capo. */
void aggancio_avvia(aggancio_t *a, int64_t t);

/* Si ferma subito, qualunque cosa stia facendo: esito AGG_FERMO. */
void aggancio_annulla(aggancio_t *a);

/* 1 se in questo momento il comando alle ruote lo decide l'aggancio. */
int aggancio_attivo(const aggancio_t *a);

/* 1 se serve il permesso di muoversi con i contatti chiusi (per premere sul primo contatto, o per staccarsi dalla base). */
int aggancio_stacco(const aggancio_t *a);

/* Da chiamare a ogni giro completo del lidar. */
void aggancio_giro(aggancio_t *a, const lidar_giro_t *g, int64_t t);

/* Da chiamare a ogni frame dei sensori: aggiorna a->v, a->w. */
void aggancio_frame(aggancio_t *a, const gd32_sensori_t *s, int64_t t);

#endif /* AGGANCIO_H */
