/*
 * aggancio.h - Messa in carica: dal punto davanti alla base fino ai contatti, in retromarcia (docs/PIANO.md).
 *
 * I task portano il robot sull'asse della base, a AGG_CENTRO_INIZIO_M dalla sua faccia, di spalle alla base, poi
 * chiamano drv_aggancia(1). Da lì in poi il driver ignora le velocità dei task e guida da solo, con le sue protezioni
 * ancora attive (gd32/sicurezza.h). Niente eccezioni nella sicurezza dei task: la zona attorno alla base resta proibita
 * per loro, ci entra solo questo codice.
 *
 * Come funziona. I contatti sono dietro e i ricevitori infrarossi davanti, quindi in retromarcia l'infrarosso non
 * guida. Il lidar invece è dietro e vede bene la faccia della base (prova dock1 del 6/10: 28,4 cm di faccia visti
 * a 34 cm). A ogni giro:
 *   - si cerca la faccia: fra i punti dietro al robot, una sequenza di punti vicini, quasi in linea e larga come la base;
 *   - dalla retta si ricavano ANGOLO del robot rispetto alla faccia, DISTANZA e, se si vedono le due estremità,
 *     SCARTO LATERALE del robot dal centro della base.
 * Fra un giro e l'altro l'angolo si aggiorna con l'IMU: nella prova dock1 il robot è ruotato di 28° in 3 s in
 * retromarcia, con gli encoder che non segnavano niente (ed era ancora a 20 cm dalla base), e il comando non correggeva.
 *
 * Fasi: cerca la faccia -> arretra correggendo -> controllo al cancello (a 6 cm dalla base: se è storto non la tocca,
 * si ritira e riprova) -> ultimi centimetri piano, tenendo la direzione con l'IMU, perché sotto ~11 cm il lidar non
 * misura più -> contatto: fermo, aspetta la carica. Al massimo AGG_TENTATIVI tentativi, poi "fallito".
 *
 * È logica pura, senza Linux e senza stato globale: la provano i test con una registrazione vera e con un modello
 * del robot (test/test_driver_logica.c). Dentro il driver la protegge d.mx.
 */
#ifndef AGGANCIO_H
#define AGGANCIO_H

#include <stdint.h>

#include "../gd32/gd32.h"
#include "lidar_giro.h"

/* Il punto da cui partire: il centro del robot a questa distanza dalla faccia della base. Con la base all'origine della
 * mappa (robot agganciato in (0, 0), angolo 0, faccia a x = -ROBOT_RAGGIO) è il punto (AGG_PUNTO_X_M, 0) con angolo 0. */
#define AGG_PUNTO_X_M       0.30
#define AGG_CENTRO_INIZIO_M (AGG_PUNTO_X_M + 0.1745)
/* Tolleranza con cui i task devono arrivarci: oltre, la faccia può restare fuori dalla finestra di ricerca. */
#define AGG_TOLLERANZA_M    0.10
#define AGG_TOLLERANZA_RAD  0.26         /* 15° */

#define AGG_TENTATIVI       10

/* Esito, in drv_stato_t.aggancio */
enum { AGG_FERMO = 0, AGG_IN_CORSO, AGG_FATTO, AGG_FALLITO };

/* Fasi interne, in drv_stato_t.aggancio_fase (per i registri e le prove) */
typedef enum {
	AGG_F_NIENTE = 0,
	AGG_F_CERCA,      /* fermo: cerca la faccia nei giri del lidar */
	AGG_F_ARRETRA,    /* indietro correggendo con il lidar, fino al cancello */
	AGG_F_ULTIMI,     /* ultimi centimetri, piano, con la direzione tenuta dall'IMU */
	AGG_F_CONTATTO,   /* fermo sui contatti: aspetta la carica */
	AGG_F_RITIRA,     /* avanti fino al punto di partenza, per riprovare */
} agg_fase_t;

/* La faccia vista in un giro, nel sistema del robot */
typedef struct {
	int valida;
	double angolo;       /* radianti: + = robot ruotato in senso antiorario rispetto a "di spalle alla faccia" */
	double distanza;     /* metri, dal centro del robot alla retta della faccia */
	int ha_centro;       /* si vedono le due estremità, quindi lo scarto vale */
	double scarto;       /* metri: + = il centro della base è a sinistra (y positiva) dell'asse del robot */
	double larghezza;    /* metri */
} agg_faccia_t;

typedef struct {
	int esito;           /* AGG_FERMO, AGG_IN_CORSO, AGG_FATTO, AGG_FALLITO */
	agg_fase_t fase;
	int tentativo;       /* da 1 */
	const char *motivo;  /* perché è fallito o si è ritirato (testo fisso) */

	/* comando verso le ruote, m/s e rad/s */
	double v, w;

	/* stima */
	int ha_stima;
	double angolo, distanza, scarto;
	int ha_scarto;

	/* per l'angolo fra un giro e l'altro e la strada percorsa */
	int ha_prec;
	int16_t imbardata_prec;
	int32_t enc_sx_prec, enc_dx_prec;

	int64_t t_inizio, t_fase, t_ultima_faccia;
	int giri_visti;

	/* sui contatti */
	int frame_contatto, frame_totali;
	int64_t t_senza_contatto;
	double strada_ultimi;   /* strada percorsa negli ultimi centimetri */
} aggancio_t;

void aggancio_init(aggancio_t *a);

/* Parte da capo. Va chiamata con il robot sul punto, di spalle alla base. */
void aggancio_avvia(aggancio_t *a, int64_t t);

/* Si ferma subito, qualunque cosa stia facendo: esito AGG_FERMO. */
void aggancio_annulla(aggancio_t *a);

/* 1 se in questo momento il comando alle ruote lo decide l'aggancio. */
int aggancio_attivo(const aggancio_t *a);

/* 1 se serve il permesso di muoversi con i contatti chiusi (quando si ritira, per staccarsi dalla base). */
int aggancio_stacco(const aggancio_t *a);

/* Da chiamare a ogni giro completo del lidar. */
void aggancio_giro(aggancio_t *a, const lidar_giro_t *g, int64_t t);

/* Da chiamare a ogni frame dei sensori: aggiorna a->v, a->w. */
void aggancio_frame(aggancio_t *a, const gd32_sensori_t *s, int64_t t);

/* Cerca la faccia della base nel giro, scartando le rette più lontane di tolleranza dalla distanza attesa. */
agg_faccia_t aggancio_trova_faccia(const lidar_giro_t *g, double distanza_attesa, double tolleranza);

#endif /* AGGANCIO_H */
