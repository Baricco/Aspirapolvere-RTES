/*
 * robot.h - Dati fisici del Proscenic M6 Pro (3irobotix CRL-200S) usati dal simulatore, con la fonte di ognuno.
 *
 * Fonti:
 *   [firmware] letto nel disassemblato di everest-server o nei file di /mnt/UDISK/config (docs/hardware.md)
 *   [scheda]   schede tecniche e recensioni del modello (diametro 13,75", altezza 3,75", 3,4 kg, 14,8 V 3200 mAh)
 *   [misura]   misurato sul robot nelle fasi 1-3 (docs/fase2.md, docs/fase3.md)
 *   [utente]   detto dall'utente guardando il robot
 *   [?]        stima: da misurare quando il robot è disponibile (elenco in docs/fase3.md)
 *
 * Sistema del robot: x in avanti, y a sinistra, angoli in senso antiorario visti dall'alto, origine al centro.
 * Le parti dinamiche (risposta dei motori, rumori, ritardi) sono invece in sim/parametri.txt, ricavate dalle
 * registrazioni con tools/taratura/.
 */
#ifndef ROBOT_H
#define ROBOT_H

#include <math.h>

/* forma */
#define ROBOT_RAGGIO          0.1745     /* [scheda] diametro 349 mm */
#define ROBOT_ALTEZZA         0.095      /* [scheda] con la torretta del lidar */
#define ROBOT_ALTEZZA_CORPO   0.075      /* [?] sopra c'è solo la torretta del lidar, che NON ha interruttori [misura] */
#define TORRETTA_RAGGIO       0.050      /* [?] raggio della torretta del lidar */
#define LIDAR_ALTEZZA         0.087      /* [?] altezza del piano di misura del lidar */
#define LIDAR_X               0.0        /* [?] posizione del lidar nel robot */
#define LIDAR_Y               0.0

/* ruote e odometria */
#define RUOTA_RAGGIO          0.03475    /* [firmware] */
#define CARREGGIATA           0.2415     /* [firmware] distanza fra le ruote */
#define TICK_GIRO             3616       /* [firmware] + [misura] */
#define SCALA_V               16561.27   /* [firmware] 0x66: tick/s per m/s */
#define SCALA_W               1999.77    /* [firmware] 0x66: tick/s per rad/s */
#define V_MAX                 0.5        /* [firmware] limiti di everest */
#define W_MAX                 (2 * M_PI)

/* bumper: arco anteriore, due contatti (sinistro, destro) [misura]; everest segna l'ostacolo 2 cm oltre il contorno
 * [firmware: CObstacleGeneration, scostamento 0,02 m] e divide il bumper in settori di 15° [firmware: hwdrivers.ini] */
#define BUMPER_MEZZO_ARCO     (90 * M_PI / 180)   /* [?] da -90° a +90° */
#define BUMPER_CENTRO         (25 * M_PI / 180)   /* [?] entro +-25° si premono tutti e due */

/* anti-caduta: 4 sensori sul davanti, da sinistra a destra = bit 0..3 del byte 3 [misura, fase 2 D3];
 * portata 2-15 cm [firmware: hwdrivers.ini cliff_ir_min/max_range] */
#define CLIFF_DISTANZA        0.150      /* [?] distanza dal centro */
static const double CLIFF_ANGOLI[4] = {  /* [?] angoli rispetto al davanti */
	50 * M_PI / 180, 18 * M_PI / 180, -18 * M_PI / 180, -50 * M_PI / 180,
};

/* base: i contatti di carica sono DIETRO [utente]; i ricevitori IR della base sono davanti [misura, fase 2 D2]:
 * il robot trova la base guardandola, si gira ed entra all'indietro. Per uscire va avanti. */
#define CONTATTI_DIETRO       1
#define BASE_LARGHEZZA        0.30       /* [?] */
#define BASE_PROFONDITA       0.12       /* [?] */

/* batteria */
#define BATTERIA_AH           3.2        /* [scheda] Li-ion 4S, 14,8 V nominali */

#endif /* ROBOT_H */
