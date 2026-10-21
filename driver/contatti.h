/*
 * contatti.h - Registro dei contatti: gli scatti degli anti-caduta e dei bumper, e gli incastri, tenuti da parte per
 * i task.
 *
 * Il driver legge ogni frame dei sensori (50 Hz), ma un task lento (la mappatura, ogni 200 ms) può non vedere mai un
 * sensore che resta attivo per poco: il driver impedisce solo il moto in avanti, e una retromarcia veloce lo spegne
 * prima della lettura successiva. Qui il driver annota ogni scatto in una lista circolare con un numero progressivo;
 * chi legge applica solo gli eventi con numero maggiore dell'ultimo che ha già visto.
 *
 * Quando nasce un evento:
 *   - anti-caduta k (0 sx laterale, 1 sx anteriore, 2 dx anteriore, 3 dx laterale): "vuoto" per CONTATTI_FRAME_VUOTO
 *     frame di fila (60 ms). Filtra i lampi dei falsi allarmi: gli anti-caduta sono a infrarossi e sui tappeti scuri
 *     possono vedere "vuoto" per un attimo;
 *   - bumper sinistro o destro: premuto per CONTATTI_FRAME_BUMPER frame (1: è un contatto meccanico);
 *   - incastro in avanti: un contatto senza sensore (es. la torretta del lidar sotto un mobile basso, senza bumper).
 *     Lo riconosce il task di sicurezza e lo dice al driver a ogni frame con drv_incastro(); qui conta come attivo
 *     per CONTATTI_FRAME_INCASTRO frame (1: il filtro, ~0,75 s di ruote ferme, lo fa già la sicurezza).
 * Un evento per scatto: il sensore deve tornare normale prima di poterne dare un altro. L'istante dell'evento è il
 * primo frame in cui il sensore è risultato attivo.
 *
 * Posizione: ogni evento porta l'ultima posizione del robot che il moto aveva dato al driver con drv_posa() nel primo
 * frame in cui il sensore è risultato attivo (di solito quella del frame prima: 20 ms, pochi millimetri), con il suo
 * istante. Così chi applica l'evento sa dove stava il
 * robot in quel momento, qualunque cosa abbia fatto dopo (si è fermato, è tornato indietro, ha girato).
 *
 * Quando NON nasce: anti-caduta, bumper e incastro non significano niente (e i contatori ripartono da zero) se i sensori sono
 * spenti (0x9d 00), se il robot è sulla base, o se tutti e quattro gli anti-caduta vedono "vuoto" (robot sollevato).
 * Limite noto: sollevando il robot inclinato, un anti-caduta può vedere "vuoto" per più di 60 ms prima degli altri
 * tre e dare un evento.
 *
 * Logica pura, senza Linux: si prova con gli unit test (test/test_driver_logica.c).
 */
#ifndef CONTATTI_H
#define CONTATTI_H

#include <stdint.h>
#include "../gd32/gd32.h"

#define CONTATTI_N            32     /* eventi tenuti nella lista (chi legge ogni 200 ms ne vede al massimo ~10) */
#define CONTATTI_FRAME_VUOTO  3      /* frame di fila con "vuoto" per un evento di anti-caduta (60 ms) */
#define CONTATTI_FRAME_BUMPER 1      /* frame con il bumper premuto per un evento */
#define CONTATTI_FRAME_INCASTRO 1    /* frame con l'incastro segnalato per un evento */

/* Quale sensore: 0..3 gli anti-caduta da sinistra a destra (come gd32_sensori_t.cliff), poi i due bumper, poi
 * l'incastro in avanti (non è un sensore: lo dà la sicurezza con drv_incastro). */
enum {
	CONTATTO_VUOTO_SX_LAT = 0, CONTATTO_VUOTO_SX_ANT = 1, CONTATTO_VUOTO_DX_ANT = 2, CONTATTO_VUOTO_DX_LAT = 3,
	CONTATTO_BUMPER_SX = 4, CONTATTO_BUMPER_DX = 5, CONTATTO_INCASTRO = 6,
	CONTATTO_SENSORI = 7,
};

/* Posizione del robot data dal moto (drv_posa): metri e radianti nel sistema della mappa, come movement. */
typedef struct {
	double x, y, theta;
	int64_t t_ns;                /* quando il moto l'ha data (CLOCK_MONOTONIC, come rt_adesso) */
} drv_posa_t;

typedef struct {
	uint32_t numero;             /* progressivo dell'evento (1, 2, 3...) */
	uint8_t sensore;             /* CONTATTO_* */
	int64_t t_ns;                /* primo frame in cui il sensore è risultato attivo (CLOCK_MONOTONIC, come rt_adesso) */
	int posa_valida;             /* 0 = il moto non aveva ancora mai dato una posizione: x, y, theta non valgono */
	drv_posa_t posa;             /* ultima posizione data dal moto al primo frame attivo (guarda posa.t_ns: se è vecchia,
	                                il moto ha smesso di darla, per esempio con il robot sollevato) */
} drv_contatto_t;

typedef struct {
	uint32_t ultimo;             /* numero dell'ultimo evento (0 = nessuno ancora) */
	drv_contatto_t ev[CONTATTI_N];   /* l'evento numero k sta in ev[k % CONTATTI_N]; validi gli ultimi CONTATTI_N */
} drv_contatti_t;

/* Stato interno del registro (lo tiene il driver). */
typedef struct {
	int conta[CONTATTO_SENSORI];         /* frame di fila con il sensore attivo */
	int64_t t_inizio[CONTATTO_SENSORI];  /* primo di quei frame */
	int ha_posa[CONTATTO_SENSORI];       /* c'era una posizione del moto al primo frame */
	drv_posa_t posa_inizio[CONTATTO_SENSORI];   /* la posizione del moto al primo frame */
	int segnato[CONTATTO_SENSORI];       /* evento già dato per questo scatto */
	drv_contatti_t pub;                  /* la lista da pubblicare */
} contatti_t;

void contatti_init(contatti_t *c);

/* Un frame dei sensori arrivato all'istante t; posa = ultima posizione data dal moto, NULL se non ce n'è ancora una;
 * incastro = 1 se la sicurezza dice che il robot è incastrato andando avanti (drv_incastro).
 * Ritorna 1 se ha aggiunto almeno un evento (c->pub è cambiata). */
int contatti_frame(contatti_t *c, const gd32_sensori_t *s, int64_t t, const drv_posa_t *posa, int incastro);

#endif /* CONTATTI_H */
