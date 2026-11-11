/*
 * contacts.h - Registro dei contatti: gli scatti degli anti-caduta e dei bumper, e gli incastri, tenuti da parte per
 * i task.
 *
 * Il driver legge ogni frame dei sensori (50 Hz), ma un task lento (la mappatura, ogni 200 ms) può non vedere mai un
 * sensore che resta attivo per poco: il driver impedisce solo il moto in avanti, e una retromarcia veloce lo spegne
 * prima della lettura successiva. Qui il driver annota ogni scatto in una lista circolare con un numero progressivo;
 * chi legge applica solo gli eventi con numero maggiore dell'ultimo che ha già visto.
 *
 * Quando nasce un evento:
 *   - anti-caduta k (0 sx laterale, 1 sx anteriore, 2 dx anteriore, 3 dx laterale): "vuoto" per CONTACTS_VOID_FRAMES
 *     frame di fila (60 ms). Filtra i lampi dei falsi allarmi: gli anti-caduta sono a infrarossi e sui tappeti scuri
 *     possono vedere "vuoto" per un attimo;
 *   - bumper sinistro o destro: premuto per CONTACTS_BUMPER_FRAMES frame (1: è un contatto meccanico);
 *   - incastro in avanti: un contatto senza sensore (es. la torretta del lidar sotto un mobile basso, senza bumper).
 *     Lo riconosce il task di sicurezza e lo dice al driver a ogni frame con drv_stuck(); qui conta come attivo
 *     per CONTACTS_STUCK_FRAMES frame (1: il filtro, ~0,75 s di ruote ferme, lo fa già la sicurezza).
 * Un evento per scatto: il sensore deve tornare normale prima di poterne dare un altro. L'istante dell'evento è il
 * primo frame in cui il sensore è risultato attivo.
 *
 * Posizione: ogni evento porta l'ultima posizione del robot che il moto aveva dato al driver con drv_pose() nel primo
 * frame in cui il sensore è risultato attivo (di solito quella del frame prima: 20 ms, pochi millimetri), con il suo
 * istante. Così chi applica l'evento sa dove stava il
 * robot in quel momento, qualunque cosa abbia fatto dopo (si è fermato, è tornato indietro, ha girato).
 *
 * Robot inclinato (beccheggio o rollio oltre CONTACTS_TILT_CDEG, in piano sta entro ±1,5°): un "vuoto" degli
 * anti-caduta non dà eventi. Il robot sta superando qualcosa (il bordo di un tappeto, una soglia) e il vuoto è quasi
 * sempre il pavimento accanto a quello che sta superando: prova p2 del 7/11, sul bordo del tappeto davanti alla base
 * beccheggio -5,7° e rollio +6,5°, falso vuoto sul laterale destro, finito nella mappa come un gradino. Una scala vera gli
 * anti-caduta la vedono con il robot in piano (stanno davanti alle ruote). Il moto in avanti lo fermano comunque il
 * driver e il GD32: cambia solo il ricordo nella mappa.
 *
 * Quando NON nasce: anti-caduta, bumper e incastro non significano niente (e i contatori ripartono da zero) se i sensori sono
 * spenti (0x9d 00), se il robot è sulla base, o se tutti e quattro gli anti-caduta vedono "vuoto" (robot sollevato).
 * Limite noto: sollevando il robot inclinato, un anti-caduta può vedere "vuoto" per più di 60 ms prima degli altri
 * tre e dare un evento.
 *
 * Logica pura, senza Linux: si prova con gli unit test (test/test_driver_logica.c).
 */
#ifndef CONTACTS_H
#define CONTACTS_H

#include <stdint.h>
#include "../gd32/gd32.h"

#define CONTACTS_N            32     /* eventi tenuti nella lista (chi legge ogni 200 ms ne vede al massimo ~10) */
#define CONTACTS_VOID_FRAMES  3      /* frame di fila con "vuoto" per un evento di anti-caduta (60 ms) */
#define CONTACTS_TILT_CDEG 300  /* beccheggio o rollio oltre cui (centesimi di grado) un "vuoto" non dà eventi: vedi sotto */
#define CONTACTS_BUMPER_FRAMES 1      /* frame con il bumper premuto per un evento */
#define CONTACTS_STUCK_FRAMES 1    /* frame con l'incastro segnalato per un evento */

/* Quale sensore: 0..3 gli anti-caduta da sinistra a destra (come gd32_sensors_t.cliff), poi i due bumper, poi
 * l'incastro in avanti (non è un sensore: lo dà la sicurezza con drv_stuck). */
enum {
	CONTACT_VOID_LEFT_SIDE = 0, CONTACT_VOID_LEFT_FRONT = 1, CONTACT_VOID_RIGHT_FRONT = 2, CONTACT_VOID_RIGHT_SIDE = 3,
	CONTACT_BUMPER_LEFT = 4, CONTACT_BUMPER_RIGHT = 5, CONTACT_STUCK = 6,
	CONTACT_SENSORS = 7,
};

/* Posizione del robot data dal moto (drv_pose): metri e radianti nel sistema della mappa, come movement. */
typedef struct {
	double x, y, theta;
	int64_t t_ns;                /* quando il moto l'ha data (CLOCK_MONOTONIC, come rt_now) */
} drv_pose_t;

typedef struct {
	uint32_t number;             /* progressivo dell'evento (1, 2, 3...) */
	uint8_t sensor;             /* CONTATTO_* */
	int64_t t_ns;                /* primo frame in cui il sensore è risultato attivo (CLOCK_MONOTONIC, come rt_now) */
	int pose_valid;             /* 0 = il moto non aveva ancora mai dato una posizione: x, y, theta non valgono */
	drv_pose_t pose;             /* ultima posizione data dal moto al primo frame attivo (guarda posa.t_ns: se è vecchia,
	                                il moto ha smesso di darla, per esempio con il robot sollevato) */
} drv_contact_t;

typedef struct {
	uint32_t last;             /* numero dell'ultimo evento (0 = nessuno ancora) */
	drv_contact_t events[CONTACTS_N];   /* l'evento numero k sta in ev[k % CONTACTS_N]; validi gli ultimi CONTACTS_N */
} drv_contacts_t;

/* Stato interno del registro (lo tiene il driver). */
typedef struct {
	int count[CONTACT_SENSORS];         /* frame di fila con il sensore attivo */
	int64_t t_start[CONTACT_SENSORS];  /* primo di quei frame */
	int has_pose[CONTACT_SENSORS];       /* c'era una posizione del moto al primo frame */
	drv_pose_t start_pose[CONTACT_SENSORS];   /* la posizione del moto al primo frame */
	int reported[CONTACT_SENSORS];       /* evento già dato per questo scatto */
	drv_contacts_t published;                  /* la lista da pubblicare */
} contacts_t;

void contacts_init(contacts_t *c);

/* Un frame dei sensori arrivato all'istante t; posa = ultima posizione data dal moto, NULL se non ce n'è ancora una;
 * incastro = 1 se la sicurezza dice che il robot è incastrato andando avanti (drv_stuck).
 * Ritorna 1 se ha aggiunto almeno un evento (c->published è cambiata). */
int contacts_frame(contacts_t *c, const gd32_sensors_t *s, int64_t t, const drv_pose_t *pose, int stuck);

#endif /* CONTACTS_H */
