/*
 * mondo.h - La stanza del simulatore: un elenco di oggetti con forma, posizione, rotazione e quote.
 *
 * Coordinate: metri, x verso destra, y verso l'alto (vista dall'alto), angoli in gradi nel file e in radianti nel
 * codice, in senso antiorario. Ogni oggetto ha due QUOTE (altezza dal pavimento, da / a), che decidono come interagisce
 * con il robot (sim/robot.h):
 *   - tocca il CORPO (0 .. ROBOT_ALTEZZA_CORPO): ferma il robot e preme il bumper se è davanti;
 *   - tocca solo la TORRETTA del lidar (ROBOT_ALTEZZA_CORPO .. ROBOT_ALTEZZA): ferma il robot SENZA bumper (sul robot
 *     vero la torretta non ha interruttori: il robot si incastra sotto un mobile basso);
 *   - comprende il PIANO DEL LIDAR (LIDAR_ALTEZZA): il lidar lo vede;
 *   - sta tutto sopra il robot: il robot ci passa sotto. Sotto 2 cm (soglie, tappeti) il robot ci passa sopra.
 * Il VUOTO (scale, gradini) è un poligono sul pavimento: lo vedono gli anti-caduta e, se il centro del robot ci finisce
 * sopra, il robot cade. Fuori dal rettangolo della stanza vale "muro".
 *
 * Formato del file (una riga per oggetto, "chiave=valore", # commento):
 *     stanza nome=soggiorno larghezza=6.0 profondita=3.9
 *     robot x=0.8 y=2.5 angolo=-90                        posizione iniziale
 *     base x=0.6 y=0.145 angolo=90                        centro del lato con i contatti, verso dove guarda
 *     muro da=0,0 a=6,0 spessore=0.10 [quota_a=2.5]
 *     rettangolo nome=divano x=1.1 y=3.4 larghezza=2.2 profondita=0.9 angolo=0 quota_da=0 quota_a=0.45 [gruppo=g]
 *     cerchio nome=gamba x=3.45 y=1.65 raggio=0.025 quota_da=0 quota_a=0.72 [gruppo=tavolo]
 *     vuoto nome=scala punti=5,0 6,0 6,0.9 5,0.9
 * x, y di rettangoli e cerchi sono il CENTRO; larghezza va lungo l'angolo, profondità di traverso.
 */
#ifndef MONDO_H
#define MONDO_H

#include <stddef.h>

#define MONDO_MAX_OGGETTI 2000
#define MONDO_MAX_PUNTI   32
#define MONDO_NOME        32

typedef enum { OGG_RETTANGOLO = 0, OGG_CERCHIO, OGG_VUOTO } ogg_forma_t;

typedef struct {
	ogg_forma_t forma;
	int muro;                     /* rettangolo nato da "muro da= a=" (si riscrive così) */
	char nome[MONDO_NOME], gruppo[MONDO_NOME];
	double x, y, angolo;          /* centro, rotazione (radianti) */
	double larghezza, profondita; /* rettangolo; per il muro: lunghezza e spessore */
	double raggio;                /* cerchio */
	double quota_da, quota_a;     /* metri dal pavimento */
	int n_punti;                  /* vuoto: poligono in coordinate della stanza */
	double px[MONDO_MAX_PUNTI], py[MONDO_MAX_PUNTI];
} oggetto_t;

typedef struct {
	char nome[64];
	double larghezza, profondita;           /* rettangolo della stanza, da (0,0) */
	double robot_x, robot_y, robot_dir;     /* posizione iniziale (radianti) */
	int ha_base;
	double base_x, base_y, base_dir;
	int n;
	oggetto_t o[MONDO_MAX_OGGETTI];
} mondo_t;

/* Stanza vuota con i quattro muri. */
void mondo_nuovo(mondo_t *m, double larghezza, double profondita);

/* Lettura e scrittura del formato di testo. Ritorna 0, oppure -1 con il motivo in errore. */
int mondo_leggi_testo(mondo_t *m, const char *testo, char *errore, size_t dim);
int mondo_leggi_file(mondo_t *m, const char *percorso, char *errore, size_t dim);
/* Scrive la stanza in un buffer allocato con malloc (da liberare), o NULL. */
char *mondo_testo(const mondo_t *m);

/* 1 se il punto è dentro il blocco della base. */
int mondo_in_base(const mondo_t *m, double x, double y);

/* 1 se il punto è sopra un vuoto. */
int mondo_vuoto(const mondo_t *m, double x, double y);

/*
 * Distanza dal punto (x, y) al primo oggetto che taglia il piano all'altezza `quota`, lungo la direzione `ang`
 * (radianti), fino a max_m. Base compresa. Ritorna max_m + 1 se non c'è niente.
 */
double mondo_raggio(const mondo_t *m, double x, double y, double ang, double quota, double max_m);

/*
 * Un cilindro verticale (centro x, y, raggio r, da quota z0 a z1) tocca qualcosa? Se sì ritorna 1 e mette in
 * *ang_contatto la direzione (assoluta, radianti) dal centro verso il contatto. Conta gli oggetti con quote che si
 * sovrappongono a z0..z1 e alti almeno 2 cm (soglie più basse si superano), la base, e i bordi della stanza.
 */
int mondo_urto(const mondo_t *m, double x, double y, double r, double z0, double z1, double *ang_contatto);

/* Un oggetto in coordinate del mondo: 1 se il punto (x, y) è dentro la sua impronta (per la selezione). */
int mondo_dentro(const oggetto_t *o, double x, double y);

#endif /* MONDO_H */
