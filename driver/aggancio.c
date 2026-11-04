/* aggancio.c - Messa in carica in retromarcia, guidata dal lidar e dall'IMU (vedi aggancio.h). */
#include <math.h>
#include <string.h>

#include "aggancio.h"

#define RAGGIO_ROBOT      0.1745

/* Velocità. Lente: la base non va spostata. */
#define V_ARRETRA         0.05       /* m/s */
#define V_ULTIMI          0.03
#define V_RITIRA          0.08
#define W_MAX             0.40       /* rad/s */

/* Controllo: la direzione voluta dipende dallo scarto (un metro di scarto, K_SCARTO radianti, con un massimo), la
 * rotazione dalla differenza fra direzione voluta e attuale. Con 8 rad/m lo scarto si dimezza ogni ~9 cm di strada. */
#define K_SCARTO          8.0        /* rad/m */
#define ANGOLO_MAX        0.35       /* 20°: direzione massima voluta */
#define ANGOLO_MAX_VICINO 0.09       /* 5°: da vicino non si fanno più manovre larghe */
#define K_ANGOLO          2.0        /* 1/s */
#define SOLO_ROTAZIONE    0.35       /* differenza oltre la quale il robot gira soltanto (20°) */

/* Distanze del centro del robot dalla faccia. */
#define DIST_VICINO       0.30       /* da qui ANGOLO_MAX_VICINO */
#define DIST_CANCELLO     0.235      /* 6 cm di spazio dietro: l'ultimo controllo, il lidar misura ancora (faccia a 12 cm) */

/* Controllo al cancello: tanto storto o spostato, non si tocca la base. */
#define CANCELLO_SCARTO   0.020
#define CANCELLO_ANGOLO   0.087      /* 5° */

/* Ultimi centimetri: non si va oltre questa strada, altrimenti si sposta la base. */
#define STRADA_ULTIMI_EXTRA 0.020

/* Sui contatti: dopo CONTATTO_ATTESA senza carica, la carica "non serve" se il contatto è rimasto chiuso (batteria piena). */
#define CONTATTO_ATTESA_NS  (15000 * 1000000LL)
#define CONTATTO_PERSO_NS   (1500 * 1000000LL)
#define CONTATTO_QUOTA      0.8      /* bit "contatto" acceso almeno in questa frazione di frame: è intermittente */

/* Tempi massimi */
#define CERCA_MAX_NS      (3000 * 1000000LL)
#define ARRETRA_MAX_NS    (20000 * 1000000LL)
#define RITIRA_MAX_NS     (8000 * 1000000LL)
#define FACCIA_PERSA_NS   (1200 * 1000000LL)

/* Ricerca della faccia. Larghezza vista nella prova dock1: 28,4 cm. */
#define FIN_X_MIN        -0.75
#define FIN_X_MAX        -0.18
#define FIN_Y             0.40
#define SALTO_MAX         0.035      /* due punti consecutivi più lontani di così non sono la stessa superficie */
#define PUNTI_MIN         8
#define LARG_MIN          0.18       /* sotto questa la retta non basta nemmeno per l'angolo */
#define LARG_CENTRO_MIN   0.23       /* con almeno questa si vedono le due estremità */
#define LARG_CENTRO_MAX   0.37
#define RESIDUO_MAX       0.012      /* scarto quadratico dalla retta: la faccia è piatta, il rumore è 2-7 mm */
#define ATTESA_DIST_MAX   0.06       /* con una stima: quanto la distanza può differire da quella stimata */

/* Scala dell'IMU: conta circa il 3% in meno (la stessa taratura del task del moto). */
#define SCALA_IMBARDATA   1.0285

static double normale(double a)
{
	while (a > M_PI) a -= 2 * M_PI;
	while (a < -M_PI) a += 2 * M_PI;
	return a;
}

static double limita(double x, double m) { return x > m ? m : x < -m ? -m : x; }

void aggancio_init(aggancio_t *a)
{
	memset(a, 0, sizeof *a);
}

static void passa_a(aggancio_t *a, agg_fase_t f, int64_t t)
{
	a->fase = f;
	a->t_fase = t;
	a->v = a->w = 0;
	a->frame_contatto = a->frame_totali = 0;
	a->t_senza_contatto = 0;
	a->strada_ultimi = 0;
}

void aggancio_avvia(aggancio_t *a, int64_t t)
{
	aggancio_init(a);
	a->esito = AGG_IN_CORSO;
	a->tentativo = 1;
	a->t_inizio = t;
	passa_a(a, AGG_F_CERCA, t);
}

void aggancio_annulla(aggancio_t *a)
{
	aggancio_init(a);
}

int aggancio_attivo(const aggancio_t *a) { return a->esito == AGG_IN_CORSO; }
int aggancio_stacco(const aggancio_t *a) { return a->esito == AGG_IN_CORSO && a->fase == AGG_F_RITIRA; }

static void fallisce(aggancio_t *a, const char *motivo)
{
	a->esito = AGG_FALLITO;
	a->motivo = motivo;
	a->fase = AGG_F_NIENTE;
	a->v = a->w = 0;
}

/* Lascia perdere questo tentativo: indietro non si va più, si torna al punto di partenza. */
static void ritira(aggancio_t *a, const char *motivo, int64_t t)
{
	a->motivo = motivo;
	passa_a(a, AGG_F_RITIRA, t);
}

/* ------------------------------------------------------------------------------------- ricerca della faccia --- */

/* Retta x = q + m y sui punti da i a j compresi (ai minimi quadrati): con la faccia di traverso x dipende poco da y. */
static void retta(const double *x, const double *y, int i, int j, double *q, double *m)
{
	double sx = 0, sy = 0, syy = 0, sxy = 0;
	int n = j - i + 1;
	for (int k = i; k <= j; k++) {
		sx += x[k];
		sy += y[k];
		syy += y[k] * y[k];
		sxy += x[k] * y[k];
	}
	double den = n * syy - sy * sy;
	*m = den > 1e-9 ? (n * sxy - sx * sy) / den : 0;
	*q = (sx - *m * sy) / n;
}

agg_faccia_t aggancio_trova_faccia(const lidar_giro_t *g, double distanza_attesa, double tolleranza)
{
	agg_faccia_t migliore = { 0 };
	double miglior_punteggio = 1e9;

	/* i punti della finestra, nell'ordine del giro: la faccia è dietro, attorno all'angolo 0 del lidar, e il giro può
	 * cominciare in un punto qualsiasi (anche in mezzo alla faccia), quindi si parte dal punto che passa per 180°, davanti */
	static double x[LIDAR_GIRO_MAX_PUNTI], y[LIDAR_GIRO_MAX_PUNTI];   /* una sola chiamata alla volta: sotto d.mx */
	int n = 0, inizio = 0;
	for (int k = 0; k < g->n; k++)
		if (g->p[k].angolo_cdeg >= 18000 && g->p[(k + g->n - 1) % g->n].angolo_cdeg < 18000) {
			inizio = k;
			break;
		}
	for (int k = 0; k < g->n; k++) {
		const lidar_punto_t *p = &g->p[(inizio + k) % g->n];
		if (!p->distanza_mm)
			continue;
		double px, py;
		lidar_punto_xy(p, &px, &py);
		if (px < FIN_X_MIN || px > FIN_X_MAX || fabs(py) > FIN_Y)
			continue;
		x[n] = px;
		y[n] = py;
		n++;
	}

	/* si taglia in tratti dove due punti consecutivi sono lontani */
	for (int i = 0; i < n;) {
		int j = i;
		while (j + 1 < n && hypot(x[j + 1] - x[j], y[j + 1] - y[j]) <= SALTO_MAX)
			j++;
		int fine_tratto = j;
		/* se il tratto contiene anche il fianco della base, si tolgono punti dalle estremità finché è una retta */
		int a = i, b = j, retta_ok = 0;
		double q = 0, m = 0;
		while (b - a + 1 >= PUNTI_MIN) {
			retta(x, y, a, b, &q, &m);
			double ra = fabs(x[a] - (q + m * y[a])), rb = fabs(x[b] - (q + m * y[b])), somma = 0;
			for (int k = a; k <= b; k++) {
				double r = x[k] - (q + m * y[k]);
				somma += r * r;
			}
			if (somma / (b - a + 1) <= RESIDUO_MAX * RESIDUO_MAX) {
				retta_ok = 1;
				break;
			}
			if (ra > rb)
				a++;
			else
				b--;
		}
		if (retta_ok) {
			double larg = hypot(x[b] - x[a], y[b] - y[a]);
			double ang = atan(m), c = cos(ang), s = sin(ang);
			double dist = -q * c;      /* distanza dell'origine dalla retta x = q + m y */
			if (larg >= LARG_MIN && larg <= LARG_CENTRO_MAX && fabs(dist - distanza_attesa) <= tolleranza) {
				/* punteggio: più è vicina alla larghezza della base e alla distanza attesa, meglio è */
				double punti = fabs(larg - 0.30) + fabs(dist - distanza_attesa);
				if (punti < miglior_punteggio) {
					miglior_punteggio = punti;
					migliore.valida = 1;
					migliore.angolo = ang;
					migliore.distanza = dist;
					migliore.larghezza = larg;
					migliore.ha_centro = larg >= LARG_CENTRO_MIN;
					/* il centro della faccia, riportato al sistema "di spalle alla faccia" */
					double cx = (x[a] + x[b]) / 2, cy = (y[a] + y[b]) / 2;
					migliore.scarto = cx * s + cy * c;
				}
			}
		}
		i = fine_tratto + 1;
	}
	return migliore;
}

void aggancio_giro(aggancio_t *a, const lidar_giro_t *g, int64_t t)
{
	if (a->esito != AGG_IN_CORSO)
		return;
	/* negli ultimi centimetri il lidar vede solo i pezzi laterali della faccia (sotto ~11 cm non misura) e la retta è
	 * sbagliata di qualche grado: conta solo l'IMU */
	if (a->fase == AGG_F_ULTIMI || a->fase == AGG_F_CONTATTO)
		return;
	agg_faccia_t f = a->ha_stima ? aggancio_trova_faccia(g, a->distanza, ATTESA_DIST_MAX)
	                             : aggancio_trova_faccia(g, AGG_CENTRO_INIZIO_M, AGG_TOLLERANZA_M);
	/* senza una stima, la prima volta vale solo una faccia intera */
	if (!f.valida || (!a->ha_stima && !f.ha_centro))
		return;
	a->t_ultima_faccia = t;
	a->giri_visti++;
	a->angolo = f.angolo;
	a->distanza = f.distanza;
	if (f.ha_centro) {
		a->scarto = a->ha_scarto ? 0.5 * a->scarto + 0.5 * f.scarto : f.scarto;   /* le estremità ballano di mezzo cm */
		a->ha_scarto = 1;
	}
	a->ha_stima = 1;
}

/* ------------------------------------------------------------------------------------------------ guida --- */

void aggancio_frame(aggancio_t *a, const gd32_sensori_t *s, int64_t t)
{
	/* quanto si è mosso dall'ultimo frame: angolo dall'IMU, strada dagli encoder (avanti = positiva) */
	double strada = 0;
	if (a->ha_prec) {
		double dimb = (int16_t)(s->angolo[0] - a->imbardata_prec) * (M_PI / 18000) * SCALA_IMBARDATA;
		a->angolo = normale(a->angolo + dimb);
		strada = (((int32_t)((uint32_t)s->encoder_sx - (uint32_t)a->enc_sx_prec) +
		           (int32_t)((uint32_t)s->encoder_dx - (uint32_t)a->enc_dx_prec)) / 2.0) / GD32_SCALA_V;
		a->distanza += strada * cos(a->angolo);        /* avanti: ci si allontana dalla faccia */
		a->scarto -= strada * sin(a->angolo);          /* lo scarto cambia finché non arriva il giro successivo */
	}
	a->imbardata_prec = s->angolo[0];
	a->enc_sx_prec = s->encoder_sx;
	a->enc_dx_prec = s->encoder_dx;
	a->ha_prec = 1;

	if (a->esito != AGG_IN_CORSO) {
		a->v = a->w = 0;
		return;
	}

	switch (a->fase) {
	case AGG_F_CERCA:
		a->v = a->w = 0;
		if (a->giri_visti >= 2) {
			passa_a(a, AGG_F_ARRETRA, t);
			a->t_ultima_faccia = t;
		} else if (t - a->t_fase > CERCA_MAX_NS) {
			if (a->tentativo >= AGG_TENTATIVI) {
				fallisce(a, "la faccia della base non si vede");
			} else {
				a->tentativo++;
				a->t_fase = t;
			}
		}
		break;

	case AGG_F_ARRETRA: {
		if (s->sulla_base) {                           /* già toccata: va bene così, si controlla la carica */
			passa_a(a, AGG_F_CONTATTO, t);
			break;
		}
		if (t - a->t_ultima_faccia > FACCIA_PERSA_NS) {
			ritira(a, "faccia della base persa", t);
			break;
		}
		if (t - a->t_fase > ARRETRA_MAX_NS) {
			ritira(a, "retromarcia troppo lunga", t);
			break;
		}
		double massimo = a->distanza < DIST_VICINO ? ANGOLO_MAX_VICINO : ANGOLO_MAX;
		double voluto = a->ha_scarto ? limita(-K_SCARTO * a->scarto, massimo) : 0;
		double diff = normale(voluto - a->angolo);
		if (a->distanza <= DIST_CANCELLO) {
			/* non si tocca la base se è storto: se è spostato si ritira, se è solo inclinato si raddrizza sul posto */
			if (fabs(a->scarto) > CANCELLO_SCARTO) {
				ritira(a, "al cancello: troppo spostato", t);
				break;
			}
			if (fabs(a->angolo) > CANCELLO_ANGOLO) {
				a->v = 0;
				a->w = limita(-K_ANGOLO * a->angolo, W_MAX);   /* girare sul posto è sicuro: il robot è rotondo */
				break;
			}
			passa_a(a, AGG_F_ULTIMI, t);
			break;
		}
		/* in retromarcia, con la base a sinistra (scarto > 0), serve un angolo negativo */
		a->v = fabs(diff) > SOLO_ROTAZIONE ? 0 : -V_ARRETRA;
		a->w = limita(K_ANGOLO * diff, W_MAX);
		break;
	}

	case AGG_F_ULTIMI: {
		if (s->sulla_base) {
			passa_a(a, AGG_F_CONTATTO, t);
			break;
		}
		a->strada_ultimi += -strada;
		if (a->strada_ultimi > (DIST_CANCELLO - RAGGIO_ROBOT) + STRADA_ULTIMI_EXTRA) {
			ritira(a, "nessun contatto alla distanza attesa", t);
			break;
		}
		if (t - a->t_fase > ARRETRA_MAX_NS) {
			ritira(a, "ultimi centimetri troppo lunghi", t);
			break;
		}
		a->v = -V_ULTIMI;
		a->w = limita(-K_ANGOLO * a->angolo, W_MAX);
		break;
	}

	case AGG_F_CONTATTO:
		a->v = a->w = 0;
		if (s->in_carica) {
			a->esito = AGG_FATTO;
			a->fase = AGG_F_NIENTE;
			break;
		}
		a->frame_totali++;
		if (s->sulla_base) {
			a->frame_contatto++;
			a->t_senza_contatto = 0;
		} else {
			if (!a->t_senza_contatto)
				a->t_senza_contatto = t;
			if (t - a->t_senza_contatto > CONTATTO_PERSO_NS) {
				ritira(a, "contatto perso", t);
				break;
			}
		}
		if (t - a->t_fase > CONTATTO_ATTESA_NS) {
			if (a->frame_contatto >= CONTATTO_QUOTA * a->frame_totali) {
				a->esito = AGG_FATTO;                  /* contatto stabile senza carica: batteria già piena */
				a->fase = AGG_F_NIENTE;
			} else {
				ritira(a, "contatto instabile", t);
			}
		}
		break;

	case AGG_F_RITIRA:
		if (a->distanza >= AGG_CENTRO_INIZIO_M || t - a->t_fase > RITIRA_MAX_NS) {
			if (a->tentativo >= AGG_TENTATIVI) {
				fallisce(a, a->motivo ? a->motivo : "troppi tentativi");
				break;
			}
			a->tentativo++;
			a->giri_visti = 0;
			a->ha_stima = 0;
			a->ha_scarto = 0;
			passa_a(a, AGG_F_CERCA, t);
			break;
		}
		a->v = V_RITIRA;
		a->w = limita(-K_ANGOLO * a->angolo, W_MAX);
		break;

	default:
		a->v = a->w = 0;
		break;
	}

	if (a->esito == AGG_IN_CORSO && t - a->t_inizio > AGG_TENTATIVI * (CERCA_MAX_NS + ARRETRA_MAX_NS + CONTATTO_ATTESA_NS + RITIRA_MAX_NS))
		fallisce(a, "tempo massimo");
}
