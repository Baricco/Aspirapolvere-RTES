/* aggancio.c - Messa in carica guidata dagli infrarossi della base, con lo stesso approccio del firmware originale (vedi aggancio.h). */
#include <math.h>
#include <string.h>

#include "aggancio.h"

/* Ricerca: ruota su se stesso. Dal segnale trovato al segnale perso si torna indietro di questa frazione dell'arco
 * (CActionTransferMotionGoDock::calculateMiddlePhi: 0,4, non 0,5, perché il robot si ferma dopo aver perso il segnale). */
#define W_CERCA           0.75       /* rad/s: rapporto 1,5 x massimo 0,5 dell'originale */
#define W_CENTRA          0.50
#define FRAZIONE_ARCO     0.4
#define FRAME_PERSO_CERCA 5          /* il segnale è intermittente: perso vuol dire assente per 5 frame di fila */

/* Avvicinamento di fronte. Le velocità in rotazione sono quelle dell'originale (CActionDockAlignState). */
#define V_LONTANO         0.15       /* m/s */
#define V_FASCIO_CORTO    0.05       /* DIVERSO dall'originale: vedendo il fascio corto si è già vicini anche se il lidar non vede
                                        la base (nera): senza questo limite arriverebbe contro la base a 15 cm/s */
#define V_VICINO          0.025      /* lidar a meno di 0,5 m dalla faccia */
#define DIST_VICINO       0.50
#define W_LONTANO         0.50       /* rad/s */
#define W_VICINO          0.25       /* lidar a meno di 0,55 m */
#define DIST_W_VICINO     0.55
#define W_PERSO           0.35       /* appena perso il segnale: torna dalla parte da cui l'ha perso */
#define FRAME_PERSO       51         /* ~1 s senza segnale: ricomincia a cercare */

/* Lidar: angolo 180° = davanti. Spazio davanti = distanza - profilo del corpo (dal lidar al muso: 0,28 m nella tabella
 * dell'originale). Si guarda il settore 170..190°, l'originale chiede 10 punti: la base nera ne dà pochi. */
#define SETTORE_DA_CDEG   17000
#define SETTORE_A_CDEG    19000
#define PROFILO_MUSO      0.28
#define SPAZIO_ARRIVO     0.06       /* m: il muso è a 6 cm dalla base */
#define PUNTI_ARRIVO      5
#define FRAME_ARRIVO      8
#define DIST_MIN          0.30       /* come CDockMiddlePoint::getAlignDistance: la media vale fra 0,3 e 1,5 m */
#define DIST_MAX          1.50
#define GIRO_VECCHIO_NS   (500 * 1000000LL)
#define SEGNALE_RECENTE_NS (1000 * 1000000LL)

/* Rifinitura a pochi centimetri: oscilla piano e prende la direzione a metà dei due angoli in cui il segnale cambia.
 * L'originale avanza intanto a 1 cm/s fino al bumper: qui si avanza solo per STRADA_RIFINITURA. */
#define V_RIFINITURA      0.01
#define W_RIFINITURA      0.03
#define STRADA_RIFINITURA 0.03
#define TOLL_RIFINITURA   0.003      /* rad */
#define ARCO_MIN_RIFINITURA 0.03     /* rad: due letture quasi nello stesso angolo sono il segnale che balla, non i due bordi */
#define FRAME_STABILE     51
#define FRAME_DIREZIONE   100        /* dopo tanti frame senza cambi, la direzione vale anche con un solo fascio */
#define RIFINITURA_MAX_NS (30000 * 1000000LL)

#define STACCA_M          0.02       /* m: dopo un bumper si stacca di tanto, come l'originale */
#define V_STACCA          0.03
#define W_GIRA            0.50       /* rad/s: rallenta negli ultimi 30° */
#define GIRA_RALLENTA     0.5236
#define GIRA_TOLL         0.01

/* Retromarcia: l'originale chiede 20 cm a 3 cm/s; qui ne bastano 15 (dal muso a 6 cm i contatti sono a pochi cm). */
#define V_ULTIMI          0.03
#define K_ANGOLO          2.0        /* 1/s */
#define W_MAX             0.40
#define STRADA_ULTIMI     0.15
#define ULTIMI_MAX_NS     (8000 * 1000000LL)  /* senza contatto: se le ruote non avanzano gli encoder non bastano */

/* Arrivato lontano dall'asse: i contatti non si chiuderebbero. Si torna indietro e si riprova, come l'originale. */
#define V_INDIETRO        0.08
#define STRADA_INDIETRO   0.20
#define RIPROVE_ASSE      2

/* Dopo il primo contatto continua a premere finché la carica è attiva per 6 frame, come l'originale
 * (processBackwardToDock): fermandosi al primo tocco il robot resta a filo e bastano pochi mm per perdere il contatto. */
#define FRAME_PREMI         6
#define STRADA_PREMI        0.01     /* m: oltre si sposterebbe la base */
#define PREMI_MAX_NS        (1000 * 1000000LL)

#define FRAME_CARICA        26       /* come l'originale: il contatto vale solo se resta per tanti frame di fila */
#define CONTATTO_ATTESA_NS  (15000 * 1000000LL)
#define CONTATTO_PERSO_NS   (1500 * 1000000LL)
#define CONTATTO_QUOTA      0.8      /* bit "contatto" acceso almeno in questa frazione di frame: è intermittente */

#define V_RITIRA          0.08
#define STRADA_RITIRA     0.25
#define RITIRA_MAX_NS     (8000 * 1000000LL)
#define AVVICINA_MAX_NS   (120000 * 1000000LL)
#define TEMPO_MAX_NS      (300000 * 1000000LL)

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
	a->strada = 0;
	a->arco = 0;
	a->frame = a->frame2 = 0;
	a->frame_contatto = a->frame_totali = 0;
	a->t_senza_contatto = 0;
}

static void inizia_ricerca(aggancio_t *a, int64_t t)
{
	passa_a(a, AGG_F_CERCA, t);
	a->visto = 0;
	a->parte_dentro = 0;
	a->verso = -1;
}

void aggancio_avvia(aggancio_t *a, int64_t t)
{
	aggancio_init(a);
	a->esito = AGG_IN_CORSO;
	a->tentativo = 1;
	a->t_inizio = t;
	inizia_ricerca(a, t);
}

void aggancio_annulla(aggancio_t *a)
{
	aggancio_init(a);
}

int aggancio_attivo(const aggancio_t *a) { return a->esito == AGG_IN_CORSO; }
int aggancio_stacco(const aggancio_t *a) { return a->esito == AGG_IN_CORSO && (a->fase == AGG_F_RITIRA || a->fase == AGG_F_PREMI); }

static void fallisce(aggancio_t *a, const char *motivo)
{
	a->esito = AGG_FALLITO;
	a->motivo = motivo;
	a->fase = AGG_F_NIENTE;
	a->v = a->w = 0;
}

void aggancio_interrompi(aggancio_t *a, const char *motivo)
{
	fallisce(a, motivo);
}

static void riesce(aggancio_t *a)
{
	a->esito = AGG_FATTO;
	a->fase = AGG_F_NIENTE;
	a->v = a->w = 0;
}

/* Lascia perdere questo tentativo: avanti di un po', poi si ricomincia a cercare. */
static void ritira(aggancio_t *a, const char *motivo, int64_t t)
{
	a->motivo = motivo;
	passa_a(a, AGG_F_RITIRA, t);
	a->direzione = a->yaw;
}

/* -------------------------------------------------------------------------------------------- infrarossi --- */

/* Il valore di un ricevitore come CObservationDock: 3 = fasci 1 e 2 insieme, altrimenti il fascio più basso. */
static int valore_ir(int nibble)
{
	if ((nibble & 3) == 3) return 3;
	if (nibble & 1) return 1;
	if (nibble & 2) return 2;
	return nibble;
}

/* Da che parte sta il robot rispetto all'asse (CActionDockAlignState::getDockAlignCondition): l'asse è il confine fra
 * il fascio 1 (destra del robot che guarda la base) e il 2 (sinistra); il 4 è oltre l'1, l'8 oltre il 2.
 * Ritorna 0 = niente segnale, 1 = sull'asse, 2 = verso il fascio 2 (deve girare a destra), 3 = verso il fascio 1 (a sinistra). */
static int lato(int sx, int dx)
{
	int l = sx == 4 ? 1 : sx == 8 ? 2 : sx > 8 ? 0 : sx;
	int r = dx == 4 ? 1 : dx == 8 ? 2 : dx > 8 ? 0 : dx;
	if (l == 3 && r == 2) return 2;
	if (l == 2 && r == 3) return 2;
	if (l == 3 && r == 1) return 3;
	if (l == 1 && r == 3) return 3;
	int u = l | r;
	return u == 3 ? 1 : u == 2 ? 2 : u == 1 ? 3 : 0;
}

/* -------------------------------------------------------------------------------------------------- lidar --- */

void aggancio_giro(aggancio_t *a, const lidar_giro_t *g, int64_t t)
{
	if (a->esito != AGG_IN_CORSO)
		return;
	double somma = 0;
	int n = 0, vicini = 0;
	for (int i = 0; i < g->n; i++) {
		const lidar_punto_t *p = &g->p[i];
		if (!p->distanza_mm || p->angolo_cdeg < SETTORE_DA_CDEG || p->angolo_cdeg > SETTORE_A_CDEG)
			continue;
		double r = p->distanza_mm / 1000.0;
		if (r - PROFILO_MUSO < SPAZIO_ARRIVO)
			vicini++;
		if (r > DIST_MIN && r < DIST_MAX) {
			somma += r;
			n++;
		}
	}
	a->dist_davanti = n ? somma / n : DIST_MAX;
	a->punti_vicini = vicini;
	a->t_giro = t;
}

/* ------------------------------------------------------------------------------------------------ guida --- */

/* Una fase in cui si ruota: 1 se è arrivato. */
static int ruota_verso(aggancio_t *a, double obiettivo)
{
	double d = normale(obiettivo - a->yaw);
	if (fabs(d) < GIRA_TOLL) {
		a->w = 0;
		return 1;
	}
	double r = fabs(d) > GIRA_RALLENTA ? 1.0 : fabs(d) / GIRA_RALLENTA;
	a->w = (d > 0 ? 1 : -1) * W_GIRA * (r < 0.2 ? 0.2 : r);
	return 0;
}

void aggancio_frame(aggancio_t *a, const gd32_sensori_t *s, int64_t t)
{
	/* quanto si è mosso dall'ultimo frame: angolo dall'IMU, strada dagli encoder (avanti = positiva) */
	double strada = 0, dimb = 0;
	if (a->ha_prec) {
		/* il GD32 passa da +180° a -180°: la differenza si porta fra -180° e +180° (non basta il wrap di int16) */
		int diff = s->angolo[0] - a->imbardata_prec;
		diff = ((diff + 18000) % 36000 + 36000) % 36000 - 18000;
		dimb = diff * (M_PI / 18000) * SCALA_IMBARDATA;
		a->yaw = normale(a->yaw + dimb);
		strada = (((int32_t)((uint32_t)s->encoder_sx - (uint32_t)a->enc_sx_prec) +
		           (int32_t)((uint32_t)s->encoder_dx - (uint32_t)a->enc_dx_prec)) / 2.0) / GD32_SCALA_V;
	}
	a->imbardata_prec = s->angolo[0];
	a->enc_sx_prec = s->encoder_sx;
	a->enc_dx_prec = s->encoder_dx;
	a->ha_prec = 1;

	if (a->esito != AGG_IN_CORSO) {
		a->v = a->w = 0;
		return;
	}
	a->strada += strada;
	a->arco += dimb;

	/* nibble basso = ricevitore sinistro, alto = destro (docs/fase3.md, prova ir1) */
	int sx = valore_ir(s->ir_base_b), dx = valore_ir(s->ir_base_a);
	int segnale = sx || dx;
	if (segnale)
		a->t_segnale = t;
	int giro_buono = a->t_giro && t - a->t_giro < GIRO_VECCHIO_NS;
	int bumper = s->bumper_sx || s->bumper_dx;

	/* un segnale di carica ferma tutto, ma "riuscito" lo decide la fase del contatto, quando è stabile */
	if (s->in_carica && a->fase != AGG_F_RITIRA && a->fase != AGG_F_CONTATTO && a->fase != AGG_F_ULTIMI && a->fase != AGG_F_PREMI)
		passa_a(a, AGG_F_CONTATTO, t);

	switch (a->fase) {
	case AGG_F_CERCA:
		if (s->sulla_base) {                           /* già sui contatti: si controlla la carica */
			passa_a(a, AGG_F_CONTATTO, t);
			break;
		}
		a->v = 0;
		a->w = a->verso * W_CERCA;
		if (!a->visto) {
			if (segnale) {
				a->visto = 1;
				a->yaw_primo = a->yaw;
				a->frame = 0;
				/* se lo vede già all'inizio, il primo bordo dell'arco non è questo: lo trova tornando indietro */
				a->parte_dentro = fabs(a->arco) < 0.05;
			} else if (fabs(a->arco) > 2 * M_PI) {
				fallisce(a, "un giro intero senza vedere la base");
			}
			break;
		}
		a->frame = segnale ? 0 : a->frame + 1;
		if (a->frame >= FRAME_PERSO_CERCA && a->parte_dentro) {
			a->parte_dentro = 0;
			a->visto = 0;
			a->verso = -a->verso;
			a->arco = 0;
			break;
		}
		if (a->frame >= FRAME_PERSO_CERCA || fabs(a->arco) > 4 * M_PI) {
			double arco = normale(a->yaw_primo - a->yaw);
			a->yaw_centro = normale(a->yaw + arco * FRAZIONE_ARCO);
			passa_a(a, AGG_F_CENTRA, t);
		}
		break;

	case AGG_F_CENTRA:
		a->v = 0;
		if (ruota_verso(a, a->yaw_centro)) {
			passa_a(a, AGG_F_AVVICINA, t);
			a->lato_prec = 0;
		}
		break;

	case AGG_F_AVVICINA: {
		int arrivato = giro_buono && a->punti_vicini >= PUNTI_ARRIVO && segnale;
		a->frame2 = arrivato ? a->frame2 + 1 : 0;
		a->frame_asse = !arrivato ? 0 : a->frame_asse + ((sx == 2 && dx == 1) || sx == 3 || dx == 3 ? 1 : 0);
		if (a->frame2 >= FRAME_ARRIVO) {
			if (a->frame_asse == 0 && a->riprove_asse < RIPROVE_ASSE) {
				a->riprove_asse++;
				passa_a(a, AGG_F_INDIETRO, t);
				break;
			}
			passa_a(a, AGG_F_RIFINISCI, t);
			a->ha_a = a->ha_b = 0;
			break;
		}
		if (bumper) {
			/* con il segnale visto da poco è la base (se il lidar non l'ha vista, è nera), altrimenti è un ostacolo */
			if (t - a->t_segnale < SEGNALE_RECENTE_NS) {
				a->dopo_stacco = AGG_F_RIFINISCI;
				passa_a(a, AGG_F_STACCA, t);
			} else {
				a->dopo_stacco = AGG_F_CERCA;
				passa_a(a, AGG_F_STACCA, t);
			}
			break;
		}
		a->frame = segnale ? 0 : a->frame + 1;
		if (a->frame >= FRAME_PERSO) {
			inizia_ricerca(a, t);
			break;
		}
		if (t - a->t_fase > AVVICINA_MAX_NS) {
			ritira(a, "avvicinamento troppo lungo", t);
			break;
		}
		double dist = giro_buono ? a->dist_davanti : DIST_MAX;
		a->v = dist < DIST_VICINO ? V_VICINO : (sx == 2 || dx == 2 || sx == 3 || dx == 3) ? V_FASCIO_CORTO : V_LONTANO;
		double wm = dist < DIST_W_VICINO ? W_VICINO : W_LONTANO;
		int l = lato(sx, dx);
		if (l == 1)
			a->w = 0;
		else if (l == 2)
			a->w = -wm;
		else if (l == 3)
			a->w = wm;
		else if (a->lato_prec == 2)
			a->w = W_PERSO;
		else if (a->lato_prec == 3)
			a->w = -W_PERSO;
		a->lato_prec = l;
		break;
	}

	case AGG_F_RIFINISCI: {
		/* CActionDockAlignState::fineTuneAlign: fra le due parti del confine, oscillando piano */
		a->frame++;
		double w = -W_RIFINITURA;
		if (sx == 2 && dx == 1) {
			w = 0;
		} else if ((dx == 2 || dx == 3) && (sx == 2 || sx == 3)) {
			a->yaw_a = a->yaw;
			a->ha_a = 1;
		} else if ((sx == 1 && (dx == 1 || dx == 3)) || (dx == 1 && sx == 3)) {
			a->yaw_b = a->yaw;
			a->ha_b = 1;
			w = W_RIFINITURA;
		} else if ((dx | sx) & 1) {
			w = W_RIFINITURA;
			if (a->frame > FRAME_DIREZIONE) {
				a->yaw_b = a->yaw;
				a->ha_b = 1;
			}
		} else if (a->frame > FRAME_DIREZIONE) {
			a->yaw_a = a->yaw;
			a->ha_a = 1;
		}
		a->w = w;
		a->v = (!bumper && a->strada < STRADA_RIFINITURA) ? V_RIFINITURA : 0;

		/* fatta se è a metà fra i due angoli, o se il segnale "sull'asse" resta stabile per ~1 s (CActionDockAlignState::isActionFinish) */
		int centrato = a->ha_a && a->ha_b && fabs(normale(a->yaw_b - a->yaw_a)) > ARCO_MIN_RIFINITURA && fabs(normale(a->yaw - normale(a->yaw_a + normale(a->yaw_b - a->yaw_a) / 2))) < TOLL_RIFINITURA;
		int asse = (sx == 3 && dx) || (dx == 3 && sx) || (sx == 2 && (dx == 1 || dx == 2)) || (sx == 4 && dx == 1) ||
		           (dx == 8 && sx == 2) || (sx == 1 && dx == 1);
		a->frame2 = asse ? a->frame2 + 1 : 0;
		if (centrato || a->frame2 >= FRAME_STABILE || t - a->t_fase > RIFINITURA_MAX_NS) {
			a->yaw_centro = normale(a->yaw + M_PI);
			passa_a(a, AGG_F_GIRA, t);
		} else if (t - a->t_segnale > 3 * SEGNALE_RECENTE_NS) {
			inizia_ricerca(a, t);
		}
		break;
	}

	case AGG_F_INDIETRO:
		a->v = -V_INDIETRO;
		a->w = 0;
		if (a->strada <= -STRADA_INDIETRO || t - a->t_fase > RITIRA_MAX_NS)
			inizia_ricerca(a, t);
		break;

	case AGG_F_STACCA:
		a->v = -V_STACCA;
		a->w = 0;
		if (a->strada <= -STACCA_M) {
			if (a->dopo_stacco == AGG_F_RIFINISCI) {
				passa_a(a, AGG_F_RIFINISCI, t);
				a->ha_a = a->ha_b = 0;
			} else {
				inizia_ricerca(a, t);
			}
		}
		break;

	case AGG_F_GIRA:
		a->v = 0;
		if (ruota_verso(a, a->yaw_centro)) {
			passa_a(a, AGG_F_ULTIMI, t);
			a->direzione = a->yaw_centro;
		}
		break;

	case AGG_F_ULTIMI:
		if (s->sulla_base) {
			passa_a(a, AGG_F_PREMI, t);
			a->v = -V_ULTIMI;
			a->w = 0;
			break;
		}
		if (-a->strada > STRADA_ULTIMI || t - a->t_fase > ULTIMI_MAX_NS) {
			ritira(a, "nessun contatto arretrando", t);
			break;
		}
		a->v = -V_ULTIMI;
		a->w = limita(K_ANGOLO * normale(a->direzione - a->yaw), W_MAX);
		break;

	case AGG_F_PREMI:
		a->frame = s->in_carica ? a->frame + 1 : 0;
		if (a->frame >= FRAME_PREMI || -a->strada >= STRADA_PREMI || t - a->t_fase > PREMI_MAX_NS) {
			passa_a(a, AGG_F_CONTATTO, t);
			break;
		}
		a->v = -V_ULTIMI;
		a->w = 0;
		break;

	case AGG_F_CONTATTO:
		a->v = a->w = 0;
		/* il bit "in carica" del GD32 si accende solo a carica finita: con la batteria scarica non arriva mai,
		   mentre sui contatti la carica passa (prova carica1 dell'8/11). Quindi conta il contatto */
		a->frame = s->sulla_base ? a->frame + 1 : 0;
		if (a->frame >= FRAME_CARICA) {
			riesce(a);
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
			if (a->frame_contatto >= CONTATTO_QUOTA * a->frame_totali)
				riesce(a);                             /* contatto stabile senza carica: batteria già piena */
			else
				ritira(a, "contatto instabile", t);
		}
		break;

	case AGG_F_RITIRA:
		if (a->strada >= STRADA_RITIRA || t - a->t_fase > RITIRA_MAX_NS) {
			if (a->tentativo >= AGG_TENTATIVI) {
				fallisce(a, a->motivo ? a->motivo : "troppi tentativi");
				break;
			}
			a->tentativo++;
			inizia_ricerca(a, t);
			break;
		}
		a->v = V_RITIRA;
		a->w = limita(K_ANGOLO * normale(a->direzione - a->yaw), W_MAX);
		break;

	default:
		a->v = a->w = 0;
		break;
	}

	if (a->esito == AGG_IN_CORSO && t - a->t_inizio > TEMPO_MAX_NS)
		fallisce(a, "tempo massimo");
}
