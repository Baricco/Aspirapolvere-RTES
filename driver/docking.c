/* docking.c - Messa in carica guidata dagli infrarossi della base, con lo stesso approccio del firmware originale (vedi docking.h). */
#include <math.h>
#include <string.h>

#include "docking.h"

/* Ricerca: ruota su se stesso. Dal segnale trovato al segnale perso si torna indietro di questa frazione dell'arco
 * (CActionTransferMotionGoDock::calculateMiddlePhi: 0,4, non 0,5, perché il robot si ferma dopo aver perso il segnale). */
#define W_SEARCH           0.75       /* rad/s: rapporto 1,5 x massimo 0,5 dell'originale */
#define W_CENTER          0.50
#define ARC_FRACTION     0.4
#define SEARCH_LOST_FRAMES 5          /* il segnale è intermittente: perso vuol dire assente per 5 frame di fila */

/* Avvicinamento di fronte. Le velocità in rotazione sono quelle dell'originale (CActionDockAlignState). */
#define V_FAR         0.15       /* m/s */
#define V_SHORT_BEAM    0.05       /* DIVERSO dall'originale: vedendo il fascio corto si è già vicini anche se il lidar non vede
                                        la base (nera): senza questo limite arriverebbe contro la base a 15 cm/s */
#define V_NEAR          0.025      /* lidar a meno di 0,5 m dalla faccia */
#define DIST_NEAR       0.50
#define W_FAR         0.50       /* rad/s */
#define W_NEAR          0.25       /* lidar a meno di 0,55 m */
#define DIST_W_NEAR     0.55
#define W_LOST           0.35       /* appena perso il segnale: torna dalla parte da cui l'ha perso */
#define LOST_FRAMES       51         /* ~1 s senza segnale: ricomincia a cercare */

/* Lidar: angolo 180° = davanti. Spazio davanti = distanza - profilo del corpo (dal lidar al muso: 0,28 m nella tabella
 * dell'originale). Si guarda il settore 170..190°, l'originale chiede 10 punti: la base nera ne dà pochi. */
#define SECTOR_FROM_CDEG   17000
#define SECTOR_TO_CDEG    19000
#define NOSE_PROFILE      0.28
#define ARRIVAL_GAP     0.06       /* m: il muso è a 6 cm dalla base */
#define ARRIVAL_POINTS      5
#define ARRIVAL_FRAMES      8
#define DIST_MIN          0.30       /* come CDockMiddlePoint::getAlignDistance: la media vale fra 0,3 e 1,5 m */
#define DIST_MAX          1.50
#define SCAN_OLD_NS   (500 * 1000000LL)
#define SIGNAL_RECENT_NS (1000 * 1000000LL)

/* Rifinitura a pochi centimetri: oscilla piano e prende la direzione a metà dei due angoli in cui il segnale cambia.
 * L'originale avanza intanto a 1 cm/s fino al bumper: qui si avanza solo per FINE_TUNE_DISTANCE. */
#define V_FINE_TUNE      0.01
#define W_FINE_TUNE      0.03
#define FINE_TUNE_DISTANCE 0.03
#define FINE_TUNE_TOL   0.003      /* rad */
#define FINE_TUNE_MIN_ARC 0.03     /* rad: due letture quasi nello stesso angolo sono il segnale che balla, non i due bordi */
#define STABLE_FRAMES     51
#define DIRECTION_FRAMES   100        /* dopo tanti frame senza cambi, la direzione vale anche con un solo fascio */
#define FINE_TUNE_MAX_NS (30000 * 1000000LL)

#define DETACH_M          0.02       /* m: dopo un bumper si stacca di tanto, come l'originale */
#define V_DETACH          0.03
#define W_TURN            0.50       /* rad/s: rallenta negli ultimi 30° */
#define TURN_SLOWDOWN     0.5236
#define TURN_TOL         0.01

/* Retromarcia: l'originale chiede 20 cm a 3 cm/s; qui ne bastano 15 (dal muso a 6 cm i contatti sono a pochi cm). */
#define V_LAST_CM          0.03
#define K_ANGLE          2.0        /* 1/s */
#define W_MAX             0.40
#define LAST_CM_DISTANCE     0.15
#define LAST_CM_MAX_NS     (8000 * 1000000LL)  /* senza contatto: se le ruote non avanzano gli encoder non bastano */

/* Arrivato lontano dall'asse: i contatti non si chiuderebbero. Si torna indietro e si riprova, come l'originale. */
#define V_BACK_OFF        0.08
#define BACK_OFF_DISTANCE   0.20
#define AXIS_RETRIES      2

/* Dopo il primo contatto continua a premere finché la carica è attiva per 6 frame, come l'originale
 * (processBackwardToDock): fermandosi al primo tocco il robot resta a filo e bastano pochi mm per perdere il contatto. */
#define PRESS_FRAMES         6
#define PRESS_DISTANCE        0.01     /* m: oltre si sposterebbe la base */
#define PRESS_MAX_NS        (1000 * 1000000LL)

#define CHARGE_FRAMES        26       /* come l'originale: il contatto vale solo se resta per tanti frame di fila */
#define CONTACT_WAIT_NS  (15000 * 1000000LL)
#define CONTACT_LOST_NS   (1500 * 1000000LL)
#define CONTACT_RATIO      0.8      /* bit "contatto" acceso almeno in questa frazione di frame: è intermittente */

#define V_RETREAT          0.08
#define RETREAT_DISTANCE     0.25

/* Raddrizzamento: il muro davanti è la retta con più punti del lidar fra WALL_MIN e WALL_MAX metri, entro WALL_HALF_ANGLE
 * dalla direzione del robot; la base è la parte di quei punti che sporge dal muro fra BASE_MIN e BASE_MAX metri, entro
 * BASE_HALF_WIDTH lungo il muro (nelle prove: sporge 6,1-6,9 cm, larga 12-13,5 cm). Sui dati veri l'angolo misurato così
 * coincide con quello della mappa entro 1-2,5°. */
#define WALL_MIN             0.05
#define WALL_MAX             1.20
#define WALL_HALF_ANGLE      1.0472     /* rad (60°) */
#define WALL_FIT_TOL         0.01       /* m: punti entro tanto dalla retta, all'ultimo passo dell'adattamento */
#define WALL_MIN_POINTS      30
#define BASE_MIN             0.03
#define BASE_MAX             0.15
#define BASE_HALF_WIDTH      0.35
#define BASE_MIN_POINTS      5
#define STRAIGHTEN_MAX_ANGLE 0.7854     /* rad (45°): oltre, la misura del muro non è credibile e si gira come prima */
#define STRAIGHTEN_MAX_NS    (10000 * 1000000LL)
#define RETREAT_MAX_NS     (8000 * 1000000LL)
#define APPROACH_MAX_NS   (120000 * 1000000LL)
#define TOTAL_MAX_NS      (300000 * 1000000LL)

/* Scala dell'IMU: conta circa il 3% in meno (la stessa taratura del task del moto). */
#define YAW_SCALE   1.0285

static double normalize_angle(double a)
{
	while (a > M_PI) a -= 2 * M_PI;
	while (a < -M_PI) a += 2 * M_PI;
	return a;
}

static double clamp_abs(double x, double m) { return x > m ? m : x < -m ? -m : x; }

void docking_init(docking_t *a)
{
	memset(a, 0, sizeof *a);
}

static void go_to_phase(docking_t *a, dock_phase_t f, int64_t t)
{
	a->phase = f;
	a->t_phase = t;
	a->v = a->w = 0;
	a->distance = 0;
	a->arc = 0;
	a->frame = a->frame2 = 0;
	a->contact_frames = a->total_frames = 0;
	a->t_no_contact = 0;
}

static void start_search(docking_t *a, int64_t t)
{
	go_to_phase(a, DOCK_P_SEARCH, t);
	a->seen = 0;
	a->starts_inside = 0;
	a->turn_dir = -1;
}

void docking_start(docking_t *a, int64_t t)
{
	docking_init(a);
	a->result = DOCK_RUNNING;
	a->attempt = 1;
	a->t_start = t;
	start_search(a, t);
}

void docking_cancel(docking_t *a)
{
	docking_init(a);
}

int docking_active(const docking_t *a) { return a->result == DOCK_RUNNING; }
int docking_detach(const docking_t *a) { return a->result == DOCK_RUNNING && (a->phase == DOCK_P_RETREAT || a->phase == DOCK_P_PRESS); }

static void fail(docking_t *a, const char *reason)
{
	a->result = DOCK_FAILED;
	a->reason = reason;
	a->phase = DOCK_P_NONE;
	a->v = a->w = 0;
}

void docking_abort(docking_t *a, const char *reason)
{
	fail(a, reason);
}

static void succeed(docking_t *a)
{
	a->result = DOCK_DONE;
	a->phase = DOCK_P_NONE;
	a->v = a->w = 0;
}

/* Lascia perdere questo tentativo: avanti di un po', poi si ricomincia a cercare. */
static void retreat(docking_t *a, const char *reason, int64_t t)
{
	a->reason = reason;
	go_to_phase(a, DOCK_P_RETREAT, t);
	a->direction = a->yaw;
}

/* -------------------------------------------------------------------------------------------- infrarossi --- */

/* Il valore di un ricevitore come CObservationDock: 3 = fasci 1 e 2 insieme, altrimenti il fascio più basso. */
static int ir_value(int nibble)
{
	if ((nibble & 3) == 3) return 3;
	if (nibble & 1) return 1;
	if (nibble & 2) return 2;
	return nibble;
}

/* Da che parte sta il robot rispetto all'asse (CActionDockAlignState::getDockAlignCondition): l'asse è il confine fra
 * il fascio 1 (destra del robot che guarda la base) e il 2 (sinistra); il 4 è oltre l'1, l'8 oltre il 2.
 * Ritorna 0 = niente segnale, 1 = sull'asse, 2 = verso il fascio 2 (deve girare a destra), 3 = verso il fascio 1 (a sinistra). */
static int side(int left, int right)
{
	int l = left == 4 ? 1 : left == 8 ? 2 : left > 8 ? 0 : left;
	int r = right == 4 ? 1 : right == 8 ? 2 : right > 8 ? 0 : right;
	if (l == 3 && r == 2) return 2;
	if (l == 2 && r == 3) return 2;
	if (l == 3 && r == 1) return 3;
	if (l == 1 && r == 3) return 3;
	int u = l | r;
	return u == 3 ? 1 : u == 2 ? 2 : u == 1 ? 3 : 0;
}

/* -------------------------------------------------------------------------------------------------- lidar --- */

/* Il muro davanti e la base, da un giro del lidar. Il muro: retta x = k y + q (sistema del robot) adattata ai minimi
 * quadrati tre volte, ogni volta solo con i punti vicini alla retta precedente (5, 2, 1 cm): la base e gli oggetti davanti
 * escono dal conto. La base: i punti che sporgono dal muro fra BASE_MIN e BASE_MAX, entro BASE_HALF_WIDTH dal punto del
 * muro davanti al robot; il suo centro lungo il muro è il punto medio fra il punto più a destra e quello più a sinistra. */
static void measure_wall_and_base(docking_t *a, const lidar_scan_t *g, int64_t t)
{
	static double px[LIDAR_SCAN_MAX_POINTS], py[LIDAR_SCAN_MAX_POINTS];
	int n = 0;

	a->wall_valid = 0;
	a->base_seen = 0;

	for (int i = 0; i < g->n; i++) {
		if (!g->p[i].distance_mm)
			continue;
		double x, y;
		lidar_point_xy(&g->p[i], &x, &y);
		double r = hypot(x, y);
		if (x <= 0 || r < WALL_MIN || r > WALL_MAX || fabs(atan2(y, x)) > WALL_HALF_ANGLE)
			continue;
		px[n] = x;
		py[n] = y;
		n++;
	}
	if (n < WALL_MIN_POINTS)
		return;

	double k = 0, q = 0;
	const double tol[3] = { 1e9, 0.05, 0.02 };
	int used = 0;
	for (int pass = 0; pass < 4; pass++) {
		double band = pass < 3 ? tol[pass] : WALL_FIT_TOL;
		double sy = 0, sx = 0, syy = 0, sxy = 0;
		used = 0;
		for (int i = 0; i < n; i++) {
			if (pass > 0 && fabs(px[i] - (k * py[i] + q)) * cos(atan(k)) > band)
				continue;
			sy += py[i]; sx += px[i]; syy += py[i] * py[i]; sxy += py[i] * px[i];
			used++;
		}
		double den = used * syy - sy * sy;
		if (used < WALL_MIN_POINTS || fabs(den) < 1e-9)
			return;
		k = (used * sxy - sy * sx) / den;
		q = (sx - k * sy) / used;
	}

	/* normale verso il muro: x cos(c) + y sin(c) = dist, con k = -tan(c) */
	double c = -atan(k);
	a->wall_angle = c;
	a->wall_dist = q * cos(c);
	a->wall_valid = 1;
	a->t_wall = t;

	/* la base: punti davanti al muro di BASE_MIN..BASE_MAX, entro BASE_HALF_WIDTH lungo il muro */
	double lo = 1e9, hi = -1e9, prot_sum = 0;
	int nb = 0;
	for (int i = 0; i < n; i++) {
		double normal = px[i] * cos(c) + py[i] * sin(c);
		double along = -px[i] * sin(c) + py[i] * cos(c);
		double prot = a->wall_dist - normal;
		if (prot < BASE_MIN || prot > BASE_MAX || fabs(along) > BASE_HALF_WIDTH)
			continue;
		if (along < lo) lo = along;
		if (along > hi) hi = along;
		prot_sum += prot;
		nb++;
	}
	if (nb >= BASE_MIN_POINTS) {
		a->base_seen = 1;
		a->base_along = (lo + hi) / 2;
		a->base_protrusion = prot_sum / nb;
	}
}

void docking_scan(docking_t *a, const lidar_scan_t *g, int64_t t)
{
	if (a->result != DOCK_RUNNING)
		return;
	double sum = 0;
	int n = 0, near = 0;
	for (int i = 0; i < g->n; i++) {
		const lidar_point_t *p = &g->p[i];
		if (!p->distance_mm || p->angle_cdeg < SECTOR_FROM_CDEG || p->angle_cdeg > SECTOR_TO_CDEG)
			continue;
		double r = p->distance_mm / 1000.0;
		if (r - NOSE_PROFILE < ARRIVAL_GAP)
			near++;
		if (r > DIST_MIN && r < DIST_MAX) {
			sum += r;
			n++;
		}
	}
	a->front_dist = n ? sum / n : DIST_MAX;
	a->near_points = near;
	a->t_scan = t;

	if (a->phase == DOCK_P_FINE_TUNE || a->phase == DOCK_P_STRAIGHTEN)
		measure_wall_and_base(a, g, t);
}

/* ------------------------------------------------------------------------------------------------ guida --- */

/* Una fase in cui si ruota: 1 se è arrivato. */
static int rotate_towards(docking_t *a, double target)
{
	double d = normalize_angle(target - a->yaw);
	if (fabs(d) < TURN_TOL) {
		a->w = 0;
		return 1;
	}
	double r = fabs(d) > TURN_SLOWDOWN ? 1.0 : fabs(d) / TURN_SLOWDOWN;
	a->w = (d > 0 ? 1 : -1) * W_TURN * (r < 0.2 ? 0.2 : r);
	return 0;
}

void docking_frame(docking_t *a, const gd32_sensors_t *s, int64_t t)
{
	/* quanto si è mosso dall'ultimo frame: angolo dall'IMU, strada dagli encoder (avanti = positiva) */
	double distance = 0, d_yaw = 0;
	if (a->has_prev) {
		/* il GD32 passa da +180° a -180°: la differenza si porta fra -180° e +180° (non basta il wrap di int16) */
		int diff = s->angle[0] - a->prev_yaw_raw;
		diff = ((diff + 18000) % 36000 + 36000) % 36000 - 18000;
		d_yaw = diff * (M_PI / 18000) * YAW_SCALE;
		a->yaw = normalize_angle(a->yaw + d_yaw);
		distance = (((int32_t)((uint32_t)s->encoder_left - (uint32_t)a->prev_enc_left) +
		           (int32_t)((uint32_t)s->encoder_right - (uint32_t)a->prev_enc_right)) / 2.0) / GD32_SCALE_V;
	}
	a->prev_yaw_raw = s->angle[0];
	a->prev_enc_left = s->encoder_left;
	a->prev_enc_right = s->encoder_right;
	a->has_prev = 1;

	if (a->result != DOCK_RUNNING) {
		a->v = a->w = 0;
		return;
	}
	a->distance += distance;
	a->arc += d_yaw;

	/* nibble basso = ricevitore sinistro, alto = destro (docs/fase3.md, prova ir1) */
	int left = ir_value(s->ir_dock_left), right = ir_value(s->ir_dock_right);
	int ir_signal = left || right;
	if (ir_signal)
		a->t_signal = t;
	int scan_fresh = a->t_scan && t - a->t_scan < SCAN_OLD_NS;
	int bumper = s->bumper_left || s->bumper_right;

	/* un segnale di carica ferma tutto, ma "riuscito" lo decide la fase del contatto, quando è stabile */
	if (s->charging && a->phase != DOCK_P_RETREAT && a->phase != DOCK_P_CONTACT && a->phase != DOCK_P_LAST_CM && a->phase != DOCK_P_PRESS)
		go_to_phase(a, DOCK_P_CONTACT, t);

	switch (a->phase) {
	case DOCK_P_SEARCH:
		if (s->on_base) {                           /* già sui contatti: si controlla la carica */
			go_to_phase(a, DOCK_P_CONTACT, t);
			break;
		}
		a->v = 0;
		a->w = a->turn_dir * W_SEARCH;
		if (!a->seen) {
			if (ir_signal) {
				a->seen = 1;
				a->yaw_first = a->yaw;
				a->frame = 0;
				/* se lo vede già all'inizio, il primo bordo dell'arco non è questo: lo trova tornando indietro */
				a->starts_inside = fabs(a->arc) < 0.05;
			} else if (fabs(a->arc) > 2 * M_PI) {
				fail(a, "un giro intero senza vedere la base");
			}
			break;
		}
		a->frame = ir_signal ? 0 : a->frame + 1;
		if (a->frame >= SEARCH_LOST_FRAMES && a->starts_inside) {
			a->starts_inside = 0;
			a->seen = 0;
			a->turn_dir = -a->turn_dir;
			a->arc = 0;
			break;
		}
		if (a->frame >= SEARCH_LOST_FRAMES || fabs(a->arc) > 4 * M_PI) {
			double arc = normalize_angle(a->yaw_first - a->yaw);
			a->yaw_center = normalize_angle(a->yaw + arc * ARC_FRACTION);
			go_to_phase(a, DOCK_P_CENTER, t);
		}
		break;

	case DOCK_P_CENTER:
		a->v = 0;
		if (rotate_towards(a, a->yaw_center)) {
			go_to_phase(a, DOCK_P_APPROACH, t);
			a->prev_side = 0;
			a->axis_seen = 0;
		}
		break;

	case DOCK_P_APPROACH: {
		int arrived = scan_fresh && a->near_points >= ARRIVAL_POINTS && ir_signal;
		a->frame2 = arrived ? a->frame2 + 1 : 0;
		a->axis_frames = !arrived ? 0 : a->axis_frames + ((left == 2 && right == 1) || left == 3 || right == 3 ? 1 : 0);
		if (a->frame2 >= ARRIVAL_FRAMES) {
			if (a->axis_frames == 0 && a->axis_retries < AXIS_RETRIES) {
				a->axis_retries++;
				go_to_phase(a, DOCK_P_BACK_OFF, t);
				break;
			}
			go_to_phase(a, DOCK_P_FINE_TUNE, t);
			a->has_a = a->has_b = 0;
			break;
		}
		if (bumper) {
			/* con il segnale visto da poco è la base (se il lidar non l'ha vista, è nera), altrimenti è un ostacolo.
			   Senza mai il fascio centrale il robot è arrivato fuori asse, di sbieco contro lo spigolo della base: la rifinitura
			   da lì ruoterebbe con il paraurti premuto. Indietro di BACK_OFF_DISTANCE e si ricomincia a cercare, dove i fasci si
			   distinguono (prova del 16/11 sera: 4 urti di fila con il paraurti sinistro e i soli fasci esterni) */
			if (t - a->t_signal < SIGNAL_RECENT_NS && !a->axis_seen && a->axis_retries < AXIS_RETRIES) {
				a->axis_retries++;
				go_to_phase(a, DOCK_P_BACK_OFF, t);
			} else if (t - a->t_signal < SIGNAL_RECENT_NS) {
				a->after_detach = DOCK_P_FINE_TUNE;
				go_to_phase(a, DOCK_P_DETACH, t);
			} else {
				a->after_detach = DOCK_P_SEARCH;
				go_to_phase(a, DOCK_P_DETACH, t);
			}
			break;
		}
		a->frame = ir_signal ? 0 : a->frame + 1;
		if (a->frame >= LOST_FRAMES) {
			start_search(a, t);
			break;
		}
		if (t - a->t_phase > APPROACH_MAX_NS) {
			retreat(a, "avvicinamento troppo lungo", t);
			break;
		}
		double dist = scan_fresh ? a->front_dist : DIST_MAX;
		a->v = dist < DIST_NEAR ? V_NEAR : (left == 2 || right == 2 || left == 3 || right == 3) ? V_SHORT_BEAM : V_FAR;
		double w_limit = dist < DIST_W_NEAR ? W_NEAR : W_FAR;
		int l = side(left, right);
		if (l == 1)
			a->axis_seen = 1;
		if (l == 1)
			a->w = 0;
		else if (l == 2)
			a->w = -w_limit;
		else if (l == 3)
			a->w = w_limit;
		else if (a->prev_side == 2)
			a->w = W_LOST;
		else if (a->prev_side == 3)
			a->w = -W_LOST;
		a->prev_side = l;
		break;
	}

	case DOCK_P_FINE_TUNE: {
		/* CActionDockAlignState::fineTuneAlign: fra le due parti del confine, oscillando piano */
		a->frame++;
		double w = -W_FINE_TUNE;
		if (left == 2 && right == 1) {
			w = 0;
		} else if ((right == 2 || right == 3) && (left == 2 || left == 3)) {
			a->yaw_a = a->yaw;
			a->has_a = 1;
		} else if ((left == 1 && (right == 1 || right == 3)) || (right == 1 && left == 3)) {
			a->yaw_b = a->yaw;
			a->has_b = 1;
			w = W_FINE_TUNE;
		} else if ((right | left) & 1) {
			w = W_FINE_TUNE;
			if (a->frame > DIRECTION_FRAMES) {
				a->yaw_b = a->yaw;
				a->has_b = 1;
			}
		} else if (a->frame > DIRECTION_FRAMES) {
			a->yaw_a = a->yaw;
			a->has_a = 1;
		}
		a->w = w;
		a->v = (!bumper && a->distance < FINE_TUNE_DISTANCE) ? V_FINE_TUNE : 0;

		/* fatta se è a metà fra i due angoli, o se il segnale "sull'asse" resta stabile per ~1 s (CActionDockAlignState::isActionFinish) */
		int centered = a->has_a && a->has_b && fabs(normalize_angle(a->yaw_b - a->yaw_a)) > FINE_TUNE_MIN_ARC && fabs(normalize_angle(a->yaw - normalize_angle(a->yaw_a + normalize_angle(a->yaw_b - a->yaw_a) / 2))) < FINE_TUNE_TOL;
		int on_axis = (left == 3 && right) || (right == 3 && left) || (left == 2 && (right == 1 || right == 2)) || (left == 4 && right == 1) ||
		           (right == 8 && left == 2) || (left == 1 && right == 1);
		a->frame2 = on_axis ? a->frame2 + 1 : 0;
		if (centered || a->frame2 >= STABLE_FRAMES || t - a->t_phase > FINE_TUNE_MAX_NS) {
			/* raddrizza se il lidar vede il muro con un angolo credibile, altrimenti gira come prima */
			if (a->wall_valid && t - a->t_wall < SCAN_OLD_NS && fabs(a->wall_angle) < STRAIGHTEN_MAX_ANGLE) {
				a->direction = normalize_angle(a->yaw + a->wall_angle);
				a->t_straight = 0;
				go_to_phase(a, DOCK_P_STRAIGHTEN, t);
			} else {
				a->yaw_center = normalize_angle(a->yaw + M_PI);
				go_to_phase(a, DOCK_P_TURN, t);
			}
		} else if (t - a->t_signal > 3 * SIGNAL_RECENT_NS) {
			start_search(a, t);
		}
		break;
	}

	case DOCK_P_BACK_OFF:
		a->v = -V_BACK_OFF;
		a->w = 0;
		if (a->distance <= -BACK_OFF_DISTANCE || t - a->t_phase > RETREAT_MAX_NS)
			start_search(a, t);
		break;

	case DOCK_P_DETACH:
		a->v = -V_DETACH;
		a->w = 0;
		if (a->distance <= -DETACH_M) {
			if (a->after_detach == DOCK_P_FINE_TUNE) {
				go_to_phase(a, DOCK_P_FINE_TUNE, t);
				a->has_a = a->has_b = 0;
			} else {
				start_search(a, t);
			}
		}
		break;

	case DOCK_P_STRAIGHTEN:
		/* ruota sul posto fino a essere perpendicolare al muro, poi aspetta un giro del lidar fatto da fermo e misura la
		   base. Senza base (o oltre il tempo massimo) gira lo stesso: l'aggancio non deve dipendere dalla misura */
		a->v = 0;
		if (!a->t_straight) {
			if (rotate_towards(a, a->direction))
				a->t_straight = t;
		} else {
			a->w = 0;
			if (a->t_wall > a->t_straight && a->wall_valid) {
				if (a->base_seen) {
					double c = a->wall_angle, face = a->wall_dist - a->base_protrusion;
					a->base_x = face * cos(c) - a->base_along * sin(c);
					a->base_y = face * sin(c) + a->base_along * cos(c);
					a->base_theta = normalize_angle(c + M_PI);
					a->base_t = a->t_wall;
					a->base_number++;
				}
				a->yaw_center = normalize_angle(a->yaw + M_PI);
				go_to_phase(a, DOCK_P_TURN, t);
				break;
			}
		}
		if (t - a->t_phase > STRAIGHTEN_MAX_NS) {
			a->yaw_center = normalize_angle(a->yaw + M_PI);
			go_to_phase(a, DOCK_P_TURN, t);
		}
		break;

	case DOCK_P_TURN:
		a->v = 0;
		if (rotate_towards(a, a->yaw_center)) {
			go_to_phase(a, DOCK_P_LAST_CM, t);
			a->direction = a->yaw_center;
		}
		break;

	case DOCK_P_LAST_CM:
		if (s->on_base) {
			go_to_phase(a, DOCK_P_PRESS, t);
			a->v = -V_LAST_CM;
			a->w = 0;
			break;
		}
		if (-a->distance > LAST_CM_DISTANCE || t - a->t_phase > LAST_CM_MAX_NS) {
			retreat(a, "nessun contatto arretrando", t);
			break;
		}
		a->v = -V_LAST_CM;
		a->w = clamp_abs(K_ANGLE * normalize_angle(a->direction - a->yaw), W_MAX);
		break;

	case DOCK_P_PRESS:
		a->frame = s->charging ? a->frame + 1 : 0;
		if (a->frame >= PRESS_FRAMES || -a->distance >= PRESS_DISTANCE || t - a->t_phase > PRESS_MAX_NS) {
			go_to_phase(a, DOCK_P_CONTACT, t);
			break;
		}
		a->v = -V_LAST_CM;
		a->w = 0;
		break;

	case DOCK_P_CONTACT:
		a->v = a->w = 0;
		/* il bit "in carica" del GD32 si accende solo a carica finita: con la batteria scarica non arriva mai,
		   mentre sui contatti la carica passa (prova carica1 dell'8/11). Quindi conta il contatto */
		a->frame = s->on_base ? a->frame + 1 : 0;
		if (a->frame >= CHARGE_FRAMES) {
			succeed(a);
			break;
		}
		a->total_frames++;
		if (s->on_base) {
			a->contact_frames++;
			a->t_no_contact = 0;
		} else {
			if (!a->t_no_contact)
				a->t_no_contact = t;
			if (t - a->t_no_contact > CONTACT_LOST_NS) {
				retreat(a, "contatto perso", t);
				break;
			}
		}
		if (t - a->t_phase > CONTACT_WAIT_NS) {
			if (a->contact_frames >= CONTACT_RATIO * a->total_frames)
				succeed(a);                             /* contatto stabile senza carica: batteria già piena */
			else
				retreat(a, "contatto instabile", t);
		}
		break;

	case DOCK_P_RETREAT:
		if (a->distance >= RETREAT_DISTANCE || t - a->t_phase > RETREAT_MAX_NS) {
			if (a->attempt >= DOCK_ATTEMPTS) {
				fail(a, a->reason ? a->reason : "troppi tentativi");
				break;
			}
			a->attempt++;
			start_search(a, t);
			break;
		}
		a->v = V_RETREAT;
		a->w = clamp_abs(K_ANGLE * normalize_angle(a->direction - a->yaw), W_MAX);
		break;

	default:
		a->v = a->w = 0;
		break;
	}

	if (a->result == DOCK_RUNNING && t - a->t_start > TOTAL_MAX_NS)
		fail(a, "tempo massimo");
}
