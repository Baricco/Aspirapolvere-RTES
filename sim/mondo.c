/* mondo.c - La stanza a oggetti del simulatore: vedi mondo.h. */
#include "mondo.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "robot.h"

#define GRADI (M_PI / 180)
#define QUOTA_MURO 2.5
#define QUOTA_MINIMA 0.02          /* sotto 2 cm il robot ci passa sopra (supera soglie fino a ~2 cm [scheda]) */
#define BASE_QUOTA 0.10            /* [?] altezza della base */

static void aggiungi_muro(mondo_t *m, double x0, double y0, double x1, double y1, double spessore)
{
	if (m->n >= MONDO_MAX_OGGETTI)
		return;
	oggetto_t *o = &m->o[m->n++];
	memset(o, 0, sizeof *o);
	o->forma = OGG_RETTANGOLO;
	o->muro = 1;
	snprintf(o->nome, sizeof o->nome, "muro");
	o->x = (x0 + x1) / 2;
	o->y = (y0 + y1) / 2;
	o->angolo = atan2(y1 - y0, x1 - x0);
	o->larghezza = hypot(x1 - x0, y1 - y0);
	o->profondita = spessore;
	o->quota_a = QUOTA_MURO;
}

void mondo_nuovo(mondo_t *m, double w, double h)
{
	memset(m, 0, sizeof *m);
	snprintf(m->nome, sizeof m->nome, "nuova");
	m->larghezza = w;
	m->profondita = h;
	const double s = 0.10;
	aggiungi_muro(m, 0, s / 2, w, s / 2, s);
	aggiungi_muro(m, 0, h - s / 2, w, h - s / 2, s);
	aggiungi_muro(m, s / 2, s, s / 2, h - s, s);
	aggiungi_muro(m, w - s / 2, s, w - s / 2, h - s, s);
	m->robot_x = w / 2;
	m->robot_y = h / 2;
	m->robot_dir = M_PI / 2;
}

/* ---------------------------------------------------------------------------------------- lettura --- */

static int coppia(const char *s, double *a, double *b)
{
	return sscanf(s, "%lf,%lf", a, b) == 2;
}

int mondo_leggi_testo(mondo_t *m, const char *testo, char *errore, size_t dim)
{
	static mondo_t nuovo;          /* grande: non sullo stack */
	memset(&nuovo, 0, sizeof nuovo);
	snprintf(nuovo.nome, sizeof nuovo.nome, "senza nome");
	nuovo.larghezza = nuovo.profondita = -1;
	nuovo.robot_dir = M_PI / 2;
	int ha_robot = 0, nriga = 0;
	const char *p = testo;
	while (*p) {
		const char *fine = strchr(p, '\n');
		size_t n = fine ? (size_t)(fine - p) : strlen(p);
		char riga[2048];
		snprintf(riga, sizeof riga, "%.*s", (int)(n < sizeof riga - 1 ? n : sizeof riga - 1), p);
		p = fine ? fine + 1 : p + n;
		nriga++;
		char *c = strchr(riga, '#');
		if (c)
			*c = 0;
		char *salva, *tipo = strtok_r(riga, " \t\r", &salva);
		if (!tipo)
			continue;
		oggetto_t o;
		memset(&o, 0, sizeof o);
		double da_x = 0, da_y = 0, a_x = 0, a_y = 0, spessore = 0.1, x = 0, y = 0, ang = 0;
		double larg = -1, prof = -1, raggio = -1, qda = 0, qa = -1;
		int ha_da = 0, ha_a = 0, in_punti = 0, ha_xy = 0;
		char nome[MONDO_NOME] = "", gruppo[MONDO_NOME] = "";
		for (char *t = strtok_r(NULL, " \t\r", &salva); t; t = strtok_r(NULL, " \t\r", &salva)) {
			char *uguale = strchr(t, '=');
			if (!uguale) {
				if (in_punti && o.n_punti < MONDO_MAX_PUNTI &&
				    coppia(t, &o.px[o.n_punti], &o.py[o.n_punti])) {
					o.n_punti++;
					continue;
				}
				snprintf(errore, dim, "riga %d: \"%s\" non è chiave=valore", nriga, t);
				return -1;
			}
			*uguale = 0;
			const char *k = t, *v = uguale + 1;
			in_punti = 0;
			if (!strcmp(k, "nome")) snprintf(nome, sizeof nome, "%s", v);
			else if (!strcmp(k, "gruppo")) snprintf(gruppo, sizeof gruppo, "%s", v);
			else if (!strcmp(k, "x")) { x = atof(v); ha_xy |= 1; }
			else if (!strcmp(k, "y")) { y = atof(v); ha_xy |= 2; }
			else if (!strcmp(k, "angolo")) ang = atof(v) * GRADI;
			else if (!strcmp(k, "larghezza")) larg = atof(v);
			else if (!strcmp(k, "profondita")) prof = atof(v);
			else if (!strcmp(k, "raggio")) raggio = atof(v);
			else if (!strcmp(k, "quota_da")) qda = atof(v);
			else if (!strcmp(k, "quota_a")) qa = atof(v);
			else if (!strcmp(k, "spessore")) spessore = atof(v);
			else if (!strcmp(k, "da")) ha_da = coppia(v, &da_x, &da_y);
			else if (!strcmp(k, "a")) ha_a = coppia(v, &a_x, &a_y);
			else if (!strcmp(k, "punti")) {
				in_punti = 1;
				if (*v && coppia(v, &o.px[0], &o.py[0]))
					o.n_punti = 1;
			} else {
				snprintf(errore, dim, "riga %d: chiave sconosciuta \"%s\"", nriga, k);
				return -1;
			}
		}
		if (!strcmp(tipo, "stanza")) {
			if (nome[0])
				snprintf(nuovo.nome, sizeof nuovo.nome, "%s", nome);
			nuovo.larghezza = larg;
			nuovo.profondita = prof;
			continue;
		}
		if (!strcmp(tipo, "robot") || !strcmp(tipo, "base")) {
			if (ha_xy != 3) {
				snprintf(errore, dim, "riga %d: %s senza x e y", nriga, tipo);
				return -1;
			}
			if (tipo[0] == 'r') {
				nuovo.robot_x = x, nuovo.robot_y = y, nuovo.robot_dir = ang;
				ha_robot = 1;
			} else {
				nuovo.ha_base = 1;
				nuovo.base_x = x, nuovo.base_y = y, nuovo.base_dir = ang;
			}
			continue;
		}
		if (nuovo.n >= MONDO_MAX_OGGETTI) {
			snprintf(errore, dim, "troppi oggetti (massimo %d)", MONDO_MAX_OGGETTI);
			return -1;
		}
		snprintf(o.nome, sizeof o.nome, "%s", nome[0] ? nome : tipo);
		snprintf(o.gruppo, sizeof o.gruppo, "%s", gruppo);
		o.quota_da = qda;
		if (!strcmp(tipo, "muro")) {
			if (!ha_da || !ha_a || spessore <= 0) {
				snprintf(errore, dim, "riga %d: muro senza da=, a= o spessore", nriga);
				return -1;
			}
			o.forma = OGG_RETTANGOLO;
			o.muro = 1;
			o.x = (da_x + a_x) / 2, o.y = (da_y + a_y) / 2;
			o.angolo = atan2(a_y - da_y, a_x - da_x);
			o.larghezza = hypot(a_x - da_x, a_y - da_y);
			o.profondita = spessore;
			o.quota_a = qa > 0 ? qa : QUOTA_MURO;
		} else if (!strcmp(tipo, "rettangolo")) {
			if (ha_xy != 3 || larg <= 0 || prof <= 0) {
				snprintf(errore, dim, "riga %d: rettangolo senza x, y, larghezza o profondita", nriga);
				return -1;
			}
			o.forma = OGG_RETTANGOLO;
			o.x = x, o.y = y, o.angolo = ang, o.larghezza = larg, o.profondita = prof;
			o.quota_a = qa > 0 ? qa : 0.5;
		} else if (!strcmp(tipo, "cerchio")) {
			if (ha_xy != 3 || raggio <= 0) {
				snprintf(errore, dim, "riga %d: cerchio senza x, y o raggio", nriga);
				return -1;
			}
			o.forma = OGG_CERCHIO;
			o.x = x, o.y = y, o.raggio = raggio;
			o.quota_a = qa > 0 ? qa : 0.5;
		} else if (!strcmp(tipo, "vuoto")) {
			if (o.n_punti < 3) {
				snprintf(errore, dim, "riga %d: vuoto con meno di 3 punti", nriga);
				return -1;
			}
			o.forma = OGG_VUOTO;
			double sx = 0, sy = 0;
			for (int i = 0; i < o.n_punti; i++)
				sx += o.px[i], sy += o.py[i];
			o.x = sx / o.n_punti, o.y = sy / o.n_punti;
		} else {
			snprintf(errore, dim, "riga %d: tipo sconosciuto \"%s\"", nriga, tipo);
			return -1;
		}
		if (o.quota_a <= o.quota_da && o.forma != OGG_VUOTO) {
			snprintf(errore, dim, "riga %d: quota_a deve essere più alta di quota_da", nriga);
			return -1;
		}
		nuovo.o[nuovo.n++] = o;
	}
	if (nuovo.larghezza <= 0 || nuovo.profondita <= 0 || nuovo.larghezza > 50 || nuovo.profondita > 50) {
		snprintf(errore, dim, "manca la riga \"stanza larghezza=... profondita=...\" (da 0 a 50 m)");
		return -1;
	}
	if (!ha_robot) {
		nuovo.robot_x = nuovo.larghezza / 2;
		nuovo.robot_y = nuovo.profondita / 2;
	}
	memcpy(m, &nuovo, sizeof nuovo);
	return 0;
}

int mondo_leggi_file(mondo_t *m, const char *percorso, char *errore, size_t dim)
{
	FILE *f = fopen(percorso, "r");
	if (!f) {
		snprintf(errore, dim, "non riesco ad aprire %s", percorso);
		return -1;
	}
	static char t[1 << 20];
	size_t n = fread(t, 1, sizeof t - 1, f);
	fclose(f);
	t[n] = 0;
	return mondo_leggi_testo(m, t, errore, dim);
}

/* ------------------------------------------------------------------------------------------ scrittura --- */

typedef struct {
	char *b;
	size_t n, cap;
} buf_t;

static void ag(buf_t *s, const char *fmt, ...) __attribute__((format(printf, 2, 3)));
static void ag(buf_t *s, const char *fmt, ...)
{
	if (!s->b)
		return;
	for (;;) {
		va_list ap;
		va_start(ap, fmt);
		int k = vsnprintf(s->b + s->n, s->cap - s->n, fmt, ap);
		va_end(ap);
		if (k >= 0 && (size_t)k < s->cap - s->n) {
			s->n += (size_t)k;
			return;
		}
		char *nb = realloc(s->b, s->cap * 2 + (size_t)(k > 0 ? k : 0));
		if (!nb) {
			free(s->b);
			s->b = NULL;
			return;
		}
		s->b = nb;
		s->cap = s->cap * 2 + (size_t)(k > 0 ? k : 0);
	}
}

/* Numero senza zeri inutili (0.100 -> 0.1). */
static const char *num(char *out, double v)
{
	snprintf(out, 32, "%.4f", v);
	char *p = out + strlen(out) - 1;
	while (*p == '0')
		*p-- = 0;
	if (*p == '.')
		*p = 0;
	if (!strcmp(out, "-0"))
		strcpy(out, "0");
	return out;
}

char *mondo_testo(const mondo_t *m)
{
	buf_t s = { malloc(8192), 0, 8192 };
	char a[32], b[32], c[32], d[32], e[32], f[32], g[32];
	ag(&s, "# stanza del simulatore RTES (formato in sim/mondo.h)\n");
	ag(&s, "stanza nome=%s larghezza=%s profondita=%s\n", m->nome, num(a, m->larghezza), num(b, m->profondita));
	ag(&s, "robot x=%s y=%s angolo=%s\n", num(a, m->robot_x), num(b, m->robot_y), num(c, m->robot_dir / GRADI));
	if (m->ha_base)
		ag(&s, "base x=%s y=%s angolo=%s\n", num(a, m->base_x), num(b, m->base_y), num(c, m->base_dir / GRADI));
	for (int i = 0; i < m->n; i++) {
		const oggetto_t *o = &m->o[i];
		char extra[160] = "";
		size_t k = 0;
		if (o->nome[0] && strcmp(o->nome, o->muro ? "muro" : o->forma == OGG_CERCHIO ? "cerchio" :
		                         o->forma == OGG_VUOTO ? "vuoto" : "rettangolo"))
			k += (size_t)snprintf(extra + k, sizeof extra - k, " nome=%s", o->nome);
		if (o->gruppo[0])
			k += (size_t)snprintf(extra + k, sizeof extra - k, " gruppo=%s", o->gruppo);
		if (o->forma == OGG_VUOTO) {
			ag(&s, "vuoto%s punti=", extra);
			for (int j = 0; j < o->n_punti; j++)
				ag(&s, "%s%s,%s", j ? " " : "", num(a, o->px[j]), num(b, o->py[j]));
			ag(&s, "\n");
		} else if (o->muro) {
			double cx = cos(o->angolo) * o->larghezza / 2, cy = sin(o->angolo) * o->larghezza / 2;
			ag(&s, "muro%s da=%s,%s a=%s,%s spessore=%s", extra, num(a, o->x - cx), num(b, o->y - cy),
			   num(c, o->x + cx), num(d, o->y + cy), num(e, o->profondita));
			if (o->quota_da > 0 || fabs(o->quota_a - QUOTA_MURO) > 1e-9)
				ag(&s, " quota_da=%s quota_a=%s", num(a, o->quota_da), num(b, o->quota_a));
			ag(&s, "\n");
		} else if (o->forma == OGG_RETTANGOLO) {
			ag(&s, "rettangolo%s x=%s y=%s larghezza=%s profondita=%s angolo=%s quota_da=%s quota_a=%s\n", extra,
			   num(a, o->x), num(b, o->y), num(c, o->larghezza), num(d, o->profondita), num(e, o->angolo / GRADI),
			   num(f, o->quota_da), num(g, o->quota_a));
		} else {
			ag(&s, "cerchio%s x=%s y=%s raggio=%s quota_da=%s quota_a=%s\n", extra, num(a, o->x), num(b, o->y),
			   num(c, o->raggio), num(d, o->quota_da), num(e, o->quota_a));
		}
	}
	return s.b;
}

/* ------------------------------------------------------------------------------------------- geometria --- */

/* Il punto nel sistema di un rettangolo centrato in (cx, cy) e ruotato di ang. */
static void locale(double cx, double cy, double ang, double x, double y, double *u, double *v)
{
	double dx = x - cx, dy = y - cy, c = cos(ang), s = sin(ang);
	*u = dx * c + dy * s;
	*v = -dx * s + dy * c;
}

/* Raggio contro un rettangolo [-hu,hu] x [-hv,hv] nel suo sistema. Ritorna la distanza, o INFINITY (anche se il punto
 * di partenza è dentro). */
static double raggio_rett(double u0, double v0, double du, double dv, double hu, double hv)
{
	if (fabs(u0) <= hu && fabs(v0) <= hv)
		return INFINITY;
	double tmin = 0, tmax = INFINITY, o[2] = { u0, v0 }, d[2] = { du, dv }, h[2] = { hu, hv };
	for (int k = 0; k < 2; k++) {
		if (fabs(d[k]) < 1e-12) {
			if (o[k] < -h[k] || o[k] > h[k])
				return INFINITY;
			continue;
		}
		double t1 = (-h[k] - o[k]) / d[k], t2 = (h[k] - o[k]) / d[k];
		if (t1 > t2) {
			double t = t1;
			t1 = t2;
			t2 = t;
		}
		if (t1 > tmin) tmin = t1;
		if (t2 < tmax) tmax = t2;
		if (tmin > tmax)
			return INFINITY;
	}
	return tmin;
}

static double raggio_cerchio(double x, double y, double dx, double dy, double cx, double cy, double r)
{
	double ox = x - cx, oy = y - cy;
	double b = ox * dx + oy * dy, c = ox * ox + oy * oy - r * r;
	if (c <= 0)
		return INFINITY;                 /* dentro */
	double disc = b * b - c;
	if (disc < 0)
		return INFINITY;
	double t = -b - sqrt(disc);
	return t >= 0 ? t : INFINITY;
}

int mondo_in_base(const mondo_t *m, double x, double y)
{
	if (!m->ha_base)
		return 0;
	double u, v;
	locale(m->base_x, m->base_y, m->base_dir, x, y, &u, &v);
	return u <= 0 && u >= -BASE_PROFONDITA && fabs(v) <= BASE_LARGHEZZA / 2;
}

static int dentro_poligono(const double *px, const double *py, int n, double x, double y)
{
	int dentro = 0;
	for (int i = 0, j = n - 1; i < n; j = i++)
		if ((py[i] > y) != (py[j] > y) && x < (px[j] - px[i]) * (y - py[i]) / (py[j] - py[i]) + px[i])
			dentro = !dentro;
	return dentro;
}

int mondo_dentro(const oggetto_t *o, double x, double y)
{
	if (o->forma == OGG_VUOTO)
		return dentro_poligono(o->px, o->py, o->n_punti, x, y);
	if (o->forma == OGG_CERCHIO)
		return hypot(x - o->x, y - o->y) <= o->raggio;
	double u, v;
	locale(o->x, o->y, o->angolo, x, y, &u, &v);
	return fabs(u) <= o->larghezza / 2 && fabs(v) <= o->profondita / 2;
}

int mondo_vuoto(const mondo_t *m, double x, double y)
{
	for (int i = 0; i < m->n; i++)
		if (m->o[i].forma == OGG_VUOTO && dentro_poligono(m->o[i].px, m->o[i].py, m->o[i].n_punti, x, y))
			return 1;
	return 0;
}

double mondo_raggio(const mondo_t *m, double x, double y, double ang, double quota, double max_m)
{
	double dx = cos(ang), dy = sin(ang), best = INFINITY;
	for (int i = 0; i < m->n; i++) {
		const oggetto_t *o = &m->o[i];
		if (o->forma == OGG_VUOTO || quota < o->quota_da || quota > o->quota_a)
			continue;
		double t;
		if (o->forma == OGG_CERCHIO) {
			t = raggio_cerchio(x, y, dx, dy, o->x, o->y, o->raggio);
		} else {
			double u, v;
			locale(o->x, o->y, o->angolo, x, y, &u, &v);
			double c = cos(o->angolo), s = sin(o->angolo);
			t = raggio_rett(u, v, dx * c + dy * s, -dx * s + dy * c, o->larghezza / 2, o->profondita / 2);
		}
		if (t < best)
			best = t;
	}
	if (m->ha_base && quota <= BASE_QUOTA) {
		double u, v, c = cos(m->base_dir), s = sin(m->base_dir);
		locale(m->base_x, m->base_y, m->base_dir, x, y, &u, &v);
		/* il blocco della base va da u = -profondità a u = 0: centro a -profondità/2 */
		double t = raggio_rett(u + BASE_PROFONDITA / 2, v, dx * c + dy * s, -dx * s + dy * c, BASE_PROFONDITA / 2,
		                       BASE_LARGHEZZA / 2);
		if (t < best)
			best = t;
	}
	/* bordi della stanza */
	double tb = INFINITY;
	if (dx > 1e-12) tb = fmin(tb, (m->larghezza - x) / dx);
	if (dx < -1e-12) tb = fmin(tb, -x / dx);
	if (dy > 1e-12) tb = fmin(tb, (m->profondita - y) / dy);
	if (dy < -1e-12) tb = fmin(tb, -y / dy);
	if (tb >= 0 && tb < best)
		best = tb;
	return best <= max_m ? best : max_m + 1;
}

static void contatto(double cx, double cy, double px, double py, double *sx, double *sy, int *n)
{
	double dx = px - cx, dy = py - cy, d = hypot(dx, dy);
	if (d > 1e-9) {
		*sx += dx / d;
		*sy += dy / d;
	}
	(*n)++;
}

int mondo_urto(const mondo_t *m, double x, double y, double r, double z0, double z1, double *ang_contatto)
{
	double sx = 0, sy = 0;
	int n = 0;
	for (int i = 0; i < m->n; i++) {
		const oggetto_t *o = &m->o[i];
		if (o->forma == OGG_VUOTO || o->quota_a < QUOTA_MINIMA || o->quota_da >= z1 || o->quota_a <= z0)
			continue;
		if (o->forma == OGG_CERCHIO) {
			if (hypot(x - o->x, y - o->y) < r + o->raggio)
				contatto(x, y, o->x, o->y, &sx, &sy, &n);
			continue;
		}
		double u, v;
		locale(o->x, o->y, o->angolo, x, y, &u, &v);
		double cu = fmax(-o->larghezza / 2, fmin(u, o->larghezza / 2));
		double cv = fmax(-o->profondita / 2, fmin(v, o->profondita / 2));
		if (hypot(u - cu, v - cv) < r) {
			double c = cos(o->angolo), s = sin(o->angolo);
			if (fabs(u - cu) < 1e-12 && fabs(v - cv) < 1e-12)       /* centro dentro: verso il centro dell'oggetto */
				contatto(x, y, o->x, o->y, &sx, &sy, &n);
			else
				contatto(x, y, o->x + cu * c - cv * s, o->y + cu * s + cv * c, &sx, &sy, &n);
		}
	}
	if (m->ha_base && z0 < BASE_QUOTA) {
		double u, v, c = cos(m->base_dir), s = sin(m->base_dir);
		locale(m->base_x, m->base_y, m->base_dir, x, y, &u, &v);
		double cu = fmax(-BASE_PROFONDITA, fmin(u, 0)), cv = fmax(-BASE_LARGHEZZA / 2, fmin(v, BASE_LARGHEZZA / 2));
		if (hypot(u - cu, v - cv) < r)
			contatto(x, y, m->base_x + cu * c - cv * s, m->base_y + cu * s + cv * c, &sx, &sy, &n);
	}
	if (x - r < 0) contatto(x, y, 0, y, &sx, &sy, &n);
	if (x + r > m->larghezza) contatto(x, y, m->larghezza, y, &sx, &sy, &n);
	if (y - r < 0) contatto(x, y, x, 0, &sx, &sy, &n);
	if (y + r > m->profondita) contatto(x, y, x, m->profondita, &sx, &sy, &n);
	if (!n)
		return 0;
	if (ang_contatto)
		*ang_contatto = atan2(sy, sx);
	return 1;
}
