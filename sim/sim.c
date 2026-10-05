/*
 * sim.c - Simulatore del robot: GD32 e lidar finti su due pseudo-seriali, stanza a oggetti, pagina web.
 *
 * Un programma che parla con il robot (il driver, i task) gira sul PC SENZA modifiche: invece di /dev/ttyS3 e
 * /dev/ttyS1 apre le due pseudo-seriali del simulatore, con nomi fissi:
 *     build/sim/gd32    -> il GD32 simulato (115200, frame FA FB ...)
 *     build/sim/lidar   -> il lidar simulato (pacchetti AA ...)
 * (./rtes.sh sim lo fa da solo con RTES_GD32 e RTES_LIDAR). La pagina http://localhost:8080 mostra la stanza, il robot
 * e il lidar, permette di costruire la stanza (editor a oggetti) e di ispezionare ogni messaggio scambiato.
 *
 * Fedeltà: geometria in sim/robot.h, dinamica tarata in sim/parametri.txt (tools/taratura/tara.py). I frame arrivano
 * al SoC con il ritardo misurato sul robot (intervalli fra ~17 e ~25 ms invece di 20,0 esatti).
 *
 * HTTP: GET / e i file di sim/web; GET /stato (JSON); GET /stanza (testo) e POST /stanza (sostituisce la stanza);
 *       GET /stanze; POST /comando (una riga: vedi esegui_comando); GET /campi (descrizione dei messaggi, JSON);
 *       GET /storia?dopo=N (messaggi registrati dopo il numero N, JSON).
 *
 * Uso: build/host/sim [opzioni] [stanza.txt]
 *   --porta N            porta della pagina (default 8080; 0 = niente pagina)
 *   --ascolta IP         indirizzo su cui ascoltare (default 127.0.0.1; 0.0.0.0 per gli altri PC della rete)
 *   --dir DIR            dove creare i collegamenti gd32 e lidar (default build/sim)
 *   --stanze DIR         cartella delle stanze (default sim/stanze)
 *   --web DIR            cartella della pagina (default sim/web)
 *   --parametri FILE     parametri tarati (default sim/parametri.txt; senza file valgono i default)
 *   --batteria-veloce K  la batteria si scarica e si carica K volte più in fretta (default 1)
 *   --scarto-lidar G     scarto in gradi fra lo 0 del lidar e il robot (default 14,5, come il robot vero)
 *   --timeout-keepalive S  secondi senza keepalive 0x0d prima del reset del GD32 (default 190)
 *   --senza-ritardi      consegna i frame senza il ritardo misurato
 *   --durata S           esce dopo S secondi (per i test)
 *   --stampa             una riga di stato al secondo sul terminale
 *   --seme N             seme del rumore
 */
#define _GNU_SOURCE
#include <arpa/inet.h>
#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <math.h>
#include <netinet/in.h>
#include <pthread.h>
#include <signal.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#include "../gd32/descrizione.h"
#include "../gd32/gd32.h"
#include "gd32sim.h"
#include "lidarsim.h"
#include "mondo.h"
#include "parametri.h"
#include "robot.h"

static pthread_mutex_t mx = PTHREAD_MUTEX_INITIALIZER;
static mondo_t mondo;
static gd32sim_t g;
static lidarsim_t l;
static double t0;
static const char *dir_stanze = "sim/stanze", *dir_web = "sim/web";
static char stanza_corrente[600] = "(nuova)";
static volatile sig_atomic_t fermati;

static void al_segnale(int s) { (void)s; fermati = 1; }

static double adesso(void)
{
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return ts.tv_sec + ts.tv_nsec / 1e9;
}

/* --------------------------------------------------------------------------------------- storia --- */
/* Tutti i messaggi, nelle due direzioni e del lidar, per l'ispettore della pagina. ~80 s. */

enum { CAN_GD32_SOC = 0, CAN_SOC_GD32 = 1, CAN_LIDAR = 2 };
#define STORIA_N 16384

typedef struct {
	uint64_t n;
	double t;
	uint8_t canale, len;
	uint8_t b[110];
} voce_t;

static voce_t storia[STORIA_N];
static uint64_t storia_n;                 /* voci registrate in tutto; la voce k sta in storia[k % STORIA_N] */

static void registra(double t, int canale, const uint8_t *b, size_t n)
{
	voce_t *v = &storia[storia_n % STORIA_N];
	v->n = storia_n++;
	v->t = t;
	v->canale = (uint8_t)canale;
	v->len = (uint8_t)(n > sizeof v->b ? sizeof v->b : n);
	memcpy(v->b, b, v->len);
}

/* ------------------------------------------------------------------------------------------- pty --- */

typedef struct {
	int master, slave;      /* il simulatore tiene aperto anche lo slave: chi lo apre e lo chiude non lo rompe */
	char nome[64];
	uint64_t scartati;      /* byte buttati perché nessuno leggeva */
} pty_t;

static int apri_pty(pty_t *p, const char *collegamento)
{
	p->master = posix_openpt(O_RDWR | O_NOCTTY);
	if (p->master < 0 || grantpt(p->master) || unlockpt(p->master) || ptsname_r(p->master, p->nome, sizeof p->nome))
		return -1;
	p->slave = open(p->nome, O_RDWR | O_NOCTTY);
	if (p->slave < 0)
		return -1;
	struct termios t;
	tcgetattr(p->slave, &t);
	cfmakeraw(&t);
	cfsetispeed(&t, B115200);
	cfsetospeed(&t, B115200);
	tcsetattr(p->slave, TCSANOW, &t);
	fcntl(p->master, F_SETFL, O_NONBLOCK);
	unlink(collegamento);
	if (symlink(p->nome, collegamento)) {
		fprintf(stderr, "sim: non riesco a creare %s: %s\n", collegamento, strerror(errno));
		return -1;
	}
	return 0;
}

static void scrivi_pty(pty_t *p, const uint8_t *b, size_t n)
{
	ssize_t k = write(p->master, b, n);
	if (k < (ssize_t)n)
		p->scartati += n - (size_t)(k > 0 ? k : 0);   /* nessuno legge: i dati vecchi non servono */
}

/* -------------------------------------------------------------------------------------- comandi --- */

static int nome_valido(const char *s)
{
	if (!*s || strlen(s) > 60)
		return 0;
	for (; *s; s++)
		if (!((*s >= 'a' && *s <= 'z') || (*s >= 'A' && *s <= 'Z') || (*s >= '0' && *s <= '9') || *s == '-' || *s == '_'))
			return 0;
	return 1;
}

static int carica_stanza(const char *nome, char *err, size_t dim)
{
	char p[700];
	snprintf(p, sizeof p, "%s/%s.txt", dir_stanze, nome);
	if (mondo_leggi_file(&mondo, p, err, dim))
		return -1;
	snprintf(stanza_corrente, sizeof stanza_corrente, "%s", nome);
	gd32sim_sposta(&g, mondo.robot_x, mondo.robot_y, mondo.robot_dir);
	return 0;
}

/* Un comando della pagina. Ritorna 0 e scrive "ok" o un messaggio in risp. Va chiamata con mx preso. */
static int esegui_comando(const char *c, char *risp, size_t dim)
{
	double a, b, d;
	char nome[61];
	int i;
	double t = adesso() - t0;
	snprintf(risp, dim, "ok");
	if (!strcmp(c, "solleva")) {
		g.sollevato = 1;
	} else if (!strcmp(c, "posa")) {
		g.sollevato = 0;
	} else if (sscanf(c, "sposta %lf %lf %lf", &a, &b, &d) == 3) {
		gd32sim_sposta(&g, a, b, d * M_PI / 180);
	} else if (sscanf(c, "cassetto %d", &i) == 1) {
		g.cassetto = i != 0;
	} else if (sscanf(c, "tasto accensione %d", &i) == 1) {
		g.tasto_da[0] = i ? t : -1;
	} else if (sscanf(c, "tasto casa %d", &i) == 1) {
		g.tasto_da[1] = i ? t : -1;
	} else if (sscanf(c, "batteria %lf", &a) == 1) {
		g.soc = fmax(0, fmin(1, a / 100));
	} else if (sscanf(c, "nuova %lf %lf", &a, &b) == 2) {
		if (a < 0.5 || b < 0.5 || a > 50 || b > 50) {
			snprintf(risp, dim, "dimensioni non valide (da 0,5 a 50 m)");
			return -1;
		}
		mondo_nuovo(&mondo, a, b);
		snprintf(stanza_corrente, sizeof stanza_corrente, "(nuova)");
		gd32sim_sposta(&g, mondo.robot_x, mondo.robot_y, mondo.robot_dir);
	} else if (sscanf(c, "salva %60s", nome) == 1) {
		if (!nome_valido(nome)) {
			snprintf(risp, dim, "nome non valido: solo lettere, cifre, - e _");
			return -1;
		}
		char p[700];
		snprintf(p, sizeof p, "%s/%s.txt", dir_stanze, nome);
		snprintf(mondo.nome, sizeof mondo.nome, "%s", nome);
		char *testo = mondo_testo(&mondo);
		FILE *f = testo ? fopen(p, "w") : NULL;
		int ok = f && fputs(testo, f) >= 0;
		if (f)
			fclose(f);
		free(testo);
		if (!ok) {
			snprintf(risp, dim, "non riesco a scrivere %s", p);
			return -1;
		}
		snprintf(stanza_corrente, sizeof stanza_corrente, "%s", nome);
		snprintf(risp, dim, "salvata in %s", p);
	} else if (sscanf(c, "carica %60s", nome) == 1) {
		char err[600];
		if (!nome_valido(nome)) {
			snprintf(risp, dim, "nome non valido");
			return -1;
		}
		if (carica_stanza(nome, err, sizeof err)) {
			snprintf(risp, dim, "%s", err);
			return -1;
		}
	} else {
		snprintf(risp, dim, "comando sconosciuto: %s", c);
		return -1;
	}
	return 0;
}

/* ------------------------------------------------------------------------------------------ json --- */

typedef struct {
	char *b;
	size_t n, cap;
} testo_t;

static void ag(testo_t *s, const char *fmt, ...) __attribute__((format(printf, 2, 3)));
static void ag(testo_t *s, const char *fmt, ...)
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
		size_t nuova = s->cap * 2 + (size_t)(k > 0 ? k : 0);
		char *p = realloc(s->b, nuova);
		if (!p) {
			free(s->b);
			s->b = NULL;
			return;
		}
		s->b = p;
		s->cap = nuova;
	}
}

/* Stringa JSON con le virgolette e i caratteri speciali protetti. */
static void ag_stringa(testo_t *s, const char *t)
{
	ag(s, "\"");
	for (; t && *t; t++) {
		if (*t == '"' || *t == '\\')
			ag(s, "\\%c", *t);
		else if ((unsigned char)*t < 32)
			ag(s, " ");
		else
			ag(s, "%c", *t);
	}
	ag(s, "\"");
}

static void stato_json(testo_t *s)
{
	double t = adesso() - t0, v, w;
	gd32sim_velocita(&g, &v, &w);
	ag(s, "{\"t\":%.3f,\"stanza\":", t);
	ag_stringa(s, stanza_corrente);
	ag(s, ",\"robot\":{\"x\":%.4f,\"y\":%.4f,\"dir\":%.2f},", g.x, g.y, g.dir * 180 / M_PI);
	ag(s, "\"geometria\":{\"raggio\":%.4f,\"torretta\":%.3f,\"lidar\":[%.3f,%.3f],\"cliff_distanza\":%.3f,\"cliff_angoli\":[",
	   ROBOT_RAGGIO, TORRETTA_RAGGIO, LIDAR_X, LIDAR_Y, CLIFF_DISTANZA);
	for (int k = 0; k < 4; k++)
		ag(s, "%s%.1f", k ? "," : "", CLIFF_ANGOLI[k] * 180 / M_PI);
	ag(s, "],\"bumper_arco\":%.1f,\"contatti_dietro\":%d,\"base\":[%.3f,%.3f]},", BUMPER_MEZZO_ARCO * 180 / M_PI,
	   CONTATTI_DIETRO, BASE_LARGHEZZA, BASE_PROFONDITA);
	ag(s, "\"v\":%.3f,\"w\":%.3f,\"sollevato\":%d,\"caduto\":%d,\"bloccato\":%d,\"incastrato\":%d,\"bumper\":[%d,%d],"
	      "\"cliff\":%d,\"ir_base\":%d,", v, w, g.sollevato, g.caduto, g.bloccato, g.incastrato, g.bumper_sx, g.bumper_dx,
	   g.cliff, g.ir_base);
	ag(s, "\"contatto_base\":%d,\"in_carica\":%d,\"batteria\":{\"v\":%.2f,\"soc\":%.1f,\"i\":%.2f},\"cassetto\":%d,",
	   g.contatto_base, g.in_carica, g.tensione, g.soc * 100, g.corrente, g.cassetto);
	ag(s, "\"led\":%d,\"modo\":%d,\"cmd_v\":%.3f,\"cmd_w\":%.3f,\"spazzole\":[%d,%d],\"ventola\":%.0f,\"imbardata\":%.1f,",
	   g.led, g.modo, g.cmd_v / SCALA_V, g.cmd_w / SCALA_W, g.eco_lat, g.eco_cen, g.ventola_giri, g.imbardata * 180 / M_PI);
	ag(s, "\"gd32\":{\"acceso\":%d,\"sospeso\":%d,\"motori\":%d,\"senza_hb\":%.1f,\"senza_ka\":%.1f,\"reset\":%llu,"
	      "\"rx\":%llu,\"sensori\":%llu,\"sensori_accesi\":%d,\"cliff_retromarcia\":%d,\"persi\":%llu},",
	   g.acceso, g.sospeso, g.motori_alimentati, fmin(999, t - g.ultimo_hb), t - g.ultimo_ka,
	   (unsigned long long)g.reset, (unsigned long long)g.frame_ricevuti, (unsigned long long)g.frame_sensori,
	   g.sensori_accesi, g.cliff_retromarcia, (unsigned long long)g.persi);
	ag(s, "\"lidar\":{\"alimentato\":%d,\"potenza\":%d,\"giri_s\":%.2f,\"giri\":%llu,\"punti\":[", l.alimentato,
	   g.lidar_potenza, l.giri_s, (unsigned long long)l.giri);
	for (int i = 0; i < l.n_giro; i++)
		ag(s, "%s%.3f,%.3f", i ? "," : "", l.giro_x[i], l.giro_y[i]);
	ag(s, "]},\"avviso\":");
	ag_stringa(s, g.avviso);
	ag(s, ",\"t_avviso\":%.1f,\"storia_n\":%llu}", g.t_avviso, (unsigned long long)storia_n);
}

static void campo_json(testo_t *s, const campo_t *c)
{
	ag(s, "{\"nome\":");
	ag_stringa(s, c->nome);
	ag(s, ",\"byte\":%d,\"n_byte\":%d,\"bit\":%d,\"n_bit\":%d,\"tipo\":\"%s\",\"scala\":%.10g,\"unita\":", c->byte,
	   c->n_byte, c->bit, c->n_bit, c->tipo, c->scala);
	ag_stringa(s, c->unita);
	ag(s, ",\"aff\":\"%c\",\"gruppo\":%d,\"spiegazione\":", c->affidabilita, c->gruppo);
	ag_stringa(s, c->spiegazione);
	ag(s, "}");
}

static void campi_json(testo_t *s)
{
	ag(s, "{\"messaggi\":[");
	for (int i = 0; i < N_MESSAGGI; i++) {
		const messaggio_t *m = &MESSAGGI[i];
		ag(s, "%s{\"cmd\":%d,\"nome\":", i ? "," : "", m->cmd);
		ag_stringa(s, m->nome);
		ag(s, ",\"verso_gd32\":%d,\"spiegazione\":", m->verso_gd32);
		ag_stringa(s, m->spiegazione);
		ag(s, ",\"campi\":[");
		for (int k = 0; k < m->n_campi; k++) {
			if (k)
				ag(s, ",");
			campo_json(s, &m->campi[k]);
		}
		ag(s, "]}");
	}
	ag(s, "],\"nomi\":{");
	int primo = 1;
	for (int c = 0; c < 256; c++) {
		const char *n = gd32_nome_cmd((uint8_t)c);
		if (n) {
			ag(s, "%s\"%d\":\"%s\"", primo ? "" : ",", c, n);
			primo = 0;
		}
	}
	ag(s, "}}");
}

static void storia_json(testo_t *s, uint64_t dopo)
{
	uint64_t primo = storia_n > STORIA_N ? storia_n - STORIA_N : 0;
	uint64_t da = dopo + 1 > primo ? dopo + 1 : primo;
	if (dopo == UINT64_MAX)
		da = primo;
	if (storia_n - da > 4000)
		da = storia_n - 4000;              /* al massimo 4000 per volta: la pagina richiede il resto */
	ag(s, "{\"primo\":%llu,\"ultimo\":%lld,\"voci\":[", (unsigned long long)primo, (long long)storia_n - 1);
	for (uint64_t k = da; k < storia_n; k++) {
		const voce_t *v = &storia[k % STORIA_N];
		ag(s, "%s[%llu,%.4f,%d,\"", k == da ? "" : ",", (unsigned long long)v->n, v->t, v->canale);
		for (int i = 0; i < v->len; i++)
			ag(s, "%02x", v->b[i]);
		ag(s, "\"]");
	}
	ag(s, "]}");
}

/* ------------------------------------------------------------------------------------------ http --- */

static void rispondi(int fd, const char *stato, const char *tipo, const char *corpo, size_t n)
{
	char h[256];
	int k = snprintf(h, sizeof h, "HTTP/1.1 %s\r\nContent-Type: %s\r\nContent-Length: %zu\r\nCache-Control: no-store\r\n"
	                 "Connection: close\r\n\r\n", stato, tipo, n);
	if (write(fd, h, (size_t)k) < 0)
		return;
	size_t fatti = 0;
	while (fatti < n) {
		ssize_t w = write(fd, corpo + fatti, n - fatti);
		if (w <= 0)
			return;
		fatti += (size_t)w;
	}
}

static void rispondi_testo(int fd, const char *stato, const char *testo)
{
	rispondi(fd, stato, "text/plain; charset=utf-8", testo, strlen(testo));
}

static char *leggi_file(const char *p, size_t *n)
{
	FILE *f = fopen(p, "rb");
	if (!f)
		return NULL;
	fseek(f, 0, SEEK_END);
	long dim = ftell(f);
	fseek(f, 0, SEEK_SET);
	char *b = malloc((size_t)dim + 1);
	if (b && fread(b, 1, (size_t)dim, f) != (size_t)dim) {
		free(b);
		b = NULL;
	}
	fclose(f);
	if (b) {
		b[dim] = 0;
		*n = (size_t)dim;
	}
	return b;
}

/* File statico della pagina: solo nomi semplici dentro dir_web (niente ".." né sottocartelle). */
static void servi_file(int fd, const char *nome)
{
	for (const char *p = nome; *p; p++)
		if (!((*p >= 'a' && *p <= 'z') || (*p >= '0' && *p <= '9') || *p == '-' || *p == '_' || *p == '.')) {
			rispondi_testo(fd, "404 Not Found", "");
			return;
		}
	if (nome[0] == '.') {
		rispondi_testo(fd, "404 Not Found", "");
		return;
	}
	const char *est = strrchr(nome, '.');
	const char *tipo = !est ? NULL : !strcmp(est, ".html") ? "text/html; charset=utf-8" :
	                   !strcmp(est, ".js") ? "text/javascript; charset=utf-8" :
	                   !strcmp(est, ".css") ? "text/css; charset=utf-8" : NULL;
	char p[700];
	snprintf(p, sizeof p, "%s/%s", dir_web, nome);
	size_t k;
	char *b = tipo ? leggi_file(p, &k) : NULL;
	if (!b) {
		rispondi_testo(fd, "404 Not Found", "file non trovato: avvia il simulatore dalla cartella della repository");
		return;
	}
	rispondi(fd, "200 OK", tipo, b, k);
	free(b);
}

static void servi_json(int fd, void (*f)(testo_t *))
{
	testo_t s = { malloc(65536), 0, 65536 };
	pthread_mutex_lock(&mx);
	f(&s);
	pthread_mutex_unlock(&mx);
	if (s.b)
		rispondi(fd, "200 OK", "application/json", s.b, s.n);
	free(s.b);
}

static void servi(int fd)
{
	static char req[1 << 20];
	size_t n = 0;
	char *fine_testa = NULL;
	struct timeval to = { .tv_sec = 2 };
	setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &to, sizeof to);
	while (n < sizeof req - 1) {
		ssize_t k = read(fd, req + n, sizeof req - 1 - n);
		if (k <= 0)
			break;
		n += (size_t)k;
		req[n] = 0;
		if (!fine_testa)
			fine_testa = strstr(req, "\r\n\r\n");
		if (fine_testa) {
			char *cl = strcasestr(req, "Content-Length:");
			size_t lung = cl && cl < fine_testa ? (size_t)atol(cl + 15) : 0;
			if (n >= (size_t)(fine_testa + 4 - req) + lung)
				break;
		}
	}
	if (!fine_testa) {
		rispondi_testo(fd, "400 Bad Request", "");
		return;
	}
	*fine_testa = 0;
	char *corpo = fine_testa + 4;
	char metodo[8] = "", percorso[256] = "";
	sscanf(req, "%7s %255s", metodo, percorso);
	char *domanda = strchr(percorso, '?');
	if (domanda)
		*domanda++ = 0;
	int get = !strcmp(metodo, "GET"), post = !strcmp(metodo, "POST");

	if (get && !strcmp(percorso, "/")) {
		servi_file(fd, "index.html");
	} else if (get && !strcmp(percorso, "/stato")) {
		servi_json(fd, stato_json);
	} else if (get && !strcmp(percorso, "/campi")) {
		servi_json(fd, campi_json);
	} else if (get && !strcmp(percorso, "/storia")) {
		uint64_t dopo = UINT64_MAX;
		if (domanda && !strncmp(domanda, "dopo=", 5) && domanda[5] != '-')
			dopo = strtoull(domanda + 5, NULL, 10);
		testo_t s = { malloc(1 << 20), 0, 1 << 20 };
		pthread_mutex_lock(&mx);
		storia_json(&s, dopo);
		pthread_mutex_unlock(&mx);
		if (s.b)
			rispondi(fd, "200 OK", "application/json", s.b, s.n);
		free(s.b);
	} else if (get && !strcmp(percorso, "/stanza")) {
		pthread_mutex_lock(&mx);
		char *t = mondo_testo(&mondo);
		pthread_mutex_unlock(&mx);
		if (t) {
			rispondi_testo(fd, "200 OK", t);
			free(t);
		}
	} else if (get && !strcmp(percorso, "/stanze")) {
		testo_t s = { malloc(4096), 0, 4096 };
		ag(&s, "[");
		DIR *d = opendir(dir_stanze);
		struct dirent *e;
		int primo = 1;
		while (d && (e = readdir(d))) {
			size_t k = strlen(e->d_name);
			if (k > 4 && !strcmp(e->d_name + k - 4, ".txt")) {
				ag(&s, "%s\"%.*s\"", primo ? "" : ",", (int)(k - 4), e->d_name);
				primo = 0;
			}
		}
		if (d)
			closedir(d);
		ag(&s, "]");
		if (s.b)
			rispondi(fd, "200 OK", "application/json", s.b, s.n);
		free(s.b);
	} else if (post && !strcmp(percorso, "/stanza")) {
		char err[600];
		pthread_mutex_lock(&mx);
		int r = mondo_leggi_testo(&mondo, corpo, err, sizeof err);
		if (!r && domanda && strstr(domanda, "riposiziona=1"))
			gd32sim_sposta(&g, mondo.robot_x, mondo.robot_y, mondo.robot_dir);
		pthread_mutex_unlock(&mx);
		rispondi_testo(fd, r ? "400 Bad Request" : "200 OK", r ? err : "ok");
	} else if (post && !strcmp(percorso, "/comando")) {
		char risp[800];
		corpo[strcspn(corpo, "\r\n")] = 0;
		pthread_mutex_lock(&mx);
		int r = esegui_comando(corpo, risp, sizeof risp);
		pthread_mutex_unlock(&mx);
		rispondi_testo(fd, r ? "400 Bad Request" : "200 OK", risp);
	} else if (get && percorso[0] == '/' && !strchr(percorso + 1, '/')) {
		servi_file(fd, percorso + 1);
	} else {
		rispondi_testo(fd, "404 Not Found", "");
	}
}

static void *thread_web(void *arg)
{
	int s = *(int *)arg;
	while (!fermati) {
		int c = accept(s, NULL, NULL);
		if (c < 0)
			continue;
		servi(c);
		close(c);
	}
	return NULL;
}

/* ------------------------------------------------------------------------------------------ main --- */

static void ricevuto(const gd32_frame_t *f, void *ctx)
{
	double t = *(const double *)ctx;
	uint8_t b[GD32_MAX_FRAME];
	size_t k = gd32_componi(f->cmd, f->dati, f->n_dati, b, sizeof b);
	registra(t, CAN_SOC_GD32, b, k);
	gd32sim_ricevi(&g, f, t);
}

static void stampa_stato(double t)
{
	printf("%8.1f s  x %.2f y %.2f dir %4.0f°  modo %d  bump %d%d cliff %x base %d/%d  %.1f V  led %2d  "
	       "lidar %.1f giri/s  rx %llu  hb %.1f s fa  ka %.1f s fa%s%s%s\n",
	       t, g.x, g.y, g.dir * 180 / M_PI, g.modo, g.bumper_sx, g.bumper_dx, g.cliff, g.contatto_base, g.in_carica,
	       g.tensione, g.led, l.giri_s, (unsigned long long)g.frame_ricevuti, fmin(999, t - g.ultimo_hb),
	       t - g.ultimo_ka, g.reset ? "  RESET!" : "", g.sollevato ? "  sollevato" : "", g.incastrato ? "  incastrato" : "");
	fflush(stdout);
}

int main(int argc, char **argv)
{
	int porta = 8080, durata = 0, stampa = 0, senza_ritardi = 0;
	const char *ascolta = "127.0.0.1", *dir = "build/sim", *file_par = "sim/parametri.txt";
	double batteria = 1, scarto = LIDAR_SCARTO_CDEG / 100.0, timeout_ka = 190;
	uint32_t seme = 12345;
	static const struct option opz[] = {
		{ "porta", required_argument, 0, 'p' }, { "ascolta", required_argument, 0, 'a' },
		{ "dir", required_argument, 0, 'd' }, { "stanze", required_argument, 0, 's' },
		{ "web", required_argument, 0, 'W' }, { "parametri", required_argument, 0, 'P' },
		{ "batteria-veloce", required_argument, 0, 'b' }, { "scarto-lidar", required_argument, 0, 'o' },
		{ "timeout-keepalive", required_argument, 0, 'k' }, { "senza-ritardi", no_argument, 0, 'R' },
		{ "durata", required_argument, 0, 't' }, { "stampa", no_argument, 0, 'S' },
		{ "seme", required_argument, 0, 'r' }, { 0, 0, 0, 0 },
	};
	int o;
	while ((o = getopt_long(argc, argv, "", opz, NULL)) != -1) {
		switch (o) {
		case 'p': porta = atoi(optarg); break;
		case 'a': ascolta = optarg; break;
		case 'd': dir = optarg; break;
		case 's': dir_stanze = optarg; break;
		case 'W': dir_web = optarg; break;
		case 'P': file_par = optarg; break;
		case 'b': batteria = atof(optarg); break;
		case 'o': scarto = atof(optarg); break;
		case 'k': timeout_ka = atof(optarg); break;
		case 'R': senza_ritardi = 1; break;
		case 't': durata = atoi(optarg); break;
		case 'S': stampa = 1; break;
		case 'r': seme = (uint32_t)strtoul(optarg, NULL, 0); break;
		default:
			fprintf(stderr, "uso: sim [opzioni] [stanza.txt] (vedi sim/sim.c)\n");
			return 2;
		}
	}
	parametri_frame_default();
	if (parametri_leggi(file_par))
		fprintf(stderr, "sim: %s non trovato, uso i parametri di default\n", file_par);
	char err[600];
	if (optind < argc) {
		if (mondo_leggi_file(&mondo, argv[optind], err, sizeof err)) {
			fprintf(stderr, "sim: %s: %s\n", argv[optind], err);
			return 1;
		}
		const char *b = strrchr(argv[optind], '/');
		snprintf(stanza_corrente, sizeof stanza_corrente, "%s", b ? b + 1 : argv[optind]);
		char *punto = strrchr(stanza_corrente, '.');
		if (punto)
			*punto = 0;
	} else {
		mondo_nuovo(&mondo, 4, 3);
	}

	mkdir(dir, 0755);
	char p_gd32[512], p_lidar[512];
	snprintf(p_gd32, sizeof p_gd32, "%s/gd32", dir);
	snprintf(p_lidar, sizeof p_lidar, "%s/lidar", dir);
	pty_t pg, pl;
	if (apri_pty(&pg, p_gd32) || apri_pty(&pl, p_lidar)) {
		fprintf(stderr, "sim: non riesco a creare le pseudo-seriali: %s\n", strerror(errno));
		return 1;
	}

	t0 = adesso();
	g.seme = seme;
	g.velocita_batteria = batteria;
	g.timeout_keepalive = timeout_ka;
	g.senza_ritardi = senza_ritardi;
	gd32sim_init(&g, &mondo, 0);
	l.scarto_gradi = scarto;
	l.seme = seme * 7 + 1;
	l.senza_ritardi = senza_ritardi;
	lidarsim_init(&l, 0);

	struct sigaction sa = { .sa_handler = al_segnale };
	sigaction(SIGINT, &sa, NULL);
	sigaction(SIGTERM, &sa, NULL);
	signal(SIGPIPE, SIG_IGN);

	static int sock = -1;
	if (porta > 0) {
		sock = socket(AF_INET, SOCK_STREAM, 0);
		int uno = 1;
		setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &uno, sizeof uno);
		struct sockaddr_in a = { .sin_family = AF_INET, .sin_port = htons((uint16_t)porta) };
		inet_pton(AF_INET, ascolta, &a.sin_addr);
		if (bind(sock, (struct sockaddr *)&a, sizeof a) || listen(sock, 16)) {
			fprintf(stderr, "sim: porta %d occupata o non valida (%s): uso --porta\n", porta, strerror(errno));
			return 1;
		}
		pthread_t tw;
		pthread_create(&tw, NULL, thread_web, &sock);
		pthread_detach(tw);
	}
	printf("sim: stanza %s (%.1f x %.1f m, %d oggetti)\n", stanza_corrente, mondo.larghezza, mondo.profondita, mondo.n);
	printf("sim: GD32 su %s (%s), lidar su %s (%s)\n", p_gd32, pg.nome, p_lidar, pl.nome);
	if (porta > 0)
		printf("sim: pagina su http://%s:%d\n", strcmp(ascolta, "0.0.0.0") ? ascolta : "localhost", porta);
	fflush(stdout);

	gd32_deframer_t dfr;
	gd32_deframer_init(&dfr);
	double prossimo_passo = 0.002, prossima_stampa = 1;
	while (!fermati) {
		/* dorme fino al prossimo passo della fisica (2 ms) o alla prossima consegna, se viene prima */
		pthread_mutex_lock(&mx);
		double sveglia = fmin(prossimo_passo, fmin(gd32sim_prossima_consegna(&g), lidarsim_prossima_consegna(&l)));
		pthread_mutex_unlock(&mx);
		double ass = t0 + sveglia;
		struct timespec ts = { (time_t)ass, (long)((ass - floor(ass)) * 1e9) };
		clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, NULL);
		double t = adesso() - t0;
		if (durata > 0 && t >= durata)
			break;
		uint8_t buf[4096];
		ssize_t k;
		pthread_mutex_lock(&mx);
		gd32sim_avanza(&g, &mondo, t);
		while ((k = read(pg.master, buf, sizeof buf)) > 0)
			gd32_deframer_scrivi(&dfr, buf, (size_t)k, ricevuto, &t);
		while (read(pl.master, buf, sizeof buf) > 0)
			;                                                        /* il SoC non scrive mai al lidar */
		lidarsim_avanza(&l, &mondo, t, g.lidar_alimentato && g.acceso, g.lidar_potenza, g.x, g.y, g.dir);
		const sim_uscita_t *u;
		while ((u = gd32sim_prossimo(&g, t))) {
			scrivi_pty(&pg, u->b, u->n);
			registra(u->t, CAN_GD32_SOC, u->b, u->n);
		}
		const lidarsim_uscita_t *ul;
		while ((ul = lidarsim_prossimo(&l, t))) {
			scrivi_pty(&pl, ul->b, ul->n);
			registra(ul->t, CAN_LIDAR, ul->b, ul->n);
		}
		pthread_mutex_unlock(&mx);
		while (prossimo_passo <= t)
			prossimo_passo += 0.002;
		if (stampa && t >= prossima_stampa) {
			pthread_mutex_lock(&mx);
			stampa_stato(t);
			pthread_mutex_unlock(&mx);
			prossima_stampa += 1;
		}
	}
	unlink(p_gd32);
	unlink(p_lidar);
	printf("sim: fine (frame ricevuti %llu, sensori mandati %llu, reset %llu)\n", (unsigned long long)g.frame_ricevuti,
	       (unsigned long long)g.frame_sensori, (unsigned long long)g.reset);
	return g.reset ? 3 : 0;
}
