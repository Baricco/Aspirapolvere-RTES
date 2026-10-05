/* driver.c - Il driver del robot: vedi driver.h. */
#define _GNU_SOURCE
#include "driver.h"

#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <math.h>
#include <poll.h>
#include <sched.h>
#include <spawn.h>
#include <stdarg.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/eventfd.h>
#include <sys/wait.h>
#include <termios.h>
#include <unistd.h>

#include "../gd32/sicurezza.h"
#include "../spia/registro.h"
#include "scambio.h"

#define SCALA_V GD32_SCALA_V          /* 0x66: unità di v per m/s (gd32/gd32.h, misura del 5/10) */
#define SCALA_W GD32_SCALA_W          /* 0x66: unità di ω per rad/s */
#define TICK_NS           RT_MS(20)
#define HEARTBEAT_NS      RT_MS(200)
#define KEEPALIVE_NS      RT_S(3)
#define SENSORI_VECCHI_NS RT_MS(100)
#define SENSORI_MUTI_NS   RT_S(1)
#define MAX_ISCRITTI      8
#define MAX_SORVEGLIATI   16
#define MAX_EVENTI        64
#define LED_NORMALE       1
#define LED_BATTERIA      6
#define LED_CARICA        4
#define LED_GUASTO        8

extern char **environ;

/* ------------------------------------------------------------------------------------ stato condiviso --- */

/* Comandi dei task: campi atomici, letti dal driver a ogni risveglio. */
static struct {
	_Atomic int64_t vw;                    /* v (tick/s, 32 bit alti) e omega (tick/s, 32 bit bassi) */
	_Atomic int spazzola_lat, spazzola_cen, aspirazione, led, lidar_acceso, lidar_cgiri, uscita_base;
	_Atomic int azzera_guasto;
	_Atomic(const char *) suono;
} cmd;

/* Dati pubblicati: doppio buffer senza mutex (scambio.h). */
static scambio_t sc_sensori, sc_giro, sc_stato;
static drv_sensori_t buf_sensori[2];
static lidar_giro_t buf_giro[2];
static drv_stato_t buf_stato[2];

/* Task iscritti alle sveglie e sorvegliati: gli slot si riempiono una volta e si svuotano con drv_dimentica. */
static _Atomic(rt_task_t *) iscritti[MAX_ISCRITTI];
static struct {
	_Atomic(rt_task_t *) t;
	int critico;
	int64_t soglia_ns;
	int segnalato;                          /* solo il driver lo usa */
} sorvegliati[MAX_SORVEGLIATI];
static _Atomic uint64_t giri_driver;       /* risvegli completati: drv_dimentica aspetta che avanzi */

/* Eventi (testo) dal driver al thread di servizio, che li stampa: il driver non fa mai I/O lento. */
static char eventi[MAX_EVENTI][160];
static _Atomic uint32_t ev_testa, ev_coda;

/* ------------------------------------------------------------------------------------------ il driver --- */

static struct {
	drv_config_t cfg;
	int fd_gd32, fd_lidar, fd_sveglia;
	pthread_t thread, ricezione, servizio;
	pthread_mutex_t mx;                   /* stato interno, fra il thread del tempo e quello di ricezione */
	atomic_int stop;
	int avviato;
	int senza_rt;
	int gd32_chiusa, lidar_chiusa;
	int64_t t0;

	gd32_deframer_t dg;
	lidar_deframer_t dl;
	lidar_assemblatore_t ass;
	lidar_regolatore_t reg;
	lidar_giro_t giro;                    /* appoggio per il giro appena completato (grande: non sullo stack) */
	batteria_t bat;
	sicurezza_t sic;                      /* solo banco di prova */

	int fase;
	char guasto[96];
	int ha_sensori;
	gd32_sensori_t s;
	int64_t t_sensori;
	uint32_t n_sensori;
	uint64_t sensori_secondo[2];          /* conteggio per i frame al secondo */
	int64_t t_secondo;
	char versione[33];

	/* sequenza di avvio */
	int i_avvio;
	int64_t t_avvio;

	/* ultimo stato mandato al GD32 */
	int32_t v_inv, w_inv;
	int limiti;
	int lat_inv, cen_inv, asp_inv, led_inv, lidar_inv, pot_inv, sensori_inv;
	int asp_limiti;              /* perché l'aspirazione chiesta non va (drv_stato_t.aspirazione_limiti) */
	int64_t t_rinfresca, t_potenza;
	int sulla_base_prec;
	int64_t t_cambio_base;
	int lidar_ok;
	int64_t t_lidar_acceso;
	int bat_prec;

	/* tempi */
	int64_t prossimo_tick, prossimo_hb, prossimo_ka;
	int64_t tick_ritardo_max, lavoro_max, intervallo_max, intervallo_min;
	uint64_t risvegli, keepalive, heartbeat, hb_gd32, altri, errori_scrittura;
} d;

static int64_t adesso(void) { return rt_adesso(); }

static void evento(const char *fmt, ...)
{
	uint32_t t = atomic_load(&ev_testa);
	if (t - atomic_load(&ev_coda) >= MAX_EVENTI)
		return;                                    /* coda piena: l'evento si perde, il driver non aspetta */
	char *e = eventi[t % MAX_EVENTI];
	int k = snprintf(e, sizeof eventi[0], "%8.3f ", (double)(adesso() - d.t0) / RT_NS_PER_S);
	va_list ap;
	va_start(ap, fmt);
	vsnprintf(e + k, sizeof eventi[0] - (size_t)k, fmt, ap);
	va_end(ap);
	atomic_store(&ev_testa, t + 1);
}

/* ------------------------------------------------------------------------------------------ registrazione --- */

/* Registrazione dei byte grezzi (drv_config_t.registra), nel formato della spia, così le prove fatte con il driver
 * servono anche a tools/taratura e a tools/valida-sim.py. I thread del driver non scrivono MAI sul file: mettono i
 * record in un anello in RAM (sempre con d.mx preso, oppure dopo che i due thread sono finiti: un solo produttore
 * alla volta) e il thread di servizio, non real-time, li scrive. Se l'anello è pieno il record si perde e si conta. */
#define ANELLO (1u << 20)                /* 1 MB: ~75 s di traffico (GD32 + lidar ~14 KB/s) */
#define CANALE_GD32 0
#define CANALE_LIDAR 1
static struct {
	FILE *f;
	uint8_t *buf;
	_Atomic uint32_t testa, coda;        /* byte messi / tolti, modulo 2^32 */
	_Atomic uint64_t persi;
} rg;

static void anello_copia_in(uint32_t pos, const void *src, size_t n)
{
	const uint8_t *s = src;
	for (size_t i = 0; i < n; i++)
		rg.buf[(pos + i) & (ANELLO - 1)] = s[i];
}

static void anello_copia_da(uint32_t pos, void *dst, size_t n)
{
	uint8_t *o = dst;
	for (size_t i = 0; i < n; i++)
		o[i] = rg.buf[(pos + i) & (ANELLO - 1)];
}

static void registra(uint8_t canale, uint8_t direzione, const void *dati, size_t n, int64_t t)
{
	if (!rg.f || n == 0 || n > 4096)
		return;
	uint32_t testa = atomic_load_explicit(&rg.testa, memory_order_relaxed);
	uint32_t usato = testa - atomic_load_explicit(&rg.coda, memory_order_acquire);
	if (12 + n > ANELLO - usato) {
		atomic_fetch_add(&rg.persi, 1);
		return;
	}
	uint8_t h[12];
	uint16_t nn = (uint16_t)n;
	memcpy(h, &t, 8);
	h[8] = canale;
	h[9] = direzione;
	memcpy(h + 10, &nn, 2);
	anello_copia_in(testa, h, 12);
	anello_copia_in(testa + 12, dati, n);
	atomic_store_explicit(&rg.testa, testa + 12 + (uint32_t)n, memory_order_release);
}

/* Dal thread di servizio: scrive sul file tutti i record nell'anello. */
static void registra_svuota(void)
{
	static uint8_t dati[4096];
	if (!rg.f)
		return;
	uint32_t coda = atomic_load_explicit(&rg.coda, memory_order_relaxed);
	uint32_t testa = atomic_load_explicit(&rg.testa, memory_order_acquire);
	while (coda != testa) {
		uint8_t h[12];
		int64_t t;
		uint16_t n;
		anello_copia_da(coda, h, 12);
		memcpy(&t, h, 8);
		memcpy(&n, h + 10, 2);
		anello_copia_da(coda + 12, dati, n);
		reg_scrivi_record(rg.f, t, h[8], h[9], dati, n);
		coda += 12 + n;
		atomic_store_explicit(&rg.coda, coda, memory_order_release);
	}
}

static int registra_apri(const drv_config_t *c)
{
	memset(&rg, 0, sizeof rg);
	if (!c->registra)
		return 0;
	if (!(rg.buf = calloc(1, ANELLO)) || !(rg.f = fopen(c->registra, "wb"))) {
		free(rg.buf);
		rg.buf = NULL;
		return -1;
	}
	reg_intestazione_t h;
	memset(&h, 0, sizeof h);
	h.n_canali = c->dev_lidar ? 2 : 1;
	h.canali[0].id = CANALE_GD32;
	h.canali[0].protocollo = REG_PROTO_GD32;
	snprintf(h.canali[0].nome, sizeof h.canali[0].nome, "gd32");
	snprintf(h.canali[0].dispositivo, sizeof h.canali[0].dispositivo, "%s", c->dev_gd32);
	if (c->dev_lidar) {
		h.canali[1].id = CANALE_LIDAR;
		h.canali[1].protocollo = REG_PROTO_LIDAR;
		snprintf(h.canali[1].nome, sizeof h.canali[1].nome, "lidar");
		snprintf(h.canali[1].dispositivo, sizeof h.canali[1].dispositivo, "%s", c->dev_lidar);
	}
	return reg_scrivi_intestazione(rg.f, &h);
}

static void registra_chiudi(void)
{
	if (!rg.f)
		return;
	registra_svuota();
	fclose(rg.f);
	rg.f = NULL;
	free(rg.buf);
	rg.buf = NULL;
}

/* -------------------------------------------------------------------------------------------- seriale --- */

static void invia(uint8_t cmd_gd32, const void *dati, size_t n)
{
	uint8_t fr[GD32_MAX_FRAME];
	size_t k = gd32_componi(cmd_gd32, dati, n, fr, sizeof fr);
	if (k && write(d.fd_gd32, fr, k) != (ssize_t)k)
		d.errori_scrittura++;
	registra(CANALE_GD32, REG_DAL_SOC, fr, k, adesso());
}

static void invia1(uint8_t c, uint8_t v) { invia(c, &v, 1); }

static void invia_i32(uint8_t c, int32_t v)
{
	uint8_t b[4] = { (uint8_t)v, (uint8_t)(v >> 8), (uint8_t)(v >> 16), (uint8_t)(v >> 24) };
	invia(c, b, 4);
}

static void invia_velocita(int32_t v, int32_t w)
{
	uint8_t b[8];
	for (int i = 0; i < 4; i++) {
		b[i] = (uint8_t)((uint32_t)v >> (8 * i));
		b[4 + i] = (uint8_t)((uint32_t)w >> (8 * i));
	}
	invia(GD32_VELOCITA, b, 8);
	d.v_inv = v;
	d.w_inv = w;
}

static void arresto(void)
{
	uint8_t buf[256];
	size_t k = sic_frame_arresto(buf, sizeof buf);
	if (k && write(d.fd_gd32, buf, k) != (ssize_t)k)
		d.errori_scrittura++;
	registra(CANALE_GD32, REG_DAL_SOC, buf, k, adesso());
	d.v_inv = d.w_inv = 0;
	d.lat_inv = d.cen_inv = d.asp_inv = 0;
}

/* Un altro processo ha già aperto la seriale? (es. AuxCtrl dello stack originale, o il custode) */
static int occupato(const char *disp)
{
	char vero[PATH_MAX];
	if (!realpath(disp, vero) || strncmp(vero, "/dev/tty", 8))
		return 0;                    /* le pseudo-seriali del simulatore (/dev/pts) le tiene aperte il simulatore */
	DIR *proc = opendir("/proc");
	if (!proc)
		return 0;
	struct dirent *p;
	int trovato = 0;
	while (!trovato && (p = readdir(proc))) {
		if (p->d_name[0] < '1' || p->d_name[0] > '9' || atoi(p->d_name) == getpid())
			continue;
		char dir[300];
		snprintf(dir, sizeof dir, "/proc/%s/fd", p->d_name);
		DIR *fds = opendir(dir);
		struct dirent *f;
		while (fds && (f = readdir(fds))) {
			char link[600], dest[PATH_MAX];
			snprintf(link, sizeof link, "%s/%s", dir, f->d_name);
			ssize_t k = readlink(link, dest, sizeof dest - 1);
			if (k > 0) {
				dest[k] = 0;
				if (!strcmp(dest, vero)) {
					fprintf(stderr, "driver: %s è già aperto dal processo %s (fermalo prima)\n", disp, p->d_name);
					trovato = 1;
					break;
				}
			}
		}
		if (fds)
			closedir(fds);
	}
	closedir(proc);
	return trovato;
}

static int apri_seriale(const char *disp)
{
	int fd = open(disp, O_RDWR | O_NOCTTY | O_NONBLOCK);
	if (fd < 0)
		return -1;
	struct termios t;
	if (tcgetattr(fd, &t) == 0) {
		cfmakeraw(&t);
		cfsetispeed(&t, B115200);
		cfsetospeed(&t, B115200);
		t.c_cflag |= CLOCAL | CREAD;
		t.c_cflag &= ~(CSTOPB | CRTSCTS | PARENB);
		tcsetattr(fd, TCSANOW, &t);
	}
	tcflush(fd, TCIOFLUSH);              /* via i byte vecchi rimasti nel buffer */
	return fd;
}

/* -------------------------------------------------------------------------------------- stato e guasto --- */

static void entra_in_guasto(const char *motivo)
{
	if (d.fase == DRV_GUASTO)
		return;
	d.fase = DRV_GUASTO;
	snprintf(d.guasto, sizeof d.guasto, "%s", motivo);
	arresto();
	invia1(GD32_LED_TASTO, LED_GUASTO);
	d.led_inv = LED_GUASTO;
	atomic_store(&cmd.suono, "sound_erro_harefault");
	evento("GUASTO: %s. Motori fermi, LED 8", motivo);
}

static void prova_ad_azzerare_guasto(int64_t t)
{
	/* la richiesta si consuma sempre: una richiesta fatta quando non c'era un guasto non deve azzerare il prossimo */
	int richiesta = atomic_exchange(&cmd.azzera_guasto, 0);
	if (d.fase != DRV_GUASTO || !richiesta)
		return;
	if (!d.ha_sensori || t - d.t_sensori > SENSORI_VECCHI_NS || (d.cfg.banco && d.sic.scattato)) {
		evento("guasto NON azzerato: la causa c'è ancora");
		return;
	}
	for (int i = 0; i < MAX_SORVEGLIATI; i++) {
		rt_task_t *k = atomic_load(&sorvegliati[i].t);
		if (k && sorvegliati[i].critico &&
		    (rt_task_bloccato(k, t, sorvegliati[i].soglia_ns) || atomic_load(&k->mancate_di_fila) >= 3)) {
			evento("guasto NON azzerato: il task %s è ancora bloccato", k->cfg.nome ? k->cfg.nome : "?");
			return;
		}
	}
	d.fase = DRV_PRONTO;
	d.guasto[0] = 0;
	d.led_inv = -1;                       /* rimanda il LED giusto al prossimo ciclo */
	for (int i = 0; i < MAX_SORVEGLIATI; i++)
		sorvegliati[i].segnalato = 0;
	evento("guasto azzerato");
}

static void pubblica_stato(int64_t t)
{
	drv_stato_t s;
	memset(&s, 0, sizeof s);
	s.t_ns = t;
	s.fase = d.fase;
	memcpy(s.guasto, d.guasto, sizeof s.guasto);
	memcpy(s.versione_gd32, d.versione, sizeof s.versione_gd32);
	s.volt = batteria_volt(&d.bat);
	s.percentuale = batteria_percentuale(&d.bat);
	s.batteria = d.bat.livello;
	s.in_carica = d.s.in_carica;
	s.sulla_base = d.s.sulla_base;
	s.temperatura_c = d.s.temperatura_c;
	s.v = d.v_inv / SCALA_V;
	s.w = d.w_inv / SCALA_W;
	s.limiti = d.limiti;
	s.spazzola_laterale = d.lat_inv;
	s.spazzola_centrale = d.cen_inv;
	s.aspirazione_giri = d.asp_inv;
	s.aspirazione_limiti = d.asp_limiti;
	s.led = d.led_inv;
	s.lidar_acceso = d.lidar_inv > 0;
	s.lidar_potenza = d.lidar_inv > 0 ? d.pot_inv : 0;
	s.lidar_ok = d.lidar_ok;
	s.lidar_giri_s = d.ass.giri_s;
	s.lidar_giri = d.ass.giri;
	s.frame_sensori = d.n_sensori;
	s.frame_errati = d.dg.checksum_errati + d.dg.len_errati;
	s.byte_scartati = d.dg.byte_scartati;
	s.frame_gd32_altri = d.altri;
	s.heartbeat_gd32 = d.hb_gd32;
	s.keepalive = d.keepalive;
	s.heartbeat = d.heartbeat;
	s.errori_scrittura = d.errori_scrittura;
	s.sensori_hz = (double)d.sensori_secondo[1];
	s.eta_sensori_ns = d.ha_sensori ? t - d.t_sensori : -1;
	s.tick_ritardo_max_ns = d.tick_ritardo_max;
	s.intervallo_sensori_max_ns = d.intervallo_max;
	s.intervallo_sensori_min_ns = d.intervallo_min;
	s.lavoro_max_ns = d.lavoro_max;
	s.risvegli = d.risvegli;
	s.senza_rt = d.senza_rt;
	scambio_scrivi(&sc_stato, &s);
}

/* ------------------------------------------------------------------------------------ limiti e comandi --- */

/* Le condizioni per qualunque motore (ruote, spazzole, aspirazione), prima dei limiti specifici delle ruote. */
static int limiti_generali(int64_t t)
{
	if (!d.cfg.motori)
		return DRV_LIM_DISABILITATI;
	if (d.fase == DRV_AVVIO)
		return DRV_LIM_NON_PRONTO;
	if (d.fase != DRV_PRONTO)
		return DRV_LIM_GUASTO;
	if (!d.ha_sensori || t - d.t_sensori > SENSORI_VECCHI_NS)
		return DRV_LIM_SENSORI_VECCHI;
	if (d.bat.livello == BAT_CRITICA && !d.s.in_carica)
		return DRV_LIM_BATTERIA;
	return 0;
}

/* Comando di velocità dei task -> quello che si può davvero mandare. */
static int limita_velocita(int64_t t, int32_t *v, int32_t *w)
{
	int64_t vw = atomic_load(&cmd.vw);
	*v = (int32_t)(vw >> 32);
	*w = (int32_t)(uint32_t)vw;
	int lim = limiti_generali(t);
	if (lim) {
		*v = *w = 0;
		return lim;
	}
	if (d.cfg.banco) {
		uint8_t b[8];
		for (int i = 0; i < 4; i++) {
			b[i] = (uint8_t)((uint32_t)*v >> (8 * i));
			b[4 + i] = (uint8_t)((uint32_t)*w >> (8 * i));
		}
		if (sic_permesso(&d.sic, GD32_VELOCITA, b, 8, t) != SIC_PERMESSO) {
			*v = *w = 0;
			return DRV_LIM_BANCO;
		}
		return 0;
	}
	return sic_terra_limita(&d.s, atomic_load(&cmd.uscita_base), v, w);
}

/* Spazzole e aspirazione: niente se il robot è sollevato o sulla base (a terra), o se il banco non lo permette.
 * Ritorna 0 se il comando si può mandare, altrimenti il motivo (SIC_LIM_* | DRV_LIM_*). */
static int attuatori_limiti(int64_t t, uint8_t cmd_gd32, int valore)
{
	if (valore == 0)
		return 0;
	int lim = limiti_generali(t);
	if (lim)
		return lim;
	if (d.cfg.banco) {
		uint8_t b = (uint8_t)valore;
		return sic_permesso(&d.sic, cmd_gd32, &b, 1, t) == SIC_PERMESSO ? 0 : DRV_LIM_BANCO;
	}
	int32_t v = 1, w = 0;
	return sic_terra_limita(&d.s, 0, &v, &w) & (SIC_LIM_SOLLEVATO | SIC_LIM_BASE | SIC_LIM_SENSORI);
}

static int clamp(int v, int lo, int hi) { return v < lo ? lo : v > hi ? hi : v; }

/* Ruote: manda 0x66 se il comando limitato è cambiato (o sempre, con forza = 1: ogni 20 ms). */
static void aggiorna_ruote(int64_t t, int forza)
{
	int32_t v, w;
	d.limiti = limita_velocita(t, &v, &w);
	if (d.fase == DRV_AVVIO)
		return;                            /* durante l'avvio va solo la sequenza (e l'arresto iniziale) */
	if (forza || v != d.v_inv || w != d.w_inv)
		invia_velocita(v, w);
}

static void aggiorna_attuatori(int64_t t)
{
	int rinfresca = t - d.t_rinfresca > RT_S(2);   /* ogni 2 s si rimanda tutto: il GD32 potrebbe essersi resettato */
	if (rinfresca)
		d.t_rinfresca = t;

	int lat = clamp(atomic_load(&cmd.spazzola_lat), -100, 100);
	int cen = clamp(atomic_load(&cmd.spazzola_cen), -100, 100);
	int asp = clamp(atomic_load(&cmd.aspirazione), 0, 30000);
	if (attuatori_limiti(t, GD32_SPAZZOLA_LATERALE, lat)) lat = 0;
	if (attuatori_limiti(t, GD32_SPAZZOLA_CENTRALE, cen)) cen = 0;
	/* Aspirazione: anche niente cassetto della polvere (byte 4 bit 2). Il GD32 non la ferma da solo (provato il 5/10:
	   cassetto sfilato per 10 s con la ventola a 8000 giri/min, la ventola ha continuato); l'originale non la accende
	   (eccezione 6, EXCEPTION_DUSTBOX_INEXISTENCE). */
	int asp_lim = !asp ? 0 : !d.cfg.aspirazione ? DRV_LIM_VIETATA : attuatori_limiti(t, GD32_ASPIRAZIONE, 1);
	if (asp && !asp_lim && !d.s.cassetto)
		asp_lim = DRV_LIM_CASSETTO;
	d.asp_limiti = asp_lim;
	if (asp_lim) asp = 0;
	if (lat != d.lat_inv || rinfresca) {
		invia1(GD32_SPAZZOLA_LATERALE, (uint8_t)(int8_t)lat);
		d.lat_inv = lat;
	}
	if (cen != d.cen_inv || (rinfresca && cen >= 0)) {   /* lo sblocco (< 0) non si ripete: il GD32 lo ferma da solo */
		invia1(GD32_SPAZZOLA_CENTRALE, (uint8_t)(int8_t)cen);
		d.cen_inv = cen;
	}
	if (asp != d.asp_inv || (rinfresca && asp)) {
		int16_t a = (int16_t)(asp / 10);
		uint8_t b[2] = { (uint8_t)a, (uint8_t)(a >> 8) };
		invia(GD32_ASPIRAZIONE, b, 2);
		d.asp_inv = asp;
	}

	/* LED: guasto > batteria critica o bassa > in carica > quello chiesto dai task */
	int led = clamp(atomic_load(&cmd.led), 0, 31);
	if (d.fase == DRV_GUASTO)
		led = LED_GUASTO;
	else if (d.bat.livello != BAT_OK)
		led = LED_BATTERIA;
	else if (d.s.in_carica)
		led = LED_CARICA;
	if (led != d.led_inv || rinfresca) {
		invia1(GD32_LED_TASTO, (uint8_t)led);
		d.led_inv = led;
	}

	/* alimentazione dei sensori: spenti sulla base, accesi fuori (come everest), dopo 0,5 s di stato stabile */
	if (d.ha_sensori) {
		if (d.s.sulla_base != d.sulla_base_prec) {
			d.sulla_base_prec = d.s.sulla_base;
			d.t_cambio_base = t;
		}
		int voluto = !d.s.sulla_base;
		if (voluto != d.sensori_inv && t - d.t_cambio_base > RT_MS(500)) {
			invia1(GD32_ALIMENTAZIONE_SENSORI, (uint8_t)voluto);
			d.sensori_inv = voluto;
			evento("sensori %s (%s)", voluto ? "accesi" : "spenti", voluto ? "fuori dalla base" : "sulla base");
		}
	}
}

static void aggiorna_lidar(int64_t t)
{
	if (d.fd_lidar < 0)
		return;
	int acceso = atomic_load(&cmd.lidar_acceso);
	if (acceso != d.lidar_inv) {
		invia1(GD32_ALIMENTAZIONE_LIDAR, (uint8_t)acceso);
		d.lidar_inv = acceso;
		d.t_lidar_acceso = t;
		d.lidar_ok = 1;
		lidar_regolatore_init(&d.reg, 0);
		if (!acceso) {
			invia_i32(GD32_MOTORE_LIDAR, 0);
			d.pot_inv = 0;
		}
		evento("lidar %s", acceso ? "acceso" : "spento");
	}
	if (!acceso)
		return;
	double obiettivo = atomic_load(&cmd.lidar_cgiri) / 100.0;
	if (obiettivo <= 0)
		obiettivo = d.cfg.lidar_giri_s;
	d.reg.obiettivo = obiettivo;
	int fresca = d.ass.t_velocita && t - d.ass.t_velocita < RT_MS(500) && d.ass.t_velocita > d.t_lidar_acceso;
	int pot = lidar_regola(&d.reg, fresca ? d.ass.giri_s : 0);
	if (!fresca && t - d.t_lidar_acceso > RT_S(4)) {
		if (d.lidar_ok)
			evento("ATTENZIONE: il lidar non manda dati da 0,5 s (potenza tenuta a 60)");
		d.lidar_ok = 0;
		pot = 60;
		d.reg.integrale = 0;
	} else if (fresca && !d.lidar_ok) {
		d.lidar_ok = 1;
		evento("il lidar manda di nuovo dati");
	}
	if (pot != d.pot_inv || t - d.t_potenza > RT_S(1)) {
		invia_i32(GD32_MOTORE_LIDAR, pot);
		d.pot_inv = pot;
		d.t_potenza = t;
	}
}

/* ------------------------------------------------------------------------------------ frame in arrivo --- */

static void frame_gd32(const gd32_frame_t *f, void *ctx)
{
	int64_t t = *(const int64_t *)ctx;
	if (f->cmd == GD32_SENSORI) {
		if (gd32_leggi_sensori(f, &d.s))
			return;
		/* intervallo fra due frame visti dal driver: il GD32 li manda ogni 20,0 ms, quindi lo scarto misura i ritardi
		 * del percorso seriale (interrupt, kernel, thread del driver) */
		if (d.ha_sensori && d.n_sensori > 50) {
			int64_t dt = t - d.t_sensori;
			if (dt > d.intervallo_max)
				d.intervallo_max = dt;
			if (dt < d.intervallo_min || d.intervallo_min == 0)
				d.intervallo_min = dt;
		}
		d.ha_sensori = 1;
		d.t_sensori = t;
		d.n_sensori++;
		d.sensori_secondo[0]++;
		/* "in movimento" come l'originale: v + |ω| misurati (byte 32 e 36) maggiori di zero */
		int in_moto = d.s.vel_sx / SCALA_V + fabs(d.s.vel_dx / SCALA_W) > 0;
		int liv = batteria_aggiorna(&d.bat, d.s.tensione_dv, d.s.in_carica, d.s.sulla_base, in_moto);
		if (liv != d.bat_prec) {
			static const char *nomi[] = { "normale", "BASSA (LED 6)", "CRITICA: motori fermi" };
			evento("batteria %s, %.2f V", nomi[liv], batteria_volt(&d.bat));
			d.bat_prec = liv;
		}
		if (d.cfg.banco && sic_ricevuto(&d.sic, f, t)) {
			arresto();
			entra_in_guasto(d.sic.motivo);
		}
		drv_sensori_t p = { .numero = d.n_sensori, .t_ns = t, .s = d.s };
		scambio_scrivi(&sc_sensori, &p);
		for (int i = 0; i < MAX_ISCRITTI; i++) {
			rt_task_t *k = atomic_load(&iscritti[i]);
			if (k)
				rt_task_sveglia(k, t);
		}
		/* reazione immediata: se i nuovi sensori cambiano il comando permesso (bumper, vuoto), lo si manda ora */
		aggiorna_ruote(t, 0);
		return;
	}
	switch (f->cmd) {
	case GD32_HEARTBEAT:
		d.hb_gd32++;
		break;
	case GD32_VERSIONE: {
		size_t n = f->n_dati < 32 ? f->n_dati : 32;
		memcpy(d.versione, f->dati, n);
		d.versione[n] = 0;
		for (size_t i = 0; i < n; i++)
			if (d.versione[i] < 32 || d.versione[i] > 126)
				d.versione[i] = 0;
		evento("GD32 firmware %s", d.versione);
		break;
	}
	case GD32_WAKEUP_ACK:
		invia(GD32_WAKEUP_ACK, NULL, 0);     /* non lo mettiamo mai in sospensione, ma se chiede conferma la diamo */
		d.altri++;
		break;
	case GD32_CODICE_ERRORE:
	case GD32_DEBUG:
		evento("GD32 manda 0x%02x (%s), %d byte", f->cmd, gd32_nome_cmd(f->cmd), f->n_dati);
		d.altri++;
		break;
	default:
		d.altri++;
		break;
	}
}

static void pacchetto_lidar(const lidar_pacchetto_t *p, void *ctx)
{
	int64_t t = *(const int64_t *)ctx;
	if (lidar_assembla(&d.ass, p, t, &d.giro))
		scambio_scrivi(&sc_giro, &d.giro);
}

/* ------------------------------------------------------------------------------------------ ogni 20 ms --- */

/* Sequenza di avvio di everest-server (docs/protocollo.md), un frame per ciclo. */
static const struct { uint8_t cmd, n, dati[4]; } AVVIO[] = {
	{ GD32_SORVEGLIA_SOC, 1, { 1 } }, { GD32_VERSIONE, 0, { 0 } }, { GD32_IMU_GREZZA, 4, { 1, 0, 0, 0 } },
	{ GD32_FUNZIONI_MACCHINA, 0, { 0 } }, { GD32_MOTORE_LIDAR, 4, { 0, 0, 0, 0 } }, { GD32_TIPO_CONTROLLO, 1, { 0 } },
	{ GD32_CLIFF_RETROMARCIA, 1, { 1 } }, { GD32_MASCHERA_SENSORI, 3, { 0, 0xff, 0xff } }, { GD32_CLIFF_IR, 1, { 1 } },
	{ GD32_IMU_ZERO, 0, { 0 } }, { GD32_TIPO_CONTROLLO, 1, { 2 } },
};
#define N_AVVIO ((int)(sizeof AVVIO / sizeof AVVIO[0]))

static void tick(int64_t t)
{
	if (d.fase == DRV_AVVIO) {
		if (d.i_avvio < N_AVVIO) {
			invia(AVVIO[d.i_avvio].cmd, AVVIO[d.i_avvio].dati, AVVIO[d.i_avvio].n);
			d.i_avvio++;
		} else if (d.ha_sensori) {
			d.fase = DRV_PRONTO;
			d.led_inv = -1;
			d.sensori_inv = -1;
			invia_velocita(0, 0);
			evento("pronto: GD32 %s, %u frame dei sensori già ricevuti",
			       d.versione[0] ? d.versione : "(versione non ricevuta)", d.n_sensori);
		} else if (t - d.t_avvio > RT_S(5)) {
			entra_in_guasto("il GD32 non manda i sensori (5 s dall'avvio)");
		}
	}

	/* il GD32 tace? */
	if (d.fase == DRV_PRONTO && d.ha_sensori && t - d.t_sensori > SENSORI_MUTI_NS)
		entra_in_guasto("nessun frame dei sensori da 1 s");

	/* watchdog dei task */
	for (int i = 0; i < MAX_SORVEGLIATI; i++) {
		rt_task_t *k = atomic_load(&sorvegliati[i].t);
		if (!k || sorvegliati[i].segnalato)
			continue;
		const char *nome = k->cfg.nome ? k->cfg.nome : "?";
		char motivo[96] = "";
		if (rt_task_bloccato(k, t, sorvegliati[i].soglia_ns))
			snprintf(motivo, sizeof motivo, "il task %s è bloccato", nome);
		else if (atomic_load(&k->mancate_di_fila) >= 3)
			snprintf(motivo, sizeof motivo, "il task %s ha mancato 3 scadenze di fila", nome);
		if (!motivo[0])
			continue;
		sorvegliati[i].segnalato = 1;
		if (sorvegliati[i].critico)
			entra_in_guasto(motivo);
		else
			evento("ATTENZIONE: %s (non critico)", motivo);
	}
	prova_ad_azzerare_guasto(t);

	if (d.fase != DRV_AVVIO) {
		aggiorna_ruote(t, 1);
		aggiorna_attuatori(t);
		aggiorna_lidar(t);
	}
	if (t - d.t_secondo >= RT_S(1)) {
		d.sensori_secondo[1] = d.sensori_secondo[0];
		d.sensori_secondo[0] = 0;
		d.t_secondo += RT_S(1);
		if (t - d.t_secondo > RT_S(1))
			d.t_secondo = t;
	}
	pubblica_stato(t);
}

/* --------------------------------------------------------------------------------------------- thread --- */

/* Legge tutto quello che c'è. Ritorna -1 se la seriale è chiusa dall'altra parte (succede solo con le
 * pseudo-seriali del simulatore, quando il simulatore esce). */
static int leggi(int fd, int gd32, int64_t t)
{
	uint8_t buf[1024];
	ssize_t k;
	while ((k = read(fd, buf, sizeof buf)) > 0) {
		registra(gd32 ? CANALE_GD32 : CANALE_LIDAR, REG_VERSO_SOC, buf, (size_t)k, t);
		if (gd32)
			gd32_deframer_scrivi(&d.dg, buf, (size_t)k, frame_gd32, &t);
		else
			lidar_deframer_scrivi(&d.dl, buf, (size_t)k, pacchetto_lidar, &t);
	}
	return (k == 0 || (k < 0 && errno != EAGAIN && errno != EINTR)) ? -1 : 0;
}

static void fissa_cpu(void)
{
	if (d.cfg.cpu >= 0) {
		cpu_set_t set;
		CPU_ZERO(&set);
		CPU_SET(d.cfg.cpu, &set);
		pthread_setaffinity_np(pthread_self(), sizeof set, &set);
	}
}

static int64_t minimo(int64_t a, int64_t b) { return a < b ? a : b; }

static void lavoro_fatto(int64_t inizio)
{
	int64_t lavoro = adesso() - inizio;
	if (lavoro > d.lavoro_max && d.risvegli > 100)
		d.lavoro_max = lavoro;
	d.risvegli++;
	atomic_fetch_add(&giri_driver, 1);
}

/*
 * Il driver ha due thread, sullo stesso core (docs/fase3.md, prove r4-r6: un thread che dorme in ppoll sulle seriali
 * a volte si sveglia con millisecondi di ritardo, uno che dorme con clock_nanosleep no):
 *   - TEMPO (priorità cfg.priorita, 99): dorme con clock_nanosleep fino al prossimo evento a tempo e fa heartbeat,
 *     keepalive, il ciclo da 20 ms (0x66, attuatori, lidar, watchdog, stato);
 *   - RICEZIONE (priorità cfg.priorita - 1, 98): dorme in ppoll sulle seriali e sull'eventfd dei comandi; decodifica,
 *     pubblica, sveglia i task, reagisce subito ai sensori e ai comandi nuovi.
 * Lo stato interno del driver (d) lo usano entrambi, sempre con d.mx preso (mutex con priority inheritance; sezioni
 * di decine di microsecondi). I task non lo toccano mai.
 */
static void *ciclo_tempo(void *arg)
{
	(void)arg;
	fissa_cpu();
	pthread_mutex_lock(&d.mx);
	int64_t t = adesso();
	d.t_avvio = d.t_secondo = t;
	d.prossimo_tick = d.prossimo_hb = d.prossimo_ka = t;
	arresto();                                  /* stato sicuro prima di tutto */
	evento("driver avviato (CPU %d, %s), sequenza di avvio del GD32", d.cfg.cpu,
	       d.senza_rt ? "SENZA SCHED_FIFO" : "SCHED_FIFO 99 + ricezione 98");
	pthread_mutex_unlock(&d.mx);

	while (!atomic_load(&d.stop)) {
		rt_dormi_fino_a(minimo(d.prossimo_tick, minimo(d.prossimo_hb, d.prossimo_ka)));
		if (atomic_load(&d.stop))
			break;
		pthread_mutex_lock(&d.mx);
		int64_t inizio = adesso();
		t = inizio;
		if (t >= d.prossimo_hb) {
			invia(GD32_HEARTBEAT, NULL, 0);
			d.heartbeat++;
			d.prossimo_hb += HEARTBEAT_NS;
			if (d.prossimo_hb <= t)
				d.prossimo_hb = t + HEARTBEAT_NS;
		}
		if (t >= d.prossimo_ka) {
			invia(GD32_KEEPALIVE, NULL, 0);
			d.keepalive++;
			d.prossimo_ka += KEEPALIVE_NS;
			if (d.prossimo_ka <= t)
				d.prossimo_ka = t + KEEPALIVE_NS;
		}
		if (t >= d.prossimo_tick) {
			int64_t ritardo = t - d.prossimo_tick;
			if (ritardo > d.tick_ritardo_max && d.risvegli > 100) {  /* i primi giri (avvio) non contano */
				d.tick_ritardo_max = ritardo;
				if (ritardo > RT_US(300))
					evento("ritardo del ciclo da 20 ms: %.0f us (nuovo massimo)", ritardo / 1e3);
			}
			tick(t);
			d.prossimo_tick += TICK_NS;
			if (d.prossimo_tick <= t)
				d.prossimo_tick = t + TICK_NS;
		}
		lavoro_fatto(inizio);
		pthread_mutex_unlock(&d.mx);
	}
	return NULL;
}

static void *ciclo_ricezione(void *arg)
{
	(void)arg;
	fissa_cpu();
	while (!atomic_load(&d.stop)) {
		struct pollfd p[3] = {
			{ .fd = d.gd32_chiusa ? -1 : d.fd_gd32, .events = POLLIN }, { .fd = d.fd_sveglia, .events = POLLIN },
			{ .fd = d.lidar_chiusa ? -1 : d.fd_lidar, .events = POLLIN },
		};
		struct timespec to = { .tv_sec = 1 };
		if (ppoll(p, d.fd_lidar >= 0 ? 3 : 2, &to, NULL) <= 0)
			continue;
		pthread_mutex_lock(&d.mx);
		int64_t inizio = adesso(), t = inizio;
		if ((p[0].revents & (POLLIN | POLLHUP | POLLERR)) && leggi(d.fd_gd32, 1, t) < 0 && !d.gd32_chiusa) {
			d.gd32_chiusa = 1;
			entra_in_guasto("la seriale del GD32 è stata chiusa");
		}
		if (d.fd_lidar >= 0 && (p[2].revents & (POLLIN | POLLHUP | POLLERR)) && leggi(d.fd_lidar, 0, t) < 0 &&
		    !d.lidar_chiusa) {
			d.lidar_chiusa = 1;
			evento("ATTENZIONE: la seriale del lidar è stata chiusa");
		}
		if (p[1].revents & POLLIN) {
			uint64_t n;
			if (read(d.fd_sveglia, &n, sizeof n) < 0)
				n = 0;
			if (d.fase != DRV_AVVIO && !atomic_load(&d.stop)) {   /* comandi nuovi dei task: subito */
				aggiorna_ruote(t, 0);
				aggiorna_attuatori(t);
			}
		}
		lavoro_fatto(inizio);
		pthread_mutex_unlock(&d.mx);
	}
	return NULL;
}

/* Dopo che i due thread sono terminati: tutto fermo. */
static void spegni(void)
{
	arresto();
	if (d.fd_lidar >= 0) {
		invia1(GD32_ALIMENTAZIONE_LIDAR, 0);
		invia_i32(GD32_MOTORE_LIDAR, 0);
	}
	invia1(GD32_LED_TASTO, LED_NORMALE);
	tcdrain(d.fd_gd32);
	d.fase = DRV_FINE;
	pubblica_stato(adesso());
	evento("driver fermato: motori a zero, lidar spento");
}

/* Thread NON real-time: stampa gli eventi e suona. Così il driver non fa mai I/O lento. */
static void *ciclo_servizio(void *arg)
{
	(void)arg;
	for (;;) {
		int fine = atomic_load(&d.stop) && d.fase == DRV_FINE;
		while (atomic_load(&ev_coda) != atomic_load(&ev_testa)) {
			uint32_t c = atomic_load(&ev_coda);
			fprintf(stderr, "driver %s\n", eventi[c % MAX_EVENTI]);
			atomic_store(&ev_coda, c + 1);
		}
		const char *s = atomic_exchange(&cmd.suono, NULL);
		if (s && d.cfg.cmd_suono) {
			char c[300];
			snprintf(c, sizeof c, d.cfg.cmd_suono, s);
			char *argv[] = { "sh", "-c", c, NULL };
			pid_t pid;
			if (posix_spawn(&pid, "/bin/sh", NULL, NULL, argv, environ) == 0)
				waitpid(pid, NULL, 0);
		}
		registra_svuota();
		if (fine)
			break;
		usleep(20000);
	}
	return NULL;
}

/* ------------------------------------------------------------------------------------------------ API --- */

static void sveglia_driver(void);

/* Thread SCHED_FIFO alla priorità data, sullo stack di RT_STACK_DEFAULT; senza permessi (solo con cfg.senza_rt)
 * parte con lo scheduling normale. */
static int crea_thread(pthread_t *th, void *(*f)(void *), int prio)
{
	pthread_attr_t a;
	struct sched_param sp = { .sched_priority = prio };
	pthread_attr_init(&a);
	pthread_attr_setinheritsched(&a, PTHREAD_EXPLICIT_SCHED);
	pthread_attr_setschedpolicy(&a, SCHED_FIFO);
	pthread_attr_setschedparam(&a, &sp);
	pthread_attr_setstacksize(&a, RT_STACK_DEFAULT);
	int r = pthread_create(th, &a, f, NULL);
	if (r == EPERM && d.cfg.senza_rt) {
		pthread_attr_setinheritsched(&a, PTHREAD_INHERIT_SCHED);
		if (!d.senza_rt)
			fprintf(stderr, "driver: niente permessi per SCHED_FIFO: gira con lo scheduling normale (solo per il PC)\n");
		d.senza_rt = 1;
		r = pthread_create(th, &a, f, NULL);
	}
	pthread_attr_destroy(&a);
	return r;
}

void drv_config_default(drv_config_t *c)
{
	memset(c, 0, sizeof(*c));
	/* nel simulatore ./rtes.sh sim imposta RTES_GD32 e RTES_LIDAR: lo stesso programma va sul PC e sul robot */
	c->dev_gd32 = getenv("RTES_GD32") ? getenv("RTES_GD32") : "/dev/ttyS3";
	c->dev_lidar = getenv("RTES_LIDAR") ? getenv("RTES_LIDAR") : "/dev/ttyS1";
	c->senza_rt = getenv("RTES_GD32") != NULL;   /* nel simulatore si parte anche senza permessi per SCHED_FIFO */
	c->cpu = 0;
	c->priorita = RT_PRIO_DRIVER;
	c->motori = 1;
	c->aspirazione = 0;
	c->cliff_atteso = -1;
	c->lidar_giri_s = 6.0;
	c->scarto_lidar_cdeg = LIDAR_SCARTO_CDEG;
	c->cmd_suono = access("/bin/tinyplayer", X_OK) == 0 ? "tinyplayer /usr/share/audio/english/%s.mp3 >/dev/null 2>&1"
	                                                     : NULL;
}

int drv_avvia(const drv_config_t *c)
{
	memset(&d, 0, sizeof d);
	d.cfg = *c;
	d.fd_lidar = -1;
	d.t0 = adesso();
	d.fase = DRV_AVVIO;
	d.lat_inv = d.cen_inv = d.asp_inv = 0;
	d.led_inv = d.sensori_inv = -1;
	d.bat_prec = BAT_OK;
	gd32_deframer_init(&d.dg);
	lidar_deframer_init(&d.dl);
	lidar_assemblatore_init(&d.ass, c->scarto_lidar_cdeg);
	lidar_regolatore_init(&d.reg, 0);
	batteria_init(&d.bat);
	sic_init(&d.sic, 1, SENSORI_VECCHI_NS);
	if (c->cliff_atteso >= 0)
		sic_cliff_atteso(&d.sic, (uint8_t)c->cliff_atteso);
	scambio_init(&sc_sensori, &buf_sensori[0], &buf_sensori[1], sizeof buf_sensori[0]);
	scambio_init(&sc_giro, &buf_giro[0], &buf_giro[1], sizeof buf_giro[0]);
	scambio_init(&sc_stato, &buf_stato[0], &buf_stato[1], sizeof buf_stato[0]);
	atomic_store(&cmd.vw, 0);
	atomic_store(&cmd.spazzola_lat, 0);
	atomic_store(&cmd.spazzola_cen, 0);
	atomic_store(&cmd.aspirazione, 0);
	atomic_store(&cmd.led, LED_NORMALE);
	atomic_store(&cmd.lidar_acceso, 0);
	atomic_store(&cmd.lidar_cgiri, 0);
	atomic_store(&cmd.uscita_base, 0);
	atomic_store(&cmd.azzera_guasto, 0);
	atomic_store(&cmd.suono, NULL);
	atomic_store(&ev_testa, 0);
	atomic_store(&ev_coda, 0);
	atomic_init(&d.stop, 0);

	if (!c->forza && (occupato(c->dev_gd32) || (c->dev_lidar && occupato(c->dev_lidar))))
		return EBUSY;
	if (registra_apri(c) != 0) {
		int e = errno;
		fprintf(stderr, "driver: registrazione in %s: %s\n", c->registra, strerror(e));
		return e ? e : EIO;
	}
	if ((d.fd_gd32 = apri_seriale(c->dev_gd32)) < 0) {
		int e = errno;
		fprintf(stderr, "driver: apro %s: %s\n", c->dev_gd32, strerror(e));
		registra_chiudi();
		return e;
	}
	if (c->dev_lidar && *c->dev_lidar && (d.fd_lidar = apri_seriale(c->dev_lidar)) < 0) {
		int e = errno;
		fprintf(stderr, "driver: apro %s: %s\n", c->dev_lidar, strerror(e));
		close(d.fd_gd32);
		registra_chiudi();
		return e;
	}
	d.fd_sveglia = eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC);
	if (rt_init() != 0)
		fprintf(stderr, "driver: mlockall non riuscito (sul PC va bene, sul robot no)\n");

	rt_mutex_init(&d.mx);
	int r = crea_thread(&d.thread, ciclo_tempo, c->priorita);
	if (r == 0) {
		r = crea_thread(&d.ricezione, ciclo_ricezione, c->priorita > RT_PRIO_MIN ? c->priorita - 1 : c->priorita);
		if (r) {
			atomic_store(&d.stop, 1);
			pthread_join(d.thread, NULL);
		}
	}
	if (r) {
		fprintf(stderr, "driver: non riesco a creare i thread: %s\n", strerror(r));
		close(d.fd_gd32);
		if (d.fd_lidar >= 0)
			close(d.fd_lidar);
		registra_chiudi();
		return r;
	}
	pthread_attr_t a;
	/* Il thread di servizio con uno stack piccolo: dopo mlockall ogni stack è bloccato in RAM, e quello di default
	 * (8 MB) supera il limite di memoria bloccabile dei normali utenti (sul PC pthread_create fallirebbe). */
	pthread_attr_init(&a);
	pthread_attr_setstacksize(&a, 128 * 1024);
	r = pthread_create(&d.servizio, &a, ciclo_servizio, NULL);
	pthread_attr_destroy(&a);
	if (r) {
		fprintf(stderr, "driver: non riesco a creare il thread di servizio: %s\n", strerror(r));
		atomic_store(&d.stop, 1);
		sveglia_driver();
		pthread_join(d.thread, NULL);
		pthread_join(d.ricezione, NULL);
		close(d.fd_gd32);
		if (d.fd_lidar >= 0)
			close(d.fd_lidar);
		registra_chiudi();
		return r;
	}
	d.avviato = 1;
	return 0;
}

static void sveglia_driver(void)
{
	uint64_t uno = 1;
	if (write(d.fd_sveglia, &uno, sizeof uno) < 0)
		return;
}

void drv_ferma(void)
{
	if (!d.avviato)
		return;
	d.avviato = 0;
	atomic_store(&d.stop, 1);
	sveglia_driver();
	pthread_join(d.ricezione, NULL);
	pthread_join(d.thread, NULL);        /* si sveglia al prossimo evento a tempo: al massimo 20 ms */
	spegni();
	pthread_join(d.servizio, NULL);
	registra_chiudi();
	close(d.fd_gd32);
	if (d.fd_lidar >= 0)
		close(d.fd_lidar);
	close(d.fd_sveglia);
}

uint32_t drv_sensori(drv_sensori_t *out) { return scambio_leggi(&sc_sensori, out); }
uint32_t drv_giro(lidar_giro_t *out) { return scambio_leggi(&sc_giro, out); }
uint32_t drv_stato(drv_stato_t *out) { return scambio_leggi(&sc_stato, out); }

void drv_punto_xy(const lidar_punto_t *p, double *x, double *y)
{
	/* angolo del lidar: 0 = dietro, ORARIO; nel sistema del robot b = 180° - angolo (0 = davanti, antiorario) */
	double b = (180.0 - p->angolo_cdeg / 100.0) * M_PI / 180.0;
	double r = p->distanza_mm / 1000.0;
	/* il punto è a distanza r lungo b, spostato di LIDAR_OTTICO_M a destra del raggio (triangolatore) */
	*x = LIDAR_X_M + r * cos(b) + LIDAR_OTTICO_M * sin(b);
	*y = LIDAR_Y_M + r * sin(b) - LIDAR_OTTICO_M * cos(b);
}

void drv_velocita(double v, double w)
{
	if (!(v == v)) v = 0;                         /* NaN */
	if (!(w == w)) w = 0;
	v = fmax(-0.5, fmin(0.5, v));
	w = fmax(-2 * M_PI, fmin(2 * M_PI, w));
	int32_t vt = (int32_t)lround(v * SCALA_V), wt = (int32_t)lround(w * SCALA_W);
	atomic_store(&cmd.vw, (int64_t)((uint64_t)(uint32_t)vt << 32 | (uint32_t)wt));
	sveglia_driver();
}

void drv_spazzole(int laterale, int centrale)
{
	atomic_store(&cmd.spazzola_lat, laterale);
	atomic_store(&cmd.spazzola_cen, centrale);
	sveglia_driver();
}

void drv_aspirazione(int giri_min)
{
	atomic_store(&cmd.aspirazione, giri_min);
	sveglia_driver();
}

void drv_led(int modo)
{
	atomic_store(&cmd.led, modo);
	sveglia_driver();
}

void drv_lidar(int acceso, double giri_s)
{
	atomic_store(&cmd.lidar_cgiri, giri_s > 0 ? (int)lround(giri_s * 100) : 0);
	atomic_store(&cmd.lidar_acceso, acceso ? 1 : 0);
}

void drv_esci_dalla_base(int si)
{
	atomic_store(&cmd.uscita_base, si ? 1 : 0);
	sveglia_driver();
}

void drv_suono(const char *nome)
{
	atomic_store(&cmd.suono, nome);
}

int drv_azzera_guasto(void)
{
	atomic_store(&cmd.azzera_guasto, 1);
	for (int i = 0; i < 10; i++) {           /* aspetta fino a 2 tick */
		usleep(5000);
		drv_stato_t s;
		if (drv_stato(&s) && s.fase != DRV_GUASTO)
			return 0;
	}
	return -1;
}

int drv_sveglia_ai_sensori(rt_task_t *t)
{
	if (!t->cfg.sporadico)
		return EINVAL;
	for (int i = 0; i < MAX_ISCRITTI; i++) {
		rt_task_t *vuoto = NULL;
		if (atomic_compare_exchange_strong(&iscritti[i], &vuoto, t))
			return 0;
	}
	return ENOSPC;
}

int drv_sorveglia(rt_task_t *t, int critico, int64_t soglia_ns)
{
	for (int i = 0; i < MAX_SORVEGLIATI; i++) {
		if (atomic_load(&sorvegliati[i].t))
			continue;
		sorvegliati[i].critico = critico;
		sorvegliati[i].soglia_ns = soglia_ns > 0 ? soglia_ns : 3 * t->cfg.periodo_ns;
		sorvegliati[i].segnalato = 0;
		rt_task_t *vuoto = NULL;
		if (atomic_compare_exchange_strong(&sorvegliati[i].t, &vuoto, t))
			return 0;
	}
	return ENOSPC;
}

void drv_dimentica(rt_task_t *t)
{
	for (int i = 0; i < MAX_ISCRITTI; i++) {
		rt_task_t *x = t;
		atomic_compare_exchange_strong(&iscritti[i], &x, NULL);
	}
	for (int i = 0; i < MAX_SORVEGLIATI; i++) {
		rt_task_t *x = t;
		atomic_compare_exchange_strong(&sorvegliati[i].t, &x, NULL);
	}
	/* il driver potrebbe avere in mano il puntatore: si aspetta che finisca il risveglio in corso */
	if (d.fase == DRV_FINE || atomic_load(&d.stop))
		return;
	uint64_t g = atomic_load(&giri_driver);
	for (int i = 0; i < 100 && atomic_load(&giri_driver) < g + 2; i++) {
		sveglia_driver();
		usleep(1000);
	}
}
