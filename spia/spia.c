/*
 * spia.c - Intercetta e registra tutto il traffico tra i programmi originali del robot e l'hardware.
 *
 * Comandi:
 *   spia proxy [opzioni]          si mette in mezzo sulle seriali e registra tutto (vedi sotto)
 *   spia leggi <registro> [-c canale]   decodifica il registro: un frame per riga
 *   spia riepilogo <registro>     per ogni canale, direzione e cmd: quanti frame, lunghezze, periodi
 *   spia termios <disp>...        stampa le impostazioni di una seriale (sul robot non c'è stty)
 *
 * Come funziona "proxy", per ogni canale (di default gd32 = /dev/ttyS3 e lidar = /dev/ttyS1):
 *
 *     AuxCtrl ⇄ /dev/ttyS3 = pty (bind mount) ⇄ spia ⇄ /dev/spia-gd32-vero (mknod, stessa UART) ⇄ GD32
 *
 *   1. controlla che nessun processo tenga aperto il dispositivo (altrimenti leggerebbe la UART insieme a noi);
 *   2. crea /dev/spia-<nome>-vero con mknod, con gli stessi major/minor del dispositivo, e lo apre: è la UART vera.
 *      Le impostazioni della UART (velocità, 8N1...) NON si toccano: restano quelle lasciate da AuxCtrl;
 *   3. crea una pty e monta il suo lato "slave" sopra il dispositivo (mount --bind). Da quel momento chi apre
 *      /dev/ttyS3 apre la pty, cioè parla con noi. AuxCtrl non usa le linee di controllo della seriale
 *      (getCTS/getDSR/getDCD non sono mai chiamate), quindi una pty gli basta;
 *   4. inoltra i byte nelle due direzioni appena arrivano e li scrive nel registro (spia/registro.h);
 *   5. all'uscita (Ctrl-C, SIGTERM) smonta tutto: il dispositivo torna quello vero.
 * Gira con SCHED_FIFO e la memoria bloccata in RAM, così non resta mai indietro rispetto alla UART.
 *
 * Sequenza d'uso sul robot: si ferma lo stack originale, si avvia "spia proxy", si riavvia lo stack.
 * Con --senza-mount (prove sul PC) la spia apre il dispositivo indicato così com'è e stampa il percorso della
 * pty che fa da finto dispositivo.
 */
#define _GNU_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <limits.h>
#include <poll.h>
#include <sched.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <dirent.h>
#include <sys/mount.h>
#include <sys/stat.h>
#include <sys/sysmacros.h>
#include <termios.h>
#include <unistd.h>

#include "../gd32/gd32.h"
#include "../lidar/lidar.h"
#include "../rt/rt.h"
#include "registro.h"

/* ------------------------------------------------------------------------------------------------------ */
/* Decodifica in tempo reale, solo per le statistiche (il registro contiene comunque i byte grezzi).       */

typedef struct {
	uint8_t protocollo;
	gd32_deframer_t gd32;
	lidar_deframer_t lidar;
} decod_t;

static void decod_init(decod_t *d, uint8_t protocollo)
{
	d->protocollo = protocollo;
	gd32_deframer_init(&d->gd32);
	lidar_deframer_init(&d->lidar);
}

static void decod_scrivi(decod_t *d, const uint8_t *b, size_t n, gd32_frame_cb cg, lidar_pacchetto_cb cl, void *ctx)
{
	if (d->protocollo == REG_PROTO_GD32)
		gd32_deframer_scrivi(&d->gd32, b, n, cg, ctx);
	else if (d->protocollo == REG_PROTO_LIDAR)
		lidar_deframer_scrivi(&d->lidar, b, n, cl, ctx);
}

static void decod_contatori(const decod_t *d, uint64_t *ok, uint64_t *err, uint64_t *scartati)
{
	*ok = *err = *scartati = 0;
	if (d->protocollo == REG_PROTO_GD32) {
		*ok = d->gd32.frame_ok;
		*err = d->gd32.checksum_errati + d->gd32.len_errati;
		*scartati = d->gd32.byte_scartati;
	} else if (d->protocollo == REG_PROTO_LIDAR) {
		*ok = d->lidar.pacchetti_ok;
		*err = d->lidar.checksum_errati + d->lidar.len_errati;
		*scartati = d->lidar.byte_scartati;
	}
}

/* ------------------------------------------------------------------------------------------------------ */
/* proxy                                                                                                  */

typedef struct {
	reg_canale_t info;
	char vero[PATH_MAX];      /* nodo creato con mknod, oppure il dispositivo stesso con --senza-mount */
	int fd_vero;              /* la periferica vera */
	int fd_master;            /* lato master della pty: ci parliamo noi */
	int fd_slave;             /* lo teniamo aperto anche noi, così la pty non va in errore senza client */
	char slave[PATH_MAX];
	int montato, nodo_creato;
	uint64_t byte[2], blocchi[2], persi[2];
	decod_t decod[2];
} canale_t;

static volatile sig_atomic_t fermati;
static void al_segnale(int s) { (void)s; fermati = 1; }

static int imposta_raw(int fd)
{
	struct termios t;
	if (tcgetattr(fd, &t))
		return -1;
	cfmakeraw(&t);
	return tcsetattr(fd, TCSANOW, &t);
}

/* Elenca su stderr i processi che tengono aperto `disp`. Ritorna quanti sono. */
static int chi_lo_usa(const char *disp)
{
	int quanti = 0;
	DIR *proc = opendir("/proc");
	if (!proc)
		return 0;
	struct dirent *p;
	while ((p = readdir(proc))) {
		if (p->d_name[0] < '1' || p->d_name[0] > '9' || atoi(p->d_name) == getpid())
			continue;
		char dir[300];
		snprintf(dir, sizeof dir, "/proc/%s/fd", p->d_name);
		DIR *fds = opendir(dir);
		if (!fds)
			continue;
		struct dirent *f;
		while ((f = readdir(fds))) {
			char link[600], dest[PATH_MAX];
			snprintf(link, sizeof link, "%s/%s", dir, f->d_name);
			ssize_t k = readlink(link, dest, sizeof dest - 1);
			if (k <= 0)
				continue;
			dest[k] = 0;
			if (!strcmp(dest, disp)) {
				char comm[64] = "?", pc[300];
				snprintf(pc, sizeof pc, "/proc/%s/comm", p->d_name);
				FILE *c = fopen(pc, "r");
				if (c) {
					if (fgets(comm, sizeof comm, c))
						comm[strcspn(comm, "\n")] = 0;
					fclose(c);
				}
				fprintf(stderr, "spia: %s è aperto dal processo %s (%s)\n", disp, p->d_name, comm);
				quanti++;
				break;
			}
		}
		closedir(fds);
	}
	closedir(proc);
	return quanti;
}

static int apri_canale(canale_t *c, int senza_mount)
{
	const char *disp = c->info.dispositivo;
	c->fd_vero = c->fd_master = c->fd_slave = -1;

	if (senza_mount) {
		snprintf(c->vero, sizeof c->vero, "%s", disp);
	} else {
		struct stat st;
		if (stat(disp, &st) || !S_ISCHR(st.st_mode)) {
			fprintf(stderr, "spia: %s non è un dispositivo a caratteri\n", disp);
			return -1;
		}
		/* in /dev e non in /tmp: sul robot /tmp è montata nodev e lì un nodo di dispositivo non si apre */
		snprintf(c->vero, sizeof c->vero, "/dev/spia-%s-vero", c->info.nome);
		unlink(c->vero);
		if (mknod(c->vero, S_IFCHR | 0600, st.st_rdev)) {
			fprintf(stderr, "spia: mknod %s: %s\n", c->vero, strerror(errno));
			return -1;
		}
		c->nodo_creato = 1;
	}
	c->fd_vero = open(c->vero, O_RDWR | O_NOCTTY | O_NONBLOCK);
	if (c->fd_vero < 0) {
		fprintf(stderr, "spia: apro %s: %s\n", c->vero, strerror(errno));
		return -1;
	}

	c->fd_master = posix_openpt(O_RDWR | O_NOCTTY | O_NONBLOCK);
	if (c->fd_master < 0 || grantpt(c->fd_master) || unlockpt(c->fd_master) ||
	    ptsname_r(c->fd_master, c->slave, sizeof c->slave)) {
		fprintf(stderr, "spia: creazione della pty: %s\n", strerror(errno));
		return -1;
	}
	c->fd_slave = open(c->slave, O_RDWR | O_NOCTTY);
	if (c->fd_slave < 0 || imposta_raw(c->fd_slave)) {
		fprintf(stderr, "spia: apro %s: %s\n", c->slave, strerror(errno));
		return -1;
	}
	/* stessi permessi del dispositivo originale, così chi lo apriva può ancora farlo */
	struct stat st;
	if (!senza_mount && stat(disp, &st) == 0)
		fchmod(c->fd_slave, st.st_mode & 07777);

	if (!senza_mount) {
		if (mount(c->slave, disp, NULL, MS_BIND, NULL)) {
			fprintf(stderr, "spia: mount --bind %s %s: %s\n", c->slave, disp, strerror(errno));
			return -1;
		}
		c->montato = 1;
	}
	decod_init(&c->decod[REG_VERSO_SOC], c->info.protocollo);
	decod_init(&c->decod[REG_DAL_SOC], c->info.protocollo);
	return 0;
}

static void chiudi_canale(canale_t *c)
{
	if (c->montato && umount2(c->info.dispositivo, MNT_DETACH))
		fprintf(stderr, "spia: ATTENZIONE, umount %s: %s\n", c->info.dispositivo, strerror(errno));
	c->montato = 0;
	if (c->fd_slave >= 0) close(c->fd_slave);
	if (c->fd_master >= 0) close(c->fd_master);
	if (c->fd_vero >= 0) close(c->fd_vero);
	c->fd_slave = c->fd_master = c->fd_vero = -1;
	if (c->nodo_creato)
		unlink(c->vero);
	c->nodo_creato = 0;
}

/* Scrive tutto il blocco su fd, aspettando al massimo attesa_ms. Ritorna i byte NON scritti. */
static size_t scrivi_tutto(int fd, const uint8_t *b, size_t n, int attesa_ms)
{
	int64_t limite = rt_adesso() + RT_MS(attesa_ms);
	while (n > 0) {
		ssize_t k = write(fd, b, n);
		if (k > 0) {
			b += k;
			n -= (size_t)k;
			continue;
		}
		if (k < 0 && errno != EAGAIN && errno != EINTR)
			break;
		int64_t resto = limite - rt_adesso();
		if (resto <= 0)
			break;
		struct pollfd p = { .fd = fd, .events = POLLOUT };
		poll(&p, 1, (int)(resto / RT_NS_PER_MS) + 1);
	}
	return n;
}

static void stampa_stato(canale_t *cc, int n, int64_t t0)
{
	fprintf(stderr, "[%7.1f s]", (double)(rt_adesso() - t0) / RT_NS_PER_S);
	for (int i = 0; i < n; i++) {
		canale_t *c = &cc[i];
		fprintf(stderr, "  %s:", c->info.nome);
		for (int d = 0; d < 2; d++) {
			uint64_t ok, err, sc;
			decod_contatori(&c->decod[d], &ok, &err, &sc);
			fprintf(stderr, " %s %llu B", d == REG_VERSO_SOC ? "rx" : "tx", (unsigned long long)c->byte[d]);
			if (c->info.protocollo != REG_PROTO_GREZZO)
				fprintf(stderr, " %llu fr %llu err", (unsigned long long)ok, (unsigned long long)err);
			if (c->persi[d])
				fprintf(stderr, " %llu PERSI", (unsigned long long)c->persi[d]);
			fprintf(stderr, d == 0 ? "," : "");
		}
	}
	fputc('\n', stderr);
}

static int analizza_canale(const char *s, reg_canale_t *c, uint8_t id)
{
	/* formato: nome,protocollo,dispositivo   es. gd32,gd32,/dev/ttyS3 */
	char tmp[256];
	snprintf(tmp, sizeof tmp, "%s", s);
	char *nome = strtok(tmp, ","), *proto = strtok(NULL, ","), *disp = strtok(NULL, ",");
	if (!nome || !proto || !disp || reg_protocollo_da_nome(proto) < 0)
		return -1;
	memset(c, 0, sizeof(*c));
	c->id = id;
	c->protocollo = (uint8_t)reg_protocollo_da_nome(proto);
	snprintf(c->nome, sizeof c->nome, "%s", nome);
	snprintf(c->dispositivo, sizeof c->dispositivo, "%s", disp);
	return 0;
}

static int cmd_proxy(int argc, char **argv)
{
	const char *uscita = NULL;
	int priorita = 80, senza_mount = 0, forza = 0, ogni_s = 10;
	static canale_t cc[REG_MAX_CANALI];
	int n = 0;

	static const struct option opz[] = {
		{ "uscita", required_argument, 0, 'o' }, { "canale", required_argument, 0, 'c' },
		{ "priorita", required_argument, 0, 'p' }, { "stato", required_argument, 0, 's' },
		{ "senza-mount", no_argument, 0, 'M' }, { "forza", no_argument, 0, 'F' }, { 0, 0, 0, 0 },
	};
	int o;
	while ((o = getopt_long(argc, argv, "o:c:p:s:", opz, NULL)) != -1) {
		switch (o) {
		case 'o': uscita = optarg; break;
		case 'p': priorita = atoi(optarg); break;
		case 's': ogni_s = atoi(optarg); break;
		case 'M': senza_mount = 1; break;
		case 'F': forza = 1; break;
		case 'c':
			if (n == REG_MAX_CANALI || analizza_canale(optarg, &cc[n].info, (uint8_t)n)) {
				fprintf(stderr, "spia: canale non valido: %s (formato nome,gd32|lidar|grezzo,dispositivo)\n", optarg);
				return 2;
			}
			n++;
			break;
		default: return 2;
		}
	}
	if (!uscita) {
		fprintf(stderr, "spia proxy: manca -o <registro>\n");
		return 2;
	}
	if (n == 0) {
		analizza_canale("gd32,gd32,/dev/ttyS3", &cc[n].info, (uint8_t)n); n++;
		analizza_canale("lidar,lidar,/dev/ttyS1", &cc[n].info, (uint8_t)n); n++;
	}

	if (!senza_mount) {
		int occupati = 0;
		for (int i = 0; i < n; i++)
			occupati += chi_lo_usa(cc[i].info.dispositivo);
		if (occupati && !forza) {
			fprintf(stderr, "spia: prima ferma chi usa i dispositivi (es. /etc/init.d/robotManager stop)\n");
			return 1;
		}
	}

	FILE *reg = fopen(uscita, "wb");
	if (!reg) {
		fprintf(stderr, "spia: apro %s: %s\n", uscita, strerror(errno));
		return 1;
	}
	static char bufreg[1 << 16];
	setvbuf(reg, bufreg, _IOFBF, sizeof bufreg);
	reg_intestazione_t h = { .n_canali = (uint8_t)n };
	for (int i = 0; i < n; i++)
		h.canali[i] = cc[i].info;
	reg_scrivi_intestazione(reg, &h);

	struct sigaction sa = { .sa_handler = al_segnale };
	sigaction(SIGINT, &sa, NULL);
	sigaction(SIGTERM, &sa, NULL);
	sigaction(SIGHUP, &sa, NULL);
	signal(SIGPIPE, SIG_IGN);

	int esito = 0;
	for (int i = 0; i < n; i++) {
		if (apri_canale(&cc[i], senza_mount)) {
			esito = 1;
			goto fine;
		}
		printf("canale %s: %s <-> %s (pty %s)\n", cc[i].info.nome, cc[i].info.dispositivo, cc[i].vero, cc[i].slave);
	}
	fflush(stdout);

	/* Tempo reale: memoria bloccata e SCHED_FIFO. Se non si può (sul PC senza root) si va avanti lo stesso. */
	if (rt_init())
		fprintf(stderr, "spia: mlockall non riuscito (si continua)\n");
	struct sched_param sp = { .sched_priority = priorita };
	if (priorita > 0 && sched_setscheduler(0, SCHED_FIFO, &sp))
		fprintf(stderr, "spia: SCHED_FIFO %d non riuscito: %s (si continua)\n", priorita, strerror(errno));

	struct pollfd pf[2 * REG_MAX_CANALI];
	static uint8_t buf[4096];
	int64_t t0 = rt_adesso(), prossimo_stato = t0 + RT_S(ogni_s), prossimo_flush = t0 + RT_S(1);

	while (!fermati) {
		for (int i = 0; i < n; i++) {
			pf[2 * i] = (struct pollfd){ .fd = cc[i].fd_vero, .events = POLLIN };
			pf[2 * i + 1] = (struct pollfd){ .fd = cc[i].fd_master, .events = POLLIN };
		}
		int r = poll(pf, (nfds_t)(2 * n), 200);
		if (r < 0 && errno != EINTR) {
			perror("spia: poll");
			esito = 1;
			break;
		}
		int64_t adesso = rt_adesso();
		for (int i = 0; i < n && r > 0; i++) {
			canale_t *c = &cc[i];
			for (int lato = 0; lato < 2; lato++) {
				struct pollfd *p = &pf[2 * i + lato];
				if (!(p->revents & (POLLIN | POLLHUP | POLLERR)))
					continue;
				ssize_t k = read(p->fd, buf, sizeof buf);
				if (k <= 0)
					continue;   /* EAGAIN, oppure la pty senza client (EIO): si riprova al giro dopo */
				/* lato 0: dalla periferica verso il SoC; lato 1: dai programmi verso la periferica */
				int dir = lato == 0 ? REG_VERSO_SOC : REG_DAL_SOC;
				int dest = lato == 0 ? c->fd_master : c->fd_vero;
				reg_scrivi_record(reg, adesso, c->info.id, (uint8_t)dir, buf, (uint16_t)k);
				c->byte[dir] += (uint64_t)k;
				c->blocchi[dir]++;
				c->persi[dir] += scrivi_tutto(dest, buf, (size_t)k, 20);
				decod_scrivi(&c->decod[dir], buf, (size_t)k, NULL, NULL, NULL);
			}
		}
		if (adesso >= prossimo_flush) {
			fflush(reg);
			prossimo_flush = adesso + RT_S(1);
		}
		if (ogni_s > 0 && adesso >= prossimo_stato) {
			stampa_stato(cc, n, t0);
			prossimo_stato = adesso + RT_S(ogni_s);
		}
	}
	stampa_stato(cc, n, t0);

fine:
	for (int i = 0; i < n; i++)
		chiudi_canale(&cc[i]);
	if (fclose(reg)) {
		fprintf(stderr, "spia: chiudo %s: %s\n", uscita, strerror(errno));
		esito = 1;
	}
	return esito;
}

/* ------------------------------------------------------------------------------------------------------ */
/* leggi / riepilogo                                                                                      */

typedef struct {
	const reg_record_t *rec;
	const reg_canale_t *can;
	FILE *out;
} ctx_leggi_t;

static void stampa_testa(const ctx_leggi_t *x)
{
	fprintf(x->out, "%.6f %s %s ", (double)x->rec->tempo_ns / RT_NS_PER_S, x->can->nome,
	        x->rec->direzione == REG_VERSO_SOC ? "rx" : "tx");
}

static void stampa_hex(FILE *out, const uint8_t *b, size_t n)
{
	for (size_t i = 0; i < n; i++)
		fprintf(out, i ? " %02x" : "%02x", b[i]);
	fputc('\n', out);
}

static void leggi_gd32(const gd32_frame_t *f, void *ctx)
{
	const ctx_leggi_t *x = ctx;
	const char *nome = gd32_nome_cmd(f->cmd);
	stampa_testa(x);
	fprintf(x->out, "0x%02x %s %u ", f->cmd, nome ? nome : "?", f->n_dati);
	stampa_hex(x->out, f->dati, f->n_dati);
}

static void leggi_lidar(const lidar_pacchetto_t *p, void *ctx)
{
	const ctx_leggi_t *x = ctx;
	const char *nome = lidar_nome_cmd(p->cmd);
	stampa_testa(x);
	fprintf(x->out, "0x%02x %s %u ", p->cmd, nome ? nome : "?", p->n_param);
	stampa_hex(x->out, p->param, p->n_param);
}

/* Statistiche per (canale, direzione, cmd). */
typedef struct {
	uint64_t n;
	unsigned len_min, len_max;
	int64_t primo, ultimo, dt_min, dt_max;
} stat_cmd_t;

static stat_cmd_t statistiche[REG_MAX_CANALI][2][256];

typedef struct {
	const reg_record_t *rec;
} ctx_riep_t;

static void conta_cmd(uint8_t can, uint8_t dir, uint8_t cmd, unsigned len, int64_t t)
{
	stat_cmd_t *s = &statistiche[can][dir][cmd];
	if (s->n == 0) {
		s->len_min = s->len_max = len;
		s->primo = t;
		s->dt_min = INT64_MAX;
	} else {
		int64_t dt = t - s->ultimo;
		if (dt < s->dt_min) s->dt_min = dt;
		if (dt > s->dt_max) s->dt_max = dt;
		if (len < s->len_min) s->len_min = len;
		if (len > s->len_max) s->len_max = len;
	}
	s->ultimo = t;
	s->n++;
}

static void riep_gd32(const gd32_frame_t *f, void *ctx)
{
	const reg_record_t *r = ((ctx_riep_t *)ctx)->rec;
	conta_cmd(r->canale, r->direzione, f->cmd, f->n_dati, r->tempo_ns);
}

static void riep_lidar(const lidar_pacchetto_t *p, void *ctx)
{
	const reg_record_t *r = ((ctx_riep_t *)ctx)->rec;
	conta_cmd(r->canale, r->direzione, p->cmd, p->n_param, r->tempo_ns);
}

static int cmd_leggi(int argc, char **argv, int riepilogo)
{
	const char *solo = NULL;
	int o;
	while ((o = getopt(argc, argv, "c:")) != -1) {
		if (o == 'c') solo = optarg;
		else return 2;
	}
	if (optind >= argc) {
		fprintf(stderr, "uso: spia %s <registro>%s\n", riepilogo ? "riepilogo" : "leggi", riepilogo ? "" : " [-c canale]");
		return 2;
	}
	FILE *f = fopen(argv[optind], "rb");
	if (!f) {
		fprintf(stderr, "spia: apro %s: %s\n", argv[optind], strerror(errno));
		return 1;
	}
	reg_intestazione_t h;
	if (reg_leggi_intestazione(f, &h) != 1) {
		fprintf(stderr, "spia: %s non è un registro della spia\n", argv[optind]);
		fclose(f);
		return 1;
	}
	static decod_t dec[REG_MAX_CANALI][2];
	static uint64_t byte[REG_MAX_CANALI][2];
	for (int i = 0; i < h.n_canali; i++)
		for (int d = 0; d < 2; d++)
			decod_init(&dec[i][d], h.canali[i].protocollo);

	static reg_record_t r;
	int64_t t_primo = -1, t_ultimo = 0;
	int esito;
	while ((esito = reg_leggi_record(f, &r)) == 1) {
		if (r.canale >= h.n_canali || r.direzione > 1)
			continue;
		const reg_canale_t *c = &h.canali[r.canale];
		if (solo && strcmp(solo, c->nome))
			continue;
		if (t_primo < 0) t_primo = r.tempo_ns;
		t_ultimo = r.tempo_ns;
		byte[r.canale][r.direzione] += r.n;
		if (riepilogo) {
			ctx_riep_t x = { &r };
			decod_scrivi(&dec[r.canale][r.direzione], r.dati, r.n, riep_gd32, riep_lidar, &x);
		} else {
			ctx_leggi_t x = { &r, c, stdout };
			if (c->protocollo == REG_PROTO_GREZZO) {
				stampa_testa(&x);
				fprintf(stdout, "grezzo %u ", r.n);
				stampa_hex(stdout, r.dati, r.n);
			} else {
				decod_scrivi(&dec[r.canale][r.direzione], r.dati, r.n, leggi_gd32, leggi_lidar, &x);
			}
		}
	}
	fclose(f);
	if (esito < 0)
		fprintf(stderr, "spia: il registro finisce a metà di un record (copiato mentre la spia scriveva?)\n");

	if (riepilogo) {
		printf("durata %.1f s\n", t_primo < 0 ? 0.0 : (double)(t_ultimo - t_primo) / RT_NS_PER_S);
		for (int i = 0; i < h.n_canali; i++) {
			const reg_canale_t *c = &h.canali[i];
			if (solo && strcmp(solo, c->nome))
				continue;
			for (int d = 0; d < 2; d++) {
				uint64_t ok, err, sc;
				decod_contatori(&dec[i][d], &ok, &err, &sc);
				printf("\n== %s (%s, %s) %s: %llu byte, %llu frame, %llu errori, %llu byte scartati\n",
				       c->nome, c->dispositivo, reg_nome_protocollo(c->protocollo),
				       d == REG_VERSO_SOC ? "periferica -> SoC" : "SoC -> periferica",
				       (unsigned long long)byte[i][d], (unsigned long long)ok, (unsigned long long)err,
				       (unsigned long long)sc);
				int titolo = 0;
				for (int k = 0; k < 256; k++) {
					stat_cmd_t *s = &statistiche[i][d][k];
					if (!s->n)
						continue;
					if (!titolo++)
						printf("  cmd  nome                    frame   dati(byte)   periodo medio/min/max (ms)\n");
					const char *nome = c->protocollo == REG_PROTO_LIDAR ? lidar_nome_cmd((uint8_t)k)
					                                                    : gd32_nome_cmd((uint8_t)k);
					printf("  0x%02x %-22s %7llu   %3u-%-3u", k, nome ? nome : "?", (unsigned long long)s->n,
					       s->len_min, s->len_max);
					if (s->n > 1)
						printf("    %9.1f %9.1f %9.1f", (double)(s->ultimo - s->primo) / (double)(s->n - 1) / 1e6,
						       (double)s->dt_min / 1e6, (double)s->dt_max / 1e6);
					putchar('\n');
				}
			}
		}
	}
	return esito < 0 ? 1 : 0;
}

/* ------------------------------------------------------------------------------------------------------ */
/* termios                                                                                                */

/* Dai bit CBAUD di c_cflag: con la musl e il kernel 3.4 del robot cfgetispeed() restituisce 0. */
static long velocita(speed_t s)
{
	switch (s) {
	case B9600: return 9600;     case B19200: return 19200;   case B38400: return 38400;
	case B57600: return 57600;   case B115200: return 115200; case B230400: return 230400;
	case B460800: return 460800; case B921600: return 921600; default: return -1;
	}
}

static int cmd_termios(int argc, char **argv)
{
	int esito = 0;
	for (int i = 1; i < argc; i++) {
		int fd = open(argv[i], O_RDONLY | O_NOCTTY | O_NONBLOCK);
		struct termios t;
		if (fd < 0 || tcgetattr(fd, &t)) {
			fprintf(stderr, "spia: %s: %s\n", argv[i], strerror(errno));
			esito = 1;
			if (fd >= 0) close(fd);
			continue;
		}
		close(fd);
		int bit = (t.c_cflag & CSIZE) == CS8 ? 8 : (t.c_cflag & CSIZE) == CS7 ? 7 : (t.c_cflag & CSIZE) == CS6 ? 6 : 5;
		char parita = !(t.c_cflag & PARENB) ? 'N' : (t.c_cflag & PARODD) ? 'O' : 'E';
		printf("%s: %ld baud %d%c%d, %s, %s, %s, %s, VMIN=%d VTIME=%d\n"
		       "  iflag=0%o oflag=0%o cflag=0%o lflag=0%o\n",
		       argv[i], velocita(t.c_cflag & CBAUD), bit, parita, (t.c_cflag & CSTOPB) ? 2 : 1,
		       (t.c_cflag & CRTSCTS) ? "controllo di flusso RTS/CTS" : "senza controllo di flusso hardware",
		       (t.c_iflag & (IXON | IXOFF)) ? "XON/XOFF attivo" : "senza XON/XOFF",
		       (t.c_lflag & ICANON) ? "modo canonico (a righe)" : "modo raw",
		       (t.c_lflag & ECHO) ? "eco attiva" : "senza eco",
		       t.c_cc[VMIN], t.c_cc[VTIME],
		       (unsigned)t.c_iflag, (unsigned)t.c_oflag, (unsigned)t.c_cflag, (unsigned)t.c_lflag);
	}
	return esito;
}

/* ------------------------------------------------------------------------------------------------------ */

static void uso(void)
{
	fprintf(stderr,
	        "uso: spia proxy -o <registro> [-c nome,protocollo,dispositivo]... [-p priorità] [-s secondi]\n"
	        "                [--senza-mount] [--forza]\n"
	        "       (canali di default: gd32,gd32,/dev/ttyS3 e lidar,lidar,/dev/ttyS1)\n"
	        "     spia leggi <registro> [-c canale]\n"
	        "     spia riepilogo <registro> [-c canale]\n"
	        "     spia termios <dispositivo>...\n");
}

int main(int argc, char **argv)
{
	if (argc < 2) {
		uso();
		return 2;
	}
	const char *cmd = argv[1];
	argv++;
	argc--;
	if (!strcmp(cmd, "proxy"))     return cmd_proxy(argc, argv);
	if (!strcmp(cmd, "leggi"))     return cmd_leggi(argc, argv, 0);
	if (!strcmp(cmd, "riepilogo")) return cmd_leggi(argc, argv, 1);
	if (!strcmp(cmd, "termios"))   return cmd_termios(argc, argv);
	uso();
	return 2;
}
