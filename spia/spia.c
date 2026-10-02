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
#include "everest.h"
#include "registro.h"

#include <sys/socket.h>
#include <sys/un.h>

/* ------------------------------------------------------------------------------------------------------ */
/* Decodifica in tempo reale, solo per le statistiche (il registro contiene comunque i byte grezzi).       */

typedef struct {
	uint8_t protocollo;
	gd32_deframer_t gd32;
	lidar_deframer_t lidar;
	everest_deframer_t everest;
} decod_t;

/* Callback dei messaggi everest: impostata da chi chiama decod_scrivi (NULL = solo contatori). */
static everest_msg_cb decod_cb_everest;

static void decod_init(decod_t *d, uint8_t protocollo)
{
	d->protocollo = protocollo;
	gd32_deframer_init(&d->gd32);
	lidar_deframer_init(&d->lidar);
	everest_deframer_init(&d->everest);
}

static void decod_scrivi(decod_t *d, const uint8_t *b, size_t n, gd32_frame_cb cg, lidar_pacchetto_cb cl, void *ctx)
{
	if (d->protocollo == REG_PROTO_GD32)
		gd32_deframer_scrivi(&d->gd32, b, n, cg, ctx);
	else if (d->protocollo == REG_PROTO_LIDAR)
		lidar_deframer_scrivi(&d->lidar, b, n, cl, ctx);
	else if (d->protocollo == REG_PROTO_EVEREST)
		everest_deframer_scrivi(&d->everest, b, n, (cg || cl) ? decod_cb_everest : NULL, ctx);
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
	} else if (d->protocollo == REG_PROTO_EVEREST) {
		*ok = d->everest.messaggi_ok;
		*err = d->everest.testate_errate;
		*scartati = d->everest.byte_scartati;
	}
}


/* ------------------------------------------------------------------------------------------------------ */
/* Eventi in tempo reale (--eventi FILE): per le prove dei sensori.                                       */
/*
 * Scrive una riga per ogni cosa "interessante":
 *   - nel frame dei sensori 0x15, i byte che cambiano rispetto al frame precedente. Nei primi --impara secondi
 *     dopo --attesa secondi (il tempo che lo stack originale finisca di avviarsi; robot fermo) la spia impara quali byte cambiano da soli (IMU, temperature, rumore dei sensori analogici)
 *     e poi li ignora. Un byte che dopo cambia più di 10 volte in un secondo diventa anche lui "rumoroso";
 *   - ogni altro frame del GD32 (esclusi l'heartbeat 0x06 e i sensori), in entrambe le direzioni;
 *   - il comando di velocità 0x66 solo quando cambia;
 *   - i pacchetti del lidar diversi dalle scansioni.
 */
#define EV_SENSORI 96

typedef struct {
	FILE *out;
	int64_t t0, inizio_apprendimento, fine_apprendimento;
	uint8_t prec[EV_SENSORI];
	int ha_prec;
	uint8_t rumoroso[EV_SENSORI];
	int64_t finestra[EV_SENSORI];      /* inizio della finestra di 1 s per contare i cambi */
	int cambi[EV_SENSORI];
	uint8_t vel_prec[8];
	int ha_vel;
	const char *istantanea;            /* file con min/max di ogni byte dei sensori nell'ultimo secondo */
	uint8_t win_min[EV_SENSORI], win_max[EV_SENSORI];
	int64_t win_inizio;
	int dir;                           /* direzione del blocco in corso */
	int64_t adesso;
} eventi_t;

static eventi_t ev;

static void ev_testa(void)
{
	fprintf(ev.out, "%8.2f %s ", (double)(ev.adesso - ev.t0) / RT_NS_PER_S, ev.dir == REG_VERSO_SOC ? "rx" : "tx");
}

static void ev_frame(const gd32_frame_t *f)
{
	const char *nome = gd32_nome_cmd(f->cmd);
	ev_testa();
	fprintf(ev.out, "0x%02x %s:", f->cmd, nome ? nome : "?");
	for (int i = 0; i < f->n_dati; i++)
		fprintf(ev.out, " %02x", f->dati[i]);
	fputc('\n', ev.out);
}

/* Ogni secondo riscrive il file dell'istantanea: tre righe da 96 byte esadecimali (min, max, ultimo frame). */
static void ev_istantanea(const gd32_frame_t *f, int n)
{
	if (!ev.istantanea)
		return;
	if (ev.win_inizio == 0) {
		memcpy(ev.win_min, f->dati, (size_t)n);
		memcpy(ev.win_max, f->dati, (size_t)n);
		ev.win_inizio = ev.adesso;
	}
	for (int i = 0; i < n; i++) {
		if (f->dati[i] < ev.win_min[i]) ev.win_min[i] = f->dati[i];
		if (f->dati[i] > ev.win_max[i]) ev.win_max[i] = f->dati[i];
	}
	if (ev.adesso - ev.win_inizio < RT_S(1))
		return;
	char tmp[PATH_MAX];
	snprintf(tmp, sizeof tmp, "%s.tmp", ev.istantanea);
	FILE *o = fopen(tmp, "w");
	if (o) {
		fprintf(o, "t %.2f\n", (double)(ev.adesso - ev.t0) / RT_NS_PER_S);
		const uint8_t *righe[3] = { ev.win_min, ev.win_max, f->dati };
		const char *nomi[3] = { "min", "max", "ultimo" };
		for (int r = 0; r < 3; r++) {
			fprintf(o, "%s", nomi[r]);
			for (int i = 0; i < n; i++)
				fprintf(o, " %02x", righe[r][i]);
			fputc('\n', o);
		}
		fclose(o);
		rename(tmp, ev.istantanea);   /* chi legge vede sempre un file completo */
	}
	ev.win_inizio = 0;
}

static void ev_sensori(const gd32_frame_t *f)
{
	int n = f->n_dati < EV_SENSORI ? f->n_dati : EV_SENSORI;
	ev_istantanea(f, n);
	if (!ev.ha_prec) {
		memcpy(ev.prec, f->dati, (size_t)n);
		ev.ha_prec = 1;
		return;
	}
	if (ev.adesso < ev.inizio_apprendimento) {   /* lo stack si sta ancora avviando: si osserva e basta */
		memcpy(ev.prec, f->dati, (size_t)n);
		return;
	}
	int apprendo = ev.adesso < ev.fine_apprendimento;
	int scritto = 0;
	for (int i = 0; i < n; i++) {
		if (f->dati[i] == ev.prec[i] || ev.rumoroso[i])
			continue;
		if (apprendo) {
			ev.rumoroso[i] = 1;
			continue;
		}
		if (ev.adesso - ev.finestra[i] > RT_S(1)) {
			ev.finestra[i] = ev.adesso;
			ev.cambi[i] = 0;
		}
		if (++ev.cambi[i] > 10) {
			ev.rumoroso[i] = 1;
			ev_testa();
			fprintf(ev.out, "(byte %d dei sensori cambia troppo spesso: da ora lo ignoro)\n", i);
			continue;
		}
		if (!scritto++) {
			ev_testa();
			fprintf(ev.out, "sensori");
		}
		fprintf(ev.out, "  [%d] %02x->%02x", i, ev.prec[i], f->dati[i]);
	}
	if (scritto)
		fputc('\n', ev.out);
	if (apprendo == 0 && ev.fine_apprendimento && ev.t0) {
		/* stampa una volta sola l'elenco dei byte ignorati, alla fine dell'apprendimento */
		static int detto;
		if (!detto++) {
			ev_testa();
			fprintf(ev.out, "fine apprendimento; byte dei sensori ignorati perché cambiano da soli:");
			for (int i = 0; i < EV_SENSORI; i++)
				if (ev.rumoroso[i])
					fprintf(ev.out, " %d", i);
			fputc('\n', ev.out);
		}
	}
	memcpy(ev.prec, f->dati, (size_t)n);
}

static void ev_gd32(const gd32_frame_t *f, void *ctx)
{
	(void)ctx;
	if (f->cmd == GD32_HEARTBEAT)
		return;
	if (ev.dir == REG_VERSO_SOC && f->cmd == GD32_SENSORI) {
		ev_sensori(f);
		return;
	}
	if (ev.dir == REG_DAL_SOC && f->cmd == GD32_VELOCITA && f->n_dati == 8) {
		if (ev.ha_vel && !memcmp(ev.vel_prec, f->dati, 8))
			return;
		memcpy(ev.vel_prec, f->dati, 8);
		ev.ha_vel = 1;
	}
	ev_frame(f);
}

static void ev_lidar(const lidar_pacchetto_t *p, void *ctx)
{
	(void)ctx;
	if (p->cmd == LIDAR_SCANSIONE || p->cmd == LIDAR_SCANSIONE_NUOVA)
		return;
	const char *nome = lidar_nome_cmd(p->cmd);
	ev_testa();
	fprintf(ev.out, "lidar 0x%02x %s:", p->cmd, nome ? nome : "?");
	for (int i = 0; i < p->n_param; i++)
		fprintf(ev.out, " %02x", p->param[i]);
	fputc('\n', ev.out);
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
	int priorita = 80, senza_mount = 0, forza = 0, ogni_s = 10, impara_s = 10, attesa_s = 0;
	const char *file_eventi = NULL;
	static canale_t cc[REG_MAX_CANALI];
	int n = 0;

	static const struct option opz[] = {
		{ "uscita", required_argument, 0, 'o' }, { "canale", required_argument, 0, 'c' },
		{ "priorita", required_argument, 0, 'p' }, { "stato", required_argument, 0, 's' },
		{ "senza-mount", no_argument, 0, 'M' }, { "forza", no_argument, 0, 'F' },
		{ "eventi", required_argument, 0, 'E' }, { "impara", required_argument, 0, 'I' },
		{ "attesa", required_argument, 0, 'A' },
		{ "istantanea", required_argument, 0, 'T' }, { 0, 0, 0, 0 },
	};
	int o;
	while ((o = getopt_long(argc, argv, "o:c:p:s:", opz, NULL)) != -1) {
		switch (o) {
		case 'o': uscita = optarg; break;
		case 'p': priorita = atoi(optarg); break;
		case 's': ogni_s = atoi(optarg); break;
		case 'M': senza_mount = 1; break;
		case 'F': forza = 1; break;
		case 'E': file_eventi = optarg; break;
		case 'I': impara_s = atoi(optarg); break;
		case 'A': attesa_s = atoi(optarg); break;
		case 'T': ev.istantanea = optarg; break;
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
	static decod_t dec_ev[REG_MAX_CANALI][2];
	if (file_eventi) {
		ev.out = fopen(file_eventi, "w");
		if (!ev.out) {
			fprintf(stderr, "spia: apro %s: %s\n", file_eventi, strerror(errno));
			esito = 1;
			goto fine;
		}
		setvbuf(ev.out, NULL, _IOLBF, 0);
		ev.t0 = t0;
		ev.inizio_apprendimento = t0 + RT_S(attesa_s);
		ev.fine_apprendimento = ev.inizio_apprendimento + RT_S(impara_s);
		fprintf(ev.out, "spia: tra %d s imparo per %d s quali byte dei sensori cambiano da soli (non toccare il robot)\n",
		        attesa_s, impara_s);
		for (int i = 0; i < n; i++)
			for (int d = 0; d < 2; d++)
				decod_init(&dec_ev[i][d], cc[i].info.protocollo);
	}

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
				if (ev.out) {
					ev.dir = dir;
					ev.adesso = adesso;
					decod_scrivi(&dec_ev[i][dir], buf, (size_t)k, ev_gd32, ev_lidar, NULL);
				}
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
	if (ev.out)
		fclose(ev.out);
	if (fclose(reg)) {
		fprintf(stderr, "spia: chiudo %s: %s\n", uscita, strerror(errno));
		esito = 1;
	}
	return esito;
}


/* ------------------------------------------------------------------------------------------------------ */
/* socket: si mette in mezzo su socket Unix (stream e datagrammi)                                         */
/*
 * Stream (es. /tmp/AuxEverest_Sock, server = AuxCtrl, client = RobotApp):
 *   1. rinomina il socket del server in <percorso>.vero: il server continua ad ascoltare lì;
 *   2. si mette in ascolto su <percorso>; ogni client che arriva viene collegato a <percorso>.vero;
 *   3. inoltra e registra nelle due direzioni.
 *   I client già collegati prima NON passano dalla spia: chi la usa deve far ricollegare il client
 *   (tools/spia-robot.sh termina RobotApp, che Monitor rilancia).
 * Datagrammi: rinomina il socket di chi riceve in <percorso>.vero, riceve al suo posto e inoltra.
 * All'uscita rimette tutto com'era (cancella i propri socket e rinomina .vero al nome originale).
 */
#define SOCK_MAX_COPPIE 4

typedef struct {
	reg_canale_t info;
	int datagrammi;
	char vero[PATH_MAX];
	int fd_ascolto;                    /* stream: in ascolto; datagrammi: il socket che riceve */
	int fd_client[SOCK_MAX_COPPIE];    /* stream: lato client (RobotApp) */
	int fd_server[SOCK_MAX_COPPIE];    /* stream: lato server (AuxCtrl, su .vero) */
	int rinominato;
	uint64_t byte[2], connessioni;
	decod_t decod[2];
} sock_t;

static int indirizzo(const char *percorso, struct sockaddr_un *a)
{
	memset(a, 0, sizeof(*a));
	a->sun_family = AF_UNIX;
	if (strlen(percorso) >= sizeof a->sun_path)
		return -1;
	strcpy(a->sun_path, percorso);
	return 0;
}

static int apri_sock(sock_t *c)
{
	const char *p = c->info.dispositivo;
	struct sockaddr_un a;
	struct stat st;
	for (int i = 0; i < SOCK_MAX_COPPIE; i++)
		c->fd_client[i] = c->fd_server[i] = -1;
	c->fd_ascolto = -1;
	if (stat(p, &st) || !S_ISSOCK(st.st_mode)) {
		fprintf(stderr, "spia: %s non esiste o non è un socket\n", p);
		return -1;
	}
	snprintf(c->vero, sizeof c->vero, "%s.vero", p);
	if (rename(p, c->vero)) {
		fprintf(stderr, "spia: rinomino %s: %s\n", p, strerror(errno));
		return -1;
	}
	c->rinominato = 1;
	c->fd_ascolto = socket(AF_UNIX, (c->datagrammi ? SOCK_DGRAM : SOCK_STREAM) | SOCK_NONBLOCK, 0);
	if (c->fd_ascolto < 0 || indirizzo(p, &a) || bind(c->fd_ascolto, (struct sockaddr *)&a, sizeof a) ||
	    (!c->datagrammi && listen(c->fd_ascolto, 8))) {
		fprintf(stderr, "spia: socket su %s: %s\n", p, strerror(errno));
		return -1;
	}
	chmod(p, st.st_mode & 07777);
	decod_init(&c->decod[0], c->info.protocollo);
	decod_init(&c->decod[1], c->info.protocollo);
	return 0;
}

static void chiudi_coppia(sock_t *c, int i)
{
	if (c->fd_client[i] >= 0) close(c->fd_client[i]);
	if (c->fd_server[i] >= 0) close(c->fd_server[i]);
	c->fd_client[i] = c->fd_server[i] = -1;
}

static void chiudi_sock(sock_t *c)
{
	for (int i = 0; i < SOCK_MAX_COPPIE; i++)
		chiudi_coppia(c, i);
	if (c->fd_ascolto >= 0)
		close(c->fd_ascolto);
	c->fd_ascolto = -1;
	if (c->rinominato) {
		unlink(c->info.dispositivo);
		if (rename(c->vero, c->info.dispositivo))
			fprintf(stderr, "spia: ATTENZIONE, non riesco a rimettere %s: %s\n", c->info.dispositivo, strerror(errno));
	}
	c->rinominato = 0;
}

static void accetta(sock_t *c, int64_t adesso, FILE *reg)
{
	int fc = accept4(c->fd_ascolto, NULL, NULL, SOCK_NONBLOCK);
	if (fc < 0)
		return;
	struct sockaddr_un a;
	int fs = socket(AF_UNIX, SOCK_STREAM, 0);
	if (fs < 0 || indirizzo(c->vero, &a) || connect(fs, (struct sockaddr *)&a, sizeof a)) {
		fprintf(stderr, "spia: %s: collegamento al server %s fallito: %s\n", c->info.nome, c->vero, strerror(errno));
		close(fc);
		if (fs >= 0) close(fs);
		return;
	}
	fcntl(fs, F_SETFL, fcntl(fs, F_GETFL) | O_NONBLOCK);
	int i = 0;
	while (i < SOCK_MAX_COPPIE && c->fd_client[i] >= 0)
		i++;
	if (i == SOCK_MAX_COPPIE) {
		fprintf(stderr, "spia: %s: troppi client\n", c->info.nome);
		close(fc);
		close(fs);
		return;
	}
	c->fd_client[i] = fc;
	c->fd_server[i] = fs;
	c->connessioni++;
	/* un record vuoto segna l'inizio di una nuova connessione (i deframer ripartono da capo) */
	reg_scrivi_record(reg, adesso, c->info.id, REG_DAL_SOC, NULL, 0);
	decod_init(&c->decod[0], c->info.protocollo);
	decod_init(&c->decod[1], c->info.protocollo);
	fprintf(stderr, "spia: %s: nuovo client collegato (coppia %d)\n", c->info.nome, i);
}

static int cmd_socket(int argc, char **argv)
{
	const char *uscita = NULL;
	int ogni_s = 10, n = 0;
	static sock_t cc[REG_MAX_CANALI];
	static const struct option opz[] = {
		{ "uscita", required_argument, 0, 'o' }, { "stream", required_argument, 0, 'S' },
		{ "dgram", required_argument, 0, 'D' }, { "stato", required_argument, 0, 's' }, { 0, 0, 0, 0 },
	};
	int o;
	while ((o = getopt_long(argc, argv, "o:s:", opz, NULL)) != -1) {
		switch (o) {
		case 'o': uscita = optarg; break;
		case 's': ogni_s = atoi(optarg); break;
		case 'S':
		case 'D':
			if (n == REG_MAX_CANALI || analizza_canale(optarg, &cc[n].info, (uint8_t)n)) {
				fprintf(stderr, "spia: canale non valido: %s (formato nome,everest|grezzo,percorso)\n", optarg);
				return 2;
			}
			cc[n].datagrammi = o == 'D';
			n++;
			break;
		default: return 2;
		}
	}
	if (!uscita || n == 0) {
		fprintf(stderr, "uso: spia socket -o <registro> --stream nome,proto,percorso [--dgram nome,proto,percorso]...\n");
		return 2;
	}
	FILE *reg = fopen(uscita, "wb");
	if (!reg) {
		fprintf(stderr, "spia: apro %s: %s\n", uscita, strerror(errno));
		return 1;
	}
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
		if (apri_sock(&cc[i])) {
			esito = 1;
			goto fine;
		}
		printf("canale %s (%s): %s <-> %s\n", cc[i].info.nome, cc[i].datagrammi ? "datagrammi" : "stream",
		       cc[i].info.dispositivo, cc[i].vero);
	}
	fflush(stdout);

	static uint8_t buf[REG_MAX_BLOCCO];
	int64_t t0 = rt_adesso(), prossimo_stato = t0 + RT_S(ogni_s), prossimo_flush = t0 + RT_S(1);
	while (!fermati) {
		struct pollfd pf[REG_MAX_CANALI * (1 + 2 * SOCK_MAX_COPPIE)];
		int quale[REG_MAX_CANALI * (1 + 2 * SOCK_MAX_COPPIE)][2];   /* canale, ruolo: -1 ascolto, 2i client, 2i+1 server */
		int np = 0;
		for (int i = 0; i < n; i++) {
			pf[np] = (struct pollfd){ .fd = cc[i].fd_ascolto, .events = POLLIN };
			quale[np][0] = i; quale[np++][1] = -1;
			for (int k = 0; k < SOCK_MAX_COPPIE; k++) {
				if (cc[i].fd_client[k] < 0)
					continue;
				pf[np] = (struct pollfd){ .fd = cc[i].fd_client[k], .events = POLLIN };
				quale[np][0] = i; quale[np++][1] = 2 * k;
				pf[np] = (struct pollfd){ .fd = cc[i].fd_server[k], .events = POLLIN };
				quale[np][0] = i; quale[np++][1] = 2 * k + 1;
			}
		}
		int r = poll(pf, (nfds_t)np, 200);
		if (r < 0 && errno != EINTR) {
			perror("spia: poll");
			esito = 1;
			break;
		}
		int64_t adesso = rt_adesso();
		for (int j = 0; j < np && r > 0; j++) {
			if (!(pf[j].revents & (POLLIN | POLLHUP | POLLERR)))
				continue;
			sock_t *c = &cc[quale[j][0]];
			int ruolo = quale[j][1];
			if (ruolo < 0 && c->datagrammi) {
				ssize_t k = recv(c->fd_ascolto, buf, sizeof buf, 0);
				if (k < 0)
					continue;
				struct sockaddr_un a;
				indirizzo(c->vero, &a);
				sendto(c->fd_ascolto, buf, (size_t)k, MSG_DONTWAIT, (struct sockaddr *)&a, sizeof a);
				reg_scrivi_record(reg, adesso, c->info.id, REG_VERSO_SOC, buf, (uint16_t)k);
				c->byte[0] += (uint64_t)k;
				decod_scrivi(&c->decod[0], buf, (size_t)k, NULL, NULL, NULL);
				continue;
			}
			if (ruolo < 0) {
				accetta(c, adesso, reg);
				continue;
			}
			int coppia = ruolo / 2, dal_client = ruolo % 2 == 0;
			int da = dal_client ? c->fd_client[coppia] : c->fd_server[coppia];
			int a_fd = dal_client ? c->fd_server[coppia] : c->fd_client[coppia];
			if (da < 0)
				continue;
			ssize_t k = read(da, buf, sizeof buf);
			if (k == 0 || (k < 0 && errno != EAGAIN && errno != EINTR)) {
				fprintf(stderr, "spia: %s: il %s ha chiuso la connessione (coppia %d)\n", c->info.nome,
				        dal_client ? "client" : "server", coppia);
				chiudi_coppia(c, coppia);
				continue;
			}
			if (k < 0)
				continue;
			int dir = dal_client ? REG_DAL_SOC : REG_VERSO_SOC;
			reg_scrivi_record(reg, adesso, c->info.id, (uint8_t)dir, buf, (uint16_t)k);
			c->byte[dir] += (uint64_t)k;
			decod_scrivi(&c->decod[dir], buf, (size_t)k, NULL, NULL, NULL);
			if (scrivi_tutto(a_fd, buf, (size_t)k, 1000))
				fprintf(stderr, "spia: %s: byte persi inoltrando (coppia %d)\n", c->info.nome, coppia);
		}
		if (adesso >= prossimo_flush) {
			fflush(reg);
			prossimo_flush = adesso + RT_S(1);
		}
		if (ogni_s > 0 && adesso >= prossimo_stato) {
			fprintf(stderr, "[%7.1f s]", (double)(adesso - t0) / RT_NS_PER_S);
			for (int i = 0; i < n; i++) {
				uint64_t ok0, e0, s0, ok1, e1, s1;
				decod_contatori(&cc[i].decod[0], &ok0, &e0, &s0);
				decod_contatori(&cc[i].decod[1], &ok1, &e1, &s1);
				fprintf(stderr, "  %s: %llu conn, server->client %llu B %llu msg, client->server %llu B %llu msg",
				        cc[i].info.nome, (unsigned long long)cc[i].connessioni,
				        (unsigned long long)cc[i].byte[0], (unsigned long long)ok0,
				        (unsigned long long)cc[i].byte[1], (unsigned long long)ok1);
			}
			fputc('\n', stderr);
			prossimo_stato = adesso + RT_S(ogni_s);
		}
	}
fine:
	for (int i = 0; i < n; i++)
		chiudi_sock(&cc[i]);
	if (fclose(reg))
		esito = 1;
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

static void leggi_everest(const everest_msg_t *m, void *ctx)
{
	const ctx_leggi_t *x = ctx;
	stampa_testa(x);
	fprintf(x->out, "cmd %d len %d c12 %d t %llu: ", m->comando, m->lunghezza, m->campo12, (unsigned long long)m->tempo);
	stampa_hex(x->out, m->dati, m->n_dati < 256 ? m->n_dati : 256);
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

/* I comandi everest sono int32: si tengono in una tabella a parte. */
#define EV_MAX_CMD 1024
static struct { uint8_t can, dir; int32_t cmd; stat_cmd_t s; } stat_everest[EV_MAX_CMD];
static int n_stat_everest;

static void riep_everest(const everest_msg_t *m, void *ctx)
{
	const reg_record_t *r = ((ctx_riep_t *)ctx)->rec;
	int i = 0;
	while (i < n_stat_everest && !(stat_everest[i].can == r->canale && stat_everest[i].dir == r->direzione &&
	                                stat_everest[i].cmd == m->comando))
		i++;
	if (i == n_stat_everest) {
		if (n_stat_everest == EV_MAX_CMD)
			return;
		stat_everest[n_stat_everest++] = (typeof(stat_everest[0])){ r->canale, r->direzione, m->comando, { 0 } };
	}
	stat_cmd_t *s = &stat_everest[i].s;
	unsigned len = (unsigned)m->lunghezza;
	if (s->n == 0) {
		s->len_min = s->len_max = len;
		s->primo = r->tempo_ns;
		s->dt_min = INT64_MAX;
	} else {
		int64_t dt = r->tempo_ns - s->ultimo;
		if (dt < s->dt_min) s->dt_min = dt;
		if (dt > s->dt_max) s->dt_max = dt;
		if (len < s->len_min) s->len_min = len;
		if (len > s->len_max) s->len_max = len;
	}
	s->ultimo = r->tempo_ns;
	s->n++;
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
		if (r.n == 0 && c->protocollo == REG_PROTO_EVEREST) {   /* nuova connessione: i deframer ripartono */
			decod_init(&dec[r.canale][0], c->protocollo);
			decod_init(&dec[r.canale][1], c->protocollo);
			if (!riepilogo)
				printf("%.6f %s --- nuova connessione ---\n", (double)r.tempo_ns / RT_NS_PER_S, c->nome);
			continue;
		}
		decod_cb_everest = riepilogo ? riep_everest : leggi_everest;
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
				       strncmp(c->dispositivo, "/dev/", 5) == 0
				               ? (d == REG_VERSO_SOC ? "periferica -> SoC" : "SoC -> periferica")
				               : (d == REG_VERSO_SOC ? "server -> client" : "client -> server"),
				       (unsigned long long)byte[i][d], (unsigned long long)ok, (unsigned long long)err,
				       (unsigned long long)sc);
				int titolo = 0;
				if (c->protocollo == REG_PROTO_EVEREST) {
					for (int k = 0; k < n_stat_everest; k++) {
						if (stat_everest[k].can != i || stat_everest[k].dir != d)
							continue;
						stat_cmd_t *s = &stat_everest[k].s;
						if (!titolo++)
							printf("  cmd          messaggi   dati(byte)     periodo medio/min/max (ms)\n");
						printf("  %-10d %10llu   %5u-%-5u", stat_everest[k].cmd, (unsigned long long)s->n, s->len_min, s->len_max);
						if (s->n > 1)
							printf("  %9.1f %9.1f %9.1f", (double)(s->ultimo - s->primo) / (double)(s->n - 1) / 1e6,
							       (double)s->dt_min / 1e6, (double)s->dt_max / 1e6);
						putchar('\n');
					}
					continue;
				}
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
	        "                [--eventi <file> [--attesa s] [--impara s] [--istantanea <file>]] [--senza-mount] [--forza]\n"
	        "       (canali di default: gd32,gd32,/dev/ttyS3 e lidar,lidar,/dev/ttyS1)\n"
	        "     spia socket -o <registro> --stream nome,everest|grezzo,percorso [--dgram nome,proto,percorso]...\n"
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
	if (!strcmp(cmd, "socket"))    return cmd_socket(argc, argv);
	if (!strcmp(cmd, "leggi"))     return cmd_leggi(argc, argv, 0);
	if (!strcmp(cmd, "riepilogo")) return cmd_leggi(argc, argv, 1);
	if (!strcmp(cmd, "termios"))   return cmd_termios(argc, argv);
	uso();
	return 2;
}
