/*
 * rtes-driver - Il driver da solo, comandato a mano: per provarlo nel simulatore e sul robot.
 *
 * Avvia il driver (driver.h), stampa una riga di stato al secondo e legge comandi, una riga per comando, dal
 * terminale (stdin) e, con --comandi FIFO, da una FIFO (es. dal robot: echo "lidar 1" > /tmp/rtes/comandi):
 *     v V W              velocità: V m/s (avanti +), W rad/s (antiorario +)
 *     stop               velocità zero
 *     spazzole L C       spazzola laterale e centrale (-100..100)
 *     aspirazione G      giri/min (serve --aspirazione)
 *     led N              modo del LED (0x8d)
 *     lidar 0|1 [G]      spegne / accende il lidar (G giri/s, default 6)
 *     esci_base 0|1      permette di muoversi piano anche sulla base
 *     azzera             prova a uscire dallo stato di guasto
 *     suono NOME         es. suono sound_beep_beep
 *     blocca             il task di prova (--task-prova) si blocca: il watchdog deve mandare il driver in guasto
 *     salva_giro FILE    scrive l'ultimo giro del lidar in FILE (CSV: angolo del lidar in gradi, x e y nel sistema
 *                        del robot in metri, distanza in mm, intensità)
 *     fine               esce
 *
 * Opzioni:
 *   --gd32 DEV --lidar DEV   seriali (default /dev/ttyS3 e /dev/ttyS1; nel simulatore build/sim/gd32 e build/sim/lidar)
 *   --senza-lidar            non apre la seriale del lidar
 *   --cpu N                  core del driver (default 0)
 *   --senza-motori           rifiuta qualunque comando ai motori (ruote, spazzole, aspirazione)
 *   --aspirazione            permette l'aspirazione (di default è vietata)
 *   --banco [--cliff-atteso X]   banco di prova: motori solo con il robot sollevato (gd32/sicurezza.h)
 *   --lidar-giri G           velocità del lidar (default 6)
 *   --scarto-lidar CDEG      scarto dello 0 del lidar in centesimi di grado (default 800)
 *   --task-prova             avvia un task sporadico (svegliato dai sensori, CPU 1, priorità 90) e uno periodico
 *                            da 200 ms (CPU 2) che leggono i dati del driver; entrambi sorvegliati, critici
 *   --durata S               esce dopo S secondi
 *   --csv FILE               una riga al secondo con lo stato (per le prove lunghe)
 *   --silenzioso             niente riga di stato al secondo
 *   --ogni MS                una riga di stato ogni MS millisecondi (default 1000)
 *   --senza-rt               se mancano i permessi per SCHED_FIFO parte lo stesso (PC)
 *   --forza                  non controlla se le seriali sono già usate da un altro programma
 * Codice d'uscita: 0 normale, 1 errore, 4 il driver è andato in guasto durante la prova.
 */
#define _GNU_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <poll.h>
#include <signal.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#include "../gd32/sicurezza.h"
#include "driver.h"

static volatile sig_atomic_t fermati;
static void al_segnale(int s) { (void)s; fermati = 1; }

/* ---- task di prova ---- */
static rt_task_t t_sensori, t_lidar;
static atomic_int blocca;
static drv_sensori_t ultimo_s;
static uint32_t visti_lidar, punti_lidar;

static void job_sensori(void *arg)
{
	(void)arg;
	drv_sensori(&ultimo_s);
	while (atomic_load(&blocca) && !atomic_load(&t_sensori.stop))
		usleep(10000);                   /* "si blocca": per esempio un mutex mai rilasciato */
}

static void job_lidar(void *arg)
{
	(void)arg;
	static lidar_giro_t g;
	if (drv_giro(&g)) {
		visti_lidar = g.numero;
		punti_lidar = g.n;
	}
}

/* ---- comandi ---- */
static int esegui(const char *riga)
{
	double a, b;
	int i, j;
	char nome[100];
	while (*riga == ' ' || *riga == '\t')
		riga++;
	if (!*riga || *riga == '#')
		return 0;
	if (sscanf(riga, "v %lf %lf", &a, &b) == 2)
		drv_velocita(a, b);
	else if (!strncmp(riga, "stop", 4))
		drv_velocita(0, 0);
	else if (sscanf(riga, "spazzole %d %d", &i, &j) == 2)
		drv_spazzole(i, j);
	else if (sscanf(riga, "aspirazione %d", &i) == 1)
		drv_aspirazione(i);
	else if (sscanf(riga, "led %d", &i) == 1)
		drv_led(i);
	else if (sscanf(riga, "lidar %d %lf", &i, &a) == 2)
		drv_lidar(i, a);
	else if (sscanf(riga, "lidar %d", &i) == 1)
		drv_lidar(i, 0);
	else if (sscanf(riga, "esci_base %d", &i) == 1)
		drv_esci_dalla_base(i);
	else if (!strncmp(riga, "azzera", 6))
		printf("azzera guasto: %s\n", drv_azzera_guasto() ? "NO, la causa c'è ancora" : "ok");
	else if (sscanf(riga, "suono %99s", nome) == 1) {
		static char s[100];
		snprintf(s, sizeof s, "%s", nome);
		drv_suono(s);
	} else if (sscanf(riga, "salva_giro %99s", nome) == 1) {
		static lidar_giro_t g;
		FILE *f = drv_giro(&g) ? fopen(nome, "w") : NULL;
		if (!f) {
			printf("nessun giro del lidar, o non riesco a scrivere %s\n", nome);
			return -1;
		}
		fprintf(f, "angolo_gradi,x_m,y_m,distanza_mm,intensita\n");
		for (int k = 0; k < g.n; k++) {
			double x, y;
			drv_punto_xy(&g.p[k], &x, &y);
			fprintf(f, "%.2f,%.4f,%.4f,%u,%u\n", g.p[k].angolo_cdeg / 100.0, x, y, g.p[k].distanza_mm, g.p[k].intensita);
		}
		fclose(f);
		printf("giro %u (%u punti, %.2f giri/s) salvato in %s\n", g.numero, g.n, g.giri_s, nome);
	} else if (!strncmp(riga, "blocca", 6))
		atomic_store(&blocca, 1);
	else if (!strncmp(riga, "fine", 4))
		fermati = 1;
	else {
		printf("comando non valido: %s\n", riga);
		return -1;
	}
	return 0;
}

/* Legge righe da fd e le esegue. Ritorna -1 a fine file. */
static int leggi_comandi(int fd, char *buf, size_t *n, size_t dim)
{
	ssize_t k = read(fd, buf + *n, dim - 1 - *n);
	if (k == 0)
		return -1;
	if (k < 0)
		return errno == EAGAIN ? 0 : -1;
	*n += (size_t)k;
	char *nl;
	while ((nl = memchr(buf, '\n', *n))) {
		*nl = 0;
		esegui(buf);
		fflush(stdout);
		size_t usati = (size_t)(nl - buf) + 1;
		memmove(buf, nl + 1, *n - usati);
		*n -= usati;
	}
	if (*n == dim - 1)
		*n = 0;
	return 0;
}

static const char *nome_fase(int f)
{
	static const char *n[] = { "AVVIO", "pronto", "GUASTO", "fine" };
	return f >= 0 && f <= 3 ? n[f] : "?";
}

static void limiti_testo(int l, char *out, size_t dim)
{
	static const struct { int bit; const char *nome; } nomi[] = {
		{ SIC_LIM_SOLLEVATO, "sollevato" }, { SIC_LIM_BASE, "base" }, { SIC_LIM_SENSORI, "sensori-spenti" },
		{ SIC_LIM_BUMPER, "bumper" }, { SIC_LIM_CLIFF, "vuoto" }, { DRV_LIM_DISABILITATI, "motori-vietati" },
		{ DRV_LIM_NON_PRONTO, "avvio" }, { DRV_LIM_GUASTO, "guasto" }, { DRV_LIM_SENSORI_VECCHI, "sensori-vecchi" },
		{ DRV_LIM_BATTERIA, "batteria" }, { DRV_LIM_BANCO, "banco" },
	};
	out[0] = 0;
	for (size_t i = 0; i < sizeof nomi / sizeof nomi[0]; i++)
		if (l & nomi[i].bit) {
			size_t k = strlen(out);
			snprintf(out + k, dim - k, "%s%s", k ? "," : "", nomi[i].nome);
		}
	if (!out[0])
		snprintf(out, dim, "-");
}

static void riga_stato(double t, const drv_stato_t *st, const drv_sensori_t *s, int task)
{
	char lim[160];
	limiti_testo(st->limiti, lim, sizeof lim);
	printf("%7.1f %-6s %5.2f V %3d%%%s%s | bump %d%d vuoto %x | sens %2.0f Hz | v %+.2f w %+.2f lim %s | spazz %d/%d led %d |"
	       " lidar %s %.2f g/s p%d giri %u | tick+%.0f lav %.0f us",
	       t, nome_fase(st->fase), st->volt, st->percentuale, st->in_carica ? " carica" : "",
	       st->sulla_base ? " BASE" : "", s->s.bumper_sx, s->s.bumper_dx,
	       s->s.cliff[0] | s->s.cliff[1] << 1 | s->s.cliff[2] << 2 | s->s.cliff[3] << 3, st->sensori_hz, st->v, st->w,
	       lim, st->spazzola_laterale, st->spazzola_centrale, st->led, st->lidar_acceso ? (st->lidar_ok ? "ok" : "MUTO") : "off",
	       st->lidar_giri_s, st->lidar_potenza, st->lidar_giri, st->tick_ritardo_max_ns / 1e3, st->lavoro_max_ns / 1e3);
	if (task)
		printf(" | task: sens %llu job lat %.0f esec %.0f us, persi %llu; lidar %llu job, giro %u da %u punti",
		       (unsigned long long)t_sensori.stats.job, t_sensori.stats.latenza_max_ns / 1e3,
		       t_sensori.stats.esec_max_ns / 1e3, (unsigned long long)atomic_load(&t_sensori.attivazioni_perse),
		       (unsigned long long)t_lidar.stats.job, visti_lidar, punti_lidar);
	if (st->fase == DRV_GUASTO)
		printf(" | GUASTO: %s", st->guasto);
	putchar('\n');
	fflush(stdout);
}

int main(int argc, char **argv)
{
	drv_config_t c;
	drv_config_default(&c);
	int durata = 0, task = 0, silenzioso = 0, ogni_ms = 1000;
	const char *fifo = NULL, *file_csv = NULL;
	static const struct option opz[] = {
		{ "gd32", required_argument, 0, 'g' }, { "lidar", required_argument, 0, 'l' },
		{ "senza-lidar", no_argument, 0, 'L' }, { "cpu", required_argument, 0, 'c' },
		{ "senza-motori", no_argument, 0, 'M' }, { "aspirazione", no_argument, 0, 'a' },
		{ "banco", no_argument, 0, 'b' }, { "cliff-atteso", required_argument, 0, 'A' },
		{ "lidar-giri", required_argument, 0, 'G' }, { "scarto-lidar", required_argument, 0, 's' },
		{ "task-prova", no_argument, 0, 'T' }, { "durata", required_argument, 0, 't' },
		{ "comandi", required_argument, 0, 'C' }, { "csv", required_argument, 0, 'v' },
		{ "silenzioso", no_argument, 0, 'q' }, { "senza-rt", no_argument, 0, 'R' },
		{ "forza", no_argument, 0, 'F' }, { "ogni", required_argument, 0, 'o' }, { 0, 0, 0, 0 },
	};
	int o;
	while ((o = getopt_long(argc, argv, "", opz, NULL)) != -1) {
		switch (o) {
		case 'g': c.dev_gd32 = optarg; break;
		case 'l': c.dev_lidar = optarg; break;
		case 'L': c.dev_lidar = NULL; break;
		case 'c': c.cpu = atoi(optarg); break;
		case 'M': c.motori = 0; break;
		case 'a': c.aspirazione = 1; break;
		case 'b': c.banco = 1; break;
		case 'A': c.cliff_atteso = (int)strtol(optarg, NULL, 0) & 0x0f; break;
		case 'G': c.lidar_giri_s = atof(optarg); break;
		case 's': c.scarto_lidar_cdeg = atoi(optarg); break;
		case 'T': task = 1; break;
		case 't': durata = atoi(optarg); break;
		case 'C': fifo = optarg; break;
		case 'v': file_csv = optarg; break;
		case 'q': silenzioso = 1; break;
		case 'R': c.senza_rt = 1; break;
		case 'F': c.forza = 1; break;
		case 'o': ogni_ms = atoi(optarg) > 0 ? atoi(optarg) : 1000; break;
		default:
			fprintf(stderr, "opzioni: vedi l'inizio di driver/rtes-driver.c\n");
			return 1;
		}
	}
	rt_consenti_senza_privilegi(c.senza_rt);
	struct sigaction sa = { .sa_handler = al_segnale };
	sigaction(SIGINT, &sa, NULL);
	sigaction(SIGTERM, &sa, NULL);
	sigaction(SIGHUP, &sa, NULL);
	signal(SIGPIPE, SIG_IGN);

	int fd_fifo = -1;
	if (fifo) {
		struct stat st;
		if (stat(fifo, &st) != 0 && mkfifo(fifo, 0600) != 0) {
			fprintf(stderr, "rtes-driver: mkfifo %s: %s\n", fifo, strerror(errno));
			return 1;
		}
		fd_fifo = open(fifo, O_RDWR | O_NONBLOCK);   /* lettura e scrittura: niente fine file quando chi scrive chiude */
	}
	FILE *csv = file_csv ? fopen(file_csv, "w") : NULL;
	if (csv)
		fprintf(csv, "t_s,fase,volt,percentuale,in_carica,sulla_base,sensori_hz,eta_sensori_ms,tick_ritardo_max_us,"
		             "lavoro_max_us,intervallo_sensori_max_ms,intervallo_sensori_min_ms,lidar_giri_s,lidar_potenza,lidar_giri,frame_errati,errori_scrittura,keepalive,"
		             "v,w,limiti,task_lat_max_us,task_esec_max_us,task_persi,task_mancate\n");

	if (drv_avvia(&c))
		return 1;
	if (task) {
		int64_t t0 = rt_adesso() + RT_MS(200);
		rt_task_cfg_t a = { .nome = "prova-sensori", .cpu = 1, .sporadico = 1, .periodo_ns = RT_MS(20),
		                    .job = job_sensori };
		rt_task_cfg_t b = { .nome = "prova-lidar", .cpu = 2, .periodo_ns = RT_MS(200), .job = job_lidar };
		rt_task_cfg_t *v[] = { &a, &b };
		rt_priorita_rm(v, 2, RT_PRIO_TASK_MAX);
		if (rt_task_avvia(&t_sensori, &a, t0) || rt_task_avvia(&t_lidar, &b, t0)) {
			fprintf(stderr, "rtes-driver: non riesco ad avviare i task di prova\n");
			drv_ferma();
			return 1;
		}
		drv_sveglia_ai_sensori(&t_sensori);
		drv_sorveglia(&t_sensori, 1, RT_MS(100));
		drv_sorveglia(&t_lidar, 1, 0);
		printf("task di prova: %s prio %d (CPU 1, sveglia ai sensori), %s prio %d (CPU 2, 200 ms)\n", a.nome,
		       a.priorita, b.nome, b.priorita);
	}

	int64_t t0 = rt_adesso(), prossima = t0 + RT_MS(ogni_ms);
	int stdin_ok = 1, guasto_visto = 0;
	static char b_in[1024], b_fifo[1024];
	size_t n_in = 0, n_fifo = 0;
	fcntl(0, F_SETFL, fcntl(0, F_GETFL) | O_NONBLOCK);
	while (!fermati) {
		int64_t ora = rt_adesso();
		if (durata > 0 && ora - t0 >= RT_S(durata))
			break;
		struct pollfd p[2] = { { .fd = stdin_ok ? 0 : -1, .events = POLLIN }, { .fd = fd_fifo, .events = POLLIN } };
		int attesa = (int)((prossima - ora) / RT_NS_PER_MS);
		poll(p, 2, attesa > 0 ? attesa : 0);
		if (p[0].revents & (POLLIN | POLLHUP))
			if (leggi_comandi(0, b_in, &n_in, sizeof b_in) < 0)
				stdin_ok = 0;
		if (p[1].revents & POLLIN)
			leggi_comandi(fd_fifo, b_fifo, &n_fifo, sizeof b_fifo);
		ora = rt_adesso();
		if (ora >= prossima) {
			drv_stato_t st;
			drv_sensori_t s = { 0 };
			drv_stato(&st);
			drv_sensori(&s);
			if (st.fase == DRV_GUASTO)
				guasto_visto = 1;
			double t = (double)(ora - t0) / RT_NS_PER_S;
			if (!silenzioso)
				riga_stato(t, &st, &s, task);
			if (csv) {
				fprintf(csv, "%.1f,%d,%.2f,%d,%d,%d,%.0f,%.1f,%.1f,%.1f,%.3f,%.3f,%.2f,%d,%u,%llu,%llu,%llu,%.3f,%.3f,%d,%.1f,%.1f,%llu,%llu\n",
				        t, st.fase, st.volt, st.percentuale, st.in_carica, st.sulla_base, st.sensori_hz,
				        st.eta_sensori_ns / 1e6, st.tick_ritardo_max_ns / 1e3, st.lavoro_max_ns / 1e3,
				        st.intervallo_sensori_max_ns / 1e6, st.intervallo_sensori_min_ns / 1e6, st.lidar_giri_s,
				        st.lidar_potenza, st.lidar_giri, (unsigned long long)st.frame_errati,
				        (unsigned long long)st.errori_scrittura, (unsigned long long)st.keepalive, st.v, st.w, st.limiti,
				        task ? t_sensori.stats.latenza_max_ns / 1e3 : 0, task ? t_sensori.stats.esec_max_ns / 1e3 : 0,
				        task ? (unsigned long long)atomic_load(&t_sensori.attivazioni_perse) : 0ULL,
				        task ? (unsigned long long)atomic_load(&t_sensori.mancate_totali) : 0ULL);
				fflush(csv);
			}
			prossima += RT_MS(ogni_ms);
		}
	}
	if (task) {
		atomic_store(&blocca, 0);
		drv_dimentica(&t_sensori);
		drv_dimentica(&t_lidar);
		rt_task_ferma(&t_sensori);
		rt_task_ferma(&t_lidar);
	}
	drv_stato_t st;
	drv_ferma();
	drv_stato(&st);
	if (csv)
		fclose(csv);
	printf("fine: %llu frame dei sensori, %llu errati, %llu keepalive, %llu heartbeat, ritardo max del ciclo %.1f us, "
	       "lavoro max %.1f us, intervallo fra due frame da %.2f a %.2f ms, giri del lidar %u%s\n",
	       (unsigned long long)st.frame_sensori, (unsigned long long)st.frame_errati, (unsigned long long)st.keepalive,
	       (unsigned long long)st.heartbeat, st.tick_ritardo_max_ns / 1e3, st.lavoro_max_ns / 1e3,
	       st.intervallo_sensori_min_ns / 1e6, st.intervallo_sensori_max_ns / 1e6, st.lidar_giri,
	       guasto_visto ? ", GUASTO durante la prova" : "");
	if (task) {
		rt_stats_stampa(&t_sensori);
		rt_stats_stampa(&t_lidar);
	}
	return guasto_visto ? 4 : 0;
}
