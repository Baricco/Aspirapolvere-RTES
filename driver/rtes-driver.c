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
 *     guasto N           fault asked by a task, N = drv_task_fault_t (drv_report_fault)
 *     azzera             prova a uscire dallo stato di guasto
 *     suono NOME         es. suono sound_beep_beep
 *     blocca             il task di prova (--task-prova) si blocca: il watchdog deve mandare il driver in guasto
 *     sfora              un job del task di prova dura 60 ms, oltre la sua scadenza di 20: sul robot deve bastare per il guasto
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
 *   --banco [--cliff-atteso X]   banco di prova: motori solo con il robot sollevato (gd32/interlock.h)
 *   --lidar-giri G           velocità del lidar (default 6)
 *   --scarto-lidar CDEG      scarto dello 0 del lidar in centesimi di grado (default 1450, driver/lidar_scan.h)
 *   --task-prova             avvia un task sporadico (svegliato dai sensori, CPU 1, priorità 90) e uno periodico
 *                            da 200 ms (CPU 2) che leggono i dati del driver; entrambi sorvegliati, critici
 *   --durata S               esce dopo S secondi
 *   --csv FILE               una riga al secondo con lo stato (per le prove lunghe)
 *   --registra FILE          registra i byte grezzi di GD32 e lidar nel formato della spia (per tools/taratura e
 *                            tools/valida-sim.py; si legge con tools/estrai-frame.py)
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

#include "../gd32/interlock.h"
#include "driver.h"

static volatile sig_atomic_t stop_requested;
static void on_signal(int s) { (void)s; stop_requested = 1; }

/* ---- task di prova ---- */
static rt_task_t t_sensors, t_lidar;
static atomic_int block, overrun;
static drv_sensors_t last_s;
static uint32_t seen_scans, scan_points;

static void job_sensors(void *arg)
{
	(void)arg;
	drv_sensors(&last_s);
	while (atomic_load(&block) && !atomic_load(&t_sensors.stop))
		usleep(10000);                   /* "si blocca": per esempio un mutex mai rilasciato */
	if (atomic_exchange(&overrun, 0))
		for (int64_t end = rt_now() + RT_MS(60); rt_now() < end;)
			;                        /* calcola per 60 ms: esecuzione e CPU devono coincidere */
}

static void job_lidar(void *arg)
{
	(void)arg;
	static lidar_scan_t g;
	if (drv_scan(&g)) {
		seen_scans = g.number;
		scan_points = g.n;
	}
}

/* ---- comandi ---- */
static int run_command(const char *line)
{
	double a, b;
	int i, j;
	char name[100];
	while (*line == ' ' || *line == '\t')
		line++;
	if (!*line || *line == '#')
		return 0;
	if (sscanf(line, "v %lf %lf", &a, &b) == 2)
		drv_velocity(a, b);
	else if (!strncmp(line, "stop", 4))
		drv_velocity(0, 0);
	else if (sscanf(line, "spazzole %d %d", &i, &j) == 2)
		drv_brushes(i, j);
	else if (sscanf(line, "aspirazione %d", &i) == 1)
		drv_suction(i);
	else if (sscanf(line, "led %d", &i) == 1)
		drv_led(i);
	else if (sscanf(line, "lidar %d %lf", &i, &a) == 2)
		drv_lidar(i, a);
	else if (sscanf(line, "lidar %d", &i) == 1)
		drv_lidar(i, 0);
	else if (sscanf(line, "aggancia %d", &i) == 1)
		drv_dock(i);
	else if (sscanf(line, "esci_base %d", &i) == 1)
		drv_leave_base(i);
	else if (sscanf(line, "guasto %d", &i) == 1)
		drv_report_fault((drv_task_fault_t)i);
	else if (!strncmp(line, "azzera", 6))
		printf("azzera guasto: %s\n", drv_clear_fault() ? "NO, la causa c'è ancora" : "ok");
	else if (sscanf(line, "suono %99s", name) == 1) {
		static char s[100];
		snprintf(s, sizeof s, "%s", name);
		drv_sound(s);
	} else if (sscanf(line, "salva_giro %99s", name) == 1) {
		static lidar_scan_t g;
		FILE *f = drv_scan(&g) ? fopen(name, "w") : NULL;
		if (!f) {
			printf("nessun giro del lidar, o non riesco a scrivere %s\n", name);
			return -1;
		}
		fprintf(f, "angolo_gradi,x_m,y_m,distanza_mm,intensita\n");
		for (int k = 0; k < g.n; k++) {
			double x, y;
			drv_point_xy(&g.p[k], &x, &y);
			fprintf(f, "%.2f,%.4f,%.4f,%u,%u\n", g.p[k].angle_cdeg / 100.0, x, y, g.p[k].distance_mm, g.p[k].intensity);
		}
		fclose(f);
		printf("giro %u (%u punti, %.2f giri/s) salvato in %s\n", g.number, g.n, g.rev_s, name);
	} else if (!strncmp(line, "blocca", 6))
		atomic_store(&block, 1);
	else if (!strncmp(line, "sfora", 5))
		atomic_store(&overrun, 1);
	else if (!strncmp(line, "fine", 4))
		stop_requested = 1;
	else {
		printf("comando non valido: %s\n", line);
		return -1;
	}
	return 0;
}

/* Legge righe da fd e le esegue. Ritorna -1 a fine file. */
static int read_command_lines(int fd, char *buf, size_t *n, size_t size)
{
	ssize_t k = read(fd, buf + *n, size - 1 - *n);
	if (k == 0)
		return -1;
	if (k < 0)
		return errno == EAGAIN ? 0 : -1;
	*n += (size_t)k;
	char *nl;
	while ((nl = memchr(buf, '\n', *n))) {
		*nl = 0;
		run_command(buf);
		fflush(stdout);
		size_t used_bytes = (size_t)(nl - buf) + 1;
		memmove(buf, nl + 1, *n - used_bytes);
		*n -= used_bytes;
	}
	if (*n == size - 1)
		*n = 0;
	return 0;
}

static const char *phase_name(int f)
{
	static const char *n[] = { "AVVIO", "pronto", "GUASTO", "fine" };
	return f >= 0 && f <= 3 ? n[f] : "?";
}

static void limits_text(int l, char *out, size_t size)
{
	static const struct { int bit; const char *name; } names[] = {
		{ INTERLOCK_LIM_LIFTED, "sollevato" }, { INTERLOCK_LIM_BASE, "base" }, { INTERLOCK_LIM_SENSORS, "sensori-spenti" },
		{ INTERLOCK_LIM_BUMPER, "bumper" }, { INTERLOCK_LIM_CLIFF, "vuoto" }, { DRV_LIM_DISABLED, "motori-vietati" },
		{ DRV_LIM_NOT_READY, "avvio" }, { DRV_LIM_FAULT, "guasto" }, { DRV_LIM_OLD_SENSORS, "sensori-vecchi" },
		{ DRV_LIM_BATTERY, "batteria" }, { DRV_LIM_BENCH, "banco" }, { DRV_LIM_NO_DUSTBIN, "senza-cassetto" },
		{ DRV_LIM_FORBIDDEN, "vietata" },
	};
	out[0] = 0;
	for (size_t i = 0; i < sizeof names / sizeof names[0]; i++)
		if (l & names[i].bit) {
			size_t k = strlen(out);
			snprintf(out + k, size - k, "%s%s", k ? "," : "", names[i].name);
		}
	if (!out[0])
		snprintf(out, size, "-");
}

static void status_line(double t, const drv_status_t *st, const drv_sensors_t *s, int task)
{
	char lim[160], suction_now[180] = "";
	limits_text(st->limits, lim, sizeof lim);
	if (st->suction_limits) {
		snprintf(suction_now, sizeof suction_now, " (ferma: ");
		limits_text(st->suction_limits, suction_now + strlen(suction_now), sizeof suction_now - strlen(suction_now) - 1);
		strcat(suction_now, ")");
	}
	printf("%7.1f %-6s %5.2f V %3d%%%s%s | bump %d%d vuoto %x | sens %2.0f Hz | v %+.2f w %+.2f lim %s"
	       " (misurate v %+.3f w %+.3f, imbardata %+.1f°) | cassetto %d ir %x/%x ventola %d%s | spazz %d/%d led %d |"
	       " lidar %s %.2f g/s p%d giri %u | tick+%.0f lav %.0f us | tasti %u/%u",
	       t, phase_name(st->phase), st->volts, st->percentage,
	       st->charge_complete ? " carica completa" : st->charging ? " carica" : "",
	       st->on_base ? " BASE" : "", s->s.bumper_left, s->s.bumper_right,
	       s->s.cliff[0] | s->s.cliff[1] << 1 | s->s.cliff[2] << 2 | s->s.cliff[3] << 3, st->sensors_hz, st->v, st->w,
	       lim, s->s.v_measured / GD32_SCALE_V, s->s.w_measured / GD32_SCALE_W, s->s.angle[0] / 100.0,
	       s->s.dustbin, s->s.ir_dock_right, s->s.ir_dock_left, s->s.fan_rpm, suction_now, st->side_brush, st->main_brush, st->led, st->lidar_on ? (st->lidar_ok ? "ok" : "MUTO") : "off",
	       st->lidar_rev_s, st->lidar_power, st->lidar_revolutions, st->tick_delay_max_ns / 1e3, st->work_max_ns / 1e3,
	       st->power_presses, st->home_presses);
	if (task)
		printf(" | task: sens %llu job lat %.0f esec %.0f us, persi %llu; lidar %llu job, giro %u da %u punti",
		       (unsigned long long)t_sensors.stats.job, t_sensors.stats.latency_max_ns / 1e3,
		       t_sensors.stats.exec_max_ns / 1e3, (unsigned long long)atomic_load(&t_sensors.lost_activations),
		       (unsigned long long)t_lidar.stats.job, seen_scans, scan_points);
	if (st->phase == DRV_FAULT)
		printf(" | GUASTO: %s", st->fault);
	putchar('\n');
	fflush(stdout);
}

int main(int argc, char **argv)
{
	drv_config_t c;
	drv_config_test(&c);
	int duration = 0, task = 0, quiet = 0, every_ms = 1000;
	const char *fifo = NULL, *csv_file = NULL;
	static const struct option options[] = {
		{ "gd32", required_argument, 0, 'g' }, { "lidar", required_argument, 0, 'l' },
		{ "senza-lidar", no_argument, 0, 'L' }, { "cpu", required_argument, 0, 'c' },
		{ "senza-motori", no_argument, 0, 'M' }, { "aspirazione", no_argument, 0, 'a' },
		{ "banco", no_argument, 0, 'b' }, { "cliff-atteso", required_argument, 0, 'A' },
		{ "lidar-giri", required_argument, 0, 'G' }, { "scarto-lidar", required_argument, 0, 's' },
		{ "task-prova", no_argument, 0, 'T' }, { "durata", required_argument, 0, 't' },
		{ "comandi", required_argument, 0, 'C' }, { "csv", required_argument, 0, 'v' },
		{ "silenzioso", no_argument, 0, 'q' }, { "senza-rt", no_argument, 0, 'R' },
		{ "forza", no_argument, 0, 'F' }, { "ogni", required_argument, 0, 'o' },
		{ "registra", required_argument, 0, 'r' }, { 0, 0, 0, 0 },
	};
	int o;
	while ((o = getopt_long(argc, argv, "", options, NULL)) != -1) {
		switch (o) {
		case 'g': c.dev_gd32 = optarg; break;
		case 'l': c.dev_lidar = optarg; break;
		case 'L': c.dev_lidar = NULL; break;
		case 'c': c.cpu = atoi(optarg); break;
		case 'M': c.motors = 0; break;
		case 'a': c.suction = 1; break;
		case 'b': c.bench = 1; break;
		case 'A': c.expected_cliff = (int)strtol(optarg, NULL, 0) & 0x0f; break;
		case 'G': c.lidar_rev_s = atof(optarg); break;
		case 's': c.lidar_offset_cdeg = atoi(optarg); break;
		case 'T': task = 1; break;
		case 't': duration = atoi(optarg); break;
		case 'C': fifo = optarg; break;
		case 'v': csv_file = optarg; break;
		case 'q': quiet = 1; break;
		case 'R': c.without_rt = 1; break;
		case 'F': c.force = 1; break;
		case 'r': c.record = optarg; break;
		case 'o': every_ms = atoi(optarg) > 0 ? atoi(optarg) : 1000; break;
		default:
			fprintf(stderr, "opzioni: vedi l'inizio di driver/rtes-driver.c\n");
			return 1;
		}
	}
	rt_allow_without_privileges(c.without_rt);
	struct sigaction sa = { .sa_handler = on_signal };
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
	FILE *csv = csv_file ? fopen(csv_file, "w") : NULL;
	if (csv)
		fprintf(csv, "t_s,fase,volt,percentuale,in_carica,sulla_base,sensori_hz,eta_sensori_ms,tick_ritardo_max_us,"
		             "lavoro_max_us,intervallo_sensori_max_ms,intervallo_sensori_min_ms,lidar_giri_s,lidar_potenza,lidar_giri,frame_errati,errori_scrittura,keepalive,"
		             "v,w,limiti,task_lat_max_us,task_esec_max_us,task_persi,task_mancate,carica_completa\n");

	if (drv_start(&c))
		return 1;
	if (task) {
		int64_t t0 = rt_now() + RT_MS(200);
		rt_task_cfg_t a = { .name = "prova-sensori", .cpu = 1, .sporadic = 1, .period_ns = RT_MS(20),
		                    .job = job_sensors };
		rt_task_cfg_t b = { .name = "prova-lidar", .cpu = 2, .period_ns = RT_MS(200), .job = job_lidar };
		rt_task_cfg_t *v[] = { &a, &b };
		rt_priority_rm(v, 2, RT_PRIO_TASK_MAX);
		if (rt_task_start(&t_sensors, &a, t0) || rt_task_start(&t_lidar, &b, t0)) {
			fprintf(stderr, "rtes-driver: non riesco ad avviare i task di prova\n");
			drv_stop();
			return 1;
		}
		drv_wake_on_sensors(&t_sensors);
		drv_watch(&t_sensors, 1, RT_MS(100));
		drv_watch(&t_lidar, 1, 0);
		printf("task di prova: %s prio %d (CPU 1, sveglia ai sensori), %s prio %d (CPU 2, 200 ms)\n", a.name,
		       a.priority, b.name, b.priority);
	}

	int64_t t0 = rt_now(), next_print = t0 + RT_MS(every_ms);
	int stdin_ok = 1, fault_seen = 0;
	static char b_in[1024], b_fifo[1024];
	size_t n_in = 0, n_fifo = 0;
	fcntl(0, F_SETFL, fcntl(0, F_GETFL) | O_NONBLOCK);
	while (!stop_requested) {
		int64_t t_now = rt_now();
		if (duration > 0 && t_now - t0 >= RT_S(duration))
			break;
		struct pollfd p[2] = { { .fd = stdin_ok ? 0 : -1, .events = POLLIN }, { .fd = fd_fifo, .events = POLLIN } };
		int pending = (int)((next_print - t_now) / RT_NS_PER_MS);
		poll(p, 2, pending > 0 ? pending : 0);
		if (p[0].revents & (POLLIN | POLLHUP))
			if (read_command_lines(0, b_in, &n_in, sizeof b_in) < 0)
				stdin_ok = 0;
		if (p[1].revents & POLLIN)
			read_command_lines(fd_fifo, b_fifo, &n_fifo, sizeof b_fifo);
		t_now = rt_now();
		if (t_now >= next_print) {
			drv_status_t st;
			drv_sensors_t s = { 0 };
			drv_status(&st);
			drv_sensors(&s);
			if (st.phase == DRV_FAULT)
				fault_seen = 1;
			double t = (double)(t_now - t0) / RT_NS_PER_S;
			if (!quiet)
				status_line(t, &st, &s, task);
			if (csv) {
				fprintf(csv, "%.1f,%d,%.2f,%d,%d,%d,%.0f,%.1f,%.1f,%.1f,%.3f,%.3f,%.2f,%d,%u,%llu,%llu,%llu,%.3f,%.3f,%d,%.1f,%.1f,%llu,%llu,%d\n",
				        t, st.phase, st.volts, st.percentage, st.charging, st.on_base, st.sensors_hz,
				        st.sensors_age_ns / 1e6, st.tick_delay_max_ns / 1e3, st.work_max_ns / 1e3,
				        st.sensors_interval_max_ns / 1e6, st.sensors_interval_min_ns / 1e6, st.lidar_rev_s,
				        st.lidar_power, st.lidar_revolutions, (unsigned long long)st.bad_frames,
				        (unsigned long long)st.write_errors, (unsigned long long)st.keepalives, st.v, st.w, st.limits,
				        task ? t_sensors.stats.latency_max_ns / 1e3 : 0, task ? t_sensors.stats.exec_max_ns / 1e3 : 0,
				        task ? (unsigned long long)atomic_load(&t_sensors.lost_activations) : 0ULL,
				        task ? (unsigned long long)atomic_load(&t_sensors.total_misses) : 0ULL, st.charge_complete);
				fflush(csv);
			}
			next_print += RT_MS(every_ms);
		}
	}
	if (task) {
		atomic_store(&block, 0);
		drv_forget(&t_sensors);
		drv_forget(&t_lidar);
		rt_task_stop(&t_sensors);
		rt_task_stop(&t_lidar);
	}
	drv_status_t st;
	drv_stop();
	drv_status(&st);
	if (csv)
		fclose(csv);
	printf("fine: %llu frame dei sensori, %llu errati, %llu keepalive, %llu heartbeat, ritardo max del ciclo %.1f us, "
	       "lavoro max %.1f us, intervallo fra due frame da %.2f a %.2f ms, giri del lidar %u%s\n",
	       (unsigned long long)st.sensor_frames, (unsigned long long)st.bad_frames, (unsigned long long)st.keepalives,
	       (unsigned long long)st.heartbeats, st.tick_delay_max_ns / 1e3, st.work_max_ns / 1e3,
	       st.sensors_interval_min_ns / 1e6, st.sensors_interval_max_ns / 1e6, st.lidar_revolutions,
	       fault_seen ? ", GUASTO durante la prova" : "");
	if (task) {
		rt_stats_print(&t_sensors);
		rt_stats_print(&t_lidar);
	}
	return fault_seen ? 4 : 0;
}
