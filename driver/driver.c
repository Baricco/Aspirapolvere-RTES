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

#include "../gd32/interlock.h"
#include "../spia/registro.h"
#include "exchange.h"

#define SCALE_V GD32_SCALE_V          /* 0x66: unità di v per m/s (gd32/gd32.h, misura del 5/10) */
#define SCALE_W GD32_SCALE_W          /* 0x66: unità di ω per rad/s */
#define TICK_NS           RT_MS(20)
#define HEARTBEAT_NS      RT_MS(200)
#define KEEPALIVE_NS      RT_S(3)
#define SENSORS_OLD_NS RT_MS(100)
#define SENSORS_SILENT_NS   RT_S(1)
#define MAX_SUBSCRIBERS      8
#define MAX_WATCHED   16
#define MAX_EVENTS        64
#define REST_CHARGING_NS     RT_S(5)   /* a carica attiva da tanto si va a riposo */
#define REST_CONTACT_NS   RT_S(20)  /* senza carica (batteria piena): contatto stabile da tanto */
#define WAKE_LIDAR_MAX_NS RT_S(6)
#define LED_NORMAL       DRV_LED_BLUE
#define LED_BATTERY      6
#define LED_CHARGING        4
#define LED_FAULT        8

extern char **environ;

/* ------------------------------------------------------------------------------------ stato condiviso --- */

/* Comandi dei task: campi atomici, letti dal driver a ogni risveglio. */
static struct {
	_Atomic int64_t vw;                    /* v (tick/s, 32 bit alti) e omega (tick/s, 32 bit bassi) */
	_Atomic int side_brush_cmd, main_brush_cmd, suction, led, lidar_on, lidar_crev_s, leaving_base;
	_Atomic int clear_fault;
	_Atomic int stuck;                  /* incastro in avanti, dalla sicurezza (drv_stuck) */
	_Atomic int dock;                  /* 1 = parti, -1 = annulla, 0 = niente: lo azzera il driver (drv_dock) */
	_Atomic(const char *) sound;
	_Atomic int task_fault;                /* drv_task_fault_t asked by a task (drv_report_fault), DRV_FAULT_NONE = none */
} cmd;

/* Dati pubblicati: doppio buffer senza mutex (exchange.h). */
static exchange_t sensors_exchange, scan_exchange, status_exchange, contacts_exchange;
static drv_contacts_t contacts_buffers[2];
/* Nell'altro verso: la posizione che il moto dà al driver (un solo scrittore, il task del moto). */
static exchange_t pose_exchange;
static drv_pose_t pose_buffers[2];
static _Atomic int pose_ready;               /* pose_exchange inizializzato: drv_pose può scrivere */
static drv_sensors_t sensors_buffers[2];
static lidar_scan_t scan_buffers[2];
static drv_status_t status_buffers[2];

/* Task iscritti alle sveglie e sorvegliati: gli slot si riempiono una volta e si svuotano con drv_forget. */
static _Atomic(rt_task_t *) subscribers[MAX_SUBSCRIBERS];
static struct {
	_Atomic(rt_task_t *) t;
	int critical;
	int64_t threshold_ns;
	int reported_once;                          /* solo il driver lo usa */
	uint64_t seen_misses;                 /* solo il driver: scadenze mancate già trattate (azzerando il guasto) */
} watched[MAX_WATCHED];
static _Atomic uint64_t driver_rounds;       /* risvegli completati: drv_forget aspetta che avanzi */

/* Eventi (testo) dal driver al thread di servizio, che li stampa: il driver non fa mai I/O lento. */
static char event_queue[MAX_EVENTS][160];
static _Atomic uint32_t event_head, event_tail;

/* ------------------------------------------------------------------------------------------ il driver --- */

static struct {
	drv_config_t cfg;
	int fd_gd32, fd_lidar, fd_wakeup;
	pthread_t thread, receiver, service;
	pthread_mutex_t mx;                   /* stato interno, fra il thread del tempo e quello di ricezione */
	_Atomic int stop;
	int started;
	int without_rt;
	int gd32_closed, lidar_closed;
	int64_t t0;

	gd32_deframer_t dg;
	lidar_deframer_t dl;
	lidar_assembler_t assembler;
	lidar_regulator_t regulator;
	lidar_scan_t scan;                    /* appoggio per il giro appena completato (grande: non sullo stack) */
	battery_t bat;
	interlock_t interlock;                      /* solo banco di prova */
	contacts_t contacts;                  /* scatti di anti-caduta e bumper (contacts.h) */
	docking_t dock_state;                       /* messa in carica in retromarcia (docking.h) */
	int prev_on_base, prev_charging;
	int button_max_ms[2];                  /* durata della pressione in corso (0 = tasto rilasciato): accensione, casa */
	int pending_presses[2];               /* pressioni brevi viste dalla ricezione, da contare al prossimo tick */
	uint32_t presses[2];                /* pressioni brevi contate (pubblicate in drv_status_t) */
	int64_t t_off_base;                 /* da quando il robot è fuori dalla base (0 = è sulla base) */
	int back_on_base;                     /* il robot è tornato sulla base dopo almeno 1 s fuori: da guardare al prossimo tick */
	int64_t t_base_change, t_charging_change;
	int rest_level;                           /* 0 sveglio, 1 motori spenti, 2 anche sensori e lidar spenti */
	int lifted;                        /* resta attivo finché non rivede il pavimento (update_lifting) */
	uint64_t void_40;                    /* ultimi 40 frame: 1 = tutti e 4 gli anti-caduta sul vuoto */
	int n_crossed, n_front, n_tilted, n_floor;   /* frame di fila con la condizione vera */
	int64_t t_rest, t_wake, t_sensors_on;
	int prev_dock_phase, prev_dock_result;    /* per scrivere un evento quando cambiano */

	int phase;
	char fault[96];
	int task_fault;                       /* drv_task_fault_t of the current fault, if a task asked for it */
	int has_sensors;
	gd32_sensors_t s;
	int64_t t_sensors;
	uint32_t n_sensors;
	uint64_t sensors_per_second[2];          /* conteggio per i frame al secondo */
	int64_t t_second;
	char version[33];

	/* sequenza di avvio */
	int startup_step;
	int64_t t_startup;

	/* ultimo stato mandato al GD32 */
	int32_t v_sent, w_sent;
	int limits;
	int side_sent, main_sent, suction_sent, led_sent, lidar_sent, power_sent, sensors_sent;
	int suction_limits;              /* perché l'aspirazione chiesta non va (drv_status_t.suction_limits) */
	int64_t t_refresh, t_power;
	int lidar_ok;
	int64_t t_lidar_on;
	int prev_battery;

	/* tempi */
	int64_t next_tick, next_hb, next_ka;
	int64_t tick_delay_max, work_max, interval_max, interval_min;
	uint64_t wakeups_done, keepalives, heartbeats, gd32_hb, others, write_errors;
} d;

static int64_t now(void) { return rt_now(); }

static void event(const char *fmt, ...)
{
	uint32_t t = atomic_load(&event_head);
	if (t - atomic_load(&event_tail) >= MAX_EVENTS)
		return;                                    /* coda piena: l'evento si perde, il driver non aspetta */
	char *e = event_queue[t % MAX_EVENTS];
	int k = snprintf(e, sizeof event_queue[0], "%8.3f ", (double)(now() - d.t0) / RT_NS_PER_S);
	va_list ap;
	va_start(ap, fmt);
	vsnprintf(e + k, sizeof event_queue[0] - (size_t)k, fmt, ap);
	va_end(ap);
	atomic_store(&event_head, t + 1);
}

/* ------------------------------------------------------------------------------------------ registrazione --- */

/* Registrazione dei byte grezzi (drv_config_t.record), nel formato della spia, così le prove fatte con il driver
 * servono anche a tools/taratura e a tools/valida-sim.py. I thread del driver non scrivono MAI sul file: mettono i
 * record in un anello in RAM (sempre con d.mx preso, oppure dopo che i due thread sono finiti: un solo produttore
 * alla volta) e il thread di servizio, non real-time, li scrive. Se l'anello è pieno il record si perde e si conta. */
#define RING_SIZE (1u << 20)                /* 1 MB: ~75 s di traffico (GD32 + lidar ~14 KB/s) */
#define CHANNEL_GD32 0
#define CHANNEL_LIDAR 1
static struct {
	FILE *f;
	uint8_t *buf;
	_Atomic uint32_t testa, tail;        /* byte messi / tolti, modulo 2^32 */
	_Atomic uint64_t lost;
} rec;

static void ring_copy_in(uint32_t pos, const void *src, size_t n)
{
	const uint8_t *s = src;
	for (size_t i = 0; i < n; i++)
		rec.buf[(pos + i) & (RING_SIZE - 1)] = s[i];
}

static void ring_copy_out(uint32_t pos, void *dst, size_t n)
{
	uint8_t *o = dst;
	for (size_t i = 0; i < n; i++)
		o[i] = rec.buf[(pos + i) & (RING_SIZE - 1)];
}

static void record(uint8_t canale, uint8_t direction, const void *data, size_t n, int64_t t)
{
	if (!rec.f || n == 0 || n > 4096)
		return;
	uint32_t testa = atomic_load_explicit(&rec.testa, memory_order_relaxed);
	uint32_t used = testa - atomic_load_explicit(&rec.tail, memory_order_acquire);
	if (12 + n > RING_SIZE - used) {
		atomic_fetch_add(&rec.lost, 1);
		return;
	}
	uint8_t h[12];
	uint16_t nn = (uint16_t)n;
	memcpy(h, &t, 8);
	h[8] = canale;
	h[9] = direction;
	memcpy(h + 10, &nn, 2);
	ring_copy_in(testa, h, 12);
	ring_copy_in(testa + 12, data, n);
	atomic_store_explicit(&rec.testa, testa + 12 + (uint32_t)n, memory_order_release);
}

/* Dal thread di servizio: scrive sul file tutti i record nell'anello. */
static void record_flush(void)
{
	static uint8_t data[4096];
	if (!rec.f)
		return;
	uint32_t tail = atomic_load_explicit(&rec.tail, memory_order_relaxed);
	uint32_t testa = atomic_load_explicit(&rec.testa, memory_order_acquire);
	while (tail != testa) {
		uint8_t h[12];
		int64_t t;
		uint16_t n;
		ring_copy_out(tail, h, 12);
		memcpy(&t, h, 8);
		memcpy(&n, h + 10, 2);
		ring_copy_out(tail + 12, data, n);
		reg_scrivi_record(rec.f, t, h[8], h[9], data, n);
		tail += 12 + n;
		atomic_store_explicit(&rec.tail, tail, memory_order_release);
	}
}

static int record_open(const drv_config_t *c)
{
	memset(&rec, 0, sizeof rec);
	if (!c->record)
		return 0;
	if (!(rec.buf = calloc(1, RING_SIZE)) || !(rec.f = fopen(c->record, "wb"))) {
		free(rec.buf);
		rec.buf = NULL;
		return -1;
	}
	reg_intestazione_t h;
	memset(&h, 0, sizeof h);
	h.n_canali = c->dev_lidar ? 2 : 1;
	h.canali[0].id = CHANNEL_GD32;
	h.canali[0].protocollo = REG_PROTO_GD32;
	snprintf(h.canali[0].name, sizeof h.canali[0].name, "gd32");
	snprintf(h.canali[0].dispositivo, sizeof h.canali[0].dispositivo, "%s", c->dev_gd32);
	if (c->dev_lidar) {
		h.canali[1].id = CHANNEL_LIDAR;
		h.canali[1].protocollo = REG_PROTO_LIDAR;
		snprintf(h.canali[1].name, sizeof h.canali[1].name, "lidar");
		snprintf(h.canali[1].dispositivo, sizeof h.canali[1].dispositivo, "%s", c->dev_lidar);
	}
	return reg_scrivi_intestazione(rec.f, &h);
}

static void record_close(void)
{
	if (!rec.f)
		return;
	record_flush();
	fclose(rec.f);
	rec.f = NULL;
	free(rec.buf);
	rec.buf = NULL;
}

/* -------------------------------------------------------------------------------------------- seriale --- */

static void send(uint8_t cmd_gd32, const void *data, size_t n)
{
	uint8_t fr[GD32_MAX_FRAME];
	size_t k = gd32_build_frame(cmd_gd32, data, n, fr, sizeof fr);
	if (k && write(d.fd_gd32, fr, k) != (ssize_t)k)
		d.write_errors++;
	record(CHANNEL_GD32, REG_DAL_SOC, fr, k, now());
}

static void send_u8(uint8_t c, uint8_t v) { send(c, &v, 1); }

static void send_i32(uint8_t c, int32_t v)
{
	uint8_t b[4] = { (uint8_t)v, (uint8_t)(v >> 8), (uint8_t)(v >> 16), (uint8_t)(v >> 24) };
	send(c, b, 4);
}

static void send_velocity(int32_t v, int32_t w)
{
	uint8_t b[8];
	for (int i = 0; i < 4; i++) {
		b[i] = (uint8_t)((uint32_t)v >> (8 * i));
		b[4 + i] = (uint8_t)((uint32_t)w >> (8 * i));
	}
	send(GD32_VELOCITY, b, 8);
	d.v_sent = v;
	d.w_sent = w;
}

static void full_stop(void)
{
	uint8_t buf[256];
	size_t k = interlock_stop_frames(buf, sizeof buf);
	if (k && write(d.fd_gd32, buf, k) != (ssize_t)k)
		d.write_errors++;
	record(CHANNEL_GD32, REG_DAL_SOC, buf, k, now());
	d.v_sent = d.w_sent = 0;
	d.side_sent = d.main_sent = d.suction_sent = 0;
}

/* Un altro processo ha già aperto la seriale? (es. AuxCtrl dello stack originale, o il custode) */
static int device_busy(const char *device)
{
	char real_path[PATH_MAX];
	if (!realpath(device, real_path) || strncmp(real_path, "/dev/tty", 8))
		return 0;                    /* le pseudo-seriali del simulatore (/dev/pts) le tiene aperte il simulatore */
	DIR *proc = opendir("/proc");
	if (!proc)
		return 0;
	struct dirent *p;
	int found_one = 0;
	while (!found_one && (p = readdir(proc))) {
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
				if (!strcmp(dest, real_path)) {
					fprintf(stderr, "driver: %s è già aperto dal processo %s (fermalo prima)\n", device, p->d_name);
					found_one = 1;
					break;
				}
			}
		}
		if (fds)
			closedir(fds);
	}
	closedir(proc);
	return found_one;
}

static int open_serial(const char *device)
{
	int fd = open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);
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

static void enter_fault(const char *reason)
{
	if (d.phase == DRV_FAULT)
		return;
	d.phase = DRV_FAULT;
	snprintf(d.fault, sizeof d.fault, "%s", reason);
	full_stop();
	send_u8(GD32_BUTTON_LED, LED_FAULT);
	d.led_sent = LED_FAULT;
	atomic_store(&cmd.sound, "sound_erro_harefault");
	event("GUASTO: %s. Motori fermi, LED 8", reason);
}

static void try_clear_fault(int64_t t)
{
	/* la richiesta si consuma sempre: una richiesta fatta quando non c'era un guasto non deve azzerare il prossimo */
	int request = atomic_exchange(&cmd.clear_fault, 0);
	if (d.phase != DRV_FAULT || !request)
		return;
	if (!d.has_sensors || t - d.t_sensors > SENSORS_OLD_NS || (d.cfg.bench && d.interlock.tripped)) {
		event("guasto NON azzerato: la causa c'è ancora");
		return;
	}
	for (int i = 0; i < MAX_WATCHED; i++) {
		rt_task_t *k = atomic_load(&watched[i].t);
		if (k && watched[i].critical &&
		    (rt_task_is_stuck(k, t, watched[i].threshold_ns) || atomic_load(&k->consecutive_misses) >= 3)) {
			event("guasto NON azzerato: il task %s è ancora bloccato", k->cfg.name ? k->cfg.name : "?");
			return;
		}
	}
	d.phase = DRV_READY;
	d.fault[0] = 0;
	d.task_fault = DRV_FAULT_NONE;
	d.led_sent = -1;                       /* rimanda il LED giusto al prossimo ciclo */
	for (int i = 0; i < MAX_WATCHED; i++) {
		rt_task_t *k = atomic_load(&watched[i].t);
		watched[i].reported_once = 0;
		if (k)
			watched[i].seen_misses = atomic_load(&k->total_misses);
	}
	event("guasto azzerato");
}

/* Every value of drv_task_fault_t needs its own case: -Wswitch-enum warns about an enum value without a case even when
   there is a default, and the pragma turns that warning into an error for this function only, whatever the build
   flags. The default is still needed for numbers that are not in the enum (for example a wrong cast). */
#pragma GCC diagnostic push
#pragma GCC diagnostic error "-Wswitch-enum"
const char *drv_task_fault_text(drv_task_fault_t reason)
{
	switch (reason) {
	case DRV_FAULT_NONE:                      return "no fault";
	case DRV_FAULT_UNLOCK_ATTEMPTS_EXHAUSTED: return "all the unlock attempts failed";
	case DRV_FAULT_NO_ESCAPE_CELL:            return "no free cell reachable to escape to";
	case DRV_FAULT_LEAVE_BASE_TIMEOUT:        return "could not leave the charging base in time";
	case DRV_FAULT_NO_PATH_HOME:              return "no path back to the charging base";
	case DRV_FAULT_DOCKING_FAILED:            return "docking failed";
	case DRV_FAULT_DOCKING_TIMEOUT:           return "docking took too long";
	case DRV_FAULT_INCONSISTENT_STATE:        return "inconsistent navigation state";
	case DRV_FAULT_UNKNOWN_OBJECTIVE:         return "unknown objective type";
	case DRV_FAULT_POSITION_NOT_FOUND:        return "position not found after the robot was lifted";
	case DRV_FAULT_BRUSH_STUCK:               return "main brush still blocked after its unlock";
	default:                                 return "generic fault";
	}
}
#pragma GCC diagnostic pop

/* Fault asked by a task (drv_report_fault). Checked after the reset request: if in the same tick one task asks for
   the fault and another one for the reset, the fault wins. */
static void task_fault_request(void)
{
	int reason = atomic_exchange(&cmd.task_fault, DRV_FAULT_NONE);
	if (reason == DRV_FAULT_NONE || d.phase == DRV_FAULT)
		return;
	char text[sizeof d.fault];
	snprintf(text, sizeof text, "task fault %d: %s", reason, drv_task_fault_text((drv_task_fault_t)reason));
	enter_fault(text);
	d.task_fault = reason;
}

static void publish_status(int64_t t)
{
	drv_status_t s;
	memset(&s, 0, sizeof s);
	s.t_ns = t;
	s.phase = d.phase;
	memcpy(s.fault, d.fault, sizeof s.fault);
	s.task_fault = d.task_fault;
	memcpy(s.gd32_version, d.version, sizeof s.gd32_version);
	s.volts = battery_volts(&d.bat);
	s.percentage = battery_percentage(&d.bat);
	s.battery = d.bat.level;
	if (d.cfg.test_return_s > 0 && (double)(t - d.t0) / RT_NS_PER_S >= d.cfg.test_return_s) {
		static int told;
		if (!told++)
			event("PROVA: da adesso i task leggono una batteria bassa (quella vera non cambia)");
		s.battery = BAT_LOW;
		if (s.percentage > 50)
			s.percentage = 50;          /* sotto la soglia per ripartire: una volta agganciato resta sulla base */
	}
	/* il bit "in carica" del GD32 vuol dire carica finita: la carica passa ogni volta che i contatti toccano */
	s.charging = d.s.on_base;
	s.charge_complete = d.s.charging;
	s.lifted = d.lifted;
	s.power_presses = d.presses[0];
	if (d.cfg.test_press_s > 0 && (double)(t - d.t0) / RT_NS_PER_S >= d.cfg.test_press_s) {
		static int told;
		if (!told++)
			event("PROVA: da adesso i task vedono una pressione del tasto di accensione in più");
		s.power_presses++;
	}
	s.home_presses = d.presses[1];
	s.on_base = d.s.on_base;
	s.temperature_c = d.s.temperature_c;
	s.v = d.v_sent / SCALE_V;
	s.w = d.w_sent / SCALE_W;
	s.limits = d.limits;
	s.side_brush = d.side_sent;
	s.main_brush = d.main_sent;
	s.suction_rpm = d.suction_sent;
	s.suction_limits = d.suction_limits;
	s.docking = d.dock_state.result;
	s.docking_phase = (int)d.dock_state.phase;
	s.docking_attempt = d.dock_state.attempt;
	s.led = d.led_sent;
	s.lidar_on = d.lidar_sent > 0;
	s.lidar_power = d.lidar_sent > 0 ? d.power_sent : 0;
	s.lidar_ok = d.lidar_ok;
	s.lidar_rev_s = d.assembler.rev_s;
	s.lidar_revolutions = d.assembler.revolutions;
	s.sensor_frames = d.n_sensors;
	s.bad_frames = d.dg.bad_checksums + d.dg.bad_lengths;
	s.discarded_bytes = d.dg.discarded_bytes;
	s.other_gd32_frames = d.others;
	s.gd32_heartbeats = d.gd32_hb;
	s.keepalives = d.keepalives;
	s.heartbeats = d.heartbeats;
	s.write_errors = d.write_errors;
	s.sensors_hz = (double)d.sensors_per_second[1];
	s.sensors_age_ns = d.has_sensors ? t - d.t_sensors : -1;
	s.tick_delay_max_ns = d.tick_delay_max;
	s.sensors_interval_max_ns = d.interval_max;
	s.sensors_interval_min_ns = d.interval_min;
	s.work_max_ns = d.work_max;
	s.wakeups_done = d.wakeups_done;
	s.without_rt = d.without_rt;
	exchange_write(&status_exchange, &s);
}

/* ------------------------------------------------------------------------------------ limiti e comandi --- */

/* Le condizioni per qualunque motore (ruote, spazzole, aspirazione), prima dei limiti specifici delle ruote. */
static int general_limits(int64_t t)
{
	if (!d.cfg.motors)
		return DRV_LIM_DISABLED;
	if (d.phase == DRV_STARTING)
		return DRV_LIM_NOT_READY;
	if (d.phase != DRV_READY)
		return DRV_LIM_FAULT;
	if (!d.has_sensors || t - d.t_sensors > SENSORS_OLD_NS)
		return DRV_LIM_OLD_SENSORS;
	if (d.bat.level == BAT_CRITICAL && !d.s.on_base)
		return DRV_LIM_BATTERY;
	return 0;
}

/* Comando di velocità dei task -> quello che si può davvero mandare. */
static int limit_velocity(int64_t t, int32_t *v, int32_t *w)
{
	int64_t vw = atomic_load(&cmd.vw);
	*v = (int32_t)(vw >> 32);
	*w = (int32_t)(uint32_t)vw;
	/* durante l'aggancio comanda lui, non i task; i limiti qui sotto restano (al contatto le ruote si fermano da sole) */
	int docking = docking_active(&d.dock_state);
	if (docking) {
		*v = (int32_t)lround(d.dock_state.v * SCALE_V);
		*w = (int32_t)lround(d.dock_state.w * SCALE_W);
	}
	int lim = general_limits(t);
	if (lim) {
		*v = *w = 0;
		return lim;
	}
	if (d.cfg.bench) {
		uint8_t b[8];
		for (int i = 0; i < 4; i++) {
			b[i] = (uint8_t)((uint32_t)*v >> (8 * i));
			b[4 + i] = (uint8_t)((uint32_t)*w >> (8 * i));
		}
		if (interlock_permit(&d.interlock, GD32_VELOCITY, b, 8, t) != INTERLOCK_ALLOWED) {
			*v = *w = 0;
			return DRV_LIM_BENCH;
		}
		return 0;
	}
	int ground_lim = interlock_ground_limit(&d.s, docking ? docking_detach(&d.dock_state) : atomic_load(&cmd.leaving_base), v, w);
	/* si parte solo con gli encoder che contano: vedi l'alimentazione dei sensori più sotto */
	if (!(d.s.control & GD32_CONTROL_SENSORS_ON) || t - d.t_sensors_on < RT_MS(200) || d.rest_level) {
		*v = *w = 0;
		ground_lim |= INTERLOCK_LIM_SENSORS;
	}
	if (d.lifted) {
		*v = *w = 0;
		ground_lim |= INTERLOCK_LIM_LIFTED;
	}
	/* dopo il riposo si parte con la mappa che funziona: si aspetta il primo giro del lidar, al massimo qualche secondo */
	if (d.t_wake && atomic_load(&cmd.lidar_on) && d.scan.t_end_ns < d.t_wake && t - d.t_wake < WAKE_LIDAR_MAX_NS) {
		*v = *w = 0;
		ground_lim |= INTERLOCK_LIM_SENSORS;
	}
	return ground_lim;
}

/* Spazzole e aspirazione: niente se il robot è sollevato o sulla base (a terra), o se il banco non lo permette.
 * Ritorna 0 se il comando si può mandare, altrimenti il motivo (SIC_LIM_* | DRV_LIM_*). */
static int actuator_limits(int64_t t, uint8_t cmd_gd32, int value)
{
	if (value == 0)
		return 0;
	int lim = general_limits(t);
	if (lim)
		return lim;
	if (d.cfg.bench) {
		uint8_t b = (uint8_t)value;
		return interlock_permit(&d.interlock, cmd_gd32, &b, 1, t) == INTERLOCK_ALLOWED ? 0 : DRV_LIM_BENCH;
	}
	if (d.lifted)
		return INTERLOCK_LIM_LIFTED;
	int32_t v = 1, w = 0;
	return interlock_ground_limit(&d.s, 0, &v, &w) & (INTERLOCK_LIM_LIFTED | INTERLOCK_LIM_BASE | INTERLOCK_LIM_SENSORS);
}

static int clamp(int v, int lo, int hi) { return v < lo ? lo : v > hi ? hi : v; }

/* Ruote: manda 0x66 se il comando limitato è cambiato (o sempre, con forza = 1: ogni 20 ms). */
static void update_wheels(int64_t t, int force)
{
	int32_t v, w;
	d.limits = limit_velocity(t, &v, &w);
	if (d.phase == DRV_STARTING)
		return;                            /* durante l'avvio va solo la sequenza (e l'arresto iniziale) */
	if (force || v != d.v_sent || w != d.w_sent)
		send_velocity(v, w);
}

static void update_actuators(int64_t t)
{
	int refresh = t - d.t_refresh > RT_S(2);   /* ogni 2 s si rimanda tutto: il GD32 potrebbe essersi resettato */
	if (refresh)
		d.t_refresh = t;

	int side = clamp(atomic_load(&cmd.side_brush_cmd), -100, 100);
	int center = clamp(atomic_load(&cmd.main_brush_cmd), -100, 100);
	int suction_now = clamp(atomic_load(&cmd.suction), 0, 30000);
	if (actuator_limits(t, GD32_SIDE_BRUSH, side)) side = 0;
	if (actuator_limits(t, GD32_MAIN_BRUSH, center)) center = 0;
	/* Aspirazione: anche niente cassetto della polvere (byte 4 bit 2). Il GD32 non la ferma da solo (provato il 5/10:
	   cassetto sfilato per 10 s con la ventola a 8000 giri/min, la ventola ha continuato); l'originale non la accende
	   (eccezione 6, EXCEPTION_DUSTBOX_INEXISTENCE). */
	int suction_lim = !suction_now ? 0 : !d.cfg.suction ? DRV_LIM_FORBIDDEN : actuator_limits(t, GD32_SUCTION, 1);
	if (suction_now && !suction_lim && !d.s.dustbin)
		suction_lim = DRV_LIM_NO_DUSTBIN;
	d.suction_limits = suction_lim;
	if (suction_lim) suction_now = 0;
	if (side != d.side_sent || refresh) {
		send_u8(GD32_SIDE_BRUSH, (uint8_t)(int8_t)side);
		d.side_sent = side;
	}
	if (center != d.main_sent || (refresh && center >= 0)) {   /* lo sblocco (< 0) non si ripete: il GD32 lo ferma da solo */
		send_u8(GD32_MAIN_BRUSH, (uint8_t)(int8_t)center);
		d.main_sent = center;
	}
	if (suction_now != d.suction_sent || (refresh && suction_now)) {
		int16_t a = (int16_t)(suction_now / 10);
		uint8_t b[2] = { (uint8_t)a, (uint8_t)(a >> 8) };
		send(GD32_SUCTION, b, 2);
		d.suction_sent = suction_now;
	}

	/* LED: guasto > batteria critica o bassa > in carica > quello chiesto dai task */
	int led = clamp(atomic_load(&cmd.led), 0, 31);
	if (d.phase == DRV_FAULT)
		led = LED_FAULT;
	else if (d.bat.level != BAT_OK)
		led = LED_BATTERY;
	else if (d.s.on_base)
		led = LED_CHARGING;
	if (led != d.led_sent || refresh) {
		send_u8(GD32_BUTTON_LED, (uint8_t)led);
		d.led_sent = led;
	}

	/* Riposo sulla base: a carica stabile lidar, motori e sensori si spengono; prima di uscire si riaccendono.
	   Spegnere i sensors_on (0x9d 00) ferma gli encoder, e il GD32 regola la velocità proprio con quelli: nelle prove del 5/11
	   lo spegnimento dava a volte una lettura falsa dell'encoder sinistro (r2, r10) con il robot che scattava in avanti di
	   ~12 cm staccandosi dai contatti, e uscendo con i sensori spenti spariva dall'odometria un tratto di ~11 cm (o1, o2).
	   Per questo si spengono prima i motors (0x65 00, come freeMotorControl dell'originale), e si esce solo a sensori accesi. */
	if (d.has_sensors) {
		if (d.s.on_base != d.prev_on_base) {
			d.prev_on_base = d.s.on_base;
			d.t_base_change = t;
		}
		if (d.s.charging != d.prev_charging) {
			d.prev_charging = d.s.charging;
			d.t_charging_change = t;
		}
		int leaving = atomic_load(&cmd.leaving_base) || docking_active(&d.dock_state);
		int stable = d.s.on_base && !leaving &&
		              ((d.s.charging && t - d.t_charging_change > REST_CHARGING_NS) || t - d.t_base_change > REST_CONTACT_NS);
		int wakeup = leaving || (!d.s.on_base && t - d.t_base_change > RT_MS(500));
		if (d.rest_level == 0) {
			if (d.sensors_sent != 1) {
				send_u8(GD32_SENSOR_POWER, 1);
				d.sensors_sent = 1;
				d.t_sensors_on = t;
				event("sensori accesi");
			} else if (stable) {
				send_u8(GD32_CONTROL_TYPE, 0);
				d.rest_level = 1;
				d.t_rest = t;
			}
		} else if (d.rest_level == 1) {
			if (wakeup) {
				send_u8(GD32_CONTROL_TYPE, 2);
				d.rest_level = 0;
			} else if (t - d.t_rest > RT_MS(200)) {
				send_u8(GD32_SENSOR_POWER, 0);
				d.sensors_sent = 0;
				d.rest_level = 2;
				event("riposo: motori, lidar e sensori spenti");
			}
		} else if (wakeup) {
			send_u8(GD32_SENSOR_POWER, 1);
			send_u8(GD32_CONTROL_TYPE, 2);
			d.sensors_sent = 1;
			d.t_sensors_on = t;
			d.t_wake = t;
			d.rest_level = 0;
			event("sveglia: sensori, motori e lidar accesi");
		}
	}
}

static void update_lidar(int64_t t)
{
	if (d.fd_lidar < 0)
		return;
	int on = atomic_load(&cmd.lidar_on) && !d.rest_level && !d.lifted;
	if (on != d.lidar_sent) {
		send_u8(GD32_LIDAR_POWER, (uint8_t)on);
		d.lidar_sent = on;
		d.t_lidar_on = t;
		d.lidar_ok = 1;
		lidar_regulator_init(&d.regulator, 0);
		if (!on) {
			send_i32(GD32_LIDAR_MOTOR, 0);
			d.power_sent = 0;
		}
		event("lidar %s", on ? "acceso" : "spento");
	}
	if (!on)
		return;
	double target = atomic_load(&cmd.lidar_crev_s) / 100.0;
	if (target <= 0)
		target = d.cfg.lidar_rev_s;
	d.regulator.target = target;
	int fresh = d.assembler.t_speed && t - d.assembler.t_speed < RT_MS(500) && d.assembler.t_speed > d.t_lidar_on;
	int power_now = lidar_regulate(&d.regulator, fresh ? d.assembler.rev_s : 0);
	if (!fresh && t - d.t_lidar_on > RT_S(4)) {
		if (d.lidar_ok)
			event("ATTENZIONE: il lidar non manda dati da 0,5 s (potenza tenuta a 60)");
		d.lidar_ok = 0;
		power_now = 60;
		d.regulator.integral = 0;
	} else if (fresh && !d.lidar_ok) {
		d.lidar_ok = 1;
		event("il lidar manda di nuovo dati");
	}
	if (power_now != d.power_sent || t - d.t_power > RT_S(1)) {
		send_i32(GD32_LIDAR_MOTOR, power_now);
		d.power_sent = power_now;
		d.t_power = t;
	}
}

/* ------------------------------------------------------------------------------------ frame in arrivo --- */

/* A ogni frame dei sensori: comandi dei task per l'aggancio, un passo della guida, eventi quando cambia qualcosa. */
static void update_docking(int64_t t)
{
	int c = atomic_exchange(&cmd.dock, 0);
	if (c > 0)
		docking_start(&d.dock_state, t);
	else if (c < 0)
		docking_cancel(&d.dock_state);
	if (docking_active(&d.dock_state))
		docking_frame(&d.dock_state, &d.s, t);
	if ((int)d.dock_state.phase != d.prev_dock_phase || d.dock_state.result != d.prev_dock_result) {
		event("aggancio: esito %d, fase %d, tentativo %d, ir %02x, davanti %.3f m, angolo %+.1f°%s%s", d.dock_state.result,
		       d.dock_state.phase, d.dock_state.attempt, (d.s.ir_dock_right << 4) | d.s.ir_dock_left, d.dock_state.front_dist, d.dock_state.yaw * 180 / M_PI,
		       d.dock_state.reason ? ", " : "", d.dock_state.reason ? d.dock_state.reason : "");
		d.prev_dock_phase = (int)d.dock_state.phase;
		d.prev_dock_result = d.dock_state.result;
	}
}

/* Sollevamento, a ogni frame dei sensori (50 al secondo), come CHwdriverExceptionDetect::detectWithCliffIRSensor
   dell'originale. Il robot è sollevato quando una di queste condizioni vale per tutta la sua finestra:
     - tutti e 4 gli anti-caduta sul vuoto, in almeno il 90% degli ultimi 40 frame (0,8 s);
     - due anti-caduta "incrociati" sul empty (sx anteriore + dx laterale, dx anteriore + sx laterale, i due laterali)
       per 50 frame (1 s): un bordo dritto, come un gradino, non li può dare;
     - almeno un anti-caduta sul vuoto e beccheggio fuori da -16°..+14° per 10 frame (0,2 s);
     - i due anteriori sul vuoto per 250 frame (5 s): davanti a un gradino il robot fa retromarcia molto prima
       (retromarcia automatica del GD32 e task di sicurezza), quindi qui la finestra è lunga.
   Con il robot in mano un sensore può vedere la mano o il braccio: nella prova l1 il laterale sinistro ha visto
   "pavimento" per tutto il tempo, e "tutti e 4 sul vuoto" da sola non scattava mai. Riappoggiato quando tutti e 4
   vedono il pavimento per 40 frame (0,8 s). Sollevato, i motori si spengono (0x65 00), come l'originale ("free motor
   when wheel up"); riappoggiato, si riprende a comandare solo dopo il primo giro del lidar: cosa fare poi lo decidono
   i task (drv_status_t.lifted). Sulla base non si entra in "sollevato"; a riposo o a sensori spenti (o accesi da
   meno di 300 ms) non si guarda niente, perché gli anti-caduta non valgono. */
static void update_lifting(int64_t t)
{
	if (d.rest_level || !(d.s.control & GD32_CONTROL_SENSORS_ON) || t - d.t_sensors_on <= RT_MS(300)) {
		d.void_40 = 0;
		d.n_crossed = d.n_front = d.n_tilted = d.n_floor = 0;
		return;
	}
	const int *c = d.s.cliff;
	int all = c[0] && c[1] && c[2] && c[3], none = !c[0] && !c[1] && !c[2] && !c[3];
	double pitch = d.s.angle[1] / 100.0;
	d.void_40 = (d.void_40 << 1 | (uint64_t)all) & ((1ULL << 40) - 1);
	d.n_crossed = (c[1] && c[3]) || (c[2] && c[0]) || (c[0] && c[3]) ? d.n_crossed + 1 : 0;
	d.n_front = c[1] && c[2] ? d.n_front + 1 : 0;
	d.n_tilted = !none && (pitch < -16 || pitch > 14) ? d.n_tilted + 1 : 0;
	d.n_floor = none ? d.n_floor + 1 : 0;
	if (!d.lifted) {
		const char *reason = d.s.on_base ? NULL
		                   : __builtin_popcountll(d.void_40) >= 36 ? "tutti e 4 gli anti-caduta sul vuoto"
		                   : d.n_crossed >= 50 ? "due anti-caduta incrociati sul vuoto"
		                   : d.n_tilted >= 10 ? "anti-caduta sul vuoto e robot inclinato"
		                   : d.n_front >= 250 ? "i due anti-caduta anteriori sul vuoto da 5 s" : NULL;
		if (reason) {
			d.lifted = 1;
			send_u8(GD32_CONTROL_TYPE, 0);
			if (docking_active(&d.dock_state))
				docking_abort(&d.dock_state, "robot sollevato");
			event("sollevato (%s, beccheggio %+.1f°): motori e lidar spenti", reason, pitch);
		}
	} else if (d.n_floor >= 40) {
		d.lifted = 0;
		d.void_40 = 0;
		d.n_crossed = d.n_front = d.n_tilted = 0;
		send_u8(GD32_CONTROL_TYPE, 2);
		d.t_wake = t;
		event("riappoggiato: motori e lidar accesi");
	}
}

/* Tasti e base, a ogni frame dei sensori. Il GD32 manda da quanto è premuto ogni tasto (byte 58 e 62, 0 se rilasciato):
   al rilascio, se la pressione è durata meno di DRV_SHORT_PRESS_MS, è una pressione breve. Il frame arriva ogni 20 ms,
   quindi qui si vedono anche le pressioni brevissime, che un task da 100 ms potrebbe perdere. Le pressioni si contano al
   tick, dopo l'azzeramento del fault (vedi buttons_and_base). Lo stesso per il ritorno sulla base dopo almeno 1 s fuori */
static void update_buttons_and_base(int64_t t)
{
	const int pressed_ms[2] = { d.s.power_button_ms, d.s.home_button_ms };
	static const char *names[2] = { "accensione", "casa" };
	for (int k = 0; k < 2; k++) {
		if (pressed_ms[k] > 0) {
			if (pressed_ms[k] > d.button_max_ms[k])
				d.button_max_ms[k] = pressed_ms[k];
		} else if (d.button_max_ms[k] > 0) {
			if (d.button_max_ms[k] < DRV_SHORT_PRESS_MS) {
				d.pending_presses[k]++;
				event("tasto %s: pressione breve (%d ms)", names[k], d.button_max_ms[k]);
			} else {
				event("tasto %s: pressione lunga (%d ms), non conta", names[k], d.button_max_ms[k]);
			}
			d.button_max_ms[k] = 0;
		}
	}
	if (!d.s.on_base) {
		if (!d.t_off_base)
			d.t_off_base = t;
	} else {
		if (d.t_off_base && t - d.t_off_base >= RT_S(1))
			d.back_on_base = 1;
		d.t_off_base = 0;
	}
}

/* Al tick: una pressione breve o il ritorno sulla base azzerano il guasto (se la causa non c'è più), poi le pressioni si
   contano. Così un task che vede crescere il numero trova già il guasto azzerato (decisione dell'utente, 2026-11-06) */
static void buttons_and_base(int64_t t)
{
	int pressed = d.pending_presses[0] + d.pending_presses[1];
	if (d.phase == DRV_FAULT && (pressed || d.back_on_base)) {
		event("guasto: provo ad azzerarlo (%s)", pressed ? "tasto premuto" : "robot rimesso sulla base");
		atomic_store(&cmd.clear_fault, 1);
	}
	d.back_on_base = 0;
	try_clear_fault(t);
	for (int k = 0; k < 2; k++) {
		d.presses[k] += (uint32_t)d.pending_presses[k];
		d.pending_presses[k] = 0;
	}
}

static void on_gd32_frame(const gd32_frame_t *f, void *ctx)
{
	int64_t t = *(const int64_t *)ctx;
	if (f->cmd == GD32_SENSORS) {
		if (gd32_read_sensors(f, &d.s))
			return;
		/* intervallo fra due frame visti dal driver: il GD32 li manda ogni 20,0 ms, quindi lo scarto misura i ritardi
		 * del percorso seriale (interrupt, kernel, thread del driver) */
		if (d.has_sensors && d.n_sensors > 50) {
			int64_t dt = t - d.t_sensors;
			if (dt > d.interval_max)
				d.interval_max = dt;
			if (dt < d.interval_min || d.interval_min == 0)
				d.interval_min = dt;
		}
		d.has_sensors = 1;
		d.t_sensors = t;
		d.n_sensors++;
		d.sensors_per_second[0]++;
		update_lifting(t);
		update_buttons_and_base(t);
		/* "in movimento" come l'originale: v + |ω| misurati (byte 32 e 36) maggiori di zero */
		int moving = d.s.v_measured / SCALE_V + fabs(d.s.w_measured / SCALE_W) > 0;
		int new_level = battery_update(&d.bat, d.s.voltage_dv, d.s.charging, d.s.on_base, moving, d.s.fan_rpm);
		if (new_level != d.prev_battery) {
			static const char *names[] = { "normale", "BASSA (LED 6)", "CRITICA: motori fermi" };
			event("batteria %s, %.2f V", names[new_level], battery_volts(&d.bat));
			d.prev_battery = new_level;
		}
		if (d.cfg.bench && interlock_received(&d.interlock, f, t)) {
			full_stop();
			enter_fault(d.interlock.reason);
		}
		drv_sensors_t p = { .number = d.n_sensors, .t_ns = t, .s = d.s };
		exchange_write(&sensors_exchange, &p);
		drv_pose_t pose;
		int has_pose = exchange_read(&pose_exchange, &pose) != 0;
		int stuck = atomic_load(&cmd.stuck);
		if (contacts_frame(&d.contacts, &d.s, t, has_pose ? &pose : NULL, stuck))   /* prima di svegliare i task: trovano già l'evento */
			exchange_write(&contacts_exchange, &d.contacts.published);
		for (int i = 0; i < MAX_SUBSCRIBERS; i++) {
			rt_task_t *k = atomic_load(&subscribers[i]);
			if (k)
				rt_task_wake(k, t);
		}
		update_docking(t);
		/* reazione immediata: se i nuovi sensori cambiano il comando permesso (bumper, vuoto), lo si manda ora */
		update_wheels(t, 0);
		return;
	}
	switch (f->cmd) {
	case GD32_HEARTBEAT:
		d.gd32_hb++;
		break;
	case GD32_VERSION: {
		size_t n = f->data_len < 32 ? f->data_len : 32;
		memcpy(d.version, f->data, n);
		d.version[n] = 0;
		for (size_t i = 0; i < n; i++)
			if (d.version[i] < 32 || d.version[i] > 126)
				d.version[i] = 0;
		event("GD32 firmware %s", d.version);
		break;
	}
	case GD32_WAKEUP_ACK:
		send(GD32_WAKEUP_ACK, NULL, 0);     /* non lo mettiamo mai in sospensione, ma se chiede conferma la diamo */
		d.others++;
		break;
	case GD32_ERROR_CODE:
	case GD32_DEBUG:
		event("GD32 manda 0x%02x (%s), %d byte", f->cmd, gd32_cmd_name(f->cmd), f->data_len);
		d.others++;
		break;
	default:
		d.others++;
		break;
	}
}

static void on_lidar_packet(const lidar_packet_t *p, void *ctx)
{
	int64_t t = *(const int64_t *)ctx;
	if (lidar_assemble(&d.assembler, p, t, &d.scan)) {
		exchange_write(&scan_exchange, &d.scan);
		docking_scan(&d.dock_state, &d.scan, t);
	}
}

/* ------------------------------------------------------------------------------------------ ogni 20 ms --- */

/* Sequenza di avvio di everest-server (docs/protocollo.md), un frame per ciclo. */
static const struct { uint8_t cmd, n, data[4]; } STARTUP[] = {
	{ GD32_SOC_WATCHDOG, 1, { 1 } }, { GD32_VERSION, 0, { 0 } }, { GD32_IMU_RAW, 4, { 1, 0, 0, 0 } },
	{ GD32_MACHINE_FUNCTIONS, 0, { 0 } }, { GD32_LIDAR_MOTOR, 4, { 0, 0, 0, 0 } }, { GD32_CONTROL_TYPE, 1, { 0 } },
	{ GD32_CLIFF_BACKOFF, 1, { 1 } }, { GD32_SENSOR_MASK, 3, { 0, 0xff, 0xff } }, { GD32_CLIFF_IR, 1, { 1 } },
	{ GD32_IMU_ZERO, 0, { 0 } }, { GD32_CONTROL_TYPE, 1, { 2 } },
};
#define N_STARTUP ((int)(sizeof STARTUP / sizeof STARTUP[0]))

static void tick(int64_t t)
{
	if (d.phase == DRV_STARTING) {
		if (d.startup_step < N_STARTUP) {
			send(STARTUP[d.startup_step].cmd, STARTUP[d.startup_step].data, STARTUP[d.startup_step].n);
			d.startup_step++;
		} else if (d.has_sensors) {
			d.phase = DRV_READY;
			d.led_sent = -1;
			d.sensors_sent = -1;
			send_velocity(0, 0);
			event("pronto: GD32 %s, %u frame dei sensori già ricevuti",
			       d.version[0] ? d.version : "(versione non ricevuta)", d.n_sensors);
		} else if (t - d.t_startup > RT_S(5)) {
			enter_fault("il GD32 non manda i sensori (5 s dall'avvio)");
		}
	}

	/* il GD32 tace? */
	if (d.phase == DRV_READY && d.has_sensors && t - d.t_sensors > SENSORS_SILENT_NS)
		enter_fault("nessun frame dei sensori da 1 s");

	/* watchdog dei task */
	for (int i = 0; i < MAX_WATCHED; i++) {
		rt_task_t *k = atomic_load(&watched[i].t);
		if (!k || watched[i].reported_once)
			continue;
		const char *name = k->cfg.name ? k->cfg.name : "?";
		char reason[96] = "";
		/* Sul robot una sola scadenza mancata, di qualsiasi task, è un guasto (decisione del 2026-11-04). Nel
		   simulatore, dove lo scheduling è quello normale del PC, restano solo contate. */
		int deadline_missed = !d.cfg.without_rt && atomic_load(&k->total_misses) > watched[i].seen_misses;
		if (deadline_missed) {
			snprintf(reason, sizeof reason, "il task %s ha mancato una scadenza", name);
			event("PRIMA SCADENZA MANCATA %s job %llu: risposta %.3f ms su %.3f (attesa %.3f, esecuzione %.3f, di cui CPU %.3f)",
			       name, (unsigned long long)k->first_miss.job, k->first_miss.response_ns / 1e6,
			       k->cfg.deadline_ns / 1e6, k->first_miss.latency_ns / 1e6,
			       k->first_miss.execution_ns / 1e6, k->first_miss.cpu_ns / 1e6);
		} else if (rt_task_is_stuck(k, t, watched[i].threshold_ns))
			snprintf(reason, sizeof reason, "il task %s è bloccato", name);
		else if (atomic_load(&k->consecutive_misses) >= 3)
			snprintf(reason, sizeof reason, "il task %s ha mancato 3 scadenze di fila", name);
		if (!reason[0])
			continue;
		watched[i].reported_once = 1;
		if (watched[i].critical || deadline_missed)
			enter_fault(reason);
		else
			event("ATTENZIONE: %s (non critico)", reason);
	}
	buttons_and_base(t);
	task_fault_request();

	if (d.phase != DRV_STARTING) {
		update_wheels(t, 1);
		update_actuators(t);
		update_lidar(t);
	}
	if (t - d.t_second >= RT_S(1)) {
		d.sensors_per_second[1] = d.sensors_per_second[0];
		d.sensors_per_second[0] = 0;
		d.t_second += RT_S(1);
		if (t - d.t_second > RT_S(1))
			d.t_second = t;
	}
	publish_status(t);
}

/* --------------------------------------------------------------------------------------------- thread --- */

/* Legge tutto quello che c'è. Ritorna -1 se la seriale è chiusa dall'altra parte (succede solo con le
 * pseudo-seriali del simulatore, quando il simulatore esce). */
static int read_serial(int fd, int is_gd32, int64_t t)
{
	uint8_t buf[1024];
	ssize_t k;
	while ((k = read(fd, buf, sizeof buf)) > 0) {
		record(is_gd32 ? CHANNEL_GD32 : CHANNEL_LIDAR, REG_VERSO_SOC, buf, (size_t)k, t);
		if (is_gd32)
			gd32_deframer_feed(&d.dg, buf, (size_t)k, on_gd32_frame, &t);
		else
			lidar_deframer_feed(&d.dl, buf, (size_t)k, on_lidar_packet, &t);
	}
	return (k == 0 || (k < 0 && errno != EAGAIN && errno != EINTR)) ? -1 : 0;
}

static void pin_cpu(void)
{
	if (d.cfg.cpu >= 0) {
		cpu_set_t set;
		CPU_ZERO(&set);
		CPU_SET(d.cfg.cpu, &set);
		pthread_setaffinity_np(pthread_self(), sizeof set, &set);
	}
}

/*
 * Sposta sul core `cpu` il lavoro della flash che non è nostro (prova sul robot del 2026-11-07, docs/PIANO.md, task
 * aperiodico): le interruzioni della flash e del DMA, che il kernel manda al core 0 (quello del driver), e i thread del
 * kernel che scrivono sulla flash. Così una scrittura del task aperiodico (core 3) non ritarda il driver. Le
 * interruzioni si cercano per nome in /proc/interrupts, i thread per nome in /proc/PID/comm. kblockd e i kworker/u il
 * kernel 3.4 non li lascia spostare. Si fa una volta, all'avvio; lo stato resta fino al riavvio del robot.
 */
static void move_flash_work(int cpu)
{
	int irq = 0, thread = 0;
	char line[256], path[64];

	FILE *f = fopen("/proc/interrupts", "r");
	while (f && fgets(line, sizeof line, f)) {
		size_t n = strlen(line);
		while (n && (line[n - 1] == '\n' || line[n - 1] == ' '))
			line[--n] = 0;
		int num;
		if ((n >= 5 && !strcmp(line + n - 5, " nand")) || (n >= 11 && !strcmp(line + n - 11, " sunxi_dmac"))) {
			if (sscanf(line, " %d:", &num) != 1)
				continue;
			snprintf(path, sizeof path, "/proc/irq/%d/smp_affinity", num);
			FILE *a = fopen(path, "w");
			if (a && fprintf(a, "%x\n", 1u << cpu) > 0 && fclose(a) == 0)
				irq++;
			else if (a)
				fclose(a);
		}
	}
	if (f)
		fclose(f);

	cpu_set_t set;
	CPU_ZERO(&set);
	CPU_SET(cpu, &set);
	DIR *dir = opendir("/proc");
	struct dirent *e;
	while (dir && (e = readdir(dir))) {
		int pid = atoi(e->d_name);
		if (pid <= 0)
			continue;
		snprintf(path, sizeof path, "/proc/%d/comm", pid);
		FILE *c = fopen(path, "r");
		if (!c)
			continue;
		char name[32] = "";
		if (fgets(name, sizeof name, c))
			name[strcspn(name, "\n")] = 0;
		fclose(c);
		if (!strcmp(name, "nand") || !strcmp(name, "nand_rcd") || !strcmp(name, "bdi-default") ||
		    !strcmp(name, "sync_supers") || !strncmp(name, "jbd2/", 5) || !strncmp(name, "flush-", 6))
			if (sched_setaffinity(pid, sizeof set, &set) == 0)
				thread++;
	}
	if (dir)
		closedir(dir);
	event("flash sul core %d: %d interruzioni e %d thread del kernel spostati", cpu, irq, thread);
}

static int64_t min_ns(int64_t a, int64_t b) { return a < b ? a : b; }

static void work_done(int64_t start)
{
	int64_t work = now() - start;
	if (work > d.work_max && d.wakeups_done > 100)
		d.work_max = work;
	d.wakeups_done++;
	atomic_fetch_add(&driver_rounds, 1);
}

/*
 * Il driver ha due thread, sullo stesso core (docs/fase3.md, prove r4-r6: un thread che dorme in ppoll sulle seriali
 * a volte si sveglia con millisecondi di ritardo, uno che dorme con clock_nanosleep no):
 *   - TEMPO (priorità cfg.priority, 99): dorme con clock_nanosleep fino al prossimo evento a tempo e fa heartbeat,
 *     keepalive, il ciclo da 20 ms (0x66, attuatori, lidar, watchdog, stato);
 *   - RICEZIONE (priorità cfg.priority - 1, 98): dorme in ppoll sulle seriali e sull'eventfd dei comandi; decodifica,
 *     pubblica, sveglia i task, reagisce subito ai sensori e ai comandi nuovi.
 * Lo stato interno del driver (d) lo usano entrambi, sempre con d.mx preso (mutex con priority inheritance; sezioni
 * di decine di microsecondi). I task non lo toccano mai.
 */
static void *timer_loop(void *arg)
{
	(void)arg;
	pin_cpu();
	pthread_mutex_lock(&d.mx);
	int64_t t = now();
	d.t_startup = d.t_second = t;
	d.next_tick = d.next_hb = d.next_ka = t;
	full_stop();                                  /* stato sicuro prima di tutto */
	event("driver avviato (CPU %d, %s), sequenza di avvio del GD32", d.cfg.cpu,
	       d.without_rt ? "SENZA SCHED_FIFO" : "SCHED_FIFO 99 + ricezione 98");
	if (!d.cfg.without_rt && !d.without_rt && d.cfg.cpu_flash >= 0)      /* solo sul robot: nel simulatore cfg.without_rt è 1 */
		move_flash_work(d.cfg.cpu_flash);
	pthread_mutex_unlock(&d.mx);

	while (!atomic_load(&d.stop)) {
		rt_sleep_until(min_ns(d.next_tick, min_ns(d.next_hb, d.next_ka)));
		if (atomic_load(&d.stop))
			break;
		pthread_mutex_lock(&d.mx);
		int64_t start = now();
		t = start;
		if (t >= d.next_hb) {
			send(GD32_HEARTBEAT, NULL, 0);
			d.heartbeats++;
			d.next_hb += HEARTBEAT_NS;
			if (d.next_hb <= t)
				d.next_hb = t + HEARTBEAT_NS;
		}
		if (t >= d.next_ka) {
			send(GD32_KEEPALIVE, NULL, 0);
			d.keepalives++;
			d.next_ka += KEEPALIVE_NS;
			if (d.next_ka <= t)
				d.next_ka = t + KEEPALIVE_NS;
		}
		if (t >= d.next_tick) {
			int64_t delay = t - d.next_tick;
			if (delay > d.tick_delay_max && d.wakeups_done > 100) {  /* i primi giri (avvio) non contano */
				d.tick_delay_max = delay;
				if (delay > RT_US(300))
					event("ritardo del ciclo da 20 ms: %.0f us (nuovo massimo)", delay / 1e3);
			}
			tick(t);
			d.next_tick += TICK_NS;
			if (d.next_tick <= t)
				d.next_tick = t + TICK_NS;
		}
		work_done(start);
		pthread_mutex_unlock(&d.mx);
	}
	return NULL;
}

static void *receiver_loop(void *arg)
{
	(void)arg;
	pin_cpu();
	while (!atomic_load(&d.stop)) {
		struct pollfd p[3] = {
			{ .fd = d.gd32_closed ? -1 : d.fd_gd32, .events = POLLIN }, { .fd = d.fd_wakeup, .events = POLLIN },
			{ .fd = d.lidar_closed ? -1 : d.fd_lidar, .events = POLLIN },
		};
		struct timespec to = { .tv_sec = 1 };
		if (ppoll(p, d.fd_lidar >= 0 ? 3 : 2, &to, NULL) <= 0)
			continue;
		pthread_mutex_lock(&d.mx);
		int64_t start = now(), t = start;
		if ((p[0].revents & (POLLIN | POLLHUP | POLLERR)) && read_serial(d.fd_gd32, 1, t) < 0 && !d.gd32_closed) {
			d.gd32_closed = 1;
			enter_fault("la seriale del GD32 è stata chiusa");
		}
		if (d.fd_lidar >= 0 && (p[2].revents & (POLLIN | POLLHUP | POLLERR)) && read_serial(d.fd_lidar, 0, t) < 0 &&
		    !d.lidar_closed) {
			d.lidar_closed = 1;
			event("ATTENZIONE: la seriale del lidar è stata chiusa");
		}
		if (p[1].revents & POLLIN) {
			uint64_t n;
			if (read(d.fd_wakeup, &n, sizeof n) < 0)
				n = 0;
			if (d.phase != DRV_STARTING && !atomic_load(&d.stop)) {   /* comandi nuovi dei task: subito */
				update_wheels(t, 0);
				update_actuators(t);
			}
		}
		work_done(start);
		pthread_mutex_unlock(&d.mx);
	}
	return NULL;
}

/* Dopo che i due thread sono terminati: tutto fermo. */
static void shutdown(void)
{
	full_stop();
	if (d.fd_lidar >= 0) {
		send_u8(GD32_LIDAR_POWER, 0);
		send_i32(GD32_LIDAR_MOTOR, 0);
	}
	send_u8(GD32_BUTTON_LED, LED_NORMAL);
	tcdrain(d.fd_gd32);
	d.phase = DRV_STOPPED;
	publish_status(now());
	event("driver fermato: motori a zero, lidar spento");
}

/* Thread NON real-time: stampa gli eventi e suona. Così il driver non fa mai I/O lento. */
static void *service_loop(void *arg)
{
	(void)arg;
	for (;;) {
		int end = atomic_load(&d.stop) && d.phase == DRV_STOPPED;
		while (atomic_load(&event_tail) != atomic_load(&event_head)) {
			uint32_t c = atomic_load(&event_tail);
			fprintf(stderr, "driver %s\n", event_queue[c % MAX_EVENTS]);
			atomic_store(&event_tail, c + 1);
		}
		const char *s = atomic_exchange(&cmd.sound, NULL);
		if (s && d.cfg.sound_cmd) {
			char c[300];
			snprintf(c, sizeof c, d.cfg.sound_cmd, s);
			char *argv[] = { "sh", "-c", c, NULL };
			pid_t pid;
			if (posix_spawn(&pid, "/bin/sh", NULL, NULL, argv, environ) == 0)
				waitpid(pid, NULL, 0);
		}
		record_flush();
		if (end)
			break;
		usleep(20000);
	}
	return NULL;
}

/* ------------------------------------------------------------------------------------------------ API --- */

static void wake_driver(void);

/* Thread SCHED_FIFO alla priorità data, sullo stack di RT_STACK_DEFAULT; senza permessi (solo con cfg.without_rt)
 * parte con lo scheduling normale. */
static int create_thread(pthread_t *th, void *(*f)(void *), int prio)
{
	pthread_attr_t a;
	struct sched_param sp = { .sched_priority = prio };
	pthread_attr_init(&a);
	pthread_attr_setinheritsched(&a, PTHREAD_EXPLICIT_SCHED);
	pthread_attr_setschedpolicy(&a, SCHED_FIFO);
	pthread_attr_setschedparam(&a, &sp);
	pthread_attr_setstacksize(&a, RT_STACK_DEFAULT);
	int r = pthread_create(th, &a, f, NULL);
	if (r == EPERM && d.cfg.without_rt) {
		pthread_attr_setinheritsched(&a, PTHREAD_INHERIT_SCHED);
		if (!d.without_rt)
			fprintf(stderr, "driver: niente permessi per SCHED_FIFO: gira con lo scheduling normale (solo per il PC)\n");
		d.without_rt = 1;
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
	c->without_rt = getenv("RTES_GD32") != NULL;   /* nel simulatore si parte anche senza permessi per SCHED_FIFO */
	c->record = getenv("RTES_REGISTRA");       /* i programmi dei task non hanno opzioni: la registrazione si chiede così */
	c->test_return_s = getenv("RTES_PROVA_RIENTRO_DOPO_S") ? atof(getenv("RTES_PROVA_RIENTRO_DOPO_S")) : 0;
	c->test_press_s = getenv("RTES_PROVA_PREMI_DOPO_S") ? atof(getenv("RTES_PROVA_PREMI_DOPO_S")) : 0;
	c->cpu = 0;
	c->cpu_flash = 3;
	c->priority = RT_PRIO_DRIVER;
	c->motors = 1;
	c->suction = 1;
	c->expected_cliff = -1;
	c->lidar_rev_s = 6.0;
	c->lidar_offset_cdeg = LIDAR_OFFSET_CDEG;
	c->sound_cmd = access("/bin/tinyplayer", X_OK) == 0 ? "tinyplayer /usr/share/audio/english/%s.mp3 >/dev/null 2>&1"
	                                                     : NULL;
														 
}

void drv_config_test(drv_config_t *c)
{
	drv_config_default(c);
	c->suction = 0;
}

int drv_start(const drv_config_t *c)
{
	memset(&d, 0, sizeof d);
	d.cfg = *c;
	d.fd_lidar = -1;
	d.t0 = now();
	d.phase = DRV_STARTING;
	d.side_sent = d.main_sent = d.suction_sent = 0;
	d.led_sent = d.sensors_sent = -1;
	d.prev_battery = BAT_OK;
	gd32_deframer_init(&d.dg);
	lidar_deframer_init(&d.dl);
	lidar_assembler_init(&d.assembler, c->lidar_offset_cdeg);
	lidar_regulator_init(&d.regulator, 0);
	battery_init(&d.bat);
	contacts_init(&d.contacts);
	docking_init(&d.dock_state);
	atomic_store(&cmd.dock, 0);
	interlock_init(&d.interlock, 1, SENSORS_OLD_NS);
	if (c->expected_cliff >= 0)
		interlock_set_expected_cliff(&d.interlock, (uint8_t)c->expected_cliff);
	exchange_init(&sensors_exchange, &sensors_buffers[0], &sensors_buffers[1], sizeof sensors_buffers[0]);
	exchange_init(&scan_exchange, &scan_buffers[0], &scan_buffers[1], sizeof scan_buffers[0]);
	exchange_init(&status_exchange, &status_buffers[0], &status_buffers[1], sizeof status_buffers[0]);
	exchange_init(&contacts_exchange, &contacts_buffers[0], &contacts_buffers[1], sizeof contacts_buffers[0]);
	exchange_init(&pose_exchange, &pose_buffers[0], &pose_buffers[1], sizeof pose_buffers[0]);
	atomic_store(&pose_ready, 1);
	atomic_store(&cmd.vw, 0);
	atomic_store(&cmd.side_brush_cmd, 0);
	atomic_store(&cmd.main_brush_cmd, 0);
	atomic_store(&cmd.suction, 0);
	atomic_store(&cmd.led, LED_NORMAL);
	atomic_store(&cmd.lidar_on, 0);
	atomic_store(&cmd.lidar_crev_s, 0);
	atomic_store(&cmd.leaving_base, 0);
	atomic_store(&cmd.clear_fault, 0);
	atomic_store(&cmd.sound, NULL);
	atomic_store(&cmd.task_fault, DRV_FAULT_NONE);
	atomic_store(&event_head, 0);
	atomic_store(&event_tail, 0);
	atomic_init(&d.stop, 0);

	if (!c->force && (device_busy(c->dev_gd32) || (c->dev_lidar && device_busy(c->dev_lidar))))
		return EBUSY;
	if (record_open(c) != 0) {
		int e = errno;
		fprintf(stderr, "driver: registrazione in %s: %s\n", c->record, strerror(e));
		return e ? e : EIO;
	}
	if ((d.fd_gd32 = open_serial(c->dev_gd32)) < 0) {
		int e = errno;
		fprintf(stderr, "driver: apro %s: %s\n", c->dev_gd32, strerror(e));
		record_close();
		return e;
	}
	if (c->dev_lidar && *c->dev_lidar && (d.fd_lidar = open_serial(c->dev_lidar)) < 0) {
		int e = errno;
		fprintf(stderr, "driver: apro %s: %s\n", c->dev_lidar, strerror(e));
		close(d.fd_gd32);
		record_close();
		return e;
	}
	d.fd_wakeup = eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC);
	if (rt_init() != 0)
		fprintf(stderr, "driver: mlockall non riuscito (sul PC va bene, sul robot no)\n");

	rt_mutex_init(&d.mx);
	int r = create_thread(&d.thread, timer_loop, c->priority);
	if (r == 0) {
		r = create_thread(&d.receiver, receiver_loop, c->priority > RT_PRIO_MIN ? c->priority - 1 : c->priority);
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
		record_close();
		return r;
	}
	pthread_attr_t a;
	/* Il thread di servizio con uno stack piccolo: dopo mlockall ogni stack è bloccato in RAM, e quello di default
	 * (8 MB) supera il limite di memoria bloccabile dei normali utenti (sul PC pthread_create fallirebbe). */
	pthread_attr_init(&a);
	pthread_attr_setstacksize(&a, 128 * 1024);
	r = pthread_create(&d.service, &a, service_loop, NULL);
	pthread_attr_destroy(&a);
	if (r) {
		fprintf(stderr, "driver: non riesco a creare il thread di servizio: %s\n", strerror(r));
		atomic_store(&d.stop, 1);
		wake_driver();
		pthread_join(d.thread, NULL);
		pthread_join(d.receiver, NULL);
		close(d.fd_gd32);
		if (d.fd_lidar >= 0)
			close(d.fd_lidar);
		record_close();
		return r;
	}
	d.started = 1;
	return 0;
}

static void wake_driver(void)
{
	uint64_t one = 1;
	if (write(d.fd_wakeup, &one, sizeof one) < 0)
		return;
}

void drv_stop(void)
{
	if (!d.started)
		return;
	d.started = 0;
	atomic_store(&d.stop, 1);
	wake_driver();
	pthread_join(d.receiver, NULL);
	pthread_join(d.thread, NULL);        /* si sveglia al prossimo evento a tempo: al massimo 20 ms */
	shutdown();
	pthread_join(d.service, NULL);
	record_close();
	close(d.fd_gd32);
	if (d.fd_lidar >= 0)
		close(d.fd_lidar);
	close(d.fd_wakeup);
}

uint32_t drv_sensors(drv_sensors_t *out) { return exchange_read(&sensors_exchange, out); }
uint32_t drv_scan(lidar_scan_t *out) { return exchange_read(&scan_exchange, out); }
uint32_t drv_status(drv_status_t *out) { return exchange_read(&status_exchange, out); }
uint32_t drv_contacts(drv_contacts_t *out) { return exchange_read(&contacts_exchange, out); }

void drv_point_xy(const lidar_point_t *p, double *x, double *y)
{
	lidar_point_xy(p, x, y);
}

void drv_pose(double x, double y, double theta)
{
	if (!atomic_load(&pose_ready) || !(x == x) || !(y == y) || !(theta == theta))   /* prima di drv_start, o NaN */
		return;
	drv_pose_t p = { .x = x, .y = y, .theta = theta, .t_ns = now() };
	exchange_write(&pose_exchange, &p);
}

void drv_velocity(double v, double w)
{
	if (!(v == v)) v = 0;                         /* NaN */
	if (!(w == w)) w = 0;
	v = fmax(-0.5, fmin(0.5, v));
	w = fmax(-2 * M_PI, fmin(2 * M_PI, w));
	int32_t vt = (int32_t)lround(v * SCALE_V), wt = (int32_t)lround(w * SCALE_W);
	atomic_store(&cmd.vw, (int64_t)((uint64_t)(uint32_t)vt << 32 | (uint32_t)wt));
	wake_driver();
}

void drv_brushes(int side_speed, int main_speed)
{
	atomic_store(&cmd.side_brush_cmd, side_speed);
	atomic_store(&cmd.main_brush_cmd, main_speed);
	wake_driver();
}

void drv_suction(int rpm)
{
	atomic_store(&cmd.suction, rpm);
	wake_driver();
}

void drv_led(int mode)
{
	atomic_store(&cmd.led, mode);
	wake_driver();
}

void drv_lidar(int on, double rev_s)
{
	atomic_store(&cmd.lidar_crev_s, rev_s > 0 ? (int)lround(rev_s * 100) : 0);
	atomic_store(&cmd.lidar_on, on ? 1 : 0);
}

void drv_leave_base(int enable)
{
	atomic_store(&cmd.leaving_base, enable ? 1 : 0);
	wake_driver();
}

void drv_dock(int enable)
{
	atomic_store(&cmd.dock, enable ? 1 : -1);      /* lo legge il prossimo frame: niente risveglio */
}

void drv_stuck(int enable)
{
	atomic_store(&cmd.stuck, enable ? 1 : 0);   /* lo legge il prossimo frame: niente risveglio */
}

void drv_sound(const char *name)
{
	atomic_store(&cmd.sound, name);
}

void drv_report_fault(drv_task_fault_t reason)
{
	atomic_store(&cmd.task_fault, (int)reason);   /* read at the next tick */
}

int drv_clear_fault(void)
{
	atomic_store(&cmd.clear_fault, 1);
	for (int i = 0; i < 10; i++) {           /* aspetta fino a 2 tick */
		usleep(5000);
		drv_status_t s;
		if (drv_status(&s) && s.phase != DRV_FAULT)
			return 0;
	}
	return -1;
}

int drv_wake_on_sensors(rt_task_t *t)
{
	if (!t->cfg.sporadic)
		return EINVAL;
	for (int i = 0; i < MAX_SUBSCRIBERS; i++) {
		rt_task_t *empty = NULL;
		if (atomic_compare_exchange_strong(&subscribers[i], &empty, t))
			return 0;
	}
	return ENOSPC;
}

int drv_watch(rt_task_t *t, int critical, int64_t threshold_ns)
{
	for (int i = 0; i < MAX_WATCHED; i++) {
		if (atomic_load(&watched[i].t))
			continue;
		watched[i].critical = critical;
		watched[i].threshold_ns = threshold_ns > 0 ? threshold_ns : 3 * t->cfg.period_ns;
		watched[i].reported_once = 0;
		watched[i].seen_misses = 0;   /* conta anche quelle di prima: il task parte prima di essere sorvegliato */
		rt_task_t *empty = NULL;
		if (atomic_compare_exchange_strong(&watched[i].t, &empty, t))
			return 0;
	}
	return ENOSPC;
}

void drv_forget(rt_task_t *t)
{
	for (int i = 0; i < MAX_SUBSCRIBERS; i++) {
		rt_task_t *x = t;
		atomic_compare_exchange_strong(&subscribers[i], &x, NULL);
	}
	for (int i = 0; i < MAX_WATCHED; i++) {
		rt_task_t *x = t;
		atomic_compare_exchange_strong(&watched[i].t, &x, NULL);
	}
	/* il driver potrebbe avere in mano il puntatore: si aspetta che finisca il risveglio in corso */
	if (d.phase == DRV_STOPPED || atomic_load(&d.stop))
		return;
	uint64_t g = atomic_load(&driver_rounds);
	for (int i = 0; i < 100 && atomic_load(&driver_rounds) < g + 2; i++) {
		wake_driver();
		usleep(1000);
	}
}
