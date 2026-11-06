/*
 * driver.h - Il driver del robot: il nostro sostituto di AuxCtrl e della parte di everest-server che parla con il GD32.
 *
 * Il driver sono DUE thread SCHED_FIFO fissati su un core (di default CPU0, dove arrivano comunque tutti gli
 * interrupt), FUORI dallo scheduling dei task (docs/PIANO.md, "Architettura RT"): il thread del TEMPO a priorità 99
 * (RT_PRIO_DRIVER), che dorme con clock_nanosleep e fa le cose periodiche, e quello di RICEZIONE a 98, che aspetta i
 * byte delle seriali e i comandi dei task (perché due: docs/fase3.md, prove r4-r6). Fa da solo tutto quello che serve
 * a tenere vivo e sicuro il robot:
 *   - all'avvio: stato sicuro (frame di arresto di tutti i motori), poi la sequenza di avvio di everest
 *     (0x0c 01, 0x07, 0x17, 0x18, 0x71 0, 0x65 00, 0x19 01, 0x7c 00 ff ff, 0x78 01, 0x08, 0x65 02);
 *   - keepalive 0x0d ogni 3 s (senza, il GD32 resetta il robot dopo ~190 s) e heartbeat 0x06 ogni 200 ms (senza, il
 *     GD32 toglie potenza ai motori dopo ~1,9 s); velocità 0x66 ogni 20 ms;
 *   - sensori: decodifica il frame 0x15 (50 Hz), lo pubblica e sveglia i task iscritti (drv_sveglia_ai_sensori);
 *   - lidar: alimentazione, regolazione della velocità (come AuxCtrl), giri completi pubblicati;
 *   - contatti: registro degli scatti di anti-caduta e bumper, e degli incastri, per i task lenti (contatti.h,
 *     drv_contatti);
 *   - batteria: tensione media, percentuale, soglie (batteria.h); alimentazione dei sensori secondo la base (0x9d);
 *   - sicurezza: ogni comando ai motori passa dai limiti "a terra" (sic_terra_limita in gd32/sicurezza.h), oppure
 *     dal blocco del banco di prova se il robot è sollevato apposta (drv_config_t.banco);
 *   - watchdog dei task iscritti (drv_sorveglia): se un task critico si blocca o manca 3 scadenze di fila, il driver
 *     ferma tutti i motori e va nello STATO DI GUASTO: LED 8 (rosso, lampeggio veloce) e un suono.
 *
 * I task NON parlano con le seriali: leggono i dati pubblicati dal driver e gli danno comandi, sempre senza mutex
 * (driver/scambio.h): il driver non aspetta mai un task.
 *
 * Esempio (la guida completa è in docs/GUIDA.md, "Scrivere un task"):
 *
 *     drv_config_t c;
 *     drv_config_default(&c);                    // /dev/ttyS3, /dev/ttyS1, CPU0, priorità 99
 *     if (drv_avvia(&c)) return 1;
 *     ...
 *     drv_sensori_t s;
 *     drv_sensori(&s);                           // ultimo frame dei sensori
 *     drv_velocita(0.10, 0.0);                   // 10 cm/s in avanti
 *     ...
 *     drv_ferma();                               // motori a zero, lidar spento
 *
 * Il driver è uno solo per processo (c'è un solo robot): le funzioni non hanno un parametro "driver".
 */
#ifndef DRIVER_H
#define DRIVER_H

#include <stddef.h>
#include <stdint.h>

#include "../gd32/gd32.h"
#include "../rt/rt.h"
#include "aggancio.h"
#include "batteria.h"
#include "contatti.h"
#include "lidar_giro.h"

/* ---------------------------------------------------------------------------------------- configurazione --- */

typedef struct {
	const char *dev_gd32;        /* seriale del GD32 (default /dev/ttyS3; nel simulatore build/sim/gd32) */
	const char *dev_lidar;       /* seriale del lidar (default /dev/ttyS1); NULL = niente lidar */
	int cpu;                     /* core del driver (default 0) */
	int priorita;                /* default RT_PRIO_DRIVER (99) */
	int motori;                  /* 0 = rifiuta TUTTI i comandi ai motori: ruote, spazzole, aspirazione (default 1) */
	int aspirazione;             /* 1 = aspirazione permessa (default 1; 0 con drv_config_prova) */
	int banco;                   /* 1 = banco di prova: motori solo se il robot è SOLLEVATO (gd32/sicurezza.h) */
	int cliff_atteso;            /* solo banco: anti-caduta che valgono "sollevato" (-1 = 0x0f, la regola normale) */
	double lidar_giri_s;         /* velocità del lidar quando è acceso (default 6, come AuxCtrl) */
	int scarto_lidar_cdeg;       /* scarto fra lo 0 del lidar e il robot da togliere (default LIDAR_SCARTO_CDEG) */
	const char *cmd_suono;       /* comando per i suoni, %s = nome (es. "sound_beep_beep"); NULL = nessun suono */
	const char *registra;        /* file dove registrare i byte grezzi delle seriali, nel formato della spia
	                                (spia/registro.h: canali "gd32" e "lidar"); NULL = niente (default) */
	double prova_rientro_s;      /* SOLO PROVE (env RTES_PROVA_RIENTRO_DOPO_S): dopo questi secondi i task leggono una batteria
	                                bassa, così rientrano alla base senza finire la missione. Vale solo per drv_stato():
	                                le protezioni del driver usano sempre la tensione vera. 0 = mai (default) */
	int senza_rt;                /* 1 = se mancano i permessi per SCHED_FIFO parte lo stesso (solo per il PC) */
	int forza;                   /* 1 = non controlla se un altro programma usa già le seriali */
} drv_config_t;

/*
 * Valori di default. Le seriali sono /dev/ttyS3 e /dev/ttyS1, oppure quelle nelle variabili d'ambiente RTES_GD32 e
 * RTES_LIDAR, che ./rtes.sh sim imposta sulle pseudo-seriali del simulatore: così lo STESSO programma gira sul PC
 * (simulatore) e sul robot senza cambiare una riga. Il suono usa tinyplayer se c'è (sul robot), altrimenti niente.
 */
void drv_config_default(drv_config_t *c);

/* Come drv_config_default, ma con l'aspirazione vietata: per i programmi di prova, che non devono accendere la ventola
 * se non lo chiede chi li lancia (rtes-driver: --aspirazione). */
void drv_config_prova(drv_config_t *c);

/* Apre le seriali e avvia il thread del driver. Ritorna 0 oppure un codice errno (con un messaggio su stderr). */
int drv_avvia(const drv_config_t *c);

/* Ferma il driver: motori a zero, lidar spento, LED azzurro. Dopo, nessuno manda più il keepalive al GD32! */
void drv_ferma(void);

/* ----------------------------------------------------------------------------------------- dati pubblicati --- */

typedef struct {
	uint32_t numero;             /* progressivo del frame (1, 2, 3...) */
	int64_t t_ns;                /* quando il driver l'ha ricevuto (CLOCK_MONOTONIC, come rt_adesso()) */
	gd32_sensori_t s;            /* tutti i campi decodificati: vedi gd32.h */
} drv_sensori_t;

enum { DRV_AVVIO = 0, DRV_PRONTO, DRV_GUASTO, DRV_FINE };

/* Motivi per cui il comando di velocità è stato ridotto (oltre ai SIC_LIM_* di gd32/sicurezza.h). */
enum {
	DRV_LIM_DISABILITATI = 1 << 8,   /* drv_config_t.motori = 0 */
	DRV_LIM_NON_PRONTO   = 1 << 9,   /* sequenza di avvio in corso */
	DRV_LIM_GUASTO       = 1 << 10,  /* stato di guasto */
	DRV_LIM_SENSORI_VECCHI = 1 << 11,/* nessun frame dei sensori da più di 100 ms */
	DRV_LIM_BATTERIA     = 1 << 12,  /* batteria critica */
	DRV_LIM_BANCO        = 1 << 13,  /* banco di prova: il robot non risulta sollevato, o il blocco è scattato */
	DRV_LIM_CASSETTO     = 1 << 14,  /* solo aspirazione: manca il cassetto della polvere (il GD32 non la ferma) */
	DRV_LIM_VIETATA      = 1 << 15,  /* solo aspirazione: drv_config_t.aspirazione = 0 */
};

typedef struct {
	int64_t t_ns;
	int fase;                    /* DRV_AVVIO, DRV_PRONTO, DRV_GUASTO, DRV_FINE */
	char guasto[96];             /* perché è in guasto ("" se non lo è) */
	int task_fault;              /* drv_task_fault_t: why a task asked for the fault (DRV_FAULT_NONE if no task did) */
	char versione_gd32[33];

	/* batteria */
	double volt;                 /* media dell'ultimo secondo */
	int percentuale;             /* stima come il software originale (batteria.h) */
	int batteria;                /* BAT_OK, BAT_BASSA, BAT_CRITICA */
	int in_carica, sulla_base, temperatura_c;
	int sollevato;               /* 1 da quando sotto tutti e 4 gli anti-caduta c'è il vuoto a quando rivedono il pavimento per 0,5 s */

	/* tasti: pressioni brevi (meno di DRV_TASTO_BREVE_MS), contate al rilascio. Un task confronta il numero con quello
	 * dell'ultima lettura: se è cresciuto c'è stata una pressione. Una pressione azzera anche il guasto, prima di essere
	 * contata: chi vede il numero crescere trova già il guasto azzerato (se la causa non c'è più). Le pressioni lunghe
	 * non contano */
	uint32_t pressioni_accensione, pressioni_casa;

	/* motori: quello che il driver sta mandando DAVVERO al GD32 (dopo i limiti) */
	double v, w;                 /* m/s, rad/s */
	int limiti;                  /* SIC_LIM_* | DRV_LIM_*: perché v, w sono diversi dal comando (0 = nessun limite) */
	int spazzola_laterale, spazzola_centrale, aspirazione_giri, led;
	int aspirazione_limiti;      /* perché l'aspirazione chiesta non va (SIC_LIM_* | DRV_LIM_*; 0 = va, o non chiesta) */
	int aggancio;                /* AGG_FERMO, AGG_IN_CORSO, AGG_FATTO, AGG_FALLITO (aggancio.h, drv_aggancia) */
	int aggancio_fase, aggancio_tentativo;

	/* lidar */
	int lidar_acceso, lidar_potenza, lidar_ok;
	double lidar_giri_s;
	uint32_t lidar_giri;

	/* salute del collegamento e tempi del driver */
	uint64_t frame_sensori, frame_errati, byte_scartati, frame_gd32_altri, heartbeat_gd32;
	uint64_t keepalive, heartbeat, errori_scrittura;
	double sensori_hz;           /* frame dei sensori nell'ultimo secondo */
	int64_t eta_sensori_ns;      /* età dell'ultimo frame dei sensori */
	int64_t tick_ritardo_max_ns; /* ritardo massimo del ciclo da 20 ms rispetto all'istante previsto */
	int64_t intervallo_sensori_max_ns; /* intervallo massimo fra due frame dei sensori (il GD32 li manda ogni 20 ms) */
	int64_t intervallo_sensori_min_ns; /* intervallo MINIMO: è il T da usare nell'analisi dei task sporadici */
	int64_t lavoro_max_ns;       /* durata massima di un risveglio del driver (lettura + elaborazione + invio) */
	uint64_t risvegli;
	int senza_rt;                /* il driver gira senza SCHED_FIFO (PC) */
} drv_stato_t;

/*
 * Copiano in out l'ultima versione pubblicata e ne ritornano il numero (0 = niente ancora). Non si bloccano mai.
 * Chi legge due volte lo stesso numero sa che non è arrivato niente di nuovo.
 */
uint32_t drv_sensori(drv_sensori_t *out);
uint32_t drv_giro(lidar_giro_t *out);       /* ultimo giro completo del lidar (struttura grande: ~4 KB) */
uint32_t drv_stato(drv_stato_t *out);
/* Registro dei contatti (driver/contatti.h): gli ultimi CONTATTI_N scatti di anti-caduta, bumper e incastri, ciascuno con il
 * suo numero progressivo, il sensore e l'istante. Il numero di versione cambia solo quando arriva un evento nuovo. */
uint32_t drv_contatti(drv_contatti_t *out);

/* Punto del lidar nel sistema del robot: origine al CENTRO del robot, x in avanti, y a sinistra, in metri. Tiene conto
 * di verso degli angoli, scarto (già tolto in p->angolo_cdeg), posizione del lidar e scostamento ottico
 * (driver/lidar_giro.h). */
void drv_punto_xy(const lidar_punto_t *p, double *x, double *y);

/* --------------------------------------------------------------------------------------------- comandi --- */
/* Non si bloccano mai. Il driver applica il comando entro il suo prossimo risveglio (di solito < 1 ms). */

void drv_velocita(double v_m_s, double w_rad_s);    /* v in [-0,5, 0,5] m/s, omega in [-2pi, 2pi] rad/s */
void drv_spazzole(int laterale, int centrale);      /* -100..100; centrale < 0 = sblocco per ~4 s (lo ferma il GD32) */
void drv_aspirazione(int giri_min);                 /* 0..30000; rifiutata se drv_config_t.aspirazione = 0 */
void drv_led(int modo);                             /* 0x8d: 0 spento, 1 azzurro... (tabella in docs/RIEPILOGO.md) */
/* Modi del LED che possono servire ai task (gli altri in docs/fase2.md). Guasto, batteria e carica li decide il driver e
 * vincono su quello chiesto dai task */
#define DRV_LED_AZZURRO          1   /* azzurro fisso: acceso, in riposo o al lavoro */
#define DRV_LED_AZZURRO_RESPIRA  12  /* azzurro che respira: in pausa */
/* Pressione breve di un tasto: rilasciato prima di questi millisecondi */
#define DRV_TASTO_BREVE_MS       1000
void drv_lidar(int acceso, double giri_s);          /* giri_s <= 0: la velocità della configurazione */
/* La posizione del robot (metri e radianti nel sistema della mappa), dal task del moto a ogni frame quando è valida:
 * il driver la attacca agli eventi del registro dei contatti (contatti.h). La deve chiamare UN SOLO task. */
void drv_posa(double x, double y, double theta);
void drv_esci_dalla_base(int si);                   /* 1 = permette di muoversi piano anche sulla base (i contatti
                                                       sono dietro: si esce IN AVANTI) */
/* Incastro in avanti (lo decide il task di navigazione, quando parte lo sblocco): 1 finché va segnato, 0 altrimenti.
 * Va chiamata a ogni frame, come un sensore: il driver ne fa un evento CONTATTO_INCASTRO nel registro dei contatti,
 * con la posizione del primo frame. La deve chiamare UN SOLO task. */
void drv_incastro(int si);
/* Messa in carica (driver/aggancio.h): con il robot in (AGG_PUNTO_X_M, 0) e angolo 0, cioè sull'asse della base e di
 * spalle, 1 fa partire la manovra e 0 la annulla. Da quel momento il driver ignora drv_velocita e i task leggono
 * l'esito in drv_stato_t.aggancio. Prima di chiamarla: drv_esci_dalla_base(0), e spazzole e aspirazione spente. */
void drv_aggancia(int si);
void drv_suono(const char *nome);                   /* es. "sound_beep_beep" (file in /usr/share/audio/english) */

/* Why a task asks for the fault state. To add a reason: add a value here and its case in drv_task_fault_text
 * (driver.c); if the case is missing the build stops with an error. */
typedef enum {
	DRV_FAULT_NONE = 0,                  /* no fault asked by a task */
	DRV_FAULT_UNLOCK_ATTEMPTS_EXHAUSTED, /* navigation: all the unlock attempts failed */
	DRV_FAULT_NO_ESCAPE_CELL,            /* navigation: no free cell reachable to escape to */
	DRV_FAULT_LEAVE_BASE_TIMEOUT,        /* navigation: the robot could not leave the charging base in time */
	DRV_FAULT_NO_PATH_HOME,              /* navigation: no path back to the charging base */
	DRV_FAULT_DOCKING_FAILED,            /* navigation: the driver reported a failed docking */
	DRV_FAULT_DOCKING_TIMEOUT,           /* navigation: docking took too long */
	DRV_FAULT_INCONSISTENT_STATE,        /* navigation: inconsistent state machine (a bug) */
	DRV_FAULT_UNKNOWN_OBJECTIVE,         /* movement: unknown objective type (a bug) */
	DRV_FAULT_POSITION_NOT_FOUND,        /* mapping: position not found again after the robot was lifted */
} drv_task_fault_t;

/* A task puts the driver in the fault state: motors stopped, LED 8 and sound, like the driver's own faults. It happens
 * at the next driver tick (within 20 ms); the reason goes to the log, to drv_stato_t.guasto (text) and to
 * drv_stato_t.task_fault (code). The only way out is drv_azzera_guasto. */
void drv_segnala_guasto(drv_task_fault_t reason);

/* Readable text of a reason, for logs and app. Never NULL. */
const char *drv_task_fault_text(drv_task_fault_t reason);

/* Prova a uscire dallo stato di guasto (solo se la causa non c'è più). Ritorna 0 se ci riesce. ATTENZIONE: aspetta fino a 2
 * tick (50 ms), quindi non va chiamata da un task real-time. Il guasto si azzera anche da solo, sempre se la causa non c'è
 * più: con una pressione breve di un tasto e quando il robot viene rimesso sulla base dopo almeno 1 s fuori. */
int drv_azzera_guasto(void);

/* ------------------------------------------------------------------------------------------------ task --- */

/* Il task sporadico t (cfg.sporadico = 1) riceve una sveglia a ogni frame dei sensori (50 Hz). Max 8 task. */
int drv_sveglia_ai_sensori(rt_task_t *t);

/*
 * Il driver sorveglia il task t: se sembra bloccato da più di soglia_ns (rt_task_bloccato; 0 = 3 periodi) o manca 3
 * scadenze di fila e critico = 1, il driver va nello stato di guasto. Per i task non critici scrive solo un evento.
 * Sul robot (senza_rt = 0) basta invece UNA scadenza mancata, di qualsiasi task, critico o no: guasto, e un evento con
 * job, risposta, attesa, esecuzione e tempo di CPU. Nel simulatore le scadenze mancate valgono solo con la regola di fila.
 */
int drv_sorveglia(rt_task_t *t, int critico, int64_t soglia_ns);

/* Toglie il task da sveglie e sorveglianza: va chiamata PRIMA di rt_task_ferma(t). */
void drv_dimentica(rt_task_t *t);

#endif /* DRIVER_H */
