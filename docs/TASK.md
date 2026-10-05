# Scrivere i task: guida pratica e riferimento del driver

Questa guida spiega dove si scrivono i task, come si avviano e come si usano sensori e attuatori. L'esempio completo
è `tasks/primo.c`. La teoria (modello dei task, Rate Monotonic, permessi, perché `mlockall`) è in `GUIDA.md`,
sezioni 4–6 e 13.

## 1. Come è fatto un programma

Un **programma** è un eseguibile. Dentro un programma girano:

```
 un processo (es. build/arm/primo)
 ├─ il driver: 2 thread su CPU0, priorità 99 e 98      parla con GD32 e lidar, sicurezza, batteria, watchdog
 ├─ i tuoi task: un thread SCHED_FIFO ciascuno          su CPU1, CPU2, CPU3 (un task sta sempre sullo stesso core)
 └─ il main: un thread normale, non real-time           avvia e ferma tutto; qui si può stampare
```

- Il **driver** non si scrive: è già pronto (`driver/`). Si avvia con una chiamata e da lì tiene vivo il robot da solo.
- Un **task** è fatto di tre cose:
  1. una funzione **job**, cioè il lavoro di un'attivazione (`void job(void *arg)`);
  2. una **configurazione** (`rt_task_cfg_t`): nome, core, priorità, periodo, se è periodico o sporadico;
  3. una variabile **`rt_task_t`**: il thread, il suo stato e le statistiche.

  Il framework (`rt/`) crea il thread e chiama il job al momento giusto: ogni periodo (task **periodico**) oppure a
  ogni sveglia (task **sporadico**: per esempio a ogni frame dei sensori, 50 volte al secondo).
- I task si creano e si avviano nel **`main`** del programma.
- Sul robot gira **un programma alla volta**: le seriali del GD32 e del lidar sono una. Il programma "vero" della
  fase 4 conterrà tutti i task insieme (moto, sicurezza, lidar, navigazione, mappa).

## 2. Dove si scrivono i file

Due modi, a scelta. In tutti e due non serve toccare il Makefile.

**Un programma in un file solo**: `tasks/<nome>.c`. Basta il `.c`, senza `.h`. Va bene per le prove e i programmi
piccoli (es. `tasks/primo.c`).

**Un programma in più file**: la cartella `tasks/<nome>/`, con un `main.c` obbligatorio e gli altri file che vuoi.
Conviene quando i task sono tanti: un `.c` e un `.h` per ogni task. Per esempio:

```
tasks/pulizia/
  main.c      avvia il driver e i task, aspetta Ctrl-C, ferma tutto
  moto.c      il job del controllo del moto e il suo stato (static)
  moto.h      quello che gli altri vedono: void moto_job(void *arg); e i dati che pubblica
  mappa.c
  mappa.h
  comune.h    strutture condivise fra più task
```

Si compilano tutti i `.c` della cartella. Nel `main.c` si scrive `#include "moto.h"`.

**Compilare**: `make build/host/<nome>` per il PC (simulatore) e `make build/arm/<nome>` per il robot. Driver, `rt/`
e protocolli sono già collegati.

**Include**: basta `#include "driver/driver.h"`, che porta con sé anche `rt/rt.h` (task), `gd32/gd32.h` (sensori),
`driver/lidar_giro.h` (lidar) e `driver/batteria.h`. Il file `compile_flags.txt` nella radice serve all'editor
(clangd) per trovare questi include.

## 3. Il primo task, pezzo per pezzo (`tasks/primo.c`)

Il programma fa andare avanti il robot piano. Se tocca qualcosa o vede un vuoto torna indietro, gira di 90° e riparte.
C'è un solo task, `guida`, **sporadico**: il driver lo sveglia a ogni frame dei sensori.

Qui sotto ci sono degli estratti; il file completo, che si compila così com'è, è `tasks/primo.c`.

### 3.1 Il job

```c
static rt_task_t guida;                 /* il task (globale: lo usano main e driver) */

enum { AVANTI, INDIETRO, GIRA };
static int fase = AVANTI;               /* stato del comportamento: lo usa solo questo job, niente mutex */
static int frame_in_fase;

static void job_guida(void *arg)
{
	(void)arg;                          /* non usato */
	drv_sensori_t s;
	if (drv_sensori(&s) == 0)
		return;                         /* nessun frame ancora */

	int urto = s.s.bumper_sx || s.s.bumper_dx;
	int vuoto = s.s.cliff[0] || s.s.cliff[1] || s.s.cliff[2] || s.s.cliff[3];
	int ostacolo = !s.s.sulla_base && (urto || vuoto);

	frame_in_fase++;
	switch (fase) {
	case AVANTI:
		if (ostacolo)
			cambia_fase(INDIETRO, &s);
		else
			drv_velocita(0.10, 0);      /* 10 cm/s in avanti */
		break;
	...
	}
}
```

- `drv_sensori(&s)` copia l'ultimo frame dei sensori. Non si blocca mai e non serve un mutex: il driver pubblica i
  dati in un doppio buffer.
- `drv_velocita(v, w)` manda il comando: v in m/s, w in rad/s (positivo = gira a sinistra). Il driver lo applica
  **dopo i suoi limiti di sicurezza**: per esempio con il bumper premuto non lascia andare avanti, anche se il task
  sbaglia (sezione 7).
- Lo stato del comportamento (`fase`, `frame_in_fase`) è `static` e lo usa solo questo job, quindi non serve
  proteggerlo.
- Nel job **niente** `printf`, `malloc`, file o `sleep`: possono bloccarsi per millisecondi (sezione 9).

Per girare di 90° il job guarda l'imbardata dell'IMU (`s.s.angolo[0]`, in centesimi di grado) e si ferma 6° prima:
il GD32 non si ferma di colpo (a terra risponde con una costante di tempo di ~73 ms in rotazione, più ~29 ms di
ritardo), quindi dopo lo stop il robot gira ancora un po'. Nel simulatore succede lo stesso, perché la risposta è
quella misurata sul robot (fase 3, prova val1). Nota: l'IMU segna il ~3 % in meno della rotazione vera (lidar).

### 3.2 Il main: avvio

```c
int main(void)
{
	/* 1. il driver */
	drv_config_t c;
	drv_config_default(&c);                    /* robot: /dev/ttyS3 e /dev/ttyS1; simulatore: le sue pseudo-seriali */
	if (drv_avvia(&c))
		return 1;
	rt_consenti_senza_privilegi(c.senza_rt);   /* sul PC i task partono anche senza permessi per SCHED_FIFO */

	/* 2. il task */
	rt_task_cfg_t cfg = {
		.nome = "guida",
		.cpu = 1,                              /* CPU0 è del driver: i task su CPU1-3 */
		.priorita = RT_PRIO_TASK_MAX,          /* 90; con più task le assegna rt_priorita_rm (sezione 5) */
		.sporadico = 1,                        /* un job per ogni sveglia */
		.periodo_ns = RT_MS(15),               /* intervallo MINIMO fra due sveglie: il T dell'analisi */
		.job = job_guida,
	};
	if (rt_task_avvia(&guida, &cfg, rt_adesso())) {
		drv_ferma();
		return 1;
	}
	drv_sveglia_ai_sensori(&guida);            /* il driver lo sveglia a ogni frame dei sensori */
	drv_sorveglia(&guida, 1, RT_MS(100));      /* critico: se si blocca 100 ms, motori fermi e stato di guasto */
```

- `drv_config_default` sceglie da solo le seriali: lo **stesso eseguibile** gira nel simulatore e sul robot.
- Perché `periodo_ns = 15 ms` in un task sporadico: il GD32 manda un frame ogni 20 ms, ma i frame arrivano fra ~15 e
  ~25 ms l'uno dall'altro (ritardi del kernel). Per l'analisi conta l'intervallo **minimo**. Il driver lo misura in
  `drv_stato().intervallo_sensori_min_ns`.
- `drv_sorveglia(t, critico, soglia)`: se il task si blocca per più di `soglia`, o manca 3 scadenze di fila, il driver
  ferma i motori e va nello stato di guasto (LED rosso veloce e un suono). Con `critico = 0` scrive solo un avviso.

### 3.3 Il main: attesa e uscita

```c
	while (!fine) {                            /* fine = 1 quando arriva Ctrl-C (SIGINT) o SIGTERM */
		sleep(1);
		drv_stato_t st;
		drv_stato(&st);
		printf("%.2f V %d %%  v %+.2f  w %+.2f  limiti 0x%x\n", st.volt, st.percentuale, st.v, st.w, st.limiti);
	}
	drv_dimentica(&guida);                     /* 1. il driver smette di svegliarlo e sorvegliarlo */
	rt_task_ferma(&guida);                     /* 2. il task finisce il job in corso e si ferma */
	drv_ferma();                               /* 3. motori a zero, lidar spento */
	rt_stats_stampa(&guida);                   /* job, scadenze mancate, latenza ed esecuzione min/media/max */
	return 0;
}
```

L'ordine dell'uscita conta: se si ferma prima il task, il driver lo vedrebbe "bloccato" e andrebbe in guasto.

## 4. Provarlo

**Nel simulatore** (sul PC):

```
make build/host/primo
./rtes.sh sim sim/stanze/soggiorno.txt build/host/primo
```

- Apri http://localhost:8080:
  - nella stanza il robot va verso la base, tocca, torna indietro, gira e riparte;
  - nella scheda "Messaggi" si vedono i comandi `0x66` che il driver manda al GD32.
- Nel terminale c'è una riga al secondo. Ctrl-C ferma tutto e stampa le statistiche del task.
- Puoi spostare il robot o aggiungere ostacoli dalla pagina mentre il programma gira.
- I tempi (latenza, esecuzione) sul PC non valgono per l'analisi: i WCET si misurano sul robot.

**Sul robot** (con il robot a terra in uno spazio libero, perché si muove):

```
tools/driver-robot.sh programma primo p1     # compila per ARM, copia in /tmp/rtes e avvia (uscita in p1.out)
tools/driver-robot.sh stato                  # ultime righe dell'uscita
tools/driver-robot.sh ferma                  # SIGTERM: si ferma in ordine; poi riparte il custode
tools/driver-robot.sh scarica p1             # copia l'uscita in build/robot/fase3/p1/
```

## 5. Aggiungere un secondo task (periodico) e condividere dati

Un task **lidar** periodico ogni 200 ms (CPU2) calcola la distanza dell'ostacolo più vicino davanti. Il task `guida`
la legge e cambia direzione prima di toccare. Il dato condiviso è protetto da un **mutex con priority inheritance**
(`rt_mutex_init`). Questa variante l'ho provata nel simulatore: il robot gira prima di toccare la base e il muretto.

```c
#include <math.h>
#include <pthread.h>

static rt_task_t lidar;
static pthread_mutex_t mx_distanza;
static double distanza_davanti = 99;     /* m, dal centro del robot; protetta da mx_distanza */

static void job_lidar(void *arg)         /* periodico, 200 ms */
{
	(void)arg;
	static lidar_giro_t g;               /* ~4 KB: static, non sullo stack del task */
	static uint32_t ultimo;
	uint32_t n = drv_giro(&g);           /* ultimo giro completo del lidar */
	if (n == 0 || n == ultimo)
		return;                          /* nessun giro nuovo */
	ultimo = n;
	double minima = 99;
	for (int i = 0; i < g.n; i++) {
		if (g.p[i].distanza_mm == 0)
			continue;                    /* nessuna eco */
		double x, y;
		drv_punto_xy(&g.p[i], &x, &y);   /* metri: x in avanti, y a sinistra */
		if (x > 0 && fabs(y) < 0.18 && x < minima)   /* nel corridoio largo quanto il robot */
			minima = x;
	}
	pthread_mutex_lock(&mx_distanza);
	distanza_davanti = minima;
	pthread_mutex_unlock(&mx_distanza);
}
```

Nel job di `guida`:

```c
	pthread_mutex_lock(&mx_distanza);
	double davanti = distanza_davanti;
	pthread_mutex_unlock(&mx_distanza);
	int ostacolo = !s.s.sulla_base && (urto || vuoto || davanti < 0.40);
```

Nel `main` (al posto dell'avvio del solo `guida`; da `cfg` di guida si toglie `.priorita`):

```c
	rt_mutex_init(&mx_distanza);
	drv_lidar(1, 0);                                   /* lidar acceso alla velocità di default (6 giri/s) */

	rt_task_cfg_t cfg_l = { .nome = "lidar", .cpu = 2, .periodo_ns = RT_MS(200), .job = job_lidar };
	rt_task_cfg_t *tutti[] = { &cfg, &cfg_l };
	rt_priorita_rm(tutti, 2, RT_PRIO_TASK_MAX);        /* Rate Monotonic: guida (15 ms) 90, lidar (200 ms) 89 */

	int64_t t0 = rt_adesso() + RT_MS(100);             /* stesso istante zero per tutti */
	if (rt_task_avvia(&guida, &cfg, t0) || rt_task_avvia(&lidar, &cfg_l, t0)) {
		drv_ferma();
		return 1;
	}
	drv_sveglia_ai_sensori(&guida);
	drv_sorveglia(&guida, 1, RT_MS(100));
	drv_sorveglia(&lidar, 0, 0);                       /* non critico: solo un avviso */
	...
	drv_dimentica(&guida); drv_dimentica(&lidar);
	rt_task_ferma(&guida); rt_task_ferma(&lidar);
	drv_ferma();
	rt_stats_stampa(&guida); rt_stats_stampa(&lidar);
```

## 6. Riferimento: cosa si legge

### 6.1 Sensori del GD32: `drv_sensori(&s)`

`drv_sensori_t s` contiene `s.numero` (progressivo del frame: se è uguale alla volta prima non è arrivato niente di
nuovo), `s.t_ns` (quando è arrivato, nello stesso orologio di `rt_adesso()`) e i campi in `s.s`:

| Campo (`s.s.`) | Unità | Significato |
|---|---|---|
| `bumper_sx`, `bumper_dx` | 0/1 | bumper sinistro e destro premuti |
| `cliff[0..3]` | 0/1 | anti-caduta, 1 = vuoto. Da sinistra a destra: 0 sinistro laterale, 1 sinistro anteriore, 2 destro anteriore, 3 destro laterale. Sulla base leggono tutti 1 |
| `cassetto` | 0/1 | cassetto della polvere presente |
| `ir_base_a`, `ir_base_b` | bit | due ricevitori IR davanti: ogni bit (1, 2, 4, 8) è un fascio della base visto. Si accendono solo guardando la base |
| `in_carica` | 0/1 | in carica |
| `sulla_base` | 0/1 | sui contatti della base (o in carica). I contatti sono dietro |
| `tensione_dv` | 0,1 V | tensione della batteria (meglio `drv_stato().volt`, già mediata) |
| `temperatura_c` | °C | temperatura della batteria |
| `encoder_sx`, `encoder_dx` | tick | contatori assoluti delle ruote: **~4580 tick per metro** (`GD32_SCALA_V`, misura del 5/10; i 3616 tick per giro di everest non tornano); in avanti aumentano. Carreggiata 241,5 mm |
| `vel_sx` | 1/`GD32_SCALA_V` m/s | **v misurata** dal GD32 sugli encoder in ~70 ms (nome storico: non è la ruota sinistra) |
| `vel_dx` | 1/`GD32_SCALA_W` rad/s | **ω misurata** (nome storico: non è la ruota destra) |
| `angolo[0]` | 0,01° | **imbardata** integrata dal GD32, positiva in senso antiorario; parte da 0 all'avvio del driver, deriva di ~0,3°/h |
| `angolo[1]`, `angolo[2]` | 0,01° | beccheggio (+ = naso in giù), rollio (+ = fianco sinistro in giù) |
| `vel_angolare[0..2]` | 0,01 °/s | velocità angolari: imbardata, beccheggio, rollio |
| `gravita[0..2]` | 0,01 g | direzione "su": x indietro, y destra, z su (z = 100 a robot piano) |
| `tasto_accensione_ms`, `tasto_casa_ms` | ms | da quanto è premuto il tasto (0 = non premuto) |
| `controllo` | bit | bit 5 = sensori alimentati (spenti sulla base: bumper e anti-caduta non valgono), bit 7 = lidar alimentato |
| `led` | | modo del LED in vigore |
| `byte67`, `byte68` | | eco dei comandi delle spazzole laterale e centrale |
| `byte69_x100` | giri/min | giri della ventola dell'aspirazione |
| `correnti[0..4]` | mA | ruota sinistra, ruota destra, spazzola laterale, spazzola centrale (~58 a riposo), aspirazione (~14 a riposo) |

La spiegazione di ogni byte, con il suo stato di verifica, è anche nel simulatore: scheda "Messaggi", passando il
mouse sulla griglia dei 96 byte. I campi senza significato noto sono nella stessa struttura (`gd32/gd32.h`).

### 6.2 Lidar: `drv_giro(&g)`

Il lidar va acceso con `drv_lidar(1, 0)`. Fa ~6 giri al secondo; il driver consegna ogni giro completo.

| Campo | Significato |
|---|---|
| `g.numero` | progressivo del giro (uguale alla volta prima = niente di nuovo) |
| `g.n`, `g.p[0..n-1]` | punti del giro (~336 a 6 giri/s) |
| `g.p[i].distanza_mm` | distanza in mm; **0 = nessuna eco** (da saltare) |
| `g.p[i].angolo_cdeg` | angolo **del lidar** in centesimi di grado (0 = dietro il robot): meglio usare `drv_punto_xy` |
| `g.p[i].intensita` | intensità dell'eco |
| `g.giri_s`, `g.t_inizio_ns`, `g.t_fine_ns` | velocità e istanti del giro |

`drv_punto_xy(&g.p[i], &x, &y)` dà il punto in metri nel sistema del robot: origine al **centro del robot**, x in
avanti, y a sinistra. Tiene già conto di tutto quello che si è misurato il 5/10: gli angoli del lidar crescono in senso
**orario**, lo 0 è spostato di 14,5°, il lidar sta 11,2 cm dietro il centro e misura con uno scostamento ottico di 2,5
cm (`driver/lidar_giro.h`). `lidar_giro_t` è grande (~4 KB): dichiaralo `static`, non sullo stack di un task.

### 6.3 Stato del driver: `drv_stato(&st)`

| Campo | Significato |
|---|---|
| `fase` | `DRV_AVVIO`, `DRV_PRONTO`, `DRV_GUASTO`, `DRV_FINE`; in guasto `guasto` dice perché |
| `volt`, `percentuale` | tensione media dell'ultimo secondo; percentuale calcolata come il software originale |
| `batteria` | `BAT_OK`, `BAT_BASSA` (sotto 14,2 V), `BAT_CRITICA` (sotto 13,8 V: motori fermi) |
| `in_carica`, `sulla_base`, `temperatura_c` | come nei sensori |
| `v`, `w` | velocità che il driver sta **davvero** mandando, dopo i limiti |
| `limiti` | perché il comando è stato ridotto: bit `SIC_LIM_*` e `DRV_LIM_*` (sezione 7); 0 = nessun limite |
| `spazzola_laterale`, `spazzola_centrale`, `aspirazione_giri`, `led` | comandi in vigore |
| `lidar_acceso`, `lidar_giri_s`, `lidar_giri` | stato del lidar |
| `intervallo_sensori_min_ns` | intervallo minimo misurato fra due frame: il T dei task sporadici |
| `sensori_hz`, `eta_sensori_ns` | salute del collegamento con il GD32 |

## 7. Riferimento: cosa si comanda

Tutti i comandi tornano subito (non si bloccano). Il driver li applica entro il suo prossimo risveglio (di solito
meno di 1 ms) e il GD32 ~29 ms dopo.

| Funzione | Argomenti | Note |
|---|---|---|
| `drv_velocita(v, w)` | v in m/s (±0,5), w in rad/s (±2π, + = sinistra) | a terra il GD32 risponde con una costante di tempo di ~110 ms (v) e ~73 ms (ω), dopo ~29 ms; **non va oltre ~0,39 m/s** |
| `drv_spazzole(lat, cen)` | -100..100 | la laterale non è montata sul nostro robot; centrale < 0 = sblocco (~4 s alla massima velocità all'indietro, poi la ferma il GD32) |
| `drv_aspirazione(giri)` | 0..30000 giri/min | serve `c.aspirazione = 1` prima di `drv_avvia`; il driver la ferma senza cassetto della polvere, con il robot sollevato o sulla base (`st.aspirazione_limiti` dice perché). Livelli dell'originale: eco 12 800, normale 15 500, massimo 19 500 |
| `drv_led(modo)` | 0..18 | tabella sotto |
| `drv_lidar(acceso, giri_s)` | 0/1, giri al secondo (0 = default 6) | il driver regola da solo la velocità |
| `drv_esci_dalla_base(si)` | 0/1 | sulla base i motori sono fermi; con 1 il robot può muoversi piano (0,1 m/s, 0,5 rad/s) per uscire, in avanti |
| `drv_suono(nome)` | nome del file senza `.mp3` | sul robot; nel simulatore niente. Es. `sound_beep_beep`, `sound_test`, `sound_auto_clean_start`, `sound_auto_clean_finish`, `sound_start_go_charge`, `sound_erro_bumper_sensor` (elenco: `/usr/share/audio/english` sul robot) |
| `drv_azzera_guasto()` | | esce dallo stato di guasto se la causa non c'è più; 0 = riuscito |

**Modi del LED** (`drv_led`), verificati sul robot:

| modo | aspetto | modo | aspetto |
|---|---|---|---|
| 0 | spento | 7 | rosso fisso |
| 1 | azzurro fisso | 8 | rosso, lampeggio veloce (lo usa il driver per il guasto) |
| 3 | giallo fisso | 10 | rosso e giallo, lampeggio lento |
| 4 | giallo che respira (lo usa il driver in carica) | 11 | azzurro, lampeggio lento |
| 6 | giallo, lampeggio lento (lo usa il driver con la batteria bassa) | 12 | azzurro che respira |
| 16 | rosso, lampeggio lento | 17, 18 | rosso + azzurro tenue, giallo + rosso |

Il LED chiesto dal task si vede solo se il driver non ha niente di più importante da mostrare: guasto > batteria
critica o bassa > in carica > il tuo.

**Limiti che il driver applica sempre** (bit di `drv_stato().limiti`):

| Bit | Quando | Effetto |
|---|---|---|
| `SIC_LIM_SOLLEVATO` | tutti e 4 gli anti-caduta nel vuoto fuori dalla base | ruote ferme, spazzole e aspirazione spente |
| `SIC_LIM_BASE` | sulla base | ruote ferme, salvo `drv_esci_dalla_base(1)`; spazzole e aspirazione spente |
| `SIC_LIM_SENSORI` | sensori spenti fuori dalla base | niente moto in avanti; spazzole e aspirazione spente |
| `SIC_LIM_BUMPER` | un bumper premuto | niente moto in avanti (indietro e rotazione sì) |
| `SIC_LIM_CLIFF` | un anti-caduta nel vuoto | niente moto in avanti |
| `DRV_LIM_BATTERIA` | batteria critica (sotto 13,8 V) | tutti i motori fermi |
| `DRV_LIM_GUASTO` | stato di guasto | tutti i motori fermi |
| `DRV_LIM_SENSORI_VECCHI` | nessun frame dei sensori da più di 100 ms | tutti i motori fermi |
| `DRV_LIM_NON_PRONTO` | subito dopo `drv_avvia`, mentre il driver manda la sequenza di avvio al GD32 | tutti i motori fermi |
| `DRV_LIM_DISABILITATI` | `c.motori = 0` | tutti i motori fermi |

## 8. Riferimento: task e tempo (`rt/rt.h`, `driver/driver.h`)

| Cosa | Uso |
|---|---|
| `rt_task_cfg_t` | `.nome`, `.cpu` (1–3), `.priorita` (o `rt_priorita_rm`), `.sporadico` (0/1), `.periodo_ns` (periodo, o intervallo minimo se sporadico), `.deadline_ns` (0 = uguale al periodo), `.fase_ns` (ritardo del primo rilascio), `.job`, `.arg` (passato al job), `.stack_byte` (0 = 256 KB), `.campioni`/`.max_campioni` (buffer preallocato per salvare ogni misura) |
| `rt_task_avvia(&t, &cfg, t0)` | crea il thread; il primo job parte a `t0 + fase` (stesso `t0` per tutti = partenza insieme) |
| `rt_task_ferma(&t)` | finisce il job in corso e ferma il task |
| `rt_stats_stampa(&t)`, `t.stats` | job, scadenze mancate, latenza ed esecuzione min/media/max, risposta massima |
| `rt_priorita_rm(cfg[], n, RT_PRIO_TASK_MAX)` | priorità Rate Monotonic: periodo più corto = priorità più alta |
| `rt_mutex_init(&m)` | mutex con priority inheritance, per i dati condivisi fra task |
| `rt_adesso()` | adesso, in ns (CLOCK_MONOTONIC) |
| `RT_MS(x)`, `RT_US(x)`, `RT_S(x)` | conversioni in ns |
| `drv_sveglia_ai_sensori(&t)` | il task sporadico t riceve una sveglia a ogni frame dei sensori (max 8 task) |
| `drv_sorveglia(&t, critico, soglia_ns)` | watchdog del driver sul task (soglia 0 = 3 periodi) |
| `drv_dimentica(&t)` | toglie il task da sveglie e watchdog: prima di `rt_task_ferma` |
| `rt_consenti_senza_privilegi(si)` | sul PC, senza permessi per SCHED_FIFO, fa partire i task lo stesso (tempi non validi) |

## 9. Regole d'oro

- **Nel job**: niente `printf`, `malloc`/`free`, file, `sleep`, chiamate che possono bloccarsi. Per i log: un buffer
  riempito dal job e svuotato dal main (o da un thread non RT).
- **Strutture grandi** (`lidar_giro_t`, mappe): `static` o globali, allocate prima di avviare i task; mai sullo stack.
- **Core**: CPU0 è del driver e dell'interrupt del Wi-Fi; i task vanno su CPU1–3, ognuno su un core fisso.
- **Dati condivisi** fra task: mutex con `rt_mutex_init`, tenuto il meno possibile. Fra task e driver niente mutex:
  le funzioni `drv_*` non si bloccano mai.
- **Uscita**: `drv_dimentica` → `rt_task_ferma` → `drv_ferma`, in quest'ordine.
- **Misure**: i tempi di esecuzione per l'analisi di schedulabilità si misurano sul robot (`rt_stats_stampa`,
  `cfg.campioni`), non sul PC.

## 10. Dove leggere il codice

- `driver/driver.h`: tutte le funzioni `drv_*`, commentate;
- `rt/rt.h`: task, statistiche, mutex;
- `gd32/gd32.h`: tutti i campi dei sensori (`gd32_sensori_t`);
- `driver/lidar_giro.h`: i punti del lidar;
- `gd32/sicurezza.h`: i limiti del moto;
- `tasks/primo.c`: l'esempio di questa guida.
