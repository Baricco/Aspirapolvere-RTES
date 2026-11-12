# `driver/`: il driver del robot

Il driver prende il posto di AuxCtrl e della parte di everest-server che parlava con il GD32. È l'unico che usa le
seriali: i task leggono i dati che il driver pubblica e gli danno comandi, **sempre senza mutex** (`exchange.h`), così
il driver non aspetta mai un task. Fa da solo tutto quello che serve a tenere il robot vivo e sicuro: sequenza di
avvio, keepalive e heartbeat, decodifica dei sensori, lidar, batteria, limiti di sicurezza, watchdog dei task, stato
di guasto, messa in carica.

| File | Cosa fa | Linux? |
|---|---|---|
| `exchange.h/.c` | doppio buffer con numero di versione: un solo scrittore, tanti lettori, nessuna attesa | no |
| `lidar_scan.h/.c` | dai pacchetti del lidar ai giri completi; regolatore della velocità del motore del lidar | no |
| `battery.h/.c` | tensione media, percentuale come l'originale, livelli di allarme | no |
| `contacts.h/.c` | registro degli scatti di anti-caduta e bumper e degli incastri, per i task lenti | no |
| `docking.h/.c` | messa in carica guidata dagli infrarossi della base | no |
| `driver.h/.c` | il driver vero: thread, seriali, comandi, pubblicazione, sicurezza, guasto | sì |
| `rtes-driver.c` | programma per comandare il driver a mano (prove nel simulatore e sul robot) | sì |

I primi cinque sono **logica pura**, senza thread né stato globale: si provano con `test/test_driver_logica.c`. Il
driver li usa tenendo il suo mutex interno.

Altri documenti: uso e opzioni in `docs/GUIDA.md`, capitolo 11 "Il driver" (riga 255), e capitolo 13 "Scrivere un task"
(riga 487); diario della scrittura e prove in `docs/fase3.md`, capitolo "Driver (`driver/`)" (riga 57), prove
`r4`–`r7` (righe 109–158: perché due thread), "Tre errori trovati nel driver" (riga 304). Batteria dell'originale:
`docs/hardware.md`, capitolo "Batteria" (riga 110). Protocollo: `docs/gd32.md` e `docs/lidar.md`.

---

## `exchange.h` e `exchange.c`: scambio senza mutex

**Problema.** Il driver (priorità 99) pubblica i sensori ogni 20 ms e i task li leggono quando vogliono. Con un mutex
il driver potrebbe dover aspettare un task che lo tiene (inversione di priorità). Qui lo scrittore **non aspetta mai**:
se serve, è il lettore che ripete la lettura.

**Come funziona** (una variante del seqlock del kernel Linux). Ci sono due copie dei dati. Lo scrittore scrive sempre
nella copia che **non** contiene l'ultima versione, con il suo contatore `seq` dispari mentre scrive, poi annuncia la
versione nuova. Il lettore prende la copia dell'ultima versione, legge il contatore, copia i dati e rilegge il
contatore: se è cambiato (o era dispari) riprova. Con due copie il lettore riprova solo se lo scrittore scrive **due
volte** durante una lettura: in pratica mai.

- **`exchange_t`**: `version` (scritture completate; l'ultima è nella copia `version & 1`), `seq[2]`,
  `copy_version[2]`, `size`, `copy[2]` (le due copie, date da chi chiama), `retries` (letture ripetute).

| Funzione | Cosa fa |
|---|---|
| `exchange_init(s, copy0, copy1, size)` | prepara lo scambio con le due copie già allocate |
| `exchange_write(s, data)` | solo lo scrittore: scrive nella copia libera e pubblica la versione nuova. Non si blocca mai |
| `exchange_read(s, out)` | qualsiasi thread: copia l'ultima versione e ne ritorna il numero (0 = mai scritto). Chi legge due volte lo stesso numero sa che non c'è niente di nuovo |
| `exchange_version(s)` | il numero dell'ultima versione, senza copiare i dati |

---

## `lidar_scan.h` e `lidar_scan.c`: giri del lidar e regolatore

Il lidar manda un pacchetto `LIDAR_SCAN_NEW` per ogni settore di 22,5° (16 per giro): velocità, angolo di inizio e n
punti (intensità, distanza in quarti di mm). **Angoli del lidar**: 0° = dietro il robot, 90° = sinistra, 180° =
davanti, crescenti in senso **orario** visto dall'alto (il lidar è montato capovolto). Il commento in cima al file
racconta come è stato scoperto.

### Costanti e tipi

- `LIDAR_SCAN_MAX_POINTS` (800 punti per giro, abbondante).
- Montaggio, misurato: `LIDAR_X_M` (centro di rotazione 11,2 cm dietro il centro del robot), `LIDAR_Y_M`,
  `LIDAR_OFFSET_CDEG` (lo 0 del lidar è spostato di 14,5°), `LIDAR_OPTICAL_M` (il lidar è un triangolatore: il punto
  sta 2,5 cm a destra del raggio).
- **`lidar_point_t`**: `angle_cdeg` (angolo del lidar già corretto dello scarto, in centesimi di grado),
  `distance_mm` (0 = nessuna eco), `intensity`.
- **`lidar_scan_t`**: un giro completo: `number` (progressivo), `t_start_ns`, `t_end_ns`, `rev_s` (velocità media),
  `sectors` (16 se completo), `n` punti e `p[]`.
- **`lidar_assembler_t`**: `current` (il giro in costruzione), `last_start` (angolo dell'ultimo settore),
  `speed_sum`, `offset_cdeg`, `rev_s` e `t_speed` (ultima velocità letta e quando), i contatori `revolutions`,
  `discarded_scans`, `sectors`.
- **`lidar_regulator_t`**: `target` (giri/s), `integral`, `prev_error`, `power` (0..100).

| Funzione | Cosa fa |
|---|---|
| `lidar_assembler_init(a, offset_cdeg)` | prepara l'assemblatore |
| `u16be(b)` (static) | numero big-endian da 2 byte |
| `lidar_assemble(a, p, t, scan)` | un pacchetto arrivato all'istante `t`. Aggiorna sempre la velocità (da `LIDAR_SPEED` o `LIDAR_SCAN_NEW`). Quando l'angolo di inizio torna indietro il giro è finito: se ha almeno 12 settori su 16 lo copia in `scan` e ritorna 1, altrimenti lo scarta. Poi aggiunge i punti del settore, con l'angolo corretto dello scarto |
| `lidar_regulator_init(r, target)` | prepara il regolatore |
| `lidar_regulate(r, measured)` | un passo (ogni 20 ms) del regolatore di AuxCtrl: `power = 60 + I + 10·e + 10·(e − e_prec)`, con `I` limitato a ±20; ritorna la potenza da mandare con `GD32_LIDAR_MOTOR` |
| `lidar_point_xy(p, x, y)` | il punto nel sistema del robot (origine al centro, x avanti, y a sinistra, metri): converte l'angolo (orario, 0 dietro) e aggiunge posizione del lidar e scostamento ottico |

---

## `battery.h` e `battery.c`: batteria

Il GD32 dà solo la tensione (decimi di volt) e la temperatura, non la corrente. La **percentuale** è la stessa stima
dell'originale (classe `CBatteryCapacity3200` di everest-server), così il numero coincide con quello che mostrava
l'app: media della tensione su 100 s, curve diverse secondo il carico (fermo, in moto, ventola a vari giri, sulla base),
lontano dalla base non risale mai e cala a tempo, sulla base sale dell'1% ogni ~100 s. Il **livello** di allarme usa
la media dell'ultimo secondo, con isteresi: `BAT_LOW` sotto 14,2 V (LED giallo), `BAT_CRITICAL` sotto 13,8 V (motori
fermi); in carica sempre `BAT_OK`.

- Livelli `BAT_OK`, `BAT_LOW`, `BAT_CRITICAL`; soglie `BAT_LOW_THRESHOLD_DV`, `BAT_LOW_EXIT_DV`,
  `BAT_CRITICAL_THRESHOLD_DV`, `BAT_CRITICAL_EXIT_DV`.
- Stati di carico `BAT_STATE_CHARGING`, `BAT_STATE_2600PA` ... `BAT_STATE_600PA`, `BAT_STATE_MOVING`,
  `BAT_STATE_IDLE`, con i numeri dell'originale (l'ordine conta).
- Medie: `BAT_AVG_SAMPLES` (1 s), `BAT_AVG_ESTIMATE` (100 s), `BAT_AVG_CHARGING` (30 s).
- **`battery_t`**: anello per le soglie (`samples`, `n`, `i`, `sum`) e il `level`, anello delle tensioni
  (`voltages`, `n_t`, `i_t`, `sum_t`), anello del bit di carica (`charge`, `n_c`, `i_c`, `sum_c`), `status` (stato di
  carico), `estimate` (0..1, < 0 prima del primo frame), `charge_count`, `rise_count`.
- In `battery.c`: `STEP` (1%), `DECAY`, `FULL_CHARGE_V`; **`curve_t`** (coefficienti dei tre tratti, `limit` = frame
  del calo a tempo, `strong` per le ventole più forti) e la tabella `CURVES`.

| Funzione | Cosa fa |
|---|---|
| `battery_init(b)` | azzera; stato iniziale "fermo" come l'originale |
| `battery_update(b, voltage_dv, charging, on_base, moving, fan_rpm)` | un frame dei sensori: aggiorna la stima e la media dell'ultimo secondo, poi il livello con l'isteresi (servono almeno 10 frame). Ritorna il livello |
| `battery_load_state(moving, revolutions)` | lo stato di carico che l'originale sceglierebbe, dai giri della ventola e dal moto |
| `battery_volts(b)` | tensione media dell'ultimo secondo, in volt |
| `battery_percentage(b)` | percentuale 0..100 (0 prima del primo frame) |
| `battery_curve(status, v)` | la curva dell'originale da tensione a carica per uno stato di carico |
| `ring_push(...)` (static) | aggiunge un valore a un anello con somma corrente |
| `curve_of(status)` (static) | la curva di uno stato (quella "fermo" se lo stato non ne ha una) |
| `discharge(b, v)` (static) | lontano dalla base: la stima non sale mai; scende piano verso la curva, o cala dell'1% a tempo se la tensione è risalita |
| `charge(b, v, stable_charging)` (static) | sulla base: prima stima dalla tensione, poi +1% ogni ~100 s |
| `update_estimate(...)` (static) | sceglie lo stato di carico, fa ripartire la media se il carico diventa più forte, aggiorna gli anelli e la stima |

---

## `contacts.h` e `contacts.c`: registro dei contatti

Il driver legge ogni frame (50 Hz), ma un task lento (la mappatura, ogni 200 ms) può non vedere mai un sensore che resta
attivo per poco. Il driver annota ogni scatto in una lista circolare con un numero progressivo; chi legge applica solo
gli eventi con numero maggiore dell'ultimo visto. Ogni evento porta l'ultima posizione che il task del moto aveva dato
al driver con `drv_pose()` (cioè dove era il robot quando è successo). Non nasce nessun evento con i sensori spenti,
sulla base o con il robot sollevato; un "vuoto" degli anti-caduta non conta con il robot inclinato (sta superando un
tappeto). Il commento in cima a `contacts.h` dà tutti i casi e le prove da cui vengono.

- `CONTACTS_N` (32 eventi tenuti), `CONTACTS_VOID_FRAMES` (3 frame di vuoto di fila), `CONTACTS_TILT_CDEG` (3°),
  `CONTACTS_BUMPER_FRAMES`, `CONTACTS_STUCK_FRAMES`.
- Sensori: `CONTACT_VOID_LEFT_SIDE`, `CONTACT_VOID_LEFT_FRONT`, `CONTACT_VOID_RIGHT_FRONT`, `CONTACT_VOID_RIGHT_SIDE`,
  `CONTACT_BUMPER_LEFT`, `CONTACT_BUMPER_RIGHT`, `CONTACT_STUCK` (incastro in avanti, segnalato dalla navigazione con
  `drv_stuck`); `CONTACT_SENSORS` è il loro numero.
- **`drv_pose_t`** (`x`, `y`, `theta`, `t_ns`), **`drv_contact_t`** (`number`, `sensor`, `t_ns`, `pose_valid`, `pose`),
  **`drv_contacts_t`** (`last` e `events[]`: l'evento k sta in `events[k % CONTACTS_N]`), **`contacts_t`** (stato
  interno: `count`, `t_start`, `has_pose`, `start_pose`, `reported` per sensore, e la lista `published`).

| Funzione | Cosa fa |
|---|---|
| `contacts_init(c)` | azzera |
| `reset_counters(c)` (static) | azzera i contatori dei sensori (quando i valori non significano niente) |
| `contacts_frame(c, s, t, pose, stuck)` | un frame dei sensori: per ogni sensore conta i frame attivi di fila, ricorda istante e posizione del primo, e quando supera la soglia aggiunge **un** evento per scatto. Ritorna 1 se ha aggiunto eventi |

---

## `docking.h` e `docking.c`: messa in carica

I ricevitori infrarossi sono davanti e i contatti dietro. Quindi il robot arriva **di fronte** alla base guidato
dagli infrarossi, si ferma col muso a pochi centimetri, gira di 180° e arretra alla cieca per quei pochi centimetri.
Stesso approccio dell'originale (`CActionTransferMotionGoDock`, `CActionDockAlignState`), non una copia di tutti i suoi
stati. Il lidar dice solo quanto spazio c'è davanti al muso. Durante la manovra il driver ignora le velocità dei task
e usa `v` e `w` calcolati qui, con i limiti di sicurezza ancora attivi.

**Fasi** (`dock_phase_t`): `DOCK_P_SEARCH` (ruota finché trova il segnale e lo perde) → `DOCK_P_CENTER` (torna verso il
centro dell'arco) → `DOCK_P_APPROACH` (avanti sul confine fra i fasci 1 e 2, che è l'asse della base) →
`DOCK_P_FINE_TUNE` (a pochi cm, oscilla piano e si mette a metà) → `DOCK_P_TURN` (180° con l'IMU) → `DOCK_P_LAST_CM`
(retromarcia) → `DOCK_P_PRESS` (ancora pochi mm finché la carica è stabile) → `DOCK_P_CONTACT` (fermo, aspetta). Fasi
di recupero: `DOCK_P_DETACH` (indietro di 2 cm dopo un bumper), `DOCK_P_BACK_OFF` (arrivato lontano dall'asse: indietro
di 20 cm e si riprova), `DOCK_P_RETREAT` (avanti, poi nuovo tentativo; al massimo `DOCK_ATTEMPTS`).

- Esito in `drv_status_t.docking`: `DOCK_IDLE`, `DOCK_RUNNING`, `DOCK_DONE`, `DOCK_FAILED`.
- `DOCK_POINT_X_M` (0,30 m): il punto davanti alla base a cui la navigazione porta il robot prima di chiedere
  l'aggancio, e fino a cui esce dalla base.
- **`docking_t`**: `result`, `phase`, `attempt`, `reason`; il comando `v`, `w`; dall'IMU e dagli encoder `yaw`,
  `distance` e `arc` della fase corrente (con `prev_yaw_raw`, `prev_enc_left`, `prev_enc_right`); dal lidar
  `front_dist`, `near_points`, `t_scan`; ricerca e direzione (`seen`, `starts_inside`, `turn_dir`, `yaw_first`,
  `yaw_center`, `direction`, `yaw_a`, `yaw_b` con `has_a`, `has_b`, `prev_side`, `axis_frames`, `axis_retries`,
  `after_detach`); contatori e tempi (`frame`, `frame2`, `t_start`, `t_phase`, `t_signal`, `contact_frames`,
  `total_frames`, `t_no_contact`).
- In `docking.c` tutte le velocità, distanze e tempi sono `#define` con il commento che dice da dove vengono
  (`W_SEARCH`, `V_FAR`, `V_SHORT_BEAM`, `ARRIVAL_GAP`, `FINE_TUNE_*`, `PRESS_*`, `CHARGE_FRAMES`, `YAW_SCALE`...).

| Funzione | Cosa fa |
|---|---|
| `docking_init(a)` | azzera (esito `DOCK_IDLE`) |
| `docking_start(a, t)` | parte da capo, dal primo tentativo, con la ricerca |
| `docking_cancel(a)` | si ferma subito: esito `DOCK_IDLE` |
| `docking_abort(a, reason)` | si ferma con esito `DOCK_FAILED` (per esempio robot sollevato) |
| `docking_active(a)` | 1 se in questo momento il comando alle ruote lo decide l'aggancio |
| `docking_detach(a)` | 1 se serve il permesso di muoversi con i contatti chiusi (per premere sui contatti o staccarsi) |
| `docking_scan(a, g, t)` | a ogni giro del lidar: distanza media davanti (settore 170°..190°, fra 0,3 e 1,5 m) e quanti punti sono a meno di 6 cm dal muso |
| `docking_frame(a, s, t)` | a ogni frame dei sensori: aggiorna angolo (IMU) e strada (encoder), legge gli infrarossi e fa un passo della fase corrente; aggiorna `v`, `w`. C'è anche un tempo massimo totale |
| `normalize_angle(a)` (static) | angolo in (−π, π] |
| `clamp_abs(x, m)` (static) | limita `x` fra −m e m |
| `go_to_phase(a, f, t)` (static) | passa a una fase e azzera comando, strada, arco e contatori |
| `start_search(a, t)` (static) | ricomincia la ricerca del segnale |
| `fail(a, reason)`, `succeed(a)` (static) | chiudono con `DOCK_FAILED` o `DOCK_DONE` |
| `retreat(a, reason, t)` (static) | lascia perdere il tentativo: avanti di un po', poi si riprova |
| `ir_value(nibble)` (static) | il valore di un ricevitore come l'originale: 3 = fasci 1 e 2 insieme, altrimenti il fascio più basso |
| `side(left, right)` (static) | da che parte sta il robot rispetto all'asse: 0 niente segnale, 1 sull'asse, 2 deve girare a destra, 3 a sinistra |
| `rotate_towards(a, target)` (static) | fasi in rotazione: velocità angolare verso l'angolo voluto, rallentando negli ultimi 30°; 1 se è arrivato |

---

## `driver.h` e `driver.c`: il driver

### Architettura

Il driver sono **due thread `SCHED_FIFO`** sullo stesso core (di default CPU0, dove arrivano comunque gli interrupt),
fuori dallo scheduling dei task:
- **tempo** (priorità 99, `timer_loop`): dorme con `clock_nanosleep` fino al prossimo evento a tempo e fa heartbeat
  `0x06` (ogni 200 ms), keepalive `0x0d` (ogni 3 s) e il ciclo da 20 ms (`tick`);
- **ricezione** (priorità 98, `receiver_loop`): dorme in `ppoll` sulle due seriali e su un `eventfd` dei comandi;
  decodifica, pubblica, sveglia i task, reagisce subito ai sensori e ai comandi nuovi.

Due thread invece di uno perché un thread che dorme in `ppoll` a volte si sveglia con millisecondi di ritardo, uno che
dorme con `clock_nanosleep` no (`docs/fase3.md`, prove r4–r7, righe 109–158). Lo stato interno (`d`) lo usano entrambi,
sempre con il mutex `d.mx` (priority inheritance, sezioni di decine di microsecondi); i task non lo toccano mai.
C'è poi un terzo thread, **di servizio**, non real-time: stampa gli eventi, suona, scrive la registrazione su file.
Così i thread real-time non fanno mai I/O lento.


1. Thread "tempo" (timer_loop, driver/driver.c:1163, priorità 99)
Dorme con clock_nanosleep fino all'evento più vicino tra tre scadenze fisse:
- heartbeat 0x06 ogni 200 ms: senza heartbeat il GD32 spegne i motori dopo circa 2 s;
- keepalive 0x0d ogni 3 s: senza keepalive il GD32 riavvia tutto il robot;
- ciclo da 20 ms (tick), il lavoro periodico del driver:
  - all'accensione fa la sequenza di avvio del GD32;
  - controlla che il GD32 continui a mandare i sensori;
  - fa il watchdog dei task: scadenze mancate e task bloccati portano al guasto;
  - legge i tasti e lo stato della base;
  - gestisce le richieste di guasto che arrivano dai task;
  - ripete i comandi a ruote, spazzole e aspirazione;
  - regola la velocità del lidar;
  - pubblica lo stato del driver.

All'inizio, prima di ogni altra cosa, mette il robot in stato sicuro: tutti i motori a zero.

2. Thread "ricezione" (receiver_loop, driver/driver.c:1217, priorità 98)
Dorme in ppoll su tre cose e si sveglia quando una di queste ha dati:
- seriale del GD32: decodifica il frame dei sensori (ne arriva uno ogni 20 ms), lo pubblica per i task e sveglia i task a 20 ms. Se un sensore cambia il comando permesso (un bumper premuto, un anti-caduta sul vuoto), manda subito il nuovo comando alle ruote, senza aspettare il ciclo da 20 ms;
- seriale del lidar: decodifica le misure e ricompone il giro;
- eventfd dei comandi: un task ha appena dato un comando nuovo (velocità, spazzole...) e il thread lo manda subito al GD32.

3. Thread "di servizio" (non real-time)
Quello che avevi già capito: stampa gli eventi, suona e scrive la registrazione su file. Così i due thread real-time non fanno mai operazioni lente.

### Configurazione e avvio (`driver.h`)

- **`drv_config_t`**: `dev_gd32`, `dev_lidar` (seriali), `cpu` (core del driver), `cpu_flash` (core su cui spostare
  il lavoro della flash, quello del task aperiodico), `priority`, `motors` (0 = nessun comando ai motori), `suction`
  (aspirazione permessa), `bench` e `expected_cliff` (banco di prova, `docs/gd32.md`), `lidar_rev_s`,
  `lidar_offset_cdeg`, `sound_cmd`, `record` (file della registrazione dei byte grezzi), `test_return_s` e
  `test_press_s` (solo prove: batteria bassa finta, pressione del tasto finta), `without_rt`, `force`.
- `DRV_STARTING`, `DRV_READY`, `DRV_FAULT`, `DRV_STOPPED`: la fase del driver.
- `DRV_LIM_*`: motivi per cui un comando è ridotto, oltre agli `INTERLOCK_LIM_*`: `DISABLED`, `NOT_READY`, `FAULT`,
  `OLD_SENSORS`, `BATTERY`, `BENCH`, `NO_DUSTBIN`, `FORBIDDEN`.
- `DRV_LED_BLUE`, `DRV_LED_BLUE_BREATHING`, `DRV_SHORT_PRESS_MS` (una pressione è "breve" sotto 1 s).
- `drv_task_fault_t` (`DRV_FAULT_NONE`, `DRV_FAULT_UNLOCK_ATTEMPTS_EXHAUSTED`, ...): perché un task chiede il guasto.

### Dati pubblicati (`driver.h`)

- **`drv_sensors_t`**: `number`, `t_ns` e il frame decodificato `s` (`gd32_sensors_t`).
- **`drv_status_t`**: tutto lo stato del driver. Fase e guasto (`phase`, `fault` testo, `task_fault`,
  `gd32_version`); batteria (`volts`, `percentage`, `battery` livello, `charging` = la carica passa, `charge_complete`
  = bit "in carica" del GD32, che si accende a carica finita, `on_base`, `temperature_c`); `lifted`; tasti
  (`power_presses`, `home_presses`: pressioni brevi contate, un task confronta il numero con quello di prima); motori
  **dopo i limiti** (`v`, `w`, `limits`, `side_brush`, `main_brush`, `suction_rpm`, `suction_limits`, `led`);
  aggancio (`docking`, `docking_phase`, `docking_attempt`); lidar (`lidar_on`, `lidar_power`, `lidar_ok`,
  `lidar_rev_s`, `lidar_revolutions`); salute del collegamento e tempi (`sensor_frames`, `bad_frames`,
  `discarded_bytes`, `other_gd32_frames`, `gd32_heartbeats`, `keepalives`, `heartbeats`, `write_errors`, `sensors_hz`,
  `sensors_age_ns`, `tick_delay_max_ns`, `sensors_interval_max_ns`, `sensors_interval_min_ns` (il T degli sporadici
  nell'analisi), `work_max_ns`, `wakeups_done`, `without_rt`).

### Funzioni pubbliche

| Funzione | Cosa fa |
|---|---|
| `drv_config_default(c)` | valori di default: `/dev/ttyS3` e `/dev/ttyS1`, oppure le pseudo-seriali del simulatore da `RTES_GD32` e `RTES_LIDAR` (lo **stesso** programma gira sul PC e sul robot); legge anche `RTES_REGISTRA` e le variabili delle prove |
| `drv_config_test(c)` | come il default, ma con l'aspirazione vietata (programmi di prova) |
| `drv_start(c)` | controlla che nessun altro usi le seriali, apre registrazione e seriali, `rt_init`, crea i tre thread. Ritorna 0 o un codice `errno` |
| `drv_stop()` | ferma i thread, poi motori a zero, lidar spento, LED azzurro. Dopo, nessuno manda più il keepalive |
| `drv_sensors(out)`, `drv_scan(out)`, `drv_status(out)`, `drv_contacts(out)` | copiano l'ultima versione pubblicata (sensori, giro del lidar, stato, registro dei contatti) e ne ritornano il numero. Non si bloccano mai |
| `drv_point_xy(p, x, y)` | punto del lidar nel sistema del robot (è `lidar_point_xy`) |
| `drv_velocity(v, w)` | comando di velocità (m/s, rad/s), limitato a ±0,5 m/s e ±2π rad/s, convertito in tick e messo in una variabile atomica a 64 bit; sveglia il driver |
| `drv_brushes(side_speed, main_speed)` | spazzole (−100..100; la centrale negativa gira al contrario per sbloccarsi) |
| `drv_suction(rpm)` | aspirazione (0..30000 giri/min) |
| `drv_led(mode)` | modo del LED; guasto, batteria e carica li decide il driver e vincono |
| `drv_lidar(on, rev_s)` | accende o spegne il lidar, con la velocità voluta |
| `drv_pose(x, y, theta)` | il task del moto dà la posizione del robot: il driver la attacca agli eventi dei contatti |
| `drv_leave_base(enable)` | permette di muoversi piano anche sulla base (per uscirne in avanti) |
| `drv_stuck(enable)` | la navigazione segnala un incastro in avanti: diventa un evento `CONTACT_STUCK` |
| `drv_dock(enable)` | 1 fa partire la messa in carica, 0 la annulla |
| `drv_sound(name)` | un suono, suonato dal thread di servizio |
| `drv_report_fault(reason)` | un task manda il driver in guasto (al prossimo tick) |
| `drv_task_fault_text(reason)` | il testo di un motivo; ogni valore di `drv_task_fault_t` deve avere il suo caso, altrimenti la compilazione si ferma |
| `drv_clear_fault()` | prova a uscire dal guasto (solo se la causa non c'è più); aspetta fino a 2 tick, quindi **non** va chiamata da un task real-time |
| `drv_wake_on_sensors(t)` | il task sporadico `t` riceve una sveglia a ogni frame dei sensori (al massimo 8 task) |
| `drv_watch(t, critical, threshold_ns)` | il driver sorveglia il task: bloccato da più di `threshold_ns` o 3 scadenze di fila → guasto se critico, evento se no. Sul robot basta **una** scadenza mancata di qualsiasi task per il guasto |
| `drv_forget(t)` | toglie il task da sveglie e sorveglianza (prima di `rt_task_stop`) e aspetta che il driver non abbia più in mano il puntatore |

### Stato condiviso (`driver.c`)

- **`cmd`**: i comandi dei task, tutti atomici (`vw` = v e ω in un solo 64 bit, `side_brush_cmd`, `main_brush_cmd`,
  `suction`, `led`, `lidar_on`, `lidar_crev_s`, `leaving_base`, `clear_fault`, `stuck`, `dock`, `sound`,
  `task_fault`).
- Gli scambi `sensors_exchange`, `scan_exchange`, `status_exchange`, `contacts_exchange` (driver → task) e
  `pose_exchange` (moto → driver), con le loro coppie di buffer (`*_buffers`) e `pose_ready`.
- `subscribers` (task da svegliare ai sensori), `watched` (task sorvegliati, con `critical`, `threshold_ns`,
  `reported_once`, `seen_misses`), `driver_rounds` (risvegli completati, per `drv_forget`).
- `event_queue` con `event_head` ed `event_tail`: gli eventi in testo, scritti dai thread real-time e stampati da quello
  di servizio.
- **`d`**: lo stato interno (seriali e thread, riconoscitori e moduli logici `assembler`, `regulator`, `bat`,
  `interlock`, `contacts`, `dock_state`, stato di riposo e sollevamento, tasti, ultimi valori mandati al GD32
  `*_sent`, istanti dei prossimi eventi a tempo, misure dei tempi).
- **`rec`**: la registrazione dei byte grezzi: un anello in RAM da 1 MB (`RING_SIZE`) che i thread real-time
  riempiono e quello di servizio svuota sul file. Canali `CHANNEL_GD32` e `CHANNEL_LIDAR`; i record persi si contano in
  `lost`.

### Funzioni interne (`driver.c`)

**Eventi e registrazione**

| Funzione | Cosa fa |
|---|---|
| `now()` | `rt_now()` |
| `event(fmt, ...)` | aggiunge un evento in testo alla coda; se è piena l'evento si perde (il driver non aspetta) |
| `ring_copy_in`, `ring_copy_out` | copiano byte dentro e fuori l'anello della registrazione |
| `record(channel, direction, data, n, t)` | mette un record nell'anello (dai thread real-time); se non c'è posto lo conta come perso |
| `record_flush()` | dal thread di servizio: scrive sul file tutti i record dell'anello |
| `record_open(c)`, `record_close()` | aprono il file con l'intestazione della spia e lo chiudono svuotando l'anello |

**Seriale**

| Funzione | Cosa fa |
|---|---|
| `send(cmd_gd32, data, n)` | costruisce e scrive un frame verso il GD32 (e lo registra) |
| `send_u8`, `send_i32` | frame con un byte o con un intero a 32 bit |
| `send_velocity(v, w)` | il frame `0x66` con v e ω in tick, e ricorda cosa ha mandato |
| `full_stop()` | manda i frame di arresto di tutti i motori (`interlock_stop_frames`) |
| `device_busy(device)` | cerca in `/proc/*/fd` se un altro processo ha già aperto la seriale (per esempio AuxCtrl dello stack originale) |
| `open_serial(device)` | apre la seriale in modo grezzo a 115200 8N1 e butta i byte vecchi |

**Stato e guasto**

| Funzione | Cosa fa |
|---|---|
| `enter_fault(reason)` | stato di guasto: arresto, LED 8, suono, evento |
| `try_clear_fault(t)` | se qualcuno ha chiesto di azzerare il guasto, lo fa solo se la causa non c'è più (sensori recenti, nessun task critico bloccato) |
| `task_fault_request()` | guasto chiesto da un task: controllato **dopo** l'azzeramento, così se nello stesso tick un task chiede il guasto e un altro l'azzeramento vince il guasto |
| `publish_status(t)` | compone `drv_status_t` e lo pubblica (con le finzioni delle prove, che toccano solo quello che vedono i task) |

**Limiti e comandi**

| Funzione | Cosa fa |
|---|---|
| `general_limits(t)` | le condizioni per qualsiasi motore: motori abilitati, driver pronto e non in guasto, sensori recenti, batteria non critica |
| `limit_velocity(t, v, w)` | dal comando dei task (o dell'aggancio) a quello che si può mandare: limiti generali, poi banco o limiti a terra, poi sensori accesi da almeno 200 ms, robot non sollevato, primo giro del lidar dopo il riposo |
| `actuator_limits(t, cmd_gd32, value)` | spazzole e aspirazione: niente se il robot è sollevato o sulla base, o se il banco non lo permette |
| `clamp(v, lo, hi)` | limita un intero |
| `update_wheels(t, force)` | manda `0x66` se il comando limitato è cambiato (o sempre, ogni 20 ms) |
| `update_actuators(t)` | spazzole, aspirazione (anche niente cassetto: il GD32 non la ferma da solo), LED con le sue priorità, e il **riposo** sulla base: a carica stabile spegne prima i motori, poi i sensori; per uscire riaccende tutto e parte solo a sensori accesi (il commento spiega le prove del 5/11) |
| `update_lidar(t)` | alimentazione del lidar e un passo del regolatore; se il lidar non manda dati tiene la potenza a 60 e lo segnala |

**Frame in arrivo**

| Funzione | Cosa fa |
|---|---|
| `update_docking(t)` | a ogni frame: comandi dei task per l'aggancio, un passo della guida, evento quando cambiano fase o esito |
| `update_lifting(t)` | riconosce il sollevamento come l'originale (4 anti-caduta sul vuoto nel 90% di 0,8 s; due incrociati per 1 s; uno e robot inclinato per 0,2 s; i due anteriori per 5 s) e il riappoggio (pavimento per 0,8 s). Sollevato: motori spenti e aggancio interrotto |
| `update_buttons_and_base(t)` | misura le pressioni dei tasti (breve se rilasciato prima di 1 s) e il ritorno sulla base dopo almeno 1 s fuori |
| `buttons_and_base(t)` | al tick: una pressione breve o il ritorno sulla base azzerano il guasto, **poi** le pressioni si contano. Così un task che vede il numero crescere trova già il guasto azzerato |
| `on_gd32_frame(f, ctx)` | per ogni frame del GD32. Sensori: intervalli fra i frame, sollevamento, tasti, batteria, banco, pubblicazione, contatti, sveglia dei task iscritti, aggancio e reazione immediata delle ruote. Altri: heartbeat, versione, risveglio, errori |
| `on_lidar_packet(p, ctx)` | per ogni pacchetto del lidar: se completa un giro lo pubblica e lo passa all'aggancio |

**Ciclo da 20 ms e thread**

| Funzione | Cosa fa |
|---|---|
| `tick(t)` | ogni 20 ms: sequenza di avvio (`STARTUP`, un frame per ciclo), GD32 muto da 1 s, **watchdog dei task**, tasti e base, guasto chiesto dai task, ruote, attuatori, lidar, frame al secondo, pubblicazione dello stato |
| `read_serial(fd, is_gd32, t)` | legge tutto quello che c'è su una seriale e lo passa al riconoscitore giusto; -1 se la seriale è stata chiusa (simulatore uscito) |
| `pin_cpu()` | fissa il thread al core del driver |
| `move_flash_work(cpu)` | sposta sul core del task aperiodico le interruzioni della flash e del DMA e i thread del kernel che scrivono sulla flash, così le scritture non ritardano il driver (solo sul robot, una volta all'avvio) |
| `min_ns(a, b)` | il minimo fra due istanti |
| `work_done(start)` | a fine risveglio: durata massima del lavoro e contatore dei risvegli |
| `timer_loop(arg)` | il thread del tempo (vedi Architettura) |
| `receiver_loop(arg)` | il thread di ricezione (vedi Architettura) |
| `shutdown()` | dopo la fine dei thread: tutto fermo, LED azzurro, stato finale pubblicato |
| `service_loop(arg)` | il thread di servizio: stampa gli eventi, suona con `posix_spawn`, svuota la registrazione |
| `wake_driver()` | scrive sull'`eventfd` per svegliare il thread di ricezione |
| `create_thread(th, f, prio)` | crea un thread `SCHED_FIFO`; senza permessi (solo sul PC) con lo scheduling normale |

---

## `rtes-driver.c`: il driver comandato a mano

Un programma che avvia il driver, stampa una riga di stato al secondo e legge comandi dal terminale o da una FIFO
(`v`, `stop`, `spazzole`, `aspirazione`, `led`, `lidar`, `aggancia`, `esci_base`, `guasto`, `azzera`, `suono`, `blocca`,
`sfora`, `salva_giro`, `fine`: i comandi e le opzioni restano in italiano, sono il suo modo d'uso). Con `--task-prova`
avvia anche due task di prova sorvegliati, per provare sveglie e watchdog. Elenco completo di comandi e opzioni nel
commento in cima al file; uso in `docs/GUIDA.md`, capitolo 11 (riga 255).

| Funzione | Cosa fa |
|---|---|
| `on_signal(s)` | Ctrl+C e simili: chiede l'uscita |
| `job_sensors(arg)` | job del task di prova sporadico: legge i sensori; con `blocca` resta fermo (per provare il watchdog), con `sfora` calcola per 60 ms (per provare la scadenza mancata) |
| `job_lidar(arg)` | job del task di prova periodico: legge l'ultimo giro del lidar |
| `run_command(line)` | interpreta ed esegue una riga di comando |
| `read_command_lines(fd, buf, n, size)` | legge da un descrittore ed esegue ogni riga completa |
| `phase_name(f)` | nome della fase del driver per la riga di stato |
| `limits_text(l, out, size)` | i bit dei limiti in testo |
| `status_line(t, st, s, task)` | stampa la riga di stato |
| `main(argc, argv)` | opzioni, driver, task di prova, ciclo dei comandi e delle righe di stato, CSV, riepilogo finale. Esce con 4 se il driver è andato in guasto |
