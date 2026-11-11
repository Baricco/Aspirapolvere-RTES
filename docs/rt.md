# `rt/`: il framework real-time

Il framework trasforma un thread POSIX di Linux in un **task real-time** come quelli della teoria e di FreeRTOS: chi
scrive un task scrive solo la funzione `job` (il lavoro di un periodo); il ciclo "aspetta il rilascio → esegui il job
→ misura" lo fa il framework. Ogni task è un thread `SCHED_FIFO` con priorità fissa (1..99), fissato a un solo core
(scheduling partizionato). Il kernel del robot è `SMP PREEMPT`, non RT: le misure che dicono che basta sono in
`docs/fase0.md` e `docs/misure-lunghe.md`.

| File | Cosa fa |
|---|---|
| `rt_time.h` | il tempo: nanosecondi su `CLOCK_MONOTONIC`, conversioni, "adesso", "dormi fino a" |
| `rt.h`, `rt.c` | task periodici e sporadici, statistiche, watchdog, priorità Rate Monotonic, task aperiodico, mutex con priority inheritance |

Uso con esempi completi: `docs/GUIDA.md`, capitolo 4 "Come funziona un task" (riga 66), capitolo 5 "Perché `rt_init()`
e lo stack esplicito" (riga 149), capitolo 6 "Permessi" (riga 159). Decisioni sull'architettura: `docs/PIANO.md`, punto
"Architettura RT" (riga 34) e "Task aperiodico" (riga 53). Diario della scrittura: `docs/fase3.md`, capitolo "Framework
(`rt/`)" (riga 48).

---

## `rt_time.h`

Nel progetto il tempo è un `int64_t` di **nanosecondi** su `CLOCK_MONOTONIC`: parte dall'accensione, non torna mai
indietro e non cambia se qualcuno modifica data e ora (a differenza di `CLOCK_REALTIME`). In 64 bit ci stanno ~292 anni.

- Costanti `RT_NS_PER_US`, `RT_NS_PER_MS`, `RT_NS_PER_S` e macro `RT_US(x)`, `RT_MS(x)`, `RT_S(x)` (per esempio
  `RT_MS(20)` = 20 ms in nanosecondi).

| Funzione | Cosa fa |
|---|---|
| `rt_timespec_to_ns(ts)` | da `struct timespec` a nanosecondi |
| `rt_ns_to_timespec(ns)` | il contrario; tiene `tv_nsec` in [0, 10^9) anche per i valori negativi |
| `rt_now()` | l'istante attuale in nanosecondi |
| `rt_sleep_until(time_ns)` | dorme fino a un istante **assoluto** (`clock_nanosleep` con `TIMER_ABSTIME`), come `vTaskDelayUntil` di FreeRTOS: l'errore non si accumula da un periodo all'altro. Riprova se un segnale interrompe il sonno |

---

## `rt.h` e `rt.c`

### Il modello

- **Periodico**: il job k-esimo è rilasciato a `zero_time + phase + k·period` e deve finire entro `rilascio + deadline`.
- **Sporadico** (`sporadic = 1`): un job per ogni `rt_task_wake()` (nel progetto: a ogni frame dei sensori, vedi
  `drv_wake_on_sensors` in `docs/driver.md`). `period_ns` è allora l'intervallo **minimo** fra due attivazioni: il T
  dell'analisi.
- **Aperiodico**: un thread con lo scheduling **normale** di Linux su un core tutto suo, senza periodo e senza scadenza,
  che esegue un job a ogni sveglia. Fuori da Rate Monotonic e dalla sorveglianza del driver.

### Costanti

- `RT_PRIO_MIN`, `RT_PRIO_MAX` (1 e 99). `RT_CPU_ANY` = nessun core fissato.
- `RT_PRIO_DRIVER` (99, il driver, fuori dallo scheduling dei task) e `RT_PRIO_TASK_MAX` (90, la priorità più alta
  per i task).
- `RT_STACK_DEFAULT` (256 KB): lo stack va dichiarato, perché quello di default è 8 MB e con `mlockall` resterebbe
  tutto in RAM.

### Tipi

- **`rt_task_cfg_t`**: la configurazione. `name`, `priority`, `cpu`, `sporadic`, `period_ns`, `deadline_ns` (0 =
  uguale al periodo), `phase_ns`, `job` e `arg`, `max_job` (0 = mai fermarsi da solo), `stack_bytes`, e il buffer
  facoltativo `samples`/`max_samples` dove salvare le misure di ogni job.
- **`rt_sample_t`**: la misura di un job: `latency_ns` (inizio effettivo − rilascio) ed `execution_ns`.
- **`rt_stats_t`**: le statistiche: `job`, `missed_deadlines`, latenza e esecuzione minime, massime e somme
  (`latency_*_ns`, `exec_*_ns`), `cpu_max_ns` (il tempo di CPU peggiore di un job, senza il tempo passato senza girare),
  `response_max_ns`, `n_samples`. Si leggono dopo `rt_task_stop`.
- **`rt_task_t`**: il task in esecuzione. Oltre a `cfg`, `thread`, `stats`: `first_release_ns`, `stop`, `started` e
  `start_error` (il thread dice al creatore se si è configurato), `active`, `without_rt` (partito senza `SCHED_FIFO`,
  solo sul PC), `wakeup` e `pending_release_ns` (per gli sporadici), `lost_activations`. Per il watchdog del driver,
  atomici: `heartbeat_ns` (fine dell'ultimo job), `job_started_ns` (0 se aspetta), `consecutive_misses`,
  `total_misses`, e `first_miss` (job, latenza, esecuzione, CPU e risposta della **prima** scadenza mancata, per
  spiegarla).
- **`rt_aperiodic_cfg_t`** (`name`, `cpu`, `job`, `arg`, `stack_bytes`), **`rt_aperiodic_stats_t`** (`job`, `wakeups`,
  esecuzione massima e somma, `cpu_max_ns`, `response_max_ns` dalla prima sveglia servita alla fine del job) e
  **`rt_aperiodic_t`** (con `request_ns`, l'istante della prima sveglia non ancora servita).

### Funzioni pubbliche

| Funzione | Cosa fa |
|---|---|
| `rt_init()` | una volta all'avvio: blocca in RAM tutta la memoria (`mlockall`), così un task non si ferma mai per un page fault. Sul PC senza root blocca solo la memoria di adesso (il limite di 8 MB non basterebbe per gli stack futuri) |
| `rt_task_start(t, cfg, zero_time_ns)` | controlla la configurazione, crea il thread `SCHED_FIFO` con la priorità e lo stack chiesti e aspetta che si sia fissato al core. Lo stesso `zero_time_ns` per tutti i task li fa partire insieme (l'istante critico dell'analisi). Ritorna 0 o un codice `errno` |
| `rt_task_stop(t)` | chiede al task di fermarsi dopo il job in corso (sveglia anche uno sporadico che aspetta) e lo aspetta |
| `rt_task_join(t)` | aspetta che il task finisca da solo (serve `max_job > 0`) |
| `rt_stats_print(t)` | stampa le statistiche in microsecondi (testo rimasto in italiano: lo leggono anche gli script delle prove) |
| `rt_task_wake(t, release_ns)` | attiva uno sporadico. Non si blocca mai. Se l'attivazione precedente non è ancora partita questa si perde, si conta in `lost_activations` e ritorna 0 |
| `rt_task_is_stuck(t, now, threshold_ns)` | per il watchdog: 1 se da più di `threshold_ns` un job è iniziato e non finito, o (sporadico) un'attivazione aspetta, o (periodico) nessun job finisce oltre il periodo |
| `rt_priority_rm(cfg[], n, prio_max)` | Rate Monotonic: periodo più corto = priorità più alta; periodi uguali, priorità uguale. Ordina una copia dei puntatori (insertion sort, n piccolo) |
| `rt_allow_without_privileges(enable)` | solo sul PC: senza permessi per `SCHED_FIFO` i task partono con lo scheduling normale invece di fallire. I tempi allora non valgono |
| `rt_aperiodic_start(a, cfg)` | crea il thread aperiodico con lo scheduling normale dichiarato esplicitamente (chi lo crea potrebbe essere `SCHED_FIFO`) e lo mette in attesa |
| `rt_aperiodic_wake(a)` | sveglia il task aperiodico, da qualsiasi thread, senza bloccarsi. Solo la prima sveglia non servita fa il `sem_post`: più sveglie prima che il job parta valgono come una. Cosa fare lo dicono i dati che chi chiede ha scritto prima |
| `rt_aperiodic_stop(a)` | lo ferma dopo il job in corso e copia il numero di sveglie nelle statistiche |
| `rt_aperiodic_print(a)` | stampa le sue statistiche |
| `rt_mutex_init(m)` | mutex con **priority inheritance**: chi tiene il mutex eredita la priorità di chi lo aspetta (il caso del Mars Pathfinder). Il priority ceiling non c'è in musl |

### Funzioni interne (`rt.c`)

| Funzione | Cosa fa |
|---|---|
| `stats_reset(s)` | azzera le statistiche, con i minimi a `INT64_MAX` |
| `thread_cpu_ns()` | il tempo di CPU usato finora dal thread (`CLOCK_THREAD_CPUTIME_ID`) |
| `stats_update(t, release, start, end, cpu)` | dopo ogni job: latenza, esecuzione, risposta, scadenza mancata (con la prima salvata in `first_miss` e i contatori per il watchdog), minimi e massimi, campione nel buffer |
| `pin_to_core(cpu)` | fissa il thread chiamante a un core (`pthread_setaffinity_np`) |
| `task_loop(arg)` | il corpo di ogni task: si fissa al core (dall'interno: musl non ha `pthread_attr_setaffinity_np`), lo dice al creatore, poi ripete "aspetta il rilascio (o la sveglia) → job → misure". Il rilascio dopo è sempre un periodo dopo il **precedente**, non dopo "adesso": un job che sfora non sposta i successivi |
| `aperiodic_loop(arg)` | il corpo del task aperiodico: aspetta la sveglia, azzera la richiesta **prima** del job (una sveglia durante il job ne fa partire un altro), esegue e misura |
