# Guida allo sviluppo

Questa guida spiega come è fatto il progetto e come si lavora: dal codice di un task fino
all'esecuzione sul robot. Va letta prima di scrivere un task.

## 1. Il quadro generale

```
  PC (x86, Linux)                                  Robot (ARM Cortex-A7, Linux 3.4)
 ┌──────────────────────────┐                      ┌──────────────────────────────┐
 │ sorgenti C (questa repo) │                      │ /tmp/rtes/<programma>        │
 │        │                 │   ./rtes.sh deploy   │   (in RAM: sparisce al       │
 │   gcc ─┼─> build/host/   │   ── SSH via Wi-Fi ─>│    riavvio)                  │
 │        │   test, sim     │                      │                              │
 │ arm-linux-gcc ─> build/arm/ ───────────────────>│ gira con SCHED_FIFO, root    │
 └──────────────────────────┘                      └──────────────────────────────┘
```

- **Lo stesso sorgente si compila due volte:**
  - con il `gcc` del PC, per i test e il simulatore;
  - con il cross-compilatore ARM, per il robot.
- **Cross-compilatore** vuol dire un compilatore che gira sul PC ma produce codice per un altro processore.
  Il nostro è in `toolchain/`, si installa con `tools/installa-toolchain.sh` ed è escluso da git.
- **I binari ARM sono statici:** contengono già la libreria C (musl 1.2.6). Sul robot non serve installare niente:
  si copia il file e si esegue.

## 2. Struttura della repository

| Percorso | Contenuto |
|---|---|
| `rt/rt_tempo.h` | tempo in nanosecondi (`int64_t`), conversioni, `rt_adesso()`, `rt_dormi_fino_a()` |
| `rt/rt.h`, `rt/rt.c` | task periodici SCHED_FIFO: creazione, ciclo periodico, statistiche, mutex con priority inheritance |
| `bench/` | programmi di misura: `latenza.c` (latenza e jitter), `carico.c` (carico CPU sui core scelti) |
| `gd32/gd32.h`, `gd32/gd32.c` | protocollo seriale con il GD32 (motori, sensori, carica): checksum, costruzione dei frame, deframer che si risincronizza, codici dei comandi. Senza Linux e senza allocazioni |
| `lidar/lidar.h`, `lidar/lidar.c` | protocollo del lidar 3irobotix LDS3: pacchetti `AA`, checksum a somma o CRC-16/MODBUS, deframer, codici |
| `spia/` | `spia`: si mette in mezzo tra i programmi originali e l'hardware e registra tutto (vedi sezione 9); `registro.h` è il formato del file |
| `tasks/` | i task applicativi, **da scrivere** |
| `hal/`, `sim/` | interfaccia hardware e simulatore (fasi successive) |
| `test/` | unit test: un file `test_*.c` è un programma; framework minimo in `test/test.h` |
| `tools/` | `installa-toolchain.sh`, `misure-fase0.sh` (campagna di misure sul robot), `misura-lunga.sh` e `notte.sh` (misure lunghe sorvegliate, vedi `docs/misure-lunghe.md`), `analizza-latenza.py` (percentili e istogrammi dei CSV), `estrai-frame.py` e `disassembla.sh` (reverse del protocollo GD32, vedi `docs/protocollo.md`), `spia-robot.sh` (inserisce/toglie la spia sul robot), `prova-spia.py` (prova della spia sul PC con periferiche finte) |
| `docs/` | piano, questa guida, diario delle fasi, misure |
| `Makefile` | regole di compilazione (di solito si passa da `rtes.sh`) |
| `rtes.sh` | tutti i comandi del progetto |

## 3. I comandi

| Comando | Cosa fa | Serve il robot? |
|---|---|---|
| `./rtes.sh test` | compila per il PC ed esegue tutti i test | no |
| `./rtes.sh test-arm` | esegue i test compilati per ARM con l'emulatore `qemu-arm-static` | no |
| `./rtes.sh build` | compila tutto per il robot in `build/arm/` | no |
| `./rtes.sh deploy latenza -d 10` | copia `build/arm/latenza` in `/tmp/rtes/` sul robot e lo esegue con quegli argomenti | sì |
| `./rtes.sh scarica /tmp/rtes/lat.csv` | copia un file dal robot in `build/robot/` | sì |
| `./rtes.sh ssh` | shell sul robot | sì |

**Regola del progetto:** prima di ogni commit si lancia `./rtes.sh test`, e devono passare tutti i test.
I test con l'emulatore verificano che il binario ARM *funzioni*, ma **i tempi misurati sotto emulazione o sul
PC non dicono niente sul robot**. Latenze e WCET si misurano solo sul robot.

## 4. Come funziona un task (`rt/rt.h`)

### Il modello
Un task periodico ha:
- un **periodo** T;
- una **deadline relativa** D (se non la indichi, D = T);
- una **fase** φ;
- una **priorità** fissa;
- un **core**.

Il job k viene rilasciato all'istante `istante_zero + φ + k·T` e deve finire entro `rilascio + D`.

Tu scrivi solo la funzione `job`, cioè il lavoro di un periodo. Il ciclo lo gestisce `rt.c`:

```
ripeti:
    dormi fino al rilascio            (clock_nanosleep con istante ASSOLUTO)
    inizio = adesso
    job(arg)                          ← il tuo codice
    fine = adesso
    aggiorna statistiche: latenza = inizio - rilascio
                          esecuzione = fine - inizio
                          risposta = fine - rilascio  (> D ⇒ deadline mancata)
    rilascio += T                     (mai "adesso + T": niente deriva)
```

Corrispondenze con FreeRTOS:

| FreeRTOS | Qui |
|---|---|
| `xTaskCreate(f, nome, stack, arg, prio, &h)` | `rt_task_avvia(&t, &cfg, istante_zero)` con `cfg.stack_byte`, `cfg.priorita` |
| il ciclo `for(;;){ ...; vTaskDelayUntil(&w, T); }` | lo fa `rt.c`; tu scrivi solo il corpo, cioè `job` |
| priorità più alta = numero più alto | uguale: 1..99, con 99 la più alta |
| `xSemaphoreCreateMutex()` (con priority inheritance) | `rt_mutex_init(&m)` |
| `vTaskDelete` | `rt_task_ferma(&t)` |

### Esempio completo

```c
#include "rt/rt.h"

static void controllo_job(void *arg)
{
    stato_t *s = arg;
    /* leggi sensori, calcola, comanda motori: deve stare nel WCET previsto */
}

int main(void)
{
    rt_init();                                     /* 1. blocca la memoria in RAM */
    stato_t stato = {0};
    rt_task_cfg_t cfg = {
        .nome = "controllo",
        .priorita = 80,                            /* rate-monotonic: periodo breve, priorità alta */
        .cpu = 2,                                  /* scheduling partizionato */
        .periodo_ns = RT_MS(20),
        .deadline_ns = 0,                          /* 0 = uguale al periodo */
        .job = controllo_job,
        .arg = &stato,
    };
    rt_task_t controllo;
    int64_t t0 = rt_adesso() + RT_MS(100);         /* 2. istante zero comune a tutti i task */
    if (rt_task_avvia(&controllo, &cfg, t0) != 0)  /* 3. avvio */
        return 1;
    /* ... */
    rt_task_ferma(&controllo);                     /* 4. arresto e statistiche */
    rt_stats_stampa(&controllo);
}
```

### Regole per scrivere un `job`
1. **Niente allocazioni** (`malloc`, `calloc`) dentro il job: tutta la memoria si prepara prima di avviare i task.
   Un'allocazione può chiedere memoria al kernel e durare un tempo imprevedibile.
2. **Niente I/O lento o bloccante** (`printf`, file, rete) nei task hard: la telemetria va in un task a
   priorità bassa o `SCHED_OTHER`, che legge i dati da una coda o da una struttura condivisa.
3. **I dati condivisi tra task si proteggono** con un mutex creato da `rt_mutex_init`, oppure con una coda.
   Non si conta mai sulle priorità per avere la mutua esclusione: con più core due task con priorità diverse
   possono girare *nello stesso istante*.
4. **Il job deve finire.** Un ciclo infinito a priorità alta blocca il suo core. Di default Linux concede ai task RT
   al massimo il 95% di ogni secondo (`/proc/sys/kernel/sched_rt_runtime_us` = 950000; `bench/latenza` stampa il
   valore effettivo), così il sistema resta recuperabile, ma il task salterebbe comunque tutte le sue deadline.
5. **Il tempo** si misura sempre con `rt_adesso()`, in ns su `CLOCK_MONOTONIC`, mai con `CLOCK_REALTIME` o `time()`.

## 5. Perché `rt_init()` e lo stack esplicito

- **`mlockall`:** senza, la prima volta che il programma tocca una pagina di memoria (o dopo che il kernel l'ha
  spostata su disco) il task si ferma per un *page fault*. È un ritardo da microsecondi a millisecondi, non
  prevedibile. `rt_init()` blocca in RAM tutta la memoria, presente e futura, come in un RTOS.
- **Stack esplicito:** con `mlockall` anche lo stack di ogni thread resta bloccato in RAM. Il default di glibc
  (sul PC) è 8 MB per thread, e un utente normale può bloccare solo 8 MB in tutto (`ulimit -l`). Per questo
  ogni task dichiara il suo stack (default `RT_STACK_DEFAULT` = 256 KB), come in `xTaskCreate`.
  Questo problema l'abbiamo incontrato davvero durante la fase 0.

## 6. Permessi

- `SCHED_FIFO` richiede root, oppure un limite `rtprio` sufficiente.
- **Sul robot** tutto gira come root.
- **Sul PC** il tuo utente ha `ulimit -r` = 99, quindi i test funzionano anche senza sudo.
- Su un altro PC, se `ulimit -r` vale 0, i test che creano task vengono **saltati**, con un messaggio, e non
  falliscono.

## 7. Accesso al robot

- Gli script di accesso stanno **fuori dalla repository**, in `~/Aspirapolvere/`:
  - `avvia-robot.sh`: Wi-Fi e AuxCtrl all'accensione;
  - `ssh-robot.sh`: SSH con la chiave dedicata.

  Restano fuori perché legati a questo PC (chiave SSH, rete). `rtes.sh` li richiama come `../ssh-robot.sh`.
  - `Script/trova-robot.sh`: trova l'IP del robot dal suo MAC (lo usa `ssh-robot.sh`).
- **Cosa fare in pratica (dal 2026-10-02):** accendi il robot; dopo circa 30 secondi è in Wi-Fi e
  funzionano `./rtes.sh deploy …` e `./rtes.sh ssh`. All'avvio parte lo stack originale (serve per caricare la
  batteria). `../avvia-robot.sh` resta come piano di riserva (vedi `~/Aspirapolvere/LEGGIMI.md`).
- **I file in `/tmp/rtes/` sul robot sono in RAM:** al riavvio spariscono. Nessuna di queste operazioni scrive
  sulla memoria flash del robot.

## 8. Aggiungere un programma o un test

- **Un nuovo test:** crea `test/test_qualcosa.c` con `CONTROLLA(...)` ed `ESEGUI(...)` (vedi `test/test.h`) e
  aggiungi `test_qualcosa` alla variabile `TEST` del `Makefile`.
- **Un nuovo programma di misura:** crea `bench/nome.c` e aggiungi `nome` alla variabile `PROGRAMMI` del
  `Makefile`. Una regola generica lo compila per PC e robot.

## 9. La spia (`spia/spia.c`)

Serve a vedere **tutto** quello che i programmi originali si scambiano con l'hardware, per ricavarne i protocolli.

```
AuxCtrl ⇄ /dev/ttyS3 = pty (bind mount) ⇄ spia ⇄ /dev/spia-gd32-vero (mknod, stessa UART) ⇄ GD32
AuxCtrl ⇄ /dev/ttyS1 = pty (bind mount) ⇄ spia ⇄ /dev/spia-lidar-vero                      ⇄ lidar
```

La spia crea una pty e la monta sopra il dispositivo: chi apre `/dev/ttyS3` parla in realtà con la spia, che
inoltra ogni byte alla UART vera e lo scrive nel registro. AuxCtrl non se ne accorge (non usa le linee di controllo
della seriale). La spia gira con `SCHED_FIFO` e la memoria bloccata, quindi non perde byte (strace ne perdeva un
terzo).

| Comando | Cosa fa |
|---|---|
| `tools/spia-robot.sh avvia` | copia la spia sul robot, ferma per qualche secondo lo stack originale, inserisce la spia e riavvia lo stack |
| `tools/spia-robot.sh stato` | byte e frame visti finora, errori |
| `tools/spia-robot.sh scarica [file]` | copia il registro sul PC (default `build/robot/spia/<data>.spia`) |
| `tools/spia-robot.sh ferma` | toglie la spia e riavvia lo stack |
| `build/host/spia riepilogo <file>` | per canale, direzione e comando: quanti frame, lunghezze, periodi |
| `build/host/spia leggi <file> [-c gd32\|lidar]` | un frame per riga, con tempo, direzione, codice, nome e dati |
| `tools/spia-robot.sh eventi` | segue in diretta gli eventi: byte dei sensori che cambiano, comandi diversi da heartbeat e velocità |
| `tools/sensori.py base` / `diff` | salva lo stato dei 96 byte dei sensori come riferimento / mostra quali byte sono cambiati (per le prove fisiche) |
| `tools/ritmo.py [secondi]` | trova i bit che cambiano a ritmo (azione ripetuta 3 volte) |
| `tools/spia-robot.sh socket` / `socket-stato` / `socket-ferma` / `socket-scarica` | spia sul canale tra AuxCtrl ed everest-server/RobotApp (`spia socket`, vedi `protocollo.md`) |
| `spia guida --invia 0x0d:3000 ...` (sul robot, stack fermo) | parla direttamente con il GD32 mandando solo i frame indicati (mai motori); usato per misurare il watchdog |
| `spia termios /dev/ttyS3` (sul robot) | impostazioni della seriale (sul robot non c'è `stty`) |

Tutto è in RAM: un riavvio del robot toglie la spia. Il registro contiene i byte grezzi così come sono arrivati,
quindi si può rianalizzare quando i decodificatori migliorano. Formato: `spia/registro.h`.

