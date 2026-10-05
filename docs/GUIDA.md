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
| `driver/` | il driver del robot (fase 3): due thread (tempo 99, ricezione 98), dati pubblicati senza mutex, limiti di sicurezza, watchdog dei task; `rtes-driver` per provarlo a mano (sezione 11) |
| `sim/` | simulatore (fase 3): GD32 e lidar finti sulle seriali, stanza disegnabile da una pagina web, stanze in `sim/stanze/` (sezione 12) |
| `tasks/` | i tuoi programmi con i task (fase 4): `tasks/NOME.c` diventa `build/host/NOME` e `build/arm/NOME` (sezione 13) |
| `hal/` | vuota: l'idea di un'interfaccia hardware a due backend è stata sostituita dal simulatore a livello di seriali |
| `test/` | unit test: un file `test_*.c` è un programma; framework minimo in `test/test.h` |
| `tools/` | `prova-driver.py` (driver contro il simulatore, dentro `make test`), `driver-robot.sh` (prove sul robot), `installa-toolchain.sh`, `misure-fase0.sh` (campagna di misure sul robot), `misura-lunga.sh` e `notte.sh` (misure lunghe sorvegliate, vedi `docs/misure-lunghe.md`), `analizza-latenza.py` (percentili e istogrammi dei CSV), `estrai-frame.py` e `disassembla.sh` (reverse del protocollo GD32, vedi `docs/protocollo.md`), `spia-robot.sh` (inserisce/toglie la spia sul robot), `prova-spia.py` (prova della spia sul PC con periferiche finte) |
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
| `./rtes.sh sim [stanza] [programma]` | simulatore + pagina (http://localhost:8080) + `rtes-driver` da tastiera, oppure il tuo programma | no |
| `./rtes.sh sim-solo [stanza]` | solo simulatore e pagina | no |
| `make build/host/NOME build/arm/NOME` | compila `tasks/NOME.c` per il PC e per il robot | no |
| `tools/driver-robot.sh ...` | prove del driver o dei tuoi programmi sul robot, con il custode al posto dello stack (sezione 11) | sì |

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
| `spia guida ...` (sul robot, stack fermo) | banco di prova: parla direttamente con il GD32 (sezione 10) |
| `spia termios /dev/ttyS3` (sul robot) | impostazioni della seriale (sul robot non c'è `stty`) |

Tutto è in RAM: un riavvio del robot toglie la spia. Il registro contiene i byte grezzi così come sono arrivati,
quindi si può rianalizzare quando i decodificatori migliorano. Formato: `spia/registro.h`.


## 10. Il banco di prova (`spia guida`, fase 2)

`spia guida` prende il posto dello stack originale e parla direttamente con il GD32: manda keepalive `0x0d`
(3 s) e heartbeat `0x06` (200 ms), i comandi che gli chiedi, e registra tutto. Ogni comando passa dal **blocco di
sicurezza** (`gd32/sicurezza.h`): ruote, spazzole e aspirazione diverse da zero partono solo con `--sollevato` e
con il frame dei sensori che dice "sollevato" (anti-caduta `0f`, sensori alimentati, fuori dalla base); se smette
di essere vero, o i sensori tacciono 100 ms, ferma tutti i motori fino alla fine. All'uscita ferma sempre i motori.

| Comando (dal PC) | Cosa fa |
|---|---|
| `tools/banco-robot.sh avvia NOME [opzioni]` | ferma lo stack e avvia `spia guida -o NOME.reg --csv NOME.csv [opzioni]` sul robot; **quando finisce, per qualsiasi motivo, lo stack riparte da solo** |
| `tools/banco-robot.sh lidar NOME [opzioni]` | come `avvia`, registrando anche la seriale del lidar (attenzione: ~45 MB/h in RAM) |
| `tools/banco-robot.sh stato` / `segui` / `ferma` / `scarica NOME` | ultime righe / in diretta / termina (motori a zero) / copia i file in `build/robot/fase2/NOME/` |
| `tools/finestra.py NOME [N] [campi]` | media, minimo e massimo degli ultimi N frame del CSV sul robot |
| `tools/analizza-lidar.py NOME-lidar.reg [NOME.reg]` | velocità per potenza, settori e punti per giro, distanze |

Opzioni di `spia guida`:

| Opzione | Significato |
|---|---|
| `--una CMD[:DATI]` | comando singolo all'avvio. DATI: esadecimale (`0x97:01`) o tipi (`0x66:i32=828,i32=0`, tipi i8 u8 i16 u16 i32 u32) |
| `--invia CMD[:DATI]@MS` | comando periodico |
| `--sequenza FILE` | comandi a tempi stabiliti: righe `T cmd`, `T ogni MS cmd`, `T basta CMD`, `T fine` (esempi in `tools/sequenze/`) |
| `--comandi FIFO` | comandi dal vivo, uno per riga: sul robot `echo 0x8d:08 > /tmp/rtes/comandi` |
| `--sollevato` | ammette i motori (sempre sotto il blocco di sicurezza) |
| `--tensione-minima DV` | esce (codice 3) se la tensione media scende sotto DV decimi di volt |
| `--csv FILE [--csv-ogni MS]` | tutti i campi decodificati dei sensori (`gd32_sensori_t`) |
| `--durata S`, `--senza-keepalive`, `--senza-heartbeat` | durata; prove sul watchdog del GD32 |

Esempio: `tools/banco-robot.sh avvia prova --sollevato --sequenza tools/sequenze/c2-ruote.seq --tensione-minima 140`.
Il primo comando ai motori deve venire dopo ≥ 500 ms (prima non ci sono frame dei sensori e viene rifiutato).
Codici d'uscita: 0 normale, 2 opzioni sbagliate, 3 tensione minima, 4 blocco di sicurezza scattato.
Le prove sul PC con un GD32 finto sono in `tools/prova-guida.py` (dentro `make test`).

## 11. Il driver (`driver/`, fase 3)

Il driver è il nostro sostituto di AuxCtrl e della parte di everest-server che parla con il GD32. Sono **due thread**
`SCHED_FIFO` fissati su CPU0, **fuori dallo scheduling dei task**: il thread del **tempo** a priorità 99
(`RT_PRIO_DRIVER`), che dorme con `clock_nanosleep` e fa tutto ciò che è periodico, e quello di **ricezione** a 98,
che dorme sulle seriali e sui comandi dei task. Sono due perché un thread che dorme in `ppoll` su una seriale, con
questo kernel, a volte si sveglia con millisecondi di ritardo, mentre uno che dorme su un timer no (prove r4–r7 in
`docs/fase3.md`). I task non parlano mai con le seriali: leggono i dati che il driver pubblica e gli danno comandi.

```
                       ┌──────────── driver (CPU0: tempo a priorità 99, ricezione a 98) ───────┐
 /dev/ttyS3 (GD32) ◀──▶│ avvio, keepalive 0x0d 3 s, heartbeat 0x06 200 ms, 0x66 ogni 20 ms     │
 /dev/ttyS1 (lidar) ──▶│ sensori 0x15 (50 Hz) · giri del lidar + regolazione · batteria · 0x9d │
                       │ limiti di sicurezza · watchdog dei task · stato di guasto (LED 8)     │
                       └───────┬──────────────────────────────────────────────▲───────────────┘
            pubblica (senza mutex: driver/scambio.h)              comandi (atomici, non bloccano)
                drv_sensori() drv_giro() drv_stato()     drv_velocita() drv_spazzole() drv_led() ...
                       ▼                                                       │
        task sporadici a 20 ms (sveglia a ogni frame)    task periodici (100, 200, 500 ms...)
```

| File | Contenuto |
|---|---|
| `driver/driver.h`, `driver.c` | l'interfaccia per i task e il thread del driver |
| `driver/scambio.h`, `.c` | doppio buffer con numero di versione: il driver scrive senza mai aspettare, i lettori ottengono sempre una copia coerente |
| `driver/batteria.h`, `.c` | percentuale calcolata come il software originale (`hardware.md`, "Batteria"); livelli con isteresi sulla media di 1 s: bassa < 14,2 V (LED 6), critica < 13,8 V (motori fermi) |
| `driver/lidar_giro.h`, `.c` | settori `0xAD` → giri completi; regolatore di AuxCtrl (`60 + I + 10 e + 10 Δe`) |
| `gd32/sicurezza.h` | `sic_terra_limita`: i limiti del moto "a terra"; `sic_permesso`: il blocco del banco di prova |
| `driver/rtes-driver.c` | il programma `rtes-driver`: il driver da solo, comandato a mano |

**Cosa fa il driver da solo** (senza che i task debbano pensarci):
- **all'avvio** manda subito i frame di arresto di tutti i motori, poi la sequenza di avvio di everest
  (`0x0c 01`, `0x07`, `0x17`, `0x18`, `0x71 0`, `0x65 00`, `0x19 01`, `0x7c 00 ff ff`, `0x78 01`, `0x08`, `0x65 02`),
  un frame ogni 20 ms. È "pronto" quando sono arrivati i sensori (~0,25 s);
- **keepalive** `0x0d` ogni 3 s e **heartbeat** `0x06` ogni 200 ms; **velocità** `0x66` ogni 20 ms (anche zero);
- **sensori**: a ogni frame `0x15` li decodifica, li pubblica e **sveglia i task iscritti**;
- **lidar**: lo accende/spegne (`0x97`), regola la velocità ogni 20 ms (`0x71`) e pubblica ogni giro completo;
- **batteria e base**: sensori spenti sulla base e accesi fuori (`0x9d`, come everest), LED 4 in carica, LED 6
  con la batteria bassa;
- **limiti**: ogni comando di velocità passa da `sic_terra_limita` (vedi sotto), le spazzole si fermano con il
  robot sollevato o sulla base, l'aspirazione è vietata se non la si abilita;
- **watchdog dei task** e **stato di guasto**: se un task critico si blocca o manca 3 scadenze di fila, se i
  sensori tacciono per 1 s o se la seriale si chiude, il driver ferma tutti i motori, accende il LED 8 (rosso,
  lampeggio veloce), suona `sound_erro_harefault` e da lì rifiuta i comandi ai motori finché qualcuno non chiama
  `drv_azzera_guasto()` (che riesce solo se la causa è sparita).

**Limiti del moto "a terra"** (`sic_terra_limita`, il robot appoggiato sul pavimento):

| Situazione (dall'ultimo frame dei sensori) | Cosa passa |
|---|---|
| bumper premuto, oppure un anti-caduta nel vuoto | tutto tranne il moto **in avanti** (indietro e rotazione sì, per allontanarsi) |
| tutti e 4 gli anti-caduta nel vuoto, fuori dalla base | niente: il robot è stato **sollevato** |
| sulla base | niente, salvo `drv_esci_dalla_base(1)`: piano (≤ 0,1 m/s, ≤ 0,5 rad/s) |
| sensori spenti fuori dalla base | niente moto in avanti |
| sensori vecchi (> 100 ms), batteria critica, avvio, guasto | niente |

Con `rtes-driver --banco [--cliff-atteso X]` vale invece il blocco del banco di prova (motori solo con il robot
**sollevato**, come `spia guida --sollevato`).

**`rtes-driver`** (il driver comandato a mano): una riga di stato al secondo e comandi da tastiera o da FIFO:
`v 0.1 0`, `stop`, `spazzole 0 30`, `led 3`, `lidar 1`, `esci_base 1`, `azzera`, `suono sound_beep_beep`,
`salva_giro giro.csv`, `blocca` (con `--task-prova`: blocca un task per provare il watchdog), `fine`.
Opzioni utili: `--senza-motori`, `--aspirazione`, `--task-prova`, `--durata S`, `--csv FILE`, `--ogni MS`,
`--registra FILE` (i byte grezzi di GD32 e lidar nel formato della spia: si rileggono con `tools/estrai-frame.py` e
servono a `./rtes.sh taratura` e `./rtes.sh valida-sim`; li scrive un thread non real-time, il driver non si blocca).
La riga di stato mostra anche v e ω misurati dal GD32, l'imbardata dell'IMU, il cassetto, l'IR della base e i giri
della ventola (con il motivo, se l'aspirazione chiesta è ferma).

Sul robot (stanotte senza lo stack originale: al suo posto gira il **custode**, `spia guida` con solo keepalive e
heartbeat, che riparte da solo quando il driver termina):

| Comando (dal PC) | Cosa fa |
|---|---|
| `tools/driver-robot.sh avvia NOME [opzioni di rtes-driver]` | ferma il custode e avvia `rtes-driver` in background (`/tmp/rtes/NOME.out`, `NOME.csv`) |
| `tools/driver-robot.sh comando "lidar 1"` | manda un comando al driver che gira |
| `tools/driver-robot.sh stato` / `segui` / `ferma` / `scarica NOME` | ultime righe / in diretta / termina / copia in `build/robot/fase3/NOME/` (anche `NOME.reg`, se c'è) |
| `tools/driver-robot.sh custode` | fa partire il custode se nessuno sorveglia il GD32 |
| `tools/driver-robot.sh prepara` | all'inizio di una sessione, dopo un'accensione: ferma lo stack originale e avvia il custode in un solo comando |

## 12. Il simulatore (`sim/`)

Serve a lavorare ai task **senza il robot**. Finge di essere il GD32 e il lidar **sulle seriali**, con gli stessi byte del
robot vero, quindi il driver e i tuoi task girano sul PC **senza cambiare una riga**. È tarato sulle registrazioni del
robot e confrontato con esse automaticamente. Le prove di sicurezza sul robot si fanno comunque.

```
 ./rtes.sh sim                                         il tuo programma (o rtes-driver)
 ┌──────────── build/host/sim ────────────────┐       ┌──────────────────────────────┐
 │ stanza a oggetti · fisica ogni 2 ms         │ gd32  │ driver (drv_avvia) + task    │
 │ GD32 simulato  ── pseudo-seriale ───────────┼──────▶│ come /dev/ttyS3              │
 │ lidar simulato ── pseudo-seriale ───────────┼──────▶│ come /dev/ttyS1              │
 │ storia di tutti i messaggi                  │ lidar └──────────────────────────────┘
 │ pagina http://localhost:8080 (sim/web/)     │
 └─────────────────────────────────────────────┘
```

**Da dove vengono i numeri.**
- `sim/robot.h`: la geometria, con la fonte di ogni valore:
  - schede tecniche: diametro 349 mm, altezza 95 mm, batteria 3200 mAh;
  - firmware: ruote, carreggiata, tick, scale, limiti;
  - misure: ordine degli anti-caduta, IR della base davanti;
  - utente: contatti di carica **dietro**.
- `sim/parametri.txt`: la dinamica, ricavata dalle registrazioni del robot con `./rtes.sh taratura`
  (`tools/taratura/tara.py`), anche qui con la fonte di ogni valore:

| Cosa | Valore tarato | Registrazione |
|---|---|---|
| ritardo di consegna dei frame | quasi sempre ~0,34 ms, coda fino a 5 ms: intervalli da ~15 a ~25 ms | d (16 min) + fase 3 r7 |
| comando delle ruote | vale dopo 29 ms | c2, c2b |
| risposta a terra | del primo ordine: costante di tempo 109 ms per v, 73 ms per ω; al massimo 1772 tick/s (~0,39 m/s) | val1 (fase 3) |
| rampe da sollevato | v ~16 600 tick/s²; ω ~2 200 tick/s² (in rotazione il GD32 è lento) | c2, c2b |
| scale di `0x66` | ~4580 tick per metro, 553 unità per rad/s (`gd32/gd32.h`) | lidar e IMU, fase 3 |
| lidar sul robot | angoli orari, scarto 14,5°, 11,2 cm dietro il centro, scostamento ottico 2,5 cm | fase 3, kaiaai |
| velocità misurata (byte 32/36) | sugli encoder in 70 ms (passi di ~7 unità) | c2 |
| senza heartbeat | potenza tolta dopo 2,04 s, poi le ruote si fermano per inerzia (~1 880 tick/s²) | c2b |
| IMU | deriva dell'imbardata −0,3°/h; beccheggio −0,55°, rollio +0,38° a riposo | c8scarica (5 h) |
| lidar | giri/s = 0,0856 × potenza + 0,17; salita 0,18 s; ~2040 misure/s; rumore 7 mm fino a 3 m, 37 mm oltre 6 m | c1 |
| spazzole e aspirazione | correnti, echi, 219 giri/min per punto di PWM, corrente ∝ giri^2,27 | c3, c3b |
| batteria | curva di scarica; ~2,45 Ah utili fra 16,3 e 13,8 V | c8scarica |
| byte non calcolati | presi da un frame vero a riposo (es. byte 88 = 6403) | d |

**Validazione.** `./rtes.sh valida-sim c2 c2b c3 c1 val1` rimanda al simulatore, con i tempi originali, i comandi
registrati sul robot e confronta le uscite. Risultati al 5/10 (val1 è a terra, registrata dal driver con `--registra`):

| Prova | Risultato |
|---|---|
| encoder | 0,2% (c2) e 1% (c2b) da sollevato; **1% (val1) a terra** |
| velocità misurata | 16 unità di scarto quadratico medio (828 = 5 cm/s) |
| spazzola, ventola, correnti | identiche |
| velocità del lidar | scarto 0,00 giri/s |

La versione breve (c2) è dentro `make test`; val1 (circa 5 minuti) si lancia a mano.

**Cosa resta stimato [?]** (elenco aggiornato in `docs/fase3.md`):
- altezze di corpo (7,5 cm), torretta (9,5 cm) e piano del lidar (8,7 cm);
- raggio della torretta (la posizione del lidar invece è misurata: 11,2 cm dietro il centro);
- posizione degli anti-caduta (±18° e ±50° a 15 cm);
- forma e altezza della base;
- confini esatti dei fasci dell'IR della base (modello dalla prova ir1, fase 3; il fascio 8 non si è mai visto);
- corrente di carica e consumo del SoC;
- dinamica a terra;
- il fatto che il GD32 ignori gli anti-caduta con i sensori spenti (serve per uscire dalla base).

**La stanza** (`sim/stanze/*.txt`, formato in `sim/mondo.h`) è un elenco di oggetti, una riga ciascuno:

```
stanza nome=soggiorno larghezza=6 profondita=3.9
robot x=0.8 y=2.5 angolo=-90                    posizione iniziale (gradi: 0 = verso destra, antiorario)
base x=0.6 y=0.22 angolo=90                     centro del lato con i contatti, verso dove guarda
muro da=0,0.05 a=6,0.05 spessore=0.1
rettangolo nome=divano x=1.15 y=3.4 larghezza=2.1 profondita=0.8 angolo=0 quota_da=0 quota_a=0.45
cerchio nome=gamba x=3.45 y=1.7 raggio=0.025 quota_da=0 quota_a=0.72 gruppo=tavolo
vuoto nome=scala punti=5,0.1 5.9,0.1 5.9,0.9 5,0.9
```

Le **quote** (altezza da / a, in metri) decidono cosa succede. I colori sono quelli della pagina.

| L'oggetto… | Effetto | Colore |
|---|---|---|
| tocca il corpo (sotto 7,5 cm) e taglia il piano del lidar | bumper e lidar | grigio scuro |
| tocca il corpo ma sta sotto il piano del lidar (sotto 8,7 cm) | solo bumper: il lidar non lo vede | marrone |
| sta fra 7,5 e 9,5 cm (mobile rialzato) | il robot si **incastra con la torretta, senza bumper**: la torretta non ha interruttori | arancione |
| sta sopra il robot, oppure è più basso di 2 cm (soglie, tappeti) | il robot ci passa | tratteggiato |
| vuoto | lo vedono gli anti-caduta; se il centro del robot ci finisce sopra, il robot cade | azzurro |

Le stanze del vecchio formato a griglia si convertono con `tools/converti-stanza.py`.

**La pagina** (`http://localhost:8080`) ha tre schede.
- **Stanza** (editor):
  - clic su un oggetto, sul robot o sulla base per selezionarlo;
  - si trascina per spostare, con la maniglia tonda si ruota (Shift: scatti di 15°), con i quadratini si cambia misura;
    sui muri si trascinano i capi, sui vuoti i vertici;
  - gli oggetti con lo stesso `gruppo` (un tavolo con le gambe) si muovono insieme;
  - il pannello mostra e modifica x, y, angolo, misure e quote;
  - tastiera: R / Shift+R ruota di ±15°, frecce 1 cm (Shift: 10 cm), Canc, Ctrl+D duplica, Ctrl+Z / Ctrl+Y;
  - strumenti per disegnare muri, rettangoli, cerchi e vuoti;
  - oggetti pronti: divano, tavolo, sedia, letto, armadio, credenza rialzata, pianta, soglia, gradino, scala, base;
  - nuova, salva e carica.
- **Robot**: solleva / posa, cassetto, tasti, batteria; stato di robot, GD32 e lidar.
- **Messaggi** (ispettore):
  - griglia dei **96 byte** del frame dei sensori, colorata per campo, con i byte cambiati riquadrati. Al passaggio del
    mouse: nome, valore grezzo e decodificato con l'unità, affidabilità e spiegazione (la descrizione unica è in
    `gd32/descrizione.c`);
  - bit allargati dei byte a bit e tutti i valori decodificati;
  - ultimo comando di ogni tipo, decodificato;
  - ultimo pacchetto del lidar, punto per punto;
  - lista di tutti i messaggi. Gli interruttori mostrano anche le ripetizioni (heartbeat, keepalive, `0x66` uguale,
    rinfresco ogni 2 s), i frame dei sensori e i pacchetti del lidar;
  - **Pausa** (barra spaziatrice) ferma la *vista* (la simulazione continua: fermarla manderebbe il driver in guasto).
    Con ◀ ▶ o con le frecce si scorre un messaggio alla volta, e griglia, comandi e lidar mostrano quell'istante. Un clic
    su una riga della lista ci salta.

L'indirizzo può aprire direttamente una scheda o una selezione: `#messaggi`, `#robot`, `#sel=4`, `#sel=robot`.

| Comando | Cosa fa |
|---|---|
| `./rtes.sh sim` | simulatore (soggiorno) + pagina + `rtes-driver` da tastiera |
| `./rtes.sh sim sim/stanze/vuota.txt build/host/mio-programma` | un'altra stanza e il **tuo** programma, sui core 0–3 come sul robot |
| `./rtes.sh sim-solo [stanza] [opzioni]` | solo simulatore e pagina (es. `--porta 8081`, `--batteria-veloce 50`, `--senza-ritardi`) |
| `./rtes.sh valida-sim [c2 c2b c3 c1]` | confronto con le registrazioni del robot |
| `./rtes.sh taratura` | ricalcola `sim/parametri.txt` dalle registrazioni |
| `./rtes.sh prova-pagina` | prova automatica della pagina (serve Brave o Chromium) |

`./rtes.sh sim` mette le pseudo-seriali in `RTES_GD32` e `RTES_LIDAR`, e `drv_config_default()` le usa al posto di
`/dev/ttyS3` e `/dev/ttyS1`.

**Tempi.** Nel simulatore i task girano con le stesse priorità, gli stessi core e lo stesso SCHED_FIFO del robot, e i
frame arrivano con i ritardi misurati. La **logica** e le **attivazioni** sono quindi realistiche. I **tempi di
esecuzione** sul PC invece sono più corti che sul robot: il fattore si misurerà con il robot. WCET e analisi si fanno con
le misure prese sul robot.

## 13. Scrivere un task (fase 4)

Guida pratica passo per passo, con l'esempio `tasks/primo.c` e il riferimento di sensori e attuatori: **`docs/TASK.md`**.

Un programma con i tuoi task ha questa forma (i nomi sono esempi):

```c
#include "driver/driver.h"

static rt_task_t moto, mappa;

static void job_moto(void *arg)            /* sporadico: gira a ogni frame dei sensori (50 Hz) */
{
    drv_sensori_t s;
    drv_sensori(&s);                       /* copia coerente dell'ultimo frame, senza mutex */
    double v = 0.15, w = 0.0;
    /* ... odometria con s.s.encoder_sx / encoder_dx e s.s.angolo[0], controllo ... */
    drv_velocita(v, w);                    /* il driver applica i limiti di sicurezza */
}

static void job_mappa(void *arg)           /* periodico: 200 ms */
{
    static lidar_giro_t g;                 /* ~4 KB: static, non sullo stack */
    if (drv_giro(&g))
        for (int i = 0; i < g.n; i++) {
            double x, y;
            if (g.p[i].distanza_mm == 0) continue;     /* nessuna eco */
            drv_punto_xy(&g.p[i], &x, &y);             /* metri, x avanti, y a sinistra */
            /* ... */
        }
}

int main(void)
{
    drv_config_t c;
    drv_config_default(&c);                /* robot, oppure simulatore con ./rtes.sh sim */
    if (drv_avvia(&c)) return 1;
    drv_lidar(1, 6.0);

    rt_task_cfg_t a = { .nome = "moto", .cpu = 1, .sporadico = 1, .periodo_ns = RT_MS(20), .job = job_moto };
    rt_task_cfg_t b = { .nome = "mappa", .cpu = 2, .periodo_ns = RT_MS(200), .job = job_mappa };
    rt_task_cfg_t *tutti[] = { &a, &b };
    rt_priorita_rm(tutti, 2, RT_PRIO_TASK_MAX);       /* Rate Monotonic: periodo più corto = priorità più alta */

    int64_t t0 = rt_adesso() + RT_MS(100);
    rt_task_avvia(&moto, &a, t0);
    rt_task_avvia(&mappa, &b, t0);
    drv_sveglia_ai_sensori(&moto);                    /* sveglia a ogni frame */
    drv_sorveglia(&moto, 1, RT_MS(60));               /* critico: se si blocca, guasto */
    drv_sorveglia(&mappa, 0, 0);                      /* non critico: solo un avviso */

    /* ... aspetta (segnale, durata, comandi) ... */

    drv_dimentica(&moto); drv_dimentica(&mappa);       /* prima di fermarli */
    rt_task_ferma(&moto); rt_task_ferma(&mappa);
    drv_ferma();
    rt_stats_stampa(&moto); rt_stats_stampa(&mappa);
}
```

Cose da sapere:
- **Task sporadico** (`.sporadico = 1`): il job parte a ogni `rt_task_sveglia()`, che il driver chiama a ogni frame
  dei sensori. `periodo_ns` è l'intervallo **minimo** fra due attivazioni: il T dell'analisi. Attenzione: il GD32
  manda ogni 20,0 ms, ma i frame arrivano con qualche ms di variazione (il kernel), quindi l'intervallo minimo vero è
  un po' sotto 20 ms: il driver lo misura (`drv_stato().intervallo_sensori_min_ns`, colonna del CSV di `rtes-driver`). Il rilascio di
  ogni job è l'istante in cui è arrivato il frame, e latenza, risposta e scadenze si misurano da lì. Se un frame arriva
  mentre il job precedente non è ancora partito, l'attivazione si perde e si conta in `attivazioni_perse`.
- **Priorità**: `rt_priorita_rm` le assegna con Rate Monotonic sotto `RT_PRIO_TASK_MAX` (90); il driver sta a 99 su
  CPU0. L'**analisi di schedulabilità** (utilizzazione per core, test di Liu e Layland, tempi di risposta) è tua: per i
  WCET usa `rt_stats_t` (`esec_max_ns`) o i campioni (`cfg.campioni`) misurati **sul robot**.
- **Core**: CPU0 ha il driver e l'interrupt del Wi-Fi; i task vanno su CPU1–3 (partizionati: ogni task su un core).
- **Dati condivisi fra task**: mutex con priority inheritance (`rt_mutex_init`). Fra task e driver niente mutex:
  `drv_*()` non si bloccano mai.
- **Mai** dentro un job: `printf`, `malloc`, file, `sleep`. Per i log usa un buffer e un thread non RT.
- **Comandi**: `drv_velocita(v, w)` (m/s, rad/s), `drv_spazzole(lat, cen)`, `drv_aspirazione(giri)` (serve
  `c.aspirazione = 1`), `drv_led(modo)`, `drv_lidar(acceso, giri_s)`, `drv_esci_dalla_base(1)`, `drv_suono(nome)`.
  Il driver applica sempre i limiti: guarda `drv_stato().limiti` per sapere perché un comando non passa.
- **Unità dei sensori** (`gd32_sensori_t` in `gd32/gd32.h`): encoder in tick (~4580 per metro, `GD32_SCALA_V`),
  imbardata in 0,01° (positiva in senso antiorario, deriva), velocità angolare in 0,01 °/s, tensione in decimi di volt.
