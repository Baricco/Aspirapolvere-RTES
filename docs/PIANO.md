# Piano progetto RTES: task real-time in C sul Proscenic M6 Pro

Obiettivo: un nostro software C con task real-time, che gira sul SoC del robot, comanda direttamente **tutto**
l'hardware (non solo il minimo) e si controlla da un'app: avanti, indietro, rotazione, aspirazione, telemetria.
I task dell'esame li scrive l'utente da solo, sopra un framework e un driver preparati insieme.

## Decisioni prese
- **Niente Valetudo, niente flash.** Abbiamo già root (SSH/adb). Valetudo parla solo con il software originale.
- **Linux resta il sistema operativo** (kernel 3.4.39 `SMP PREEMPT`, non RT): niente cambio di OS né di kernel.
  Niente `SCHED_DEADLINE` (il kernel 3.4 non lo ha).
- **Ogni task RT** usa `mlockall()` e risvegli ad istanti assoluti (`clock_nanosleep(TIMER_ABSTIME)` o equivalente).
- **Sviluppo anche senza robot:** HAL con due backend, robot e simulatore, più unit test.
- **Repository git** `Aspirapolvere-RTES`, privata su GitHub, senza GitHub Actions. `./rtes.sh test` prima di ogni
  commit.
- **Persistenza senza flash:** binari in `/mnt/UDISK/rt/`, script in `/etc/init.d/` (overlay ext4).
- **All'accensione, a lavoro finito, parte il nostro software**, con interruttore di sicurezza
  (`/mnt/UDISK/rt/enable`) e ritorno allo stack originale se il nostro non parte.
- **Gestione della batteria nel driver:** il driver espone percentuale vera, corrente, temperatura e controllo
  della carica già pronti; i task usano solo i valori.
- **Robustezza:** stato sicuro all'avvio (motori, spazzole, aspirazione a zero prima di tutto), watchdog dei task
  (se un task critico salta le scadenze o non risponde, tutti i motori si fermano) e **stato di guasto ben
  riconoscibile** (colore del LED + suono dall'altoparlante). Il rilancio automatico del programma si aggiunge in
  futuro.
- **Mappatura:** griglia di occupazione + allineamento delle scansioni lidar con un numero fisso di iterazioni,
  tentata come task **hard real-time** periodico (un giro del lidar); se non regge le scadenze, passa al server
  aperiodico.
- **Niente serbatoio dell'acqua:** pompa e sensori del serbatoio restano solo documentati.

## Ancora da decidere
- **Framework RT** (da chiarire nella fase 3). Requisiti: real-time **vero, non solo nel nome**; didatticamente
  sensato (per questo l'idea di FreeRTOS); sfruttare al massimo l'hardware (4 core). Candidati:
  - FreeRTOS ufficiale con il port POSIX: API identica, **un solo core**, niente interrupt veri, chiamate bloccanti
    e `printf` da gestire, stack minimi (`PTHREAD_STACK_MIN`). La documentazione ufficiale dice che "non si comporta
    da sistema real-time"; ma con il tick su un thread `SCHED_FIFO` a priorità massima e risvegli assoluti
    (modifica di 10 righe) abbiamo misurato sul robot **−11…+13 µs di ritardo e 21 µs di jitter** (task a 10 ms,
    3000 risvegli, task a bassa priorità che consuma CPU sullo stesso core) — 2026-10-02;
  - uno strato con API stile FreeRTOS sopra i thread `SCHED_FIFO` di Linux (4 core, ma scheduler di Linux);
  - un ibrido: FreeRTOS su un core per i task dell'esame, thread Linux per driver e rete.
- **Server aperiodico** per i comandi dell'app: polling, deferrable o sporadic server (confrontabili con le misure).
- **Tipo di app** (pagina web, Android, programma sul PC).
- **Partizionamento dei task sui core** (per carico o per criticità), quando conosceremo WCET e periodi.

## Architettura del software originale (dettagli in `RIEPILOGO.md`)
```
GD32 ⇄ /dev/ttyS3 ⇄ AuxCtrl ⇄ /tmp/AuxEverest_Sock ⇄ everest-server ⇄ /tmp/everest_sock ⇄ RobotApp ⇄ cloud
lidar → /dev/ttyS1 → AuxCtrl
```
- **AuxCtrl**: driver delle periferiche (seriali, keepalive `0x0d`, lidar, audio).
- **everest-server**: il cervello (stati, movimento, SLAM, eccezioni, protocollo del GD32). Esporta ~10.500 nomi di
  funzione: è la fonte principale per completare il protocollo.
- **RobotApp**: app, cloud, mappe, sospensione, suoni.
- Il watchdog del GD32 sorveglia solo `0x0d`: senza, reset del robot dopo ~190 s.
- Il nostro software deve replicare AuxCtrl e la parte di everest-server che parla con il GD32.

## Fasi
0. **Toolchain e valori di riferimento RT** — FATTA. Latenza max 38 µs in 60 s; 5 h di misure lunghe con 0 deadline
   mancate (max 455,5 µs nel caso peggiore). `docs/fase0.md`, `docs/misure-lunghe.md`.
1. **Reverse dei protocolli e accesso comodo** — FATTA. Spia sulle seriali e sui socket, frame dei sensori mappato
   con prove fisiche, canale AuxCtrl ↔ everest-server, watchdog. `docs/fase1.md`, `docs/RIEPILOGO.md`.
2. **Caratterizzazione completa dell'hardware** (insieme, PROSSIMA) — ogni sensore, attuatore e comando capito e
   provato: disassemblato di everest-server, lidar (scansioni), ruote, spazzole, aspirazione, comandi di avvio,
   carica, batteria (corrente, temperatura, percentuale), IMU, IR analogici, audio. Banco di prova con interblocco
   di sicurezza. Passi in `docs/fase2-piano.md`.
3. **Driver e framework RT** (insieme) — il nostro driver al posto di AuxCtrl + parte di everest-server (keepalive,
   avvio, sensori decodificati, attuatori, lidar, audio, batteria, stato sicuro, stato di guasto); scelta e
   preparazione del framework RT (vedi "Ancora da decidere"), con esempi e guida.
4. **Task RT** (utente, da solo) — sicurezza, controllo del moto, lidar, comportamento, watchdog dei task, mappatura
   in hard RT; WCET, jitter, analisi di schedulabilità.
5. **App e server aperiodico** (insieme) — scelta del tipo di app e del server; comandi e telemetria.
6. **Installazione persistente e relazione** — avvio automatico del nostro software con ritorno all'originale,
   eventuale rilancio automatico, SSH solo con chiave, documentazione finale.

Fasi trasversali: simulatore (a livello di protocollo, con i dati delle catture) e test sul PC.

## Bozza dei task (da rivedere nella fase 4)
| Task | Tipo | Periodo/deadline | Criticità |
|---|---|---|---|
| Comunicazione GD32 (sensori ogni 20 ms, keepalive `0x0d` ogni 3 s) | periodico | 20 ms | hard |
| Sicurezza (bumper, anti-caduta, stop motori) | periodico/sporadico | entro un periodo dei sensori | hard |
| Controllo moto (velocità `0x66`) | periodico | 20 ms | hard |
| Watchdog dei task + stato di guasto | periodico | ~50 ms | hard |
| Acquisizione lidar | periodico, guidato dai dati | ~1 giro (~200 ms) | soft/hard |
| Mappatura (griglia + allineamento) | periodico | 1 giro del lidar | hard (tentativo) |
| Batteria (nel driver) | periodico | ~1 s | soft |
| Comandi dall'app | aperiodico, tramite server | budget del server | soft |
| Telemetria e log | aperiodico | best effort | non RT |

## Comandi (`./rtes.sh`)
| Comando | Cosa fa |
|---|---|
| `test` | compila per il PC ed esegue unit test e prove della spia |
| `build` | compila per il robot (ARM, statico) |
| `deploy` | copia il binario in `/tmp/rtes` sul robot e lo avvia (solo in RAM) |
| `scarica` | copia un file dal robot |
| `ssh` | apre una shell sul robot |
| `sim`, `install`, `uninstall` | da fare |
