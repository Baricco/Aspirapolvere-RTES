# Piano progetto RTES: task real-time in C sul Proscenic M6 Pro

Obiettivo: un nostro software C con task real-time, che gira sul SoC del robot, comanda direttamente **tutto**
l'hardware (non solo il minimo) e si controlla da un'app: avanti, indietro, rotazione, aspirazione, telemetria.
I task dell'esame li scrive l'utente da solo, sopra un framework e un driver preparati insieme.

## Decisioni prese
- **Niente Valetudo, niente flash.** Abbiamo già root (SSH/adb). Valetudo parla solo con il software originale.
- **Linux resta il sistema operativo** (kernel 3.4.39 `SMP PREEMPT`, non RT): niente cambio di OS né di kernel.
  Niente `SCHED_DEADLINE` (il kernel 3.4 non lo ha).
- **Ogni task RT** usa `mlockall()` e risvegli ad istanti assoluti (`clock_nanosleep(TIMER_ABSTIME)` o equivalente).
- **Sviluppo anche senza robot:** simulatore **a livello di seriali** (`sim/`, fase 3): finge di essere il GD32 e il lidar
  su due pseudo-seriali, quindi driver e task girano sul PC senza cambiare una riga (niente HAL separata). Stanza
  disegnabile da una pagina web. Più unit test e prove d'insieme. La prova finale si fa sempre sul robot.
- **Repository git** `Aspirapolvere-RTES`, privata su GitHub, senza GitHub Actions. `./rtes.sh test` prima di ogni
  commit.
- **Persistenza senza flash:** binari in `/mnt/UDISK/rt/`, script in `/etc/init.d/` (overlay ext4).
- **All'accensione, a lavoro finito, parte il nostro software**, con interruttore di sicurezza
  (`/mnt/UDISK/rt/enable`) e ritorno allo stack originale se il nostro non parte.
- **Gestione della batteria nel driver:** il driver espone tensione, temperatura, stato di carica e la percentuale
  stimata come il software originale (il GD32 non dà la corrente); le soglie di sicurezza (fermare i motori, LED 6)
  stanno nel driver; i task usano solo i valori.
- **Quando non gira un nostro programma** gira il custode (keepalive e heartbeat al GD32, motori a zero): il robot non
  si resetta e sulla base si carica. A progetto finito sul robot resta installato il nostro software.
- **Robustezza:** stato sicuro all'avvio (motori, spazzole, aspirazione a zero prima di tutto), watchdog dei task
  (se un task critico salta le scadenze o non risponde, tutti i motori si fermano) e **stato di guasto ben
  riconoscibile** (colore del LED + suono dall'altoparlante). Il rilancio automatico del programma si aggiunge in
  futuro.
- **Mappatura:** griglia di occupazione + allineamento delle scansioni lidar con un numero fisso di iterazioni,
  tentata come task **hard real-time** periodico (un giro del lidar); se non regge le scadenze, passa al server
  aperiodico.
- **Niente serbatoio dell'acqua:** pompa e sensori del serbatoio restano solo documentati.

- **Architettura RT (decisa il 2026-10-03, fine della fase 2):**
  - **driver** = livello fisso a **priorità massima, fuori dallo scheduling dei task**, su un core suo: seriale del
    GD32 (keepalive, heartbeat, avvio, decodifica), blocco di sicurezza (`gd32/sicurezza.h`), lidar con la sua
    regolazione di velocità (come AuxCtrl), soglie della batteria e **watchdog dei task** (deve stare sopra i task
    che sorveglia: con RM un watchdog a 50 ms non girerebbe se un task a 20 ms si blocca sullo stesso core);
  - **task con Rate Monotonic, partizionato** sui core (ogni task legato a un core; su ogni core vale l'analisi a un
    processore: Liu e Layland, tempi di risposta), direttamente con i thread `SCHED_FIFO` di Linux e la libreria
    `rt/`. Scartati: FreeRTOS POSIX (un solo core, scheduler simulato con i segnali, non più RT del Linux sotto),
    EDF sopra `SCHED_FIFO` (fattibile con un dispatcher, ma più complesso), RM globale (analisi debole, niente
    garanzie dei protocolli dei mutex fra core);
  - **mutua esclusione con Priority Inheritance** (`PTHREAD_PRIO_INHERIT`, verificato sul robot; `PRIO_PROTECT` non
    esiste in musl). Fra driver e task **nessun mutex**: scambio senza blocchi (doppio buffer con numero di versione),
    così il driver non aspetta mai un task;
  - task a 20 ms **svegliati dall'arrivo del frame** del GD32 (dati freschi, ogni frame una volta sola; nell'analisi
    sono sporadici con T = intervallo minimo), gli altri **a timer**;
  - **server aperiodico** per batteria (lettura), spazzole, aspirazione, LED, comandi da remoto;
  - rete di sicurezza del kernel sui task RT (95% della CPU per secondo) lasciata attiva.

## Ancora da decidere
- **Tipo di server aperiodico** (polling, deferrable, sporadic) e **tipo di app** (con l'home server, più avanti:
  zone proibite, preferenze, programmazione della pulizia).
- **Velocità del lidar** (6 giri/s come AuxCtrl, oppure più veloce con meno punti per giro: ~2050 misure al secondo
  fisse; provare la potenza 100) e periodo della navigazione (100 o 200 ms).
- **Assegnazione dei task ai core**, quando conosceremo i WCET (Worst-Fit sulle utilizzazioni per bilanciare).

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
2. **Caratterizzazione completa dell'hardware** — FATTA (2026-10-03). Disassemblato di everest-server (tutti i
   comandi, ogni byte del frame dei sensori), banco di prova con blocco di sicurezza (`spia guida`,
   `gd32/sicurezza.h`), prove su lidar, ruote, spazzole, aspirazione, carica, LED, audio, IMU, anti-caduta, IR della
   base. Aperti: carica ferma a 16,3 V (batteria degradata?), curva di scarica, scarto di ~8° del lidar (risolto il 5/10: 14,5°, angoli orari, vedi fase3.md).
   `docs/fase2.md`, `docs/RIEPILOGO.md`.
3. **Driver, framework RT e simulatore** — FATTA (4-6/10, `docs/fase3.md`). Driver (`driver/`: avvio,
   keepalive, heartbeat, sensori, lidar regolato, batteria, limiti di sicurezza "a terra", watchdog dei task, stato di
   guasto con LED 8 e suono), framework (`rt/`: task sporadici, watchdog, priorità RM), simulatore con pagina web,
   guida (`GUIDA.md` sezioni 11–13). Provato sul robot **senza motori** (0 scadenze mancate, ritardo del ciclo
   < 160 µs, poi < 110 µs con il driver a due thread). Il 5/10 il simulatore è diventato fedele: geometria vera, dinamica
   tarata sulle registrazioni e validata contro di esse, stanza a oggetti con editor, ispettore dei messaggi. Il 5-6/10
   prove con il robot a terra: corretti il verso del lidar e le scale di velocità (~3,6 volte), aspirazione con blocco
   senza cassetto e curve della batteria della ventola, IR della base, registrazione grezza nel driver, simulatore
   validato anche a terra. Aperti per le fasi dopo: aggancio alla base con l'IR, gradino.
4. **Task RT** (utente, da solo) — sicurezza, controllo del moto, lidar, comportamento, watchdog dei task, mappatura
   in hard RT; WCET, jitter, analisi di schedulabilità.
5. **App e server aperiodico** (insieme) — scelta del tipo di app e del server; comandi e telemetria.
6. **Installazione persistente e relazione** — avvio automatico del nostro software con ritorno all'originale,
   eventuale rilancio automatico, SSH solo con chiave, documentazione finale.

Fasi trasversali: simulatore (a livello di protocollo, con i dati delle catture) e test sul PC.

## Bozza dei task (da rivedere nella fase 4)
Driver (fisso, priorità massima, fuori da RM): GD32 ogni 20 ms (keepalive 3 s, heartbeat 200 ms), blocco di
sicurezza (bumper, anti-caduta, sensori assenti), lidar e regolazione della sua velocità (20 ms), soglie della
batteria, watchdog dei task + stato di guasto (~50 ms).

| Task (RM partizionato) | Attivazione | Periodo | Cosa fa |
|---|---|---|---|
| Controllo del moto | frame del GD32 | 20 ms | odometria (encoder + imbardata dell'IMU) e v/ω verso l'obiettivo della navigazione, in anello chiuso sulle misure |
| Sicurezza del moto | frame del GD32 | 20 ms | incastrato/slittamento (comandato vs encoder, correnti, IMU), bordo delle zone proibite: ferma il robot |
| Lidar | timer | 200 ms | elaborazione del giro: ostacoli, distanze, dati per mappa e navigazione |
| Navigazione | timer | 200 ms (forse 100) | dove andare: copertura, seguimuro, evitare ostacoli e zone proibite |
| Mappatura | timer | 200 ms (500 se il WCET non ci sta) | griglia di occupazione + allineamento delle scansioni |
| Comandi, LED, spazzole, aspirazione, stato | server aperiodico | budget del server | eventi dall'app e dall'home server |
| Telemetria e log | sfondo | best effort | non RT |

## Comandi (`./rtes.sh`)
| Comando | Cosa fa |
|---|---|
| `test` | compila per il PC ed esegue unit test e prove della spia |
| `build` | compila per il robot (ARM, statico) |
| `deploy` | copia il binario in `/tmp/rtes` sul robot e lo avvia (solo in RAM) |
| `scarica` | copia un file dal robot |
| `ssh` | apre una shell sul robot |
| `sim [stanza] [programma]` | simulatore + pagina web (http://localhost:8080) + `rtes-driver` o il tuo programma |
| `sim-solo [stanza]` | solo simulatore e pagina |
| `install`, `uninstall` | da fare (fase 6) |
