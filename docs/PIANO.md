# Piano progetto RTES: task real-time in C sul Proscenic M6 Pro

Obiettivo: un nostro programma C con task real-time, che gira sul SoC del robot e comanda direttamente l'hardware.

## Decisioni prese
- **Niente Valetudo, niente flash.** Abbiamo già root (SSH/adb). Valetudo parla solo con RobotApp, non con l'hardware.
- **Scheduling: `SCHED_FIFO`**, cioè la semantica di FreeRTOS (priorità fissa con preemption).
  Niente `SCHED_DEADLINE`: il kernel 3.4 non lo ha, e le alternative (backport, kernel mainline) costano troppo
  e rischiano di briccare il robot. Priorità assegnate con rate-monotonic, più l'analisi di schedulabilità classica.
- **Multi-core partizionato:** ogni task è fissato a un core (`sched_setaffinity`), e ogni core si analizza come un
  sistema single-core. I dati condivisi tra core si proteggono con mutex o code, mai contando sulle priorità.
- **Ogni task RT** usa `mlockall()` e `clock_nanosleep(TIMER_ABSTIME)`.
- **Sviluppo anche senza robot:** livello HAL con due backend, robot e simulatore 2D, più unit test.
- **Repository git** `Aspirapolvere-RTES`, privata su GitHub, senza GitHub Actions. I test si lanciano a mano
  con `./rtes.sh` prima di ogni commit.
- **Persistenza senza flash:** binari in `/mnt/UDISK/rt/` e script in `/etc/init.d/` (overlay ext4 su `rootfs_data`).
  L'avvio è sicuro: parte solo se esiste `/mnt/UDISK/rt/enable`, con un ritardo, e se il nostro programma termina
  in modo anomalo si torna ad AuxCtrl.

## Ancora da decidere
- **Criterio di partizionamento dei task sui core:**
  - per carico: i task veloci su un core, quelli pesanti (lidar, navigazione) su un altro;
  - per criticità: i task hard real-time su un core, quelli soft real-time sugli altri.

  La scelta si fa quando conosceremo WCET e periodi reali. Si possono anche confrontare le due soluzioni con le
  misure. In ogni caso Linux e i servizi non RT restano sui core 0–1.

## Architettura trovata (dal backup, 2026-10-01)
```
 RobotApp (SLAM, pianificazione, cloud)          <- da sostituire con il nostro codice
     |  socket Unix /tmp/AuxEverest_Sock
 AuxCtrl (driver: heartbeat, sensori, lidar)     <- da sostituire con il nostro codice
     |  /dev/ttyS3            |  /dev/ttyS1
 MCU GD32F103 (motori,        Lidar 3irobotix LDS3
 bumper, cliff, batteria)
```
- SoC Allwinner A33 (4x Cortex-A7), Linux 3.4.39 `SMP PREEMPT` (non RT), HZ=100, timer ad alta risoluzione, musl.
- AuxCtrl e RobotApp sono binari C++ con i simboli; sul robot ci sono `strace` e `gdbserver`.
- Il contenuto estratto dal backup è in `~/Aspirapolvere/Estratto/` (fuori dalla repository: contiene binari proprietari).
- Vincolo critico: senza heartbeat verso il GD32 il robot si resetta dopo ~4 minuti.

## Fasi
0. **Toolchain e valori di riferimento RT:** cross-compiler armv7 musl statico, `qemu-user-static` sul PC, hello world
   sul robot. Misura di latenza e jitter di un task `SCHED_FIFO`, su core isolato e con carico sugli altri.
   Verifica di `PTHREAD_PRIO_INHERIT` con la nostra musl.
   *Stato al 2026-10-01: quasi completata, vedi `docs/fase0.md`. Latenza massima 38 µs in 60 s, 0 deadline
   mancate. Mancano la misura lunga, quella con traffico Wi-Fi e i test sotto qemu.*
   *Stato al 2026-10-02: misure lunghe e Wi-Fi fatte (`docs/misure-lunghe.md`): 5 h, 0 deadline mancate, max
   455,5 µs nel caso peggiore. Unit test eseguiti direttamente sul robot (51/51), qemu non serve più.*
1. **Reverse del protocollo GD32 e lidar:** Ghidra su AuxCtrl più `strace -x` sui suoi read/write.
   Il risultato va in `docs/protocollo.md`.
   *Stato al 2026-10-02: passi 1 e 2 fatti senza robot (catture strace + disassemblato con
   `tools/disassembla.sh`). Noti: formato del frame e checksum, nomi e codici di 24 comandi, heartbeat `0x06`
   ogni 200 ms, velocità `0x66` ogni 20 ms (con conversione in tick), sequenza di avvio, campi della batteria
   nel frame dei sensori `0x15`. Mancano le prove sul robot per gli altri campi dei sensori, e il lidar.
   Prima delle prove: la fase trasversale "accesso comodo".*
2. **Driver minimo:** il nostro programma al posto di AuxCtrl, con heartbeat e lettura dei sensori. Nessun reset in 10 minuti.
3. **Attuazione:** ruote, spazzole, aspirazione, con il robot sollevato.
4. **Architettura RT:** task, partizionamento sui core, misure di WCET e jitter, analisi di schedulabilità.
5. **Comportamento** (evitamento ostacoli, segui-muro), poi installazione persistente e relazione.

Fasi trasversali: simulatore (dalla fase 0, poi a livello di protocollo dopo la fase 1) e accesso comodo
(Wi-Fi di casa persistente, avvio automatico di AuxCtrl, SSH solo con chiave).

## Bozza dei task
| Task | Tipo | Periodo/deadline | Criticità |
|---|---|---|---|
| Comunicazione GD32 (heartbeat, ricezione sensori) | periodico | ~10-20 ms (da misurare) | hard |
| Sicurezza (bumper, cliff, ruota sollevata, stop motori) | sporadico | deadline stretta | hard |
| Controllo moto (velocità ruote) | periodico | 20 ms | hard |
| Acquisizione lidar | periodico, guidato dai dati | ~1 giro (~200 ms) | soft |
| Navigazione / comportamento | periodico | 100 ms | soft |
| Telemetria e log via Wi-Fi | aperiodico | best effort | non RT (SCHED_OTHER) |

## Comandi (`./rtes.sh`)
| Comando | Cosa fa |
|---|---|
| `test` | compila per il PC ed esegue gli unit test |
| `sim` | esegue i task nel simulatore e salva la traccia |
| `build` | compila per il robot (ARM, statico) |
| `deploy` | copia il binario in `/tmp` sul robot e lo avvia (solo in RAM) |
| `install` / `uninstall` | installa o rimuove la versione persistente con avvio sicuro |
| `ssh` | apre una shell sul robot |
