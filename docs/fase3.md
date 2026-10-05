# Fase 3: diario del driver, del framework e del simulatore

Piano della notte: `~/.claude/plans/dobbiamo-fare-la-fase-jolly-beaver.md` (riassunto in `PIANO.md`). Qui c'è quello
che è stato fatto davvero, in ordine, con i risultati, e cosa resta da fare.

## 2026-10-04/05, notte: lavoro autonomo (l'utente dorme)

### Decisioni dell'utente prima di andare
- niente programma di esempio; l'**analisi di schedulabilità la fa l'utente** (il framework dà solo le misure);
- **simulatore con stanza disegnabile** e unit test, per lavorare ai task anche lontano dal robot;
- ruote e spazzole si potevano muovere (robot sollevato su delle scatole), **l'aspirazione mai** (in quelle prove senza
  nessuno presente: nel progetto l'aspirazione si userà); i suoni vanno bene;
- **non rilanciare lo stack originale** (ogni avvio suona a volume alto): al suo posto un **custode**;
- se possibile abbassare il suono di avvio;
- niente commit né push: li fa l'utente.

### Cose successe con l'utente ancora sveglio
- **Suono di avvio**: lo suona RobotApp (`sound_system_power_on.mp3`) con il volume di `device_config.ini`, convertito
  da `CDeviceR16::setSystemVolume(tipo, volume)` in un valore del mixer `speaker volume control` (0–63). Tabella dal
  disassemblato: volume 10 → 50–54, volume 1 → 36–40. Con l'OK dell'utente: **`volume=1`** in
  `/mnt/UDISK/config/device_config.ini` (originale in `device_config.ini.orig-volume`; `.orig` è quella di
  `sleep_enable`). Effetto **non ancora verificato** (serve un riavvio, che di notte non si fa): non è detto che
  RobotApp imposti il volume prima di suonare.
- **Stack originale fermato** (con `spia guida` per la prova veloce) e sostituito dal **custode**: `spia guida
  --comandi /tmp/rtes/comandi`, cioè keepalive 0x0d e heartbeat 0x06 con i motori a zero.
- **Prova dei motori NON fatta.** Il robot è sollevato su scatole che coprono i due anti-caduta **anteriori**: il byte 3
  vale `0x09` (vuoto solo i laterali) e il blocco di sicurezza del banco (che vuole `0x0f`) ha rifiutato giustamente
  ruote e spazzola. Su richiesta dell'utente ho aggiunto un'eccezione esplicita, `sic_cliff_atteso()` /
  `--cliff-atteso 0x09` (in `spia guida` e `rtes-driver --banco`): vale come "sollevato" solo lo schema indicato, e se
  cambia il blocco scatta come prima; senza l'opzione la regola resta `0x0f`. Test in `test_sicurezza.c`
  (`cliff_atteso`). Il modo automatico dei permessi di Claude Code ha però bloccato l'esecuzione della prova sul
  robot; l'utente ha deciso: **niente motori stanotte**.

### Simulatore (`sim/`)
Fatto per primo, su richiesta dell'utente. In C (tempi realistici, nessuna dipendenza), con pagina web servita dallo
stesso programma (sul PC non ci sono Tk né pygame). Dettagli e limiti in `GUIDA.md` sezione 12.
- `sim/mondo.{h,c}`: stanza a griglia da/su testo, raggi (Amanatides–Woo) per il lidar, urti del robot (cerchio),
  base come rettangolo, pennello;
- `sim/gd32sim.{h,c}`: fisica e firmware del GD32 da quello che abbiamo misurato (vedi l'intestazione del file);
- `sim/lidarsim.{h,c}`: lidar con le stesse regole del vero (velocità dalla potenza, 0xAE/0xAD, scarto di 8°);
- `sim/sim.c`: due pseudo-seriali con nomi fissi (`build/sim/gd32`, `build/sim/lidar`), fisica ogni 2 ms, thread web
  (`/`, `/stato`, `/stanza`, `/stanze`, `/comando`);
- `sim/pagina.html`: disegno della stanza, robot, lidar, sensori, comandi; controllata con uno screenshot di Brave
  headless (stanza, punti del lidar, pannello);
- `sim/stanze/`: `vuota` (3 x 3 m) e `soggiorno` (6 x 3,9 m con divano, pianta, muretto, gambe del tavolo basse, scala
  nel vuoto, base).

### Framework (`rt/`)
- **task sporadici** (`cfg.sporadico = 1`, `rt_task_sveglia()`): il rilascio è l'istante della sveglia, le
  attivazioni che arrivano mentre la precedente aspetta si contano come perse;
- campi atomici per il **watchdog** (`battito_ns`, `job_iniziato_ns`, `mancate_di_fila`) e `rt_task_bloccato()`;
- `rt_priorita_rm()`: priorità Rate Monotonic dai periodi (periodi uguali, stessa priorità), sotto
  `RT_PRIO_TASK_MAX` = 90; il driver a `RT_PRIO_DRIVER` = 99;
- `rt_consenti_senza_privilegi()`: sul PC senza permessi per SCHED_FIFO i task partono lo stesso (solo per provare la
  logica). Sul PC di sviluppo `ulimit -r` è 99, quindi di solito non serve.

### Driver (`driver/`)
Interfaccia in `driver/driver.h`, spiegazione in `GUIDA.md` sezioni 11 e 13. Punti di progetto:
- **un solo thread** a priorità 99 su CPU0, che dorme in `ppoll` sulle due seriali e su un `eventfd` (i comandi dei task
  lo svegliano subito) fino al prossimo evento a tempo (ciclo di 20 ms, heartbeat, keepalive);
- **niente I/O lento nel thread del driver**: eventi e suoni passano da un thread di servizio non RT (coda senza
  mutex); il suono parte con `posix_spawn` (`tinyplayer`);
- **scambio senza mutex** (`driver/scambio.h`): doppio buffer con contatore per copia (variante del seqlock); provato con
  1 scrittore e 3 lettori per 0,3 s (1,3 milioni di versioni, nessuna copia incoerente);
- **reazione immediata**: quando arriva un frame dei sensori che cambia il comando permesso (bumper, vuoto) il driver
  manda subito il nuovo `0x66`, senza aspettare il ciclo da 20 ms;
- **seriale chiusa** (succede solo con il simulatore che esce): guasto e la seriale esce dal `ppoll`, niente giri a vuoto.

Problema trovato e risolto: dopo `mlockall(MCL_FUTURE)` lo stack di 8 MB di un thread creato con gli attributi di
default supera il limite di memoria bloccabile di un utente normale (`ulimit -l`), e `pthread_create` fallisce. Il
thread di servizio ora ha 128 KB di stack. (I task hanno già 256 KB, `RT_STACK_DEFAULT`.)

### Prove sul PC
`./rtes.sh test`: unit test (rt 23 + 28, GD32 76, sicurezza 88, lidar 18, registro 23, **logica del driver 50**,
**simulatore 43**), prove di spia, socket, banco e **`tools/prova-driver.py`** (driver contro il simulatore):

| Scenario | Risultato |
|---|---|
| avvio e keepalive (reset del GD32 simulato accorciato a 4 s) | pronto in ~0,2 s, sensori a 50 Hz, nessun reset; senza driver il GD32 simulato si resetta |
| lidar | 6,00 giri/s con potenza 68, giri completi |
| bumper | si ferma contro il muro (x = 2,810, atteso 2,81), poi indietro si può |
| vuoto | si ferma prima del bordo, non cade |
| sollevato | velocità zero, spazzole ferme |
| base | niente ruote, sensori spenti (`0x9d 00`); con `esci_base 1` si allontana e i sensori si riaccendono |
| batteria critica | motori fermi, LED 6 |
| watchdog | task bloccato → guasto in ~0,15 s, LED 8, motori fermi e comandi rifiutati |
| simulatore chiuso | guasto, nessun giro a vuoto |

### Prove sul robot (stack originale fermo, motori vietati con `--senza-motori`)
Robot sollevato sulle scatole, fuori dalla base, ~16,1 V. Script: `tools/driver-robot.sh`.

**r1 — 60 s, driver da solo, lidar spento**: pronto in 0,23 s, versione `2.0.1_19082727`; 2998 frame dei sensori
(50,0 Hz), 0 errati; 21 keepalive, 301 heartbeat; **ritardo massimo del ciclo da 20 ms 32 µs, risveglio più lungo
57 µs**. Alla fine il custode è ripartito da solo.

**r2 — 200 s, lidar acceso, task di prova** (sporadico ai sensori su CPU1, periodico 200 ms su CPU2):
il lidar vero si regola a **6,00 giri/s con potenza 67–68** (come AuxCtrl) e dà **336 punti per giro**, 6 giri al
secondo, come il simulatore. Task svegliato dai sensori: latenza massima ~99 µs, esecuzione ≤ 12 µs, 0 attivazioni
perse.

**r3 — watchdog sul robot**: il task sporadico di prova si blocca apposta (`blocca`): **guasto ~0,1 s dopo** (soglia
100 ms), LED 8, motori fermi; il driver continua a mandare keepalive e heartbeat e il lidar resta regolato; le sveglie
perse si contano (50 al secondo). Il suono `sound_erro_harefault` parte con `tinyplayer` (file presenti), ma **non è
stato verificato a orecchio**. Giro del lidar salvato (`salva_giro`, `build/robot/fase3/giro-r3.txt`): 336 punti,
188 con eco, distanze 0,10–5,97 m, plausibili (robot sulle scatole).

**Unit test ARM sul robot**: tutti i test compilati per ARM eseguiti sul robot, 349 controlli, 0 falliti.

**r4 — 1 ora, driver su CPU0, lidar acceso, task di prova** (`tools/analizza-driver.py build/robot/fase3/r4/r4.csv`):

| Misura | Valore |
|---|---|
| frame dei sensori | 180 005 in 3600 s (50,00 Hz di media, mai sotto 49 in un secondo), 0 errati |
| keepalive / heartbeat | 1201 / 18 002, nessun reset |
| lidar | 6,000 giri/s di media, 21 656 giri |
| task sporadico ai sensori (CPU1, prio 90) | 179 995 job, latenza max 137 µs, esecuzione max 21,5 µs, 0 attivazioni perse, 0 scadenze mancate |
| task periodico 200 ms (CPU2, prio 89) | 18 001 job, latenza max 134 µs, 0 scadenze mancate |
| risveglio più lungo del driver | 125 µs |
| **ritardo massimo del ciclo da 20 ms del driver** | **3491 µs** (a 2995 s); prima 1814 µs (2892 s), 1664 µs (1229 s), 457 µs (462 s); nei primi 6 minuti < 112 µs |
| batteria | 16,0 → 15,6 V |

I picchi del driver non toccano i task su CPU1 e CPU2. Ipotesi: **tutti gli interrupt arrivano su CPU0**
(`/proc/interrupts`: `uart3` del GD32, `sunxi-mmc` del Wi-Fi, `nand`, ...), e durante la prova un controllo dal PC
apriva una sessione SSH ogni 30 s (Wi-Fi e cifratura). Il lavoro degli interrupt e dei softirq su CPU0 ritarda il
driver, che sta sullo stesso core. Per il GD32 3,5 ms non sono un problema (heartbeat 200 ms, keepalive 3 s), ma
nell'analisi di schedulabilità il driver è l'interferenza più alta, quindi conviene che sia regolare.

**r5 — 1 ora, stesse condizioni ma driver su CPU3** (`--cpu 3`): 180 004 frame, 0 errati, 0 scadenze mancate, task ai
sensori con latenza max 108 µs; **ritardo massimo del ciclo del driver 2794 µs** (salti a 413, 530, 1153 e 2121 s).
Quasi uguale a CPU0: **l'ipotesi degli interrupt su CPU0 non regge**. I picchi sono nel *risveglio* del driver (il
lavoro di un risveglio resta sotto 125 µs). Confronto con la fase 0: con il solo AuxCtrl e lo stesso controllo SSH ogni
30 s, `latenza` (che dorme con `clock_nanosleep`) aveva un massimo di 53 µs in un'ora. Differenza principale: il nostro
driver dorme in `ppoll` sulle due seriali. Ipotesi: nel kernel 3.4 i byte ricevuti passano da un `kworker` a priorità
normale (`flush_to_ldisc`), e il `poll` di una tty può dover aspettare i suoi lock: un'inversione di priorità nel
kernel che dura finché il `kworker` non torna a girare.

**r6 — 25 min, driver su CPU0 (lidar, task di prova) e INSIEME `bench/latenza` su CPU3** (priorità 98, periodo 1 ms,
`clock_nanosleep`), con il controllo SSH ogni 30 s:

| Chi | Come dorme | Ritardo massimo del risveglio |
|---|---|---|
| `latenza` su CPU3 (1 490 000 job) | `clock_nanosleep` | **59,7 µs** |
| driver su CPU0 (ciclo da 20 ms) | `ppoll` sulle seriali | **1650 µs** (a 108 e 1064 s) |

Inoltre (misura nuova del driver) l'**intervallo fra due frame dei sensori** arriva a **23,8 ms** invece di 20: anche i
dati del GD32 a volte arrivano con ~4 ms di ritardo, e quello dipende dal percorso dei byte nel kernel (interrupt della
`uart3`, poi `kworker` a priorità normale che li passa alla tty), non dal driver. **Conclusione: il kernel sveglia
puntuale chi dorme su un timer; chi dorme in `ppoll` su una tty no.**

**Rimedio (fatto)**: il driver ora ha **due thread** sullo stesso core. Il thread del **tempo** (priorità 99) dorme con
`clock_nanosleep` e fa tutto ciò che è periodico (heartbeat, keepalive, 0x66 ogni 20 ms, attuatori, lidar, watchdog,
stato). Il thread di **ricezione** (98) dorme in `ppoll` sulle seriali e sull'`eventfd` dei comandi, decodifica,
pubblica, sveglia i task e reagisce subito. Lo stato interno del driver è protetto da un mutex con priority inheritance
usato solo da questi due thread (sezioni di decine di µs); i task restano senza mutex. Il driver misura anche
l'intervallo **minimo** fra due frame: è il T da usare per i task sporadici (dopo un frame in ritardo il successivo
arriva prima di 20 ms).

**r7 — 25 min, stesse condizioni di r6, driver a due thread**:

| Misura | r6 (un thread, `ppoll`) | r7 (due thread) |
|---|---|---|
| ritardo massimo del ciclo da 20 ms del driver | 1650 µs | **109 µs** |
| `latenza` su CPU3 nello stesso momento | 59,7 µs | 51,8 µs |
| intervallo fra due frame dei sensori | fino a 23,8 ms | da **14,9** a 25,1 ms |
| risveglio più lungo del driver | 202 µs | 141 µs |
| frame errati, scadenze mancate, attivazioni perse | 0, 0, 0 | 0, 0, 0 |

Il ciclo del driver ora è puntuale come `latenza`. Restano le variazioni di arrivo dei dati (kernel): per i task
svegliati dai sensori il T minimo da usare nell'analisi è ~15 ms, non 20. Anche `test_rt_task` (con i nuovi test dei
task sporadici) è passato sul robot: 56 controlli, 0 falliti.

**Non provato di notte, da decidere con l'utente**: la modalità "bassa latenza" della seriale (`ASYNC_LOW_LATENCY` con
`TIOCSSERIAL`), che fa consegnare i byte direttamente dall'interrupt senza il `kworker`. Toglierebbe i ~4 ms di ritardo
dei dati, ma con il driver seriale del costruttore e il kernel 3.4 può causare blocchi del kernel (un riavvio, con il
suono di accensione): meglio provarla con l'utente presente.

**r8 — dalle ~05:30 fino al risveglio dell'utente**: driver a due thread, task di prova, **lidar spento** (per la
batteria), motori vietati, nessuna durata. Serve a vedere il driver nuovo per ore. Si ferma con
`tools/driver-robot.sh ferma` (poi riparte il custode), si scarica con `tools/driver-robot.sh scarica r8` e si riassume
con `tools/analizza-driver.py build/robot/fase3/r8/r8.csv`.

### Stato del robot al mattino
- sollevato sulle scatole, **fuori dalla base, non in carica**: alle 05:30 era a 14,8 V (~51%). Va **messo in carica
  appena possibile** (sotto 14,2 V il LED diventa giallo lampeggiante; a ~12 V la protezione lo spegne);
- stack originale fermo dalle 06:25 del 5/10; gira `rtes-driver` (prova r8) e, se termina, il custode;
- in `/mnt/UDISK/config/device_config.ini`: `volume=1` (originale in `device_config.ini.orig-volume`);
- tutto il resto è in `/tmp/rtes` (RAM): un riavvio riporta il robot com'era, con lo stack originale.

## 2026-10-05: simulatore fedele (senza il robot)

Richiesta dell'utente, guardando il simulatore della notte:
1. usare i dati veri del robot invece di quelli inventati;
2. un editor in cui gli oggetti si selezionano, si spostano e si ruotano;
3. vedere che cosa si scambiano driver e GD32: griglia dei 96 byte con le spiegazioni, campi a bit, ultimo comando di
   ogni tipo, pausa, lista con un interruttore per le ripetizioni;
4. poter provare i task nel simulatore come sul robot.
Informazione nuova: **i contatti di carica sono dietro**. Il robot non era disponibile: tutto viene dal firmware, dalle
schede e dalle registrazioni già sul PC. Piano in `~/.claude/plans/dobbiamo-fare-la-fase-jolly-beaver.md`.

**Dati fisici.**
- **Dal firmware** si ricava poco: `CObstacleGeneration` usa una tabella di 360 raggi (un profilo per segnare gli
  ostacoli sulla mappa, da 7 a 28 cm: non è il contorno del robot) e mette il bumper 2 cm oltre il contorno; i default
  del pianificatore (0,15 / 0,2 / 0,05 m) non sono etichettati. Le posizioni dei sensori non sono costanti scritte.
- **Dalle schede** (non ufficiali Proscenic): 349 mm, 95 mm, 3,4 kg, 14,8 V 3200 mAh.
- Tutto in `sim/robot.h` con la fonte; i valori dinamici in `sim/parametri.txt`, tarati sulle registrazioni con
  `tools/taratura/tara.py` (tabella in `GUIDA.md` sezione 12 e in `hardware.md`).
- **Scoperte nuove** dalle registrazioni:
  - il **bit 7 del byte 64 = lidar alimentato**;
  - **1 unità di `0x66` = 1 tick/s** per ruota;
  - il GD32 **applica la velocità 29 ms dopo** il comando;
  - la **rampa in rotazione è ~7 volte più lenta** di quella lineare;
  - senza heartbeat la potenza va via dopo **2,04 s** (non 1,9) e le ruote si fermano per inerzia;
  - la **deriva dell'IMU è −0,3°/h** (il simulatore della notte la aveva 100 volte più grande).

**Simulatore riscritto.**
- `sim/mondo` a oggetti, con le quote: corpo / torretta / piano del lidar / sopra; vuoti a poligono; soglie sotto 2 cm
  superabili.
- `sim/gd32sim` con la dinamica tarata:
  - frame con il ritardo di consegna misurato;
  - urti del corpo (bumper) distinti da quelli della torretta (incastro senza bumper);
  - contatti dietro;
  - con i sensori spenti il GD32 ignora gli anti-caduta (serve per uscire dalla base, dove leggono `0f`) [?].
- `sim/lidarsim` tarato.
- `sim/sim.c` con la storia di tutti i messaggi, `/campi`, `/storia`, i file statici di `sim/web/`.
- `gd32/descrizione.c`: la descrizione unica dei campi, controllata contro il decodificatore (`test_descrizione`, 174
  controlli).

**Pagina** (`sim/web/`):
- editor a oggetti: selezione, maniglie, gruppi, pannello delle proprietà, tastiera, annulla/ripeti, strumenti, oggetti
  pronti;
- schede Robot e Messaggi (ispettore con pausa).

Provata con `tools/prova-pagina.mjs`, che pilota Brave headless (protocollo DevTools): clic, trascinamenti, rotazione con
la maniglia e con R, frecce, annulla/ripeti, tavolo in gruppo, robot, muro disegnato, pausa dell'ispettore. Ha trovato due
difetti, corretti:
- annulla perdeva la selezione;
- la maniglia di rotazione di un oggetto vicino al bordo alto finiva fuori dallo schermo: ora si sposta dove si vede.

**Validazione contro il robot vero** (`tools/valida-sim.py`, che rimanda i comandi registrati con i tempi originali):

| Registrazione | Confronto | Risultato |
|---|---|---|
| c2 | encoder | 0,1% |
| c2 | velocità misurata | 16 unità di scarto quadratico medio |
| c2b | encoder | 1% (prima della taratura dell'inerzia: 8%) |
| c3 | spazzola, ventola, correnti | identiche |
| c1 | velocità del lidar | scarto mediano 0,00 giri/s |

La versione breve (c2) è in `make test`.

**Tempi dei task**: `./rtes.sh sim` limita il programma ai core 0–3 come sul robot; i task vedono intervalli fra i frame
realistici (18–22 ms nelle prove, fino a ~15–25 ms raramente).

`./rtes.sh test`: 567 controlli + spia, socket, banco, driver contro simulatore, validazione: tutto verde.

**Prova r8 sul robot**: il robot non risponde più in rete (probabilmente spento dalla batteria scarica). I dati di r8
erano in `/tmp` (RAM) e sono persi; r4–r7 bastano per le conclusioni.

**Batteria come il software originale** (5/10, su richiesta dell'utente): ricavate da everest-server le soglie e la
stima della percentuale dell'originale (dettagli in `hardware.md`, sezione "Batteria"). L'originale decide sulla
percentuale, non sulla tensione: sotto il 15 % non parte (~13,8 V da fermo), sotto l'8 % interrompe il lavoro
(~13,4 V da fermo, ~13,25 V in movimento), protezione a 12,0 V. Decisione dell'utente: **le nostre soglie restano**
(14,2 V avviso, 13,8 V stop, più prudenti), mentre la **percentuale** del driver ora è calcolata come nell'originale
(stesse curve per robot fermo, in movimento e sulla base, stessa media di 100 s, stessa salita in carica), così coincide
con quella che mostrava l'app. Prima era l'autonomia fino a 13,8 V dalla curva c8scarica: 0 % a 13,8 V, dove
l'originale segna ancora 16–22 %. Con l'aspirazione accesa l'originale usa curve diverse, non ancora riprodotte
(da fare: l'aspirazione si userà).
Test nuovi in `test_driver_logica` (curve, discesa senza salti, niente risalite, calo a tempo, carica);
`./rtes.sh test` tutto verde (586 controlli + spia, banco, driver contro simulatore, validazione), pagina OK.

## Decisioni del 5/10 (dopo il simulatore)

- **Soglie della batteria confermate**: avviso sotto 14,2 V, motori fermi sotto 13,8 V. La percentuale è calcolata come
  nell'originale.
- **Quando non gira un nostro programma resta il custode** (keepalive `0x0d` + heartbeat `0x06`, motori a zero): il
  GD32 non resetta il robot e la carica sulla base parte da sola. A progetto finito sul robot resta sempre installato il
  nostro software (fase 6).
- **L'aspirazione si userà** nel progetto. Il "mai" della notte valeva solo per le prove senza nessuno presente. Nel
  driver c'è già il comando (`drv_aspirazione`, `0x68` in giri/min ÷ 10), vietato di default (`drv_config_t.aspirazione`)
  e mai provato sul robot attraverso il driver: va finito (sotto).
- Contatti di carica **dietro** (detto dall'utente): si esce dalla base in avanti.
- Fatto il commit della fase 3 (non pushato). La fase 4 (task) può partire nel simulatore.

## Cosa resta della fase 3

### Da preparare prima, senza il robot

1. **Aspirazione nel driver**:
   - curve della batteria per gli 8 livelli della ventola, ricavate dal firmware come le altre
     (`calculate600PaStateCapacity` … `calculate2600PaStateCapacity`, scelte con i giri della ventola fra 1000 e oltre
     20 000 giri/min);
   - niente aspirazione senza il cassetto della polvere (byte 4 bit 2), come l'originale (eccezione 6);
   - quale comando usa l'originale durante la pulizia (`0x86`, PWM compensato con la tensione, oppure `0x68` in
     giri/min) e con quali livelli: offrire ai task gli stessi livelli;
   - test e simulatore (la ventola è già simulata, tarata su c3 e c3b).
2. **Registrazione dei frame grezzi nel driver** (`rtes-driver --registra NOME.reg`, stesso formato della spia), così le
   prove della sessione servono anche a `./rtes.sh taratura` e `./rtes.sh valida-sim` (per esempio la dinamica delle
   ruote a terra). Oggi si registra solo con `tools/banco-robot.sh`, che alla fine fa ripartire lo stack originale (con
   il suono).
3. **`tools/driver-robot.sh prepara`**: ferma lo stack originale e avvia il custode, in un solo comando (serve all'inizio
   della sessione, perché alla riaccensione del robot riparte lo stack originale).

### Con il robot: piano della sessione

In quest'ordine. Prima di ogni prova: `tools/driver-robot.sh stato`; dopo: `tools/driver-robot.sh scarica NOME`
(i dati vanno in `build/robot/fase3/NOME/`). Ogni prova con il driver si avvia con
`tools/driver-robot.sh avvia NOME [opzioni]` e si comanda con `tools/driver-robot.sh comando "..."`.

1. **Accensione** (~5 min). Il robot si sarà spento o riavviato, quindi all'accensione partirà lo stack originale:
   - ascoltare il **suono di accensione** con `volume=1`. Se è ancora troppo forte, l'unica strada è sostituire il file
     mp3 (decisione dell'utente);
   - `tools/driver-robot.sh prepara` (o a mano: fermare lo stack e `tools/driver-robot.sh custode`).
2. **Sulla base, carica con il nostro driver** (~10 min): `avvia carica1 --senza-motori --task-prova`.
   - Atteso: LED 4 (giallo che respira), sensori spenti (byte 64 bit 5 = 0), percentuale che sale dell'1 % ogni ~100 s.
   - Nel frattempo, **suono di guasto** a orecchio: `comando "blocca"` → guasto, LED 8 e suono; poi `comando "azzera"`.
   - **Anti-caduta con i sensori spenti**: sollevare il robot dalla base e guardare il byte 3 (il simulatore oggi suppone
     che il GD32 li ignori).
3. **Ruote e spazzole, sollevato** (~10 min): `avvia m1 --banco` (con `--cliff-atteso 0x09` se le scatole coprono gli
   anti-caduta anteriori), poi `comando "v -0.05 0"`, `"v 0 0.3"`, `"spazzole 0 30"`, `"stop"`. In avanti, da sollevato,
   il GD32 rifiuta (fase 2).
4. **A terra, prove di sicurezza** (~20 min): `avvia t1` (senza `--banco`), velocità basse (`v 0.1 0`):
   - **uscita dalla base in avanti**: `comando "esci_base 1"`, poi `v 0.1 0`;
   - **bumper**: contro un ostacolo morbido → si ferma, nello stato compare il limite del bumper;
   - **anti-caduta**: verso il bordo di un gradino o un foglio scuro → si ferma;
   - **sollevato mentre va** → si ferma;
   - la **percentuale** in movimento (curva "in movimento").
5. **Aspirazione** (~10 min; serve il **cassetto della polvere** rimesso): a terra, `avvia a1 --aspirazione`, poi
   `comando "aspirazione 8000"`, `12000`, `18000`, `0`, circa 30 s ciascuno.
   - Atteso: giri (byte 69 × 100) e corrente (byte 82) come in c3/c3b, calo della tensione sotto carico, percentuale con
     le curve della ventola.
   - Deve fermarsi sollevando il robot, sulla base e togliendo il cassetto.
6. **Scarto di 8° del lidar** (~10 min): una scatola a distanza e angolo misurati (per esempio 1,00 m davanti e 1,00 m a
   sinistra), `comando "salva_giro /tmp/rtes/giro1.csv"`, scaricare e confrontare l'angolo (`scarto_lidar_cdeg = 800`
   di default).
7. **Registrazioni per il simulatore** (~20 min, con la registrazione grezza del punto 2 della preparazione):
   - **ruote a terra**: gradini di velocità (0,1 / 0,2 / 0,3 m/s) e di rotazione (0,5 / 1 rad/s) su pavimento libero;
   - **IR della base**: robot fermo davanti alla base a 0,5 / 1 / 2 m e a vari angoli (byte 6);
   - poi `./rtes.sh taratura` e `./rtes.sh valida-sim` con le registrazioni nuove.
8. **Misure con il metro** (robot fermo; ~15 min), per togliere i [?] da `sim/robot.h`:
   - altezze di corpo, torretta e piano del lidar (centro della finestra), bordo inferiore e superiore del bumper;
   - posizione del centro del lidar rispetto al centro del robot;
   - posizione degli anti-caduta (foto da sotto con un righello);
   - base: larghezza, profondità, altezza, posizione dei contatti (foto).
9. **Fattore fra PC e robot**: lo stesso programma (i primi task della fase 4) nel simulatore e sul robot
   (`tools/driver-robot.sh programma ...`), confrontando i tempi di esecuzione di `rt_stats_stampa`.
10. **Facoltativo, per ultimo: seriale a "bassa latenza"** (`ASYNC_LOW_LATENCY`). Toglierebbe fino a ~5 ms di variazione
    nell'arrivo dei dati del GD32 (oggi fra 14,9 e 25,1 ms), ma rischia di bloccare il kernel con il driver seriale del
    costruttore: robot sulla base, prima con un'opzione apposta di `rtes-driver`. Se il robot si blocca, al riavvio
    riparte lo stack originale (con il suono).
11. **Fine**: robot sulla base con il custode; dati scaricati; aggiornare `sim/robot.h`, `docs/hardware.md` e questo
    diario; commit.
