# Protocolli SoC ↔ GD32 e SoC ↔ lidar (fase 1)

I protocolli con cui AuxCtrl parla con il microcontrollore GD32F103 (motori, sensori, batteria, motore del
lidar) su `/dev/ttyS3` e con il lidar su `/dev/ttyS1`. Servono per scrivere il nostro driver al posto di AuxCtrl
(fase 2). La mappa di tutto l'hardware è in `hardware.md`.

Ogni informazione è marcata con la sua fonte, perché hanno affidabilità diverse:
- **[codice]**: letto nel disassemblato di AuxCtrl. È certo.
- **[osservato]**: visto nelle catture (strace, poi la spia). È certo per quelle condizioni (robot fermo sulla base).
- **[ipotesi]**: dedotto dai valori. Va verificato con il robot (passo 3 della fase 1).

Stato al 2026-10-02: fatti i passi 1 (analisi delle catture) e 2 (disassemblato di AuxCtrl). Mancano le catture
mirate (bumper, ruote sollevate, motori) e il lidar.

## Strumenti

| Strumento | Cosa fa |
|---|---|
| `tools/estrai-frame.py <traccia>` | ricompone i frame dai `read`/`write` di una traccia strace di AuxCtrl, verifica i checksum e stampa un catalogo per codice `cmd` (frequenza, lunghezza, contenuti, byte che cambiano). `--csv` salva tutti i frame |
| `spia` (`spia/spia.c`, `tools/spia-robot.sh`) | si mette in mezzo sulle seriali e registra tutto **senza perdere byte**; `spia riepilogo` e `spia leggi` decodificano il registro con le librerie `gd32/` e `lidar/` (vedi `GUIDA.md`, sezione 9) |
| `tools/disassembla.sh <regex> [binario]` | disassembla le funzioni di AuxCtrl (o di un altro binario) cercandole per nome. Il binario non è in git: sta in `../Estratto/` |

Catture usate (fuori da git, in `~/Aspirapolvere/Log/`):
- `cattura-ricarica/cattura/aux.txt`: 236 s, contiene l'**avvio** di AuxCtrl/RobotApp (handshake) e la messa
  in carica;
- `cattura-stream/20261002-000030.txt.gz`: 325 s a regime, robot sulla base.

## Seriale [codice] [osservato]

`/dev/ttyS3` e `/dev/ttyS1`: **115200 baud, 8N1, raw, senza controllo di flusso** (`spia termios` sul robot,
`cflag=014262`; nel codice `CSerialConnection::set_opt`, chiamata una sola volta). AuxCtrl **non** usa le linee di
controllo: `getCTS`, `getDSR`, `getDCD` e `getRing` esistono ma non sono mai chiamate.

## Formato del frame [codice]

```
 FA FB | len | cmd | dati (len-3 byte) | ck_hi ck_lo
```
- `FA FB`: intestazione fissa.
- `len`: numero di byte che seguono, cioè **cmd + dati + 2 byte di checksum**. Massimo 255.
- `cmd`: codice del messaggio. Le due direzioni usano gli stessi codici per richiesta e risposta (es. `0x07`).
- I dati multi-byte sono **little-endian** (il GD32 e l'ARM sono entrambi little-endian, e AuxCtrl copia i byte
  direttamente in una struct C).
- **Checksum** (`CRobotPacket::calcCheckSum`, indirizzo `0x55cf0`): calcolato su cmd + dati. Si sommano, modulo
  2^16, le parole **big-endian** da 16 bit (`b[0]<<8 | b[1]`, `b[2]<<8 | b[3]`, …). Se il numero di byte è
  dispari, l'ultimo byte **non si somma ma si mette in XOR** con il risultato. Il checksum si trasmette
  big-endian.

```c
uint16_t ck = 0;
size_t i;
for (i = 0; i + 1 < n; i += 2)
	ck += (uint16_t)(b[i] << 8 | b[i + 1]);
if (n % 2)
	ck ^= b[n - 1];
```

  Esempio: il frame di heartbeat è `FA FB 03 06 00 06`: len 3, cmd `0x06`, nessun dato, checksum `0x0006`. Con il
  solo `0x06`, che è un byte dispari, il checksum vale `0 ^ 0x06`.

  Lo XOR finale è il dettaglio che non si indovina dai dati. Con una somma "normale" i frame con un numero
  dispari di byte non tornano, e la ricostruzione dalle catture era fallita proprio per questo.

## Comandi SoC → GD32

Nomi e codici dalle funzioni `CSerialMessagePacket::packet*` di AuxCtrl [codice]. Ogni funzione costruisce il
pacchetto con il codice passato al costruttore `CRobotPacket(cmd, dati, len)`. Le frequenze sono [osservato].

| cmd | Funzione in AuxCtrl | Dati | Visto nelle catture |
|---|---|---|---|
| `0x04` | `packetStm32Sleep` | – | no |
| `0x05` | `packetWakeupAck` | – | no |
| **`0x06`** | **`packetHeartBeat`** | nessuno | **ogni ~204 ms**, sempre |
| `0x07` | `packetRequireSystemVersion` | nessuno | all'avvio (risposta: vedi sotto) |
| `0x08` | `packetSetIMUZero` | nessuno | all'avvio |
| `0x0a` | `packetResetErrorCode` | nessuno | raro |
| `0x65` | `packetMotorControlType` | 1 byte (tipo) | ogni ~100 ms durante l'avvio, poi sparisce; valori `00`, `02` |
| **`0x66`** | **`packetMotorVelocity(v, ω)`** | 2 × int32 | **ogni ~20 ms**, sempre; quasi sempre zero |
| `0x67` | `packetMotorSpeed(sx, dx)` | 2 × int32 | no |
| `0x68` | `packetBlowerSpeed` | 2 byte | raro (`00 00`: aspirazione spenta) |
| `0x69` | `packetBrushSpeed` | 1 byte con segno | raro (`00`) |
| `0x6a` | `packetRollingSpeed` | 1 byte con segno | raro (`00`, spazzola centrale) |
| `0x78` | `packetCliffIRControl` | 1 byte (bool) | all'avvio (`01`) |
| `0x79` | `packetCliffIRDirection` | 1 byte (bool) | no |
| `0x8d` | `packetButtonLEDState` | 1 byte | ogni tanto (`00`, `01`, `08`) |
| `0x97` | `packetLidarPower` | 1 byte (bool) | raro (`00`: lidar spento sulla base) |
| `0x99` | `packetR16Power` | 1 byte (bool) | raro |
| `0x9a` | `packetRestartR16System` | – | no |
| `0x9b` | `packetChargerPower` | 1 byte (bool) | no |
| `0xa1` | `packetIMUCalibaration`, `packetIMUFactoryCalibrate` | – | no |
| `0xa2` | `packetIMUFactoryCalibrateState` | 4 byte | 10 volte (`10 0e 00 00`) |
| `0xa3` / `0xa4` | calibrazione del magnetometro / stato | – | no |

Codici visti nelle catture ma senza una funzione `packet*` con quel nome (li costruisce altro codice: da capire):
`0x0c` (`01`, all'avvio, con risposta `0x0c 01`), `0x17` (`01 80 cf f0`), `0x18`,
`0x19` (`01`), `0x6b` (1 byte), `0x7c` (`00 ff ff`), `0x86` (`00`), `0x9d` (`00`/`01`).

`0x0d` (nessun dato) invece è noto: è il **keepalive di AuxCtrl**, vedi "Il vincolo dell'heartbeat".

`0x71` invece è noto: **potenza del motore del lidar** [codice]. `controlLidarPwm(int p)` limita p a 0..100 e manda
`0x71` con p come int32 little-endian. All'avvio AuxCtrl manda `0x71 00 00 00 00` (motore fermo). Il lidar quindi
lo fa girare il GD32: dal SoC al lidar non passa nessun byte.

### Velocità delle ruote (`0x66`) [codice]
`packetMotorVelocity(double v, double ω)`:
1. limita |v| e |ω| a due massimi letti da un oggetto di configurazione (valori non ancora ricavati);
2. converte in tick dell'encoder al secondo e invia due `int32` little-endian:
   - `v_tick = (int)(v [m/s] × 16561,27)`
   - `ω_tick = (int)(ω [rad/s] × 1999,77)`

I due fattori vengono dai parametri meccanici in `CRobotParams::CRobotParams()` [codice]:

| Parametro | Valore | Significato |
|---|---|---|
| tick per giro della ruota | 3616 | risoluzione dell'encoder |
| raggio della ruota | 34,75 mm | |
| carreggiata | 241,5 mm | distanza tra le due ruote |
| metri per tick | 2π·r / 3616 = 60,4 µm | |
| fattore lineare | 3616 / (2π·r) = 16561 tick/m | |
| fattore angolare | 3616·carreggiata / (4π·r) = 1999,8 tick/rad | |
| altri valori | 0,175 · 0,35 · 0,35 | [ipotesi] raggio del robot (m), velocità massime |

Nelle catture compare `f9 ff ff ff 00 00 00 00`, cioè v = −7 tick/s (−0,4 mm/s) e ω = 0, nei secondi in cui il
robot si sistema sulla base.

## Messaggi GD32 → SoC

| cmd | Dati | Significato | Frequenza [osservato] |
|---|---|---|---|
| **`0x15`** | **96 byte** | **stato dei sensori** (vedi sotto) | **ogni 20,0 ms** (spia: min 17,6, max 22,4) |
| `0x07` | 32 byte | versione del firmware: `"2.0.1_19082727"` in ASCII + `01` al byte 20 | risposta a `0x07` |
| `0x0c` | `01` | risposta a `0x0c` | all'avvio |
| `0x17` | `01` | risposta a `0x17` | all'avvio |
| `0x18` | `04 00 00 00 3f 00 00 00` | risposta a `0x18` | all'avvio |
| `0x08` | `01` | conferma di `0x08` (zero dell'IMU) | all'avvio |
| `0x06` | nessuno | heartbeat del GD32 | **ogni 2,0 s** (spia; con strace sembrava irregolare) |
| `0xa2` | 1 byte | stato della calibrazione dell'IMU | all'avvio |
| `0x04` | nessuno | conferma della sospensione | quando il SoC manda `0x04` |
| `0x05` | nessuno | **risveglio: tasto premuto mentre il robot dorme** | ogni 100 ms finché il SoC non risponde con `0x05` |

### Frame dei sensori (`0x15`, 96 byte)
AuxCtrl (`unpacketCRL200SSensorMessage`) azzera una struct `TSensorMessage` da 88 byte e ci copia i primi 88 byte
dei dati così come sono [codice]. Gli ultimi 8 servono probabilmente alla variante CRL-300. AuxCtrl usa solo i
campi della batteria (`updateBatterySensor`); il resto lo inoltra a RobotApp sul socket
`/tmp/AuxEverest_Sock`. RobotApp non esporta i nomi delle sue funzioni interne: il significato degli altri campi
va ricavato con le prove sul robot.

Offset in byte dall'inizio dei dati (dopo `cmd`). **[prova]** = verificato con le prove fisiche del 2026-10-02
(spia + `tools/sensori.py` / `tools/ritmo.py`, registrazione `build/robot/spia/20261002-prove-sensori.spia`).

| Offset | Tipo | Valori visti | Significato |
|---|---|---|---|
| 0 | uint8 | `01` | [ipotesi] stato/modalità |
| **1** | bit | `00` libero, `06` sulla base | **[prova] bumper: bit 2 (`0x04`) = microinterruttore sinistro, bit 1 (`0x02`) = destro**, 1 = premuto. Sulla base il robot è spinto contro la base e li tiene premuti tutti e due (`06`). Una pressione al centro può far scattare uno solo dei due |
| 2 | uint8 | `00` | |
| **3** | bit | `0f` in aria | **[prova] sensori anti-caduta digitali, 1 = nessun pavimento**: bit 3 (`0x08`) il più a sinistra, bit 2, bit 1, bit 0 (`0x01`) il più a destra (sono 4, sul semicerchio anteriore). RobotApp li usa anche per capire che il robot è sollevato (`m_wheel_up_CliffIR_filter`) |
| **4** | bit | `04` / `00` | **[prova] bit 2 (`0x04`) = cassetto della polvere presente**. Tolto → RobotApp segnala l'eccezione 6 `EXCEPTION_DUSTBOX_INEXISTENCE` e mette il LED a `08` |
| **7** | bit | `00`, `01`, `02` | **[codice] [prova] bit 0 = in carica, bit 1 = sulla base / alimentatore collegato** (AuxCtrl considera "sulla base" anche quando bit 0 = 1). `01` sulla base in carica, `00` fuori. Con contatti di carica instabili cambia di continuo |
| **8** | uint8 | `a2`–`a5` | **[codice] tensione della batteria × 10**: 16,5 V in carica, 16,2–16,4 V fuori dalla base. La notte del 2026-10-01, con la batteria in protezione, era 11,3–14 V. [ipotesi] Li-ion 4S, 16,8 V piena |
| 9 | uint8 | `1f`–`20` | [ipotesi] temperatura della batteria in °C (31–32) |
| **16–19** | int32 | `0x6930`… | **[prova] encoder della ruota sinistra** (contatore assoluto: cambia girando la ruota a mano; scende quando la ruota gira all'indietro) |
| **24–27** | int32 | `0x693c`… | **[prova] encoder della ruota destra** |
| 40–43 | int32 | varia ruotando il robot | [ipotesi] angolo integrato dal giroscopio (sale mentre il robot ruota, resta fermo dopo) |
| 44–55 | 6 × int16 | piccoli, rumorosi | [ipotesi] IMU: 44/48 rumore del giroscopio, 46/50 cambiano inclinando il robot (accelerometro) |
| 56 | uint8 | `64` (100) | [ipotesi] percentuale della batteria; si azzera e risale (00→19→4b→64) al risveglio |
| **58** | uint8 | 0 → 0a | **[prova] tasto ACCENSIONE: durata della pressione in unità di 10 ms**, torna a 0 al rilascio. [codice] AuxCtrl moltiplica 58, 60 e 62 per 10 |
| 60 | uint8 | 0 | [ipotesi] tasto "Punto" (`detectDockOrPointButtonPress`): stesso formato, il modello M6 Pro non ce l'ha |
| **62** | uint8 | 0 → 0a | **[prova] tasto CASA**, stesso formato del 58 |
| 64 | bit | `42` sulla base, `62` fuori | [ipotesi] bit 5 (`0x20`) = fuori dalla base / nessun segnale della base |
| **65** | uint8 | `00`, `01`, `04`, `08` | **[prova] stato del LED del tasto: ripete l'ultimo comando `0x8d`** (vedi i codici del LED) |
| 66 | uint8 | `00`/`02` | cambia all'avvio |
| 78, 80, 82, 84 | 4 × int16 | 1–4, 0x31–0x41, 0x0c, 2–3 | [ipotesi] valori analogici di 4 sensori IR: NON reagiscono al foglio sotto i sensori anti-caduta. Forse sensori laterali/muro |
| 88–89 | 2 byte | `03 19` | costante; `89` va a 0 per un attimo al risveglio |
| 90–95 | 3 × int16 | uguali a 94/92/44 | copia di alcuni campi dell'IMU |
| altri | | sempre 0 | da scoprire con i motori in funzione (correnti, spazzole, aspirazione) |

**Non esistono** (verificato): un interruttore sulla torretta del lidar e interruttori di "ruota sollevata"
(spingendo una ruota fino a fine corsa non cambia nessun bit).

### Codici del LED del tasto (`0x8d`, SoC → GD32) [prova]

| Codice | LED | Quando |
|---|---|---|
| `00` | spento | prima della sospensione |
| `01` | azzurro chiaro fisso | fermo, tutto a posto |
| `04` | giallo | in carica sulla base |
| `08` | rosso lampeggiante | errore (es. cassetto tolto) |

### Sospensione e risveglio [osservato]

Con `sleep_enable=1` in `/mnt/UDISK/config/device_config.ini` RobotApp addormenta il robot dopo `sleep_time` = 5 minuti
fermo fuori dalla base (dal 2026-10-02 la sospensione è disattivata: vedi `fase1.md`).
```
sospensione  SoC → GD32: 0x8d 00 (LED spento), 0x97 00 (lidar), 0x65 00, 0x68 00 00, 0x69 00, 0x6a 00, 0x6b 00,
                         0x0c 00, 0x04 (sleep)
             GD32 → SoC: 0x0c 01, 0x04; poi NESSUN frame dei sensori
risveglio    GD32 → SoC: 0x05 (alla pressione del tasto; ogni 100 ms finché il SoC non risponde)
             SoC → GD32: 0x0c 01, 0x08 (zero dell'IMU), 0x05 (conferma), 0x8d 01; poi tornano i sensori
```
Il risveglio NON rimanda `0x78 01` (infrarossi anti-caduta), che AuxCtrl manda solo al proprio avvio.
`0x0c 01` / `0x0c 00` sembra quindi "SoC attivo / SoC in sospensione".

`0x9d` (SoC → GD32, 1 byte): `01` quando il robot perde il contatto di carica, `00` quando torna sulla base.
[ipotesi] abilita qualcosa legato alla carica o all'uscita dalla base.

## Lidar 3irobotix LDS3 (`/dev/ttyS1`)

Ricavato dal disassemblato di AuxCtrl (`CLidarPacketReceiver`, `CLidarPacket`, `C3iroboticsLidar`) e verificato sui
byte veri. Implementato in `lidar/lidar.h`.

**Chi lo comanda.** Il motore del lidar lo alimenta (`0x97`) e lo regola (`0x71`, potenza 0–100) il **GD32**. Il SoC
riceve soltanto: in nessuna cattura un byte è andato dal SoC al lidar.

**Avvio [osservato].** Appena acceso, il lidar stampa un testo in cinese (codifica GBK):
```
深圳市杉川机器人科技有限公司        (Shenzhen 3irobotix Co., Ltd.)
软件作者：JL Wan                   (autore del software)
固件版本: LDS3_NOR_May  4 2018 15:36:34   (versione del firmware)
```
Poi manda solo pacchetti binari. Il primo è la versione: `...LDS3_NOR_1805041536`.

**Formato del pacchetto [codice] [osservato]:**
```
 AA | len_hi len_lo | indirizzo | tipo | cmd | n_hi n_lo | parametri (n byte) | ck_hi ck_lo
```
- `len` = byte del pacchetto **senza** i 2 del checksum (= 8 + n). AuxCtrl accetta solo `len_hi = 0` e `len_lo` tra 6
  e 100 (`processStateHeader2`, `processStateLength`). Numeri multi-byte **big-endian** (`bufToUByte2`).
- **Checksum** su tutti i byte da `AA` all'ultimo parametro, scelto dal campo indirizzo (`processStateAcquireData`,
  `verifyCheckSum`):
  - indirizzo = 1 → **somma dei byte** modulo 2^16 (`calcCheckSumSum`);
  - altrimenti → **CRC-16/MODBUS** (`calcCheckSumCRC`, a tabella: le due tabelle in AuxCtrl, a `0x6c120` e
    `0x6c220`, coincidono con quelle Modbus standard. Verificato con 5000 sequenze casuali).
- Esempio vero: `AA 00 33 01 61 AC 00 2B "2030313958425713002F0024LDS3_NOR_1805041536" 0B B0`: len 51,
  indirizzo 1, tipo 0x61, cmd 0xAC, 43 byte di parametri, somma = 0x0BB0.

**Codici** (`C3iroboticsLidar::analysisPacket`, salto su `cmd - 0xA9`) [codice]:

| cmd | Funzione | Significato |
|---|---|---|
| `0xA9` | `analysisToothScan` → `unpacketLidarScan`/`Scan2` | un settore di misure (formato vecchio) |
| `0xAA` | – | ignorato |
| `0xAB` | `analysisHealthInfo` | stato di salute del lidar |
| `0xAC` | `analysisLidarVersion` | versione (testo ASCII) |
| `0xAD` | `analysisNewToothScan` → `unpacketNewLidarScanHasSingal`/`NoSingal` | un settore di misure (formato nuovo, con o senza intensità del segnale) |
| `0xAE` | `analysisLidarSpeed` | velocità di rotazione |

Il contenuto dei pacchetti di scansione (angoli, distanze, intensità) va ricavato dalle funzioni `unpacket*` e
verificato con il lidar in rotazione (robot fuori dalla base).

## Canale AuxCtrl ↔ everest-server ↔ RobotApp (`/tmp/AuxEverest_Sock`)

Catturato il 2026-10-02 con `spia socket` (`tools/spia-robot.sh socket`; registro
`build/robot/spia/20261002-socket.spia`, 129 s, 0 errori). Implementato in `spia/everest.h`.

**Chi parla con chi** [robot] [codice]. Il percorso `/tmp/AuxEverest_Sock` compare solo in AuxCtrl (server, in
ascolto) e in **everest-server** (client), non in RobotApp. everest-server fa da smistatore: è anche client di
`/tmp/everest_sock`, su cui RobotApp è in ascolto, ed è in ascolto su TCP 0.0.0.0:6000.
```
GD32 ⇄ /dev/ttyS3 ⇄ AuxCtrl ⇄ /tmp/AuxEverest_Sock ⇄ everest-server ⇄ /tmp/everest_sock ⇄ RobotApp
lidar → /dev/ttyS1 → AuxCtrl (elabora le scansioni) ──────────── 605 ─────────────────────→ RobotApp
```
C'è anche un canale a datagrammi diretto, `/tmp/UDP_AUXCTRL_SEND_PATH` (creato da RobotApp) e
`/tmp/UDP_AUXCTRL_RECV_PATH`: in 2 minuti a robot fermo non è passato nulla.

**Formato** (`CMsgServer::Send`, `CommonClient::SendLocalData`): intestazione da 32 byte little-endian
`magia 0x5867a9b0 | int32 comando | int32 lunghezza | int32 ? | uint64 tempo | 8 byte senza significato`, poi i dati.

**Comandi** (`recvCallback` in AuxCtrl, salto su `comando - 600`; il significato viene dai nomi delle variabili
globali che ogni ramo modifica e dai messaggi di log):

| Comando | Direzione | Dati | Significato |
|---|---|---|---|
| **600** | entrambe | un frame GD32 intero (`FA FB ...`) | **inoltro dei frame del GD32**: ogni 20 ms in entrambe le direzioni |
| 602 | RobotApp → AuxCtrl | vuoto | richiesta di stato (`g_receive_status_request`); 10 volte ogni 100 ms dopo la connessione |
| 602 | AuxCtrl → RobotApp | 2 × int32 | stato: `lidar_status_flag`, `stm32_status_flag` (`01 .. 01 ..` = tutti e due a posto); lo manda `main` |
| 603 | RobotApp → AuxCtrl | int32 | alimentazione del lidar (`g_lidar_power_flag`, log "lidar power is %d") |
| 605 | AuxCtrl → RobotApp | | scansioni del lidar elaborate da AuxCtrl (`getLidarData`); assenti con il lidar spento |
| 606 | entrambe | int32 | controllo del lidar (`g_lidar_control`, log "Aux lidar control %d"); AuxCtrl lo manda da `main` in un certo stato |
| 607 | RobotApp → AuxCtrl | int32 | stato di sospensione (`g_sleep_mode`, log "Sleep state %d") |
| 601, 604 | | | non gestiti |

**Conseguenza importante.** Il protocollo del GD32 lo guida **RobotApp**, non AuxCtrl: heartbeat `0x06`, velocità
`0x66`, LED `0x8d`, sequenza di avvio (`0x07`, `0x17`, `0x18`, `0x19`, `0x78`, ...) arrivano tutti da RobotApp
dentro messaggi 600, e AuxCtrl li scrive sulla seriale. Quando everest-server si ricollega, RobotApp rifà da capo la
sequenza di avvio del GD32. AuxCtrl si occupa di: seriale e inoltro, batteria (`updateBatterySensor`), lidar
(lettura, parsing, motore), audio e comandi di fabbrica. Domanda aperta: con AuxCtrl da solo (senza RobotApp) il
robot non si resettava (fase 0), quindi probabilmente AuxCtrl manda un heartbeat suo quando non ha client: da
verificare con la spia sulla seriale.

## Sequenza di avvio [osservato]

Dalla cattura `cattura-ricarica`, quando AuxCtrl si avvia. I tempi sono quelli delle richieste; le risposte del
GD32 arrivano entro ~100 ms (con strace attivo).

```
+0 ms    SoC → 0x0c 01            GD32 → 0x0c 01
+6 ms    SoC → 0x07 (x2)          GD32 → 0x07 "2.0.1_19082727"   (versione firmware)
+7 ms    SoC → 0x17 01 80 cf f0   GD32 → 0x17 01
+8 ms    SoC → 0x18               GD32 → 0x18 04 00 00 00 3f 00 00 00
+25 ms   SoC → 0x71 00 00 00 00
+27 ms   SoC → 0x65 00            (poi ogni 100 ms per qualche minuto)
+42 ms   SoC → 0x19 01 ; 0x7c 00 ff ff
+215 ms                           GD32 → 0x08 01                    (conferma dello zero dell'IMU)
poi:     SoC → 0x06 ogni 200 ms (heartbeat) e 0x66 ogni 20 ms (velocità)
         GD32 → 0x15 (sensori) di continuo
```

## Il vincolo dell'heartbeat

Senza nessun programma che parli con il GD32 il robot si resetta dopo ~4 minuti (fase 0). Ci sono due candidati:

| Frame | Chi lo manda | Quando |
|---|---|---|
| **`0x0d`** (nessun dato) | **AuxCtrl**, da `main` [codice] | ogni 3,0 s, ma solo se la seriale funziona (`g_stm32_serial_state`) **e** dal GD32 sono arrivati dati dall'ultimo invio (`getStm32ReceiveData()` legge e azzera un flag). Significa "sono vivo e ricevo i tuoi dati" |
| `0x06` (nessun dato) | RobotApp (tramite everest-server e AuxCtrl) | ogni ~204 ms |

**Prova del 2026-10-02** (registro `build/robot/spia/20261002-auxctrl-solo.spia`): con **solo AuxCtrl** in esecuzione
(niente Monitor, RobotApp, everest-server), in 100 s AuxCtrl ha mandato al GD32 solo `0x07` (una volta, all'avvio)
e **`0x0d` ogni 3,0 s**; nessun `0x06` e nessun `0x66`. Il GD32 ha continuato a mandare i sensori ogni 20 ms e il suo
`0x06` ogni 2 s. Nella fase 0 il robot con solo AuxCtrl non si è mai resettato (oltre 7 minuti, e poi ore), quindi
**`0x0d` ogni 3 s basta a tenere buono il GD32**.

Resta da sapere se basta anche il solo `0x06`, e qual è il timeout. Per il nostro driver la scelta sicura è mandare
`0x0d` ogni 3 s (solo mentre si ricevono dati, come AuxCtrl). Il `0x06` a 5 Hz e la velocità a 50 Hz servono solo
quando si comandano i motori. Il timeout si misura con un nostro programma (blocco C4 di `fase1-piano.md`).

## Limiti delle catture

- **strace perde byte in ricezione.** AuxCtrl legge la UART un byte per chiamata (`select` + `read(fd, b, 1)`).
  Con strace ogni chiamata costa ~0,4 ms, quindi AuxCtrl legge circa 1 byte/ms. Il GD32 ne manda di più, il
  buffer della tty trabocca e i byte si perdono: il 26–34% dei frame `0x15` arriva troncato (checksum sbagliato,
  frame lunghi 52 byte invece di 102). I frame con checksum giusto sono affidabili, ma **il periodo vero di
  `0x15` non si può misurare con strace**: lo misurerà il nostro driver, che legge a blocchi.
  In trasmissione invece non si perde nulla (0 errori su 17.701 frame).
- strace fa anche terminare AuxCtrl dopo una decina di secondi (fase 0). Per le prove del passo 3 conviene
  leggere la UART direttamente con il nostro programma.
- Tutte le catture sono con il robot **fermo sulla base**: bumper, cliff, ruote sollevate, motori e lidar non
  sono mai stati sollecitati.
- Lidar (`/dev/ttyS1`): solo 439 byte in 129 s, tutti in ricezione. Il lidar era spento (`0x97 00`). Da
  catturare con il robot fuori dalla base.
- **Dal 2026-10-02 questi limiti sono superati dalla spia** (`spia/spia.c`): nessun byte perso, periodi veri,
  e AuxCtrl non si ferma.

## Prossimi passi
1. **Prove sul robot** (dopo l'accesso comodo): un programma nostro legge `/dev/ttyS3` e stampa i campi del
   frame `0x15` mentre si preme un bumper, si solleva una ruota, si gira una ruota a mano, si copre un cliff.
   Così si completa la tabella dei sensori.
2. Verificare velocità e parametri della seriale (`stty -F /dev/ttyS3 -a` sul robot) e i contatori di
   overrun (`/proc/tty/driver/`).
3. Capire quale messaggio è l'heartbeat sorvegliato dal GD32, e con quale timeout.
4. Lidar: `CLidarPacket` ha un formato e un checksum suoi (`calcCheckSumCRC`, `calcCheckSumSum`); si
   ricavano con `tools/disassembla.sh 'CLidarPacket|CLidarUnpacket'`.
