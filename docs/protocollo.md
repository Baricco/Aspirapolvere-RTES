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
`0x0c` (`01`, all'avvio, con risposta `0x0c 01`), `0x0d` (nessun dato, ogni 3,0 s), `0x17` (`01 80 cf f0`), `0x18`,
`0x19` (`01`), `0x6b` (1 byte), `0x7c` (`00 ff ff`), `0x86` (`00`), `0x9d` (`00`/`01`).

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

### Frame dei sensori (`0x15`, 96 byte)
AuxCtrl (`unpacketCRL200SSensorMessage`) azzera una struct `TSensorMessage` da 88 byte e ci copia i primi 88 byte
dei dati così come sono [codice]. Gli ultimi 8 servono probabilmente alla variante CRL-300. AuxCtrl usa solo i
campi della batteria (`updateBatterySensor`); il resto lo inoltra a RobotApp sul socket
`/tmp/AuxEverest_Sock`. RobotApp non esporta i nomi delle sue funzioni interne: il significato degli altri campi
va ricavato con le prove sul robot.

Offset in byte dall'inizio dei dati (dopo `cmd`). Valori a robot fermo sulla base.

| Offset | Tipo | Valori visti | Significato |
|---|---|---|---|
| 0–1 | 2 byte | `01 00` → `01 06` | [ipotesi] stato/modalità; cambia durante l'avvio |
| 2–3 | 2 byte | `00 0d` → `00 0f` | [ipotesi] maschera di bit (sensori abilitati?) |
| 7 | bit | bit0, bit1 | **[codice] bit0 = in carica, bit1 = sulla base / alimentatore collegato** (AuxCtrl considera "sulla base" anche quando bit0 = 1). Visto `0x02` sulla base con la batteria in protezione, `0x01` sulla base in carica (spia, 2026-10-02) |
| 8 | uint8 | 0x71–0x8c la notte del 2026-10-01 (batteria in protezione che si riprende), **0xa5 = 16,5 V** il 2026-10-02 sera dopo un giorno sulla base | **[codice] tensione della batteria × 10**. [ipotesi] batteria Li-ion 4S: 16,8 V piena, ~12 V scarica |
| 9 | uint8 | 27–28 | [ipotesi] temperatura della batteria (°C) |
| 16–19 | int32 | `0x6978`, `0x6954` | [ipotesi] contatore encoder ruota sinistra (costante: robot fermo) |
| 24–27 | int32 | `0x6978`, `0x6950` | [ipotesi] contatore encoder ruota destra |
| 44–51 | 4 × int16 | piccoli, attorno a 0 | [ipotesi] IMU (giroscopio/accelerometro): rumore a robot fermo |
| 56–57 | uint16 | 100 | [ipotesi] percentuale batteria o costante |
| 58, 60, 62 | 3 × uint16 | 0 | **[codice] tre valori che AuxCtrl moltiplica per 10** e salva insieme alla tensione: [ipotesi] correnti |
| 64–65 | uint16 | 66–3170, poi 2114 | [ipotesi] segnale IR della base o corrente di carica |
| 78–85 | 4 × int16 | 0–64 | [ipotesi] sensori di dislivello (cliff) / IR |
| 88–89 | 2 byte | `03 19` | costante |
| 90–95 | 3 × int16 | uguali a 94/92/44 | copia di alcuni campi dell'IMU (stessi valori) |
| altri | | sempre 0 | da scoprire con il robot in movimento (bumper, ruote sollevate, spazzole…) |

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

Senza AuxCtrl il robot si resetta dopo ~4 minuti (fase 0). AuxCtrl manda `0x06` ogni ~200 ms e `0x66` ogni 20 ms.
Non sappiamo ancora quale dei due il GD32 sorvegli, né con quale timeout. Per il driver minimo della fase 2 il
punto di partenza sicuro è replicare entrambi: `0x06` a 5 Hz e `0x66` con velocità zero a 50 Hz. Poi si
misura quanto si può allungare il periodo prima del reset.

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
