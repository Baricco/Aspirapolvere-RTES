# Riepilogo: hardware, software originale e protocolli del Proscenic M6 Pro

Quello che sappiamo alla fine della fase 1. I dettagli e le fonti sono in `hardware.md` e `protocollo.md`.
Affidabilità: **[verificato]** = provato sul robot; **[codice]** = letto nel disassemblato; **[ipotesi]** = da
confermare.

---

## 1. Hardware

### Schede e collegamenti

```
                 ┌──────────────────────── scheda principale ─────────────────────────┐
                 │  SoC Allwinner A33 ("R16"): 4 × Cortex-A7 1,2 GHz, 512 MB RAM,      │
                 │  NAND (partizioni nanda…nandi), Linux 3.4.39 OpenWrt                │
                 │                                                                     │
  Wi-Fi RTL871x ─┤ SDIO (wlan0)                    I2C-1 ── RTC PCF8563 (0x51)         │
  USB (adb) ─────┤ USB OTG                          GPIO 233 ── amplificatore audio ─ altoparlante
  console ───────┤ /dev/ttyS2 115200                codec audio interno (uscita + ingresso)
                 │ PMIC AXP223 (alimentazione SoC, tasto di accensione del SoC, USB)    │
                 │                                                                     │
                 │ /dev/ttyS3 115200 8N1 ───────────────┐   /dev/ttyS1 115200 8N1 ─┐   │
                 └──────────────────────────────────────┼──────────────────────────┼───┘
                                                        │                          │
         ┌──────────── MCU GD32F103VCT6 ────────────────┴───┐            Lidar 3irobotix LDS3
         │ firmware 2.0.1_19082727                          │            (solo dati: misure,
         │ ├─ 2 motori ruote + encoder (3616 tick/giro)      │             velocità, versione)
         │ ├─ spazzola laterale, spazzola centrale           │                     ▲
         │ ├─ aspirazione (blower)                           │   alimentazione e   │
         │ ├─ motore del lidar ──────────────────────────────┼── potenza motore ───┘
         │ ├─ 2 microinterruttori bumper (sinistro, destro)  │
         │ ├─ 4 sensori anti-caduta IR (digitali)            │
         │ ├─ 4 valori IR analogici [ipotesi: seguimuro]     │
         │ ├─ interruttore cassetto della polvere            │
         │ ├─ tasti accensione e casa, LED del tasto (colori)│
         │ ├─ IMU (giroscopio + accelerometro)               │
         │ ├─ batteria Li-ion 4S, contatti di carica,        │
         │ │  controllo del caricatore                       │
         │ └─ alimentazione e riavvio del SoC, sospensione   │
         └───────────────────────────────────────────────────┘
```

Il **GD32 è il padrone dell'hardware**: tutti i motori, tutti i sensori fisici, la batteria e perfino l'alimentazione
del SoC passano da lui. Il SoC comunica con lui solo tramite la seriale `/dev/ttyS3`. Il lidar manda i dati
direttamente al SoC su `/dev/ttyS1`, ma il suo motore lo comanda il GD32.

### Sensori

| Sensore | Dove arriva | Come si legge | |
|---|---|---|---|
| Bumper sinistro / destro | GD32 → frame sensori | byte 1, bit 2 / bit 1 (1 = premuto) | [verificato] |
| 4 anti-caduta (sinistra → destra) | GD32 | byte 3, bit 3, 2, 1, 0 (1 = nessun pavimento) | [verificato] |
| Cassetto della polvere | GD32 | byte 4, bit 2 (1 = presente) | [verificato] |
| Tasto accensione / casa | GD32 | byte 58 / 62 = durata della pressione in 10 ms | [verificato] |
| Encoder ruota sinistra / destra | GD32 | int32 ai byte 16 / 24 (contatore assoluto) | [verificato] |
| Carica in corso / sulla base | GD32 | byte 7, bit 0 / bit 1 | [verificato] / [codice] |
| Tensione della batteria | GD32 | byte 8 ÷ 10 = volt (16,5 V in carica) | [verificato] |
| IMU | GD32 | byte 40–55: angolo integrato e valori grezzi | [ipotesi] sui singoli campi |
| 4 IR analogici | GD32 | int16 ai byte 78, 80, 82, 84 | [ipotesi] seguimuro (everest-server ha `CObservationFollowIR`) |
| Temperatura batteria | GD32 | byte 9 (°C) | [ipotesi] |
| Lidar a 360° | SoC, `/dev/ttyS1` | pacchetti `AA` (scansioni `0xAD`/`0xA9`, velocità `0xAE`) | [codice] formato, contenuto da verificare |
| Temperatura del SoC | SoC | `thermal_zone0` | [verificato] |

**Non ci sono** interruttori sulle ruote ("ruota sollevata") né sulla torretta del lidar [verificato]: il robot
capisce di essere sollevato dai sensori anti-caduta.

### Attuatori

| Attuatore | Chi lo comanda | Comando |
|---|---|---|
| Ruote | GD32 | `0x66` velocità lineare + angolare (oppure `0x67` velocità delle due ruote) |
| Spazzola laterale / centrale | GD32 | `0x69` / `0x6a` (int8 con segno) |
| Aspirazione | GD32 | `0x68` (2 byte) |
| Motore del lidar | GD32 | `0x97` acceso/spento, `0x71` potenza 0–100 |
| LED del tasto | GD32 | `0x8d` (00 spento, 01 azzurro, 04 giallo, 08 rosso lampeggiante) |
| Emettitori IR anti-caduta | GD32 | `0x78` acceso/spento, `0x79` direzione |
| Caricatore | GD32 | `0x9b` |
| Alimentazione / riavvio del SoC | GD32 | `0x99` / `0x9a` |
| Altoparlante | SoC | `tinyplayer file.mp3`, volume con `amixer`, amplificatore con GPIO 233 |

### Parametri meccanici [codice]
Ruote: raggio 34,75 mm, carreggiata 241,5 mm, 3616 tick per giro (60,4 µm per tick). Batteria Li-ion 4S
(16,8 V piena, protezione intorno a 12 V).

---

## 2. Software originale

### Processi

| Programma | Avviato da | Cosa fa |
|---|---|---|
| **Monitor** | `/etc/init.d/robotManager` | avvia AuxCtrl, everest-server, RobotApp e log-server, e li **rilancia** se terminano |
| **AuxCtrl** (400 KB, C++) | Monitor | **driver delle periferiche**: possiede `/dev/ttyS3` e `/dev/ttyS1`; inoltra i frame del GD32 a everest-server e viceversa; manda il **keepalive `0x0d`** ogni 3 s; legge e decodifica il lidar e manda le scansioni; stato del lidar e del GD32; batteria; audio e GPIO dell'amplificatore (in collaudo); socket TCP di fabbrica (collaudo, accensione/spegnimento di adb) |
| **everest-server** (10 MB, C++) | Monitor | **il cervello del robot** (framework "everest" di 3irobotix): macchina a stati delle modalità (pulizia globale, locale, angoli, a punto, lavaggio, ritorno alla base, esplorazione, navigazione…), controllo del movimento, **SLAM** (Google Cartographer, ICP, MCL), pianificazione dei percorsi (A*, D*), rilevamento delle eccezioni (ruote, cassetto, batteria, spazzole bloccate, slittamento…), gestione delle osservazioni dei sensori, LED, **guida del protocollo del GD32** (avvio, heartbeat `0x06`, velocità `0x66`, motori). Server TCP sulla porta 6000 |
| **RobotApp** (6 MB, C++) | Monitor | **app e servizi**: collegamento al cloud del produttore (TCP verso 43.158.121.228:9090, websocket, protobuf), abbinamento con l'app (UDP 8899), mappe salvate e cronologia, programmazioni, consumabili, orari di silenzio, **sospensione** (`device_config.ini`: `sleep_enable`, `sleep_time`), suoni (`tinyplayer`, volume), orologio (`/dev/rtc1`), aggiornamenti |
| **log-server** | Monitor | carica i log nel cloud via FTP |
| **wifiManager** | `S90robotWifiManager` | `wpa_supplicant` e `udhcpc`; `hostapd` per l'abbinamento |
| dropbear, adbd, dnsmasq, cron, ntp | init | servizi di sistema |

Configurazione in `/mnt/UDISK/config/` (`device_config.ini`, `hwdrivers.ini`, `motion_params.ini`, parametri dello
SLAM in `*.lua` e `*-icp-params.ini`, ecc.). Log in `/mnt/UDISK/log/`.

### Come sono collegati
```
GD32 ⇄ /dev/ttyS3 ⇄ AuxCtrl ⇄ /tmp/AuxEverest_Sock ⇄ everest-server ⇄ /tmp/everest_sock ⇄ RobotApp ⇄ cloud
lidar → /dev/ttyS1 → AuxCtrl ────────────────────────↗ (scansioni, messaggio 605)
                     AuxCtrl ⇄ /tmp/UDP_AUXCTRL_* ⇄ RobotApp   (datagrammi, a riposo vuoto)
```

### Cosa dovremo replicare
Per far funzionare il robot con il nostro software al posto di quello originale:
1. **il lavoro di AuxCtrl**: seriale del GD32 con il keepalive `0x0d`, lettura del lidar, audio;
2. **il lavoro di everest-server verso il GD32**: sequenza di avvio, heartbeat `0x06` e velocità `0x66` quando si
   muove, comandi dei motori, LED, emettitori IR, gestione della carica, sospensione e risveglio;
3. le funzioni "da robot" di everest-server che ci interessano per il progetto: sicurezza (bumper, anti-caduta),
   controllo del movimento, eventualmente navigazione con il lidar.

RobotApp (cloud, app, mappe, programmazioni) non serve.

---

## 3. Protocolli

### 3.1 SoC ↔ GD32 (`/dev/ttyS3`, 115200 8N1, senza controllo di flusso)

**Frame:**
```
 FA FB | len | cmd | dati (len-3 byte) | ck_hi ck_lo
```
| Campo | Significato |
|---|---|
| `FA FB` | intestazione fissa |
| `len` | byte che seguono: cmd + dati + 2 di checksum (3…255) |
| `cmd` | codice del messaggio; richiesta e risposta usano lo stesso codice |
| dati | campi multi-byte little-endian |
| checksum | somma modulo 2^16 delle parole **big-endian** da 16 bit di cmd + dati; se i byte sono dispari l'ultimo va in **XOR**; trasmesso big-endian |

**Messaggi dal SoC al GD32** (frequenze a robot fermo sulla base):

| cmd | Nome | Dati | Frequenza / quando |
|---|---|---|---|
| `0x04` | sospensione | – | quando il robot va in sospensione |
| `0x05` | conferma del risveglio | – | in risposta al `0x05` del GD32 |
| `0x06` | heartbeat (everest-server) | – | ogni ~204 ms |
| `0x07` | richiesta della versione | – | all'avvio |
| `0x08` | azzera l'IMU | – | all'avvio e al risveglio |
| `0x0a` | azzera gli errori | – | ogni tanto |
| `0x0c` | SoC attivo (`01`) / sta per dormire (`00`) | 1 byte | all'avvio, al risveglio, prima della sospensione |
| **`0x0d`** | **keepalive (AuxCtrl)** | – | **ogni 3,0 s**, solo se arrivano dati dal GD32 |
| `0x17`, `0x18`, `0x19`, `0x7c`, `0x86`, `0x6b` | parte della sequenza di avvio | vari | all'avvio [significato da capire] |
| `0x65` | tipo di controllo dei motori | 1 byte (00, 02) | all'avvio e ai cambi di modalità |
| `0x66` | velocità: v e ω | int32 v·16561,27 (tick/s), int32 ω·1999,77 | **ogni 20 ms** (quasi sempre zero) |
| `0x67` | velocità delle due ruote | 2 × int32 | mai visto |
| `0x68` | aspirazione | 2 byte | ai cambi di stato |
| `0x69` / `0x6a` | spazzola laterale / centrale | int8 | ai cambi di stato |
| `0x71` | potenza del motore del lidar | int32 0–100 | all'avvio (0) |
| `0x78` / `0x79` | emettitori IR anti-caduta / direzione | bool | all'avvio di AuxCtrl/everest (`0x78 01`) |
| `0x8d` | LED del tasto | 1 byte: 00, 01, 04, 08 | quando cambia lo stato |
| `0x97` | alimentazione del lidar | bool | ai cambi di stato |
| `0x99` / `0x9a` / `0x9b` | alimentazione SoC / riavvio SoC / caricatore | bool / – / bool | mai visti |
| `0x9d` | [ipotesi] legato alla carica: `01` contatto perso, `00` sulla base | 1 byte | ai cambi del contatto di carica |
| `0xa1`–`0xa4` | calibrazioni di IMU e magnetometro | vari | `0xa2` all'avvio |

**Messaggi dal GD32 al SoC:**

| cmd | Nome | Dati | Frequenza |
|---|---|---|---|
| **`0x15`** | **sensori** | 96 byte (tabella sotto) | **ogni 20,0 ms** |
| `0x06` | heartbeat del GD32 | – | ogni 2,0 s |
| `0x07` | versione | 32 byte ASCII `"2.0.1_19082727"` | in risposta |
| `0x08`, `0x0c`, `0x17`, `0x18`, `0xa2` | conferme della sequenza di avvio | vari | all'avvio |
| `0x04` | conferma della sospensione | – | poi il GD32 tace |
| `0x05` | risveglio (tasto premuto mentre dorme) | – | ogni 100 ms finché il SoC non conferma |

**Frame dei sensori `0x15`** (offset nei 96 byte di dati):

| Byte | Campo |
|---|---|
| 0 | stato [ipotesi] (01) |
| 1 | bumper: bit 2 sinistro, bit 1 destro |
| 3 | anti-caduta: bit 3…0 da sinistra a destra, 1 = vuoto |
| 4 | bit 2 cassetto presente |
| 7 | bit 0 in carica, bit 1 sulla base |
| 8 | tensione batteria × 10 |
| 9 | temperatura batteria [ipotesi] |
| 16–19 / 24–27 | encoder sinistro / destro (int32) |
| 40–43 | angolo dal giroscopio [ipotesi] |
| 44–55 | IMU grezza [ipotesi] |
| 56 | percentuale batteria [ipotesi] |
| 58 / 60 / 62 | durata della pressione dei tasti accensione / punto (assente) / casa, in 10 ms |
| 64 | bit 5 = 1 fuori dalla base [ipotesi] |
| 65 | stato del LED (copia dell'ultimo `0x8d`) |
| 78, 80, 82, 84 | 4 IR analogici [ipotesi: seguimuro] |
| 88–89 | costante `03 19` |
| 90–95 | copia di campi dell'IMU |
| altri | 0 a robot fermo: probabilmente correnti dei motori e spazzole, da vedere con i motori accesi |

**Sequenze:**
- **Avvio:** SoC → `0x0c 01`, `0x07`, `0x17 01 80 cf f0`, `0x18`, `0x71 0`, `0x65 0`, `0x19 01`, `0x7c 00 ff ff`,
  `0x78 01`; GD32 → conferme e versione; poi sensori ogni 20 ms.
- **Sospensione:** SoC spegne LED, lidar, motori (`0x8d 00`, `0x97 00`, `0x65 00`, `0x68`, `0x69`, `0x6a`, `0x6b`),
  poi `0x0c 00` e `0x04`; il GD32 conferma e smette di trasmettere.
- **Risveglio:** tasto → GD32 `0x05` ripetuto; SoC → `0x0c 01`, `0x08`, `0x05`, `0x8d 01`. Attenzione: non rimanda
  `0x78 01`, quindi gli IR anti-caduta restano spenti finché lo stack non riparte.
- **Watchdog del GD32 [verificato]:** sorveglia **solo `0x0d`**. Senza `0x0d` il GD32 resetta tutto il robot dopo
  **~190 s**, anche se riceve `0x06` ogni 200 ms. AuxCtrl manda `0x0d` ogni 3 s, e basta da solo a tenere vivo il robot.

### 3.2 Lidar → SoC (`/dev/ttyS1`, 115200)

All'accensione un testo in cinese (GBK) con produttore e versione, poi pacchetti:
```
 AA | len_hi len_lo | indirizzo | tipo | cmd | n_hi n_lo | parametri | ck_hi ck_lo
```
| Campo | Significato |
|---|---|
| `len` | byte del pacchetto senza checksum (8 + n), da 6 a 100 |
| indirizzo | 1 → checksum = somma dei byte; altrimenti CRC-16/MODBUS |
| tipo | `0x61` |
| cmd | `0xA9` scansione (formato vecchio), `0xAB` salute, `0xAC` versione, `0xAD` scansione (nuovo formato), `0xAE` velocità |
| n | byte di parametri (big-endian) |

Il contenuto delle scansioni (angoli, distanze, intensità) si ricava con il lidar in rotazione (fase 3).

### 3.3 AuxCtrl ↔ everest-server (`/tmp/AuxEverest_Sock`, socket Unix stream)

Intestazione da 32 byte little-endian: `magia 0x5867a9b0 | comando | lunghezza | ? | tempo (64 bit) | 8 byte inutili`,
poi i dati.

| Comando | Direzione | Significato | Frequenza |
|---|---|---|---|
| 600 | entrambe | un frame GD32 intero | ogni 20 ms in entrambe le direzioni |
| 602 | everest → AuxCtrl | richiesta di stato | dopo la connessione |
| 602 | AuxCtrl → everest | stato lidar e GD32 (2 × int32, 1 = ok) | dopo la connessione |
| 603 | everest → AuxCtrl | alimentazione del lidar | ai cambi |
| 605 | AuxCtrl → everest | scansioni del lidar elaborate | con il lidar acceso |
| 606 | entrambe | controllo del lidar | ai cambi |
| 607 | everest → AuxCtrl | stato di sospensione | ai cambi |
