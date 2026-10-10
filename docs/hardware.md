# Inventario dell'hardware e dei canali di comunicazione

Obiettivo (2026-10-02): il nostro software deve usare **tutto** l'hardware del robot. Questo file elenca ogni
componente, chi lo comanda e attraverso quale canale. È la mappa che la spia deve coprire. Ogni voce dice da dove
viene: [robot] = letto sul robot acceso, [disass.] = dal disassemblato di AuxCtrl/RobotApp,
[ipotesi] = da verificare.

## Schema

```
                         SoC Allwinner A33 (Linux)
 ┌───────────────────────────────────────────────────────────────────────────┐
 │ RobotApp ─ everest-server ─ AuxEverest_Sock ─ AuxCtrl ─ ttyS3 ─┐         │
 │    │  │                              │    └─── /dev/ttyS1 ──┼──┐         │
 │    │  └ tinyplayer/amixer ─ codec audio ─ altoparlante      │  │         │
 │    │       (amplificatore: GPIO 233)                         │  │         │
 │    └ /dev/rtc1 ─ I2C-1 ─ RTC PCF8563                         │  │         │
 └──────────────────────────────────────────────────────────────┼──┼─────────┘
                    115200 8N1                                  │  │ 115200
   MCU GD32F103: ruote, spazzole, aspirazione, motore lidar,  ◄─┘  └─► Lidar LDS (dati)
   bumper, cliff, ruote sollevate, cassetto, tasti, carica, ...
```

## Componenti

| Componente | Comandato da | Canale | Note |
|---|---|---|---|
| MCU GD32F103 | AuxCtrl | `/dev/ttyS3`, 115200 [disass. `getGDData`] | frame `FA FB len cmd dati cksum`, vedi `protocollo.md`. Circa 5,2 KB/s ricevuti [robot] |
| Ruote (velocità, encoder) | GD32 | comando `0x66` | [disass.] |
| Spazzole, aspirazione | GD32 | comandi da identificare | [ipotesi] |
| **Motore del lidar** | GD32 | comando **`0x71`** + int32 potenza 0–100 | `controlLidarPwm(int)` limita a 0..100 e manda `0x71` [disass.]; il `0x71 00 00 00 00` dell'avvio = lidar fermo |
| Bumper, anti-caduta, cassetto, tasti, encoder, batteria/carica | GD32 | frame sensori `0x15` | mappa completa in `protocollo.md` e `RIEPILOGO.md`; il cassetto assente è l'eccezione 6 di everest-server [robot] |
| Tasti (pulizia, base) | GD32 [ipotesi] | da identificare | `detectDockOrPointButtonPress` in AuxCtrl [disass.] |
| LED del tasto | GD32 [ipotesi] | da identificare | la decide `CDetectControlLight` di RobotApp; nessun LED Linux in `/sys/class/leds` [robot] |
| **Lidar** (dati) | AuxCtrl | `/dev/ttyS1`, 115200 [disass. `getLidarData`] | classi `C3iroboticsLidar`/`CLidarPacket`: checksum CRC e somma, pacchetti "tooth scan", velocità, versione, salute. Con il robot fermo il lidar è spento: 8 interrupt di uart1 in 10 minuti [robot] |
| Altoparlante | RobotApp (e AuxCtrl in collaudo) | `tinyplayer <file.mp3>`, volume con `amixer cset name='speaker volume control'` | mp3 in `/usr/share/audio/<lingua>/` [disass.] |
| Amplificatore audio | AuxCtrl/RobotApp | **GPIO 233** (PH9) in uscita | `playSound`: GPIO a 1, suono, GPIO a 0 [disass.]. Oggi vale 1 [robot] |
| Codec audio (anche ingresso) | — | ALSA `card0 audiocodec`, `pcmC0D0p`/`pcmC0D0c` | c'è anche il canale di cattura; un microfono non è stato verificato [ipotesi] |
| Orologio con batteria tampone | RobotApp | `/dev/rtc1`, PCF8563 su I2C-1 indirizzo 0x51 | `rtc0` è quello interno del SoC [robot] |
| Wi-Fi | wifiManager | `wlan0` (SDIO, `sunxi-mmc`) | IRQ solo su CPU0 [fase 0] |
| PMIC AXP223 | kernel | `/sys/class/power_supply/{ac,usb,battery}` | NON vede la batteria del robot (0% fisso): la batteria passa per il GD32 [robot] |
| Sensore di temperatura del SoC | kernel | `thermal_zone0`, input `sunxi-ths` | [robot] |
| Tastiera LRADC del SoC | kernel | input `sunxi-keyboard` (`event1`) | ~25 interrupt/s, nessun programma tiene aperto `event1`: forse non c'è nessun tasto collegato [ipotesi] |
| Seriale di debug | — | `/dev/ttyS2`, 115200 | console di Linux |
| `/dev/ttyS0` | AuxCtrl, solo in collaudo | `enterTestMode`, 115200 | [disass.] |

## Canali software tra i programmi originali [robot]

| Canale | Chi | Uso |
|---|---|---|
| `/tmp/AuxEverest_Sock` (Unix) | AuxCtrl (server) ↔ **everest-server** (client) | frame del GD32 inoltrati così come sono (comando 600), scansioni del lidar (605), stato: vedi `protocollo.md` |
| `/tmp/everest_sock` | RobotApp (server) ↔ everest-server (client) | everest-server è il cervello del robot; RobotApp fa app, cloud, mappe, sospensione |
| `/tmp/UDP_*_PATH`, `/tmp/cliManager.sock` | RobotApp, wifiManager, aggiornamenti | |
| TCP 0.0.0.0:6000 | everest-server in ascolto | |
| TCP verso 43.158.121.228:9090 | RobotApp | **cloud del produttore**, connessione stabile |
| TCP verso 8.211.48.101:21 (FTP) | log-server | caricamento dei log nel cloud (in quel momento tentava la connessione) |
| UDP 8899 | RobotApp | probabilmente l'abbinamento con l'app |

## Cosa deve intercettare la spia

1. `/dev/ttyS3` (GD32), in entrambe le direzioni: motori, sensori, carica, motore del lidar, tasti, LED.
2. `/dev/ttyS1` (lidar), in entrambe le direzioni.
3. `/tmp/AuxEverest_Sock`: fatto con `spia socket` (2026-10-02). Contiene gli stessi frame della seriale più lo stato e le
   scansioni del lidar già elaborate.
4. Audio e GPIO 233: non hanno un protocollo. Basta sapere quali comandi esegue RobotApp (`tinyplayer`, `amixer`).

## Geometria e dinamica (2026-10-05, per il simulatore)

Valori usati da `sim/robot.h` (geometria) e `sim/parametri.txt` (dinamica, `tools/taratura/tara.py`). Fonti: [scheda]
schede tecniche e recensioni del modello; [firmware] everest-server e `/mnt/UDISK/config`; [misura] registrazioni delle
fasi 1–3; [utente]; [?] stima da verificare.

| Cosa | Valore | Fonte |
|---|---|---|
| Diametro, altezza, peso | 349 mm (13,75"), 95 mm (3,75") con la torretta, 3,4 kg | [scheda] |
| Batteria | Li-ion 4S, 14,8 V nominali, 3200 mAh; ~2,45 Ah utili fra 16,3 e 13,8 V | [scheda], [misura c8scarica] |
| Ruote | carreggiata 241,5 mm; **~4580 tick per metro** (everest dice raggio 34,75 mm e 3616 tick per giro, cioè 16 561 tick/m: SBAGLIATO di 3,6 volte); 1 unità di `0x66` = 1 tick/s per ruota; ω di `0x66`: 553 unità per rad/s | [firmware], [misura c2], [misura 5/10 con lidar e IMU] |
| Contatti di carica | **dietro**, sul lato opposto al bumper: il robot entra in base all'indietro | [utente] |
| Ricevitori IR della base | davanti, due: nibble basso del byte 6 = sinistro, alto = destro; campo ~40° ciascuno; la base manda 4 fasci (1 centrale e lungo, 2 corto e verso la sinistra del robot, 4 verso la destra, 8 [?]) | [misura, fase 2 D2 e fase 3 ir1] |
| Anti-caduta | 4 sul davanti, da sinistra a destra = bit 0..3; portata 2–15 cm | [misura D3], [firmware hwdrivers.ini] |
| Bumper | arco anteriore, sinistro e destro; everest segna l'ostacolo 2 cm oltre il contorno, settori di 15° | [misura], [firmware] |
| Torretta del lidar | senza interruttori: sotto un mobile basso il robot si incastra senza bumper | [misura] |
| Altezze di corpo, torretta, piano del lidar | 7,5 / 9,5 / 8,7 cm (altezza totale 94-95 mm dalle recensioni) | [scheda], [?] |
| Lidar | centro **11,2 cm dietro** il centro del robot; angoli in senso **orario** (0° = dietro, 90° = sinistra); scarto dello 0 di 14,5°; scostamento ottico 2,5 cm | [misura 5/10], kaiaai/VacuumTiger |
| Anti-caduta | a ±18° e ±50°, 15 cm dal centro | [?] |
| Soglie superabili | fino a ~2 cm | [scheda] |

Dinamica del GD32 misurata sulle registrazioni (fase 2 da sollevato; fase 3, val1, a terra):

| Cosa | Valore | Registrazione |
|---|---|---|
| Periodo del frame dei sensori | 20,00056 ms | d |
| Ritardo con cui i frame arrivano al SoC | quasi sempre ~0,34 ms; raramente fino a 3,3 ms (5 ms con il nostro driver e il lidar acceso) | d, fase 3 r7 |
| Ritardo del comando di velocità | 29 ms | c2, c2b |
| Rampe da sollevato | v: ~16 600 tick/s² su e giù; ω: ~2 200 tick/s² (molto più lenta) | c2, c2b |
| Risposta a terra | del primo ordine, non a rampa: costante di tempo ~110 ms per v e ~73 ms per ω, qualunque gradino | val1 (fase 3) |
| Velocità massima | 1772 tick/s (~0,39 m/s): comandi più alti vengono tagliati | fase 3, gradini a terra |
| IMU contro lidar | l'imbardata dell'IMU segna il ~3 % in meno della rotazione vera | fase 3 (40,7° / 42°, 201,6° / 208°, 87,9° / 90°) |
| Velocità misurata (byte 32/36) | dagli encoder in ~70 ms: passi di ~7 unità | c2 |
| Senza heartbeat `0x06` | potenza tolta dopo **2,04 s**, poi le ruote si fermano da sole (~1 880 tick/s², ~0,5 s) | c2b |
| Correnti delle ruote (a vuoto) | ~64 mA in moto, picchi fino a ~290, ~770 alla partenza | c2 |
| Spazzola centrale | 58 mA ferma, ~200 accesa, ~620 nello sblocco | c3, spazzola |
| Aspirazione | 219 giri/min per punto di PWM; corrente − 14 mA ∝ giri^2,27 (1,88 A a 18 200 giri/min) | c3, c3b |
| IMU a riposo | deriva dell'imbardata −0,3°/h; beccheggio −0,55°, rollio +0,38°; rumore ±1 (0,01 °/s) | c8scarica |
| IMU in rotazione | conta il **2,8 % in meno**: due giri a sinistra 745,2° contro 766,6° visti dal lidar, a destra −730,0° contro −750,7° (fattore 1,0285, uguale nei due versi). Corretto in `tasks/vacuum` (`YAW_SCALE`) e nel simulatore (`imu_scala`) | k1, 2026-10-10 |
| Encoder su 1,36 m dritti | 1,345 m contro 1,361 m del lidar (1,2 %, dentro l'incertezza della misura col lidar): scala 4580 tick/m lasciata com'è | k2, 2026-10-10 |
| Lidar | giri/s = 0,0856 × potenza + 0,17; salita 0,18 s, discesa 0,24 s, da spento ~0,5 s; ~2040 misure/s; rumore 7 mm fino a 3 m, 12 a 4–5 m, 37 oltre 6 m; ~2% di punti isolati senza eco sotto 2 m | c1 |
| Byte 64 | bit 7 = lidar alimentato (nuovo: `0xE2` acceso, `0x62` spento) | c1 |

## Batteria: cosa faceva il software originale (2026-10-05)

Ricavato da `everest-server` (indirizzi nella versione estratta). Nessun file di configurazione sul robot cambia questi
valori, quindi sul robot valevano i predefiniti.

**Percentuale** (classe `CBatteryCapacity3200`, `batteryCapacity` a 0x66a2b0, chiamata a ogni frame dei sensori, cioè
50 volte al secondo). Il driver la rifà uguale in `driver/batteria.c`.
- Tensione: media degli ultimi 5000 frame (100 s). La media riparte da zero quando il carico aumenta (per esempio da
  fermo a in movimento, o verso la base); non riparte quando il carico diminuisce.
- Stato di carico: **sulla base** se il byte 7 ha il bit 0 o il bit 1 acceso, oppure se il bit 0 ("in carica") è stato
  acceso per più dell'80 % degli ultimi 30 s; altrimenti, con la ventola oltre 1000 giri/min, uno dei 7 stati della
  ventola (ognuno con la sua curva, sotto; nel driver dal 6/10); altrimenti **in movimento** se
  v + |ω| misurati (byte 32 e 36) sono maggiori di zero e la ventola è a 0 (la retromarcia dritta conta come "fermo");
  altrimenti **fermo**.
- Curve da tensione V (media) a carica (0..1), tre tratti:

  | | V ≥ 14,3 | 13,3 ≤ V < 14,3 | V < 13,3 |
  |---|---|---|---|
  | Fermo (`calculateIdleStateCapacity`, 0x668f28) | 0,2845 V − 3,7294 | 0,1107 V² − 2,8129 V + 17,894 | 0,0375 V² − 0,9165 V + 5,6079 |
  | In movimento (`calculateMoppingStateCapacity`, 0x669f70) | 0,3355 V − 4,3157 | 0,2039 V² − 5,3007 V + 34,543 | 0,0618 V² − 1,5107 V + 9,2473 |

  Alcuni valori: 16,3 V → 91 % fermo (oltre il 100 % in movimento, limitato a 100); 15,0 V → 54 / 72 %; 14,2 V →
  27 / 39 %; 13,8 V → 16 / 22 %; 13,3 V → 6 / 11 %; 12,0 V → 1 / 2 %.
- Con l'**aspirazione accesa** (estratte il 5/10, **non ancora nel driver**): sette curve
  `calculate600PaStateCapacity` … `calculate2600PaStateCapacity` (0x669108–0x669f70), scelte in `batteryCapacity` in
  base ai giri della ventola: ≥ 20 000 giri/min 2600 Pa, ≥ 19 000 2000 Pa, ≥ 17 000 1500 Pa, ≥ 15 000 1200 Pa,
  ≥ 14 000 1000 Pa, ≥ 12 000 800 Pa, oltre 1000 600 Pa; sotto, "fermo" o "in movimento" (questo solo con la ventola a
  0). Stessi tre tratti (segni verificati nel disassemblato della curva da 1500 Pa; ai confini saltano meno del 2 %).
  **Nel driver dal 6/10** (`driver/batteria.c`):

  | Stato | V ≥ 14,3 | 13,3 ≤ V < 14,3 | V < 13,3 |
  |---|---|---|---|
  | 600 Pa | 0,2956 V − 3,7757 | 0,1436 V² − 3,6098 V + 22,711 | 0,1 V² − 2,4568 V + 15,09 |
  | 800 Pa | 0,2964 V − 3,7618 | 0,1327 V² − 3,2947 V + 20,455 | 0,0938 V² − 2,3008 V + 14,107 |
  | 1000 Pa | 0,2979 V − 3,749 | −0,0749 V² + 2,4066 V − 18,576 | 0,1281 V² − 3,0971 V + 18,716 |
  | 1200 Pa | 0,2748 V − 3,4517 | −0,0307 V² + 1,1779 V − 10,069 | 0,1167 V² − 2,8292 V + 17,156 |
  | 1500 Pa | 0,2544 V − 3,0593 (1,05 da 15,6 V) | −0,0909 V² + 2,8659 V − 21,825 | 0,1228 V² − 2,9405 V + 17,602 |
  | 2000 Pa | 0,2512 V − 3,0092 (1,05 da 15,6 V) | −0,2143 V² + 6,2383 V − 44,819 | 0,103 V² − 2,4044 V + 14,006 |
  | 2600 Pa | 0,2673 V − 3,2609 (1,05 da 15,6 V) | −0,0957 V² + 3,0044 V − 22,825 | 0,1469 V² − 3,5625 V + 21,608 |

  Calo a tempo dell'1 % ogni 4900 / 4050 / 3500 / 3250 / 2750 / 2400 / 2250 frame (da 600 a 2600 Pa; fermo 16 200,
  in movimento 8500). Nei tre stati più forti, sopra 15,5 V, la discesa verso la curva è 10 volte più lenta (2·10⁻⁵
  invece di 2·10⁻⁴ per frame). L'originale comanda la ventola con `0x68` (`setBlowerSpeed` →
  `packetBlowerSpeed`; `hwdrivers.ini`: `enable_blower_speed=1`) ai livelli di `controller.ini` (profilo Proscenic):
  eco 12 800, normale 15 500, massimo 19 500 giri/min.
- Lontano dalla base la stima **non sale mai**:
  - se la curva dice più dell'1 % in meno, la stima scende verso la curva del 0,02 % dello scarto a ogni frame
    (costante di tempo 100 s);
  - se la dice più dell'1 % in più (tensione risalita a carico tolto) la stima resta ferma, e ogni 16 202 frame di questo
    tipo (~324 s) da fermo o 8502 (~170 s) in movimento cala dell'1 %, come un consumo stimato;
  - altrimenti prende la curva, se è più bassa.
- Sulla base (`calculateChargeStateCapacity`, 0x66a150): la prima stima è −0,0335 x² + 1,3959 x − 12,942 con
  x = V − 0,4 (fino a 16 V; sopra, x = V), oppure 97 % se la carica è stabile o V > 16,4; poi +1 % ogni ~100 s
  (5002 frame) sopra 15 V, ogni ~120 s sotto.
- La stima resta sempre fra 0 e 100 %.

**Soglie** (`CHwdriverExceptionDetect::TConfigParams`, costruttore 0x658090; confrontate con la percentuale senza
altre medie):

| Parametro | Valore | Effetto | Tensione equivalente |
|---|---|---|---|
| `low_start_battery_threshold` | 0,15 | errore 4: non parte la pulizia | ~13,77 V fermo, ~13,53 V in movimento |
| `low_work_battery_threshold` | 0,08 | errore 3: interrompe il lavoro (presumibilmente torna alla base) | ~13,41 V fermo, ~13,25 V in movimento |
| protezione (`detectBatteryProtectionVoltage`, 0x657e20) | 12,0 V sulla media di 100 s | errore 42, se il controllo è abilitato | 3,0 V per cella |
| `battery_print_threshold` | 400 | ogni quanti controlli ripete il messaggio nel registro | – |

Nella stessa struttura: `lidar_time_out_threshold` 1,5 s, `stm32_time_out_threshold` 2,0 s (silenzio del GD32),
`lidar_start_time` 10 s. RobotApp (`CBatteryProtection`) spegne il robot dopo 12 h di inattività.

Le soglie del nostro driver (avviso sotto 14,2 V, motori fermi sotto 13,8 V, sulla media di 1 s) restano più prudenti:
13,8 V vale ~16 % da fermo e ~22 % in movimento sulla scala dell'originale.
