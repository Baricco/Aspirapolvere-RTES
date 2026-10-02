# Inventario dell'hardware e dei canali di comunicazione

Obiettivo (2026-10-02): il nostro software deve usare **tutto** l'hardware del robot. Questo file elenca ogni
componente, chi lo comanda e attraverso quale canale. È la mappa che la spia deve coprire. Ogni voce dice da dove
viene: [robot] = letto sul robot acceso, [disass.] = dal disassemblato di AuxCtrl/RobotApp,
[ipotesi] = da verificare.

## Schema

```
                         SoC Allwinner A33 (Linux)
 ┌───────────────────────────────────────────────────────────────────────────┐
 │ RobotApp ── /tmp/AuxEverest_Sock ── AuxCtrl ── /dev/ttyS3 ──┐            │
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
| Bumper, cliff, ruote sollevate, cassetto, batteria/carica, base | GD32 | frame sensori `0x15` | il cassetto assente è l'eccezione 6 `EXCEPTION_DUSTBOX_INEXISTENCE` di RobotApp [robot] |
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
| `/tmp/AuxEverest_Sock` (Unix) | AuxCtrl ↔ RobotApp | sensori e comandi già interpretati: utile per dare un nome ai campi dei frame |
| `/tmp/everest_sock` | RobotApp ↔ everest-server | |
| `/tmp/UDP_*_PATH`, `/tmp/cliManager.sock` | RobotApp, wifiManager, aggiornamenti | |
| TCP 0.0.0.0:6000 | everest-server in ascolto | |
| TCP verso 43.158.121.228:9090 | RobotApp | **cloud del produttore**, connessione stabile |
| TCP verso 8.211.48.101:21 (FTP) | log-server | caricamento dei log nel cloud (in quel momento tentava la connessione) |
| UDP 8899 | RobotApp | probabilmente l'abbinamento con l'app |

## Cosa deve intercettare la spia

1. `/dev/ttyS3` (GD32), in entrambe le direzioni: motori, sensori, carica, motore del lidar, tasti, LED.
2. `/dev/ttyS1` (lidar), in entrambe le direzioni.
3. `/tmp/AuxEverest_Sock`: non è hardware, ma dice come RobotApp interpreta ogni frame. Serve a dare un nome ai campi.
4. Audio e GPIO 233: non hanno un protocollo. Basta sapere quali comandi esegue RobotApp (`tinyplayer`, `amixer`).
