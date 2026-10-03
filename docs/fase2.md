# Fase 2: diario della caratterizzazione dell'hardware

Piano: `fase2-piano.md`. Qui c'è quello che è stato fatto davvero, in ordine, con i risultati e i dubbi aperti.
Affidabilità come in `RIEPILOGO.md`: **[V]** verificato sul robot, **[C]** letto nel codice, **[?]** da confermare.

## 2026-10-03 — lavoro autonomo (l'utente è via)

Situazione all'inizio: robot acceso da 20 h con lo stack originale, **sollevato e fuori dalla base** (lo ha messo
così l'utente prima di andare, per poter provare i motori), batteria a 16,2 V, cassetto della polvere presente.
Il PC era finito sulla rete "Wifi Barucci" (192.168.0.x) invece di "WiFi Baricco": l'utente l'ha spostato.

### Blocco A — disassemblato di everest-server

Metodo: `tools/disassembla.sh '<regex>' ../Estratto/nandd-rootfs/usr/sbin/everest-server`. everest-server è
compilato **con** ottimizzazioni (a differenza di AuxCtrl), ma resta leggibile: ogni `packet*` chiama la stessa
funzione `0x65cc14(this, cmd, dati, n)`, quindi `r1` = codice, `r3` = lunghezza.

**Comandi SoC → GD32 identificati** (tutti in `gd32/gd32.h`):

| cmd | funzione | dati | note |
|---|---|---|---|
| `0x0c` | `packetTinaHaltedDetectOnce(bool)` | 1 | "Tina" = il Linux di Allwinner: attiva/disattiva il controllo che il SoC non sia bloccato. Prima lo chiamavamo "SoC attivo" |
| `0x17` | `packetRequireIMURawData` | 4 | solo il primo byte (01) è scritto; gli altri 3 sono residui dello stack. Spiega lo strano `0x17 01 80 cf f0` dell'avvio |
| `0x18` | `packetGetMachineFunctionList` | 0 | il GD32 risponde con l'elenco delle funzioni |
| `0x19` | `packetCliffBack(bool)` | 1 | retromarcia automatica del GD32 sugli anti-caduta |
| `0x6b` | `packetWaterPumpSpeed(int8)` | 1 | **pompa dell'acqua**: everest manda 0 all'avvio e in sospensione. Vietato nel banco di prova |
| `0x7a` | `packetSelfControlParams` | 24 | struttura da capire |
| `0x7c` | `packetMaskSensorMotion` | 3 | quali sensori fermano il moto; all'avvio `00 ff ff` |
| `0x80` | `packetBumperSmooth(bool)` | 1 | |
| `0x84` | `packetBlowerControlMode` | 4 (int32) | |
| `0x85` | `packetBlowerPwmOnly` | 1 (uint8) | PWM diretto dell'aspirazione |
| `0x86` | `packetBlowerPwmBattery` | 1 (uint8) | PWM compensato con la tensione (nei log: `setBlowerPwmBattery N`) |
| `0x9d` | `packetSensorPower(bool)` | 1 | alimentazione dei sensori: `01` fuori dalla base, `00` sulla base (come osservato in fase 1) |

Scale e limiti (costruttore `CSerialMessagePacket::TParams`): velocità lineare massima **0,5 m/s**, angolare
**2π rad/s**, ogni ruota **0,5 m/s**; aspirazione `0x68` = int16 velocità/10, al massimo 30000/10 = 3000.

**Messaggi GD32 → SoC gestiti da everest** (`handleMessage`): `0x01` codice di errore (mai visto finora),
`0x07` versione, `0x08` conferma azzeramento IMU, `0x15` sensori, `0xa1`/`0xa2` calibrazione IMU, `0xab` debug.

**Frame dei sensori `0x15`** (`handleSensorMessage`, `update*Sensor`). everest copia i byte 0–85 del frame in
una struttura "packed" agli stessi offset, **ignora i byte 86–87** e sposta 88–95. Tipi e scale ricavati:

| byte | tipo | everest ne fa | significato |
|---|---|---|---|
| 0 | u8 | confronta con 1 | stato [?] |
| 1 | bit | sx = bit2\|bit3, dx = bit1\|bit0 | bumper [V]; bit 3 e 0 forse un secondo contatto per lato [?] |
| 2 | bit 1, 0 | due flag | [?] non cambiano sollevando il robot: non sono "ruota sollevata" |
| 3 | bit 3..0 | 4 flag; azzerati se il byte 64 bit 5 = 0 | anti-caduta [V] |
| 4 | bit 2, 1, 0 | 3 flag | bit 2 cassetto [V]; 1 e 0 [?] (serbatoio, panno) |
| 5 | bit 1, 0 | 2 flag | [?] |
| 6 | 2 nibble | due codici 1/2/4/8 | IR della base (`unpacketDockIR`, `dockir_1/2/4/8`) [?] |
| 7 | bit 1, 0 | in carica = bit0; sulla base = bit0\|bit1 | [V] |
| 8 | u8 | ÷10 | tensione [V] |
| 9 | **int8** | float | temperatura della batteria in °C [C] (31 °C a riposo, 27 °C nella cattura del 2/10) |
| 16, 24 | int32 | | encoder [V] |
| 32, 36 | int32 | ÷ parametro, somma e differenza | velocità delle ruote → v e ω [C] |
| 40–51 | 6 × int16 | ÷100 | IMU: 40/44/48 e 42/46/50 interlacciati (VacuumTiger: giroscopio/accelerometro) [?] |
| 52–57 | 3 × int16 | ÷100, normalizzati | **vettore di gravità** in centesimi di g [V]: Z = 100 a robot piano. Non è la percentuale della batteria |
| 58, 60, 62 | u16 | ×10 | tasti accensione / punto / casa in ms [V] |
| 64 | bit 0, 1, 5 | bit 5 = "anti-caduta e bumper validi" | `0x42` sulla base, `0x62` fuori [V] |
| 65 | u8 | | LED [V] |
| 66 | u8 & 0x0f | | [?] |
| 67 | int8 | | [?] |
| 68 | u8 | | [?] |
| 69 | u8 | ×100 | [?] |
| 70 | u8 | | livello dell'acqua (VacuumTiger) — qui non c'è il serbatoio |
| 72 | u16 | | [?] |
| 74–84 | 6 × u16 | ÷1000 | correnti dei motori in mA [?] (da confermare accendendo i motori) |
| 88 | int16 | > 0 | costante `0x1903` |
| 90, 92, 94 | 3 × int16 | ÷100 | magnetometro / sensore geomagnetico [?] |

Altre scoperte:
- nei log del 2024 compare migliaia di volte `EXCEPTION_WHEEL_UP` (ruota sollevata), ma il byte 2 resta 0 con il
  robot sollevato: probabilmente everest deduce "ruota sollevata" da anti-caduta, IMU o correnti
  (`detectWheelUp` → code di filtri, non seguita fino in fondo);
- everest gestisce anche un **sensore geomagnetico** (striscia magnetica come barriera virtuale) e tre
  ricevitori IR della base (`ir_front`, `ir_left_front`, `ir_right_front`);
- **VacuumTiger** (clonato solo in lettura nella cartella temporanea della sessione) conferma la disposizione
  dell'IMU e il byte 70, ma copre molti meno byte del disassemblato.

Codice: `gd32/gd32.h` ha tutti i codici e la struttura `gd32_sensori_t` con la decodifica completa;
`gd32_leggi_sensori()` sostituisce `gd32_leggi_batteria()`. Test in `test/test_gd32.c` con due frame veri
(sulla base del 2/10 e sollevato del 3/10) e un frame costruito per i segni e le scale.

### Blocco B — banco di prova

**`gd32/sicurezza.{h,c}`**: il blocco di sicurezza dei motori come logica pura (niente seriale, niente Linux),
riusabile dal driver della fase 3. Regole:
- comandi ai motori (`0x66 0x67 0x68 0x69 0x6a 0x84 0x85 0x86`) diversi da zero solo con `--sollevato`, byte 3
  dell'ultimo frame dei sensori = `0x0f` e frame più giovane di 100 ms;
- un comando "a zero" è sempre permesso;
- prima di aver visto `0x0f` i comandi ai motori sono solo rifiutati; **dopo** averlo visto, qualunque
  cambiamento del byte 3 o 100 ms senza sensori fanno **scattare** il blocco: frame di arresto subito e motori
  rifiutati fino alla fine del programma, anche se torna `0x0f`;
- sempre vietati: `0x6b` pompa, `0x99`/`0x9a` alimentazione e riavvio del SoC. Il motore del lidar è libero.
Test: `test/test_sicurezza.c` (72 controlli).

**`spia guida`** riscritta (`spia/spia.c`):
- keepalive `0x0d` ogni 3 s e heartbeat `0x06` ogni 200 ms automatici (`--senza-keepalive`, `--senza-heartbeat`);
- `--invia CMD[:DATI]@MS` periodico, `--una CMD[:DATI]` singolo, `--sequenza FILE` con tempi
  (`T ogni MS cmd`, `T cmd`, `T basta CMD`, `T fine`); DATI in esadecimale o con tipi (`0x66:i32=1000,i32=0`);
- riga di stato ogni secondo con i sensori decodificati, ogni altro frame del GD32 stampato subito,
  `--csv FILE [--csv-ogni MS]` con tutti i campi, `-o REGISTRO` con tutti i byte;
- arresto dei motori quando il blocco scatta e all'uscita (fine, Ctrl-C, SIGTERM, SIGHUP);
- `--tensione-minima DV`: sotto la soglia (media su 1 s) ferma tutto ed esce con codice 3.
Codici d'uscita: 0 normale, 2 opzioni sbagliate, 3 tensione minima, 4 blocco scattato.

**`tools/prova-guida.py`**: il banco di prova contro un GD32 finto su una pty, 8 scenari (motore senza
`--sollevato`, pompa, robot appoggiato, anti-caduta che cambia, sensori che smettono, SIGTERM, sequenza, tensione
minima). Misurato: arresto **3 ms** dopo il cambio dell'anti-caduta, **87 ms** dopo l'ultimo frame dei sensori.
È dentro `make test`.

### Blocco C — prove sul robot (sollevato, fuori dalla base, stack originale fermo durante le prove)

Strumento: `tools/banco-robot.sh avvia|lidar NOME [opzioni di spia guida]`. Lo stack originale riparte **da solo**
quando spia guida termina, per qualunque motivo. Sequenze in `tools/sequenze/`, dati in `build/robot/fase2/NOME/`
(fuori da git). Tensione minima di sicurezza 14,0 V; la batteria è rimasta fra 16,2 e 15,6 V (sotto l'aspirazione al massimo).

**C5/C4 — frequenza e domande al GD32** (`c5-frequenza.seq`): il frame `0x15` arriva sempre a **50,0 Hz**, con
`0x65 00` o `02` (VacuumTiger dice ~110 Hz: sbagliato per questo firmware). Versione `"2.0.1_19082727"` (byte 20 =
`01` [?]). `0x18` risponde `04 00 00 00 3f 00 00 00`: due int32, 4 e la maschera `0x3f` [?].

**C1 — lidar** (`c1-lidar.seq`, `tools/analizza-lidar.py`): si accende con `0x97 01` + `0x71 potenza`, senza altro.
- velocità ≈ 0,086 giri/s per unità di potenza: 40 → 3,59; 50 → 4,45; 60 → 5,30; 70 → 6,15; 80 → 7,02 giri/s;
- sotto ~4 giri/s il lidar manda solo `0xAE` (velocità, u8 × 0,05 giri/s) e **nessuna scansione**; sopra, manda solo
  scansioni `0xAD` (la velocità è in `param[0]`);
- `0xAD`: u8 velocità × 0,05 | int16 BE × 0,01 (sempre 0,82) [?] | u16 BE angolo di inizio × 0,01° | punti da 3 byte
  (u8 intensità, u16 BE distanza in quarti di mm). **16 settori da 22,5° per giro, ~21 punti ciascuno, ~360 punti
  per giro** (≈1°); 24% di punti senza eco; distanze 0,11–7,5 m nella stanza;
- regolazione di AuxCtrl (`lidarSpeedControl`, ogni 20 ms): `potenza = 60 + I + 10·e + 10·Δe`, `I = Σe` limitato a
  ±20, riferimento **6 giri/s** (serve potenza ≈ 68).

**C2 — ruote** (`c2-ruote.seq`, `c2b-ruote.seq`):
- `0x65` = modalità dei motori: `00` spenti (`0x66` ignorato), `02` velocità v/ω con `0x66`, `01` velocità per
  ruota con `0x67` (con `02` lo `0x67` è ignorato);
- scale confermate: 828 tick/s = 5 cm/s; ω 400 = 0,2 rad/s → ±400 tick/s per ruota. Encoder: in avanti
  **aumentano** tutti e due; con ω > 0 il sinistro diminuisce e il destro aumenta;
- **byte 32 e 36 = v e ω misurati** (stesse unità del comando), solo in modalità `02`; non sono le due ruote;
- **con gli anti-caduta su "vuoto" il GD32 non va avanti con `0x66`**: con la retromarcia automatica attiva (`0x19 01`,
  default di everest) va **indietro a metà velocità**; con `0x19 00` resta fermo. Indietro e rotazione sono normali.
  In modalità `01` con `0x67` invece la ruota va avanti lo stesso (il GD32 lì non protegge);
- risposta: prima reazione dopo un frame (20–40 ms), velocità raggiunta in ~130 ms; arresto con zero esplicito ~120 ms;
  picco di corrente all'avvio ~1,3 A per ruota, poi 30–300 mA;
- **senza `0x66` ma con l'heartbeat, il GD32 mantiene l'ultima velocità** (provato per 6 s);
- **senza heartbeat `0x06`, il GD32 toglie potenza ai motori ~1,9 s dopo l'ultimo** (le ruote si fermano per inerzia
  in ~0,6 s). Il GD32 ha quindi due sorveglianze: `0x06` (~2 s → motori spenti) e `0x0d` (~190 s → reset).

**C3 — spazzole e aspirazione** (`c3-spazzole-aspirazione.seq`, `c3b-aspirazione.seq`). La spazzola laterale
**non è montata** (correnti a vuoto). Durante `c3` il blocco di sicurezza è scattato (bit 3 del byte 3 a "pavimento"
per 4,4 s): secondo l'utente il robot è stato toccato; il blocco ha fermato tutto come previsto.
- correnti in mA: **74 ruota sinistra, 76 ruota destra, 78 spazzola laterale, 80 spazzola centrale (offset ~58),
  82 aspirazione (~14 a riposo)**; 84 resta a 2–3 [?];
- 67 = eco della spazzola laterale, 68 = eco della spazzola centrale (con −30 riporta 100 [?], corrente 306 mA contro
  122 a +30);
- **69 × 100 = velocità misurata della ventola in giri/min**: `0x86 40` → 9100, `0x86 88` → 18200 (1,9 A, la tensione
  scende a 15,6 V), `0x85 40` → 9080, `0x68 1000` → 9880: `0x68` è una velocità regolata dal GD32 (giri/min ÷ 10),
  `0x85`/`0x86` sono potenze dirette.
- nota del banco di prova: un comando ai motori a 0 ms viene rifiutato (non ci sono ancora frame dei sensori):
  nelle sequenze il primo motore va dopo ≥ 500 ms.

**C7 — LED** (`c7-led.seq`): l'eco nel byte 65 ricopia qualunque valore 0–31, quindi non dice quali modi esistono.
Nei log di everest compaiono nomi come `LED_BLUE_BRIGHT`, `LED_YELLOW_BRIGHT`, `LED_YELLOW_BREATH`, ma non c'è una
tabella. L'utente ha visto alcuni modi ma non li ricorda tutti: **da rifare nel blocco D**, un valore alla volta.

**C6 — audio**: `/bin/tinyplayer` (68 mp3 in `/usr/share/audio/english/`, ~1,1 s per `sound_beep_beep.mp3`),
volume `amixer cset iface=MIXER,name='speaker volume control' 0..63` (oggi 50; `device_config.ini` ha `volume=10`).
Prova d'ascolto con l'utente (due volte): con **GPIO 233 = 0 il suono esce lo stesso**; il volume 30 è più piano.
Il GPIO 233 (uscita, valore 1) non è quindi l'interruttore dell'amplificatore, o non solo [?].
AuxCtrl per il collaudo usa anche `aplay -D hw:0,0 -f S16_LE file.wav`.

**C5 — carica** (`c5-carica.seq`; robot appoggiato sulla base dall'utente con lo stack fermo):
- **la carica parte da sola** appena il robot tocca i contatti (byte 7 = `03` dopo ~12 s), con il solo keepalive e
  heartbeat del banco di prova; tensione 16,1 → 16,3 V. Il "carica solo con lo stack acceso" della fase 1 dipendeva
  dal reset del GD32 senza keepalive, non da un comando;
- `0x9b 01` (caricatore): nessun effetto visibile;
- `0x9d 00`: byte 64 bit 5 a 0 (`0x62` → `0x42`) e bumper letti "premuti" (`06`): **bit 5 = sensori alimentati**.
  Spiega il frame "sulla base" del 2/10 con bumper `06`;
- **sulla base gli anti-caduta leggono `0f`** come da sollevato. Nessun byte riporta la corrente di carica.

**Modifica del blocco di sicurezza** dopo C5: "sollevato" ora vuol dire anti-caduta `0f` **e** byte 64 bit 5 = 1
**e** byte 7 = 0 (non sulla base). Test aggiornati (81 controlli) e GD32 finto della prova-guida con i sensori accesi.

**C8 — batteria nel tempo**: avviata il 3/10 la registrazione della curva di **carica** (`c8carica`, sulla base,
un campione ogni 5 s, max 3 h, stack fermo; poi lo stack riparte da solo). La curva di **scarica** richiede ore
fuori dalla base: da fare di notte, d'accordo con l'utente.

### Problema aperto: la carica si ferma a 16,3 V

Registrazione `c8carica` (50 min sulla base, stack fermo): tensione **piatta a 16,3 V** con byte 7 = `01` ("in
carica"). Poi 10 min con lo stack originale acceso e la spia: **stessa cosa** (16,3 V, `01`), quindi non dipende dal
nostro software. Fuori dalla base il robot era a 16,1–16,2 V: la corrente di carica è quasi nulla. Al riavvio
RobotApp ha suonato `sound_erro_low_battery.mp3`. Ipotesi più probabile: **batteria degradata** (due anni scarica,
in protezione il 1/10 a 11,3–14 V): una cella arriva a 4,2 V e la protezione ferma la carica, il totale resta sotto
16,8 V. Non verificabile dal software (non c'è un byte con la corrente di carica né le tensioni delle celle); la
curva di scarica dà la capacità. Nota: a 311 s di `c8carica` il byte 7 è passato per un attimo a `00` (contatto perso).

## 2026-10-03 sera — blocco D con l'utente

Sessione `d` (stack fermo, sensori accesi, lidar a potenza 68, niente motori). Strumenti: `tools/finestra.py`
(statistiche degli ultimi frame dal robot) e, da metà sessione, `spia guida --comandi FIFO` (comandi dal vivo:
`echo 0x8d:05 > /tmp/rtes/comandi`), aggiunto perché le sequenze a tempo erano impossibili da seguire a occhio.

**D1 — IMU** (rotazioni a mano). Le coppie sono velocità + angolo, non giroscopio + accelerometro come scrive
VacuumTiger:

| asse | velocità (0,01 °/s) | angolo (0,01°) | positivo |
|---|---|---|---|
| imbardata | byte 40 | byte 42 | antiorario visto dall'alto (90° a mano → 106° nell'angolo, 109° dall'integrale) |
| beccheggio | byte 44 | byte 46 | naso in giù (naso su di ~17° → −1730) |
| rollio | byte 48 | byte 50 | fianco sinistro in giù (su un fianco → +8700) |

Gravità (52/54/56, 0,01 g) = direzione "su" con assi **X indietro, Y destra, Z su** (naso su → X = −33, fianco
sinistro giù → Y = +99). I byte 90–95 sono una **copia delle velocità angolari** (stessi valori), non un
magnetometro. Codice: `vel_angolare[]` e `angolo[]` in `gd32_sensori_t`.

**D2 — IR**: mano a 5–10 cm ai lati e davanti → nessun byte cambia (nemmeno 10–15, 20–23, 28–31, 71, 73, 85–87,
sempre 0). **Non ci sono IR di ostacolo/seguimuro**: everest segue i muri con il lidar.

**D3 — anti-caduta** (mano sotto ogni sensore): **bit 0 sinistro laterale, bit 1 sinistro anteriore, bit 2 destro
anteriore, bit 3 destro laterale** (come VacuumTiger; la nostra convenzione della fase 1 era sbagliata). Mano sotto
il centro del davanti → bit 2 e 1. `cliff[0..3]` ora va da sinistra a destra con questa mappa.

**D4 — lidar** (scatola a distanze misurate): **0° = dietro, 90° = destra, 180° = davanti, 270° = sinistra**, angoli
crescenti in senso antiorario visto dall'alto; scatola a destra a 16 cm misurati → minimo a 98° e **0,163 m**
(distanze esatte al cm). Scarto di circa **+8°** fra lo 0 del lidar e l'asse del robot [?] (forse il campo 0,82).

**D5 — LED** (`0x8d`, un modo alla volta con conferma dell'utente):

| modo | aspetto | modo | aspetto |
|---|---|---|---|
| 0 | spento | 10 | rosso e giallo, lampeggio lento |
| 1 | azzurro fisso (normale) | 11 | azzurro, lampeggio lento (rete) |
| 2 | azzurro fisso | 12 | azzurro che respira (avvio) |
| 3 | giallo fisso | 13 | azzurro fisso |
| 4 | giallo che respira (in carica) | 14 | azzurro fisso |
| 5 | spento | 15 | azzurro fisso |
| 6 | giallo, lampeggio lento (batteria scarica) | 16 | rosso, lampeggio lento |
| 7 | rosso fisso | 17 | rosso + azzurro tenue, fisso |
| 8 | rosso, lampeggio veloce (errore) | 18 | giallo + rosso, fisso |
| 9 | spento | | |

Significati tra parentesi dal manuale del Proscenic M6 Pro. Per lo stato di guasto del nostro software: **8**.

**D5 — spazzola centrale**: `+30` → le setole spingono verso il **dietro** (verso normale di pulizia). `−30` →
**retromarcia alla massima velocità** (eco 100, ~750 mA) e **dopo ~4,2 s il GD32 la ferma da solo** (eco 0): un
valore negativo è la manovra di sblocco ("anti-groviglio"), non una velocità.

**D2 — IR della base** (robot a terra davanti alla base, girato a mano): il byte 6 si accende solo con il robot
rivolto verso la base (imbardata ~0–25°): `0x02 0x06 0x22 0x26 0x2A 0x2E 0xAA`. **Due ricevitori** (nibble alto e
basso), ogni bit 1/2/4/8 = un **fascio codificato** della base (se ne vedono più insieme). Toccando i contatti il
byte 7 va a `02` a intermittenza: **bit 1 = contatto, bit 0 = in carica**.

### Curva di scarica (C8, notte fra il 3 e il 4/10)

`c8scarica`: robot fermo a terra lontano dalla base, stack fermo, SoC acceso e **solo il lidar a potenza 68**
(~6 giri/s), un campione ogni 5 s, arresto automatico a 14,0 V (`--tensione-minima 140`; poi lo stack è ripartito da
solo). Dati in `build/robot/fase2/c8scarica/` (CSV, uscita, registro compresso).

| tensione | tempo trascorso | autonomia residua fino a 14,0 V |
|---|---|---|
| 15,9 V | 0 | 5 h 03 min |
| 15,5 V | 49 min | 4 h 14 min |
| 15,0 V | 2 h 02 min | 3 h 02 min |
| 14,5 V | 3 h 12 min | 1 h 51 min |
| 14,2 V | 4 h 13 min | 50 min |
| 14,0 V | 5 h 03 min | – |

Discesa quasi lineare (~0,1 V ogni 14–20 min), temperatura 32–34 °C. A 14,0 V (3,5 V per cella) si è ancora sopra
il ginocchio delle celle al litio (~13,2–13,6 V): la **capacità non sembra crollata**; la carica ferma a 16,3 V
fa pensare a un limite in alto (una cella che arriva prima a 4,2 V). Per il driver: tabella tensione → autonomia
con questo carico, da correggere per la caduta sotto carico (correnti dei motori note); soglie proposte: avviso
~14,2 V (LED 6), arresto dei motori ~13,8 V, da confermare nella fase 3.

### Architettura RT decisa a fine fase (dettagli in `PIANO.md`)
Driver fisso a priorità massima fuori dallo scheduling (con watchdog dei task, regolazione del lidar e soglie della
batteria); task con **RM partizionato** su `SCHED_FIFO`; **PIP** fra i task (verificato: `PTHREAD_PRIO_PROTECT`
non è supportato da musl, `pthread_mutexattr_setprioceiling` non esiste); scambio senza blocchi fra driver e task;
task a 20 ms svegliati dal frame. Lidar: ~2050 misure/s fisse (50 → 464 punti/giro, 80 → 288 punti/giro).

### Restano aperti (per la fase 3)
Scarto di ~8° del lidar e campo 0,82; GPIO 233 (non spegne l'audio); risposta `0x18` (`4`, `0x3f`); byte 2/5/66/72/84;
carica ferma a 16,3 V (batteria?); prova delle ruote **in avanti** solo a terra (sollevato il GD32 non va avanti) o in
modalità `01`; significato di `0x7a`, `0x84`.
