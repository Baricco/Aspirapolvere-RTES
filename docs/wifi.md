# Rete Wi-Fi del robot: progettazione

Cambio della rete Wi-Fi dall'app e rete del robot (`Maurice-WiFi`) per quando la rete salvata non c'è. Il codice lo
scrive Claude (robot, backend, app) e l'utente lo rivede, come per il resto dell'app (`docs/app.md`, sezione 1:
valgono tutte le regole generali). Niente codice prima che questo documento sia approvato.

## Indice

1. [Decisioni dell'utente](#1-decisioni-dellutente)
2. [La radio del robot](#2-la-radio-del-robot)
3. [Architettura](#3-architettura)
4. [Collegamento e rete del robot](#4-collegamento-e-rete-del-robot)
5. [Cambio di rete](#5-cambio-di-rete)
6. [Scansione](#6-scansione)
7. [Interfacce](#7-interfacce)
8. [Robot](#8-robot)
9. [Backend](#9-backend)
10. [App](#10-app)
11. [Effetti sull'analisi real-time](#11-effetti-sullanalisi-real-time)
12. [Prove](#12-prove)
13. [Decisioni del 2026-11-17 sui punti aperti](#13-decisioni-del-2026-11-17-sui-punti-aperti)
14. [Ordine di lavoro](#14-ordine-di-lavoro)
15. [Prove sul robot del 2026-11-17](#15-prove-sul-robot-del-2026-11-17)
16. [Prove sul robot del 2026-11-21](#16-prove-sul-robot-del-2026-11-21)
17. [Da decidere](#17-da-decidere)

---

## 1. Decisioni dell'utente

- **Il robot fa quello che dice l'utente.** Finché la rete salvata funziona usa quella. Quando non c'è più accende la
  sua rete e la lascia in mano all'utente: la rete del robot **non si spegne mai mentre un telefono è collegato**.
- **La rete del robot si spegne solo quando valgono tutte e due** (E, non O):
  1. nessun telefono è collegato alla rete del robot;
  2. il robot raggiunge il backend dalla sua rete (quella salvata tornata, o una nuova scelta dall'app).
- Rete del robot accesa dopo **30 secondi** senza la rete salvata (all'avvio o durante il funzionamento).
- Mentre la rete del robot è accesa, il robot **continua a cercare la rete salvata** e ci si ricollega da solo.
- **Una sola rete salvata.** Una rete scelta dall'app prende il posto di quella di prima solo se funziona; se non
  funziona il robot torna a cercare quella di prima.
- Verifica dopo il collegamento: prima il **server configurato nel robot** (il backend), poi, se non risponde, un ping
  a **8.8.8.8** per distinguere "backend irraggiungibile" da "internet irraggiungibile". Con internet ma senza backend
  il robot **resta sulla rete nuova** e l'app mostra un errore. Messaggio nell'app anche quando va tutto bene.
- Messaggi distinti: password sbagliata, rete non trovata, nessun indirizzo dal router, internet irraggiungibile,
  backend irraggiungibile, collegato.
- Schermata **"Rete Wi-Fi"** aperta da un pulsante in **Avanzate**: lista delle reti vicine (aperte e WPA/WPA2 con
  password), tocco su una rete → password → collegamento; pulsante per scrivere a mano il nome di una rete che non
  compare.
- Cambio di rete **solo a riposo**.
- **Solo nell'APK**: nella versione web il pulsante non c'è.
- Sulla rete del robot l'app parla in **HTTP** semplice (la rete è protetta da WPA2).
- L'app sceglie da sola con chi parlare: robot Online → backend; robot Offline → prova il robot su `192.168.5.1`.
  Nessun interruttore.
- La scansione può durare più dei 3 s di attesa dei comandi: per lei l'attesa del backend sale a 10 s. La password
  passa dal backend senza essere salvata né scritta nei registri.
- Tutti i tempi in `#define`.

## 2. La radio del robot

Il chip è un rtl8189es (solo 2,4 GHz). Il suo driver (`8189es.ko`) è compilato con la modalità concorrente di Realtek
e all'avvio crea due interfacce sulla stessa radio:

| Interfaccia | Uso |
|---|---|
| `wlan0` | collegamento alla rete salvata o a quella nuova: `wpa_supplicant` + `udhcpc` |
| `wlan1` | rete del robot: `hostapd` + `dnsmasq` + server HTTP locale |

Le due interfacce funzionano insieme (sezione 16). La radio è una sola, quindi **le due reti stanno sullo stesso
canale**: la rete del robot segue il canale della rete che `wlan0` usa o cerca (sezione 4.3).

Sul robot ci sono `hostapd` 2.6 e `wpa_supplicant` 2.6 (con `SCAN_INTERVAL` e l'interfaccia di controllo) e `dnsmasq`.
Busybox non ha un server web.

## 3. Architettura

```
 App (APK) ──HTTPS──▶ Backend ◀──HTTPS── vacuum: thread dei comandi ──socket locale──▶ rtes-wifi
     │                   ▲                                                              │  ├─ wlan0: wpa_supplicant, udhcpc
     │                   └──────────── HTTPS (prova del backend, da wlan0) ─────────────┤  └─ wlan1: hostapd, dnsmasq
     └──── HTTP 192.168.5.1 (wlan1, solo con la rete del robot accesa) ─────────────────┘
```

**`rtes-wifi`**: programma in C, separato da `vacuum`, avviato da procd. Gestisce tutto il Wi-Fi: i processi figli, la
rete salvata, le scansioni, il cambio, la prova del collegamento, la rete del robot e il server HTTP locale.

Perché separato da `vacuum`:
- l'SSH dipende dal Wi-Fi e deve restare raggiungibile sempre (`docs/PIANO.md`, fase 6): `vacuum` si riavvia dopo un
  guasto o un programma nuovo, e dopo 3 uscite resta il custode; il Wi-Fi non deve seguirlo;
- i task real-time e i loro WCET non vengono toccati (sezione 11);
- funziona anche senza `vacuum` (per esempio con il custode acceso o durante le prove col driver).

`vacuum` resta l'unico che riceve i comandi dal backend: il suo thread dei comandi passa a `rtes-wifi` quelli del
Wi-Fi attraverso un **socket locale**, e il thread dello stato mette nello stato del robot quello che `rtes-wifi`
racconta. Il server HTTP locale di `rtes-wifi` offre **le stesse richieste con le stesse risposte** del backend: l'app
usa lo stesso codice nei due casi e cambia solo l'indirizzo.

## 4. Collegamento e rete del robot

Le due interfacce hanno ciascuna il proprio stato, indipendente dall'altra.

### 4.1 `wlan0`: il collegamento

`wpa_supplicant` gira **sempre** su `wlan0`, con la rete salvata, e la cerca da solo finché non la trova.

| Modo | Cosa vuol dire |
|---|---|
| `connecting` | `wlan0` senza indirizzo: cerca la rete salvata |
| `station` | `wlan0` collegato (`wpa_state=COMPLETED`) e con un indirizzo |
| `testing` | cambio di rete in corso (sezione 5) |

- Con `wlan0` senza rete, `wpa_supplicant` cerca la rete salvata ogni `WIFI_RETRY_SCAN_S` (`SCAN_INTERVAL`).
- Un indirizzo ottenuto dalla rete salvata basta per restare in `station`, anche senza internet: se internet manca è
  un problema del router, non della rete scelta.

### 4.2 `wlan1`: la rete del robot

**Si accende** quando `wlan0` è senza indirizzo da `WIFI_LOST_TIMEOUT_S` (30 s), fuori da un cambio di rete. Vale
all'avvio, con la rete persa durante il funzionamento (router spento, robot portato lontano) e dopo un cambio fallito.
Senza rete salvata si accende subito.

**Si spegne** quando valgono tutte e due:
1. nessun telefono collegato da `WIFI_AP_IDLE_S` (30 s);
2. `wlan0` in `station` e la prova del backend riuscita (`GET /api/device/ping`, la stessa del cambio, sezione 5).

Con la prima vera e la seconda no (`wlan0` senza rete, o backend che non risponde), `rtes-wifi` ripete la prova ogni
`WIFI_BACKEND_RECHECK_S` (30 s) e la rete del robot resta accesa. Un telefono che si collega di nuovo azzera l'attesa
della prima.

**Telefoni collegati**: `rtes-wifi` li conta dagli eventi di `hostapd` (`AP-STA-CONNECTED`, `AP-STA-DISCONNECTED`) sul
suo socket di controllo. Un telefono uscito di portata senza salutare lo toglie `hostapd` dopo
`WIFI_AP_INACTIVITY_S` (60 s, `ap_max_inactivity`).

**Com'è fatta**: SSID `Maurice-WiFi`, WPA2-PSK solo CCMP, indirizzo `192.168.5.1/24` su `wlan1`, DHCP da `dnsmasq`
per `192.168.5.10-50` senza DNS (`port=0`) e senza gateway, così il telefono non ci cerca internet. SSID e password
stanno nella configurazione privata (sezione 8.2), mai nei file versionati. L'SSH resta raggiungibile su
`192.168.5.1` e, quando `wlan0` è collegato, anche dalla rete di casa (dropbear ascolta su tutte le interfacce).

### 4.3 Canale

- All'accensione la rete del robot usa il canale dell'ultima rete a cui `wlan0` è stato collegato; se non c'è,
  `WIFI_AP_CHANNEL` (6).
- Se la rete che `wlan0` cerca (quella salvata, o quella di un cambio) compare in una scansione su un canale diverso da
  quello della rete del robot, `rtes-wifi` riaccende `hostapd` su quel canale. I telefoni collegati perdono la rete del
  robot per qualche secondo e si ricollegano da soli.

### 4.4 Esempi

| Situazione | Cosa succede |
|---|---|
| Router spento a robot in base | dopo 30 s si accende `Maurice-WiFi`; `wlan0` continua a cercare il router |
| Router riacceso, nessun telefono collegato | `wlan0` si ricollega, il backend risponde, `Maurice-WiFi` si spegne |
| Router riacceso, telefono collegato a `Maurice-WiFi` | `wlan0` si ricollega; `Maurice-WiFi` resta accesa finché il telefono non se ne va da 30 s |
| Rete nuova scelta dal telefono su `Maurice-WiFi`, riuscita | come la riga sopra, con la rete nuova salvata |
| Rete nuova scelta, fallita | `wlan0` torna a cercare la rete salvata; `Maurice-WiFi` resta accesa |
| Router acceso ma server irraggiungibile | se `Maurice-WiFi` era già accesa resta accesa; se era spenta non si accende (`wlan0` ha l'indirizzo) |

## 5. Cambio di rete

Richiesta: SSID, tipo (`open` o `wpa`) e password (vuota per le reti aperte, 8-63 caratteri per WPA).

1. **Controlli** (rifiuti subito, con `reason`): `not_resting` (navigazione non a riposo, sezione 7.1), `wifi_busy`
   (cambio già in corso), `invalid_password`, `invalid_ssid` (vuoto o oltre 32 byte).
2. **Risposta subito "accettato"**; l'esito arriva dopo (sezione 7.4). Il cambio prende il numero successivo
   (`change_number`), così l'app riconosce l'esito del suo cambio.
3. La rete del robot **resta com'è**: accesa o spenta. Se è accesa e la rete nuova è nell'ultima scansione su un altro
   canale, `hostapd` passa a quel canale prima del collegamento (sezione 4.3).
4. Aggiunge la rete nuova a `wpa_supplicant` **come seconda rete, senza salvare** (`ADD_NETWORK`, `SET_NETWORK`,
   `SELECT_NETWORK`): la rete di prima resta nella sua configurazione finché la nuova non funziona.
5. **Collegamento** entro `WIFI_ASSOC_TIMEOUT_S` (30 s). `wpa_supplicant` distingue:
   - password sbagliata (evento `CTRL-EVENT-SSID-TEMP-DISABLED` con `reason=WRONG_KEY`) → `wrong_password`;
   - rete mai vista nelle scansioni → `not_found`.
6. **Indirizzo** da `udhcpc` entro `WIFI_DHCP_TIMEOUT_S` (20 s), altrimenti `no_address`.
7. **Prova del backend**: `GET /api/device/ping` con il token del robot, in HTTPS con le stesse radici di `vacuum`,
   `WIFI_BACKEND_TRIES` tentativi (2) da `WIFI_BACKEND_TIMEOUT_S` (10 s). Risponde 200 → `ok`.
8. Se il backend non risponde: **ping a 8.8.8.8**, `WIFI_PING_TRIES` (3) da `WIFI_PING_TIMEOUT_S` (2 s), con un socket
   ICMP (`rtes-wifi` gira come root). Risponde → `backend_unreachable`, altrimenti `internet_unreachable`.

| Esito | Cosa fa il robot | Messaggio nell'app |
|---|---|---|
| `ok` | salva la rete nuova (toglie la vecchia, `SAVE_CONFIG`) e resta | "Maurice è collegato a «Casa»" |
| `backend_unreachable` | salva la rete nuova e resta; i thread di rete di `vacuum` riprovano il server da soli, e appena lo raggiungono l'esito arriva all'app con lo stato | "Maurice è su «Casa» e ha internet, ma non raggiunge il server" |
| `internet_unreachable` | torna alla rete di prima | "«Casa» non porta a internet" |
| `no_address` | torna alla rete di prima | "Il router di «Casa» non ha dato un indirizzo a Maurice" |
| `wrong_password` | torna alla rete di prima | "Password sbagliata per «Casa»" |
| `not_found` | torna alla rete di prima | "Maurice non trova «Casa»" |

- **Ritorno alla rete di prima**: toglie la rete nuova, riabilita la vecchia e passa a `connecting`; da lì valgono le
  regole della sezione 4 (rete del robot accesa dopo 30 s senza indirizzo, se non lo era già).
- Durante il cambio (`testing`) l'attesa dei 30 s della sezione 4.2 non conta.
- L'esito dell'ultimo cambio resta in memoria in `rtes-wifi` (numero, SSID, esito, ora) finché non arriva un altro
  cambio. Non va su disco: dopo un riavvio del robot non serve più.
- Il file `/etc/wifi/wpa_supplicant.conf` si scrive solo con `SAVE_CONFIG` di `wpa_supplicant`. Prima della prima
  modifica `rtes-wifi` ne tiene una copia `.bak`.

## 6. Scansione

- Sempre su `wlan0`, con la rete del robot accesa o spenta: `SCAN`, attesa dell'evento `CTRL-EVENT-SCAN-RESULTS` (al
  massimo `WIFI_SCAN_TIMEOUT_S`, 8 s), poi `SCAN_RESULTS`. `FAIL-BUSY` (una scansione già in corso) → si aspetta
  quella. I telefoni collegati alla rete del robot restano collegati (sezione 16).
- **Filtro**: solo reti con SSID visibile, aperte o WPA/WPA2 con password (`-PSK` nei flag, oppure nessun flag di
  sicurezza). Si scartano WEP, Enterprise (`EAP`) e reti solo WPA3 (`SAE` senza `PSK`). Stesso SSID più volte (più
  punti di accesso): una riga sola con il segnale più forte. Ordine: segnale.
- Per ogni rete: `ssid`, `signal_dbm`, `security` (`open`/`wpa`), `in_use`. Il canale resta dentro `rtes-wifi` (sezione
  4.3).
- Gli SSID arrivano da `wpa_supplicant` con i caratteri speciali scritti come `\xNN`: `rtes-wifi` li riporta ai byte
  veri, e in JSON viaggiano in UTF-8.

## 7. Interfacce

### 7.1 `vacuum` ↔ `rtes-wifi`

Socket Unix di tipo stream `/tmp/rtes-wifi.sock` (permessi 0600), una richiesta JSON per riga e una risposta per riga.
`vacuum` apre una connessione per richiesta.

| Richiesta | Risposta |
|---|---|
| `{"type":"scan"}` | `{"ok":true,"data":{"scanned_at":...,"networks":[...]}}` |
| `{"type":"connect","ssid":"...","security":"wpa","password":"..."}` | `{"ok":true,"data":{"change_number":5}}` o `{"ok":false,"reason":...,"message":...}` |
| `{"type":"status"}` | lo stato del Wi-Fi (sezione 7.4) |
| `{"type":"resting","value":true}` | `{"ok":true}` |

**Riposo**: il controllo `not_resting` lo fa solo `rtes-wifi`, per le richieste di tutte e due le strade: `vacuum` gli
manda `resting` a ogni cambio e ogni 10 s; senza notizie da più di 30 s (`vacuum` fermo o custode) il robot è fermo e
conta come a riposo. `vacuum` passa i comandi del Wi-Fi così come sono.

### 7.2 Server HTTP locale (solo con la rete del robot accesa)

Su `192.168.5.1:80`, solo sull'interfaccia della rete del robot, niente autenticazione (protegge la password WPA2).

| Richiesta | Come la richiesta del backend |
|---|---|
| `GET /api/wifi/networks` | `GET /api/robot/wifi/networks` |
| `POST /api/wifi/connect` | `POST /api/robot/wifi/connect` |
| `GET /api/wifi/status` | il campo `wifi` dello stato (sezione 7.4) |

Server minimo scritto a mano (richiesta fino a 4 KB, solo `Content-Length`, risposta e chiusura).

### 7.3 Backend

| Richiesta dell'app | `type` al robot | Note e rifiuti |
|---|---|---|
| `GET /api/robot/wifi/networks` | `wifi_scan` | attesa dell'esito 10 s invece di 3 s; l'esito porta i dati |
| `POST /api/robot/wifi/connect` `{"ssid","security","password"}` | `wifi_connect` | `not_resting`, `wifi_busy`, `invalid_ssid`, `invalid_password`; attesa dell'esito 6 s; 200 = accettato, l'esito arriva nello stato |

Richiesta del robot: `GET /api/device/ping` (solo il controllo del token, risponde `{"ok":true}`), usata da
`rtes-wifi` per la prova del backend.

**Esito con i dati**: l'esito di un comando è `{ok, reason, message}` con un campo facoltativo `data` (per
`wifi_scan`: `{"scanned_at", "networks"}`) che il backend gira all'app così com'è.

### 7.4 Stato del robot

Campo `wifi` nello stato JSON (`docs/app.md`, sezione 4.1), letto da `vacuum` con `status` ogni
`WIFI_STATUS_INTERVAL` (2 s) e mandato subito quando cambiano modo, rete, rete del robot, telefoni collegati o cambio:

```json
"wifi":{"mode":"station","ssid":"Casa","signal_dbm":-58,"ip":"192.168.1.40","saved_ssid":"Casa",
        "ap":{"on":false,"clients":0},
        "change":{"number":5,"ssid":"Casa","state":"done","result":"ok","time":1763390000}}
```

- `mode`: `connecting`, `station`, `testing` (solo `wlan0`, sezione 4.1).
- `ssid`, `signal_dbm`, `ip`: il collegamento di `wlan0`, `null` senza collegamento.
- `saved_ssid`: la rete salvata, `null` se non c'è.
- `ap.on`: rete del robot accesa; `ap.clients`: telefoni collegati.
- `change.state`: `none`, `testing`, `done`; `change.result`: gli esiti della sezione 5.
- Con `vacuum` senza risposta da `rtes-wifi` il campo manca.

## 8. Robot

### 8.1 File

| Percorso | Modifiche |
|---|---|
| `wifi/manager.c/.h` | stato di `wlan0` e della rete del robot separati (sezione 4); regole di accensione e spegnimento; niente più alternanza; scansione senza `refresh` |
| `wifi/radio.h`, `radio_real.c` | `wpa_supplicant` sempre acceso su `wlan0` con `SCAN_INTERVAL`; `hostapd`, `dnsmasq` e indirizzo su `wlan1`; canale della rete del robot; telefoni collegati dagli eventi di `hostapd`; canale nelle reti della scansione |
| `wifi/radio_sim.c` | le stesse funzioni con reti e telefoni finti, per `--simulate` |
| `wifi/supplicant.c/.h` | il percorso del socket di controllo come parametro, così lo stesso codice parla con `wpa_supplicant` e con `hostapd` |
| `wifi/server.c` | campi nuovi dello stato (sezione 7.4); scansione senza `refresh` |
| `tasks/vacuum/cloud/app_commands.c` | `wifi_scan` senza `refresh` |
| `tasks/vacuum/cloud/robot_state.c` | rete del robot e telefoni collegati nella chiave che fa mandare subito lo stato |
| `tasks/vacuum/cloud/config.h`, `config.example.h` | costanti della sezione 8.2 |

`rtes-wifi` resta in `/usr/sbin/rtes-wifi`, lanciato da `/usr/sbin/rtes-wifi-run` (sezione 8.3); `./rtes.sh install`
lo copia e riavvia il Wi-Fi solo se è cambiato.

### 8.2 Configurazione

In `tasks/vacuum/cloud/config.h` (privato) e `config.example.h` (valori finti):

| Costante | Valore | Cosa |
|---|---|---|
| `WIFI_INTERFACE` | `"wlan0"` | collegamento alla rete salvata o nuova |
| `WIFI_AP_INTERFACE` | `"wlan1"` | rete del robot (nuova) |
| `WIFI_LOST_TIMEOUT_S` | 30 | senza indirizzo per tanto (all'avvio o dopo): rete del robot accesa |
| `WIFI_RETRY_SCAN_S` | 10 | con `wlan0` senza rete, una ricerca della rete salvata ogni tanto (nuova, sezione 17) |
| `WIFI_AP_IDLE_S` | 30 | nessun telefono collegato da tanto: prima condizione per spegnere la rete del robot (nuova) |
| `WIFI_BACKEND_RECHECK_S` | 30 | con la rete del robot accesa e nessun telefono, prova del backend ogni tanto (nuova) |
| `WIFI_AP_INACTIVITY_S` | 60 | telefono senza risposta da tanto: `hostapd` lo considera uscito (nuova) |
| `WIFI_AP_CHANNEL` | 6 | canale della rete del robot senza un collegamento precedente |

Tolte: `WIFI_CONNECT_TIMEOUT_S` (sostituita da `WIFI_LOST_TIMEOUT_S`), `WIFI_AP_DURATION_S`, `WIFI_AP_RETRY_S`,
`WIFI_AP_SCAN_S`. Restano com'erano tutte le altre (SSID e password della rete del robot, indirizzi, tempi del cambio
e della scansione).

### 8.3 Sicurezza dell'SSH

- `wlan0` non si spegne mai: con la rete salvata presente l'SSH resta raggiungibile dalla rete di casa anche con la
  rete del robot accesa.
- `rtes-wifi-run` come oggi: se `rtes-wifi` esce lo rilancia dopo 5 s; **3 uscite in 10 minuti** → non lo rilancia
  più, chiude i processi del Wi-Fi e avvia il `wifiManager` di fabbrica.
- I processi figli si chiudono da soli se `rtes-wifi` muore (`PR_SET_PDEATHSIG`), e all'avvio `rtes-wifi` chiude
  quelli rimasti.
- Se `hostapd` su `wlan1` non parte o si chiude, `rtes-wifi` lo riavvia ogni 5 s come oggi; `wlan0` non ne risente.

## 9. Backend

- `robot_api.py`: `wifi_scan` senza `refresh`.
- `config.py`: attesa dell'esito di `wifi_connect` 6 s in `COMMAND_TIMEOUTS_S` (il robot ne usa fino a 6 prima di
  rispondere).
- `dev/fake_robot.py`: campi nuovi dello stato (`saved_ssid`, `ap`), scansione senza `refresh`.
- Prove automatiche aggiornate.
- Invariati: `/api/device/ping`, esito con `data`, password mai nel registro né nei log.

## 10. App

### 10.1 Dove

- **Avanzate**: pulsante **"Rete Wi-Fi"**, solo nell'APK (`kIsWeb` → non c'è).
- Schermata `screens/wifi/`, nello stile Vetro delle altre.

### 10.2 Con chi parla

All'apertura della schermata e poi ogni 3 s:
1. robot **Online** → backend (`/api/robot/wifi/...`);
2. robot **Offline** → `GET http://192.168.5.1/api/wifi/status` con attesa di 2 s: se risponde, **modo locale**
   (`http://192.168.5.1/api/wifi/...`);
3. nessuno dei due per **`wifiLocalMisses` controlli di fila (3)** → schermata con la spiegazione per collegarsi a
   `Maurice-WiFi`. Prima di arrivare a 3 la schermata resta com'è (lista compresa) con una riga "Maurice non risponde,
   riprovo…".

Su Android, nel modo locale, le richieste dell'app vanno forzate sul Wi-Fi (`bindProcessToNetwork` in
`MainActivity.kt`) e l'HTTP in chiaro è permesso solo verso `192.168.5.1` (`network_security_config.xml`).

### 10.3 Schermata

- **In alto**, secondo con chi parla:
  - backend: rete in uso con segnale e indirizzo; se la rete del robot è accesa, anche "Rete di Maurice accesa · N
    telefoni collegati";
  - modo locale: "Sei sulla rete di Maurice", e sotto cosa fa `wlan0`: "Maurice cerca «Casa»" (`saved_ssid`),
    "Maurice è collegato a «Casa»", oppure "Nessuna rete salvata";
  - durante un cambio chiesto dal backend, con il robot Offline: "Aspetto Maurice…" (mai la spiegazione per
    collegarsi a `Maurice-WiFi`), fino all'esito o ai 3 minuti di `wifiChangeWait`;
  - nessuno: la spiegazione, con nome e password della rete del robot e il testo "Maurice accende la sua rete quando
    non trova la rete salvata per 30 secondi, e la tiene accesa finché c'è un telefono collegato o finché non
    raggiunge il server."
- **"Riprova"**: mostra la rotellina finché il controllo finisce (se un controllo è già in corso, aspetta quello); se
  il robot non risponde scrive "Maurice non risponde su 192.168.5.1: il telefono è collegato a «Maurice-WiFi»?".
- **Lista delle reti**: nome, tacche del segnale (≥ -55, ≥ -67, ≥ -78, sotto), lucchetto se `security = wpa`, segno
  sulla rete in uso. Pulsante **Aggiorna** (una scansione nuova, senza spegnere niente). Sotto la lista: "Maurice
  vede solo le reti a 2,4 GHz".
- **Altra rete…**: nome scritto a mano e tipo (con password / aperta).
- Tocco su una rete: finestra con la password (nascosta, con l'occhio per mostrarla), oppure solo conferma per le reti
  aperte; con robot non a riposo il pulsante è spento e dice perché. **Il cambio va alla strada che c'era al tocco**:
  se nel frattempo è cambiata, non parte e l'app scrive "Maurice non risponde più da qui: riprova".
- Dopo l'invio, pannello del cambio con le fasi:
  - **dal backend**: il robot va Offline per qualche secondo; l'app guarda `wifi.change` nello stato e mostra l'esito
    con il messaggio della sezione 5 quando il robot torna Online; dopo 3 minuti senza esito: "Maurice non è tornato
    online: può essere il server irraggiungibile dalla rete «Casa»";
  - **dalla rete del robot**: il telefono resta su `Maurice-WiFi` e vede l'esito da `/api/wifi/status`. Con `ok`:
    "Maurice è collegato a «Casa» e raggiunge il server. Puoi tornare sulla tua rete: Maurice spegne la sua quando non
    c'è più nessuno collegato." Con un errore: il messaggio della sezione 5 e "La rete di Maurice resta accesa: puoi
    riprovare."
- La lista e il pannello si aggiornano a ogni cambio di modo, rete, rete del robot o esito.

## 11. Effetti sull'analisi real-time

- Task real-time: **nessuna modifica**.
- `vacuum`: il thread dei comandi (core 3, nice 5) aspetta `rtes-wifi` per la scansione (fino a 10 s) senza toccare i
  task; il thread dello stato legge `status` ogni 2 s con un socket locale.
- `rtes-wifi`: processo normale (SCHED_OTHER) sul core 3 con nice come i thread di rete. Lavora quasi solo in attesa
  (socket, eventi, timer). Si misura con gli altri nei WCET (carico di sistema).

## 12. Prove

**Fuori casa (senza robot):**
- `rtes-wifi --simulate` sul PC: rete salvata che sparisce e torna, telefoni finti che si collegano e se ne vanno,
  backend che risponde o no; regole della sezione 4 con i tempi accorciati; richieste del server HTTP locale.
- Backend: prove automatiche; `fake_robot.py` con i campi nuovi.
- App: emulatore Android verso il backend di sviluppo, e verso `rtes-wifi --simulate` per il modo locale.

**In casa (con il robot, dopo averne parlato):**
1. avvio con la rete salvata: `Maurice-WiFi` spenta;
2. router spento: `Maurice-WiFi` accesa dopo 30 s; telefono collegato per 10 minuti: nel registro nessuna uscita
   del telefono, ping al telefono durante le ricerche di `wlan0` (scelta di `WIFI_RETRY_SCAN_S`);
3. router riacceso, telefono collegato: `wlan0` si ricollega, `Maurice-WiFi` resta accesa; telefono staccato:
   `Maurice-WiFi` spenta dopo 30 s;
4. router riacceso, nessun telefono: `Maurice-WiFi` spenta appena il backend risponde;
5. canale del router cambiato mentre è spento: `Maurice-WiFi` passa al canale nuovo e il telefono si ricollega;
6. cambio dalla rete del robot: password giusta (hotspot del telefono), password sbagliata, rete che non esiste,
   hotspot a 5 GHz (non compare); esito visto sul telefono senza lasciare `Maurice-WiFi`;
7. cambio dal backend: password giusta e sbagliata;
8. SSH su `192.168.5.1` e dalla rete di casa con `Maurice-WiFi` accesa;
9. `rtes-wifi` che si chiude 3 volte: parte il `wifiManager` di fabbrica.

## 13. Decisioni del 2026-11-17 sui punti aperti

1. **Backend irraggiungibile**: se il robot non torna Online entro `wifiChangeWait` (3 minuti) l'app scrive "Maurice
   non è tornato online: può essere il server irraggiungibile dalla rete «Casa»"; quando il backend torna
   raggiungibile il robot manda con lo stato l'esito vero (`backend_unreachable`, con l'ora) e l'app lo mostra.
2. **Internet irraggiungibile**: il robot torna alla rete di prima (tabella della sezione 5).
3. **Password della rete del robot nell'app**: nella configurazione privata dell'app (`config.json`, non versionato),
   mostrata nella schermata che chiede di collegarsi a `Maurice-WiFi`.

## 14. Ordine di lavoro

1. Approvazione di questo documento (sezione 17).
2. `rtes-wifi` con `--simulate` e le prove sul PC.
3. `vacuum` (`cloud/`) e backend, con `fake_robot.py`.
4. App.
5. Revisione dell'utente.
6. Prove sul robot in casa (sezione 12), poi `./rtes.sh install`, deploy del backend e APK.
7. Aggiornamento di `docs/app.md` (sezioni 4.1, 5.3, 8.5, 8.6, 10.4) e di `docs/PIANO.md` (fase 6).

## 15. Prove sul robot del 2026-11-17

Prova a tempo (rtes-wifi in `/tmp`, 20 minuti al massimo, poi il Wi-Fi di prima di nuovo da solo), poi installazione:
- cambi dalla rete di casa all'hotspot del telefono e ritorno: riusciti; password sbagliata: `wrong_password` e ritorno a
  casa;
- hotspot spento: rete del robot accesa, telefono su `Maurice-WiFi`, lista e cambio verso la rete di casa: riusciti;
- difetti trovati e corretti:
  - l'indirizzo dato dall'hotspot restava sull'interfaccia dopo lo spegnimento, e il robot si credeva collegato:
    ora "collegato" vuol dire `wpa_state=COMPLETED` e un indirizzo (`radio_real.c`), e all'avvio si toglie l'indirizzo
    lasciato dal Wi-Fi di prima;
  - dopo un cambio la richiesta lunga dei comandi di `vacuum` restava appesa alla rete di prima fino a 40 s e la lista
    dell'app non si ricaricava: ora `network.c` interrompe una richiesta quando l'indirizzo del Wi-Fi cambia; l'app
    segue sempre modo e rete del robot, ricarica la lista a ogni cambio e ricontrolla con chi parlare ogni 3 s;
  - l'uscita di `rtes-wifi` non arrivava al registro: `stdout`/`stderr` di procd;
- attesa prima della rete del robot abbassata da 2 minuti a 30 s, richiesta dell'utente;
- installazione: riavvio del robot (rtes-wifi collegato 5 s dopo l'avvio) e ripiego (3 uscite di `rtes-wifi` →
  `wifiManager` di fabbrica sulla rete di casa) riusciti.

## 16. Prove sul robot del 2026-11-21

File in `build/robot/prova-wlan1-21-11/` (registro del robot, uscita di `hostapd`, misure).

**Prova vera con il router spento (11:18-11:48, rete del robot su `wlan0` sola, a turni di 5 minuti accesa e 1
minuto spenta):**
- router spento alle 11:18:51, rete del robot accesa alle 11:19:24 (33 s);
- rete del robot spenta ogni 5 minuti per circa 60 s anche con il telefono collegato (11:24, 11:30, 11:36, 11:42,
  11:48); a ogni "Aggiorna" spenta per la scansione: il telefono è tornato dopo 2 minuti (11:44:02 → 11:46:10);
- il telefono è uscito da solo dalla rete del robot due volte con la rete accesa (11:21:08, 11:22:54);
- alle 11:48:42 il robot si è ricollegato da solo alla rete di casa;
- nell'app: lista che compare e sparisce, "Riprova" senza nessun segno. Giudizio dell'utente: esperienza pessima,
  da qui le decisioni della sezione 1.

**Prova di `wlan1` (12:01-12:06, a tempo, spenta da sola):**
- driver `8189es` con `CONFIG_CONCURRENT_MODE`; `wlan1` già presente all'avvio, modo AP supportato;
- `hostapd` su `wlan1` (rete di prova, canale 9 come il router) con `wlan0` collegato alla rete di casa: telefono
  collegato e con indirizzo da `dnsmasq`, ping dal robot al telefono 3 su 3;
- `wlan0` sempre collegato durante la prova (segnale da −33 a −36 dBm, router raggiungibile a ogni controllo), SSH
  sempre raggiungibile;
- scansione su `wlan0` con il telefono collegato a `wlan1`: 6 reti trovate, telefono rimasto collegato, 1 ping perso
  su 12.

## 17. Da decidere

1. **`WIFI_RETRY_SCAN_S`**: ogni quanto `wlan0` cerca la rete salvata con la rete del robot accesa. Una ricerca fa
   perdere al telefono circa un ping su dodici (sezione 16). Proposta: **10 s** (il router tornato viene trovato entro
   10 s); da confermare con la prova 2 della sezione 12.
2. **`WIFI_AP_IDLE_S`**: quanto tempo senza telefoni collegati prima di poter spegnere la rete del robot. Proposta:
   **30 s**, per non spegnerla quando Android stacca e riattacca il telefono per un attimo.
3. **Nessun ripiego sul funzionamento a una sola interfaccia** se `wlan1` non partisse: il driver la crea sempre, e il
   ripiego sul `wifiManager` di fabbrica (sezione 8.3) resta per l'SSH. Proposta: niente ripiego, meno codice.
4. **Server irraggiungibile a lungo con la rete di casa che funziona**: la rete del robot non si accende (si accende
   solo senza indirizzo su `wlan0`). Proposta: va bene così, perché `vacuum` riprova il server da solo e la rete del
   robot non lo aiuterebbe.
