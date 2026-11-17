# Rete Wi-Fi del robot: progettazione

Cambio della rete Wi-Fi dall'app, chiesto dall'utente il 2026-11-13 come ultima cosa del progetto e progettato il
2026-11-17. Niente codice prima che questo documento sia approvato. Il codice lo scrive Claude (robot, backend, app) e
l'utente lo rivede, come per il resto dell'app (`docs/app.md`, sezione 1: valgono tutte le regole generali).

## Indice

1. [Decisioni dell'utente](#1-decisioni-dellutente)
2. [Com'è oggi](#2-comè-oggi)
3. [Architettura](#3-architettura)
4. [Modi del Wi-Fi](#4-modi-del-wi-fi)
5. [Cambio di rete](#5-cambio-di-rete)
6. [Scansione](#6-scansione)
7. [Interfacce](#7-interfacce)
8. [Robot](#8-robot)
9. [Backend](#9-backend)
10. [App](#10-app)
11. [Effetti sull'analisi real-time](#11-effetti-sullanalisi-real-time)
12. [Prove](#12-prove)
13. [Decisioni sui punti aperti](#13-decisioni-del-2026-11-17-sui-punti-aperti)
14. [Ordine di lavoro](#14-ordine-di-lavoro)
15. [Prove sul robot del 2026-11-17](#15-prove-sul-robot-del-2026-11-17)

---

## 1. Decisioni dell'utente

Un **solo meccanismo** per le due situazioni: robot online (si cambia rete passando dal backend) e robot senza rete (il
telefono si collega alla rete del robot).

- Il robot prova la rete salvata; se non ci riesce, accende **la sua rete** (`Maurice-WiFi`, password
  scelta dall'utente e scritta solo nella configurazione privata) a cui si collega il telefono.
- **Una sola rete salvata.** Una rete scelta dall'app prende il posto di quella di prima solo se funziona; se non
  funziona il robot torna alla rete di prima; se non va neanche quella (o all'avvio non va fin da subito) accende la
  sua rete.
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
- Tempi proposti e accettati: rete del robot accesa dopo **30 secondi** senza rete (erano 2 minuti, abbassati dall'utente il 2026-11-17 dopo le prove); poi **5 minuti** di rete del robot
  e **1 minuto** di nuovo tentativo sulla rete salvata, a ripetizione; scansione con la rete del robot accesa spegnendola
  per **5 s**. Tutti in `#define`.
- La scansione può durare più dei 3 s di attesa dei comandi: per lei l'attesa del backend sale a 10 s. La password
  passa dal backend senza essere salvata né scritta nei registri.

## 2. Com'è oggi

Dalla fase 6 (`docs/PIANO.md`, 2026-11-13) il Wi-Fi è gestito da `install/etc/init.d/rtes-wifi` (procd, START=21):

- `wpa_supplicant` con la rete salvata in `/etc/wifi/wpa_supplicant.conf`, socket di controllo in `/etc/wifi/sockets`;
- `rtes-wifi-dhcp`: avvia `udhcpc` solo quando `wlan0` è collegato (udhcpc 1.24 avviato prima si blocca);
- `wpa_cli -a rtes-wifi-event`: a ogni collegamento senza indirizzo fa ripartire `udhcpc`;
- `rtes-wifi-check` (solo all'avvio): senza indirizzo dopo 60 s avvia il `wifiManager` di fabbrica, che accende la sua
  rete `192.168.5.1` per l'app originale. Serve solo a non perdere l'SSH.

Sul robot ci sono anche `hostapd` 2.6 e `dnsmasq` (spento nella fase 6). Il chip (rtl8189es, solo 2,4 GHz) sa fare da
punto di accesso: lo faceva il `wifiManager` (`softap_up`, `/etc/wifi/hostapd.conf`). Busybox non ha un server web.

## 3. Architettura

```
 App (APK) ──HTTPS──▶ Backend ◀──HTTPS── vacuum: thread dei comandi ──socket locale──▶ rtes-wifi
     │                   ▲                                                              │  ├─ wpa_supplicant
     │                   └──────────── HTTPS (prova del backend) ───────────────────────┤  ├─ udhcpc
     └──── HTTP 192.168.5.1 (solo con la rete del robot accesa) ────────────────────────┘  ├─ hostapd
                                                                                           └─ dnsmasq
```

**`rtes-wifi`**: un programma nuovo in C, separato da `vacuum`, avviato da procd al posto degli script di oggi.
Gestisce tutto il Wi-Fi: i processi figli, la rete salvata, le scansioni, il cambio, la prova del collegamento, la
rete del robot e il server HTTP locale.

Perché separato da `vacuum`:
- l'SSH dipende dal Wi-Fi e deve restare raggiungibile sempre (`docs/PIANO.md`, fase 6): `vacuum` si riavvia dopo un
  guasto o un programma nuovo, e dopo 3 uscite resta il custode; il Wi-Fi non deve seguirlo;
- i task real-time e i loro WCET non vengono toccati (sezione 11);
- funziona anche senza `vacuum` (per esempio con il custode acceso o durante le prove col driver).

`vacuum` resta l'unico che riceve i comandi dal backend: il suo thread dei comandi passa a `rtes-wifi` quelli del
Wi-Fi attraverso un **socket locale**, e il thread dello stato mette nello stato del robot quello che `rtes-wifi`
racconta. Il server HTTP locale di `rtes-wifi` offre **le stesse richieste con le stesse risposte** del backend: l'app
usa lo stesso codice nei due casi e cambia solo l'indirizzo.

## 4. Modi del Wi-Fi

| Modo | Cosa fa | Quando finisce |
|---|---|---|
| `station` | `wpa_supplicant` + `udhcpc` sulla rete salvata | perde l'indirizzo per `WIFI_LOST_TIMEOUT` → `ap` |
| `connecting` | prova la rete salvata (all'avvio, o tornando dalla rete del robot) | indirizzo → `station`; dopo `WIFI_CONNECT_TIMEOUT` (all'avvio) o `WIFI_AP_RETRY` (dalla rete del robot) → `ap` |
| `testing` | cambio di rete in corso (sezione 5) | esito → `station`, oppure ritorno alla rete di prima |
| `ap` | rete del robot accesa: `hostapd`, `dnsmasq`, server HTTP | dopo `WIFI_AP_DURATION` → `connecting`; cambio di rete chiesto dall'app → `testing` |

- **All'avvio**: `connecting` sulla rete salvata. Senza rete salvata (file senza rete): subito `ap`.
- **Rete persa durante il funzionamento** (router spento, robot portato lontano): dopo `WIFI_LOST_TIMEOUT` senza
  indirizzo, `ap`, come all'avvio.
- **Alternanza**: `ap` per 5 minuti, `connecting` per 1 minuto, a ripetizione, finché la rete salvata risponde. Serve
  soprattutto dopo un blackout: il router ci mette 1-3 minuti a ripartire, più del robot. Il chip non tiene le due
  cose insieme.
- **Rete del robot**: SSID `Maurice-WiFi`, WPA2-PSK solo CCMP, canale 6, indirizzo `192.168.5.1/24`, DHCP da `dnsmasq`
  per `192.168.5.10-50` senza DNS (`port=0`) e senza gateway, così il telefono non ci cerca internet. SSID e password
  stanno nella configurazione privata (sezione 8.2), mai nei file versionati.
- In `ap` l'SSH resta raggiungibile su `192.168.5.1` (dropbear ascolta su tutte le interfacce).
- Un indirizzo ottenuto dalla rete salvata basta per restare in `station`, anche senza internet: se internet manca è
  un problema del router, non della rete scelta.

## 5. Cambio di rete

Richiesta: SSID, tipo (`open` o `wpa`) e password (vuota per le reti aperte, 8-63 caratteri per WPA).

1. **Controlli** (rifiuti subito, con `reason`): `not_resting` (navigazione non a riposo, sezione 7.1), `wifi_busy`
   (cambio già in corso), `invalid_password`, `invalid_ssid` (vuoto o oltre 32 byte).
2. **Risposta subito "accettato"**; l'esito arriva dopo (sezione 7.4). Il cambio prende il numero successivo
   (`change_number`), così l'app riconosce l'esito del suo cambio.
3. Se era in `ap`: spegne `hostapd`, `dnsmasq` e il server HTTP, riaccende `wpa_supplicant`.
4. Aggiunge la rete nuova a `wpa_supplicant` **come seconda rete, senza salvare** (`ADD_NETWORK`, `SET_NETWORK`,
   `SELECT_NETWORK`): la rete di prima resta nella sua configurazione finché la nuova non funziona.
5. **Collegamento** entro `WIFI_ASSOC_TIMEOUT` (30 s). `wpa_supplicant` distingue:
   - password sbagliata (evento `CTRL-EVENT-SSID-TEMP-DISABLED` con `reason=WRONG_KEY`) → `wrong_password`;
   - rete mai vista nelle scansioni → `not_found`.
6. **Indirizzo** da `udhcpc` entro `WIFI_DHCP_TIMEOUT` (20 s), altrimenti `no_address`.
7. **Prova del backend**: `GET /api/device/ping` con il token del robot, in HTTPS con le stesse radici di `vacuum`,
   `WIFI_BACKEND_TRIES` tentativi (2) da `WIFI_BACKEND_TIMEOUT` (10 s). Risponde 200 → `ok`.
8. Se il backend non risponde: **ping a 8.8.8.8**, `WIFI_PING_TRIES` (3) da `WIFI_PING_TIMEOUT` (2 s), con un socket
   ICMP (`rtes-wifi` gira come root). Risponde → `backend_unreachable`, altrimenti `internet_unreachable`.

| Esito | Cosa fa il robot | Messaggio nell'app |
|---|---|---|
| `ok` | salva la rete nuova (toglie la vecchia, `SAVE_CONFIG`) e resta | "Maurice è collegato a «Casa»" |
| `backend_unreachable` | salva la rete nuova e resta (decisione dell'utente); i thread di rete di `vacuum` riprovano il server da soli come sempre, e appena lo raggiungono l'esito arriva all'app con lo stato | "Maurice è su «Casa» e ha internet, ma non raggiunge il server" (sezione 13, punto 1) |
| `internet_unreachable` | torna alla rete di prima (sezione 13, punto 2) | "«Casa» non porta a internet" |
| `no_address` | torna alla rete di prima | "Il router di «Casa» non ha dato un indirizzo a Maurice" |
| `wrong_password` | torna alla rete di prima | "Password sbagliata per «Casa»" |
| `not_found` | torna alla rete di prima | "Maurice non trova «Casa»" |

- **Ritorno alla rete di prima**: toglie la rete nuova, riseleziona la vecchia e passa a `connecting`: se entro
  `WIFI_CONNECT_TIMEOUT` non ha un indirizzo, accende la sua rete (`ap`). Partendo da `ap`, la rete di prima è quella
  che già non andava: in pratica il robot riaccende la sua rete e il telefono può riprovare.
- L'esito dell'ultimo cambio resta in memoria in `rtes-wifi` (numero, SSID, esito, ora) finché non arriva un altro
  cambio. Non va su disco: dopo un riavvio del robot non serve più.
- Il file `/etc/wifi/wpa_supplicant.conf` si scrive solo con `SAVE_CONFIG` di `wpa_supplicant` (scrittura completa del
  file, come oggi a mano). Prima della prima modifica `rtes-wifi` ne tiene una copia `.bak`.

## 6. Scansione

- In `station`/`connecting`: `SCAN`, attesa dell'evento `CTRL-EVENT-SCAN-RESULTS` (al massimo `WIFI_SCAN_TIMEOUT`,
  8 s), poi `SCAN_RESULTS`.
- In `ap`: il chip probabilmente non può cercare reti mentre fa da punto di accesso (**da verificare sul robot**).
  `rtes-wifi` fa una scansione subito prima di accendere la sua rete e risponde con quella, insieme all'ora della
  scansione. Con "Aggiorna" (`refresh=true`) spegne la sua rete per `WIFI_AP_SCAN_S` (5 s), cerca e la riaccende: il
  telefono perde il collegamento per qualche secondo e l'app ripete la richiesta finché il robot risponde.
- **Filtro**: solo reti con SSID visibile, aperte o WPA/WPA2 con password (`[WPA-PSK` o `[WPA2-PSK` nei flag, oppure
  nessun flag di sicurezza). Si scartano WEP, Enterprise (`EAP`) e reti solo WPA3 (`SAE` senza `PSK`). Stesso SSID più
  volte (più punti di accesso): una riga sola con il segnale più forte. Ordine: rete in uso, poi segnale.
- Per ogni rete: `ssid`, `signal_dbm`, `security` (`open`/`wpa`), `in_use`.
- Gli SSID arrivano da `wpa_supplicant` con i caratteri speciali scritti come `\xNN`: `rtes-wifi` li riporta ai byte
  veri, e in JSON viaggiano in UTF-8.

## 7. Interfacce

### 7.1 `vacuum` ↔ `rtes-wifi`

Socket Unix di tipo stream `/tmp/rtes-wifi.sock` (permessi 0600), una richiesta JSON per riga e una risposta per riga.
`vacuum` apre una connessione per richiesta.

| Richiesta | Risposta |
|---|---|
| `{"type":"scan","refresh":false}` | `{"ok":true,"scanned_at":...,"networks":[...]}` |
| `{"type":"connect","ssid":"...","security":"wpa","password":"..."}` | `{"ok":true,"change_number":5}` o `{"ok":false,"reason":...,"message":...}` |
| `{"type":"status"}` | lo stato del Wi-Fi (sezione 7.4) |
| `{"type":"resting","value":true}` | `{"ok":true}` |

**Riposo**: il controllo `not_resting` lo fa solo `rtes-wifi`, per le richieste di tutte e due le strade: `vacuum` gli
manda `resting` a ogni cambio e ogni 10 s; senza notizie da più di 30 s (`vacuum` fermo o custode) il robot è fermo e
conta come a riposo. `vacuum` passa i comandi del Wi-Fi così come sono.

### 7.2 Server HTTP locale (solo in `ap`)

Su `192.168.5.1:80`, solo sull'interfaccia della rete del robot, una connessione alla volta, niente autenticazione
(protegge la password WPA2).

| Richiesta | Come la richiesta del backend |
|---|---|
| `GET /api/wifi/networks[?refresh=1]` | `GET /api/robot/wifi/networks` |
| `POST /api/wifi/connect` | `POST /api/robot/wifi/connect` |
| `GET /api/wifi/status` | il campo `wifi` dello stato (sezione 7.4) |

Server minimo scritto a mano (richiesta fino a 4 KB, solo `Content-Length`, risposta e chiusura): non serve altro.

### 7.3 Backend

| Richiesta dell'app | `type` al robot | Note e rifiuti |
|---|---|---|
| `GET /api/robot/wifi/networks[?refresh=1]` | `wifi_scan` | attesa dell'esito `WIFI_SCAN_RESULT_WAIT_S` (10 s) invece di 3 s; l'esito porta i dati (sotto) |
| `POST /api/robot/wifi/connect` `{"ssid","security","password"}` | `wifi_connect` | `not_resting`, `wifi_busy`, `invalid_ssid`, `invalid_password`; 200 = accettato, l'esito arriva nello stato |

Richiesta nuova del robot: `GET /api/device/ping` (solo il controllo del token, risponde `{"ok":true}`), usata da
`rtes-wifi` per la prova del backend.

**Esito con i dati**: oggi l'esito di un comando è `{ok, reason, message}`; si aggiunge un campo facoltativo `data`
(per `wifi_scan`: `{"scanned_at", "networks"}`) che il backend gira all'app così com'è.

### 7.4 Stato del robot

Nuovo campo `wifi` nello stato JSON (`docs/app.md`, sezione 4.1), letto da `vacuum` con `status` ogni
`WIFI_STATUS_INTERVAL` (2 s) e mandato subito quando cambia:

```json
"wifi":{"mode":"station","ssid":"Casa","signal_dbm":-58,"ip":"192.168.1.40",
        "change":{"number":5,"ssid":"Casa","state":"done","result":"ok","time":1763390000}}
```

- `mode`: `station`, `connecting`, `testing`, `ap`.
- `change.state`: `none`, `testing`, `done`; `change.result`: gli esiti della sezione 5.
- Con `vacuum` senza risposta da `rtes-wifi` il campo manca.

## 8. Robot

### 8.1 File

| Percorso | Contenuto |
|---|---|
| `wifi/` (nuova cartella, come `driver/` e `spia/`) | `rtes-wifi`: `main.c` (modi e ciclo principale), `supplicant.c` (socket di controllo di `wpa_supplicant`), `children.c` (avvio e controllo dei processi figli con `posix_spawn`), `access_point.c` (`hostapd`, `dnsmasq`), `local_http.c`, `check.c` (prova del backend e ping), `scan.c` |
| `tasks/vacuum/cloud/app_commands.c` | `wifi_scan`, `wifi_connect` passati a `rtes-wifi` |
| `tasks/vacuum/cloud/network.c` | campo `data` nell'esito |
| `tasks/vacuum/cloud/robot_state.c` | campo `wifi` dello stato; `resting` a `rtes-wifi` |
| `tasks/vacuum/cloud/wifi_link.c/.h` (nuovo) | il socket verso `rtes-wifi` |
| `install/etc/init.d/rtes-wifi`, `install/usr/sbin/rtes-wifi-run` | un'istanza sola (`rtes-wifi-run`, sezione 8.3), uscita nel registro (`logread -e rtes-wifi`). Tenuti in `wifi/install/` fino alle prove in casa del 2026-11-17, poi spostati qui; `./rtes.sh install` copia anche `/usr/sbin/rtes-wifi` e riavvia il Wi-Fi solo se è cambiato |
| `install/usr/sbin/rtes-wifi-dhcp`, `-event`, `-check` | **tolti** (anche dal robot, con `./rtes.sh install`): il loro lavoro passa in `rtes-wifi` (udhcpc avviato solo a collegamento fatto, riavviato se dopo un nuovo collegamento non arriva l'indirizzo) |
| `install/etc/wifi/hostapd-rtes.conf`, `dnsmasq-rtes.conf` | generati da `rtes-wifi` in `/tmp` all'avvio, con SSID e password della configurazione: niente file versionati con la password |
| `Makefile`, `rtes.sh install` | compilazione per ARM con le stesse librerie di `vacuum` (libcurl, mbedTLS) e installazione |

### 8.2 Configurazione

In `tasks/vacuum/cloud/config.h` (privato, già usato da `vacuum` per indirizzo del server e token), una sezione nuova:
`WIFI_AP_SSID`, `WIFI_AP_PASSWORD`, indirizzi della rete del robot, percorso del socket e tutti i tempi delle sezioni
4-6. `config.example.h` con valori finti. `rtes-wifi` legge da lì indirizzo del server e file del token, così c'è un
solo posto da cambiare.

### 8.3 Sicurezza dell'SSH

`rtes-wifi` è ora l'unica strada per l'SSH. Lo lancia `/usr/sbin/rtes-wifi-run`, come `rtes-run` fa con `vacuum`:
- se `rtes-wifi` esce, lo rilancia dopo 5 s;
- **3 uscite in 10 minuti** → non lo rilancia più, chiude i processi del Wi-Fi e avvia il `wifiManager` di fabbrica,
  come fa oggi `rtes-wifi-check`;
- i processi figli (`wpa_supplicant`, `udhcpc`, `hostapd`, `dnsmasq`) si chiudono da soli se `rtes-wifi` muore
  (`PR_SET_PDEATHSIG`), e all'avvio `rtes-wifi` chiude quelli rimasti.

L'eseguibile sta in `/usr/sbin/rtes-wifi` e non in `/mnt/UDISK/rt/bin` come `vacuum`: il Wi-Fi parte presto (START=21)
e non deve dipendere dal momento in cui `/mnt/UDISK` è montato. È statico, circa 1,5 MB: **da controllare in casa** lo
spazio libero della partizione principale.

## 9. Backend

- `robot_api.py`: le due richieste della sezione 7.3; `wifi_connect` controlla anche lui SSID e password (400) prima
  di mandarle.
- `device_api.py`: `GET /api/device/ping`; l'esito con `data`.
- `robot_link.py`: attesa dell'esito per tipo di comando (`WIFI_SCAN_RESULT_WAIT_S` in `config.py`).
- **Password**: nel registro dei comandi (`command_log.params`) si scrive solo `{"ssid","security"}`; nella coda in
  memoria resta solo fino all'esito; mai nei log di gunicorn (la password viaggia nel corpo, non nell'indirizzo). Lo
  stato del robot non la contiene, quindi il registro per le analisi non la vede.
- `dev/fake_robot.py`: risponde a `wifi_scan` con reti finte e simula gli esiti di `wifi_connect` (sezione 12).
- Prove automatiche per le due richieste e per la password tolta dal registro.

## 10. App

### 10.1 Dove

- **Avanzate**: pulsante **"Rete Wi-Fi"**, solo nell'APK (`kIsWeb` → non c'è).
- Schermata nuova `screens/wifi/`, nello stile Vetro delle altre (non c'è in `App_Mockup.png`).

### 10.2 Con chi parla

All'apertura della schermata:
1. robot **Online** → backend (`/api/robot/wifi/...`);
2. robot **Offline** → `GET http://192.168.5.1/api/wifi/status` con attesa breve (2 s): se risponde, **modo locale**
   (`http://192.168.5.1/api/wifi/...`);
3. nessuno dei due → schermata con la spiegazione: "Maurice è offline. Collega il telefono alla rete «Maurice-WiFi»
   (password nelle impostazioni del robot) e torna qui", e un pulsante "Riprova".

Su Android due cose servono per il modo locale:
- **HTTP in chiaro** permesso solo verso `192.168.5.1` (`network_security_config.xml`): per tutto il resto resta solo
  HTTPS;
- **traffico sul Wi-Fi**: con una rete senza internet Android manda le richieste sui dati mobili. Un piccolo canale
  nativo in Kotlin (`ConnectivityManager.bindProcessToNetwork` sulla rete Wi-Fi) attivo solo nel modo locale e
  tolto all'uscita dalla schermata. Niente libreria esterna.

### 10.3 Schermata

- In alto: la rete in uso con il segnale (o "Rete del robot accesa" nel modo locale), e il modo (`Online` via
  backend / collegato a Maurice).
- **Lista delle reti**: nome, tacche del segnale (da `signal_dbm`: ≥ -55, ≥ -67, ≥ -78, sotto), lucchetto se
  `security = wpa`, segno sulla rete in uso; ora della scansione se viene dalla rete del robot. Pulsante **Aggiorna**.
  Sotto la lista una nota: "Maurice vede solo le reti a 2,4 GHz".
- **Altra rete…**: nome scritto a mano e tipo (con password / aperta).
- Tocco su una rete: finestra con la password (nascosta, con l'occhio per mostrarla), oppure solo conferma per le reti
  aperte; con robot non a riposo il pulsante è spento e dice perché.
- Dopo l'invio: schermata di attesa con le fasi ("Maurice si sta collegando a «Casa»…").
  - **Dal backend**: il robot va Offline per qualche secondo; l'app guarda `wifi.change` nello stato e mostra l'esito
    con il messaggio della sezione 5 quando il robot torna Online. Se non torna entro `WIFI_CHANGE_WAIT` (3 minuti):
    "Maurice non è tornato online" (sezione 13, punto 1).
  - **Dalla rete del robot**: il telefono perde la rete del robot. Se il cambio riesce, il telefono torna da solo
    sulla sua rete di casa e l'app mostra l'esito appena vede il robot Online dal backend. Se fallisce, il robot
    riaccende la sua rete: l'app riprova `192.168.5.1` e mostra l'esito da `/api/wifi/status`.

## 11. Effetti sull'analisi real-time

- Task real-time: **nessuna modifica**.
- `vacuum`: il thread dei comandi (core 3, nice 5) aspetta `rtes-wifi` per la scansione (fino a 10 s) senza toccare i
  task; il thread dello stato legge `status` ogni 2 s con un socket locale.
- `rtes-wifi`: processo normale (SCHED_OTHER) sul core 3 con nice come i thread di rete. Lavora quasi solo in attesa
  (socket, eventi, timer). Si misura con gli altri nei WCET (carico di sistema).

## 12. Prove

**Fuori casa (senza robot):**
- `rtes-wifi --simulate` sul PC: `wpa_supplicant`, `hostapd` e il ping finti (reti finte, esiti scelti da una
  variabile d'ambiente), server HTTP locale su `localhost`. Prova dei modi, dell'alternanza (tempi accorciati) e
  delle risposte.
- Backend: prove automatiche; `fake_robot.py` con `wifi_scan` e `wifi_connect`.
- App: emulatore Android verso il backend di sviluppo, e verso `rtes-wifi --simulate` per il modo locale.

**In casa (con il robot, dopo averne parlato):**
1. avvio con la rete salvata;
2. scansione da `station` e da `ap` (il chip cerca reti con il punto di accesso acceso?);
3. cambio con password giusta, password sbagliata, rete che non esiste, rete senza internet (hotspot del telefono con
   i dati spenti);
4. router spento: rete del robot dopo 30 secondi, alternanza, ritorno da solo alla riaccensione;
5. cambio dalla rete del robot con il telefono;
6. SSH su `192.168.5.1` con la rete del robot accesa;
7. `rtes-wifi` che si chiude 3 volte: parte il `wifiManager` di fabbrica.

## 13. Decisioni del 2026-11-17 sui punti aperti

1. **Backend irraggiungibile**: se il robot non torna Online entro `WIFI_CHANGE_WAIT` (3 minuti) l'app scrive "Maurice
   non è tornato online: può essere il server irraggiungibile dalla rete «Casa»"; quando il backend torna raggiungibile
   il robot manda con lo stato l'esito vero (`backend_unreachable`, con l'ora) e l'app lo mostra. Niente ricerca del
   robot sulla rete locale.
2. **Internet irraggiungibile**: il robot torna alla rete di prima (tabella della sezione 5).
3. **Password della rete del robot nell'app**: nella configurazione privata dell'app (`config.json`, non versionato),
   mostrata nella schermata che chiede di collegarsi a `Maurice-WiFi`.

## 14. Ordine di lavoro

1. `rtes-wifi` con `--simulate` e le prove sul PC.
2. `vacuum` (`cloud/`) e backend, con `fake_robot.py`.
3. App.
4. Revisione dell'utente.
5. Prove sul robot in casa (sezione 12), poi `./rtes.sh install`.
6. Aggiornamento di `docs/app.md` (sezioni 4.1, 5.3, 8.5, 8.6, 10.4) e di `docs/PIANO.md` (fase 6).

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
- attesa prima della rete del robot abbassata da 2 minuti a 30 s (`WIFI_CONNECT_TIMEOUT_S`, `WIFI_LOST_TIMEOUT_S`),
  richiesta dell'utente;
- installazione: riavvio del robot (rtes-wifi collegato 5 s dopo l'avvio) e ripiego (3 uscite di `rtes-wifi` →
  `wifiManager` di fabbrica sulla rete di casa) riusciti.
