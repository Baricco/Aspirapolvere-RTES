# App di Maurice: progettazione

Progettazione completa dell'app, del backend e della parte di rete del robot, decisa con l'utente fra il 2026-11-09 e il
2026-11-10, un pezzo alla volta. Niente codice prima che questo documento sia approvato. Il codice lo scrive Claude
(backend, app, thread di rete e modifiche ai task che servono all'app) e l'utente lo rivede.

In tutto il documento `<server>` sta al posto del dominio vero, che non va scritto in nessun file versionato.

## Indice

1. [Regole generali](#1-regole-generali)
2. [Architettura](#2-architettura)
3. [Trasporto e sicurezza](#3-trasporto-e-sicurezza)
4. [Stato del robot](#4-stato-del-robot)
5. [Comandi](#5-comandi)
6. [Telecomando](#6-telecomando)
7. [Mappe](#7-mappe)
8. [Backend](#8-backend)
9. [Robot](#9-robot)
10. [App](#10-app)
11. [Sviluppo e prove](#11-sviluppo-e-prove)
12. [Deploy (rimandato)](#12-deploy-rimandato)
13. [Ordine di lavoro proposto](#13-ordine-di-lavoro-proposto)

---

## 1. Regole generali

- **Il real-time viene prima di tutto.** Il codice nuovo sul robot non deve mai far aspettare un task: niente mutex con
  i task, si legge solo con le funzioni `read_*` (doppi buffer, chi scrive non aspetta mai) e si scrive solo con le
  funzioni di richiesta. Gli effetti su cache e bus si misurano con l'app collegata, insieme alle altre misure.
- **Nomi nuovi in inglese**, nello stile dei task: `snake_case`, tipi con `_t`, `#define` in maiuscolo, commenti in
  italiano. Vale per campi JSON, risorse HTTP, codice C, Python e Dart, colonne del database.
- **Ogni numero regolabile in una costante** (`#define` in C, costanti in Python e Dart), mai scritto nel codice.
- **Configurazione privata mai versionata**: dominio, percorsi, token e chiavi stanno in file ignorati da git. Si
  versionano solo gli esempi con valori finti (`config.example.*`).
- **Testi dell'interfaccia in italiano**, tranne quelli indicati esplicitamente in inglese ("Online"/"Offline" e la
  riga di stato).

## 2. Architettura

```
 ┌──────────────┐  HTTPS   ┌──────────────────────────┐  HTTPS   ┌─────────────────────────────┐
 │ App Flutter  │ ───────▶ │ Backend Flask (gunicorn) │ ◀─────── │ Robot: programma vacuum     │
 │ Android, web │ ◀─────── │ SQLite + file delle mappe│ ───────▶ │ thread di rete (core 3)     │
 │ (iOS dopo)   │          │ stato vivo in memoria    │          │ task real-time (core 1-2)   │
 └──────────────┘          └──────────────────────────┘          └─────────────────────────────┘
```

- **Il robot non riceve mai connessioni**: è lui a collegarsi al server, che fa da tramite. L'app funziona anche fuori
  casa.
- Il server sta a casa dell'utente, su un'altra rete rispetto al robot, raggiungibile dal suo indirizzo pubblico.
- L'app parla solo con il server.

## 3. Trasporto e sicurezza

- **HTTPS su tutti e due i tratti**, app↔server e robot↔server. Il certificato del server è di Let's Encrypt (Certbot).
- **Il robot verifica il certificato del server** con le radici scritte nel programma (Root YE, ISRG Root X2, ISRG Root
  X1), come faceva il firmware originale. Il robot non ha un certificato suo: si presenta con il suo token.
- **libcurl** con solo HTTP e HTTPS compilati, e `CURLOPT_PROTOCOLS_STR "https"` che permette solo HTTPS; TLS con
  **mbedTLS** (ECDSA P-256 e P-384, SHA-384, RSA). Tutte e due statiche, compilate con il compilatore del robot.
- **Connessioni tenute aperte** (keep-alive): niente handshake TLS a ogni richiesta.
- **Orologio:** l'ora all'avvio la dà `rtc0`, un orologio esterno `pcf8563` (`hctosys=1`). Dopo 8 giorni acceso il
  robot era avanti di circa 28 s, che per i certificati non conta. **Da fare:** spegnere il robot per qualche minuto e
  controllare che l'ora resti giusta. Se non resta, mbedTLS si compila senza il controllo delle date
  (`MBEDTLS_HAVE_TIME_DATE` tolto) e tutto il resto della verifica resta. Il fuso del robot (`CST-8`) non conta: si usa
  sempre l'ora in secondi dal 1970.
- **Token del robot:** 32 byte casuali, in un file sul robot (`/mnt/UDISK/rt/robot-token`, permessi 0600), mai nel
  progetto. Nel database del server se ne salva solo lo SHA-256. Si genera con lo script di amministrazione.
- **Sessioni dell'app:** JWT (sezione 8.4). Password con `hashlib.scrypt` e sale casuale.

## 4. Stato del robot

### 4.1 Contenuto

JSON di circa 1,6 KB (circa 750 byte con gzip). Contiene solo dati che i task e il driver pubblicano già, più il nuovo
`navigation_status_t` (sezione 9.5).

```json
{"format":1,"robot_time":1762700000,"uptime_s":1834,"remote_control":false,
 "navigation":{"state":"follow_path","mission":"clean","interrupted_mission":"none","mission_number":3,"mission_duration_s":312},
 "objective":{"type":"move","x":1.50,"y":-0.30,"theta_deg":0.0},
 "position":{"x":1.23,"y":-0.40,"theta_deg":12.3,"v":0.21,"omega":0.05,"valid":true},
 "map":{"state":"normal","save_failed":false,"started_on_base":true,"file_crc":"9c41e2a0",
        "forbidden_zones":[{"x":-0.365,"y":-0.22,"w":0.26,"h":0.44}]},
 "safety":{"errors":["obstacle"],"max_v_forward":0.30,"max_v_backward":0.50},
 "lidar":{"valid":true,"sector_distances_mm":[0,0,0,0,0,0,0,0,0,0,0,0],"corridor_front_mm":640,"corridor_back_mm":1200},
 "battery":{"voltage":15.1,"percent":78,"level":"ok","charging":false,"charge_complete":false,"temperature_c":31},
 "motors":{"side_brush":60,"main_brush":60,"suction_rpm":12000},
 "driver":{"phase":"ready","fault":null,"limits":[],"on_base":false,"lifted":false,
           "docking":{"result":"idle","phase":"none","attempt":0},"lidar":{"ok":true,"rotation_hz":6.0}},
 "sensors":{"bumper_left":false,"bumper_right":false,"cliff":[false,false,false,false],"dustbin":true,
            "currents_ma":{"wheel_left":180,"wheel_right":175,"side_brush":90,"main_brush":220,"suction":600}},
 "tasks":[{"name":"movement","deadline_misses":0,"missed_activations":0}]}
```

- `fault`, quando c'è un guasto: `{"code": ..., "text": ...}`.
- `map.file_crc`: CRC-32 del file `map.rtm` sul robot (assente se non c'è). Il server lo usa per sapere se ha la copia
  giusta della mappa (sezione 7).
- `map.forbidden_zones`: la zona 0 è quella della base. `GET /api/robot/zones` la toglie.
- **Lasciati fuori di proposito:** il giro intero del lidar, i campi del GD32 non verificati, il magnetico, i tempi
  massimi di `rt_stats_t` (si leggono solo a task fermo), odometria, statistiche del collegamento, contatti, IMU,
  encoder.

### 4.2 Quando lo manda il robot

Dal thread dello stato (sezione 9.3), con un ciclo ogni 100 ms:
- **ogni 0,1 s** quando il robot si muove (stato della navigazione attivo, oppure v o ω diversi da 0);
- **subito** quando cambia qualcosa di importante: stato, missione, guasto, fase del driver, aggancio, telecomando,
  carica, base, esito di un comando, zone;
- **ogni 10 s** a riposo.

Se il server non risponde lo stato si butta: nessuna coda, conta solo il più nuovo.

### 4.3 Cosa aggiunge il server

| Campo | Significato |
|---|---|
| `online` | ultimo contatto da meno di 35 s (la richiesta lunga dei comandi ne fa uno ogni 25 s al massimo) |
| `last_contact` | ora dell'ultimo contatto, dall'orologio del server |
| `remote_control_user` | chi tiene il telecomando, o `null` |
| `map_version` | versione della mappa dal vivo sul server |
| `state_number` | numero progressivo dello stato |

### 4.4 Come lo riceve l'app

`GET /api/robot/state?after=N&min_interval_ms=M`:
- senza `after` risponde **subito** con l'ultimo stato che ha, anche vecchio, così all'apertura l'app mostra subito
  ultima posizione, guasto e `online`;
- con `after=N` risponde appena c'è uno stato con numero maggiore di N, ma non prima di M ms dall'ultima risposta a
  quell'app;
- se in 25 s non arriva niente risponde **304**, e l'app rimanda subito la stessa richiesta: è sempre in ascolto;
- un cambio di `online` conta come stato nuovo e sveglia le app in attesa;
- risposte JSON in gzip.

`min_interval_ms`: 100 nella schermata principale e nel telecomando, 1000 nelle altre, nessuna richiesta con l'app in
background. L'ultimo stato il server lo scrive anche su disco ogni 10 s, così lo ha anche dopo un riavvio.

## 5. Comandi

### 5.1 Come viaggia un comando

1. L'app manda la richiesta al server.
2. Il server fa i controlli suoi (autenticazione, telecomando, parametri), mette il comando in coda e **tiene aperta la
   richiesta dell'app** finché arriva l'esito, al massimo 3 s.
3. Il robot riceve i comandi in coda con la sua richiesta lunga `GET /api/device/commands`, li esegue uno alla volta e
   manda ogni esito con `POST /api/device/commands/{id}/result`.
4. Il server gira l'esito all'app.

Messaggio al robot: `{"id":57,"type":"start"}`, con i parametri accanto quando servono. Esito:
`{"id":57,"ok":false,"reason":"busy","message":"..."}`.

### 5.2 Risposte all'app

| Codice | Quando |
|---|---|
| 200 | eseguito |
| 400 | parametri sbagliati |
| 401 | non autenticato o sessione scaduta |
| 404 | risorsa che non esiste |
| 409 `{"reason","message"}` | rifiutato (dal server o dal robot) |
| 429 | login bloccato dopo troppi tentativi |
| 503 `robot_unreachable` | il robot non ha preso il comando entro 3 s: il comando si butta e **non è mai stato eseguito** |
| 504 `no_result` | il robot l'ha preso ma l'esito non è arrivato entro 3 s: **non si sa** se è stato eseguito, l'app dice di guardare lo stato |

Tutti gli errori hanno la forma `{"reason": ..., "message": ...}`. Ogni comando finisce nel registro dei comandi.

### 5.3 Tabella dei comandi

| Richiesta dell'app | `type` al robot | Note e rifiuti |
|---|---|---|
| `POST /api/robot/mission/start` | `start` | azzera anche il guasto; `busy`, `cause_still_present`, `position_lost` |
| `POST /api/robot/mission/pause` | `pause` | `not_running` |
| `POST /api/robot/mission/resume` | `resume` | azzera anche il guasto; `not_paused`, `cause_still_present`, `position_lost` |
| `POST /api/robot/mission/home` | `home` | azzera anche il guasto; con la posizione persa fa l'aggancio alla cieca; se sta già tornando: 200; `already_home`, `cause_still_present` |
| `POST /api/robot/fault/reset` | `fault_reset` | `not_in_fault`, `cause_still_present` |
| `POST /api/robot/remote/take` | `remote_take` | `remote_in_use` (con l'utente che lo tiene) |
| `POST /api/robot/remote/move` `{"v","omega"}` | `remote_move` | `remote_in_use`, `position_lost` |
| `POST /api/robot/remote/cleaning` `{"brushes","suction"}` | `remote_cleaning` | `remote_in_use`, `dustbin_missing` |
| `POST /api/robot/remote/keepalive` | `remote_keepalive` | `remote_in_use` |
| `POST /api/robot/remote/release` | `remote_release` | `not_remote_owner` |
| `PUT /api/robot/zones` `[{"x","y","w","h"}]` | `zones` | lista intera; `not_started_on_base`, `position_lost`, `too_many_zones` (più di 15), `outside_map` |
| `PUT /api/robot/maps/selected` `{"id"}` | `map_select` | sezione 7.5; `not_resting`, `download_failed` |
| `POST /api/robot/map/reset` | `map_reset` | sezione 7.6; `not_resting`, `not_backed_up` |

Con il robot non raggiungibile ogni comando risponde 503.

### 5.4 Pulsanti e tasti

- L'app mostra le missioni con **due pulsanti**: il principale (Avvia, Pausa o Riprendi secondo lo stato) e Casa.
  `fault/reset` sta solo nel menu Avanzate e nella schermata del telecomando.
- **Guasto azzerato a metà pulizia:** il robot va in **pausa** (non più in riposo), così con Riprendi continua tenendo
  le celle già pulite. Senza missione in corso va in riposo. Il tasto fisico di accensione si comporta allo stesso modo.
- **Posizione persa:** solo `home` fa qualcosa (aggancio alla cieca); `start`, `pause` e `resume` rispondono
  `position_lost`.

## 6. Telecomando

- **Un utente alla volta.** Il blocco lo tiene il server. Gli altri vedono chi lo ha. I loro comandi del telecomando
  e delle missioni ricevono `remote_in_use`; le zone restano permesse.
- **Prenderlo:** `remote_take`, oppure qualsiasi comando del telecomando (`remote_move`, `remote_cleaning`,
  `remote_keepalive`) mandato quando è libero: lo prende in automatico.
- **Effetto sul robot:** mette in pausa la missione in corso e annulla l'aggancio. Sulla base `drv_esci_dalla_base(1)`
  permette di uscire piano in avanti.
- **Rilascio:** `remote_release`, oppure dopo **30 s** senza comandi del telecomando (timer sul server e sul robot,
  keepalive compresi). Dopo il rilascio il robot va in **pausa** se c'era una missione, altrimenti in **riposo**.
- **Start, pause, resume o home di chi lo tiene** valgono come rilascio seguito dal comando. Il robot torna in
  telecomando al prossimo comando del telecomando.
- **`remote_move {v, omega}`** vale 0,5 s. La navigazione limita v e ω prima ai massimi del telecomando, poi a
  `max_v_forw` e `max_v_backw` della sicurezza.
- **Tutti i task restano attivi.** Vuoto e paraurti li blocca il driver, che ferma la direzione pericolosa e lascia
  indietro e rotazione; gli ostacoli del lidar li limita la sicurezza. **Le zone proibite, base compresa, sono
  ignorate** in telecomando.
- **Guasto durante il telecomando:** il telecomando **non** finisce. La schermata ha il pulsante Azzera guasto; dopo
  l'azzeramento il robot torna in telecomando. Si può prendere il telecomando anche con un guasto in corso: il robot
  resta fermo finché il guasto non si azzera.
- **Posizione persa:** i movimenti sono rifiutati con `position_lost` (muovere il robot rovinerebbe la ricerca della
  posizione); `home` resta possibile.
- **Batteria scarica:** il robot **non** torna da solo alla base; l'app mostra un avviso.
- **Spazzole e aspirazione:** comandate dall'app; lo sblocco della spazzola centrale funziona come sempre.

Costanti nella navigazione, con gli stessi valori di crociera dei task ma definite a parte:

| Costante | Valore |
|---|---|
| `REMOTE_MAX_V` | 0,35 m/s (come `CRUISING_SPEED`) |
| `REMOTE_MAX_V_BACK` | 0,35 m/s |
| `REMOTE_MAX_OMEGA` | 1,2 rad/s (come `MAX_ROTATION_SPEED`, sotto 1,5 oltre cui la mappatura scarta la scansione) |
| `REMOTE_MOVE_TIMEOUT` | 0,5 s |
| `REMOTE_IDLE_TIMEOUT` | 30 s |

## 7. Mappe

### 7.1 Dove stanno

- **Sul robot una sola mappa**, `map.rtm` nella cartella delle mappe: quella selezionata. Spariscono le mappe numerate
  e il file `attiva`. La memoria del robot non si può riempire di mappe.
- **Sul server tutte le mappe**, compresa una copia di quella selezionata, nei file `map-<id>.rtm` (id mai riusato).
  Nel database ci sono nome, CRC e date.
- **Nomi:** scelti dall'utente, diversi fra loro (`409 name_taken`), mostrati sempre senza estensione. Il default è
  `map-N` con N = numero di mappe + 1, salendo al primo numero libero se quel nome c'è già.
- Il formato è sempre RTM1 (`docs/formato-mappa.md`, la sezione "Dove sta" va aggiornata).
- **La mappa non cambia mai con il programma avviato**: si carica solo in `init_mapping`, prima dei task. Per cambiarla
  il programma si riavvia (sezione 7.5). Così il WCET della mappatura non cambia.

### 7.2 Allineamento fra robot e server

- Il server ricorda quale delle sue mappe è selezionata sul robot (`robot.selected_map_id`, vuoto = nessuna mappa).
- **Dopo ogni salvataggio** sul robot (fine esplorazione, aggancio, zone cambiate) il thread dello stato manda il file
  con `PUT /api/device/map/saved`, e il server aggiorna la mappa selezionata.
- **Robot senza mappa** (prima volta, "Nessuna mappa", mappa cancellata): al primo salvataggio il server crea una mappa
  nuova `map-N` e la segna come selezionata. Fino ad allora l'app mostra "mappa nuova, non ancora salvata".
- **All'avvio** il robot manda la sua mappa salvata. Se il CRC non coincide con la selezionata (file cambiato a mano,
  primo avvio dopo la modifica) il server la salva come **mappa nuova** invece di sovrascrivere: nessuna mappa si perde.

### 7.3 Mappa dal vivo

L'app mostra sempre la mappa selezionata (vuota se non ce n'è nessuna); mentre il robot si muove la aggiorna come il
visore, e al rientro mostra l'ultima mappa con la scia. Si mostra la **mappa fine a 5 cm**, la stessa dei file salvati;
la griglia di navigazione da 20 cm non viene mai mandata.

**Sul robot** (thread dello stato, core 3, priorità normale; nessun doppio buffer nuovo, nessun mutex):
1. **All'avvio** manda `map.rtm` come mappa dal vivo intera (`PUT /api/device/map`), oppure una mappa vuota se non ha
   mappa o non è partito dalla base. La sua copia privata la ottiene decodificando lo stesso file.
2. **Ogni 0,1 s**, se la posizione è valida, allarga il rettangolo delle posizioni non ancora mandate.
3. **Ogni 0,5 s**, se il rettangolo non è vuoto:
   - legge solo quel rettangolo allargato di `LIDAR_MAPPING_MAX_RANGE` (6 m) più qualche cella di margine, con
     `copy_map_categories` a rettangolo (circa 60 KB, contro 1 MB della mappa intera; chi legge rifà la copia se un job
     della mappatura ci è passato sopra, la mappatura non aspetta mai);
   - lo confronta con la copia privata, cioè l'ultima confermata dal server;
   - manda il **rettangolo delle celle cambiate in RTM1** con la versione di partenza e la nuova
     (`POST /api/device/map/patch`).
4. **Risposte:**
   - conferma: aggiorna la copia privata e svuota il rettangolo delle posizioni;
   - invio fallito: il rettangolo resta e si allarga, le celle perse partono all'invio dopo;
   - `409` (il server non ha la versione di partenza, per esempio dopo un riavvio): il robot rimanda la sua copia
     privata intera, dalla sua memoria, e poi le differenze in sospeso. **A programma avviato non si legge mai tutta
     la mappa.**

**Perché basta il rettangolo:** la mappatura scrive la mappa fine solo intorno al robot (raggi del lidar fino a
`LIDAR_MAPPING_MAX_RANGE`, celle sotto il robot, eventi del vuoto e del paraurti). Da sollevato, con la posizione persa
o non ancora confermata non scrive niente. L'unica eccezione è `reset_session_data` a inizio pulizia, che riporta a
"sconosciute" le celle degli ostacoli bassi. Ma nella mappa a categorie un ostacolo basso è già "libero"
(`fine_cell_category`): l'app le mostra libere finché il lidar non le rivede (e allora tornano libere davvero) o finché
il robot passa vicino (e il confronto le manda). Al rientro arriva comunque il file salvato, identico alla mappa del
robot.

**Sul server:** applica i rettangoli alla sua mappa dal vivo e tiene le ultime differenze numerate. `map_version`
nello stato dice all'app quando scaricare.

**Nell'app:** all'apertura `GET /api/robot/map` (intera), poi `GET /api/robot/map?after=N` (solo le differenze, o di
nuovo l'intera se è troppo indietro) quando `map_version` cambia. Incolla i rettangoli sulla sua copia e ridisegna.

### 7.4 Scia

- La registra il **server** dalle posizioni che arrivano con lo stato. Così la vede anche chi apre l'app a missione
  finita.
- Solo quella dell'**ultima missione**; ricomincia quando cambia `mission_number`. Scritta su file a fine missione.
- Nell'app è **grigia**, larga come la spazzola, così si vede dove il robot è passato.
- `GET /api/robot/trail[?after=N]`.

### 7.5 Cambio di mappa

Si può fare solo con il robot **a riposo sulla base** (altrimenti `not_resting`).
1. Il server manda `map_select` con l'id e il CRC della mappa.
2. Il thread dei comandi la scarica (`GET /api/device/maps/{id}`) in `map.rtm.download` e controlla il CRC.
3. Chiede `request_map_replace()` al task aperiodico, che **rinomina** il file sopra `map.rtm` (la rinomina sostituisce
   in un colpo solo: dopo un'interruzione di corrente c'è o la vecchia mappa intera o la nuova intera) e da quel momento
   non salva più. La rinomina la fa il task aperiodico perché è lui che salva: un salvataggio non può incrociarsi con
   la sostituzione.
4. Il thread manda l'esito e chiede l'uscita: il programma ferma i task ed esce con `RESTART_EXIT_CODE` (3).
5. Chi ha avviato il programma lo rilancia (ciclo finché il codice di uscita è 3): `tools/driver-robot.sh`,
   `rtes.sh sim`, `tools/prove-nav/prova.py`, poi l'avvio automatico della fase 6.

In caso di errore prima della rinomina il robot resta con la mappa vecchia. Durante il riavvio l'app mostra "riavvio
in corso".

### 7.6 "Nessuna mappa" e cancellazione

- **"Nessuna mappa"** nel menu delle mappe chiama `POST /api/robot/map/reset`: il robot cancella `map.rtm` (con
  `request_map_delete()`) e si riavvia; alla prossima missione esplora. Il server manda il comando solo se la sua
  copia ha lo stesso CRC del file sul robot (`map.file_crc`), altrimenti `not_backed_up`: la copia sul server è
  sempre quella che c'era sul robot.
- **Cancellare una mappa** dal server si fa **solo dall'app**, nella gestione delle mappe, con `DELETE
  /api/robot/maps/{id}`, che agisce solo sul server.
- **Cancellare la mappa selezionata:** l'app chiama prima `POST /api/robot/map/reset` e, solo se va a buon fine,
  `DELETE /api/robot/maps/{id}`. Se il robot non è a riposo sulla base o non è raggiungibile non si cancella niente.
  Le API restano semplici e si combinano nell'app.
- **Ogni cancellazione chiede conferma con un avviso**; per la mappa selezionata l'avviso dice che il robot resterà
  senza mappa.

### 7.7 Zone proibite

- Per la **mappa selezionata**: comando `zones` al robot, che le applica, salva la mappa e la rimanda al server.
- Per le **altre mappe**: il server riscrive l'intestazione del file (zone e CRC).
- `PUT /api/robot/maps/{id}/zones` sceglie da solo fra i due casi.

### 7.8 Mappe di oggi

Passaggio fatto una volta a mano: la mappa attiva diventa `map.rtm`, le altre si caricano sul server con lo script di
amministrazione (servono per provare l'app).

## 8. Backend

### 8.1 Base tecnica

- **Python 3.13**, **Flask**, **gunicorn con `--workers 1 --threads 32`**. Il processo deve essere **uno solo**,
  perché lo stato vivo sta in memoria; le attese si fanno con `threading.Condition`. Le richieste aperte insieme sono
  poche (una richiesta lunga del robot, una per ogni app aperta, qualche comando in attesa): i thread bastano.
- **SQLite** in modalità WAL, un file solo.
- Risposte JSON in **gzip** fatto da Flask (vale anche in sviluppo, senza Apache). Le mappe no: sono già compresse.
- Cartella **`backend/`** nel progetto.

### 8.2 Struttura

| Percorso | Contenuto |
|---|---|
| `app.py` | crea l'app Flask, avvia la programmazione delle pulizie |
| `auth.py` | login, refresh, logout, controllo del JWT e della sessione, cookie per il web |
| `robot_api.py` | richieste dell'app (`/api/robot/...`) |
| `device_api.py` | richieste del robot (`/api/device/...`) |
| `robot_link.py` | stato vivo in memoria: coda dei comandi, attese, telecomando, ultimo stato |
| `maps.py` | lettura e scrittura RTM1, mappa dal vivo, differenze, scia |
| `scheduler.py` | pulizie programmate |
| `db.py`, `schema.sql` | accesso a SQLite e tabelle |
| `config.py` | costanti non private: tempi, limiti |
| `admin.py` | script di amministrazione con menu (TUI) |
| `requirements.txt` | Flask, gunicorn, PyJWT, questionary |
| `tests/` | prove automatiche |
| `dev/` | `fake_robot.py` e i comandi per il certificato autofirmato |
| `deploy/` | `gunicorn.conf.py`, unità systemd e VirtualHost di esempio (con `<server>`) |

**Valori privati** (cartella dei dati, origini permesse per il web in sviluppo) in variabili d'ambiente, dall'unità
systemd in produzione e da un file `.env` ignorato da git in sviluppo. **Cartella dei dati**, fuori dal progetto:
database, mappe, chiave del JWT, ultimo stato, scia.

### 8.3 Database

| Tabella | Colonne |
|---|---|
| `users` | `id`, `username` (unico), `password_hash`, `password_salt`, `created_at`, `failed_logins`, `locked_until` |
| `sessions` | `id`, `user_id`, `created_at`, `last_used_at`, `expires_at` |
| `robot` | una riga: `token_hash`, `selected_map_id` |
| `maps` | `id`, `name` (unico), `crc32`, `size_bytes`, `created_at`, `saved_at` |
| `schedules` | `id`, `days_mask` (7 bit, lunedì-domenica), `time_minutes` (0-1439), `enabled`, `last_run_at`, `last_result` |
| `command_log` | `id`, `time`, `user_id` (vuoto = programmazione), `type`, `params`, `status`, `reason`; si tengono 30 giorni |

Con il JWT la sessione si riconosce dal suo `jti`, che è l'`id` della riga di `sessions`: non serve salvare il token.

**In memoria, non nel database:** ultimo stato, coda dei comandi, telecomando, mappa dal vivo con le ultime differenze,
scia (scritta su file a fine missione), ultimo stato (scritto su disco ogni 10 s).

### 8.4 Accesso

- **Utenti creati solo dallo script** di amministrazione; l'app fa solo il login. Tutti gli utenti possono fare tutto.
  La password si cambia solo dallo script.
- **Password:** `hashlib.scrypt` con sale casuale di 16 byte per utente, confronto a tempo costante. Dopo **5
  tentativi** sbagliati di fila lo stesso utente è bloccato per **1 minuto** (429).
- **JWT:** firma HS256 (PyJWT); `sub` = utente, `jti` = sessione, `iat`, `exp` = 30 giorni. Chiave di firma: 32 byte
  casuali in un file della cartella dei dati.
- **A ogni richiesta** il server controlla firma e scadenza, poi che la sessione `jti` esista ancora: così logout,
  cambio password e utente tolto chiudono le sessioni subito.
- **Rinnovo:** a ogni apertura l'app chiama `POST /api/auth/refresh` e riceve un JWT nuovo per la stessa sessione: chi
  usa l'app non deve mai rifare il login.
- **Telefono:** il JWT sta nella memoria sicura del sistema e va nell'intestazione `Authorization: Bearer`.
- **Web:** il JWT sta in un **cookie `HttpOnly; Secure; SameSite=Strict; Path=/api`**, che il JavaScript della pagina
  non può leggere. Login e refresh lo impostano, il logout lo cancella. Contro le richieste mandate da altri siti, il
  server rifiuta POST, PUT, PATCH e DELETE con un'intestazione `Origin` diversa da quella del sito.
- **Robot:** token casuale nell'intestazione `Authorization: Bearer`, confrontato con lo SHA-256 nel database.

### 8.5 Richieste dell'app

Tutte sotto `/api`. La versione web dell'app è servita dallo stesso indirizzo (pagina su `/`), quindi in produzione
non serve CORS; in sviluppo il backend accetta le origini scritte nella configurazione privata.

| Richiesta | Cosa fa |
|---|---|
| `POST /api/auth/login` `{username, password}` | `{token, expires_at}` (sul web solo il cookie) |
| `POST /api/auth/refresh` | JWT nuovo per la stessa sessione |
| `POST /api/auth/logout` | chiude la sessione |
| `GET /api/auth/me` | `{username}` |
| `GET /api/robot/state` | sezione 4.4 |
| comandi | sezione 5.3 |
| `GET /api/robot/zones` | zone dall'ultimo stato, senza quella della base |
| `GET /api/robot/map[?after=N]` | mappa dal vivo intera o differenze (sezione 7.3) |
| `GET /api/robot/trail[?after=N]` | scia dell'ultima missione |
| `GET /api/robot/maps` | elenco: `id`, `name`, `selected`, `saved_at`, `size_bytes` |
| `GET /api/robot/maps/{id}` | file RTM1 |
| `PATCH /api/robot/maps/{id}` `{"name"}` | rinomina; `name_taken` |
| `PUT /api/robot/maps/{id}/zones` | zone (sezione 7.7) |
| `DELETE /api/robot/maps/{id}` | cancella dal server |
| `GET /api/robot/schedules` | elenco degli orari con l'ultimo esito |
| `POST /api/robot/schedules` `{days, time, enabled}` | aggiunge |
| `PUT /api/robot/schedules/{id}` | modifica (anche solo `enabled`) |
| `DELETE /api/robot/schedules/{id}` | cancella |
| `GET /api/robot/log?before=&limit=` | registro dei comandi, dal più recente |

### 8.6 Richieste del robot

| Richiesta | Cosa fa |
|---|---|
| `GET /api/device/commands` | richiesta lunga: i comandi in coda, oppure attesa fino a 25 s; è anche il segno di vita |
| `POST /api/device/commands/{id}/result` | esito `{ok, reason, message}` |
| `POST /api/device/state` | stato JSON |
| `PUT /api/device/map` | mappa dal vivo intera |
| `POST /api/device/map/patch` | differenze; `409` se il server non ha la versione di partenza |
| `PUT /api/device/map/saved` | file salvato: allineamento con la mappa selezionata (sezione 7.2) |
| `GET /api/device/maps/{id}` | scarica la mappa da selezionare |

### 8.7 Programmazione delle pulizie

- Orari con giorni della settimana e ora, nell'ora locale del server (Europa/Roma, ora legale compresa).
- **Un solo orario attivo alla volta**, imposto dal server: attivarne uno spegne gli altri nella stessa operazione sul
  database.
- Un thread controlla ogni minuto; all'ora giusta manda `start` come farebbe l'app, e nel registro l'utente risulta
  "programmazione".
- **Se non riesce** riprova **10 volte, una al minuto**, poi `last_result` diventa un errore che l'app mostra in
  evidenza. `busy` (sta già pulendo) conta come riuscita.

### 8.8 Script di amministrazione

`backend/admin.py`, una TUI con menu a frecce (questionary):

| Voce | Cosa fa |
|---|---|
| Aggiungi utente | nome, password chiesta due volte senza mostrarla |
| Cambia password | cambia la password e chiude tutte le sessioni dell'utente |
| Togli utente | **chiede conferma con un avviso**, poi toglie l'utente e le sue sessioni |
| Elenco utenti | |
| Rigenera token del robot | **chiede di scrivere `ToKeN`** (maiuscole e minuscole contano), poi genera il token, lo stampa una volta sola e salva l'hash |
| Importa mappa | file RTM1 e nome |

## 9. Robot

### 9.1 Cartella

Tutto quello che collega il robot al server sta in **`tasks/vacuum/cloud/`**, separato dal progetto real-time:

| File | Contenuto |
|---|---|
| `network.c/.h` | i due thread, libcurl, token, indirizzo, riconnessione |
| `robot_state.c/.h` | stato JSON (cJSON) e quando mandarlo |
| `live_map.c/.h` | rettangolo delle posizioni, confronto, differenze in RTM1 |
| `app_commands.c/.h` | l'unico interprete dei comandi JSON |
| `commands.c/.h` | la FIFO `/tmp/rtes/vacuum-commands` (`RTES_COMMANDS`): ogni riga è un comando JSON passato all'interprete |
| `config.h` | **tutta la configurazione robot-server**; ignorato da git (sotto) |
| `config.example.h` | lo stesso con valori finti; versionato |
| `server_roots.h` | le tre radici, certificati pubblici di Let's Encrypt |

cJSON 1.7.19 in `third_party/cjson/`. Il Makefile va esteso per compilare anche `tasks/vacuum/cloud/` e cJSON.

### 9.2 Configurazione (`cloud/config.h`)

Un solo file con tutte le costanti dell'interfaccia robot-server: indirizzo del server, percorso del file del token,
tempi (richiesta lunga, intervalli dello stato e della mappa, tentativi di riconnessione), nomi delle variabili
d'ambiente per il simulatore. Se manca, la compilazione si ferma con un messaggio che dice di copiarlo dall'esempio.

| Cosa | Robot | PC / simulatore |
|---|---|---|
| Indirizzo | costante in `config.h` | `RTES_SERVER_URL` |
| Radici | `server_roots.h` | `RTES_SERVER_CA` = il certificato autofirmato |
| Token | `/mnt/UDISK/rt/robot-token` | `RTES_TOKEN_FILE`, in `build/` |

Senza il file del token la rete non parte (scritto nel registro) e il robot funziona come oggi. In `.gitignore`:
`tasks/vacuum/cloud/config.h` e `robot-token`.

### 9.3 I due thread

Due thread normali (SCHED_OTHER) sul core 3 con **nice 5**, così il task aperiodico (anche lui SCHED_OTHER sul core 3)
ha la precedenza quando cerca la posizione. Stack piccoli e fissi, come `commands.c`. Ognuno ha la sua connessione
libcurl tenuta aperta.

**Thread dei comandi:**
- ripete `GET /api/device/commands` (richiesta lunga di 25 s, tempo massimo di curl 40 s);
- per ogni comando chiama l'interprete, aspetta l'esito e lo manda;
- per `map_select` scarica la mappa (sezione 7.5);
- se la rete cade riprova dopo 1, 2, 5 e poi 10 s.

**Thread dello stato** (ciclo di 100 ms):
- stato secondo la sezione 4.2;
- mappa dal vivo secondo la sezione 7.3;
- quando cambia `read_map_save_count()` manda il file salvato;
- all'avvio: mappa salvata, poi mappa dal vivo intera.

### 9.4 Interprete dei comandi

`app_commands.c`, usato dal thread dei comandi e dalla FIFO; i due si alternano con un mutex fra loro soli, mai con i
task.

| Comando | Chi lo esegue |
|---|---|
| `fault_reset` | l'interprete chiama `drv_azzera_guasto()` (aspetta fino a 50 ms: non si può chiamare da un task real-time); se fallisce `cause_still_present` |
| `start`, `resume`, `home` | se il driver è in guasto prima `drv_azzera_guasto()`, poi la navigazione |
| `pause`, `remote_*` | la navigazione |
| `zones` | richiesta al task aperiodico come oggi, poi attesa di `read_applied_user_zones_version()` |
| `map_select`, `map_reset` | rete e task aperiodico; `not_resting` se la navigazione non è in riposo o il robot non è sulla base |

**Comando alla navigazione in una sola variabile `_Atomic uint64_t`**, senza doppio buffer nuovo (sull'ARMv7 una
lettura o scrittura atomica da 64 bit è un'istruzione sola, senza lock):

| Campo | Bit |
|---|---|
| numero del comando | 16 |
| tipo | 4 |
| v in mm/s | 16 |
| ω in mrad/s | 16 |
| spazzole | 1 |
| aspirazione | 1 |

La navigazione la legge a ogni job: un numero diverso dall'ultimo vuol dire comando nuovo. L'esito lo pubblica in
`navigation_status_t`. L'interprete aspetta l'esito prima di passare al comando dopo, così nessun comando ne
sovrascrive un altro.

### 9.5 Modifiche ai task

**`navigation.c/.h`**
- `nav_state_t` passa in `navigation.h`, con il nuovo stato `NAV_STATE_REMOTE`.
- **Nuovo doppio buffer `navigation_status_t`**, scritto a ogni job e letto con `read_navigation_status()`: `state`,
  `mission`, `interrupted_mission`, `mission_number`, `mission_start_t`, `is_remote`, `remote_brushes`,
  `remote_suction`, `last_command_num`, `last_command_result`.
- **I tasti diventano comandi:** `read_commands` trasforma il tasto di accensione in `start`, `pause` o `resume` secondo
  lo stato e il tasto casa in `home`; il comando dell'app arriva già esplicito. Una sola funzione applica il comando e
  ritorna l'esito. Il comportamento dei tasti resta quello di oggi.
- **Guasto azzerato:** pausa se c'era una missione, riposo altrimenti; se il telecomando era attivo torna in `REMOTE`.
- **Telecomando** (sezione 6): `OBJ_CONST_SPEED` con v e ω limitati; v e ω a 0 dopo `REMOTE_MOVE_TIMEOUT`; rilascio
  dopo `REMOTE_IDLE_TIMEOUT`; in `REMOTE` niente sblocco, attraversamento e rientro per batteria; spazzole e
  aspirazione da `remote_brushes` e `remote_suction`; `drv_esci_dalla_base(1)` sulla base.

**`safety.c`:** con `read_navigation_status().is_remote` salta il ciclo delle zone proibite, base compresa. Ostacoli,
vuoto e paraurti restano.

**`mapping.c/.h`:** `copy_map_categories` riceve un rettangolo (il salvataggio passa tutta la mappa);
`LIDAR_MAPPING_MAX_RANGE` passa in `mapping.h`.

**`aperiodic.c/.h`:** mappa sempre `map.rtm` (spariscono `ACTIVE_MAP_FILE` e il numero); la codifica RTM1 di un
rettangolo diventa una funzione usata dal salvataggio e dalla mappa dal vivo; nuove `read_map_save_count()`,
`request_map_replace()`, `request_map_delete()`.

**`main.c`:** avvia la rete dopo i task, come i comandi; esce con `RESTART_EXIT_CODE` (3) quando la rete chiede il
riavvio.

**Strumenti:** `tools/prove-nav/prova.py --comando` scrive comandi JSON nella FIFO rinominata; i lanciatori
rilanciano il programma con codice di uscita 3.

### 9.6 Compilazione

- `tools/build-net-libs.sh` scarica mbedTLS 3.6 (versione a supporto lungo) e libcurl, a versioni fisse con controllo
  dello SHA-256, e le compila statiche con il compilatore del robot in `build/arm/netlibs/`. libcurl con solo HTTP e
  HTTPS, senza zlib né altri protocolli.
- Il Makefile collega `vacuum` per ARM a quelle librerie; sul PC usa la libcurl del sistema.
- Il programma cresce di circa 1 MB (48,8 MB liberi in `/mnt/UDISK`).

### 9.7 Effetti sull'analisi real-time

- Navigazione: legge una variabile atomica e pubblica `navigation_status_t` (circa 40 byte) a ogni job.
- Sicurezza: legge `navigation_status_t` a ogni job; in telecomando fa meno calcoli.
- Mappatura: nessuna modifica nel job.
- Nessun mutex nuovo, quindi nessun tempo di blocco nuovo. Cache e bus si misurano con l'app collegata.

## 10. App

### 10.1 Base tecnica

- **Flutter**, in `app/` nel progetto. **Android e web** subito; il progetto contiene anche la cartella iOS e usa solo
  librerie che funzionano anche su iPhone, così con un Mac basterà compilare.
- **Riverpod** per lo stato dell'app: ogni dato (stato del robot, mappa, orari) è un provider che si attiva quando una
  schermata lo guarda e si ferma quando nessuno lo guarda più.
- **dio** per le richieste: annullamento della richiesta lunga quando si cambia schermata, token e refresh in un punto
  solo.
- **flutter_secure_storage** per il JWT sul telefono. Sul web il token non si salva: c'è il cookie.
- Nessuna libreria per la mappa: `CustomPainter` e `InteractiveViewer`.
- Niente notifiche push.

### 10.2 Struttura (`app/lib/`)

| Cartella | Contenuto |
|---|---|
| `config/` | configurazione (nome dell'app, indirizzo), colori dell'interfaccia e **colori della mappa in un file solo**, massimi del joystick |
| `api/` | client dio: token, refresh, errori `{reason, message}`, richiesta lunga dello stato |
| `models/` | stato del robot, mappe, orari, registro: classi Dart dai JSON |
| `map/` | decodifica RTM1, mappa dal vivo con le differenze, scia, disegno, modifica delle zone |
| `screens/` | una cartella per schermata |
| `widgets/` | pulsante principale, avvisi, notifiche, joystick, conferma con avviso |

**Configurazione privata:** `app/config.json` (indirizzo del server, nome "Maurice"), ignorato da git, letto con
`flutter run --dart-define-from-file=config.json`. Si versiona `config.example.json`.

### 10.3 Comportamenti comuni

- **Rifiuti dei comandi:** notifica in alto, in qualsiasi schermata, con il `message`.
- **Avvisi:** riquadro sotto la mappa, **alto 0** quando non ce ne sono: guasto con il testo, posizione persa,
  telecomando tenuto da un altro utente, batteria scarica in telecomando, pulizia programmata fallita, mappa non ancora
  salvata, robot in riavvio.
- **Cancellazioni:** sempre con conferma e avviso.
- **Stato:** richiesta lunga continua (sezione 4.4), con 100 ms nelle schermate con la mappa e 1000 ms nelle altre,
  ferma in background.
- **Accesso:** all'apertura `POST /api/auth/refresh`; se la sessione è scaduta si torna al login.

### 10.4 Schermate

**Accesso:** utente e password. Nessuna registrazione.

**Principale** (stile Vetro: la mappa riempie lo schermo, i comandi stanno sopra in pannelli traslucidi):
- in alto: menu ☰, nome (Maurice), **"Online"/"Offline"** (in inglese) con l'ultimo contatto, batteria;
- menu delle mappe (le mappe del server più "Nessuna mappa"), attivo solo con il robot a riposo sulla base,
  altrimenti spiega perché; accanto il pulsante della gestione delle mappe;
- mappa con scia, zone, base e robot; zoom e spostamento con le dita; pulsante per ricentrare sul robot;
- **riga di stato in inglese**: missione · stato · durata, per esempio "Cleaning · Following path · 5:12";
- avvisi;
- **tre pulsanti sulla stessa riga**: principale (Avvia, Pausa o Riprendi), Casa, Telecomando;
- menu ☰: Programmazione, Registro dei comandi, Avanzate (Azzera guasto, Diagnostica), Esci.

**Telecomando:**
- prende il telecomando all'apertura e lo rilascia alla chiusura; keepalive ogni 10 s;
- mappa piccola nella parte medio-alta con il robot in tempo reale;
- sotto: a destra **un joystick** (su e giù danno v avanti e indietro, destra e sinistra danno ω, combinabili;
  anello piccolo, quattro frecce discrete, la manopola esce dall'anello quando spinta); a sinistra **Casa** e
  **Spazzole e aspirazione** (un pulsante unico per le due cose) affiancati, sotto **Azzera guasto**, sempre
  visibile e attivo solo con un guasto;
- avvisi in tempo reale: guasto, contatti, batteria scarica, posizione persa;
- **un `remote_move` alla volta**: il successivo parte solo quando arriva la risposta del precedente; quando il joystick
  torna al centro manda v = 0 e ω = 0.

**Gestione mappe:**
- menu delle mappe, aperto sulla selezionata; la mappa con le sue zone;
- Rinomina, Seleziona sul robot, Cancella (sezione 7.6);
- modifica zone: rettangoli disegnati trascinando, spostati e ridimensionati dagli angoli, **cancellati con la x in
  alto a destra** di ogni rettangolo; **Conferma** manda la lista intera, Annulla torna com'era.

**Programmazione:**
- un riquadro per orario: a sinistra **7 quadratini dei giorni** (opacità 1 se attivi, 0,5 se no), a destra **l'ora
  scritta in grande**, l'interruttore, sotto l'ultimo esito (fallita in rosso);
- **un solo interruttore acceso alla volta**: accenderne uno spegne l'altro; spegnere quello acceso disattiva la
  programmazione;
- pulsante Aggiungi; modifica con giorni e ora; cancellazione con conferma.

**Registro dei comandi:** chi, cosa, quando, esito; dal più recente.

**Diagnostica** (in Avanzate): tutto lo stato del robot (sensori, correnti, lidar, driver, limiti, sicurezza) e le
**scadenze mancate di ogni task**, dal vivo.

### 10.5 Aspetto

- **Stile Vetro**: mappa a tutto schermo, pannelli traslucidi sfocati, angoli arrotondati (14-22 px), carattere Manrope.
- **Palette "Petrolio principale, grafite accento"**, tema chiaro e scuro che segue il telefono:

| Colore | Chiaro | Scuro | Dove |
|---|---|---|---|
| Petrolio (principale) | `#0E7C78` | `#3CC2B9` | pulsante principale, Casa e pulsanti secondari (tinta leggera), cose accese (interruttori, giorni, spazzole), Aggiungi |
| Grafite (accento) | `#1D232B` | `#E6EAF0` | robot sulla mappa, manopola del joystick |

- Grigi con una punta di blu (tinta 200); colori di stato separati (verde online, ambra avviso, rosso guasto).
- **Colori della mappa** (un file di costanti): sconosciuta, libera, occupata, vuoto, zona proibita, base, scia
  grigia, robot.
- Riferimento visivo: pagina "Palette di Maurice" fatta durante la progettazione.

## 11. Sviluppo e prove

- **Fuori casa:** backend sul PC in HTTP, app web in Chrome sul PC, `backend/dev/fake_robot.py` al posto del robot.
- **Simulatore con il programma vero:** il programma accetta solo HTTPS, quindi quando si arriva a provare il thread di
  rete serve un certificato autofirmato per `localhost` (un comando di `openssl`), passato con `RTES_SERVER_CA`.
- **In casa con il robot:** il robot verso il server vero, oppure verso il backend sul PC con `RTES_SERVER_URL` e il
  certificato autofirmato copiato sul robot.
- **Prove automatiche:** backend (client di prova di Flask), interprete dei comandi e confronto della mappa dal vivo in
  C (cartella `test/`), decodifica RTM1 e differenze nell'app.

## 12. Deploy (rimandato)

Si fa al momento del deploy, non prima:
- estendere il certificato con il nuovo sottodominio (`certbot certonly --cert-name ... --expand` con tutti i nomi);
- VirtualHost di Apache che passa `/api` a gunicorn su `127.0.0.1` e serve la versione web dell'app su `/`, copiando
  la configurazione di uno dei servizi Flask già attivi;
- unità systemd, venv con Python 3.13;
- utenti e token del robot con lo script di amministrazione; file del token sul robot.

## 13. Ordine di lavoro proposto

1. **Backend** con prove automatiche e `fake_robot.py`: si prova senza robot.
2. **Robot:** librerie, cartella `cloud/`, modifiche ai task, prove con il simulatore.
3. **App:** prima stato e mappa dal vivo, poi comandi e telecomando, poi mappe, programmazione, registro, diagnostica.
4. **Prove con il robot vero**, in casa.
5. **Deploy.**

Prima della consegna all'utente: ogni parte rivista da lui; nessuna scadenza mancata nelle misure con l'app collegata.
