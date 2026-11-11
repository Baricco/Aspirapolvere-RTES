# App di Maurice: implementazione

Resoconto del lavoro fatto fra il 2026-11-10 e il 2026-11-11 sul progetto di `docs/app.md`: backend, parte di rete del
robot, app Flutter. Tutto è stato provato sul PC, senza il robot vero e senza il server vero: niente SSH al robot,
niente copie di file, niente deploy, niente commit.

Dove questo documento dice "simulatore" vuol dire `build/host/sim` con il programma vero del robot
(`build/host/vacuum` o la sua copia di prova `build/prove-nav/vacdbg`) collegato al backend sul PC in HTTPS, con un
certificato autofirmato per `localhost`.

## 1. Cosa è stato fatto

### 1.1 Backend (`backend/`, sezione 8 del progetto)

Python 3.14 del PC (il progetto dice 3.13: il codice usa solo cose che ci sono in tutte e due), Flask, gunicorn con un
processo e 32 thread, SQLite in WAL, PyJWT, questionary. Dipendenze in `backend/.venv`, ignorato da git.

| File | Contenuto |
|---|---|
| `app.py` | crea l'app Flask: blueprint, controllo di `Origin`, intestazioni CORS solo per le origini di sviluppo, gzip delle risposte JSON, errori nella forma `{reason, message}`; avvia stato vivo e programmazione |
| `auth.py` | utenti (scrypt con sale di 16 byte), blocco dopo 5 tentativi per 1 minuto, JWT HS256 con `jti` = sessione, controllo della sessione a ogni richiesta, refresh, logout, cookie `HttpOnly; Secure; SameSite=Strict; Path=/api` per il web |
| `robot_link.py` | stato vivo in memoria: ultimo stato, `online`, numero dello stato, richiesta lunga dello stato con `min_interval_ms`, coda dei comandi con le attese (200, 409, 503, 504), richiesta lunga del robot, telecomando con il suo timer; l'ultimo stato va su disco ogni 10 s |
| `robot_api.py` | tutte le richieste dell'app della sezione 8.5 e i comandi della sezione 5.3, registro dei comandi |
| `device_api.py` | le richieste del robot della sezione 8.6, controllo del token (SHA-256 nel database) |
| `maps.py` | RTM1 (lettura con tutti i controlli, scrittura), mappe salvate e allineamento con il robot (sezione 7.2), mappa dal vivo con le ultime differenze, scia |
| `scheduler.py` | pulizie programmate: un controllo al minuto nell'ora di Roma, un solo orario attivo, 10 nuovi tentativi uno al minuto, `busy` conta come riuscita; pulizia del registro oltre 30 giorni |
| `db.py`, `schema.sql` | SQLite e le tabelle della sezione 8.3 |
| `config.py` | tutte le costanti (tempi, limiti, nomi dei file) e la lettura delle variabili d'ambiente |
| `responses.py` | forma comune delle risposte |
| `admin.py` | menu a frecce: aggiungi utente, cambia password, togli utente (con avviso), elenco, rigenera token (chiede `ToKeN`), importa mappa |
| `config.example.env`, `requirements.txt` | esempio della configurazione privata, dipendenze |
| `dev/fake_robot.py` | robot finto: parla con il backend come il programma vero, con una stanza, una pulizia a strisce, telecomando, zone, cambio di mappa con "riavvio", guasto e posizione persa da tastiera |
| `dev/run.sh`, `dev/make-cert.sh`, `dev/tls_proxy.py` | backend di sviluppo (HTTP sulla 5000), certificato autofirmato in `build/app-dev/`, porta HTTPS (5443) davanti al backend come farà Apache |
| `deploy/` | esempi per il deploy, con `<server>` e percorsi finti: configurazione di gunicorn, unità systemd, VirtualHost di Apache |
| `tests/` | 95 prove automatiche (sezione 3) |

### 1.2 Parte del robot (sezione 9 del progetto)

**Librerie.** `tools/build-net-libs.sh` scarica mbedTLS 3.6.7 (ramo a supporto lungo) e libcurl 8.23.0 dai siti
ufficiali, controlla lo SHA-256 pubblicato prima di aprire l'archivio, e le compila statiche con il compilatore del robot
in `build/arm/netlibs/`. libcurl ha solo HTTP e HTTPS (nessun altro protocollo, niente zlib, niente proxy, niente
cookie); mbedTLS ha i mutex attivi, perché i due thread di rete usano TLS insieme.

**cJSON 1.7.19** in `third_party/cjson/` (`cJSON.c`, `cJSON.h`, `LICENSE`), scaricato dal repository ufficiale.

**Cartella `tasks/vacuum/cloud/`:**

| File | Contenuto |
|---|---|
| `network.c/.h` | i due thread (core 3, scheduling normale, nice 5, stack di 256 KB), connessioni libcurl tenute aperte, solo `https` (`CURLOPT_PROTOCOLS_STR`), verifica del certificato con le radici del programma o con `RTES_SERVER_CA`, token dal file, riconnessione dopo 1, 2, 5 e 10 s, scaricamento della mappa per `map_select`, richiesta di riavvio |
| `robot_state.c/.h` | lo stato JSON della sezione 4.1, letto solo con `read_*` e `drv_*`, e la regola di invio della sezione 4.2 |
| `live_map.c/.h` | mappa dal vivo (sezione 7.3): copia privata, rettangolo delle posizioni, lettura a rettangolo, confronto, invio delle differenze, 409, mappa intera; invio della mappa salvata; controllo dei file RTM1 |
| `app_commands.c/.h` | l'unico interprete dei comandi JSON (sezione 9.4) |
| `commands.c/.h` | la FIFO dei comandi (`/tmp/rtes/vacuum-commands`, sul PC `RTES_COMMANDS`): ogni riga è un comando JSON passato all'interprete. Sostituisce `tasks/vacuum/commands.c/.h` |
| `config.example.h` | tutta la configurazione robot-server con valori finti (versionato); `config.h` è la copia privata, ignorata da git |
| `config_check.h` | ferma la compilazione con un messaggio se manca `config.h` |
| `server_roots.h` | Root YE, ISRG Root X2, ISRG Root X1, scaricate da letsencrypt.org, con le loro impronte SHA-256 |

**Modifiche ai task:** elencate una per una nella sezione 4.

**Compilazione e lanciatori:**

- `Makefile`: regole apposta per `build/host/vacuum` (libcurl del sistema) e `build/arm/vacuum` (librerie di
  `build/arm/netlibs/`), con un messaggio chiaro se mancano `config.h` o le librerie; i tre test nuovi nella lista dei
  test. Il programma per ARM compila senza avvisi: statico, 2,1 MB (1,1 MB di codice).
- `tools/prove-nav/costruisci.sh`: compila anche `cloud/` e cJSON.
- `rtes.sh sim` e `tools/driver-robot.sh` rilanciano il programma finché esce con il codice 3.
- `tools/prove-nav/prova.py`: rilancia il programma con il codice 3, `--comando` scrive comandi JSON nella FIFO,
  `--attendi` lascia continuare la prova fino alla durata, `NOMI` ha il nuovo stato `REMOTE`.

**Prove in `test/`:** `test_nav_command.c`, `test_live_map.c`, `test_app_commands.c` (sezione 3).

**Prove complete nel simulatore:** `tools/prove-app/banco.py` (backend, porta HTTPS, simulatore, programma vero),
`tools/prove-app/scenari.py` (comandi, telecomando, guasto, zone, mappe, rete), `tools/prove-app/missione.py` (la
missione di confronto con il backend collegato).

### 1.3 App Flutter (`app/`, sezione 10 del progetto)

Progetto con Android, web e la cartella iOS; Riverpod, dio, flutter_secure_storage, carattere Manrope incluso
nell'app (licenza OFL accanto). L'aspetto segue `App_Mockup.png` dove non è in contrasto con `docs/app.md`.

| Cartella | Contenuto |
|---|---|
| `lib/config/` | `app_config.dart` (nome e indirizzo da `config.json`, tempi, massimi del joystick), `app_colors.dart` (palette petrolio e grafite, chiaro e scuro, misure dello stile Vetro, tema), `map_colors.dart` (tutti i colori della mappa e le misure del disegno) |
| `lib/api/` | `api_client.dart` (dio: token sul telefono, cookie sul web, refresh, errori `{reason, message}`, richiesta lunga dello stato), `providers.dart` (sessione, notifiche, stato del robot, mappa dal vivo, scia, mappe, orari, riavvio) |
| `lib/models/` | stato del robot, mappa, orario, riga del registro |
| `lib/map/` | `rtm.dart` (decodifica RTM1, CRC-32, mappa dal vivo con le differenze), `trail.dart`, `map_painter.dart` (disegno con `CustomPainter`), `map_view.dart` (`InteractiveViewer`, ricentra, modifica delle zone con le dita), `zone_editor.dart` (disegna, sposta, ridimensiona, cancella) |
| `lib/screens/` | accesso, principale, telecomando, gestione mappe, programmazione, registro, diagnostica |
| `lib/widgets/` | pannelli di vetro, pulsanti, avvisi, notifica in alto, conferma con avviso, joystick, Online/Offline e batteria, intervallo dello stato per schermata, date in italiano |
| `config.example.json` | esempio della configurazione privata; `config.json` è ignorato da git |
| `test/` | prove automatiche (sezione 3) |

### 1.4 Documenti

- `docs/formato-mappa.md`: aggiornata la parte "Dove sta" (una sola mappa `map.rtm` sul robot, tutte sul server) e
  l'implementazione (codifica comune, altri lettori del formato).
- `docs/GUIDA.md`: nuova sezione 14 con i comandi di backend, programma collegato al backend, librerie, prove, app.
- `docs/app.md` non è stato toccato.

## 2. Scelte fatte dove il progetto non diceva niente, e differenze dal progetto

### 2.1 Decise con te durante il lavoro

- **Registro dei comandi:** i `remote_keepalive` non si scrivono; i `remote_move` solo quando cambiano rispetto
  all'ultimo (`COMMAND_LOG_SKIPPED_TYPES` e `COMMAND_LOG_ONLY_CHANGES_TYPES` in `backend/config.py`). Il progetto
  diceva "ogni comando finisce nel registro".
- **Prove nel simulatore** nella stanza vuota; una missione che finisce la pulizia e fallisce l'aggancio conta come
  riuscita.
- **Brave** al posto di Chrome per le prove dell'app web.
- **Mockup:** la UI segue `App_Mockup.png`, ma dove è in contrasto con il progetto vince il progetto (sezione 2.4).

### 2.2 Robot

- **Formato delle richieste del robot**, non scritto nel progetto: `PUT /api/device/map?version=N`,
  `POST /api/device/map/patch?base_version=N&version=N+1`, `PUT /api/device/map/saved?reason=startup|save` (corpo vuoto
  = il robot non ha la mappa). Il comando `map_select` porta `map_id` e `crc`.
- **La risposta allo stato dice al robot quale versione della mappa dal vivo ha il server** (`live_map_version`, null
  se nessuna). Aggiunta dopo una prova fallita: se il server riparte mentre il robot è fermo, nessuna differenza parte,
  quindi il 409 non arriva mai e il server resta con la mappa vuota. Con questa informazione il robot rimanda la
  copia intera (dalla sua memoria, come per il 409).
- **Interprete dei comandi:** aspetta l'esito della navigazione al massimo 1 s, le zone applicate al massimo 4 s, la
  sostituzione o cancellazione del file al massimo 5 s (`COMMAND_RESULT_TIMEOUT`, `ZONES_RESULT_TIMEOUT`,
  `MAP_FILE_RESULT_TIMEOUT` in `config.example.h`); poi risponde `no_answer`. Un motivo `invalid_command` per JSON
  sbagliato o tipo sconosciuto.
- **`remote_cleaning` con l'aspirazione e senza cassetto:** il telecomando si prende lo stesso (come ogni comando del
  telecomando) ma spazzole e aspirazione non cambiano, e la risposta è `dustbin_missing`.
- **Zone con la posizione non normale** (persa o da confermare): rifiutate con `position_lost`, perché la mappatura le
  applica solo con la posizione normale.
- **"A riposo sulla base"** per `map_select` e `map_reset` = navigazione in riposo e `sulla_base` del driver.
- **Tasto di accensione durante il telecomando:** diventa `pause`, cioè rilascio del telecomando e pausa (o riposo
  senza missione), come lo stesso comando dell'app.
- **Durata della missione** nello stato: conta dall'inizio della missione (`start`, oppure `home` da fermo, oppure la
  ripresa dopo la ricarica) e resta ferma quando la missione finisce.
- **Mappa dal vivo:** margine di 4 celle attorno al rettangolo (`LIVE_MAP_MARGIN_CELLS`); se la copia fra i job della
  mappatura non riesce si riprova al giro dopo, senza errori.
- **FIFO dei comandi:** accetta solo comandi JSON; il vecchio formato testuale (`zona aggiungi ...`) non c'è più.
- **Stato:** oltre ai cambiamenti elencati nel progetto, conta come importante anche quando il robot si ferma (così
  l'ultimo stato ha le velocità a zero) e quando si accendono o spengono spazzole e aspirazione.
- **Server e contatto:** consegnare un comando al robot non rinnova più il suo "ultimo contatto": se la connessione è
  morta il server non lo sa, e il comando sparisce. Una richiesta lunga più nuova del robot chiude le precedenti, così
  un comando non va a una connessione morta.

### 2.3 Backend

- **Comandi:** una sola finestra di 3 s dall'arrivo della richiesta; con il robot offline il 503 è immediato. Se
  l'esito arriva dopo il 504, il registro viene corretto.
- **Login dal web:** il campo `"use_cookie": true` nel corpo fa mettere il JWT solo nel cookie.
- **Orari:** `days` è una lista di 7 booleani da lunedì, `time` è `"HH:MM"`; i tentativi sono il primo più 10 nuovi.
- **Cancellazione della mappa selezionata:** `DELETE` risponde 409 `map_selected` se prima non è passata da "Nessuna
  mappa" (l'app fa i due passi da sola).
- **Messaggio della mappa dal vivo per l'app:** un byte di tipo (intera o differenze), poi parti con versione (64 bit),
  lunghezza e rettangolo RTM1. La mappa vuota è una cella sconosciuta nell'origine. La versione per le app parte
  dall'orologio, così non torna indietro dopo un riavvio del server.
- **Scia:** la risposta porta un `trail_id` che cambia quando la scia ricomincia; un punto nuovo solo dopo 3 cm.
- **Parametri:** scrypt con N = 2^14, la chiave del JWT si crea al primo avvio nella cartella dei dati, gzip solo per le
  risposte JSON di almeno 256 byte, nomi delle mappe al massimo 40 caratteri, password di almeno 8 caratteri nello
  script.

### 2.4 App

- **Mockup e progetto in contrasto:** nel mockup la schermata principale non ha la barra con menu, nome, Online e
  batteria: c'è, perché la vuole il progetto. Il resto (menu delle mappe, pulsante per ricentrare, riga di stato,
  avvisi, tre pulsanti, telecomando, programmazione, notifica in alto) segue il mockup.
- **Joystick:** lo stesso valore si ripete al massimo ogni 200 ms (`remoteMoveRepeatInterval`), un valore nuovo parte
  appena arriva la risposta del precedente; v e omega arrotondate al centesimo, zona morta del 12%. Senza questa
  pausa, con il robot finto, partivano circa 100 comandi al secondo.
- **Riga di stato:** le traduzioni inglesi di missioni e stati sono mie (per esempio `follow_path` → "Following path").
  La durata si vede solo con una missione in corso.
- **Avvisi dei guasti:** per ogni guasto chiesto da un task un titolo in italiano e cosa fare, che nomina il pulsante
  giusto (Riprendi con una missione in corso, Avvia senza). Per i guasti del driver resta il testo del robot.
- **Mappa:** si inquadra da sola finché l'utente non la tocca, poi solo con il pulsante per ricentrare. Scia larga
  16 cm. Nella modifica delle zone un dito disegna e due spostano la mappa.
- **App web:** compilata con `--pwa-strategy=none`, cioè senza il service worker che teneva in cache la versione
  vecchia.
- **Notifica in alto:** sparisce dopo 4 s o toccandola; mentre è visibile copre la parte alta dello schermo.

## 3. Risultati delle prove

### 3.1 Prove automatiche

| Dove | Prove | Esito |
|---|---|---|
| backend (`backend/tests/`) | 95: accesso, blocco del login, JWT e sessioni, cookie, `Origin` e CORS di sviluppo, token del robot, richiesta lunga dello stato (subito, 304, sveglia, `min_interval_ms`, online/offline), gzip, ultimo stato su disco, comandi (tutti i tipi, 409, 503 subito, 503 dopo 3 s con il comando buttato, 504 e esito tardivo, 400), telecomando (uno alla volta, presa automatica, timeout di 30 s, comandi di missione del proprietario, rifiuti), registro, RTM1, mappa dal vivo (intera, differenze, 409, troppo indietro), mappe salvate e allineamento, zone, scia, programmazione (orari, un solo attivo, giorni, ora legale, tentativi, `busy`) | tutte passano |
| robot, `test/test_nav_command.c` | impacchettamento del comando in 64 bit | 109 controlli, 0 falliti |
| robot, `test/test_live_map.c` | CRC, codifica RTM1, codifica e decodifica, confronto, mappa dal vivo verso un server finto (mappa vuota, differenze, finestra letta, posizione non valida, rete giù, copia non riuscita, 409, server ripartito senza differenze, mappa salvata, nuovo avvio), sostituzione e cancellazione del file nel task aperiodico | 117 controlli, 0 falliti |
| robot, `test/test_app_commands.c` | interprete: tutti i comandi, rifiuti della navigazione, telecomando e cassetto, guasto e azzeramento, zone, cambio di mappa, JSON sbagliati, navigazione che non risponde | 127 controlli, 0 falliti |
| robot, gli stessi tre compilati per ARM | con `qemu-arm-static` | tutti passano |
| `./rtes.sh test` | tutti i test del progetto | tutti passano |
| app (`app/test/`) | 23: decodifica RTM1 di file generati dal backend, CRC, file rovinati, mappa dal vivo e differenze, scia, modelli JSON, pulsante principale, riga di stato, avvisi, modifica delle zone | tutte passano |
| app, compilazione | `flutter analyze` senza problemi; versione web; versione Android (`flutter build apk`, 54 MB) | compilano; l'APK non è stato installato su un telefono |

Il salvataggio della mappa scrive gli stessi byte di prima: i file salvati dal codice vecchio e da quello nuovo
coincidono con la ricodifica del backend (stessa codifica, stesse sequenze, stesso CRC).

Su ARM il comando a 64 bit della navigazione si legge e si scrive con istruzioni atomiche (`ldrexd`/`strexd`), senza
chiamate a librerie e senza lock.

### 3.2 Prove nel simulatore con il backend collegato

Ogni scenario riparte da zero (dati del server nuovi, robot sulla base nella stanza vuota) e alla fine controlla le
scadenze mancate dei task.

| Scenario | Cosa prova | Controlli |
|---|---|---|
| comandi | collegamento in HTTPS con il certificato autofirmato, stato completo, rifiuti a riposo, start, uscita dalla base, mappa dal vivo al server, differenze, scia, pausa (robot fermo), ripresa, home due volte, registro | 32 su 32 |
| telecomando | presa automatica da un altro utente, un utente alla volta, rilascio, uscita dalla base in avanti, velocità massima, stop dopo 0,5 s, omega limitata, spazzole e aspirazione, cassetto mancante, cambio di mappa rifiutato, keepalive, **rilascio da solo dopo 30 s** su robot e server, telecomando durante una missione (pausa, rilascio, ripresa), registro | 38 su 38 |
| guasto | una scatola bassa davanti alla base fa fallire l'uscita: guasto; telecomando preso in guasto, robot fermo; robot spostato a mano; azzeramento: torna in telecomando; rilascio: pausa (non riposo); posizione ritrovata; ripresa; rientro | 21 su 21 |
| zone | zone dall'app, stato, rifiuti; in telecomando il robot entra nella zona proibita | 15 su 15 |
| mappe | mappa importata, selezione, **riavvio con codice 3** e mappa nuova sul robot, mappa dal vivo uguale a quella caricata, zone sulla selezionata salvate e rimandate al server, "Nessuna mappa" con riavvio, cancellazione | 19 su 19 |
| rete | rete del robot giù per 37 s durante una missione: comandi 503 o 504, il robot continua, il server lo dà offline, poi si ricollega da solo e la mappa dal vivo è intera; backend riavviato: il robot si ricollega e rimanda la mappa intera | 15 su 15 |

Altre prove:

- **Senza il file del token** il programma scrive "la rete non parte" e funziona come prima: le missioni di confronto
  senza rete sono fatte così.
- **Tasti fisici e FIFO** (stanza vuota, 100 s): tasto di accensione per avviare, mettere in pausa e riprendere, tasto
  casa; comandi JSON dalla FIFO (`pause`, `start` rifiutato con `busy`, `resume`, `zones`), riga testuale vecchia
  rifiutata. I tasti fanno quello che facevano prima (sezione 4.1, la sola differenza voluta è il guasto azzerato).

### 3.3 Confronto dei task prima e dopo

Stessa missione: `tools/prove-nav/prova.py --missione --dalla-base sim/stanze/vuota.txt 1 1800`, programma di prova
`vacdbg` (i task più le righe di registro della navigazione). Prima: il codice dei task come era nel repository. Dopo
senza rete: con le modifiche, file del token assente. Dopo con il backend: con le modifiche, collegato al backend con
un'app finta che tiene aperta la richiesta lunga dello stato a 100 ms (`tools/prove-app/missione.py`).

| | Prima, prova 1 | Prima, prova 2 | Dopo, senza rete | Dopo, con il backend |
|---|---|---|---|---|
| esito | pulizia finita, aggancio fallito | pulizia finita, agganciato | pulizia finita, aggancio fallito | pulizia finita, aggancio fallito |
| durata della pulizia | 1.020 s | 516 s | 619 s | 666 s |
| job lidar | 5.716 | 3.181 | 3.722 | 4.027 |
| job movimento e sicurezza | 57.140 | 31.793 | 37.202 | 40.254 |
| job navigazione | 11.431 | 6.361 | 7.444 | 8.053 |
| job mappatura | 5.717 | 3.182 | 3.723 | 4.028 |
| **scadenze mancate, tutti i task** | **0** | **0** | **0** | **0** |

Tempo di esecuzione medio / massimo dei job, in µs:

| Task | Prima, prova 1 | Prima, prova 2 | Dopo, senza rete | Dopo, con il backend |
|---|---|---|---|---|
| lidar | 162 / 362 | 89 / 270 | 125 / 344 | 89 / 266 |
| movimento | 24 / 2.879 | 14 / 536 | 15 / 66 | 14 / 374 |
| sicurezza | 12,5 / 98 | 6,6 / 62 | 8,1 / 33 | 7,0 / 86 |
| navigazione | 71 / 250 | 39 / 235 | 50 / 206 | 37 / 175 |
| mappatura | 297 / 1.288 | 169 / 876 | 227 / 2.052 | 165 / 602 |

Tempo di risposta massimo (dal rilascio alla fine del job), in µs: lidar 2.766 / 2.474 / 1.669 / 2.340; movimento
2.932 / 2.927 / 2.835 / 2.871; sicurezza 2.964 / 2.868 / 2.796 / 2.887; navigazione 3.179 / 3.108 / 3.401 / 3.008;
mappatura 2.953 / 2.960 / 3.350 / 2.942. Task aperiodico: 1-2 job (salvataggi), al massimo 2,0-2,5 ms.

Attivazioni perse: zero nella missione con il backend e in tutti gli scenari (lette dallo stato mandato all'app). La
stampa delle statistiche di `rt/` non le include, quindi per le due missioni prima delle modifiche e per quella senza
rete il registro non le mostra.

**Come leggere questi numeri.** Sul PC i task girano senza `SCHED_FIFO` (manca il permesso) e con il simulatore: la
logica e le attivazioni sono realistiche, i tempi no. Due prove identiche prima delle modifiche durano una il doppio
dell'altra, quindi i numeri dei job non si confrontano fra loro uno a uno. Le differenze dei massimi sono dentro quelle
fra due prove uguali (per esempio movimento 2.879 contro 536 µs prima delle modifiche). Quello che conta qui è che in
tutte le prove, con e senza backend, non c'è nessuna scadenza mancata. Gli effetti veri su cache e bus si misurano sul
robot (sezione 5).

### 3.4 Prova a mano dell'app nel browser

App web compilata e servita su `http://localhost:8090`, backend di sviluppo sulla 5000, robot finto. Brave senza
finestra con schermo da telefono (412 × 892), tema scuro e chiaro.

| Schermata | Provato |
|---|---|
| accesso | password sbagliata ("Utente o password sbagliati."), accesso, riapertura senza rifare l'accesso (refresh), Esci (torna all'accesso e ci resta dopo una ricarica) |
| principale | Online e batteria, menu delle mappe (con il robot fuori dalla base spiega perché non si può cambiare), mappa dal vivo con scia, zone, base e robot, inquadratura automatica, ricentra, riga di stato in inglese, avvisi (mappa non salvata, guasto con il consiglio, posizione persa, pulizia programmata fallita, riavvio in corso), Avvia, Pausa, Riprendi, Casa (rifiuto "Il robot è già sulla base." come nel mockup), cambio di mappa dal menu con riavvio, tema chiaro |
| telecomando | presa all'apertura, joystick (il robot gira in arco, al rilascio `v = 0, ω = 0`), spazzole e aspirazione, avvisi in tempo reale, Azzera guasto attivo solo con il guasto, rilascio alla chiusura e pausa della missione |
| gestione mappe | menu aperto sulla selezionata, mappa con le zone, disegno di una zona, Conferma (arriva al robot, che salva e la rimanda), x su una zona di una mappa non selezionata (il server riscrive il file), Rinomina, Cancella con l'avviso |
| programmazione | aggiunta di due orari, giorni accesi e spenti, un solo interruttore acceso, ultimo esito riuscito e fallito (in rosso) |
| registro | chi, cosa, quando, esito, parametri; i movimenti uguali una volta sola, nessun keepalive |
| diagnostica | scadenze mancate e attivazioni perse di ogni task, tutte le sezioni dello stato, dal vivo |

Difetti trovati e corretti in queste prove: testi dei pulsanti tagliati, durata non allineata a destra, mappa che si
inquadrava solo quando era ancora minuscola, versione vecchia servita dal service worker, etichetta "Spazzole e
aspirazione" spezzata a metà parola, 100 `remote_move` al secondo, numeri come `0.5700000000000001` nel registro, giorni
dei riquadri su due righe, evidenziazione dei riquadri fuori dagli angoli arrotondati, intestazioni della diagnostica
sovrapposte, consiglio del guasto con il pulsante sbagliato, durata dell'ultima missione mostrata a riposo.

## 4. Modifiche ai task di `tasks/vacuum/`, una per una

Nessuna modifica a `driver/`, `rt/`, `gd32/`, `lidar/`, `sim/`. Nessun mutex nuovo condiviso con i task: l'unico
mutex nuovo (in `cloud/app_commands.c`) lo usano solo il thread di rete e quello della FIFO.

### 4.1 `navigation.h` e `navigation.c`

1. **`nav_state_t` passa in `navigation.h`** (prima era in `navigation.c`), con il nuovo stato `NAV_STATE_REMOTE` in
   fondo, così gli altri valori non cambiano (lo usano `prova.py` e l'app).
2. **Costanti del telecomando** in `navigation.h`: `REMOTE_MAX_V` 0,35, `REMOTE_MAX_V_BACK` 0,35, `REMOTE_MAX_OMEGA`
   1,2, `REMOTE_MOVE_TIMEOUT` 0,5 s, `REMOTE_IDLE_TIMEOUT` 30 s.
3. **Comando dell'app in 64 bit** (`navigation.h`): `nav_command_type_t` (tasti e app: start, pause, resume, home,
   remote_take, remote_move, remote_cleaning, remote_keepalive, remote_release), `nav_command_result_t` (ok, busy,
   not_running, not_paused, already_home, position_lost, in_fault), `navigation_command_t`, e `pack_navigation_command`
   / `unpack_navigation_command` con i campi della tabella del progetto: numero 16 bit, tipo 4, v in mm/s 16, ω in
   mrad/s 16, spazzole 1, aspirazione 1. Le posizioni dei campi sono `#define`.
4. **Nuovo doppio buffer `navigation_status_t`**: `state`, `mission` (`SLEEP` se non c'è una missione),
   `interrupted_mission`, `mission_number` (quello che la navigazione già manda alla mappatura), `mission_start_t`,
   `is_remote`, `remote_brushes`, `remote_suction`, `last_command_num`, `last_command_result`. Lo scrive
   `publish_status()` alla fine di ogni job (anche dopo i `return` anticipati: `job_navigation` chiama
   `job_navigation_body` e poi pubblica) e lo legge `read_navigation_status()`.
5. **`send_navigation_command()`**: una scrittura atomica della variabile `_Atomic uint64_t app_command`. A ogni job la
   navigazione la legge una volta; un numero diverso dall'ultimo vuol dire comando nuovo, e l'esito va in
   `navigation_status_t`.
6. **Campi nuovi dello stato interno**: `is_mission_active` e `mission_start` (c'è una missione e da quando),
   `last_command_num` e `last_command_result`, e per il telecomando `is_remote`, `remote_v`, `remote_omega`,
   `remote_brushes`, `remote_suction`, `remote_last_move`, `remote_last_command`.
7. **I tasti diventano comandi**: `read_commands()` ora ritorna un comando. Tasto di accensione: con la posizione persa
   `home`, in riposo `start`, in pausa `resume`, altrimenti `pause`. Tasto casa: `home`.
8. **Una sola funzione applica i comandi**, `apply_command()`, sia dei tasti sia dell'app, e ritorna l'esito.
   Sostituisce `apply_commands()`. Per i tasti fa le stesse cose di prima:
   - riposo: accensione = nuova pulizia; casa = niente sulla base, rientro fuori;
   - pausa: accensione = ripresa passando dall'attesa; casa = rientro;
   - rientro e aggancio: accensione = pausa; casa = niente;
   - in missione: accensione = pausa; casa = rientro, oppure riposo se il robot è ancora sulla base;
   - posizione persa: qualsiasi tasto = aggancio alla cieca, se non è già partito.

   In più ci sono i rifiuti per l'app (`busy`, `not_running` e gli altri), che i tasti ignorano.
9. **Guasto azzerato** (`leave_failure()`, controllato a ogni job prima dei comandi): se il telecomando era attivo si
   torna in `REMOTE`; se c'era una missione si va in **pausa** (prima si andava sempre in riposo); altrimenti riposo.
   Vale anche per il tasto di accensione e per il ritorno sulla base, come chiede il progetto nella sezione 5.4. È
   l'**unico cambiamento voluto del comportamento dei tasti**: prima, dopo un guasto a metà pulizia, il tasto faceva
   partire una pulizia nuova; ora la prima pressione azzera il guasto e mette in pausa, la seconda riprende tenendo le
   celle già pulite.
10. **Telecomando**: `start_remote()` (sospende la missione, annulla l'aggancio, `drv_esci_dalla_base(1)` per uscire
    piano in avanti dalla base; con un guasto in corso il robot resta in guasto), `end_remote()` (pausa se c'era una
    missione, altrimenti riposo; `drv_esci_dalla_base(0)`), `check_remote_timeout()` (rilascio dopo
    `REMOTE_IDLE_TIMEOUT` senza comandi del telecomando). Start, pause, resume e home con il telecomando attivo lo
    rilasciano e poi vengono eseguiti. Con la posizione persa `remote_move` risponde `position_lost` e il robot resta
    fermo.
11. **Stato `NAV_STATE_REMOTE`**: `OBJ_CONST_SPEED` con v e ω dell'ultimo `remote_move`, a zero dopo
    `REMOTE_MOVE_TIMEOUT`, limitate prima ai massimi del telecomando e poi a `max_v_forw` e `max_v_backw` della
    sicurezza. Fuori dalla base segna l'uscita come fatta, così una missione ripresa non la rifà.
12. **In `REMOTE` niente sblocco, attraversamento e rientro per batteria**: aggiunto `NAV_STATE_REMOTE` alle
    eccezioni dello sblocco e di `is_battery_dead()`; l'attraversamento parte solo da segui percorso e uscita dalla
    base, quindi non serve altro.
13. **Spazzole e aspirazione**: in `REMOTE` le decidono `remote_brushes` e `remote_suction`, una per una; lo sblocco
    della spazzola centrale funziona come sempre. Se lo sblocco manda il robot in guasto, nello stesso job tutto si
    spegne (prima succedeva perché `should_activate_brushes()` veniva richiamata dopo).
14. **Pulizia attiva**: `is_active_cleaning()` esclude `REMOTE`, così il telecomando non segna celle pulite.
15. **Posizione non valida**: in `REMOTE` il robot si ferma ma resta in telecomando invece di passare all'attesa.
16. **Missione in corso**: `start_new_mission()`, `go_home_now()` e `start_blind_hook_up()` da fermo, e la ripresa dopo
    la ricarica la segnano attiva; `set_state(NAV_STATE_SLEEP)` la chiude.

### 4.2 `safety.c`

1. Include `navigation.h`; se `read_navigation_status().is_remote` il ciclo delle zone proibite si salta (base
   compresa) e le distanze dalle zone restano infinite. Ostacoli del lidar, incastro, slittamento, sforzo, spazzola e
   contatti restano.

### 4.3 `mapping.h` e `mapping.c`

1. `LIDAR_MAPPING_MAX_RANGE` passa da `mapping.c` a `mapping.h` (la usa la mappa dal vivo).
2. `copy_map_categories()` riceve un rettangolo (`rect_x`, `rect_y`, `rect_width`, `rect_height`) e copia solo i
   byte di quel rettangolo; il salvataggio passa tutta la mappa. `user_zones` può essere `NULL`. Il job della mappatura
   non cambia.

### 4.4 `aperiodic.h` e `aperiodic.c`

1. La mappa è sempre `map.rtm`: tolti `ACTIVE_MAP_FILE`, `ACTIVE_MAP_DEFAULT` e il numero della mappa;
   `map_file_path()` è più semplice.
2. La codifica RTM1 di un rettangolo diventa `encode_map_rectangle()` (a pezzi di 256 byte verso chi scrive: il file
   nel salvataggio, la memoria nella mappa dal vivo), con `find_map_rectangle()` e `get_map_category()` (in
   `aperiodic.h`). `save_map()` le usa e scrive esattamente gli stessi byte di prima.
3. `request_user_zones()` ritorna il numero della richiesta (l'interprete aspetta che la mappatura la applichi).
4. Nuove `request_map_replace()` e `request_map_delete()`, con `read_map_file_request_result()`: il task aperiodico
   rinomina `map.rtm.download` sopra `map.rtm`, oppure cancella `map.rtm`. Da quel momento non salva più, perché il
   programma sta per riavviarsi (`is_saving_disabled`).
5. Nuova `read_map_save_count()`: cresce a ogni salvataggio riuscito, così il thread dello stato sa quando mandare il
   file al server.
6. Corretta l'indentazione di una riga in `job_aperiodic()`.

### 4.5 `main.c`

1. Include `cloud/commands.h` e `cloud/network.h`; `start_network()` parte dopo i task e dopo la FIFO.
2. Il ciclo di attesa finisce anche quando la rete chiede il riavvio; dopo aver fermato i task e stampato le
   statistiche, il programma esce con `RESTART_EXIT_CODE` (3).

### 4.6 `commands.c` e `commands.h`

Spostati in `cloud/`, con i comandi JSON passati all'interprete al posto dei comandi testuali delle zone.

### 4.7 Effetti sull'analisi real-time

- Navigazione: a ogni job una lettura atomica da 64 bit, una scrittura del doppio buffer dello stato (circa 50 byte) e
  una lettura di `read_mapping_correction()` in più solo quando arriva un tasto o un comando.
- Sicurezza: a ogni job una lettura del doppio buffer della navigazione; in telecomando salta il ciclo delle zone.
- Mappatura: nessuna modifica al job; la copia a rettangolo la fanno i thread di rete fra un job e l'altro, come il
  salvataggio.
- Nessun mutex nuovo con i task, quindi nessun tempo di blocco nuovo. Thread di rete sul core 3 con nice 5.

## 5. Cosa resta da fare

1. **Prove con il robot vero** (in casa): programma ARM sul robot, file del token, collegamento al server (o al
   backend sul PC con il certificato autofirmato copiato sul robot); prove di tutti i comandi, telecomando, cambio di
   mappa con il riavvio vero (il rilancio con il codice 3 di `tools/driver-robot.sh` non è stato provato sul robot).
2. **Orologio da spento**: spegnere il robot per qualche minuto e controllare che l'ora resti giusta; se no, mbedTLS
   va compilato senza `MBEDTLS_HAVE_TIME_DATE` (sezione 3 del progetto).
3. **Misure con l'app collegata**: tempi dei task, cache e bus, con il robot vero e `SCHED_FIFO`; nessuna scadenza
   mancata. I numeri del PC qui sopra non valgono per questo.
4. **Deploy** (sezione 12 del progetto): certificato, VirtualHost, unità systemd, venv, utenti, token.
5. **App Android sul telefono**: compila, ma non è stata installata né provata su un telefono. **Attenzione**: la
   prima compilazione Android ha installato da sola CMake 3.22.1 nel tuo Android SDK (`~/Android/Sdk/cmake/3.22.1`,
   60 MB), che il modello di progetto di Flutter richiede; non te l'ho chiesto prima perché non me l'aspettavo. Se la
   togli, la prossima compilazione Android la reinstalla.
6. **iOS**: la cartella c'è e l'app usa solo librerie che funzionano su iPhone, ma si compila solo con un Mac.
7. **Da rivedere insieme**:
   - le modifiche ai task della sezione 4;
   - il cambiamento del guasto azzerato, che manda in pausa invece che in riposo;
   - le scelte della sezione 2;
   - l'aspetto dell'app rispetto al mockup.
8. **Punti aperti che ho trovato, fuori da questo lavoro:**
   - `tools/prove-nav/funzioni.sh` non compila già sul codice di prima (non collega `driver/aggancio.c`);
   - all'avvio dell'app web compare un errore nella console di Flutter. Viene da come Flutter cerca il primo elemento
     da mettere a fuoco quando la pagina riceve il focus prima che la prima schermata sia disposta. L'app funziona
     lo stesso; non ho trovato il modo di evitarlo dal codice dell'app.
