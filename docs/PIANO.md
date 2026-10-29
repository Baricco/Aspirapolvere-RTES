# Piano progetto RTES: task real-time in C sul Proscenic M6 Pro

Obiettivo: un nostro software C con task real-time, che gira sul SoC del robot, comanda direttamente **tutto**
l'hardware (non solo il minimo) e si controlla da un'app: avanti, indietro, rotazione, aspirazione, telemetria.
I task dell'esame li scrive l'utente da solo, sopra un framework e un driver preparati insieme.

## Decisioni prese
- **Niente Valetudo, niente flash.** Abbiamo già root (SSH/adb). Valetudo parla solo con il software originale.
- **Linux resta il sistema operativo** (kernel 3.4.39 `SMP PREEMPT`, non RT): niente cambio di OS né di kernel.
  Niente `SCHED_DEADLINE` (il kernel 3.4 non lo ha).
- **Ogni task RT** usa `mlockall()` e risvegli ad istanti assoluti (`clock_nanosleep(TIMER_ABSTIME)` o equivalente).
- **Sviluppo anche senza robot:** simulatore **a livello di seriali** (`sim/`, fase 3): finge di essere il GD32 e il lidar
  su due pseudo-seriali, quindi driver e task girano sul PC senza cambiare una riga (niente HAL separata). Stanza
  disegnabile da una pagina web. Più unit test e prove d'insieme. La prova finale si fa sempre sul robot.
- **Repository git** `Aspirapolvere-RTES`, privata su GitHub, senza GitHub Actions. `./rtes.sh test` prima di ogni
  commit.
- **Persistenza senza flash:** binari in `/mnt/UDISK/rt/`, script in `/etc/init.d/` (overlay ext4).
- **All'accensione, a lavoro finito, parte il nostro software**, con interruttore di sicurezza
  (`/mnt/UDISK/rt/enable`) e ritorno allo stack originale se il nostro non parte.
- **Gestione della batteria nel driver:** il driver espone tensione, temperatura, stato di carica e la percentuale
  stimata come il software originale (il GD32 non dà la corrente); le soglie di sicurezza (fermare i motori, LED 6)
  stanno nel driver; i task usano solo i valori.
- **Quando non gira un nostro programma** gira il custode (keepalive e heartbeat al GD32, motori a zero): il robot non
  si resetta e sulla base si carica. A progetto finito sul robot resta installato il nostro software.
- **Robustezza:** stato sicuro all'avvio (motori, spazzole, aspirazione a zero prima di tutto), watchdog dei task
  (se un task critico salta le scadenze o non risponde, tutti i motori si fermano) e **stato di guasto ben
  riconoscibile** (colore del LED + suono dall'altoparlante). Il rilancio automatico del programma si aggiunge in
  futuro.
- **Mappatura:** griglia di occupazione + allineamento delle scansioni lidar con un numero fisso di iterazioni,
  tentata come task **hard real-time** periodico (un giro del lidar); se non regge le scadenze, passa al server
  aperiodico.
- **Niente serbatoio dell'acqua:** pompa e sensori del serbatoio restano solo documentati.

- **Architettura RT (decisa il 2026-10-03, fine della fase 2):**
  - **driver** = livello fisso a **priorità massima, fuori dallo scheduling dei task**, su un core suo: seriale del
    GD32 (keepalive, heartbeat, avvio, decodifica), blocco di sicurezza (`gd32/sicurezza.h`), lidar con la sua
    regolazione di velocità (come AuxCtrl), soglie della batteria e **watchdog dei task** (deve stare sopra i task
    che sorveglia: con RM un watchdog a 50 ms non girerebbe se un task a 20 ms si blocca sullo stesso core);
  - **task con Rate Monotonic, partizionato** sui core (ogni task legato a un core; su ogni core vale l'analisi a un
    processore: Liu e Layland, tempi di risposta), direttamente con i thread `SCHED_FIFO` di Linux e la libreria
    `rt/`. Scartati: FreeRTOS POSIX (un solo core, scheduler simulato con i segnali, non più RT del Linux sotto),
    EDF sopra `SCHED_FIFO` (fattibile con un dispatcher, ma più complesso), RM globale (analisi debole, niente
    garanzie dei protocolli dei mutex fra core);
  - **mutua esclusione con Priority Inheritance** (`PTHREAD_PRIO_INHERIT`, verificato sul robot; `PRIO_PROTECT` non
    esiste in musl). Fra driver e task **nessun mutex**: scambio senza blocchi (doppio buffer con numero di versione),
    così il driver non aspetta mai un task;
  - task a 20 ms **svegliati dall'arrivo del frame** del GD32 (dati freschi, ogni frame una volta sola; nell'analisi
    sono sporadici con T = intervallo minimo), gli altri **a timer**;
  - **server aperiodico** per batteria (lettura), spazzole, aspirazione, LED, comandi da remoto;
  - rete di sicurezza del kernel sui task RT (95% della CPU per secondo) lasciata attiva.
- **Lidar a 6 giri/s** (come AuxCtrl, ~340 punti per giro), deciso il 2026-10-06 all'inizio della fase 4: il task
  del lidar elabora un giro ogni 200 ms.
- **Navigazione ogni 100 ms** (2026-10-11): accorcia la catena sicurezza → navigazione → moto (circa 0,3 s nel caso
  peggiore, compresa la frenata, invece di 0,4).
- **Sicurezza del moto** (2026-10-11): sporadica sul frame, come il moto. Riconosce incastro (velocità mandata contro
  misurata), slittamento (ω degli encoder contro IMU), sforzo dei motori (correnti delle ruote), ostacolo vicino
  (settori del lidar) e bordo delle zone proibite. **Non comanda i motori e non parla con il moto**: pubblica
  l'allarme, la navigazione lo legge e cambia l'obiettivo; il moto non vede la sicurezza. Per ostacoli e zone: sotto
  una soglia di "vicino" la velocità massima scende in proporzione alla distanza, e così si ottiene anche lo stop;
  il bumper del driver resta la protezione in caso di errori. L'inclinazione da sola non è un allarme (salire su un
  tappeto è normale; i casi pericolosi finiscono in incastro o slittamento).

- **Navigazione e mappatura** (2026-10-13):
  - **tre strutture**: la **griglia fine** (occupazione, 5 cm) è privata della mappatura; la **griglia di navigazione**
    (celle larghe poco meno della striscia di pulizia: libera / bloccata / proibita / sconosciuta, con gli ostacoli
    già ingrossati del raggio del robot) è **condivisa con un mutex con Priority Inheritance**, la scrive solo la
    mappatura e la legge la navigazione; la **copertura** (una maschera di bit, una per cella di navigazione) è
    privata della navigazione;
  - la mappatura aggiorna la griglia di navigazione **a ogni job**, insieme alla griglia fine, non solo al
    salvataggio: un ostacolo imprevisto visto dal lidar entra nella mappa prima che scatti la sicurezza, e la
    navigazione prende la mappa nuova (serve un segnale "mappa cambiata" dalla mappatura alla navigazione);
  - **sezioni critiche brevi**: la navigazione tiene il mutex solo per copiare la griglia di navigazione, poi lavora
    sulla copia; la mappatura solo per scrivere le celle della griglia di navigazione;
  - **due fasi**: senza una mappa salvata il robot prima la costruisce con un giro di esplorazione (verso le
    frontiere fra libero e sconosciuto), poi pulisce con **BA\*** (strisce + ritorno alla cella non pulita più vicina)
    sulla mappa. Con una mappa salvata la pulizia parte dalla base, la cui posizione è l'origine della mappa; dopo
    un'esplorazione parte da dove l'esplorazione finisce (vedi la mappa salvata, 2026-10-28);
  - **la base è una zona proibita permanente** (nella lista delle zone, non come celle occupate, così la mappatura
    non la cancella quando il lidar vede libero): il robot non deve urtarla né trascinarla. Si parte uscendo in
    avanti fino a un punto davanti alla base. **L'aggancio non lo fanno i task RT** (niente eccezioni nella
    sicurezza): la navigazione porta il robot al bordo della zona, poi chiede al driver (o a un altro componente
    fuori dai task) di fare la messa in carica con l'infrarosso della base; durante l'aggancio il driver ignora le
    velocità mandate dai task e tiene attive le sue protezioni (bumper, anti-caduta). Da progettare con il driver;
  - **dimensioni**: griglia fine 1024 × 1024 celle da 5 cm, griglia di navigazione 256 × 256 celle da 20 cm:
    51,2 m di lato, ±25,6 m dalla base. Le potenze di due fanno stare le coordinate di una cella di navigazione in un
    `uint8_t` e il suo indice in un `uint16_t`. Le celle restano da 20 cm anche per la pulizia (2026-10-28): le
    strisce da 10 cm si fanno con le mezze celle, senza cambiare la mappa (vedi la pulizia con BA\*);
  - **segnale "mappa cambiata"**: numero di versione atomico (`stdatomic.h`) nella mappa condivisa; la mappatura lo
    incrementa (dentro il mutex) quando una cella di navigazione cambia stato, la navigazione lo legge a ogni job
    senza mutex e copia la griglia solo se è cambiato, salvando la versione letta dentro il mutex;
  - **spazzola durante lo sblocco** (deciso 2026-10-27, per la pulizia): ferma la spazzola centrale (e
    l'aspirazione) per tutto lo sblocco, così un cavo sotto il robot non si avvolge mentre indietreggia; se il motore
    della spazzola va in sovraccarico, prima di tutto il suo sblocco (`drv_spazzole` con la centrale < 0: gira al
    contrario per ~4 s e la ferma il GD32);
  - **trasferimenti e ritorno alla base** cercando il percorso sulle celle già pulite (terreno già percorso), con
    ripiego su tutte le celle libere: A* quando la meta è nota (la base), ricerca in ampiezza quando la meta è "la
    cella non pulita più vicina". Per la cella non pulita il ripiego non serve: la ricerca in ampiezza si ferma alla
    prima cella sporca, quindi tutte quelle attraversate prima erano già pulite.
  - **pulizia con BA\*** (progettata 2026-10-28, codice in `navigation.c`):
    - **strisce da 10 cm con le mezze celle**: per BA\* ogni cella di navigazione conta come due mezze celle
      affiancate, di traverso rispetto alle strisce, ognuna con la sua **corsia**. La corsia sta a ±5 cm dal centro
      della cella solo se la cella vicina da quel lato è libera, altrimenti al centro. Con le celle della striscia e
      le vicine da quel lato libere, fra le due colonne non ci possono essere ostacoli (i cerchi da 20 cm attorno ai
      centri coprono la fascia), e dall'altra parte stanno ad almeno 17,3 cm dalla linea dei centri: lungo la corsia
      il corpo ha almeno 4,9 cm di margine (lungo un muro, al centro, 2,5 cm). Punto peggiore: in fondo alla striscia
      con un oggetto piccolo di sbieco, 0,6 cm; la sicurezza ferma il robot 1 cm prima, dentro la tolleranza di 2 cm
      del moto. Con lo scostamento della corsia a 0 le due corsie coincidono e si torna alle strisce da 20 cm (una
      costante sola). Scartata l'"andata e ritorno" con 1 bit per cella: la navigazione dovrebbe ricordarsi quale
      passata sta facendo, e un'interruzione durante il ritorno lascerebbe corsie mancanti su celle segnate pulite.
      Le strisce raddoppiano rispetto ai 20 cm (l'originale usa 0,15 m e 0,10 m: `Set bow shape coverage distance`
      in `everest-server`);
    - **asse delle strisce** (deciso 2026-10-28/29): x o y della mappa, **calcolato all'inizio della pulizia dopo
      un'esplorazione**, nel primo job della pulizia, da solo: per ogni cella libera si guarda se la precedente lungo x
      (e lungo y) è libera, e se non lo è lì comincia una striscia; vince la direzione con meno strisce, cioè meno
      curve (a pari numero lungo y). Il conteggio **non gira su tutta la mappa ma sulla lista delle celle raggiungibili
      lasciata nella coda di ricerca dall'ultima ricerca dell'esplorazione** (quella che non trova frontiere e quindi le
      visita tutte; fra lei e il primo job della pulizia nessuno fa altre ricerche): costa in proporzione alla casa
      (soggiorno del simulatore: 337 celle) invece che alle 65.536 della mappa (giro completo misurato: 63–96 µs sul
      PC, ~1–1,5 ms sul robot), e ignora le celle libere che il robot non può raggiungere. Con una mappa caricata dal
      file la lista non c'è, ma lì l'asse arriverà dal file. Con un asse qualsiasi la superficie pulita sarebbe la
      stessa, ma con più curve (stanza 5 × 3 m a strisce da 10 cm: 30 contro 50, circa un minuto in più su nove);
    - **asse per stanza** (più avanti, con il file della mappa): divisione in stanze (porte) e asse per stanza
      calcolati fuori dai task, nel server aperiodico al salvataggio, e salvati nel file; a tempo di esecuzione l'asse
      di una cella si legge a costo costante. Compatibile con le mezze celle: dentro una stanza tutte le celle hanno lo
      stesso asse. Lo fanno anche i robot commerciali (stanza per stanza) e il firmware originale (`CCandidateDoorExtraction`,
      `room_id`, `split_line_num`, zig-zag per `CLocalSection`);
    - **copertura**: maschera privata con 2 bit per cella di navigazione (16 KB), uno per metà, in un array piatto di
      bit con le due metà di una cella affiancate (bit numero 2 × cella + metà, quindi le due metà stanno nello stesso
      byte; l'indice arriva a 131.071 e va in un `uint32_t`). Metà 0 = verso la coordinata più bassa dell'asse
      laterale (corsia a −5 cm), metà 1 = verso la più alta (+5 cm). A ogni job, con
      spazzole e aspirazione accese, si segna pulita ogni metà la cui corsia dista meno di 2,5 cm (di lato) dal centro
      del robot: più della tolleranza del moto, così il robot fermo sulla corsia la segna sempre;
    - **lo stato di pulizia**, a ogni ingresso: se la cella del robot non è libera va alla cella libera più vicina
      (come l'esplorazione); altrimenti la prossima striscia parte dalla metà del robot se è sporca, se no dalla prima
      mezza cella vicina libera e sporca nell'ordine nord, sud, est, ovest (nord/sud = lungo le strisce, nella stessa
      metà; est/ovest = l'altra metà della stessa cella o la metà più vicina della cella accanto), e si allunga lungo
      le strisce finché la metà successiva è libera, sporca e con la corsia allo stesso scostamento. Vicolo cieco:
      ricerca della cella con almeno una metà sporca (`GOAL_DIRTY`). Niente da pulire: riposo (poi seguimuro e
      ritorno alla base);
    - **"segui percorso"** riceve uno scostamento laterale sommato a ogni punto (zero fuori dalle strisce, che sono
      dritte) e, **solo per le strisce**, se il robot è a più di 2 cm dal primo punto ci va prima: così si entrano le
      corsie e si fanno i passi laterali di 10 cm. Negli altri percorsi il primo punto è la cella del robot, e se è
      bloccata andarne al centro vorrebbe dire avvicinarsi all'ostacolo: l'esplorazione quindi non cambia (da
      verificare con la serie delle 17 prove). Tutti i percorsi partono da una funzione sola, che riceve scostamento e
      "dal primo punto";
    - **asse e segno**: l'asse si calcola nel primo job della pulizia; finché non c'è, "pulizia attiva" è falsa, così
      il segno delle metà non usa l'asse provvisorio;
    - **spazzole e aspirazione** (deciso 2026-10-28): le comanda la navigazione direttamente (`drv_spazzole`,
      `drv_aspirazione`), accese nella pulizia e spente in sblocco, attesa, guasto e riposo; il driver si chiama solo
      quando lo stato cambia. Anche la **spazzola laterale** è da considerare montata (la monterà l'utente);
    - **terminazione**: ogni passaggio dallo stato di pulizia pulisce almeno una metà nuova, oppure cambia la mappa
      (cella chiusa, contatto), oppure consuma un tentativo di sblocco (10 di fila → guasto). Buco trovato nelle
      prove (2026-10-29): l'incastro **senza contatto** non lo segnava nessuno, e i tentativi di sblocco si azzerano a
      ogni percorso finito, quindi il robot ci riprovava all'infinito (83 sblocchi sotto la credenza del soggiorno).
      **Fatto** (codice dell'utente): entrando nello sblocco per incastro, slittamento o sforzo **senza** contatto la
      navigazione chiude con `close_cell` la cella del robot (il contatto lo segna già la mappatura). Scartata la
      variante "la mappatura segna un ostacolo davanti al robot": la torretta sta 11 cm dietro il centro, quindi lo
      spigolo della credenza è sotto il robot, non davanti. Funziona **solo con la posizione giusta** (vedi le prove
      sotto);
    - **prove nel simulatore** (2026-10-29, `prova.py --pulizia`, `tools/prove-nav/disegna.py`): BA\* funziona.
      Strisce lungo il lato lungo, a coppie a 10 cm, attorno a pianta, muretto e gambe del tavolo; per 450 s
      posizione creduta e vera coincidono entro 2 cm; nessuna scadenza mancata. Sotto la **credenza** (rialzata da
      9 cm: il lidar a 8,7 cm la vede libera, la torretta a 9,5 cm ci sbatte) il robot si incastra, e mentre è fermo
      le ruote slittano (nel simulatore gli encoder contano al 30% della velocità comandata, `slitta = 0.3` in
      `sim/gd32sim.c`, come le ruote vere): l'odometria avanza, ogni sblocco aggiunge ~10 cm di errore, la cella
      chiusa è quella sbagliata e il robot ci riprova. Dopo 36 sblocchi crede di essere 2,85 m più in là, fuori dalla
      stanza; la mappa (lidar inserito con la posa sbagliata) e la maschera della copertura si spostano, e il robot
      ripulisce zone già pulite. Cucina e camera non hanno mobili a quell'altezza;
    - **PROSSIMO PASSO (deciso 2026-10-29): l'allineamento delle scansioni** nella mappatura, che corregge la
      posizione dell'odometria con il lidar (e dà un rilevatore di incastro: encoder che dicono "mi muovo" e scena
      ferma). Con la posizione giusta le celle chiuse dopo gli incastri sono quelle giuste e il bordo della credenza
      si chiude dopo pochi tentativi. Va progettato prima di scrivere il codice, come BA\*;
    - **tempi**: una striscia legge al massimo 255 celle; la ricerca della cella sporca nel caso peggiore visita tutte
      le celle raggiungibili come quella della frontiera, e parte solo nei vicoli ciechi;
    - **ancora da fare nella pulizia**: spazzole e aspirazione (passo 4), striscia accorciata quando la mappa
      cambia, sblocco delle spazzole in sovraccarico, ritorno alla base a fine pulizia (oggi lo stato
      `NAV_STATE_RETURN_HOME` è vuoto e il robot si ferma lì).
  - **mappa salvata** (deciso 2026-10-28, da fare dopo BA\* insieme al server aperiodico): all'**avvio**, prima che
    partano i task, se il robot è sulla base e il file è valido (numero magico, versione, dimensioni, controllo) la
    mappa si carica e la missione iniziale è la pulizia (dopo l'uscita dalla base); altrimenti mappa vuota ed
    esplorazione, e la pulizia parte da dove finisce l'esplorazione. **Salvataggio** dopo l'esplorazione e alla fine
    della pulizia, in due metà: la mappatura copia nel suo job la sua parte (griglia fine, griglia di navigazione,
    contatori, zone proibite: ~1,2 MB, tempo limitato, una volta sola) e sveglia il **server aperiodico**, che scrive
    la copia su un file temporaneo e lo rinomina sopra il vecchio. Nessun task aspetta la scrittura (la flash può
    bloccare per secondi: il budget del server limita la CPU, non la durata, e il salvataggio non ha scadenze). Una
    mappa ha l'origine nella base: **si salva solo se la sessione è partita dalla base**. Precisione: due agganci non
    mettono il robot nella stessa posa (3° a 5 m sono 26 cm), quindi la mappa caricata è affidabile solo con
    l'allineamento delle scansioni.
  - **mappatura, prima versione scritta** (2026-10-17, `tasks/vacuum/mapping.c`, senza allineamento delle scansioni):
    periodo 200 ms sul core 2 (ci andrà anche la navigazione), watchdog non critico. Punteggio per cella fine in
    `int8_t` (colpo +30, raggio che attraversa −10, limiti ±100), **niente isteresi**: occupata se > 50. Raggi con
    Bresenham, tagliati a 6 m senza colpo (oltre il rumore del lidar triplica, `docs/hardware.md`); punti senza eco
    scartati (possono essere ostacoli che assorbono la luce). Due contatori per cella di navigazione aggiornati solo
    quando una cella fine cambia categoria: osservate (celle fini dentro la cella con punteggio ≠ 0) e occupate
    (celle fini occupate entro 20 cm dal centro: ingrossamento preciso). Pubblicazione a ogni job del quadrato ±31
    celle attorno al robot, sotto mutex, con versione incrementata solo se qualcosa cambia.
  - **correzione del giro** (2026-10-18, fatta): ogni punto del lidar si mette in mappa con la posa del robot
    nel suo istante (tempi del giro `t_inizio_ns`/`t_fine_ns`), ricavata tornando indietro dalla posa del moto con `v`
    e `omega` (velocità costante durante il giro), anche come partenza del raggio. Rete di sicurezza: giro scartato se
    |omega| > 1,5 rad/s. Visto con la griglia sovrapposta nel simulatore: senza correzione una rotazione a 1 rad/s
    sposta i muri lontani fino a ~1 m;
  - **ostacoli che il lidar non vede** (2026-10-18): tutta la memoria degli ostacoli sta nella **mappatura** (la
    navigazione naviga e basta). Il **driver** tiene un registro dei contatti (`driver/contatti.h`, `drv_contatti()`,
    FATTO): gli ultimi 32 scatti di anti-caduta (dopo 3 frame di fila, contro i falsi allarmi dei tappeti scuri) e di
    bumper, con numero progressivo, sensore e istante; niente eventi con sensori spenti, sulla base o con tutti e
    quattro gli anti-caduta nel vuoto (sollevato). Ogni evento porta la **posizione del robot** che il moto dà al driver
    a ogni frame con `drv_posa()` (quella del primo frame attivo, 20 ms prima al massimo: la ricostruzione con `v` e
    `omega` sbagliava proprio quando il robot reagisce all'evento). La **mappatura** (fatto) applica gli eventi
    nuovi: posizione del sensore (anti-caduta a 15 cm dal centro a
    +50/+18/-18/-50 gradi; bumper 2 cm oltre il bordo a +-45 gradi, davanti se premuti tutti e due: da verificare sul
    robot), cella fine con un valore speciale che il lidar non può più cambiare (-128 vuoto, 127 ostacolo basso) e
    contatori come per una cella diventata occupata. Facoltativo: l'incastro dalla sicurezza (`read_safety`), che resta
    attivo abbastanza a lungo da non perderlo. I segni valgono per la sessione;
  - **celle di navigazione da 20 cm** (2026-10-19): lungo i muri il corpo del robot resta fra 2,5 e 22,5 cm dal
    muro (dipende da dove cade il muro rispetto alla griglia). Si lasciano così: i bordi li pulisce un **seguimuro**
    guidato dalla distanza del lidar, da aggiungere alla navigazione dopo BA*;
  - **rumore della mappa** (2026-10-18): la versione cresce anche con il robot fermo (celle di bordo che cambiano
    stato); si accetta per ora, da misurare e tarare dopo la correzione del giro.
  - **frontiere che si chiudono sempre** (2026-10-21): l'esplorazione non tiene liste di frontiere scartate (sarebbe
    memoria degli ostacoli nella navigazione). Ogni tentativo di entrare nello sconosciuto cambia comunque la mappa,
    quindi i tentativi finiscono perché le celle sono finite:
    - **mappatura, celle sotto il robot**: a ogni job con la posizione valida le celle fini sotto il corpo del robot
      (raggio 17,45 cm) diventano libere e osservate con il punteggio **normale** più libero, che il lidar può ancora
      cambiare (es. una sedia spostata dopo). Le celle speciali (vuoto dell'anti-caduta, ostacolo del bumper) non si
      toccano. Serve perché i raggi senza ritorno vengono scartati: davanti a un oggetto nero o a un vetro le celle
      possono restare sconosciute anche con il robot accanto;
    - **incastro come contatto** (2026-10-21): l'allarme di incastro della sicurezza vale frame per frame e si spegne
      appena la navigazione reagisce (~100 ms), quindi la mappatura (200 ms) lo perderebbe. Come per il bumper, passa
      dal **registro dei contatti** del driver: la sicurezza chiama `drv_incastro(si)` a ogni frame (1 = allarme di
      incastro con la velocità comandata in avanti), il driver ne fa un evento `CONTATTO_INCASTRO` con la posizione
      del primo frame (niente eventi sulla base, a sensori spenti o sollevato), la mappatura lo applica come
      `LOW_OBSTACLE` **davanti al robot**, alla distanza del bumper (es. la credenza rialzata del soggiorno, che
      incastra la torretta senza bumper). Scartate: un contatore nella sicurezza (rifà il registro, posizione in
      ritardo), l'allarme trattenuto (lega la sicurezza al periodo della mappatura), un canale dalla navigazione
      alla mappatura. Da rivedere: la gestione degli incastri è la parte più difficile da prevedere;
    - **navigazione, ultimo passo nello sconosciuto**: la ricerca trova la frontiera, poi in fondo al percorso si
      aggiunge una cella sconosciuta vicina, che è la meta vera. Il passo verso una cella non libera si fa a velocità
      ridotta: la velocità normale (crociera, limitata dalla sicurezza) moltiplicata per una costante fra 0 e 1
      (0,5 per ora, da tarare sul robot vero). L'urto con un oggetto che il lidar non vede arriva senza la frenata
      della sicurezza;
    - come finisce il passo: arrivato (celle sotto il robot: la cella diventa libera), urto (bumper), gradino
      (anti-caduta), incastro senza bumper (segno dell'incastro), ostacolo visto (lidar): negli ultimi quattro casi la
      cella diventa bloccata, e servono il ricalcolo del percorso quando la mappa cambia e lo sblocco;
    - **unica rete di sicurezza**: se la mappa non cambia (es. slittamento sul bordo di un tappeto), dopo 10 sblocchi
      di fila senza finire un percorso il robot va nello stato di guasto e si ferma (`UNLOCK_MAX_ATTEMPTS`: tanti,
      per dargli molte occasioni di uscire prima di arrendersi).
  - **percorsi di striscio** (2026-10-21, visto nel simulatore: il robot fermo a 1,7 cm dalla pianta del soggiorno):
    una cella di navigazione è libera se il suo **centro** è lontano dagli ostacoli, ma il robot passa anche fuori
    dai centri. Spazio minimo fra corpo e ostacolo nel caso peggiore (ostacolo sull'angolo della sua cella fine,
    ingrossamento di 20 cm fra i centri): +0,58 cm al centro e lungo un passo di centro in centro seguito alla
    perfezione; −1,05 cm con il robot spostato di lato di 2 cm (tolleranza del moto, archi); −7,45 cm su una
    scorciatoia che passa sul bordo di una cella, −12,45 cm sull'angolo. La sicurezza ferma il robot sotto il margine,
    e di solito ha ragione: con pochi cm di spazio, ogni ostacolo entro ±45° è sulla traiettoria. Deciso:
    - **margine della sicurezza a 1 cm** (`STOP_DISTANCE`, era 2): la frenata la fa già la legge proporzionale (con
      ~0,3 s di ritardo della catena e guadagno 1/s non va oltre il margine); il margine copre il rumore del lidar
      (7 mm fino a 3 m). Da tarare sul robot vero (forma del guscio, tocchi che danno segni permanenti del bumper);
    - **riallineamento al centro**, meccanismo a parte (uno stato di servizio, come lo sblocco): quando la sicurezza
      ferma il robot (velocità massima in avanti ~0 con l'obiettivo non raggiunto, per un tempo da stabilire) va al
      centro della sua cella con un `OBJ_MOVE` (girare sul posto è sempre sicuro: il robot è rotondo), poi prosegue
      **di centro in centro**, senza scorciatoie, nel tratto stretto. I punti di svolta sono già centri di cella;
    - **cella chiusa** se il riallineamento fallisce (si riferma anche di centro in centro): la cella verso cui
      andava diventa chiusa nella **mappatura** (la memoria degli ostacoli resta lì), così la ricerca trova
      un'altra strada invece di riprovare fino al guasto:
      - **canale** dalla navigazione (unico scrittore) alla mappatura: doppio buffer (`scambio`) con una piccola
        lista circolare di celle chiuse e un numero progressivo, come il registro dei contatti; la mappatura applica
        solo quelle con numero più alto dell'ultima vista, quindi non ne perde nessuna. Diretto e non dal driver
        (come l'incastro) perché l'informazione nasce nella navigazione ed è già una cella;
      - la navigazione chiude la cella **subito anche nella sua copia**; dato che la copia viene riscritta a ogni
        cambio di versione, dopo ogni copia rimette le chiusure della sua lista che la mappa non mostra ancora;
      - **un nuovo stato della griglia di navigazione**, `NAV_CELL_TOO_NARROW` (c'è posto: 4 stati usano 2 bit su
        8), non un valore speciale nella griglia fine: quello passerebbe per l'ingrossamento di 20 cm e chiuderebbe
        anche le 4 celle vicine (5 invece di 1, contro il requisito di passare ovunque si possa). Come per le celle
        proibite, la mappatura lo salta quando ricalcola gli stati: dura **per tutta la sessione**, il lidar non lo
        riapre. Per la navigazione non è libera e non si attraversa nemmeno uscendo da una zona bloccata (si
        richiudeva più volte la stessa cella);
      - la cella chiusa è quella verso cui il robot andava; dopo la chiusura il robot si riallinea al centro della
        sua cella (o, se non è libera, ricalcola subito il percorso);
    - **fatto** (2026-10-22, codice dell'utente): margine 1 cm, riallineamento (`NAV_STATE_ALIGN`: fermo per 1 s con
      velocità massima in avanti < 2 cm/s e senza ruotare), cella chiusa (`NAV_CELL_TOO_NARROW` = 4, lista di 16
      posti). Insieme:
      - la sicurezza non usa più il cono di ±45° ma la **striscia** larga quanto il robot (strada percorribile dritto
        prima di toccare un punto: `x − sqrt(R² − y²)`): un muro parallelo non frena più;
      - `MOVE_GAIN` 1 → 3 (rallentava a 35 cm da ogni punto di svolta) e `ALIGNMENT_THRESHOLD` 30° → 10° (le curve
        strette portavano gli ostacoli nella striscia di lato, troppo tardi per frenare);
      - se ci si ferma in una cella che la mappa nel frattempo ha bloccato si **ricalcola il percorso** (versione
        minima del "la mappa cambia sotto il percorso"); se l'esplorazione parte da una cella non libera va prima
        alla **cella libera più vicina** (`GOAL_FREE`), poi cerca la frontiera: prima l'uscita dalle zone bloccate
        faceva attraversare l'ostacolo al percorso (nella camera: fermo e riparto contro la cassettiera all'infinito);
      - prove nel simulatore (17: soggiorno con 5 semi e 4 partenze, cucina, corridoio con porta da 60 cm, camera):
        16 esplorazioni complete, 0 deadline mancate. Il guasto rimasto è un urto di lato contro una gamba sottile
        del tavolo: serve lo **sblocco** (`NAV_STATE_UNLOCK`). La mappatura fatica con gli ostacoli sottili (≤ 5 cm:
        i raggi diretti agli altri punti dello stesso oggetto tolgono punteggio); deciso di non cambiare
        `LIDAR_PASS_SCORE` perché a casa gli ostacoli sono più grossi: da verificare sul robot;
    - **sblocco** (deciso 2026-10-23, `NAV_STATE_UNLOCK`): si entra con incastro, slittamento o motore sotto sforzo,
      controllati una volta prima dello `switch` (non in attesa, guasto, riposo e sblocco: lo slittamento scatta anche
      da fermo se qualcuno gira il robot). Ogni tentativo (10, `UNLOCK_MAX_ATTEMPTS`) viene da una **tabella fissa**
      (indietro di quanto, rotazione sul posto di quanto e da che parte, velocità), diversa a ogni riga e ripetibile
      nelle prove. Si indietreggia **sulla scia** (8 posizioni a 5 cm, dove il robot è già passato: la mappa a 20 cm
      con l'ingrossamento non dice niente da così vicino), con `OBJ_MOVE` a `v` negativa (il moto punta il retro).
      **Spinta** per cavi e bordi dei tappeti (avanti a 0,3 m/s per 10 cm al posto dei tentativi): progettata e poi
      **scartata** (2026-10-27). Un cavo tirato si avvolge nella spazzola o nelle ruote o trascina quello a cui è
      attaccato; per i sensori un cavo, il bordo di un tappeto e la torretta incastrata sotto un mobile basso (la
      credenza: il lidar a 8,7 cm non la vede, la torretta non ha interruttori) sono la stessa cosa, e l'unico indizio,
      l'inclinazione dall'IMU, non è misurato; il robot supera già da solo soglie fino a ~2 cm; nel simulatore non si
      potrebbe provare davvero. Senza spinta il robot indietreggia e la mappatura segna il punto, così i percorsi dopo
      lo evitano (prezzo: una zona chiusa da un tappeto alto non viene pulita). Da riprendere solo se sul robot vero si
      incastra spesso sui tappeti, misurando prima l'inclinazione su cavo, tappeto e sotto un mobile. Se un giorno
      servisse togliere dalla mappa un ostacolo segnato per sbaglio: lista delle celle chiuse con segno (aggiungi /
      togli), con `int32_t` perché gli indici della griglia arrivano a 65.535, e il valore speciale va tolto dalla
      griglia fine;
      **fatto** (2026-10-27, codice dell'utente) senza la spinta: scia, tabella, `OBJ_MOVE` a `v` negativa (`signbit`,
      così anche −0 vuol dire indietro), fasi indietro / rotazione con timeout di 5 s (una rotazione di 180° ne dura
      ~3,5). Dalle prove: l'incastro della sicurezza non scattava mai negli urti, perché confronta la velocità che il
      driver manda **dopo** i suoi limiti (con il bumper premuto il driver la azzera); aggiunto l'allarme di
      **contatto** (`SAFETY_ERROR_CONTACT`: `limiti` del driver con `SIC_LIM_BUMPER` o `SIC_LIM_CLIFF`), che porta allo
      sblocco. Anche lo stallo nel riallineamento porta allo sblocco invece che al guasto (la garanzia del centro della
      cella libera vale solo per gli ostacoli che la mappa conosce). Serie di prove (`tools/prove-nav/serie.sh`):
      17 esplorazioni complete su 17, 0 deadline mancate; il robot che urtava 60 volte la fine del muro accanto alla
      porta da 60 cm ora fa un solo sblocco;
    - errori che i conti non vedono: rumore del lidar e, **per ora**, mappa spostata dall'odometria. Con
      l'allineamento delle scansioni (già nel piano, vedi la mappatura in cima) lo spostamento scende alla precisione
      dell'allineamento; la posizione corretta dovrà arrivare anche al moto, che oggi guida con l'odometria;
    - **resta aperto** il caso opposto: passaggi larghi fra ~36 e ~60 cm che la regola "libera" chiude se nessun
      centro di cella cade nella striscia in cui ci sta il centro del robot.

## Ancora da decidere
- **Mappa e navigazione, punti aperti** (2026-10-13): ingombro della base, aggancio alla base nel driver (passaggio
  di consegne dai task al driver e ritorno). La larghezza della striscia è decisa (10 cm con le mezze celle,
  2026-10-28), e anche l'asse delle strisce (calcolato a ogni pulizia, vedi la pulizia con BA\*).
- **Tipo di server aperiodico** (polling, deferrable, sporadic) e **tipo di app** (con l'home server, più avanti:
  zone proibite, preferenze, programmazione della pulizia).
- **Assegnazione dei task ai core**: rimandata (2026-10-06) a quando ci saranno i WCET misurati sul robot; il criterio
  (per esempio Worst-Fit sulle utilizzazioni, per bilanciare) si sceglie allora. Già deciso (2026-10-13):
  **navigazione e mappatura sullo stesso core**, perché condividono la mappa con il mutex. Sullo stesso core
  l'ereditarietà di priorità vale come nella teoria a un processore, e nessuno dei due aspetta un task che gira su un
  altro core.
- **Scambio dei dati fra i task.** Proposta (2026-10-06, da confermare): doppio buffer con numero di versione
  (`driver/scambio.h`, lo stesso del driver) per ogni dato pubblicato da un solo task (posizione, allarmi, ostacoli,
  obiettivo, posizione corretta): lo scrittore non aspetta mai e non c'è tempo di blocco nell'analisi. Mutex con
  Priority Inheritance solo dove il doppio buffer non va bene: dati grandi (la griglia della mappa, a meno che la
  tocchi solo la mappatura e pubblichi un estratto piccolo) o dati con più scrittori.

## Da fare dopo i task (fase 4)
- **Guasto chiesto da un task** (2026-10-10): oggi un task non può mandare il driver in guasto (`entra_in_guasto` è
  `static` e va chiamata solo dal thread del driver). Aggiungere `drv_segnala_guasto(const char *motivo)` con lo
  stesso schema di `drv_azzera_guasto`: campo atomico in `cmd` (come `cmd.suono`), letto e azzerato dal driver con
  `atomic_exchange` vicino a `prova_ad_azzerare_guasto`, poi `entra_in_guasto(motivo)`; azzerarlo anche all'avvio.
  Primo uso: il `default` dello `switch` del moto (tipo di obiettivo sconosciuto), che intanto fa solo stop.
- **Grande refactoring dopo la fase 4** (richiesta dell'utente, 2026-10-11), insieme alle modifiche al driver:
  - **nomi che dicono cosa c'è dentro.** Il caso peggiore: `gd32_sensori_t.vel_sx` / `vel_dx` sembrano le velocità
    delle ruote sinistra e destra, ma sono v e ω del robot (misurate dal GD32 sugli encoder). Ha già confuso il codice
    dei task (scale scambiate in `tasks/vacuum/safety.c`). Da cercare anche gli altri nomi storici o ambigui;
  - **funzioni, tipi e variabili in inglese** (driver, framework `rt/`, GD32, simulatore), come quelli scritti
    dall'utente nei task: nomi semplici e facili da capire;
  - **prima di iniziare, decidere insieme lo stile** (prefissi dei moduli, maiuscole e minuscole, nomi delle strutture
    e dei `#define`, lingua dei commenti), poi applicarlo a tutto il codice.

## Architettura del software originale (dettagli in `RIEPILOGO.md`)
```
GD32 ⇄ /dev/ttyS3 ⇄ AuxCtrl ⇄ /tmp/AuxEverest_Sock ⇄ everest-server ⇄ /tmp/everest_sock ⇄ RobotApp ⇄ cloud
lidar → /dev/ttyS1 → AuxCtrl
```
- **AuxCtrl**: driver delle periferiche (seriali, keepalive `0x0d`, lidar, audio).
- **everest-server**: il cervello (stati, movimento, SLAM, eccezioni, protocollo del GD32). Esporta ~10.500 nomi di
  funzione: è la fonte principale per completare il protocollo.
- **RobotApp**: app, cloud, mappe, sospensione, suoni.
- Il watchdog del GD32 sorveglia solo `0x0d`: senza, reset del robot dopo ~190 s.
- Il nostro software deve replicare AuxCtrl e la parte di everest-server che parla con il GD32.

## Fasi
0. **Toolchain e valori di riferimento RT** — FATTA. Latenza max 38 µs in 60 s; 5 h di misure lunghe con 0 deadline
   mancate (max 455,5 µs nel caso peggiore). `docs/fase0.md`, `docs/misure-lunghe.md`.
1. **Reverse dei protocolli e accesso comodo** — FATTA. Spia sulle seriali e sui socket, frame dei sensori mappato
   con prove fisiche, canale AuxCtrl ↔ everest-server, watchdog. `docs/fase1.md`, `docs/RIEPILOGO.md`.
2. **Caratterizzazione completa dell'hardware** — FATTA (2026-10-03). Disassemblato di everest-server (tutti i
   comandi, ogni byte del frame dei sensori), banco di prova con blocco di sicurezza (`spia guida`,
   `gd32/sicurezza.h`), prove su lidar, ruote, spazzole, aspirazione, carica, LED, audio, IMU, anti-caduta, IR della
   base. Aperti: carica ferma a 16,3 V (batteria degradata?), curva di scarica, scarto di ~8° del lidar (risolto il 5/10: 14,5°, angoli orari, vedi fase3.md).
   `docs/fase2.md`, `docs/RIEPILOGO.md`.
3. **Driver, framework RT e simulatore** — FATTA (4-6/10, `docs/fase3.md`). Driver (`driver/`: avvio,
   keepalive, heartbeat, sensori, lidar regolato, batteria, limiti di sicurezza "a terra", watchdog dei task, stato di
   guasto con LED 8 e suono), framework (`rt/`: task sporadici, watchdog, priorità RM), simulatore con pagina web,
   guida (`GUIDA.md` sezioni 11–13). Provato sul robot **senza motori** (0 scadenze mancate, ritardo del ciclo
   < 160 µs, poi < 110 µs con il driver a due thread). Il 5/10 il simulatore è diventato fedele: geometria vera, dinamica
   tarata sulle registrazioni e validata contro di esse, stanza a oggetti con editor, ispettore dei messaggi. Il 5-6/10
   prove con il robot a terra: corretti il verso del lidar e le scale di velocità (~3,6 volte), aspirazione con blocco
   senza cassetto e curve della batteria della ventola, IR della base, registrazione grezza nel driver, simulatore
   validato anche a terra. Aperti per le fasi dopo: aggancio alla base con l'IR, gradino.
4. **Task RT** (utente, da solo) — sicurezza, controllo del moto, lidar, comportamento, watchdog dei task, mappatura
   in hard RT; WCET, jitter, analisi di schedulabilità.
5. **App e server aperiodico** (insieme) — scelta del tipo di app e del server; comandi e telemetria.
6. **Installazione persistente e relazione** — avvio automatico del nostro software con ritorno all'originale,
   eventuale rilancio automatico, SSH solo con chiave, documentazione finale.

Fasi trasversali: simulatore (a livello di protocollo, con i dati delle catture) e test sul PC.

## Bozza dei task (da rivedere nella fase 4)
Driver (fisso, priorità massima, fuori da RM): GD32 ogni 20 ms (keepalive 3 s, heartbeat 200 ms), blocco di
sicurezza (bumper, anti-caduta, sensori assenti), lidar e regolazione della sua velocità (20 ms), soglie della
batteria, watchdog dei task + stato di guasto (~50 ms).

| Task (RM partizionato) | Attivazione | Periodo | Cosa fa |
|---|---|---|---|
| Controllo del moto | frame del GD32 | 20 ms | odometria (encoder + imbardata dell'IMU) e v/ω verso l'obiettivo della navigazione, in anello chiuso sulle misure |
| Sicurezza del moto | frame del GD32 | 20 ms | incastrato/slittamento (comandato vs encoder, correnti, IMU), bordo delle zone proibite: ferma il robot |
| Lidar | timer | 200 ms | elaborazione del giro: ostacoli, distanze, dati per mappa e navigazione |
| Navigazione | timer | 100 ms | dove andare: esplorazione, copertura con BA*, evitare ostacoli e zone proibite |
| Mappatura | timer | 200 ms (500 se il WCET non ci sta) | griglia di occupazione + allineamento delle scansioni |
| Comandi, LED, spazzole, aspirazione, stato | server aperiodico | budget del server | eventi dall'app e dall'home server |
| Telemetria e log | sfondo | best effort | non RT |

## Comandi (`./rtes.sh`)
| Comando | Cosa fa |
|---|---|
| `test` | compila per il PC ed esegue unit test e prove della spia |
| `build` | compila per il robot (ARM, statico) |
| `deploy` | copia il binario in `/tmp/rtes` sul robot e lo avvia (solo in RAM) |
| `scarica` | copia un file dal robot |
| `ssh` | apre una shell sul robot |
| `sim [stanza] [programma]` | simulatore + pagina web (http://localhost:8080) + `rtes-driver` o il tuo programma |
| `sim-solo [stanza]` | solo simulatore e pagina |
| `install`, `uninstall` | da fare (fase 6) |
