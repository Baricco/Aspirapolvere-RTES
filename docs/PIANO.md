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
      ricerca della cella con almeno una metà sporca (`GOAL_DIRTY`). Niente da pulire: riposo (poi ritorno alla
      base; il giro dei bordi viene prima delle strisce, deciso 2026-11-01, vedi più sotto);
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
      quando lo stato cambia. La **spazzola laterale** è montata, sulla destra del robot;
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
    - **FATTO il 2026-11-02, vedi "allineamento delle scansioni" più sotto (deciso 2026-10-29): l'allineamento delle
      scansioni** nella mappatura, che corregge la
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
    guidato dalla distanza del lidar, da aggiungere alla navigazione dopo BA* (progettato il 2026-11-01: giro dei
    bordi, anche attorno agli ostacoli, vedi più sotto);
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
      centro di cella cade nella striscia in cui ci sta il centro del robot. Risolto in parte con la mappa a due
      livelli (sotto).
  - **mappa a due livelli** (deciso 2026-10-30, da fare prima dell'allineamento delle scansioni): la regola "libera"
    guarda solo il centro della cella, cioè controlla le posizioni del robot ogni 20 cm. Cucina del simulatore
    (modello con la geometria vera, 2,5 cm di margine): il robot raggiunge il **19%** del pavimento che potrebbe
    toccare; la mappa vera dell'esplorazione coincide con il modello, quindi è la regola, non la mappatura.
    - **byte della cella** (convenzione dell'utente, commenti in `mapping.h`): 4 bit bassi = stato (i 5 valori di
      oggi; un bit resta libero), 4 bit alti = **le 4 sottocelle da 10 cm, bit a 1 = sottocella bloccata** (simboli
      `NAV_CELL_TOP_LEFT` bit 4, `NAV_CELL_TOP_RIGHT` 5, `NAV_CELL_BOTTOM_LEFT` 6, `NAV_CELL_BOTTOM_RIGHT` 7; alto =
      y più alta, destra = x più alta). "Bloccata" ha il significato di oggi per la cella intera: il centro del
      robot non può stare nel centro della sottocella (c'è una cella fine occupata entro 20 cm), non "dentro c'è un
      ostacolo". I centri delle sottocelle, a ±5 cm dal centro della cella, fanno un reticolo di punti a 10 cm. I bit
      si scrivono **in tutte le celle** (in una bloccata sono tutti a 1); una cella **tutta libera vale 1 come oggi**,
      quindi i confronti di oggi con `NAV_CELL_FREE` restano giusti e vogliono dire "tutta libera", mentre i confronti
      con gli altri stati passano dalla funzione che legge lo stato. Chiudere una cella (troppo stretta) cambia solo
      lo stato. Stessa memoria (64 KB), stessa copia, stesso file;
    - **sottocella libera** = nessuna cella fine occupata entro 20 cm dal suo centro (la regola di oggi, applicata
      ai 4 centri invece che al centro della cella); **cella libera** = almeno una sottocella libera e almeno 8 celle
      fini osservate; **bloccata** = tutte e 4 bloccate; altrimenti sconosciuta. Se restano libere solo due
      sottocelle in diagonale (non collegate fra loro) la mappatura blocca anche una delle due. La mappatura
      tiene 4 contatori per cella (256 KB) al posto di uno, e ricalcola i bit anche nelle celle proibite e troppo
      strette, senza toccarne lo stato. Scartato il bit "dentro c'è un ostacolo": la navigazione dovrebbe calcolare
      le distanze da sola a ogni ricerca, e dai quadrati da 10 cm invece che dalle celle fini da 5 cm, quindi con
      4–8 cm di margine in più (modello della cucina, `tools/prove-nav/modello.py`: 23% / 13% contro 66% / 28%);
    - **ricerca**: una sola, sulle celle da 20 cm come oggi. Passo da A a B: se tutte e due hanno i 4 punti liberi si
      passa (caso grossolano, quasi tutta la stanza), altrimenti serve una **porta**: una coppia di punti affacciati
      liberi (B a est: un punto "x alta" di A con il punto "x bassa" di B alla stessa y). Scartate le due ricerche
      "prima grossolana, poi fine se non trova": costo doppio quando la prima fallisce, e percorsi lunghi che evitano
      i passaggi stretti finché ce n'è un altro;
    - **percorso**: a ogni cella si associa un punto (il centro per le celle con 4 punti liberi e per quelle non
      libere); dopo la ricerca il percorso si allarga con i punti delle porte e, se in una cella si entra e si esce da
      angoli opposti, il punto di mezzo. Scorciatoie con la linea di vista sul reticolo dei punti da 10 cm (512 × 512).
      Fra due punti liberi vicini (10 cm) il centro del robot resta ad almeno 19,4 cm dai centri delle celle fini
      occupate; dentro un quadrato di punti liberi pure;
    - **pulizia**: una metà cella è percorribile se sono liberi i due punti della sua corsia, e la corsia sta sempre a
      ±5 cm (sparisce la corsia al centro); la ricerca della cella sporca vuole una metà **percorribile** sporca,
      altrimenti una metà bloccata resterebbe una meta per sempre. Frontiera: cella libera con una vicina sconosciuta
      da un lato dove ha un punto affacciato libero. Allineamento dopo un arresto: il punto libero più vicino;
    - **numeri** (modello della cucina, ostacoli arrotondati alle celle fini "a favore" / mezzo centimetro più
      prudenti): oggi 19% / 19%, mappa a due livelli **66% / 28%**, maschera separata da 5 cm 81% / 29%. Il centro come
      quinto punto non aggiunge niente; i punti da 5 cm in una cella possono fare 9113 figure diverse, che in un byte
      non stanno. La parte sinistra della cucina resta dietro passaggi da ~45 cm: 5 cm di gioco per il centro del
      robot, quanto una cella fine, quindi si apre o no col rumore (il margine va discusso a parte).
  - **gambe sottili che lampeggiano** (misurato 2026-10-30; **2026-11-01, decisione dell'utente: restano così**, a
    meno di trovare un approccio completamente nuovo e semplice, con poche righe di codice; in parte le copre il
    giro dei bordi con i quadranti attraversati, vedi sotto): con il
    registratore della copia di prova (`tools/prove-nav/sonda_mappa.c`, `analizza_mappa.py`), cucina, 4 minuti.
    Misure e posizione vanno bene: 800 celle fini occupate su 804 toccano un ostacolo vero, 1 fantasma; nessuna
    delle 182 celle di navigazione bloccate è senza un ostacolo vero entro 20 cm (più l'arrotondamento). Ma le celle
    delle gambe (3 cm in una cella da 5) passano la soglia fino a ~3 volte al secondo: un colpo a giro (+30) contro i
    raggi che passano accanto nella stessa cella (−10 ciascuno), punteggi fra 20 e 80 attorno alla soglia 50. Alla
    fine 8 gambe su 20 sparite (una nona mai vista), 314 celle di navigazione su 340 cambiate almeno una volta, e
    **28 celle libere con il centro a meno di 17,45 cm da una gamba** (il robot la urta, lo salva il bumper).
    Correzioni da provare sulla copia di prova: isteresi (occupata finché non scende molto sotto la soglia, per
    esempio sotto 0) e/o raggi che passano più leggeri (−3/−5 invece di −10, costo: un oggetto spostato sparisce
    più lentamente). Va fatta prima di giudicare i passaggi da 45 cm.
  - **prove della mappa a due livelli, fasi 1–3** (2026-11-01, `build/prove-nav/serie-due-livelli.txt`, confronto
    con `serie-celle-incastro.txt` del 2026-10-29): 144/144 controlli delle funzioni della pulizia; esplorazioni
    finite 14 su 17 (prima 17). Celle libere: soggiorno ~340 → ~410 (~92 in parte), cucina 116/132 → 179/160,
    camera ~183 → ~222, corridoio ~310 → ~420; cucina e camera esplorano (dopo la fase 2 finivano in 0,1 s).
    Pavimento della cucina raggiungibile secondo la mappa finale (`modello.py --sonda`): 86% e 83% (modello 66%,
    prima 19% / 29%), ma gonfiato dalle gambe sparite: in cucina_s1 mancano 5 gambe su 20 e 59 quadranti
    raggiungibili hanno il centro a meno di 17,45 cm da un ostacolo vero (20 sotto 10 cm). Problemi nuovi:
    **5 contatti del paraurti** (prima 0), contro muri, muretto e poltrona, con il centro del robot a 17,0–17,5 cm
    dalla superficie; **il corridoio non finisce in 150 s** (3 prove su 3): ~110 s attorno alla poltrona, fermato
    dalla sicurezza 5–7 volte, 4–5 celle chiuse. Causa comune, il margine: un quadrante è libero se nessun centro
    di cella fine occupata è entro 20 cm (4 celle fini, `FINE_CELL_INFLATION_RADIUS`), ma la superficie vera può
    essere fino a 3,5 cm più vicina del centro della sua cella fine, quindi un quadrante libero può stare a 16,5 cm
    dall'ostacolo; la sicurezza porta a zero la velocità in avanti quando il bordo del robot è a 1 cm (centro a
    18,45 cm). Prima i centri delle celle stavano raramente al limite, ora i percorsi passano per i quadranti sul
    limite. Modello della cucina con ingrossamento più grande (`modello.py --ingrossamento`): 22 cm → 63% / 27%,
    25 cm → 22% / 14% (20 cm: 66% / 28%). Deciso con l'utente: **22 cm** (`INFLATION_MARGIN_MM` 46, confronto in
    metri invece che in celle fini intere). Con 22 cm (`serie-margine22.txt`): **17 esplorazioni su 17**, corridoio in
    112–126 s, 4 urti in tutto (risolti dallo sblocco). Gli urti risolti dallo sblocco non sono un problema per
    l'utente: conta che esplorazione e pulizia finiscano e coprano la stanza.
  - **fase 4, pulizia sui quadranti** (2026-11-01, `pulizie-fase4.txt`): metà percorribile = cella libera e i due
    quadranti della sua corsia liberi; corsie sempre a ±5 cm; la cella sporca da cercare vuole una metà percorribile
    sporca. 156/156 controlli. Cucina: finita in 721 s, pavimento toccato 66% (modello a 22 cm 63%), metà
    percorribili pulite 76% (29 celle con una metà percorribile sporca alla fine, da capire), 20 sblocchi. Camera:
    finita in 404 s, toccato 82% (75% prima della fase 4, modello 85%), metà pulite 96,5%. Soggiorno: **il robot si
    perde** (fermato a ~830 s): strisce perfette fino a 445 s, poi si incastra più di 10 volte sotto la credenza
    rialzata (9–40 cm, il lidar a 8,7 cm ci passa sotto), le ruote slittano e la posizione deriva (oltre 50 cm dopo
    626 s, fino a 1,9 m); niente la corregge perché manca l'allineamento delle scansioni (prossimo passo).
  - **giro dei bordi** (deciso 2026-11-01, **obbligatorio**, da fare dopo l'allineamento delle scansioni): oggi il
    robot non pulisce né lungo i muri né attorno agli ostacoli, perché il centro resta ad almeno 22 cm (corpo a
    ~4,5 cm). Missione: **esplorazione → giro dei bordi → strisce (BA\*) → ritorno alla base** (i bordi prima delle
    strisce, così i quadranti attraversati aprono la mappa per le strisce).
    - **controllo che reagisce al lidar**, non alla griglia: obiettivo `OBJ_CONST_SPEED` (v e ω), ostacolo sempre dallo
      stesso lato a **3 cm** dal corpo (non 1–2: vedi i quadranti attraversati). Con la spazzola laterale pulisce la
      fascia che le strisce non toccano;
    - **prove nel simulatore del controllo** (2026-11-03, programma temporaneo su una copia, velocità 0,15 m/s, ostacolo a
      destra, il lato della spazzola laterale, con il davanti dal lato del bumper): i **12 settori non bastano per le gambe** con la taratura provata (una
      sola, non cercata a fondo): attorno a una gamba il corpo oscilla fra 1,3 e 20 cm, e sui muri si allarga fino a 20 cm
      sugli angoli sporgenti. Funziona invece il **metodo sui punti grezzi**: mediana su 5 punti vicini in angolo; si prende
      il punto più vicino davanti o dal lato seguito; partendo dalla sua direzione ci si allontana a passi di 2° fino alla
      prima direzione in cui un segmento lungo **10 cm** resta ad almeno 20,45 cm (corpo a 3 cm) da tutti i punti, oppure
      non si avvicina a chi è già più vicino; ω = 2,5 × errore (massimo 1 rad/s), con l'errore riportato all'istante attuale
      con l'angolo dell'odometria grezza; v ridotta con l'errore, a zero da 0,8 rad. Stesso metodo per muri, angoli e gambe, senza
      distinguere i casi. Risultati (distanza vera del corpo): stanza di prova con angoli 2,7–5,5 cm, mediana 3,6, 99% entro
      5 cm, un giro in ~110 s; gamba da 3 cm di diametro 3,4–4,5 cm, un giro ogni ~13 s; cucina, contorno esterno, 195 s,
      mediana 3,6, minimo 1,9, nessun urto. A 0,25 m/s: 2,0–6,5 cm, 92% entro 5 cm. Guardare 30 cm avanti taglia gli angoli
      rientranti (14 cm), 15 cm arriva a 7,5 cm: 10 cm è il migliore dei tre provati. **Non provato**: scelta del bordo successivo (gli ostacoli in mezzo alla stanza non sono stati raggiunti, tranne la gamba da cui partiva la prova, e il robot non si ferma mai da solo), robot vero (rumore,
      ruote, portata minima del lidar da vicino), tempo di calcolo sul robot (fino a ~65 000 controlli a scansione,
      da misurare e ridurre), gambe più sottili (con meno di 3 punti la mediana le cancella), oggetti che si muovono;
    - **non solo i muri: tutti gli ostacoli in mezzo alla stanza, comprese le gambe sottili** (deciso con l'utente il
      2026-11-03: niente dimensione minima, le gambe servono anche a segnare i quadranti attraversati). Due livelli
      (2026-11-03): la **mappa sceglie dove andare**, il **lidar guida lungo il bordo**. Niente percorsi completi calcolati
      in anticipo e niente caso speciale per il muro esterno: dalla mappa attuale si cerca il **quadrante di bordo più
      vicino non ancora percorso** (libero, raggiungibile, accanto a uno bloccato), il robot ci va con l'inseguimento
      del percorso e segue l'ostacolo finché torna vicino al punto di partenza o supera un tempo massimo; si ripete
      finché non restano bordi. Il tempo massimo non è un filtro sugli ostacoli piccoli: serve se il robot perde
      l'ostacolo. Il bordo lo generano **solo le celle bloccate** (non zone proibite, troppo strette, sconosciute), ma
      il robot deve comunque evitare le zone proibite. Da guardare nelle prove: se la gamba lampeggia il robot la perde;
      il lidar gira ogni 200 ms (3 cm a 0,15 m/s); i settori da 30° potrebbero non bastare per l'angolo rispetto al muro.
    - **condizioni emerse il 2026-11-02** perché risolva anche la pulizia che non finisce: (1) deve segnare come pulito tutto ciò
      che sta sotto il robot, mentre oggi `set_robot_cell_clean` segna una metà solo entro 2,5 cm dalla sua corsia; (2) le gambe
      sottili possono lampeggiare nella mappa; (3) il lidar passa sotto la credenza, quindi quel bordo va trovato per urti: da decidere.
    - **dettagli decisi con l'utente il 2026-11-03**: (1) **arrivo**: il robot arriva al quadrante di bordo e ruota sul posto dalla parte più breve fino ad avere l'ostacolo a destra (a sinistra vuol dire 180°); serve l'angolo dell'ostacolo più vicino su tutto il giro, perché la direzione libera considera solo davanti e destra. (2) **segnare il pulito** (2026-11-03, rullo misurato dall'utente: 15-17 cm, si usa 15). **Strisce sempre a 10 cm** (deciso dall'utente: pulire troppo non è un problema, le strisce sporche sì, e distanze alternate sarebbero solo una fonte di errori): sulle strisce `set_robot_cell_clean` resta com'è, segna solo la corsia del robot. **Solo mentre il robot segue un bordo** (non nei trasferimenti, altrimenti un bordo sfiorato passando risulterebbe fatto) segna le metà con il centro nella fascia da 5 cm a sinistra del centro del robot fino al bordo destro (17,45 cm, il lato della spazzola laterale), calcolata con l'angolo del robot; 5 cm e non 7,5 perché con 7,5 tra il giro e la prima striscia potevano restare 2,5 cm sporchi, con 5 nessuno (conto con rullo da 15, non provato). L'asse delle strisce va calcolato prima del giro. **Fine delle strisce**: con i muri perpendicolari la striscia si ferma al centro dell'ultima cella percorribile, fino a ~20 cm prima del punto raggiunto dal rullo nel giro; lo stesso buco c'è all'inizio della striscia. **Deciso con l'utente (2026-11-03): ogni corsia si allunga alle due estremità, senza stati nuovi**: il percorso diventa inizio, punto indietro (centro della cella precedente sulla corsia), di nuovo inizio, resto della striscia, punto oltre la fine (centro della cella successiva sulla corsia). Sui due punti aggiunti il robot considera il punto raggiunto se `corridor_front_distance` scende a ~3 cm o se è in stallo; `next_turning_point` non deve saltare il punto indietro. Calcolo di sola geometria (stanza vuota 3×2 m, griglia in 9 e 25 posizioni, rullo 15 cm sotto il centro, niente rumore): pavimento non toccato dal rullo, esclusi fascia lungo i muri e angoli, in media 1570 cm² oggi, 128 cm² con la sola fine usando la mappa, 47 cm² con inizio e fine usando la mappa (spessore 2 cm), 0 con inizio e fine usando il lidar. Contare la spazzola laterale, ipotizzata su tutto il lato destro dal rullo al bordo del robot, non chiude i 2 cm della versione con la mappa. **Corretto dopo la prima prova** (stanza vuota, seme 1, 2026-11-03: giro dei bordi in 77 s senza urti, strisce troppo lente, ~1/3 della stanza in 214 s): si allunga solo dove la cella dopo non è percorribile; la striscia parte dal punto indietro invece di andarci e tornare; parte dall'estremità della corsia sporca più vicina al robot, così le corsie non si spezzano; la prima striscia parte dall'estremo della stanza più vicino sull'asse laterale. Nella stanza vuota del calcolo la copertura non cambia (0 cm²). **Da agganciato non si segna più la metà sotto il centro**, solo prima dell'aggancio: lungo i muri perpendicolari alle strisce faceva fermare le strisce una cella prima, lasciando una fascia sporca di ~12 cm. Prove nel simulatore (seme 1): stanza vuota finita in 469 s (bordi 84 s, strisce 385 s), rullo non passato sullo 0,2% del pavimento interno, nessun urto; cucina finita in 941 s (bordi 316 s in 4 giri, strisce 625 s, tempo massimo mai scattato), giri attorno a tavolo e a tutte e 4 le sedie, 7 urti, 19 sblocchi, 14 celle chiuse, 5 metà percorribili sporche su 240. Sulla cucina restano senza rullo né spazzola il 14% del pavimento: interno delle sedie, dove il robot non entra, e gli ultimi ~3 cm lungo i muri. Soggiorno (seme 1, 2026-11-03): **guasto a 807 s, durante il giro dei bordi**, 16 sblocchi e 35 giri. Due problemi aperti: (a) dopo un giro interrotto la ricerca porta il robot al capo vicino del tratto da fare, ma tenendo l'ostacolo a destra il robot riparte verso il tratto già fatto: dopo 50 cm il giro finisce e avanza di ~40 cm ogni ~16 s; (b) sotto la credenza 10 sblocchi di fila esauriscono `UNLOCK_MAX_ATTEMPTS`, perché il lidar non la vede e la mappa non la impara. Con la zona proibita sulla credenza nella copia di prova (l'utente: la credenza è un caso da zona proibita) e le correzioni su segno attraversato e tentativi falliti (seme 1): giro dei bordi finito in 626 s (18 giri), niente incastri, ma **guasto a 1290 s durante le strisce**, 88% di pavimento toccato: vicino alle scale il punto indietro di una striscia cade oltre il vuoto, l'anti-caduta scatta prima di arrivarci, la pulizia ripianifica la stessa striscia e i 10 tentativi si esauriscono. La sicurezza non ha fatto entrare il centro nella zona, ma il corpo la sfiora fino a 1,9 cm. Corretto: il punto indietro e quello oltre la fine non si aggiungono se la cella contiene un ostacolo invisibile al lidar, cioè vuoto o ostacolo basso (`is_nav_cell_invisible_obstacle`, che legge la griglia fine). Stessa prova: **missione finita in 1807 s** (bordi 659 s in 18 giri, strisce 1112 s), pavimento toccato 92% (17,07 m²), 5 urti, 17 sblocchi, 24 celle chiuse, nessun incastro, mai nel vuoto, nessuna scadenza mancata; 594 metà percorribili pulite su 618 (12 celle con una metà sporca, non verificato se raggiungibili). Il corpo sfiora ancora la zona fino a 1,9 cm. Non provati camera, corridoio, altri semi, né il robot vero. Il firmware originale (`CCoverageMap::updateCoverMap`) segna un quadrato di ±16 cm attorno al centro lungo il tratto percorso, senza angolo e senza spazzola laterale; non è stato verificato se quella mappa decide anche dove pulire. (3) **bordi fatti**: i quadranti di bordo hanno il centro 1,5–11,5 cm a sinistra del robot, dalla parte opposta alla spazzola, quindi spesso restano fuori dalla metà segnata: un quadrante di bordo è fatto se c'è una metà pulita entro ~15 cm (nel caso peggiore, muro perpendicolare alle strisce, la più vicina è a 15 cm), senza nuove strutture. **Fine del giro** (sostituisce il ritorno al punto di partenza): il giro finisce quando il robot ha percorso ~40-50 cm senza segnare metà nuove. Così per un quadrante rimasto il robot ci va, lo pulisce e passa al successivo, e chiudere un anello è lo stesso caso. **Nessuna distanza massima** del giro. (4) **tempo massimo solo come rete di sicurezza**: serve a non girare all'infinito o restare incastrato, non deve MAI scattare; se scatta è un difetto da correggere, va contato e stampato. La fase deve finire sempre con successo: a ogni giro il robot arriva su un quadrante e ne pulisce la metà, quindi le metà da pulire calano sempre (salvo guasti). **Deciso con l'utente**: il giro finisce normalmente anche dopo uno sblocco o uno stop della sicurezza (credenza, ostacoli bassi, vuoto, zone proibite, che la guida non vede), e se l'ostacolo sparisce dal lidar per circa 1 s; poi si cerca il bordo successivo. La distanza senza metà nuove è la strada percorsa, non la distanza in linea d'aria (attorno a una gamba il robot non si allontana mai più di ~45 cm). La metà sotto il centro del robot va segnata sempre, anche durante la rotazione iniziale, in modo esplicito: con il robot in diagonale il suo centro può cadere fuori dalla fascia (fino a ~11 cm). Così basta iniziare il giro perché il quadrante di arrivo risulti fatto: da qui la garanzia che la fase finisca. La fascia invece si segna solo da agganciato, altrimenti la rotazione sul posto segnerebbe pulito tutto attorno al robot.
    - **come si scrive** (2026-11-03, approvato dall'utente): niente macchina a stati dentro la macchina a stati, si segue lo schema di esplorazione e pulizia. `NAV_STATE_FOLLOW_BORDERS` è uno stato di pianificazione: calcola l'asse delle strisce se manca; se la cella del robot è un bordo da fare parte il giro; altrimenti cerca con un nuovo obiettivo di `search_cell` e parte con `start_following_path`, che torna alla missione a fine percorso; se la ricerca non trova nulla passa alla pulizia. Il giro è **un solo stato nuovo**, senza fasi: a ogni job, se non è ancora agganciato ruota sul posto (è un `if` con un booleano), altrimenti comanda v e ω dalla direzione libera; controlla le condizioni di fine e in tutti i casi torna alla missione. Gli sblocchi non richiedono codice: il controllo degli allarmi all'inizio del job e il ritorno alla missione chiudono già il giro. L'esplorazione finita passa a `NAV_STATE_FOLLOW_BORDERS`, non più alla pulizia. Il nuovo stato va aggiunto anche a `NOMI` in `tools/prove-nav/prova.py`, che dipende dall'ordine dell'enum. I quadranti attraversati non servono a decidere i bordi fatti.
  - **base, uscita e rientro** (deciso con l'utente il 2026-11-04; la parte nel driver è scritta, quella nei task no). La base non è un ostacolo normale: il robot non deve spostarla.
    - **zona proibita automatica**, inserita dalla mappatura, scartato un valore speciale della cella (la sicurezza non lo vedrebbe e il giro dei bordi, guidato dal lidar, ci arriverebbe). Margine **7 cm** attorno alla base stimata da 30 × 12 cm: x da −0,365 a −0,105, y ±0,22 (origine = robot agganciato). Celle proibite: 4 × 4, x da −0,6 a +0,2, y ±0,4; sui percorsi il corpo resta a 25 cm davanti e 12,5 cm di lato; se il giro dei bordi la punta si ferma a ~8 cm (errore massimo di posizione visto nel soggiorno del simulatore: 7,2 cm). Prezzo: ~75 × 35 cm di pavimento sporco davanti alla base. **Unico scrittore** della lista delle zone: la mappatura; l'app le passerà le sue. "Partito dalla base": `sulla_base` visto nei primi job (il bit può essere intermittente), deciso dalla mappatura e pubblicato. Cella proibita = tocca la zona allargata del raggio del robot. `is_lattice_point_blocked` ignora i quadranti delle celle proibite; `cell_can_enter` attraversa le proibite solo se parte da una proibita.
    - **uscita**: uno stato di navigazione solo se il robot è partito dalla base: `OBJ_CONST_SPEED` a 0,1 m/s senza rotazione fino a x ≥ 0,30, senza percorsi. `drv_esci_dalla_base` a 1 solo qui (via da `main.c`). Niente sblocco qui dentro (arretrerebbe contro la base): stop e nuovi tentativi, poi guasto. Spazzole e aspirazione spente. Lo stesso stato serve dopo una ricarica.
    - **rientro**: la navigazione arriva con i percorsi normali a (0,30; 0) con angolo 0, poi chiama `drv_aggancia(1)` e legge l'esito in `drv_stato_t.aggancio`.
    - **aggancio** (`driver/aggancio.c`, provato in `test_driver_logica` e nel simulatore, **non sul robot**): a ogni giro la faccia della base si cerca fra i punti dietro al robot (sequenza di punti vicini, in linea, larga come la base) e dà angolo, distanza e scarto laterale; fra un giro e l'altro l'angolo viene dall'IMU. Retromarcia a 5 cm/s correggendo; a 6 cm dalla base controllo (scarto ≤ 2 cm, angolo ≤ 5°: se è spostato si ritira e riprova, al massimo 10 tentativi); ultimi centimetri a 3 cm/s solo con l'IMU; al contatto il driver ferma le ruote, poi si aspetta la carica (15 s; contatto stabile senza carica = batteria piena). Prove nel simulatore: 6 partenze su 6 agganciate, con scarti fino a 10 cm e angoli fino a 15° (una al secondo tentativo).
    - **riletta la prova di aggancio del 6/10** (`build/robot/fase3/dock1/dock1.reg`). Faccia vista larga 28,4 cm (stima 30 confermata; la profondità non si ricava), robot a 45,5 cm con 5,4° di inclinazione e 2,6 cm di scarto. Il lidar misura ancora a 10,8 cm, come il limite di 11 cm del simulatore; a contatto la faccia sarebbe a ~6 cm, quindi gli ultimi ~5 cm sono ciechi. **Correzione a `docs/fase3.md`**: il robot non è ruotato perché ha toccato la base. È ruotato di 28° fra 154 e 157 s, con la schiena ancora a ~20 cm dalla base, con gli encoder fermi (l'hanno visto IMU e lidar), e il comando non correggeva; ha toccato la base dopo, di lato. Causa della rotazione sconosciuta (slittamento di una ruota? cavo o bordo sul pavimento?).
    - **batteria bassa** (decisione dell'utente: riprende da solo): a `BAT_BASSA` (14,2 V) ritorno alla base; dopo la ricarica esce e continua la missione interrotta, con mappa e copertura ancora in memoria. La soglia per ripartire va su `percentuale` (la carica nelle prove si ferma a 16,3 V e il bit "in carica" può restare acceso a lungo).
    - **dieci sblocchi esauriti**: resta il guasto (decisione dell'utente), quindi con un mobile basso senza zona proibita la pulizia si ferma lì.
  - **quadranti attraversati** (deciso 2026-11-01, insieme al giro dei bordi): dove il robot è passato davvero, il
    quadrante resta libero per tutta la sessione, qualunque cosa dica il lidar. Serve in cucina: i passaggi da ~45 cm
    verso la metà sinistra, che la mappa (margine 22 cm, celle fini da 5 cm) chiude ma il giro dei bordi (guidato
    dal lidar, ~1 cm di precisione) può attraversare, e l'uscita da sotto il tavolo (le celle da cui il robot è
    entrato restano aperte anche quando le gambe lampeggiano); sui quadranti attraversati il rumore sparisce.
    - **dove si salva: il bit più alto (128) dei contatori della mappatura** `num_occupied_fine_cells[4][...]`, uno
      per quadrante, 8 bit: contano le celle fini occupate entro 22 cm, al massimo ~61, quindi il bit 128 non si usa
      mai. Niente strutture nuove e la navigazione non cambia. **Segno**: a ogni job della mappatura `|= 128` sul
      contatore del quadrante sotto il robot, **solo se il centro del robot passa entro ~2 cm dal centro del
      quadrante** (con il giro dei bordi a 3 cm, un quadrante segnato ha il robot dentro con almeno 1 cm di gioco).
      **Lettura**: in `calc_nav_cell_bitmask` un quadrante è bloccato solo se il contatore è fra 1 e 127. I ±1 dei
      contatori non toccano il bit 128 (restano sotto ~61);
    - **la cella chiusa vince**: lo stato "troppo stretta" è separato dai bit, quindi sotto la credenza (il robot ci
      arriva ma si incastra) la navigazione la chiude come oggi;
    - **scartato uno stato nuovo della cella** ("attraversabile", o il bit libero dei 4 dello stato): la cella da
      20 cm è troppo grande. Il giro dei bordi passa con il centro a ~18,5–19,5 cm dal muro, e una cella intera
      libera includerebbe quadranti con il centro a 13–14 cm dal muro (dentro il corpo); lungo tutti i muri strisce
      e percorsi andrebbero contro il muro, con urti e celle chiuse a catena. Serve un'informazione per quadrante
      (4 bit per cella), che nel byte della mappa non sta;
    - **dopo l'allineamento delle scansioni**: un quadrante attraversato non cambia più per tutta la sessione, quindi
      con la posizione sbagliata (come nel soggiorno, fino a 1,9 m) se ne segnerebbero per sempre dentro i mobili.
  - **allineamento delle scansioni** (progettato e scritto 2026-11-02 in `mapping.c` e `movement.c`, non ancora provato
    sul robot; manca la ricerca su richiesta). Scelto dopo prove su un modello semplificato (programmi temporanei, non nel
    repository), poi corretto con le prove nel simulatore sulla pulizia del soggiorno:
    - **a ogni giro, prima di scriverlo in mappa** (`update_aligned_pose`): la stima parte da quella precedente più lo
      spostamento dell'odometria, poi **5 passi di Gauss-Newton su x e y** (`align_scan_step`, sistema 2×2) con **lettura
      bilineare** della griglia fine esistente (`read_fine_grid_bilinear`: media pesata delle 4 celle attorno al punto, da cui
      esce anche il gradiente; contano solo i punteggi da 1 a 100, quindi niente celle libere né valori speciali). Alla
      diagonale si somma `ALIGN_DAMPING` (1e8): nelle direzioni piatte, come lungo un corridoio, il passo resta quasi nullo.
      Passo limitato a 5 cm. Numero di passi e di punti fissi, quindi tempo fisso. Scartati: bicubica (non migliora e costa di
      più), tentativi a 1 cm (più lenti e non più precisi). Niente seconda griglia e niente memoria rilevante in più.
    - **l'angolo viene solo dall'IMU** (deciso 2026-11-02 dopo le prove): con x, y e angolo corretti dalla mappa l'angolo
      derivava da solo (fino a 12,6° e 54 cm di errore), con il simulatore in cui l'IMU è quasi perfetta. Sul robot l'IMU
      sbaglia ~0,2° ogni due giri: da controllare. Se servirà: correzione lenta dell'angolo solo a robot quasi dritto e con
      punteggio alto, al massimo ~0,002° per job e con un tetto totale di ±1-2° rispetto all'IMU (numeri non provati).
    - **storico delle pose** (`movement_odom_history_t`, 32 pose, ~640 ms, doppio buffer a parte): il moto pubblica a ogni
      frame le ultime pose dell'odometria grezza con il loro istante, `pose_at_time` trova quella di un istante qualsiasi
      interpolando. Non si usano più `v` e `omega`: da incastrato le ruote dichiarano ancora la velocità piena (0,16 m/s a
      robot fermo) e nascevano spostamenti inventati, con salti di ~10 cm a ogni rotazione sotto la credenza.
    - **correzione al moto**: il moto tiene l'odometria grezza, mai corretta (`odom_x`, `odom_y`, `odom_theta` nella posizione
      pubblicata, usati solo dalla mappatura). La mappatura pubblica con il doppio buffer `mapping_correction_t`: odometria e
      stima all'ultimo allineamento, che sostituisce la precedente, quindi non invecchia e perderne una non crea errori. Il
      moto la applica a ogni frame (`apply_map_correction`) e guida e pubblica la posizione sulla mappa in `x`, `y`, `theta`:
      navigazione e sicurezza non cambiano e `drv_posa` riceve già quella corretta (quindi gli eventi dei contatti sono già
      sulla mappa). Dentro la mappatura `odom_to_map` fa lo stesso conto. L'avanzamento a pochi millimetri per frame non è
      stato scritto: la correzione si applica subito, e finora non è servito.
    - **rilevatore di incastro** (scritto 2026-11-02, necessario): `safety.c` confronta le velocità delle ruote con quelle
      comandate, ma da incastrato le ruote slittano a piena velocità e l'allarme non scatta. Con la posizione giusta il robot
      non "arriva" più per finta e resta a spingere per sempre (visto: oltre 8 minuti). Ora la mappatura confronta, sugli ultimi
      7 giri (~1,4 s), lo spostamento dell'odometria e quello della stima: oltre 5 cm contro meno di 2 cm vuol dire incastrato
      (`is_stuck` in `mapping_correction_t`), e la sicurezza attiva `SAFETY_ERROR_STUCK`, da cui partono l'evento e lo
      sblocco che esistevano già. Sotto ~0,12 m/s non arriva ai 5 cm. Soglie da ricontrollare sul robot: nel simulatore
      l'odometria avanza al 30% della velocità comandata, sul robot non è misurato bene.
    - **punteggi**: restano +30/-10, i migliori sul modello (con +10/-3 l'errore costante sale a 5,5 cm), quindi la mappa di
      navigazione non cambia. Provati dal firmware originale e non presi: filtro di movimento (nessun effetto), riduzione dei
      punti (-40% di tempo, picco un po' più alto), controllo dell'angolo 10°/50° (peggiora con le spinte).
    - **risultati nel simulatore** (soggiorno, seme 1, 1500 s, il limite della prova, con griglia sui muri nel caso peggiore):
      errore tipico 3,5 cm stabile per tutta la prova, 95% sotto 5,6 cm, massimo 7,2 cm, alla fine 2 cm, angolo sulle strisce
      ~0,1°; 26 sblocchi, nessuna scadenza mancata (prima: fino a 1,9 m). Il 19% del tempo sta sopra l'obiettivo di 4 cm; parte è
      ritardo di misura. Lo scarto costante è circa mezza cella: il metodo porta i muri verso il centro delle celle da 5 cm,
      causa probabile ma non verificata. **Non provato con muri in posizioni qualsiasi** né su cucina, camera, corridoio.
    - **la pulizia non finisce entro 1500 s**: dopo ~750 s il robot gira senza concludere (un ciclo di 4 trasferimenti ogni 26 s
      in una prova). Causa probabile, non dimostrata: la mappa oscilla vicino agli ostacoli, una metà sembra sporca e raggiungibile
      da lontano e si blocca da vicino. Quindi la regola "ogni passaggio pulisce una metà nuova oppure cambia la mappa" non basta
      a garantire la fine. Tutte le 14 ricerche fallite e le 4 celle del ciclo stanno entro 38 cm da un ostacolo, cioè nella fascia
      del giro dei bordi: si dovrebbe risolvere con quello (decisione dell'utente). Se non basta: limite ai tentativi per cella.
    - **tempi sul robot** (2026-11-02, programma di prova con x, y e angolo, non il codice finale, a priorità normale): controllo
      del punteggio ≤ 0,19 ms; 5 passi ≤ 0,90 ms; ricerca + 5 passi ≤ 11,8 ms. Da sommare alla scrittura in mappa e alla
      pubblicazione (`job_mapping` non è ancora stato misurato).
    - **ricerca a tentativi su richiesta: da fare dopo il giro dei bordi** (progettata, non scritta): se fra i punti su celle
      note (`!= 0`, almeno 30) meno dell'80% cade su celle con punteggio positivo, ricerca su **±20 cm a passi di 5 cm e ±5° a passi
      di 0,5°**, poi altri passi, e si tiene il risultato solo se il punteggio migliora. Non a ogni giro: nei corridoi lunghi è
      lei a portare fuori strada. Costa ~13 ms nel job che la esegue, ed è il caso peggiore per l'analisi. Oggi uno scarto oltre
      ~5-10 cm non si recupera. Con l'angolo dall'IMU il suo ±5° va rivisto.
    - **punti aperti**: corridoi lisci molto lunghi (lungo l'asse resta l'errore dell'odometria); angolo di aggancio alla base da
      misurare (griglia ruotata di 3°: picco 22 cm sul modello; la base sta contro un muro, quindi gli assi partono circa
      allineati); soglia dell'80% e rilevatore da riprovare con persone e oggetti in movimento.
  - **ricerca globale dopo un sollevamento** (deciso 2026-11-02, recupero da un guasto: **da fare dopo il giro dei
    bordi**, l'ordine preciso è da decidere). Oggi `valid` è sempre vero (`movement.c:124`): manca il collegamento al
    sollevamento riconosciuto dal driver; mappatura e navigazione già reagiscono a `valid` falso. Robot sollevato: `valid`
    falso, tutto fermo. Riappoggiato: resta fermo e cerca la posizione su tutta la mappa **a fette**, ogni job con tempo
    limitato e un numero di job libero, così nessuna scadenza si perde. Riparte solo con la posizione confermata (errore
    entro quello dell'allineamento locale), con `valid` di nuovo vero, e riprende il lavoro che stava facendo. Può girarsi
    per avere una vista migliore. Ordine di grandezza (misurato sul robot con griglia a bit da 32 KB, 10 cm, 6000
    posizioni, 85 punti): 8 ms per angolo, griglia a bit 15 ms una volta sola. Da decidere quando si fa: stanze simili,
    affidabilità, se l'IMU conta anche da sollevato (non verificato), uso anche per le spinte grandi.
  - **cosa resta per finire il progetto** (2026-11-01, aggiornato 2026-11-02, in ordine). Fase 4: (1) allineamento delle
    scansioni (scritto, vedi sopra);
    (2) giro dei bordi e quadranti attraversati (`mark_cells_under_robot`, `mapping.c:677`, va estesa ai quadranti): vanno insieme, la prima prova completa si fa solo dopo tutti e due, altrimenti sarebbe inconsistente (deciso 2026-11-03); (3) la ricerca a tentativi su richiesta dell'allineamento; (4) ritorno alla base (`RETURN_HOME`, `HOOK_UP`, `SLEEP` oggi
    vuoti): percorso fino davanti alla base (zona proibita permanente), aggancio con l'IR nel driver, poi riposo;
    (5) spazzole e aspirazione (accese in pulizia, ferme nello sblocco); (6) mappa salvata e ricaricata; (7) guasto
    chiesto da un task (`drv_segnala_guasto`); (8) prove sul robot vero con i motori; (9) WCET e jitter sul robot,
    assegnazione ai core, analisi di schedulabilità (RM partizionato). Da inserire dopo il giro dei bordi, in un punto
    da decidere: ricerca globale dopo un sollevamento. Fase 5: tipo di server aperiodico e di app,
    comandi e telemetria. Fase 6: avvio automatico con ritorno all'originale, SSH solo con chiave, refactoring di
    driver, `rt/` e simulatore (stile da decidere insieme) con revisione dell'utente, relazione.
  - **memoria bloccata sul PC** (2026-10-30): il programma blocca in RAM 7,97 MB e il limite per i normali utenti è
    8 MB; con MCL_FUTURE i thread creati dopo non partivano più (il `main` esce senza messaggio). `rt_init` ora,
    se non è root e il limite c'è, blocca solo la memoria di adesso (sul robot, root, tutto come prima).

## Ancora da decidere
- **Mappa e navigazione, punti aperti** (2026-10-13): ingombro della base, aggancio alla base nel driver (passaggio
  di consegne dai task al driver e ritorno). La larghezza della striscia è decisa (10 cm con le mezze celle,
  2026-10-28), e anche l'asse delle strisce (calcolato a ogni pulizia, vedi la pulizia con BA\*).
- **Tipo di server aperiodico** (polling, deferrable, sporadic) e **tipo di app** (con l'home server, più avanti:
  zone proibite, preferenze, programmazione della pulizia).
- **Zone proibite** (da fare, ricordato dall'utente il 2026-11-03): le inserisce l'utente dall'app e le scrive un **task aperiodico** con `write_forbidden_zone_list`; è normale che oggi nessun task la chiami. Oggi le legge solo la sicurezza (`safety.c:225`). **Manca anche la mappa**: niente segna le celle come `NAV_CELL_FORBIDDEN` a partire dalle zone (`mapping.c` conserva solo lo stato se c'è già), quindi la navigazione non le vede e la pagina del simulatore non le disegna.
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
