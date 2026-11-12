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
  - **task aperiodico sul core 3** (deciso il 2026-11-07 al posto del server aperiodico, vedi "Task aperiodico" sotto):
    salvataggio della mappa, ricerca della posizione dopo un sollevamento, più avanti comandi e telemetria dell'app.
    Spazzole, aspirazione e LED non ci passano: li comandano la navigazione e il driver;
  - rete di sicurezza del kernel sui task RT (95% della CPU per secondo) lasciata attiva.
- **Task aperiodico** (deciso il 2026-11-07, al posto del server aperiodico). Fa parte della fase 5.
  - **Niente server, un thread su un core tutto suo (il 3).** Un server aperiodico (polling, deferrable, sporadico)
    serve a limitare il tempo che il lavoro aperiodico toglie ai task periodici **dello stesso core**. Con RM
    partizionato ogni core si analizza da solo: sul core 3 non c'è nessun task periodico da proteggere, quindi un budget
    farebbe solo aspettare le richieste. Il task aperiodico si sveglia quando arriva una richiesta (`eventfd` o
    `sem_post`, che non bloccano chi li chiama) e la esegue fino in fondo. Il core 3 è quindi riservato al lavoro
    aperiodico; i task periodici stanno sui core 1 e 2 (il più pesante, la mappatura, usa ~12% di un core). Tre
    condizioni, perché il partizionamento separa la CPU ma non il resto: (1) **nessun mutex** condiviso con i task
    periodici; (2) **cache di secondo livello e bus della memoria** sono condivisi fra i core, quindi i tempi massimi dei
    task vanno misurati **mentre il task aperiodico copia e salva**; (3) la **flash** (sotto).
  - **Scambio dei dati**: comandi (più avanti dall'app), zone proibite e risultato della ricerca della posizione con
    **doppi buffer piccoli** (`scambio.h`, un solo scrittore ciascuno). La mappa no (sotto).
  - **Lettura della mappa fra un job e l'altro della mappatura.** La mappatura incrementa un contatore atomico
    all'inizio e alla fine di ogni job (dispari = job in corso), in una funzione che chiama il job, così nessun `return`
    anticipato salta l'incremento finale. Il task aperiodico legge il contatore, aspetta se è dispari, copia la mappa
    nella sua memoria, rilegge il contatore: se è cambiato butta la copia e riprova. È lo schema del doppio buffer di
    `scambio.h` (seqlock) senza la seconda copia. **Regola: la mappatura scrive la mappa solo dentro il suo job** (vero
    oggi: griglia fine e griglia di navigazione condivisa le scrive solo lei nel job, l'azzeramento della sessione è nel
    job, l'inizializzazione è prima dei task); la copia segnala un contatore rimasto dispari. Le interruzioni della
    navigazione (stesso core, priorità più alta) non cambiano niente: il contatore resta dispari per tutto il job,
    anche mentre la mappatura è sospesa. I task periodici non aspettano mai: nell'analisi non c'è nessun tempo di
    blocco nuovo; resta solo l'effetto su cache e bus, da misurare. Misure sul robot (core 3, 2026-11-07): copia di 1 MB
    0,8-2,4 ms; lettura dei punteggi con scrittura delle categorie a 2 bit 4,7-5,2 ms, contro una finestra libera fra due
    job di almeno ~176 ms. **Scartati**: doppio buffer della mappa (~3,6 MB: due copie da 1,2 MB più quella di chi
    legge, e la copia nel job della mappatura); lista delle celle cambiate (misura nel simulatore, soggiorno, 600 s:
    a regime al massimo 64 punteggi e 13 categorie cambiati per job, ma fino a 5.131 punteggi e 2.908 categorie in un
    job quando il robot vede la stanza la prima volta, quindi un buffer dimensionato sul picco, e il doppio buffer
    perde le versioni non lette); blocchi cambiati con conferma di lettura (codice nuovo nella mappatura, 8 KB più una
    copia della mappa nel task aperiodico); mappa compressa come struttura da aggiornare una cella alla volta.
  - **Cosa si salva** (decisione dell'utente): la **griglia fine a categorie**, sconosciuta / libera / occupata, 2 bit
    per cella (256 KB per tutta la griglia), senza punteggi (cambiano di continuo) e senza valori speciali (vuoto e
    ostacolo basso valgono per la sessione). Il task aperiodico scrive le categorie direttamente mentre legge i
    punteggi. Al caricamento la mappatura ricalcola contatori e griglia di navigazione; **da decidere**: il punteggio con
    cui riparte una cella di ogni categoria (per esempio occupata +100, o meno, così un mobile spostato sparisce
    prima). Compressione: non serve per il file (71 MB liberi su `/mnt/UDISK`, 1,2 MB scritti in 0,23 s); se servirà
    all'app la fa il task aperiodico, in un colpo solo dalla copia (calcolo sulle mappe del simulatore: una stanza
    pesa 0,3-0,6 KB come quadtree a 1 bit, ~1-1,6 KB come tratti di ostacolo + rettangoli liberi).
  - **Ricerca della posizione dopo un sollevamento: si sposta nel task aperiodico** (decisione dell'utente,
    2026-11-07, che potrebbe ancora cambiare). La conferma (`confirm_position`) resta nella mappatura. Il task
    aperiodico legge la griglia fine con la lettura fra i job e restituisce il risultato con un doppio buffer piccolo;
    senza fette da 10 ms per job, quindi i job della mappatura tornano corti (oggi fino a ~24 ms durante la ricerca).
  - **Flash** (prova sul robot del 2026-11-07, senza motori). `/mnt/UDISK` e `/overlay` sono ext4 montate **`sync`**:
    ogni `write` aspetta i dati sulla flash e il lavoro parte dal thread che scrive. `/tmp` è in RAM (le registrazioni
    delle prove non toccano la flash). Tutte le interruzioni vanno al core 0 (quello del driver), anche quella della
    flash (`nand`, la 102, ~15-25 al secondo anche a robot fermo) e del DMA (82). Si possono spostare sul core 3 (`8` in
    `/proc/irq/N/smp_affinity`: provato, 3.904 interruzioni su 3.904 sul core 3) e così i thread del kernel `nand`,
    `nand_rcd`, `jbd2/nandi-8`, `jbd2/nande-8`, `flush-93:0`, `bdi-default`, `sync_supers`; `kblockd` e i
    `kworker/u` il kernel 3.4 non li lascia spostare. Ritardo massimo di risveglio di un task a priorità 90 mentre si
    scrivono 1,2 MB tre volte dal core 3: senza scritture core 0 36,7 µs, core 2 21,5 µs; scritture con le impostazioni
    di fabbrica 73,0 / 44,3 µs (scritture da 0,93, 0,44, 0,23 s); interruzioni e thread spostati sul core 3 47,3 /
    37,2 µs (0,23 s ciascuna). 0 scadenze mancate. Gli spostamenti vanno messi nell'avvio del programma o nello script
    della fase 6 (sul robot non c'è `taskset`: programmino `affinita`, oggi solo nello scratchpad della sessione).
  - **Salvataggio della mappa, decisioni dell'utente** (2026-11-07; il codice lo scrive l'utente). **Quando**: a fine
    esplorazione e ogni volta che il robot torna in riposo sulla base (fine missione, tasto casa, rientro per la
    batteria). **Chi lo chiede**: la navigazione, con un doppio buffer piccolo che contiene anche l'asse delle strisce
    (dato della navigazione: con una mappa caricata l'elenco delle celle raggiungibili dell'esplorazione non c'è, quindi
    l'asse viene dal file), poi sveglia il task aperiodico. **Condizioni**: sessione partita dalla base, esplorazione
    completata almeno una volta (fatta o caricata), mappatura in modo normale, niente guasto; si salva anche dopo un
    sollevamento con la posizione ritrovata e confermata. **Contenuto**: solo il rettangolo delle celle viste, a
    categorie 2 bit; asse delle strisce; **zone proibite dell'utente nello stesso file** (la zona della base resta
    automatica). **Formato e controlli al caricamento: da rivedere con l'utente** (la proposta del 2026-11-07 aveva
    numero magico, versione, lato della griglia e della cella, rettangolo, asse, celle a righe allineate al byte,
    CRC-32). Se un controllo fallisce la mappa si ignora e si riparte con l'esplorazione (deciso). **Scrittura**: file
    temporaneo rinominato sopra il vecchio, **nessuna copia della mappa precedente** (la flash dovrà contenere più
    mappe). **Errore di salvataggio**: messaggio nel registro e un flag nel task aperiodico, che l'app mostrerà
    all'utente; niente guasto. **Percorso**: `/mnt/UDISK/rt/maps/` (sul PC da variabile d'ambiente, per non toccare i
    file del robot nelle prove). **Ripartire da zero**: dall'app. **Mobili spostati**: la mappa si aggiorna da sola
    durante la pulizia (raggi che tolgono punteggio, colpi che lo aggiungono) e si salva a fine missione; limiti: tempo
    per dimenticare un mobile legato al punteggio di caricamento, zone mai viste non esplorate con una mappa caricata,
    deriva dell'IMU che si accumula da una sessione all'altra. **Più mappe** (2026-11-07): numerate, `/mnt/UDISK/rt/maps/0.rtm`, `1.rtm`, …, e un file di
    configurazione con il numero della mappa attiva (lo cambierà l'app); se la mappa attiva manca o non supera i
    controlli si riesplora e si salva con lo stesso numero. **Zone proibite**: con un comando ricevuto dal task
    aperiodico (l'app riuserà lo stesso comando); serve anche segnarle nella mappa di navigazione, che oggi segna solo
    la zona della base all'avvio. **Valori speciali**: si salva **solo il vuoto** (quarto valore dei 2 bit), così le
    scale restano nella mappa; l'ostacolo basso resta solo per la sessione. **Zone nuove con una mappa caricata**:
    a fine pulizia il robot rientra senza esplorare; la missione parte sempre dall'esplorazione, che con una mappa
    completa non trova frontiere e passa subito oltre, e se la mappa salvata ha frontiere (celle libere accanto a
    sconosciute, viste nella sessione prima, per esempio attraverso una porta aperta) esplora solo quelle.
    **Porta riaperta e poi richiusa** (chiarito con l'utente il 2026-11-07): l'esplorazione va solo verso le frontiere
    **raggiungibili** (la ricerca si allarga solo sulle celle libere). Se la porta è di nuovo chiusa, il lidar la vede
    da lontano (ostacolo alto e solido), le sue celle tornano occupate in due giri, la navigazione ripianifica, la
    frontiera dietro non si raggiunge più e l'esplorazione finisce; al massimo un breve tratto verso la porta in quella
    sessione. Le celle dietro la porta restano nella mappa (il lidar non le vede più, nessuno le cambia) e si salvano;
    dalla sessione dopo la frontiera risulta irraggiungibile già all'avvio e il robot non si muove. Quando la porta si
    riapre e il robot la vede, la frontiera torna raggiungibile. Unico caso con urti: porte che il lidar non vede (vetro,
    cancelletti bassi), segnate dal paraurti come ostacolo basso (non salvato): un tentativo per sessione, si chiude con
    una zona proibita.
  - **Deriva dell'IMU, analisi delle registrazioni** (2026-11-07, script `deriva_imu.py` nello scratchpad della sessione).
    Per ogni giro del lidar a meno di 0,6 rad/s: direzione dei muri nel sistema dell'IMU, a meno di 90° (media circolare
    di 4 volte l'angolo dei tratti fra punti vicini, pesata con la lunghezza); con un'IMU perfetta resta costante, quanto
    cambia è l'errore dell'IMU (mediana su 20 s). Il metodo riconosce i sollevamenti (`m1` a 556 s, `m4` a 233 e 393 s,
    dove l'angolo lo ritrova la ricerca). Fuori dai sollevamenti: `m1` fino a +8,3° in 540 s, `m2` fino a +7,8° e poi
    +2°, `f2` fino a +5,6° e poi ~+1°, `r15` fino a +3,7°, le altre 1-3° di scarto medio. Nessun modello fisso spiega:
    deriva nel tempo (segno che cambia, da fermo −0,3°/h), scala unica (da −0,9% a +0,7% fra missioni, 2-3° di scarto),
    due scale oraria/antioraria (segno di solito uguale ma valori fra 0,1% e 2%, 1-2,7° di scarto). Quindi niente
    taratura: **l'angolo va corretto con il lidar**. Proposta (da decidere con l'utente): nella mappatura, a ogni giro,
    direzione dominante dei muri nella mappa (deve essere 0° a meno di 90°, perché la base sta contro un muro), piccola
    frazione dell'errore per giro tramite `mapping_correction_t`, solo con direzione netta, rotazione lenta e un tetto
    (es. ±10°). Scartato l'allineamento dell'angolo sulla mappa (il 2/11 derivava da solo fino a 12,6°).
    **Scritta da Claude su richiesta dell'utente** (2026-11-07, `correct_theta_with_walls` in `mapping.c`, costanti `WALL_*`,
    chiamata in `update_aligned_pose` prima dei passi su x e y): quarta potenza del versore dei tratti (niente funzioni
    trigonometriche nel ciclo), solo dopo `WALL_MIN_MOVE_M` (5 cm) di strada dall'ultima correzione (da fermo la stessa
    vista ripeteva lo stesso difetto: nella prima prova l'angolo era andato a −2° con il robot fermo 250 s sotto il
    tavolo). Prove nel simulatore (soggiorno, seme 1, 600 s, dove l'IMU è quasi perfetta, quindi l'errore è tutto della
    correzione): errore del metodo per giro −0,3° in media, fra −2,2° e +1,4° (10%-90%), mediana per zona di 1 m² fino a
    ±1,5°; |errore d'angolo| mediana / 90%: senza correzione 0,10° / 0,72°, `WALL_GAIN` 0,05 0,32° / 0,86°, 0,02 0,20° /
    0,63° (consigliato 0,02, non ancora messo nel codice). Errore di posizione: uguale (~3 cm) fino a 7 minuti; con la
    correzione l'ultimo minuto sale a ~9 cm lungo il muro in alto e il divano, zona dove la prova senza correzione in quel
    momento non era (percorsi diversi), quindi non attribuibile con certezza. 0 scadenze mancate. **Prova sul robot da
    fare** con l'utente presente (lì l'IMU sbaglia di 5-8°): missione con `osserva-robot` e analisi dell'angolo corretto.
    **Prove sul robot (2026-11-07)**: `WALL_GAIN` portato a 0,02 e `WALL_MIN_MOVE_M` 5 cm. Analisi con
    `deriva_corretta.py` (scratchpad: direzione dei muri con la geometria di `drv_punto_xy` e scarto del lidar
    14,5°, angolo dell'IMU grezza contro quello pubblicato nel `.oss`; attenzione: il primo frame di una registrazione
    porta l'imbardata della sessione prima, l'avvio del driver la azzera dopo ~0,2 s). `w3` (5 min fino al guasto della
    spazzola): IMU fino a −2,9°, corretto ~0 ± 1°. `w4` (12 min, esplorazione, bordi, rientro, tappeto all'uscita):
    IMU da −1,5° a −6,3° (massimo −5,8° a 510 s), corretto fra −1,2° e +1,7° (−3,1° per un minuto a 510 s), alla fine
    −1,5°; 0 scadenze mancate. **Agganci con rotazioni diverse**: in `w4` il robot si è riagganciato ruotato di ~18°
    rispetto alla partenza (come in `f2`, ~20°): con la mappa salvata l'avvio dopo parte con l'angolo 0 ma ruotato
    rispetto alla mappa, e la correzione dei muri (tetto 10°) non lo recupera. **Da risolvere nel caricamento della
    mappa**: per esempio una prima correzione senza tetto all'avvio (entro ±45° nessuna ambiguità) o la ricerca della
    posizione.
  - **Soglie della spazzola centrale corrette** (2026-11-07, dopo il guasto di `w3` a 346,9 s: le provvisorie 600 e
    1000 mA erano sotto la corrente di lavoro). Correnti vere della centrale a terra: a 60 mediana 0,34-0,54 A,
    minimo su 10 frame fino a 0,77 A (`f2`), 1,44 A una volta in `m4` vicino a un sollevamento, all'accensione fino a
    0,8 A (1,64 in `m4`); al contrario (−100) libera 2,0-2,4 A, picco 3,06 A (prova `rev1`), e con −100 il GD32 **non**
    la ferma dopo ~4,2 s (andata avanti 8,6 s: i 4,2 s valgono per −30). Firmware originale (Ghidra,
    `CFerrariExceptionDetect::detectRollBrush`, `CStallSlipDetect::isRollBrushOvercurrent`): sovraccarico grave con
    corrente > 1,2 A (e un livello a 2,0 A) per il 70% di una finestra; non gira al contrario ma ferma la spazzola per
    5, 10, 60, 300 s, alla quarta volta eccezione. Ora `THRESHOLD_BRUSH_CURRENT` 1200 mA per 35 frame (0,7 s, non
    sarebbe mai scattata in `f2`, `f3`, `m2`, `w3`) e `THRESHOLD_BRUSH_UNLOCK_CURRENT` 3000 mA (stima: una spazzola
    davvero bloccata non è mai stata misurata). In `w4` spazzola accesa 321 s, mediana 511 mA, massimo 1.367 mA,
    nessun allarme. Nuova prova a comando nel driver: `RTES_PROVA_PREMI_DOPO_S` (una pressione del tasto di
    accensione dopo N s, solo nel valore pubblicato).
  - **Task aperiodico nel framework e richieste** (decisioni dell'utente, 2026-11-07). Non un task sporadico di `rt/`
    (avrebbe periodo e scadenza finti, e le scritture sulla flash conterebbero come scadenze mancate) ma un **tipo
    nuovo**, `rt_aperiodico`: thread sul core 3 con lo scheduling **normale** di Linux (`SCHED_OTHER`, nessuna
    priorità real-time: non può affamare i thread del kernel della flash spostati sul core 3 e resta fuori dal limite
    del 95% dei thread real-time), fuori da Rate Monotonic e dalla sorveglianza del driver; `rt_aperiodico_sveglia()`
    non blocca mai, chiamabile da qualsiasi thread, più sveglie durante un job valgono come una (cosa fare lo dicono i
    numeri progressivi nei doppi buffer delle richieste); statistiche: numero di job, tempo massimo di esecuzione e
    **tempo di risposta massimo** (dalla sveglia alla fine del job), niente scadenze. **Tutte le richieste passano da
    funzioni di richiesta** in `tasks/vacuum/aperiodic.c` (`request_map_save(asse)` e, più avanti, ricerca della
    posizione e comandi e zone dell'app): ognuna scrive il suo doppio buffer piccolo e sveglia il task, così chi chiede
    fa una sola chiamata e non può sbagliare l'ordine. Il task aperiodico è l'unico che **esegue** le richieste, da
    qualunque parte arrivino.
  - **App, schema deciso** (2026-11-07, i dettagli si progettano dopo): un **thread di rete a parte** sul core 3,
    scheduling normale, che non fa lavoro vero: i comandi ricevuti diventano chiamate alle funzioni di richiesta del
    task aperiodico, lo stato da mandare lo legge dai doppi buffer che ci sono già. Così una rete lenta o bloccata non
    rallenta mai il salvataggio o la ricerca della posizione. Il robot **non** riceve connessioni dirette dall'app:
    si collega lui a un server, che fa da tramite (l'app deve funzionare anche fuori casa).
  - **Implementazione del task aperiodico e del salvataggio** (2026-11-07, scritta da Claude su richiesta dell'utente,
    da rivedere con lui): `rt_aperiodico` in `rt/` con 4 test; spostamento della flash sul core 3 all'avvio del driver
    (`sposta_flash`, `drv_config_t.cpu_flash`, solo sul robot: sul robot 2 interruzioni e 7 thread spostati);
    `tasks/vacuum/aperiodic.c/h` (task, `request_map_save`, `save_map`, CRC-32, percorso della mappa attiva);
    contatore dei job e `copy_map_categories` in `mapping.c`; caricamento all'avvio (`load_map_if_on_base`, solo
    sulla base) con punteggi `LOAD_SCORE_*`; primo angolo dai muri senza tetto (`is_initial_theta_pending`, sempre,
    anche senza mappa); zone dell'utente caricate aggiunte a quella della base; missione che parte sempre
    dall'esplorazione; asse calcolato e salvataggio a fine esplorazione; salvataggio all'aggancio riuscito;
    `tools/leggi-mappa.py`; `prova.py` usa `build/prove-nav/maps` (o `RTES_MAPS`). Prove: simulatore (salvata 667
    byte, ricaricata, esplorazione finita in 0,1 s, job del task aperiodico 2,4 ms); robot `s1` (esplorazione non
    finita in 240 s: niente salvataggio, corretto), `s2` (salvata 228 × 260 celle; a 236 s guasto della spazzola
    **vero**: alla partenza 2,3 A a 60, al contrario 6,2 A, contro 0,35-0,55 e 2,0-2,4 A con la spazzola libera).
    Il primo angolo dai muri da agganciato: 15,8° in `s1` (aggancio ruotato, recuperato), −1,7° nel simulatore con
    il robot dritto (errore di un giro solo, poi corretto dalla correzione graduale). Da fare: prova del caricamento
    sul robot.
  - **Ricerca della posizione spostata nel task aperiodico** (2026-11-07, scritta da Claude su richiesta dell'utente). La
    mappa è **prestata**: con la posizione persa la mappatura non scrive sulla mappa (il job esce dopo `relocate`: niente
    eventi dei contatti, l'azzeramento di una nuova pulizia aspetta la posizione normale), quindi il task aperiodico la
    legge direttamente, senza copie. La mappatura prepara i punti e chiama `request_position_search()`; il task
    aperiodico esegue `run_position_search()` (in `mapping.c`) tutta in una volta (griglia, albero, rifinitura) e
    pubblica il risultato in un doppio buffer (`search_result_t`: trovata, non trovata, mappa vuota, annullata); fasi
    della mappatura `SEARCH_WAIT_SCAN`, `SEARCH_WAIT_RESULT`, `SEARCH_CANCELLED`. `SEARCH_LOOKUPS_PER_JOB` e
    `SEARCH_BUILD_CELLS_PER_JOB` sono ora i pezzi fra due controlli dell'annullamento (`search_cancel_num`). Un nuovo
    sollevamento annulla la ricerca in corso e ne fa partire un'altra quando arriva il risultato. **Tasti con la
    posizione persa** (decisione dell'utente): tutti e due fanno la stessa cosa, cioè la missione finisce, la mappatura
    annulla la ricerca (`stop_search_num` nel doppio buffer delle celle chiuse) e aspetta la base, la navigazione chiede
    l'aggancio a infrarossi da dove si trova il robot (`start_blind_hook_up`); aggancio non riuscito = guasto
    (`DRV_FAULT_DOCKING_FAILED`). Prove nel simulatore: sollevato e spostato di 2,7 m, ritrovato entro 4,5 cm e 0,4°
    ~4 s dopo il riappoggio; job della mappatura al massimo 0,84 ms (prima fino a ~24 ms sul robot durante la ricerca);
    tasto casa 0,5 s dopo il riappoggio a ~1 m dalla base: aggancio alla cieca riuscito, riposo, posizione di nuovo
    nell'origine; 0 scadenze mancate. Da provare sul robot (serve sollevarlo a mano).
  - **Comando per le zone proibite** (2026-11-07, scritto da Claude su richiesta dell'utente). Thread dei comandi
    (`tasks/vacuum/commands.c`, core 3, scheduling normale, primo pezzo del thread di rete dell'app): legge righe da un
    file speciale (`/tmp/rtes/comandi-vacuum` sul robot, `RTES_COMANDI` sul PC): `zona aggiungi X Y L A` (metri, sulla
    mappa), `zona togli N`, `zone`; tiene la lista voluta (solo se la sessione è partita dalla base) e chiama
    `request_user_zones`. Il task aperiodico controlla le zone (dentro la griglia, al massimo 15), le passa alla mappatura
    (`write_user_zones`), aspetta che siano applicate (`read_applied_user_zones_version`) e salva subito la mappa se un file
    c'è già. La mappatura, nel job e con la posizione normale (`apply_user_zones`), toglie le celle proibite delle zone
    vecchie (`unmark_forbidden_cells`), ripubblica la lista (base + utente) e segna tutte le zone. Le zone dell'utente
    ora cambiano nel job, quindi `copy_map_categories` le copia nella stessa finestra della griglia. `prova.py --comando
    T TESTO`. Prova nel simulatore: aggiungi, elenca, aggiungi, togli, elenca; mappa salvata con la zona rimasta, celle
    proibite giuste (16 della base + 20 della zona), quelle della zona tolta tornate libere o bloccate.
  - **Missioni complete sul robot del 7/11 sera** (`p1`, `p2`, mappa caricata). `p1`: guasto della spazzola a 291 s
    (soglie 1,2 / 3 A): niente avvolto, la spazzola frenata dal **bordo di un tappeto** (fino a 1,8 A avanti, 3,0-3,6 A
    al contrario; libera 0,16-0,25 A dopo la pulizia, al contrario 2,0-2,4; bloccata davvero 2,3 / 6,2 A) → soglie a
    **2.000 / 4.500 mA**. `p2` (25 min): bordi fino a 662 s, **prima pulizia a strisce sul robot** fino a 1.425 s
    (15,5 m² puliti su ~34 m² di celle libere), 0 scadenze mancate, nessun allarme della spazzola. Rientro partito per
    una **falsa batteria bassa**: scendendo dal bordo del tappeto il robot si inclina in avanti (beccheggio fino a
    −7,4°), la spazzola si schiaccia sul bordo (fino a 2,6 A) e la media di 1 s della tensione scende sotto 14,2 V per
    un attimo (a riposo 15,0 V). Poi "nessuna strada per la base": un falso **vuoto** degli anti-caduta davanti alla base
    (569-570 s, superficie scura) aveva bloccato la cella di partenza dell'aggancio. **Corretto in navigazione**: il
    rientro per batteria parte solo dopo `BATTERY_LOW_CONFIRM_TIME` (20 s) di batteria bassa (provato nel simulatore:
    20,7-20,8 s dopo il calo); se la cella davanti alla base non si raggiunge il robot va alla cella libera più vicina
    entro `HOME_IR_DOCK_DIST` (1 m) dal punto di aggancio e da lì chiede l'aggancio a infrarossi (`GOAL_NEAR_HOME`; non
    ancora vista in azione: nelle prove la cella davanti alla base restava raggiungibile). Scartato (decisione
    dell'utente, pericoloso): ignorare il vuoto vicino alla base. **Contraddizione trovata**: `reset_session_data` a ogni
    missione nuova cancella anche il vuoto, quindi il vuoto salvato nel file sparisce alla prima missione (da decidere).
  - **Falso vuoto sul bordo del tappeto: causa e correzione** (2026-11-07). Nella prova `p2` il robot è salito di sbieco
    sul bordo del tappeto davanti alla base: beccheggio +5,6° in salita, poi −5,7° e rollio +6,5° in cima; il laterale
    destro ha visto "vuoto" per ~1 s (il pavimento accanto al tappeto, più basso di dove poggiavano le ruote), e il vuoto
    è entrato nella mappa. **Filtro nel driver** (`contatti_frame`, `CONTATTI_INCLINAZIONE_CDEG` 300): con beccheggio o
    rollio oltre 3° gli anti-caduta non danno eventi (la protezione immediata di driver e GD32 resta); test
    `contatti_inclinato`. Scartato il filtro per lato (decisione presa insieme): nel caso vero il sensore era alzato di
    soli 3-4 mm, non l'avrebbe scartato. **Opzione A** (decisione dell'utente): l'azzeramento di una nuova pulizia
    (`reset_session_data`) toglie solo gli ostacoli bassi, il vuoto resta (e si salva nel file).
  - **Da ragionare più avanti: modalità tappeto e spazzola sporca** (idee dell'utente, 2026-11-07). Riconoscere il tappeto
    dalla corrente della spazzola centrale (pavimento 0,16-0,25 A con la spazzola pulita, bordo del tappeto 0,6-0,8 A) e
    usare `SUCTION_POWER_MAX` (oggi mai usato) sul tappeto. Ma la corrente di base dipende da quanto è sporca la spazzola
    (prima della pulizia del 7/11 sul pavimento 0,35-0,55 A): quindi il software può anche accorgersi che la spazzola è
    sporca, se "vede tappeto" quasi sempre. Prima di scriverlo: misurare la corrente sopra il tappeto (non solo sul
    bordo) e magari leggere come lo fa l'originale (`enable_carpet=1` in `hwdrivers.ini`).
  - **Cosa serve al progetto e cosa è in più.** Serve: task aperiodico, salvataggio e caricamento della mappa,
    spostamenti della flash, ricerca della posizione nel task aperiodico, misure dei tempi con il task aperiodico al
    lavoro e analisi di schedulabilità. In più (l'utente, quando vuole): l'app (comandi, telemetria, disegno della
    mappa) e la compressione della mappa. Le zone proibite servono (la credenza) ma si possono leggere da un file
    all'avvio, senza app.
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
    - **dieci sblocchi esauriti e mobili bassi**: resta il guasto (decisione dell'utente). I mobili bassi non si gestiscono: dove serve l'utente mette a mano una zona proibita.
    - **prossime cose, in quest'ordine** (decisione dell'utente, 2026-11-04): batteria bassa, robot sollevato, `drv_segnala_guasto`, avvio.
    - **aggancio riscritto con gli infrarossi** (2026-11-05, decisione dell'utente: stesso approccio del firmware originale, letto con Ghidra in `CActionTransferMotionGoDock` e `CActionDockAlignState`, non una copia bit per bit). `driver/aggancio.c` non cerca più la faccia col lidar, che con la base nera si confondeva con una scatola: ruota finché trova l'infrarosso, si centra, avanza di fronte seguendo il confine fra i fasci 1 e 2, si ferma col muso a 6 cm (lidar) o al bumper, rifinisce, gira di 180° con l'IMU, arretra di pochi cm. Tolta la rotazione di 180° da `HOOK_UP` (richiesta dell'utente). Lasciati fuori dall'originale gli archi laterali e la manovra col lidar: da aggiungere solo se le prove lo chiedono. Prima prova sul robot (`build/robot/fase3/agg2/`): avvicinamento riuscito (fasci 1+2 su entrambi i ricevitori, muso a 6,3 cm), poi bloccato nella rotazione per un errore di `aggancio.c` nel passaggio dell'IMU da +180° a -180°, corretto, con un test. Correzione provata sul robot il 2026-11-05 (tappeto tolto). Da vicino (`agg3`): agganciato e in carica al primo tentativo. Da lontano (`r2`, ~2,7 m, 0 scadenze mancate): tutto giusto fino al contatto, poi il robot si è staccato. **Due punti aperti**: (1) `aggancio.c` dichiara riuscito al primo frame "in carica", anche se il bit dura 3 frame (l'originale ne chiede 26 stabili), quindi l'esito 2 di `r2` è falso; (2) il driver toglie l'alimentazione ai sensori (`0x9d 00`) dopo 0,5 s di contatto: da vicino c'era già la carica (byte 7 = 01) e il robot è rimasto fermo, da lontano solo il contatto (byte 7 = 02) e subito dopo gli anti-caduta sono andati a 0f, il contatto si è perso e a 81,65 s il robot andava avanti a ~0,14 m/s con comando di velocità zero. Ipotesi sulla causa, non dimostrata: una reazione del GD32 agli anti-caduta. Gli encoder non contano a sensori spenti, quindi il tratto fra 81,07 e 81,62 s non è misurato.
    - **distacco dalla base, ancora aperto** (prova `r3`, 2026-11-05). Corretti l'esito (ora servono 26 frame di carica di fila) e lo spegnimento dei sensori (solo a carica stabile). Non è bastato: in carica per 28 frame di fila, poi `0x9d 00` a 92,36 s e contatto perso a 92,42 s, sempre entro ~60 ms dal comando. Quindi l'ipotesi "manca la carica" è **sbagliata**: il legame è con lo spegnimento dei sensori, che però da vicino (`agg3`) non ha staccato niente. Prova proposta, da fare: non spegnere i sensori mentre il robot è sulla base. A sensori spenti gli encoder non contano, quindi quel tratto non è misurato. **L'esito 2 di `r3` è arrivato prima del distacco: il driver non lo ricontrolla dopo.**
    - **distacco risolto, una prova sola** (`r4`, 2026-11-05). Causa probabile: il driver fermava le ruote al primo tocco e il robot restava a filo (una spinta a mano di pochi mm ha ridato contatto e carica). Ora, come l'originale, dopo il contatto continua ad arretrare a 3 cm/s finché la carica è attiva per 6 frame, al massimo 1 cm (`AGG_F_PREMI`). In `r4`: partito dalla base, ~1,4 m di giro, rientrato, agganciato al primo tentativo, carica stabile per 41 s fino alla fine della registrazione, anche dopo lo spegnimento dei sensori (quindi non vanno tenuti accesi), 0 scadenze mancate. Ripetuta due volte (`r5`, `r6`, fino a ~2,8 m dalla base in `r6`): 3 su 3 agganciato al primo tentativo, carica stabile per oltre 30 s, 0 scadenze mancate, sempre senza tappeto. **Spiegato sotto (errore di posizione)**: agganciato il robot si crede a 14-22 cm dall'origine (`r4` -0,14; -0,03; `r5` -0,18; -0,09; `r6` -0,18; -0,13) invece che in (0, 0). Causa non analizzata; se conta, la ripartenza da sola dopo la ricarica (`LEAVE_BASE`, che usa x ≥ 0,30) partirebbe da una posizione sbagliata.
    - **errore di posizione, cause trovate** (2026-11-05, prove `o1`-`o3`, `c1`, `r5`-`r13`, rigioco dei `.reg` nel codice vero). Il robot agganciato si credeva a 14-22 cm dall'origine. Due cause.
      **(1) Sensori spenti con i motori attivi** (nel driver, corretta). Con `0x9d 00` gli encoder non contano, ma il GD32 regola la velocità con quelli. All'uscita sparivano ~11 cm dall'odometria (`o1`: contatto a -0,108); allo spegnimento sulla base il robot a volte scattava in avanti di ~12 cm, con l'encoder sinistro che segnava -146 tick in 40 ms, staccandosi dai contatti (`r2`, `r10`). Ora c'è il **riposo** nel driver: 5 s dopo l'inizio della carica (20 s di contatto a batteria piena) si spengono lidar, motori (`0x65 00`, come `freeMotorControl` dell'originale) e sensori, in quest'ordine; per uscire o agganciarsi si riaccende tutto e le ruote aspettano sensori accesi e primo giro del lidar. Prove: 13 spegnimenti su 13 senza perdere il contatto né un tick di encoder (`c1`); andata e ritorno a meno di 4 mm dopo il riposo (`o3`). Perché lo scatto sparisce con i motori spenti è dedotto, non dimostrato.
      **(2) Allineamento contro una mappa spostata** (in `mapping.c`, **la scrive l'utente**). I raggi tolgono `LIDAR_PASS_SCORE` anche alle celle del muro appena davanti al colpo, e il massimo del muro si allontana dal robot. Scelta dell'utente: griglia unica, il raggio non toglie punti nelle ultime 2 celle prima del colpo (distanza |dx| + |dy|). Errore finale in y col rigioco: `r8` da 15,6 a 4,4 cm, `r7` da 10,6 a 5,4 cm. Sul robot, su una copia: `r13` (1,9 m) finisce a (0,023; -0,028). Le due griglie separate davano 1,3 e 0,2 cm, scartate dall'utente. Scartati col rigioco: ritardo del lidar, geometria del lidar (stima: `LIDAR_X` -0,10 invece di -0,112, poco effetto).
    - **tappeto e missione lunga** (2026-11-05, `f1`, `f2`). In `f1` spingere e ruotare insieme non ha spostato il robot (ruote che slittano, IMU fermo), e lo sblocco ha chiuso `CHARGING_STATION_CELL`, facendo fallire il rientro. Corretti in `navigation.c`: `CROSS` alterna rotazione sul posto (0,5 s) e spinta (1 s), `close_cell` non chiude mai quella cella, `LEAVE_BASE` usa la posizione corretta dal lidar. Con il tappeto al suo posto, `f2` ha attraversato il bordo, esplorato, fatto il giro dei bordi, è rientrato a 3 minuti e si è agganciato al primo tentativo, 0 scadenze mancate. Una prova sola; le strisce della pulizia (`CLEAN`) non sono ancora mai state provate sul robot.
    - **attraversamento di traverso** (2026-11-05, progetto dell'utente: una ruota alla volta, minimo codice). `CROSS` ora: ruota sul posto di 30° (`OBJ_ROTATE`), spinge dritto a `CRUISING_SPEED` per 3 s, se ancora incastrato prova dall'altra parte; un solo campo (`cross_theta`), l'incastro va sulla mappa solo quando parte lo sblocco (`drv_incastro` chiamato dalla navigazione). Prova `f4` con il tappeto a pochi cm dalla base: incastrato a 6,7 s, girato di ~22°, salito al primo lato (beccheggio +5,7°, poi rollio -3,9°) e fuori a 10 s; poi rientro e aggancio al primo tentativo, 0 scadenze mancate. In corsa (`f3`) il robot passa il tappeto senza incastrarsi. Una prova per caso; **risolto** (detto dall'utente il 2026-11-07).
    - **sollevamento, decisioni dell'utente** (2026-11-05). Il robot sollevato e riappoggiato altrove deve ritrovare da solo la posizione e riprendere; se non ci riesce, deve almeno trovare la base e andarci (lo fa un altro robot della stessa marca). Nel driver è fatto, non ancora provato sul robot: `drv_stato_t.sollevato` scatta con tutti e 4 gli anti-caduta sul vuoto per 3 frame e resta attivo finché rivedono tutti il pavimento per 0,5 s; ruote, spazzole, aspirazione, lidar e motori (`0x65 00`) spenti. L'originale ha `CRelocationLidar` (filtro a particelle su tutta l'area, con fase di conferma) e rileva il sollevamento anche con l'IMU.
    - **movimento a scatti** (misurato 2026-11-05, solo in esplorazione: `r13`-`r15`): 14-18 fermate al minuto, tratto mediano 22 cm (una cella), velocità media ~0,1 m/s contro 0,35 di crociera. Cause lette nel codice: ogni punto è un arrivo da fermo (`movement.c:453`, v = distanza × guadagno), celle non libere percorse una alla volta (`next_turning_point`, `navigation.c:1332`), nessuna rampa di accelerazione, v = 0 sopra 10° di errore (`movement.c:449`). **Risolto** (detto dall'utente il 2026-11-07).
    - **superare tappeti e cavi: la prossima cosa da fare** (decisione dell'utente, 2026-11-05; prima era stata scartata l'idea di programmarlo). Sul robot il bordo di un tappeto a ~25-28 cm dalla base lo ferma mentre esce: le ruote girano a vuoto e il lidar resta fermo (`build/robot/fase3/r1/`). In `LEAVE_BASE` gli sblocchi sono disattivati, quindi dopo 15 s va in guasto; nella prima missione (`m1`) è uscito solo perché la stima ha toccato x = 0,30 appena in tempo, poi lo sblocco ruotando l'ha liberato. Idea dell'utente, da ragionare insieme: invece di prendere velocità, una routine di rotazioni parziali, perché ruotare ha già funzionato sul robot.
    - **sollevamento nei task, progetto approvato dall'utente** (2026-11-06; riusare il più possibile il codice che c'è, niente codice nuovo se non serve). **Mappatura**: fa la ricerca e ha uno stato (normale, posizione persa, da confermare, non ritrovata) che pubblica; legge `sollevato` direttamente dal driver (non `valid`, che resta falso durante la ricerca); fuori da "normale" non scrive niente sulla mappa. Ricerca a fette, in ordine: ultima posizione nota (con l'allineamento che c'è), riappoggiato sulla base (posizione della base, decisione da confermare), tutta la mappa a passi di 10 cm con un numero fisso di posizioni per job, tenendo le 2-3 migliori lontane fra loro (se la seconda vale quasi quanto la prima: ambiguo, non ritrovata). **Conferma** (corretta il 2026-11-06: ruotare sul posto non serve, il lidar vede già 360°): in `MAP_UNCONFIRMED` il robot riprende la missione normalmente (`valid` vero, navigazione invariata), la mappatura allinea e controlla a ogni scansione che almeno l'80% dei punti cada su celle occupate ma **non scrive sulla mappa**; dopo un tratto percorso (costante, ~1 m, da tarare) passa a `MAP_NORMAL`, se il controllo non regge torna a `MAP_LOST` e prova la candidata dopo. `valid` è falso solo in `MAP_LOST`. **Passi 1-3 fatti** (2026-11-06, scritti da Claude su richiesta dell'utente): stato in `mapping_correction_t.state`, `publish_correction` e `resume_after_lift` in `mapping.c` (per ora riappoggiato = si riparte dall'ultima posizione nota, ignorando odometria e IMU misurate in aria, eventi saltati), `is_position_lost` in `movement.c`, in `navigation.c` uscita dalla base annullata se sollevato e ripresa se riappoggiato sulla base. Prova nel simulatore (`prova.py --solleva T D`, stanza vuota): fermo in `WAIT` e ripresa del giro dei bordi, 0 scadenze mancate; spostato a mano al centro della stanza riparte credendosi dov'era (manca la ricerca). **Movimento**: `valid` falso se il driver dice sollevato o se la mappatura non è in "normale". **Navigazione**: ricorda la missione (come `interrupted_mission`), poi riparte dallo stato di pianificazione della missione, non dagli stati a metà strada. **Non ritrovata**: mappa **provvisoria** (precisazione dell'utente): la mappa vera resta salvata (quando ci sarà il salvataggio col server aperiodico; oggi si perde), il robot ne crea una nuova con l'origine dove è stato riappoggiato, esplora con l'esplorazione che c'è guardando gli infrarossi e, appena li vede, chiede l'aggancio al driver; esplorazione finita senza infrarossi: guasto. Agganciato, il robot è nell'origine della mappa vera, che si può ricaricare. Nota: con `valid` falso e zone proibite la sicurezza blocca il robot (`safety.c:228`), quindi nella mappa provvisoria `valid` deve tornare vero. **IMU da sollevato: non affidabile** (prova `imu1`, 2026-11-06, `build/robot/fase3/imu1/`: robot girato a mano di ~90° in un verso e ~90° nell'altro, ~15 s in aria). Il lidar, confrontando la scansione prima e dopo, dà -2,3° (67% dei punti entro 3 cm); l'IMU dà +16,8° (con +16,8° solo il 6% dei punti combacia), quindi ~19° di errore. In aria l'IMU conta ma, nei campioni a 1 al secondo, non supera +39° quando la rotazione vera era ~90°. La ricerca non può prendere l'angolo dall'IMU.
    - **ricerca globale, progetto e misure** (2026-11-06). Algoritmo approvato dall'utente: ramificazione e potatura sulla griglia fine (Olson, "Real-Time Correlative Scan Matching", 2009; come Cartographer), nella mappatura, 4 funzioni (mappe ingrossate, punti, passo della ricerca a blocchi fissi per job, rifinitura), parametri in un blocco di `#define` a parte. Scartato dall'utente l'angolo ricavato dai muri. Sulla base (in carica) la posizione è la base senza ricerca; prime candidate l'ultima posizione nota e la base. Prototipo nello scratchpad della sessione (non in repo), con rigioco dei `.reg` su pseudo-seriali e registratore delle scansioni. Misure: punteggio 2 sulla cella occupata e 1 sulle vicine (il binario perde precisione: fino a 1° nel simulatore), un punto ogni 15 cm entro 4 m (~70 punti), passo d'angolo = cella / portata (~0,7°), 6 livelli, soglia 80%; rifinitura: ±0,4° a passi di 0,1° con 5 passi di `align_scan_step` per angolo. Precisione contro la posizione allineata: simulatore (soggiorno, robot fermo) media 0,3 cm, max 1,4 cm, angolo max 0,47°; robot vero (`f2`, seconda metà della prova) max 1,5 cm e 0,22°. Sui primi 2 minuti di `f2` fino a 12 cm e 3°: la mappa finale è ruotata di ~3° rispetto all'inizio (il robot si è riagganciato ruotato di ~20° secondo il lidar, 22,6° secondo l'IMU: deriva dell'IMU di ~3° in 280 s, da guardare a parte). Nessuna posizione sbagliata in 41 + 115 ricerche; una scansione vera su 41 sotto la soglia dell'80% (al 70% le trova tutte, ma il caso peggiore raddoppia). Tempi sul robot: 22,6 ns per lettura, ricerca in media 2,6 milioni di letture (59 ms), massimo 5,0 milioni (113 ms) su una zona conosciuta di 14 × 10 m; preparazione delle mappe ingrossate 20 ms; rifinitura ~5.400 letture bilineari a 359 ns (1,9 ms). Partire dall'ultima posizione nota toglie solo il 9%. Decisioni dell'utente (2026-11-06): 10 ms di ricerca per job, soglia 80% (meglio fallire ogni tanto che essere imprecisi), niente candidate iniziali (solo in carica = base). **Una sola struttura per tutti i livelli** (idea dell'utente): il valore di una cella cresce salendo di livello (finestre una dentro l'altra), quindi per cella si salvano solo i primi livelli a cui vale ≥ 1, = 2 e contiene una cella libera, 3 bit ciascuno (7 = mai), in un `uint16_t`: 2 byte per cella invece di 14 (casa di 20 × 20 m: 320 KB), risultati identici, lettura +24% sul robot (28,4 ns con una tabellina per livello), ~15 job invece di ~12 nel caso peggiore misurato. Memoria sul PC: nessuna modifica a `rt/rt.c` (proposta sbagliata, ritirata): se il blocco della memoria non riesce il driver lo scrive e va avanti (`driver.c:1289`); provato con 320 KB in più, il programma parte e il blocco riesce ancora.
    - **ricerca globale e conferma scritte** (2026-11-06, da Claude su richiesta dell'utente, tutto in `tasks/vacuum/mapping.c`): costanti `SEARCH_*` e `CONFIRM_*` in un blocco a parte, tipi `search_phase_t` e `search_node_t`, struttura `map_state.search`; `restart_from_pose` (sostituisce `resume_after_lift`), `give_up_search` (per ora `drv_segnala_guasto(DRV_FAULT_POSITION_NOT_FOUND)`, poi la mappa provvisoria), `count_failed_attempt` (3 tentativi, poi si arrende), le 4 funzioni `build_search_grid`, `prepare_search_points`, `search_step`, `refine_search_result` con i loro aiuti, `relocate` (fasi in `MAP_LOST`) e `confirm_position` (`MAP_UNCONFIRMED`, 2 scansioni di fila sotto l'80% scartano, 1 m percorso conferma). La pila è per un angolo alla volta (+3% di letture nel prototipo, pila da 5.105 a 18 blocchi). Prove nel simulatore (soggiorno, missione dalla base, sollevato a 60 s per 6 s, `prova.py --solleva 60 6 --sposta X Y A`): rimesso dove era, spostato di 2,6 m e 66°, spostato di 4,4 m e 144°: trovato ~2 s dopo il riappoggio con 0,5-3 cm e < 0,1° di errore, confermato dopo ~15 s, 0 scadenze mancate. Da fare: tempo dei job della ricerca sul robot (budget calcolato con 28,4 ns per lettura del prototipo), provare un caso di fallimento e di posizione ambigua, mappa provvisoria.
    - **ricerca e server aperiodico** (2026-11-06): l'utente ritiene più sensato, dal punto di vista logico, che la ricerca stia nel server aperiodico (richiesta aperiodica che arriva col sollevamento). **Deciso il 2026-11-07: si sposta nel task aperiodico** (vedi "Task aperiodico" in cima); la conferma resta nella mappatura.
    - **posizione non ritrovata = guasto** (decisione dell'utente, 2026-11-06): niente mappa provvisoria, il robot va in guasto (`DRV_FAULT_POSITION_NOT_FOUND`); tolto `DRV_FAULT_BASE_NOT_FOUND`, e in `MAP_NOT_FOUND` la posizione non è valida. Prove nel simulatore: robot portato in una stanza mai vista (`separata`): 3 ricerche senza risultato, guasto 3,7 s dopo il riappoggio, corretto. **Problema aperto**: stanza vuota 12 × 3 m simmetrica (`simmetrica`, base al centro di un lato), robot appoggiato a un'estremità: la ricerca sceglie la posizione gemella ruotata di 180° (entro 4 m le due estremità sono identiche, punteggi uguali) e la conferma la accetta dopo 1 m (5-8 m di errore, la mappa verrebbe scritta nel posto sbagliato). **Decisione dell'utente**: si accetta, niente controllo dell'ambiguità (proposto: seconda ricerca lontano dalla migliore con soglia "migliore meno un margine", ambigua = tentativo fallito). Rischio residuo da tenere presente nelle prove vere: corridoi lunghi e lisci (ambiguità lungo l'asse). Codici del guasto nei task fatti: `stop_and_fail(motivo)` in `navigation.c` (9 chiamate) e `default` di `movement.c`.
    - **avvio e tasti** (decisioni dell'utente, 2026-11-06). Acceso, il robot aspetta un comando (tasto o, più avanti, app). In riposo sulla base: accensione = parte la pulizia (con esplorazione se la mappa non c'è, cioè se non è stata caricata o fatta in questa accensione), casa = niente. In missione: accensione = pausa / ripresa, casa = rientro subito (la missione finisce; la pressione dopo fa partire una pulizia nuova). Partenza fuori dalla base: come oggi. Fine missione: riposo sulla base fino al comando dopo. Guasto: entrambi i tasti lo azzerano e poi fanno la loro azione; si azzera da solo anche quando il robot viene rimesso sulla base. LED: azzurro fisso in riposo e al lavoro, azzurro che respira in pausa. Nuova pulizia nella stessa accensione: la mappa vale come caricata, ma **si azzera tutto quello che in una sessione non si aggiorna più** (celle troppo strette, quadranti attraversati, celle speciali del vuoto e degli ostacoli bassi, metà pulite); ostacoli e celle libere restano. **Driver fatto** (Claude): pressioni brevi (< 1 s, al rilascio) contate in `drv_stato_t.pressioni_accensione` / `pressioni_casa`, pressione o ritorno sulla base dopo almeno 1 s fuori = tentativo di azzerare il guasto prima di contare la pressione, `DRV_LED_AZZURRO` / `DRV_LED_AZZURRO_RESPIRA`, scenario 8c di `tools/prova-driver.py`. Navigazione e mappatura scritte da Claude su richiesta dell'utente (2026-11-06): `NAV_STATE_PAUSE`, `read_commands`, `apply_commands`, `start_new_mission`, `start_pause`, `go_home_now`, partenza in `SLEEP`, guasto del driver = `FAILURE` della navigazione; nella mappatura `reset_session_data` (chiesto con `navigation_closed_cells_t.mission_num`), `calc_nav_cell_state`, `clear_fine_cell_special_value`, `MAP_NOT_FOUND` che riprova dopo l'azzeramento del guasto. `tools/prove-nav/prova.py` preme da solo il tasto di accensione a 2 s (`--senza-avvio`, `--premi T TASTO`). Provati nel simulatore: avvio col tasto, pausa e ripresa, tasto casa, seconda pulizia con azzeramento (30 quadranti attraversati tolti), guasto azzerato dal tasto e dal ritorno sulla base, 0 scadenze mancate. **Risolto** (detto dall'utente il 2026-11-07): nel soggiorno del simulatore l'aggancio a infrarossi era rimasto 45 s fra ricerca, centratura e avvicinamento senza riuscire (prova con il tasto casa a 55 s).
    - **prove sul robot di avvio, tasti e sollevamento** (2026-11-06 sera, `build/robot/fase3/tasti1/`, `m2/`). Tasti letti anche in riposo sulla base con i sensori spenti (pressioni da 40-130 ms). Missione col tasto: partenza, pausa (LED che respira, spazzole ferme, lidar acceso: voluto), ripresa, tutto giusto, 0 scadenze mancate. **Sollevamento: due problemi.** (1) **Job della mappatura fino a 24,4 ms** invece dei 10 decisi (nessuna scadenza mancata, periodo 200 ms): sul PC, nel rigioco del `.reg`, preparazione della griglia 2,1 ms per job e passo della ricerca 1,5 ms, sul robot ~16 volte di più; i budget (`SEARCH_BUILD_CELLS_PER_JOB`, `SEARCH_LOOKUPS_PER_JOB`) vanno misurati sul robot, e la preparazione è più lenta del prototipo. (2) **Punteggi veri più bassi del simulatore**: rimesso dov'era, la posizione giusta fa 84% (simulatore > 90%); durante la conferma, nel rigioco, 257 scansioni con media 84,7% e minimo 75% (in rotazione a 1,2 rad/s): con la soglia all'80% la conferma scarta posizioni giuste (la mappatura va avanti e indietro fra persa e incerta, e l'aspirazione si accende e si spegne: l'ha sentito l'utente), e spostato di ~2 m la ricerca trova al massimo 79-80%, poi guasto. Tempo dal riappoggio alla posizione: 6,7 s (giro del lidar ~2 s, preparazione ~2 s, ricerca). Decisioni dell'utente: `SEARCH_MIN_SCORE_PCT` 70 (ricerca e conferma), `CONFIRM_MAX_BAD_SCANS` 5, scansioni in rotazione oltre `CONFIRM_MAX_OMEGA` (0,5 rad/s) non controllate in conferma; il tempo dei job della ricerca (~24 ms) va bene così, anche 30 ms. Rigiocando `m2.reg` con questi valori: primo sollevamento, una ricerca (85%), nessuna scansione sotto soglia, confermata dopo 1 m; secondo, una ricerca (80%) in una posizione a ~1,7 m da dove era il robot, plausibile, non verificabile (la registrazione finisce col robot fermo). **Prova sul robot con i valori nuovi** (`build/robot/fase3/m4/`, 2026-11-06 sera): partenza col tasto, rimesso dov'era (ritrovato 4,1 s dopo il riappoggio, 2 cm, angolo 107° invece di 126°: l'utente l'aveva girato), aspirazione ripartita una volta sola; spostato di 2,7 m (ritrovato 4,7 s dopo, ripartito in modo sensato secondo l'utente); pausa, tasto casa, rientro e aggancio al primo tentativo. 0 scadenze mancate in tutti i task, job della mappatura al massimo 23,0 ms. Prova `m3`: l'utente davanti al robot è stato mappato come ostacolo, la zona raggiungibile è diventata minuscola, bordi e pulizia finiti in 19 s e rientro senza strada = guasto `DRV_FAULT_NO_PATH_HOME`. Proposta da discutere: riprovare la ricerca del percorso per qualche secondo prima del guasto. Nota: `tools/osserva-robot` leggeva male il campo `state` della navigazione (`tools/driver-robot.sh` prendeva l'ultimo campo chiamato `state` delle informazioni di debug, quello della mappatura); **corretto** il 2026-11-07: si prende il `state` dichiarato subito prima di `mission`.
    - **ordine del resto del progetto** (decisione dell'utente, 2026-11-06): sollevamento, guasto chiesto da un task (`drv_segnala_guasto`, serve al sollevamento), avvio, server aperiodico, mappa salvata e ricaricata, misure sul robot (tempi, core, schedulabilità), fase 6.
    - **sollevamento: da ragionarci quando si scrive** (prima missione sul robot, 2026-11-05, `build/robot/fase3/m1/`). Tenuto in mano, il robot **non si ferma in modo affidabile**. Il driver lo considera sollevato solo se tutti e 4 gli anti-caduta vedono il vuoto nello stesso frame (`sic_terra_limita`), senza memoria: a 541-543 e 560-566 s un solo sensore (bit 0, il destro) vedeva qualcosa, probabilmente la mano che premeva anche il bumper destro, e le ruote hanno girato in aria ai comandi di rotazione (il bumper blocca solo l'avanti). Il lidar non si ferma mai da sollevato: è stato acceso fino alla chiusura del programma. Proposta da valutare, non fatta (decisione dell'utente: ci si pensa quando si scrive il sollevamento): "sollevato" che resta attivo finché tutti e 4 non rivedono il pavimento per ~0,5 s, ruote, spazzole, aspirazione e lidar fermi nel frattempo. Si lega a `valid` di `movement.c`, sempre vero per ora, e alla ricerca globale dopo un sollevamento.
    - **cucina, problema tenuto** (decisione dell'utente, 2026-11-04): nella prova con partenza dalla base (seme 1) il robot finisce la pulizia sotto il tavolo, le celle chiuse gli tagliano la strada e `RETURN_HOME` va in guasto per mancanza di un percorso. Proposto e scartato dall'utente: attraversare le celle chiuse durante il rientro. Fatto invece il tempo massimo in `ALIGN` (15 s, poi sblocco); **risolto** (detto dall'utente il 2026-11-07).
    - **scadenze mancate** (decisione dell'utente, 2026-11-04): **sul robot qualsiasi scadenza mancata di qualsiasi task deve causare il guasto**, anche una sola e anche di mappatura e lidar. Oggi il driver scatta dopo 3 di fila e solo per i task critici (`driver/driver.c:750`): da cambiare prima delle prove sul robot, senza fermare le prove sul PC, dove manca `SCHED_FIFO`. Nella prima missione sul PC la navigazione ha mancato una scadenza (job da 161,7 ms contro 0,09 ms di media, in `FOLLOW_PATH`): attribuita al portatile, causa non dimostrata.
    - **controllo del piano prima del server aperiodico** (2026-11-07), **fatto** (scritto da Claude su richiesta dell'utente). (1) **Task bloccato = guasto anche per lidar e mappatura**: la scadenza mancata si conta solo a job finito (`rt/rt.c:69`), quindi un job che non finisce non risultava mancato, e per i task non critici "bloccato" era solo un avviso (`driver/driver.c:1023`); ora in `main.c` tutti i task sono critici (soglia di blocco predefinita: 3 periodi, 600 ms per lidar e mappatura, contro job della mappatura fino a ~24 ms). (2) **Sblocco della spazzola centrale** (deciso il 27/10): la sicurezza accende `SAFETY_ERROR_BRUSH` (bit 6) se la corrente della centrale (`correnti[3]`) resta sopra `THRESHOLD_BRUSH_CURRENT` per `THRESHOLD_BRUSH_FRAME_COUNTER` frame (10, ~0,2 s), solo con la spazzola che gira nel verso normale; la navigazione, dove comanda le spazzole, manda `BRUSH_SPEED_UNLOCK` (-100) per `BRUSH_UNLOCK_TIME` (4,5 s: il comando negativo è una manovra del GD32 che si ferma da sola dopo ~4,2 s, `docs/fase2.md` D5), poi torna normale; con le spazzole spente lo sblocco si annulla. **Soglia di 600 mA provvisoria**: la corrente di una spazzola bloccata non è mai stata misurata (accesa ~200 mA, al contrario ~620-750); le soglie dell'originale (`detectNewRollBrushOverCurrent`, tre livelli) sono nel binario, da leggere con Ghidra o da misurare. **Guasto** (`DRV_FAULT_BRUSH_STUCK`, decisione dell'utente): dopo `BRUSH_UNLOCK_MAX_ATTEMPTS` (5) sblocchi di fila con la spazzola ancora bloccata, oppure subito se la spazzola risulta bloccata mentre gira al contrario (soglia a parte, `THRESHOLD_BRUSH_UNLOCK_CURRENT`, 1000 mA, **provvisoria**, con un contatore suo; stesso bit, la navigazione sa se lo sblocco è in corso). Uno sblocco è riuscito se poi la spazzola gira normalmente per `BRUSH_UNLOCK_RESET_TIME` (10 s): allora i tentativi ripartono da zero. Le spazzole spente (pausa, sblocco delle ruote) non annullano lo sblocco: conta il tempo dall'inizio. Provato solo che compila (PC e ARM, nessun avviso). (3) **Rientro senza strada**: c'era già, il robot riprova per `NO_PATH_HOME_TIMEOUT` (30 s) prima del guasto. (4) **Ricerca a tentativi su richiesta dell'allineamento: scartata** dall'utente (inutile: nelle prove l'allineamento non ha mai perso la posizione, la credenza diventa una zona proibita, e sul robot i punteggi in movimento scendono fino al 75%, quindi darebbe falsi allarmi). **Dopo il server**: zone proibite dell'app nella mappa di navigazione (`mark_forbidden_cells` oggi solo per la base all'avvio), prova vera della pulizia a strisce sul robot, deriva dell'IMU (~3° in 280 s, da misurare sulle registrazioni insieme alla mappa salvata).
    - **passo in diagonale sempre permesso** (deciso con l'utente il 2026-11-08, fatto da Claude). Guasto di `p5`
      (`build/robot/fase3/p5/`, `trappola.png`): a fine giro dei bordi il robot era nell'ultima fila di quadranti
      liberi accanto al muro; un urto vero del bumper destro (oggetto basso per terra, invisibile al lidar) ha chiuso
      il quadrante sopra di lui; l'unico quadrante libero vicino era in diagonale, e la ricerca non faceva passi in
      diagonale: dalla cella del robot 1 sola cella raggiungibile, rientro senza strada. Il lidar e la posizione erano
      giusti (verificato ruotando il robot sul posto e toccando il muro col bumper sinistro: `build/robot/fase3/rot1/`).
      Prima la diagonale era esclusa per prudenza (mai misurato un problema). Conto del margine: fra due punti liberi a
      10 cm il centro resta ad almeno 21,5 cm dai centri delle celle fini occupate, in diagonale 20,9 cm (6 mm in meno,
      la stessa distanza che il robot accetta già al centro di ogni cella tutta libera). Cambiato: `mapping.c`
      (`calc_nav_cell_bitmask` non blocca più un terzo quadrante quando restano liberi due quadranti in diagonale),
      `navigation.c` (`get_contact_quads` con le coppie in diagonale fra celle affiancate e la coppia d'angolo fra celle
      che si toccano solo nello spigolo; ricerca su 8 vicini, `SEARCH_NEIGHBOR_SHIFT`; scorciatoie senza il controllo
      dei due punti di fianco). La frontiera dell'esplorazione e i quadranti di bordo restano a 4 vicini. Rigiocata la
      griglia del guasto di `p5`: da 1 a 405 celle raggiungibili. **Prova sul robot** (`p6`, 2026-11-08): il robot è
      ripassato nello stesso angolo di `p5` (urto a 175,9 s) ed è uscito; guasto a 394 s per sblocchi esauriti sotto un
      mobile pieno di cavi (entrato col giro di un bordo, che non usa la ricerca: per l'utente non c'entra, ci va una
      zona proibita). 0 scadenze mancate; navigazione al massimo 223 µs (`p5`: 303 µs). Da fare: prova nel simulatore
      (cucina) quando lo dice l'utente.
    - **bordo del tappeto: celle bloccate invece dell'attraversamento** (analisi della notte 2026-11-08, da decidere
      con l'utente; nessun codice cambiato; `build/robot/fase3/p7/tappeto.png`). In `p7` a 62-93 s, a (1,6-1,8; 0,3-0,4),
      si formano la "T" bloccata (135,128-129) e le celle troppo strette (136,128-130). Ricalcolando gli allarmi della
      sicurezza dalla registrazione (comandi `0x66` e frame dei sensori): (1) 65,9 s **slittamento**, non incastro: il
      robot entra sul bordo curvando (v 0,13, ω 0,75), ruote +0,63 rad/s e IMU −0,42, beccheggio −2,6°; lo slittamento
      va dritto allo sblocco (`should_start_cross` vuole `SAFETY_ERROR_STUCK`), lo sblocco chiude la cella del robot
      (nessun contatto, quindi `close_cell` → troppo stretta) e porta `unlock_attempt` a 1; (2) 67,2 s incastro dal
      rilevatore della mappatura (odometria > 5 cm, lidar < 2 cm in 7 scansioni), ma `unlock_attempt != 0` e
      l'attraversamento non parte: `drv_incastro(1)` segna l'ostacolo basso a 19,5 cm davanti, (1,53; 0,21), che
      allargato di 22 cm fa la T; (3) 75,6 e 83,5 s slittamento in rotazione sul posto a 1,2 rad/s (ruote 1,2, IMU 0,8):
      altri sblocchi e altre celle chiuse. **In `p5`, `p6`, `p7` l'incastro misurato dagli encoder non scatta mai e
      nemmeno lo sforzo dei motori**: solo slittamento (34 volte). Rotazioni sul posto a 1,05-1,2 rad/s: sul pavimento
      IMU/ruote 1,05-1,08 in tutte le zone, sul tappeto (0,5-2,0; 0-0,5) mediana 0,61-0,97 e 10% a 0,57-0,78, cioè sul
      tappeto le ruote strisciano e lo slittamento scatta anche solo girando. Perché altrove il robot passa: arrivando
      dritto non c'è differenza di rotazione (niente slittamento), il primo allarme è l'incastro della mappatura con
      `unlock_attempt` a 0, e l'attraversamento parte. **Strade da discutere**: attraversamento anche con lo
      slittamento in avanti lungo un percorso; lo sblocco per slittamento non chiude la cella e non consuma il primo
      tentativo; lo slittamento durante la rotazione sul posto non come allarme (o rotazione più lenta, o soglia in
      proporzione alla velocità); l'incastro degli encoder che non scatta mai (le ruote girano anche sul tappeto).
      **Correzione decisa dall'utente (2026-11-08 notte, da scrivere)**: con lo slittamento (come con l'incastro)
      mentre il robot va avanti lungo un percorso si prova **subito** l'attraversamento, per un numero massimo di
      tentativi in un `#define`, contati da una variabile di stato nuova (non `unlock_attempt`), con una funzione in
      mezzo; finiti i tentativi senza successo, si fa come oggi (sblocco, cella chiusa, ostacolo basso se è incastro).
      Da decidere quando si scrive: cosa fare con lo slittamento durante la rotazione sul posto (l'attraversamento
      spinge in avanti, lì il robot non avanza) e quando si azzera il contatore dei tentativi.
      **Scritta da Claude il 2026-11-08** (decisioni dell'utente, da rivedere con lui): `should_start_cross` accetta
      incastro **o** slittamento; contatore `cross_attempt` con `CROSS_MAX_ATTEMPTS` (2), contato in `start_cross`,
      azzerato quando un attraversamento riesce, in `start_following_path` e in `start_leaving_base`; il vecchio
      controllo su `unlock_attempt` è tolto (anche `unlock_attempt = 1` in `end_cross_failed`); l'attraversamento è
      riuscito solo se a metà spinta non ci sono né incastro né slittamento. Slittamento durante la rotazione sul posto:
      soglia proporzionale in `safety.c`, `fmax(THRESHOLD_SLIP_W, THRESHOLD_SLIP_RATIO * |ω ruote|)` con
      `THRESHOLD_SLIP_RATIO` 0,5. Ricalcolato sulle registrazioni: p7 da 5 a 2 slittamenti (restano 65,8 s, il bordo
      del tappeto in curva, che ora fa partire l'attraversamento, e 199,5 s con l'IMU a zero), p6 da 25 a 17, p5 da 4 a 3;
      spariti quelli delle rotazioni sul tappeto (75,6 e 83,5 s). Compila su PC e ARM senza avvisi. Da provare sul robot.
    - **prova `p8` sul robot (2026-11-08)**, con le correzioni del tappeto e della carica, mappa caricata: 40 min di
      pulizia a strisce, 3 attraversamenti riusciti, 24 sblocchi, 0 scadenze mancate in tutti i task. Rientro per
      batteria bassa a 2397 s fallito: il robot girava a ~1 m dalla base; preso in mano dall'utente, rimesso sulla base,
      guasto 9 (posizione non ritrovata). Causa: **la mappa scivolava** (muro in basso da y −2,2/−2,4 a −3,4 m, muro a
      destra da ~3,0 a 4,2 m, a scatti di ~20 cm, solo mentre il robot puliva lì vicino), quindi anche la posizione.
      La mappa salvata (`0.rtm`) è stata cancellata dal robot (copia in `build/robot/fase3/p8/mappa-salvata/`).
    - **mappa che scivola: causa e correzione** (2026-11-08, ripassi di `p8` sul PC con `tools/rigioca/`, 10 volte più
      veloce del tempo reale). Punto più basso in cui il robot si crede, 2100-2400 s (muro vero a ~−2,5): codice di
      allora −3,60; raggi di striscio esclusi (ipotesi di Claude, sbagliata) −3,44; allineamento con 20 passi −3,70;
      scansioni ignorate sopra 0,5 rad/s −3,26; mappa ferma dopo 400 s −2,42; **niente scrittura con un ostacolo entro
      0,5 m (idea dell'utente) −2,20**; entro 1 m −2,10 ma solo il 6% delle scansioni scritte e mappa incompleta.
      Quindi l'allineamento regge su una mappa ferma: lo scivolamento nasce dal circolo mappa → posizione → mappa
      quando il robot ripassa tante volte vicino ai muri. **Scritta da Claude** (decisione dell'utente): in `mapping.c`
      `NEAR_OBSTACLE_NO_WRITE_M` (0,5) e `is_near_obstacle` (le celle `LOW_OBSTACLE` del bumper non contano); vicino a un
      ostacolo la scansione serve solo per allinearsi. Le 2 celle prima del colpo restano. Ripasso col codice vero: muro
      in basso fra −2,35 e −2,55 per tutta la missione (resta una deriva lenta di ~15 cm in 40 min), a destra fermo
      fra 2,85 e 3,4; scritto il 24% delle scansioni; a fine missione la stanza è vista come prima (6768 celle fini
      contro 6945). Da provare sul robot, con l'esplorazione da capo.
    - **blocco di ostacoli attorno a (2,6-3,4; −0,4/−2,4)**: segnalato dall'utente come ostacoli che non esistono; c'è
      già nella mappa caricata in `p8` e nel ripasso compare durante l'esplorazione, quindi il lidar vede qualcosa lì.
      Da capire con l'utente cosa c'è fisicamente; legato alla sua idea di pesare i punteggi con la distanza.
    - **prova `p9` sul robot (2026-11-08)**, con la mappa che non si scrive vicino agli ostacoli, esplorazione da capo:
      esplorazione 146 s, giro dei bordi fino a 434 s, strisce fino a 1404 s; muri fermi per tutta la pulizia (muro in
      basso −2,0/−2,2, a destra 2,8/3,2); 5 attraversamenti del tappeto tutti riusciti, 8 sblocchi (2 nelle strisce),
      5 celle troppo strette, nessuna sul bordo del tappeto. Zona proibita rimessa da Claude a 840 s con le coordinate
      di prima (x 1,05-2,50, y 0,95-1,55): l'utente ha deciso di lasciarla. Rientro per batteria bassa in 14 s e
      aggancio al primo tentativo, riuscito 0,5 s dopo il contatto (correzione della carica), LED giallo acceso.
    - **`p9` completa dall'inizio alla fine** (2026-11-08): il ritorno a 1404 s era la **fine della pulizia** (la
      batteria era risalita a 14,40 V a 1318 s), non la batteria bassa; poi carica fino al 100% (16,6 V). 0 scadenze
      mancate in tutti i task (massimi: mappatura 3,15 ms, lidar 1,30 ms, navigazione 1,08 ms, movimento 0,13 ms,
      sicurezza 0,09 ms; task aperiodico 169 ms sul core 3); 664 metà pulite (~13 m²).
    - **prova del sollevamento sul robot, riuscita** (`p10`, 2026-11-08, mappa di `p9` caricata): sollevato due volte e
      spostato di ~4 m, posizione ritrovata ~5 s dopo il riappoggio entrambe le volte (la prima volta il robot è poi
      tornato dritto, 4,3 m in 15 s, al punto dove era stato sollevato); pausa, tasto casa, rientro in 15 s e aggancio
      al primo tentativo; 0 scadenze mancate; task aperiodico al massimo 129 ms.
    - **costo della mappa che non si scrive vicino agli ostacoli, nel simulatore** (2026-11-08, una prova per versione,
      missione dalla base, 25 min): soggiorno con la regola 22 celle chiuse e 19 troppo strette, giro dei bordi 661 s;
      senza 13 e 8, 562 s. Cucina con la regola 86,5% pulito e guasto (nessuna strada per la base), senza 98,5% e
      guasto all'aggancio (68 urti): la cucina va in guasto in tutti e due i casi. Ipotesi: i bordi degli ostacoli
      restano quelli visti da lontano, meno precisi. Decisione dell'utente: si tengono 50 cm; da provare più avanti un
      raggio di 30 cm o la scrittura dei soli colpi vicino agli ostacoli (verifica: ripasso di `p8` più simulatore).
      Attenzione: `tools/prove-nav/prova.py` ricarica la mappa lasciata in `build/prove-nav/maps/` dalla prova prima,
      anche di un'altra stanza: per le missioni dalla base usare una cartella vuota con `RTES_MAPS`.
    - **aggancio storto: possibile miglioramento futuro** (misura del 2026-11-08, decisione dell'utente: per ora resta
      così, è solo estetico). Angolo del robot agganciato rispetto alla stanza, misurato con il lidar sul muro lungo a
      destra (il mobile nero dietro la base quasi non si vede): messo a mano ~0°, aggancio di `p7` ~10° verso destra
      (orario), aggancio di `p9` ~6° verso sinistra (antiorario). Non è costante, quindi una correzione fissa del giro
      di 180° non serve. Causa probabile: la stima del centro del fascio infrarosso (rifinitura in `driver/aggancio.c`),
      mentre il giro con l'IMU è preciso (180,6° in `p9`). Idea: la navigazione, quando chiede l'aggancio, passa al
      driver la direzione "dritta" presa dalla mappa (assi allineati ai muri) da usare al posto di "fascio + 180°";
      ~30 righe. Dubbio: dopo l'aggancio di `p9` la mappa credeva il robot girato di 11° contro i 6° del lidar.
      Punti e disegno in `build/rigioca/punti/` (`agganci.png`), strumento `tools/rigioca/`.
    - **bit "in carica" del GD32 = carica finita, non carica in corso** (registrazione della notte 2026-11-08,
      `build/robot/fase3/carica1/`, driver senza motori, 13,5 h). Arrivato sulla base al 46% (15,43 V), il robot ha
      caricato subito con il bit spento: tensione fino a 16,70 V in ~1 h, poi ferma; bit acceso solo a 6550 s (1 h 49 min),
      poi la tensione è scesa a 16,50 V e lì è rimasta. Il 5/11 (arrivo all'87-94%) il bit si accendeva in pochi secondi, il
      3/10 (ferma a 16,3 V) era sempre acceso: tutto torna se vuol dire "carica finita". La carica in corso la dice il bit
      **contatto** (tensione che sale ogni volta che è acceso, che scende appena si stacca). La batteria arriva a 16,70 V:
      non è degradata come si pensava il 3/10. Nel driver il bit è usato come "in carica" per LED, scritta del registro,
      `FRAME_CARICA` dell'aggancio (con la batteria scarica l'aggancio riesce solo dopo i 15 s di contatto stabile) e
      blocco dei motori per batteria critica. **Proposta, da decidere con l'utente**: "in carica" = contatto, il bit
      diventa "carica completa". I task non lo usano. Stima nostra: 90% dopo 77 min, 100% dopo 93 min.
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
      sblocco che esistevano già. Sotto ~0,12 m/s non arriva ai 5 cm. Soglie da ricontrollare sul robot: **risolto** (detto
      dall'utente il 2026-11-07).
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
- **Tipo di server aperiodico**: deciso il 2026-11-07, nessun server ma un task aperiodico sul core 3 (vedi "Task aperiodico" in cima). **Tipo di app** (con l'home server, più avanti:
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
- **Guasto chiesto da un task** — FATTO il 2026-11-06: `drv_segnala_guasto(drv_task_fault_t)` in `driver/driver.h`: motivi in un enum (scelta dell'utente), testo da `drv_task_fault_text` (switch con `-Wswitch-enum` reso errore per quella funzione: un valore senza il suo caso ferma la compilazione, verificato con gcc del PC e del robot; default = "generic fault"), codice in `drv_stato_t.task_fault`, guasto al tick dopo, si esce con `drv_azzera_guasto`; comando `guasto N` di `rtes-driver`, scenario 8b di `tools/prova-driver.py`. Progetto originale (2026-10-10): oggi un task non può mandare il driver in guasto (`entra_in_guasto` è
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
5. **Task aperiodico e app** — task aperiodico sul core 3 (salvataggio della mappa, ricerca della posizione), poi l'app (comandi e telemetria, in più).
6. **Installazione persistente e relazione** — avvio automatico del nostro software con ritorno all'originale,
   eventuale rilancio automatico, SSH solo con chiave, documentazione finale.
   **Verifica dei software attivi** (richiesta dell'utente, 2026-11-12): rifare sul robot vero l'elenco di tutto
   ciò che gira (processi, script in `/etc/rc.d`, porte aperte), disattivare gli inutili e decidere insieme cosa
   personalizzare. Si disattiva solo dopo l'OK dell'utente. Elenco preliminare, fatto sul backup (`Estratto/nandd-rootfs`):
   - **stack di fabbrica** (`S90robotManager` → Monitor → AuxCtrl, RobotApp, everest-server, log-server): parte a
     ogni accensione e suona le voci originali; lo sostituisce il nostro avvio. Con lui sparisce anche il
     **collegamento al cloud del produttore** (RobotApp verso 43.158.121.228:9090);
   - **aggiornamenti del produttore**: `/usr/sbin/upgrade` (`S95robotUpgrade`, sempre attivo, anche col nostro
     driver; scarica pacchetti via HTTP), `auto_upgrade_status=1` in `device_config.ini`, `S99aw_upgrade_autorun`:
     da disattivare, potrebbero sovrascrivere il sistema;
   - **`sayeye`** (socket in `/dev/socket`) e **`healthd`**: servizi di fabbrica mai analizzati, da capire prima di
     decidere;
   - da tenere: `wifiManager`, dropbear, adbd; da valutare: cron, dnsmasq, sysntpd, log;
   - **audio personalizzati**: l'utente vuole suonare i suoi mp3 nelle occasioni che decide lui. `tinyplayer`
     suona gli mp3 (gli originali sono MP3 44,1 kHz stereo 256 kbps), il volume si regola con `amixer`. Oggi anche il
     nostro guasto usa un file originale (`sound_erro_harefault`, percorso fisso in `driver/driver.c`). Da fare: i file
     in una cartella persistente (es. `/mnt/UDISK/audio/`), il percorso del driver, le chiamate `drv_suono` nei task.
     Limiti del driver attuale: un solo suono in attesa (se ne arriva un altro prima che parta, il primo si perde), e
     il thread di servizio aspetta la fine del suono, quindi stampa degli eventi e registrazione si fermano per tutta
     la durata (con file lunghi o ravvicinati serve una coda di suoni).

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
| Salvataggio della mappa, ricerca della posizione, comandi dell'app | task aperiodico sul core 3 | evento (nessun budget: core tutto suo) | richieste dei task e dell'app |
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
