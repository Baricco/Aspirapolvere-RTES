# Fase 1: piano dei passi successivi (dal 2026-10-02)

Situazione: i passi 1 e 2 del reverse sono fatti (`docs/protocollo.md`). Per completarlo servono catture **senza
perdita di byte** e prove sui sensori. strace non va bene: AuxCtrl legge un byte per chiamata e, rallentato da
strace, perde il 26–34% dei frame in ricezione.

Ordine di lavoro:

| Blocco | Cosa | Serve il robot | Durata stimata |
|---|---|---|---|
| **A** | strumento `gd32-spia` in C (parser, proxy, modalità autonoma) e test sul PC | no | 2–3 h |
| **B** | accesso comodo: Wi-Fi e avvio automatico persistenti, con modo per tornare indietro | sì | 1–2 h |
| **C** | sul robot: verifiche preliminari, catture con il proxy, prove dei sensori | sì | 1–2 h |

Regole che valgono per tutto il piano:
- nessuna scrittura persistente sul robot senza il tuo OK esplicito, passo per passo;
- prima di ogni scrittura persistente si annota com'era il file e come si torna indietro;
- i test (`./rtes.sh test`) devono passare prima di ogni commit;
- ogni scelta e ogni problema si annotano nel diario (`docs/fase1.md`, da creare come `fase0.md`).

---

## Blocco A: `gd32-spia` (sul PC, senza robot)

*Stato al 2026-10-02: FATTO, ed esteso a tutti i canali (GD32 e lidar). Codice in `gd32/`, `lidar/`, `spia/`;
il formato del log è in `spia/registro.h` (con un campo canale in più rispetto a A2); la decodifica la fa
`spia leggi`/`riepilogo` in C con le stesse librerie, invece di `estrai-frame.py --spia`. Manca la modalità
`solo` (A4). Diario in `fase1.md`.*

### A1. Libreria del protocollo `gd32/` con unit test
È la base del driver della fase 2, quindi va scritta bene fin da subito.

- `gd32/gd32.h`, `gd32/gd32.c`:
  - `gd32_checksum(const uint8_t *b, size_t n)`: somma big-endian a parole, ultimo byte dispari in XOR;
  - `gd32_componi(uint8_t cmd, const void *dati, size_t n, uint8_t *out)`: costruisce un frame;
  - **deframer a stati**, che riceve i byte a blocchi qualsiasi (anche spezzati a metà frame) e restituisce
    frame completi. Conta i checksum sbagliati e i byte scartati, e si risincronizza su `FA FB`;
  - costanti con i codici `cmd` (`GD32_HEARTBEAT = 0x06`, `GD32_VELOCITA = 0x66`, `GD32_SENSORI = 0x15`…);
  - decodifica dei campi noti del frame `0x15` (batteria, carica) in una struct.

  Niente allocazioni dinamiche e nessuna dipendenza da Linux: il codice si potrebbe usare così com'è anche su un
  microcontrollore.
- `test/test_gd32.c`. Controlli da coprire:
  - i checksum dei frame veri presi dalle catture (heartbeat `FA FB 03 06 00 06`, velocità zero, un frame
    `0x15` completo);
  - un frame spezzato in tutti i modi possibili, anche byte per byte;
  - frame troncati come quelli di strace (52 byte invece di 102): il deframer deve scartarli e risincronizzarsi;
  - un `FA FB` dentro i dati;
  - `len` < 3;
  - ricostruzione del flusso di una cattura vera: stessi numeri di `tools/estrai-frame.py`, cioè 0 errori in
    trasmissione.

**Fatto quando:** test verdi sul PC e sotto ARM (sul robot, nel blocco C).

### A2. Formato del log
Un file binario semplice, così il programma sul robot scrive poco e non perde tempo:
```
record: int64 tempo_ns (CLOCK_MONOTONIC) | uint8 direzione (0 = GD32→SoC, 1 = SoC→GD32) | uint16 n | n byte
```
Si registrano i **blocchi letti così come arrivano**, non i frame. Se il parser avesse un errore, i dati grezzi
restano intatti e si possono rianalizzare.
- `tools/estrai-frame.py` impara a leggere anche questo formato (`--spia`), così catalogo e CSV si
  producono come oggi.
- Il file si scrive in `/tmp` (RAM, ~400 MB liberi). A ~6 KB/s, un'ora pesa ~22 MB. Si scarica a fine cattura.

### A3. Programma `gd32/spia.c`, modalità `proxy`
```
AuxCtrl ⇄ /dev/ttyS3 = pty (bind mount) ⇄ spia ⇄ /tmp/ttyS3-vero (mknod, stessa UART) ⇄ GD32
```
1. crea `/tmp/ttyS3-vero` con `mknod`, con gli stessi major/minor di `/dev/ttyS3` (letti da `stat`);
2. apre la UART vera: 115200 baud (da `getGDData` in AuxCtrl, `0x3e6dc`), 8N1, raw;
3. apre una pty (`posix_openpt`) e la mette in modalità raw;
4. `mount --bind <pty slave> /dev/ttyS3`. Da quel momento chi apre `/dev/ttyS3` apre la pty;
5. ciclo con `poll()` sui due fd: legge blocchi fino a 4 KB, li inoltra dall'altra parte e li scrive nel log;
6. gira con `SCHED_FIFO` e `mlockall()` (riusa `rt/`), così non resta mai indietro;
7. all'uscita (anche con Ctrl-C o SIGTERM) toglie il bind mount (`umount /dev/ttyS3`).

La sequenza d'uso sul robot: si ferma AuxCtrl, si avvia `spia proxy`, si riavvia AuxCtrl (o tutto lo stack).
Per sicurezza, `spia` all'avvio controlla che nessuno tenga aperta `/dev/ttyS3` (scorrendo `/proc/*/fd`), e
altrimenti si rifiuta di partire.

**Rischio da verificare nel blocco C:** AuxCtrl ha `getCTS`/`getDSR`/`getDCD`, che leggono le linee di
controllo della seriale (`ioctl TIOCMGET`). Su una pty queste chiamate falliscono. Se AuxCtrl le usa davvero, il
proxy deve rispondere al posto suo: è più complicato, si valuta solo se serve.

### A4. Modalità `solo`
Il programma prende il posto di AuxCtrl:
1. apre la UART vera e manda l'handshake di avvio visto nelle catture (`0x0c 01`, `0x07`, `0x17 01 80 cf f0`,
   `0x18`, `0x71 00 00 00 00`, `0x65 00`, `0x19 01`, `0x7c 00 ff ff`). Si mandano gli stessi byte di AuxCtrl,
   senza inventare nulla;
2. task periodici `SCHED_FIFO` con `rt/`: **heartbeat `0x06` ogni 200 ms** e **`0x66` a velocità zero ogni
   20 ms**, come AuxCtrl. È anche il primo uso vero della nostra libreria dei task;
3. registra tutto nel log e, con `-v`, stampa a schermo i campi del frame `0x15` **che cambiano** (diff rispetto
   al frame precedente). È ciò che serve per le prove "premi il bumper e guarda cosa cambia";
4. **la modalità `solo` non manda mai comandi ai motori diversi da zero.** I comandi di movimento arrivano
   nella fase 3, con il robot sollevato.

### A5. Prove sul PC
- **Proxy:** uno script Python (`tools/gd32-finto.py`) apre una pty con `os.openpty()`, che fa la parte della
  "UART vera", e simula il GD32: risponde all'handshake e manda frame `0x15` ogni 20 ms. Non serve `socat`, che
  sul PC non c'è. Si verifica che il log
  contenga esattamente i byte passati. Il bind mount richiede root: questa prova si fa con un percorso diverso
  da `/dev/ttyS3` (opzione `--dispositivo`).
- **solo:** contro il GD32 finto: handshake completo, periodi di heartbeat e velocità rispettati (si misurano
  dai timestamp del log).

**Fatto quando:** commit con `gd32/`, test, `spia` compilato anche per ARM, `estrai-frame.py --spia` e
`docs/fase1.md` con il diario.

---

## Blocco B: accesso comodo (con il robot)

*Stato al 2026-10-02: FATTO (B0 = stack originale + adb, B1–B3), B4 rimandato a fine progetto. Vedi `fase1.md`.*

Obiettivo: accendi il robot e dopo un minuto è raggiungibile in SSH, senza USB e senza `avvia-robot.sh`.

### B0. Da decidere insieme prima di iniziare: cosa parte all'avvio
La notte scorsa ha mostrato che **la batteria la carica solo RobotApp**. Con solo AuxCtrl il robot sulla base si
scarica fino alla soglia di protezione e si riavvia. Le opzioni:

| Opzione | Cosa parte all'avvio | Pro | Contro |
|---|---|---|---|
| 1 | solo AuxCtrl (come `avvia-robot.sh`) | ambiente "pulito" per le misure | la batteria non carica: va riavviato lo stack a mano per caricare |
| 2 | stack originale completo (ripristino di `S90robotManager`) | carica, comportamento di fabbrica | RobotApp spegne adb (unico accesso di emergenza se il Wi-Fi non va) |
| **3** | stack originale + nostro script che riaccende adb dopo RobotApp | carica, Wi-Fi e adb insieme | da verificare che riaccendere adb funzioni |

Proposta: **opzione 3**, con due script per cambiare modalità durante il lavoro: `modo originale` (carica) e
`modo sviluppo` (ferma lo stack, avvia solo AuxCtrl o `spia`). Se nella prova adb non si riaccende, si ripiega
sull'opzione 1.

### B1. Indagine (solo letture)
1. Chi gestisce `wpa_supplicant`: `/bin/wifiManager` (avviato da `S90robotWifiManager`), oppure è avviato da
   `/etc/init.d/network`? Con quale file di configurazione?
2. Se `wpa_cli save_config` scrive in `/etc/wifi/wpa_supplicant.conf` (c'è `update_config=1`) e se
   wifiManager lo sovrascrive al riavvio.
3. Com'è configurato `udhcpc` sulla `wlan0` all'avvio.
4. Il router può assegnare al robot un **IP fisso** (DHCP reservation sul MAC): così `.robot_ip` non cambia
   più. Lo imposti tu dal pannello del TL-WR840N.

### B2. Wi-Fi persistente (scrittura, con OK)
In base a B1, la strada più semplice:
1. la rete di casa si aggiunge a `wpa_supplicant.conf` (PSK derivata, mai la password in chiaro);
2. prima si salva una copia: `wpa_supplicant.conf.orig`;
3. si prova con un riavvio: il robot deve tornare in rete da solo;
4. **per tornare indietro:** si ripristina `wpa_supplicant.conf.orig`.

### B3. Avvio automatico (scrittura, con OK)
Script nostro `/etc/init.d/rtes-avvio` (link `S91rtes-avvio`), installato da `./rtes.sh install-accesso`:
- parte **solo se esiste `/mnt/UDISK/rt/enable`**. È l'interruttore di sicurezza: si cancella il file via
  SSH o adb e al riavvio successivo il robot torna come oggi;
- applica la modalità scelta in B0;
- **per tornare indietro:** `./rtes.sh uninstall-accesso`, che rimuove script e link.

### B4. SSH solo con chiave
In `/etc/config/dropbear`: `PasswordAuth 0`, `RootPasswordAuth 0`. Va fatto **per ultimo**, solo dopo aver
verificato che l'accesso con la chiave funzioni dopo un riavvio. Per tornare indietro: si rimettono i valori a 1.

**Fatto quando:** robot spento e riacceso senza USB, e in SSH entro ~1 minuto. Aggiornati `LEGGIMI.md` e
`avvia-robot.sh` (che diventa solo un piano di riserva).

---

## Blocco C: sul robot

### C1. Verifiche preliminari (pochi minuti)
- `stty -F /dev/ttyS3 -a` con AuxCtrl attivo: velocità e flag reali (ci aspettiamo 115200 8N1 raw);
- `cat /proc/tty/driver/*`: contatori di overrun della UART (se il driver li espone);
- **strace leggero solo su `open` e `ioctl`** al riavvio di AuxCtrl (`strace -f -e trace=open,ioctl`): niente
  `read`/`write`, quindi nessun rallentamento. Mostra se AuxCtrl usa `TIOCMGET` (rischio del proxy, A3) e i
  parametri `termios`;
- `scaling_cur_freq` durante `bench/carico`, per escludere il throttling termico (punto rimasto dalla fase 0);
- unit test di `gd32/` sul robot (ARM).

### C2. Catture con il proxy
Con lo stack originale e la batteria abbastanza carica:
1. **avvio completo** (handshake) con il proxy attivo prima di AuxCtrl;
2. **a regime sulla base**, 10 minuti: periodo vero dei frame `0x15`, 0 checksum sbagliati;
3. **una pulizia breve** avviata dal tasto, fuori dalla base, con il robot sorvegliato: comandi motori, spazzole,
   aspirazione, lidar acceso. È la cattura più ricca per il reverse.

### C3. Prove dei sensori con `spia solo -v`
Robot fermo, motori sempre a zero. Un'azione per volta, annotando ora e azione:
- bumper sinistro, destro, entrambi;
- ruota sinistra sollevata, destra sollevata;
- ruote girate a mano in avanti/indietro (encoder);
- sensori cliff coperti/scoperti (robot sollevato);
- cassetto della polvere tolto/rimesso;
- tasti;
- robot appoggiato/tolto dalla base.

Ogni campo trovato va nella tabella di `docs/protocollo.md`, con [osservato] al posto di [ipotesi].

### C4. Heartbeat
Con `spia solo`: allungare gradualmente il periodo di `0x06` (200 → 500 → 1000 ms…) e poi di `0x66`, per
capire quale dei due sorveglia il GD32 e con quale timeout. Un reset costa solo un riavvio, che dopo il
blocco B è innocuo.

---

## Cosa resta fuori da questo piano
- **(superato il 2026-10-02)** il lidar e gli altri componenti NON restano fuori: la spia deve coprire tutti i
  canali (vedi `fase1.md`, inventario dell'hardware);
- i comandi dei motori diversi da zero: fase 3, con il robot sollevato.
