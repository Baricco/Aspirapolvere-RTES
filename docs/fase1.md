# Fase 1: diario

Il piano è in `fase1-piano.md`. Qui annoto, in ordine di data, le scelte fatte, cosa ho trovato e i problemi.

## 2026-10-02 — accesso comodo: decisione su cosa parte all'avvio

**Decisione:** all'avvio parte lo **stack originale completo** (Monitor → AuxCtrl, RobotApp…), e adb resta
acceso. È una soluzione **provvisoria**: serve solo perché la batteria la carica RobotApp. Quando avremo scritto la
nostra gestione della batteria, il software originale non servirà più e questa scelta andrà rivista.

## 2026-10-02 — accesso comodo: indagine (solo letture, robot acceso da 18 h con lo stack originale)

### Wi-Fi
- `wpa_supplicant` lo avvia **`/bin/wifiManager`** (`S90robotWifiManager` → `wifiManager -D`) con
  `-c/etc/wifi/wpa_supplicant.conf`, attraverso lo script `/etc/wifi/wifi`. Anche `udhcpc` lo avvia wifiManager
  (`/etc/wifi/udhcpc_wlan0 restart` → `udhcpc -i wlan0 -S -T 10`) quando la connessione è stabilita.
- `/etc/wifi/wpa_supplicant.conf` oggi contiene solo `ctrl_interface` e `update_config=1`, senza nessuna rete.
  Ha la data 18/10/2024 e non ha la riga di commento del modello in `/rom/etc/wifi/`: lo ha riscritto
  `wpa_supplicant` (SAVE_CONFIG) dopo aver tolto le reti. Probabilmente è stato un vecchio reset della rete fatto dall'app.
- wifiManager contiene `/rom/etc/wifi/wpa_supplicant.conf`: è il modello che si copia quando il file manca
  (comportamento standard della libreria Wi-Fi di Allwinner). Sa anche fare `REMOVE_NETWORK` e `SAVE_CONFIG`
  quando l'app collega il robot a una nuova rete.
- La rete "WiFi Baricco" è oggi la rete 0 di wpa_supplicant, ma **solo in RAM** (aggiunta da `Script/wifi-robot.sh`).
- `/etc` sta sull'overlay ext4 (`rootfs_data`), quindi un `wpa_cli save_config` resta dopo il riavvio.

**Conclusione:** per avere il Wi-Fi all'avvio dovrebbe bastare `wpa_cli save_config` (dopo aver salvato una copia
del file). Al riavvio wifiManager avvia wpa_supplicant con quel file, e wpa_supplicant si collega e lancia udhcpc.
Lo verifica la prova di riavvio. Il rischio da tenere d'occhio è che wifiManager tolga la rete se non la riconosce.

### adb e stack originale
- Con lo stack originale attivo da ore, `/sys/class/android_usb/android0/enable` vale **1**: adb funziona.
- In AuxCtrl e RobotApp `echo 0 > …/android0/enable` sta dentro `CSerTCPSocket::closeAdb()`, un comando del
  **socket di fabbrica** (`[CSerTCPSocket] close adb !`, insieme a `openAdb`/`readAdb` e ai comandi
  `FACTORY_*`). Non è un'azione automatica all'avvio. L'ipotesi annotata il 2026-09-30 ("RobotApp spegne adb")
  era sbagliata.
- `/etc/init.d/adbd` (uguale a quello in `/rom`) accende adb all'avvio. `/bin/adb_shell`, aggiunto il
  2026-09-30, rende utilizzabile la shell.

**Conclusione:** probabilmente non serve nessuno script per riaccendere adb: basta rimettere il link
`S90robotManager`. Lo verifica la prova di riavvio. Se adb si spegnesse, si aggiunge lo script previsto dal piano.

### SSH
- `/etc/config/dropbear`: `PasswordAuth on`, `RootPasswordAuth on`, porta 22. La chiave è in
  `/etc/dropbear/authorized_keys`.

### Seriale del GD32 (per il programma spia)
- `/dev/ttyS3`: major 252, minor 3. In questo momento la tiene aperta solo AuxCtrl.
- `/proc/tty/driver/` (UART 3) dopo 18 h e mezza: tx 38 186 675 byte, rx 343 420 498 byte (circa 5,2 KB/s in
  ricezione), **nessun overrun** (il contatore `oe` non compare), 2 errori di framing (`fe:2`).
- Sul robot `stty` non c'è: i parametri della seriale si leggeranno con un nostro programma (`tcgetattr`).
- CPU0 a 1,2 GHz e 47 °C: nessun throttling a riposo.

## 2026-10-02 — accesso comodo: modifiche fatte (con l'OK dell'utente) e prova di riavvio

Modifiche permanenti sul robot, ognuna si può annullare:

| Cosa | Come tornare indietro |
|---|---|
| copia `/etc/wifi/wpa_supplicant.conf.orig` (49 byte, md5 `3a978273…`) | — |
| `wpa_cli save_config`: "WiFi Baricco" salvata in `wpa_supplicant.conf` (psk come chiave derivata da 64 cifre esadecimali) | `cp wpa_supplicant.conf.orig wpa_supplicant.conf` |
| link `/etc/rc.d/S90robotManager -> ../init.d/robotManager` rimesso | `rm /etc/rc.d/S90robotManager` |

Sul PC: `Script/trova-robot.sh` trova l'IP dal MAC Wi-Fi del robot (`54:f1:5f:0a:c5:8e`) usando la tabella ARP.
Per riempirla manda un datagramma UDP a ogni indirizzo della /24: il kernel fa la richiesta ARP prima di spedirlo e
non servono i permessi di root. `ssh-robot.sh` lo usa quando `ROBOT_IP` non è impostato. L'utente non vuole un IP
fisso sul router. Tempi: 0,3 s se l'IP salvato è ancora giusto, 0,7 s con la scansione.

Prova di riavvio (`reboot` alle 18:41:14, robot sulla base con il cavo USB collegato):
- adb di nuovo disponibile dopo 21 s, Wi-Fi e SSH dopo 27 s (robot acceso da 15 s), stesso IP 192.168.1.102;
- Monitor, AuxCtrl, RobotApp e wifiManager partiti da soli; `android0/enable` = 1 per tutta la prova;
- nessun riavvio in più di 6 minuti (la soglia dell'heartbeat è circa 4 minuti);
- durante l'avvio il PC vede per 3 s il dispositivo USB `1f3a:efe8` (Allwinner, bootloader), poi `18d1:d002` (adb);
- a circa 270 s il server adb del PC ha perso il robot senza nessuna disconnessione USB nel log del kernel.
  Lo ripristina `adb kill-server; adb start-server`: è un problema del PC, non del robot.

Da fare ancora per l'accesso comodo: SSH solo con chiave (`/etc/config/dropbear`), dopo un altro riavvio di prova.

## 2026-10-02 — inventario dell'hardware

Fatto solo con letture sul robot e sul disassemblato: risultato in `hardware.md`. Cose nuove:
- il **motore del lidar lo comanda il GD32** (`0x71` = potenza 0–100, `controlLidarPwm`), il lidar manda i dati su
  `/dev/ttyS1` a 115200;
- **GPIO 233** accende l'amplificatore dell'altoparlante; l'audio lo suona `tinyplayer`, il volume si regola con
  `amixer`;
- c'è un orologio con batteria tampone (PCF8563 su I2C-1, `/dev/rtc1`);
- tasti e LED del tasto passano probabilmente dal GD32 (nessun LED o tasto Linux usato dai programmi);
- RobotApp è sempre collegato al cloud del produttore (TCP verso 43.158.121.228:9090) e log-server cerca di
  caricare i log via FTP.

Su richiesta dell'utente lo scopo è **usare tutto l'hardware**, quindi la spia copre tutti i canali.

## 2026-10-02 — blocco A: librerie e spia

- `gd32/`: checksum, costruzione dei frame, deframer con risincronizzazione (dopo un frame rovinato scarta un
  solo byte e rianalizza il resto, così un frame troncato non fa perdere quello che lo segue), codici, batteria.
  41 controlli, tra cui 3000 frame casuali mescolati a rumore e frame troncati.
- `lidar/`: ricavato dal disassemblato (sezione "Lidar" di `protocollo.md`) e verificato sui byte veri: formato
  `AA`, checksum a somma o CRC-16/MODBUS (le tabelle di AuxCtrl sono quelle standard), codici `0xA9`–`0xAE`.
- `spia/`: `spia proxy` (pty + bind mount, multi-canale, SCHED_FIFO), `leggi`, `riepilogo`, `termios`; formato
  del registro in `registro.h`. Prova completa sul PC con periferiche finte (`tools/prova-spia.py`, ora dentro
  `./rtes.sh test`): ogni byte arriva intatto in entrambe le direzioni e il registro coincide.
- Verifica nel disassemblato del rischio annotato nel piano: AuxCtrl **non** chiama mai `getCTS`, `getDSR`,
  `getDCD`, `getRing`, quindi la pty gli basta.

Problemi incontrati:
- sul robot `/tmp` è montata `nodev`: il nodo della UART vera creato lì con `mknod` non si apre
  ("Permission denied"). Ora si crea in `/dev` (tmpfs senza `nodev`);
- con la musl e il kernel 3.4 `cfgetispeed()` restituisce 0: la velocità si legge dai bit `CBAUD` di `c_cflag`.

## 2026-10-02 — prima cattura con la spia (robot sulla base, stack originale)

`tools/spia-robot.sh avvia`: lo stack originale è ripartito attraverso la spia senza accorgersene.
- `spia termios`: `/dev/ttyS3` e `/dev/ttyS1` a 115200 8N1 raw, senza controllo di flusso.
- **0 errori di checksum** in entrambe le direzioni (con strace il 26–34% dei frame era troncato).
- Frame dei sensori `0x15` **ogni 20,0 ms** (min 17,6, max 22,4); heartbeat del GD32 `0x06` **ogni 2,0 s**;
  heartbeat del SoC ogni 204 ms; `0x0d` dal SoC ogni 3,0 s.
- Batteria: `dati[8] = 0xa5` = **16,5 V**, bit "in carica" a 1 per tutta la cattura: la batteria (Li-ion 4S,
  16,8 V piena) è quasi carica. Questo conferma il messaggio `isBatteryFull` di RobotApp.
- Lidar: all'avvio dello stack il GD32 lo accende per un attimo e il lidar stampa il testo di avvio (93 byte, nessun
  pacchetto), poi resta spento perché il robot è sulla base.
- I 52 byte scartati all'inizio sono la coda di un frame già in viaggio quando la spia ha aperto la UART.

## 2026-10-02 — prove fisiche dei sensori (con l'utente)

Metodo: spia inserita con `--eventi` e `--istantanea`; l'utente fa un'azione e la tiene, oppure la ripete 3 volte
a ritmo; `tools/sensori.py diff` confronta min/max di ogni byte con lo stato di riferimento, `tools/ritmo.py`
trova i bit che cambiano a ritmo. Registrazione: `build/robot/spia/20261002-prove-sensori.spia` (27 min, 0 errori).
Risultati nella tabella del frame `0x15` di `protocollo.md`. In sintesi:
- bumper: byte 1, bit 2 (sinistro) e bit 1 (destro). Sulla base sono premuti tutti e due;
- anti-caduta: byte 3, bit 3..0 da sinistra a destra, 1 = nessun pavimento;
- cassetto: byte 4, bit 2; tasti: byte 58 (accensione) e 62 (casa) = durata della pressione in 10 ms;
- encoder delle ruote: int32 ai byte 16 e 24; LED: byte 65 ripete il comando `0x8d` (00 spento, 01 azzurro,
  04 giallo in carica, 08 rosso lampeggiante);
- non ci sono interruttori sulle ruote né sulla torretta del lidar;
- i byte 78–84 (4 valori analogici) non reagiscono ai sensori anti-caduta: da capire.

Problemi incontrati:
- primi tentativi sul bumper falliti perché il robot era sulla base, che tiene già premuti i bumper (byte 1 = `06`);
- **sospensione**: fermo fuori dalla base, dopo 5 minuti RobotApp addormenta il GD32 (sequenza in
  `protocollo.md`) e i sensori non arrivano più. Il risveglio con il tasto non riaccende gli infrarossi
  anti-caduta (`0x78 01`): durante la prova del foglio sembravano rotti;
- con l'OK dell'utente: **`/mnt/UDISK/config/device_config.ini`, `sleep_enable=1` → `0`** (copia in
  `device_config.ini.orig`; per tornare indietro: ricopiare l'originale e riavviare lo stack);
- `tools/spia-robot.sh avvia` sovrascriveva la registrazione precedente: ora la rinomina.
