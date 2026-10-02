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
