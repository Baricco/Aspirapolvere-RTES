# Fase 2: caratterizzazione completa dell'hardware — passi

Obiettivo: alla fine della fase **ogni** sensore, attuatore e comando del robot è capito, provato e documentato
(`RIEPILOGO.md` senza "[ipotesi]" importanti), e le librerie `gd32/` e `lidar/` decodificano tutto in strutture C
pronte per il driver della fase 3. Ordine deciso con l'utente: **prima tutto il lavoro autonomo, poi una sola
sessione insieme** (~45 min).

Fonti, in ordine di affidabilità:
1. prove sul robot (spia, banco di prova);
2. disassemblato di **everest-server** (`../Estratto/nandd-rootfs/usr/sbin/everest-server`, ~10.500 nomi di
   funzione) e di AuxCtrl, con `tools/disassembla.sh '<regex>' <binario>`;
3. il progetto pubblico codetiger/VacuumTiger (stessa scheda CRL-200S): seconda fonte da verificare, contiene
   errori noti (`0x0d` descritto come richiesta dati, intervallo di tensione della batteria, checksum delle
   scansioni non verificato).

## Regole di sicurezza (valgono per tutta la fase)
- Comandi ai motori **solo con il robot sollevato**, verificato dal programma: i 4 anti-caduta (byte 3 dei sensori)
  devono valere tutti "vuoto" (`0x0f`); se cambia anche un bit durante una prova, tutti i motori a zero subito.
- Velocità basse e durate brevi; all'uscita del programma (anche Ctrl-C o terminazione) tutti i motori a zero.
- Sempre il keepalive `0x0d` ogni 3 s mentre il nostro programma guida il GD32.
- Niente comandi alla pompa dell'acqua (il serbatoio non c'è) né a `0x99`/`0x9a` (alimentazione/riavvio del SoC).
- Ogni prova è registrata (spia o registro del banco di prova) e annotata nel diario `fase2.md`.

## Blocco A — Disassemblato di everest-server (autonomo, senza robot)
1. `CSerialMessagePacket::packet*` (≈40 funzioni) → codice di ogni comando, formato e unità dei dati. Tra gli altri:
   `packetSensorPower`, `packetSelfControlParams`, `packetGetMachineFunctionList`, `packetMaskSensorMotion`,
   `packetBumperSmooth`, `packetCliffBack`, `packetBlowerControlMode`, `packetBlowerPwmBattery`,
   `packetBlowerPwmOnly`, `packetRequireIMURawData`, `packetGeoMagnetismCalibrate`, `packetTinaHaltedDetectOnce`.
   Obiettivo: spiegare `0x17 0x18 0x19 0x7c 0x86 0x6b 0x9d`.
2. `CSerialMessageHandle::update{Battery,Common,Control,Else,IMU,Motor}Sensor`, `updateIMUAngleVelocity`,
   `unpacketDockIR`, `handleErrorCode`, `handleDebug1` → offset, tipo e scala di **ogni** byte del frame `0x15`
   (temperatura e corrente della batteria, correnti dei motori, IR, IMU, errori).
3. `CHwdriverExceptionDetect::*`, `CRobotExceptionDetect::*` (es. `detectBatteryTempatureAbnormal`,
   `detectDustboxFull`, `detectBrushBlock`) e i file di configurazione (`hwdrivers.ini`, `motion_params.ini`…) →
   soglie e significato fisico.
4. La sequenza di avvio completa e la gestione della carica (chi abilita la carica e come).
5. Le funzioni del lidar (`unpacketNewLidarScanHasSingal/NoSingal`, `toothScan2LidarScan`, `combineScan`, velocità e
   salute) → formato delle scansioni.

Risultato: `protocollo.md` e `RIEPILOGO.md` aggiornati; `gd32/gd32.h` con tutti i codici; nuova struct
`gd32_sensori_t` con la decodifica completa del frame `0x15` e i test con frame veri.

## Blocco B — Banco di prova (autonomo, sul PC)
Estendere `spia guida` (`spia/spia.c`):
- comandi con dati (`--invia 0x97:01`), invio singolo, sequenze da file con tempi;
- **interblocco motori** come nelle regole di sicurezza, con l'opzione esplicita `--sollevato`;
- stampa in diretta dei campi decodificati (`gd32_sensori_t`), registro di tutto.
Test sul PC con il GD32 finto (`tools/prova-spia.py`): con il byte 3 diverso da `0x0f` nessun comando ai motori
deve partire, e se il byte 3 cambia durante una prova i motori vanno a zero.

## Blocco C — Prove sul robot senza l'utente (robot sollevato sulla base, stack originale fermo)
1. **Lidar**: sequenza di accensione (`0x65 02`, `0xa2`, `0x97 01`, `0x71 N`), cattura di scansioni, salute e
   velocità; decodifica in `lidar/` (struct scansione + test); giri al secondo e punti per giro al variare della
   potenza. Serve anche alla mappatura in hard RT (periodo e dimensione dei dati).
2. **Ruote**: `0x66` e `0x67` a velocità basse; verso e scala degli encoder; tempi di risposta; **cosa fa il GD32 se
   smettiamo di mandare comandi di velocità** (si fermano? dopo quanto?).
3. **Spazzole e aspirazione**: `0x69`, `0x6a`, `0x68` e le varianti dell'aspirazione; valori ammessi, correnti nel
   frame dei sensori.
4. **Comandi sconosciuti** dell'avvio e quelli trovati nel blocco A: effetto osservato.
5. **Carica**: quale comando la abilita (il robot oggi carica solo con lo stack originale acceso); frequenza del
   frame `0x15` con `0x65 00` e `0x65 02` (VacuumTiger dice ~110 Hz, noi misuriamo 50 Hz).
6. **Audio**: `tinyplayer`, volume, GPIO 233 (per lo stato di guasto).
7. **LED**: i 19 modi di `0x8d` (per lo stato di guasto).
8. **Batteria nel tempo** (lunga, incustodita): tensione, temperatura, corrente, percentuale durante una scarica e
   una carica → curve per la stima della carica nel driver.

## Blocco D — Sessione insieme (~45 min, robot sollevato)
1. IMU: rotazione a mano di 90° su un piano, inclinazioni → assi e scale.
2. IR analogici (byte 78–84) e IR della base: mano/oggetti ai lati e davanti, robot vicino/lontano dalla base.
3. Anti-caduta: chiarire il punto di vista di "sinistra/destra" (noi: bit 3 = sinistra; VacuumTiger: bit 0).
4. Lidar: oggetto a distanza misurata con il metro, davanti e di lato.
5. Ascolto e osservazione degli attuatori (verso delle spazzole, rumori), del LED e dei suoni.
6. Conferma della temperatura della batteria se il codice non basta (confronto durante la carica).

## Uscita
- `docs/fase2.md` (diario), `protocollo.md`, `RIEPILOGO.md`, `hardware.md` aggiornati.
- `gd32/` e `lidar/` con decodifica completa e test; banco di prova con interblocco.
- Dati per la fase 3: periodi, tempi di risposta degli attuatori, comportamento del GD32 senza comandi, curve della
  batteria, formato e frequenza del lidar.
