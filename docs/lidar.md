# `lidar/`: il protocollo del lidar

Il lidar è un 3irobotix LDS3. Manda i dati al SoC su `/dev/ttyS1` a 115200 baud. Il suo motore **non** si comanda da
qui: lo alimenta e lo regola il GD32 (comandi `GD32_LIDAR_POWER` e `GD32_LIDAR_MOTOR`, vedi `docs/gd32.md`).
All'accensione il lidar stampa un testo in cinese con produttore e versione, poi manda solo pacchetti binari.

La cartella ha un solo modulo, `lidar.h` e `lidar.c`, con la stessa impostazione di `gd32/`: niente allocazioni,
niente Linux, nessuno stato globale. Riconosce i pacchetti; **cosa c'è dentro** (settori di misure, giri completi,
regolazione della velocità) lo interpreta `driver/lidar_scan.c` (`docs/driver.md`).

Fonti: `docs/protocollo.md`, capitolo "Lidar 3irobotix LDS3" (riga 210); `docs/RIEPILOGO.md`, capitolo 3.2 (riga 217).
Formato dei settori e misure della velocità: `docs/fase2.md`, blocco C (riga 114).

## Il pacchetto

```
AA | len_hi len_lo | address | type | cmd | n_hi n_lo | parametri (n byte) | ck_hi ck_lo
```

- `len` = byte del pacchetto **senza** i 2 del checksum (= 8 + n). Come l'originale si accettano solo `len_hi = 0` e
  `len_lo` fra 6 e 100 (`LIDAR_LEN_MIN`, `LIDAR_LEN_MAX`). `LIDAR_MAX_PACKET` e `LIDAR_MAX_PARAMS` sono le dimensioni
  massime che ne seguono.
- I numeri sono **big-endian** (al contrario del GD32).
- Il checksum copre i byte da `AA` all'ultimo parametro e **dipende dal campo `address`**: con 1 è la somma dei byte
  modulo 2^16, altrimenti il CRC-16/MODBUS.

## Costanti e tipi

- `LIDAR_HEADER` (`0xAA`).
- Codici dei comandi: `LIDAR_SCAN` (settore di misure, formato vecchio), `LIDAR_HEALTH`, `LIDAR_VERSION` (testo
  ASCII), **`LIDAR_SCAN_NEW`** (settore di misure, formato nuovo: quello che manda il nostro lidar), **`LIDAR_SPEED`**
  (solo la velocità di rotazione: sotto ~4 giri/s il lidar manda solo questi).
- **`lidar_packet_t`**: un pacchetto controllato: `address`, `type`, `cmd`, `n_param` (byte di parametri presenti
  davvero: `len - 8`), `n_declared` (il campo `n` del pacchetto: l'originale non controlla che coincida), `param`.
- **`lidar_deframer_t`**: come `gd32_deframer_t` (`docs/gd32.md`): `buf`/`n`, `pending`, `pending_n`, `pending_i` e i
  contatori `packets_ok`, `bad_checksums`, `bad_lengths`, `discarded_bytes` (ci finisce anche il testo di avvio).
- **`lidar_packet_cb`**: la funzione chiamata per ogni pacchetto valido.

## Funzioni

| Funzione | Cosa fa |
|---|---|
| `lidar_sum(b, n)` | somma dei byte modulo 2^16 |
| `lidar_crc16(b, n)` | CRC-16/MODBUS (polinomio `0xA001` riflesso, valore iniziale `0xFFFF`) |
| `expected_checksum(b, len)` (static) | sceglie fra somma e CRC secondo il campo `address` (`b[3]`) |
| `lidar_build_packet(address, type, cmd, param, n, out, out_size)` | costruisce un pacchetto completo; ritorna la lunghezza (`n + 10`) o 0. Il robot vero non manda pacchetti al lidar: serve alle prove e al simulatore |
| `lidar_cmd_name(cmd)` | nome leggibile del codice (stringa rimasta in italiano), o `NULL` |
| `lidar_deframer_init(d)` | azzera il riconoscitore |
| `lidar_deframer_feed(d, b, n, cb, ctx)` | come `gd32_deframer_feed`: byte in blocchi qualsiasi, `cb` per ogni pacchetto valido, ritorna quanti ne ha trovati |
| `byte(d, b, p)` (static) | aggiunge un byte: aspetta `AA`, poi `len_hi = 0`, poi `len_lo` fra 6 e 100, poi il resto; a pacchetto completo controlla il checksum e riempie `p` |
| `frame_error(d)` (static) | uguale a quella di `gd32.c`: scarta solo il primo byte e rianalizza gli altri |
