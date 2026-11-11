# Formato del file della mappa

Deciso con l'utente il 2026-11-07. Il file contiene la mappa salvata dal task aperiodico e ricaricata dalla mappatura
all'avvio, prima che partano i task. Obiettivo: file il più piccolo possibile, con lettura e scrittura semplici.

## Dove sta

Aggiornato il 2026-11-10 con il progetto dell'app (`docs/app.md`, sezione 7).

- Sul robot c'è **una sola mappa**: `/mnt/UDISK/rt/maps/map.rtm`, quella selezionata. Non ci sono più le mappe numerate
  né il file con il numero della mappa attiva.
- Tutte le mappe, compresa una copia di quella selezionata, stanno sul server dell'app nei file `map-<id>.rtm`, con lo
  stesso formato. Per cambiare mappa il robot scarica quella scelta in `map.rtm.download`, il task aperiodico la
  rinomina sopra `map.rtm` e il programma si riavvia.
- Sul PC (simulatore) la cartella arriva dalla variabile d'ambiente `RTES_MAPS`, per non toccare i file del robot.
- Scrittura: file temporaneo (`map.rtm.tmp`) nella stessa cartella, poi rinominato sopra il vecchio. Nessuna copia
  della mappa precedente. Le partizioni sono montate `sync`: finite scrittura e rinomina, i dati sono sulla flash.
- Lo stesso formato serve alla **mappa dal vivo** dell'app: il robot manda al server il rettangolo delle celle
  cambiate come un piccolo file RTM1 (senza zone), e il server lo passa all'app così com'è.
- **CRC come impronta**: il CRC-32 in fondo al file, scritto in 8 cifre esadecimali, identifica la mappa fra robot e
  server (`map.file_crc` nello stato, colonna `crc32` nel database).

## Contenuto

Tutti i numeri sono **little endian**, tutti i campi sono a **byte interi**, senza riempimenti.

| Pezzo | Tipo | Byte |
|---|---|---|
| numero magico `RTM1` ("RT" progetto, "M" mappa, "1" versione del formato) | 4 caratteri ASCII | 4 |
| x della prima colonna del rettangolo, in celle fini | `uint16` | 2 |
| y della prima riga del rettangolo, in celle fini | `uint16` | 2 |
| larghezza del rettangolo, in celle fini | `uint16` | 2 |
| altezza del rettangolo, in celle fini | `uint16` | 2 |
| numero di zone proibite (0-15) | `uint8` | 1 |
| per ogni zona: x, y, larghezza, altezza, in celle fini | 4 × `uint16` | 8 per zona |
| celle del rettangolo, a sequenze (sotto) | `uint8` ciascuna | variabile |
| CRC-32 di tutti i byte precedenti, numero magico compreso | `uint32` | 4 |

Parte fissa: 17 byte, più 8 per zona.

- **Rettangolo**: il più piccolo rettangolo della griglia fine (1024 × 1024 celle da 5 cm) che contiene tutte le celle
  non sconosciute. (x, y) è l'angolo con x e y più piccole (disegnato con la y verso l'alto: in basso a sinistra).
  Si salva solo quello che sta dentro.
- **CRC-32**: quello standard (lo stesso di zlib, `gzip` e `crc32` di Python): polinomio riflesso `0xEDB88320`,
  valore iniziale `0xFFFFFFFF`, risultato negato alla fine. Così il file si controlla anche dal PC.
- **Zone proibite**: solo quelle dell'utente; la zona della base la rimette la mappatura all'avvio, come oggi.
- **Asse delle strisce**: non c'è (deciso con l'utente il 2026-11-07). La missione parte sempre dall'esplorazione, che lo
  ricalcola dalla sua ultima ricerca: se l'esplorazione aggiunge zone nuove l'asse migliore può cambiare.
- La versione del formato, il lato della griglia e il lato della cella non sono nel file: sono costanti del programma.
  Se cambiano, cambia la cifra del numero magico (`RTM2`, …) e i file vecchi si scartano.

## Categorie delle celle

| Valore | Categoria | Da dove viene |
|---|---|---|
| 0 | sconosciuta | punteggio 0 |
| 1 | libera | osservata, punteggio fino alla soglia di occupazione (50); anche l'ostacolo basso (sotto) |
| 2 | occupata | punteggio sopra la soglia |
| 3 | vuoto | valore speciale degli anti-caduta (scale, gradini) |

Non si salvano: i punteggi (cambiano di continuo), l'ostacolo basso del paraurti, le celle chiuse, i quadranti
attraversati (valgono per la sessione). L'ostacolo basso (paraurti o incastro, invisibile al lidar) si salva come
**libera** (decisione dell'utente, 2026-11-07): è quello che il lidar vede in quella cella, e da sconosciuta farebbe
partire l'esplorazione verso quel punto a ogni avvio con la mappa salvata. Alla sessione dopo il robot lo ritrova con
il paraurti e lo rimette come ostacolo basso per quella sessione. Il punteggio con cui riparte ogni categoria al
caricamento è da decidere.

## Celle a sequenze (run length encoding)

Le celle del rettangolo si leggono come un'unica fila, **riga dopo riga** (y crescente; in ogni riga x crescente). Una
sequenza può continuare dalla fine di una riga all'inizio della successiva. Ogni sequenza di celle uguali è **1 byte**:

- bit 7-6: la categoria (0-3);
- bit 5-0: quante celle, **da 1 a 63**, scritte così come sono. Il valore 0 non si usa.

Una sequenza più lunga di 63 celle si spezza in più byte con la stessa categoria (scelta dell'utente: niente
lunghezze più lunghe a parte, il risparmio sarebbe piccolo).

**Scrittura**: si scorrono le celle contando quelle uguali di fila; quando la categoria cambia o il conteggio arriva a
63 si scrive il byte (categoria spostata di 6 bit, più il conteggio) e si riparte da 1. Alla fine si scrive l'ultima
sequenza.

**Lettura**: si legge un byte, categoria = bit 7-6, conteggio = bit 5-0; si scrivono quelle celle, finché sono state
riempite larghezza × altezza celle.

Misura sulle mappe finali del simulatore (2026-11-07): soggiorno 79 × 126 celle 701 byte di sequenze (2.489 a 2 bit
fissi), cucina e camera 69 × 79 circa 460-500 byte (1.363), corridoio 79 × 119 804 byte (2.351).

## Controlli al caricamento

La mappa si scarta (e il robot riesplora, poi salva di nuovo `map.rtm`) se:

- il file non c'è o è più corto della parte fissa;
- il numero magico non è `RTM1`;
- il rettangolo esce dalla griglia (x + larghezza > 1024 o y + altezza > 1024), oppure larghezza o altezza sono 0;
- il numero di zone è oltre 15 (il primo posto della lista è della zona della base), oppure una zona esce dalla griglia;
- una sequenza ha conteggio 0, o va oltre le celle che mancano;
- finite le celle, nel file non restano esattamente 4 byte;
- il CRC-32 non corrisponde.

## Implementazione (2026-11-07, aggiornata il 2026-11-10)

- Scrittura: `save_map` in `tasks/vacuum/aperiodic.c` (task aperiodico, richiesta `request_map_save` dalla navigazione a
  fine esplorazione e all'aggancio riuscito). La griglia si copia a categorie con `copy_map_categories`
  (`tasks/vacuum/mapping.c`, lettura fra un job e l'altro della mappatura), poi `encode_map_rectangle` scrive le
  sequenze dirette nel file temporaneo con il CRC calcolato man mano. La stessa `encode_map_rectangle` codifica i
  rettangoli della mappa dal vivo (`tasks/vacuum/cloud/live_map.c`).
- Lettura: `load_map_if_on_base` e `read_map_file` in `tasks/vacuum/mapping.c`, chiamate da `init_mapping` prima che
  partano i task, solo se il robot è sulla base. Punteggi di partenza: `LOAD_SCORE_FREE` (−100) e
  `LOAD_SCORE_OCCUPIED` (80); poi contatori e griglia di navigazione ricalcolati sul rettangolo.
- Altri lettori dello stesso formato: `decode_map_data` in `tasks/vacuum/cloud/live_map.c` (copia privata della mappa
  dal vivo e controllo della mappa scaricata), `backend/maps.py` (server), `app/lib/map/rtm.dart` (app).
- Controllo dal PC: `python3 tools/leggi-mappa.py FILE.rtm [disegno.png]`.
- Costanti comuni (numero magico, categorie, sequenze): `tasks/vacuum/mapping.h`.
