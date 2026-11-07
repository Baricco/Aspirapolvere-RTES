#ifndef TASK_COMMANDS_H
#define TASK_COMMANDS_H

// Thread dei comandi (docs/PIANO.md, app): legge comandi di testo, una riga per comando, da un file speciale (FIFO) e li
// trasforma in richieste al task aperiodico. È il primo pezzo del thread di rete dell'app: non fa lavoro vero, traduce.
// Sta sul core 3 con lo scheduling normale, come il task aperiodico.
//
// Comandi (sul robot: echo "zone" > /tmp/rtes/comandi-vacuum):
//   zona aggiungi X Y L A     aggiunge una zona proibita: angolo con x e y minime, larghezza e altezza, in metri, sulla mappa
//   zona togli N              toglie la zona numero N (come la stampa "zone")
//   zone                      stampa le zone dell'utente

// crea il file speciale (se manca) e avvia il thread. Ritorna 0, oppure -1 se non ci riesce (il programma va avanti
// senza comandi)
int start_commands(void);

#endif
