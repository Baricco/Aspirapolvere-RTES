#ifndef CLOUD_COMMANDS_H
#define CLOUD_COMMANDS_H

// Thread della FIFO dei comandi: legge comandi JSON, una riga per comando, da un file speciale e li passa all'interprete
// (app_commands.h), lo stesso che usa il thread di rete. Serve per le prove senza l'app. Sta sul core 3 con lo
// scheduling normale, come il task aperiodico. L'esito di ogni comando va nel registro del programma.
//
// Esempi (sul robot: echo '{"type":"start"}' > /tmp/rtes/vacuum-commands):
//   {"type":"start"}  {"type":"pause"}  {"type":"resume"}  {"type":"home"}  {"type":"fault_reset"}
//   {"type":"remote_move","v":0.2,"omega":0}  {"type":"remote_release"}
//   {"type":"zones","zones":[{"x":1.0,"y":1.0,"w":0.5,"h":0.5}]}     la lista intera delle zone: angolo con x e y minime, larghezza e altezza, in metri sulla mappa
//   {"type":"map_reset"}

// crea il file speciale (se manca) e avvia il thread. Ritorna 0, oppure -1 se non ci riesce (il programma va avanti senza la FIFO)
int start_commands(void);

#endif
