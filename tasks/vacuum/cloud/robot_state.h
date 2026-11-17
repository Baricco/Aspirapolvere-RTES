#ifndef CLOUD_ROBOT_STATE_H
#define CLOUD_ROBOT_STATE_H

// Stato del robot per l'app (docs/app.md, sezione 4): un JSON costruito solo con dati che i task e il driver pubblicano
// già (funzioni read_* e drv_*: chi scrive non aspetta mai), e la regola che decide quando mandarlo:
//   - a ogni ciclo quando il robot si muove (stato della navigazione attivo, oppure v o omega diversi da 0);
//   - subito quando cambia qualcosa di importante (stato, missione, guasto, fase del driver, aggancio, telecomando,
//     carica, base, esito di un comando, zone);
//   - ogni STATE_IDLE_INTERVAL a riposo.
// Lo usa solo il thread dello stato.

void init_robot_state(void);

// a ogni ciclo del thread dello stato: se lo stato è da mandare ritorna il JSON (da liberare con free), altrimenti NULL
char *build_robot_state_if_due(void);

// il server ha ricevuto l'ultimo stato costruito: fino al prossimo cambiamento non serve rimandarlo.
// Se non viene chiamata (rete giù) lo stato resta da mandare
void mark_robot_state_sent(void);

#endif
