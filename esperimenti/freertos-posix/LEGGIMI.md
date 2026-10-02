# Esperimento: FreeRTOS ufficiale sul Linux del robot (2026-10-02)

Serve alla scelta del framework RT (vedi `docs/PIANO.md`, "Ancora da decidere"). Non è codice del progetto.

- `main.c`, `FreeRTOSConfig.h`: un task periodico a 10 ms (priorità 3) misura il ritardo dei propri risvegli
  rispetto al tempo reale per 3000 periodi; un task a priorità 1 consuma CPU senza sosta. Processo fissato al core 3,
  `SCHED_FIFO` 80, `mlockall`.
- `port-tick-rt.diff`: correzione del port POSIX di FreeRTOS V11.1.0. Nell'originale il tick è un thread con
  `usleep()` alla stessa priorità dei task: (1) un task che non si blocca mai impedisce al tick di girare e
  FreeRTOS si ferma (successo alla prima prova); (2) `usleep` relativo fa derivare il tempo. La correzione dà al
  thread del tick `SCHED_FIFO` 95 (si comporta come un interrupt) e lo sveglia a istanti assoluti.
- `prova.sh`: scarica il kernel, applica la correzione, compila ed esegue sul robot.

Risultato sul robot (stack originale attivo sugli altri core): ritardo **−11…+13 µs**, medio −6 µs, ultimo −5 µs
(nessuna deriva), **jitter massimo 21 µs** tra risvegli consecutivi.

Limiti noti del port, da valutare nella scelta: un solo core; niente interrupt veri (I/O non bloccante o thread
Linux esterni); chiamate bloccanti e `printf` dentro i task da evitare o proteggere; stack minimi
(`PTHREAD_STACK_MIN`).
