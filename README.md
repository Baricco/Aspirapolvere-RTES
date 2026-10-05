# Aspirapolvere-RTES

Progetto d'esame di Real-Time Embedded Systems: task real-time in C (`SCHED_FIFO`, multi-core partizionato)
che girano direttamente sul robot Proscenic M6 Pro (3irobotix CRL-200S, Allwinner A33, Linux 3.4)
e ne comandano l'hardware al posto del firmware originale.

| Cartella | Contenuto |
|---|---|
| `rt/` | task real-time: SCHED_FIFO, priorità (anche Rate Monotonic), affinità ai core, `mlockall`, task periodici e sporadici, statistiche, mutex con priority inheritance |
| `driver/` | il driver del robot (al posto di AuxCtrl): due thread SCHED_FIFO (tempo 99, ricezione 98) che parlano con GD32 e lidar, pubblica i dati senza mutex, applica i limiti di sicurezza, sorveglia i task; `rtes-driver` per provarlo a mano |
| `gd32/`, `lidar/` | protocolli seriali del microcontrollore GD32 e del lidar; blocco di sicurezza dei motori |
| `sim/` | simulatore: GD32 e lidar finti su pseudo-seriali, stanza disegnabile da una pagina web |
| `tasks/` | i task applicativi (fase 4): `tasks/<nome>.c` o `tasks/<nome>/` diventa un programma; come si scrivono in `docs/TASK.md` |
| `spia/` | spia del traffico fra lo stack originale e l'hardware, banco di prova `spia guida` |
| `bench/` | misure di latenza e carico |
| `test/` | unit test |
| `tools/` | script di supporto (prove sul robot, prove d'insieme, analisi) |
| `docs/` | piano, guida (`GUIDA.md`), protocolli, diari delle fasi, misure |

Tutti i comandi passano da `./rtes.sh` (`./rtes.sh help`). I test si lanciano a mano prima di ogni commit.
Il piano e le decisioni sono in [docs/PIANO.md](docs/PIANO.md).
