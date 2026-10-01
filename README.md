# Aspirapolvere-RTES

Progetto d'esame di Real-Time Embedded Systems: task real-time in C (`SCHED_FIFO`, multi-core partizionato)
che girano direttamente sul robot Proscenic M6 Pro (3irobotix CRL-200S, Allwinner A33, Linux 3.4)
e ne comandano l'hardware al posto del firmware originale.

| Cartella | Contenuto |
|---|---|
| `rt/` | creazione dei task: politica, priorità, affinità ai core, `mlockall`, periodi |
| `tasks/` | i task applicativi |
| `hal/` | interfaccia hardware: `hal.h`, backend robot (UART verso il GD32) e backend simulatore |
| `sim/` | simulatore 2D (stanza, ostacoli, cinematica differenziale, bumper/cliff) |
| `test/` | unit test |
| `tools/` | script di supporto (deploy, analisi delle tracce) |
| `docs/` | piano, protocollo ricostruito, misure, relazione |

Tutti i comandi passano da `./rtes.sh` (`./rtes.sh help`). I test si lanciano a mano prima di ogni commit.
Il piano e le decisioni sono in [docs/PIANO.md](docs/PIANO.md).
