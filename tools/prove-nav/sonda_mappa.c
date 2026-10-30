/*
 * sonda_mappa.c - Registratore della mappatura per la copia di prova (tools/prove-nav/costruisci.sh lo include in fondo
 * a build/prove-nav/vac/mapping.c, quindi vede le sue variabili statiche). Il codice in tasks/ non viene toccato.
 *
 * Conta quante volte ogni cella fine passa la soglia di occupazione (ogni chiamata a update_occupied_cells) e quante
 * volte cambia il byte di ogni cella di navigazione (confronto a ogni job con la copia del job prima). All'uscita del
 * programma scrive nel file indicato da RTES_SONDA (se c'è), little endian:
 *     0  "RTSD"
 *     4  griglia fine finale            int8_t   [1024 * 1024]
 *        passaggi di soglia per cella fine uint8_t [512 * 512], fermi a 255, solo nel quadrato centrale della
 *                                       griglia fine (celle fini da 256 a 767, ±12,8 m dalla base)
 *        griglia di navigazione finale  uint8_t  [256 * 256]
 *        cambi per cella di navigazione uint16_t [256 * 256]
 * Poca memoria apposta: dopo mlockall tutta la memoria del programma è bloccata in RAM, e sul PC il limite per i
 * normali utenti è 8 MB (con un contatore da 2 byte per tutta la griglia fine i thread non partono più).
 * Lo legge tools/prove-nav/analizza_mappa.py.
 */
#include <stdio.h>
#include <stdlib.h>

#define SONDA_LATO 512
static uint8_t sonda_cambi_fine[SONDA_LATO * SONDA_LATO];
static uint16_t sonda_cambi_nav[NAV_MAP_CELL_NUMBER * NAV_MAP_CELL_NUMBER];
static uint8_t sonda_nav_prima[NAV_MAP_CELL_NUMBER * NAV_MAP_CELL_NUMBER];
static int sonda_partita = 0;

static void sonda_scrivi(void)
{
	const char *p = getenv("RTES_SONDA");
	if (!p) return;
	FILE *f = fopen(p, "wb");
	if (!f) return;
	fwrite("RTSD", 1, 4, f);
	fwrite(map_state.fine_grid, 1, sizeof map_state.fine_grid, f);
	fwrite(sonda_cambi_fine, 1, sizeof sonda_cambi_fine, f);
	fwrite(navigation_map_data.navigation_grid, 1, sizeof navigation_map_data.navigation_grid, f);
	fwrite(sonda_cambi_nav, 1, sizeof sonda_cambi_nav, f);
	fclose(f);
}

/* all'inizio di ogni job della mappatura: confronta la griglia di navigazione con quella del job prima */
static void sonda_job(void)
{
	if (!sonda_partita) {
		sonda_partita = 1;
		memcpy(sonda_nav_prima, navigation_map_data.navigation_grid, sizeof sonda_nav_prima);
		atexit(sonda_scrivi);
		return;
	}
	for (uint32_t i = 0; i < NAV_MAP_CELL_NUMBER * NAV_MAP_CELL_NUMBER; i++)
		if (navigation_map_data.navigation_grid[i] != sonda_nav_prima[i]) {
			if (sonda_cambi_nav[i] < UINT16_MAX) sonda_cambi_nav[i]++;
			sonda_nav_prima[i] = navigation_map_data.navigation_grid[i];
		}
}

/* a ogni passaggio di soglia di una cella fine (chiamata all'inizio di update_occupied_cells) */
static void sonda_fine(int fine_x, int fine_y)
{
	int x = fine_x - (FINE_MAP_CELL_NUMBER - SONDA_LATO) / 2, y = fine_y - (FINE_MAP_CELL_NUMBER - SONDA_LATO) / 2;
	if (x < 0 || y < 0 || x >= SONDA_LATO || y >= SONDA_LATO) return;
	if (sonda_cambi_fine[y * SONDA_LATO + x] < UINT8_MAX) sonda_cambi_fine[y * SONDA_LATO + x]++;
}
