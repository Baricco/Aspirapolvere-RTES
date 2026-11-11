/*
 * osserva.c - Lancia un programma dei task e ne legge da fuori la griglia di navigazione, per la pagina del simulatore.
 *
 * Il programma del robot NON viene modificato: osserva lo avvia come processo figlio e, come fa un debugger, legge la
 * sua memoria da /proc/<pid>/mem. Sul PC il kernel permette di leggere la memoria di un processo solo ai suoi antenati
 * (Yama, ptrace_scope = 1): per questo il programma deve essere figlio di osserva. Solo per il PC.
 *
 * Come trova la griglia: nel file eseguibile (compilato con i simboli) cerca il simbolo `navigation_map_data`
 * (tasks/vacuum/mapping.c), con /proc/<pid>/maps calcola dove è stato caricato, poi ogni 200 ms ne legge i primi
 * 65.536 byte (la griglia) e i 4 byte dopo (il numero di versione). Legge SENZA il mutex del programma: ogni tanto può
 * prendere una griglia aggiornata a metà, che la lettura successiva corregge. Se la struttura cambia (nomi, ordine dei
 * campi) la pagina mostra il motivo e niente griglia.
 *
 * Uscita: il file --uscita (default build/sim/mappa_robot), riscritto ogni volta con un rename (chi legge vede sempre un
 * file intero). Formato, little endian:
 *     0   "RTNM"
 *     4   uint32  pid del programma (il simulatore, quando cambia, si segna la posizione vera del robot come origine)
 *     8   uint32  stato: 0 cerco la griglia, 1 griglia letta, 2 programma terminato (resta l'ultima), 3 errore
 *    12   uint32  versione della mappa letta nel programma
 *    16   uint32  letture riuscite
 *    20   char[108] messaggio (testo, terminato da 0)
 *   128   uint8[65536] griglia di navigazione (4 bit bassi = stato, mapping_nav_cell_state; 4 alti = sottocelle bloccate, mapping.h)
 * 65664   uint8  1 se le metà pulite qui sotto sono state lette, 0 se mancano (il resto funziona lo stesso)
 * 65665   uint8  strip_axis_x: true se le strisce vanno lungo x, quindi le due metà di una cella sono y bassa e y alta
 * 65666   uint8[2] riservati
 * 65668   uint8[16384] clean_cell_mask: 2 bit per cella, metà pulite (bit = 2 * cella + metà, tasks/vacuum/navigation.c)
 *
 * Metà pulite: stanno in `nav_state`, una struttura privata di navigation.c che cambia spesso. Per non dipendere dall'ordine
 * dei campi, la posizione di clean_cell_mask e strip_axis_x si chiede alle informazioni di debug con readelf (una volta sola).
 *
 * Posizione creduta: dallo stesso eseguibile legge anche `sc_posa` e `buf_posa` (driver/driver.c), cioè l'ultima posizione
 * che il moto ha dato al driver con drv_pose() (x, y, theta nel sistema della mappa), molto più spesso della griglia
 * (ogni 20 ms) perché sulla pagina deve stare al passo con il robot vero. Non serve la struttura dei task: drv_pose è
 * l'interfaccia del driver. Se il programma non ha quei simboli la posizione manca e il resto funziona.
 * File FILE_posa, riscritto con un rename, little endian: 0 "RTPS"; 4 uint32 pid; 8 uint32 versione (scritture di
 * drv_pose); 12 uint32 riservato; 16, 24, 32 double x, y, theta.
 *
 * Uso: build/host/osserva [--uscita FILE] [--simbolo NOME] [--ogni-ms N] -- programma [argomenti...]
 * Ctrl+C arriva al programma (che si ferma da solo e stampa le statistiche); osserva aspetta che finisca ed esce con
 * il suo stesso codice.
 */
#define _GNU_SOURCE
#include <elf.h>
#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>

#define CELLE      65536                 /* NAV_MAP_CELL_NUMBER * NAV_MAP_CELL_NUMBER (tasks/vacuum/mapping.h) */
#define TESTA      128
#define POSA_DIM   32                    /* sizeof(drv_pose_t), driver/contacts.h */
#define PULITE_TESTA 4                   /* presente, strip_axis_x, 2 riservati */
#define PULITE_DIM (CELLE / 4)           /* sizeof(clean_cell_mask): 2 bit per cella */

enum { CERCO = 0, LETTA = 1, TERMINATO = 2, ERRORE = 3 };

static const char *uscita = "build/sim/mappa_robot", *simbolo = "navigation_map_data";
static pid_t figlio;
static uint8_t file_out[TESTA + CELLE + PULITE_TESTA + PULITE_DIM];
static uint32_t letture, ultima_versione;
static int fd_mem = -1;                 /* la memoria del programma */
static uint64_t ind_sc_posa, ind_buf_posa;   /* 0 = il programma non ha quei simboli */
static uint64_t ind_maschera, ind_asse;      /* 0 = metà pulite non trovate */

static void al_segnale(int s)
{
	if (figlio > 0)
		kill(figlio, s);              /* SIGTERM o SIGHUP: li passa al programma */
}

static void metti32(uint8_t *b, uint32_t v) { memcpy(b, &v, 4); }

/* Riscrive il file di uscita con lo stato, il messaggio e la griglia che ci sono adesso in file_out. */
static void scrivi(uint32_t status, uint32_t version, const char *msg)
{
	memcpy(file_out, "RTNM", 4);
	metti32(file_out + 4, (uint32_t)figlio);
	metti32(file_out + 8, status);
	metti32(file_out + 12, version);
	metti32(file_out + 16, letture);
	memset(file_out + 20, 0, TESTA - 20);
	snprintf((char *)file_out + 20, TESTA - 20, "%s", msg);
	char tmp[600];
	snprintf(tmp, sizeof tmp, "%s.tmp", uscita);
	FILE *f = fopen(tmp, "wb");
	if (!f)
		return;
	size_t ok = fwrite(file_out, 1, sizeof file_out, f);
	if (fclose(f) == 0 && ok == sizeof file_out)
		rename(tmp, uscita);
}

/* Cerca `nome` nella tabella dei simboli dell'ELF a 64 bit `percorso`. 0 se lo trova. */
static int cerca_simbolo(const char *percorso, const char *name, uint64_t *val, uint64_t *size, int *pie, char *msg, size_t n)
{
	FILE *f = fopen(percorso, "rb");
	if (!f) {
		snprintf(msg, n, "non riesco ad aprire %.100s: %s", percorso, strerror(errno));
		return -1;
	}
	fseek(f, 0, SEEK_END);
	long k = ftell(f);
	fseek(f, 0, SEEK_SET);
	uint8_t *b = k > 0 ? malloc((size_t)k) : NULL;
	if (!b || fread(b, 1, (size_t)k, f) != (size_t)k) {
		fclose(f);
		free(b);
		snprintf(msg, n, "non riesco a leggere %.100s", percorso);
		return -1;
	}
	fclose(f);
	int r = -1;
	const Elf64_Ehdr *e = (const Elf64_Ehdr *)b;
	if ((size_t)k < sizeof *e || memcmp(e->e_ident, ELFMAG, SELFMAG) || e->e_ident[EI_CLASS] != ELFCLASS64 ||
	    e->e_shoff + (uint64_t)e->e_shnum * sizeof(Elf64_Shdr) > (uint64_t)k) {
		snprintf(msg, n, "%.100s non è un eseguibile ELF a 64 bit del PC", percorso);
		goto fine;
	}
	*pie = e->e_type == ET_DYN;
	const Elf64_Shdr *sh = (const Elf64_Shdr *)(b + e->e_shoff);
	for (int i = 0; i < e->e_shnum && r; i++) {
		if (sh[i].sh_type != SHT_SYMTAB || sh[i].sh_link >= e->e_shnum)
			continue;
		const Elf64_Sym *s = (const Elf64_Sym *)(b + sh[i].sh_offset);
		const char *str = (const char *)(b + sh[sh[i].sh_link].sh_offset);
		uint64_t ns = sh[i].sh_size / sizeof *s;
		for (uint64_t j = 0; j < ns; j++)
			if (s[j].st_name && !strcmp(str + s[j].st_name, name) && ELF64_ST_TYPE(s[j].st_info) == STT_OBJECT) {
				*val = s[j].st_value;
				*size = s[j].st_size;
				r = 0;
				break;
			}
	}
	if (r)
		snprintf(msg, n, "il programma non ha il simbolo %s (non usa tasks/vacuum/mapping.c, o è strippato)", name);
fine:
	free(b);
	return r;
}

/* Indirizzo a cui è caricato l'eseguibile `percorso` nel processo `pid` (la prima mappatura con offset 0). */
static int base_caricamento(pid_t pid, const char *percorso, uint64_t *base)
{
	char p[64], riga[1200];
	snprintf(p, sizeof p, "/proc/%d/maps", (int)pid);
	FILE *f = fopen(p, "r");
	if (!f)
		return -1;
	int r = -1;
	while (fgets(riga, sizeof riga, f)) {
		unsigned long long da, off;
		int name = 0;
		if (sscanf(riga, "%llx-%*x %*s %llx %*s %*s %n", &da, &off, &name) < 2 || !name)
			continue;
		char *q = riga + name;
		q[strcspn(q, "\n")] = 0;
		if (off == 0 && !strcmp(q, percorso)) {
			*base = da;
			r = 0;
			break;
		}
	}
	fclose(f);
	return r;
}

/* Chiede a readelf la posizione di due campi di nav_state nelle informazioni di debug di `exe`. 0 se le trova tutte e due. */
static int cerca_campi_pulizia(const char *exe, uint64_t *off_maschera, uint64_t *off_asse)
{
	char cmd[700], riga[400];
	if (strchr(exe, '\''))
		return -1;
	snprintf(cmd, sizeof cmd, "LC_ALL=C readelf --debug-dump=info '%s' 2>/dev/null", exe);
	FILE *f = popen(cmd, "r");
	if (!f)
		return -1;
	uint64_t *pending = NULL;
	int trovati = 0;
	while (fgets(riga, sizeof riga, f)) {
		riga[strcspn(riga, "\n")] = 0;
		char *q;
		if (strstr(riga, "DW_AT_name")) {
			size_t n = strlen(riga);
			while (n && riga[n - 1] == ' ')
				riga[--n] = 0;
			q = strrchr(riga, ' ');
			pending = !q ? NULL : !strcmp(q + 1, "clean_cell_mask") ? off_maschera :
			         !strcmp(q + 1, "strip_axis_x") ? off_asse : NULL;
		} else if (pending && (q = strstr(riga, "DW_AT_data_member_location:"))) {
			*pending = strtoull(q + strlen("DW_AT_data_member_location:"), NULL, 0);
			trovati++;
			pending = NULL;
		}
	}
	pclose(f);
	return trovati == 2 ? 0 : -1;
}

/* Copia nel file le metà pulite, se le abbiamo trovate. */
static void leggi_pulite(void)
{
	uint8_t *d = file_out + TESTA + CELLE, asse = 0;
	d[0] = 0;
	if (!ind_maschera || pread(fd_mem, &asse, 1, (off_t)ind_asse) != 1 ||
	    pread(fd_mem, d + PULITE_TESTA, PULITE_DIM, (off_t)ind_maschera) != PULITE_DIM)
		return;
	d[1] = asse;
	d[0] = 1;
}

/* Un giro di lettura: trova la griglia (la prima volta, o se il programma ha fatto exec) e la copia nel file. */
static void leggi(void)
{
	static char exe_noto[600];
	static uint64_t address;
	char p[64], exe[600], msg[200];
	snprintf(p, sizeof p, "/proc/%d/exe", (int)figlio);
	ssize_t k = readlink(p, exe, sizeof exe - 1);
	if (k <= 0)
		return;
	exe[k] = 0;
	if (strcmp(exe, exe_noto)) {                 /* nuovo eseguibile: all'inizio c'è taskset, poi il programma */
		snprintf(exe_noto, sizeof exe_noto, "%s", exe);
		address = 0;
		ind_sc_posa = ind_buf_posa = 0;
		ind_maschera = ind_asse = 0;
		if (fd_mem >= 0)
			close(fd_mem);
		fd_mem = -1;
		uint64_t val = 0, size = 0, base = 0;
		int pie = 0;
		if (cerca_simbolo(exe, simbolo, &val, &size, &pie, msg, sizeof msg)) {
			scrivi(CERCO, 0, msg);
			return;
		}
		if (size < CELLE + 4) {
			snprintf(msg, sizeof msg, "%s è di %llu byte, meno di griglia + versione: la struttura è cambiata?", simbolo,
			         (unsigned long long)size);
			scrivi(ERRORE, 0, msg);
			return;
		}
		if (pie && base_caricamento(figlio, exe, &base)) {
			scrivi(CERCO, 0, "non trovo dove è caricato il programma (/proc/pid/maps)");
			exe_noto[0] = 0;
			return;
		}
		address = base + val;
		uint64_t v1, v2, d1, d2;
		int pie2;
		char ignora[200];
		if (!cerca_simbolo(exe, "pose_exchange", &v1, &d1, &pie2, ignora, sizeof ignora) &&
		    !cerca_simbolo(exe, "pose_buffers", &v2, &d2, &pie2, ignora, sizeof ignora) && d2 == 2 * POSA_DIM) {
			ind_sc_posa = base + v1;
			ind_buf_posa = base + v2;
		}
		uint64_t v3, d3, off_m = 0, off_a = 0;
		if (!cerca_simbolo(exe, "nav_state", &v3, &d3, &pie2, ignora, sizeof ignora) &&
		    !cerca_campi_pulizia(exe, &off_m, &off_a) && off_m + PULITE_DIM <= d3 && off_a < d3) {
			ind_maschera = base + v3 + off_m;
			ind_asse = base + v3 + off_a;
		}
		snprintf(p, sizeof p, "/proc/%d/mem", (int)figlio);
		fd_mem = open(p, O_RDONLY);
		if (fd_mem < 0) {
			snprintf(msg, sizeof msg, "non posso leggere la memoria del programma: %s", strerror(errno));
			scrivi(ERRORE, 0, msg);
			return;
		}
	}
	if (fd_mem < 0 || !address)
		return;
	uint8_t b[CELLE + 4];
	if (pread(fd_mem, b, sizeof b, (off_t)address) != (ssize_t)sizeof b) {
		snprintf(msg, sizeof msg, "lettura della memoria fallita: %s", strerror(errno));
		scrivi(ERRORE, 0, msg);
		return;
	}
	uint32_t version;
	memcpy(&version, b + CELLE, 4);
	memcpy(file_out + TESTA, b, CELLE);
	leggi_pulite();
	letture++;
	ultima_versione = version;
	scrivi(LETTA, version, "");
}

/* Copia nel file _posa l'ultima posizione data dal moto al driver, se è nuova. Legge senza fermare il programma: se nel
 * frattempo è arrivata una scrittura salta il giro (il prossimo è fra 20 ms). */
static void leggi_posa(void)
{
	static uint32_t ultima;
	static char name[640], tmp[650];
	if (fd_mem < 0 || !ind_sc_posa)
		return;
	uint32_t v, v2;
	uint8_t buf[POSA_DIM];
	if (pread(fd_mem, &v, 4, (off_t)ind_sc_posa) != 4 || v == 0 || v == ultima)   /* versione: primo campo di exchange_t */
		return;
	if (pread(fd_mem, buf, sizeof buf, (off_t)(ind_buf_posa + (v & 1) * POSA_DIM)) != (ssize_t)sizeof buf)
		return;
	if (pread(fd_mem, &v2, 4, (off_t)ind_sc_posa) != 4 || v2 != v)
		return;
	ultima = v;
	uint8_t out[40] = "RTPS";
	metti32(out + 4, (uint32_t)figlio);
	metti32(out + 8, v);
	memcpy(out + 16, buf, 24);                 /* x, y, theta: i primi campi di drv_pose_t */
	if (!name[0]) {
		snprintf(name, sizeof name, "%s_posa", uscita);
		snprintf(tmp, sizeof tmp, "%s.tmp", name);
	}
	FILE *f = fopen(tmp, "wb");
	if (!f)
		return;
	size_t ok = fwrite(out, 1, sizeof out, f);
	if (fclose(f) == 0 && ok == sizeof out)
		rename(tmp, name);
}

int main(int argc, char **argv)
{
	int ogni_ms = 200, i = 1;
	for (; i < argc && strcmp(argv[i], "--"); i++) {
		if (!strcmp(argv[i], "--uscita") && i + 1 < argc) uscita = argv[++i];
		else if (!strcmp(argv[i], "--simbolo") && i + 1 < argc) simbolo = argv[++i];
		else if (!strcmp(argv[i], "--ogni-ms") && i + 1 < argc) ogni_ms = atoi(argv[++i]);
		else break;
	}
	if (i >= argc || strcmp(argv[i], "--") || i + 1 >= argc) {
		fprintf(stderr, "uso: osserva [--uscita FILE] [--simbolo NOME] [--ogni-ms N] -- programma [argomenti...]\n");
		return 2;
	}
	char **cmd = argv + i + 1;

	figlio = fork();
	if (figlio < 0) {
		perror("osserva: fork");
		return 1;
	}
	if (figlio == 0) {
		execvp(cmd[0], cmd);
		fprintf(stderr, "osserva: non riesco ad avviare %s: %s\n", cmd[0], strerror(errno));
		_exit(127);
	}
	/* Ctrl+C va a tutto il gruppo del terminale: il programma lo gestisce, osserva aspetta che finisca */
	signal(SIGINT, SIG_IGN);
	signal(SIGQUIT, SIG_IGN);
	struct sigaction sa = { .sa_handler = al_segnale };
	sigaction(SIGTERM, &sa, NULL);
	sigaction(SIGHUP, &sa, NULL);
	scrivi(CERCO, 0, "programma appena avviato");

	int st = 0, pending = 0;
	for (;;) {
		pid_t r = waitpid(figlio, &st, WNOHANG);
		if (r == figlio)
			break;
		if (r < 0 && errno != EINTR)
			break;
		if (pending <= 0) {
			leggi();
			pending = ogni_ms;
		}
		leggi_posa();
		pending -= 20;
		struct timespec ts = { 0, 20 * 1000000L };
		nanosleep(&ts, NULL);
	}
	scrivi(TERMINATO, ultima_versione, "programma terminato: resta l'ultima griglia letta");
	if (WIFEXITED(st))
		return WEXITSTATUS(st);
	return WIFSIGNALED(st) ? 128 + WTERMSIG(st) : 1;
}
