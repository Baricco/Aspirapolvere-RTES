/* Test della mappa dal vivo dell'app (tasks/vacuum/cloud/live_map.c) e della codifica RTM1 (tasks/vacuum/aperiodic.c):
 * codifica e decodifica, confronto fra due griglie, e il giro completo verso un server finto (mappa salvata, mappa
 * intera, differenze, 409, rete giù). La mappatura, il movimento e la rete sono finti e stanno qui sotto. */
#include <semaphore.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#include "test.h"
#include "../tasks/vacuum/aperiodic.h"
#include "../tasks/vacuum/mapping.h"
#include "../tasks/vacuum/movement.h"
#include "../tasks/vacuum/cloud/config_check.h"
#include "../tasks/vacuum/cloud/live_map.h"

/* ------------------------------------------------------------------------------------------- pezzi finti --- */

static char cartella[] = "/tmp/rtes-test-live-map-XXXXXX";

/* la mappatura finta: una griglia a categorie che il test cambia a mano */
static uint8_t mappa_robot[MAP_CATEGORIES_BYTES];
static bool partito_dalla_base;
static bool copia_riesce = true;
static int ultima_finestra[4];
static movement_position_data_t posizione;

static void scrivi_cella(uint8_t *griglia, int x, int y, uint8_t categoria)
{
	uint32_t i = (uint32_t)y * FINE_MAP_CELL_NUMBER + (uint32_t)x;
	griglia[i >> 2] = (uint8_t)((griglia[i >> 2] & ~(3u << (2 * (i & 3)))) | (uint32_t)categoria << (2 * (i & 3)));
}

bool copy_map_categories(uint8_t *categories, int rect_x, int rect_y, int rect_width, int rect_height,
                         mapping_forbidden_zone_t *user_zones, int *num_user_zones)
{
	(void)user_zones; (void)num_user_zones;
	ultima_finestra[0] = rect_x; ultima_finestra[1] = rect_y; ultima_finestra[2] = rect_width; ultima_finestra[3] = rect_height;
	if (!copia_riesce) return false;
	/* come quella vera: solo il rettangolo chiesto */
	for (int y = rect_y; y < rect_y + rect_height; y++)
		for (int x = rect_x; x < rect_x + rect_width; x++)
			scrivi_cella(categories, x, y, get_map_category(mappa_robot, x, y));
	return true;
}

bool read_started_on_base(void) { return partito_dalla_base; }
movement_position_data_t read_movement(void) { return posizione; }

/* il task aperiodico è quello vero (codifica RTM1, CRC, salvataggio su file): gli servono queste altre funzioni della
 * mappatura, che qui non fanno niente */
void write_user_zones(const mapping_user_zones_t *user_zones) { (void)user_zones; }
uint32_t read_applied_user_zones_version(void) { return 0; }
void run_position_search(uint32_t request_num) { (void)request_num; }

bool append_bytes(void *context, const uint8_t *data, size_t size)
{
	byte_buffer_t *b = context;
	if (b->size + size > b->capacity) {
		b->capacity = (b->size + size) * 2;
		b->data = realloc(b->data, b->capacity);
	}
	memcpy(b->data + b->size, data, size);
	b->size += size;
	return true;
}

void free_bytes(byte_buffer_t *b) { free(b->data); b->data = NULL; b->size = b->capacity = 0; }

/* il server finto: tiene la sua copia della mappa dal vivo e la versione del robot, come backend/maps.py */
static uint8_t mappa_server[MAP_CATEGORIES_BYTES];
static long versione_server = -1;       /* -1: il server non ha nessuna versione (appena ripartito) */
static bool rete_giu;
static int richieste;
static char ultimo_percorso[128], ultimo_metodo[8];
static char ultimo_salvato[32];         /* motivo dell'ultima mappa salvata ricevuta */
static size_t dimensione_salvata;
static size_t dimensione_ultimo_corpo;

long send_http_request(http_connection_t *connection, const char *method, const char *path, const uint8_t *body,
                       size_t body_size, bool is_json, byte_buffer_t *response, long timeout_s)
{
	(void)connection; (void)is_json; (void)response; (void)timeout_s;
	if (rete_giu) return HTTP_FAILED;
	richieste++;
	snprintf(ultimo_percorso, sizeof ultimo_percorso, "%s", path);
	snprintf(ultimo_metodo, sizeof ultimo_metodo, "%s", method);
	dimensione_ultimo_corpo = body_size;
	unsigned base, versione;
	char motivo[32];
	if (sscanf(path, "/api/device/map/saved?reason=%31s", motivo) == 1) {
		snprintf(ultimo_salvato, sizeof ultimo_salvato, "%s", motivo);
		dimensione_salvata = body_size;
		return 200;
	}
	if (sscanf(path, "/api/device/map/patch?base_version=%u&version=%u", &base, &versione) == 2) {
		if (versione_server != (long)base) return 409;
		if (!decode_map_data(body, body_size, mappa_server)) return 400;
		versione_server = versione;
		return 200;
	}
	if (sscanf(path, "/api/device/map?version=%u", &versione) == 1) {
		memset(mappa_server, 0, sizeof mappa_server);
		if (!decode_map_data(body, body_size, mappa_server)) return 400;
		versione_server = versione;
		return 200;
	}
	return 404;
}

/* ----------------------------------------------------------------------------------------------- test --- */

static void riempi(uint8_t *griglia, int x, int y, int larghezza, int altezza, uint8_t categoria)
{
	for (int cy = y; cy < y + altezza; cy++)
		for (int cx = x; cx < x + larghezza; cx++)
			scrivi_cella(griglia, cx, cy, categoria);
}

static void robot_in(double x, double y)
{
	posizione.x = x; posizione.y = y; posizione.valid = true;
	track_live_map_position();
}

static void crc_standard(void)
{
	/* il valore di controllo del CRC-32 standard (zlib, gzip, crc32 di Python) */
	CONTROLLA(map_crc32(0, (const uint8_t *)"123456789", 9) == 0xCBF43926u);
	/* a pezzi dà lo stesso risultato */
	uint32_t crc = map_crc32(0, (const uint8_t *)"1234", 4);
	CONTROLLA(map_crc32(crc, (const uint8_t *)"56789", 5) == 0xCBF43926u);
}

static void codifica_rtm1(void)
{
	static uint8_t griglia[MAP_CATEGORIES_BYTES];
	memset(griglia, 0, sizeof griglia);
	/* 200 celle libere in un rettangolo 20 x 10: sequenze da 63, 63, 63 e 11 */
	riempi(griglia, 100, 200, 20, 10, MAP_CAT_FREE);

	byte_buffer_t b = {0};
	CONTROLLA(encode_map_rectangle(griglia, 100, 200, 20, 10, NULL, 0, append_bytes, &b));
	CONTROLLA(b.size == 13 + 4 + 4);
	CONTROLLA(memcmp(b.data, "RTM1", 4) == 0);
	const uint8_t atteso[] = { 100, 0, 200, 0, 20, 0, 10, 0, 0, 0x7F, 0x7F, 0x7F, 0x40 | 11 };
	CONTROLLA(memcmp(b.data + 4, atteso, sizeof atteso) == 0);
	uint32_t crc = map_crc32(0, b.data, b.size - 4);
	CONTROLLA(b.data[b.size - 4] == (uint8_t)crc && b.data[b.size - 1] == (uint8_t)(crc >> 24));
	free_bytes(&b);

	/* con le zone, in celle fini: la zona della base di docs/app.md */
	mapping_forbidden_zone_t zona = { .x = -0.365, .y = -0.22, .w = 0.26, .h = 0.44 };
	CONTROLLA(encode_map_rectangle(griglia, 100, 200, 20, 10, &zona, 1, append_bytes, &b));
	CONTROLLA(b.data[12] == 1);
	const uint8_t zona_attesa[] = { 505 & 0xFF, 505 >> 8, 508 & 0xFF, 508 >> 8, 5, 0, 9, 0 };
	CONTROLLA(memcmp(b.data + 13, zona_attesa, sizeof zona_attesa) == 0);
	CONTROLLA(decode_map_data(b.data, b.size, NULL));
	free_bytes(&b);
}

static void codifica_e_decodifica(void)
{
	static uint8_t griglia[MAP_CATEGORIES_BYTES], letta[MAP_CATEGORIES_BYTES];
	memset(griglia, 0, sizeof griglia);
	memset(letta, 0, sizeof letta);
	/* una stanza con muri, un buco sconosciuto, un vuoto e sequenze che passano da una riga all'altra */
	riempi(griglia, 300, 400, 90, 70, MAP_CAT_FREE);
	riempi(griglia, 300, 400, 90, 1, MAP_CAT_OCCUPIED);
	riempi(griglia, 300, 469, 90, 1, MAP_CAT_OCCUPIED);
	riempi(griglia, 300, 400, 1, 70, MAP_CAT_OCCUPIED);
	riempi(griglia, 320, 420, 7, 5, MAP_CAT_UNKNOWN);
	riempi(griglia, 350, 430, 3, 3, MAP_CAT_VOID);
	scrivi_cella(griglia, 1023, 1023, MAP_CAT_OCCUPIED);
	scrivi_cella(griglia, 0, 0, MAP_CAT_VOID);

	int x, y, larghezza, altezza;
	CONTROLLA(find_map_rectangle(griglia, &x, &y, &larghezza, &altezza));
	CONTROLLA(x == 0 && y == 0 && larghezza == 1024 && altezza == 1024);

	byte_buffer_t b = {0};
	CONTROLLA(encode_map_rectangle(griglia, x, y, larghezza, altezza, NULL, 0, append_bytes, &b));
	CONTROLLA(decode_map_data(b.data, b.size, letta));
	CONTROLLA(memcmp(griglia, letta, sizeof griglia) == 0);

	/* un file rovinato non passa e non tocca la griglia */
	memset(letta, 0, sizeof letta);
	b.data[b.size / 2] ^= 0x40;
	CONTROLLA(!decode_map_data(b.data, b.size, letta));
	b.data[b.size / 2] ^= 0x40;
	CONTROLLA(!decode_map_data(b.data, b.size - 1, letta));
	CONTROLLA(!decode_map_data(b.data, 10, letta));
	b.data[3] = '2';
	CONTROLLA(!decode_map_data(b.data, b.size, letta));
	static const uint8_t zeri[MAP_CATEGORIES_BYTES];
	CONTROLLA(memcmp(letta, zeri, sizeof letta) == 0);
	free_bytes(&b);

	/* griglia vuota: nessun rettangolo */
	memset(griglia, 0, sizeof griglia);
	CONTROLLA(!find_map_rectangle(griglia, &x, &y, &larghezza, &altezza));
}

static void confronto(void)
{
	static uint8_t vecchia[MAP_CATEGORIES_BYTES], nuova[MAP_CATEGORIES_BYTES];
	memset(vecchia, 0, sizeof vecchia);
	riempi(vecchia, 500, 500, 40, 40, MAP_CAT_FREE);
	memcpy(nuova, vecchia, sizeof nuova);

	int x, y, larghezza, altezza;
	CONTROLLA(!find_changed_rectangle(vecchia, nuova, 0, 0, 1024, 1024, &x, &y, &larghezza, &altezza));

	/* due celle cambiate: il rettangolo le contiene tutte e due */
	scrivi_cella(nuova, 510, 505, MAP_CAT_OCCUPIED);
	scrivi_cella(nuova, 533, 520, MAP_CAT_VOID);
	CONTROLLA(find_changed_rectangle(vecchia, nuova, 0, 0, 1024, 1024, &x, &y, &larghezza, &altezza));
	CONTROLLA(x == 510 && y == 505 && larghezza == 24 && altezza == 16);

	/* una cella cambiata fuori dalla finestra non si vede */
	scrivi_cella(nuova, 900, 900, MAP_CAT_FREE);
	CONTROLLA(find_changed_rectangle(vecchia, nuova, 400, 400, 200, 200, &x, &y, &larghezza, &altezza));
	CONTROLLA(x == 510 && y == 505 && larghezza == 24 && altezza == 16);
	CONTROLLA(!find_changed_rectangle(vecchia, nuova, 0, 0, 500, 500, &x, &y, &larghezza, &altezza));

	/* copiato il rettangolo, dentro la finestra le griglie tornano uguali */
	copy_category_rectangle(vecchia, nuova, 510, 505, 24, 16);
	CONTROLLA(!find_changed_rectangle(vecchia, nuova, 400, 400, 200, 200, &x, &y, &larghezza, &altezza));
	CONTROLLA(get_map_category(vecchia, 533, 520) == MAP_CAT_VOID);
	CONTROLLA(get_map_category(vecchia, 900, 900) == MAP_CAT_UNKNOWN);
}

static void avvio_senza_mappa(void)
{
	/* non partito dalla base: mappa salvata vuota (il robot non ha mappa), poi mappa dal vivo vuota */
	partito_dalla_base = false;
	memset(mappa_robot, 0, sizeof mappa_robot);
	versione_server = -1;
	richieste = 0;
	init_live_map();
	CONTROLLA(read_live_map_file_crc()[0] == '\0');

	CONTROLLA(send_live_map(NULL));
	CONTROLLA(strcmp(ultimo_salvato, "startup") == 0 && dimensione_salvata == 0);
	CONTROLLA(strcmp(ultimo_metodo, "PUT") == 0);
	CONTROLLA(versione_server == 0);
	CONTROLLA(dimensione_ultimo_corpo == 13 + 1 + 4);       /* una sola cella sconosciuta */
	CONTROLLA(richieste == 2);

	/* niente posizioni nuove: niente richieste */
	CONTROLLA(send_live_map(NULL));
	CONTROLLA(richieste == 2);
}

static void differenze(void)
{
	/* il robot si muove e la mappatura scrive attorno a lui */
	riempi(mappa_robot, 500, 500, 30, 30, MAP_CAT_FREE);
	robot_in(0.125, 0.125);     /* cella (514, 514) */
	CONTROLLA(send_live_map(NULL));
	CONTROLLA(versione_server == 1);
	CONTROLLA(strncmp(ultimo_percorso, "/api/device/map/patch?base_version=0&version=1", 46) == 0);
	CONTROLLA(memcmp(mappa_server, mappa_robot, sizeof mappa_robot) == 0);

	/* la finestra letta è la posizione allargata della portata del lidar (6 m = 120 celle) più il margine */
	int portata = (int)(LIDAR_MAPPING_MAX_RANGE / FINE_MAP_CELL_SIZE) + LIVE_MAP_MARGIN_CELLS;
	CONTROLLA(ultima_finestra[0] == 514 - portata && ultima_finestra[1] == 514 - portata);
	CONTROLLA(ultima_finestra[2] == 2 * portata + 1 && ultima_finestra[3] == 2 * portata + 1);

	/* robot fermo e mappa uguale: nessuna richiesta */
	int prima = richieste;
	CONTROLLA(send_live_map(NULL));
	CONTROLLA(richieste == prima);

	/* due posizioni: la finestra le copre tutte e due, e parte solo il rettangolo cambiato */
	scrivi_cella(mappa_robot, 540, 510, MAP_CAT_OCCUPIED);
	robot_in(0.125, 0.125);
	robot_in(1.125, 0.625);     /* cella (534, 524) */
	CONTROLLA(send_live_map(NULL));
	CONTROLLA(ultima_finestra[2] == 2 * portata + 21 && ultima_finestra[3] == 2 * portata + 11);
	CONTROLLA(versione_server == 2);
	CONTROLLA(dimensione_ultimo_corpo == 13 + 1 + 4);       /* una cella sola */
	CONTROLLA(get_map_category(mappa_server, 540, 510) == MAP_CAT_OCCUPIED);

	/* con la posizione non valida il rettangolo non si allarga */
	posizione.valid = false;
	scrivi_cella(mappa_robot, 541, 510, MAP_CAT_OCCUPIED);
	track_live_map_position();
	prima = richieste;
	CONTROLLA(send_live_map(NULL));
	CONTROLLA(richieste == prima);

	/* la finestra non esce dalla griglia */
	robot_in(-25.5, 25.5);
	scrivi_cella(mappa_robot, 3, 1020, MAP_CAT_FREE);
	CONTROLLA(send_live_map(NULL));
	CONTROLLA(ultima_finestra[0] == 0 && ultima_finestra[1] + ultima_finestra[3] == 1024);
	CONTROLLA(versione_server == 3);
	CONTROLLA(get_map_category(mappa_server, 3, 1020) == MAP_CAT_FREE);
	/* la cella 541 era fuori da questa finestra: parte quando il robot ci ripassa vicino */
	CONTROLLA(get_map_category(mappa_server, 541, 510) == MAP_CAT_UNKNOWN);
	robot_in(1.125, 0.625);
	CONTROLLA(send_live_map(NULL));
	CONTROLLA(memcmp(mappa_server, mappa_robot, sizeof mappa_robot) == 0);
	CONTROLLA(versione_server == 4);
}

static void rete_giu_e_poi_torna(void)
{
	/* invio fallito: il rettangolo resta, le celle perse partono all'invio dopo */
	scrivi_cella(mappa_robot, 520, 520, MAP_CAT_VOID);
	robot_in(0.5, 0.5);
	rete_giu = true;
	CONTROLLA(!send_live_map(NULL));
	CONTROLLA(versione_server == 4);
	scrivi_cella(mappa_robot, 521, 520, MAP_CAT_VOID);
	rete_giu = false;
	CONTROLLA(send_live_map(NULL));
	CONTROLLA(versione_server == 5);
	CONTROLLA(memcmp(mappa_server, mappa_robot, sizeof mappa_robot) == 0);

	/* la mappatura non lascia la finestra libera: si riprova al giro dopo, senza errori di rete */
	scrivi_cella(mappa_robot, 522, 520, MAP_CAT_VOID);
	robot_in(0.5, 0.5);
	copia_riesce = false;
	CONTROLLA(send_live_map(NULL));
	CONTROLLA(versione_server == 5);
	copia_riesce = true;
	CONTROLLA(send_live_map(NULL));
	CONTROLLA(versione_server == 6);
}

static void server_ripartito_409(void)
{
	/* il server riparte e perde la mappa dal vivo: alla prossima differenza risponde 409 */
	versione_server = -1;
	memset(mappa_server, 0, sizeof mappa_server);
	scrivi_cella(mappa_robot, 523, 520, MAP_CAT_VOID);
	robot_in(0.5, 0.5);
	CONTROLLA(send_live_map(NULL));                 /* 409 */
	CONTROLLA(versione_server == -1);

	/* il robot rimanda la sua copia privata intera (senza leggere la mappa dalla mappatura), poi la differenza in sospeso */
	ultima_finestra[2] = -1;
	CONTROLLA(send_live_map(NULL));
	CONTROLLA(strncmp(ultimo_percorso, "/api/device/map?version=6", 25) == 0);
	CONTROLLA(ultima_finestra[2] == -1);
	CONTROLLA(versione_server == 6);
	CONTROLLA(get_map_category(mappa_server, 522, 520) == MAP_CAT_VOID);
	CONTROLLA(get_map_category(mappa_server, 523, 520) == MAP_CAT_FREE);        /* com'era nell'ultima versione confermata */
	CONTROLLA(send_live_map(NULL));
	CONTROLLA(versione_server == 7);
	CONTROLLA(memcmp(mappa_server, mappa_robot, sizeof mappa_robot) == 0);
}

static void server_senza_differenze_da_mandare(void)
{
	/* il server riparte mentre la mappa del robot non cambia: nessuna differenza da mandare, quindi nessun 409. Il
	 * robot lo scopre dalla risposta allo stato, che dice quale versione ha il server, e rimanda la mappa intera */
	versione_server = -1;
	memset(mappa_server, 0, sizeof mappa_server);
	int prima = richieste;
	CONTROLLA(send_live_map(NULL));
	CONTROLLA(richieste == prima);
	check_server_map_version(7);            /* la stessa del robot: niente da fare */
	CONTROLLA(send_live_map(NULL));
	CONTROLLA(richieste == prima);
	check_server_map_version(-1);           /* il server non ne ha nessuna */
	CONTROLLA(send_live_map(NULL));
	CONTROLLA(versione_server == 7);
	CONTROLLA(memcmp(mappa_server, mappa_robot, sizeof mappa_robot) == 0);
	check_server_map_version(3);            /* il server ne ha una vecchia */
	CONTROLLA(send_live_map(NULL));
	CONTROLLA(richieste == prima + 2);
}

static void mappa_salvata(void)
{
	/* il task aperiodico (quello vero) salva la mappa: il file parte con il motivo "save", e lo stato può dire il suo CRC */
	char percorso[256], crc[MAP_CRC_TEXT_LEN];
	map_file_path(percorso, sizeof percorso, "");
	CONTROLLA(strstr(percorso, "/map.rtm") != NULL);
	uint32_t salvataggi = read_map_save_count();
	request_map_save();
	job_aperiodic(NULL);
	CONTROLLA(read_map_save_count() == salvataggi + 1);
	CONTROLLA(!read_map_save_failed());

	/* il file è quello che darebbe la codifica della mappa del robot */
	byte_buffer_t b = {0};
	int x, y, larghezza, altezza;
	find_map_rectangle(mappa_robot, &x, &y, &larghezza, &altezza);
	encode_map_rectangle(mappa_robot, x, y, larghezza, altezza, NULL, 0, append_bytes, &b);
	FILE *f;

	CONTROLLA(send_live_map(NULL));
	CONTROLLA(strcmp(ultimo_salvato, "save") == 0 && dimensione_salvata == b.size);
	uint32_t atteso = map_crc32(0, b.data, b.size - 4);
	snprintf(crc, sizeof crc, "%08x", (unsigned)atteso);
	CONTROLLA(strcmp(read_live_map_file_crc(), crc) == 0);

	/* controllo del file scaricato per map_select: formato e CRC atteso */
	CONTROLLA(check_map_file(percorso, crc));
	CONTROLLA(!check_map_file(percorso, "00000000"));
	CONTROLLA(!check_map_file("/non/esiste", crc));

	/* nuovo avvio dalla base: la copia privata parte dal file, e al server arriva identica alla mappa del robot */
	partito_dalla_base = true;
	versione_server = -1;
	memset(mappa_server, 0, sizeof mappa_server);
	init_live_map();
	CONTROLLA(strcmp(read_live_map_file_crc(), crc) == 0);
	CONTROLLA(send_live_map(NULL));
	CONTROLLA(strcmp(ultimo_salvato, "startup") == 0 && dimensione_salvata == b.size);
	CONTROLLA(versione_server == 0);
	CONTROLLA(memcmp(mappa_server, mappa_robot, sizeof mappa_robot) == 0);

	/* un file rovinato all'avvio: la copia privata resta vuota, come la mappa della mappatura */
	f = fopen(percorso, "wb");
	fwrite(b.data, 1, b.size - 3, f);
	fclose(f);
	init_live_map();
	CONTROLLA(send_live_map(NULL));
	CONTROLLA(get_map_category(mappa_server, 510, 510) == MAP_CAT_UNKNOWN);
	free_bytes(&b);

	/* sostituzione del file con quello scaricato, poi cancellazione: da lì il task aperiodico non salva più */
	char scaricato[256];
	map_file_path(scaricato, sizeof scaricato, MAP_DOWNLOAD_EXT);
	f = fopen(scaricato, "wb");
	fwrite("mappa nuova", 1, 11, f);
	fclose(f);
	bool riuscita = false;
	uint32_t richiesta = request_map_replace();
	CONTROLLA(!read_map_file_request_result(richiesta, &riuscita));
	job_aperiodic(NULL);
	CONTROLLA(read_map_file_request_result(richiesta, &riuscita) && riuscita);
	f = fopen(percorso, "rb");
	char testo[16] = {0};
	CONTROLLA(f != NULL && fread(testo, 1, 11, f) == 11 && strcmp(testo, "mappa nuova") == 0);
	if (f) fclose(f);
	CONTROLLA(fopen(scaricato, "rb") == NULL);

	salvataggi = read_map_save_count();
	request_map_save();
	job_aperiodic(NULL);
	CONTROLLA(read_map_save_count() == salvataggi);     /* niente salvataggio sopra il file nuovo */

	richiesta = request_map_delete();
	job_aperiodic(NULL);
	CONTROLLA(read_map_file_request_result(richiesta, &riuscita) && riuscita);
	CONTROLLA(fopen(percorso, "rb") == NULL);
	/* cancellare un file che non c'è va bene lo stesso */
	richiesta = request_map_delete();
	job_aperiodic(NULL);
	CONTROLLA(read_map_file_request_result(richiesta, &riuscita) && riuscita);
	/* senza il file scaricato la sostituzione non riesce */
	richiesta = request_map_replace();
	job_aperiodic(NULL);
	CONTROLLA(read_map_file_request_result(richiesta, &riuscita) && !riuscita);
}

int main(void)
{
	if (!mkdtemp(cartella)) return 1;
	setenv("RTES_MAPS", cartella, 1);       /* la cartella delle mappe del task aperiodico */
	init_aperiodic();
	sem_init(&aperiodic_task.sveglia, 0, 0);    /* il task non è avviato: le sveglie delle richieste restano lì */
	ESEGUI(crc_standard);
	ESEGUI(codifica_rtm1);
	ESEGUI(codifica_e_decodifica);
	ESEGUI(confronto);
	ESEGUI(avvio_senza_mappa);
	ESEGUI(differenze);
	ESEGUI(rete_giu_e_poi_torna);
	ESEGUI(server_ripartito_409);
	ESEGUI(server_senza_differenze_da_mandare);
	ESEGUI(mappa_salvata);
	rmdir(cartella);
	return TEST_FINE();
}
