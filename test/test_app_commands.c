/* Test dell'interprete dei comandi JSON dell'app (tasks/vacuum/cloud/app_commands.c, docs/app.md sezione 9.4).
 * Navigazione, mappatura, task aperiodico e driver sono finti e stanno qui sotto: il test controlla cosa chiede
 * l'interprete a ognuno e quale esito ritorna all'app. */
#include <stdlib.h>
#include <string.h>

#include "test.h"
#include "../driver/driver.h"
#include "../tasks/vacuum/aperiodic.h"
#include "../tasks/vacuum/mapping.h"
#include "../tasks/vacuum/navigation.h"
#include "../tasks/vacuum/cloud/app_commands.h"
#include "../tasks/vacuum/cloud/config_check.h"

/* ------------------------------------------------------------------------------------------- pezzi finti --- */

/* driver */
static drv_status_t stato_driver;
static drv_sensors_t sensori;
static int azzeramenti;             /* chiamate a drv_clear_fault */
static bool causa_presente;         /* il guasto non si azzera */

uint32_t drv_status(drv_status_t *out) { *out = stato_driver; return 1; }
uint32_t drv_sensors(drv_sensors_t *out) { *out = sensori; return 1; }
int drv_clear_fault(void)
{
	azzeramenti++;
	if (causa_presente) return -1;
	stato_driver.phase = DRV_READY;
	return 0;
}

/* navigazione: esegue subito il comando e risponde con l'esito deciso dal test */
static navigation_command_t comandi[16];
static int num_comandi;
static nav_command_result_t esito_navigazione;
static bool navigazione_risponde = true;
static navigation_status_t stato_navigazione;

void send_navigation_command(const navigation_command_t *command)
{
	if (num_comandi < 16) comandi[num_comandi] = unpack_navigation_command(pack_navigation_command(command));
	num_comandi++;
	if (navigazione_risponde) {
		stato_navigazione.last_command_num = command->num;
		stato_navigazione.last_command_result = esito_navigazione;
	}
}
navigation_status_t read_navigation_status(void) { return stato_navigazione; }

/* mappatura */
static bool partito_dalla_base = true;
static mapping_map_state stato_mappa = MAP_NORMAL;
static uint32_t zone_applicate;
static bool mappatura_applica = true;
bool read_started_on_base(void) { return partito_dalla_base; }
mapping_correction_t read_mapping_correction(void) { return (mapping_correction_t){ .state = stato_mappa }; }
uint32_t read_applied_user_zones_version(void) { return zone_applicate; }

/* task aperiodico */
static mapping_forbidden_zone_t zone_chieste[MAX_FORBIDDEN_ZONES];
static int num_zone_chieste = -1;
static uint32_t richieste_zone, richieste_file;
static int sostituzioni, cancellazioni;
static bool file_riesce = true;

uint32_t request_user_zones(const mapping_forbidden_zone_t *zones, int num_zones)
{
	num_zone_chieste = num_zones;
	for (int i = 0; i < num_zones; i++) zone_chieste[i] = zones[i];
	richieste_zone++;
	if (mappatura_applica) zone_applicate = richieste_zone;
	return richieste_zone;
}
uint32_t request_map_replace(void) { sostituzioni++; return ++richieste_file; }
uint32_t request_map_delete(void) { cancellazioni++; return ++richieste_file; }
bool read_map_file_request_result(uint32_t request_num, bool *ok) { *ok = file_riesce; return request_num == richieste_file; }
int map_file_path(char *path, size_t size, const char *suffix) { snprintf(path, size, "/cartella/map.rtm%s", suffix); return 0; }

/* scaricamento e controllo della mappa */
static int mappa_scaricata;
static char percorso_scaricato[256], crc_controllato[16];
static bool scaricamento_riesce = true, file_valido = true;

static bool scarica_mappa(int map_id, const char *path)
{
	mappa_scaricata = map_id;
	snprintf(percorso_scaricato, sizeof percorso_scaricato, "%s", path);
	return scaricamento_riesce;
}
bool check_map_file(const char *path, const char *expected_crc_text)
{
	(void)path;
	snprintf(crc_controllato, sizeof crc_controllato, "%s", expected_crc_text);
	return file_valido;
}

/* ----------------------------------------------------------------------------------------------- test --- */

static app_command_result_t esegui(const char *json)
{
	app_command_result_t result;
	num_comandi = 0;
	run_app_command(json, scarica_mappa, &result);
	return result;
}

static bool rifiutato(app_command_result_t result, const char *reason)
{
	return !result.ok && result.reason != NULL && strcmp(result.reason, reason) == 0 && result.message[0] != '\0';
}

static void comandi_della_navigazione(void)
{
	struct { const char *json; nav_command_type_t type; } casi[] = {
		{ "{\"type\":\"start\"}", NAV_COMMAND_START },
		{ "{\"type\":\"pause\"}", NAV_COMMAND_PAUSE },
		{ "{\"type\":\"resume\"}", NAV_COMMAND_RESUME },
		{ "{\"type\":\"home\"}", NAV_COMMAND_HOME },
		{ "{\"type\":\"remote_take\"}", NAV_COMMAND_REMOTE_TAKE },
		{ "{\"type\":\"remote_keepalive\"}", NAV_COMMAND_REMOTE_KEEPALIVE },
		{ "{\"type\":\"remote_release\"}", NAV_COMMAND_REMOTE_RELEASE },
		{ "{\"id\":57,\"type\":\"start\"}", NAV_COMMAND_START },      /* come arriva dal server, con l'id */
	};
	uint16_t last = 0;
	for (unsigned i = 0; i < sizeof(casi) / sizeof(casi[0]); i++) {
		app_command_result_t result = esegui(casi[i].json);
		CONTROLLA(result.ok && result.reason == NULL && result.message[0] == '\0' && !result.is_restart_needed);
		CONTROLLA(num_comandi == 1 && comandi[0].type == casi[i].type);
		/* ogni comando ha un numero nuovo, mai 0 */
		CONTROLLA(comandi[0].num != 0 && comandi[0].num != last);
		last = comandi[0].num;
	}
	CONTROLLA(azzeramenti == 0);
}

static void rifiuti_della_navigazione(void)
{
	struct { nav_command_result_t result; const char *json; const char *reason; } casi[] = {
		{ NAV_RESULT_BUSY, "{\"type\":\"start\"}", "busy" },
		{ NAV_RESULT_NOT_RUNNING, "{\"type\":\"pause\"}", "not_running" },
		{ NAV_RESULT_NOT_PAUSED, "{\"type\":\"resume\"}", "not_paused" },
		{ NAV_RESULT_ALREADY_HOME, "{\"type\":\"home\"}", "already_home" },
		{ NAV_RESULT_POSITION_LOST, "{\"type\":\"start\"}", "position_lost" },
		{ NAV_RESULT_POSITION_LOST, "{\"type\":\"remote_move\",\"v\":0.1,\"omega\":0}", "position_lost" },
		{ NAV_RESULT_IN_FAULT, "{\"type\":\"resume\"}", "cause_still_present" },
	};
	for (unsigned i = 0; i < sizeof(casi) / sizeof(casi[0]); i++) {
		esito_navigazione = casi[i].result;
		CONTROLLA(rifiutato(esegui(casi[i].json), casi[i].reason));
	}
	esito_navigazione = NAV_RESULT_OK;
}

static void telecomando(void)
{
	app_command_result_t result = esegui("{\"type\":\"remote_move\",\"v\":0.2,\"omega\":-0.5}");
	CONTROLLA(result.ok && num_comandi == 1 && comandi[0].type == NAV_COMMAND_REMOTE_MOVE);
	CONTROLLA(comandi[0].v > 0.1999 && comandi[0].v < 0.2001 && comandi[0].omega < -0.4999 && comandi[0].omega > -0.5001);

	result = esegui("{\"type\":\"remote_cleaning\",\"brushes\":true,\"suction\":false}");
	CONTROLLA(result.ok && comandi[0].type == NAV_COMMAND_REMOTE_CLEANING && comandi[0].brushes && !comandi[0].suction);

	/* con il cassetto l'aspirazione parte */
	sensori.s.dustbin = 1;
	result = esegui("{\"type\":\"remote_cleaning\",\"brushes\":false,\"suction\":true}");
	CONTROLLA(result.ok && comandi[0].type == NAV_COMMAND_REMOTE_CLEANING && !comandi[0].brushes && comandi[0].suction);

	/* senza il cassetto è rifiutata, ma il telecomando si prende lo stesso (un segno di vita, senza cambiare le spazzole) */
	sensori.s.dustbin = 0;
	result = esegui("{\"type\":\"remote_cleaning\",\"brushes\":true,\"suction\":true}");
	CONTROLLA(rifiutato(result, "dustbin_missing"));
	CONTROLLA(num_comandi == 1 && comandi[0].type == NAV_COMMAND_REMOTE_KEEPALIVE);
	/* senza aspirazione il cassetto non serve */
	result = esegui("{\"type\":\"remote_cleaning\",\"brushes\":true,\"suction\":false}");
	CONTROLLA(result.ok);

	/* parametri mancanti o sbagliati: niente arriva alla navigazione */
	const char *sbagliati[] = {
		"{\"type\":\"remote_move\",\"v\":0.2}", "{\"type\":\"remote_move\",\"v\":\"veloce\",\"omega\":0}",
		"{\"type\":\"remote_cleaning\",\"brushes\":1,\"suction\":true}", "{\"type\":\"remote_cleaning\"}",
	};
	for (unsigned i = 0; i < sizeof(sbagliati) / sizeof(sbagliati[0]); i++) {
		CONTROLLA(rifiutato(esegui(sbagliati[i]), "invalid_command"));
		CONTROLLA(num_comandi == 0);
	}
}

static void fault(void)
{
	/* fault_reset senza guasto */
	stato_driver.phase = DRV_READY;
	CONTROLLA(rifiutato(esegui("{\"type\":\"fault_reset\"}"), "not_in_fault"));
	CONTROLLA(azzeramenti == 0 && num_comandi == 0);

	/* fault_reset con il guasto: lo azzera l'interprete, la navigazione non riceve comandi */
	stato_driver.phase = DRV_FAULT;
	CONTROLLA(esegui("{\"type\":\"fault_reset\"}").ok);
	CONTROLLA(azzeramenti == 1 && num_comandi == 0 && stato_driver.phase == DRV_READY);

	/* la causa c'è ancora */
	stato_driver.phase = DRV_FAULT;
	causa_presente = true;
	CONTROLLA(rifiutato(esegui("{\"type\":\"fault_reset\"}"), "cause_still_present"));

	/* start, resume e home azzerano anche il guasto, prima di passare alla navigazione; se non si azzera non ci passano */
	const char *azzerano[] = { "{\"type\":\"start\"}", "{\"type\":\"resume\"}", "{\"type\":\"home\"}" };
	for (unsigned i = 0; i < 3; i++) {
		stato_driver.phase = DRV_FAULT;
		causa_presente = true;
		azzeramenti = 0;
		CONTROLLA(rifiutato(esegui(azzerano[i]), "cause_still_present"));
		CONTROLLA(azzeramenti == 1 && num_comandi == 0);
		causa_presente = false;
		CONTROLLA(esegui(azzerano[i]).ok);
		CONTROLLA(azzeramenti == 2 && num_comandi == 1 && stato_driver.phase == DRV_READY);
	}

	/* pause e i comandi del telecomando non azzerano il guasto: il telecomando si può prendere anche in guasto */
	stato_driver.phase = DRV_FAULT;
	azzeramenti = 0;
	CONTROLLA(esegui("{\"type\":\"pause\"}").ok);
	CONTROLLA(esegui("{\"type\":\"remote_take\"}").ok);
	CONTROLLA(esegui("{\"type\":\"remote_move\",\"v\":0.1,\"omega\":0}").ok);
	CONTROLLA(azzeramenti == 0 && stato_driver.phase == DRV_FAULT);
	stato_driver.phase = DRV_READY;
}

static void zone(void)
{
	app_command_result_t result = esegui("{\"type\":\"zones\",\"zones\":[{\"x\":1.0,\"y\":-2.0,\"w\":0.5,\"h\":0.25},{\"x\":-3,\"y\":3,\"w\":1,\"h\":1}]}");
	CONTROLLA(result.ok && num_comandi == 0);
	CONTROLLA(num_zone_chieste == 2);
	CONTROLLA(zone_chieste[0].x == 1.0 && zone_chieste[0].y == -2.0 && zone_chieste[0].w == 0.5 && zone_chieste[0].h == 0.25);
	CONTROLLA(zone_chieste[1].x == -3 && zone_chieste[1].h == 1);

	/* lista vuota: toglie tutte le zone */
	CONTROLLA(esegui("{\"type\":\"zones\",\"zones\":[]}").ok && num_zone_chieste == 0);

	/* 15 zone vanno bene, 16 no */
	char molte[2048] = "{\"type\":\"zones\",\"zones\":[";
	for (int i = 0; i < 15; i++) strcat(molte, i ? ",{\"x\":1,\"y\":1,\"w\":1,\"h\":1}" : "{\"x\":1,\"y\":1,\"w\":1,\"h\":1}");
	char troppe[2048];
	snprintf(troppe, sizeof troppe, "%s,{\"x\":1,\"y\":1,\"w\":1,\"h\":1}]}", molte);
	strcat(molte, "]}");
	CONTROLLA(esegui(molte).ok && num_zone_chieste == 15);
	uint32_t prima = richieste_zone;
	CONTROLLA(rifiutato(esegui(troppe), "too_many_zones"));

	/* fuori dalla griglia (±25,6 m) o più piccola di una cella */
	CONTROLLA(rifiutato(esegui("{\"type\":\"zones\",\"zones\":[{\"x\":25.5,\"y\":0,\"w\":0.5,\"h\":0.5}]}"), "outside_map"));
	CONTROLLA(rifiutato(esegui("{\"type\":\"zones\",\"zones\":[{\"x\":0,\"y\":-26,\"w\":0.5,\"h\":0.5}]}"), "outside_map"));
	CONTROLLA(rifiutato(esegui("{\"type\":\"zones\",\"zones\":[{\"x\":0,\"y\":0,\"w\":0.01,\"h\":0.5}]}"), "outside_map"));
	CONTROLLA(rifiutato(esegui("{\"type\":\"zones\",\"zones\":[{\"x\":0,\"y\":0,\"w\":0.5}]}"), "invalid_command"));
	CONTROLLA(rifiutato(esegui("{\"type\":\"zones\"}"), "invalid_command"));

	/* sessione non partita dalla base, posizione persa */
	partito_dalla_base = false;
	CONTROLLA(rifiutato(esegui("{\"type\":\"zones\",\"zones\":[]}"), "not_started_on_base"));
	partito_dalla_base = true;
	stato_mappa = MAP_LOST;
	CONTROLLA(rifiutato(esegui("{\"type\":\"zones\",\"zones\":[]}"), "position_lost"));
	stato_mappa = MAP_NORMAL;
	CONTROLLA(richieste_zone == prima);         /* nessuno dei rifiuti è arrivato al task aperiodico */
}

static void cambio_di_mappa(void)
{
	stato_navigazione.state = NAV_STATE_SLEEP;
	stato_driver.on_base = 1;

	/* map_select: scarica in map.rtm.download, controlla il CRC, chiede la sostituzione, poi serve il riavvio */
	app_command_result_t result = esegui("{\"id\":9,\"type\":\"map_select\",\"map_id\":42,\"crc\":\"9c41e2a0\"}");
	CONTROLLA(result.ok && result.is_restart_needed);
	CONTROLLA(mappa_scaricata == 42 && strcmp(percorso_scaricato, "/cartella/map.rtm.download") == 0);
	CONTROLLA(strcmp(crc_controllato, "9c41e2a0") == 0);
	CONTROLLA(sostituzioni == 1 && cancellazioni == 0 && num_comandi == 0);

	/* scaricamento fallito, o file che non supera i controlli: la mappa vecchia resta e non si riavvia */
	scaricamento_riesce = false;
	result = esegui("{\"type\":\"map_select\",\"map_id\":42,\"crc\":\"9c41e2a0\"}");
	CONTROLLA(rifiutato(result, "download_failed") && !result.is_restart_needed && sostituzioni == 1);
	scaricamento_riesce = true;
	file_valido = false;
	CONTROLLA(rifiutato(esegui("{\"type\":\"map_select\",\"map_id\":42,\"crc\":\"9c41e2a0\"}"), "download_failed"));
	file_valido = true;
	file_riesce = false;
	result = esegui("{\"type\":\"map_select\",\"map_id\":42,\"crc\":\"9c41e2a0\"}");
	CONTROLLA(rifiutato(result, "download_failed") && !result.is_restart_needed);
	file_riesce = true;
	CONTROLLA(rifiutato(esegui("{\"type\":\"map_select\",\"crc\":\"9c41e2a0\"}"), "invalid_command"));
	CONTROLLA(rifiutato(esegui("{\"type\":\"map_select\",\"map_id\":42}"), "invalid_command"));

	/* dalla FIFO non c'è modo di scaricare */
	app_command_result_t dalla_fifo;
	run_app_command("{\"type\":\"map_select\",\"map_id\":42,\"crc\":\"9c41e2a0\"}", NULL, &dalla_fifo);
	CONTROLLA(rifiutato(dalla_fifo, "download_failed"));

	/* map_reset: chiede la cancellazione, poi serve il riavvio */
	result = esegui("{\"type\":\"map_reset\"}");
	CONTROLLA(result.ok && result.is_restart_needed && cancellazioni == 1);

	/* solo a riposo sulla base */
	int prima = sostituzioni + cancellazioni;
	stato_driver.on_base = 0;
	CONTROLLA(rifiutato(esegui("{\"type\":\"map_reset\"}"), "not_resting"));
	stato_driver.on_base = 1;
	nav_state_t non_a_riposo[] = { NAV_STATE_PAUSE, NAV_STATE_FOLLOW_PATH, NAV_STATE_REMOTE, NAV_STATE_FAILURE, NAV_STATE_WAIT };
	for (unsigned i = 0; i < sizeof(non_a_riposo) / sizeof(non_a_riposo[0]); i++) {
		stato_navigazione.state = non_a_riposo[i];
		CONTROLLA(rifiutato(esegui("{\"type\":\"map_select\",\"map_id\":42,\"crc\":\"9c41e2a0\"}"), "not_resting"));
		CONTROLLA(rifiutato(esegui("{\"type\":\"map_reset\"}"), "not_resting"));
	}
	CONTROLLA(sostituzioni + cancellazioni == prima);
	stato_navigazione.state = NAV_STATE_SLEEP;
}

static void comandi_non_validi(void)
{
	const char *sbagliati[] = {
		"", "niente", "{", "[]", "42", "{\"type\":5}", "{\"tipo\":\"start\"}", "{\"type\":\"vola\"}", "{\"type\":\"START\"}",
	};
	for (unsigned i = 0; i < sizeof(sbagliati) / sizeof(sbagliati[0]); i++) {
		CONTROLLA(rifiutato(esegui(sbagliati[i]), "invalid_command"));
		CONTROLLA(num_comandi == 0);
	}
}

static void navigazione_che_non_risponde(void)
{
	/* se la navigazione non esegue il comando entro COMMAND_RESULT_TIMEOUT l'interprete non resta bloccato */
	navigazione_risponde = false;
	int64_t inizio = rt_now();
	CONTROLLA(rifiutato(esegui("{\"type\":\"start\"}"), "no_answer"));
	int64_t durata = rt_now() - inizio;
	CONTROLLA(durata >= COMMAND_RESULT_TIMEOUT && durata < COMMAND_RESULT_TIMEOUT + RT_MS(500));
	navigazione_risponde = true;

	/* lo stesso se la mappatura non applica le zone */
	mappatura_applica = false;
	CONTROLLA(rifiutato(esegui("{\"type\":\"zones\",\"zones\":[]}"), "no_answer"));
	mappatura_applica = true;
}

int main(void)
{
	stato_driver.phase = DRV_READY;
	ESEGUI(comandi_della_navigazione);
	ESEGUI(rifiuti_della_navigazione);
	ESEGUI(telecomando);
	ESEGUI(fault);
	ESEGUI(zone);
	ESEGUI(cambio_di_mappa);
	ESEGUI(comandi_non_validi);
	ESEGUI(navigazione_che_non_risponde);
	return TEST_FINE();
}
