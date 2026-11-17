#include <math.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/driver.h"
#include "third_party/cjson/cJSON.h"

#include "app_commands.h"
#include "config_check.h"
#include "live_map.h"
#include "wifi_link.h"
#include "../aperiodic.h"
#include "../mapping.h"
#include "../navigation.h"

// mutex fra chi chiama l'interprete (thread dei comandi della rete e FIFO): un comando alla volta. I task non lo usano mai
static pthread_mutex_t commands_mutex = PTHREAD_MUTEX_INITIALIZER;

// numero dell'ultimo comando mandato alla navigazione (0 = nessun comando, quindi si salta)
static uint16_t navigation_command_num;

// un motivo di rifiuto e il testo che l'app mostra all'utente
typedef struct {
    const char *reason;
    const char *message;
} reason_message_t;

static const reason_message_t REASON_MESSAGES[] = {
    { "busy",                   "Il robot sta già facendo una missione." },
    { "not_running",            "Non c'è una missione in corso." },
    { "not_paused",             "Il robot non è in pausa." },
    { "already_home",           "Il robot è già sulla base." },
    { "position_lost",          "Il robot non sa dove si trova." },
    { "cause_still_present",    "La causa del guasto c'è ancora." },
    { "not_in_fault",           "Il robot non è in guasto." },
    { "dustbin_missing",        "Manca il cassetto della polvere." },
    { "not_started_on_base",    "Il robot non è partito dalla base: le zone non si possono cambiare." },
    { "too_many_zones",         "Troppe zone proibite." },
    { "too_many_walls",         "Troppi muri virtuali." },
    { "outside_map",            "Una zona o un muro esce dalla mappa o è troppo piccolo." },
    { "not_resting",            "Il robot deve essere a riposo sulla base." },
    { "download_failed",        "Non sono riuscito a scaricare la mappa." },
    { "no_map",                 "Il robot non ha una mappa caricata: deve essere partito dalla base con una mappa." },
    { "not_reachable",          "Il robot non può arrivare lì: il punto non è libero sulla mappa o non c'è un percorso." },
    { "invalid_command",        "Comando non valido." },
    { "no_answer",              "Il robot non ha eseguito il comando in tempo." },
    { "wifi_unreachable",       "Il programma del Wi-Fi del robot non risponde." },
};

// funzione helper che scrive un rifiuto nell'esito
static void refuse(app_command_result_t *result, const char *reason) {

    result->ok = false;
    result->reason = reason;
    result->message[0] = '\0';

    for (size_t i = 0; i < sizeof REASON_MESSAGES / sizeof REASON_MESSAGES[0]; i++) {
        if (strcmp(REASON_MESSAGES[i].reason, reason) == 0) {
            snprintf(result->message, sizeof result->message, "%s", REASON_MESSAGES[i].message);
            break;
        }
    }
}

// funzione helper che porta l'esito della navigazione nel motivo del rifiuto. NULL = eseguito
static const char *navigation_result_reason(nav_command_result_t navigation_result) {
    switch (navigation_result) {
        case NAV_RESULT_OK:             return NULL;
        case NAV_RESULT_BUSY:           return "busy";
        case NAV_RESULT_NOT_RUNNING:    return "not_running";
        case NAV_RESULT_NOT_PAUSED:     return "not_paused";
        case NAV_RESULT_ALREADY_HOME:   return "already_home";
        case NAV_RESULT_POSITION_LOST:  return "position_lost";
        case NAV_RESULT_IN_FAULT:       return "cause_still_present";
        case NAV_RESULT_NOT_RESTING:    return "not_resting";
        case NAV_RESULT_NO_MAP:         return "no_map";
        case NAV_RESULT_NOT_REACHABLE:  return "not_reachable";
        default:                        return "invalid_command";
    }
}

// funzione helper che manda un comando alla navigazione e aspetta che lo esegua (lo legge al job dopo, entro 100 ms):
// l'esito sta in navigation_status_t. Solo così il comando dopo non sovrascrive questo
static void run_navigation_command(navigation_command_t command, app_command_result_t *result) {

    // numero nuovo, mai 0 (che vuol dire nessun comando)
    navigation_command_num++;
    if (navigation_command_num == 0) navigation_command_num = 1;
    command.num = navigation_command_num;

    send_navigation_command(&command);

    int64_t limit = rt_now() + COMMAND_RESULT_TIMEOUT;
    for (;;) {
        navigation_status_t status = read_navigation_status();
        if (status.last_command_num == command.num) {
            const char *reason = navigation_result_reason(status.last_command_result);
            if (reason != NULL) refuse(result, reason);
            return;
        }
        if (rt_now() > limit) {
            refuse(result, "no_answer");
            return;
        }
        rt_sleep_until(rt_now() + COMMAND_RESULT_POLL);
    }
}

// funzione helper che, se il driver è in guasto, prova ad azzerarlo. Ritorna false se la causa c'è ancora.
// drv_clear_fault aspetta fino a 50 ms: per questo la chiama l'interprete e non un task real-time
static bool clear_fault_if_any(void) {

    drv_status_t drv_state = {0};
    drv_status(&drv_state);

    if (drv_state.phase != DRV_FAULT) return true;

    return drv_clear_fault() == 0;
}

// funzione helper che legge un numero da un campo del comando. Ritorna false se manca o non è un numero finito
static bool read_number(const cJSON *command, const char *name, double *value) {
    const cJSON *item = cJSON_GetObjectItemCaseSensitive(command, name);
    if (!cJSON_IsNumber(item) || !isfinite(item->valuedouble)) return false;
    *value = item->valuedouble;
    return true;
}

// funzione helper che legge un booleano da un campo del comando. Ritorna false se manca o non è un booleano
static bool read_bool(const cJSON *command, const char *name, bool *value) {
    const cJSON *item = cJSON_GetObjectItemCaseSensitive(command, name);
    if (!cJSON_IsBool(item)) return false;
    *value = cJSON_IsTrue(item);
    return true;
}

// comando zones: la lista intera delle zone dell'utente e, se c'è il campo "walls", quella dei muri virtuali
// ([{"x1","y1","x2","y2"}], in metri; senza il campo i muri restano come sono). Richiesta al task aperiodico, poi attesa
// che la mappatura le applichi
static void run_zones_command(const cJSON *command, app_command_result_t *result) {

    const cJSON *list = cJSON_GetObjectItemCaseSensitive(command, "zones");
    if (!cJSON_IsArray(list)) {
        refuse(result, "invalid_command");
        return;
    }

    // le coordinate delle zone hanno senso solo se la sessione è partita dalla base, dove sta l'origine della mappa
    if (!read_started_on_base()) {
        refuse(result, "not_started_on_base");
        return;
    }

    // con la posizione persa o da confermare la mappatura non applica le zone
    if (read_mapping_correction().state != MAP_NORMAL) {
        refuse(result, "position_lost");
        return;
    }

    int num_zones = cJSON_GetArraySize(list);
    if (num_zones > MAX_FORBIDDEN_ZONES - 1) {
        refuse(result, "too_many_zones");
        return;
    }

    // ogni zona deve stare dentro la griglia ed essere larga e alta almeno una cella (gli stessi controlli del task aperiodico)
    mapping_forbidden_zone_t zones[MAX_FORBIDDEN_ZONES - 1];
    const double half_side = FINE_MAP_ORIGIN * FINE_MAP_CELL_SIZE;
    for (int i = 0; i < num_zones; i++) {
        const cJSON *zone = cJSON_GetArrayItem(list, i);
        mapping_forbidden_zone_t *z = &zones[i];
        if (!read_number(zone, "x", &z->x) || !read_number(zone, "y", &z->y) || !read_number(zone, "w", &z->w) || !read_number(zone, "h", &z->h)) {
            refuse(result, "invalid_command");
            return;
        }
        if (z->w < FINE_MAP_CELL_SIZE || z->h < FINE_MAP_CELL_SIZE || z->x < -half_side || z->y < -half_side ||
            z->x + z->w > half_side || z->y + z->h > half_side) {
            refuse(result, "outside_map");
            return;
        }
    }

    // muri virtuali: stessi controlli del task aperiodico
    const cJSON *wall_list = cJSON_GetObjectItemCaseSensitive(command, "walls");
    bool has_walls = wall_list != NULL;
    int num_walls = has_walls ? cJSON_GetArraySize(wall_list) : 0;
    if (has_walls && !cJSON_IsArray(wall_list)) {
        refuse(result, "invalid_command");
        return;
    }
    if (num_walls > MAX_VIRTUAL_WALLS) {
        refuse(result, "too_many_walls");
        return;
    }
    mapping_virtual_wall_t walls[MAX_VIRTUAL_WALLS];
    for (int i = 0; i < num_walls; i++) {
        const cJSON *wall = cJSON_GetArrayItem(wall_list, i);
        mapping_virtual_wall_t *w = &walls[i];
        if (!read_number(wall, "x1", &w->x1) || !read_number(wall, "y1", &w->y1) || !read_number(wall, "x2", &w->x2) || !read_number(wall, "y2", &w->y2)) {
            refuse(result, "invalid_command");
            return;
        }
        if (!is_virtual_wall_valid(*w)) {
            refuse(result, "outside_map");
            return;
        }
    }

    uint32_t request_num = request_user_zones(zones, num_zones, has_walls, walls, num_walls);

    // la mappatura le applica nel suo job (ogni 200 ms); poi il task aperiodico salva la mappa
    int64_t limit = rt_now() + ZONES_RESULT_TIMEOUT;
    while (read_applied_user_zones_version() != request_num) {
        if (rt_now() > limit) {
            refuse(result, "no_answer");
            return;
        }
        rt_sleep_until(rt_now() + COMMAND_RESULT_POLL);
    }
}

// comandi map_select e map_reset: si possono fare solo con il robot a riposo sulla base. Il file viene sostituito (o
// cancellato) dal task aperiodico, che poi ricarica la mappa senza riavvio: l'esito arriva a ricarica fatta
static void run_map_command(const cJSON *command, bool is_select, app_map_download_t download_map, app_command_result_t *result) {

    drv_status_t drv_state = {0};
    drv_status(&drv_state);

    if (read_navigation_status().state != NAV_STATE_SLEEP || !drv_state.on_base) {
        refuse(result, "not_resting");
        return;
    }

    uint32_t request_num;

    if (is_select) {

        // scarica la mappa in map.rtm.download e controllane il formato e il CRC
        double map_id;
        const cJSON *crc = cJSON_GetObjectItemCaseSensitive(command, "crc");
        char download_path[256];
        if (!read_number(command, "map_id", &map_id) || !cJSON_IsString(crc)) {
            refuse(result, "invalid_command");
            return;
        }
        if (download_map == NULL || map_file_path(download_path, sizeof download_path, MAP_DOWNLOAD_EXT) != 0 ||
            !download_map((int)map_id, download_path) || !check_map_file(download_path, crc->valuestring)) {
            refuse(result, "download_failed");
            return;
        }

        request_num = request_map_replace();

    } else request_num = request_map_delete();

    // aspetta che il task aperiodico abbia fatto la rinomina o la cancellazione
    bool is_done_ok = false;
    int64_t limit = rt_now() + MAP_FILE_RESULT_TIMEOUT;
    while (!read_map_file_request_result(request_num, &is_done_ok)) {
        if (rt_now() > limit) {
            refuse(result, "no_answer");
            return;
        }
        rt_sleep_until(rt_now() + COMMAND_RESULT_POLL);
    }

    if (!is_done_ok) refuse(result, is_select ? "download_failed" : "no_answer");
}

// motivi dei rifiuti di rtes-wifi (docs/wifi.md, sezione 7): il testo per l'utente lo manda rtes-wifi
static const char *const WIFI_REASONS[] = { "not_resting", "wifi_busy", "invalid_ssid", "invalid_password", "scan_failed" };

// comandi wifi_scan e wifi_connect: passati a rtes-wifi, il programma che gestisce il Wi-Fi (docs/wifi.md). Il riposo
// lo controlla lui. La scansione porta le reti nei dati dell'esito
static void run_wifi_command(const cJSON *command, bool is_scan, app_command_result_t *result) {

    cJSON *request = cJSON_CreateObject();

    if (is_scan) {
        bool is_refresh = false;
        read_bool(command, "refresh", &is_refresh);
        cJSON_AddStringToObject(request, "type", "scan");
        cJSON_AddBoolToObject(request, "refresh", is_refresh);
    } else {
        const cJSON *ssid = cJSON_GetObjectItemCaseSensitive(command, "ssid");
        const cJSON *security = cJSON_GetObjectItemCaseSensitive(command, "security");
        const cJSON *password = cJSON_GetObjectItemCaseSensitive(command, "password");
        if (!cJSON_IsString(ssid) || !cJSON_IsString(security) || (password != NULL && !cJSON_IsString(password))) {
            cJSON_Delete(request);
            refuse(result, "invalid_command");
            return;
        }
        cJSON_AddStringToObject(request, "type", "connect");
        cJSON_AddStringToObject(request, "ssid", ssid->valuestring);
        cJSON_AddStringToObject(request, "security", security->valuestring);
        cJSON_AddStringToObject(request, "password", password != NULL ? password->valuestring : "");
    }

    char *request_text = cJSON_PrintUnformatted(request);
    cJSON_Delete(request);
    if (request_text == NULL) {
        refuse(result, "invalid_command");
        return;
    }

    char *response_text = wifi_request(request_text, is_scan ? WIFI_SCAN_COMMAND_TIMEOUT_MS : WIFI_CONNECT_COMMAND_TIMEOUT_MS);

    // la password non resta in memoria
    memset(request_text, 0, strlen(request_text));
    free(request_text);

    cJSON *response = cJSON_Parse(response_text != NULL ? response_text : "");
    free(response_text);
    if (!cJSON_IsObject(response)) {
        cJSON_Delete(response);
        refuse(result, "wifi_unreachable");
        return;
    }

    // {"ok": true, "data": {...}} oppure {"ok": false, "reason": ..., "message": ...}
    if (cJSON_IsTrue(cJSON_GetObjectItemCaseSensitive(response, "ok"))) {
        const cJSON *data = cJSON_GetObjectItemCaseSensitive(response, "data");
        if (data != NULL) result->data = cJSON_PrintUnformatted(data);
    } else {
        const cJSON *reason = cJSON_GetObjectItemCaseSensitive(response, "reason");
        const cJSON *message = cJSON_GetObjectItemCaseSensitive(response, "message");
        refuse(result, "invalid_command");
        for (size_t i = 0; i < sizeof WIFI_REASONS / sizeof WIFI_REASONS[0]; i++) {
            if (cJSON_IsString(reason) && strcmp(reason->valuestring, WIFI_REASONS[i]) == 0) result->reason = WIFI_REASONS[i];
        }
        if (cJSON_IsString(message)) snprintf(result->message, sizeof result->message, "%s", message->valuestring);
    }

    cJSON_Delete(response);
}

// funzione helper che esegue un comando già letto dal JSON
static void run_parsed_command(const cJSON *command, app_map_download_t download_map, app_command_result_t *result) {

    const cJSON *type_item = cJSON_GetObjectItemCaseSensitive(command, "type");
    if (!cJSON_IsString(type_item)) {
        refuse(result, "invalid_command");
        return;
    }
    const char *type = type_item->valuestring;

    navigation_command_t navigation_command = {0};

    if (strcmp(type, "fault_reset") == 0) {

        drv_status_t drv_state = {0};
        drv_status(&drv_state);

        if (drv_state.phase != DRV_FAULT) refuse(result, "not_in_fault");
        else if (drv_clear_fault() != 0) refuse(result, "cause_still_present");

    } else if (strcmp(type, "start") == 0 || strcmp(type, "resume") == 0 || strcmp(type, "home") == 0) {

        // questi comandi azzerano anche il guasto, poi li esegue la navigazione
        if (!clear_fault_if_any()) {
            refuse(result, "cause_still_present");
            return;
        }

        navigation_command.type = strcmp(type, "start") == 0 ? NAV_COMMAND_START :
                                  strcmp(type, "resume") == 0 ? NAV_COMMAND_RESUME : NAV_COMMAND_HOME;
        run_navigation_command(navigation_command, result);

    } else if (strcmp(type, "pause") == 0) {

        navigation_command.type = NAV_COMMAND_PAUSE;
        run_navigation_command(navigation_command, result);

    } else if (strcmp(type, "remote_take") == 0) {

        navigation_command.type = NAV_COMMAND_REMOTE_TAKE;
        run_navigation_command(navigation_command, result);

    } else if (strcmp(type, "remote_keepalive") == 0) {

        navigation_command.type = NAV_COMMAND_REMOTE_KEEPALIVE;
        run_navigation_command(navigation_command, result);

    } else if (strcmp(type, "remote_release") == 0) {

        navigation_command.type = NAV_COMMAND_REMOTE_RELEASE;
        run_navigation_command(navigation_command, result);

    } else if (strcmp(type, "remote_move") == 0) {

        if (!read_number(command, "v", &navigation_command.v) || !read_number(command, "omega", &navigation_command.omega)) {
            refuse(result, "invalid_command");
            return;
        }

        navigation_command.type = NAV_COMMAND_REMOTE_MOVE;
        run_navigation_command(navigation_command, result);

    } else if (strcmp(type, "remote_cleaning") == 0) {

        if (!read_bool(command, "brushes", &navigation_command.brushes) || !read_bool(command, "suction", &navigation_command.suction)) {
            refuse(result, "invalid_command");
            return;
        }

        // senza il cassetto della polvere l'aspirazione non parte: il telecomando si prende lo stesso, come con ogni
        // comando del telecomando, ma spazzole e aspirazione restano come sono
        drv_sensors_t sensors = {0};
        drv_sensors(&sensors);
        if (navigation_command.suction && !sensors.s.dustbin) {
            navigation_command.type = NAV_COMMAND_REMOTE_KEEPALIVE;
            run_navigation_command(navigation_command, result);
            if (result->ok) refuse(result, "dustbin_missing");
            return;
        }

        navigation_command.type = NAV_COMMAND_REMOTE_CLEANING;
        run_navigation_command(navigation_command, result);

    } else if (strcmp(type, "go_to") == 0) {

        // obiettivo sulla mappa: x e y in metri, poi la navigazione controlla la cella e il percorso
        if (!read_number(command, "x", &navigation_command.v) || !read_number(command, "y", &navigation_command.omega)) {
            refuse(result, "invalid_command");
            return;
        }
        navigation_command.type = NAV_COMMAND_GO_TO;
        run_navigation_command(navigation_command, result);

    } else if (strcmp(type, "zones") == 0) {

        run_zones_command(command, result);

    } else if (strcmp(type, "map_select") == 0) {

        run_map_command(command, true, download_map, result);

    } else if (strcmp(type, "map_reset") == 0) {

        run_map_command(command, false, download_map, result);

    } else if (strcmp(type, "reposition_base") == 0) {

        // Riposiziona base: la navigazione controlla che il robot sia a riposo sulla base con una mappa e lo fa partire.
        // L'esito arriva dopo, nello stato (map.reposition), e se è riuscito il programma si riavvia
        navigation_command.type = NAV_COMMAND_REPOSITION_BASE;
        run_navigation_command(navigation_command, result);

    } else if (strcmp(type, "wifi_scan") == 0 || strcmp(type, "wifi_connect") == 0) {

        run_wifi_command(command, strcmp(type, "wifi_scan") == 0, result);

    } else refuse(result, "invalid_command");
}

void run_app_command(const char *json, app_map_download_t download_map, app_command_result_t *result) {

    // se non viene rifiutato, il comando è eseguito
    result->ok = true;
    result->reason = NULL;
    result->message[0] = '\0';
    result->is_restart_needed = false;
    result->data = NULL;

    cJSON *command = cJSON_Parse(json);
    if (!cJSON_IsObject(command)) {
        cJSON_Delete(command);
        refuse(result, "invalid_command");
        return;
    }

    // un comando alla volta, fra il thread di rete e la FIFO
    pthread_mutex_lock(&commands_mutex);
    run_parsed_command(command, download_map, result);
    pthread_mutex_unlock(&commands_mutex);

    cJSON_Delete(command);
}
