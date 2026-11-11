#include <math.h>
#include <stdatomic.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#include "driver/driver.h"
#include "gd32/interlock.h"
#include "third_party/cjson/cJSON.h"

#include "config_check.h"
#include "live_map.h"
#include "robot_state.h"
#include "../aperiodic.h"
#include "../lidar.h"
#include "../mapping.h"
#include "../movement.h"
#include "../navigation.h"
#include "../safety.h"

#define STATE_FORMAT            1           // versione del formato dello stato
#define MOVING_SPEED_MIN        0.001       // sotto questa velocità misurata (m/s e rad/s) il robot conta come fermo
#define RAD_TO_DEG              (180.0 / M_PI)
#define FAULT_TEXT_MAX          96          // lunghezza massima del testo del guasto del driver

// nomi degli stati della navigazione, nello stesso ordine di nav_state_t
static const char *const NAV_STATE_NAMES[] = {
    "wait", "explore", "clean", "follow_borders", "return_home", "hook_up", "follow_path", "unlock", "sleep", "align",
    "failure", "border_lap", "leave_base", "cross", "pause", "remote",
};

// nomi dei tipi di obiettivo, nello stesso ordine di objective_type_t
static const char *const OBJECTIVE_NAMES[] = { "stop", "rotate", "move", "const_speed" };

// nomi degli stati della mappa, nello stesso ordine di mapping_map_state
static const char *const MAP_STATE_NAMES[] = { "normal", "lost", "unconfirmed", "not_found" };

// nomi degli allarmi della sicurezza, nello stesso ordine dei bit di error_mask
static const char *const SAFETY_ERROR_NAMES[] = { "stuck", "slip", "motor_overload", "obstacle", "forbidden_zone", "contact", "brush" };

// nomi dei livelli della batteria (BAT_OK, BAT_LOW, BAT_CRITICAL) e delle fasi del driver (DRV_STARTING...)
static const char *const BATTERY_LEVEL_NAMES[] = { "ok", "low", "critical" };
static const char *const DRIVER_PHASE_NAMES[] = { "startup", "ready", "fault", "stopped" };

// nomi dell'esito dell'aggancio (DOCK_IDLE...) e delle sue fasi (DOCK_P_NONE...)
static const char *const DOCKING_RESULT_NAMES[] = { "idle", "in_progress", "done", "failed" };
static const char *const DOCKING_PHASE_NAMES[] = {
    "none", "search", "center", "approach", "refine", "detach", "back", "turn", "last_stretch", "contact", "retreat", "press",
};

// un motivo per cui il driver limita il moto e il suo nome
typedef struct {
    int bit;
    const char *name;
} limit_name_t;

static const limit_name_t LIMIT_NAMES[] = {
    { INTERLOCK_LIM_LIFTED, "lifted" }, { INTERLOCK_LIM_BASE, "on_base" }, { INTERLOCK_LIM_SENSORS, "sensors_off" },
    { INTERLOCK_LIM_BUMPER, "bumper" }, { INTERLOCK_LIM_CLIFF, "cliff" }, { DRV_LIM_DISABLED, "motors_disabled" },
    { DRV_LIM_NOT_READY, "not_ready" }, { DRV_LIM_FAULT, "fault" }, { DRV_LIM_OLD_SENSORS, "stale_sensors" },
    { DRV_LIM_BATTERY, "battery" }, { DRV_LIM_BENCH, "test_bench" },
};

// un task real-time e il nome con cui compare nello stato
typedef struct {
    rt_task_t *task;
    const char *name;
} task_name_t;

static const task_name_t TASK_NAMES[] = {
    { &movement_task, "movement" }, { &safety_task, "safety" }, { &lidar_task, "lidar" },
    { &navigation_task, "navigation" }, { &mapping_task, "mapping" },
};

#define ARRAY_SIZE(a) ((int)(sizeof(a) / sizeof((a)[0])))

// le cose importanti: quando una cambia, lo stato parte subito
typedef struct {
    nav_state_t state, mission, interrupted_mission;
    uint32_t mission_number;
    bool is_remote, remote_brushes, remote_suction;
    uint16_t last_command_num;
    nav_command_result_t last_command_result;
    int driver_phase, task_fault;
    char fault_text[FAULT_TEXT_MAX];
    int docking_result;
    bool charging, charge_complete, on_base, lifted;
    int battery_level;
    mapping_map_state map_state;
    bool save_failed, started_on_base;
    char file_crc_text[MAP_CRC_TEXT_LEN];
    mapping_forbidden_zone_list_t zones;
    bool is_moving;                 // quando il robot si ferma parte ancora uno stato, con le velocità a zero
    bool is_brushes_on, is_suction_on;  // spazzole e aspirazione accese o spente
} important_state_t;

// tutto quello che serve a costruire lo stato, letto una volta per ciclo
typedef struct {
    navigation_status_t navigation;
    navigation_objective_t objective;
    movement_position_data_t position;
    mapping_correction_t correction;
    mapping_forbidden_zone_list_t zones;
    safety_alarm_t alarm;
    lidar_scan_data_t scan;
    drv_status_t driver;
    drv_sensors_t sensors;
} state_readings_t;

// stato interno: lo usa solo il thread dello stato
static struct {
    important_state_t sent;         // le cose importanti dell'ultimo stato ricevuto dal server
    important_state_t built;        // quelle dell'ultimo stato costruito
    bool is_first_sent;             // true dopo il primo stato ricevuto dal server
    int64_t last_sent_t;            // istante dell'ultimo stato ricevuto dal server
    int64_t start_t;                // istante di avvio, per uptime_s
    int64_t mission_duration_s;     // durata dell'ultima missione: resta ferma quando la missione finisce
} robot_state;

void init_robot_state(void) {
    memset(&robot_state, 0, sizeof robot_state);
    robot_state.start_t = rt_now();
}

void mark_robot_state_sent(void) {
    robot_state.sent = robot_state.built;
    robot_state.is_first_sent = true;
    robot_state.last_sent_t = rt_now();
}

// funzione helper che legge tutto dai doppi buffer dei task e del driver
static void read_everything(state_readings_t *readings) {
    readings->navigation = read_navigation_status();
    readings->objective = read_navigation();
    readings->position = read_movement();
    readings->correction = read_mapping_correction();
    readings->zones = read_forbidden_zone_list();
    readings->alarm = read_safety();
    readings->scan = read_lidar();
    memset(&readings->driver, 0, sizeof readings->driver);
    memset(&readings->sensors, 0, sizeof readings->sensors);
    drv_status(&readings->driver);
    drv_sensors(&readings->sensors);
}

// funzione helper che raccoglie le cose importanti. La struttura parte azzerata: si confronta con memcmp
static void fill_important_state(const state_readings_t *readings, important_state_t *important) {

    memset(important, 0, sizeof *important);

    important->state = readings->navigation.state;
    important->mission = readings->navigation.mission;
    important->interrupted_mission = readings->navigation.interrupted_mission;
    important->mission_number = readings->navigation.mission_number;
    important->is_remote = readings->navigation.is_remote;
    important->remote_brushes = readings->navigation.remote_brushes;
    important->remote_suction = readings->navigation.remote_suction;
    important->last_command_num = readings->navigation.last_command_num;
    important->last_command_result = readings->navigation.last_command_result;
    important->driver_phase = readings->driver.phase;
    important->task_fault = readings->driver.task_fault;
    snprintf(important->fault_text, sizeof important->fault_text, "%s", readings->driver.fault);
    important->docking_result = readings->driver.docking;
    important->charging = readings->driver.charging;
    important->charge_complete = readings->driver.charge_complete;
    important->on_base = readings->driver.on_base;
    important->lifted = readings->driver.lifted;
    important->battery_level = readings->driver.battery;
    important->is_brushes_on = readings->driver.side_brush != 0 || readings->driver.main_brush != 0;
    important->is_suction_on = readings->driver.suction_rpm != 0;
    important->map_state = readings->correction.state;
    important->save_failed = read_map_save_failed();
    important->started_on_base = read_started_on_base();
    snprintf(important->file_crc_text, sizeof important->file_crc_text, "%s", read_live_map_file_crc());

    // le zone: solo quelle presenti, il resto dell'array resta azzerato
    important->zones.num_zones = readings->zones.num_zones;
    for (int i = 0; i < readings->zones.num_zones && i < MAX_FORBIDDEN_ZONES; i++) important->zones.forbidden_zones[i] = readings->zones.forbidden_zones[i];
}

// funzione helper che restituisce il nome numero index di una tabella di nomi, "unknown" se è fuori dalla tabella
static const char *name_of(const char *const *names, int count, int index) {
    return (index >= 0 && index < count) ? names[index] : "unknown";
}

// funzione helper che arrotonda un numero a un certo numero di cifre decimali, per non riempire il JSON di cifre inutili
static double round_to(double value, double decimals) {
    if (!isfinite(value)) return 0;
    double scale = pow(10, decimals);
    return round(value * scale) / scale;
}

// funzione helper che costruisce il JSON dello stato
static char *build_json(const state_readings_t *readings) {

    const drv_status_t *driver = &readings->driver;
    const gd32_sensors_t *sensors = &readings->sensors.s;
    int64_t now = rt_now();

    cJSON *root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "format", STATE_FORMAT);
    cJSON_AddNumberToObject(root, "robot_time", (double)time(NULL));
    cJSON_AddNumberToObject(root, "uptime_s", (double)((now - robot_state.start_t) / RT_S(1)));
    cJSON_AddBoolToObject(root, "remote_control", readings->navigation.is_remote);

    // navigazione. La durata della missione conta finché la missione è in corso, poi resta quella finale
    bool is_mission_active = readings->navigation.mission != NAV_STATE_SLEEP;
    if (is_mission_active) robot_state.mission_duration_s = (now - readings->navigation.mission_start_t) / RT_S(1);
    cJSON *navigation = cJSON_AddObjectToObject(root, "navigation");
    cJSON_AddStringToObject(navigation, "state", name_of(NAV_STATE_NAMES, ARRAY_SIZE(NAV_STATE_NAMES), readings->navigation.state));
    cJSON_AddStringToObject(navigation, "mission", is_mission_active ? name_of(NAV_STATE_NAMES, ARRAY_SIZE(NAV_STATE_NAMES), readings->navigation.mission) : "none");
    cJSON_AddStringToObject(navigation, "interrupted_mission", readings->navigation.interrupted_mission == NAV_STATE_WAIT ? "none" :
                            name_of(NAV_STATE_NAMES, ARRAY_SIZE(NAV_STATE_NAMES), readings->navigation.interrupted_mission));
    cJSON_AddNumberToObject(navigation, "mission_number", readings->navigation.mission_number);
    cJSON_AddNumberToObject(navigation, "mission_duration_s", (double)robot_state.mission_duration_s);

    // obiettivo mandato al movimento
    cJSON *objective = cJSON_AddObjectToObject(root, "objective");
    cJSON_AddStringToObject(objective, "type", name_of(OBJECTIVE_NAMES, ARRAY_SIZE(OBJECTIVE_NAMES), readings->objective.obj_type));
    cJSON_AddNumberToObject(objective, "x", round_to(readings->objective.x, 3));
    cJSON_AddNumberToObject(objective, "y", round_to(readings->objective.y, 3));
    cJSON_AddNumberToObject(objective, "theta_deg", round_to(readings->objective.theta * RAD_TO_DEG, 1));

    // posizione sulla mappa
    cJSON *position = cJSON_AddObjectToObject(root, "position");
    cJSON_AddNumberToObject(position, "x", round_to(readings->position.x, 3));
    cJSON_AddNumberToObject(position, "y", round_to(readings->position.y, 3));
    cJSON_AddNumberToObject(position, "theta_deg", round_to(readings->position.theta * RAD_TO_DEG, 1));
    cJSON_AddNumberToObject(position, "v", round_to(readings->position.v, 3));
    cJSON_AddNumberToObject(position, "omega", round_to(readings->position.omega, 3));
    cJSON_AddBoolToObject(position, "valid", readings->position.valid);

    // mappa: il CRC del file c'è solo se il file c'è; la zona 0 è quella della base
    cJSON *map = cJSON_AddObjectToObject(root, "map");
    cJSON_AddStringToObject(map, "state", name_of(MAP_STATE_NAMES, ARRAY_SIZE(MAP_STATE_NAMES), readings->correction.state));
    cJSON_AddBoolToObject(map, "save_failed", read_map_save_failed());
    cJSON_AddBoolToObject(map, "started_on_base", read_started_on_base());
    if (read_live_map_file_crc()[0] != '\0') cJSON_AddStringToObject(map, "file_crc", read_live_map_file_crc());
    cJSON *zones = cJSON_AddArrayToObject(map, "forbidden_zones");
    for (int i = 0; i < readings->zones.num_zones && i < MAX_FORBIDDEN_ZONES; i++) {
        cJSON *zone = cJSON_CreateObject();
        cJSON_AddNumberToObject(zone, "x", round_to(readings->zones.forbidden_zones[i].x, 3));
        cJSON_AddNumberToObject(zone, "y", round_to(readings->zones.forbidden_zones[i].y, 3));
        cJSON_AddNumberToObject(zone, "w", round_to(readings->zones.forbidden_zones[i].w, 3));
        cJSON_AddNumberToObject(zone, "h", round_to(readings->zones.forbidden_zones[i].h, 3));
        cJSON_AddItemToArray(zones, zone);
    }

    // sicurezza
    cJSON *safety = cJSON_AddObjectToObject(root, "safety");
    cJSON *errors = cJSON_AddArrayToObject(safety, "errors");
    for (int bit = 0; bit < ARRAY_SIZE(SAFETY_ERROR_NAMES); bit++) {
        if (readings->alarm.error_mask & (1u << bit)) cJSON_AddItemToArray(errors, cJSON_CreateString(SAFETY_ERROR_NAMES[bit]));
    }
    cJSON_AddNumberToObject(safety, "max_v_forward", round_to(readings->alarm.max_v_forw, 3));
    cJSON_AddNumberToObject(safety, "max_v_backward", round_to(readings->alarm.max_v_backw, 3));

    // lidar: solo i settori e la strada libera, non il giro intero
    cJSON *lidar = cJSON_AddObjectToObject(root, "lidar");
    cJSON_AddBoolToObject(lidar, "valid", readings->scan.valid);
    cJSON *sectors = cJSON_AddArrayToObject(lidar, "sector_distances_mm");
    for (int i = 0; i < SECTOR_NUMBER; i++) cJSON_AddItemToArray(sectors, cJSON_CreateNumber(readings->scan.sector_distance[i]));
    cJSON_AddNumberToObject(lidar, "corridor_front_mm", readings->scan.corridor_front_distance);
    cJSON_AddNumberToObject(lidar, "corridor_back_mm", readings->scan.corridor_back_distance);

    // batteria
    cJSON *battery = cJSON_AddObjectToObject(root, "battery");
    cJSON_AddNumberToObject(battery, "voltage", round_to(driver->volts, 2));
    cJSON_AddNumberToObject(battery, "percent", driver->percentage);
    cJSON_AddStringToObject(battery, "level", name_of(BATTERY_LEVEL_NAMES, ARRAY_SIZE(BATTERY_LEVEL_NAMES), driver->battery));
    cJSON_AddBoolToObject(battery, "charging", driver->charging);
    cJSON_AddBoolToObject(battery, "charge_complete", driver->charge_complete);
    cJSON_AddNumberToObject(battery, "temperature_c", driver->temperature_c);

    // motori della pulizia: quello che il driver sta mandando davvero
    cJSON *motors = cJSON_AddObjectToObject(root, "motors");
    cJSON_AddNumberToObject(motors, "side_brush", driver->side_brush);
    cJSON_AddNumberToObject(motors, "main_brush", driver->main_brush);
    cJSON_AddNumberToObject(motors, "suction_rpm", driver->suction_rpm);

    // driver
    cJSON *driver_json = cJSON_AddObjectToObject(root, "driver");
    cJSON_AddStringToObject(driver_json, "phase", name_of(DRIVER_PHASE_NAMES, ARRAY_SIZE(DRIVER_PHASE_NAMES), driver->phase));
    if (driver->phase == DRV_FAULT) {
        cJSON *fault = cJSON_AddObjectToObject(driver_json, "fault");
        cJSON_AddNumberToObject(fault, "code", driver->task_fault);
        cJSON_AddStringToObject(fault, "text", driver->fault);
    } else cJSON_AddNullToObject(driver_json, "fault");
    cJSON *limits = cJSON_AddArrayToObject(driver_json, "limits");
    for (int i = 0; i < ARRAY_SIZE(LIMIT_NAMES); i++) {
        if (driver->limits & LIMIT_NAMES[i].bit) cJSON_AddItemToArray(limits, cJSON_CreateString(LIMIT_NAMES[i].name));
    }
    cJSON_AddBoolToObject(driver_json, "on_base", driver->on_base);
    cJSON_AddBoolToObject(driver_json, "lifted", driver->lifted);
    cJSON *docking = cJSON_AddObjectToObject(driver_json, "docking");
    cJSON_AddStringToObject(docking, "result", name_of(DOCKING_RESULT_NAMES, ARRAY_SIZE(DOCKING_RESULT_NAMES), driver->docking));
    cJSON_AddStringToObject(docking, "phase", name_of(DOCKING_PHASE_NAMES, ARRAY_SIZE(DOCKING_PHASE_NAMES), driver->docking_phase));
    cJSON_AddNumberToObject(docking, "attempt", driver->docking_attempt);
    cJSON *driver_lidar = cJSON_AddObjectToObject(driver_json, "lidar");
    cJSON_AddBoolToObject(driver_lidar, "ok", driver->lidar_ok);
    cJSON_AddNumberToObject(driver_lidar, "rotation_hz", round_to(driver->lidar_rev_s, 1));

    // sensori: solo i campi del GD32 verificati
    cJSON *sensors_json = cJSON_AddObjectToObject(root, "sensors");
    cJSON_AddBoolToObject(sensors_json, "bumper_left", sensors->bumper_left);
    cJSON_AddBoolToObject(sensors_json, "bumper_right", sensors->bumper_right);
    cJSON *cliff = cJSON_AddArrayToObject(sensors_json, "cliff");
    for (int i = 0; i < 4; i++) cJSON_AddItemToArray(cliff, cJSON_CreateBool(sensors->cliff[i]));
    cJSON_AddBoolToObject(sensors_json, "dustbin", sensors->dustbin);
    cJSON *currents = cJSON_AddObjectToObject(sensors_json, "currents_ma");
    cJSON_AddNumberToObject(currents, "wheel_left", sensors->currents[0]);
    cJSON_AddNumberToObject(currents, "wheel_right", sensors->currents[1]);
    cJSON_AddNumberToObject(currents, "side_brush", sensors->currents[2]);
    cJSON_AddNumberToObject(currents, "main_brush", sensors->currents[3]);
    cJSON_AddNumberToObject(currents, "suction", sensors->currents[4]);

    // task real-time: i due contatori sono atomici, si leggono anche mentre i task girano (i tempi massimi no)
    cJSON *tasks = cJSON_AddArrayToObject(root, "tasks");
    for (int i = 0; i < ARRAY_SIZE(TASK_NAMES); i++) {
        cJSON *task = cJSON_CreateObject();
        cJSON_AddStringToObject(task, "name", TASK_NAMES[i].name);
        cJSON_AddNumberToObject(task, "deadline_misses", (double)atomic_load(&TASK_NAMES[i].task->total_misses));
        cJSON_AddNumberToObject(task, "missed_activations", (double)atomic_load(&TASK_NAMES[i].task->lost_activations));
        cJSON_AddItemToArray(tasks, task);
    }

    char *json = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    return json;
}

char *build_robot_state_if_due(void) {

    state_readings_t readings;
    read_everything(&readings);

    important_state_t important;
    fill_important_state(&readings, &important);

    // il robot si muove: stato della navigazione attivo, oppure v o omega diversi da 0
    nav_state_t state = readings.navigation.state;
    bool is_moving = (state != NAV_STATE_SLEEP && state != NAV_STATE_PAUSE && state != NAV_STATE_FAILURE && state != NAV_STATE_REMOTE) ||
                     fabs(readings.position.v) > MOVING_SPEED_MIN || fabs(readings.position.omega) > MOVING_SPEED_MIN;

    important.is_moving = is_moving;

    bool is_changed = !robot_state.is_first_sent || memcmp(&important, &robot_state.sent, sizeof important) != 0;
    bool is_idle_due = rt_now() - robot_state.last_sent_t >= STATE_IDLE_INTERVAL;

    if (!is_moving && !is_changed && !is_idle_due) return NULL;

    robot_state.built = important;
    return build_json(&readings);
}
