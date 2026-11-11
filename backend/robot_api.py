"""Richieste dell'app sotto /api/robot (docs/app.md, sezioni 4.4, 5 e 8.5)."""
import json
import math
import time

from flask import Blueprint, Response, current_app, g, jsonify, request

import config
import maps
import robot_link
from auth import require_session
from responses import api_error, api_ok, invalid_parameters, not_found

robot_blueprint = Blueprint("robot", __name__, url_prefix="/api/robot")

REMOTE_COMMAND_TYPES = ("remote_take", "remote_move", "remote_cleaning", "remote_keepalive")
MISSION_COMMAND_TYPES = ("start", "pause", "resume", "home")

LOG_STATUS_OK = "ok"
LOG_STATUS_REJECTED = "rejected"

# testi dei rifiuti e degli errori decisi dal server (quelli del robot arrivano con il suo esito)
MESSAGES = {
    "robot_unreachable": "Il robot non è raggiungibile: il comando non è stato eseguito.",
    "no_result": "Il robot non ha risposto in tempo: guarda lo stato per sapere se il comando è stato eseguito.",
    "remote_in_use": "Il telecomando è in uso da {user}.",
    "not_remote_owner": "Il telecomando non è tuo.",
    "too_many_zones": "Troppe zone proibite: al massimo %d." % config.MAX_USER_ZONES,
    "outside_map": "Una zona esce dalla mappa o è troppo piccola.",
    "not_backed_up": "La mappa sul robot non è ancora stata copiata sul server: non si può cancellare.",
    "name_taken": "Esiste già una mappa con questo nome.",
    "map_selected": "Questa mappa è selezionata sul robot: prima scegli \"Nessuna mappa\".",
}


def services():
    return current_app.extensions["maurice"]


# ------------------------------------------------------------------------------------------------- comandi

def log_command(app_services, user_id, command_type, params, status, reason):
    """Scrive un comando nel registro. Ritorna l'id della riga, o None se quel tipo non si registra."""
    if command_type in config.COMMAND_LOG_SKIPPED_TYPES:
        return None
    if command_type in config.COMMAND_LOG_ONLY_CHANGES_TYPES:
        # per esempio remote_move: il joystick fermo in una posizione ripete lo stesso comando, e si scrive una volta sola
        key = (user_id, status, reason, json.dumps(params, sort_keys=True))
        if app_services.last_logged.get(command_type) == key:
            return None
        app_services.last_logged[command_type] = key
    cursor = app_services.database.execute(
        "INSERT INTO command_log (time, user_id, type, params, status, reason) VALUES (?, ?, ?, ?, ?, ?)",
        (int(time.time()), user_id, command_type, json.dumps(params) if params else None, status, reason))
    return cursor.lastrowid


def rejected(app_services, user_id, command_type, params, reason, **extra):
    """Rifiuto deciso dal server: 409 con il motivo, e una riga nel registro."""
    log_command(app_services, user_id, command_type, params, LOG_STATUS_REJECTED, reason)
    return 409, dict(reason=reason, message=MESSAGES[reason].format(**extra), **extra)


def apply_success(app_services, command_type, params):
    """Quello che il server cambia quando il robot ha eseguito il comando."""
    if command_type == "map_select":
        app_services.map_store.set_selected(params["map_id"])
    elif command_type == "map_reset":
        app_services.map_store.set_selected(None)


def execute_command(app_services, command_type, params, user_id, username):
    """Controlli del server, comando in coda e attesa dell'esito (docs/app.md, sezione 5.1).

    user_id None vuol dire la programmazione delle pulizie. Ritorna (codice HTTP, corpo JSON)."""
    link = app_services.link
    holder_id, holder_name = link.remote_holder()
    took_remote = False

    # telecomando: un utente alla volta. Gli altri non possono usare né il telecomando né le missioni
    if command_type in REMOTE_COMMAND_TYPES or command_type in MISSION_COMMAND_TYPES:
        if holder_id is not None and holder_id != user_id:
            return rejected(app_services, user_id, command_type, params, "remote_in_use", user=holder_name)
    if command_type in REMOTE_COMMAND_TYPES:
        # qualsiasi comando del telecomando lo prende, se è libero, e ne rinnova il tempo
        took_remote = link.remote_take(user_id, username)
    elif command_type == "remote_release":
        if holder_id is None or holder_id != user_id:
            return rejected(app_services, user_id, command_type, params, "not_remote_owner")
    elif command_type in MISSION_COMMAND_TYPES and holder_id is not None:
        # start, pause, resume e home di chi tiene il telecomando valgono come rilascio seguito dal comando
        link.remote_release()

    log_ids = []

    def late_result(result):
        # l'esito è arrivato dopo che l'app aveva già ricevuto 504: si corregge almeno il registro
        if result.get("ok"):
            apply_success(app_services, command_type, params)
        if log_ids and log_ids[0] is not None:
            app_services.database.execute(
                "UPDATE command_log SET status = ?, reason = ? WHERE id = ?",
                (LOG_STATUS_OK if result.get("ok") else LOG_STATUS_REJECTED, result.get("reason"), log_ids[0]))

    outcome, result = link.send_command(command_type, params, late_result)

    if command_type == "remote_release" or (took_remote and outcome == robot_link.COMMAND_UNREACHABLE):
        link.remote_release()

    if outcome == robot_link.COMMAND_UNREACHABLE:
        log_command(app_services, user_id, command_type, params, outcome, None)
        return 503, {"reason": outcome, "message": MESSAGES[outcome]}
    if outcome == robot_link.COMMAND_NO_RESULT:
        log_ids.append(log_command(app_services, user_id, command_type, params, outcome, None))
        return 504, {"reason": outcome, "message": MESSAGES[outcome]}
    if result.get("ok"):
        apply_success(app_services, command_type, params)
        log_command(app_services, user_id, command_type, params, LOG_STATUS_OK, None)
        return 200, {"ok": True}
    log_command(app_services, user_id, command_type, params, LOG_STATUS_REJECTED, result.get("reason"))
    return 409, {"reason": result.get("reason", "rejected"), "message": result.get("message", "")}


def command_response(command_type, params=None):
    status, body = execute_command(services(), command_type, params or {}, g.user_id, g.username)
    response = jsonify(body)
    response.status_code = status
    return response


def is_number(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def parse_zones(body):
    """Lista di zone [{"x", "y", "w", "h"}] in metri, oppure None se il corpo non ha quella forma."""
    if not isinstance(body, list):
        return None
    zones = []
    for zone in body:
        if not isinstance(zone, dict) or not all(is_number(zone.get(key)) for key in ("x", "y", "w", "h")):
            return None
        zones.append({key: float(zone[key]) for key in ("x", "y", "w", "h")})
    return zones


def int_argument(name):
    """Parametro intero dell'indirizzo: (valore o None, True se era scritto male)."""
    value = request.args.get(name)
    if value is None or value == "":
        return None, False
    try:
        return int(value), False
    except ValueError:
        return None, True


# --------------------------------------------------------------------------------------------------- stato

@robot_blueprint.get("/state")
@require_session
def get_state():
    after, after_bad = int_argument("after")
    min_interval_ms, interval_bad = int_argument("min_interval_ms")
    if after_bad or interval_bad:
        return invalid_parameters()
    min_interval_ms = min(max(min_interval_ms or 0, 0), config.MIN_INTERVAL_MAX_MS)
    state = services().link.wait_state(after, min_interval_ms, g.session_id)
    if state is None:
        return Response(status=304)
    return jsonify(state)


# ------------------------------------------------------------------------------------- missioni e guasto

@robot_blueprint.post("/mission/start")
@require_session
def mission_start():
    return command_response("start")


@robot_blueprint.post("/mission/pause")
@require_session
def mission_pause():
    return command_response("pause")


@robot_blueprint.post("/mission/resume")
@require_session
def mission_resume():
    return command_response("resume")


@robot_blueprint.post("/mission/home")
@require_session
def mission_home():
    return command_response("home")


@robot_blueprint.post("/fault/reset")
@require_session
def fault_reset():
    return command_response("fault_reset")


# ---------------------------------------------------------------------------------------------- telecomando

@robot_blueprint.post("/remote/take")
@require_session
def remote_take():
    return command_response("remote_take")


@robot_blueprint.post("/remote/move")
@require_session
def remote_move():
    body = request.get_json(silent=True)
    if not isinstance(body, dict) or not is_number(body.get("v")) or not is_number(body.get("omega")):
        return invalid_parameters("Servono v e omega.")
    return command_response("remote_move", {"v": float(body["v"]), "omega": float(body["omega"])})


@robot_blueprint.post("/remote/cleaning")
@require_session
def remote_cleaning():
    body = request.get_json(silent=True)
    if not isinstance(body, dict) or not isinstance(body.get("brushes"), bool) or \
            not isinstance(body.get("suction"), bool):
        return invalid_parameters("Servono brushes e suction.")
    return command_response("remote_cleaning", {"brushes": body["brushes"], "suction": body["suction"]})


@robot_blueprint.post("/remote/keepalive")
@require_session
def remote_keepalive():
    return command_response("remote_keepalive")


@robot_blueprint.post("/remote/release")
@require_session
def remote_release():
    return command_response("remote_release")


# ------------------------------------------------------------------------------------------- zone proibite

def user_zones_from_state(state):
    """Le zone dell'utente dall'ultimo stato: la zona 0 è quella della base, che il robot aggiunge da solo."""
    map_state = state.get("map") or {}
    zones = map_state.get("forbidden_zones") or []
    return zones[1:] if map_state.get("started_on_base") else zones


@robot_blueprint.get("/zones")
@require_session
def get_zones():
    return jsonify(user_zones_from_state(services().link.current_state()))


def zones_command(zones):
    reason = maps.check_zones(zones)
    if reason:
        status, body = rejected(services(), g.user_id, "zones", {"zones": zones}, reason)
        return api_error(status, body["reason"], body["message"])
    return command_response("zones", {"zones": zones})


@robot_blueprint.put("/zones")
@require_session
def put_zones():
    zones = parse_zones(request.get_json(silent=True))
    if zones is None:
        return invalid_parameters("Serve la lista delle zone.")
    return zones_command(zones)


# --------------------------------------------------------------------------------- mappa dal vivo e scia

@robot_blueprint.get("/map")
@require_session
def get_live_map():
    after, bad = int_argument("after")
    if bad:
        return invalid_parameters()
    # le mappe non passano dal gzip: le sequenze dell'RTM1 sono già una compressione
    return Response(services().live_map.message(after), mimetype="application/octet-stream")


@robot_blueprint.get("/trail")
@require_session
def get_trail():
    after, bad = int_argument("after")
    if bad:
        return invalid_parameters()
    return jsonify(services().trail.message(after))


@robot_blueprint.post("/map/reset")
@require_session
def map_reset():
    """"Nessuna mappa": il robot cancella map.rtm e si riavvia. Solo se il server ha la copia di quel file."""
    app_services = services()
    map_state = app_services.link.current_state().get("map") or {}
    robot_crc = map_state.get("file_crc")
    if robot_crc is not None:
        selected = app_services.map_store.selected_id()
        row = app_services.map_store.get(selected) if selected is not None else None
        if row is None or row["crc32"] != robot_crc:
            status, body = rejected(app_services, g.user_id, "map_reset", {}, "not_backed_up")
            return api_error(status, body["reason"], body["message"])
    return command_response("map_reset")


# --------------------------------------------------------------------------------------------- mappe salvate

@robot_blueprint.get("/maps")
@require_session
def list_maps():
    return jsonify(services().map_store.list_maps())


@robot_blueprint.put("/maps/selected")
@require_session
def select_map():
    body = request.get_json(silent=True)
    if not isinstance(body, dict) or not isinstance(body.get("id"), int) or isinstance(body.get("id"), bool):
        return invalid_parameters("Serve l'id della mappa.")
    row = services().map_store.get(body["id"])
    if row is None:
        return not_found("La mappa non esiste.")
    return command_response("map_select", {"map_id": row["id"], "crc": row["crc32"]})


@robot_blueprint.get("/maps/<int:map_id>")
@require_session
def get_map(map_id):
    if services().map_store.get(map_id) is None:
        return not_found("La mappa non esiste.")
    return Response(services().map_store.read_file(map_id), mimetype="application/octet-stream")


@robot_blueprint.patch("/maps/<int:map_id>")
@require_session
def rename_map(map_id):
    body = request.get_json(silent=True)
    name = body.get("name") if isinstance(body, dict) else None
    if not isinstance(name, str) or not name.strip() or len(name.strip()) > config.MAP_NAME_MAX_LENGTH:
        return invalid_parameters("Serve un nome, lungo al massimo %d caratteri." % config.MAP_NAME_MAX_LENGTH)
    if services().map_store.get(map_id) is None:
        return not_found("La mappa non esiste.")
    try:
        services().map_store.rename(map_id, name.strip())
    except maps.MapNameTaken:
        return api_error(409, "name_taken", MESSAGES["name_taken"])
    return api_ok()


@robot_blueprint.put("/maps/<int:map_id>/zones")
@require_session
def put_map_zones(map_id):
    """Zone di una mappa: per la selezionata le applica il robot, per le altre il server riscrive il file."""
    zones = parse_zones(request.get_json(silent=True))
    if zones is None:
        return invalid_parameters("Serve la lista delle zone.")
    store = services().map_store
    if store.get(map_id) is None:
        return not_found("La mappa non esiste.")
    if map_id == store.selected_id():
        return zones_command(zones)
    reason = maps.check_zones(zones)
    if reason:
        return api_error(409, reason, MESSAGES[reason])
    store.set_zones(map_id, zones)
    return api_ok()


@robot_blueprint.delete("/maps/<int:map_id>")
@require_session
def delete_map(map_id):
    """Cancella una mappa dal server. La selezionata va prima tolta dal robot con POST /api/robot/map/reset."""
    store = services().map_store
    if store.get(map_id) is None:
        return not_found("La mappa non esiste.")
    if map_id == store.selected_id():
        return api_error(409, "map_selected", MESSAGES["map_selected"])
    store.delete(map_id)
    return api_ok()


# ---------------------------------------------------------------------------------------------- programmazione

DAYS_IN_WEEK = 7
MINUTES_IN_DAY = 24 * 60


def schedule_to_json(row):
    return {"id": row["id"],
            "days": [bool(row["days_mask"] >> day & 1) for day in range(DAYS_IN_WEEK)],     # da lunedì a domenica
            "time": "%02d:%02d" % divmod(row["time_minutes"], 60),
            "enabled": bool(row["enabled"]),
            "last_run_at": row["last_run_at"],
            "last_result": row["last_result"]}


def parse_schedule(body, is_partial):
    """Dai campi {"days", "time", "enabled"} alle colonne della tabella. Con is_partial possono mancare. None se sbagliati."""
    if not isinstance(body, dict):
        return None
    columns = {}
    if "days" in body:
        days = body["days"]
        if not isinstance(days, list) or len(days) != DAYS_IN_WEEK or not all(isinstance(day, bool) for day in days):
            return None
        columns["days_mask"] = sum(1 << index for index, day in enumerate(days) if day)
    if "time" in body:
        try:
            hours, minutes = (int(part) for part in str(body["time"]).split(":"))
        except ValueError:
            return None
        if not (0 <= hours < 24 and 0 <= minutes < 60):
            return None
        columns["time_minutes"] = hours * 60 + minutes
    if "enabled" in body:
        if not isinstance(body["enabled"], bool):
            return None
        columns["enabled"] = int(body["enabled"])
    if not is_partial and not ("days_mask" in columns and "time_minutes" in columns):
        return None
    return columns


def write_schedule(schedule_id, columns):
    """Aggiunge (schedule_id None) o modifica un orario. Un solo orario attivo: attivarne uno spegne gli altri nella
    stessa operazione sul database. Ritorna l'id."""
    database = services().database
    with database.transaction():
        if columns.get("enabled"):
            database.execute("UPDATE schedules SET enabled = 0")
        if schedule_id is None:
            cursor = database.execute("INSERT INTO schedules (days_mask, time_minutes, enabled) VALUES (?, ?, ?)",
                                      (columns["days_mask"], columns["time_minutes"], columns.get("enabled", 0)))
            schedule_id = cursor.lastrowid
        else:
            for column, value in columns.items():
                # i nomi delle colonne vengono solo da parse_schedule
                database.execute(f"UPDATE schedules SET {column} = ? WHERE id = ?", (value, schedule_id))
    return schedule_id


def get_schedule(schedule_id):
    return services().database.query_one("SELECT * FROM schedules WHERE id = ?", (schedule_id,))


@robot_blueprint.get("/schedules")
@require_session
def list_schedules():
    rows = services().database.query_all("SELECT * FROM schedules ORDER BY time_minutes, id")
    return jsonify([schedule_to_json(row) for row in rows])


@robot_blueprint.post("/schedules")
@require_session
def add_schedule():
    columns = parse_schedule(request.get_json(silent=True), is_partial=False)
    if columns is None:
        return invalid_parameters("Servono i giorni e l'ora.")
    return api_ok(schedule_to_json(get_schedule(write_schedule(None, columns))), status=201)


@robot_blueprint.put("/schedules/<int:schedule_id>")
@require_session
def change_schedule(schedule_id):
    columns = parse_schedule(request.get_json(silent=True), is_partial=True)
    if columns is None:
        return invalid_parameters()
    if get_schedule(schedule_id) is None:
        return not_found("L'orario non esiste.")
    write_schedule(schedule_id, columns)
    return api_ok(schedule_to_json(get_schedule(schedule_id)))


@robot_blueprint.delete("/schedules/<int:schedule_id>")
@require_session
def delete_schedule(schedule_id):
    if get_schedule(schedule_id) is None:
        return not_found("L'orario non esiste.")
    services().database.execute("DELETE FROM schedules WHERE id = ?", (schedule_id,))
    return api_ok()


# ------------------------------------------------------------------------------------------------- registro

@robot_blueprint.get("/log")
@require_session
def get_log():
    before, before_bad = int_argument("before")
    limit, limit_bad = int_argument("limit")
    if before_bad or limit_bad:
        return invalid_parameters()
    limit = min(max(limit or config.COMMAND_LOG_PAGE_DEFAULT, 1), config.COMMAND_LOG_PAGE_MAX)
    rows = services().database.query_all(
        "SELECT command_log.*, users.username FROM command_log LEFT JOIN users ON users.id = command_log.user_id "
        "WHERE (? IS NULL OR command_log.id < ?) ORDER BY command_log.id DESC LIMIT ?", (before, before, limit))
    return jsonify([{"id": row["id"], "time": row["time"], "user": row["username"],
                     "scheduled": row["user_id"] is None,       # comando mandato dalla programmazione delle pulizie
                     "type": row["type"], "params": json.loads(row["params"]) if row["params"] else None,
                     "status": row["status"], "reason": row["reason"]} for row in rows])
