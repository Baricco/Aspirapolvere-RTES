"""Richieste del robot sotto /api/device (docs/app.md, sezione 8.6). Il robot si presenta con il suo token."""
import functools
import hashlib
import hmac
import secrets

from flask import Blueprint, Response, current_app, jsonify, request

import config
import maps
from responses import api_error, api_ok, invalid_parameters, not_found

device_blueprint = Blueprint("device", __name__, url_prefix="/api/device")

BEARER_PREFIX = "Bearer "


def services():
    return current_app.extensions["maurice"]


def token_hash(token):
    return hashlib.sha256(token.encode("utf-8")).hexdigest()


def new_robot_token(database):
    """Genera un token nuovo (32 byte casuali, scritti in esadecimale) e ne salva solo lo SHA-256. Ritorna il token:
    va copiato nel file del token sul robot, il server non può più mostrarlo."""
    token = secrets.token_hex(config.ROBOT_TOKEN_BYTES)
    database.execute("UPDATE robot SET token_hash = ? WHERE id = 1", (token_hash(token),))
    return token


def require_robot(view):
    """Decoratore delle richieste del robot: senza il token giusto rispondono 401."""
    @functools.wraps(view)
    def wrapper(*args, **kwargs):
        header = request.headers.get("Authorization", "")
        saved = services().database.query_one("SELECT token_hash FROM robot WHERE id = 1")["token_hash"]
        if not header.startswith(BEARER_PREFIX) or not saved or \
                not hmac.compare_digest(token_hash(header[len(BEARER_PREFIX):]), saved):
            return api_error(401, "unauthenticated", "Token del robot non valido.")
        return view(*args, **kwargs)
    return wrapper


def version_argument(name):
    try:
        return int(request.args[name])
    except (KeyError, ValueError):
        return None


@device_blueprint.get("/commands")
@require_robot
def get_commands():
    """Richiesta lunga: i comandi in coda, oppure una lista vuota dopo l'attesa. È anche il segno di vita del robot."""
    return jsonify({"commands": services().link.take_commands()})


@device_blueprint.post("/commands/<int:command_id>/result")
@require_robot
def post_command_result(command_id):
    body = request.get_json(silent=True)
    if not isinstance(body, dict) or not isinstance(body.get("ok"), bool):
        return invalid_parameters()
    result = {"ok": body["ok"], "reason": body.get("reason"), "message": body.get("message", "")}
    if not services().link.set_command_result(command_id, result):
        return not_found("Il comando non esiste o non aspetta più un esito.")
    return api_ok()


@device_blueprint.post("/state")
@require_robot
def post_state():
    state = request.get_json(silent=True)
    if not isinstance(state, dict):
        return invalid_parameters()
    services().trail.add_state(state)
    services().link.update_state(state)
    # la versione della mappa dal vivo che il server ha ricevuto dal robot (null se non ne ha): se non è quella del
    # robot, per esempio dopo un riavvio del server, il robot rimanda la mappa intera anche se non è cambiato niente
    return api_ok({"ok": True, "live_map_version": services().live_map.device_version})


@device_blueprint.put("/map")
@require_robot
def put_live_map():
    """Mappa dal vivo intera: all'avvio del robot, o dopo un 409 sulle differenze."""
    version = version_argument("version")
    if version is None:
        return invalid_parameters()
    try:
        services().live_map.set_full(request.get_data(), version)
    except maps.MapFormatError as error:
        return invalid_parameters(f"Mappa non valida: {error}.")
    services().link.device_contact()
    return api_ok()


@device_blueprint.post("/map/patch")
@require_robot
def post_live_map_patch():
    """Rettangolo delle celle cambiate, dalla versione base_version alla versione version."""
    base_version, version = version_argument("base_version"), version_argument("version")
    if base_version is None or version is None:
        return invalid_parameters()
    try:
        services().live_map.apply_patch(request.get_data(), base_version, version)
    except maps.MapFormatError as error:
        return invalid_parameters(f"Mappa non valida: {error}.")
    except maps.MapVersionMismatch:
        return api_error(409, "version_mismatch", "Il server non ha la versione di partenza: serve la mappa intera.")
    services().link.device_contact()
    return api_ok()


@device_blueprint.put("/map/saved")
@require_robot
def put_saved_map():
    """Il file map.rtm del robot (corpo vuoto: il robot non ha mappa): allineamento con la mappa selezionata."""
    reason = request.args.get("reason")
    if reason not in (maps.SAVED_REASON_STARTUP, maps.SAVED_REASON_SAVE):
        return invalid_parameters()
    try:
        selected = services().map_store.saved_from_robot(request.get_data(), reason)
    except maps.MapFormatError as error:
        return invalid_parameters(f"Mappa non valida: {error}.")
    services().link.device_contact()
    return api_ok({"selected_map_id": selected})


@device_blueprint.get("/maps/<int:map_id>")
@require_robot
def get_map(map_id):
    if services().map_store.get(map_id) is None:
        return not_found("La mappa non esiste.")
    return Response(services().map_store.read_file(map_id), mimetype="application/octet-stream")
