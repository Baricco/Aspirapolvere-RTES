"""Crea l'app Flask e avvia la programmazione delle pulizie (docs/app.md, sezione 8).

Produzione:  gunicorn --config deploy/gunicorn.conf.py "app:create_app()"   (un solo processo: lo stato vivo sta in memoria)
Sviluppo:    backend/dev/run.sh
"""
import atexit
import gzip
import os
import urllib.parse

from flask import Flask, Response, request

import auth
import config
import db
import maps
import robot_api
import robot_link
import scheduler
from device_api import device_blueprint
from responses import api_error

UNSAFE_METHODS = ("POST", "PUT", "PATCH", "DELETE")     # quelle che cambiano qualcosa: per loro si controlla Origin
CORS_ALLOWED_HEADERS = "Content-Type, Authorization"
CORS_ALLOWED_METHODS = "GET, POST, PUT, PATCH, DELETE, OPTIONS"
CORS_MAX_AGE_S = 600
GZIP_LEVEL = 6


class Services:
    """Tutto quello che le richieste condividono: database, stato vivo, mappe, scia, programmazione."""

    def __init__(self, data_dir):
        os.makedirs(data_dir, exist_ok=True)
        self.data_dir = data_dir
        self.database = db.Database(os.path.join(data_dir, config.DATABASE_FILE))
        self.jwt_key = auth.load_jwt_key(data_dir)
        self.link = robot_link.RobotLink(data_dir)
        self.map_store = maps.MapStore(self.database, data_dir)
        self.trail = maps.Trail(data_dir)
        self.live_map = maps.LiveMap()
        self.live_map.on_change = lambda: self.link.set_map_version(self.live_map.version)
        self.scheduler = scheduler.Scheduler(self.database, self.scheduled_start)
        self.last_logged = {}       # tipo di comando -> ultimo scritto nel registro, per quelli che si scrivono solo se cambiano
        self.load_selected_map()

    def load_selected_map(self):
        """Finché il robot non manda la sua, la mappa dal vivo è quella selezionata (vuota se non ce n'è nessuna)."""
        selected = self.map_store.selected_id()
        try:
            data = self.map_store.read_file(selected) if selected is not None else maps.empty_rtm()
            self.live_map.set_full(data, None)
        except (OSError, maps.MapFormatError):
            self.live_map.set_full(maps.empty_rtm(), None)

    def scheduled_start(self):
        """Lo start della programmazione: come quello dell'app, ma nel registro l'utente è vuoto."""
        status, body = robot_api.execute_command(self, "start", {}, None, None)
        return "ok" if status == 200 else body["reason"]

    def start(self):
        self.link.start()
        self.scheduler.start()

    def stop(self):
        self.scheduler.stop()
        self.link.stop()


def origin_allowed(origin):
    """L'origine della richiesta è quella del sito (stesso host), oppure una di quelle permesse in sviluppo."""
    if urllib.parse.urlsplit(origin).netloc == request.host:
        return True
    return origin.rstrip("/") in config.allowed_origins()


def create_app(data_dir=None, start_background=True):
    config.load_env_file()
    app = Flask(__name__)
    app.json.sort_keys = False
    app.config["MAX_CONTENT_LENGTH"] = config.MAP_UPLOAD_MAX_BYTES
    app_services = Services(data_dir or config.data_dir())
    app.extensions["maurice"] = app_services

    app.register_blueprint(auth.auth_blueprint)
    app.register_blueprint(robot_api.robot_blueprint)
    app.register_blueprint(device_blueprint)

    @app.before_request
    def check_origin():
        origin = request.headers.get("Origin")
        if origin is None:
            return None             # telefono e robot: nessun browser di mezzo
        if not origin_allowed(origin):
            # richieste mandate da altri siti: quelle che cambiano qualcosa si rifiutano
            if request.method in UNSAFE_METHODS:
                return api_error(403, "bad_origin", "Richiesta da un sito non permesso.")
            return None
        if request.method == "OPTIONS":
            return Response(status=204)     # controllo preliminare del browser (solo in sviluppo, con origini diverse)
        return None

    @app.after_request
    def finish_response(response):
        # in sviluppo l'app web gira su un'altra origine: il browser vuole queste intestazioni per usare il cookie
        origin = request.headers.get("Origin")
        if origin and origin.rstrip("/") in config.allowed_origins():
            response.headers["Access-Control-Allow-Origin"] = origin
            response.headers["Access-Control-Allow-Credentials"] = "true"
            response.headers["Access-Control-Allow-Headers"] = CORS_ALLOWED_HEADERS
            response.headers["Access-Control-Allow-Methods"] = CORS_ALLOWED_METHODS
            response.headers["Access-Control-Max-Age"] = str(CORS_MAX_AGE_S)
            response.headers.add("Vary", "Origin")

        # risposte JSON in gzip, fatto qui così vale anche senza Apache davanti
        if (response.mimetype == "application/json" and response.status_code == 200
                and "gzip" in request.headers.get("Accept-Encoding", "")
                and not response.direct_passthrough and len(response.get_data()) >= config.GZIP_MIN_BYTES):
            response.set_data(gzip.compress(response.get_data(), GZIP_LEVEL))
            response.headers["Content-Encoding"] = "gzip"
            response.headers.add("Vary", "Accept-Encoding")
        return response

    @app.errorhandler(404)
    def handle_not_found(error):
        return api_error(404, "not_found", "La risorsa non esiste.")

    @app.errorhandler(405)
    def handle_method_not_allowed(error):
        return api_error(405, "method_not_allowed", "Metodo non permesso.")

    @app.errorhandler(413)
    def handle_too_large(error):
        return api_error(413, "too_large", "Richiesta troppo grande.")

    @app.errorhandler(500)
    def handle_internal_error(error):
        return api_error(500, "internal_error", "Errore interno del server.")

    if start_background:
        app_services.start()
        atexit.register(app_services.stop)      # alla chiusura l'ultimo stato va su disco
    return app


if __name__ == "__main__":
    # solo per provare a mano: in sviluppo si usa dev/run.sh, in produzione gunicorn
    create_app().run(host="127.0.0.1", port=5000, threaded=True)
