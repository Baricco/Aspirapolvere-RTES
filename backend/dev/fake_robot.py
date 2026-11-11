#!/usr/bin/env python3
"""Robot finto per provare backend e app senza il robot e senza il simulatore (docs/app.md, sezione 11).

Parla con il backend come il programma vero: richiesta lunga dei comandi, esiti, stato, mappa dal vivo a differenze,
mappa salvata. Dentro c'è un modello molto semplice: una stanza rettangolare con la base nell'origine, una pulizia a
strisce, il telecomando, le zone, il cambio di mappa con il "riavvio". Non c'è niente di real-time: serve solo a far
vedere all'app dati verosimili.

Uso, dalla cartella backend/:
    .venv/bin/python dev/fake_robot.py --token-file FILE [--server http://127.0.0.1:5000] [--ca certificato.pem]
Comandi da tastiera mentre gira: fault (guasto), lost (posizione persa sì/no), battery N (percentuale), dustbin
(cassetto sì/no), offline N (sparisce per N secondi), quit.
"""
import argparse
import json
import math
import os
import ssl
import sys
import threading
import time
import urllib.error
import urllib.request

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import config   # noqa: E402
import maps     # noqa: E402

# tempi, come quelli del programma vero (tasks/vacuum/cloud/config.example.h)
STATE_CYCLE_S = 0.1                 # ciclo del thread dello stato
STATE_IDLE_INTERVAL_S = 10.0        # a riposo lo stato parte ogni tanto
LIVE_MAP_INTERVAL_S = 0.5           # ogni quanto partono le differenze della mappa
COMMANDS_TIMEOUT_S = 40.0           # tempo massimo della richiesta lunga
REQUEST_TIMEOUT_S = 5.0
RECONNECT_DELAYS_S = (1, 2, 5, 10)
RESTART_TIME_S = 3.0                # quanto dura il "riavvio" dopo un cambio di mappa
REMOTE_MOVE_TIMEOUT_S = 0.5
REMOTE_IDLE_TIMEOUT_S = 30.0
REMOTE_MAX_V = 0.35
REMOTE_MAX_OMEGA = 1.2

# il mondo finto, in metri sulla mappa: la base è nell'origine, contro il muro di sinistra
ROOM_X_MIN, ROOM_X_MAX = -0.25, 4.75
ROOM_Y_MIN, ROOM_Y_MAX = -2.0, 2.0
LIDAR_RANGE = 2.5                   # quanto lontano "vede" il robot finto
CRUISING_SPEED = 0.30
ROTATION_SPEED = 1.5
STRIP_STEP = 0.3
STRIP_MARGIN = 0.35
HOME_X = 0.30                       # punto davanti alla base
BASE_ZONE = {"x": -0.365, "y": -0.22, "w": 0.26, "h": 0.44}
BATTERY_DRAIN_PER_S = 0.01
BATTERY_CHARGE_PER_S = 0.05
BATTERY_LOW_PERCENT = 15

MESSAGES = {
    "busy": "Il robot sta già facendo una missione.",
    "not_running": "Non c'è una missione in corso.",
    "not_paused": "Il robot non è in pausa.",
    "already_home": "Il robot è già sulla base.",
    "position_lost": "Il robot non sa dove si trova.",
    "cause_still_present": "La causa del guasto c'è ancora.",
    "not_in_fault": "Il robot non è in guasto.",
    "dustbin_missing": "Manca il cassetto della polvere.",
    "not_started_on_base": "Il robot non è partito dalla base: le zone non si possono cambiare.",
    "not_resting": "Il robot deve essere a riposo sulla base.",
    "download_failed": "Non sono riuscito a scaricare la mappa.",
    "unknown_command": "Comando sconosciuto.",
}


class Server:
    """Le richieste HTTP al backend, con il token del robot."""

    def __init__(self, url, token, ca_file):
        self.url = url.rstrip("/")
        self.token = token
        self.context = ssl.create_default_context(cafile=ca_file) if self.url.startswith("https") else None

    def request(self, method, path, body=None, is_json=True, timeout=REQUEST_TIMEOUT_S):
        """Ritorna (codice, corpo). Con la rete giù solleva OSError."""
        headers = {"Authorization": "Bearer " + self.token}
        data = None
        if body is not None:
            data = json.dumps(body).encode() if is_json else body
            headers["Content-Type"] = "application/json" if is_json else "application/octet-stream"
        request = urllib.request.Request(self.url + path, data=data, method=method, headers=headers)
        try:
            with urllib.request.urlopen(request, timeout=timeout, context=self.context) as response:
                return response.status, response.read()
        except urllib.error.HTTPError as error:
            return error.code, error.read()


class FakeRobot:
    def __init__(self, server, maps_dir):
        self.server = server
        self.map_path = os.path.join(maps_dir, "map.rtm")
        self.lock = threading.Lock()        # fra il thread dei comandi, quello dello stato e la tastiera
        self.start_time = time.monotonic()
        self.offline_until = 0.0
        self.running = True

        # robot
        self.x, self.y, self.theta = 0.0, 0.0, 0.0
        self.v, self.omega = 0.0, 0.0
        self.state = "sleep"
        self.mission = "none"
        self.mission_number = 0
        self.mission_start = 0.0
        self.mission_duration = 0
        self.waypoints = []
        self.on_base = True
        self.fault = None
        self.is_position_lost = False
        self.has_dustbin = True
        self.battery_percent = 80.0
        self.is_remote = False
        self.remote_brushes = False
        self.remote_suction = False
        self.remote_v, self.remote_omega = 0.0, 0.0
        self.remote_last_move = 0.0
        self.remote_last_command = 0.0
        self.remote_had_mission = False
        self.paused_state = "follow_path"   # stato da riprendere dopo una pausa
        self.restart_pending = False        # un cambio di mappa chiede il "riavvio"
        self.important_change = True        # c'è qualcosa da mandare subito

        # mappa
        side = config.FINE_MAP_CELL_NUMBER
        self.grid = bytearray(side * side)          # la mappa del robot, a categorie
        self.confirmed = bytearray(side * side)     # l'ultima confermata dal server
        self.map_version = 0
        self.user_zones = []
        self.file_crc = None
        self.is_map_ready = False
        self.save_pending = None            # motivo del prossimo invio della mappa salvata: "startup" o "save"
        self.full_map_pending = True
        self.load_map_file()

    # ---------------------------------------------------------------------------------------------- mappa

    def load_map_file(self):
        """All'avvio: carica map.rtm se c'è, e prepara gli invii della mappa salvata e di quella dal vivo."""
        side = config.FINE_MAP_CELL_NUMBER
        self.grid = bytearray(side * side)
        self.user_zones = []
        self.file_crc = None
        self.is_map_ready = False
        try:
            with open(self.map_path, "rb") as map_file:
                data = map_file.read()
            saved = maps.decode_rtm(data)
            for row in range(saved.height):
                start = (saved.y + row) * side + saved.x
                self.grid[start:start + saved.width] = saved.cells[row * saved.width:(row + 1) * saved.width]
            self.user_zones = [maps.zone_from_cells(zone) for zone in saved.zones]
            self.file_crc = maps.file_crc(data)
            self.is_map_ready = True
        except (OSError, maps.MapFormatError):
            pass
        self.save_pending = maps.SAVED_REASON_STARTUP
        self.full_map_pending = True

    def bounding_rectangle(self, grid, x_min=0, y_min=0, x_max=None, y_max=None, other=None):
        """Il rettangolo più piccolo con le celle non sconosciute (o, con other, diverse da other) dentro la finestra."""
        side = config.FINE_MAP_CELL_NUMBER
        x_max = side - 1 if x_max is None else x_max
        y_max = side - 1 if y_max is None else y_max
        left, right, bottom, top = side, -1, side, -1
        for y in range(y_min, y_max + 1):
            row = grid[y * side + x_min:y * side + x_max + 1]
            reference = other[y * side + x_min:y * side + x_max + 1] if other is not None else bytes(len(row))
            if row == reference:
                continue
            changed = [index for index in range(len(row)) if row[index] != reference[index]]
            left, right = min(left, x_min + changed[0]), max(right, x_min + changed[-1])
            bottom, top = min(bottom, y), y
        return (left, bottom, right - left + 1, top - bottom + 1) if right >= 0 else None

    def encode_rectangle(self, grid, rectangle, zones=()):
        side = config.FINE_MAP_CELL_NUMBER
        x, y, width, height = rectangle
        cells = b"".join(bytes(grid[row * side + x:row * side + x + width]) for row in range(y, y + height))
        return maps.encode_rtm(x, y, width, height, list(zones), cells)

    def save_map_file(self):
        """Salva map.rtm come fa il task aperiodico: file temporaneo e rinomina."""
        rectangle = self.bounding_rectangle(self.grid)
        if rectangle is None:
            return
        data = self.encode_rectangle(self.grid, rectangle, [maps.zone_to_cells(zone) for zone in self.user_zones])
        maps.write_file_atomic(self.map_path, data)
        self.file_crc = maps.file_crc(data)
        self.save_pending = maps.SAVED_REASON_SAVE
        self.important_change = True

    def see_room(self):
        """Il lidar finto: le celle della stanza entro LIDAR_RANGE diventano libere, quelle del bordo occupate."""
        size = config.FINE_MAP_CELL_SIZE
        origin = config.FINE_MAP_ORIGIN
        side = config.FINE_MAP_CELL_NUMBER
        reach = int(LIDAR_RANGE / size)
        center_x, center_y = int(math.floor(self.x / size)) + origin, int(math.floor(self.y / size)) + origin
        for cell_y in range(center_y - reach, center_y + reach + 1):
            for cell_x in range(center_x - reach, center_x + reach + 1):
                x, y = (cell_x - origin + 0.5) * size, (cell_y - origin + 0.5) * size
                if math.hypot(x - self.x, y - self.y) > LIDAR_RANGE:
                    continue
                inside = ROOM_X_MIN <= x <= ROOM_X_MAX and ROOM_Y_MIN <= y <= ROOM_Y_MAX
                if not inside:
                    continue
                on_wall = min(x - ROOM_X_MIN, ROOM_X_MAX - x, y - ROOM_Y_MIN, ROOM_Y_MAX - y) < size
                self.grid[cell_y * side + cell_x] = maps.MAP_CAT_OCCUPIED if on_wall else maps.MAP_CAT_FREE

    def send_maps(self):
        """Dal thread dello stato: mappa salvata, mappa dal vivo intera, differenze. Ritorna False se la rete è giù."""
        with self.lock:
            save_pending, full_pending = self.save_pending, self.full_map_pending
        if save_pending:
            try:
                with open(self.map_path, "rb") as map_file:
                    data = map_file.read()
            except OSError:
                data = b""
            status, _ = self.server.request("PUT", f"/api/device/map/saved?reason={save_pending}", data, is_json=False)
            if status != 200:
                return False
            with self.lock:
                if self.save_pending == save_pending:
                    self.save_pending = None
        if full_pending:
            with self.lock:
                snapshot = bytearray(self.grid)
            rectangle = self.bounding_rectangle(snapshot)
            data = self.encode_rectangle(snapshot, rectangle) if rectangle else maps.empty_rtm()
            status, _ = self.server.request("PUT", f"/api/device/map?version={self.map_version}", data, is_json=False)
            if status != 200:
                return False
            self.confirmed = snapshot
            with self.lock:
                self.full_map_pending = False
            return True
        with self.lock:
            snapshot = bytearray(self.grid)
        rectangle = self.bounding_rectangle(snapshot, other=self.confirmed)
        if rectangle is None:
            return True
        data = self.encode_rectangle(snapshot, rectangle)
        status, _ = self.server.request(
            "POST", f"/api/device/map/patch?base_version={self.map_version}&version={self.map_version + 1}", data,
            is_json=False)
        if status == 200:
            self.confirmed = snapshot
            self.map_version += 1
        elif status == 409:
            # il server non ha la versione di partenza: si rimanda la copia confermata intera, poi le differenze
            rectangle = self.bounding_rectangle(self.confirmed)
            data = self.encode_rectangle(self.confirmed, rectangle) if rectangle else maps.empty_rtm()
            status, _ = self.server.request("PUT", f"/api/device/map?version={self.map_version}", data, is_json=False)
        return status == 200

    # ------------------------------------------------------------------------------------------ movimento

    def plan_cleaning(self):
        """Una pulizia a strisce che copre la stanza, poi il ritorno davanti alla base."""
        self.waypoints = [(HOME_X, 0.0)]
        y = ROOM_Y_MIN + STRIP_MARGIN
        left, right = ROOM_X_MIN + STRIP_MARGIN + 0.4, ROOM_X_MAX - STRIP_MARGIN
        to_right = True
        while y <= ROOM_Y_MAX - STRIP_MARGIN:
            self.waypoints += [(left, y), (right, y)] if to_right else [(right, y), (left, y)]
            to_right = not to_right
            y += STRIP_STEP

    def set_state(self, state):
        if state != self.state:
            self.state = state
            self.important_change = True

    def go_home(self):
        self.mission = "return_home"
        self.waypoints = [(HOME_X, 0.0)]
        self.set_state("return_home")

    def end_remote(self):
        self.is_remote = False
        self.remote_v = self.remote_omega = 0.0
        self.remote_brushes = self.remote_suction = False
        if self.state == "remote":
            self.set_state("pause" if self.remote_had_mission else "sleep")
        self.important_change = True

    def step(self, dt):
        """Un passo del modello, ogni STATE_CYCLE_S."""
        now = time.monotonic()
        self.v = self.omega = 0.0
        if self.is_remote and now - self.remote_last_command >= REMOTE_IDLE_TIMEOUT_S:
            self.end_remote()
        if self.fault:
            self.set_state("failure")
        elif self.state == "failure":
            # guasto azzerato: telecomando se era attivo, pausa se c'era una missione, altrimenti riposo
            self.set_state("remote" if self.is_remote else "pause" if self.mission != "none" else "sleep")

        if self.state == "remote":
            if now - self.remote_last_move >= REMOTE_MOVE_TIMEOUT_S or self.is_position_lost:
                self.remote_v = self.remote_omega = 0.0
            self.v, self.omega = self.remote_v, self.remote_omega
            self.theta = math.remainder(self.theta + self.omega * dt, 2 * math.pi)
            new_x = self.x + self.v * math.cos(self.theta) * dt
            new_y = self.y + self.v * math.sin(self.theta) * dt
            if ROOM_X_MIN + 0.2 <= new_x <= ROOM_X_MAX - 0.2 and ROOM_Y_MIN + 0.2 <= new_y <= ROOM_Y_MAX - 0.2:
                self.x, self.y = new_x, new_y
            else:
                self.v = 0.0
        elif self.state in ("leave_base", "follow_path", "return_home", "hook_up") and not self.is_position_lost:
            self.follow_waypoints(dt)

        self.on_base = math.hypot(self.x, self.y) < 0.03
        if self.on_base and self.state in ("sleep", "pause"):
            self.battery_percent = min(100.0, self.battery_percent + BATTERY_CHARGE_PER_S * dt)
        elif self.state not in ("sleep", "pause", "failure"):
            self.battery_percent = max(0.0, self.battery_percent - BATTERY_DRAIN_PER_S * dt)
        if self.mission != "none" and self.state not in ("sleep", "pause", "failure"):
            self.mission_duration = int(now - self.mission_start)
        if self.v or self.omega:
            self.see_room()

    def follow_waypoints(self, dt):
        if not self.waypoints:
            if self.state == "follow_path":         # pulizia finita: si torna a casa
                self.go_home()
            elif self.state == "return_home":       # davanti alla base: aggancio all'indietro
                self.waypoints = [(0.0, 0.0)]
                self.set_state("hook_up")
            elif self.state == "hook_up":
                self.x, self.y, self.theta = 0.0, 0.0, 0.0
                self.mission = "none"
                self.set_state("sleep")
                self.is_map_ready = True
                self.save_map_file()
            return
        target_x, target_y = self.waypoints[0]
        distance = math.hypot(target_x - self.x, target_y - self.y)
        if distance < 0.02:
            self.waypoints.pop(0)
            if self.state == "leave_base":
                self.mission = "clean"
                self.set_state("follow_path")
            return
        backwards = self.state == "hook_up"
        heading = math.atan2(target_y - self.y, target_x - self.x) + (math.pi if backwards else 0.0)
        error = math.remainder(heading - self.theta, 2 * math.pi)
        if abs(error) > 0.05:
            self.omega = math.copysign(min(ROTATION_SPEED, abs(error) / dt), error)
            self.theta = math.remainder(self.theta + self.omega * dt, 2 * math.pi)
            return
        speed = min(CRUISING_SPEED if not backwards else 0.05, distance / dt)
        self.v = -speed if backwards else speed
        self.x += speed * (target_x - self.x) / distance * dt
        self.y += speed * (target_y - self.y) / distance * dt

    # ---------------------------------------------------------------------------------------------- stato

    def build_state(self):
        on = self.state in ("follow_path", "leave_base") or (self.state == "remote" and self.remote_brushes)
        suction = self.state == "follow_path" or (self.state == "remote" and self.remote_suction)
        zones = [BASE_ZONE] + self.user_zones
        map_state = {"state": "lost" if self.is_position_lost else "normal", "save_failed": False,
                     "started_on_base": True}
        if self.file_crc:
            map_state["file_crc"] = self.file_crc
        map_state["forbidden_zones"] = zones
        objective = {"type": "stop", "x": 0.0, "y": 0.0, "theta_deg": 0.0}
        if self.waypoints and self.state in ("leave_base", "follow_path", "return_home", "hook_up"):
            objective = {"type": "move", "x": self.waypoints[0][0], "y": self.waypoints[0][1], "theta_deg": 0.0}
        elif self.state == "remote":
            objective["type"] = "const_speed"
        front = int(max(0.0, (ROOM_X_MAX - self.x) * abs(math.cos(self.theta)) + 0.3) * 1000)
        return {
            "format": 1, "robot_time": int(time.time()), "uptime_s": int(time.monotonic() - self.start_time),
            "remote_control": self.is_remote,
            "navigation": {"state": self.state, "mission": self.mission, "interrupted_mission": "none",
                           "mission_number": self.mission_number, "mission_duration_s": self.mission_duration},
            "objective": objective,
            "position": {"x": round(self.x, 3), "y": round(self.y, 3), "theta_deg": round(math.degrees(self.theta), 1),
                         "v": round(self.v, 3), "omega": round(self.omega, 3), "valid": not self.is_position_lost},
            "map": map_state,
            "safety": {"errors": [], "max_v_forward": 0.5, "max_v_backward": 0.5},
            "lidar": {"valid": True, "sector_distances_mm": [1500] * 12, "corridor_front_mm": front,
                      "corridor_back_mm": 1200},
            "battery": {"voltage": round(13.8 + 2.8 * self.battery_percent / 100, 2),
                        "percent": int(self.battery_percent),
                        "level": "low" if self.battery_percent < BATTERY_LOW_PERCENT else "ok",
                        "charging": self.on_base, "charge_complete": self.on_base and self.battery_percent >= 100,
                        "temperature_c": 31},
            "motors": {"side_brush": 60 if on else 0, "main_brush": 60 if on else 0,
                       "suction_rpm": 15500 if suction else 0},
            "driver": {"phase": "fault" if self.fault else "ready", "fault": self.fault, "limits": [],
                       "on_base": self.on_base, "lifted": False,
                       "docking": {"result": "in_progress" if self.state == "hook_up" else "idle", "phase": "none",
                                   "attempt": 0},
                       "lidar": {"ok": True, "rotation_hz": 6.0}},
            "sensors": {"bumper_left": False, "bumper_right": False, "cliff": [False] * 4, "dustbin": self.has_dustbin,
                        "currents_ma": {"wheel_left": 180 if self.v else 20, "wheel_right": 175 if self.v else 20,
                                        "side_brush": 90 if on else 5, "main_brush": 220 if on else 58,
                                        "suction": 600 if suction else 14}},
            "tasks": [{"name": name, "deadline_misses": 0, "missed_activations": 0}
                      for name in ("movement", "safety", "lidar", "navigation", "mapping")],
        }

    def state_loop(self):
        last_sent = 0.0
        last_map = 0.0
        while self.running:
            time.sleep(STATE_CYCLE_S)
            now = time.monotonic()
            with self.lock:
                self.step(STATE_CYCLE_S)
                moving = self.v != 0 or self.omega != 0 or self.state not in ("sleep", "pause", "failure", "remote")
                due = self.important_change or moving or now - last_sent >= STATE_IDLE_INTERVAL_S
                state = self.build_state() if due else None
                self.important_change = False
            if now < self.offline_until:
                continue
            try:
                if state is not None:
                    self.server.request("POST", "/api/device/state", state)     # se non parte si butta
                    last_sent = now
                if now - last_map >= LIVE_MAP_INTERVAL_S:
                    last_map = now
                    self.send_maps()
            except OSError:
                pass

    # --------------------------------------------------------------------------------------------- comandi

    def refuse(self, reason):
        return {"ok": False, "reason": reason, "message": MESSAGES[reason]}

    def clear_fault(self):
        """Come drv_azzera_guasto: qui la causa sparisce sempre."""
        self.fault = None
        self.important_change = True
        return True

    def take_remote(self):
        self.remote_last_command = time.monotonic()
        if not self.is_remote:
            self.remote_had_mission = self.mission != "none"
            self.is_remote = True
            self.important_change = True
            if self.state != "failure":
                self.set_state("remote")

    def run_command(self, command):
        """L'interprete dei comandi finto. Ritorna l'esito {"ok", "reason", "message"}."""
        kind = command["type"]
        ok = {"ok": True}
        running = self.state in ("leave_base", "follow_path", "return_home", "hook_up")

        if kind == "fault_reset":
            if not self.fault:
                return self.refuse("not_in_fault")
            self.clear_fault()
            return ok
        if kind in ("start", "pause", "resume", "home"):
            if self.fault and kind != "pause":
                self.clear_fault()
                self.step(0.0)
            if self.is_remote:
                self.end_remote()           # comando di chi tiene il telecomando: rilascio, poi il comando
                if kind == "pause" and self.state == "pause":
                    return ok
            running = self.state in ("leave_base", "follow_path", "return_home", "hook_up")
            if self.is_position_lost and kind != "home":
                return self.refuse("position_lost")
        if kind == "start":
            if self.state != "sleep":
                return self.refuse("busy")
            self.mission_number += 1
            self.mission_start = time.monotonic()
            self.mission_duration = 0
            self.mission = "explore"
            self.plan_cleaning()
            self.set_state("leave_base")
        elif kind == "pause":
            if not running:
                return self.refuse("not_running")
            self.paused_state = self.state
            self.set_state("pause")
        elif kind == "resume":
            if self.state != "pause":
                return self.refuse("not_paused")
            self.set_state(self.paused_state if self.waypoints else "sleep")
        elif kind == "home":
            if self.state == "sleep" and self.on_base:
                return self.refuse("already_home")
            if self.state not in ("return_home", "hook_up"):
                if self.mission == "none":
                    self.mission_start = time.monotonic()
                self.go_home()
        elif kind in ("remote_take", "remote_keepalive"):
            self.take_remote()
        elif kind == "remote_move":
            self.take_remote()
            if self.is_position_lost:
                return self.refuse("position_lost")
            self.remote_v = max(-REMOTE_MAX_V, min(REMOTE_MAX_V, float(command.get("v", 0))))
            self.remote_omega = max(-REMOTE_MAX_OMEGA, min(REMOTE_MAX_OMEGA, float(command.get("omega", 0))))
            self.remote_last_move = time.monotonic()
        elif kind == "remote_cleaning":
            self.take_remote()
            if command.get("suction") and not self.has_dustbin:
                return self.refuse("dustbin_missing")
            self.remote_brushes, self.remote_suction = bool(command.get("brushes")), bool(command.get("suction"))
            self.important_change = True
        elif kind == "remote_release":
            self.end_remote()
        elif kind == "zones":
            self.user_zones = list(command.get("zones", []))
            if self.file_crc:
                self.save_map_file()
            self.important_change = True
        elif kind in ("map_select", "map_reset"):
            if self.state != "sleep" or not self.on_base:
                return self.refuse("not_resting")
            if kind == "map_select":
                try:
                    status, data = self.server.request("GET", f"/api/device/maps/{command['map_id']}")
                    if status != 200 or maps.file_crc(data) != command.get("crc"):
                        return self.refuse("download_failed")
                    maps.decode_rtm(data)
                except (OSError, maps.MapFormatError, KeyError):
                    return self.refuse("download_failed")
                maps.write_file_atomic(self.map_path, data)
            elif os.path.exists(self.map_path):
                os.remove(self.map_path)
            self.restart_pending = True
        else:
            return self.refuse("unknown_command")
        return ok

    def restart(self):
        """Il "riavvio" dopo un cambio di mappa: il robot sparisce per un po', poi riparte con la mappa nuova."""
        print("riavvio...", flush=True)
        self.offline_until = time.monotonic() + RESTART_TIME_S
        time.sleep(RESTART_TIME_S)
        with self.lock:
            self.start_time = time.monotonic()
            self.mission_number = 0
            self.mission = "none"
            self.map_version = 0
            self.restart_pending = False
            self.load_map_file()
            self.important_change = True

    def commands_loop(self):
        failures = 0
        while self.running:
            if time.monotonic() < self.offline_until:
                time.sleep(0.2)
                continue
            try:
                status, body = self.server.request("GET", "/api/device/commands", timeout=COMMANDS_TIMEOUT_S)
                if status != 200:
                    raise OSError(f"risposta {status}")
                failures = 0
                for command in json.loads(body)["commands"]:
                    with self.lock:
                        result = self.run_command(command)
                    print(f"comando {command}: {result}", flush=True)
                    self.server.request("POST", f"/api/device/commands/{command['id']}/result", result)
                    if self.restart_pending:
                        self.restart()
            except (OSError, ValueError) as error:
                delay = RECONNECT_DELAYS_S[min(failures, len(RECONNECT_DELAYS_S) - 1)]
                failures += 1
                print(f"server non raggiungibile ({error}), riprovo fra {delay} s", flush=True)
                time.sleep(delay)

    def keyboard_loop(self):
        for line in sys.stdin:
            words = line.split()
            if not words:
                continue
            with self.lock:
                if words[0] == "fault":
                    self.fault = {"code": 1, "text": "task fault 1: all the unlock attempts failed"}
                elif words[0] == "lost":
                    self.is_position_lost = not self.is_position_lost
                elif words[0] == "battery" and len(words) > 1:
                    self.battery_percent = float(words[1])
                elif words[0] == "dustbin":
                    self.has_dustbin = not self.has_dustbin
                elif words[0] == "offline" and len(words) > 1:
                    self.offline_until = time.monotonic() + float(words[1])
                elif words[0] == "quit":
                    self.running = False
                self.important_change = True
            if not self.running:
                break


def main():
    parser = argparse.ArgumentParser(description="Robot finto per provare backend e app.")
    parser.add_argument("--server", default="http://127.0.0.1:5000")
    parser.add_argument("--token-file", required=True, help="file con il token del robot (quello stampato da admin.py)")
    parser.add_argument("--ca", help="certificato del server, se è HTTPS con un certificato autofirmato")
    parser.add_argument("--maps-dir", default=None, help="cartella dove il robot finto tiene map.rtm")
    arguments = parser.parse_args()
    with open(arguments.token_file, encoding="utf-8") as token_file:
        token = token_file.read().strip()
    maps_dir = arguments.maps_dir or os.path.join(os.path.dirname(os.path.abspath(arguments.token_file)), "fake-robot")
    os.makedirs(maps_dir, exist_ok=True)
    robot = FakeRobot(Server(arguments.server, token, arguments.ca), maps_dir)
    threading.Thread(target=robot.state_loop, daemon=True).start()
    threading.Thread(target=robot.commands_loop, daemon=True).start()
    print(f"robot finto collegato a {arguments.server} (mappa in {maps_dir}); scrivi fault, lost, battery N, dustbin, "
          "offline N, quit", flush=True)
    try:
        robot.keyboard_loop()
        while robot.running:        # senza tastiera (uscita reindirizzata) resta acceso
            time.sleep(1)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
