"""Stato vivo in memoria: ultimo stato del robot, coda dei comandi con le attese, telecomando.

Il processo del backend è uno solo (gunicorn con un worker e tanti thread): tutto quello che sta qui è condiviso fra i
thread e protetto da una sola threading.Condition, su cui dormono le richieste lunghe dell'app e del robot.
"""
import json
import os
import threading
import time

import config
import maps

COMMAND_UNREACHABLE = "robot_unreachable"   # il robot non ha preso il comando: non è mai stato eseguito
COMMAND_NO_RESULT = "no_result"             # il robot l'ha preso ma l'esito non è arrivato in tempo
COMMAND_DONE = "done"                       # esito arrivato

SERVICE_MAX_SLEEP_S = 1.0                   # il thread di servizio ricontrolla le scadenze almeno una volta al secondo


class Command:
    def __init__(self, command_id, command_type, params):
        self.id = command_id
        self.type = command_type
        self.params = params
        self.taken = False          # il robot l'ha ricevuto con la sua richiesta lunga
        self.result = None          # {"ok", "reason", "message"} quando arriva l'esito
        self.on_late_result = None  # chiamata se l'esito arriva dopo che l'app ha smesso di aspettare


class RobotLink:
    def __init__(self, data_dir):
        self.condition = threading.Condition()
        self.last_state_path = os.path.join(data_dir, config.LAST_STATE_FILE)

        # ultimo stato
        self.robot_state = None             # il JSON mandato dal robot, così com'è
        self.state_number = int(time.time())    # parte dall'orologio: dopo un riavvio del server non torna indietro
        self.last_contact = None            # ora dell'ultimo contatto, dall'orologio del server (secondi dal 1970)
        self.last_contact_monotonic = None
        self.online = False
        self.state_saved = True             # False se c'è uno stato più nuovo di quello su disco
        self.last_responses = {}            # sessione dell'app -> istante dell'ultima risposta alla richiesta dello stato
        self.map_version = 0

        # comandi
        self.next_command_id = int(time.time())
        self.queued_commands = []           # comandi che il robot non ha ancora preso
        self.taken_commands = {}            # id -> comando preso, in attesa dell'esito
        self.poll_number = 0                # numero dell'ultima richiesta lunga del robot: le più vecchie si chiudono

        # telecomando
        self.remote_user_id = None
        self.remote_username = None
        self.remote_last_command = 0.0

        self.stopping = False
        self.stop_event = threading.Event()
        self.thread = None
        self.load_last_state()

    # ------------------------------------------------------------------------------------------------ stato

    def load_last_state(self):
        """L'ultimo stato scritto su disco prima di un riavvio: l'app lo vede subito, con il robot offline."""
        try:
            with open(self.last_state_path, encoding="utf-8") as state_file:
                saved = json.load(state_file)
            self.robot_state = saved["state"]
            self.last_contact = saved["last_contact"]
        except (OSError, ValueError, KeyError, TypeError):
            pass

    def save_last_state(self):
        with self.condition:
            if self.state_saved or self.robot_state is None:
                return
            body = json.dumps({"state": self.robot_state, "last_contact": self.last_contact})
            self.state_saved = True
        maps.write_file_atomic(self.last_state_path, body.encode("utf-8"))

    def contact(self):
        """Il robot si è fatto sentire. Con self.condition preso."""
        self.last_contact = time.time()
        self.last_contact_monotonic = time.monotonic()
        if not self.online:
            self.online = True
            self.changed()

    def changed(self):
        """Qualcosa che le app devono vedere è cambiato: conta come uno stato nuovo. Con self.condition preso."""
        self.state_number += 1
        self.condition.notify_all()

    def device_contact(self):
        with self.condition:
            self.contact()

    def update_state(self, state):
        with self.condition:
            self.robot_state = state
            self.state_saved = False
            self.contact()
            self.changed()

    def set_map_version(self, version):
        """La mappa dal vivo è cambiata: le app in attesa dello stato si svegliano e la scaricano."""
        with self.condition:
            self.map_version = version
            self.changed()

    def snapshot(self):
        """Lo stato per l'app: quello del robot più i campi del server. Con self.condition preso."""
        state = dict(self.robot_state) if self.robot_state else {}
        state["online"] = self.online
        state["last_contact"] = self.last_contact
        state["remote_control_user"] = self.current_remote_username()
        state["map_version"] = self.map_version
        state["state_number"] = self.state_number
        return state

    def current_state(self):
        with self.condition:
            return self.snapshot()

    def wait_state(self, after, min_interval_ms, client_key):
        """Richiesta lunga dello stato (docs/app.md, sezione 4.4). Ritorna lo stato, oppure None se non è cambiato niente."""
        with self.condition:
            now = time.monotonic()
            if after is None or after > self.state_number:
                self.last_responses[client_key] = now
                return self.snapshot()
            deadline = now + config.STATE_LONG_POLL_S
            earliest = self.last_responses.get(client_key, 0.0) + min_interval_ms / 1000.0
            while True:
                now = time.monotonic()
                is_new = self.state_number > after
                if is_new and now >= earliest:
                    self.last_responses[client_key] = now
                    return self.snapshot()
                if now >= deadline or self.stopping:
                    return None
                self.condition.wait(min(deadline, earliest) - now if is_new else deadline - now)

    # ---------------------------------------------------------------------------------------------- comandi

    def send_command(self, command_type, params, on_late_result=None):
        """Mette un comando in coda e aspetta l'esito, al massimo COMMAND_TIMEOUT_S.

        Ritorna (COMMAND_DONE, esito), (COMMAND_UNREACHABLE, None) o (COMMAND_NO_RESULT, None)."""
        with self.condition:
            if not self.online:
                return COMMAND_UNREACHABLE, None
            command = Command(self.next_command_id, command_type, params)
            self.next_command_id += 1
            self.queued_commands.append(command)
            self.condition.notify_all()
            deadline = time.monotonic() + config.COMMAND_TIMEOUT_S
            while command.result is None:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                self.condition.wait(remaining)
            if command.result is not None:
                return COMMAND_DONE, command.result
            if command.taken:
                command.on_late_result = on_late_result
                return COMMAND_NO_RESULT, None
            self.queued_commands.remove(command)    # mai preso: si butta, il robot non lo eseguirà
            return COMMAND_UNREACHABLE, None

    def take_commands(self):
        """Richiesta lunga del robot: i comandi in coda, oppure una lista vuota dopo DEVICE_LONG_POLL_S."""
        with self.condition:
            self.contact()
            self.poll_number += 1
            poll_number = self.poll_number
            deadline = time.monotonic() + config.DEVICE_LONG_POLL_S
            while not self.queued_commands:
                remaining = deadline - time.monotonic()
                # una richiesta più nuova del robot vuol dire che questa connessione è morta: non deve prendere comandi
                if remaining <= 0 or poll_number != self.poll_number or self.stopping:
                    return []
                self.condition.wait(remaining)
            if poll_number != self.poll_number:
                return []
            commands = self.queued_commands
            self.queued_commands = []
            for command in commands:
                command.taken = True
                self.taken_commands[command.id] = command
            # qui il contatto non si rinnova: se la connessione del robot è morta il server non lo sa, e consegnare un
            # comando non prova che il robot ci sia
            return [dict(command.params, id=command.id, type=command.type) for command in commands]

    def set_command_result(self, command_id, result):
        """Esito mandato dal robot. Ritorna False se il comando non esiste."""
        with self.condition:
            self.contact()
            command = self.taken_commands.pop(command_id, None)
            if command is None:
                return False
            command.result = result
            late = command.on_late_result
            self.condition.notify_all()
        if late:
            late(result)
        return True

    # ------------------------------------------------------------------------------------------ telecomando

    def is_remote_free(self):
        """True se nessuno tiene il telecomando, o se chi lo teneva non manda comandi da REMOTE_IDLE_TIMEOUT_S. Con self.condition preso."""
        return self.remote_user_id is None or \
            time.monotonic() - self.remote_last_command >= config.REMOTE_IDLE_TIMEOUT_S

    def current_remote_username(self):
        """Chi tiene il telecomando, o None. Con self.condition preso."""
        return None if self.is_remote_free() else self.remote_username

    def remote_holder(self):
        """(id, nome) di chi tiene il telecomando, oppure (None, None)."""
        with self.condition:
            if self.is_remote_free():
                return None, None
            return self.remote_user_id, self.remote_username

    def remote_take(self, user_id, username):
        """Prende il telecomando, o ne rinnova il tempo. Ritorna True se prima era libero."""
        with self.condition:
            was_free = self.is_remote_free()
            self.remote_user_id = user_id
            self.remote_username = username
            self.remote_last_command = time.monotonic()
            if was_free:
                self.changed()
            return was_free

    def remote_release(self):
        with self.condition:
            if self.remote_user_id is not None:
                self.remote_user_id = None
                self.remote_username = None
                self.changed()

    # ------------------------------------------------------------------------------------- thread di servizio

    def start(self):
        self.thread = threading.Thread(target=self.service_loop, name="robot_link", daemon=True)
        self.thread.start()

    def stop(self):
        self.stop_event.set()
        with self.condition:
            self.stopping = True
            self.condition.notify_all()
        if self.thread:
            self.thread.join()
        self.save_last_state()

    def service_tick(self):
        """Le cose che cambiano con il passare del tempo: robot offline, telecomando scaduto. Ritorna fra quanto ricontrollare."""
        with self.condition:
            now = time.monotonic()
            wait = config.STATE_SAVE_INTERVAL_S
            if self.online:
                expires = self.last_contact_monotonic + config.ONLINE_TIMEOUT_S
                if now >= expires:
                    self.online = False
                    self.changed()
                else:
                    wait = min(wait, expires - now)
            if self.remote_user_id is not None:
                expires = self.remote_last_command + config.REMOTE_IDLE_TIMEOUT_S
                if now >= expires:
                    self.remote_user_id = None
                    self.remote_username = None
                    self.changed()
                else:
                    wait = min(wait, expires - now)
            # le risposte vecchie non servono più: la mappa delle app non cresce senza limite
            oldest = now - config.STATE_LONG_POLL_S - config.MIN_INTERVAL_MAX_MS / 1000.0
            self.last_responses = {key: when for key, when in self.last_responses.items() if when >= oldest}
            return wait

    def service_loop(self):
        last_save = time.monotonic()
        while not self.stop_event.wait(min(self.service_tick(), SERVICE_MAX_SLEEP_S)):
            if time.monotonic() - last_save >= config.STATE_SAVE_INTERVAL_S:
                last_save = time.monotonic()
                self.save_last_state()
