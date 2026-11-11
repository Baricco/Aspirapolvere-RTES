"""Base comune delle prove automatiche del backend: app Flask in una cartella temporanea, tempi accorciati, robot finto."""
import os
import shutil
import sys
import tempfile
import threading
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import auth            # noqa: E402
import config          # noqa: E402
import device_api      # noqa: E402
import maps            # noqa: E402
from app import create_app     # noqa: E402

# tempi accorciati: le prove delle attese durano decimi di secondo invece di decine di secondi
FAST_CONFIG = {
    "STATE_LONG_POLL_S": 0.4,
    "COMMAND_TIMEOUT_S": 0.4,
    "DEVICE_LONG_POLL_S": 0.4,
    "REMOTE_IDLE_TIMEOUT_S": 0.5,
    "ONLINE_TIMEOUT_S": 0.6,
    "LOGIN_LOCK_S": 2,
    "SCRYPT_N": 2 ** 4,
}

BASE_URL = "https://localhost"      # https: il cookie della sessione è Secure
USER_PASSWORD = "password-di-prova"


def room_rtm(width=20, height=10, x=500, y=505, zones=()):
    """Una mappa di prova: una stanza libera con il bordo occupato."""
    cells = bytearray([maps.MAP_CAT_FREE]) * (width * height)
    for column in range(width):
        cells[column] = cells[(height - 1) * width + column] = maps.MAP_CAT_OCCUPIED
    for row in range(height):
        cells[row * width] = cells[row * width + width - 1] = maps.MAP_CAT_OCCUPIED
    return maps.encode_rtm(x, y, width, height, list(zones), bytes(cells))


class BackendTestCase(unittest.TestCase):
    def setUp(self):
        self.saved_config = {name: getattr(config, name) for name in FAST_CONFIG}
        for name, value in FAST_CONFIG.items():
            setattr(config, name, value)
        self.saved_origins = os.environ.pop(config.ALLOWED_ORIGINS_ENV, None)
        self.data_dir = tempfile.mkdtemp(prefix="maurice-test-")
        self.app = create_app(data_dir=self.data_dir, start_background=False)
        self.services = self.app.extensions["maurice"]
        self.client = self.app.test_client()
        auth.add_user(self.services.database, "anna", USER_PASSWORD)
        auth.add_user(self.services.database, "bruno", USER_PASSWORD)
        self.robot_token = device_api.new_robot_token(self.services.database)
        self.robot = {"Authorization": "Bearer " + self.robot_token}
        self.threads = []

    def tearDown(self):
        for thread in self.threads:
            thread.join(5)
        self.services.database.close()
        shutil.rmtree(self.data_dir, ignore_errors=True)
        for name, value in self.saved_config.items():
            setattr(config, name, value)
        if self.saved_origins is not None:
            os.environ[config.ALLOWED_ORIGINS_ENV] = self.saved_origins

    def login(self, username="anna"):
        """Login da telefono: ritorna le intestazioni con il JWT."""
        response = self.client.post("/api/auth/login", json={"username": username, "password": USER_PASSWORD})
        self.assertEqual(response.status_code, 200)
        return {"Authorization": "Bearer " + response.get_json()["token"]}

    def in_thread(self, function, *args):
        """Esegue una funzione in un altro thread e ne raccoglie il risultato in una lista."""
        results = []
        thread = threading.Thread(target=lambda: results.append(function(*args)))
        thread.start()
        self.threads.append(thread)
        return thread, results

    def robot_online(self):
        """Un contatto del robot: da qui il server lo considera online."""
        self.services.link.device_contact()

    def robot_post_state(self, state):
        response = self.app.test_client().post("/api/device/state", json=state, headers=self.robot)
        self.assertEqual(response.status_code, 200)

    def robot_answer(self, result=None, expected_type=None):
        """Il robot finto in un altro thread: prende i comandi con la richiesta lunga e manda l'esito di ognuno.
        result None = prende i comandi ma non risponde. Ritorna (thread, lista dei comandi presi)."""
        def run():
            client = self.app.test_client()
            taken = client.get("/api/device/commands", headers=self.robot).get_json()["commands"]
            for command in taken:
                if expected_type is not None:
                    self.assertEqual(command["type"], expected_type)
                if result is not None:
                    client.post(f"/api/device/commands/{command['id']}/result", json=result, headers=self.robot)
            return taken
        return self.in_thread(run)

    def command(self, path, headers, robot_result={"ok": True}, method="post", **kwargs):
        """Manda un comando dall'app mentre il robot finto risponde. Ritorna (risposta, comandi presi dal robot)."""
        self.robot_online()
        thread, taken = self.robot_answer(robot_result)
        response = getattr(self.client, method)(path, headers=headers, **kwargs)
        thread.join(5)
        return response, (taken[0] if taken else [])
