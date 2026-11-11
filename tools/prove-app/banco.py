"""Banco per le prove complete dell'app nel simulatore: backend sul PC (con la porta HTTPS e un certificato
autofirmato), simulatore e programma vero del robot collegato al backend (docs/app.md, sezione 11).

Lo usano tools/prove-app/scenari.py (comandi, telecomando, guasto, zone, mappe, rete) e tools/prove-app/missione.sh.
Tutto quello che crea sta in build/app-prove/, che git ignora. Non tocca né il robot né il server vero.
"""
import http.client
import json
import os
import signal
import subprocess
import sys
import time
import urllib.error
import urllib.request

REPO = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
BACKEND = os.path.join(REPO, 'backend')
PYTHON = os.path.join(BACKEND, '.venv', 'bin', 'python')
WORK = os.path.join(REPO, 'build', 'app-prove')
HTTP_PORT = 5100                    # backend delle prove (quello di sviluppo usa 5000)
HTTPS_PORT = 5543                   # porta HTTPS davanti al backend, per il programma del robot
SIM_URL = 'http://127.0.0.1:8080'   # pagina del simulatore
USERS = ('prova', 'altro')          # due utenti: il secondo serve per il telecomando in uso
PASSWORD = 'password-di-prova'
RESTART_EXIT_CODE = 3

sys.path.insert(0, BACKEND)


class Banco:
    def __init__(self, nome):
        self.dir = os.path.join(WORK, nome)
        self.data_dir = os.path.join(self.dir, 'server')
        self.maps_dir = os.path.join(self.dir, 'maps')
        self.token_file = os.path.join(self.dir, 'robot-token')
        self.cert = os.path.join(self.dir, 'localhost-cert.pem')
        self.processes = {}
        self.tokens = {}
        self.failed = 0
        self.checks = 0
        os.makedirs(self.maps_dir, exist_ok=True)

    # ------------------------------------------------------------------------------------------- backend

    def prepare_backend(self):
        """Cartella dei dati nuova, due utenti, token del robot, certificato autofirmato per localhost."""
        os.makedirs(self.data_dir, exist_ok=True)
        os.environ['MAURICE_DATA_DIR'] = self.data_dir
        import auth, config, db, device_api      # noqa: E401
        database = db.Database(os.path.join(self.data_dir, config.DATABASE_FILE))
        for user in USERS:
            auth.add_user(database, user, PASSWORD)
        token = device_api.new_robot_token(database)
        database.close()
        with open(os.open(self.token_file, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600), 'w') as token_out:
            token_out.write(token + '\n')
        if not os.path.exists(self.cert):
            subprocess.run([os.path.join(BACKEND, 'dev', 'make-cert.sh'), self.dir], check=True, stdout=subprocess.DEVNULL)

    def start_backend(self):
        env = dict(os.environ, MAURICE_DATA_DIR=self.data_dir)
        log = open(os.path.join(self.dir, 'backend.log'), 'a')
        self.processes['backend'] = subprocess.Popen(
            [os.path.join(BACKEND, '.venv', 'bin', 'gunicorn'), '--workers', '1', '--threads', '32', '--worker-class', 'gthread',
             '--bind', f'127.0.0.1:{HTTP_PORT}', '--graceful-timeout', '2', 'app:create_app()'],
            cwd=BACKEND, env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        for _ in range(50):
            try:
                urllib.request.urlopen(f'http://127.0.0.1:{HTTP_PORT}/api/auth/me', timeout=1)
            except urllib.error.HTTPError:
                return
            except OSError:
                time.sleep(0.2)
        raise RuntimeError('il backend non parte: vedi backend.log')

    def start_proxy(self):
        log = open(os.path.join(self.dir, 'proxy.log'), 'a')
        self.processes['proxy'] = subprocess.Popen(
            [PYTHON, os.path.join(BACKEND, 'dev', 'tls_proxy.py'), '--cert', self.cert,
             '--key', os.path.join(self.dir, 'localhost-key.pem'), '--listen', str(HTTPS_PORT), '--backend', str(HTTP_PORT)],
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        time.sleep(0.5)

    def stop(self, name):
        process = self.processes.pop(name, None)
        if process and process.poll() is None:
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)

    def robot_env(self):
        """Le variabili d'ambiente che collegano il programma del robot al backend delle prove."""
        return dict(os.environ, RTES_GD32='build/sim/gd32', RTES_LIDAR='build/sim/lidar', RTES_MAPS=self.maps_dir,
                    RTES_COMMANDS=os.path.join(self.dir, 'vacuum-commands'), RTES_TOKEN_FILE=self.token_file,
                    RTES_SERVER_URL=f'https://localhost:{HTTPS_PORT}', RTES_SERVER_CA=self.cert)

    # ---------------------------------------------------------------------------------- simulatore e robot

    def start_sim(self, room='sim/stanze/vuota.txt', seed='1', extra_lines=()):
        """Simulatore con il robot agganciato alla base (come prova.py --dalla-base)."""
        import math, re      # noqa: E401
        text = open(os.path.join(REPO, room)).read()
        base = re.search(r'^base x=(\S+) y=(\S+) angolo=(\S+)', text, flags=re.M)
        bx, by, bd = map(float, base.groups())
        rx, ry = bx + 0.1745 * math.cos(math.radians(bd)), by + 0.1745 * math.sin(math.radians(bd))
        text = re.sub(r'^robot .*$', f'robot x={rx:.4f} y={ry:.4f} angolo={bd}', text, flags=re.M)
        text += ''.join(line + '\n' for line in extra_lines)
        self.base = (bx, by, bd)
        room_file = os.path.join(self.dir, 'stanza.txt')
        open(room_file, 'w').write(text)
        self.processes['sim'] = subprocess.Popen(
            ['build/host/sim', '--seme', seed, room_file], cwd=REPO, stdout=open(os.path.join(self.dir, 'sim.log'), 'w'),
            stderr=subprocess.STDOUT, start_new_session=True)
        for _ in range(100):
            if self.sim_state():
                return
            time.sleep(0.1)
        raise RuntimeError('il simulatore non parte')

    def sim_state(self):
        try:
            return json.load(urllib.request.urlopen(SIM_URL + '/stato', timeout=1))
        except Exception:
            return None

    def sim_command(self, text):
        urllib.request.urlopen(urllib.request.Request(SIM_URL + '/comando', data=text.encode(), method='POST'), timeout=2).read()

    def start_robot(self, program='build/host/vacuum', mode='w', env=None):
        """Il programma vero del robot, sui core 0-3 come sul robot. Se esce con il codice 3 lo rilancia poll_robot."""
        self.program = program
        self.robot_environment = env or self.robot_env()
        self.robot_log = os.path.join(self.dir, 'robot.log')
        self.processes['robot'] = subprocess.Popen(
            ['taskset', '-c', '0-3', program], cwd=REPO, env=self.robot_environment, stdout=open(self.robot_log, mode),
            stderr=subprocess.STDOUT, start_new_session=True)

    def poll_robot(self):
        """Come i lanciatori veri: se il programma è uscito chiedendo il riavvio (cambio di mappa) lo rilancia."""
        process = self.processes.get('robot')
        if process is not None and process.poll() == RESTART_EXIT_CODE:
            self.restarts = getattr(self, 'restarts', 0) + 1
            self.start_robot(self.program, 'a', self.robot_environment)
            return True
        return False

    def stop_robot(self):
        process = self.processes.pop('robot', None)
        if process and process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(15)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)

    def stop_all(self):
        self.stop_robot()
        for name in list(self.processes):
            self.stop(name)

    # ----------------------------------------------------------------------------------------------- app

    def login(self, user):
        status, body = self.api('POST', '/api/auth/login', {'username': user, 'password': PASSWORD}, user=None)
        self.tokens[user] = body['token']

    def api(self, method, path, body=None, user=USERS[0], raw=False, timeout=40):
        """Una richiesta come la farebbe l'app. Ritorna (codice, corpo)."""
        headers = {}
        if user is not None:
            if user not in self.tokens:
                self.login(user)
            headers['Authorization'] = 'Bearer ' + self.tokens[user]
        data = None
        if body is not None:
            data = json.dumps(body).encode()
            headers['Content-Type'] = 'application/json'
        connection = http.client.HTTPConnection('127.0.0.1', HTTP_PORT, timeout=timeout)
        try:
            connection.request(method, path, body=data, headers=headers)
            response = connection.getresponse()
            content = response.read()
        finally:
            connection.close()
        if raw or response.status == 304:
            return response.status, content
        try:
            return response.status, json.loads(content)
        except ValueError:
            return response.status, content

    def state(self):
        return self.api('GET', '/api/robot/state')[1]

    def wait_state(self, description, condition, timeout=30):
        """Aspetta che lo stato visto dall'app soddisfi la condizione. Ritorna lo stato, o None se non succede in tempo."""
        limit = time.time() + timeout
        last = None
        while time.time() < limit:
            self.poll_robot()
            last = self.state()
            try:
                if condition(last):
                    return last
            except (KeyError, TypeError):
                pass
            time.sleep(0.2)
        navigation = (last or {}).get('navigation')
        print(f'   (non è successo in {timeout} s: {description}; ultimo stato: online {last.get("online")}, {navigation})')
        return None

    # -------------------------------------------------------------------------------------------- esiti

    def check(self, description, condition):
        self.checks += 1
        if not condition:
            self.failed += 1
        print(('  ok      ' if condition else '  FALLITO ') + description, flush=True)
        return condition

    def command(self, description, method, path, body=None, expect=200, reason=None, user=USERS[0]):
        """Manda un comando e controlla il codice (e il motivo del rifiuto)."""
        status, answer = self.api(method, path, body, user=user)
        got = answer.get('reason') if isinstance(answer, dict) else None
        ok = status == expect and (reason is None or got == reason)
        self.check(f'{description}: {status}' + (f' {got}' if got else '') +
                   ('' if ok else f'  (atteso {expect}' + (f' {reason}' if reason else '') + ')'), ok)
        return status, answer

    def task_numbers(self):
        """Job e scadenze mancate dei task, dall'uscita del programma del robot (dopo stop_robot)."""
        import re
        text = open(self.robot_log, errors='replace').read()
        return re.findall(r'(task_\w+): prio.*\n\s+job (\d+), deadline mancate (\d+)', text)
