"""Costanti non private del backend: tempi, limiti e nomi dei file (docs/app.md).

I valori privati (cartella dei dati, origini permesse in sviluppo) arrivano dalle variabili d'ambiente: in produzione
dall'unità systemd, in sviluppo dal file .env di questa cartella, che git ignora (esempio: config.example.env).
Gli altri moduli leggono le costanti come config.NOME, così le prove automatiche possono accorciare i tempi.
"""
import os

# --- variabili d'ambiente con i valori privati ---
DATA_DIR_ENV = "MAURICE_DATA_DIR"                # cartella dei dati: database, mappe, chiave del JWT, ultimo stato, scia
ALLOWED_ORIGINS_ENV = "MAURICE_ALLOWED_ORIGINS"  # solo in sviluppo: origini del web permesse, separate da virgole
ENV_FILE_NAME = ".env"                           # file con le variabili d'ambiente per lo sviluppo, accanto a questo

# --- file dentro la cartella dei dati ---
DATABASE_FILE = "maurice.sqlite"
JWT_KEY_FILE = "jwt-key"
LAST_STATE_FILE = "last-state.json"
TRAIL_FILE = "trail.json"
MAPS_DIR = "maps"
MAP_FILE_PREFIX = "map-"                         # i file delle mappe si chiamano map-<id>.rtm
MAP_FILE_EXTENSION = ".rtm"

# --- stato del robot (docs/app.md, sezione 4) ---
ONLINE_TIMEOUT_S = 35                # il robot è online se l'ultimo contatto è più recente di così
STATE_LONG_POLL_S = 25               # attesa massima di GET /api/robot/state prima di rispondere 304
STATE_SAVE_INTERVAL_S = 10           # ogni quanto l'ultimo stato viene scritto su disco
MIN_INTERVAL_MAX_MS = 10000          # valore massimo accettato per min_interval_ms
SESSION_TOUCH_INTERVAL_S = 60        # last_used_at della sessione si riscrive al massimo una volta ogni tanto

# --- comandi (sezione 5) ---
COMMAND_TIMEOUT_S = 3                # attesa massima dell'esito di un comando: poi 503 o 504
DEVICE_LONG_POLL_S = 25              # attesa massima di GET /api/device/commands

# --- telecomando (sezione 6) ---
REMOTE_IDLE_TIMEOUT_S = 30           # il telecomando si libera dopo questo tempo senza comandi del telecomando

# --- accesso (sezione 8.4) ---
LOGIN_MAX_FAILED = 5                 # tentativi sbagliati di fila prima del blocco
LOGIN_LOCK_S = 60                    # durata del blocco
JWT_ALGORITHM = "HS256"
JWT_LIFETIME_S = 30 * 24 * 3600      # durata di un JWT e della sessione dopo l'ultimo rinnovo
JWT_KEY_BYTES = 32
PASSWORD_SALT_BYTES = 16
PASSWORD_MIN_LENGTH = 8
SCRYPT_N = 2 ** 14                   # parametri di hashlib.scrypt
SCRYPT_R = 8
SCRYPT_P = 1
SCRYPT_KEY_BYTES = 32
ROBOT_TOKEN_BYTES = 32
ROBOT_TOKEN_CONFIRM_WORD = "ToKeN"   # parola da scrivere per rigenerare il token del robot
SESSION_COOKIE_NAME = "maurice_session"
SESSION_COOKIE_PATH = "/api"

# --- registro dei comandi (sezione 8.3) ---
COMMAND_LOG_KEEP_DAYS = 30
COMMAND_LOG_PAGE_DEFAULT = 50
COMMAND_LOG_PAGE_MAX = 200
# tipi di comando che non si scrivono mai nel registro (deciso con l'utente il 2026-11-10: una sessione di telecomando
# ne manderebbe una decina al secondo)
COMMAND_LOG_SKIPPED_TYPES = ("remote_keepalive",)
# tipi di comando che si scrivono solo quando i parametri cambiano rispetto all'ultimo dello stesso tipo
COMMAND_LOG_ONLY_CHANGES_TYPES = ("remote_move",)

# --- programmazione delle pulizie (sezione 8.7) ---
SCHEDULE_TIMEZONE = "Europe/Rome"
SCHEDULE_CHECK_INTERVAL_S = 60       # il thread controlla una volta al minuto
SCHEDULE_MAX_RETRIES = 10            # nuovi tentativi dopo il primo fallito
SCHEDULE_RETRY_INTERVAL_S = 60       # uno al minuto
SCHEDULE_RESULT_OK = "ok"
SCHEDULE_RESULT_RETRYING = "retrying"

# --- mappe (sezione 7 e docs/formato-mappa.md) ---
FINE_MAP_CELL_NUMBER = 1024          # celle per lato della griglia fine
FINE_MAP_CELL_SIZE = 0.05            # lato di una cella fine, in metri
FINE_MAP_ORIGIN = FINE_MAP_CELL_NUMBER // 2
MAX_USER_ZONES = 15                  # zone proibite dell'utente in una mappa (la sedicesima è quella della base)
MAP_PATCH_HISTORY = 240              # differenze della mappa dal vivo tenute in memoria (2 minuti a una ogni 0,5 s)
MAP_UPLOAD_MAX_BYTES = 2 * 1024 * 1024
MAP_NAME_MAX_LENGTH = 40
MAP_DEFAULT_NAME_PREFIX = "map-"

# --- scia (sezione 7.4) ---
TRAIL_MIN_STEP_M = 0.03              # un punto nuovo solo se il robot si è spostato almeno di così
TRAIL_MAX_POINTS = 100000

# --- risposte ---
GZIP_MIN_BYTES = 256                 # le risposte JSON più corte non si comprimono


def load_env_file():
    """Legge backend/.env (se c'è) senza sovrascrivere le variabili già impostate."""
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), ENV_FILE_NAME)
    if not os.path.exists(path):
        return
    with open(path, encoding="utf-8") as env_file:
        for line in env_file:
            line = line.strip()
            if not line or line.startswith("#") or "=" not in line:
                continue
            name, value = line.split("=", 1)
            os.environ.setdefault(name.strip(), value.strip())


def data_dir():
    """Cartella dei dati, dalla variabile d'ambiente. Se manca il backend non parte."""
    path = os.environ.get(DATA_DIR_ENV)
    if not path:
        raise RuntimeError(f"manca la variabile d'ambiente {DATA_DIR_ENV} (vedi config.example.env)")
    return os.path.abspath(path)


def allowed_origins():
    """Origini del web permesse oltre a quella del sito (serve solo in sviluppo)."""
    value = os.environ.get(ALLOWED_ORIGINS_ENV, "")
    return [origin.strip().rstrip("/") for origin in value.split(",") if origin.strip()]
