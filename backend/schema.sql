-- Tabelle del backend (docs/app.md, sezione 8.3). Tutti gli istanti sono secondi dal 1970.

CREATE TABLE IF NOT EXISTS users (
    id            INTEGER PRIMARY KEY AUTOINCREMENT,   -- mai riusato: il registro dei comandi lo cita
    username      TEXT    NOT NULL UNIQUE,
    password_hash BLOB    NOT NULL,                     -- hashlib.scrypt
    password_salt BLOB    NOT NULL,
    created_at    INTEGER NOT NULL,
    failed_logins INTEGER NOT NULL DEFAULT 0,           -- tentativi sbagliati di fila
    locked_until  INTEGER NOT NULL DEFAULT 0            -- login bloccato fino a questo istante
);

-- una sessione per ogni login: il suo id è il jti del JWT
CREATE TABLE IF NOT EXISTS sessions (
    id           INTEGER PRIMARY KEY AUTOINCREMENT,
    user_id      INTEGER NOT NULL REFERENCES users(id) ON DELETE CASCADE,
    created_at   INTEGER NOT NULL,
    last_used_at INTEGER NOT NULL,
    expires_at   INTEGER NOT NULL
);

CREATE TABLE IF NOT EXISTS maps (
    id         INTEGER PRIMARY KEY AUTOINCREMENT,       -- mai riusato: è nel nome del file map-<id>.rtm
    name       TEXT    NOT NULL UNIQUE,
    crc32      TEXT    NOT NULL,                        -- CRC-32 in fondo al file, 8 cifre esadecimali
    size_bytes INTEGER NOT NULL,
    created_at INTEGER NOT NULL,
    saved_at   INTEGER NOT NULL
);

-- una riga sola
CREATE TABLE IF NOT EXISTS robot (
    id              INTEGER PRIMARY KEY CHECK (id = 1),
    token_hash      TEXT,                               -- SHA-256 del token, esadecimale; vuoto = nessun token
    selected_map_id INTEGER REFERENCES maps(id)         -- mappa selezionata sul robot; vuoto = nessuna mappa
);
INSERT OR IGNORE INTO robot (id) VALUES (1);

CREATE TABLE IF NOT EXISTS schedules (
    id           INTEGER PRIMARY KEY AUTOINCREMENT,
    days_mask    INTEGER NOT NULL,                      -- 7 bit: bit 0 lunedì ... bit 6 domenica
    time_minutes INTEGER NOT NULL,                      -- minuti dalla mezzanotte, 0-1439
    enabled      INTEGER NOT NULL DEFAULT 0,
    last_run_at  INTEGER,
    last_result  TEXT
);

CREATE TABLE IF NOT EXISTS command_log (
    id      INTEGER PRIMARY KEY AUTOINCREMENT,
    time    INTEGER NOT NULL,
    user_id INTEGER,                                    -- vuoto = programmazione delle pulizie
    type    TEXT    NOT NULL,
    params  TEXT,                                       -- JSON dei parametri, vuoto se non ce ne sono
    status  TEXT    NOT NULL,                           -- ok, rejected, robot_unreachable, no_result
    reason  TEXT
);
CREATE INDEX IF NOT EXISTS command_log_time ON command_log (time);
