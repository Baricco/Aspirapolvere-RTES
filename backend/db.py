"""Accesso a SQLite: un file solo in modalità WAL, una connessione per thread."""
import contextlib
import os
import sqlite3
import threading

SCHEMA_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "schema.sql")
BUSY_TIMEOUT_MS = 5000      # attesa massima se un altro thread sta scrivendo


class Database:
    def __init__(self, path):
        self.path = path
        self.local = threading.local()
        self.connections = []               # tutte le connessioni aperte, per chiuderle alla fine
        self.connections_lock = threading.Lock()
        with open(SCHEMA_FILE, encoding="utf-8") as schema_file:
            self.connection().executescript(schema_file.read())

    def connection(self):
        """Connessione del thread corrente, creata al primo uso."""
        connection = getattr(self.local, "connection", None)
        if connection is None:
            # isolation_level None: le transazioni si aprono solo con transaction()
            # check_same_thread False solo perché close() le chiude tutte da un thread solo: ognuna la usa il suo thread
            connection = sqlite3.connect(self.path, isolation_level=None, check_same_thread=False)
            connection.row_factory = sqlite3.Row
            connection.execute("PRAGMA journal_mode = WAL")
            connection.execute("PRAGMA foreign_keys = ON")
            connection.execute(f"PRAGMA busy_timeout = {BUSY_TIMEOUT_MS}")
            self.local.connection = connection
            with self.connections_lock:
                self.connections.append(connection)
        return connection

    def execute(self, sql, params=()):
        return self.connection().execute(sql, params)

    def query_one(self, sql, params=()):
        return self.connection().execute(sql, params).fetchone()

    def query_all(self, sql, params=()):
        return self.connection().execute(sql, params).fetchall()

    @contextlib.contextmanager
    def transaction(self):
        """Più scritture in una sola operazione: o tutte o nessuna."""
        connection = self.connection()
        connection.execute("BEGIN IMMEDIATE")
        try:
            yield connection
        except BaseException:
            connection.execute("ROLLBACK")
            raise
        connection.execute("COMMIT")

    def close(self):
        """Chiude le connessioni di tutti i thread (alla fine del programma o di una prova)."""
        with self.connections_lock:
            for connection in self.connections:
                connection.close()
            self.connections = []
        self.local = threading.local()
