"""Accesso dell'app: utenti, password, sessioni con JWT, cookie per il web (docs/app.md, sezione 8.4)."""
import functools
import hashlib
import hmac
import os
import secrets
import time

import jwt
from flask import Blueprint, current_app, g, request

import config
from responses import api_error, api_ok, invalid_parameters

auth_blueprint = Blueprint("auth", __name__, url_prefix="/api/auth")

BEARER_PREFIX = "Bearer "


# ---------------------------------------------------------------------------------------------------- utenti

def hash_password(password, salt):
    return hashlib.scrypt(password.encode("utf-8"), salt=salt, n=config.SCRYPT_N, r=config.SCRYPT_R,
                          p=config.SCRYPT_P, dklen=config.SCRYPT_KEY_BYTES)


def add_user(database, username, password):
    """Crea un utente (lo usa solo lo script di amministrazione). Ritorna False se il nome esiste già."""
    if database.query_one("SELECT 1 FROM users WHERE username = ?", (username,)):
        return False
    salt = secrets.token_bytes(config.PASSWORD_SALT_BYTES)
    database.execute("INSERT INTO users (username, password_hash, password_salt, created_at) VALUES (?, ?, ?, ?)",
                     (username, hash_password(password, salt), salt, int(time.time())))
    return True


def change_password(database, username, password):
    """Cambia la password e chiude tutte le sessioni dell'utente. Ritorna False se l'utente non esiste."""
    user = database.query_one("SELECT id FROM users WHERE username = ?", (username,))
    if user is None:
        return False
    salt = secrets.token_bytes(config.PASSWORD_SALT_BYTES)
    with database.transaction():
        database.execute("UPDATE users SET password_hash = ?, password_salt = ?, failed_logins = 0, locked_until = 0 "
                         "WHERE id = ?", (hash_password(password, salt), salt, user["id"]))
        database.execute("DELETE FROM sessions WHERE user_id = ?", (user["id"],))
    return True


def remove_user(database, username):
    """Toglie l'utente e le sue sessioni. Ritorna False se l'utente non esiste."""
    user = database.query_one("SELECT id FROM users WHERE username = ?", (username,))
    if user is None:
        return False
    with database.transaction():
        database.execute("DELETE FROM sessions WHERE user_id = ?", (user["id"],))
        database.execute("DELETE FROM users WHERE id = ?", (user["id"],))
    return True


def list_users(database):
    return database.query_all("SELECT username, created_at FROM users ORDER BY username")


# ----------------------------------------------------------------------------------------------------- JWT

def load_jwt_key(data_dir):
    """La chiave di firma: 32 byte casuali in un file della cartella dei dati, creato al primo avvio."""
    path = os.path.join(data_dir, config.JWT_KEY_FILE)
    if not os.path.exists(path):
        descriptor = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
        with os.fdopen(descriptor, "wb") as key_file:
            key_file.write(secrets.token_bytes(config.JWT_KEY_BYTES))
    with open(path, "rb") as key_file:
        return key_file.read()


def services():
    return current_app.extensions["maurice"]


def make_token(user_id, session_id, now):
    """JWT per una sessione. Ritorna (token, scadenza)."""
    expires_at = now + config.JWT_LIFETIME_S
    token = jwt.encode({"sub": str(user_id), "jti": str(session_id), "iat": now, "exp": expires_at},
                       services().jwt_key, algorithm=config.JWT_ALGORITHM)
    return token, expires_at


def read_token():
    """Il JWT della richiesta: dall'intestazione Authorization (telefono) o dal cookie (web). Ritorna (token, dal_cookie)."""
    header = request.headers.get("Authorization", "")
    if header.startswith(BEARER_PREFIX):
        return header[len(BEARER_PREFIX):], False
    cookie = request.cookies.get(config.SESSION_COOKIE_NAME)
    if cookie:
        return cookie, True
    return None, False


def check_session():
    """Controlla firma e scadenza del JWT, poi che la sessione esista ancora. Ritorna False se la richiesta non è autenticata."""
    token, from_cookie = read_token()
    if not token:
        return False
    try:
        claims = jwt.decode(token, services().jwt_key, algorithms=[config.JWT_ALGORITHM],
                            options={"require": ["sub", "jti", "exp"]})
        session_id = int(claims["jti"])
    except (jwt.InvalidTokenError, ValueError):
        return False
    database = services().database
    now = int(time.time())
    row = database.query_one(
        "SELECT sessions.id, sessions.user_id, sessions.last_used_at, users.username FROM sessions "
        "JOIN users ON users.id = sessions.user_id WHERE sessions.id = ? AND sessions.expires_at > ?",
        (session_id, now))
    if row is None or str(row["user_id"]) != claims["sub"]:
        return False
    if now - row["last_used_at"] >= config.SESSION_TOUCH_INTERVAL_S:
        database.execute("UPDATE sessions SET last_used_at = ? WHERE id = ?", (now, session_id))
    g.user_id = row["user_id"]
    g.username = row["username"]
    g.session_id = session_id
    g.session_from_cookie = from_cookie
    return True


def unauthenticated():
    return api_error(401, "unauthenticated", "Accesso non valido o sessione scaduta.")


def require_session(view):
    """Decoratore delle richieste dell'app: senza una sessione valida rispondono 401."""
    @functools.wraps(view)
    def wrapper(*args, **kwargs):
        if not check_session():
            return unauthenticated()
        return view(*args, **kwargs)
    return wrapper


def session_response(token, expires_at, use_cookie):
    """Risposta di login e refresh: sul web il JWT va solo nel cookie, sul telefono nel corpo."""
    if not use_cookie:
        return api_ok({"token": token, "expires_at": expires_at})
    response = api_ok({"expires_at": expires_at})
    response.set_cookie(config.SESSION_COOKIE_NAME, token, max_age=config.JWT_LIFETIME_S, httponly=True, secure=True,
                        samesite="Strict", path=config.SESSION_COOKIE_PATH)
    return response


# ----------------------------------------------------------------------------------------------- richieste

@auth_blueprint.post("/login")
def login():
    body = request.get_json(silent=True)
    if not isinstance(body, dict) or not isinstance(body.get("username"), str) or \
            not isinstance(body.get("password"), str):
        return invalid_parameters("Servono utente e password.")
    database = services().database
    now = int(time.time())
    user = database.query_one("SELECT * FROM users WHERE username = ?", (body["username"],))

    if user is not None and user["locked_until"] > now:
        return api_error(429, "login_locked", "Troppi tentativi sbagliati: riprova fra un minuto.",
                         retry_after_s=user["locked_until"] - now)

    # il calcolo si fa anche per un utente che non esiste, così il tempo di risposta non dice se il nome è giusto
    salt = user["password_salt"] if user is not None else bytes(config.PASSWORD_SALT_BYTES)
    password_hash = hash_password(body["password"], salt)
    if user is None or not hmac.compare_digest(password_hash, user["password_hash"]):
        if user is not None:
            failed = user["failed_logins"] + 1
            if failed >= config.LOGIN_MAX_FAILED:
                database.execute("UPDATE users SET failed_logins = 0, locked_until = ? WHERE id = ?",
                                 (now + config.LOGIN_LOCK_S, user["id"]))
            else:
                database.execute("UPDATE users SET failed_logins = ? WHERE id = ?", (failed, user["id"]))
        return api_error(401, "invalid_credentials", "Utente o password sbagliati.")

    with database.transaction():
        database.execute("UPDATE users SET failed_logins = 0, locked_until = 0 WHERE id = ?", (user["id"],))
        cursor = database.execute(
            "INSERT INTO sessions (user_id, created_at, last_used_at, expires_at) VALUES (?, ?, ?, ?)",
            (user["id"], now, now, now + config.JWT_LIFETIME_S))
        session_id = cursor.lastrowid
        database.execute("DELETE FROM sessions WHERE expires_at <= ?", (now,))
    token, expires_at = make_token(user["id"], session_id, now)
    return session_response(token, expires_at, body.get("use_cookie") is True)


@auth_blueprint.post("/refresh")
@require_session
def refresh():
    """JWT nuovo per la stessa sessione: chi usa l'app non deve mai rifare il login."""
    now = int(time.time())
    token, expires_at = make_token(g.user_id, g.session_id, now)
    services().database.execute("UPDATE sessions SET last_used_at = ?, expires_at = ? WHERE id = ?",
                                (now, expires_at, g.session_id))
    return session_response(token, expires_at, g.session_from_cookie)


@auth_blueprint.post("/logout")
@require_session
def logout():
    services().database.execute("DELETE FROM sessions WHERE id = ?", (g.session_id,))
    response = api_ok()
    response.delete_cookie(config.SESSION_COOKIE_NAME, path=config.SESSION_COOKIE_PATH, httponly=True, secure=True,
                           samesite="Strict")
    return response


@auth_blueprint.get("/me")
@require_session
def me():
    return api_ok({"username": g.username})
