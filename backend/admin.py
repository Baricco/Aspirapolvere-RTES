#!/usr/bin/env python3
"""Script di amministrazione del backend: un menu a frecce (docs/app.md, sezione 8.8).

Uso, dalla cartella backend/:  .venv/bin/python admin.py
Lavora sul database della cartella dei dati (variabile MAURICE_DATA_DIR, o backend/.env in sviluppo). Il backend può
restare acceso: gli utenti e il token nuovi valgono subito, perché ogni richiesta li rilegge dal database.
"""
import os
import sys

import questionary

import auth
import config
import db
import device_api
import maps

MENU_ADD_USER = "Aggiungi utente"
MENU_CHANGE_PASSWORD = "Cambia password"
MENU_REMOVE_USER = "Togli utente"
MENU_LIST_USERS = "Elenco utenti"
MENU_ROBOT_TOKEN = "Rigenera token del robot"
MENU_IMPORT_MAP = "Importa mappa"
MENU_EXIT = "Esci"


def ask_password():
    """Chiede la password due volte senza mostrarla. Ritorna None se non coincidono o se è troppo corta."""
    password = questionary.password("Password:").ask()
    if password is None:
        return None
    if len(password) < config.PASSWORD_MIN_LENGTH:
        print(f"La password deve avere almeno {config.PASSWORD_MIN_LENGTH} caratteri.")
        return None
    if questionary.password("Ripeti la password:").ask() != password:
        print("Le due password non coincidono.")
        return None
    return password


def choose_user(database, message):
    usernames = [row["username"] for row in auth.list_users(database)]
    if not usernames:
        print("Non ci sono utenti.")
        return None
    return questionary.select(message, choices=usernames).ask()


def add_user(database):
    username = questionary.text("Nome dell'utente:").ask()
    if not username or not username.strip():
        return
    password = ask_password()
    if password is None:
        return
    if auth.add_user(database, username.strip(), password):
        print(f"Utente {username.strip()} aggiunto.")
    else:
        print(f"L'utente {username.strip()} esiste già.")


def change_password(database):
    username = choose_user(database, "Di quale utente?")
    if username is None:
        return
    password = ask_password()
    if password is None:
        return
    auth.change_password(database, username, password)
    print(f"Password di {username} cambiata. Tutte le sue sessioni sono state chiuse: deve rifare il login.")


def remove_user(database):
    username = choose_user(database, "Quale utente togliere?")
    if username is None:
        return
    print(f"ATTENZIONE: l'utente {username} e tutte le sue sessioni vengono tolti. Non si può annullare.")
    if not questionary.confirm(f"Togliere {username}?", default=False).ask():
        print("Non ho tolto niente.")
        return
    auth.remove_user(database, username)
    print(f"Utente {username} tolto.")


def list_users(database):
    users = auth.list_users(database)
    if not users:
        print("Non ci sono utenti.")
    for user in users:
        print(f"  {user['username']}")


def robot_token(database):
    print("ATTENZIONE: il token di adesso smette di valere subito. Il robot non potrà più collegarsi finché il token")
    print("nuovo non viene copiato nel suo file del token.")
    word = questionary.text(f"Per continuare scrivi {config.ROBOT_TOKEN_CONFIRM_WORD} (maiuscole e minuscole contano):").ask()
    if word != config.ROBOT_TOKEN_CONFIRM_WORD:
        print("Non ho cambiato niente.")
        return
    token = device_api.new_robot_token(database)
    print("Token nuovo del robot (lo vedi solo adesso: il server ne salva soltanto lo SHA-256):")
    print()
    print(f"  {token}")
    print()
    print("Va scritto, da solo su una riga, nel file del token del robot.")


def import_map(database):
    path = questionary.path("File della mappa (RTM1):").ask()
    if not path:
        return
    try:
        with open(os.path.expanduser(path), "rb") as map_file:
            data = map_file.read()
        maps.decode_rtm(data)
    except OSError as error:
        print(f"Non riesco a leggere il file: {error}")
        return
    except maps.MapFormatError as error:
        print(f"Non è una mappa valida: {error}")
        return
    store = maps.MapStore(database, config.data_dir())
    name = questionary.text("Nome della mappa (vuoto: nome automatico):").ask()
    if name is None:
        return
    name = name.strip()
    if len(name) > config.MAP_NAME_MAX_LENGTH:
        print(f"Il nome può avere al massimo {config.MAP_NAME_MAX_LENGTH} caratteri.")
        return
    try:
        map_id = store.add(data, name or None)
    except maps.MapNameTaken:
        print(f"Esiste già una mappa che si chiama {name}.")
        return
    print(f"Mappa importata: {store.get(map_id)['name']} (id {map_id}).")


ACTIONS = {
    MENU_ADD_USER: add_user,
    MENU_CHANGE_PASSWORD: change_password,
    MENU_REMOVE_USER: remove_user,
    MENU_LIST_USERS: list_users,
    MENU_ROBOT_TOKEN: robot_token,
    MENU_IMPORT_MAP: import_map,
}


def main():
    config.load_env_file()
    try:
        data_dir = config.data_dir()
    except RuntimeError as error:
        sys.exit(str(error))
    os.makedirs(data_dir, exist_ok=True)
    database = db.Database(os.path.join(data_dir, config.DATABASE_FILE))
    print(f"Cartella dei dati: {data_dir}")
    while True:
        choice = questionary.select("Cosa vuoi fare?", choices=list(ACTIONS) + [MENU_EXIT]).ask()
        if choice is None or choice == MENU_EXIT:
            break
        ACTIONS[choice](database)
        print()
    database.close()


if __name__ == "__main__":
    main()
