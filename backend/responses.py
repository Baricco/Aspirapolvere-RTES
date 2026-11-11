"""Forma comune delle risposte: errori {"reason", "message"} e JSON."""
from flask import jsonify


def api_error(status, reason, message, **extra):
    """Risposta di errore nella forma unica {"reason": ..., "message": ...}."""
    body = {"reason": reason, "message": message}
    body.update(extra)
    response = jsonify(body)
    response.status_code = status
    return response


def api_ok(body=None, status=200):
    response = jsonify({"ok": True} if body is None else body)
    response.status_code = status
    return response


def invalid_parameters(message="Parametri sbagliati."):
    return api_error(400, "invalid_parameters", message)


def not_found(message="La risorsa non esiste."):
    return api_error(404, "not_found", message)
