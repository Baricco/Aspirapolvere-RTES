"""Configurazione di gunicorn per il backend di Maurice (esempio: si usa al momento del deploy).

Avvio, dalla cartella backend/:  .venv/bin/gunicorn --config deploy/gunicorn.conf.py "app:create_app()"
"""

# Il processo deve essere UNO solo: lo stato vivo (ultimo stato, coda dei comandi, telecomando, mappa dal vivo) sta in
# memoria. Le richieste lunghe dormono in un thread ciascuna: ne servono poche (robot, app aperte, comandi in attesa)
workers = 1
threads = 32
worker_class = "gthread"

# solo sulla macchina: davanti c'è Apache, che fa l'HTTPS e passa /api
bind = "127.0.0.1:8730"

# più lungo delle richieste lunghe (25 s): alla chiusura si lasciano finire
graceful_timeout = 30
keepalive = 75

accesslog = None
errorlog = "-"
