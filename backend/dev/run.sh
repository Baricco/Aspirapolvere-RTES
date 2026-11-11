#!/bin/bash
# Backend di sviluppo sul PC: gunicorn in HTTP su 127.0.0.1 (un processo, 32 thread, come in produzione) e, se c'è il
# certificato di dev/make-cert.sh, la porta HTTPS per il programma del robot (docs/app.md, sezione 11).
# Uso: backend/dev/run.sh [porta HTTP] [porta HTTPS]      (default 5000 e 5443)
# La cartella dei dati viene da backend/.env (vedi config.example.env).
set -e
cd "$(dirname "$0")/.."
HTTP_PORT=${1:-5000}
HTTPS_PORT=${2:-5443}
CERT_DIR=../build/app-dev
if [ -f "$CERT_DIR/localhost-cert.pem" ]; then
    .venv/bin/python dev/tls_proxy.py --cert "$CERT_DIR/localhost-cert.pem" --key "$CERT_DIR/localhost-key.pem" \
        --listen "$HTTPS_PORT" --backend "$HTTP_PORT" &
    trap 'kill $! 2>/dev/null' EXIT
fi
.venv/bin/gunicorn --workers 1 --threads 32 --worker-class gthread --bind "127.0.0.1:$HTTP_PORT" \
    --graceful-timeout 2 "app:create_app()"
