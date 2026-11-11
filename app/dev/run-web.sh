#!/bin/bash
# App web di sviluppo sul PC: Flutter apre l'app in Brave sulla porta 8090, l'unica origine che il backend di sviluppo
# accetta (MAURICE_ALLOWED_ORIGINS in backend/.env). L'indirizzo del backend viene da app/config.json.
# Uso: app/dev/run-web.sh [porta]      (default 8090)
# Mentre gira, nel terminale: R riavvia l'app da capo, q esce.
set -e
cd "$(dirname "$0")/.."
WEB_PORT=${1:-8090}
DEV_ORIGINS_PORT=8090               # porta scritta in MAURICE_ALLOWED_ORIGINS di backend/.env
BROWSER=/usr/bin/brave              # niente Chrome sul PC

if [ ! -f config.json ]; then
    echo "manca app/config.json: copia config.example.json e scrivi l'indirizzo del backend (http://localhost:5000)" >&2
    exit 1
fi
if [ "$WEB_PORT" != "$DEV_ORIGINS_PORT" ]; then
    echo "attenzione: il backend accetta solo http://localhost:$DEV_ORIGINS_PORT; con la porta $WEB_PORT" \
         "va cambiato MAURICE_ALLOWED_ORIGINS in backend/.env (e riavviato il backend)" >&2
fi
export CHROME_EXECUTABLE=${CHROME_EXECUTABLE:-$BROWSER}
exec flutter run -d chrome --web-port "$WEB_PORT" --dart-define-from-file=config.json
