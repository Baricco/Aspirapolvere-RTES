#!/bin/bash
# Certificato autofirmato per localhost, per provare il programma del robot (che accetta solo HTTPS) contro il backend
# sul PC (docs/app.md, sezione 11). Il certificato e la chiave finiscono nella cartella indicata (default: build/app-dev,
# che git ignora); al programma del robot il certificato si passa con RTES_SERVER_CA.
# Uso: backend/dev/make-cert.sh [cartella]
set -e
DIR=${1:-"$(cd "$(dirname "$0")/../.." && pwd)/build/app-dev"}
DAYS=825        # validità del certificato, in giorni
mkdir -p "$DIR"
openssl req -x509 -newkey ec -pkeyopt ec_paramgen_curve:prime256v1 -nodes -days $DAYS \
    -keyout "$DIR/localhost-key.pem" -out "$DIR/localhost-cert.pem" \
    -subj "/CN=localhost" -addext "subjectAltName=DNS:localhost,IP:127.0.0.1" 2>/dev/null
chmod 600 "$DIR/localhost-key.pem"
echo "certificato: $DIR/localhost-cert.pem"
echo "chiave:      $DIR/localhost-key.pem"
