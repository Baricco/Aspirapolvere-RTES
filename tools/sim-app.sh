#!/bin/bash
# Simulatore con il programma vero del robot (build/host/vacuum) collegato al backend di sviluppo sul PC, per comandare
# il robot dall'app (docs/app.md, sezione 11). Il backend deve girare con la porta HTTPS: backend/dev/run.sh.
#
# Uso, dalla cartella del progetto: tools/sim-app.sh [stanza.txt] [porta HTTPS]
#        (default sim/stanze/vuota.txt e 5443)
#
# Il robot parte agganciato alla base della stanza (la mappa ha l'origine sulla base): la riga "robot" della stanza
# viene sostituita in una copia, build/app-dev/stanza.txt. Token, certificato, mappe e file dei comandi stanno in
# build/app-dev/, come per backend/dev/run.sh. Pagina del simulatore: http://localhost:8080. Ctrl+C ferma tutto.
# Non far girare insieme backend/dev/fake_robot.py: stesso token, due robot per il server.
set -e
cd "$(dirname "$0")/.."
ROOM=${1:-sim/stanze/vuota.txt}
HTTPS_PORT=${2:-5443}
DOCKED_OFFSET_M=0.1745              # distanza fra la base e il centro del robot agganciato (come tools/prove-app/banco.py)
DEV_DIR=build/app-dev
TOKEN_FILE=$DEV_DIR/robot-token
CERT=$DEV_DIR/localhost-cert.pem
MAPS_DIR=$DEV_DIR/robot-maps
COMMANDS_FILE=$DEV_DIR/vacuum-commands
DOCKED_ROOM=$DEV_DIR/stanza.txt

[ -f "$ROOM" ] || { echo "stanza $ROOM non trovata (sono in sim/stanze/)" >&2; exit 1; }
[ -f "$TOKEN_FILE" ] || { echo "manca $TOKEN_FILE: token da backend/admin.py, \"Rigenera token del robot\"" >&2; exit 1; }
[ -f "$CERT" ] || { echo "manca $CERT: crealo con backend/dev/make-cert.sh e riavvia backend/dev/run.sh" >&2; exit 1; }
if ! (exec 3<>"/dev/tcp/127.0.0.1/$HTTPS_PORT") 2>/dev/null; then
    echo "attenzione: nessuno ascolta sulla porta $HTTPS_PORT (backend/dev/run.sh è partito?)." \
         "Il programma parte lo stesso e si collega quando il backend c'è." >&2
fi

# copia della stanza con il robot sulla base, rivolto come la base
awk -v d="$DOCKED_OFFSET_M" '
    /^base / { for (i = 2; i <= NF; i++) { split($i, kv, "="); b[kv[1]] = kv[2] } }
    { lines[NR] = $0 }
    END {
        if (!("x" in b)) { print "nella stanza manca la riga base" > "/dev/stderr"; exit 1 }
        a = b["angolo"] * atan2(0, -1) / 180
        robot = sprintf("robot x=%.4f y=%.4f angolo=%s", b["x"] + d * cos(a), b["y"] + d * sin(a), b["angolo"])
        for (i = 1; i <= NR; i++) print (lines[i] ~ /^robot / ? robot : lines[i])
    }' "$ROOM" > "$DOCKED_ROOM"
mkdir -p "$MAPS_DIR"
echo "== $(grep '^robot ' "$DOCKED_ROOM") (stanza $ROOM), backend https://localhost:$HTTPS_PORT"

export RTES_TOKEN_FILE=$TOKEN_FILE RTES_SERVER_URL=https://localhost:$HTTPS_PORT RTES_SERVER_CA=$CERT \
       RTES_MAPS=$MAPS_DIR RTES_COMMANDS=$COMMANDS_FILE
exec ./rtes.sh sim "$DOCKED_ROOM" build/host/vacuum
