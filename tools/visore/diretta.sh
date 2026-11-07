#!/bin/bash
# diretta.sh NOME - Copia in diretta i file della missione NOME dal robot a build/robot/fase3/NOME/, ogni 0,5 s, per il
# visore (tools/visore/, casella "in diretta"). Prende solo la parte nuova di NOME.oss, NOME.map e NOME.out e l'ultima
# griglia (NOME.ultima, riscritta da osserva-robot ogni 0,5 s). Una sola connessione SSH resta aperta (ControlMaster),
# così ogni copia costa pochi millisecondi. Finisce quando il programma sul robot non gira più.
set -u
NOME=${1:?"serve il nome della missione, es. tools/visore/diretta.sh p4"}
DIR="$(cd "$(dirname "$0")/../.." && pwd)"
DEST="$DIR/build/robot/fase3/$NOME"
R=/tmp/rtes
IP=${ROBOT_IP:-$("$DIR/../Script/trova-robot.sh")} || exit 1
CTL="/tmp/visore-ssh-%r@%h:%p"
S=(ssh -i ~/.ssh/id_rsa_robot -o StrictHostKeyChecking=accept-new -o HostKeyAlgorithms=+ssh-rsa
   -o PubkeyAcceptedAlgorithms=+ssh-rsa -o ConnectTimeout=8 -o LogLevel=ERROR
   -o ControlMaster=auto -o ControlPath="$CTL" -o ControlPersist=60 root@"$IP")
mkdir -p "$DEST"
touch "$DEST/$NOME.oss" "$DEST/$NOME.map" "$DEST/$NOME.out"

# prende i byte nuovi di un file del robot (dalla dimensione di quello locale in poi)
nuovi() {
	local f=$1 n
	n=$(stat -c %s "$DEST/$f")
	"${S[@]}" "tail -c +$((n + 1)) $R/$f 2>/dev/null" >> "$DEST/$f"
}

echo "diretta: $NOME da $IP verso $DEST (Ctrl+C per fermare)"
while :; do
	nuovi "$NOME.oss"
	nuovi "$NOME.map"
	nuovi "$NOME.out"
	if "${S[@]}" "cat $R/$NOME.ultima 2>/dev/null" > "$DEST/$NOME.ultima.tmp" && [ -s "$DEST/$NOME.ultima.tmp" ]; then
		mv "$DEST/$NOME.ultima.tmp" "$DEST/$NOME.ultima"
	fi
	"${S[@]}" "pidof vacuum >/dev/null" || break
	sleep 0.5
done
nuovi "$NOME.oss"; nuovi "$NOME.out"
echo "diretta: il programma non gira più, fine"
