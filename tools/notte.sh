#!/bin/bash
# Sequenza di misure per la notte, senza sorveglianza. Parte con il robot sulla base e lo
# stack originale attivo (RobotApp carica la batteria), e con Script/stream-strace.sh in corso.
#   1. "originale": 1 h di latenza mentre gira lo stack originale (e strace): la batteria intanto carica.
#      Poi altri ATTESA secondi di sola carica.
#   2. ferma lo streaming di strace e lo stack originale, avvia solo AuxCtrl (senza riavvio:
#      il Wi-Fi resta configurato).
#   3. misure lunghe nel nostro setup: riposo, wifi, carico, peggiore (carico + Wi-Fi).
#   4. alla fine riavvia lo stack originale, così il robot continua a caricare.
# Se il robot non risponde più (per esempio si è riavviato), si ferma: senza USB non si può
# rimettere in rete. Il riepilogo è in build/robot/notte.log.
#
# Uso: tools/notte.sh        (DURATA per misura, default 3600 s; ATTESA, default 1800 s)
set -uo pipefail
DIR="$(cd "$(dirname "$0")/.." && pwd)"
cd "$DIR"
DURATA=${DURATA:-3600}
ATTESA=${ATTESA:-1800}
ROBOT="$DIR/../ssh-robot.sh"
STREAM_PID="$DIR/../Log/cattura-stream/stream.pid"
LOG=build/robot/notte.log
mkdir -p build/robot

log() { echo "$(date '+%F %T') $*" | tee -a "$LOG"; }
robot() { timeout 30 "$ROBOT" "$@"; }
raggiungibile() {   # fino a ~5 minuti di tentativi: il Wi-Fi a volte cade per qualche decina di secondi
  for i in $(seq 10); do robot true && return 0; sleep 30; done
  return 1
}
misura() {   # misura <nome> <carico 0/1> <ping 0/1>
  log "== misura $1 (carico $2, ping $3, $DURATA s)"
  NOME=$1 CARICO=$2 PING=$3 DURATA=$DURATA tools/misura-lunga.sh > /dev/null 2>&1
  local st=$?
  log "   esito $1: $([ $st -eq 0 ] && echo completata || echo "NON completata (vedi build/robot/$1-monitor.log)")"
  grep -E "latenza +n=" "build/robot/$1-monitor.log" 2>/dev/null | tail -1 | tee -a "$LOG"
  raggiungibile || { log "ALLARME: robot non raggiungibile dopo $1: interrompo la notte"; exit 1; }
}

log "inizio notte: stack originale $(robot 'pidof RobotApp >/dev/null && echo attivo || echo NON attivo')"
raggiungibile || { log "ALLARME: robot non raggiungibile"; exit 1; }

misura originale 0 0
log "solo carica per $ATTESA s"
sleep "$ATTESA"

log "fermo lo streaming di strace e lo stack originale, avvio solo AuxCtrl"
[ -f "$STREAM_PID" ] && kill "$(cat "$STREAM_PID")" 2>/dev/null
robot 'killall strace 2>/dev/null; /etc/init.d/robotManager stop; kill $(pidof Monitor RobotApp log-server everest-server) 2>/dev/null; sleep 2
  kill $(pidof AuxCtrl) 2>/dev/null; sleep 1
  start-stop-daemon -S -b -m -p /tmp/auxctrl.pid -x /usr/sbin/AuxCtrl -- -D; sleep 3
  echo "AuxCtrl=$(pidof AuxCtrl) RobotApp=$(pidof RobotApp) Monitor=$(pidof Monitor)"' | tee -a "$LOG"
raggiungibile || { log "ALLARME: robot non raggiungibile dopo il cambio di stack"; exit 1; }

# Il caso peggiore per ultimo: è il più pesante (CPU e temperatura); se fa cadere il robot,
# le altre misure sono già salve.
misura riposo 0 0
misura wifi 0 1
misura carico 1 0
misura peggiore 1 1

log "riavvio lo stack originale per continuare la carica"
robot 'kill $(pidof AuxCtrl) 2>/dev/null; sleep 1; /etc/init.d/robotManager start; sleep 5; echo "RobotApp=$(pidof RobotApp) AuxCtrl=$(pidof AuxCtrl)"' | tee -a "$LOG"
log "fine notte"
