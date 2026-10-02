#!/bin/bash
# Misura lunga di latenza sul robot (docs/fase0.md). Di default è il caso peggiore:
#   - task di misura SCHED_FIFO sul core 3 (latenza -c 3, periodo 1 ms);
#   - CARICO=1: carico al 100% sui core 0, 1 e 2 (bench/carico);
#   - PING=1: Wi-Fi saturo con 4 ping dal PC (pacchetti da 8000 byte ogni 2 ms): sul
#     robot lavora solo il kernel (driver, risposta ICMP), niente processi utente.
# ATTENZIONE: con la batteria sotto la soglia di protezione il robot si spegne o si
# riavvia (EXCEPTION_BATTERY_PROTECTION_VOLTAGE nel log di RobotApp): la batteria va
# caricata prima, con lo stack originale sulla base.
#
# Sul robot latenza e carico girano in background (start-stop-daemon), quindi la
# misura continua anche se cade la connessione SSH. Il PC fa da sorvegliante:
# ogni INTERVALLO secondi controlla il robot e scrive una riga di stato, con
# ALLARME quando qualcosa non va (robot irraggiungibile, riavvio, programma
# terminato). Se AuxCtrl si ferma lo riavvia, per evitare il reset del GD32
# (non quando gira lo stack originale: lì AuxCtrl lo gestisce Monitor).
#
# Lo stato si segue da un altro terminale con:  tail -f build/robot/<NOME>-monitor.log
# Ctrl-C ferma tutto (anche i programmi sul robot).
#
# Uso: tools/misura-lunga.sh                         caso peggiore, 1 ora, NOME=lunga
#      NOME=riposo CARICO=0 PING=0 tools/misura-lunga.sh
#      DURATA=120 tools/misura-lunga.sh               prova breve
set -uo pipefail
DIR="$(cd "$(dirname "$0")/.." && pwd)"
cd "$DIR"
DURATA=${DURATA:-3600}
INTERVALLO=${INTERVALLO:-30}
NOME=${NOME:-lunga}
CARICO=${CARICO:-1}
PING=${PING:-1}
ROBOT="$DIR/../ssh-robot.sh"
ADB_RQ="$DIR/../Script/rq.sh"   # comando breve via USB, per i controlli quando il Wi-Fi non risponde
IP=${ROBOT_IP:-$(cat "$DIR/../.robot_ip")}
R=/tmp/rtes
OUT=build/robot
LOG=$OUT/$NOME-monitor.log
mkdir -p "$OUT"

log() { echo "$(date '+%F %T') $*" | tee -a "$LOG"; }
robot() { timeout 20 "$ROBOT" "$@"; }

make --no-print-directory build/arm/latenza build/arm/carico || exit 1
for p in latenza carico; do
  robot "mkdir -p $R && cat > $R/.$p.tmp && chmod +x $R/.$p.tmp && mv $R/.$p.tmp $R/$p" < "build/arm/$p" \
    || { echo "copia di $p sul robot non riuscita" >&2; exit 1; }
done

ferma_tutto() {
  kill $(jobs -p) 2>/dev/null
  robot "for p in latenza carico; do start-stop-daemon -K -p $R/\$p.pid 2>/dev/null; done"
}
trap 'log "INTERROTTA (Ctrl-C): fermo ping e programmi sul robot"; ferma_tutto; exit 130' INT TERM

stack_originale=$(robot 'pidof RobotApp >/dev/null && echo si || echo no')
: > "$LOG"
log "inizio $NOME: $DURATA s, latenza sul core 3, carico core 0-2: $CARICO, ping saturo: $PING, stack originale: $stack_originale"
avvio_carico=""
[ "$CARICO" = 1 ] && avvio_carico="start-stop-daemon -S -b -m -p $R/carico.pid -x /bin/sh -- -c 'exec $R/carico -c 0,1,2 -d $((DURATA + 30)) >/dev/null 2>&1'; sleep 1;"
robot "cd $R || exit 1; rm -f $NOME.csv $NOME.log; $avvio_carico
  start-stop-daemon -S -b -m -p $R/latenza.pid -x /bin/sh -- -c 'cd $R && exec ./latenza -c 3 -d $DURATA -s $INTERVALLO -o $NOME.csv > $NOME.log 2>&1'
  sleep 2; pidof latenza >/dev/null" \
  || { log "ALLARME: avvio sul robot non riuscito"; ferma_tutto; exit 1; }
if [ "$PING" = 1 ]; then
  for i in 1 2 3 4; do ping -q -i 0.002 -s 8000 -w $((DURATA + 60)) "$IP" >/dev/null 2>&1 & done
fi

uptime_prec=0; irraggiungibile=0; esito=1
inizio=$(date +%s)
while :; do
  sleep "$INTERVALLO"
  s=$(robot 'u=$(cut -d" " -f1 /proc/uptime); u=${u%.*}
    echo "$u|$(pidof latenza)|$(pidof carico)|$(pidof AuxCtrl)|$(cat /sys/class/thermal/thermal_zone0/temp)|$(grep MemFree /proc/meminfo | tr -s " " | cut -d" " -f2)|$(tail -1 '$R/$NOME'.log)"')
  if [ -z "$s" ]; then
    irraggiungibile=$((irraggiungibile + 1))
    # Il Wi-Fi saturo può far fallire un controllo; la USB (adb) dice se il robot è vivo.
    up_usb=$(timeout 20 "$ADB_RQ" 'cut -d" " -f1 /proc/uptime' 2>/dev/null | grep -E '^[0-9]' | cut -d. -f1)
    if [ -n "$up_usb" ] && [ "$up_usb" -lt "$uptime_prec" ]; then
      log "ALLARME: il robot si è RIAVVIATO (uptime via USB $up_usb s): misura persa, Wi-Fi e AuxCtrl da rilanciare (../avvia-robot.sh)"
      break
    fi
    log "ALLARME: robot non raggiungibile via Wi-Fi ($irraggiungibile volte di fila), uptime via USB: ${up_usb:-nessuna risposta}"
    [ $((irraggiungibile * INTERVALLO)) -gt $((DURATA + 600)) ] && break
    continue
  fi
  irraggiungibile=0
  IFS='|' read -r up pid_lat pid_car pid_aux temp memfree ultima <<< "$s"
  [ "$up" -lt "$uptime_prec" ] && { log "ALLARME: il robot si è RIAVVIATO (uptime $up s): misura persa"; break; }
  uptime_prec=$up
  if [ -z "$pid_aux" ] && [ "$stack_originale" = no ]; then
    log "ALLARME: AuxCtrl non gira, lo riavvio"
    robot 'start-stop-daemon -S -b -m -p /tmp/auxctrl.pid -x /usr/sbin/AuxCtrl -- -D'
  fi
  log "uptime ${up}s, ${temp}°C, RAM libera $((memfree / 1024)) MB, latenza:${pid_lat:-NO} carico:${pid_car:-NO} AuxCtrl:${pid_aux:-NO} | $ultima"
  if [ -z "$pid_lat" ]; then
    if [[ "$ultima" == campioni\ salvati* ]]; then esito=0
    else log "ALLARME: latenza è terminata prima della fine"; fi
    break
  fi
  [ $(( $(date +%s) - inizio )) -gt $((DURATA + 300)) ] && { log "ALLARME: latenza non termina"; break; }
done

ferma_tutto
[ $esito -ne 0 ] && { log "misura NON completata"; exit 1; }
robot "cat $R/$NOME.log" > "$OUT/$NOME.log"
# Il CSV si cancella dal robot solo se il download è completo (stesso numero di righe):
# una caduta del Wi-Fi a metà trasferimento non deve far perdere la misura.
attese=$(robot "wc -l < $R/$NOME.csv" | tr -d ' \r')
for tentativo in 1 2 3; do
  timeout 600 "$ROBOT" "cat $R/$NOME.csv" > "$OUT/$NOME.csv"
  [ -n "$attese" ] && [ "$(wc -l < "$OUT/$NOME.csv")" = "$attese" ] && break
  log "download di $NOME.csv incompleto (tentativo $tentativo), riprovo"; sleep 30
done
if [ -n "$attese" ] && [ "$(wc -l < "$OUT/$NOME.csv")" = "$attese" ]; then
  log "scaricati $OUT/$NOME.log e $OUT/$NOME.csv ($((attese - 1)) campioni)"
  robot "rm -f $R/$NOME.csv $R/$NOME.log" && log "file cancellati dal robot"
else
  log "ALLARME: download di $NOME.csv non riuscito, il file resta sul robot in $R"
fi
cat "$OUT/$NOME.log"
python3 tools/analizza-latenza.py -o "$OUT/$NOME.png" "$OUT/$NOME.csv" | tee -a "$LOG"
log "fine"
