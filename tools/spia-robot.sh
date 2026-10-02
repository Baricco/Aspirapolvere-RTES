#!/bin/bash
# Inserisce e toglie la spia (spia/spia.c) sul robot, dal PC, via SSH. Tutto in RAM: un riavvio del robot
# riporta tutto com'era.
#
#   tools/spia-robot.sh avvia      copia la spia, ferma lo stack originale, avvia "spia proxy" (gd32 + lidar),
#                                  riavvia lo stack originale: da quel momento tutto il traffico passa dalla spia
#   tools/spia-robot.sh stato      la spia gira? quanti byte e frame ha visto (ultime righe del suo stato)
#   tools/spia-robot.sh ferma      ferma lo stack, ferma la spia (che smonta i bind mount), riavvia lo stack
#   tools/spia-robot.sh scarica [dest]   copia il registro sul PC (default build/robot/spia/<data>.spia)
#
# Lo stack originale resta fermo solo per qualche secondo: il GD32 resetta il robot dopo ~4 minuti senza
# heartbeat, quindi non c'è rischio. Se la spia si chiudesse male lasciando /dev/ttyS3 montato, AuxCtrl
# parlerebbe con una pty senza nessuno dall'altra parte: il GD32 resetterebbe il robot e il riavvio
# sistemerebbe tutto.
set -eu
DIR="$(cd "$(dirname "$0")/.." && pwd)"
robot() { "$DIR/../ssh-robot.sh" "$@"; }
D=/tmp/rtes
REG=$D/spia.reg

# Ferma Monitor e tutti i programmi che lancia (procd ferma solo Monitor).
FERMA_STACK='/etc/init.d/robotManager stop; kill $(pidof Monitor RobotApp AuxCtrl log-server everest-server) 2>/dev/null; sleep 2; kill -9 $(pidof Monitor RobotApp AuxCtrl log-server everest-server) 2>/dev/null; true'

case "${1:-}" in
  avvia)
    make -C "$DIR" --no-print-directory build/arm/spia
    robot "mkdir -p $D && cat > $D/.spia && chmod +x $D/.spia && mv $D/.spia $D/spia" < "$DIR/build/arm/spia"
    robot "pidof spia >/dev/null && { echo 'la spia gira già'; exit 1; }
      $FERMA_STACK
      start-stop-daemon -S -b -m -p $D/spia.pid -x /bin/sh -- -c 'exec $D/spia proxy -o $REG -s 10 >$D/spia.out 2>$D/spia.err'
      sleep 1
      cat $D/spia.out
      grep -E 'ttyS[13]' /proc/mounts || { echo 'ERRORE: bind mount assente'; cat $D/spia.err; /etc/init.d/robotManager start; exit 1; }
      /etc/init.d/robotManager start
      sleep 3
      echo \"stack: Monitor=\$(pidof Monitor) AuxCtrl=\$(pidof AuxCtrl) RobotApp=\$(pidof RobotApp)\"" ;;
  stato)
    robot "echo \"spia=\$(pidof spia) AuxCtrl=\$(pidof AuxCtrl) RobotApp=\$(pidof RobotApp)\"; ls -la $REG 2>/dev/null; tail -3 $D/spia.err" ;;
  ferma)
    robot "$FERMA_STACK
      kill \$(cat $D/spia.pid 2>/dev/null) 2>/dev/null; sleep 1
      grep -E 'ttyS[13]' /proc/mounts && echo 'ATTENZIONE: bind mount ancora presente'
      tail -2 $D/spia.err
      /etc/init.d/robotManager start; sleep 3
      echo \"stack: Monitor=\$(pidof Monitor) AuxCtrl=\$(pidof AuxCtrl) RobotApp=\$(pidof RobotApp)\"" ;;
  scarica)
    dest=${2:-$DIR/build/robot/spia/$(date +%Y%m%d-%H%M%S).spia}
    mkdir -p "$(dirname "$dest")"
    robot "cat $REG" > "$dest"
    echo "salvato $dest ($(stat -c %s "$dest") byte)" ;;
  *) sed -n '2,15p' "$0"; exit 2 ;;
esac
