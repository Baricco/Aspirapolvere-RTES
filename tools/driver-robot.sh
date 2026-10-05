#!/bin/bash
# Prove del driver (fase 3) sul robot, dal PC via SSH. Tutto in RAM (/tmp/rtes): un riavvio riporta il robot com'era.
#
# Lo stack originale deve essere FERMO (la prima volta: tools/banco-robot.sh o a mano). Al suo posto, quando non gira
# il driver, gira il CUSTODE: "spia guida" che manda solo keepalive 0x0d e heartbeat 0x06 con i motori a zero, così il
# GD32 non resetta il robot e lo stack originale non riparte (con il suo suono di accensione).
#
#   tools/driver-robot.sh avvia NOME [opzioni di rtes-driver...]
#        ferma il custode e avvia in background "rtes-driver --comandi /tmp/rtes/comandi --csv NOME.csv [opzioni]",
#        uscita in NOME.out. Quando il driver termina, PER QUALSIASI MOTIVO, riparte il custode.
#        Esempio: tools/driver-robot.sh avvia d1 --senza-motori --task-prova
#   tools/driver-robot.sh programma PROG NOME [argomenti...]
#        come avvia, ma con un TUO programma (tasks/PROG.c -> build/arm/PROG): uscita in NOME.out, e quando termina
#        riparte il custode. Il programma deve fermarsi con SIGTERM (tools/driver-robot.sh ferma lo manda).
#   tools/driver-robot.sh comando TESTO   manda un comando al driver (es. "lidar 1", "led 3", "blocca", "fine")
#   tools/driver-robot.sh stato           ultime righe dell'uscita, processi attivi, tensione
#   tools/driver-robot.sh segui           segue l'uscita in diretta (Ctrl-C smette di seguire, la prova continua)
#   tools/driver-robot.sh ferma           ferma il driver ("fine") e aspetta che riparta il custode
#   tools/driver-robot.sh prepara         all'inizio di una sessione (dopo un'accensione): ferma lo stack originale e avvia
#        il custode, in un solo comando
#   tools/driver-robot.sh custode         avvia il custode se non gira niente che tenga vivo il GD32
#   tools/driver-robot.sh scarica NOME    copia NOME.out e NOME.csv in build/robot/fase3/NOME/
set -eu
DIR="$(cd "$(dirname "$0")/.." && pwd)"
robot() { "$DIR/../ssh-robot.sh" "$@"; }
D=/tmp/rtes
FERMA_STACK='/etc/init.d/robotManager stop; kill $(pidof Monitor RobotApp AuxCtrl log-server everest-server) 2>/dev/null; sleep 2; kill -9 $(pidof Monitor RobotApp AuxCtrl log-server everest-server) 2>/dev/null; true'
CUSTODE="start-stop-daemon -S -b -m -p $D/custode.pid -x $D/spia -- guida --comandi $D/comandi"

copia() {   # copia NOME: build/arm/NOME -> /tmp/rtes/NOME (rinomina alla fine: un programma in esecuzione non si rovina)
  make -C "$DIR" --no-print-directory "build/arm/$1" >/dev/null
  robot "mkdir -p $D && cat > $D/.$1 && chmod +x $D/.$1 && mv $D/.$1 $D/$1" < "$DIR/build/arm/$1"
}

avvia() {   # avvia PROGRAMMA NOME argomenti...
  local prog=$1 nome=$2; shift 2
  [[ $nome =~ ^[A-Za-z0-9_-]+$ ]] || { echo "nome non valido: $nome" >&2; exit 2; }
  [[ $prog =~ ^[A-Za-z0-9_-]+$ ]] || { echo "programma non valido: $prog" >&2; exit 2; }
  copia "$prog"
  copia spia
  robot "pidof AuxCtrl everest-server >/dev/null && { echo 'lo stack originale è acceso: prima fermalo'; exit 1; }
      [ -f $D/driver.pid ] && kill -0 \$(cat $D/driver.pid) 2>/dev/null && { echo 'una prova gira già: prima tools/driver-robot.sh ferma'; exit 1; }
      start-stop-daemon -K -q -p $D/custode.pid; i=0
      while pidof spia >/dev/null && [ \$i -lt 5 ]; do sleep 1; i=\$((i+1)); done
      pidof spia >/dev/null && { echo 'il custode non si ferma'; exit 1; }
      cd $D; rm -f $nome.out $nome.csv
      start-stop-daemon -S -b -m -p $D/driver.pid -x /bin/sh -- -c '
        cd $D; ./$prog $* > $nome.out 2>&1
        echo \"uscita \$?\" >> $nome.out
        $CUSTODE'
      sleep 2; tail -n 8 $nome.out"
}

case "${1:-}" in
  avvia)
    shift
    nome=${1:?"serve un nome, es. tools/driver-robot.sh avvia d1 --senza-motori"}; shift
    avvia rtes-driver "$nome" --comandi $D/comandi --csv "$nome.csv" "$@" ;;
  programma)
    shift
    prog=${1:?"serve il programma, es. tools/driver-robot.sh programma pulizia p1"}; nome=${2:?"serve un nome"}; shift 2
    avvia "$prog" "$nome" "$@" ;;
  comando)
    shift
    robot "pidof rtes-driver >/dev/null || { echo 'rtes-driver non gira'; exit 1; }; echo '$*' > $D/comandi; sleep 2;
      tail -n 3 \$(ls -t $D/*.out | head -1)" ;;
  stato)
    robot "f=\$(ls -t $D/*.out 2>/dev/null | head -1); echo \"== \$f\"; tail -n 6 \"\$f\" 2>/dev/null
      echo \"== rtes-driver=\$(pidof rtes-driver) custode=\$(pidof spia) stack=\$(pidof AuxCtrl everest-server) uptime=\$(cut -d' ' -f1 /proc/uptime) s\"" ;;
  segui)
    robot "tail -n 20 -f \$(ls -t $D/*.out | head -1)" ;;
  ferma)
    robot "pidof spia >/dev/null && { echo 'gira solo il custode'; exit 0; }
      if pidof rtes-driver >/dev/null; then echo fine > $D/comandi; else
        for p in \$(pgrep -P \$(cat $D/driver.pid 2>/dev/null) 2>/dev/null); do kill \$p; done; fi; i=0
      while ! pidof spia >/dev/null && [ \$i -lt 10 ]; do sleep 1; i=\$((i+1)); done
      tail -n 4 \$(ls -t $D/*.out | head -1); echo \"custode: \$(pidof spia)\"" ;;
  prepara)
    copia spia
    robot "pidof rtes-driver spia >/dev/null && { echo 'gira già il driver o il custode'; exit 0; }
      $FERMA_STACK
      pidof AuxCtrl everest-server >/dev/null && { echo 'lo stack originale non si ferma'; exit 1; }
      $CUSTODE; sleep 1; echo \"stack fermo, custode: \$(pidof spia)\"" ;;
  custode)
    robot "if pidof rtes-driver spia AuxCtrl >/dev/null; then echo 'il GD32 è già sorvegliato'; else $CUSTODE; sleep 1; echo \"custode: \$(pidof spia)\"; fi" ;;
  scarica)
    nome=${2:?"serve il nome della prova"}
    dest="$DIR/build/robot/fase3/$nome"
    mkdir -p "$dest"
    for f in out csv reg; do robot "cat $D/$nome.$f 2>/dev/null" > "$dest/$nome.$f" || true; done
    [ -s "$dest/$nome.reg" ] || rm -f "$dest/$nome.reg"
    ls -la "$dest" ;;
  *) sed -n '2,20p' "$0"; exit 2 ;;
esac
