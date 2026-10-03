#!/bin/bash
# Banco di prova sul robot (fase 2, blocco C): "spia guida" al posto dello stack originale, dal PC via SSH.
# Tutto in RAM (/tmp/rtes): un riavvio del robot riporta tutto com'era.
#
#   tools/banco-robot.sh avvia NOME [opzioni di spia guida...]
#        ferma lo stack originale (e la spia proxy, se gira) e avvia in background
#        "spia guida -o NOME.reg --csv NOME.csv [opzioni]". Quando spia guida termina, PER QUALSIASI MOTIVO
#        (fine della sequenza, blocco di sicurezza, tensione minima, errore, ferma), lo stack originale riparte
#        da solo: il GD32 non resta mai senza keepalive.
#        Una sequenza (--sequenza) si passa come file del PC: tools/banco-robot.sh avvia NOME --sequenza prova.seq
#   tools/banco-robot.sh lidar NOME      come avvia, ma registra anche la seriale del lidar (/dev/ttyS1) in NOME-lidar.reg
#   tools/banco-robot.sh stato           ultime righe dell'uscita di spia guida
#   tools/banco-robot.sh segui           segue l'uscita in diretta (Ctrl-C per smettere di seguire, la prova continua)
#   tools/banco-robot.sh ferma           termina spia guida (motori a zero) e aspetta che lo stack riparta
#   tools/banco-robot.sh scarica NOME    copia NOME.out, NOME.csv, NOME.reg (e NOME-lidar.reg) in build/robot/fase2/
set -eu
DIR="$(cd "$(dirname "$0")/.." && pwd)"
robot() { "$DIR/../ssh-robot.sh" "$@"; }
D=/tmp/rtes

FERMA_STACK='/etc/init.d/robotManager stop; kill $(pidof Monitor RobotApp AuxCtrl log-server everest-server) 2>/dev/null; sleep 2; kill -9 $(pidof Monitor RobotApp AuxCtrl log-server everest-server) 2>/dev/null; true'

copia_spia() {
  make -C "$DIR" --no-print-directory build/arm/spia >/dev/null
  robot "mkdir -p $D && cat > $D/.spia && chmod +x $D/.spia && mv $D/.spia $D/spia" < "$DIR/build/arm/spia"
}

avvia() {   # avvia LIDAR(0|1) NOME opzioni...
  local lidar=$1 nome=$2; shift 2
  [[ $nome =~ ^[A-Za-z0-9_-]+$ ]] || { echo "nome non valido: $nome" >&2; exit 2; }
  local opz=() a
  while [ $# -gt 0 ]; do
    a=$1; shift
    if [ "$a" = --sequenza ]; then          # il file della sequenza va copiato sul robot
      robot "mkdir -p $D && cat > $D/$nome.seq" < "$1"; shift
      opz+=(--sequenza "$D/$nome.seq")
    else
      opz+=("$a")
    fi
  done
  copia_spia
  robot "pidof spia >/dev/null && { echo 'spia (proxy o guida) gira già: prima tools/spia-robot.sh ferma o banco-robot.sh ferma'; exit 1; }
    $FERMA_STACK
    cd $D; rm -f $nome.out $nome.csv $nome.reg $nome-lidar.reg
    if [ $lidar = 1 ]; then
      start-stop-daemon -S -b -m -p $D/lidar.pid -x $D/spia -- proxy -c lidar,lidar,/dev/ttyS1 --senza-mount -o $D/$nome-lidar.reg -s 3600
    fi
    start-stop-daemon -S -b -m -p $D/banco.pid -x /bin/sh -- -c '
      cd $D; ./spia guida -o $nome.reg --csv $nome.csv ${opz[*]} > $nome.out 2>&1
      echo \"uscita \$?\" >> $nome.out
      [ -f lidar.pid ] && kill \$(cat lidar.pid) 2>/dev/null; rm -f lidar.pid
      /etc/init.d/robotManager start'
    sleep 2; tail -5 $nome.out"
}

case "${1:-}" in
  avvia) shift; [ $# -ge 1 ] || { sed -n '2,20p' "$0"; exit 2; }; avvia 0 "$@" ;;
  lidar) shift; [ $# -ge 1 ] || { sed -n '2,20p' "$0"; exit 2; }; avvia 1 "$@" ;;
  stato) robot "f=\$(ls -t $D/*.out 2>/dev/null | head -1); echo \"\$f\"; tail -n 15 \"\$f\"; echo \"spia=\$(pidof spia) AuxCtrl=\$(pidof AuxCtrl)\"" ;;
  segui) robot "tail -n 20 -f \$(ls -t $D/*.out | head -1)" ;;
  ferma)
    robot "kill \$(pidof spia) 2>/dev/null; for i in 1 2 3 4 5 6 7 8 9 10; do pidof AuxCtrl >/dev/null && break; sleep 1; done
      sleep 2; tail -3 \$(ls -t $D/*.out | head -1); echo \"stack: Monitor=\$(pidof Monitor) AuxCtrl=\$(pidof AuxCtrl)\"" ;;
  scarica)
    nome=${2:?nome}; dest="$DIR/build/robot/fase2/$nome"; mkdir -p "$dest"
    for f in out csv reg; do robot "cat $D/$nome.$f 2>/dev/null" > "$dest/$nome.$f" || true; done
    robot "[ -f $D/$nome-lidar.reg ] && cat $D/$nome-lidar.reg" > "$dest/$nome-lidar.reg" 2>/dev/null || rm -f "$dest/$nome-lidar.reg"
    ls -la "$dest" ;;
  *) sed -n '2,20p' "$0"; exit 2 ;;
esac
