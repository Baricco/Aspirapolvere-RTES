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
#        riparte il custode. Il programma deve fermarsi con SIGTERM (tools/driver-robot.sh ferma lo manda). Se esce con il
#        codice 3 (chiede il riavvio: cambio di mappa dall'app, docs/app.md sezione 7.5) viene rilanciato subito.
#   tools/driver-robot.sh missione NOME [VAR=valore...]
#        la missione vera: tasks/vacuum come è, con i byte grezzi delle seriali in NOME.reg e, da fuori, stato della
#        navigazione e mappa in NOME.oss e NOME.map (tools/osserva-robot.c). IL ROBOT PARTE SUBITO: deve stare sulla base.
#        Esempio, per provare il rientro: tools/driver-robot.sh missione r1 RTES_PROVA_RIENTRO_DOPO_S=45
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
  local bin="$DIR/build/arm/$1"
  if [ "$1" = vacuum ] && [ -n "${VACUUM_BIN:-}" ]; then bin=$VACUUM_BIN; else make -C "$DIR" --no-print-directory "build/arm/$1" >/dev/null; fi
  robot "mkdir -p $D && cat > $D/.$1 && chmod +x $D/.$1 && mv $D/.$1 $D/$1" < "$bin"
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
      cd $D; rm -f $nome.out $nome.csv $nome.reg $nome.oss $nome.map
      start-stop-daemon -S -b -m -p $D/driver.pid -x /bin/sh -- -c '
        cd $D; codice=3
        while [ \$codice = 3 ]; do ${AMBIENTE:-} ./$prog $* >> $nome.out 2>&1; codice=\$?; done
        echo \"uscita \$codice\" >> $nome.out
        $CUSTODE'
      sleep 2; tail -n 8 $nome.out"
}

case "${1:-}" in
  avvia)
    shift
    nome=${1:?"serve un nome, es. tools/driver-robot.sh avvia d1 --senza-motori"}; shift
    avvia rtes-driver "$nome" --comandi $D/comandi --csv "$nome.csv" "$@" ;;   # AMBIENTE="VAR=valore" passa variabili al programma
  programma)
    shift
    prog=${1:?"serve il programma, es. tools/driver-robot.sh programma pulizia p1"}; nome=${2:?"serve un nome"}; shift 2
    avvia "$prog" "$nome" "$@" ;;
  missione)
    nome=${2:?"serve un nome, es. tools/driver-robot.sh missione m1"}
    make -C "$DIR" --no-print-directory build/arm/vacuum build/arm/osserva-robot >/dev/null
    B="$DIR/toolchain/bin/arm-linux" EXE="${VACUUM_BIN:-$DIR/build/arm/vacuum}"   # VACUUM_BIN: un eseguibile di prova al posto di quello dei task
    simbolo() { "$B-nm" "$EXE" | awk -v s="$1" '$3 == s { print $1 }'; }
    campo() {   # posizione di un campo di nav_state, dalle informazioni di debug
      LC_ALL=C "$B-readelf" --debug-dump=info "$EXE" | awk -v s="$1" '
        /DW_AT_name/ { n = $NF; next }
        /DW_AT_data_member_location/ && n == s { v = $NF } END { print v }'
    }
    campo_prima() {   # l'ultimo campo $1 visto prima del campo $2: "state" c'è anche in altre strutture (la mappatura)
      LC_ALL=C "$B-readelf" --debug-dump=info "$EXE" | awk -v s="$1" -v t="$2" '
        /DW_AT_name/ { n = $NF; next }
        /DW_AT_data_member_location/ { if (n == s) v = $NF; else if (n == t && !fatto) { print v; fatto = 1 } }'
    }
    nav=$((16#$(simbolo nav_state)))
    ind() { printf %x $((nav + $(campo "$1"))); }
    ind_stato=$(printf %x $((nav + $(campo_prima state mission))))
    arg="$(simbolo navigation_map_data) $ind_stato $(ind mission) $(ind clean_cell_mask) $(ind strip_axis_x) $(simbolo pose_exchange) $(simbolo pose_buffers)"
    copia osserva-robot
    AMBIENTE="RTES_REGISTRA=$nome.reg ${*:3}" avvia vacuum "$nome"
    robot "cd $D; start-stop-daemon -S -b -m -p $D/osserva.pid -x $D/osserva-robot -- \$(pidof vacuum) $nome $arg; sleep 1; tail -n 2 $nome.oss"
    ;;
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
    for f in out csv reg oss map; do robot "cat $D/$nome.$f 2>/dev/null" > "$dest/$nome.$f" || true; done
    for f in reg oss map; do [ -s "$dest/$nome.$f" ] || rm -f "$dest/$nome.$f"; done
    ls -la "$dest" ;;
  *) sed -n '2,20p' "$0"; exit 2 ;;
esac
