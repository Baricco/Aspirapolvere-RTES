#!/bin/bash
# Punto di ingresso unico del progetto: ./rtes.sh <comando>
# I comandi non ancora implementati lo dicono ed escono con errore.
#
# I comandi che parlano con il robot usano ../ssh-robot.sh (fuori dalla
# repository, insieme agli script di accesso: vedi docs/GUIDA.md) e
# richiedono il robot acceso e raggiungibile (../avvia-robot.sh).
set -eu
DIR="$(cd "$(dirname "$0")" && pwd)"
SSH_ROBOT="$DIR/../ssh-robot.sh"
DIR_ROBOT=/tmp/rtes     # cartella sul robot, in RAM: sparisce al riavvio

uso() {
  cat <<EOF
Uso: ./rtes.sh <comando> [argomenti]
  test                      compila per il PC ed esegue gli unit test
  test-arm                  compila per ARM ed esegue i test con qemu (verifica i binari del robot)
  build                     compila tutto per il robot (ARM, statico) in build/arm/
  deploy <prog> [arg...]    copia build/arm/<prog> in $DIR_ROBOT sul robot e lo esegue
  scarica <file> [dest]     copia un file dal robot (es. un CSV) in dest (default build/robot/)
  sim [stanza] [prog arg...] simulatore + pagina (http://localhost:8080) + il driver da tastiera (rtes-driver),
                            oppure il TUO programma: le seriali sono in RTES_GD32 e RTES_LIDAR (drv_config_default)
                            es. ./rtes.sh sim sim/stanze/soggiorno.txt build/host/mio-programma
                            (un build/host/<nome> di tasks/ viene ricompilato prima; il programma parte sotto
                            build/host/osserva, che ne mostra la griglia di navigazione sulla pagina)
  sim-solo [stanza] [opz]   solo il simulatore e la pagina (opzioni in sim/sim.c, es. --porta 8081)
  valida-sim [c2 c2b c3 c1] confronta il simulatore con le registrazioni del robot (build/robot/fase2)
  taratura                  ricalcola sim/parametri.txt dalle registrazioni (tools/taratura/tara.py)
  prova-pagina              prova automatica della pagina del simulatore (serve Brave o Chromium)
  install / uninstall       installa o rimuove la versione persistente con avvio sicuro
  ssh [comando]             apre una shell sul robot (o esegue un comando)
EOF
}

da_fare() { echo "'$1' non ancora implementato (vedi docs/PIANO.md, $2)." >&2; exit 2; }

robot() { "$SSH_ROBOT" "$@"; }

cd "$DIR"
case "${1:-}" in
  test)      make --no-print-directory test ;;
  test-arm)
    command -v qemu-arm-static >/dev/null || { echo "Manca qemu-arm-static: sudo pacman -S qemu-user-static" >&2; exit 1; }
    make --no-print-directory test-arm ;;
  build)     make --no-print-directory arm ;;
  deploy)
    shift
    prog=${1:?"specifica il programma, es. ./rtes.sh deploy latenza -d 10"}; shift
    make --no-print-directory "build/arm/$prog"
    # Il binario viaggia dentro la connessione SSH (cat), senza scp: lo scp di
    # OpenSSH recente usa SFTP, che il dropbear del robot non supporta.
    # Si scrive su un file temporaneo e poi si rinomina: se il programma è in
    # esecuzione, il vecchio eseguibile non viene corrotto.
    robot "mkdir -p $DIR_ROBOT && cat > $DIR_ROBOT/.$prog.tmp && chmod +x $DIR_ROBOT/.$prog.tmp && mv $DIR_ROBOT/.$prog.tmp $DIR_ROBOT/$prog" < "build/arm/$prog"
    echo "== $prog copiato in $DIR_ROBOT, eseguo: $prog $*"
    robot "cd $DIR_ROBOT && ./$prog $(printf '%q ' "$@")" ;;
  scarica)
    remoto=${2:?"specifica il file sul robot, es. ./rtes.sh scarica /tmp/rtes/lat.csv"}
    dest=${3:-build/robot}
    mkdir -p "$dest"
    robot "cat '$remoto'" > "$dest/$(basename "$remoto")"
    echo "salvato $dest/$(basename "$remoto")" ;;
  sim|sim-solo)
    quale=$1; shift
    stanza=sim/stanze/soggiorno.txt
    if [ $# -gt 0 ] && [ -f "$1" ] && [[ $1 == *.txt ]]; then stanza=$1; shift; fi
    make --no-print-directory build/host/sim build/host/rtes-driver build/host/osserva
    # un programma dei task (tasks/<nome>.c o tasks/<nome>/) si ricompila da solo: niente binari vecchi per sbaglio
    if [ $# -gt 0 ] && [[ $1 == build/host/* ]]; then
      nome=${1#build/host/}
      if [ -f "tasks/$nome.c" ] || [ -f "tasks/$nome/main.c" ]; then make --no-print-directory "$1" || exit 1; fi
    fi
    if [ "$quale" = sim-solo ]; then exec build/host/sim --stampa "$@" "$stanza"; fi
    mkdir -p build/sim; rm -f build/sim/mappa_robot
    # setsid: Ctrl+C va solo al programma, che si ferma da solo; il simulatore lo chiude la trap qui sotto, dopo.
    # Senza, il simulatore chiudeva le seriali qualche ms prima e il driver finiva in "GUASTO" all'uscita.
    setsid build/host/sim "$stanza" > build/sim.log 2>&1 &
    pid_sim=$!
    trap 'kill $pid_sim 2>/dev/null; wait $pid_sim 2>/dev/null' EXIT
    for _ in $(seq 50); do [ -e build/sim/lidar ] && break; sleep 0.1; done
    kill -0 $pid_sim 2>/dev/null || { cat build/sim.log; exit 1; }
    echo "== simulatore: $stanza, pagina http://localhost:8080 (registro in build/sim.log)"
    export RTES_GD32=build/sim/gd32 RTES_LIDAR=build/sim/lidar
    # come sul robot: 4 core (0-3), così affinità e priorità dei task si comportano allo stesso modo
    core=(); command -v taskset >/dev/null && core=(taskset -c 0-3)
    if [ $# -gt 0 ]; then
      echo "== avvio $* sui core 0-3 (RTES_GD32=$RTES_GD32, RTES_LIDAR=$RTES_LIDAR)"
      # osserva lancia il programma come suo figlio e ne legge la griglia di navigazione (sim/osserva.c)
      build/host/osserva --uscita build/sim/mappa_robot -- "${core[@]}" "$@"
    else
      echo "== rtes-driver: scrivi i comandi (v 0.1 0, stop, lidar 1, spazzole 0 30, led 3, fine...)"
      "${core[@]}" build/host/rtes-driver --senza-rt
    fi ;;
  valida-sim) shift; make --no-print-directory build/host/sim; exec python3 tools/valida-sim.py "$@" ;;
  taratura)  python3 tools/taratura/tara.py > sim/parametri.txt && echo "scritto sim/parametri.txt" ;;
  prova-pagina) shift; make --no-print-directory build/host/sim; exec node tools/prova-pagina.mjs "$@" ;;
  install)   da_fare install "fase 5" ;;
  uninstall) da_fare uninstall "fase 5" ;;
  ssh)       shift; exec "$SSH_ROBOT" "$@" ;;
  ""|-h|--help|help) uso ;;
  *) echo "Comando sconosciuto: $1" >&2; uso; exit 1 ;;
esac
