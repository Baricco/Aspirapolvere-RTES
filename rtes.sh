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
  sim                       esegue i task nel simulatore
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
  sim)       da_fare sim "simulatore" ;;
  install)   da_fare install "fase 5" ;;
  uninstall) da_fare uninstall "fase 5" ;;
  ssh)       shift; exec "$SSH_ROBOT" "$@" ;;
  ""|-h|--help|help) uso ;;
  *) echo "Comando sconosciuto: $1" >&2; uso; exit 1 ;;
esac
