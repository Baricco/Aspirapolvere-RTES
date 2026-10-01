#!/bin/bash
# Punto di ingresso unico del progetto: ./rtes.sh <comando>
# I comandi non ancora implementati lo dicono ed escono con errore.
set -eu
DIR="$(cd "$(dirname "$0")" && pwd)"

uso() {
  cat <<EOF
Uso: ./rtes.sh <comando>
  test       compila per il PC ed esegue gli unit test
  sim        esegue i task nel simulatore e salva la traccia
  build      compila per il robot (ARM, statico)
  deploy     copia il binario in /tmp sul robot e lo avvia (solo RAM)
  install    installa la versione persistente con avvio sicuro
  uninstall  rimuove la versione persistente
  ssh        apre una shell sul robot
EOF
}

da_fare() { echo "'$1' non ancora implementato (vedi docs/PIANO.md, $2)." >&2; exit 2; }

case "${1:-}" in
  test)      da_fare test "fase 0" ;;
  sim)       da_fare sim "fase 0" ;;
  build)     da_fare build "fase 0" ;;
  deploy)    da_fare deploy "fase 0" ;;
  install)   da_fare install "fase 5" ;;
  uninstall) da_fare uninstall "fase 5" ;;
  ssh)       shift; exec "$DIR/../ssh-robot.sh" "$@" ;;
  ""|-h|--help|help) uso ;;
  *) echo "Comando sconosciuto: $1" >&2; uso; exit 1 ;;
esac
