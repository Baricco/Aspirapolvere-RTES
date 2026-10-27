#!/bin/bash
# La serie di prove della navigazione: 17 prove su soggiorno (5 semi, 4 partenze), cucina, corridoio e camera.
# Uso (dalla cartella del progetto): tools/prove-nav/serie.sh [file di uscita]   (default build/prove-nav/serie.txt)
cd "$(dirname "$0")/../.."
make --no-print-directory build/host/sim build/host/osserva >/dev/null || exit 1
tools/prove-nav/costruisci.sh || { echo "costruzione fallita" >&2; exit 1; }
OUT=${1:-build/prove-nav/serie.txt}
P() { python3 -I tools/prove-nav/prova.py "$@"; }
{
for s in 1 2 3 4 5; do P sim/stanze/soggiorno.txt $s 120; done
P sim/stanze/soggiorno.txt 7 120 2.0 1.5 0
P sim/stanze/soggiorno.txt 8 120 1.6 0.6 90
P sim/stanze/soggiorno.txt 9 120 3.0 3.0 -45
P sim/stanze/soggiorno.txt 10 120 4.6 1.5 180
P sim/stanze/cucina.txt 1 120 3.0 2.2 180
P sim/stanze/cucina.txt 2 120 0.5 2.6 -90
for s in 1 2 3; do P sim/stanze/corridoio.txt $s 150; done
for s in 1 2 3; do P sim/stanze/camera.txt $s 120; done
} > "$OUT" 2>&1
grep -c "ESPLORAZIONE FINITA" "$OUT" | xargs -I{} echo "esplorazioni finite: {} su $(grep -c '^==' "$OUT") (dettagli in $OUT)"
