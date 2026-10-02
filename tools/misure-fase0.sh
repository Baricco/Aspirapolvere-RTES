#!/bin/bash
# Esegue sul robot le misure di latenza della fase 0 (docs/fase0.md, sezione 5),
# scarica i CSV in build/robot/ e stampa le statistiche.
# Richiede il robot acceso e raggiungibile (../avvia-robot.sh).
# Durata totale: circa 5 minuti con DURATA=60.
#
# La misura 4 (traffico Wi-Fi) si fa a parte: serve generare traffico dal PC
# mentre gira la misura.
set -euo pipefail
DIR="$(cd "$(dirname "$0")/.." && pwd)"
cd "$DIR"
DURATA=${DURATA:-60}
ROBOT="$DIR/../ssh-robot.sh"
R=/tmp/rtes

make --no-print-directory build/arm/latenza build/arm/carico
for p in latenza carico; do
  "$ROBOT" "mkdir -p $R && cat > $R/.$p.tmp && chmod +x $R/.$p.tmp && mv $R/.$p.tmp $R/$p" < "build/arm/$p"
done
echo "== programmi copiati in $R; 4 misure da $DURATA s, durata totale circa $((4 * (DURATA + 10) / 60 + 1)) minuti"

misura() {   # misura <nome> <argomenti-carico-o-vuoto> <argomenti latenza>
  local nome=$1 carico=$2; shift 2
  echo "== $nome: dura circa $DURATA s (l'output arriva alla fine)"
  local pre=""
  [ -n "$carico" ] && pre="./carico $carico -d $((DURATA + 10)) >/dev/null & sleep 1; "
  # Attenzione alla precedenza: in "cd X && A & B" il cd finirebbe dentro il
  # processo in background insieme ad A. Per questo il cd è un comando a sé.
  # Lo stato di uscita di latenza viene salvato prima di "wait", così un errore
  # ferma lo script (set -e) invece di passare inosservato.
  "$ROBOT" "cd $R || exit 1; $pre./latenza -c 3 -d $DURATA -o $nome.csv $*; st=\$?; wait; exit \$st"
}

misura idle   ""
misura altri  "-c 0,1,2"
misura stesso "-c 3"
misura p10ms  "" -p 10000

mkdir -p build/robot
for n in idle altri stesso p10ms; do
  "$ROBOT" "cat $R/$n.csv" > "build/robot/$n.csv"
done
echo "== CSV scaricati in build/robot/"
python3 tools/analizza-latenza.py -o build/robot/fase0.png build/robot/{idle,altri,stesso,p10ms}.csv
