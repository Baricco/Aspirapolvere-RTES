#!/bin/bash
# Disassembla le funzioni di un binario ARM del robot (AuxCtrl, RobotApp) cercandole per nome.
#
# I binari del robot non hanno le section header, quindi objdump non li legge come ELF.
# Hanno però la tabella dei simboli dinamici con i nomi C++ (nm -D -C). Lo script:
#   1. elenca i simboli in ordine di indirizzo; ogni funzione finisce dove inizia la successiva;
#   2. ricava l'indirizzo di caricamento dal primo segmento LOAD (readelf -l);
#   3. disassembla il file "grezzo" (-b binary) tra inizio e fine della funzione.
# Il codice è in modalità ARM (non Thumb), compilato senza ottimizzazioni: si legge bene.
#
# Uso: tools/disassembla.sh <regex sul nome> [binario]
#      tools/disassembla.sh 'CRobotPacket::calcCheckSum'
#      tools/disassembla.sh 'CSerialMessagePacket::packet' > /tmp/packet.s
# Binario di default: ../Estratto/nandd-rootfs/usr/sbin/AuxCtrl (fuori dalla repository:
# è proprietario e non va messo in git).
set -euo pipefail
DIR="$(cd "$(dirname "$0")/.." && pwd)"
[ $# -ge 1 ] || { sed -n '2,13p' "$0"; exit 2; }
REGEX=$1
BIN=${2:-$DIR/../Estratto/nandd-rootfs/usr/sbin/AuxCtrl}
T="$DIR/toolchain/bin/arm-linux"
[ -x "$T-objdump" ] || { echo "manca la toolchain: tools/installa-toolchain.sh" >&2; exit 1; }

# Primo segmento LOAD: indirizzo virtuale - offset nel file = base da passare a objdump
base=$("$T-readelf" -lW "$BIN" | awk '$1=="LOAD"{printf "0x%x\n", strtonum($3) - strtonum($2); exit}')
simboli=$(mktemp); trap 'rm -f "$simboli"' EXIT
"$T-nm" -D -C -n "$BIN" | awk '$2 ~ /^[TtWw]$/ && $1 != ""' | awk '!visto[$1]++' > "$simboli"

trovati=0
while IFS=: read -r n riga; do
  inizio=0x${riga%% *}
  fine=0x$(sed -n "$((n + 1))p" "$simboli" | cut -d' ' -f1)
  [ "$fine" = 0x ] && fine=$(printf '0x%x' $((inizio + 0x400)))   # ultima funzione: 1 KB
  echo "=== $riga"
  "$T-objdump" -D -b binary -marm --adjust-vma="$base" --start-address="$inizio" --stop-address="$fine" "$BIN" \
    | tail -n +8 | grep -v '^$'
  trovati=$((trovati + 1))
done < <(grep -n -E -- "$REGEX" "$simboli" || true)
[ $trovati -gt 0 ] || { echo "nessun simbolo corrisponde a: $REGEX" >&2; exit 1; }
