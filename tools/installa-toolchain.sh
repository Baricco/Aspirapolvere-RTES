#!/bin/bash
# Scarica e installa in toolchain/ il cross-compilatore per il robot:
# Bootlin armv7-eabihf, musl, "stable" 2026.08-1 (gcc 15.3, musl 1.2.6).
# Serve una volta per ogni computer su cui si clona la repository.
# Il file scaricato viene verificato con il checksum SHA-256 pubblicato da Bootlin.
set -euo pipefail
DIR="$(cd "$(dirname "$0")/.." && pwd)"
NOME=armv7-eabihf--musl--stable-2026.08-1
URL=https://toolchains.bootlin.com/downloads/releases/toolchains/armv7-eabihf/tarballs
DEST="$DIR/toolchain"

if [ -x "$DEST/bin/arm-linux-gcc" ]; then
  echo "Toolchain già presente in $DEST"
  exit 0
fi

TMP=$(mktemp -d)
trap 'rm -rf "$TMP"' EXIT
cd "$TMP"
echo "Scarico $NOME (circa 100 MB)..."
curl -sSfO "$URL/$NOME.tar.xz"
curl -sSfO "$URL/$NOME.sha256"
sha256sum -c "$NOME.sha256"
tar xf "$NOME.tar.xz"
mv "$NOME" "$DEST"
# La toolchain contiene percorsi assoluti: dopo lo spostamento vanno aggiornati.
(cd "$DEST" && ./relocate-sdk.sh >/dev/null 2>&1 || true)
"$DEST/bin/arm-linux-gcc" --version | head -1
echo "Toolchain installata in $DEST"
