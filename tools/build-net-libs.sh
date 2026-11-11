#!/bin/bash
# Scarica e compila le librerie di rete del programma del robot (docs/app.md, sezione 9.6): mbedTLS (TLS) e libcurl
# (HTTPS), statiche, con il compilatore del robot, in build/arm/netlibs/ (include/ e lib/). Il Makefile collega
# build/arm/vacuum a queste. Sul PC non servono: lì si usa la libcurl del sistema.
#
# Le versioni sono fisse e ogni archivio viene controllato con il suo SHA-256 prima di essere aperto: per cambiare
# versione vanno cambiati qui sotto versione e SHA-256 (quelli pubblicati sui siti ufficiali).
#   mbedTLS: https://github.com/Mbed-TLS/mbedtls/releases (ramo 3.6, a supporto lungo; file mbedtls-X-sha256sum.txt)
#   libcurl: https://curl.se/download.html (lo SHA-256 è in https://curl.se/info)
#
# Uso (dalla cartella del progetto): tools/build-net-libs.sh        (serve la rete solo la prima volta)
set -euo pipefail
cd "$(dirname "$0")/.."

MBEDTLS_VERSION=3.6.7
MBEDTLS_SHA256=a7e8bcbec0e6f761b4af24f25677626b35f762f68eef79c08677a363212d11f6
MBEDTLS_URL=https://github.com/Mbed-TLS/mbedtls/releases/download/mbedtls-$MBEDTLS_VERSION/mbedtls-$MBEDTLS_VERSION.tar.bz2

CURL_VERSION=8.23.0
CURL_SHA256=e3de77cfa6367083c776a26647fc9f633b538c06c3baa73d931a252ac8f72b49
CURL_URL=https://curl.se/download/curl-$CURL_VERSION.tar.gz

TOOLCHAIN=$PWD/toolchain
CROSS_HOST=arm-buildroot-linux-musleabihf
CC_ARM=$TOOLCHAIN/bin/arm-linux-gcc
AR_ARM=$TOOLCHAIN/bin/arm-linux-ar
RANLIB_ARM=$TOOLCHAIN/bin/arm-linux-ranlib
# le stesse opzioni del Makefile per il robot (Cortex-A7 con FPU), ottimizzando per la dimensione
CFLAGS_ARM="-Os -mcpu=cortex-a7 -mfpu=neon-vfpv4 -mfloat-abi=hard -ffunction-sections -fdata-sections"
JOBS=${JOBS:-$(nproc)}      # processi di compilazione in parallelo

OUT=$PWD/build/arm/netlibs
DOWNLOADS=$OUT/downloads
SRC=$OUT/src

[ -x "$CC_ARM" ] || { echo "manca il cross-compilatore: tools/installa-toolchain.sh" >&2; exit 1; }
mkdir -p "$DOWNLOADS" "$OUT/include" "$OUT/lib"

# scarica FILE da URL (se non c'è già) e ne controlla lo SHA-256: se non coincide si ferma senza aprirlo
scarica() {
    local url=$1 sha256=$2 file=$DOWNLOADS/$(basename "$1")
    [ -f "$file" ] || curl -sSfL --proto '=https' -o "$file" "$url"
    echo "$sha256  $file" | sha256sum --check --quiet - || { echo "SHA-256 sbagliato per $file: lo cancello" >&2; rm -f "$file"; exit 1; }
    echo "$file"
}

# ---------------------------------------------------------------------------------------------------- mbedTLS
archivio=$(scarica "$MBEDTLS_URL" "$MBEDTLS_SHA256")
rm -rf "$SRC/mbedtls"; mkdir -p "$SRC/mbedtls"
tar -xjf "$archivio" -C "$SRC/mbedtls" --strip-components=1
(
    cd "$SRC/mbedtls"
    # i due thread di rete usano TLS insieme: servono i mutex di mbedTLS (pthread)
    # (si attivano togliendo il commento nel file di configurazione, che poi viene copiato con le intestazioni)
    for opzione in MBEDTLS_THREADING_C MBEDTLS_THREADING_PTHREAD; do
        sed -i "s|^//#define $opzione\$|#define $opzione|" include/mbedtls/mbedtls_config.h
        grep -q "^#define $opzione\$" include/mbedtls/mbedtls_config.h || { echo "non trovo $opzione in mbedtls_config.h" >&2; exit 1; }
    done
    make -s -j"$JOBS" -C library CC="$CC_ARM" AR="$AR_ARM" CFLAGS="$CFLAGS_ARM" libmbedcrypto.a libmbedx509.a libmbedtls.a
    cp library/libmbedcrypto.a library/libmbedx509.a library/libmbedtls.a "$OUT/lib/"
    rm -rf "$OUT/include/mbedtls" "$OUT/include/psa"
    cp -r include/mbedtls include/psa "$OUT/include/"
)
echo "== mbedTLS $MBEDTLS_VERSION compilata"

# ---------------------------------------------------------------------------------------------------- libcurl
archivio=$(scarica "$CURL_URL" "$CURL_SHA256")
rm -rf "$SRC/curl"; mkdir -p "$SRC/curl"
tar -xzf "$archivio" -C "$SRC/curl" --strip-components=1
(
    cd "$SRC/curl"
    # solo HTTP e HTTPS (il programma poi permette solo HTTPS), TLS con mbedTLS, senza zlib né altre librerie
    ./configure --quiet --host="$CROSS_HOST" --prefix="$OUT" \
        CC="$CC_ARM" AR="$AR_ARM" RANLIB="$RANLIB_ARM" CFLAGS="$CFLAGS_ARM" \
        --enable-static --disable-shared --with-mbedtls="$OUT" \
        --without-zlib --without-brotli --without-zstd --without-libpsl --without-libidn2 --without-nghttp2 \
        --without-libssh2 --without-librtmp --without-ca-bundle --without-ca-path --without-ca-fallback \
        --enable-http --disable-ftp --disable-file --disable-ldap --disable-ldaps --disable-rtsp --disable-dict \
        --disable-telnet --disable-tftp --disable-pop3 --disable-imap --disable-smb --disable-smtp --disable-gopher \
        --disable-mqtt --disable-ipfs --disable-websockets --disable-proxy --disable-manual --disable-docs \
        --disable-alt-svc --disable-hsts --disable-cookies --disable-netrc --disable-ntlm --disable-kerberos-auth \
        --disable-negotiate-auth --disable-aws --disable-doh --disable-mime --disable-form-api --disable-dateparse \
        --disable-unix-sockets --disable-libcurl-option --disable-verbose --disable-progress-meter \
        --disable-bindlocal --disable-headers-api --disable-socketpair
    make -s -j"$JOBS" -C lib
    make -s -C lib install
    make -s -C include install
)
echo "== libcurl $CURL_VERSION compilata"

ls -l "$OUT/lib"/*.a
echo "librerie pronte in $OUT: ora make build/arm/vacuum"
