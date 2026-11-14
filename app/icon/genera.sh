#!/bin/bash
# Rigenera le icone dell'app dai quattro SVG di questa cartella (serve rsvg-convert). Dalla cartella app/: icon/genera.sh
set -e
cd "$(dirname "$0")/.."
res=android/app/src/main/res
r() { rsvg-convert -w "$2" -h "$2" "icon/$1" -o "$3"; }
for d in mdpi:48:108 hdpi:72:162 xhdpi:96:216 xxhdpi:144:324 xxxhdpi:192:432; do
    IFS=: read -r nome classica adattiva <<< "$d"
    r icona.svg "$classica" "$res/mipmap-$nome/ic_launcher.png"
    r icona-primo-piano.svg "$adattiva" "$res/mipmap-$nome/ic_launcher_foreground.png"
    r icona-sfondo.svg "$adattiva" "$res/mipmap-$nome/ic_launcher_background.png"
done
r icona.svg 192 web/icons/Icon-192.png
r icona.svg 512 web/icons/Icon-512.png
r icona-maskable.svg 192 web/icons/Icon-maskable-192.png
r icona-maskable.svg 512 web/icons/Icon-maskable-512.png
r icona.svg 64 web/favicon.png
# icona adattiva (Android 8 e seguenti): sfondo e disegno separati, la forma la sceglie il telefono
mkdir -p "$res/mipmap-anydpi-v26"
cat > "$res/mipmap-anydpi-v26/ic_launcher.xml" <<XML
<?xml version="1.0" encoding="utf-8"?>
<adaptive-icon xmlns:android="http://schemas.android.com/apk/res/android">
    <background android:drawable="@mipmap/ic_launcher_background"/>
    <foreground android:drawable="@mipmap/ic_launcher_foreground"/>
</adaptive-icon>
XML
echo "icone rigenerate"
