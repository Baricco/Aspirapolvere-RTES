#!/usr/bin/env python3
"""leggi-mappa.py - Controlla un file di mappa (.rtm, formato in docs/formato-mappa.md) e lo disegna.

Uso: python3 tools/leggi-mappa.py FILE.rtm [disegno.png]

Fa gli stessi controlli della mappatura al caricamento, stampa rettangolo, zone, quante celle per categoria e quante
sequenze, e se si dà un nome scrive un PNG (1 pixel per cella fine, y verso l'alto): sconosciuta grigia, libera
bianca, occupata nera, vuoto rosso, zone proibite bordate di blu. Solo libreria standard.
"""
import struct
import sys
import zlib

LATO = 1024
MAX_ZONE = 15
COLORI = {0: (128, 128, 128), 1: (255, 255, 255), 2: (0, 0, 0), 3: (220, 40, 40)}
NOMI = {0: 'sconosciute', 1: 'libere', 2: 'occupate', 3: 'vuoto'}


def errore(testo):
    print(f'MAPPA NON VALIDA: {testo}')
    sys.exit(1)


def leggi(percorso):
    d = open(percorso, 'rb').read()
    if len(d) < 17 + 4:
        errore(f'file troppo corto ({len(d)} byte)')
    if d[:4] != b'RTM1':
        errore(f'numero magico {d[:4]!r} invece di RTM1')
    crc_file = struct.unpack_from('<I', d, len(d) - 4)[0]
    if zlib.crc32(d[:-4]) & 0xFFFFFFFF != crc_file:
        errore('CRC-32 sbagliato')
    x0, y0, w, h = struct.unpack_from('<HHHH', d, 4)
    if w == 0 or h == 0 or x0 + w > LATO or y0 + h > LATO:
        errore(f'rettangolo fuori dalla griglia: x {x0} y {y0} larghezza {w} altezza {h}')
    nz = d[12]
    if nz > MAX_ZONE:
        errore(f'{nz} zone, al massimo {MAX_ZONE}')
    i = 13
    zone = []
    for _ in range(nz):
        zone.append(struct.unpack_from('<HHHH', d, i))
        i += 8
    celle = []
    sequenze = 0
    totale = w * h
    while len(celle) < totale:
        if i >= len(d) - 4:
            errore('le sequenze finiscono prima delle celle')
        b = d[i]
        i += 1
        n = b & 0x3F
        if n == 0 or n > totale - len(celle):
            errore(f'sequenza con conteggio {n} al byte {i - 1}')
        celle += [b >> 6] * n
        sequenze += 1
    if i != len(d) - 4:
        errore(f'{len(d) - 4 - i} byte in più fra le celle e il CRC')
    return x0, y0, w, h, zone, celle, sequenze, len(d)


def png(percorso, w, h, pixel):
    righe = b''
    for y in range(h - 1, -1, -1):            # y verso l'alto
        righe += b'\x00' + bytes(c for p in pixel[y * w:(y + 1) * w] for c in p)
    def blocco(tipo, dati):
        return struct.pack('>I', len(dati)) + tipo + dati + struct.pack('>I', zlib.crc32(tipo + dati) & 0xFFFFFFFF)
    with open(percorso, 'wb') as f:
        f.write(b'\x89PNG\r\n\x1a\n' + blocco(b'IHDR', struct.pack('>IIBBBBB', w, h, 8, 2, 0, 0, 0)) +
                blocco(b'IDAT', zlib.compress(righe, 9)) + blocco(b'IEND', b''))


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(2)
    x0, y0, w, h, zone, celle, sequenze, dim = leggi(sys.argv[1])
    print(f'mappa valida: {dim} byte, rettangolo da ({x0}, {y0}) di {w} x {h} celle fini '
          f'({w * 0.05:.2f} x {h * 0.05:.2f} m), {sequenze} sequenze')
    conta = {c: celle.count(c) for c in range(4)}
    print('celle: ' + ', '.join(f'{NOMI[c]} {conta[c]}' for c in range(4)))
    for k, (zx, zy, zw, zh) in enumerate(zone):
        print(f'zona {k}: da ({zx}, {zy}) celle fini, {zw} x {zh}')
    if len(sys.argv) > 2:
        pixel = [COLORI[c] for c in celle]
        for zx, zy, zw, zh in zone:
            for x in range(zx, zx + zw):
                for y in range(zy, zy + zh):
                    if x in (zx, zx + zw - 1) or y in (zy, zy + zh - 1):
                        if x0 <= x < x0 + w and y0 <= y < y0 + h:
                            pixel[(y - y0) * w + (x - x0)] = (40, 80, 230)
        png(sys.argv[2], w, h, pixel)
        print(f'disegno in {sys.argv[2]}')


main()
