#!/usr/bin/env python3
"""mappa-in-stanza.py - Costruisce una stanza del simulatore (formato in sim/mondo.h) da una mappa salvata dal robot
(.rtm, formato in docs/formato-mappa.md), per provare il programma in una stanza simile a quella vera.

- Ogni cella occupata della mappa (5 cm) diventa un pezzo di rettangolo alto ALTEZZA_OSTACOLI: il lidar lo vede e il
  paraurti lo tocca. Le celle vicine uguali si uniscono in rettangoli più grandi (prima per righe, poi le righe uguali
  una sopra l'altra), così il file resta modificabile a mano.
- Le celle occupate isolate (meno di MIN_VICINE celle occupate attorno) si scartano: sono rumore del lidar.
- I buchi nei muri (punti che il lidar non ha mai visto) si chiudono: ogni cella sconosciuta accanto a una cella libera
  diventa occupata, così il robot simulato resta dentro la parte di casa che quello vero conosce.
- La base si mette dove il robot si trova all'avvio, cioè nell'origine della mappa (0; 0) con il robot rivolto verso
  +x e i contatti dietro; le celle occupate sopra la base (il lidar vede la base stessa) si tolgono.
- Quello che la mappa non sa (oggetti sotto il piano del lidar, mobili sotto cui il robot passa, tappeti) non c'è:
  si aggiunge a mano nell'editor del simulatore.

Le coordinate della stanza partono da (0; 0) nell'angolo in basso a sinistra: la mappa si sposta di (dx; dy), stampati
nel commento in cima al file.

Uso: python3 tools/mappa-in-stanza.py MAPPA.rtm > sim/stanze/casa.txt
"""
import struct
import sys

LATO = 1024                 # celle per lato della griglia (costante del programma)
CELLA = 0.05                # lato della cella fine in metri
ORIGINE = LATO // 2         # cella dell'origine della mappa (la base)
ALTEZZA_OSTACOLI = 0.5      # metri: quota_a degli ostacoli presi dalla mappa
MIN_VICINE = 2              # celle occupate fra le 8 vicine sotto cui la cella è rumore
MARGINE = 0.30              # metri di stanza attorno alle celle conosciute
ROBOT_RAGGIO = 0.1745       # sim/robot.h
BASE_PROFONDITA = 0.12      # sim/robot.h
BASE_LARGHEZZA = 0.30       # sim/robot.h
BASE_MARGINE = 0.05         # metri attorno alla base in cui le celle occupate si tolgono


def leggi(percorso):
    d = open(percorso, 'rb').read()
    if d[:4] != b'RTM1':
        sys.exit('non è un file RTM1')
    x0, y0, w, h = struct.unpack_from('<HHHH', d, 4)
    i = 13 + 8 * d[12]
    celle = []
    while len(celle) < w * h:
        b = d[i]
        i += 1
        celle += [b >> 6] * (b & 0x3F)
    return x0, y0, w, h, [celle[r * w:(r + 1) * w] for r in range(h)]


def main():
    x0, y0, w, h, g = leggi(sys.argv[1])
    occ = [[g[r][c] == 2 for c in range(w)] for r in range(h)]
    chiuse = 0
    for r in range(h):
        for c in range(w):
            if g[r][c] == 0 and any(0 <= r + a < h and 0 <= c + b < w and g[r + a][c + b] == 1
                                    for a, b in ((1, 0), (-1, 0), (0, 1), (0, -1))):
                occ[r][c] = True
                chiuse += 1

    def vicine(r, c):
        return sum(occ[r + a][c + b] for a in (-1, 0, 1) for b in (-1, 0, 1)
                   if (a or b) and 0 <= r + a < h and 0 <= c + b < w)

    # centro della cella (r, c) in metri nel riferimento della mappa
    mx = lambda c: (x0 + c - ORIGINE + 0.5) * CELLA
    my = lambda r: (y0 + r - ORIGINE + 0.5) * CELLA
    base_x = -ROBOT_RAGGIO                       # centro del lato con i contatti, robot agganciato in (0; 0)
    tieni = [[False] * w for _ in range(h)]
    scartate = sopra_base = 0
    for r in range(h):
        for c in range(w):
            if not occ[r][c]:
                continue
            if (base_x - BASE_PROFONDITA - BASE_MARGINE <= mx(c) <= base_x + BASE_MARGINE
                    and abs(my(r)) <= BASE_LARGHEZZA / 2 + BASE_MARGINE):
                sopra_base += 1
            elif vicine(r, c) < MIN_VICINE:
                scartate += 1
            else:
                tieni[r][c] = True

    # rettangoli: sequenze per riga, poi unione delle sequenze uguali nelle righe successive
    aperti, rett = {}, []
    for r in range(h + 1):
        seq, c = set(), 0
        while r < h and c < w:
            if tieni[r][c]:
                c2 = c
                while c2 + 1 < w and tieni[r][c2 + 1]:
                    c2 += 1
                seq.add((c, c2))
                c = c2 + 1
            else:
                c += 1
        for s in list(aperti):
            if s not in seq:
                rett.append((s[0], s[1], aperti.pop(s), r - 1))
        for s in seq:
            aperti.setdefault(s, r)

    righe_note = [r for r in range(h) if any(g[r][c] != 0 for c in range(w))]
    col_note = [c for c in range(w) if any(g[r][c] != 0 for r in range(h))]
    xmin, xmax = mx(col_note[0]) - CELLA / 2 - MARGINE, mx(col_note[-1]) + CELLA / 2 + MARGINE
    ymin, ymax = my(righe_note[0]) - CELLA / 2 - MARGINE, my(righe_note[-1]) + CELLA / 2 + MARGINE
    dx, dy = -xmin, -ymin
    f = lambda v: f'{v:.3f}'.rstrip('0').rstrip('.')

    print(f'# costruita da {sys.argv[1]} con tools/mappa-in-stanza.py: {len(rett)} rettangoli da {sum(map(sum, tieni))}'
          f' celle occupate ({chiuse} sconosciute chiuse, {scartate} isolate scartate, {sopra_base} sopra la base tolte)')
    print(f'# stanza = mappa spostata di ({f(dx)}; {f(dy)}) m: la base (origine della mappa) è in ({f(dx)}; {f(dy)})')
    print('# mancano gli oggetti sotto il piano del lidar, i mobili sotto cui il robot passa e i tappeti: vanno aggiunti')
    print(f'stanza nome=casa larghezza={f(xmax - xmin)} profondita={f(ymax - ymin)}')
    print(f'robot x={f(dx)} y={f(dy)} angolo=0')
    print(f'base x={f(base_x + dx)} y={f(dy)} angolo=0')
    for k, (c1, c2, r1, r2) in enumerate(rett):
        cx = (mx(c1) + mx(c2)) / 2 + dx
        cy = (my(r1) + my(r2)) / 2 + dy
        print(f'rettangolo nome=m{k} x={f(cx)} y={f(cy)} larghezza={f((c2 - c1 + 1) * CELLA)} '
              f'profondita={f((r2 - r1 + 1) * CELLA)} angolo=0 quota_da=0 quota_a={f(ALTEZZA_OSTACOLI)}')


if __name__ == '__main__':
    main()
