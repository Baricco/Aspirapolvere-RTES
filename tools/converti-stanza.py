#!/usr/bin/env python3
"""converti-stanza.py - Converte una stanza del vecchio formato a griglia (fase 3, prima versione del simulatore) nel
formato a oggetti di sim/mondo.h: le celle uguali vicine diventano rettangoli ('#' muro alto, 'b' ostacolo basso alto
5 cm, 'v' vuoto). Robot e base si copiano (la base vecchia aveva i contatti davanti: controllare).

Uso: tools/converti-stanza.py vecchia.txt > nuova.txt
"""
import sys


def main():
    righe = open(sys.argv[1]).read().split("\n")
    cella, robot, base, mappa, i = 0.025, None, None, [], 0
    for i, r in enumerate(righe):
        p = r.split()
        if not p or p[0].startswith("#"):
            continue
        if p[0] == "cella": cella = float(p[1])
        elif p[0] == "robot": robot = p[1:4]
        elif p[0] == "base": base = p[1:4]
        elif p[0] == "mappa":
            mappa = [x.rstrip() for x in righe[i + 1:]]
            break
    while mappa and not mappa[-1]:
        mappa.pop()
    h, w = len(mappa), max(len(x) for x in mappa)
    g = [[(mappa[h - 1 - j][i] if i < len(mappa[h - 1 - j]) else ".") for i in range(w)] for j in range(h)]
    print("# convertita da", sys.argv[1], "con tools/converti-stanza.py")
    print(f"stanza nome=convertita larghezza={w * cella:g} profondita={h * cella:g}")
    if robot: print(f"robot x={robot[0]} y={robot[1]} angolo={robot[2]}")
    if base: print(f"base x={base[0]} y={base[1]} angolo={base[2]}")
    usato = [[False] * w for _ in range(h)]
    for j in range(h):
        for i in range(w):
            c = g[j][i]
            if c == "." or usato[j][i]:
                continue
            i2 = i
            while i2 + 1 < w and g[j][i2 + 1] == c and not usato[j][i2 + 1]:
                i2 += 1
            j2 = j
            while j2 + 1 < h and all(g[j2 + 1][k] == c and not usato[j2 + 1][k] for k in range(i, i2 + 1)):
                j2 += 1
            for jj in range(j, j2 + 1):
                for ii in range(i, i2 + 1):
                    usato[jj][ii] = True
            x0, y0, x1, y1 = i * cella, j * cella, (i2 + 1) * cella, (j2 + 1) * cella
            if c == "v":
                print(f"vuoto punti={x0:g},{y0:g} {x1:g},{y0:g} {x1:g},{y1:g} {x0:g},{y1:g}")
            else:
                q = "quota_da=0 quota_a=" + ("2.5" if c == "#" else "0.05")
                print(f"rettangolo x={(x0 + x1) / 2:g} y={(y0 + y1) / 2:g} larghezza={x1 - x0:g} profondita={y1 - y0:g} angolo=0 {q}")


if __name__ == "__main__":
    main()
