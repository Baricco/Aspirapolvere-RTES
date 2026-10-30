# Ricalcola da fuori i quadranti bloccati di ogni cella di navigazione dalla griglia fine registrata da sonda_mappa.c
# e li confronta con i 4 bit alti scritti dalla mappatura (stessa regola: cella fine occupata entro 20 cm dal centro
# del quadrante; due soli quadranti liberi in diagonale -> resta libero quello in basso).
# Uso: python3 tools/prove-nav/controlla_quadranti.py <prova>
import os, sys
import numpy as np
D = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'build', 'prove-nav')
b = open(os.path.join(D, f'sonda_{sys.argv[1]}.bin'), 'rb').read(); NF = 1024
fine = np.frombuffer(b, np.int8, NF * NF, 4).reshape(NF, NF)
nav = np.frombuffer(b, np.uint8, 65536, 4 + NF * NF + 512 * 512).reshape(256, 256)
occ = (fine > 50) | (fine == -128)          # 127 è già > 50; -128 (vuoto) conta come occupata
oy, ox = np.nonzero(occ)
# quadranti: bit 0 alto sinistra, 1 alto destra, 2 basso sinistra, 3 basso destra; centri in unità di celle fini
# (la cella di navigazione i va da 4i a 4i+4: centro 4i+2, quadranti a 4i+1 e 4i+3; alto = y più alta)
QX, QY = (1, 3, 1, 3), (3, 3, 1, 1)
errori = provate = 0
celle = [(y, x) for y in range(1, 255) for x in range(1, 255) if (nav[y, x] & 15) != 0 or nav[y, x] >> 4]
for y, x in celle:
    vicini = (abs(ox + 0.5 - (4 * x + 2)) <= 6) & (abs(oy + 0.5 - (4 * y + 2)) <= 6)
    m = 0
    for q in range(4):
        cx, cy = 4 * x + QX[q], 4 * y + QY[q]
        if (((ox[vicini] + 0.5 - cx) ** 2 + (oy[vicini] + 0.5 - cy) ** 2) <= 16).any(): m |= 1 << q
    if m in (9, 6): m |= 3
    provate += 1
    if m != nav[y, x] >> 4:
        errori += 1
        if errori <= 5: print(f"cella ({x}, {y}): mappa {nav[y, x] >> 4:04b}, ricalcolata {m:04b}, stato {nav[y, x] & 15}")
print(f"celle controllate {provate}, quadranti diversi {errori}")
