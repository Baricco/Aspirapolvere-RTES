# Confronta la mappa registrata da sonda_mappa.c (build/prove-nav/sonda_<prova>.bin, scritta all'uscita della copia di
# prova) con la geometria vera della stanza (build/prove-nav/stanza_<prova>.txt, con la partenza del robot).
# Uso (dalla cartella del progetto): python3 tools/prove-nav/analizza_mappa.py <prova>
# Dice: quanto sono lontane dagli ostacoli veri le celle fini occupate (giuste, spostate di una cella, fantasma), quali
# gambe sottili mancano, quante celle fini passano la soglia più volte (mappa "ballerina") e dove, e quante celle di
# navigazione sono bloccate senza motivo o libere troppo vicino a un ostacolo.
import math, os, sys
import numpy as np

REPO = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
D = os.path.join(REPO, 'build', 'prove-nav')
tag = sys.argv[1]
LIDAR = 0.087                       # altezza del lidar: vede gli oggetti che la attraversano
R, INGR, NF, NN = 0.1745, 0.20, 1024, 256


def campi(riga):
    return dict(p.split('=', 1) for p in riga.split()[1:] if '=' in p)


rett, cerchi, dim, start = [], [], (0, 0), None
for riga in open(os.path.join(D, f'stanza_{tag}.txt')):
    riga = riga.split('#')[0].strip()
    if not riga: continue
    k, c = riga.split()[0], campi(riga)
    visto = float(c.get('quota_da', 0)) <= LIDAR <= float(c.get('quota_a', 9))
    if k == 'stanza': dim = (float(c['larghezza']), float(c['profondita']))
    elif k == 'robot': start = (float(c['x']), float(c['y']), math.radians(float(c['angolo'])))
    elif k == 'muro':
        (x1, y1), (x2, y2) = [tuple(map(float, c[q].split(','))) for q in ('da', 'a')]
        rett.append(((x1 + x2) / 2, (y1 + y2) / 2, math.hypot(x2 - x1, y2 - y1) / 2, float(c.get('spessore', 0.1)) / 2,
                     math.atan2(y2 - y1, x2 - x1), 'muro'))
    elif k == 'rettangolo' and visto:
        rett.append((float(c['x']), float(c['y']), float(c['larghezza']) / 2, float(c['profondita']) / 2,
                     math.radians(float(c.get('angolo', 0))), c.get('nome', 'mobile')))
    elif k == 'cerchio' and visto:
        cerchi.append((float(c['x']), float(c['y']), float(c['raggio']), c.get('nome', 'gamba')))


def distanza(x, y):
    """distanza dall'ostacolo più vicino visto dal lidar (0 dentro) e se è una gamba sottile"""
    d = np.full(np.shape(x), 9.0); gamba = np.zeros(np.shape(x), bool)
    for cx, cy, w, h, ang, _ in rett:
        u = (x - cx) * math.cos(ang) + (y - cy) * math.sin(ang); v = -(x - cx) * math.sin(ang) + (y - cy) * math.cos(ang)
        dd = np.hypot(np.maximum(abs(u) - w, 0), np.maximum(abs(v) - h, 0)); gamba &= dd >= d; d = np.minimum(d, dd)
    for cx, cy, r, _ in cerchi:
        dd = np.maximum(np.hypot(x - cx, y - cy) - r, 0); gamba |= dd < d; d = np.minimum(d, dd)
    return d, gamba


b = open(os.path.join(D, f'sonda_{tag}.bin'), 'rb').read()
if b[:4] != b'RTSD': sys.exit("file della sonda non valido")
o = 4
fine = np.frombuffer(b, np.int8, NF * NF, o).reshape(NF, NF); o += NF * NF            # [y, x]
LATO = 512; cambi_f = np.zeros((NF, NF), np.int32)                                  # contati solo nel quadrato centrale
q0 = (NF - LATO) // 2; cambi_f[q0:q0 + LATO, q0:q0 + LATO] = np.frombuffer(b, np.uint8, LATO * LATO, o).reshape(LATO, LATO); o += LATO * LATO
nav = np.frombuffer(b, np.uint8, NN * NN, o).reshape(NN, NN); o += NN * NN
cambi_n = np.frombuffer(b, np.uint16, NN * NN, o).reshape(NN, NN)
x0, y0, a0 = start


def a_stanza(xr, yr):
    return x0 + math.cos(a0) * xr - math.sin(a0) * yr, y0 + math.sin(a0) * xr + math.cos(a0) * yr


def dentro(X, Y, m=0.05):
    return (X > -m) & (X < dim[0] + m) & (Y > -m) & (Y < dim[1] + m)


# ---- celle fini ----
iy, ix = np.mgrid[0:NF, 0:NF]
FX, FY = a_stanza((ix - NF / 2 + 0.5) * 0.05, (iy - NF / 2 + 0.5) * 0.05)
zona = dentro(FX, FY)
occ = zona & ((fine > 50) | (fine == 127))
d_occ, _ = distanza(FX[occ], FY[occ])
giuste, spostate, fantasma = (d_occ <= 0.036).sum(), ((d_occ > 0.036) & (d_occ <= 0.086)).sum(), (d_occ > 0.086).sum()
print(f"== {tag}")
print(f"celle fini occupate alla fine: {occ.sum()}")
print(f"   toccano un ostacolo vero (entro 3,6 cm dal centro): {giuste}")
print(f"   spostate di una cella (3,6-8,6 cm):                 {spostate}")
print(f"   fantasma (oltre 8,6 cm da ogni ostacolo):           {fantasma}")
if fantasma:
    fx, fy = FX[occ][d_occ > 0.086], FY[occ][d_occ > 0.086]
    print("      fantasma più lontane: " + ", ".join(f"({x:.2f}, {y:.2f}) a {d * 100:.0f} cm" for x, y, d in
                                                  sorted(zip(fx, fy, d_occ[d_occ > 0.086]), key=lambda t: -t[2])[:6]))
# gambe sottili senza nessuna cella occupata entro 5 cm dal bordo
mancano = []
for cx, cy, r, nome in cerchi:
    if not (dentro(np.array(cx), np.array(cy))): continue
    vicino = occ & (np.hypot(FX - cx, FY - cy) <= r + 0.05)
    if not vicino.any(): mancano.append(f"{nome} ({cx:.2f}, {cy:.2f})")
print(f"gambe sottili senza celle occupate vicino: {len(mancano)} su {len(cerchi)}" + (": " + ", ".join(mancano) if mancano else ""))

# ---- passaggi di soglia (mappa ballerina) ----
cz = cambi_f * zona
print(f"passaggi di soglia delle celle fini: {int(cz.sum())} in tutto, celle che l'hanno passata almeno 3 volte: {(cz >= 3).sum()}")
balla = cz >= 3
if balla.any():
    d_b, gamba_b = distanza(FX[balla], FY[balla])
    print(f"   di queste: vicino a una gamba sottile {(gamba_b & (d_b <= 0.086)).sum()}, vicino a muri o mobili "
          f"{(~gamba_b & (d_b <= 0.086)).sum()}, lontano da tutto {(d_b > 0.086).sum()}")
    top = np.argsort(-cz[balla])[:5]
    print("   le più ballerine: " + ", ".join(f"({FX[balla][i]:.2f}, {FY[balla][i]:.2f}) {cz[balla][i]} volte" for i in top))

# ---- celle di navigazione ----
jy, jx = np.mgrid[0:NN, 0:NN]
NX, NY = a_stanza((jx - NN / 2 + 0.5) * 0.2, (jy - NN / 2 + 0.5) * 0.2)
zn = dentro(NX, NY, 0)
stato = nav & 15
d_n, _ = distanza(NX, NY)
print(f"celle di navigazione nella stanza: {zn.sum()}; cambiate almeno una volta: {(zn & (cambi_n > 0)).sum()}, "
      f"almeno 5 volte: {(zn & (cambi_n >= 5)).sum()}")
bl = zn & (stato == 2)
print(f"   bloccate alla fine: {bl.sum()}; con il centro a più di 23,5 cm dagli ostacoli veri (non spiegabili con "
      f"l'arrotondamento): {(bl & (d_n > 0.235)).sum()}")
li = zn & (stato == 1)
print(f"   libere alla fine: {li.sum()}; con il centro a meno di {R * 100:.2f} cm da un ostacolo vero (il robot lo "
      f"tocca): {(li & (d_n < R)).sum()}")
