# Modello geometrico della mappa di navigazione: quanto pavimento di una stanza del simulatore il robot raggiunge
# partendo da (x, y) con diverse regole della mappa, senza simulatore né rumore del lidar.
# Uso (dalla cartella del progetto): python3 tools/prove-nav/modello.py sim/stanze/cucina.txt x y [--tolleranza M]
#   --tolleranza: una cella fine è occupata se il suo quadrato arriva entro M metri da un ostacolo (0 = solo se lo
#   tocca; 0.005 = mezzo centimetro più prudente, come se il rumore spostasse gli ostacoli verso il robot).
# Celle allineate alla partenza (la griglia del robot ha l'origine lì), partenza con angolo multiplo di 90 gradi.
# Il "pavimento toccato" è l'unione dei cerchi del robot attorno ai centri raggiungibili, su un reticolo da 1 cm.
import math, sys
from collections import deque
import numpy as np

R = 0.1745                      # raggio del robot
INGR = 0.20                     # ingrossamento della mappatura: 4 celle fini dai centri
FINE = 0.05
a = sys.argv[1:]
toll = float(a[a.index('--tolleranza') + 1]) if '--tolleranza' in a else 0.0
stanza, sx, sy = a[0], float(a[1]), float(a[2])


def campi(riga):
    return dict(p.split('=', 1) for p in riga.split()[1:] if '=' in p)


rett, cerchi, dim = [], [], (0, 0)          # rettangoli: (cx, cy, mezza w, mezza h, angolo)
for riga in open(stanza):
    riga = riga.split('#')[0].strip()
    if not riga: continue
    k, c = riga.split()[0], campi(riga)
    if k == 'stanza': dim = (float(c['larghezza']), float(c['profondita']))
    elif k == 'muro':
        (x1, y1), (x2, y2) = [tuple(map(float, c[q].split(','))) for q in ('da', 'a')]
        rett.append(((x1 + x2) / 2, (y1 + y2) / 2, math.hypot(x2 - x1, y2 - y1) / 2, float(c.get('spessore', 0.1)) / 2,
                     math.atan2(y2 - y1, x2 - x1)))
    elif k == 'rettangolo' and float(c.get('quota_da', 0)) < 0.095:      # sotto la torretta: lo urta
        rett.append((float(c['x']), float(c['y']), float(c['larghezza']) / 2, float(c['profondita']) / 2,
                     math.radians(float(c.get('angolo', 0)))))
    elif k == 'cerchio' and float(c.get('quota_da', 0)) < 0.095:
        cerchi.append((float(c['x']), float(c['y']), float(c['raggio'])))


def distanza(x, y):
    """distanza di ogni punto dall'ostacolo più vicino (0 dentro)"""
    d = np.full(np.shape(x), 9.0)
    for cx, cy, w, h, ang in rett:
        u = (x - cx) * math.cos(ang) + (y - cy) * math.sin(ang); v = -(x - cx) * math.sin(ang) + (y - cy) * math.cos(ang)
        d = np.minimum(d, np.hypot(np.maximum(abs(u) - w, 0), np.maximum(abs(v) - h, 0)))
    for cx, cy, r in cerchi:
        d = np.minimum(d, np.maximum(np.hypot(x - cx, y - cy) - r, 0))
    return d


def raggiungi(libero, s):
    """ricerca in ampiezza a 4 vicini su una matrice di bool, dalla casella s"""
    visto = np.zeros_like(libero); q = deque([s]); visto[s] = True
    while q:
        i, j = q.popleft()
        for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            u, v = i + di, j + dj
            if 0 <= u < libero.shape[0] and 0 <= v < libero.shape[1] and libero[u, v] and not visto[u, v]:
                visto[u, v] = True; q.append((u, v))
    return visto


# reticolo da 1 cm per il pavimento
H = 0.01
gx = np.arange(0, dim[0], H) + H / 2; gy = np.arange(0, dim[1], H) + H / 2
GX, GY = np.meshgrid(gx, gy, indexing='ij'); D = distanza(GX, GY)
pavimento = D > 0
K = int(R / H)
DISCO = [(i, j) for i in range(-K, K + 1) for j in range(-K, K + 1) if (i * i + j * j) * H * H <= R * R]


def tocca(centri):
    """pavimento toccato dal robot con il centro sui punti indicati (matrice di bool sul reticolo da 1 cm)"""
    P = np.pad(centri, K); out = np.zeros_like(centri)
    for i, j in DISCO: out |= P[K + i:K + i + centri.shape[0], K + j:K + j + centri.shape[1]]
    return (out & pavimento).sum() * H * H


def su_reticolo(xs, ys, mask):
    m = np.zeros(GX.shape, bool)
    for x, y in zip(xs[mask], ys[mask]):
        i, j = int(x / H), int(y / H)
        if 0 <= i < m.shape[0] and 0 <= j < m.shape[1]: m[i, j] = True
    return m


ok = D >= R
vero = raggiungi(ok, (int(sx / H), int(sy / H)))
TOT = tocca(vero)
print(f"{stanza}, partenza ({sx}, {sy}), tolleranza {toll * 100:.1f} cm: pavimento che il robot può toccare {TOT:.2f} m2")


def riga(nome, area):
    print(f"   {nome:58s} {area:5.2f} m2 = {100 * area / TOT:3.0f}%")


def punti(passo, offset):
    """punti allineati alla partenza: sx + offset + k * passo dentro la stanza"""
    xs = np.arange(sx - passo * math.ceil(sx / passo) + offset, dim[0], passo); ys = np.arange(sy - passo * math.ceil(sy / passo) + offset, dim[1], passo)
    xs = xs[(xs > 0) & (xs < dim[0])]; ys = ys[(ys > 0) & (ys < dim[1])]
    return np.meshgrid(xs, ys, indexing='ij')


def area_regola(PX, PY, libero):
    s = np.unravel_index(np.argmin((PX - sx) ** 2 + (PY - sy) ** 2 + 1e3 * ~libero), PX.shape)
    r = raggiungi(libero, s)
    return tocca(su_reticolo(PX, PY, r))


riga("senza griglia, margine come la mappa (20 cm dagli ostacoli)", tocca(raggiungi(D > INGR, (int(sx / H), int(sy / H)))))

# celle fini occupate (allineate alla partenza)
FX, FY = punti(FINE, FINE / 2)
d_fine = np.full(FX.shape, 9.0)
for u in np.linspace(-FINE / 2, FINE / 2, 11):
    for v in np.linspace(-FINE / 2, FINE / 2, 11):
        d_fine = np.minimum(d_fine, distanza(FX + u, FY + v))
occ = d_fine <= toll + 1e-9
ox, oy = FX[occ], FY[occ]


def libero_dai_centri_fini(PX, PY):
    """regola della mappatura: nessun centro di cella fine occupata entro 20 cm"""
    lib = np.ones(PX.shape, bool)
    for x0, y0 in zip(ox, oy): lib &= (PX - x0) ** 2 + (PY - y0) ** 2 > INGR ** 2 + 1e-12
    return lib


for nome, passo in (("oggi: centri delle celle da 20 cm", 0.20), ("sottocelle da 10 cm, bit = il robot non ci sta", 0.10),
                    ("celle fini da 5 cm, bit = il robot non ci sta", 0.05)):
    PX, PY = punti(passo, passo / 2)
    riga(nome, area_regola(PX, PY, libero_dai_centri_fini(PX, PY)))

# bit = "nella sottocella da 10 cm c'è un ostacolo": la distanza la deve calcolare la navigazione dai quadrati da 10 cm
SX, SY = punti(0.10, 0.05)
occ_sotto = np.zeros(SX.shape, bool)
for x0, y0 in zip(ox, oy):
    i = np.argmin(abs(SX[:, 0] - x0) + 1e3 * (abs(SX[:, 0] - x0) > 0.05)); j = np.argmin(abs(SY[0, :] - y0) + 1e3 * (abs(SY[0, :] - y0) > 0.05))
    occ_sotto[i, j] = True
qx, qy = SX[occ_sotto], SY[occ_sotto]
for nome, soglia in (("sottocelle da 10 cm, bit = ostacolo, garanzia di oggi", INGR - FINE * math.sqrt(2) / 2),
                     ("sottocelle da 10 cm, bit = ostacolo, il robot lo sfiora", R)):
    lib = np.ones(SX.shape, bool)
    for x0, y0 in zip(qx, qy):
        lib &= np.hypot(np.maximum(abs(SX - x0) - 0.05, 0), np.maximum(abs(SY - y0) - 0.05, 0)) > soglia
    riga(nome, area_regola(SX, SY, lib))
