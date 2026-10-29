# Disegna una prova di tools/prove-nav/prova.py: la stanza, il percorso VERO del robot nel simulatore
# (vero_<prova>.csv) e quello che CREDE il robot (righe POSA del programma, riportate nelle coordinate della stanza
# con la posa di partenza), gli sblocchi e, sotto, l'errore di posizione nel tempo.
# Uso (dalla cartella del progetto): python3 tools/prove-nav/disegna.py <prova> [--da S] [--a S]
#   <prova> è il nome usato da prova.py, per esempio soggiorno_s1_pulizia; --da/--a limitano il tempo (secondi
#   dall'avvio del programma). Scrive build/prove-nav/percorso_<prova>.png e stampa un riassunto.
import bisect, math, os, sys
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon, Circle

REPO = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
D = os.path.join(REPO, 'build', 'prove-nav')
NOMI = ['WAIT', 'EXPLORE', 'CLEAN', 'BORDERS', 'HOME', 'HOOK', 'FOLLOW', 'UNLOCK', 'SLEEP', 'ALIGN', 'FAILURE']

a = sys.argv[1:]
tag = a[0]
da = float(a[a.index('--da') + 1]) if '--da' in a else -1e9
fino = float(a[a.index('--a') + 1]) if '--a' in a else 1e9


def campi(riga):
    return dict(p.split('=', 1) for p in riga.split()[1:] if '=' in p)


def rettangolo(cx, cy, w, h, ang):
    c, s = math.cos(ang), math.sin(ang)
    return [(cx + c * dx - s * dy, cy + s * dx + c * dy) for dx, dy in ((-w/2, -h/2), (w/2, -h/2), (w/2, h/2), (-w/2, h/2))]


# ---- la stanza ----
oggetti, start, dim = [], None, (0, 0)
for riga in open(os.path.join(D, f'stanza_{tag}.txt')):
    riga = riga.split('#')[0].strip()
    if not riga: continue
    k, c = riga.split()[0], campi(riga)
    if k == 'stanza': dim = (float(c['larghezza']), float(c['profondita']))
    elif k == 'robot': start = (float(c['x']), float(c['y']), math.radians(float(c['angolo'])))
    elif k == 'muro':
        (x1, y1), (x2, y2) = [tuple(map(float, c[q].split(','))) for q in ('da', 'a')]
        oggetti.append(('muro', rettangolo((x1 + x2) / 2, (y1 + y2) / 2, math.hypot(x2 - x1, y2 - y1),
                                           float(c.get('spessore', 0.1)), math.atan2(y2 - y1, x2 - x1)), ''))
    elif k == 'rettangolo':
        rialzato = float(c.get('quota_da', 0)) > 0
        oggetti.append(('rialzato' if rialzato else 'mobile', rettangolo(float(c['x']), float(c['y']), float(c['larghezza']),
                        float(c['profondita']), math.radians(float(c.get('angolo', 0)))), c.get('nome', '')))
    elif k == 'cerchio':
        oggetti.append(('cerchio', (float(c['x']), float(c['y']), float(c['raggio'])), c.get('nome', '')))
    elif k == 'vuoto':   # i punti sono separati da spazi: si prendono dalla riga intera
        pts = riga.split('punti=')[1].split()
        oggetti.append(('vuoto', [tuple(map(float, p.split(','))) for p in pts if ',' in p and '=' not in p], c.get('nome', '')))
    elif k == 'base':
        oggetti.append(('base', rettangolo(float(c['x']), float(c['y']), 0.30, 0.10, math.radians(float(c['angolo'])) + math.pi / 2), 'base'))
x0, y0, a0 = start


def a_stanza(xn, yn):
    """dalla posizione che crede il robot (origine e asse x alla partenza) alle coordinate della stanza"""
    return x0 + math.cos(a0) * xn - math.sin(a0) * yn, y0 + math.sin(a0) * xn + math.cos(a0) * yn


# ---- il percorso vero e quello creduto ----
vero = []
for riga in list(open(os.path.join(D, f'vero_{tag}.csv')))[1:]:
    m, t, x, y, d, b, bl, inc = riga.strip().split(',')
    vero.append((float(m), float(x), float(y), int(b), int(bl), int(inc)))
posa, nav, percorsi = [], [], []
for riga in open(os.path.join(D, f'prog_{tag}.log'), errors='replace'):
    p = riga.split()
    if not p: continue
    if p[0] == 'POSA' and len(p) >= 6: posa.append((float(p[1]), float(p[2]), float(p[3]), float(p[4]), int(p[5])))
    elif p[0] == 'NAV' and len(p) >= 11: nav.append(p)
    elif p[0] == 'PERCORSO' and len(p) >= 5: percorsi.append(p)
if not posa: sys.exit("nessuna riga POSA nel registro: la copia di prova va ricostruita con tools/prove-nav/costruisci.sh")
t_avvio = posa[0][0]
t_pul = next((float(n[1]) for n in nav if n[2] == '1' and n[3] == '2'), None)
mono_vero = [v[0] for v in vero]


def vero_a(t):
    i = bisect.bisect_left(mono_vero, t)
    if i <= 0: return vero[0]
    if i >= len(vero): return vero[-1]
    return vero[i] if mono_vero[i] - t < t - mono_vero[i - 1] else vero[i - 1]


sel = [q for q in posa if da <= q[0] - t_avvio <= fino]
creduto = [a_stanza(q[1], q[2]) for q in sel]
sel_v = [v for v in vero if da <= v[0] - t_avvio <= fino]
errore = [(q[0] - t_avvio, math.hypot(c[0] - vero_a(q[0])[1], c[1] - vero_a(q[0])[2])) for q, c in zip(sel, creduto)]
sblocchi = [(float(n[1]), float(n[4]), float(n[5])) for n in nav if n[3] == '7' and da <= float(n[1]) - t_avvio <= fino]

# ---- disegno ----
fig = plt.figure(figsize=(13, 11))
ax = fig.add_axes([0.05, 0.30, 0.9, 0.66])
ax.set_aspect('equal'); ax.set_title(f'{tag}: percorso vero (blu) e creduto dal robot (arancione), da {max(da, 0):.0f} s')
for tipo, g, nome in oggetti:
    if tipo == 'cerchio':
        ax.add_patch(Circle(g[:2], g[2], color='0.35'))
    else:
        stile = {'muro': dict(color='0.2'), 'mobile': dict(color='0.55', alpha=0.8),
                 'rialzato': dict(facecolor='none', edgecolor='purple', hatch='//', linestyle='--'),
                 'vuoto': dict(facecolor='none', edgecolor='red', hatch='xx'), 'base': dict(color='green', alpha=0.7)}[tipo]
        ax.add_patch(Polygon(g, closed=True, **stile))
        if nome and tipo != 'muro':
            cx, cy = sum(p[0] for p in g) / len(g), sum(p[1] for p in g) / len(g)
            ax.text(cx, cy, nome, ha='center', va='center', fontsize=8, color='purple' if tipo == 'rialzato' else 'k')
if sel_v: ax.plot([v[1] for v in sel_v], [v[2] for v in sel_v], '-', color='tab:blue', lw=0.8, label='vero (simulatore)')
if creduto: ax.plot([c[0] for c in creduto], [c[1] for c in creduto], '--', color='tab:orange', lw=0.8, label='creduto (moto)')
for t, xn, yn in sblocchi:
    cx, cy = a_stanza(xn, yn); v = vero_a(t)
    ax.plot([cx, v[1]], [cy, v[2]], ':', color='red', lw=0.6)
    ax.plot(cx, cy, 'x', color='red', ms=5); ax.plot(v[1], v[2], 'o', color='red', ms=3, mfc='none')
ax.plot(x0, y0, 'k^', ms=8, label='partenza')
ax.plot([], [], 'rx', label='sblocco: dove crede di essere'); ax.plot([], [], 'ro', mfc='none', label='sblocco: dove è davvero')
xs = [0, dim[0]] + [c[0] for c in creduto]; ys = [0, dim[1]] + [c[1] for c in creduto]
ax.set_xlim(min(xs) - 0.3, max(xs) + 0.3); ax.set_ylim(min(ys) - 0.3, max(ys) + 0.3)
ax.legend(loc='upper right', fontsize=8); ax.grid(alpha=0.3)
bx = fig.add_axes([0.05, 0.05, 0.9, 0.2])
if errore: bx.plot([e[0] for e in errore], [e[1] for e in errore], color='tab:red', lw=0.8)
if t_pul: bx.axvline(t_pul - t_avvio, color='k', ls='--', lw=0.8); bx.text(t_pul - t_avvio, 0, ' inizio pulizia', fontsize=8)
for t, _, _ in sblocchi: bx.axvline(t - t_avvio, color='red', alpha=0.25, lw=0.6)
bx.set_xlabel('secondi dall\'avvio del programma'); bx.set_ylabel('errore di posizione (m)'); bx.grid(alpha=0.3)
uscita = os.path.join(D, f'percorso_{tag}.png'); fig.savefig(uscita, dpi=110)

# ---- riassunto ----
print(f"{tag}: {len(sel)} pose, {len(sel_v)} punti veri, {len(sblocchi)} sblocchi")
for soglia in (0.05, 0.10, 0.20, 0.50):
    oltre = next((e[0] for e in errore if e[1] > soglia), None)
    print(f"   errore oltre {soglia * 100:.0f} cm: " + (f"dopo {oltre:.1f} s" if oltre is not None else "mai"))
if errore: print(f"   errore massimo {max(e[1] for e in errore):.2f} m, alla fine {errore[-1][1]:.2f} m")
print(f"   immagine: {uscita}")
