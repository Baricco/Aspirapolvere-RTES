# Una prova della navigazione: simulatore (stanza, seme del rumore, partenza) + build/prove-nav/vacdbg.
# Uso (dalla cartella del progetto): python3 tools/prove-nav/prova.py [--pulizia] stanza.txt seme durata_s [x y angolo]
# Si ferma quando la navigazione arriva alla pulizia o al guasto, o alla durata. Con --pulizia va avanti fino al
# riposo o al ritorno alla base (fine della pulizia) o al guasto, e stampa anche strisce, trasferimenti e metà pulite (righe PERCORSO e
# PULIZIA di costruisci.sh). Stampa un riassunto; i registri restano in build/prove-nav/, compreso il percorso vero
# del robot nel simulatore (vero_<prova>.csv: orologio monotono, t del simulatore, x, y, dir, contatti), che
# tools/prove-nav/disegna.py mette a confronto con quello che crede il robot (righe POSA).
import json, os, re, signal, subprocess, sys, time, urllib.request
REPO = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
D = os.path.join(REPO, 'build', 'prove-nav')
NOMI = ['WAIT', 'EXPLORE', 'CLEAN', 'BORDERS', 'HOME', 'HOOK', 'FOLLOW', 'UNLOCK', 'SLEEP', 'ALIGN', 'FAILURE']
CLEAN, FAILURE, SLEEP, HOME = NOMI.index('CLEAN'), NOMI.index('FAILURE'), NOMI.index('SLEEP'), NOMI.index('HOME')
a = sys.argv[1:]
PULIZIA = '--pulizia' in a
a = [x for x in a if x != '--pulizia']
FINE = (SLEEP, HOME, FAILURE) if PULIZIA else (CLEAN, FAILURE)
stanza, seme, durata = a[0], a[1], float(a[2])
testo = open(os.path.join(REPO, stanza)).read()
if len(a) >= 6:
    testo = re.sub(r'^robot .*$', f'robot x={a[3]} y={a[4]} angolo={a[5]}', testo, flags=re.M)
tag = f"{os.path.basename(stanza)[:-4]}_s{seme}" + (f"_{a[3]}_{a[4]}_{a[5]}" if len(a) >= 6 else '') + ('_pulizia' if PULIZIA else '')
fst = os.path.join(D, f'stanza_{tag}.txt'); open(fst, 'w').write(testo)
os.chdir(REPO)
if os.path.exists('build/sim/mappa_robot'): os.remove('build/sim/mappa_robot')
sim = subprocess.Popen(['build/host/sim', '--seme', seme, fst], stdout=open(os.path.join(D, f'sim_{tag}.log'), 'w'),
                       stderr=subprocess.STDOUT, start_new_session=True)
def stato():
    try: return json.load(urllib.request.urlopen('http://127.0.0.1:8080/stato', timeout=1))
    except Exception: return None
for _ in range(100):
    if stato(): break
    time.sleep(0.1)
env = dict(os.environ, RTES_GD32='build/sim/gd32', RTES_LIDAR='build/sim/lidar',
           RTES_SONDA=os.path.join(D, f'sonda_{tag}.bin'))   # registratore della mappatura (sonda_mappa.c)
fout = os.path.join(D, f'prog_{tag}.log')
prog = subprocess.Popen(['build/host/osserva', '--uscita', 'build/sim/mappa_robot', '--', 'taskset', '-c', '0-3',
                         os.path.join(D, 'vacdbg')], env=env, stdout=open(fout, 'w'), stderr=subprocess.STDOUT,
                        start_new_session=True)
def righe_nav():
    return [l.split() for l in open(fout, errors='replace') if l.startswith('NAV ')]
eventi, prec, t0, fine, ultimo = [], {}, time.time(), None, None
vero = open(os.path.join(D, f'vero_{tag}.csv'), 'w'); vero.write('mono,t,x,y,dir,bumper,bloccato,incastrato\n')
while time.time() - t0 < durata:
    d = stato()
    if d:
        for k in ('bumper', 'bloccato', 'incastrato', 'caduto'):
            v = d[k]; on = any(v) if isinstance(v, list) else bool(v)
            if on and not prec.get(k):
                eventi.append(f"{k} t={d['t']:.1f} x={d['robot']['x']:.2f} y={d['robot']['y']:.2f} dir={d['robot']['dir']:.0f}")
            prec[k] = on
        ultimo = d
        r = d['robot']
        vero.write(f"{time.monotonic():.3f},{d['t']:.3f},{r['x']:.4f},{r['y']:.4f},{r['dir']:.1f},"
                   f"{int(any(d['bumper']))},{int(bool(d['bloccato']))},{int(bool(d['incastrato']))}\n")
    nav = righe_nav()
    if nav and int(nav[-1][3]) in FINE and fine is None: fine = time.time()
    if fine and time.time() - fine > 1.5: break
    time.sleep(0.1)
for l in subprocess.run(['pgrep', '-f', '^' + os.path.join(D, 'vacdbg')], capture_output=True, text=True).stdout.split():
    os.kill(int(l), signal.SIGINT)
try: prog.wait(10)
except Exception: prog.kill()
os.killpg(sim.pid, signal.SIGTERM); sim.wait(); vero.close()
nav = righe_nav()
t_start = next((float(n[1]) for n in nav if n[2] == '0' and n[3] == '1'), None)
esito = 'TIMEOUT'
fine_espl = next((i for i, n in enumerate(nav) if n[2] == '1' and n[3] == str(CLEAN)), None)
if PULIZIA:
    if nav: esito = {SLEEP: 'PULIZIA FINITA', HOME: 'PULIZIA FINITA', FAILURE: 'GUASTO'}.get(int(nav[-1][3]), 'TIMEOUT')
else:
    guasto = next((i for i, n in enumerate(nav) if n[3] == str(FAILURE)), None)
    if guasto is not None and (fine_espl is None or guasto < fine_espl): esito, nav = 'GUASTO', nav[:guasto + 1]
    elif fine_espl is not None: esito, nav = 'ESPLORAZIONE FINITA', nav[:fine_espl + 1]
cont = ''
if os.path.exists('build/sim/mappa_robot'):
    g = open('build/sim/mappa_robot', 'rb').read()
    if len(g) >= 128 + 65536 and g[:4] == b'RTNM':
        gg = g[128:128 + 65536]; st = [v & 15 for v in gg]   # 4 bit bassi = stato, 4 alti = sottocelle bloccate (mapping.h)
        parz = sum(1 for v in gg if v & 15 == 1 and v >> 4)
        cont = f"libere {st.count(1)} (in parte {parz}) bloccate {st.count(2)} troppo strette {st.count(4)}"
mancate = re.findall(r'(task_\w+).*\n\s+job \d+, deadline mancate (\d+)', open(fout, errors='replace').read())
fin = nav[-1] if nav else None
print(f"== {tag}: {esito}" + (f" in {float(fin[1]) - t_start:.1f}s" if fin and t_start and esito != 'TIMEOUT' else '') + f"  | mappa: {cont}")
if ultimo: print(f"   posizione finale nel simulatore: x={ultimo['robot']['x']:.2f} y={ultimo['robot']['y']:.2f} dir={ultimo['robot']['dir']:.0f}")
print(f"   riallineamenti: {sum(1 for n in nav if NOMI[int(n[3])] == 'ALIGN')}, sblocchi: {sum(1 for n in nav if NOMI[int(n[3])] == 'UNLOCK')}, "
      f"celle chiuse: {fin[10] if fin else 0}, deadline mancate: {[m for m in mancate if m[1] != '0'] or 'nessuna'}")
if PULIZIA:
    righe = [l.split() for l in open(fout, errors='replace') if l.startswith(('PERCORSO ', 'PULIZIA '))]
    t_pul = next((float(n[1]) for n in nav if n[2] == '1' and n[3] == str(CLEAN)), None)
    if t_pul is None: print("   la pulizia non è cominciata")
    else:
        dopo = [r for r in righe if float(r[1]) >= t_pul]
        strisce = [r for r in dopo if r[0] == 'PERCORSO' and r[4] == '1']
        trasf = [r for r in dopo if r[0] == 'PERCORSO' and r[4] == '0']
        fine_t = float(fin[1]) if fin and int(fin[3]) in (SLEEP, HOME, FAILURE) else None
        print(f"   esplorazione {t_pul - (t_start or t_pul):.1f}s" + (f", pulizia {fine_t - t_pul:.1f}s" if fine_t else ", pulizia non finita"))
        print(f"   strisce: {len(strisce)} ({sum(int(r[2]) for r in strisce)} celle, di una cella sola: "
              f"{sum(1 for r in strisce if r[2] == '1')}), trasferimenti e uscite: {len(trasf)}")
        nav_pul = [n for n in nav if float(n[1]) >= t_pul]
        print(f"   nella pulizia: riallineamenti {sum(1 for n in nav_pul if NOMI[int(n[3])] == 'ALIGN')}, "
              f"sblocchi {sum(1 for n in nav_pul if NOMI[int(n[3])] == 'UNLOCK')}")
        for r in dopo:
            if r[0] == 'PULIZIA':
                lib, pul, sp = int(r[3]), int(r[5]), int(r[7])
                print(f"   metà pulite {pul} su {2 * lib} ({100 * pul / max(1, 2 * lib):.1f}%), celle libere con una metà sporca: {sp}")
for e in eventi: print('   EVENTO', e)
for n in nav:
    if n[2] == n[3] == '0': continue
    da, verso = NOMI[int(n[2])], NOMI[int(n[3])]
    if verso in (('ALIGN', 'UNLOCK', 'FAILURE', 'SLEEP', 'HOME') if PULIZIA else ('ALIGN', 'UNLOCK', 'FAILURE', 'CLEAN')) or \
       (PULIZIA and da == 'EXPLORE' and verso == 'CLEAN'):
        print(f"   {float(n[1]) - (t_start or 0):6.1f}s {da}->{verso} pos=({n[4]},{n[5]}) th={n[6]} maxv={n[7]} c2c={n[8]} chiuse={n[10]}")
