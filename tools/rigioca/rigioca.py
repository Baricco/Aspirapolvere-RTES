#!/usr/bin/env python3
"""rigioca.py - Ripassa una registrazione del robot (NOME.reg, formato della spia) nel programma vero, sul PC.

Crea due terminali finti (pty) al posto delle seriali del GD32 e del lidar, lancia il programma con RTES_GD32 e
RTES_LIDAR che puntano lì, e ci scrive i byte arrivati dal robot con i loro tempi. Quello che il programma manda
(comandi ai motori, keepalive...) si legge e si butta: il robot registrato ha fatto quello che ha fatto, quindi la
posizione e la mappa del ripasso sono quelle che il programma calcola dai dati veri.
Con --velocita N il ripasso va N volte più in fretta: il programma è caricato con tempo_veloce.so (LD_PRELOAD), che
gli fa vedere il tempo scorrere N volte più in fretta, quindi i tempi dei dati restano quelli originali.

Uso: tools/rigioca/rigioca.py NOME.reg PROGRAMMA [--velocita N] [--fino S] [--env VAR=valore ...]
"""
import argparse, os, select, struct, subprocess, sys, time, tty

a = argparse.ArgumentParser()
a.add_argument('reg'); a.add_argument('programma')
a.add_argument('--velocita', type=float, default=1.0)
a.add_argument('--fino', type=float, default=1e9, help='secondi della registrazione dopo cui fermarsi')
a.add_argument('--env', action='append', default=[])
a.add_argument('--lib', default=os.path.join(os.path.dirname(os.path.abspath(__file__)), '../../build/host/tempo_veloce.so'))
a.add_argument('--uscita', default='-', help='file per stdout/stderr del programma')
o = a.parse_args()

f = open(o.reg, 'rb')
assert f.read(6) == b'SPIA1\n', 'non è una registrazione della spia'
proto = {}
for _ in range(f.read(1)[0]):
    cid, pr, ln = f.read(3); f.read(ln); f.read(f.read(1)[0]); proto[cid] = pr
canale = {c: ('gd32' if p == 1 else 'lidar') for c, p in proto.items() if p in (1, 2)}

pty = {}
for nome in ('gd32', 'lidar'):
    m, s = os.openpty(); tty.setraw(s); pty[nome] = (m, s, os.ttyname(s))
env = dict(os.environ, RTES_GD32=pty['gd32'][2], RTES_LIDAR=pty['lidar'][2], RTES_VELOCITA=str(o.velocita))
if o.velocita != 1: env['LD_PRELOAD'] = o.lib
for e in o.env:
    k, v = e.split('=', 1); env[k] = v
out = sys.stdout if o.uscita == '-' else open(o.uscita, 'w')
p = subprocess.Popen([o.programma], env=env, stdout=out, stderr=subprocess.STDOUT)
inizio = time.monotonic()
masters = [pty[n][0] for n in pty]
t_reg0 = None; n = 0; ultimo_avviso = 0
try:
    while True:
        h = f.read(12)
        if len(h) < 12: break
        t, c, d, k = struct.unpack('<qBBH', h); dati = f.read(k)
        if t_reg0 is None: t_reg0 = t
        tr = (t - t_reg0) / 1e9
        if tr > o.fino: break
        if d != 0 or c not in canale: continue
        quando = inizio + tr / o.velocita
        while True:
            resta = quando - time.monotonic()
            r, _, _ = select.select(masters, [], [], max(resta, 0))
            for m in r:
                try: os.read(m, 65536)
                except OSError: pass
            if resta <= 0: break
        if p.poll() is not None: print('il programma è uscito', file=sys.stderr); break
        os.write(pty[canale[c]][0], dati); n += 1
        if tr - ultimo_avviso >= 300:
            ultimo_avviso = tr
            print(f'rigioca: {tr:.0f} s della registrazione, ritardo {time.monotonic() - quando:.3f} s', file=sys.stderr, flush=True)
finally:
    if p.poll() is None:
        p.terminate()
        try: p.wait(20)
        except subprocess.TimeoutExpired: p.kill()
    print(f'rigioca: fine, {n} blocchi scritti', file=sys.stderr)
