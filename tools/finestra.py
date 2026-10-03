#!/usr/bin/env python3
"""finestra.py - Statistiche degli ultimi frame dei sensori registrati dal banco di prova sul robot (fase 2, blocco D).

Legge le ultime N righe di /tmp/rtes/NOME.csv (scritto da "spia guida --csv") e stampa, per ogni campo scelto,
media, minimo e massimo. Serve durante le prove insieme all'utente: si fa un'azione (ruotare il robot, mettere la
mano davanti a un sensore) e si guarda cosa cambia.

Uso: tools/finestra.py NOME [N=50] [campo,campo,...]      (senza campi: tutti quelli che variano o sono != 0)
"""
import csv, io, os, statistics, subprocess, sys

DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
nome = sys.argv[1]
n = int(sys.argv[2]) if len(sys.argv) > 2 else 50
campi = sys.argv[3].split(",") if len(sys.argv) > 3 else None

testo = subprocess.run([os.path.join(DIR, "..", "ssh-robot.sh"),
                        f"head -1 /tmp/rtes/{nome}.csv; tail -n {n} /tmp/rtes/{nome}.csv"],
                       capture_output=True, text=True, check=True).stdout
righe = list(csv.DictReader(io.StringIO(testo)))
righe = [r for r in righe if r.get("t_s") and r["t_s"] != "t_s" and None not in r.values() and "" not in r.values()]
print(f"{len(righe)} frame, t = {righe[0]['t_s']} .. {righe[-1]['t_s']} s")
for c in campi or righe[0].keys():
    if c == "t_s":
        continue
    v = [int(r[c]) for r in righe]
    if campi or min(v) != max(v) or v[0] != 0:
        print(f"  {c:14s} media {statistics.mean(v):9.1f}   min {min(v):7d}   max {max(v):7d}")
