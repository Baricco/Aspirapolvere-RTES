#!/usr/bin/env python3
"""ritmo.py - Trova i bit dei sensori che seguono un'azione ripetuta (premi/rilascia N volte).

Scarica il registro della spia dal robot, prende i frame dei sensori 0x15 degli ultimi `secondi` e stampa i bit
che cambiano poche volte (da 2 a 16 transizioni), con gli istanti dei cambi. I byte "analogici" (encoder,
IMU, sensori IR), in cui cambiano molti bit insieme, finiscono in fondo, separati.

Uso: tools/ritmo.py [secondi=30]
"""
import os, subprocess, sys

DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sec = float(sys.argv[1]) if len(sys.argv) > 1 else 30.0
reg = os.path.join(DIR, "build", "robot", "spia", "prove.spia")
subprocess.run([os.path.join(DIR, "tools", "spia-robot.sh"), "scarica", reg], check=True, capture_output=True)
out = subprocess.run([os.path.join(DIR, "build", "host", "spia"), "leggi", "-c", "gd32", reg],
                     capture_output=True, text=True).stdout
righe = []
for l in out.splitlines():
    p = l.split()
    if len(p) > 6 and p[2] == "rx" and p[3] == "0x15":
        righe.append((float(p[0]), [int(x, 16) for x in p[6:]]))
t0, fine = righe[0][0], righe[-1][0]
w = [r for r in righe if r[0] > fine - sec]
print(f"finestra {w[0][0] - t0:.1f} .. {w[-1][0] - t0:.1f} s (tempo della spia)")
digitali, analogici = [], []
for i in range(len(w[0][1])):
    bit_cambiati = []
    for b in range(8):
        v = [(r[1][i] >> b) & 1 for r in w]
        tr = [k for k in range(1, len(v)) if v[k] != v[k - 1]]
        if tr:
            bit_cambiati.append((b, tr))
    if not bit_cambiati:
        continue
    if len(bit_cambiati) <= 2 and all(2 <= len(tr) <= 16 for _, tr in bit_cambiati):
        for b, tr in bit_cambiati:
            istanti = " ".join(f"{w[k][0] - t0:.2f}({(w[k][1][i] >> b) & 1})" for k in tr)
            digitali.append(f"  byte {i:2d} bit {b} (0x{1 << b:02x}): {len(tr)} cambi: {istanti}")
    else:
        valori = sorted({r[1][i] for r in w})
        analogici.append(f"{i}({len(valori)} valori)")
print("bit che cambiano poche volte (istante(nuovo valore)):")
print("\n".join(digitali) if digitali else "  nessuno")
print("byte analogici/rumorosi:", " ".join(analogici))
