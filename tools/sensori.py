#!/usr/bin/env python3
"""sensori.py - Confronta in diretta i 96 byte del frame dei sensori (0x15) con uno stato di riferimento.

Legge l'istantanea che la spia riscrive ogni secondo sul robot (/tmp/rtes/spia.sensori: min, max e ultimo
valore di ogni byte nell'ultimo secondo). Serve per le prove dei sensori: si salva lo stato a riposo, si fa
un'azione (premere un bumper...) e si vede quali byte sono cambiati.

  tools/sensori.py base          salva l'istantanea attuale come riferimento (build/robot/sensori-base.txt)
  tools/sensori.py diff          byte il cui intervallo [min, max] dell'ultimo secondo non si sovrappone a quello
                                 di riferimento (cambiati davvero, non rumore)
  tools/sensori.py tutto         tutti i 96 byte, attuale e riferimento
"""
import os, subprocess, sys

DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BASE = os.path.join(DIR, "build", "robot", "sensori-base.txt")


def leggi_robot():
    out = subprocess.run([os.path.join(DIR, "..", "ssh-robot.sh"), "cat /tmp/rtes/spia.sensori"],
                         capture_output=True, text=True, check=True).stdout
    return analizza(out)


def analizza(testo):
    righe = dict(r.split(" ", 1) for r in testo.strip().splitlines())
    return {"t": float(righe["t"]),
            **{k: [int(x, 16) for x in righe[k].split()] for k in ("min", "max", "ultimo")}}


def testo(ist):
    return f"t {ist['t']:.2f}\n" + "".join(f"{k} {' '.join(f'{x:02x}' for x in ist[k])}\n"
                                          for k in ("min", "max", "ultimo"))


def main():
    cmd = sys.argv[1] if len(sys.argv) > 1 else "diff"
    ora = leggi_robot()
    if cmd == "base":
        os.makedirs(os.path.dirname(BASE), exist_ok=True)
        with open(BASE, "w") as f:
            f.write(testo(ora))
        print(f"riferimento salvato (t = {ora['t']:.2f} s)")
        return
    with open(BASE) as f:
        base = analizza(f.read())
    print(f"spia t = {ora['t']:.2f} s, riferimento t = {base['t']:.2f} s")
    for i in range(len(ora["ultimo"])):
        bmin, bmax, omin, omax = base["min"][i], base["max"][i], ora["min"][i], ora["max"][i]
        cambiato = omax < bmin or omin > bmax
        if cmd == "tutto" or cambiato:
            print(f"  [{i:2d}] riferimento {bmin:02x}..{bmax:02x}  ora {omin:02x}..{omax:02x}  ultimo {ora['ultimo'][i]:02x}"
                  f" ({ora['ultimo'][i]:08b}){'   <== CAMBIATO' if cambiato else ''}")


if __name__ == "__main__":
    main()
