#!/usr/bin/env python3
"""
analizza-latenza.py - Statistiche e istogrammi dei CSV prodotti da bench/latenza.

Uso:
    tools/analizza-latenza.py build/robot/idle.csv build/robot/altri.csv ...
    tools/analizza-latenza.py -o docs/misure/fase0.png build/robot/*.csv

Per ogni file stampa min, media, percentili e massimo di latenza ed esecuzione,
in microsecondi. Nel real-time conta la coda: il 99,9° percentile e soprattutto
il massimo, non la media.
Con -o salva un'immagine con un istogramma per file (asse y logaritmico, così
si vedono anche i pochi campioni lenti), tutti con lo stesso asse x per
confrontarli a colpo d'occhio.
"""
import argparse
import csv
import os
import sys


def carica(percorso):
    lat, ese = [], []
    with open(percorso) as f:
        for riga in csv.DictReader(f):
            lat.append(int(riga["latenza_ns"]) / 1000.0)
            ese.append(int(riga["esecuzione_ns"]) / 1000.0)
    return lat, ese


def percentile(ordinati, p):
    if not ordinati:
        return float("nan")
    k = min(len(ordinati) - 1, int(round(p / 100.0 * (len(ordinati) - 1))))
    return ordinati[k]


def riassunto(valori):
    s = sorted(valori)
    return {
        "n": len(s),
        "min": s[0],
        "media": sum(s) / len(s),
        "p50": percentile(s, 50),
        "p99": percentile(s, 99),
        "p99.9": percentile(s, 99.9),
        "max": s[-1],
    }


def stampa(nome, r):
    print(f"  {nome:<11} n={r['n']:<7} min {r['min']:8.1f}  media {r['media']:8.1f}  "
          f"p50 {r['p50']:8.1f}  p99 {r['p99']:8.1f}  p99.9 {r['p99.9']:8.1f}  max {r['max']:8.1f} us")


def grafico(dati, uscita):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    colore, testo, testo2, griglia = "#2a78d6", "#0b0b0b", "#52514e", "#e4e3df"
    n = len(dati)
    xmax = max(max(lat) for _, lat in dati) * 1.05
    fig, assi = plt.subplots(n, 1, figsize=(8, 1.9 * n + 0.6), sharex=True, squeeze=False)
    fig.patch.set_facecolor("#fcfcfb")
    for ax, (nome, lat) in zip(assi[:, 0], dati):
        ax.set_facecolor("#fcfcfb")
        ax.hist(lat, bins=200, range=(0, xmax), color=colore, log=True)
        r = riassunto(lat)
        ax.set_title(f"{nome}  ·  p99.9 {r['p99.9']:.0f} µs  ·  max {r['max']:.0f} µs",
                     loc="left", fontsize=10, color=testo)
        ax.grid(axis="y", color=griglia, linewidth=0.6)
        ax.set_axisbelow(True)
        for lato in ("top", "right"):
            ax.spines[lato].set_visible(False)
        for lato in ("left", "bottom"):
            ax.spines[lato].set_color(griglia)
        ax.tick_params(colors=testo2, labelsize=8)
        ax.set_ylabel("job", color=testo2, fontsize=8)
    assi[-1, 0].set_xlabel("latenza di risveglio (µs)", color=testo2, fontsize=9)
    fig.tight_layout()
    os.makedirs(os.path.dirname(uscita) or ".", exist_ok=True)
    fig.savefig(uscita, dpi=130)
    print(f"grafico salvato in {uscita}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", nargs="+", help="file CSV di bench/latenza")
    ap.add_argument("-o", "--uscita", help="salva gli istogrammi in questo file PNG")
    a = ap.parse_args()

    dati = []
    for p in a.csv:
        lat, ese = carica(p)
        if not lat:
            print(f"{p}: vuoto", file=sys.stderr)
            continue
        nome = os.path.splitext(os.path.basename(p))[0]
        print(nome)
        stampa("latenza", riassunto(lat))
        stampa("esecuzione", riassunto(ese))
        dati.append((nome, lat))
    if a.uscita and dati:
        grafico(dati, a.uscita)


if __name__ == "__main__":
    main()
