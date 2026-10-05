#!/usr/bin/env python3
"""analizza-driver.py - Riassunto di una prova di rtes-driver dal suo CSV (--csv, una riga al secondo).

Stampa: durata, frame dei sensori al secondo (minimo e media), età massima dell'ultimo frame, ritardo massimo del ciclo
da 20 ms e durata massima di un risveglio del driver (sono massimi dall'inizio: conta l'ultimo valore), frame errati
ed errori di scrittura, velocità del lidar (media, minimo, massimo con il lidar acceso), tensione all'inizio e alla
fine, e per il task di prova (--task-prova) latenza ed esecuzione massime, attivazioni perse e scadenze mancate.
Segnala i secondi "anomali": meno di 49 frame, età dei sensori oltre 40 ms.

Uso: tools/analizza-driver.py build/robot/fase3/NOME/NOME.csv
"""
import csv, statistics, sys


def main():
    righe = list(csv.DictReader(open(sys.argv[1])))
    if not righe:
        print("CSV vuoto")
        return
    f = lambda r, k: float(r[k])
    ult = righe[-1]
    hz = [f(r, "sensori_hz") for r in righe[1:]]           # il primo secondo comprende l'avvio
    eta = [f(r, "eta_sensori_ms") for r in righe]
    lidar = [f(r, "lidar_giri_s") for r in righe if f(r, "lidar_potenza") > 0]
    print(f"durata {f(ult, 't_s'):.0f} s ({len(righe)} righe)")
    if hz:
        print(f"sensori: media {statistics.mean(hz):.2f} Hz, minimo {min(hz):.0f} Hz; età massima dell'ultimo frame "
              f"{max(eta):.1f} ms")
    print(f"driver: ritardo massimo del ciclo {f(ult, 'tick_ritardo_max_us'):.1f} us, risveglio più lungo "
          f"{f(ult, 'lavoro_max_us'):.1f} us")
    if "intervallo_sensori_max_ms" in ult:
        mn = f"da {f(ult, 'intervallo_sensori_min_ms'):.2f} " if "intervallo_sensori_min_ms" in ult else ""
        print(f"intervallo fra due frame dei sensori: {mn}fino a {f(ult, 'intervallo_sensori_max_ms'):.2f} ms (atteso 20)")
    print(f"frame errati {ult['frame_errati']}, errori di scrittura {ult['errori_scrittura']}, keepalive {ult['keepalive']}")
    if lidar:
        print(f"lidar: {statistics.mean(lidar):.3f} giri/s di media (min {min(lidar):.2f}, max {max(lidar):.2f}), "
              f"{ult['lidar_giri']} giri")
    print(f"batteria: {righe[0]['volt']} V -> {ult['volt']} V")
    if f(ult, "task_lat_max_us") > 0:
        print(f"task di prova: latenza max {f(ult, 'task_lat_max_us'):.1f} us, esecuzione max "
              f"{f(ult, 'task_esec_max_us'):.1f} us, attivazioni perse {ult['task_persi']}, scadenze mancate "
              f"{ult['task_mancate']}")
    fasi = sorted(set(r["fase"] for r in righe))
    print("fasi viste:", ", ".join({"0": "avvio", "1": "pronto", "2": "GUASTO", "3": "fine"}.get(x, x) for x in fasi))
    anomali = [r for r in righe[2:] if f(r, "sensori_hz") < 49 or f(r, "eta_sensori_ms") > 40]
    print(f"secondi anomali: {len(anomali)}")
    for r in anomali[:10]:
        print(f"  t {r['t_s']} s: {r['sensori_hz']} Hz, età {r['eta_sensori_ms']} ms")


if __name__ == "__main__":
    main()
