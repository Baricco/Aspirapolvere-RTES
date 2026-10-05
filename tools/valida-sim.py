#!/usr/bin/env python3
"""valida-sim.py - Confronta il simulatore con il robot vero: rimanda al GD32 simulato, con i tempi originali, i comandi
registrati in una prova sul robot e confronta i frame che produce con quelli registrati.

Prove disponibili (registrazioni della fase 2, in build/robot/fase2/, fuori da git):
  c2   ruote: gradini di 0x66 e 0x67 a robot SOLLEVATO   -> encoder, velocità misurata (byte 32/36)
  c2b  ruote: retromarcia sugli anti-caduta, heartbeat   -> encoder
  c3   spazzole e aspirazione                             -> eco, giri della ventola, correnti
  c1   lidar a potenze 40..80                             -> velocità del lidar
  val1 ruote A TERRA: gradini di v e omega con il driver    -> encoder, velocità misurata (fase 3, build/robot/fase3/)
In quelle della fase 2 il robot era sollevato e il GD32 aveva già ricevuto 0x19 01 da everest: il simulatore parte nello
stesso stato. val1 l'ha registrata il driver (--registra) con il robot a terra, dall'avvio.

Stampa l'errore di ogni grandezza e lo confronta con una soglia. Se la registrazione non c'è (per esempio su un altro
PC) la prova si salta. Uso: tools/valida-sim.py [c2 c1 ...]  (default: c2)
"""
import json, os, select, struct, subprocess, sys, time, urllib.request

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "taratura"))
from registri import DAL_SOC, VERSO_SOC, frame_gd32, leggi_registro, pacchetti_lidar, i32, u16, ck_gd32

DIR_REG = "build/robot/fase2"
PORTA = 18900 + os.getpid() % 90
falliti = 0


def controlla(cond, testo):
    global falliti
    print(f"  {'ok' if cond else 'FALLITO'}: {testo}")
    if not cond:
        falliti += 1


def frame(cmd, dati=b""):
    corpo = bytes([cmd]) + dati
    c = ck_gd32(corpo)
    return bytes([0xFA, 0xFB, len(corpo) + 2]) + corpo + bytes([c >> 8, c & 0xFF])


def deframe(buf):
    out, i = [], 0
    while i + 6 <= len(buf):
        if buf[i] != 0xFA or buf[i + 1] != 0xFB:
            i += 1
            continue
        n = buf[i + 2]
        if i + 3 + n > len(buf):
            break
        corpo = bytes(buf[i + 3:i + 1 + n])
        if n >= 3 and ck_gd32(corpo) == (buf[i + 1 + n] << 8 | buf[i + 2 + n]):
            out.append((corpo[0], corpo[1:]))
            i += 3 + n
        else:
            i += 1
    return out, buf[i:]


A_TERRA = {"val1"}


def cartella(nome):
    return "build/robot/fase3" if nome in A_TERRA else DIR_REG


def rigioca(nome, con_lidar=False):
    """Rimanda i comandi registrati al simulatore; ritorna (registrati, simulati, pacchetti del lidar veri e simulati)."""
    rec = leggi_registro(f"{cartella(nome)}/{nome}/{nome}.reg")["gd32"]
    tx = frame_gd32(rec, DAL_SOC)
    rx = [(t, d) for t, c, d in frame_gd32(rec, VERSO_SOC) if c == 0x15]
    t0 = tx[0][0]
    durata = (tx[-1][0] - t0) / 1e9 + 1
    d_sim = f"build/sim-valida-{PORTA}"
    sim = subprocess.Popen(["build/host/sim", "--porta", str(PORTA), "--dir", d_sim, "--durata", str(int(durata) + 5),
                            "sim/stanze/vuota.txt"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    for _ in range(100):
        if os.path.exists(f"{d_sim}/lidar"):
            break
        time.sleep(0.02)
    time.sleep(0.1)
    if nome not in A_TERRA:
        urllib.request.urlopen(urllib.request.Request(f"http://127.0.0.1:{PORTA}/comando", data=b"solleva"), timeout=2).read()
    g = os.open(f"{d_sim}/gd32", os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    l = os.open(f"{d_sim}/lidar", os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    # stato lasciato da everest prima della prova: retromarcia sugli anti-caduta e modalità dei motori
    os.write(g, frame(0x19, b"\x01"))
    os.write(g, frame(0x06))
    time.sleep(0.1)
    sim_rx, buf, lid = [], b"", bytearray()
    lid_t = []
    inizio = time.monotonic()
    i = 0
    while time.monotonic() - inizio < durata:
        ora = time.monotonic() - inizio
        while i < len(tx) and (tx[i][0] - t0) / 1e9 <= ora:
            os.write(g, frame(tx[i][1], tx[i][2]))
            i += 1
        r, _, _ = select.select([g, l], [], [], 0.002)
        if g in r:
            try:
                buf += os.read(g, 65536)
            except BlockingIOError:
                pass
            fr, buf = deframe(buf)
            t = time.monotonic() - inizio
            sim_rx += [(t, d) for c, d in fr if c == 0x15]
        if l in r:
            try:
                b = os.read(l, 65536)
                lid += b
                lid_t += [time.monotonic() - inizio] * len(b)
            except BlockingIOError:
                pass
    os.close(g)
    os.close(l)
    sim.terminate()
    sim.wait()
    for f in os.listdir(d_sim):
        os.unlink(f"{d_sim}/{f}")
    os.rmdir(d_sim)
    reg = [((t - t0) / 1e9, d) for t, d in rx]
    lv_reg, lv_sim = [], []
    if con_lidar:
        lv_reg = [((t - t0) / 1e9, p[0] * 0.05) for t, c, p in pacchetti_lidar(leggi_registro(f"{DIR_REG}/{nome}/{nome}-lidar.reg")["lidar"])
                  if c in (0xAD, 0xAE) and p]
        lv_sim = [(t / 1e9, p[0] * 0.05) for t, c, p in pacchetti_lidar([(int(tt * 1e9), VERSO_SOC, bytes([x])) for tt, x in zip(lid_t, lid)])
                  if c in (0xAD, 0xAE) and p]
    return reg, sim_rx, lv_reg, lv_sim


def vicino(serie, t):
    """Il campione di serie (ordinata per tempo) più vicino a t."""
    lo, hi = 0, len(serie) - 1
    while lo < hi:
        m = (lo + hi) // 2
        if serie[m][0] < t:
            lo = m + 1
        else:
            hi = m
    return serie[lo]


def valida_ruote(nome):
    reg, sim, _, _ = rigioca(nome)
    e0r, e0s = i32(reg[0][1], 16), i32(sim[0][1], 16)
    err_enc, err_v, tot = [], [], 0
    for t, d in reg:
        if t < 0.2 or t > reg[-1][0] - 0.2:
            continue
        ts, ds = vicino(sim, t)
        er = i32(d, 16) - e0r
        es = i32(ds, 16) - e0s
        err_enc.append(abs(er - es))
        tot = max(tot, abs(er))
        err_v.append(abs(i32(d, 32) - i32(ds, 32)))
    rms_v = (sum(x * x for x in err_v) / len(err_v)) ** 0.5
    print(f"  encoder: scarto massimo {max(err_enc)} tick su {tot} percorsi ({max(err_enc) / max(tot, 1):.1%}); "
          f"velocità misurata: scarto quadratico medio {rms_v:.0f} unità (828 = 5 cm/s)")
    controlla(max(err_enc) <= max(60, 0.03 * tot), "encoder entro il 3% (o 60 tick ~ 3,6 mm)")
    controlla(rms_v < 120, "velocità misurata entro 120 unità di scarto quadratico medio (si sposta di un frame: ~40)")


def valida_attuatori(nome):
    reg, sim, _, _ = rigioca(nome)
    def serie(fr, k, conv):
        return [(t, conv(d)) for t, d in fr]
    for nome_c, conv, soglia in (("eco spazzola centrale (byte 68)", lambda d: d[68], 5),
                                 ("giri della ventola (byte 69 x 100)", lambda d: d[69] * 100, 1500),
                                 ("corrente della spazzola centrale (byte 80, mA)", lambda d: u16(d, 80), 60)):
        errori = []
        for t, d in reg:
            if t < 0.5:
                continue
            ts, ds = vicino(sim, t)
            errori.append(abs(conv(d) - conv(ds)))
        errori.sort()
        med = errori[len(errori) // 2]
        p90 = errori[int(0.9 * len(errori))]
        print(f"  {nome_c}: scarto mediano {med}, 90° percentile {p90}")
        controlla(med <= soglia, f"{nome_c} entro {soglia} (mediana)")


def valida_lidar(nome):
    _, _, lr, ls = rigioca(nome, con_lidar=True)
    errori = []
    for t, v in lr:
        if t < 1.0 or not ls:
            continue
        ts, vs = vicino(ls, t)
        if abs(ts - t) < 0.2:
            errori.append(abs(v - vs))
    errori.sort()
    med = errori[len(errori) // 2] if errori else 99
    print(f"  velocità del lidar: scarto mediano {med:.2f} giri/s, 90° percentile {errori[int(0.9 * len(errori))]:.2f} su {len(errori)} confronti")
    controlla(med < 0.3, "velocità del lidar entro 0,3 giri/s (mediana)")


def main():
    prove = sys.argv[1:] or ["c2"]
    for p in prove:
        if not os.path.exists(f"{cartella(p)}/{p}/{p}.reg"):
            print(f"{p}: registrazione non trovata in {cartella(p)}: SALTATA")
            continue
        print(f"{p}:")
        {"c2": valida_ruote, "c2b": valida_ruote, "val1": valida_ruote, "c3": valida_attuatori, "c3b": valida_attuatori, "c1": valida_lidar}[p](p)
    print("VALIDA SIM: " + ("OK" if not falliti else f"{falliti} CONTROLLI FALLITI"))
    sys.exit(1 if falliti else 0)


if __name__ == "__main__":
    main()
