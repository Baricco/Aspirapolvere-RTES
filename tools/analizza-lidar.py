#!/usr/bin/env python3
"""analizza-lidar.py - Analisi di una cattura del lidar fatta con il banco di prova (fase 2, blocco C1).

Legge il registro della seriale del lidar (NOME-lidar.reg, spia proxy) e, se c'è, quello del GD32 (NOME.reg,
spia guida) per sapere quale potenza 0x71 era comandata in ogni istante (i due registri usano lo stesso orologio
del robot). Decodifica come AuxCtrl (CLidarUnpacket, docs/protocollo.md):
  0xAE velocità:            param[0] u8 x 0,05 = giri/s
  0xAD scansione (nuova):   param[0] u8 x 0,05 giri/s | int16 BE x 0,01 [?] | u16 BE x 0,01 angolo di inizio (gradi)
                            | punti da 3 byte: u8 intensità, u16 BE distanza in quarti di mm
Stampa: velocità media per potenza, settori e punti per giro, distanze e intensità.

Uso: tools/analizza-lidar.py build/robot/fase2/c1/c1-lidar.reg [build/robot/fase2/c1/c1.reg]
"""
import statistics, struct, sys


def leggi_registro(percorso):
    """Ritorna {nome_canale: [(t_ns, direzione, byte)]}."""
    d = open(percorso, "rb").read()
    assert d.startswith(b"SPIA1\n"), "intestazione sbagliata"
    i = 6
    n = d[i]; i += 1
    nomi = {}
    for _ in range(n):
        cid = d[i]; i += 2
        l = d[i]; i += 1; nomi[cid] = d[i:i + l].decode(); i += l
        l = d[i]; i += 1 + l
    rec = {}
    while i + 12 <= len(d):
        t, c, direz, k = struct.unpack_from("<qBBH", d, i); i += 12
        rec.setdefault(nomi[c], []).append((t, direz, d[i:i + k])); i += k
    return rec


def crc16(b):
    ck = 0xFFFF
    for x in b:
        ck ^= x
        for _ in range(8):
            ck = (ck >> 1) ^ 0xA001 if ck & 1 else ck >> 1
    return ck


def pacchetti_lidar(records):
    """(t_ns, cmd, param) per ogni pacchetto valido; il tempo è quello del blocco che lo completa."""
    buf, tempi, out = bytearray(), [], []
    for t, direz, b in records:
        if direz != 0:
            continue
        buf += b
        tempi += [t] * len(b)
        i = 0
        while i + 10 <= len(buf):
            if buf[i] != 0xAA or buf[i + 1] != 0:
                i += 1
                continue
            ln = buf[i + 2]
            if not 6 <= ln <= 100:
                i += 1
                continue
            if i + ln + 2 > len(buf):
                break
            p = bytes(buf[i:i + ln])
            ck = buf[i + ln] << 8 | buf[i + ln + 1]
            if ck == ((sum(p) & 0xFFFF) if p[3] == 1 else crc16(p)):
                out.append((tempi[i + ln + 1], p[5], p[8:]))
                i += ln + 2
            else:
                i += 1
        del buf[:i]
        del tempi[:i]
    return out


def potenze(records_gd32):
    """[(t_ns, potenza)] dai frame 0x71 mandati dal SoC."""
    out = []
    for t, direz, b in records_gd32:
        if direz != 1:
            continue
        i = 0
        while i + 6 <= len(b):
            if b[i] == 0xFA and b[i + 1] == 0xFB and b[i + 3] == 0x71 and b[i + 2] == 7 and i + 10 <= len(b):
                out.append((t, struct.unpack_from("<i", b, i + 4)[0]))
                i += 10
            else:
                i += 1
    return out


def main():
    lid = leggi_registro(sys.argv[1])
    pk = pacchetti_lidar(next(iter(lid.values())))
    pot = potenze(next(iter(leggi_registro(sys.argv[2]).values()))) if len(sys.argv) > 2 else []
    t0 = pk[0][0] if pk else 0
    print(f"{len(pk)} pacchetti validi; potenze comandate: {[p for _, p in pot]}")

    vel = [(t, p[0] * 0.05) for t, c, p in pk if c == 0xAE and p]
    # velocità anche dalle scansioni (param[0]): il lidar manda 0xAE solo quando non fa scansioni
    vel_sc = [(t, p[0] * 0.05) for t, c, p in pk if c == 0xAD and p]
    if pot:
        print("\npotenza  durata | 0xAE: n  media giri/s | scansioni: n  media  min  max giri/s   (esclusi i primi 3 s)")
        fine_cattura = pk[-1][0] + 1
        for k, (t, p) in enumerate(pot):
            fine = pot[k + 1][0] if k + 1 < len(pot) else fine_cattura
            v = [x for tt, x in vel if t + 3e9 <= tt < fine]
            w = [x for tt, x in vel_sc if t + 3e9 <= tt < fine]
            a = f"{len(v):4d}  {statistics.mean(v):5.2f}" if v else "   0     - "
            b = f"{len(w):5d}  {statistics.mean(w):5.2f} {min(w):5.2f} {max(w):5.2f}" if w else "    0"
            print(f"  {p:4d}  {(fine - t) / 1e9:5.1f} s | {a}        | {b}")
        prima = [x for tt, x in vel if tt < pot[0][0]]
        print(f"  prima della prima potenza: {len(prima)} pacchetti 0xAE, valori {sorted(set(prima))[:5]}")

    # scansioni: settori, giri, punti
    sc = []
    for t, c, p in pk:
        if c != 0xAD or len(p) < 5:
            continue
        v = p[0] * 0.05
        corr = struct.unpack(">h", p[1:3])[0] * 0.01
        inizio = struct.unpack(">H", p[3:5])[0] * 0.01
        punti = [(p[5 + 3 * k], (p[6 + 3 * k] << 8 | p[7 + 3 * k]) / 4000.0) for k in range((len(p) - 5) // 3)]
        sc.append((t, v, corr, inizio, punti))
    if not sc:
        print("nessuna scansione")
        return
    print(f"\n{len(sc)} settori di scansione; punti per settore: {sorted(set(len(s[4]) for s in sc))}")
    print(f"angoli di inizio (primi 20): {[round(s[3], 2) for s in sc[:20]]}")
    print(f"correzioni (valori distinti): {sorted(set(round(s[2], 2) for s in sc))[:20]}")
    giri, cur = [], []
    for s in sc:
        if cur and s[3] < cur[-1][3]:
            giri.append(cur)
            cur = []
        cur.append(s)
    giri = giri[1:]                       # il primo giro può essere parziale
    if giri:
        ps = [sum(len(s[4]) for s in g) for g in giri]
        ns = [len(g) for g in giri]
        durate = [(giri[k + 1][0][0] - giri[k][0][0]) / 1e9 for k in range(len(giri) - 1)]
        print(f"giri completi: {len(giri)}; settori per giro: {statistics.mode(ns)} (min {min(ns)}, max {max(ns)});"
              f" punti per giro: media {statistics.mean(ps):.0f} (min {min(ps)}, max {max(ps)})")
        if durate:
            print(f"durata di un giro: media {statistics.mean(durate) * 1000:.1f} ms -> {1 / statistics.mean(durate):.2f} giri/s")
        g = giri[len(giri) // 2]
        print("\ngiro di esempio (a metà cattura): angolo di inizio, punti, distanze min/max (m), intensità min/max")
        for s in g:
            d = [x for _, x in s[4] if x > 0]
            q = [x for x, _ in s[4]]
            print(f"  {s[3]:7.2f}°  {len(s[4]):3d} punti  {min(d) if d else 0:6.3f} {max(d) if d else 0:6.3f}   {min(q):3d} {max(q):3d}")
        tutti = [x for s in sc for _, x in s[4]]
        zeri = sum(1 for x in tutti if x == 0)
        print(f"\npunti totali {len(tutti)}, a distanza 0 (nessun eco) {zeri} ({100 * zeri / len(tutti):.1f}%)")


if __name__ == "__main__":
    main()
