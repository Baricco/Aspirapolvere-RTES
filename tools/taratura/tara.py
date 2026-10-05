#!/usr/bin/env python3
"""tara.py - Ricava i parametri del simulatore dalle registrazioni del robot e li scrive in sim/parametri.txt.

Tutto viene da registrazioni già fatte (fase 2, build/robot/fase2/), senza il robot:
  d         16 min, sensori a 50 Hz e lidar a 6 giri/s  -> ritardi di arrivo dei frame
  c2, c2b   gradini di velocità con 0x66 (robot SOLLEVATO) -> ritardo, rampa delle ruote, velocità misurata, correnti
  c8scarica 5 h fermo                                   -> deriva dell'IMU, inclinazione, byte "costanti"
  c1        lidar a potenze 40..80                       -> punti per giro, rumore, assenze di eco, risposta del motore
  c3, c3b   spazzole e aspirazione                       -> echi, correnti, giri della ventola
  c8scarica curva di scarica                             -> capacità utile
Ogni valore scritto porta la sua fonte. Il simulatore usa dei default uguali se il file manca.

Uso: tools/taratura/tara.py [cartella delle registrazioni, default build/robot/fase2] > sim/parametri.txt
     (le spiegazioni vanno su stderr)
"""
import csv, math, os, statistics as st, struct, sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from registri import (DAL_SOC, VERSO_SOC, frame_gd32, i16, i32, leggi_registro, pacchetti_lidar, u16)

DIR = sys.argv[1] if len(sys.argv) > 1 else "build/robot/fase2"
righe = []


def scrivi(chiave, valore, fonte):
    righe.append(f"{chiave} = {valore}    # {fonte}")


def info(*a):
    print(*a, file=sys.stderr)


def gd32(nome):
    return leggi_registro(f"{DIR}/{nome}/{nome}.reg")["gd32"]


# ---------------------------------------------------------------------------------------- arrivo dei frame ---
def tara_arrivi():
    s = [t for t, c, d in frame_gd32(gd32("d"), VERSO_SOC) if c == 0x15]
    n = len(s)
    mk, mt = (n - 1) / 2, st.mean(s)
    b = sum((k - mk) * (t - mt) for k, t in enumerate(s)) / sum((k - mk) ** 2 for k in range(n))
    a = mt - b * mk
    res = [t - (a + b * k) for k, t in enumerate(s)]
    m = min(res)
    rit = sorted((x - m) / 1e6 for x in res)
    probs = [0, .01, .05, .10, .25, .50, .75, .90, .95, .99, .999, .9999, 1]
    q = [rit[min(n - 1, int(p * (n - 1)))] for p in probs]
    # la sessione d è di spia guida con il lidar acceso; il nostro driver con il lidar (fase 3, r7) ha visto intervalli
    # fino a 25,1 ms: la coda si allunga fino a 5 ms
    q[-1] = max(q[-1], 5.0)
    info(f"arrivi: periodo del GD32 {b / 1e6:.5f} ms su {n} frame; ritardo (ms) {[round(x, 3) for x in q]}")
    scrivi("gd32_periodo_ms", f"{b / 1e6:.5f}", f"sessione d, {n} frame")
    scrivi("arrivo_probabilita", " ".join(str(p) for p in probs), "quantili della distribuzione qui sotto")
    scrivi("arrivo_ritardo_ms", " ".join(f"{x:.3f}" for x in q),
           "sessione d (spia guida, lidar acceso) + coda fino a 5 ms vista dal driver in fase 3 (r7)")


# ------------------------------------------------------------------------------------------------- ruote ---
def tara_ruote():
    """Rampe di v e di omega separate (unità di 0x66 = tick/s per ruota), dalla velocità delle ruote calcolata sugli
    encoder frame per frame. Il GD32 limita le due rampe in modo diverso (in rotazione frena molto più piano)."""
    rampe = {("v", 1): [], ("v", -1): [], ("w", 1): [], ("w", -1): []}
    ritardi, picchi, base = [], [], []
    for nome in ("c2", "c2b"):
        r = gd32(nome)
        tx = [(t, i32(d, 0), i32(d, 4)) for t, c, d in frame_gd32(r, DAL_SOC) if c == 0x66]
        rx = [(t, i32(d, 16), i32(d, 24), u16(d, 74), u16(d, 76)) for t, c, d in frame_gd32(r, VERSO_SOC) if c == 0x15]
        for i in range(1, len(tx)):
            if tx[i][1:] == tx[i - 1][1:]:
                continue
            tg = tx[i][0]
            fin = [x for x in rx if tg - 60e6 <= x[0] <= tg + 400e6]
            serie = []
            for a, b in zip(fin, fin[1:]):
                dt = (b[0] - a[0]) / 1e9
                sx, dx = (b[1] - a[1]) / dt, (b[2] - a[2]) / dt
                serie.append(((b[0] - tg) / 1e6, (sx + dx) / 2, (dx - sx) / 2, max(b[3], b[4])))
            for k, nome_g in ((1, "v"), (2, "w")):
                prima = st.mean(x[k] for x in serie if x[0] < 0)
                dopo = st.mean(x[k] for x in serie if x[0] > 250)
                salto = dopo - prima
                if abs(salto) < 300:
                    continue
                t10 = next(x[0] for x in serie if x[0] > 0 and (x[k] - prima) / salto >= 0.1)
                t90 = next(x[0] for x in serie if x[0] > 0 and (x[k] - prima) / salto >= 0.9)
                verso = 1 if abs(dopo) > abs(prima) else -1          # 1 = accelera, -1 = rallenta
                rampe[(nome_g, verso)].append(0.8 * abs(salto) / max((t90 - t10) / 1000, 0.02))
                ritardi.append(t10 - 20)
                picchi.append(max(x[3] for x in serie if 0 < x[0] < 120))
            base += [x[3] for x in serie if x[0] > 250 and abs(x[1]) + abs(x[2]) > 300]
    def med(k, alt):
        return st.median(rampe[k]) if rampe[k] else alt
    va, vd = med(("v", 1), 10000), med(("v", -1), 12000)
    wa, wd = med(("w", 1), va), med(("w", -1), 2500)
    info(f"ruote: ritardo {st.median(ritardi):.0f} ms; rampe (tick/s² per ruota) v su {va:.0f} giù {vd:.0f}, "
         f"omega su {wa:.0f} giù {wd:.0f} ({ {k: len(v) for k, v in rampe.items()} }); picco di corrente "
         f"{st.median(picchi):.0f} mA, a regime mediana {st.median(base):.0f} mA (90% {sorted(base)[int(.9 * len(base))]:.0f})")
    scrivi("ruote_ritardo_ms", f"{max(st.median(ritardi), 5):.0f}", "c2/c2b: dal gradino al 10% della risposta, meno un frame")
    scrivi("ruote_rampa_v", f"{va:.0f} {vd:.0f}", "c2/c2b: tick/s² per ruota, accelerando e rallentando, robot SOLLEVATO (a terra: ruote_tau_terra_s)")
    scrivi("ruote_rampa_w", f"{wa:.0f} {wd:.0f}", "c2/c2b: idem per la rotazione (il GD32 frena piano in rotazione)")
    scrivi("ruote_finestra_misura_ms", "70", "v e omega dei byte 32/36: passi di ~7 unità = 0,5 tick / 70 ms (c2)")
    scrivi("ruote_corrente_base_ma", f"{st.median(base):.0f}", "mediana in movimento, robot sollevato (c2)")
    scrivi("ruote_corrente_picchi_ma", f"{sorted(base)[int(.9 * len(base))]:.0f}", "90° percentile: picchi sparsi (c2)")
    scrivi("ruote_corrente_avvio_ma", f"{st.median(picchi):.0f}", "picco alla partenza (c2/c2b)")


def tara_ruote_terra():
    """val1 (fase 3, robot A TERRA, registrata dal driver): il GD32 risponde ai gradini di v e omega come un sistema
    del primo ordine (stesso tempo dal 10 al 90% per gradini piccoli e grandi), non con una rampa: tau = t(10-90) / ln 9.
    Le rampe di c2/c2b (sollevato) restano per il robot sollevato."""
    f = os.path.join(os.path.dirname(DIR), "fase3/val1/val1.reg")
    if not os.path.exists(f):
        info("ruote a terra: manca build/robot/fase3/val1/val1.reg, restano i valori di default")
        return
    r = leggi_registro(f)["gd32"]
    tx = [(t, i32(d, 0), i32(d, 4)) for t, c, d in frame_gd32(r, DAL_SOC) if c == 0x66]
    rx = [(t, i32(d, 16), i32(d, 24)) for t, c, d in frame_gd32(r, VERSO_SOC) if c == 0x15]
    tau = {"v": [], "w": []}
    for i in range(1, len(tx)):
        if tx[i][1:] == tx[i - 1][1:]:
            continue
        tg = tx[i][0]
        fin = [x for x in rx if tg - 60e6 <= x[0] <= tg + 600e6]
        serie = []
        for a, b in zip(fin, fin[1:]):
            dt = (b[0] - a[0]) / 1e9
            sx, dx = (b[1] - a[1]) / dt, (b[2] - a[2]) / dt
            serie.append(((b[0] - tg) / 1e6, (sx + dx) / 2, (dx - sx) / 2))
        for k, n in ((1, "v"), (2, "w")):
            prima = [x[k] for x in serie if x[0] < 0]
            dopo = [x[k] for x in serie if x[0] > 400]
            if not prima or not dopo or abs(st.mean(dopo) - st.mean(prima)) < 200:
                continue
            p0, salto = st.mean(prima), st.mean(dopo) - st.mean(prima)
            t10 = next(x[0] for x in serie if x[0] > 0 and (x[k] - p0) / salto >= 0.1)
            t90 = next(x[0] for x in serie if x[0] > 0 and (x[k] - p0) / salto >= 0.9)
            tau[n].append((t90 - t10) / 1000 / math.log(9))
    tv, tw = st.median(tau["v"]), st.median(tau["w"])
    info(f"ruote a terra: tau v {tv * 1000:.0f} ms ({len(tau['v'])} gradini), tau omega {tw * 1000:.0f} ms ({len(tau['w'])})")
    scrivi("ruote_tau_terra_s", f"{tv:.3f} {tw:.3f}", "val1 (fase 3, a terra): risposta del primo ordine di v e omega")


def tara_inerzia():
    """c2b: a un certo punto la prova smette di mandare l'heartbeat 0x06 con le ruote in moto. Dopo quanto il GD32 toglie
    potenza, e con che decelerazione le ruote si fermano da sole (inerzia, senza freno)."""
    r = gd32("c2b")
    hb = [t for t, c, d in frame_gd32(r, DAL_SOC) if c == 0x06]
    # il buco più lungo fra due heartbeat è quello della prova
    buchi = [(hb[i + 1] - hb[i], hb[i]) for i in range(len(hb) - 1)]
    _, t_hb = max(buchi)
    rx = [(t, i32(d, 16)) for t, c, d in frame_gd32(r, VERSO_SOC) if c == 0x15]
    vel = [((b[0] + a[0]) / 2, (b[1] - a[1]) / ((b[0] - a[0]) / 1e9)) for a, b in zip(rx, rx[1:])]
    prima = [v for t, v in vel if t_hb - 1e9 < t < t_hb]
    v0 = abs(sum(prima) / len(prima))
    dopo = [(t, v) for t, v in vel if t > t_hb]
    t_off = next(t for t, v in dopo if abs(v) < 0.8 * v0)          # inizia a rallentare
    enc_off = next(e for t, e in rx if t >= t_off - 30e6)
    fermo = next(t for t, v in dopo if t > t_off and abs(v) < 1)
    enc_fermo = next(e for t, e in rx if t >= fermo)
    tick = abs(enc_fermo - enc_off)
    dec = v0 * v0 / (2 * max(tick, 1))
    info(f"inerzia: velocità {v0:.0f} tick/s, potenza tolta {(t_off - t_hb) / 1e9:.2f} s dopo l'ultimo heartbeat, "
         f"poi {tick} tick fino a fermarsi in {(fermo - t_off) / 1e9:.2f} s: decelerazione {dec:.0f} tick/s²")
    scrivi("ruote_timeout_heartbeat_s", f"{(t_off - t_hb) / 1e9:.2f}", "c2b: senza 0x06 il GD32 toglie potenza ai motori dopo...")
    scrivi("ruote_inerzia", f"{dec:.0f}", "c2b: decelerazione delle ruote senza potenza (si fermano da sole), tick/s²")


# ---------------------------------------------------------------------------------- IMU e valori a riposo ---
def tara_riposo():
    r = list(csv.DictReader(open(f"{DIR}/c8scarica/c8scarica.csv")))
    # nella sessione C8 le colonne avevano i nomi vecchi: giro_* = velocità (byte 40/44/48), acc_* = angoli (42/46/50)
    fermi = [x for x in r if abs(float(x["giro_x"])) < 20 and abs(float(x["giro_y"])) < 20]
    t = [float(x["t_s"]) for x in fermi]
    ang = [float(x["acc_x"]) for x in fermi]
    mt, ma = st.mean(t), st.mean(ang)
    pend = sum((a - mt) * (b - ma) for a, b in zip(t, ang)) / sum((a - mt) ** 2 for a in t)   # 0,01° al secondo
    info(f"IMU: deriva dell'imbardata {pend * 3600 / 100:.2f} °/h, beccheggio {st.median(float(x['acc_y']) for x in fermi):.0f}, "
         f"rollio {st.median(float(x['acc_z']) for x in fermi):.0f} (0,01°), velocità a riposo dev {st.pstdev(float(x['giro_x']) for x in fermi):.2f}")
    scrivi("imu_deriva_gradi_s", f"{pend / 100:.6f}", "c8scarica: 5 h fermo, pendenza dell'angolo di imbardata")
    scrivi("imu_rumore_velocita", "0.5", "0,01 °/s: a riposo 0 o +-1 (c8scarica)")
    scrivi("imu_beccheggio", f"{st.median(float(x['acc_y']) for x in fermi):.0f}", "0,01°, robot fermo a terra (c8scarica)")
    scrivi("imu_rollio", f"{st.median(float(x['acc_z']) for x in fermi):.0f}", "0,01°, robot fermo a terra (c8scarica)")
    for col, chiave, fonte in (("c78", "corrente_riposo_laterale_ma", "byte 78"), ("c80", "corrente_riposo_centrale_ma", "byte 80"),
                               ("c82", "corrente_riposo_aspirazione_ma", "byte 82"), ("c84", "byte84_riposo", "byte 84")):
        scrivi(chiave, f"{st.median(float(x[col]) for x in fermi):.0f}", f"{fonte} a riposo (c8scarica)")
    scrivi("temperatura_c", f"{st.median(float(x['temperatura_c']) for x in r):.0f}", "c8scarica (32-34 °C)")
    # un frame vero a riposo, intero: i byte che il simulatore non calcola restano quelli veri
    rx = [d for t, c, d in frame_gd32(gd32("d"), VERSO_SOC) if c == 0x15 and len(d) == 96]
    tipico = bytes(st.mode(x[i] for x in rx[:2000]) for i in range(96))
    scrivi("frame_riposo", tipico.hex(), "byte più frequenti di 2000 frame veri (sessione d)")


# ------------------------------------------------------------------------------------------------- lidar ---
def tara_lidar():
    r = gd32("c1")
    rx = [t for t, c, d in frame_gd32(r, VERSO_SOC) if c == 0x15]
    pot = [(t, struct.unpack("<i", d)[0]) for t, c, d in frame_gd32(r, DAL_SOC) if c == 0x71]
    pk = pacchetti_lidar(leggi_registro(f"{DIR}/c1/c1-lidar.reg")["lidar"])
    vel = [(t, p[0] * 0.05) for t, c, p in pk if c in (0xAD, 0xAE) and p]
    # regime per potenza e costante di tempo (63%)
    regime, tau_su, tau_giu = [], [], []
    for i, (tp, p) in enumerate(pot):
        fine = pot[i + 1][0] if i + 1 < len(pot) else tp + 2e9
        seg = [(t - tp, v) for t, v in vel if tp <= t < fine]
        if len(seg) < 20 or not any(t > 3e9 for t, _ in seg):
            continue
        v0 = st.mean(v for t, v in vel if tp - 0.5e9 <= t < tp) if any(tp - 0.5e9 <= t < tp for t, _ in vel) else 0
        vf = st.mean(v for t, v in seg if t > 3e9)
        if p > 0:
            regime.append((p, vf))
        soglia = v0 + 0.63 * (vf - v0)
        for t, v in seg:
            if (vf > v0 and v >= soglia) or (vf < v0 and v <= soglia):
                (tau_su if vf > v0 else tau_giu).append(t / 1e9)
                break
    n = len(regime)
    mx, my = st.mean(p for p, _ in regime), st.mean(v for _, v in regime)
    a = sum((p - mx) * (v - my) for p, v in regime) / sum((p - mx) ** 2 for p, _ in regime)
    b = my - a * mx
    # spegnimento (potenza 0): giri/s nel tempo
    t0 = [tp for tp, p in pot if p == 0]
    giu0 = []
    if t0:
        seg = [(t - t0[0], v) for t, v in vel if t0[0] <= t]
        v0 = seg[0][1]
        for t, v in seg:
            if v <= v0 * 0.37:
                giu0.append(t / 1e9)
                break
    # punti per giro, rumore, buchi isolati
    giri, cur, ultimo = [], [], -1
    for t, c, p in pk:
        if c != 0xAD or len(p) < 5:
            continue
        a0 = p[3] << 8 | p[4]
        if a0 <= ultimo and cur:
            giri.append(cur)
            cur = []
        ultimo = a0
        k = (len(p) - 5) // 3
        for i in range(k):
            cur.append((p[0] * 0.05, a0 + i * 2250 // k, (p[6 + 3 * i] << 8 | p[7 + 3 * i]) / 4000.0, p[5 + 3 * i]))
    pps = [len(g) * st.mean(x[0] for x in g) for g in giri[1:] if g]
    rum = {}
    for g1, g2 in zip(giri[1:], giri[2:]):
        if len(g1) != len(g2):
            continue
        for x, y in zip(g1, g2):
            if x[2] > 0 and y[2] > 0 and x[1] == y[1] and abs(x[2] - y[2]) < 0.2:
                rum.setdefault(min(int(x[2]), 6), []).append(x[2] - y[2])
    sig = {k: st.pstdev(v) / math.sqrt(2) * 1000 for k, v in rum.items() if len(v) > 50}
    buchi, tot = 0, 0
    for g in giri[1:]:
        for j in range(1, len(g) - 1):
            if 0 < g[j - 1][2] < 2 and g[j + 1][2] > 0 and abs(g[j - 1][2] - g[j + 1][2]) < 0.1:
                tot += 1
                buchi += g[j][2] == 0
    inten = [x[3] for g in giri[1:] for x in g if x[2] > 0]
    dist = [x[2] for g in giri[1:] for x in g if x[2] > 0]
    info(f"lidar: giri/s = {a:.4f} x potenza + {b:.3f} ({n} potenze), tau salita {st.median(tau_su):.2f} s, "
         f"discesa {st.median(tau_giu) if tau_giu else 0:.2f} s, spegnimento {giu0[0] if giu0 else 0:.2f} s; "
         f"{st.mean(pps):.0f} misure/s; rumore (mm per m) {({k: round(v, 1) for k, v in sorted(sig.items())})}; "
         f"buchi isolati sotto 2 m {buchi / max(tot, 1):.1%}; intensità {st.mean(inten):.0f} +- {st.pstdev(inten):.0f}; "
         f"distanza massima {max(dist):.2f} m")
    scrivi("lidar_giri_per_potenza", f"{a:.4f} {b:.3f}", "c1: giri/s = a x potenza + b (regime a 40..80)")
    scrivi("lidar_tau_salita_s", f"{st.median(tau_su):.2f}", "c1: 63% del salto di velocità")
    scrivi("lidar_tau_discesa_s", f"{st.median(tau_giu) if tau_giu else 0.25:.2f}", "c1: 63% della discesa con potenza > 0")
    scrivi("lidar_tau_spento_s", f"{giu0[0] if giu0 else 0.5:.2f}", "c1: potenza 0, il rotore rallenta da solo")
    scrivi("lidar_misure_al_secondo", f"{st.mean(pps):.0f}", "c1: punti per giro x giri/s")
    scrivi("lidar_rumore_mm", " ".join(f"{sig.get(k, sig[max(sig)]):.1f}" for k in range(7)),
           "c1: deviazione standard per 0-1, 1-2, ... 6+ m (stesso angolo, giri consecutivi)")
    scrivi("lidar_buchi_isolati", f"{buchi / max(tot, 1):.4f}", "c1: punto senza eco fra due vicini validi, sotto 2 m")
    scrivi("lidar_intensita", f"{st.mean(inten):.0f} {st.pstdev(inten):.0f}", "c1: media e deviazione standard")
    scrivi("lidar_portata_m", f"{max(dist):.2f}", "c1: distanza massima misurata nella stanza")


# ---------------------------------------------------------------------------------- spazzole e aspirazione ---
def tara_attuatori():
    cen, asp, ven = {}, [], []
    for nome in ("c3", "c3b", "spazzola"):
        if os.path.exists(f"{DIR}/{nome}/{nome}.reg"):
            _attuatori(gd32(nome), cen, asp, ven)
    _scrivi_attuatori(cen, asp, ven)


def _attuatori(r, cen, asp, ven):
    tx = [(t, c, d) for t, c, d in frame_gd32(r, DAL_SOC) if c in (0x68, 0x69, 0x6a, 0x85, 0x86)]
    rx = [(t, d) for t, c, d in frame_gd32(r, VERSO_SOC) if c == 0x15]
    for i, (tc, c, d) in enumerate(tx):
        fine = tx[i + 1][0] if i + 1 < len(tx) else rx[-1][0]
        seg = [x for t, x in rx if tc + 1e9 <= t < fine]
        if len(seg) < 20:
            continue
        if c == 0x6a:
            v = struct.unpack("<b", d[:1])[0]
            cen[v] = (st.median(u16(x, 80) for x in seg[:150]), st.median(x[68] for x in seg[:150]))
        if c in (0x85, 0x86) and d[0]:
            asp.append((c, d[0], st.median(x[69] * 100 for x in seg), st.median(u16(x, 82) for x in seg)))
        if c == 0x68:
            g = struct.unpack("<h", d[:2])[0] * 10
            if g:
                ven.append((g, st.median(x[69] * 100 for x in seg), st.median(u16(x, 82) for x in seg)))


def _scrivi_attuatori(cen, asp, ven):
    info(f"spazzola centrale (comando: corrente mA, eco): {cen}; aspirazione PWM (cmd, %, giri, mA): {asp}; "
         f"aspirazione regolata (giri chiesti, misurati, mA): {ven}")
    pos = sorted((v, c) for v, (c, e) in cen.items() if v > 0)
    if pos:
        # quasi la stessa corrente a 30 e a 60: si usa un valore "accesa"
        scrivi("centrale_corrente_accesa_ma", f"{st.mean(c for _, c in pos):.0f}", f"c3/spazzola: comandi {[v for v, _ in pos]}")
    if any(v < 0 for v in cen):
        neg = [c for v, (c, e) in cen.items() if v < 0]
        scrivi("centrale_corrente_sblocco_ma", f"{st.median(neg):.0f}", "c3/spazzola: comando negativo (sblocco)")
    if asp:
        k = sum(g / p for _, p, g, _ in asp) / len(asp)
        scrivi("aspirazione_giri_per_pwm", f"{k:.0f}", "c3/c3b: giri/min per punto di PWM")
        mx = max(asp, key=lambda x: x[2])
        mn = min(asp, key=lambda x: x[2])
        esp = math.log((mx[3] - 14) / (mn[3] - 14)) / math.log(mx[2] / mn[2])
        scrivi("aspirazione_corrente_max", f"{mx[3]:.0f} {mx[2]:.0f}", "c3: corrente (mA) ai giri/min indicati")
        scrivi("aspirazione_esponente", f"{esp:.2f}", "c3: corrente - 14 mA proporzionale a giri^esponente")


# ---------------------------------------------------------------------------------------------- batteria ---
def tara_batteria():
    r = list(csv.DictReader(open(f"{DIR}/c8scarica/c8scarica.csv")))
    t = [float(x["t_s"]) for x in r]
    v = [int(x["tensione_dv"]) for x in r]
    i159 = next(i for i, x in enumerate(v) if x <= 159)
    i140 = next(i for i, x in enumerate(v) if x <= 140)
    ore = (t[i140] - t[i159]) / 3600
    corrente = 0.45                     # [?] SoC + lidar a 6 giri/s: il GD32 non misura la corrente
    utile = corrente * ore / (0.94 - 0.06)
    info(f"batteria: 15,9 -> 14,0 V in {ore:.2f} h; con {corrente} A stimati la capacità fra 16,3 e 13,8 V è {utile:.2f} Ah")
    scrivi("batteria_capacita_utile_ah", f"{utile:.2f}", f"c8scarica: 15,9->14,0 V in {ore:.2f} h a ~{corrente} A stimati [?]")
    scrivi("batteria_capacita_totale_ah", "3.2", "scheda tecnica (3200 mAh): sotto 13,8 V c'è la riserva fino alla protezione")


def main():
    print("# Parametri del simulatore ricavati dalle registrazioni del robot (tools/taratura/tara.py).")
    print("# Formato: chiave = valore    # fonte. Il simulatore li legge all'avvio (sim/parametri.c).")
    tara_arrivi()
    tara_ruote()
    tara_ruote_terra()
    tara_inerzia()
    tara_riposo()
    tara_lidar()
    tara_attuatori()
    tara_batteria()
    print("\n".join(righe))


if __name__ == "__main__":
    main()
