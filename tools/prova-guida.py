#!/usr/bin/env python3
"""prova-guida.py - Prova del banco di prova "spia guida" sul PC, con un GD32 finto (fase 2, blocco B).

Il GD32 finto è questo script: tiene il lato master di una pty, manda il frame dei sensori 0x15 ogni 20 ms
(byte 3 = anti-caduta, byte 8 = tensione) e legge tutto quello che spia guida manda. Ogni scenario controlla
le regole di sicurezza di gd32/sicurezza.h:

  1. comando a un motore senza --sollevato: rifiutato alla lettura delle opzioni (codice 2);
  2. pompa dell'acqua: vietata (codice 2);
  3. robot appoggiato (byte 3 = 0x00) con --sollevato: nessun frame ai motori diverso da zero; keepalive 0x0d
     e heartbeat 0x06 regolari;
  4. sollevato, poi il byte 3 cambia: arresto (frame dei motori a zero) entro un frame, poi nessun comando ai
     motori diverso da zero, codice 4;
  5. sollevato, poi i sensori smettono di arrivare: arresto entro ~100 ms;
  6. SIGTERM durante la prova: frame di arresto prima di uscire;
  7. sequenza da file con ogni / basta / fine;
  8. tensione sotto --tensione-minima: uscita con codice 3;
  9. comandi dal vivo con --comandi FIFO (e i vietati rifiutati).

Uso: tools/prova-guida.py [build/host/spia]
"""
import os, select, signal, subprocess, sys, tempfile, time, tty

SPIA = sys.argv[1] if len(sys.argv) > 1 else "build/host/spia"
MOTORI = {0x66, 0x67, 0x68, 0x69, 0x6A, 0x84, 0x85, 0x86}
falliti = 0


def ck_gd32(b):
    ck = 0
    for i in range(0, len(b) - 1, 2):
        ck = (ck + (b[i] << 8 | b[i + 1])) & 0xFFFF
    if len(b) % 2:
        ck ^= b[-1]
    return ck


def frame_gd32(cmd, dati=b""):
    corpo = bytes([cmd]) + dati
    ck = ck_gd32(corpo)
    return bytes([0xFA, 0xFB, len(corpo) + 2]) + corpo + bytes([ck >> 8, ck & 0xFF])


def sensori(cliff, tensione_dv=162):
    d = bytearray(96)
    d[0], d[3], d[8], d[9], d[64] = 1, cliff, tensione_dv, 31, 0x62   # 0x62: sensori alimentati
    return frame_gd32(0x15, bytes(d))


def deframe(buf):
    """Frame validi (t, cmd, dati) da una lista di (t, byte)."""
    out, b, i = [], bytes(x for _, x in buf), 0
    tempi = [t for t, _ in buf]
    while i + 6 <= len(b):
        if b[i] != 0xFA or b[i + 1] != 0xFB:
            i += 1
            continue
        n = b[i + 2]
        if n < 3 or i + 3 + n > len(b):
            i += 1
            continue
        corpo = b[i + 3:i + 3 + n - 2]
        if ck_gd32(corpo) == (b[i + 1 + n] << 8 | b[i + 2 + n]):
            out.append((tempi[i + 2 + n], corpo[0], corpo[1:]))
            i += 3 + n
        else:
            i += 1
    return out


def controlla(cond, testo):
    global falliti
    print(f"  {'ok' if cond else 'FALLITO'}: {testo}")
    if not cond:
        falliti += 1


def esegui(args, cliff_al_tempo, durata, segnale_a=None):
    """Avvia spia guida contro il GD32 finto. cliff_al_tempo(t) -> byte 3, oppure None = non mandare sensori,
    oppure (cliff, tensione). Ritorna (codice, frame ricevuti dal GD32 finto, stdout)."""
    m, s = os.openpty()
    tty.setraw(s)
    os.set_blocking(m, False)
    disp = os.ttyname(s)
    p = subprocess.Popen([SPIA, "guida", "-d", disp, "--forza"] + args, stdout=subprocess.PIPE,
                         stderr=subprocess.STDOUT, text=True)
    t0 = time.monotonic()
    prossimo = t0
    ricevuti = []
    mandato_segnale = False
    while p.poll() is None and time.monotonic() - t0 < durata + 2:
        adesso = time.monotonic()
        if adesso >= prossimo:
            v = cliff_al_tempo(adesso - t0)
            if v is not None:
                cliff, tens = v if isinstance(v, tuple) else (v, 162)
                try:
                    os.write(m, sensori(cliff, tens))
                except BlockingIOError:
                    pass
            prossimo += 0.020
        if segnale_a is not None and not mandato_segnale and adesso - t0 >= segnale_a:
            p.send_signal(signal.SIGTERM)
            mandato_segnale = True
        r, _, _ = select.select([m], [], [], 0.005)
        if r:
            try:
                for x in os.read(m, 4096):
                    ricevuti.append((time.monotonic() - t0, x))
            except OSError:
                pass
    if p.poll() is None:
        p.kill()
    out = p.communicate()[0]
    # gli ultimi byte (l'arresto all'uscita)
    try:
        while True:
            r, _, _ = select.select([m], [], [], 0.05)
            if not r:
                break
            for x in os.read(m, 4096):
                ricevuti.append((time.monotonic() - t0, x))
    except OSError:
        pass
    os.close(m)
    os.close(s)
    return p.returncode, deframe(ricevuti), out


def motori_non_zero(frames, dopo=0.0):
    return [f for f in frames if f[0] >= dopo and f[1] in MOTORI and any(f[2])]


def arresti(frames):
    """Tempi in cui è arrivato un 0x66 a zero (parte di ogni sequenza di arresto)."""
    return [t for t, c, d in frames if c == 0x66 and not any(d)]


def main():
    print("1. motore senza --sollevato")
    p = subprocess.run([SPIA, "guida", "-d", "/dev/null", "--forza", "--invia", "0x66:i32=1000,i32=0@20"],
                       capture_output=True, text=True)
    controlla(p.returncode == 2 and "sollevato" in p.stderr, "rifiutato con codice 2")

    print("2. pompa dell'acqua")
    p = subprocess.run([SPIA, "guida", "-d", "/dev/null", "--forza", "--sollevato", "--una", "0x6b:00"],
                       capture_output=True, text=True)
    controlla(p.returncode == 2 and "vietato" in p.stderr, "vietata con codice 2")

    print("3. robot appoggiato")
    cod, fr, out = esegui(["--sollevato", "--invia", "0x66:i32=1000,i32=0@20", "--una", "0x69:i8=40",
                           "--durata", "4"], lambda t: 0x00, 4)
    controlla(cod == 0, f"codice 0 (era {cod})")
    controlla(not motori_non_zero(fr), "nessun comando ai motori diverso da zero")
    ka = [t for t, c, _ in fr if c == 0x0D]
    hb = [t for t, c, _ in fr if c == 0x06]
    controlla(len(ka) == 2, f"keepalive 0x0d ogni 3 s ({len(ka)} in 4 s)")
    controlla(18 <= len(hb) <= 22, f"heartbeat 0x06 ogni 200 ms ({len(hb)} in 4 s)")
    controlla("RIFIUTATO" in out, "il rifiuto è scritto in uscita")

    print("4. sollevato, poi un anti-caduta vede il pavimento")
    cod, fr, out = esegui(["--sollevato", "--invia", "0x66:i32=1000,i32=0@20", "--invia", "0x69:i8=40@100",
                           "--durata", "3"], lambda t: 0x0F if t < 1.5 else 0x0E, 3)
    prima = [f for f in motori_non_zero(fr) if f[0] < 1.5]
    controlla(len(prima) > 50, f"comandi ai motori mentre è sollevato ({len(prima)})")
    ar = [t for t in arresti(fr) if t >= 1.5]
    controlla(bool(ar) and ar[0] < 1.5 + 0.06, f"arresto subito dopo il cambio ({ar[0] - 1.5:.3f} s)" if ar else "arresto")
    ultimo_motore = max(f[0] for f in motori_non_zero(fr))
    controlla(ultimo_motore < 1.5 + 0.06, f"nessun comando ai motori dopo il cambio (ultimo a {ultimo_motore:.3f} s)")
    controlla(cod == 4 and "ARRESTO" in out, f"codice 4 e ARRESTO scritto (codice {cod})")

    print("5. sollevato, poi i sensori smettono di arrivare")
    cod, fr, out = esegui(["--sollevato", "--invia", "0x66:i32=1000,i32=0@20", "--durata", "2"],
                          lambda t: 0x0F if t < 1.0 else None, 2)
    ar = [t for t in arresti(fr) if t >= 1.0]
    controlla(bool(ar) and ar[0] < 1.0 + 0.2, f"arresto entro 200 ms ({ar[0] - 1.0:.3f} s)" if ar else "arresto")
    controlla(not motori_non_zero(fr, 1.0 + 0.15), "nessun comando ai motori dopo l'arresto")

    print("6. SIGTERM durante la prova")
    cod, fr, out = esegui(["--sollevato", "--invia", "0x66:i32=1000,i32=0@20", "--durata", "10"],
                          lambda t: 0x0F, 10, segnale_a=1.0)
    controlla(cod == 0, f"esce con codice 0 (era {cod})")
    ultimo = fr[-6:] if len(fr) >= 6 else []
    controlla(len(ultimo) == 6 and all(c in MOTORI and not any(d) for _, c, d in ultimo),
              "gli ultimi frame sono l'arresto di tutti i motori")

    print("7. sequenza da file")
    with tempfile.NamedTemporaryFile("w", suffix=".seq", delete=False) as f:
        f.write("# prova\n0 ogni 20 0x66:i32=500,i32=0\n300 0x69:i8=30\n1000 basta 0x66\n1500 fine\n")
        seq = f.name
    cod, fr, out = esegui(["--sollevato", "--sequenza", seq], lambda t: 0x0F, 3)
    os.unlink(seq)
    v = [t for t, c, d in fr if c == 0x66 and any(d)]
    controlla(len(v) > 40 and max(v) < 1.05, f"0x66 da 0 a 1 s ({len(v)} frame, ultimo a {max(v) if v else 0:.3f} s)")
    sp = [(t, d) for t, c, d in fr if c == 0x69 and any(d)]
    controlla(len(sp) == 1 and 0.28 < sp[0][0] < 0.4 and sp[0][1] == bytes([30]), "0x69 = 30 una volta a 300 ms")
    fine = arresti(fr)
    controlla(cod == 0 and fine and 1.45 < fine[-1] < 1.7, f"esce a 1,5 s con l'arresto (codice {cod})")

    print("8. tensione minima")
    cod, fr, out = esegui(["--tensione-minima", "125", "--durata", "5"], lambda t: (0x00, 120), 5)
    controlla(cod == 3 and "sotto la minima" in out, f"codice 3 (era {cod})")

    print("9. comandi dal vivo (--comandi FIFO)")
    fifo = tempfile.mktemp(suffix=".fifo")
    import threading
    def scrivi():
        time.sleep(1.0)
        with open(fifo, "w") as f:
            f.write("0x8d:05\n0x6b:00\n")
    threading.Thread(target=scrivi, daemon=True).start()
    cod, fr, out = esegui(["--comandi", fifo, "--durata", "2"], lambda t: 0x00, 2)
    led = [(t, d) for t, c, d in fr if c == 0x8D]
    controlla(len(led) == 1 and led[0][1] == bytes([5]) and 0.9 < led[0][0] < 1.3, "0x8d 05 mandato quando scritto")
    controlla(not [1 for _, c, _ in fr if c == 0x6B] and "non valido" in out, "pompa rifiutata anche dal vivo")
    if os.path.exists(fifo):
        os.unlink(fifo)

    print("PROVA GUIDA: " + ("OK" if falliti == 0 else f"{falliti} CONTROLLI FALLITI"))
    sys.exit(1 if falliti else 0)


if __name__ == "__main__":
    main()
