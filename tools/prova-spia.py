#!/usr/bin/env python3
"""prova-spia.py - Prova completa di "spia proxy" sul PC, senza robot e senza root.

Simula le due periferiche e AuxCtrl con delle pty:

    "AuxCtrl" (questo script) <-> pty della spia <-> spia <-> pty "vera" <-> GD32 e lidar finti (questo script)

  - il GD32 finto risponde all'handshake di AuxCtrl e manda il frame dei sensori 0x15 ogni 20 ms;
  - il lidar finto manda il testo di avvio in GBK, il pacchetto di versione e pacchetti di scansione;
  - "AuxCtrl" manda l'handshake, l'heartbeat 0x06 ogni 200 ms e la velocità 0x66 ogni 20 ms.

Alla fine controlla che:
  1. ogni lato abbia ricevuto esattamente i byte spediti dall'altro (niente persi, niente aggiunti);
  2. il registro contenga gli stessi byte, per canale e per direzione;
  3. "spia riepilogo" conti i frame giusti, con 0 errori.

Uso: tools/prova-spia.py [build/host/spia] [durata_s]
"""
import os, re, select, signal, struct, subprocess, sys, tempfile, termios, time, tty

SPIA = sys.argv[1] if len(sys.argv) > 1 else "build/host/spia"
DURATA = float(sys.argv[2]) if len(sys.argv) > 2 else 3.0


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


def pacchetto_lidar(cmd, param, indirizzo=1, tipo=0x61):
    p = bytes([0xAA, 0, 8 + len(param), indirizzo, tipo, cmd, len(param) >> 8, len(param) & 0xFF]) + param
    if indirizzo == 1:
        ck = sum(p) & 0xFFFF
    else:
        ck = 0xFFFF
        for x in p:
            ck ^= x
            for _ in range(8):
                ck = (ck >> 1) ^ 0xA001 if ck & 1 else ck >> 1
    return p + bytes([ck >> 8, ck & 0xFF])


def pty_raw():
    m, s = os.openpty()
    tty.setraw(s)
    os.set_blocking(m, False)
    return m, s, os.ttyname(s)


def leggi_registro(percorso):
    with open(percorso, "rb") as f:
        d = f.read()
    assert d.startswith(b"SPIA1\n"), "intestazione sbagliata"
    i = 6
    n = d[i]; i += 1
    nomi = {}
    for _ in range(n):
        cid, proto = d[i], d[i + 1]; i += 2
        l = d[i]; i += 1; nome = d[i:i + l].decode(); i += l
        l = d[i]; i += 1 + l
        nomi[cid] = nome
    flussi = {}
    while i < len(d):
        t, c, direz, k = struct.unpack_from("<qBBH", d, i); i += 12
        flussi.setdefault((nomi[c], direz), bytearray()).extend(d[i:i + k]); i += k
    return flussi


def main():
    # le periferiche "vere": la spia apre lo slave, il nostro finto dispositivo usa il master
    g_m, g_s, g_nome = pty_raw()
    l_m, l_s, l_nome = pty_raw()
    reg = tempfile.NamedTemporaryFile(suffix=".spia", delete=False).name
    extra = os.environ.get("SPIA_ARGS", "").split()
    spia = subprocess.Popen([SPIA, "proxy", "-o", reg, "--senza-mount", "-p", "0", "-s", "0", *extra,
                             "-c", f"gd32,gd32,{g_nome}", "-c", f"lidar,lidar,{l_nome}"],
                            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    pty_spia = {}
    for _ in range(2):
        riga = spia.stdout.readline()
        m = re.match(r"canale (\w+): .* \(pty (\S+)\)", riga)
        assert m, f"riga inattesa dalla spia: {riga!r} {spia.stderr.read() if spia.poll() is not None else ''}"
        pty_spia[m.group(1)] = m.group(2)

    # "AuxCtrl" apre i dispositivi finti della spia, come farebbe con /dev/ttyS3 e /dev/ttyS1
    aux_g = os.open(pty_spia["gd32"], os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    aux_l = os.open(pty_spia["lidar"], os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    for fd in (aux_g, aux_l):
        tty.setraw(fd)

    spedito = {("gd32", 0): bytearray(), ("gd32", 1): bytearray(), ("lidar", 0): bytearray()}
    ricevuto = {("gd32", 0): bytearray(), ("gd32", 1): bytearray(), ("lidar", 0): bytearray()}
    attesi = {"gd32_tx": {}, "gd32_rx": {}, "lidar_rx": {}}

    def manda(fd, chiave, dati, conta=None, cmd=None):
        os.write(fd, dati)
        spedito[chiave].extend(dati)
        if conta is not None:
            conta[cmd] = conta.get(cmd, 0) + 1

    # avvio del lidar: testo GBK + versione
    testo = "\r\n深圳市杉川机器人科技有限公司\r\n固件版本:LDS3_NOR\r\n".encode("gbk")
    manda(l_m, ("lidar", 0), testo)
    manda(l_m, ("lidar", 0), pacchetto_lidar(0xAC, b"2030313958425713002F0024LDS3_NOR_1805041536"),
          attesi["lidar_rx"], 0xAC)
    # handshake di AuxCtrl
    for cmd, dati in [(0x0c, b"\x01"), (0x07, b""), (0x17, b"\x01\x80\xcf\xf0"), (0x18, b""),
                      (0x71, b"\0\0\0\0"), (0x65, b"\0"), (0x19, b"\x01"), (0x7c, b"\0\xff\xff")]:
        manda(aux_g, ("gd32", 1), frame_gd32(cmd, dati), attesi["gd32_tx"], cmd)

    sensori = bytearray(96)
    t0 = time.monotonic()
    prossimo = {"hb": t0, "vel": t0, "sens": t0, "scan": t0}
    k = 0
    while time.monotonic() - t0 < DURATA:
        ora = time.monotonic()
        if ora >= prossimo["hb"]:
            manda(aux_g, ("gd32", 1), frame_gd32(0x06), attesi["gd32_tx"], 0x06); prossimo["hb"] += 0.2
        if ora >= prossimo["vel"]:
            manda(aux_g, ("gd32", 1), frame_gd32(0x66, struct.pack("<ii", -7, k)), attesi["gd32_tx"], 0x66)
            prossimo["vel"] += 0.02
        if ora >= prossimo["sens"]:
            sensori[8] = 0x7e; sensori[16:20] = struct.pack("<i", k)
            sensori[30] = 0xFA; sensori[31] = 0xFB   # un "FA FB" dentro i dati
            manda(g_m, ("gd32", 0), frame_gd32(0x15, bytes(sensori)), attesi["gd32_rx"], 0x15)
            prossimo["sens"] += 0.02; k += 1
        if ora >= prossimo["scan"]:
            param = bytes((k * 7 + j) & 0xFF for j in range(80))
            manda(l_m, ("lidar", 0), pacchetto_lidar(0xAD, param, indirizzo=2), attesi["lidar_rx"], 0xAD)
            prossimo["scan"] += 0.01
        # legge tutto quello che arriva
        for fd, chiave in ((aux_g, ("gd32", 0)), (g_m, ("gd32", 1)), (aux_l, ("lidar", 0))):
            try:
                ricevuto[chiave].extend(os.read(fd, 65536))
            except BlockingIOError:
                pass
        time.sleep(0.001)

    # svuota quello che è ancora in viaggio
    fine = time.monotonic() + 0.5
    while time.monotonic() < fine:
        for fd, chiave in ((aux_g, ("gd32", 0)), (g_m, ("gd32", 1)), (aux_l, ("lidar", 0))):
            try:
                ricevuto[chiave].extend(os.read(fd, 65536))
            except BlockingIOError:
                pass
        time.sleep(0.01)

    spia.send_signal(signal.SIGTERM)
    _, err = spia.communicate(timeout=5)
    ok = True

    def controlla(cond, msg):
        nonlocal ok
        print(("ok      " if cond else "FALLITO ") + msg)
        ok &= bool(cond)

    controlla(spia.returncode == 0, f"la spia termina con codice 0 (codice {spia.returncode}) {err.strip()[-300:]}")
    for chiave in spedito:
        controlla(ricevuto[chiave] == spedito[chiave],
                  f"{chiave[0]} {'periferica->SoC' if chiave[1] == 0 else 'SoC->periferica'}: "
                  f"{len(spedito[chiave])} byte spediti, {len(ricevuto[chiave])} ricevuti, identici")
    flussi = leggi_registro(reg)
    for chiave in spedito:
        controlla(bytes(flussi.get(chiave, b"")) == bytes(spedito[chiave]), f"registro {chiave}: stessi byte")
    controlla(("lidar", 1) not in flussi, "registro: nessun byte dal SoC al lidar")

    riep = subprocess.run([SPIA, "riepilogo", reg], capture_output=True, text=True).stdout
    print(riep)

    def frame_contati(canale, verso, cmd):
        blocco = riep.split(f"== {canale} ")
        for b in blocco[1:]:
            if verso in b.splitlines()[0]:
                m = re.search(rf"^\s+0x{cmd:02x} \S+\s+(\d+)", b, re.M)
                return int(m.group(1)) if m else 0
        return -1

    for cmd, n in attesi["gd32_tx"].items():
        controlla(frame_contati("gd32", "SoC -> periferica", cmd) == n, f"riepilogo gd32 tx 0x{cmd:02x}: {n} frame")
    for cmd, n in attesi["gd32_rx"].items():
        controlla(frame_contati("gd32", "periferica -> SoC", cmd) == n, f"riepilogo gd32 rx 0x{cmd:02x}: {n} frame")
    for cmd, n in attesi["lidar_rx"].items():
        controlla(frame_contati("lidar", "periferica -> SoC", cmd) == n, f"riepilogo lidar rx 0x{cmd:02x}: {n} pacchetti")
    controlla(" 0 errori" in riep and "errori" in riep and not re.search(r" [1-9]\d* errori", riep),
              "riepilogo: 0 errori di checksum/lunghezza")

    if os.environ.get("TIENI_REGISTRO"):
        print("registro:", reg)
    else:
        os.unlink(reg)
    print("PROVA SPIA:", "OK" if ok else "FALLITA")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
