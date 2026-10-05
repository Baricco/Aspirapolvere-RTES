"""registri.py - Funzioni comuni della taratura: leggere i registri della spia (formato di spia/registro.h) e
ricavarne i frame del GD32 e i pacchetti del lidar, ognuno con l'istante in cui è arrivato.

Direzioni (viste dal SoC): VERSO_SOC = 0 (dal GD32 o dal lidar), DAL_SOC = 1 (comandi del SoC).
"""
import gzip, struct

VERSO_SOC, DAL_SOC = 0, 1


def leggi_registro(percorso):
    """{nome_canale: [(t_ns, direzione, byte)]} dal file della spia (anche .gz)."""
    apri = gzip.open if percorso.endswith(".gz") else open
    d = apri(percorso, "rb").read()
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


def ck_gd32(b):
    ck = 0
    for i in range(0, len(b) - 1, 2):
        ck = (ck + (b[i] << 8 | b[i + 1])) & 0xFFFF
    if len(b) % 2:
        ck ^= b[-1]
    return ck


def frame_gd32(record, direzione):
    """[(t_ns, cmd, dati)] dei frame validi in una direzione; t = arrivo del blocco che completa il frame."""
    buf, tempi, out = bytearray(), [], []
    for t, dz, b in record:
        if dz != direzione:
            continue
        buf += b
        tempi += [t] * len(b)
        i = 0
        while i + 6 <= len(buf):
            if buf[i] != 0xFA or buf[i + 1] != 0xFB:
                i += 1
                continue
            n = buf[i + 2]
            if n < 3:
                i += 1
                continue
            if i + 3 + n > len(buf):
                break
            corpo = bytes(buf[i + 3:i + 1 + n])
            if ck_gd32(corpo) == (buf[i + 1 + n] << 8 | buf[i + 2 + n]):
                out.append((tempi[i + 2 + n], corpo[0], corpo[1:]))
                i += 3 + n
            else:
                i += 1
        del buf[:i]
        del tempi[:i]
    return out


def crc16(b):
    ck = 0xFFFF
    for x in b:
        ck ^= x
        for _ in range(8):
            ck = (ck >> 1) ^ 0xA001 if ck & 1 else ck >> 1
    return ck


def pacchetti_lidar(record):
    """[(t_ns, cmd, param)] dei pacchetti validi del lidar."""
    buf, tempi, out = bytearray(), [], []
    for t, dz, b in record:
        if dz != VERSO_SOC:
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


def i16(b, o): return struct.unpack_from("<h", b, o)[0]
def u16(b, o): return struct.unpack_from("<H", b, o)[0]
def i32(b, o): return struct.unpack_from("<i", b, o)[0]
