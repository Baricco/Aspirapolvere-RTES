#!/usr/bin/env python3
"""
estrai-frame.py - Ricostruisce i frame del protocollo GD32 da una traccia strace di AuxCtrl.

La traccia viene da Script/stream-strace.sh (o da strace -f -tt -x a mano). AuxCtrl
parla con il GD32 su /dev/ttyS3: legge un byte alla volta e scrive frame interi.
Lo script:
  1. legge la mappa dei file descriptor (il file <segmento>-fd.txt), per sapere
     quali thread sono di AuxCtrl e quale fd è /dev/ttyS3;
  2. ricompone le chiamate read/write spezzate da strace ("<unfinished ...>" e
     "<... resumed>") e mette in fila i byte, separati per direzione;
  3. taglia i flussi in frame  FA FB len cmd dati... ck_hi ck_lo
     dove len conta cmd + dati + checksum. Il checksum (CRobotPacket::calcCheckSum
     in AuxCtrl, 0x55cf0) è la somma modulo 2^16 delle parole big-endian da 16 bit
     di cmd + dati; se i byte sono dispari, l'ultimo viene messo in XOR (non sommato)
     come byte basso;
  4. stampa un catalogo: per ogni direzione e codice cmd, quanti frame, quale
     lunghezza, ogni quanto arrivano, quanti checksum sono sbagliati e alcuni
     esempi di contenuto (i byte che cambiano sono quelli interessanti).

Uso:
    tools/estrai-frame.py ../Log/cattura-stream/20261002-000030.txt.gz
    tools/estrai-frame.py --fd ../Log/cattura-ricarica/cattura/fd.txt ../Log/cattura-ricarica/cattura/aux.txt
    tools/estrai-frame.py --dev ttyS1 ...      (stesso lavoro sul lidar: qui il framing è diverso)
    tools/estrai-frame.py --csv frame.csv ...  (tutti i frame, uno per riga, per analisi successive)
"""
import argparse
import collections
import csv
import gzip
import os
import re
import statistics
import sys

# "[pid  687] 06:00:33.495530 read(..." (strace -f) oppure "687   05:45:36.428110 read(..." (strace -ff -o)
RIGA = re.compile(r'^(?:\[pid\s+(\d+)\]|(\d+))?\s*(\d\d):(\d\d):(\d\d\.\d+)\s+(.*)$')
CHIAMATA = re.compile(r'^(read|write)\((\d+),\s*(.*)$')
RIPRESA = re.compile(r'^<\.\.\. (read|write) resumed>\s*(.*)$')
STRINGA = re.compile(r'"((?:[^"\\]|\\.)*)"')
RISULTATO = re.compile(r'\)\s*=\s*(-?\d+)')


def apri(percorso):
    return gzip.open(percorso, 'rt', errors='replace') if percorso.endswith('.gz') else open(percorso, errors='replace')


def leggi_mappa_fd(percorso, processo, dev):
    """Restituisce (insieme dei thread del processo, fd del dispositivo)."""
    thread, fd, attuale = set(), None, None
    with open(percorso) as f:
        for riga in f:
            m = re.match(r'^== (\d+) (\S+)', riga)
            if m:
                attuale = m.group(2)
                if attuale == processo:
                    thread.add(int(m.group(1)))
                continue
            if attuale != processo:
                continue
            m = re.search(r'\s(\d+) -> /dev/' + re.escape(dev) + r'\s*$', riga)
            if m:
                fd = int(m.group(1))
            elif riga.startswith('thread:'):
                thread.update(int(t) for t in riga.split()[1:])
            elif re.fullmatch(r'\d+\s*', riga):      # vecchio formato: un thread per riga
                thread.add(int(riga))
    return thread, fd


def decodifica(s):
    """Stringa di strace -x ("\\xfa\\xfbi...") in bytes."""
    out, i = bytearray(), 0
    escape = {'n': 10, 't': 9, 'r': 13, 'v': 11, 'f': 12, '\\': 92, '"': 34, '0': 0}
    while i < len(s):
        c = s[i]
        if c == '\\':
            n = s[i + 1]
            if n == 'x':
                out.append(int(s[i + 2:i + 4], 16)); i += 4; continue
            if n.isdigit():                         # ottale \NNN
                m = re.match(r'[0-7]{1,3}', s[i + 1:])
                out.append(int(m.group(0), 8)); i += 1 + len(m.group(0)); continue
            out.append(escape.get(n, ord(n))); i += 2; continue
        out.append(ord(c)); i += 1
    return bytes(out)


def eventi(percorso, thread, fd_dev):
    """Genera (tempo_s, 'rx'|'tx', bytes) per i read/write del processo sul dispositivo."""
    in_sospeso = {}     # pid -> (chiamata, fd, dati già visti nella prima metà)
    giorno = 0.0
    prec = None
    with apri(percorso) as f:
        for riga in f:
            m = RIGA.match(riga.rstrip('\n'))
            if not m:
                continue
            pid = int(m.group(1) or m.group(2) or 0)
            if thread and pid not in thread:
                continue
            t = int(m.group(3)) * 3600 + int(m.group(4)) * 60 + float(m.group(5))
            if prec is not None and t + giorno < prec - 43200:   # passaggio dalla mezzanotte
                giorno += 86400
            t += giorno
            prec = t
            resto = m.group(6)

            c = CHIAMATA.match(resto)
            if c:
                nome, fd, coda = c.group(1), int(c.group(2)), c.group(3)
                if fd != fd_dev:
                    continue
                if coda.endswith('<unfinished ...>'):
                    s = STRINGA.search(coda)
                    in_sospeso[pid] = (nome, s.group(1) if (s and nome == 'write') else None)
                    continue
                s, r = STRINGA.search(coda), RISULTATO.search(coda)
                if s and r and int(r.group(1)) > 0:
                    dati = decodifica(s.group(1))[:int(r.group(1))]
                    yield t, ('rx' if nome == 'read' else 'tx'), dati
                continue

            p = RIPRESA.match(resto)
            if p and pid in in_sospeso:
                nome, dati_write = in_sospeso.pop(pid)
                if nome != p.group(1):
                    continue
                r = RISULTATO.search(p.group(2))
                if not r or int(r.group(1)) <= 0:
                    continue
                n = int(r.group(1))
                if nome == 'read':
                    s = STRINGA.search(p.group(2))
                    if s:
                        yield t, 'rx', decodifica(s.group(1))[:n]
                elif dati_write is not None:
                    yield t, 'tx', decodifica(dati_write)[:n]


def checksum(corpo):
    """Come CRobotPacket::calcCheckSum: somma delle parole big-endian, ultimo byte dispari in XOR."""
    somma = 0
    for i in range(0, len(corpo) - 1, 2):
        somma = (somma + (corpo[i] << 8 | corpo[i + 1])) & 0xFFFF
    if len(corpo) % 2:
        somma ^= corpo[-1]
    return somma


class Deframer:
    """Taglia un flusso di byte in frame FA FB len corpo[len]."""

    def __init__(self):
        self.buf = bytearray()
        self.t0 = None
        self.scartati = 0

    def aggiungi(self, t, dati):
        if not self.buf:
            self.t0 = t
        self.buf += dati
        while True:
            i = self.buf.find(b'\xfa\xfb')
            if i < 0:
                tieni = 1 if self.buf.endswith(b'\xfa') else 0
                self.scartati += len(self.buf) - tieni
                del self.buf[:len(self.buf) - tieni]
                return
            if i > 0:
                self.scartati += i
                del self.buf[:i]
                self.t0 = t
            if len(self.buf) < 3:
                return
            n = self.buf[2]
            if n < 3:                        # cmd + checksum = almeno 3 byte
                self.scartati += 2
                del self.buf[:2]
                continue
            if len(self.buf) < 3 + n:
                return
            corpo = bytes(self.buf[3:3 + n])
            del self.buf[:3 + n]
            cmd, dati, ck = corpo[0], corpo[1:-2], corpo[-2] << 8 | corpo[-1]
            yield self.t0, cmd, dati, ck == checksum(corpo[:-2])
            self.t0 = t


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('traccia', help='traccia strace (.txt o .txt.gz)')
    ap.add_argument('--fd', help='mappa dei file descriptor (default: <traccia senza .txt.gz>-fd.txt)')
    ap.add_argument('--processo', default='AuxCtrl')
    ap.add_argument('--dev', default='ttyS3')
    ap.add_argument('--esempi', type=int, default=4, help='contenuti diversi da mostrare per ogni cmd')
    ap.add_argument('--csv', help='salva tutti i frame in questo CSV')
    ap.add_argument('--grezzo', action='store_true', help='solo statistiche dei byte, senza framing (per il lidar)')
    a = ap.parse_args()

    fd_file = a.fd or re.sub(r'\.txt(\.gz)?$', '', a.traccia) + '-fd.txt'
    if not os.path.exists(fd_file):
        sys.exit(f'mappa dei fd non trovata: {fd_file} (usa --fd)')
    thread, fd_dev = leggi_mappa_fd(fd_file, a.processo, a.dev)
    if fd_dev is None:
        sys.exit(f'{a.processo} non ha /dev/{a.dev} aperto secondo {fd_file}')
    print(f'{a.processo}: thread {sorted(thread)}, /dev/{a.dev} = fd {fd_dev}')

    deframer = {'rx': Deframer(), 'tx': Deframer()}
    byte_tot = collections.Counter()
    chiamate = collections.Counter()
    frame = collections.defaultdict(list)        # (dir, cmd) -> [(t, dati, ok)]
    t_min = t_max = None
    out = None
    if a.csv:
        out = csv.writer(open(a.csv, 'w', newline=''))
        out.writerow(['tempo_s', 'dir', 'cmd', 'len_dati', 'dati_hex', 'checksum_ok'])

    for t, d, dati in eventi(a.traccia, thread, fd_dev):
        t_min = t if t_min is None else t_min
        t_max = t
        byte_tot[d] += len(dati)
        chiamate[d] += 1
        if a.grezzo:
            continue
        for tf, cmd, corpo, ok in deframer[d].aggiungi(t, dati):
            frame[(d, cmd)].append((tf, corpo, ok))
            if out:
                out.writerow([f'{tf:.6f}', d, f'0x{cmd:02x}', len(corpo), corpo.hex(' '), int(ok)])

    if t_min is None:
        sys.exit('nessun read/write sul dispositivo nella traccia')
    durata = t_max - t_min
    print(f'durata {durata:.1f} s')
    for d, nome in (('rx', 'GD32 -> SoC (read)'), ('tx', 'SoC -> GD32 (write)')):
        print(f'{nome}: {chiamate[d]} chiamate, {byte_tot[d]} byte'
              + ('' if a.grezzo else f', {deframer[d].scartati} byte fuori frame'))
    if a.grezzo:
        return

    for d, nome in (('tx', 'SoC -> GD32'), ('rx', 'GD32 -> SoC')):
        cmds = sorted(c for (dd, c) in frame if dd == d)
        print(f'\n== {nome}: {len(cmds)} codici cmd')
        print(f'{"cmd":>5} {"frame":>7} {"ck err":>6} {"len":>8} {"periodo ms (mediana, min-max)":>32}  primo-ultimo (s)')
        for c in cmds:
            v = frame[(d, c)]
            lun = sorted({len(x[1]) for x in v})
            err = sum(1 for x in v if not x[2])
            tempi = [x[0] for x in v]
            if len(tempi) > 1:
                dt = [(b - a_) * 1000 for a_, b in zip(tempi, tempi[1:])]
                per = f'{statistics.median(dt):8.1f} ({min(dt):.1f}-{max(dt):.1f})'
            else:
                per = '-'
            lun_s = ','.join(map(str, lun)) if len(lun) <= 3 else f'{lun[0]}..{lun[-1]}'
            print(f' 0x{c:02x} {len(v):7d} {err:6d} {lun_s:>8} {per:>32}  {tempi[0] - t_min:.1f}-{tempi[-1] - t_min:.1f}')
        for c in cmds:
            v = frame[(d, c)]
            visti = collections.Counter(x[1] for x in v)
            print(f'\n  0x{c:02x} ({nome}): {len(visti)} contenuti diversi; i più frequenti:')
            for corpo, n in visti.most_common(a.esempi):
                print(f'    {n:6d}×  {corpo.hex(" ") or "(vuoto)"}')
            if len(visti) > 1:
                # Quali byte cambiano tra un frame e l'altro (a parità di lunghezza)
                lunghezza = collections.Counter(len(x[1]) for x in v).most_common(1)[0][0]
                stessi = [x[1] for x in v if len(x[1]) == lunghezza]
                variabili = [i for i in range(lunghezza) if len({s[i] for s in stessi}) > 1]
                if variabili:
                    print(f'    byte che cambiano (len {lunghezza}): {variabili}')


if __name__ == '__main__':
    main()
