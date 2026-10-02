#!/usr/bin/env python3
"""prova-spia-socket.py - Prova di "spia socket" sul PC, senza robot.

  "AuxCtrl" finto: server in ascolto su un socket Unix stream (come /tmp/AuxEverest_Sock)
  "RobotApp" finto: si collega allo stesso percorso DOPO che la spia si è inserita
  più un socket a datagrammi (come /tmp/UDP_AUXCTRL_SEND_PATH) che riceve messaggi.

Controlla che i messaggi arrivino intatti in entrambe le direzioni, che il registro li contenga, che
"spia riepilogo" li conti, e che all'uscita la spia rimetta i socket originali al loro posto.

Uso: tools/prova-spia-socket.py [build/host/spia]
"""
import os, re, signal, socket, struct, subprocess, sys, tempfile, time

SPIA = sys.argv[1] if len(sys.argv) > 1 else "build/host/spia"
MAGIA = 0x5867a9b0


def messaggio(cmd, dati, c12=0, t=0):
    return struct.pack("<IiiiQQ", MAGIA, cmd, len(dati), c12, t, 0xdeadbeef) + dati


def ricevi_tutto(s, n, limite=3.0):
    out = bytearray()
    fine = time.monotonic() + limite
    while len(out) < n and time.monotonic() < fine:
        try:
            out += s.recv(65536)
        except BlockingIOError:
            time.sleep(0.01)
    return bytes(out)


def main():
    d = tempfile.mkdtemp()
    p_stream, p_dgram, reg = f"{d}/AuxEverest_Sock", f"{d}/UDP_AUXCTRL_SEND_PATH", f"{d}/sock.spia"
    server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    server.bind(p_stream)
    server.listen(4)
    ricevitore = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    ricevitore.bind(p_dgram)
    ricevitore.setblocking(False)

    spia = subprocess.Popen([SPIA, "socket", "-o", reg, "-s", "0",
                             "--stream", f"aux,everest,{p_stream}", "--dgram", f"udp,grezzo,{p_dgram}"],
                            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    for _ in range(2):
        riga = spia.stdout.readline()
        assert riga.startswith("canale"), f"riga inattesa: {riga!r} {spia.stderr.read() if spia.poll() is not None else ''}"

    client = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    client.connect(p_stream)               # arriva alla spia, che si collega al server su .vero
    server.settimeout(3)
    lato_server, _ = server.accept()
    client.setblocking(False)
    lato_server.setblocking(False)

    verso_server = b"".join(messaggio(10 + i, bytes(range(i * 7 % 200)), c12=i, t=1000 + i) for i in range(50))
    verso_client = b"".join(messaggio(500 + i % 3, os.urandom(3000 if i % 10 == 0 else 96)) for i in range(200))
    client.sendall(verso_server)
    lato_server.sendall(verso_client)
    arrivati_server = ricevi_tutto(lato_server, len(verso_server))
    arrivati_client = ricevi_tutto(client, len(verso_client))

    mittente = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    datagrammi = [os.urandom(20 + i) for i in range(5)]
    for x in datagrammi:
        mittente.sendto(x, p_dgram)        # arriva alla spia, che inoltra a .vero
    time.sleep(0.3)
    ricevuti_dgram = []
    while True:
        try:
            ricevuti_dgram.append(ricevitore.recv(65536))
        except BlockingIOError:
            break

    spia.send_signal(signal.SIGTERM)
    _, err = spia.communicate(timeout=5)
    ok = True

    def controlla(cond, msg):
        nonlocal ok
        print(("ok      " if cond else "FALLITO ") + msg)
        ok &= bool(cond)

    controlla(spia.returncode == 0, f"la spia termina con codice 0 ({spia.returncode}) {err.strip()[-200:]}")
    controlla(arrivati_server == verso_server, f"client -> server: {len(verso_server)} byte intatti")
    controlla(arrivati_client == verso_client, f"server -> client: {len(verso_client)} byte intatti")
    controlla(ricevuti_dgram == datagrammi, f"datagrammi: {len(datagrammi)} intatti")
    controlla(not os.path.exists(p_stream + ".vero") and not os.path.exists(p_dgram + ".vero"),
              "all'uscita i .vero sono stati rinominati")
    nuovo = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    try:
        nuovo.connect(p_stream)
        server.settimeout(1)
        server.accept()
        controlla(True, "dopo l'uscita il percorso originale porta di nuovo al server")
    except OSError as e:
        controlla(False, f"dopo l'uscita il percorso originale porta di nuovo al server ({e})")

    riep = subprocess.run([SPIA, "riepilogo", reg], capture_output=True, text=True).stdout
    print(riep)
    controlla(re.search(r"client -> server: \d+ byte, 50 frame, 0 errori", riep) is not None,
              "riepilogo: 50 messaggi client -> server, 0 errori")
    controlla(re.search(r"server -> client: \d+ byte, 200 frame, 0 errori", riep) is not None,
              "riepilogo: 200 messaggi server -> client, 0 errori")
    leggi = subprocess.run([SPIA, "leggi", "-c", "aux", reg], capture_output=True, text=True).stdout
    controlla("cmd 13 len 21 c12 3 t 1003" in leggi, "leggi: campi dell'intestazione decodificati")
    print("PROVA SPIA SOCKET:", "OK" if ok else "FALLITA")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
