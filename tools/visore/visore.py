#!/usr/bin/env python3
"""visore.py - Visualizzatore delle missioni registrate sul robot (strumento di debug).

Uso: python3 tools/visore/visore.py [porta]      (porta predefinita 8090), poi http://localhost:8090

Serve la pagina di tools/visore/ e i file delle missioni in build/robot/fase3/NOME/: NOME.map e NOME.oss (scritti da
tools/osserva-robot.c durante tools/driver-robot.sh missione) e NOME.out (registro del programma). Solo libreria standard.
"""
import http.server
import json
import os
import sys
import urllib.parse

CARTELLA = os.path.dirname(os.path.abspath(__file__))
PROGETTO = os.path.dirname(os.path.dirname(CARTELLA))
MISSIONI = os.path.join(PROGETTO, 'build', 'robot', 'fase3')
TIPI = {'map': 'application/octet-stream', 'ultima': 'application/octet-stream', 'oss': 'application/octet-stream',
        'out': 'application/octet-stream'}


def missioni():
    """Le cartelle con almeno NOME.map e NOME.oss, dalla più recente."""
    out = []
    if not os.path.isdir(MISSIONI):
        return out
    for nome in os.listdir(MISSIONI):
        d = os.path.join(MISSIONI, nome)
        if os.path.isfile(os.path.join(d, nome + '.map')) and os.path.isfile(os.path.join(d, nome + '.oss')):
            out.append((os.path.getmtime(os.path.join(d, nome + '.oss')), nome))
    return [n for _, n in sorted(out, reverse=True)]


class Gestore(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *a, **k):
        super().__init__(*a, directory=CARTELLA, **k)

    def log_message(self, *a):
        pass

    def manda(self, codice, tipo, dati):
        self.send_response(codice)
        self.send_header('Content-Type', tipo)
        self.send_header('Content-Length', str(len(dati)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(dati)

    def do_GET(self):
        u = urllib.parse.urlparse(self.path)
        if u.path == '/missioni':
            return self.manda(200, 'application/json', json.dumps(missioni()).encode())
        if u.path == '/file':
            q = urllib.parse.parse_qs(u.query)
            nome, tipo = q.get('nome', [''])[0], q.get('tipo', [''])[0]
            da = int(q.get('da', ['0'])[0] or 0)               # solo i byte da qui in poi (per la diretta)
            if tipo not in TIPI or nome not in missioni():
                return self.manda(404, 'text/plain', b'non trovato')
            p = os.path.join(MISSIONI, nome, f'{nome}.{tipo}')
            if not os.path.isfile(p):
                return self.manda(200, TIPI[tipo], b'')
            with open(p, 'rb') as f:
                f.seek(max(0, da))
                return self.manda(200, TIPI[tipo], f.read())
        return super().do_GET()


def main():
    porta = int(sys.argv[1]) if len(sys.argv) > 1 else 8090
    s = http.server.ThreadingHTTPServer(('127.0.0.1', porta), Gestore)
    print(f'visore: http://localhost:{porta}  (missioni in {MISSIONI}; Ctrl+C per uscire)', flush=True)
    try:
        s.serve_forever()
    except KeyboardInterrupt:
        pass


main()
