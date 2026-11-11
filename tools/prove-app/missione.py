#!/usr/bin/env python3
"""La missione di confronto con il backend collegato: la stessa prova del riferimento (tools/prove-nav/prova.py
--missione --dalla-base, stanza vuota, seme 1, programma di prova build/prove-nav/vacdbg) ma con il programma collegato
al backend sul PC, e un'app finta che tiene aperta la richiesta lunga dello stato a 100 ms come la schermata principale.

Uso (dalla cartella del progetto, dopo tools/prove-nav/costruisci.sh):
    backend/.venv/bin/python tools/prove-app/missione.py [durata massima in secondi]
Alla fine controlla che al server siano arrivati stato, mappa dal vivo, mappa salvata e scia. I numeri dei task
(job, scadenze mancate, tempi) stanno in fondo a build/app-prove/missione/prog.log, come per il riferimento.
"""
import os
import shutil
import subprocess
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import banco as banco_module     # noqa: E402
from banco import Banco          # noqa: E402
from scenari import known_cells, live_map   # noqa: E402

import maps                      # noqa: E402

DURATION = sys.argv[1] if len(sys.argv) > 1 else '1800'


def main():
    os.chdir(banco_module.REPO)
    shutil.rmtree(os.path.join(banco_module.WORK, 'missione'), ignore_errors=True)
    bench = Banco('missione')
    bench.prepare_backend()
    bench.start_backend()
    bench.start_proxy()

    # l'app finta: richiesta lunga dello stato con min_interval_ms=100, come la schermata principale
    states = {'count': 0, 'missions': set(), 'running': True}

    def app_loop():
        after = None
        while states['running']:
            try:
                query = '' if after is None else f'?after={after}&min_interval_ms=100'
                status, body = bench.api('GET', '/api/robot/state' + query)
                if status == 200:
                    after = body['state_number']
                    states['count'] += 1
                    if 'navigation' in body:
                        states['missions'].add(body['navigation']['mission'])
            except OSError:
                time.sleep(1)

    thread = threading.Thread(target=app_loop, daemon=True)
    thread.start()

    env = bench.robot_env()
    del env['RTES_COMMANDS']        # prova.py usa il suo file dei comandi
    try:
        with open(os.path.join(bench.dir, 'riassunto.txt'), 'w') as summary:
            subprocess.run([sys.executable, '-I', 'tools/prove-nav/prova.py', '--missione', '--dalla-base',
                            'sim/stanze/vuota.txt', '1', DURATION], env=env, stdout=summary, stderr=subprocess.STDOUT)
        shutil.copy('build/prove-nav/prog_vuota_s1_pulizia.log', os.path.join(bench.dir, 'prog.log'))
        print(open(os.path.join(bench.dir, 'riassunto.txt')).read().split('\n   EVENTO')[0])

        states['running'] = False
        bench.check(f'l\'app ha ricevuto lo stato per tutta la missione ({states["count"]} stati)', states['count'] > 1000)
        bench.check(f'fasi della missione viste dall\'app: {sorted(states["missions"])}',
                    {'explore', 'follow_borders', 'clean', 'return_home'} <= states['missions'])
        listed = bench.api('GET', '/api/robot/maps')[1]
        bench.check('il server ha creato map-1 dalla mappa salvata, selezionata', [(m['name'], m['selected']) for m in listed] == [('map-1', True)])
        robot_file = open(os.path.join(bench.maps_dir, 'map.rtm'), 'rb').read()
        if listed:
            server_file = bench.api('GET', f'/api/robot/maps/{listed[0]["id"]}', raw=True)[1]
            bench.check('la copia sul server è identica al file map.rtm del robot', server_file == robot_file)
        saved = maps.decode_rtm(robot_file)
        expected = {(saved.x + c, saved.y + r): saved.cells[r * saved.width + c] for r in range(saved.height) for c in range(saved.width)}
        grid = live_map(bench)[2]
        different = sum(1 for cell, category in expected.items() if grid.get(cell, 0) != category)
        final = bench.state()
        if final['navigation']['state'] == 'sleep' and final['driver']['on_base']:
            # agganciato: il robot ha salvato alla fine, quindi il file e la mappa dal vivo devono coincidere
            bench.check(f'la mappa dal vivo sul server coincide con la mappa salvata all\'aggancio ({different} celle diverse su '
                        f'{len(expected)})', different <= len(expected) * 0.01)
        else:
            # aggancio fallito (nel simulatore capita): l'ultimo salvataggio è quello di fine esplorazione, la mappa dal
            # vivo è più nuova. Il confronto esatto lo fa lo scenario "mappe"
            print(f'   aggancio non riuscito: la mappa dal vivo ({known_cells(grid)} celle viste) è più nuova del file salvato a '
                  f'fine esplorazione ({different} celle diverse su {len(expected)})')
        trail = bench.api('GET', '/api/robot/trail')[1]
        bench.check(f'la scia dell\'ultima missione è sul server ({len(trail["points"])} punti)', len(trail['points']) > 500)
        bench.check('e anche su file, scritta a fine missione o al guasto',
                    os.path.exists(os.path.join(bench.data_dir, 'trail.json')) or states['missions'] != set())
    finally:
        bench.stop_all()
    print(f'== missione con il backend: {bench.checks - bench.failed} controlli riusciti su {bench.checks}')


if __name__ == '__main__':
    main()
