#!/usr/bin/env python3
"""Prove complete dell'app nel simulatore, con il programma vero del robot collegato al backend sul PC.

Uso (dalla cartella del progetto, dopo make build/host/sim build/host/vacuum):
    backend/.venv/bin/python tools/prove-app/scenari.py [comandi] [telecomando] [guasto] [zone] [mappe] [rete]
Senza argomenti li fa tutti. Ogni scenario riparte da zero: backend con dati nuovi, simulatore con il robot sulla base
nella stanza vuota, programma del robot. Alla fine di ognuno stampa job e scadenze mancate dei task: una scadenza
mancata fa fallire la prova. I registri restano in build/app-prove/<scenario>/.
"""
import os
import shutil
import struct
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import banco as banco_module     # noqa: E402
from banco import Banco, USERS   # noqa: E402

import maps                      # noqa: E402  (backend/maps.py)

MOVING_STATES = ('leave_base', 'explore', 'follow_path', 'follow_borders', 'border_lap', 'clean', 'wait', 'align', 'unlock', 'cross')


def nav(state):
    return state['navigation']['state']


def live_map(bench, after=None):
    """La mappa dal vivo come la vede l'app: (tipo, versione, griglia)."""
    status, data = bench.api('GET', '/api/robot/map' + (f'?after={after}' if after is not None else ''), raw=True)
    grid = {}
    offset, version = 1, None
    while offset < len(data):
        version, length = maps.LIVE_PART_HEADER.unpack_from(data, offset)
        offset += maps.LIVE_PART_HEADER.size
        part = maps.decode_rtm(data[offset:offset + length])
        offset += length
        for row in range(part.height):
            for column in range(part.width):
                grid[(part.x + column, part.y + row)] = part.cells[row * part.width + column]
    return data[0], version, grid


def known_cells(grid):
    return sum(1 for category in grid.values() if category != maps.MAP_CAT_UNKNOWN)


def start_everything(bench, extra_lines=()):
    bench.prepare_backend()
    bench.start_backend()
    bench.start_proxy()
    bench.start_sim(extra_lines=extra_lines)
    bench.start_robot()
    online = bench.wait_state('robot online con lo stato', lambda s: s['online'] and 'navigation' in s, 20)
    bench.check('il robot si collega al backend in HTTPS e manda lo stato', online is not None)
    return online


def finish(bench):
    bench.stop_robot()
    numbers = bench.task_numbers()
    missed = [(name, misses) for name, jobs, misses in numbers if misses != '0']
    print('   task: ' + ', '.join(f'{name[5:]} {jobs} job' for name, jobs, misses in numbers[-5:]))
    bench.check('nessuna scadenza mancata nei task', bool(numbers) and not missed)
    bench.stop_all()


def scenario_comandi(bench):
    """Comandi dell'app: start, pause, resume, home, con i loro rifiuti; stato, mappa dal vivo e scia."""
    state = start_everything(bench)
    bench.check('a riposo sulla base, senza missione', nav(state) == 'sleep' and state['navigation']['mission'] == 'none'
                and state['driver']['on_base'])
    bench.check('lo stato ha i campi del robot e quelli del server', all(key in state for key in (
        'format', 'robot_time', 'uptime_s', 'navigation', 'objective', 'position', 'map', 'safety', 'lidar', 'battery',
        'motors', 'driver', 'sensors', 'tasks', 'online', 'last_contact', 'remote_control_user', 'map_version', 'state_number')))
    bench.check('i cinque task con le scadenze mancate a zero', len(state['tasks']) == 5 and
                all(task['deadline_misses'] == 0 for task in state['tasks']))
    kind, version, grid = live_map(bench)
    bench.check('robot senza mappa: la mappa dal vivo è vuota', known_cells(grid) == 0)
    bench.check('nessuna mappa selezionata sul server', bench.api('GET', '/api/robot/maps')[1] == [])

    bench.command('pause a riposo', 'POST', '/api/robot/mission/pause', expect=409, reason='not_running')
    bench.command('resume a riposo', 'POST', '/api/robot/mission/resume', expect=409, reason='not_paused')
    bench.command('home a riposo sulla base', 'POST', '/api/robot/mission/home', expect=409, reason='already_home')
    bench.command('fault/reset senza guasto', 'POST', '/api/robot/fault/reset', expect=409, reason='not_in_fault')

    bench.command('start', 'POST', '/api/robot/mission/start')
    state = bench.wait_state('missione partita', lambda s: s['navigation']['mission'] != 'none' and nav(s) != 'sleep', 5)
    bench.check('la missione parte (numero 1, esplorazione)', state is not None and state['navigation']['mission_number'] == 1
                and state['navigation']['mission'] == 'explore')
    bench.command('start con la missione in corso', 'POST', '/api/robot/mission/start', expect=409, reason='busy')
    state = bench.wait_state('robot uscito dalla base', lambda s: not s['driver']['on_base'] and s['position']['x'] > 0.25, 30)
    bench.check('il robot esce dalla base', state is not None)
    state = bench.wait_state('robot in movimento', lambda s: nav(s) in ('follow_path', 'border_lap'), 30)
    bench.check('il robot esplora', state is not None)

    time.sleep(3)
    kind, version, grid = live_map(bench)
    bench.check(f'la mappa dal vivo arriva al server ({known_cells(grid)} celle viste)', known_cells(grid) > 500)
    kind2, version2, patches = live_map(bench, after=version - 2)
    bench.check('l\'app può chiedere solo le differenze', kind2 == maps.LIVE_MESSAGE_PATCHES and version2 == version)
    trail = bench.api('GET', '/api/robot/trail')[1]
    bench.check(f'la scia si allunga ({len(trail["points"])} punti)', len(trail['points']) >= 3)

    bench.command('pause', 'POST', '/api/robot/mission/pause')
    state = bench.wait_state('in pausa', lambda s: nav(s) == 'pause', 3)
    bench.check('il robot va in pausa, la missione resta', state is not None and state['navigation']['mission'] != 'none')
    time.sleep(1.5)
    state = bench.state()
    bench.check('in pausa è fermo', state['position']['v'] == 0 and state['position']['omega'] == 0)
    bench.command('pause in pausa', 'POST', '/api/robot/mission/pause', expect=409, reason='not_running')
    bench.command('start in pausa', 'POST', '/api/robot/mission/start', expect=409, reason='busy')
    bench.command('resume', 'POST', '/api/robot/mission/resume')
    state = bench.wait_state('ripartito', lambda s: nav(s) in MOVING_STATES and nav(s) != 'wait', 8)
    bench.check('il robot riprende la stessa missione', state is not None and state['navigation']['mission_number'] == 1)
    bench.command('resume fuori dalla pausa', 'POST', '/api/robot/mission/resume', expect=409, reason='not_paused')

    bench.command('home', 'POST', '/api/robot/mission/home')
    state = bench.wait_state('rientro', lambda s: s['navigation']['mission'] == 'return_home', 3)
    bench.check('il robot rientra', state is not None)
    bench.command('home mentre sta già tornando', 'POST', '/api/robot/mission/home')
    state = bench.wait_state('fine del rientro', lambda s: nav(s) in ('sleep', 'failure'), 240)
    bench.check('il rientro finisce (aggancio riuscito o, nel simulatore, fallito): ' + (nav(state) if state else 'no'), state is not None)

    log = bench.api('GET', '/api/robot/log?limit=50')[1]
    bench.check('il registro ha i comandi con chi li ha mandati', [entry['type'] for entry in log][:2] == ['home', 'home']
                and all(entry['user'] == USERS[0] for entry in log))
    finish(bench)


def scenario_telecomando(bench):
    """Telecomando: presa automatica, movimento, tempo di 0,5 s, un utente alla volta, rilascio dopo 30 s, pausa della missione."""
    start_everything(bench)
    bench.command('remote/move da un altro utente a telecomando libero lo prende', 'POST', '/api/robot/remote/move',
                  {'v': 0.2, 'omega': 0}, user=USERS[1])
    state = bench.wait_state('in telecomando', lambda s: nav(s) == 'remote' and s['remote_control'], 3)
    bench.check('il robot è in telecomando, e il server dice chi lo tiene',
                state is not None and state['remote_control_user'] == USERS[1])
    bench.command('remote/move di un altro utente', 'POST', '/api/robot/remote/move', {'v': 0.1, 'omega': 0},
                  expect=409, reason='remote_in_use')
    bench.command('start di un altro utente', 'POST', '/api/robot/mission/start', expect=409, reason='remote_in_use')
    bench.command('remote/release di chi non lo tiene', 'POST', '/api/robot/remote/release', expect=409, reason='not_remote_owner')
    bench.command('remote/release', 'POST', '/api/robot/remote/release', user=USERS[1])
    state = bench.wait_state('rilasciato', lambda s: not s['remote_control'] and nav(s) == 'sleep', 3)
    bench.check('rilasciato senza missione: riposo', state is not None and state['remote_control_user'] is None)

    # si guida fuori dalla base: sui contatti il driver lascia uscire piano in avanti
    bench.command('remote/take', 'POST', '/api/robot/remote/take')
    start_x = bench.state()['position']['x']
    limit = time.time() + 12
    speeds = []
    while time.time() < limit:
        bench.api('POST', '/api/robot/remote/move', {'v': 0.3, 'omega': 0})
        speeds.append(bench.state()['position']['v'])
        time.sleep(0.15)
    state = bench.state()
    bench.check(f'il robot esce dalla base in avanti (x da {start_x:.2f} a {state["position"]["x"]:.2f} m)',
                state['position']['x'] > start_x + 0.5 and not state['driver']['on_base'])
    bench.check(f'la velocità non supera il massimo del telecomando ({max(speeds):.2f} m/s)', max(speeds) <= 0.37)
    time.sleep(1.2)
    state = bench.state()
    bench.check('senza comandi nuovi il robot si ferma dopo 0,5 s', abs(state['position']['v']) < 0.01 and nav(state) == 'remote')

    bench.command('remote/move con v e omega fuori dai massimi', 'POST', '/api/robot/remote/move', {'v': 5, 'omega': -9})
    time.sleep(0.4)
    state = bench.state()
    bench.check(f'omega limitata al massimo del telecomando ({state["position"]["omega"]:.2f} rad/s)',
                -1.3 <= state['position']['omega'] <= 0)
    bench.command('remote/move senza parametri', 'POST', '/api/robot/remote/move', {'v': 0.1}, expect=400)

    bench.command('remote/cleaning: spazzole sì, aspirazione no', 'POST', '/api/robot/remote/cleaning',
                  {'brushes': True, 'suction': False})
    state = bench.wait_state('spazzole accese', lambda s: s['motors']['main_brush'] > 0, 3)
    bench.check('spazzole accese e aspirazione spenta', state is not None and state['motors']['suction_rpm'] == 0)
    bench.command('remote/cleaning: tutte e due', 'POST', '/api/robot/remote/cleaning', {'brushes': True, 'suction': True})
    state = bench.wait_state('aspirazione accesa', lambda s: s['motors']['suction_rpm'] > 0, 5)
    bench.check('aspirazione accesa', state is not None)
    bench.sim_command('cassetto 0')
    time.sleep(0.5)
    bench.command('remote/cleaning con l\'aspirazione senza cassetto', 'POST', '/api/robot/remote/cleaning',
                  {'brushes': True, 'suction': True}, expect=409, reason='dustbin_missing')
    bench.sim_command('cassetto 1')

    bench.command('mappe: cambio di mappa in telecomando', 'POST', '/api/robot/map/reset', expect=409, reason='not_resting')

    # rilascio dopo 30 s senza comandi del telecomando: i keepalive lo tengono, poi si smette
    for _ in range(2):
        time.sleep(12)
        bench.command('remote/keepalive', 'POST', '/api/robot/remote/keepalive')
    state = bench.state()
    bench.check('dopo 24 s con i keepalive è ancora in telecomando', nav(state) == 'remote' and state['remote_control_user'] == USERS[0])
    released_at = time.time()
    state = bench.wait_state('rilascio per tempo scaduto', lambda s: not s['remote_control'] and s['remote_control_user'] is None, 40)
    waited = time.time() - released_at
    bench.check(f'rilascio da solo dopo circa 30 s senza comandi ({waited:.0f} s), su robot e server', state is not None and 26 <= waited <= 36)
    bench.check('rilasciato senza missione: riposo, spazzole e aspirazione spente', state is not None and nav(state) == 'sleep'
                and state['motors']['main_brush'] == 0 and state['motors']['suction_rpm'] == 0)

    # telecomando durante una missione: la mette in pausa, e i comandi di missione di chi lo tiene lo rilasciano
    bench.command('home da fermo fuori dalla base', 'POST', '/api/robot/mission/home')
    state = bench.wait_state('rientro', lambda s: s['navigation']['mission'] == 'return_home' and nav(s) != 'sleep', 5)
    bench.check('parte il rientro', state is not None)
    bench.command('remote/take durante il rientro', 'POST', '/api/robot/remote/take')
    state = bench.wait_state('telecomando', lambda s: nav(s) == 'remote', 3)
    bench.check('il telecomando sospende la missione', state is not None and state['navigation']['mission'] == 'return_home')
    bench.command('pause di chi tiene il telecomando: rilascio e pausa', 'POST', '/api/robot/mission/pause')
    state = bench.wait_state('pausa', lambda s: nav(s) == 'pause' and not s['remote_control'], 3)
    bench.check('rilasciato con una missione: pausa', state is not None and state['remote_control_user'] is None)
    bench.command('remote/take', 'POST', '/api/robot/remote/take')
    bench.command('resume di chi tiene il telecomando: rilascio e ripresa', 'POST', '/api/robot/mission/resume')
    state = bench.wait_state('ripresa', lambda s: not s['remote_control'] and nav(s) not in ('pause', 'remote', 'sleep'), 8)
    bench.check('la missione riprende', state is not None and state['navigation']['mission'] == 'return_home')
    state = bench.wait_state('fine del rientro', lambda s: nav(s) in ('sleep', 'failure'), 240)
    bench.check('il rientro finisce: ' + (nav(state) if state else 'no'), state is not None)
    log = [entry['type'] for entry in bench.api('GET', '/api/robot/log?limit=200')[1]]
    bench.check(f'registro: nessun keepalive, i remote_move uguali una volta sola ({log.count("remote_move")} righe)',
                'remote_keepalive' not in log and 2 <= log.count('remote_move') <= 6)
    finish(bench)


def scenario_guasto(bench):
    """Guasto: una scatola davanti alla base ferma l'uscita. Telecomando preso in guasto, azzeramento, pausa, ripresa."""
    bx, by, bd = 0, 0, 0
    bench.prepare_backend()
    # una scatola bassa a 25 cm davanti al robot: il lidar non la vede, il robot non riesce a uscire dalla base
    import math, re     # noqa: E401
    text = open(os.path.join(banco_module.REPO, 'sim/stanze/vuota.txt')).read()
    bx, by, bd = map(float, re.search(r'^base x=(\S+) y=(\S+) angolo=(\S+)', text, flags=re.M).groups())
    box_x = bx + (0.1745 * 2 + 0.12) * math.cos(math.radians(bd))
    box_y = by + (0.1745 * 2 + 0.12) * math.sin(math.radians(bd))
    box = f'rettangolo nome=scatola x={box_x:.3f} y={box_y:.3f} larghezza=0.5 profondita=0.12 angolo={bd + 90} quota_da=0 quota_a=0.06'
    bench.start_backend()
    bench.start_proxy()
    bench.start_sim(extra_lines=[box])
    bench.start_robot()
    bench.check('robot online', bench.wait_state('online', lambda s: s['online'] and 'navigation' in s, 20) is not None)

    bench.command('start', 'POST', '/api/robot/mission/start')
    state = bench.wait_state('guasto', lambda s: nav(s) == 'failure' and s['driver']['phase'] == 'fault', 90)
    bench.check('la scatola davanti alla base manda il robot in guasto: ' +
                (str(state['driver']['fault']) if state else 'no'), state is not None and state['driver']['phase'] == 'fault'
                and state['driver']['fault']['text'] != '' and state['navigation']['mission'] != 'none')
    bench.command('pause in guasto', 'POST', '/api/robot/mission/pause', expect=409, reason='not_running')

    # il telecomando si prende anche in guasto: il robot resta fermo finché il guasto non si azzera
    bench.command('remote/take in guasto', 'POST', '/api/robot/remote/take')
    state = bench.wait_state('telecomando in guasto', lambda s: s['remote_control'], 3)
    bench.check('telecomando preso, il robot resta in guasto', state is not None and nav(state) == 'failure')
    bench.command('remote/move in guasto', 'POST', '/api/robot/remote/move', {'v': -0.1, 'omega': 0})
    time.sleep(0.6)
    state = bench.state()
    bench.check('in guasto il robot non si muove', nav(state) == 'failure' and state['position']['v'] == 0)
    bench.sim_command('solleva')        # tolto dalla scatola a mano, come farebbe l'utente
    time.sleep(1)
    bench.sim_command(f'sposta {bx + 1.5 * math.cos(math.radians(bd)):.2f} {by + 1.5 * math.sin(math.radians(bd)):.2f} {bd}')
    bench.sim_command('posa')
    time.sleep(1)
    bench.command('fault/reset', 'POST', '/api/robot/fault/reset')
    state = bench.wait_state('telecomando dopo il guasto', lambda s: nav(s) == 'remote', 5)
    bench.check('azzerato il guasto il robot torna in telecomando', state is not None and state['driver']['phase'] == 'ready'
                and state['driver']['fault'] is None and state['remote_control'])
    bench.command('fault/reset senza guasto', 'POST', '/api/robot/fault/reset', expect=409, reason='not_in_fault')
    bench.command('remote/release', 'POST', '/api/robot/remote/release')
    state = bench.wait_state('pausa', lambda s: nav(s) == 'pause', 3)
    bench.check('rilasciato con la missione di prima: pausa (non riposo)', state is not None and state['navigation']['mission'] != 'none')

    # guasto azzerato a metà missione senza telecomando: pausa, e Riprendi continua
    # ritrovata ma da confermare: la conferma arriva dopo un metro di strada, quindi non in pausa
    state = bench.wait_state('posizione ritrovata', lambda s: s['map']['state'] in ('normal', 'unconfirmed') and s['position']['valid'], 40)
    bench.check('dopo lo spostamento a mano il robot ritrova la posizione: ' + (state['map']['state'] if state else 'no'), state is not None)
    bench.command('resume', 'POST', '/api/robot/mission/resume')
    state = bench.wait_state('ripresa', lambda s: nav(s) not in ('pause', 'wait', 'sleep', 'failure'), 10)
    bench.check('la missione riprende: ' + (nav(state) if state else 'no'), state is not None and state['navigation']['mission_number'] == 1)
    bench.command('home', 'POST', '/api/robot/mission/home')
    state = bench.wait_state('fine', lambda s: nav(s) in ('sleep', 'failure'), 240)
    bench.check('il rientro finisce: ' + (nav(state) if state else 'no'), state is not None)
    if state is not None and nav(state) == 'failure':
        # nel simulatore l'aggancio a volte fallisce: buona occasione per provare che home azzera il guasto
        bench.command('home in guasto azzera il guasto e riparte', 'POST', '/api/robot/mission/home')
        state = bench.wait_state('rientro di nuovo', lambda s: nav(s) != 'failure', 5)
        bench.check('guasto azzerato da home', state is not None and state['driver']['phase'] == 'ready')
    finish(bench)


def scenario_zone(bench):
    """Zone proibite dall'app, con il robot senza file della mappa: si applicano subito e arrivano nello stato."""
    state = start_everything(bench)
    bench.check('all\'inizio c\'è solo la zona della base', len(state['map']['forbidden_zones']) == 1 and state['map']['started_on_base'])
    bench.check('GET zones senza la base', bench.api('GET', '/api/robot/zones')[1] == [])
    zones = [{'x': 1.0, 'y': 0.5, 'w': 0.5, 'h': 0.4}, {'x': -1.0, 'y': -1.0, 'w': 0.3, 'h': 0.3}]
    bench.command('PUT zones con due zone', 'PUT', '/api/robot/zones', zones)
    state = bench.wait_state('zone nello stato', lambda s: len(s['map']['forbidden_zones']) == 3, 5)
    bench.check('lo stato ha la base più le due zone', state is not None and state['map']['forbidden_zones'][1:] == zones)
    bench.check('GET zones le ritorna', bench.api('GET', '/api/robot/zones')[1] == zones)
    bench.command('PUT zones con una zona sola (lista intera)', 'PUT', '/api/robot/zones', zones[:1])
    state = bench.wait_state('una zona', lambda s: len(s['map']['forbidden_zones']) == 2, 5)
    bench.check('resta una zona', state is not None)
    bench.command('16 zone', 'PUT', '/api/robot/zones', zones[:1] * 16, expect=409, reason='too_many_zones')
    bench.command('zona fuori dalla mappa', 'PUT', '/api/robot/zones', [{'x': 30, 'y': 0, 'w': 1, 'h': 1}], expect=409, reason='outside_map')
    bench.command('zone scritte male', 'PUT', '/api/robot/zones', {'x': 1}, expect=400)

    # la sicurezza le rispetta in missione e le ignora in telecomando
    bench.command('zona davanti alla base', 'PUT', '/api/robot/zones', [{'x': 0.9, 'y': -1.0, 'w': 0.4, 'h': 2.0}])
    bench.wait_state('zona applicata', lambda s: len(s['map']['forbidden_zones']) == 2, 5)
    limit = time.time() + 22
    while time.time() < limit:
        bench.api('POST', '/api/robot/remote/move', {'v': 0.3, 'omega': 0})
        time.sleep(0.15)
    state = bench.state()
    bench.check(f'in telecomando il robot entra nella zona proibita (x = {state["position"]["x"]:.2f} m)', state['position']['x'] > 0.95)
    bench.command('remote/release', 'POST', '/api/robot/remote/release')
    finish(bench)


def scenario_mappe(bench):
    """Cambio di mappa con riavvio, zone sulla mappa selezionata, "Nessuna mappa", allineamento robot-server."""
    reference = os.path.join(banco_module.WORK, 'dopo-senza-rete', 'maps', 'map.rtm')
    if not os.path.exists(reference):
        print('   manca la mappa della missione di riferimento: salto')
        return
    data = open(reference, 'rb').read()
    start_everything(bench)
    # una mappa sul server, come quelle caricate con lo script di amministrazione (il backend rilegge il database)
    import config, db     # noqa: E401
    database = db.Database(os.path.join(bench.data_dir, config.DATABASE_FILE))
    map_id = maps.MapStore(database, bench.data_dir).add(data, 'stanza vuota')
    database.close()
    listed = bench.api('GET', '/api/robot/maps')[1]
    bench.check('la mappa importata è nell\'elenco, non selezionata', [(m['name'], m['selected']) for m in listed] == [('stanza vuota', False)])
    bench.command('selezione di una mappa che non esiste', 'PUT', '/api/robot/maps/selected', {'id': 999}, expect=404)

    uptime = bench.state()['uptime_s']
    bench.command('PUT maps/selected', 'PUT', '/api/robot/maps/selected', {'id': map_id})
    state = bench.wait_state('robot ripartito con la mappa', lambda s: s['online'] and s['map'].get('file_crc') == maps.file_crc(data)
                             and s['uptime_s'] <= uptime + 2 and nav(s) == 'sleep', 30)
    bench.check('il programma si riavvia (codice di uscita 3, rilanciato) con la mappa nuova',
                state is not None and getattr(bench, 'restarts', 0) == 1)
    robot_file = os.path.join(bench.maps_dir, 'map.rtm')
    bench.check('sul robot map.rtm è la mappa scelta, senza file temporanei',
                open(robot_file, 'rb').read() == data and os.listdir(bench.maps_dir) == ['map.rtm'])
    bench.check('il server la segna selezionata, e non ne crea un\'altra',
                [(m['id'], m['selected']) for m in bench.api('GET', '/api/robot/maps')[1]] == [(map_id, True)])
    time.sleep(1.5)
    kind, version, grid = live_map(bench)
    saved = maps.decode_rtm(data)
    expected = {(saved.x + c, saved.y + r): saved.cells[r * saved.width + c] for r in range(saved.height) for c in range(saved.width)}
    different = sum(1 for cell, category in expected.items() if grid.get(cell, 0) != category)
    bench.check(f'la mappa dal vivo sul server è la mappa caricata ({different} celle diverse su {len(expected)})', different <= 40)

    # zone sulla mappa selezionata: le applica il robot, salva e rimanda il file
    zones = [{'x': 1.0, 'y': 0.5, 'w': 0.5, 'h': 0.4}]
    bench.command('PUT maps/{id}/zones sulla selezionata', 'PUT', f'/api/robot/maps/{map_id}/zones', zones)
    state = bench.wait_state('file salvato con la zona', lambda s: s['map'].get('file_crc') not in (None, maps.file_crc(data)), 10)
    bench.check('il robot salva la mappa con la zona (il CRC del file cambia)', state is not None)
    time.sleep(1.5)
    server_file = bench.api('GET', f'/api/robot/maps/{map_id}', raw=True)[1]
    bench.check('il server riceve il file salvato: la sua copia è identica a quella del robot, con la zona',
                server_file == open(robot_file, 'rb').read() and maps.decode_rtm(server_file).zones == [maps.zone_to_cells(zones[0])])
    bench.check('nessuna mappa in più sul server', len(bench.api('GET', '/api/robot/maps')[1]) == 1)

    # cancellare la selezionata: prima "Nessuna mappa" sul robot, poi la cancellazione sul server
    bench.command('DELETE della mappa selezionata', 'DELETE', f'/api/robot/maps/{map_id}', expect=409, reason='map_selected')
    bench.command('map/reset ("Nessuna mappa")', 'POST', '/api/robot/map/reset')
    state = bench.wait_state('robot ripartito senza mappa', lambda s: s['online'] and 'file_crc' not in s['map'] and nav(s) == 'sleep', 30)
    bench.check('il programma si riavvia senza mappa', state is not None and getattr(bench, 'restarts', 0) == 2
                and not os.path.exists(robot_file))
    listed = bench.api('GET', '/api/robot/maps')[1]
    bench.check('la copia resta sul server, non più selezionata', [(m['id'], m['selected']) for m in listed] == [(map_id, False)])
    bench.command('DELETE dopo il reset', 'DELETE', f'/api/robot/maps/{map_id}')
    bench.check('elenco vuoto', bench.api('GET', '/api/robot/maps')[1] == [])
    finish(bench)


def scenario_rete(bench):
    """Rete che cade e torna, e backend che riparte: il robot continua, poi stato e mappa dal vivo si riallineano."""
    start_everything(bench)
    bench.command('start', 'POST', '/api/robot/mission/start')
    bench.check('il robot esplora', bench.wait_state('in movimento', lambda s: nav(s) in ('follow_path', 'border_lap'), 60) is not None)
    time.sleep(4)
    before = known_cells(live_map(bench)[2])

    bench.stop('proxy')         # la rete del robot cade: il backend resta, ma il robot non lo raggiunge
    time.sleep(12)
    state = bench.state()
    # il server non sa che la connessione del robot è morta: il comando parte verso il nulla e l'esito non arriva (504);
    # 503 se nel frattempo la richiesta lunga del robot era già scaduta. In nessun caso 200
    bench.check('con la rete giù i comandi non riescono (503 o 504)', bench.api('POST', '/api/robot/mission/pause')[0] in (503, 504))
    simulator = bench.sim_state()
    time.sleep(3)
    moved = bench.sim_state()
    bench.check('il robot continua la missione senza rete',
                abs(moved['robot']['x'] - simulator['robot']['x']) + abs(moved['robot']['y'] - simulator['robot']['y']) +
                abs(moved['robot']['dir'] - simulator['robot']['dir']) > 0.01)
    time.sleep(25)
    state = bench.state()
    bench.check('dopo 35 s senza contatti il server lo dà offline, con l\'ultimo stato', not state['online'] and 'navigation' in state)
    bench.start_proxy()
    state = bench.wait_state('di nuovo online', lambda s: s['online'], 25)
    bench.check('tornata la rete il robot si ricollega da solo', state is not None)
    time.sleep(3)
    after = known_cells(live_map(bench)[2])
    bench.check(f'la mappa dal vivo è ancora tutta lì dopo il ritorno della rete (da {before} a {after} celle)', after >= before)
    bench.command('pause dopo il ritorno', 'POST', '/api/robot/mission/pause')

    # il backend riparte e perde la mappa dal vivo: alla prima differenza risponde 409 e il robot la rimanda intera
    bench.command('resume', 'POST', '/api/robot/mission/resume')
    bench.stop('backend')
    time.sleep(2)
    bench.start_backend()
    bench.tokens.clear()
    state = bench.wait_state('online dopo il riavvio del backend', lambda s: s['online'] and 'navigation' in s, 30)
    bench.check('riavviato il backend il robot si ricollega', state is not None)
    time.sleep(4)
    restored = known_cells(live_map(bench)[2])
    bench.check(f'il robot rimanda la mappa intera al backend ripartito ({restored} celle)', restored >= after * 0.9)
    bench.command('home', 'POST', '/api/robot/mission/home')
    state = bench.wait_state('fine', lambda s: nav(s) in ('sleep', 'failure'), 240)
    bench.check('il rientro finisce: ' + (nav(state) if state else 'no'), state is not None)
    finish(bench)


SCENARIOS = {
    'comandi': scenario_comandi,
    'telecomando': scenario_telecomando,
    'guasto': scenario_guasto,
    'zone': scenario_zone,
    'mappe': scenario_mappe,
    'rete': scenario_rete,
}


def main():
    names = sys.argv[1:] or list(SCENARIOS)
    os.chdir(banco_module.REPO)
    total_failed = 0
    for name in names:
        print(f'== {name}: {SCENARIOS[name].__doc__}', flush=True)
        shutil.rmtree(os.path.join(banco_module.WORK, 'scenario-' + name), ignore_errors=True)
        bench = Banco('scenario-' + name)
        try:
            SCENARIOS[name](bench)
        except Exception as error:     # un errore del banco non deve lasciare processi accesi
            bench.check(f'errore durante lo scenario: {error!r}', False)
        finally:
            bench.stop_all()
        print(f'== {name}: {bench.checks - bench.failed} controlli riusciti su {bench.checks}', flush=True)
        total_failed += bench.failed
        time.sleep(1)
    sys.exit(1 if total_failed else 0)


if __name__ == '__main__':
    main()
