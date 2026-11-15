"""Registro per le analisi: tutti gli stati del robot e, durante le missioni, la mappa dal vivo (docs/app.md, sezione 8.9).

Il robot manda già lo stato completo ogni 100 ms mentre lavora (ogni 10 s a riposo): qui il server se lo tiene, invece
di conservare solo l'ultimo. Una riga JSON per evento, in file gzip, uno al giorno, nella cartella config.RECORDER_DIR:

    {"t": <ora del server, secondi>, "state": {...stato del robot com'è arrivato...}}
    {"t": <ora del server>, "map_version": <versione della mappa dal vivo>, "map_rtm": "<file RTM1 in base64>"}

Regola di fondo: il registro non deve mai disturbare il resto. La richiesta del robot fa solo json.dumps e mette la riga
in una coda limitata (se è piena la riga si scarta); scrittura, compressione e pulizia le fa un thread a parte, e
qualsiasi errore lì dentro si segnala nel log del servizio e si ignora. Ogni blocco di righe è un membro gzip
completo scritto in una volta: se il server si spegne di colpo si perdono al massimo gli ultimi secondi.

Lettura dal PC: zcat stati-AAAA-MM-GG.jsonl.gz | head
"""
import base64
import gzip
import json
import os
import queue
import shutil
import sys
import threading
import time

import config


class Recorder:
    def __init__(self, data_dir, live_map):
        self.dir = os.path.join(data_dir, config.RECORDER_DIR)
        self.live_map = live_map
        self.lines = queue.Queue(maxsize=config.RECORDER_QUEUE_MAX)
        self.stop_event = threading.Event()
        self.thread = None
        self.mission_active = False         # dall'ultimo stato: le istantanee della mappa si fanno solo in missione
        self.dropped = 0                    # righe scartate perché la coda era piena (si segnalano a ogni scrittura)
        self.reported_errors = set()        # tipi di errore già segnalati nel log, per non riempirlo
        self.last_map_t = 0.0
        self.last_map_version = None
        self.was_mission_active = False

    # ------------------------------------------------------------------------------------- dalla richiesta del robot

    def add_state(self, state):
        """Chiamata da POST /api/device/state: veloce, non blocca e non solleva mai eccezioni."""
        try:
            navigation = state.get("navigation")
            self.mission_active = isinstance(navigation, dict) and navigation.get("mission", "none") != "none"
            line = json.dumps({"t": round(time.time(), 3), "state": state}, separators=(",", ":"))
            self.lines.put_nowait(line)
        except queue.Full:
            self.dropped += 1
        except Exception as error:      # noqa: BLE001 - il registro non deve mai far fallire la richiesta del robot
            self.report("add_state", error)

    # ------------------------------------------------------------------------------------- thread di scrittura

    def start(self):
        self.thread = threading.Thread(target=self.loop, name="recorder", daemon=True)
        self.thread.start()

    def stop(self):
        self.stop_event.set()
        if self.thread:
            self.thread.join(config.RECORDER_STOP_TIMEOUT_S)

    def loop(self):
        while not self.stop_event.wait(config.RECORDER_FLUSH_INTERVAL_S):
            self.flush()
        self.flush()                        # alla chiusura: quello che è rimasto in coda

    def flush(self):
        """Prende le righe in coda (più l'istantanea della mappa, se è il momento) e le scrive come un membro gzip."""
        try:
            batch = []
            while True:
                try:
                    batch.append(self.lines.get_nowait())
                except queue.Empty:
                    break
            map_line = self.map_snapshot_line()
            if map_line:
                batch.append(map_line)
            if self.dropped:
                self.report("coda", f"{self.dropped} righe scartate (coda piena)", once=False)
                self.dropped = 0
            if not batch or not self.can_write():
                return
            os.makedirs(self.dir, exist_ok=True)
            data = gzip.compress(("\n".join(batch) + "\n").encode("utf-8"), config.RECORDER_GZIP_LEVEL)
            with open(self.today_path(), "ab") as day_file:
                day_file.write(data)
            self.reported_errors.discard("scrittura")     # se torna a sbagliare si segnala di nuovo
            self.enforce_size_limit()
        except Exception as error:      # noqa: BLE001 - errori del disco: si segnalano e si va avanti
            self.report("scrittura", error)

    def map_snapshot_line(self):
        """La mappa dal vivo ogni RECORDER_MAP_INTERVAL_S durante una missione, se è cambiata, e una alla fine."""
        active = self.mission_active
        ended = self.was_mission_active and not active
        self.was_mission_active = active
        now = time.time()
        if not ended and (not active or now - self.last_map_t < config.RECORDER_MAP_INTERVAL_S):
            return None
        with self.live_map.lock:
            version = self.live_map.version
            if version == self.last_map_version:
                return None
            data = self.live_map.full_rtm()
        self.last_map_t = now
        self.last_map_version = version
        return json.dumps({"t": round(now, 3), "map_version": version,
                           "map_rtm": base64.b64encode(data).decode("ascii")}, separators=(",", ":"))

    # ------------------------------------------------------------------------------------- limiti del disco

    def today_path(self):
        name = config.RECORDER_FILE_PREFIX + time.strftime("%Y-%m-%d") + config.RECORDER_FILE_EXTENSION
        return os.path.join(self.dir, name)

    def can_write(self):
        """Non si scrive se il disco ha poco spazio libero."""
        free = shutil.disk_usage(os.path.dirname(self.dir)).free
        if free < config.RECORDER_MIN_FREE_BYTES:
            self.report("spazio", f"solo {free // (1024 * 1024)} MB liberi: il registro non scrive")
            return False
        self.reported_errors.discard("spazio")
        return True

    def files(self):
        """I file del registro, dal più vecchio al più nuovo (il nome contiene la data)."""
        names = sorted(name for name in os.listdir(self.dir)
                       if name.startswith(config.RECORDER_FILE_PREFIX) and name.endswith(config.RECORDER_FILE_EXTENSION))
        return [os.path.join(self.dir, name) for name in names]

    def enforce_size_limit(self):
        """Oltre RECORDER_MAX_BYTES si cancellano i giorni più vecchi; il file di oggi, se da solo supera il tetto, si
        svuota (succede solo con un difetto: una giornata intera di pulizia è una piccola parte del tetto)."""
        paths = self.files()
        sizes = {path: os.path.getsize(path) for path in paths}
        total = sum(sizes.values())
        for path in paths:
            if total <= config.RECORDER_MAX_BYTES:
                return
            if path == self.today_path():
                os.truncate(path, 0)
                self.report("tetto", f"il registro di oggi da solo supera {config.RECORDER_MAX_BYTES} byte: svuotato")
            else:
                os.remove(path)
            total -= sizes[path]

    def report(self, what, error, once=True):
        """Scrive nel log del servizio (journald). Con once, un tipo di errore si scrive una volta sola finché non
        torna tutto a posto: un disco pieno non riempie il log con una riga ogni pochi secondi."""
        if once and what in self.reported_errors:
            return
        self.reported_errors.add(what)
        print(f"registro: {what}: {error}", file=sys.stderr, flush=True)
