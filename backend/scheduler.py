"""Pulizie programmate (docs/app.md, sezione 8.7): un thread controlla una volta al minuto e all'ora giusta manda start."""
import datetime
import threading
import time
import zoneinfo

import config

SECONDS_IN_MINUTE = 60


class Scheduler:
    def __init__(self, database, start_mission):
        """start_mission() manda il comando start come farebbe l'app e ritorna "ok" oppure il motivo per cui non è riuscito."""
        self.database = database
        self.start_mission = start_mission
        self.timezone = zoneinfo.ZoneInfo(config.SCHEDULE_TIMEZONE)
        self.last_check = None          # istante dell'ultimo controllo: un orario scatta se cade fra quello e adesso
        self.retry_schedule_id = None   # orario che sta riprovando, None se nessuno
        self.retries_left = 0
        self.next_retry = None
        self.stop_event = threading.Event()
        self.thread = None

    def now(self):
        return datetime.datetime.now(self.timezone)

    def enabled_schedule(self):
        """L'unico orario attivo, o None."""
        return self.database.query_one("SELECT * FROM schedules WHERE enabled = 1 ORDER BY id LIMIT 1")

    def is_due(self, schedule, previous, now):
        """True se l'orario cade fra il controllo di prima (escluso) e adesso (compreso), nell'ora locale del server."""
        hours, minutes = divmod(schedule["time_minutes"], 60)
        scheduled = now.replace(hour=hours, minute=minutes, second=0, microsecond=0)
        if not schedule["days_mask"] >> scheduled.weekday() & 1:
            return False
        return previous < scheduled <= now

    def attempt(self, schedule_id, now):
        """Un tentativo di avvio. busy (sta già pulendo) conta come riuscita; finiti i tentativi resta l'errore."""
        outcome = self.start_mission()
        if outcome in ("ok", "busy"):
            result = config.SCHEDULE_RESULT_OK
            self.retry_schedule_id = None
        elif self.retries_left > 0:
            result = config.SCHEDULE_RESULT_RETRYING
            self.retries_left -= 1
            self.next_retry = now + datetime.timedelta(seconds=config.SCHEDULE_RETRY_INTERVAL_S)
        else:
            result = outcome                # il motivo dell'ultimo fallimento: l'app lo mostra in evidenza
            self.retry_schedule_id = None
        self.database.execute("UPDATE schedules SET last_run_at = ?, last_result = ? WHERE id = ?",
                              (int(now.timestamp()), result, schedule_id))

    def tick(self, now=None):
        """Un controllo: fa partire la pulizia se è l'ora, oppure riprova quella che non è riuscita."""
        now = now or self.now()
        previous = self.last_check or now
        self.last_check = now
        schedule = self.enabled_schedule()

        # l'orario che stava riprovando è stato spento o cancellato: si smette
        if self.retry_schedule_id is not None and (schedule is None or schedule["id"] != self.retry_schedule_id):
            self.retry_schedule_id = None

        if schedule is not None and self.is_due(schedule, previous, now):
            self.retry_schedule_id = schedule["id"]
            self.retries_left = config.SCHEDULE_MAX_RETRIES
            self.attempt(schedule["id"], now)
        elif self.retry_schedule_id is not None and now >= self.next_retry:
            self.attempt(self.retry_schedule_id, now)

        # il registro dei comandi tiene solo gli ultimi giorni
        oldest = int(time.time()) - config.COMMAND_LOG_KEEP_DAYS * 24 * 3600
        self.database.execute("DELETE FROM command_log WHERE time < ?", (oldest,))

    def loop(self):
        while True:
            # si sveglia poco dopo l'inizio di ogni minuto
            wait = config.SCHEDULE_CHECK_INTERVAL_S - time.time() % config.SCHEDULE_CHECK_INTERVAL_S + 1
            if self.stop_event.wait(wait):
                return
            self.tick()

    def start(self):
        self.last_check = self.now()
        self.thread = threading.Thread(target=self.loop, name="scheduler", daemon=True)
        self.thread.start()

    def stop(self):
        self.stop_event.set()
        if self.thread:
            self.thread.join()
