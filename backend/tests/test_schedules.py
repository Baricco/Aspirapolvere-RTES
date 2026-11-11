"""Prove della programmazione delle pulizie: orari, un solo orario attivo, tentativi."""
import datetime
import unittest
import zoneinfo

import config
from tests.helpers import BackendTestCase

ROME = zoneinfo.ZoneInfo("Europe/Rome")
ALL_DAYS = [True] * 7
MONDAY_ONLY = [True] + [False] * 6


def at(day, hours, minutes, seconds=1):
    """Un istante di novembre 2026 nell'ora di Roma (il 9 è un lunedì)."""
    return datetime.datetime(2026, 11, day, hours, minutes, seconds, tzinfo=ROME)


class ScheduleApiTest(BackendTestCase):
    def setUp(self):
        super().setUp()
        self.user = self.login()

    def add(self, days=ALL_DAYS, time="08:30", enabled=False):
        response = self.client.post("/api/robot/schedules", headers=self.user,
                                    json={"days": days, "time": time, "enabled": enabled})
        self.assertEqual(response.status_code, 201)
        return response.get_json()

    def listed(self):
        return self.client.get("/api/robot/schedules", headers=self.user).get_json()

    def test_add_list_change_delete(self):
        schedule = self.add(days=MONDAY_ONLY, time="07:05")
        self.assertEqual(schedule, {"id": schedule["id"], "days": MONDAY_ONLY, "time": "07:05", "enabled": False,
                                    "last_run_at": None, "last_result": None})
        row = self.services.database.query_one("SELECT * FROM schedules")
        self.assertEqual((row["days_mask"], row["time_minutes"]), (1, 7 * 60 + 5))
        response = self.client.put(f"/api/robot/schedules/{schedule['id']}", headers=self.user,
                                   json={"days": [False] * 6 + [True], "time": "23:59"})
        self.assertEqual((response.get_json()["days"], response.get_json()["time"]), ([False] * 6 + [True], "23:59"))
        self.assertEqual(self.services.database.query_one("SELECT days_mask FROM schedules")["days_mask"], 64)
        self.assertEqual(self.client.delete(f"/api/robot/schedules/{schedule['id']}", headers=self.user).status_code, 200)
        self.assertEqual(self.listed(), [])
        self.assertEqual(self.client.delete(f"/api/robot/schedules/{schedule['id']}", headers=self.user).status_code, 404)
        self.assertEqual(self.client.put(f"/api/robot/schedules/{schedule['id']}", headers=self.user, json={"enabled": True}).status_code, 404)

    def test_only_one_enabled_at_a_time(self):
        first = self.add(time="08:00", enabled=True)
        second = self.add(time="09:00", enabled=True)       # aggiunto acceso: spegne l'altro
        self.assertEqual({entry["id"]: entry["enabled"] for entry in self.listed()}, {first["id"]: False, second["id"]: True})
        third = self.add(time="10:00")
        self.client.put(f"/api/robot/schedules/{third['id']}", headers=self.user, json={"enabled": True})
        self.assertEqual([entry["enabled"] for entry in self.listed()], [False, False, True])
        # spegnere quello acceso disattiva la programmazione
        self.client.put(f"/api/robot/schedules/{third['id']}", headers=self.user, json={"enabled": False})
        self.assertEqual([entry["enabled"] for entry in self.listed()], [False, False, False])
        # cambiare l'ora di uno spento non tocca quello acceso
        self.client.put(f"/api/robot/schedules/{first['id']}", headers=self.user, json={"enabled": True})
        self.client.put(f"/api/robot/schedules/{second['id']}", headers=self.user, json={"time": "09:30"})
        self.assertEqual([entry["enabled"] for entry in self.listed()], [True, False, False])

    def test_bad_parameters(self):
        for body in ({"days": ALL_DAYS}, {"time": "08:00"}, {"days": [True] * 6, "time": "08:00"},
                     {"days": ALL_DAYS, "time": "24:00"}, {"days": ALL_DAYS, "time": "8"},
                     {"days": ALL_DAYS, "time": "08:00", "enabled": "si"}, {"days": [1] * 7, "time": "08:00"}, []):
            self.assertEqual(self.client.post("/api/robot/schedules", headers=self.user, json=body).status_code, 400, body)


class SchedulerTest(BackendTestCase):
    def setUp(self):
        super().setUp()
        self.outcomes = []          # esiti che il robot finto darà ai prossimi start
        self.starts = 0
        self.scheduler = self.services.scheduler
        self.scheduler.start_mission = self.fake_start
        self.user = self.login()

    def fake_start(self):
        self.starts += 1
        return self.outcomes.pop(0) if self.outcomes else "ok"

    def add(self, days=ALL_DAYS, time="08:30", enabled=True):
        return self.client.post("/api/robot/schedules", headers=self.user,
                                json={"days": days, "time": time, "enabled": enabled}).get_json()["id"]

    def result(self, schedule_id):
        row = self.services.database.query_one("SELECT last_result, last_run_at FROM schedules WHERE id = ?", (schedule_id,))
        return row["last_result"]

    def test_starts_at_the_right_minute_once(self):
        schedule_id = self.add(time="08:30")
        self.scheduler.tick(at(9, 8, 28))
        self.scheduler.tick(at(9, 8, 29))
        self.assertEqual(self.starts, 0)
        self.scheduler.tick(at(9, 8, 30))
        self.assertEqual(self.starts, 1)
        self.assertEqual(self.result(schedule_id), "ok")
        self.scheduler.tick(at(9, 8, 31))
        self.scheduler.tick(at(9, 8, 32))
        self.assertEqual(self.starts, 1)
        self.scheduler.tick(at(10, 8, 29))
        self.scheduler.tick(at(10, 8, 30))                  # il giorno dopo riparte
        self.assertEqual(self.starts, 2)

    def test_only_on_the_chosen_days(self):
        self.add(days=MONDAY_ONLY, time="08:30")
        self.scheduler.tick(at(10, 8, 29))
        self.scheduler.tick(at(10, 8, 30))                  # martedì
        self.assertEqual(self.starts, 0)
        self.scheduler.tick(at(16, 8, 29))
        self.scheduler.tick(at(16, 8, 30))                  # il lunedì dopo
        self.assertEqual(self.starts, 1)

    def test_disabled_schedules_do_not_start(self):
        self.add(time="08:30", enabled=False)
        self.scheduler.tick(at(9, 8, 29))
        self.scheduler.tick(at(9, 8, 30))
        self.assertEqual(self.starts, 0)

    def test_busy_counts_as_done(self):
        schedule_id = self.add(time="08:30")
        self.outcomes = ["busy"]
        self.scheduler.tick(at(9, 8, 29))
        self.scheduler.tick(at(9, 8, 30))
        self.assertEqual(self.result(schedule_id), "ok")
        self.scheduler.tick(at(9, 8, 31))
        self.assertEqual(self.starts, 1)

    def test_retries_once_a_minute_then_succeeds(self):
        schedule_id = self.add(time="08:30")
        self.outcomes = ["robot_unreachable", "robot_unreachable", "ok"]
        self.scheduler.tick(at(9, 8, 29))
        self.scheduler.tick(at(9, 8, 30))
        self.assertEqual((self.starts, self.result(schedule_id)), (1, "retrying"))
        self.scheduler.tick(at(9, 8, 30, 30))               # non è ancora passato un minuto
        self.assertEqual(self.starts, 1)
        self.scheduler.tick(at(9, 8, 31))
        self.assertEqual((self.starts, self.result(schedule_id)), (2, "retrying"))
        self.scheduler.tick(at(9, 8, 32))
        self.assertEqual((self.starts, self.result(schedule_id)), (3, "ok"))
        self.scheduler.tick(at(9, 8, 33))
        self.assertEqual(self.starts, 3)

    def test_gives_up_after_ten_retries_with_the_error(self):
        schedule_id = self.add(time="08:30")
        self.outcomes = ["robot_unreachable"] * 20
        self.scheduler.tick(at(9, 8, 29))
        for minute in range(30, 50):
            self.scheduler.tick(at(9, 8, minute))
        # il primo tentativo più 10 nuovi tentativi, uno al minuto, poi basta
        self.assertEqual(self.starts, 1 + config.SCHEDULE_MAX_RETRIES)
        self.assertEqual(self.result(schedule_id), "robot_unreachable")
        entry = self.client.get("/api/robot/schedules", headers=self.user).get_json()[0]
        self.assertEqual(entry["last_result"], "robot_unreachable")
        self.assertEqual(entry["last_run_at"], int(at(9, 8, 40).timestamp()))

    def test_turning_the_schedule_off_stops_the_retries(self):
        schedule_id = self.add(time="08:30")
        self.outcomes = ["robot_unreachable"] * 5
        self.scheduler.tick(at(9, 8, 29))
        self.scheduler.tick(at(9, 8, 30))
        self.client.put(f"/api/robot/schedules/{schedule_id}", headers=self.user, json={"enabled": False})
        self.scheduler.tick(at(9, 8, 31))
        self.scheduler.tick(at(9, 8, 32))
        self.assertEqual(self.starts, 1)

    def test_a_late_check_does_not_skip_the_start(self):
        self.add(time="08:30")
        self.scheduler.tick(at(9, 8, 29))
        self.scheduler.tick(at(9, 8, 32))                   # il server era occupato: il controllo arriva tardi
        self.assertEqual(self.starts, 1)

    def test_summer_time(self):
        # 29 marzo 2026: alle 2 le lancette vanno alle 3. Un orario alle 8:30 parte alle 8:30 locali, prima e dopo
        schedule_id = self.add(time="08:30")
        for day in (28, 29):
            self.scheduler.tick(datetime.datetime(2026, 3, day, 8, 29, 1, tzinfo=ROME))
            self.scheduler.tick(datetime.datetime(2026, 3, day, 8, 30, 1, tzinfo=ROME))
        self.assertEqual(self.starts, 2)
        self.assertEqual(self.result(schedule_id), "ok")


class ScheduledStartTest(BackendTestCase):
    """Lo start vero della programmazione: passa dalla stessa strada dei comandi dell'app."""

    def setUp(self):
        super().setUp()
        self.user = self.login()

    def log(self):
        return self.client.get("/api/robot/log", headers=self.user).get_json()

    def test_start_reaches_the_robot_and_the_log_says_scheduled(self):
        self.robot_online()
        thread, taken = self.robot_answer({"ok": True})
        self.assertEqual(self.services.scheduled_start(), "ok")
        thread.join(2)
        self.assertEqual(taken[0][0]["type"], "start")
        entry = self.log()[0]
        self.assertEqual((entry["type"], entry["status"], entry["user"], entry["scheduled"]), ("start", "ok", None, True))

    def test_reasons_of_failure(self):
        self.assertEqual(self.services.scheduled_start(), "robot_unreachable")
        self.robot_online()
        thread, _ = self.robot_answer({"ok": False, "reason": "busy", "message": ""})
        self.assertEqual(self.services.scheduled_start(), "busy")
        thread.join(2)
        # qualcuno tiene il telecomando: la programmazione è rifiutata come gli altri utenti
        self.command("/api/robot/remote/take", self.user)
        self.assertEqual(self.services.scheduled_start(), "remote_in_use")


if __name__ == "__main__":
    unittest.main()
