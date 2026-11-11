"""Prove dello stato (richiesta lunga), dei comandi (200, 409, 503, 504), del telecomando e del registro."""
import gzip
import json
import time
import unittest

import config
from tests.helpers import BackendTestCase

ROBOT_STATE = {
    "format": 1, "robot_time": 1762700000, "uptime_s": 12, "remote_control": False,
    "navigation": {"state": "sleep", "mission": "none", "interrupted_mission": "none", "mission_number": 0,
                   "mission_duration_s": 0},
    "position": {"x": 0.0, "y": 0.0, "theta_deg": 0.0, "v": 0.0, "omega": 0.0, "valid": True},
    "map": {"state": "normal", "save_failed": False, "started_on_base": True, "file_crc": "9c41e2a0",
            "forbidden_zones": [{"x": -0.365, "y": -0.22, "w": 0.26, "h": 0.44}, {"x": 1.0, "y": 1.0, "w": 0.5, "h": 0.5}]},
    "tasks": [{"name": name, "deadline_misses": 0, "missed_activations": 0}
              for name in ("movement", "safety", "lidar", "navigation", "mapping")],
}


class StateTest(BackendTestCase):
    def setUp(self):
        super().setUp()
        self.user = self.login()

    def get_state(self, query=""):
        return self.client.get("/api/robot/state" + query, headers=self.user)

    def test_first_request_answers_at_once_even_without_robot(self):
        start = time.monotonic()
        body = self.get_state().get_json()
        self.assertLess(time.monotonic() - start, 0.2)
        self.assertFalse(body["online"])
        self.assertIsNone(body["last_contact"])
        self.assertIsNone(body["remote_control_user"])
        self.assertIn("map_version", body)
        self.assertIn("state_number", body)

    def test_server_fields_are_added_to_the_robot_state(self):
        self.robot_post_state(ROBOT_STATE)
        body = self.get_state().get_json()
        self.assertEqual(body["navigation"], ROBOT_STATE["navigation"])
        self.assertTrue(body["online"])
        self.assertAlmostEqual(body["last_contact"], time.time(), delta=2)

    def test_long_poll_304_when_nothing_changes(self):
        number = self.get_state().get_json()["state_number"]
        start = time.monotonic()
        response = self.get_state(f"?after={number}")
        self.assertEqual(response.status_code, 304)
        self.assertGreaterEqual(time.monotonic() - start, config.STATE_LONG_POLL_S - 0.05)

    def test_long_poll_wakes_on_a_new_state(self):
        number = self.get_state().get_json()["state_number"]
        thread, results = self.in_thread(lambda: self.app.test_client().get(f"/api/robot/state?after={number}",
                                                                           headers=self.user))
        time.sleep(0.1)
        self.assertFalse(results)           # sta aspettando
        self.robot_post_state(ROBOT_STATE)
        thread.join(2)
        self.assertEqual(results[0].status_code, 200)
        self.assertGreater(results[0].get_json()["state_number"], number)

    def test_min_interval_between_answers(self):
        self.robot_post_state(ROBOT_STATE)
        number = self.get_state().get_json()["state_number"]
        self.robot_post_state(ROBOT_STATE)
        # lo stato nuovo c'è già, ma la risposta non arriva prima di 200 ms dall'ultima a questa app
        start = time.monotonic()
        response = self.get_state(f"?after={number}&min_interval_ms=200")
        self.assertEqual(response.status_code, 200)
        self.assertGreaterEqual(time.monotonic() - start, 0.15)
        # senza intervallo minimo arriva subito
        self.robot_post_state(ROBOT_STATE)
        start = time.monotonic()
        self.get_state(f"?after={response.get_json()['state_number']}")
        self.assertLess(time.monotonic() - start, 0.1)

    def test_online_change_counts_as_a_new_state(self):
        config.STATE_LONG_POLL_S = 3        # più lunga del tempo dopo cui il robot risulta offline
        self.robot_post_state(ROBOT_STATE)
        body = self.get_state().get_json()
        self.assertTrue(body["online"])
        thread, results = self.in_thread(lambda: self.app.test_client().get(
            f"/api/robot/state?after={body['state_number']}", headers=self.user))
        time.sleep(config.ONLINE_TIMEOUT_S + 0.05)
        self.services.link.service_tick()   # nel backend vero lo fa il thread di servizio
        thread.join(2)
        self.assertEqual(results[0].status_code, 200)
        self.assertFalse(results[0].get_json()["online"])
        self.assertEqual(results[0].get_json()["navigation"], ROBOT_STATE["navigation"])     # resta l'ultimo stato

    def test_number_from_before_a_server_restart_answers_at_once(self):
        number = self.get_state().get_json()["state_number"]
        start = time.monotonic()
        self.assertEqual(self.get_state(f"?after={number + 1000}").status_code, 200)
        self.assertLess(time.monotonic() - start, 0.2)

    def test_bad_parameters(self):
        self.assertEqual(self.get_state("?after=abc").status_code, 400)
        self.assertEqual(self.get_state("?min_interval_ms=x").status_code, 400)

    def test_json_answers_are_gzipped(self):
        self.robot_post_state(ROBOT_STATE)
        response = self.client.get("/api/robot/state", headers=dict(self.user, **{"Accept-Encoding": "gzip"}))
        self.assertEqual(response.headers["Content-Encoding"], "gzip")
        self.assertEqual(json.loads(gzip.decompress(response.data))["navigation"], ROBOT_STATE["navigation"])
        # chi non lo chiede lo riceve in chiaro
        self.assertNotIn("Content-Encoding", self.get_state().headers)

    def test_last_state_is_written_to_disk_and_read_at_restart(self):
        import robot_link
        self.robot_post_state(ROBOT_STATE)
        self.services.link.save_last_state()
        restarted = robot_link.RobotLink(self.data_dir)
        body = restarted.current_state()
        self.assertEqual(body["navigation"], ROBOT_STATE["navigation"])
        self.assertFalse(body["online"])
        self.assertIsNotNone(body["last_contact"])

    def test_zones_without_the_base(self):
        self.robot_post_state(ROBOT_STATE)
        response = self.client.get("/api/robot/zones", headers=self.user)
        self.assertEqual(response.get_json(), [{"x": 1.0, "y": 1.0, "w": 0.5, "h": 0.5}])


class CommandTest(BackendTestCase):
    def setUp(self):
        super().setUp()
        self.user = self.login()

    def log(self):
        return self.client.get("/api/robot/log", headers=self.user).get_json()

    def test_every_command_reaches_the_robot_with_its_type(self):
        cases = [("post", "/api/robot/mission/start", None, "start", {}),
                 ("post", "/api/robot/mission/pause", None, "pause", {}),
                 ("post", "/api/robot/mission/resume", None, "resume", {}),
                 ("post", "/api/robot/mission/home", None, "home", {}),
                 ("post", "/api/robot/fault/reset", None, "fault_reset", {}),
                 ("post", "/api/robot/remote/take", None, "remote_take", {}),
                 ("post", "/api/robot/remote/move", {"v": 0.2, "omega": -0.5}, "remote_move", {"v": 0.2, "omega": -0.5}),
                 ("post", "/api/robot/remote/cleaning", {"brushes": True, "suction": False}, "remote_cleaning",
                  {"brushes": True, "suction": False}),
                 ("post", "/api/robot/remote/keepalive", None, "remote_keepalive", {}),
                 ("post", "/api/robot/remote/release", None, "remote_release", {}),
                 ("put", "/api/robot/zones", [{"x": 1, "y": 1, "w": 0.5, "h": 0.5}], "zones",
                  {"zones": [{"x": 1.0, "y": 1.0, "w": 0.5, "h": 0.5}]})]
        for method, path, body, command_type, params in cases:
            response, taken = self.command(path, self.user, method=method, json=body)
            self.assertEqual(response.status_code, 200, path)
            self.assertEqual(response.get_json(), {"ok": True})
            self.assertEqual(len(taken), 1, path)
            self.assertEqual(taken[0]["type"], command_type)
            self.assertIsInstance(taken[0]["id"], int)
            self.assertEqual({key: value for key, value in taken[0].items() if key not in ("id", "type")}, params)
        logged = [case[3] for case in reversed(cases) if case[3] != "remote_keepalive"]     # i keepalive non si registrano
        self.assertEqual([entry["type"] for entry in self.log()], logged)

    def test_remote_moves_are_logged_only_when_they_change(self):
        for v in (0.2, 0.2, 0.2, 0.0, 0.0, 0.2):
            self.command("/api/robot/remote/move", self.user, json={"v": v, "omega": 0})
            self.command("/api/robot/remote/keepalive", self.user)
        entries = [entry for entry in self.log() if entry["type"] == "remote_move"]
        self.assertEqual([entry["params"]["v"] for entry in reversed(entries)], [0.2, 0.0, 0.2])
        self.assertEqual([entry["type"] for entry in self.log() if entry["type"] == "remote_keepalive"], [])

    def test_refused_by_the_robot_409(self):
        response, _ = self.command("/api/robot/mission/start", self.user,
                                   robot_result={"ok": False, "reason": "busy", "message": "Sto già pulendo."})
        self.assertEqual(response.status_code, 409)
        self.assertEqual(response.get_json(), {"reason": "busy", "message": "Sto già pulendo."})
        entry = self.log()[0]
        self.assertEqual((entry["type"], entry["status"], entry["reason"], entry["user"], entry["scheduled"]),
                         ("start", "rejected", "busy", "anna", False))

    def test_robot_offline_503_at_once(self):
        start = time.monotonic()
        response = self.client.post("/api/robot/mission/start", headers=self.user)
        self.assertEqual(response.status_code, 503)
        self.assertEqual(response.get_json()["reason"], "robot_unreachable")
        self.assertLess(time.monotonic() - start, 0.2)
        self.assertEqual(self.log()[0]["status"], "robot_unreachable")

    def test_command_not_taken_503_and_dropped(self):
        self.robot_online()
        start = time.monotonic()
        response = self.client.post("/api/robot/mission/start", headers=self.user)
        self.assertEqual(response.status_code, 503)
        self.assertGreaterEqual(time.monotonic() - start, config.COMMAND_TIMEOUT_S - 0.05)
        # il comando è stato buttato: quando il robot torna non lo riceve, quindi non è mai stato eseguito
        commands = self.client.get("/api/device/commands", headers=self.robot).get_json()["commands"]
        self.assertEqual(commands, [])

    def test_taken_but_no_result_504_then_late_result_fixes_the_log(self):
        self.robot_online()
        thread, taken = self.robot_answer(None)
        response = self.client.post("/api/robot/mission/start", headers=self.user)
        thread.join(2)
        self.assertEqual(response.status_code, 504)
        self.assertEqual(response.get_json()["reason"], "no_result")
        self.assertEqual(self.log()[0]["status"], "no_result")
        command_id = taken[0][0]["id"]
        late = self.client.post(f"/api/device/commands/{command_id}/result", headers=self.robot,
                                json={"ok": False, "reason": "busy", "message": ""})
        self.assertEqual(late.status_code, 200)
        self.assertEqual((self.log()[0]["status"], self.log()[0]["reason"]), ("rejected", "busy"))
        # un secondo esito per lo stesso comando non esiste più
        again = self.client.post(f"/api/device/commands/{command_id}/result", headers=self.robot, json={"ok": True})
        self.assertEqual(again.status_code, 404)

    def test_bad_parameters_400(self):
        self.robot_online()
        for path, body in (("/api/robot/remote/move", {"v": "veloce", "omega": 0}),
                           ("/api/robot/remote/move", {"v": 0.1}),
                           ("/api/robot/remote/move", {"v": float("inf"), "omega": 0}),
                           ("/api/robot/remote/cleaning", {"brushes": 1, "suction": True}),
                           ("/api/robot/remote/cleaning", None)):
            self.assertEqual(self.client.post(path, headers=self.user, json=body).status_code, 400, body)
        for body in ({"x": 1}, [{"x": 1, "y": 1, "w": 1}], [[1, 2, 3, 4]], "zone"):
            self.assertEqual(self.client.put("/api/robot/zones", headers=self.user, json=body).status_code, 400, body)
        self.assertEqual(self.client.post("/api/device/commands/1/result", headers=self.robot, json={}).status_code, 400)

    def test_zones_refused_by_the_server(self):
        self.robot_online()
        too_many = [{"x": 0.1 * index, "y": 1, "w": 0.5, "h": 0.5} for index in range(config.MAX_USER_ZONES + 1)]
        response = self.client.put("/api/robot/zones", headers=self.user, json=too_many)
        self.assertEqual((response.status_code, response.get_json()["reason"]), (409, "too_many_zones"))
        for zone in ({"x": 25.5, "y": 0, "w": 0.5, "h": 0.5}, {"x": 0, "y": 0, "w": 0.01, "h": 0.5},
                     {"x": -26, "y": 0, "w": 1, "h": 1}):
            response = self.client.put("/api/robot/zones", headers=self.user, json=[zone])
            self.assertEqual((response.status_code, response.get_json()["reason"]), (409, "outside_map"), zone)

    def test_unknown_resource_404_with_reason(self):
        response = self.client.get("/api/robot/niente", headers=self.user)
        self.assertEqual(response.status_code, 404)
        self.assertEqual(response.get_json()["reason"], "not_found")

    def test_an_old_robot_request_does_not_take_commands(self):
        # la connessione vecchia del robot è morta ma il server non lo sa: la richiesta lunga nuova la chiude
        self.robot_online()
        old_thread, old_taken = self.robot_answer({"ok": True})
        time.sleep(0.05)
        new_thread, new_taken = self.robot_answer({"ok": True})
        time.sleep(0.05)
        response = self.client.post("/api/robot/mission/start", headers=self.user)
        old_thread.join(2)
        new_thread.join(2)
        self.assertEqual(response.status_code, 200)
        self.assertEqual(old_taken[0], [])
        self.assertEqual(len(new_taken[0]), 1)

    def test_log_pages(self):
        for _ in range(5):
            self.client.post("/api/robot/mission/start", headers=self.user)
        entries = self.log()
        self.assertEqual(len(entries), 5)
        self.assertEqual([entry["id"] for entry in entries], sorted((entry["id"] for entry in entries), reverse=True))
        page = self.client.get(f"/api/robot/log?before={entries[1]['id']}&limit=2", headers=self.user).get_json()
        self.assertEqual([entry["id"] for entry in page], [entries[2]["id"], entries[3]["id"]])

    def test_old_log_entries_are_deleted(self):
        self.client.post("/api/robot/mission/start", headers=self.user)
        self.client.post("/api/robot/mission/home", headers=self.user)
        old = int(time.time()) - (config.COMMAND_LOG_KEEP_DAYS + 1) * 24 * 3600
        self.services.database.execute("UPDATE command_log SET time = ? WHERE type = 'start'", (old,))
        self.services.scheduler.tick()
        self.assertEqual([entry["type"] for entry in self.log()], ["home"])


class RemoteControlTest(BackendTestCase):
    def setUp(self):
        super().setUp()
        self.anna = self.login("anna")
        self.bruno = self.login("bruno")

    def remote_user(self):
        return self.client.get("/api/robot/state", headers=self.anna).get_json()["remote_control_user"]

    def test_one_user_at_a_time(self):
        response, _ = self.command("/api/robot/remote/take", self.anna)
        self.assertEqual(response.status_code, 200)
        self.assertEqual(self.remote_user(), "anna")
        # l'altro utente: telecomando e missioni rifiutati dal server, con il nome di chi lo tiene
        for path in ("/api/robot/remote/take", "/api/robot/remote/keepalive", "/api/robot/mission/start",
                     "/api/robot/mission/pause", "/api/robot/mission/resume", "/api/robot/mission/home"):
            response = self.client.post(path, headers=self.bruno)
            self.assertEqual(response.status_code, 409, path)
            self.assertEqual(response.get_json()["reason"], "remote_in_use")
            self.assertEqual(response.get_json()["user"], "anna")
            self.assertIn("anna", response.get_json()["message"])
        response = self.client.post("/api/robot/remote/move", headers=self.bruno, json={"v": 0.1, "omega": 0})
        self.assertEqual(response.get_json()["reason"], "remote_in_use")
        response = self.client.post("/api/robot/remote/release", headers=self.bruno)
        self.assertEqual((response.status_code, response.get_json()["reason"]), (409, "not_remote_owner"))
        # le zone restano permesse
        response, taken = self.command("/api/robot/zones", self.bruno, method="put", json=[])
        self.assertEqual(response.status_code, 200)
        self.assertEqual(taken[0]["type"], "zones")
        # e anche l'azzeramento del guasto
        response, _ = self.command("/api/robot/fault/reset", self.bruno)
        self.assertEqual(response.status_code, 200)
        self.assertEqual(self.remote_user(), "anna")

    def test_any_remote_command_takes_it_when_free(self):
        response, taken = self.command("/api/robot/remote/move", self.bruno, json={"v": 0.1, "omega": 0.2})
        self.assertEqual(response.status_code, 200)
        self.assertEqual(taken[0]["type"], "remote_move")
        self.assertEqual(self.remote_user(), "bruno")

    def test_release(self):
        self.command("/api/robot/remote/take", self.anna)
        response, taken = self.command("/api/robot/remote/release", self.anna)
        self.assertEqual(response.status_code, 200)
        self.assertEqual(taken[0]["type"], "remote_release")
        self.assertIsNone(self.remote_user())
        response = self.client.post("/api/robot/remote/release", headers=self.anna)
        self.assertEqual(response.get_json()["reason"], "not_remote_owner")

    def test_timeout_without_remote_commands(self):
        self.command("/api/robot/remote/take", self.anna)
        time.sleep(config.REMOTE_IDLE_TIMEOUT_S * 0.6)
        self.command("/api/robot/remote/keepalive", self.anna)      # il keepalive rinnova il tempo
        time.sleep(config.REMOTE_IDLE_TIMEOUT_S * 0.6)
        self.assertEqual(self.remote_user(), "anna")
        number = self.client.get("/api/robot/state", headers=self.anna).get_json()["state_number"]
        time.sleep(config.REMOTE_IDLE_TIMEOUT_S * 0.5)
        self.services.link.service_tick()
        body = self.client.get("/api/robot/state", headers=self.anna).get_json()
        self.assertIsNone(body["remote_control_user"])
        self.assertGreater(body["state_number"], number)            # le app in attesa vedono il rilascio
        response, _ = self.command("/api/robot/remote/take", self.bruno)
        self.assertEqual(response.status_code, 200)

    def test_mission_command_of_the_owner_releases_then_runs(self):
        self.command("/api/robot/remote/take", self.anna)
        response, taken = self.command("/api/robot/mission/home", self.anna)
        self.assertEqual(response.status_code, 200)
        self.assertEqual(taken[0]["type"], "home")
        self.assertIsNone(self.remote_user())

    def test_not_taken_if_the_robot_is_unreachable(self):
        response = self.client.post("/api/robot/remote/take", headers=self.anna)
        self.assertEqual(response.status_code, 503)
        self.assertIsNone(self.remote_user())

    def test_refused_by_the_robot_keeps_the_lock(self):
        # posizione persa: il robot rifiuta il movimento, ma il telecomando resta a chi lo ha preso
        response, _ = self.command("/api/robot/remote/move", self.anna, json={"v": 0.1, "omega": 0},
                                   robot_result={"ok": False, "reason": "position_lost", "message": "Posizione persa."})
        self.assertEqual((response.status_code, response.get_json()["reason"]), (409, "position_lost"))
        self.assertEqual(self.remote_user(), "anna")


if __name__ == "__main__":
    unittest.main()
