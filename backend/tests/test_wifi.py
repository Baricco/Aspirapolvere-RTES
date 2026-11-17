"""Prove del cambio della rete Wi-Fi dall'app (docs/wifi.md, sezioni 7.3 e 9)."""
import json
import unittest

from helpers import BackendTestCase

NETWORKS = {"scanned_at": 1763390000,
            "networks": [{"ssid": "Casa", "signal_dbm": -50, "security": "wpa", "in_use": True},
                         {"ssid": "Caffè ☕", "signal_dbm": -80, "security": "open", "in_use": False}]}


class WifiTest(BackendTestCase):
    def setUp(self):
        super().setUp()
        self.user = self.login()

    def log(self):
        return self.client.get("/api/robot/log", headers=self.user).get_json()

    def test_scan_answers_with_the_networks_of_the_robot(self):
        response, taken = self.command("/api/robot/wifi/networks?refresh=1", self.user, method="get",
                                       robot_result={"ok": True, "data": NETWORKS})
        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.get_json(), NETWORKS)
        self.assertEqual((taken[0]["type"], taken[0]["refresh"]), ("wifi_scan", True))
        response, taken = self.command("/api/robot/wifi/networks", self.user, method="get",
                                       robot_result={"ok": True, "data": NETWORKS})
        self.assertFalse(taken[0]["refresh"])
        self.assertEqual(self.log(), [])        # le scansioni non vanno nel registro

    def test_connect_passes_the_password_to_the_robot_but_never_to_the_log(self):
        body = {"ssid": "Casa", "security": "wpa", "password": "password-casa"}
        response, taken = self.command("/api/robot/wifi/connect", self.user, json=body,
                                       robot_result={"ok": True, "data": {"change_number": 3}})
        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.get_json(), {"change_number": 3})
        self.assertEqual({key: taken[0][key] for key in ("type", "ssid", "security", "password")},
                         {"type": "wifi_connect", "ssid": "Casa", "security": "wpa", "password": "password-casa"})
        entry = self.log()[0]
        self.assertEqual((entry["type"], entry["params"]), ("wifi_connect", {"ssid": "Casa", "security": "wpa"}))
        row = self.services.database.query_one("SELECT params FROM command_log WHERE type = 'wifi_connect'")
        self.assertNotIn("password-casa", row["params"])
        self.assertEqual(json.loads(row["params"]), {"ssid": "Casa", "security": "wpa"})

    def test_open_network_without_password(self):
        response, taken = self.command("/api/robot/wifi/connect", self.user,
                                       json={"ssid": "Caffè ☕", "security": "open"},
                                       robot_result={"ok": True, "data": {"change_number": 1}})
        self.assertEqual(response.status_code, 200)
        self.assertEqual((taken[0]["ssid"], taken[0]["password"]), ("Caffè ☕", ""))

    def test_refused_by_the_robot(self):
        response, _ = self.command("/api/robot/wifi/connect", self.user,
                                   json={"ssid": "Casa", "security": "wpa", "password": "12345678"},
                                   robot_result={"ok": False, "reason": "not_resting",
                                                 "message": "Maurice deve essere a riposo per cambiare rete"})
        self.assertEqual(response.status_code, 409)
        self.assertEqual(response.get_json()["reason"], "not_resting")
        self.assertEqual((self.log()[0]["status"], self.log()[0]["reason"]), ("rejected", "not_resting"))

    def test_bad_parameters_never_reach_the_robot(self):
        cases = [({"security": "wpa", "password": "12345678"}, "invalid_ssid"),
                 ({"ssid": "", "security": "wpa", "password": "12345678"}, "invalid_ssid"),
                 ({"ssid": "x" * 33, "security": "wpa", "password": "12345678"}, "invalid_ssid"),
                 ({"ssid": "è" * 17, "security": "wpa", "password": "12345678"}, "invalid_ssid"),   # 34 byte
                 ({"ssid": "Casa", "security": "wep", "password": "12345678"}, "invalid_parameters"),
                 ({"ssid": "Casa", "security": "wpa", "password": "corta"}, "invalid_password"),
                 ({"ssid": "Casa", "security": "wpa", "password": "x" * 64}, "invalid_password"),
                 ({"ssid": "Casa", "security": "wpa", "password": "pässword"}, "invalid_password"),
                 ({"ssid": "Casa", "security": "wpa"}, "invalid_password"),
                 ({"ssid": "Ospiti", "security": "open", "password": "12345678"}, "invalid_password")]
        self.robot_online()
        for body, reason in cases:
            response = self.client.post("/api/robot/wifi/connect", headers=self.user, json=body)
            self.assertEqual(response.status_code, 400, body)
            self.assertEqual(response.get_json()["reason"], reason, body)
        self.assertEqual(self.services.link.queued_commands, [])
        self.assertEqual(self.log(), [])

    def test_needs_a_session(self):
        self.assertEqual(self.client.get("/api/robot/wifi/networks").status_code, 401)
        self.assertEqual(self.client.post("/api/robot/wifi/connect", json={}).status_code, 401)

    def test_robot_ping(self):
        self.assertEqual(self.client.get("/api/device/ping", headers=self.robot).status_code, 200)
        self.assertEqual(self.client.get("/api/device/ping").status_code, 401)
        self.assertEqual(self.client.get("/api/device/ping",
                                         headers={"Authorization": "Bearer sbagliato"}).status_code, 401)


if __name__ == "__main__":
    unittest.main()
