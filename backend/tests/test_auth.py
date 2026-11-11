"""Prove dell'accesso: login, blocco, JWT, sessioni, cookie del web, controllo di Origin, token del robot."""
import os
import time
import unittest

import jwt

import auth
import config
from tests.helpers import BASE_URL, USER_PASSWORD, BackendTestCase


class LoginTest(BackendTestCase):
    def test_login_and_me(self):
        headers = self.login("anna")
        response = self.client.get("/api/auth/me", headers=headers)
        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.get_json(), {"username": "anna"})

    def test_login_returns_token_and_expiry(self):
        body = self.client.post("/api/auth/login", json={"username": "anna", "password": USER_PASSWORD}).get_json()
        self.assertAlmostEqual(body["expires_at"], time.time() + config.JWT_LIFETIME_S, delta=5)
        claims = jwt.decode(body["token"], self.services.jwt_key, algorithms=["HS256"])
        self.assertEqual(claims["exp"], body["expires_at"])
        row = self.services.database.query_one("SELECT * FROM sessions WHERE id = ?", (int(claims["jti"]),))
        self.assertEqual(str(row["user_id"]), claims["sub"])

    def test_wrong_password_and_unknown_user(self):
        for username, password in (("anna", "sbagliata"), ("nessuno", USER_PASSWORD)):
            response = self.client.post("/api/auth/login", json={"username": username, "password": password})
            self.assertEqual(response.status_code, 401)
            self.assertEqual(response.get_json()["reason"], "invalid_credentials")

    def test_bad_body(self):
        self.assertEqual(self.client.post("/api/auth/login", json={"username": "anna"}).status_code, 400)
        self.assertEqual(self.client.post("/api/auth/login", data="x").status_code, 400)

    def test_lock_after_failed_logins(self):
        for _ in range(config.LOGIN_MAX_FAILED):
            response = self.client.post("/api/auth/login", json={"username": "anna", "password": "sbagliata"})
            self.assertEqual(response.status_code, 401)
        # bloccato: anche la password giusta riceve 429
        response = self.client.post("/api/auth/login", json={"username": "anna", "password": USER_PASSWORD})
        self.assertEqual(response.status_code, 429)
        self.assertEqual(response.get_json()["reason"], "login_locked")
        # l'altro utente non è bloccato
        self.login("bruno")
        # finito il blocco si rientra
        time.sleep(config.LOGIN_LOCK_S + 1)
        self.login("anna")

    def test_failed_logins_reset_by_success(self):
        for _ in range(config.LOGIN_MAX_FAILED - 1):
            self.client.post("/api/auth/login", json={"username": "anna", "password": "sbagliata"})
        self.login("anna")
        for _ in range(config.LOGIN_MAX_FAILED - 1):
            self.client.post("/api/auth/login", json={"username": "anna", "password": "sbagliata"})
        self.login("anna")


class SessionTest(BackendTestCase):
    def test_requests_need_a_session(self):
        for method, path in (("get", "/api/auth/me"), ("get", "/api/robot/state"), ("post", "/api/robot/mission/start"),
                             ("get", "/api/robot/maps"), ("post", "/api/auth/refresh"), ("post", "/api/auth/logout")):
            response = getattr(self.client, method)(path)
            self.assertEqual(response.status_code, 401, path)
            self.assertEqual(response.get_json()["reason"], "unauthenticated")

    def test_invalid_tokens(self):
        other_key = jwt.encode({"sub": "1", "jti": "1", "exp": int(time.time()) + 60}, os.urandom(32), algorithm="HS256")
        expired = jwt.encode({"sub": "1", "jti": "1", "exp": int(time.time()) - 60}, self.services.jwt_key,
                             algorithm="HS256")
        for token in ("niente", other_key, expired):
            response = self.client.get("/api/auth/me", headers={"Authorization": "Bearer " + token})
            self.assertEqual(response.status_code, 401)

    def test_refresh_keeps_the_session(self):
        headers = self.login()
        old_claims = jwt.decode(headers["Authorization"][7:], self.services.jwt_key, algorithms=["HS256"])
        time.sleep(1.1)
        body = self.client.post("/api/auth/refresh", headers=headers).get_json()
        new_claims = jwt.decode(body["token"], self.services.jwt_key, algorithms=["HS256"])
        self.assertEqual(new_claims["jti"], old_claims["jti"])
        self.assertGreater(new_claims["exp"], old_claims["exp"])
        self.assertEqual(self.client.get("/api/auth/me", headers={"Authorization": "Bearer " + body["token"]}).status_code, 200)

    def test_logout_closes_the_session(self):
        headers = self.login()
        self.assertEqual(self.client.post("/api/auth/logout", headers=headers).status_code, 200)
        self.assertEqual(self.client.get("/api/auth/me", headers=headers).status_code, 401)

    def test_password_change_and_user_removal_close_sessions(self):
        anna, bruno = self.login("anna"), self.login("bruno")
        self.assertTrue(auth.change_password(self.services.database, "anna", "un'altra password"))
        self.assertEqual(self.client.get("/api/auth/me", headers=anna).status_code, 401)
        self.assertEqual(self.client.post("/api/auth/login", json={"username": "anna", "password": USER_PASSWORD}).status_code, 401)
        self.assertEqual(self.client.post("/api/auth/login", json={"username": "anna", "password": "un'altra password"}).status_code, 200)
        self.assertTrue(auth.remove_user(self.services.database, "bruno"))
        self.assertEqual(self.client.get("/api/auth/me", headers=bruno).status_code, 401)
        self.assertFalse(auth.remove_user(self.services.database, "bruno"))
        self.assertFalse(auth.add_user(self.services.database, "anna", "x"))

    def test_expired_session(self):
        headers = self.login()
        self.services.database.execute("UPDATE sessions SET expires_at = ?", (int(time.time()) - 1,))
        self.assertEqual(self.client.get("/api/auth/me", headers=headers).status_code, 401)

    def test_password_is_stored_hashed(self):
        row = self.services.database.query_one("SELECT * FROM users WHERE username = 'anna'")
        self.assertEqual(len(row["password_salt"]), config.PASSWORD_SALT_BYTES)
        self.assertNotIn(USER_PASSWORD.encode(), row["password_hash"])
        other = self.services.database.query_one("SELECT * FROM users WHERE username = 'bruno'")
        self.assertNotEqual(row["password_salt"], other["password_salt"])
        self.assertNotEqual(row["password_hash"], other["password_hash"])


class WebCookieTest(BackendTestCase):
    def web_login(self, client):
        return client.post("/api/auth/login", json={"username": "anna", "password": USER_PASSWORD, "use_cookie": True},
                           base_url=BASE_URL)

    def test_cookie_attributes_and_no_token_in_body(self):
        response = self.web_login(self.client)
        self.assertEqual(response.status_code, 200)
        self.assertNotIn("token", response.get_json())
        cookie = response.headers["Set-Cookie"]
        for attribute in ("HttpOnly", "Secure", "SameSite=Strict", "Path=/api"):
            self.assertIn(attribute, cookie)

    def test_cookie_authenticates_refreshes_and_logs_out(self):
        self.web_login(self.client)
        self.assertEqual(self.client.get("/api/auth/me", base_url=BASE_URL).get_json(), {"username": "anna"})
        response = self.client.post("/api/auth/refresh", base_url=BASE_URL)
        self.assertNotIn("token", response.get_json())
        self.assertIn(config.SESSION_COOKIE_NAME, response.headers["Set-Cookie"])
        response = self.client.post("/api/auth/logout", base_url=BASE_URL)
        self.assertIn("Max-Age=0", response.headers["Set-Cookie"])
        self.assertEqual(self.client.get("/api/auth/me", base_url=BASE_URL).status_code, 401)

    def test_origin_of_another_site_is_refused(self):
        self.web_login(self.client)
        for method in ("post", "put", "patch", "delete"):
            response = getattr(self.client, method)("/api/robot/mission/start", base_url=BASE_URL,
                                                    headers={"Origin": "https://altro-sito.example"})
            self.assertEqual(response.status_code, 403, method)
            self.assertEqual(response.get_json()["reason"], "bad_origin")
        # stessa origine del sito: passa (qui il robot non c'è, quindi 503)
        response = self.client.post("/api/robot/mission/start", base_url=BASE_URL, headers={"Origin": BASE_URL})
        self.assertEqual(response.status_code, 503)
        # le letture non cambiano niente: non si rifiutano, e senza intestazioni CORS il browser non le mostra all'altro sito
        response = self.client.get("/api/auth/me", base_url=BASE_URL, headers={"Origin": "https://altro-sito.example"})
        self.assertEqual(response.status_code, 200)
        self.assertNotIn("Access-Control-Allow-Origin", response.headers)

    def test_development_origin(self):
        os.environ[config.ALLOWED_ORIGINS_ENV] = "http://localhost:8090"
        try:
            response = self.client.options("/api/auth/login", base_url=BASE_URL, headers={
                "Origin": "http://localhost:8090", "Access-Control-Request-Method": "POST"})
            self.assertEqual(response.status_code, 204)
            self.assertEqual(response.headers["Access-Control-Allow-Origin"], "http://localhost:8090")
            self.assertEqual(response.headers["Access-Control-Allow-Credentials"], "true")
            response = self.client.post("/api/auth/login", base_url=BASE_URL, headers={"Origin": "http://localhost:8090"},
                                        json={"username": "anna", "password": USER_PASSWORD, "use_cookie": True})
            self.assertEqual(response.status_code, 200)
            self.assertEqual(response.headers["Access-Control-Allow-Origin"], "http://localhost:8090")
        finally:
            del os.environ[config.ALLOWED_ORIGINS_ENV]


class RobotTokenTest(BackendTestCase):
    def test_device_requests_need_the_token(self):
        for headers in ({}, {"Authorization": "Bearer sbagliato"}, self.login()):
            response = self.client.post("/api/device/state", json={}, headers=headers)
            self.assertEqual(response.status_code, 401)
        self.assertEqual(self.client.post("/api/device/state", json={}, headers=self.robot).status_code, 200)

    def test_only_the_hash_is_stored_and_a_new_token_replaces_the_old(self):
        import device_api
        saved = self.services.database.query_one("SELECT token_hash FROM robot")["token_hash"]
        self.assertNotEqual(saved, self.robot_token)
        self.assertEqual(len(self.robot_token), 2 * config.ROBOT_TOKEN_BYTES)
        new_token = device_api.new_robot_token(self.services.database)
        self.assertEqual(self.client.post("/api/device/state", json={}, headers=self.robot).status_code, 401)
        self.assertEqual(self.client.post("/api/device/state", json={},
                                          headers={"Authorization": "Bearer " + new_token}).status_code, 200)

    def test_user_token_is_not_a_robot_token(self):
        self.assertEqual(self.client.get("/api/robot/state", headers=self.robot).status_code, 401)


if __name__ == "__main__":
    unittest.main()
