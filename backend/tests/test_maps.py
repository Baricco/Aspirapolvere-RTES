"""Prove delle mappe: formato RTM1, mappa dal vivo con le differenze e il 409, allineamento robot-server, zone, scia."""
import os
import struct
import unittest
import zlib

import config
import maps
from tests.helpers import BackendTestCase, room_rtm
from tests.test_state_commands import ROBOT_STATE


def read_live_message(data):
    """Decodifica il messaggio della mappa dal vivo: (tipo, [(versione, rettangolo)])."""
    parts, offset = [], 1
    while offset < len(data):
        version, length = maps.LIVE_PART_HEADER.unpack_from(data, offset)
        offset += maps.LIVE_PART_HEADER.size
        parts.append((version, maps.decode_rtm(data[offset:offset + length])))
        offset += length
    return data[0], parts


def patch_rtm(x, y, width, height, category):
    return maps.encode_rtm(x, y, width, height, [], bytes([category]) * (width * height))


class RtmFormatTest(unittest.TestCase):
    def test_round_trip(self):
        cells = bytes([maps.MAP_CAT_FREE] * 200 + [maps.MAP_CAT_OCCUPIED] * 3 + [maps.MAP_CAT_VOID] + [maps.MAP_CAT_UNKNOWN] * 96)
        zones = [(510, 500, 12, 8), (0, 0, 1, 1)]
        data = maps.encode_rtm(100, 200, 30, 10, zones, cells)
        decoded = maps.decode_rtm(data)
        self.assertEqual((decoded.x, decoded.y, decoded.width, decoded.height), (100, 200, 30, 10))
        self.assertEqual(decoded.zones, zones)
        self.assertEqual(decoded.cells, cells)

    def test_layout_matches_the_documented_format(self):
        # 200 celle libere: sequenze da 63, 63, 63 e 11, un byte ciascuna (categoria nei 2 bit alti)
        data = maps.encode_rtm(1, 2, 20, 10, [], bytes([maps.MAP_CAT_FREE]) * 200)
        self.assertEqual(data[:4], b"RTM1")
        self.assertEqual(struct.unpack_from("<4HB", data, 4), (1, 2, 20, 10, 0))
        self.assertEqual(data[13:-4], bytes([0x7F, 0x7F, 0x7F, 0x40 | 11]))
        self.assertEqual(struct.unpack_from("<I", data, len(data) - 4)[0], zlib.crc32(data[:-4]))
        self.assertEqual(maps.file_crc(data), "%08x" % zlib.crc32(data[:-4]))

    def test_checks_at_loading(self):
        good = room_rtm()

        def with_crc(body):
            return bytes(body) + struct.pack("<I", zlib.crc32(bytes(body)))

        bad_files = {
            "troppo corto": good[:10],
            "numero magico": with_crc(b"RTM2" + good[4:-4]),
            "rettangolo fuori": with_crc(good[:4] + struct.pack("<4H", 1020, 0, 20, 10) + good[12:-4]),
            "larghezza zero": with_crc(good[:4] + struct.pack("<4H", 0, 0, 0, 10) + good[12:-4]),
            "troppe zone": with_crc(good[:12] + bytes([16]) + good[13:-4]),
            "sequenza a zero": with_crc(good[:13] + bytes([0x40]) + good[14:-4]),
            "sequenze in più": with_crc(good[:-4] + bytes([0x41])),
            "sequenze in meno": with_crc(good[:-5]),
            "crc": good[:-1] + bytes([good[-1] ^ 1]),
        }
        for name, data in bad_files.items():
            with self.assertRaises(maps.MapFormatError, msg=name):
                maps.decode_rtm(data)
        maps.decode_rtm(good)

    def test_zone_conversion_like_the_robot(self):
        cells = maps.zone_to_cells({"x": -0.365, "y": -0.22, "w": 0.26, "h": 0.44})
        self.assertEqual(cells, (505, 508, 5, 9))
        self.assertEqual(maps.zone_from_cells((532, 532, 10, 10)), {"x": 1.0, "y": 1.0, "w": 0.5, "h": 0.5})

    def test_replace_zones_keeps_the_cells(self):
        old = room_rtm(zones=[(1, 2, 3, 4)])
        new = maps.replace_zones(old, [(5, 6, 7, 8), (9, 10, 11, 12)])
        self.assertEqual(maps.decode_rtm(new).cells, maps.decode_rtm(old).cells)
        self.assertEqual(maps.decode_rtm(new).zones, [(5, 6, 7, 8), (9, 10, 11, 12)])
        self.assertNotEqual(maps.file_crc(new), maps.file_crc(old))


class LiveMapTest(BackendTestCase):
    def setUp(self):
        super().setUp()
        self.user = self.login()

    def get_map(self, query=""):
        response = self.client.get("/api/robot/map" + query, headers=self.user)
        self.assertEqual(response.status_code, 200)
        return read_live_message(response.data)

    def map_version(self):
        return self.client.get("/api/robot/state", headers=self.user).get_json()["map_version"]

    def test_empty_map_without_selected_map(self):
        kind, parts = self.get_map()
        self.assertEqual(kind, maps.LIVE_MESSAGE_FULL)
        self.assertEqual(parts[0][1].cells, bytes([maps.MAP_CAT_UNKNOWN]))
        self.assertEqual(parts[0][0], self.map_version())

    def test_full_map_from_the_robot(self):
        data = room_rtm()
        response = self.client.put("/api/device/map?version=1", data=data, headers=self.robot)
        self.assertEqual(response.status_code, 200)
        kind, parts = self.get_map()
        self.assertEqual(kind, maps.LIVE_MESSAGE_FULL)
        full = parts[0][1]
        sent = maps.decode_rtm(data)
        self.assertEqual((full.x, full.y, full.width, full.height, full.cells),
                         (sent.x, sent.y, sent.width, sent.height, sent.cells))
        self.assertEqual(parts[0][0], self.map_version())

    def test_patches_then_the_app_asks_only_for_the_differences(self):
        self.client.put("/api/device/map?version=7", data=room_rtm(), headers=self.robot)
        _, parts = self.get_map()
        first_version = parts[0][0]
        patch_a = patch_rtm(520, 505, 4, 3, maps.MAP_CAT_FREE)          # la stanza si allarga a destra
        patch_b = patch_rtm(503, 507, 2, 2, maps.MAP_CAT_VOID)
        self.assertEqual(self.client.post("/api/device/map/patch?base_version=7&version=8", data=patch_a,
                                          headers=self.robot).status_code, 200)
        self.assertEqual(self.client.post("/api/device/map/patch?base_version=8&version=9", data=patch_b,
                                          headers=self.robot).status_code, 200)
        self.assertEqual(self.map_version(), first_version + 2)

        kind, parts = self.get_map(f"?after={first_version}")
        self.assertEqual(kind, maps.LIVE_MESSAGE_PATCHES)
        self.assertEqual([version for version, _ in parts], [first_version + 1, first_version + 2])
        self.assertEqual((parts[0][1].x, parts[0][1].width, parts[0][1].height), (520, 4, 3))
        self.assertEqual(parts[1][1].cells, bytes([maps.MAP_CAT_VOID]) * 4)

        kind, parts = self.get_map(f"?after={first_version + 1}")
        self.assertEqual([version for version, _ in parts], [first_version + 2])
        kind, parts = self.get_map(f"?after={first_version + 2}")       # già aggiornata: nessuna parte
        self.assertEqual((kind, parts), (maps.LIVE_MESSAGE_PATCHES, []))

        # la mappa intera contiene i rettangoli incollati
        kind, parts = self.get_map()
        full = parts[0][1]
        self.assertEqual((full.x, full.width), (500, 24))
        row = full.cells[2 * full.width:3 * full.width]                 # riga y = 507
        self.assertEqual(row[3:5], bytes([maps.MAP_CAT_VOID]) * 2)
        self.assertEqual(row[20:24], bytes([maps.MAP_CAT_FREE]) * 4)

    def test_patch_from_a_version_the_server_does_not_have_409(self):
        patch = patch_rtm(520, 505, 4, 3, maps.MAP_CAT_FREE)
        # il server è appena ripartito: non ha nessuna versione del robot
        response = self.client.post("/api/device/map/patch?base_version=41&version=42", data=patch, headers=self.robot)
        self.assertEqual((response.status_code, response.get_json()["reason"]), (409, "version_mismatch"))
        # il robot rimanda la sua copia intera, poi la differenza in sospeso
        self.assertEqual(self.client.put("/api/device/map?version=41", data=room_rtm(), headers=self.robot).status_code, 200)
        self.assertEqual(self.client.post("/api/device/map/patch?base_version=41&version=42", data=patch,
                                          headers=self.robot).status_code, 200)
        # una differenza che salta una versione viene rifiutata, e la mappa non cambia
        version = self.map_version()
        response = self.client.post("/api/device/map/patch?base_version=50&version=51", data=patch, headers=self.robot)
        self.assertEqual(response.status_code, 409)
        self.assertEqual(self.map_version(), version)

    def test_app_too_far_behind_gets_the_full_map_again(self):
        config.MAP_PATCH_HISTORY = 3
        try:
            self.client.put("/api/device/map?version=0", data=room_rtm(), headers=self.robot)
            first_version = self.get_map()[1][0][0]
            for step in range(5):
                self.client.post(f"/api/device/map/patch?base_version={step}&version={step + 1}",
                                 data=patch_rtm(501 + step, 506, 1, 1, maps.MAP_CAT_OCCUPIED), headers=self.robot)
            kind, parts = self.get_map(f"?after={first_version}")       # le prime differenze non ci sono più
            self.assertEqual(kind, maps.LIVE_MESSAGE_FULL)
            self.assertEqual(parts[0][0], first_version + 5)
            kind, parts = self.get_map(f"?after={first_version + 2}")   # queste ci sono ancora
            self.assertEqual((kind, len(parts)), (maps.LIVE_MESSAGE_PATCHES, 3))
            kind, _ = self.get_map(f"?after={first_version + 99}")      # versione di un'altra vita del server
            self.assertEqual(kind, maps.LIVE_MESSAGE_FULL)
        finally:
            config.MAP_PATCH_HISTORY = 240

    def test_state_answer_tells_the_robot_which_version_the_server_has(self):
        # appena partito il server non ha nessuna versione del robot: il robot capisce che deve mandare la mappa intera
        response = self.client.post("/api/device/state", json={}, headers=self.robot)
        self.assertIsNone(response.get_json()["live_map_version"])
        self.client.put("/api/device/map?version=7", data=room_rtm(), headers=self.robot)
        self.client.post("/api/device/map/patch?base_version=7&version=8", data=patch_rtm(520, 505, 4, 3, maps.MAP_CAT_FREE),
                         headers=self.robot)
        response = self.client.post("/api/device/state", json={}, headers=self.robot)
        self.assertEqual(response.get_json()["live_map_version"], 8)

    def test_bad_requests(self):
        self.assertEqual(self.client.put("/api/device/map?version=1", data=b"niente", headers=self.robot).status_code, 400)
        self.assertEqual(self.client.put("/api/device/map", data=room_rtm(), headers=self.robot).status_code, 400)
        self.assertEqual(self.client.post("/api/device/map/patch?version=1", data=room_rtm(), headers=self.robot).status_code, 400)
        self.assertEqual(self.client.get("/api/robot/map?after=x", headers=self.user).status_code, 400)

    def test_map_change_wakes_the_state_long_poll(self):
        number = self.client.get("/api/robot/state", headers=self.user).get_json()["state_number"]
        self.client.put("/api/device/map?version=1", data=room_rtm(), headers=self.robot)
        response = self.client.get(f"/api/robot/state?after={number}", headers=self.user)
        self.assertEqual(response.status_code, 200)


class SavedMapsTest(BackendTestCase):
    def setUp(self):
        super().setUp()
        self.user = self.login()

    def saved(self, data, reason):
        response = self.client.put(f"/api/device/map/saved?reason={reason}", data=data, headers=self.robot)
        self.assertEqual(response.status_code, 200)
        return response.get_json()["selected_map_id"]

    def list_maps(self):
        return self.client.get("/api/robot/maps", headers=self.user).get_json()

    def test_first_save_creates_map_1_selected(self):
        self.assertEqual(self.list_maps(), [])
        data = room_rtm()
        map_id = self.saved(data, "save")
        listed = self.list_maps()
        self.assertEqual([(entry["id"], entry["name"], entry["selected"], entry["size_bytes"]) for entry in listed],
                         [(map_id, "map-1", True, len(data))])
        self.assertEqual(self.client.get(f"/api/robot/maps/{map_id}", headers=self.user).data, data)
        self.assertTrue(os.path.exists(os.path.join(self.data_dir, "maps", f"map-{map_id}.rtm")))

    def test_save_overwrites_the_selected_map(self):
        map_id = self.saved(room_rtm(), "save")
        bigger = room_rtm(width=30)
        self.assertEqual(self.saved(bigger, "save"), map_id)
        self.assertEqual(len(self.list_maps()), 1)
        self.assertEqual(self.client.get(f"/api/robot/maps/{map_id}", headers=self.user).data, bigger)

    def test_startup_with_the_same_file_changes_nothing(self):
        map_id = self.saved(room_rtm(), "save")
        self.assertEqual(self.saved(room_rtm(), "startup"), map_id)
        self.assertEqual(len(self.list_maps()), 1)

    def test_startup_with_a_different_file_becomes_a_new_map(self):
        first = self.saved(room_rtm(), "save")
        changed = room_rtm(width=30)
        second = self.saved(changed, "startup")
        self.assertNotEqual(second, first)
        listed = self.list_maps()
        self.assertEqual([(entry["name"], entry["selected"]) for entry in listed], [("map-1", False), ("map-2", True)])
        self.assertEqual(self.client.get(f"/api/robot/maps/{first}", headers=self.user).data, room_rtm())   # nessuna mappa persa

    def test_robot_without_map(self):
        self.saved(room_rtm(), "save")
        self.assertIsNone(self.saved(b"", "startup"))
        self.assertEqual([entry["selected"] for entry in self.list_maps()], [False])
        # al primo salvataggio dopo, mappa nuova selezionata
        self.saved(room_rtm(width=30), "save")
        self.assertEqual([(entry["name"], entry["selected"]) for entry in self.list_maps()],
                         [("map-1", False), ("map-2", True)])

    def test_default_name_skips_taken_names(self):
        first = self.saved(room_rtm(), "save")
        self.client.patch(f"/api/robot/maps/{first}", headers=self.user, json={"name": "map-2"})
        second = self.saved(room_rtm(width=30), "startup")       # 1 mappa + 1 = map-2, ma c'è già: map-3
        names = {entry["id"]: entry["name"] for entry in self.list_maps()}
        self.assertEqual(names[second], "map-3")

    def test_bad_saved_file(self):
        response = self.client.put("/api/device/map/saved?reason=save", data=b"rotto", headers=self.robot)
        self.assertEqual(response.status_code, 400)
        response = self.client.put("/api/device/map/saved?reason=boh", data=room_rtm(), headers=self.robot)
        self.assertEqual(response.status_code, 400)
        self.assertEqual(self.list_maps(), [])

    def test_rename(self):
        first = self.saved(room_rtm(), "save")
        second = self.services.map_store.add(room_rtm(width=30), "cucina")
        response = self.client.patch(f"/api/robot/maps/{first}", headers=self.user, json={"name": "  soggiorno "})
        self.assertEqual(response.status_code, 200)
        self.assertEqual(self.list_maps()[0]["name"], "soggiorno")
        response = self.client.patch(f"/api/robot/maps/{second}", headers=self.user, json={"name": "soggiorno"})
        self.assertEqual((response.status_code, response.get_json()["reason"]), (409, "name_taken"))
        self.assertEqual(self.client.patch(f"/api/robot/maps/{second}", headers=self.user, json={"name": "cucina"}).status_code, 200)
        for name in ("", "   ", "x" * (config.MAP_NAME_MAX_LENGTH + 1), 5):
            self.assertEqual(self.client.patch(f"/api/robot/maps/{first}", headers=self.user, json={"name": name}).status_code, 400)
        self.assertEqual(self.client.patch("/api/robot/maps/999", headers=self.user, json={"name": "x"}).status_code, 404)

    def test_delete_only_from_the_server_and_ids_are_never_reused(self):
        selected = self.saved(room_rtm(), "save")
        other = self.services.map_store.add(room_rtm(width=30), "cucina")
        response = self.client.delete(f"/api/robot/maps/{selected}", headers=self.user)
        self.assertEqual((response.status_code, response.get_json()["reason"]), (409, "map_selected"))
        self.assertEqual(self.client.delete(f"/api/robot/maps/{other}", headers=self.user).status_code, 200)
        self.assertFalse(os.path.exists(os.path.join(self.data_dir, "maps", f"map-{other}.rtm")))
        self.assertEqual(self.client.delete(f"/api/robot/maps/{other}", headers=self.user).status_code, 404)
        self.assertEqual(self.client.get(f"/api/robot/maps/{other}", headers=self.user).status_code, 404)
        self.assertGreater(self.services.map_store.add(room_rtm(width=31), "bagno"), other)

    def test_select_map(self):
        current = self.saved(room_rtm(), "save")
        other = self.services.map_store.add(room_rtm(width=30), "cucina")
        other_crc = maps.file_crc(room_rtm(width=30))
        # il robot rifiuta: resta selezionata quella di prima
        response, taken = self.command("/api/robot/maps/selected", self.user, method="put", json={"id": other},
                                       robot_result={"ok": False, "reason": "not_resting", "message": "Non sono a riposo."})
        self.assertEqual((response.status_code, response.get_json()["reason"]), (409, "not_resting"))
        self.assertEqual(taken[0], dict(taken[0], type="map_select", map_id=other, crc=other_crc))
        self.assertEqual(self.services.map_store.selected_id(), current)
        # il robot la scarica e accetta
        self.assertEqual(self.client.get(f"/api/device/maps/{other}", headers=self.robot).data, room_rtm(width=30))
        response, _ = self.command("/api/robot/maps/selected", self.user, method="put", json={"id": other})
        self.assertEqual(response.status_code, 200)
        self.assertEqual(self.services.map_store.selected_id(), other)
        # dopo il riavvio il robot manda il file: è quello della selezionata, non nasce una mappa nuova
        self.saved(room_rtm(width=30), "startup")
        self.assertEqual(len(self.list_maps()), 2)
        self.assertEqual(self.client.put("/api/robot/maps/selected", headers=self.user, json={"id": 999}).status_code, 404)
        self.assertEqual(self.client.put("/api/robot/maps/selected", headers=self.user, json={"id": "x"}).status_code, 400)
        self.assertEqual(self.client.get("/api/device/maps/999", headers=self.robot).status_code, 404)

    def test_no_map_only_if_the_server_has_the_copy(self):
        data = room_rtm()
        self.saved(data, "save")
        state = dict(ROBOT_STATE, map=dict(ROBOT_STATE["map"], file_crc="00000000"))
        self.robot_post_state(state)            # il robot ha un file che il server non ha
        response = self.client.post("/api/robot/map/reset", headers=self.user)
        self.assertEqual((response.status_code, response.get_json()["reason"]), (409, "not_backed_up"))
        state["map"]["file_crc"] = maps.file_crc(data)
        self.robot_post_state(state)
        response, taken = self.command("/api/robot/map/reset", self.user)
        self.assertEqual(response.status_code, 200)
        self.assertEqual(taken[0]["type"], "map_reset")
        self.assertIsNone(self.services.map_store.selected_id())
        self.assertEqual(len(self.list_maps()), 1)                # la copia sul server resta

    def test_no_map_when_the_robot_has_no_file(self):
        state = dict(ROBOT_STATE, map={key: value for key, value in ROBOT_STATE["map"].items() if key != "file_crc"})
        self.robot_post_state(state)
        response, taken = self.command("/api/robot/map/reset", self.user)
        self.assertEqual(response.status_code, 200)

    def test_zones_of_another_map_rewrite_the_file(self):
        selected = self.saved(room_rtm(), "save")
        other = self.services.map_store.add(room_rtm(width=30), "cucina")
        old_crc = self.services.map_store.get(other)["crc32"]
        zones = [{"x": 1.0, "y": 1.0, "w": 0.5, "h": 0.5}]
        self.assertEqual(self.client.put(f"/api/robot/maps/{other}/zones", headers=self.user, json=zones).status_code, 200)
        data = self.client.get(f"/api/robot/maps/{other}", headers=self.user).data
        decoded = maps.decode_rtm(data)
        self.assertEqual(decoded.zones, [(532, 532, 10, 10)])
        self.assertEqual(decoded.cells, maps.decode_rtm(room_rtm(width=30)).cells)
        row = self.services.map_store.get(other)
        self.assertEqual(row["crc32"], maps.file_crc(data))
        self.assertNotEqual(row["crc32"], old_crc)
        # per la selezionata le zone vanno al robot come comando
        response, taken = self.command(f"/api/robot/maps/{selected}/zones", self.user, method="put", json=zones)
        self.assertEqual(response.status_code, 200)
        self.assertEqual((taken[0]["type"], taken[0]["zones"]), ("zones", zones))
        # rifiuti
        too_many = zones * (config.MAX_USER_ZONES + 1)
        response = self.client.put(f"/api/robot/maps/{other}/zones", headers=self.user, json=too_many)
        self.assertEqual((response.status_code, response.get_json()["reason"]), (409, "too_many_zones"))
        self.assertEqual(self.client.put("/api/robot/maps/999/zones", headers=self.user, json=zones).status_code, 404)

    def test_selected_map_is_the_live_map_until_the_robot_sends_its_own(self):
        data = room_rtm()
        self.saved(data, "save")
        self.services.load_selected_map()       # quello che fa il server quando parte
        kind, parts = read_live_message(self.client.get("/api/robot/map", headers=self.user).data)
        self.assertEqual(parts[0][1].cells, maps.decode_rtm(data).cells)


class TrailTest(BackendTestCase):
    def setUp(self):
        super().setUp()
        self.user = self.login()

    def post_position(self, x, y, mission="clean", mission_number=1, valid=True):
        state = dict(ROBOT_STATE)
        state["navigation"] = dict(ROBOT_STATE["navigation"], mission=mission, mission_number=mission_number)
        state["position"] = dict(ROBOT_STATE["position"], x=x, y=y, valid=valid)
        self.robot_post_state(state)

    def trail(self, query=""):
        return self.client.get("/api/robot/trail" + query, headers=self.user).get_json()

    def test_points_are_added_while_the_robot_moves_in_a_mission(self):
        self.post_position(0, 0, mission="none", mission_number=0)      # a riposo: niente scia
        self.post_position(0.0, 0.0)
        self.post_position(0.01, 0.0)                                   # troppo vicino all'ultimo punto
        self.post_position(0.1, 0.0)
        self.post_position(5.0, 5.0, valid=False)                       # posizione non valida
        self.post_position(0.2, 0.0)
        body = self.trail()
        self.assertEqual(body["points"], [[0.0, 0.0], [0.1, 0.0], [0.2, 0.0]])
        self.assertEqual(body["offset"], 0)
        more = self.trail("?after=2")
        self.assertEqual((more["offset"], more["points"], more["trail_id"]), (2, [[0.2, 0.0]], body["trail_id"]))
        self.assertEqual(self.trail("?after=3")["points"], [])

    def test_new_mission_starts_a_new_trail(self):
        self.post_position(0.0, 0.0)
        self.post_position(0.5, 0.0)
        first = self.trail()
        self.post_position(0.5, 0.0, mission="none")                    # fine missione: la scia resta
        self.assertEqual(self.trail()["points"], first["points"])
        self.post_position(1.0, 1.0, mission_number=2)
        second = self.trail()
        self.assertEqual(second["points"], [[1.0, 1.0]])
        self.assertNotEqual(second["trail_id"], first["trail_id"])
        # un'app rimasta indietro chiede dopo il punto 2 della scia vecchia: riceve quella nuova dall'inizio
        self.assertEqual(self.trail("?after=2")["offset"], 0)

    def test_trail_is_saved_at_the_end_of_the_mission(self):
        self.post_position(0.0, 0.0)
        self.post_position(0.5, 0.0)
        self.assertFalse(os.path.exists(os.path.join(self.data_dir, config.TRAIL_FILE)))
        self.post_position(0.5, 0.0, mission="none")
        reloaded = maps.Trail(self.data_dir)
        self.assertEqual(reloaded.message()["points"], [(0.0, 0.0), (0.5, 0.0)])

    def test_robot_restart_keeps_the_old_trail_until_a_new_mission(self):
        self.post_position(0.0, 0.0, mission_number=3)
        self.post_position(0.5, 0.0, mission_number=3)
        self.post_position(0.0, 0.0, mission="none", mission_number=0)  # il programma del robot è ripartito
        self.assertEqual(len(self.trail()["points"]), 2)
        self.post_position(0.0, 0.0, mission_number=1)
        self.assertEqual(self.trail()["points"], [[0.0, 0.0]])


if __name__ == "__main__":
    unittest.main()
