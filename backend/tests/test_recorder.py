"""Prove del registro per le analisi: righe degli stati, istantanee della mappa, tetto, disco pieno, errori che non
devono mai disturbare la richiesta del robot."""
import base64
import copy
import gzip
import json
import os
import time
import unittest

import config
import maps
from tests.helpers import BackendTestCase, room_rtm
from tests.test_state_commands import ROBOT_STATE


def mission_state(mission="clean", state="border_lap"):
    moving = copy.deepcopy(ROBOT_STATE)
    moving["navigation"]["mission"] = mission
    moving["navigation"]["state"] = state
    return moving


class RecorderTest(BackendTestCase):
    def setUp(self):
        super().setUp()
        self.recorder = self.services.recorder
        self.saved_recorder_config = {name: getattr(config, name) for name in
                                      ("RECORDER_MAX_BYTES", "RECORDER_MIN_FREE_BYTES", "RECORDER_QUEUE_MAX",
                                       "RECORDER_MAP_INTERVAL_S")}

    def tearDown(self):
        for name, value in self.saved_recorder_config.items():
            setattr(config, name, value)
        super().tearDown()

    def read_lines(self):
        """Tutte le righe di tutti i file del registro, nell'ordine."""
        lines = []
        for path in self.recorder.files():
            with gzip.open(path, "rt", encoding="utf-8") as day_file:
                lines += [json.loads(line) for line in day_file]
        return lines

    def test_every_state_is_written_with_the_server_time(self):
        for number in range(3):
            state = copy.deepcopy(ROBOT_STATE)
            state["uptime_s"] = number
            self.robot_post_state(state)
        self.recorder.flush()
        lines = self.read_lines()
        self.assertEqual([line["state"]["uptime_s"] for line in lines], [0, 1, 2])
        self.assertEqual(lines[0]["state"], ROBOT_STATE | {"uptime_s": 0})
        self.assertAlmostEqual(lines[0]["t"], time.time(), delta=5)
        self.assertTrue(os.path.basename(self.recorder.today_path()).startswith(config.RECORDER_FILE_PREFIX))

    def test_several_writes_make_one_readable_file(self):
        # ogni scrittura aggiunge un membro gzip: il file resta uno e si legge tutto di fila
        for number in range(4):
            state = copy.deepcopy(ROBOT_STATE)
            state["uptime_s"] = number
            self.robot_post_state(state)
            self.recorder.flush()
        self.assertEqual(len(self.recorder.files()), 1)
        self.assertEqual([line["state"]["uptime_s"] for line in self.read_lines()], [0, 1, 2, 3])

    def test_map_snapshot_only_during_a_mission_and_one_at_the_end(self):
        config.RECORDER_MAP_INTERVAL_S = 0
        self.services.live_map.set_full(room_rtm(), 1)
        self.robot_post_state(ROBOT_STATE)                  # a riposo: niente mappa
        self.recorder.flush()
        self.robot_post_state(mission_state())              # in missione: la mappa c'è
        self.recorder.flush()
        self.robot_post_state(mission_state())              # mappa non cambiata: non si ripete
        self.recorder.flush()
        self.services.live_map.set_full(room_rtm(width=30), 2)
        self.robot_post_state(ROBOT_STATE)                  # fine della missione: l'ultima mappa
        self.recorder.flush()
        snapshots = [line for line in self.read_lines() if "map_rtm" in line]
        self.assertEqual(len(snapshots), 2)
        self.assertEqual(maps.decode_rtm(base64.b64decode(snapshots[0]["map_rtm"])).width, 20)
        self.assertEqual(maps.decode_rtm(base64.b64decode(snapshots[1]["map_rtm"])).width, 30)

    def test_map_snapshot_waits_for_the_interval(self):
        config.RECORDER_MAP_INTERVAL_S = 3600
        self.robot_post_state(mission_state())
        self.recorder.flush()
        self.services.live_map.set_full(room_rtm(width=30), 2)
        self.robot_post_state(mission_state())
        self.recorder.flush()
        self.assertEqual(len([line for line in self.read_lines() if "map_rtm" in line]), 1)

    def test_size_limit_removes_the_oldest_days(self):
        os.makedirs(self.recorder.dir)
        old = [os.path.join(self.recorder.dir, f"{config.RECORDER_FILE_PREFIX}2026-11-0{day}{config.RECORDER_FILE_EXTENSION}")
               for day in (1, 2, 3)]
        for path in old:
            with open(path, "wb") as day_file:
                day_file.write(b"x" * 1000)
        config.RECORDER_MAX_BYTES = 2500
        self.robot_post_state(ROBOT_STATE)
        self.recorder.flush()
        remaining = self.recorder.files()
        self.assertNotIn(old[0], remaining)
        self.assertIn(self.recorder.today_path(), remaining)
        self.assertLessEqual(sum(os.path.getsize(path) for path in remaining), config.RECORDER_MAX_BYTES)

    def test_today_alone_over_the_limit_is_emptied(self):
        config.RECORDER_MAX_BYTES = 10
        self.robot_post_state(ROBOT_STATE)
        self.recorder.flush()
        self.assertEqual(os.path.getsize(self.recorder.today_path()), 0)

    def test_no_writing_when_the_disk_is_almost_full(self):
        config.RECORDER_MIN_FREE_BYTES = 1 << 62
        self.robot_post_state(ROBOT_STATE)
        self.recorder.flush()
        self.assertFalse(os.path.exists(self.recorder.dir))

    def test_full_queue_drops_lines_but_the_robot_request_still_works(self):
        self.recorder.lines.maxsize = 2
        for _ in range(5):
            self.robot_post_state(ROBOT_STATE)              # controlla anche la risposta 200
        self.assertEqual(self.recorder.dropped, 3)
        self.recorder.flush()
        self.assertEqual(len(self.read_lines()), 2)
        self.assertEqual(self.recorder.dropped, 0)

    def test_write_errors_do_not_reach_the_robot_request(self):
        with open(self.recorder.dir, "w") as blocking_file:   # un file al posto della cartella: la scrittura fallisce
            blocking_file.write("")
        self.robot_post_state(ROBOT_STATE)
        self.recorder.flush()
        self.robot_post_state(ROBOT_STATE)
        self.assertEqual(self.services.link.current_state()["navigation"], ROBOT_STATE["navigation"])

    def test_thread_writes_the_rest_when_it_stops(self):
        self.recorder.start()
        self.robot_post_state(ROBOT_STATE)
        self.recorder.stop()
        self.assertEqual(len(self.read_lines()), 1)


if __name__ == "__main__":
    unittest.main()
