"""Mappe: lettura e scrittura del formato RTM1, mappe salvate sul server, mappa dal vivo con le differenze, scia.

Il formato RTM1 è descritto in docs/formato-mappa.md; la mappa dal vivo e la scia in docs/app.md, sezione 7.
"""
import collections
import dataclasses
import json
import math
import os
import re
import struct
import threading
import time
import zlib

import config

MAP_FILE_MAGIC = b"RTM1"
MAP_HEADER_LEN = 13                 # numero magico, rettangolo (4 uint16), numero di zone
MAP_ZONE_LEN = 8                    # x, y, larghezza, altezza (uint16, in celle fini)
MAP_CRC_LEN = 4
MAP_RUN_MAX = 63                    # celle massime in una sequenza
MAP_RUN_CATEGORY_SHIFT = 6
MAP_RUN_COUNT_MASK = 0x3F

MAP_CAT_UNKNOWN = 0
MAP_CAT_FREE = 1
MAP_CAT_OCCUPIED = 2
MAP_CAT_VOID = 3

# messaggio della mappa dal vivo per l'app: un byte con il tipo, poi una o più parti
# (versione uint64, lunghezza uint32, rettangolo in RTM1), tutto little endian
LIVE_MESSAGE_FULL = 0               # una parte sola: la mappa intera, che sostituisce quella dell'app
LIVE_MESSAGE_PATCHES = 1            # zero o più parti: i rettangoli cambiati, da incollare in ordine
LIVE_PART_HEADER = struct.Struct("<QI")

SAVED_REASON_STARTUP = "startup"    # il robot manda la mappa che ha all'avvio
SAVED_REASON_SAVE = "save"          # il robot ha appena salvato la mappa

RUN_PATTERN = re.compile(rb"(.)\1*", re.DOTALL)     # una sequenza di byte uguali
SINGLE_CELL = [bytes([category]) for category in range(4)]


class MapFormatError(ValueError):
    """Il file non supera i controlli del formato RTM1."""


class MapNameTaken(Exception):
    """Esiste già una mappa con quel nome."""


class MapVersionMismatch(Exception):
    """La differenza della mappa dal vivo non parte dalla versione che ha il server."""


@dataclasses.dataclass
class RtmMap:
    x: int                  # prima colonna del rettangolo, in celle fini
    y: int                  # prima riga
    width: int
    height: int
    zones: list             # zone proibite dell'utente: tuple (x, y, larghezza, altezza) in celle fini
    cells: bytes            # larghezza * altezza categorie, riga dopo riga


def decode_rtm(data):
    """Decodifica un file RTM1 facendo tutti i controlli di docs/formato-mappa.md."""
    if len(data) < MAP_HEADER_LEN + MAP_CRC_LEN:
        raise MapFormatError("file troppo corto")
    if data[:len(MAP_FILE_MAGIC)] != MAP_FILE_MAGIC:
        raise MapFormatError("numero magico sbagliato")
    x, y, width, height = struct.unpack_from("<4H", data, len(MAP_FILE_MAGIC))
    num_zones = data[MAP_HEADER_LEN - 1]
    side = config.FINE_MAP_CELL_NUMBER
    if width == 0 or height == 0 or x + width > side or y + height > side:
        raise MapFormatError("il rettangolo esce dalla griglia")
    if num_zones > config.MAX_USER_ZONES:
        raise MapFormatError("troppe zone")
    end = len(data) - MAP_CRC_LEN
    offset = MAP_HEADER_LEN
    if offset + num_zones * MAP_ZONE_LEN > end:
        raise MapFormatError("file troppo corto")
    zones = []
    for _ in range(num_zones):
        zone = struct.unpack_from("<4H", data, offset)
        offset += MAP_ZONE_LEN
        if zone[0] + zone[2] > side or zone[1] + zone[3] > side:
            raise MapFormatError("una zona esce dalla griglia")
        zones.append(zone)
    total = width * height
    cells = bytearray()
    while len(cells) < total:
        if offset >= end:
            raise MapFormatError("le sequenze finiscono prima delle celle")
        run = data[offset]
        offset += 1
        count = run & MAP_RUN_COUNT_MASK
        if count == 0 or count > total - len(cells):
            raise MapFormatError("sequenza non valida")
        cells += SINGLE_CELL[run >> MAP_RUN_CATEGORY_SHIFT] * count
    if offset != end:
        raise MapFormatError("byte in più dopo le celle")
    if struct.unpack_from("<I", data, end)[0] != zlib.crc32(data[:end]):
        raise MapFormatError("CRC sbagliato")
    return RtmMap(x, y, width, height, zones, bytes(cells))


def encode_rtm(x, y, width, height, zones, cells):
    """Codifica un rettangolo di categorie in RTM1."""
    if len(cells) != width * height:
        raise ValueError("numero di celle diverso da larghezza per altezza")
    data = bytearray(MAP_FILE_MAGIC)
    data += struct.pack("<4HB", x, y, width, height, len(zones))
    for zone in zones:
        data += struct.pack("<4H", *zone)
    for match in RUN_PATTERN.finditer(cells):
        category = cells[match.start()] << MAP_RUN_CATEGORY_SHIFT
        full_runs, rest = divmod(match.end() - match.start(), MAP_RUN_MAX)
        data += bytes([category | MAP_RUN_MAX]) * full_runs
        if rest:
            data.append(category | rest)
    data += struct.pack("<I", zlib.crc32(data))
    return bytes(data)


def file_crc(data):
    """Il CRC-32 scritto in fondo al file, come 8 cifre esadecimali: è l'impronta della mappa."""
    return "%08x" % struct.unpack_from("<I", data, len(data) - MAP_CRC_LEN)[0]


def empty_rtm():
    """Una mappa vuota: una sola cella sconosciuta nell'origine."""
    origin = config.FINE_MAP_ORIGIN
    return encode_rtm(origin, origin, 1, 1, [], SINGLE_CELL[MAP_CAT_UNKNOWN])


def zone_to_cells(zone):
    """Da una zona in metri {"x", "y", "w", "h"} alla stessa in celle fini, arrotondata come fa il robot."""
    size = config.FINE_MAP_CELL_SIZE
    return (int(math.floor(zone["x"] / size + 0.5)) + config.FINE_MAP_ORIGIN,
            int(math.floor(zone["y"] / size + 0.5)) + config.FINE_MAP_ORIGIN,
            int(math.floor(zone["w"] / size + 0.5)),
            int(math.floor(zone["h"] / size + 0.5)))


def zone_from_cells(zone):
    size = config.FINE_MAP_CELL_SIZE
    return {"x": round((zone[0] - config.FINE_MAP_ORIGIN) * size, 3),
            "y": round((zone[1] - config.FINE_MAP_ORIGIN) * size, 3),
            "w": round(zone[2] * size, 3),
            "h": round(zone[3] * size, 3)}


def check_zones(zones):
    """Controlla una lista di zone in metri con le stesse regole del robot. Ritorna None oppure il motivo del rifiuto."""
    if len(zones) > config.MAX_USER_ZONES:
        return "too_many_zones"
    size = config.FINE_MAP_CELL_SIZE
    half_side = config.FINE_MAP_ORIGIN * size
    for zone in zones:
        if (zone["w"] < size or zone["h"] < size or zone["x"] < -half_side or zone["y"] < -half_side
                or zone["x"] + zone["w"] > half_side or zone["y"] + zone["h"] > half_side):
            return "outside_map"
    return None


def replace_zones(data, zones_cells):
    """Riscrive l'intestazione di un file RTM1 con una nuova lista di zone, e il CRC."""
    old = decode_rtm(data)
    return encode_rtm(old.x, old.y, old.width, old.height, zones_cells, old.cells)


def write_file_atomic(path, data):
    """Scrive su un file temporaneo e lo rinomina sopra quello vero: mai un file a metà."""
    temporary_path = path + ".tmp"
    with open(temporary_path, "wb") as output:
        output.write(data)
    os.replace(temporary_path, path)


class MapStore:
    """Le mappe salvate sul server: i file map-<id>.rtm e la tabella maps."""

    def __init__(self, database, data_dir):
        self.database = database
        self.maps_dir = os.path.join(data_dir, config.MAPS_DIR)
        self.lock = threading.Lock()        # un solo cambiamento alla volta fra file e tabella
        os.makedirs(self.maps_dir, exist_ok=True)

    def file_path(self, map_id):
        return os.path.join(self.maps_dir, f"{config.MAP_FILE_PREFIX}{map_id}{config.MAP_FILE_EXTENSION}")

    def selected_id(self):
        return self.database.query_one("SELECT selected_map_id FROM robot WHERE id = 1")["selected_map_id"]

    def set_selected(self, map_id):
        self.database.execute("UPDATE robot SET selected_map_id = ? WHERE id = 1", (map_id,))

    def list_maps(self):
        selected = self.selected_id()
        rows = self.database.query_all("SELECT id, name, saved_at, size_bytes FROM maps ORDER BY id")
        return [{"id": row["id"], "name": row["name"], "selected": row["id"] == selected,
                 "saved_at": row["saved_at"], "size_bytes": row["size_bytes"]} for row in rows]

    def get(self, map_id):
        return self.database.query_one("SELECT * FROM maps WHERE id = ?", (map_id,))

    def read_file(self, map_id):
        with open(self.file_path(map_id), "rb") as map_file:
            return map_file.read()

    def default_name(self):
        """map-N con N = numero di mappe + 1, salendo al primo numero libero."""
        number = self.database.query_one("SELECT COUNT(*) AS count FROM maps")["count"] + 1
        while self.database.query_one("SELECT 1 FROM maps WHERE name = ?",
                                      (f"{config.MAP_DEFAULT_NAME_PREFIX}{number}",)):
            number += 1
        return f"{config.MAP_DEFAULT_NAME_PREFIX}{number}"

    def add(self, data, name=None):
        """Aggiunge una mappa nuova (il file deve essere un RTM1 valido). Ritorna il suo id."""
        decode_rtm(data)
        with self.lock:
            if name is None:
                name = self.default_name()
            elif self.database.query_one("SELECT 1 FROM maps WHERE name = ?", (name,)):
                raise MapNameTaken(name)
            now = int(time.time())
            cursor = self.database.execute(
                "INSERT INTO maps (name, crc32, size_bytes, created_at, saved_at) VALUES (?, ?, ?, ?, ?)",
                (name, file_crc(data), len(data), now, now))
            write_file_atomic(self.file_path(cursor.lastrowid), data)
            return cursor.lastrowid

    def overwrite(self, map_id, data):
        decode_rtm(data)
        with self.lock:
            write_file_atomic(self.file_path(map_id), data)
            self.database.execute("UPDATE maps SET crc32 = ?, size_bytes = ?, saved_at = ? WHERE id = ?",
                                  (file_crc(data), len(data), int(time.time()), map_id))

    def rename(self, map_id, name):
        with self.lock:
            if self.database.query_one("SELECT 1 FROM maps WHERE name = ? AND id != ?", (name, map_id)):
                raise MapNameTaken(name)
            self.database.execute("UPDATE maps SET name = ? WHERE id = ?", (name, map_id))

    def delete(self, map_id):
        with self.lock:
            self.database.execute("DELETE FROM maps WHERE id = ?", (map_id,))
            try:
                os.remove(self.file_path(map_id))
            except FileNotFoundError:
                pass

    def set_zones(self, map_id, zones):
        """Zone di una mappa non selezionata: si riscrive l'intestazione del file (zone e CRC)."""
        data = replace_zones(self.read_file(map_id), [zone_to_cells(zone) for zone in zones])
        self.overwrite(map_id, data)

    def zones(self, map_id):
        return [zone_from_cells(zone) for zone in decode_rtm(self.read_file(map_id)).zones]

    def saved_from_robot(self, data, reason):
        """Allineamento con il robot (docs/app.md, sezione 7.2). data è il file map.rtm del robot, vuoto se non ne ha.

        - robot senza file: nessuna mappa selezionata;
        - nessuna mappa selezionata: il file diventa una mappa nuova, selezionata;
        - dopo un salvataggio: il file sostituisce la copia della mappa selezionata;
        - all'avvio: se il CRC non coincide con la selezionata il file diventa una mappa nuova, così non se ne perde nessuna.
        Ritorna l'id della mappa selezionata (None se il robot non ha mappa)."""
        if not data:
            self.set_selected(None)
            return None
        decode_rtm(data)
        selected = self.selected_id()
        row = self.get(selected) if selected is not None else None
        if row is None:
            selected = self.add(data)
        elif row["crc32"] == file_crc(data):
            return selected
        elif reason == SAVED_REASON_SAVE:
            self.overwrite(selected, data)
            return selected
        else:
            selected = self.add(data)
        self.set_selected(selected)
        return selected


class LiveMap:
    """La mappa dal vivo: la griglia fine a categorie com'è adesso sul robot, più le ultime differenze numerate.

    Due numeri di versione distinti: device_version è quello del robot (serve a rifiutare una differenza che non parte
    dalla versione giusta), version è quello per le app, che parte dall'orologio così non torna mai indietro, nemmeno
    dopo un riavvio del server."""

    def __init__(self, on_change=None):
        side = config.FINE_MAP_CELL_NUMBER
        self.grid = bytearray(side * side)
        self.lock = threading.Lock()
        self.version = int(time.time() * 1000)
        self.device_version = None
        self.patches = collections.deque()      # (versione, rettangolo in RTM1)
        self.full_cache = None                  # (versione, mappa intera in RTM1)
        self.on_change = on_change

    def paste(self, rectangle):
        side = config.FINE_MAP_CELL_NUMBER
        for row in range(rectangle.height):
            start = (rectangle.y + row) * side + rectangle.x
            self.grid[start:start + rectangle.width] = rectangle.cells[row * rectangle.width:(row + 1) * rectangle.width]

    def set_full(self, data, device_version):
        """Sostituisce tutta la mappa (all'avvio del robot, o dopo un 409)."""
        rectangle = decode_rtm(data)
        with self.lock:
            self.grid = bytearray(len(self.grid))
            self.paste(rectangle)
            self.version += 1
            self.device_version = device_version
            self.patches.clear()
            self.full_cache = None
        if self.on_change:
            self.on_change()

    def apply_patch(self, data, base_version, device_version):
        """Incolla un rettangolo cambiato. Se non parte dalla versione che ha il server: MapVersionMismatch."""
        rectangle = decode_rtm(data)
        with self.lock:
            if self.device_version is None or base_version != self.device_version:
                raise MapVersionMismatch()
            self.paste(rectangle)
            self.version += 1
            self.device_version = device_version
            self.patches.append((self.version, data))
            while len(self.patches) > config.MAP_PATCH_HISTORY:
                self.patches.popleft()
            self.full_cache = None
        if self.on_change:
            self.on_change()

    def full_rtm(self):
        """La mappa intera in RTM1: il rettangolo più piccolo che contiene le celle non sconosciute. Con self.lock preso."""
        if self.full_cache and self.full_cache[0] == self.version:
            return self.full_cache[1]
        side = config.FINE_MAP_CELL_NUMBER
        x_min, x_max, y_min, y_max = side, -1, side, -1
        for y in range(side):
            row = self.grid[y * side:(y + 1) * side]
            right = len(row.rstrip(b"\0"))
            if right == 0:
                continue
            left = side - len(row.lstrip(b"\0"))
            x_min, x_max = min(x_min, left), max(x_max, right - 1)
            y_min, y_max = min(y_min, y), y
        if x_max < 0:
            data = empty_rtm()
        else:
            width = x_max - x_min + 1
            cells = b"".join(bytes(self.grid[y * side + x_min:y * side + x_min + width]) for y in range(y_min, y_max + 1))
            data = encode_rtm(x_min, y_min, width, y_max - y_min + 1, [], cells)
        self.full_cache = (self.version, data)
        return data

    def message(self, after=None):
        """Il messaggio per l'app: la mappa intera, oppure le differenze dopo la versione after se il server le ha ancora."""
        with self.lock:
            oldest = self.patches[0][0] if self.patches else self.version + 1
            if after is not None and oldest - 1 <= after <= self.version:
                parts = [(version, data) for version, data in self.patches if version > after]
                kind = LIVE_MESSAGE_PATCHES
            else:
                parts = [(self.version, self.full_rtm())]
                kind = LIVE_MESSAGE_FULL
        message = bytearray([kind])
        for version, data in parts:
            message += LIVE_PART_HEADER.pack(version, len(data))
            message += data
        return bytes(message)


class Trail:
    """La scia dell'ultima missione, registrata dalle posizioni che arrivano con lo stato (docs/app.md, sezione 7.4)."""

    def __init__(self, data_dir):
        self.path = os.path.join(data_dir, config.TRAIL_FILE)
        self.lock = threading.Lock()
        self.trail_id = int(time.time() * 1000)     # cambia quando la scia ricomincia: l'app capisce che deve buttare la sua
        self.points = []
        self.last_mission_number = None
        self.mission_active = False
        self.load()

    def load(self):
        try:
            with open(self.path, encoding="utf-8") as trail_file:
                saved = json.load(trail_file)
            self.points = [tuple(point) for point in saved["points"]]
            self.last_mission_number = saved.get("mission_number")
        except (OSError, ValueError, KeyError, TypeError):
            pass

    def save(self):
        """Con self.lock preso."""
        body = json.dumps({"mission_number": self.last_mission_number, "points": self.points})
        write_file_atomic(self.path, body.encode("utf-8"))

    def add_state(self, state):
        """Guarda uno stato appena arrivato dal robot: scia nuova se è cambiata la missione, un punto in più se si è mosso."""
        navigation = state.get("navigation") or {}
        position = state.get("position") or {}
        mission_number = navigation.get("mission_number")
        mission_active = navigation.get("mission", "none") != "none"
        with self.lock:
            # il numero torna a 0 quando il programma del robot riparte: la scia vecchia resta finché non parte una missione
            if mission_number != self.last_mission_number:
                if mission_number and self.last_mission_number is not None:
                    self.points = []
                    self.trail_id += 1
                self.last_mission_number = mission_number
            if mission_active and position.get("valid") and len(self.points) < config.TRAIL_MAX_POINTS:
                point = (round(position.get("x", 0.0), 3), round(position.get("y", 0.0), 3))
                if not self.points or math.dist(point, self.points[-1]) >= config.TRAIL_MIN_STEP_M:
                    self.points.append(point)
            if self.mission_active and not mission_active:
                self.save()                         # fine missione: la scia va su file
            self.mission_active = mission_active

    def message(self, after=None):
        with self.lock:
            offset = after if after is not None and 0 <= after <= len(self.points) else 0
            return {"trail_id": self.trail_id, "offset": offset, "points": self.points[offset:]}
