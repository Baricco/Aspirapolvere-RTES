#!/usr/bin/env python3
"""prova-driver.py - Prova d'insieme del driver (fase 3): rtes-driver contro il simulatore, sul PC.

Ogni scenario avvia un simulatore nuovo (build/host/sim, su una porta sua) e il driver (build/host/rtes-driver) sulle
sue pseudo-seriali, manda comandi al driver (stdin) e alla stanza (POST /comando), poi controlla la "verità" del
simulatore (GET /stato) e le righe di stato del driver.

  1. avvio e keepalive: il driver è pronto, sensori a 50 Hz; con il reset del GD32 accorciato a 4 s non c'è reset,
     mentre senza driver il GD32 si resetta (controllo della prova stessa);
  2. lidar: acceso, regolato a 6 giri/s, giri completi pubblicati;
  3. bumper: il robot va contro il muro, si ferma (bumper), poi torna indietro;
  4. vuoto: il robot va verso una scala, si ferma prima del bordo e non cade;
  5. sollevato: nessun movimento;
  6. base: sulla base niente ruote e sensori spenti; con esci_base 1 si allontana e i sensori si riaccendono;
  7. batteria critica: motori fermi e LED 6;
  8. watchdog: un task critico si blocca -> guasto, LED 8, motori fermi, codice d'uscita 4;
  9. il simulatore si chiude: il driver va in guasto senza girare a vuoto.

Uso: tools/prova-driver.py [cartella dei programmi, default build/host]
"""
import json, os, re, subprocess, sys, threading, time, urllib.request

BIN = sys.argv[1] if len(sys.argv) > 1 else "build/host"
falliti = 0
porta_base = 18100 + os.getpid() % 500


def controlla(cond, testo):
    global falliti
    print(f"  {'ok' if cond else 'FALLITO'}: {testo}")
    if not cond:
        falliti += 1


class Sim:
    def __init__(self, stanza, *opzioni):
        global porta_base
        porta_base += 1
        self.porta = porta_base
        self.dir = f"build/sim-prova-{self.porta}"
        self.p = subprocess.Popen([f"{BIN}/sim", "--porta", str(self.porta), "--dir", self.dir, *opzioni, stanza],
                                  stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        for _ in range(100):
            if os.path.exists(f"{self.dir}/lidar"):
                break
            time.sleep(0.02)
        time.sleep(0.1)

    def stato(self):
        return json.load(urllib.request.urlopen(f"http://127.0.0.1:{self.porta}/stato", timeout=2))

    def aggiungi(self, riga):
        """Aggiunge una riga (un oggetto) alla stanza corrente."""
        testo = urllib.request.urlopen(f"http://127.0.0.1:{self.porta}/stanza", timeout=2).read().decode()
        urllib.request.urlopen(urllib.request.Request(f"http://127.0.0.1:{self.porta}/stanza",
                                                      data=(testo + riga + "\n").encode()), timeout=2).read()

    def comando(self, c):
        r = urllib.request.urlopen(urllib.request.Request(f"http://127.0.0.1:{self.porta}/comando", data=c.encode()),
                                   timeout=2)
        return r.read().decode()

    def ferma(self):
        if self.p.poll() is None:
            self.p.terminate()
            self.p.wait()


class Driver:
    def __init__(self, sim, *opzioni):
        self.p = subprocess.Popen([f"{BIN}/rtes-driver", "--gd32", f"{sim.dir}/gd32", "--lidar", f"{sim.dir}/lidar",
                                   "--ogni", "100", "--senza-rt", *opzioni],
                                  stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                                  bufsize=1)
        self.righe = []
        threading.Thread(target=self._leggi, daemon=True).start()

    def _leggi(self):
        for r in self.p.stdout:
            self.righe.append(r.rstrip("\n"))

    def manda(self, c):
        self.p.stdin.write(c + "\n")
        self.p.stdin.flush()

    def ultima(self):
        """L'ultima riga di stato (quelle che iniziano con il tempo)."""
        for r in reversed(self.righe):
            if re.match(r"\s*\d+\.\d ", r):
                return r
        return ""

    def limiti(self):
        """I motivi per cui il driver sta limitando il moto (campo "lim" dell'ultima riga di stato)."""
        m = re.search(r" lim (\S+) ", self.ultima())
        return set(m.group(1).split(",")) if m else set()

    def aspetta(self, cond, max_s):
        t0 = time.time()
        while time.time() - t0 < max_s:
            if cond():
                return True
            time.sleep(0.05)
        return False

    def ferma(self):
        if self.p.poll() is None:
            self.manda("fine")
            try:
                self.p.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.p.kill()
                self.p.wait()
        return self.p.returncode


def scenario(nome, stanza, funz, opz_sim=(), opz_driver=()):
    print(nome)
    sim = Sim(stanza, *opz_sim)
    drv = Driver(sim, *opz_driver)
    try:
        pronto = drv.aspetta(lambda: " pronto " in drv.ultima(), 3)
        controlla(pronto, "driver pronto entro 3 s")
        if pronto:
            funz(sim, drv)
    finally:
        cod = drv.ferma()
        sim.ferma()
    return cod, drv


def s_avvio(sim, drv):
    time.sleep(6)
    st = sim.stato()
    controlla(st["gd32"]["reset"] == 0, f"nessun reset del GD32 in 6 s (timeout del keepalive accorciato a 4 s)")
    controlla(st["gd32"]["senza_ka"] < 3.5, f"ultimo keepalive {st['gd32']['senza_ka']:.1f} s fa (< 3,5)")
    controlla(st["gd32"]["senza_hb"] < 0.3, f"ultimo heartbeat {st['gd32']['senza_hb']:.2f} s fa (< 0,3)")
    m = re.search(r"sens (\d+) Hz", drv.ultima())
    controlla(m and 48 <= int(m.group(1)) <= 52, f"sensori a 50 Hz ({m.group(1) if m else '?'})")
    controlla(st["modo"] == 2 and st["led"] == 1, f"modalità motori 02 e LED azzurro (modo {st['modo']}, led {st['led']})")
    controlla(st["gd32"]["cliff_retromarcia"] == 1, "retromarcia automatica sugli anti-caduta attiva (0x19 01)")


def s_lidar(sim, drv):
    drv.manda("lidar 1")
    time.sleep(6)
    st = sim.stato()
    controlla(5.8 <= st["lidar"]["giri_s"] <= 6.2, f"lidar regolato a 6 giri/s ({st['lidar']['giri_s']:.2f})")
    m = re.search(r"lidar ok ([\d.]+) g/s p(\d+) giri (\d+)", drv.ultima())
    controlla(m and int(m.group(3)) >= 15, f"giri completi ricevuti dal driver ({m.group(3) if m else '?'})")
    controlla(m and 60 <= int(m.group(2)) <= 76, f"potenza a regime ~68 ({m.group(2) if m else '?'})")
    drv.manda("lidar 0")
    time.sleep(1.5)
    st = sim.stato()
    controlla(not st["lidar"]["alimentato"] and st["lidar"]["potenza"] == 0, "lidar spento (0x97 00, 0x71 0)")


def s_bumper(sim, drv):
    sim.comando("sposta 2.3 1.5 0")
    drv.manda("v 0.2 0")
    visto = drv.aspetta(lambda: "bumper" in drv.limiti(), 5)
    time.sleep(0.5)
    st = sim.stato()
    controlla(visto, "il driver vede il bumper e limita il moto in avanti")
    controlla(2.70 <= st["robot"]["x"] <= 2.75, f"il robot è fermo contro il muro (x = {st['robot']['x']:.3f}, atteso 2,726)")
    drv.manda("v -0.1 0")
    time.sleep(1.5)
    st2 = sim.stato()
    controlla(st2["robot"]["x"] < st["robot"]["x"] - 0.08, f"indietro si può ({st['robot']['x']:.3f} -> {st2['robot']['x']:.3f})")


def s_vuoto(sim, drv):
    sim.aggiungi("vuoto nome=scala punti=2.5,0.1 2.9,0.1 2.9,2.9 2.5,2.9")
    sim.comando("sposta 1.6 1.5 0")
    drv.manda("v 0.15 0")
    visto = drv.aspetta(lambda: "vuoto" in drv.limiti(), 8)
    time.sleep(1.0)
    st = sim.stato()
    controlla(visto, "il driver vede il vuoto e limita il moto in avanti")
    controlla(not st["caduto"] and st["robot"]["x"] < 2.42, f"il robot non cade (x = {st['robot']['x']:.3f}, bordo a 2,5)")
    controlla(abs(st["v"]) < 0.02, f"ed è fermo (v = {st['v']:.3f})")


def s_sollevato(sim, drv):
    sim.comando("solleva")
    time.sleep(0.3)
    drv.manda("v 0.1 0.5")
    drv.manda("spazzole 0 30")
    time.sleep(1)
    st = sim.stato()
    controlla("sollevato" in drv.limiti(), "il driver capisce che il robot è sollevato")
    controlla(st["cmd_v"] == 0 and st["cmd_w"] == 0, f"al GD32 va velocità zero ({st['cmd_v']}, {st['cmd_w']})")
    controlla(st["spazzole"][1] == 0, "spazzola centrale ferma")


def s_base(sim, drv):
    sim.comando("sposta 0.6 0.3955 90")             # contatti DIETRO: le spalle alla base (y = 0,22 + raggio)
    ok = drv.aspetta(lambda: "BASE" in drv.ultima(), 2)
    time.sleep(1.0)
    st = sim.stato()
    controlla(ok and st["contatto_base"], "il driver vede la base")
    controlla(st["gd32"]["sensori_accesi"] == 0, "sensori spenti sulla base (0x9d 00, come everest)")
    drv.manda("v -0.1 0")
    time.sleep(1)
    st = sim.stato()
    controlla(st["contatto_base"] and abs(st["v"]) < 0.01, "sulla base le ruote non si muovono")
    drv.manda("esci_base 1")
    drv.manda("v 0.1 0")                            # si esce IN AVANTI
    time.sleep(2.5)
    st = sim.stato()
    controlla(not st["contatto_base"] and st["robot"]["y"] > 0.5, f"con esci_base 1 si allontana in avanti (y = {st['robot']['y']:.2f})")
    time.sleep(1)
    controlla(sim.stato()["gd32"]["sensori_accesi"] == 1, "fuori dalla base i sensori si riaccendono")


def s_batteria(sim, drv):
    sim.comando("batteria 0")
    ok = drv.aspetta(lambda: "batteria" in drv.limiti(), 4)
    drv.manda("v 0.1 0")
    time.sleep(0.5)
    st = sim.stato()
    controlla(ok, "batteria critica vista dal driver (" + (re.search(r"[\d.]+ V", drv.ultima()) or [""])[0] + ")")
    controlla(st["cmd_v"] == 0, "motori fermi")
    controlla(st["led"] == 6, f"LED 6 (giallo, lampeggio lento): {st['led']}")


def s_watchdog(sim, drv):
    drv.manda("azzera")                  # richiesta fatta senza guasto: non deve valere per il guasto che verrà
    time.sleep(0.5)
    drv.manda("v 0.05 0")
    time.sleep(0.5)
    t0 = time.time()
    drv.manda("blocca")
    ok = drv.aspetta(lambda: "GUASTO" in drv.ultima(), 2)
    dt = time.time() - t0
    time.sleep(0.1)                     # il GD32 (anche quello simulato) applica la velocità ~29 ms dopo il comando
    st = sim.stato()
    controlla(ok, f"guasto dopo {dt:.2f} s dal blocco del task")
    controlla(ok and "prova-sensori" in drv.ultima(), "il motivo nomina il task")
    controlla(st["led"] == 8 and st["cmd_v"] == 0, f"LED 8 e motori fermi (led {st['led']}, v {st['cmd_v']})")
    drv.manda("v 0.1 0")
    time.sleep(0.5)
    controlla(sim.stato()["cmd_v"] == 0, "in guasto i comandi ai motori sono rifiutati")
    controlla("GUASTO" in drv.ultima(), "il guasto resta (la richiesta di azzeramento fatta prima non vale)")
    drv.manda("azzera")
    time.sleep(0.5)
    controlla(any("NO, la causa" in r for r in drv.righe), "con il task ancora bloccato il guasto non si azzera")


def main():
    vuota, soggiorno = "sim/stanze/vuota.txt", "sim/stanze/soggiorno.txt"

    cod, _ = scenario("1. avvio e keepalive", vuota, s_avvio, ("--timeout-keepalive", "4"))
    controlla(cod == 0, f"codice d'uscita 0 ({cod})")
    print("1b. controllo: senza driver il GD32 simulato si resetta")
    sim = Sim(vuota, "--timeout-keepalive", "2")
    time.sleep(3)
    controlla(sim.stato()["gd32"]["reset"] >= 1, "reset senza keepalive")
    sim.ferma()

    scenario("2. lidar", vuota, s_lidar)
    scenario("3. bumper", vuota, s_bumper)
    scenario("4. vuoto", vuota, s_vuoto)
    scenario("5. sollevato", vuota, s_sollevato)
    scenario("6. base", soggiorno, s_base)
    scenario("7. batteria critica", vuota, s_batteria, ("--batteria-veloce", "200"))
    cod, _ = scenario("8. watchdog dei task", vuota, s_watchdog, (), ("--task-prova",))
    controlla(cod == 4, f"codice d'uscita 4 (guasto durante la prova): {cod}")

    print("9. il simulatore si chiude")
    sim = Sim(vuota, "--durata", "2")
    drv = Driver(sim)
    ok = drv.aspetta(lambda: "GUASTO" in drv.ultima(), 5)
    controlla(ok and ("chiusa" in drv.ultima() or "sensori" in drv.ultima()), "guasto: seriale chiusa o sensori assenti")
    cpu0 = time.process_time()
    time.sleep(1)
    with open(f"/proc/{drv.p.pid}/stat") as f:
        campi = f.read().rsplit(")", 1)[1].split()
        cpu = (int(campi[11]) + int(campi[12])) / os.sysconf("SC_CLK_TCK")
    controlla(cpu < 0.5, f"il driver non gira a vuoto ({cpu:.2f} s di CPU in totale)")
    drv.ferma()
    sim.ferma()

    for d in os.listdir("build"):
        if d.startswith("sim-prova-"):
            for f in os.listdir(f"build/{d}"):
                os.unlink(f"build/{d}/{f}")
            os.rmdir(f"build/{d}")
    print("PROVA DRIVER: " + ("OK" if falliti == 0 else f"{falliti} CONTROLLI FALLITI"))
    sys.exit(1 if falliti else 0)


if __name__ == "__main__":
    main()
