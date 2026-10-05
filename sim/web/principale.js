// principale.js - Schede, stato del robot e del GD32 (GET /stato a 10 Hz), azioni sul robot simulato.
"use strict";
(() => {
  const $ = id => document.getElementById(id);
  const LED = { 0: "spento", 1: "azzurro fisso", 2: "azzurro fisso", 3: "giallo fisso", 4: "giallo che respira (carica)", 5: "spento",
    6: "giallo, lampeggio lento (batteria scarica)", 7: "rosso fisso", 8: "rosso, lampeggio veloce (errore)", 9: "spento",
    10: "rosso e giallo, lento", 11: "azzurro, lampeggio lento", 12: "azzurro che respira (avvio)", 13: "azzurro fisso",
    14: "azzurro fisso", 15: "azzurro fisso", 16: "rosso, lampeggio lento", 17: "rosso + azzurro tenue", 18: "giallo + rosso" };
  const BIT = ["sx lat", "sx ant", "dx ant", "dx lat"];
  let nomeStanza = null;

  // schede
  document.querySelectorAll("#schede button").forEach(b => b.onclick = () => {
    App.scheda = b.dataset.scheda;
    document.querySelectorAll("#schede button").forEach(x => x.classList.toggle("attivo", x === b));
    for (const s of ["stanza", "robot", "messaggi"]) $("s-" + s).hidden = s !== App.scheda;
    if (App.scheda === "messaggi") Ispettore.disegna();
  });

  // azioni
  $("solleva").onclick = () => Editor.comando(App.stato && App.stato.sollevato ? "posa" : "solleva");
  $("cassetto").onclick = () => Editor.comando("cassetto " + (App.stato && App.stato.cassetto ? 0 : 1));
  for (const [id, nome] of [["tasto_acc", "accensione"], ["tasto_casa", "casa"]]) {
    const b = $(id);
    b.onmousedown = () => { Editor.comando(`tasto ${nome} 1`); b.classList.add("attivo"); };
    b.onmouseup = b.onmouseleave = () => { if (b.classList.contains("attivo")) { Editor.comando(`tasto ${nome} 0`); b.classList.remove("attivo"); } };
  }
  $("imposta_soc").onclick = () => Editor.comando("batteria " + $("soc").value);

  const riga = (a, b, c) => `<tr><td>${a}</td><td class="${c || ""}">${b}</td></tr>`;
  function pannello(s) {
    const v = BIT.filter((_, k) => (s.cliff >> k) & 1), cl = App.coloreLed(s.led);
    $("tab-robot").innerHTML =
      riga("Posizione", `x ${s.robot.x.toFixed(3)}  y ${s.robot.y.toFixed(3)} m  ${s.robot.dir.toFixed(1)}°` + (s.sollevato ? " (sollevato)" : "") + (s.caduto ? " CADUTO" : ""), s.caduto ? "err" : "") +
      riga("Velocità vera", `${s.v.toFixed(3)} m/s, ${s.w.toFixed(3)} rad/s`) +
      riga("Comando in vigore", `${s.cmd_v.toFixed(3)} m/s, ${s.cmd_w.toFixed(3)} rad/s (applicato ~29 ms dopo 0x66)`) +
      riga("Modalità motori", (["00 spenti", "01 per ruota (0x67)", "02 v/ω (0x66)"][s.modo] || s.modo) + (s.gd32.motori ? "" : " — senza potenza: manca l'heartbeat"), s.gd32.motori ? "" : "att") +
      riga("LED (0x8d)", `<span class="led" style="background:${cl || "transparent"}"></span>${s.led}: ${LED[s.led] || "?"}`) +
      riga("Bumper", (s.bumper[0] ? "SINISTRO " : "") + (s.bumper[1] ? "DESTRO" : "") || "liberi", s.bumper[0] || s.bumper[1] ? "err" : "") +
      riga("Incastro", s.incastrato ? "la torretta del lidar è incastrata (niente bumper!)" : "no", s.incastrato ? "err" : "") +
      riga("Anti-caduta", `0x${s.cliff.toString(16)} ` + (v.length ? "vuoto: " + v.join(", ") : "pavimento"), v.length ? "att" : "") +
      riga("Base", s.contatto_base ? (s.in_carica ? "sui contatti, in carica" : "sui contatti (la carica parte dopo ~12 s)") : "no") +
      riga("IR della base", s.ir_base ? "0x" + s.ir_base.toString(16) : "niente") +
      riga("Batteria", `${s.batteria.v.toFixed(2)} V, ${s.batteria.i.toFixed(2)} A, carica vera ${s.batteria.soc.toFixed(0)} % ` +
        `<span class="tenue">(il driver stima la percentuale con le curve del software originale: è un'altra scala)</span>`,
        s.batteria.v < 13.8 ? "err" : s.batteria.v < 14.2 ? "att" : "") +
      riga("Spazzole", `laterale ${s.spazzole[0]}, centrale ${s.spazzole[1]}`) +
      riga("Aspirazione", `${s.ventola.toFixed(0)} giri/min`) +
      riga("Imbardata IMU", `${s.imbardata.toFixed(1)}°`);
    $("tab-gd32").innerHTML =
      riga("GD32", !s.gd32.acceso ? "SPENTO (reset)" : s.gd32.sospeso ? "in sospensione" : "acceso" + (s.gd32.sensori_accesi ? "" : ", sensori spenti (0x9d 00)") + (s.gd32.reset ? `, ${s.gd32.reset} reset!` : ""), !s.gd32.acceso || s.gd32.reset ? "err" : "") +
      riga("Heartbeat 0x06", s.gd32.senza_hb >= 999 ? "mai ricevuto" : `${s.gd32.senza_hb.toFixed(1)} s fa`, s.gd32.senza_hb > 1.9 ? "att" : "ok") +
      riga("Keepalive 0x0d", `${s.gd32.senza_ka.toFixed(0)} s fa (reset a 190 s)`, s.gd32.senza_ka > 150 ? "err" : s.gd32.senza_ka > 10 ? "att" : "ok") +
      riga("Frame", `${s.gd32.rx} ricevuti dal SoC, ${s.gd32.sensori} frame dei sensori mandati`) +
      riga("Lidar", s.lidar.alimentato ? `potenza ${s.lidar.potenza}, ${s.lidar.giri_s.toFixed(2)} giri/s, ${s.lidar.punti.length / 2} punti nell'ultimo giro` : "spento");
    $("solleva").textContent = s.sollevato ? "Posa a terra" : "Solleva";
    $("cassetto").textContent = s.cassetto ? "Togli cassetto" : "Rimetti cassetto";
    if (s.avviso && performance.now() - (App.tAvviso || 0) > 4000) {
      $("avviso").textContent = `${s.t_avviso.toFixed(1)} s: ${s.avviso}`;
      $("avviso").className = /RESET|PERICOLO|CADUTO|esaurita/.test(s.avviso) ? "err" : "att";
    }
  }

  async function ciclo() {
    try {
      const s = await (await fetch("/stato")).json();
      App.stato = s;
      if (nomeStanza !== null && s.stanza !== nomeStanza) await Editor.ricarica();   // caricata da fuori
      nomeStanza = s.stanza;
      if (App.scheda === "robot") pannello(s);
      document.title = `Simulatore RTES – ${s.stanza}`;
    } catch (e) {
      $("avviso").textContent = "il simulatore non risponde"; $("avviso").className = "err";
    }
    setTimeout(ciclo, 100);
  }

  // indirizzo con la scheda (#messaggi, #robot) e/o la selezione (#sel=4, #sel=robot, #sel=base)
  async function daIndirizzo() {
    const h = location.hash.slice(1);
    for (const parte of h.split("&")) {
      const [k, v] = parte.split("=");
      if (["stanza", "robot", "messaggi"].includes(k)) document.querySelector(`#schede button[data-scheda=${k}]`).click();
      if (k === "sel") { App.sel = v === "robot" ? { k: "robot" } : v === "base" ? { k: "base" } : { k: "o", i: +v }; Editor.aggiornaPannello(); }
    }
  }
  Editor.avvia().then(daIndirizzo);
  Ispettore.avvia();
  ciclo();
})();
