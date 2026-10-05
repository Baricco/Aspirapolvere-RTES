// ispettore.js - Ispettore dei messaggi: griglia dei 96 byte del frame dei sensori con le spiegazioni, bit allargati,
// valori decodificati, ultimo comando di ogni tipo, ultimo pacchetto del lidar, lista dei messaggi con i filtri, e
// pausa per scorrere i messaggi uno per uno. I dati vengono da GET /storia (tutti i messaggi registrati dal simulatore)
// e la descrizione dei campi da GET /campi (gd32/descrizione.c).
"use strict";
const Ispettore = (() => {
  const MAX = 30000;
  let campi = null, descrSensori = [], nomi = {};
  let voci = [], ultimo = -1, pausa = false, cursore = -1, campoEvid = null;
  const DIREZIONE = ["GD32→SoC", "SoC→GD32", "lidar"];
  const LIDAR_NOMI = { 0xa9: "scansione (vecchia)", 0xab: "salute", 0xac: "versione", 0xad: "scansione", 0xae: "velocità" };
  const $ = id => document.getElementById(id);
  const hex = b => Array.from(b, x => x.toString(16).padStart(2, "0")).join(" ");

  function daHex(h) { const b = new Uint8Array(h.length / 2); for (let i = 0; i < b.length; i++) b[i] = parseInt(h.substr(2 * i, 2), 16); return b; }

  // una voce della storia -> {n, t, c, cmd, dati, testo, rip}
  const precedente = {};
  function analizza([n, t, c, h]) {
    const b = daHex(h), v = { n, t, c, b, cmd: -1, dati: b };
    if (c < 2 && b[0] === 0xfa && b[1] === 0xfb) { v.cmd = b[3]; v.dati = b.slice(4, b.length - 2); }
    else if (c === 2 && b[0] === 0xaa) { v.cmd = b[5]; v.dati = b.slice(8, b[2]); }
    else v.testo = new TextDecoder("latin1").decode(b).replace(/[\r\n]+/g, " ");
    const k = c + ":" + v.cmd, hd = hex(v.dati);
    v.rip = precedente[k] === hd;                    // uguale al messaggio precedente dello stesso tipo
    precedente[k] = hd;
    return v;
  }

  function grezzo(c, d) {
    if (d.length < c.byte + c.n_byte) return null;
    if (c.bit >= 0) return (d[c.byte] >> c.bit) & ((1 << c.n_bit) - 1);
    let v = 0;
    for (let i = c.n_byte - 1; i >= 0; i--) v = v * 256 + d[c.byte + i];
    if (c.tipo[0] === "i") { const m = 2 ** (8 * c.n_byte); if (v >= m / 2) v -= m; }
    return v;
  }
  function testoValore(c, d) {
    const g = grezzo(c, d);
    if (g === null) return "?";
    if (c.bit >= 0) return c.n_bit > 1 ? `${g} (${g.toString(2).padStart(c.n_bit, "0")})` : `${g}`;
    const v = g * c.scala;
    const s = Number.isInteger(c.scala) ? String(v) : Math.abs(v) >= 100 ? v.toFixed(1) : v.toFixed(3);
    return s + (c.unita ? " " + c.unita : "") + (c.scala !== 1 ? `  (grezzo ${g})` : "");
  }
  const AFF = { V: "verificato sul robot", C: "letto nel codice di everest", "?": "ipotesi da verificare" };

  function descriviMessaggio(v) {
    if (v.c === 2) {
      if (v.testo !== undefined) return "testo: " + v.testo;
      const p = v.dati;
      if (v.cmd === 0xad && p.length >= 5) return `${(p[0] * 0.05).toFixed(2)} giri/s, settore da ${((p[3] << 8 | p[4]) / 100).toFixed(1)}°, ${(p.length - 5) / 3} punti`;
      if (v.cmd === 0xae && p.length >= 1) return `${(p[0] * 0.05).toFixed(2)} giri/s`;
      if (v.cmd === 0xac) return new TextDecoder("latin1").decode(p);
      return "";
    }
    const m = campi && campi.messaggi.find(x => x.cmd === v.cmd && x.verso_gd32 === v.c);
    if (!m || v.cmd === 0x15) return v.cmd === 0x15 ? "frame dei sensori" : "";
    if (v.cmd === 0x07 && v.c === 0) return new TextDecoder("latin1").decode(v.dati).replace(/\0.*$/, "");
    return m.campi.map(c => `${c.nome} = ${testoValore(c, v.dati).replace(/\s+\(grezzo.*\)$/, "")}`).join(", ");
  }
  function nomeComando(v) {
    if (v.c === 2) return v.testo !== undefined ? "testo" : (LIDAR_NOMI[v.cmd] || "?");
    return nomi[v.cmd] || "?";
  }

  // ------------------------------------------------------------------------------------------- vista ---
  function indiceCorrente() { return pausa ? cursore : voci.length - 1; }
  function cercaIndietro(da, pred) { for (let i = da; i >= 0; i--) if (pred(voci[i])) return i; return -1; }

  function disegnaGriglia(d, dPrima) {
    const g = $("griglia");
    if (!g.dataset.fatta) {
      let h = "";
      for (let r = 0; r < 8; r++) {
        h += `<div class="ind">${r * 12}</div>`;
        for (let k = 0; k < 12; k++) h += `<div class="b" data-i="${r * 12 + k}"></div>`;
      }
      g.innerHTML = h; g.dataset.fatta = 1;
      g.querySelectorAll(".b").forEach(el => {
        el.onmouseenter = () => mostraByte(+el.dataset.i);
        el.onmouseleave = () => { campoEvid = null; evidenzia(); };
      });
    }
    g.querySelectorAll(".b").forEach(el => {
      const i = +el.dataset.i, c = descrSensori.find(c => i >= c.byte && i < c.byte + c.n_byte);
      el.textContent = d ? d[i].toString(16).padStart(2, "0") : "··";
      el.className = "b g" + (c ? c.gruppo : 7) + (d && dPrima && d[i] !== dPrima[i] ? " cambiato" : "");
    });
    // bit allargati: i byte con campi a bit
    const conBit = [...new Set(descrSensori.filter(c => c.bit >= 0).map(c => c.byte))];
    const b = $("bit");
    let h = "";
    for (const by of conBit) {
      h += `<div class="ind">byte ${by}</div>`;
      for (let k = 7; k >= 0; k--) {
        const c = descrSensori.find(c => c.byte === by && c.bit >= 0 && k >= c.bit && k < c.bit + c.n_bit);
        const val = d ? (d[by] >> k) & 1 : "·";
        h += `<div class="bb g${c ? c.gruppo : 7}${val === 1 ? " uno" : ""}" data-by="${by}" data-k="${k}" title="bit ${k}${c ? ": " + c.nome : ""}">${val}</div>`;
      }
    }
    b.innerHTML = h;
    b.querySelectorAll(".bb").forEach(el => {
      el.onmouseenter = () => {
        const by = +el.dataset.by, k = +el.dataset.k;
        const c = descrSensori.find(c => c.byte === by && c.bit >= 0 && k >= c.bit && k < c.bit + c.n_bit);
        mostraCampi(c ? [c] : [], `byte ${by}, bit ${k}`);
      };
      el.onmouseleave = () => { campoEvid = null; evidenzia(); };
    });
    // tutti i valori
    if ($("valori").open && d)
      $("tab-valori").innerHTML = descrSensori.map(c => `<tr><td>${c.byte}${c.bit >= 0 ? "." + c.bit : ""} ${c.nome}</td><td>${testoValore(c, d)}</td><td title="${AFF[c.aff]}">${c.aff}</td></tr>`).join("");
  }
  let ultimoFrame = null;
  function mostraByte(i) {
    mostraCampi(descrSensori.filter(c => i >= c.byte && i < c.byte + c.n_byte), `byte ${i}`);
  }
  function mostraCampi(lista, dove) {
    const d = ultimoFrame;
    campoEvid = lista[0] || null;
    evidenzia();
    if (!lista.length) { $("dettaglio").innerHTML = `${dove}: nessun campo conosciuto (sempre 0 o non usato da everest)`; return; }
    $("dettaglio").innerHTML = lista.map(c =>
      `<b>${c.nome}</b> — ${dove}, ${c.bit >= 0 ? `bit ${c.bit}${c.n_bit > 1 ? "-" + (c.bit + c.n_bit - 1) : ""}` : `byte ${c.byte}${c.n_byte > 1 ? "-" + (c.byte + c.n_byte - 1) : ""}, ${c.tipo}`}<br>` +
      `valore: <b>${d ? testoValore(c, d) : "?"}</b> · <span title="affidabilità">${AFF[c.aff]}</span><br><span class="tenue">${c.spiegazione}</span>`).join("<hr>");
  }
  function evidenzia() {
    document.querySelectorAll("#griglia .b").forEach(el => {
      const i = +el.dataset.i;
      el.classList.toggle("evid", !!campoEvid && i >= campoEvid.byte && i < campoEvid.byte + campoEvid.n_byte);
    });
  }

  function disegna() {
    if (!campi || App.scheda !== "messaggi") return;
    const fine = indiceCorrente();
    // frame dei sensori al momento scelto e quello prima
    const is = cercaIndietro(fine, v => v.c === 0 && v.cmd === 0x15);
    const ip = is > 0 ? cercaIndietro(is - 1, v => v.c === 0 && v.cmd === 0x15) : -1;
    ultimoFrame = is >= 0 ? voci[is].dati : null;
    disegnaGriglia(ultimoFrame, ip >= 0 ? voci[ip].dati : null);
    $("sens-info").textContent = is >= 0 ? `t = ${voci[is].t.toFixed(3)} s` + (ip >= 0 ? `, ${((voci[is].t - voci[ip].t) * 1000).toFixed(2)} ms dopo il precedente` : "") : "";
    // ultimo comando di ogni tipo (tutto tranne sensori e lidar), fino al momento scelto
    const visti = new Map();
    for (let i = fine; i >= 0 && visti.size < 60; i--) {
      const v = voci[i];
      if (v.c === 2 || v.cmd === 0x15) continue;
      const k = v.c + ":" + v.cmd;
      if (!visti.has(k)) visti.set(k, v);
    }
    $("tab-comandi").innerHTML = "<tr><th>t (s)</th><th>verso</th><th>messaggio</th><th>significato</th></tr>" +
      [...visti.values()].sort((a, b) => a.c - b.c || a.cmd - b.cmd).map(v =>
        `<tr><td>${v.t.toFixed(2)}</td><td>${DIREZIONE[v.c]}</td><td class="mono">0x${v.cmd.toString(16).padStart(2, "0")} ${nomeComando(v)}<br><span class="tenue">${hex(v.dati)}</span></td><td>${descriviMessaggio(v)}</td></tr>`).join("");
    // lidar
    const il = cercaIndietro(fine, v => v.c === 2);
    if (il >= 0) {
      const v = voci[il];
      let h = `t = ${v.t.toFixed(3)} s · ${nomeComando(v)} · ${descriviMessaggio(v)}`;
      if (v.cmd === 0xad && v.dati.length >= 5) {
        const p = v.dati, n = (p.length - 5) / 3, a0 = (p[3] << 8 | p[4]) / 100, pts = [];
        for (let i = 0; i < n; i++) {
          const q = p[6 + 3 * i] << 8 | p[7 + 3 * i];
          pts.push(`${(a0 + i * 22.5 / n).toFixed(1).padStart(5)}°: ${q ? (q / 4000).toFixed(3) + " m" : "  —    "} i${String(p[5 + 3 * i]).padStart(3)}`);
        }
        h += "<br>angolo del lidar (0° = dietro, antiorario): distanza, intensità<br>" + pts.join(" · ");
      }
      $("lidar-pacchetto").innerHTML = h;
    }
    const ultimi = voci.filter(v => v.c === 2 && v.t > (voci.length ? voci[voci.length - 1].t - 1 : 0)).length;
    $("lidar-info").textContent = pausa ? "" : `${ultimi} pacchetti/s`;
    disegnaLista(fine);
    $("posizione").textContent = pausa && cursore >= 0 ? `messaggio ${voci[cursore].n} (t = ${voci[cursore].t.toFixed(3)} s): ${voci.length} in memoria` : "";
  }

  function passa(v) {
    if (v.c === 0 && v.cmd === 0x15) return $("f-sensori").checked;
    if (v.c === 2) return $("f-lidar").checked;
    if (v.rip) return $("f-ripetizioni").checked;
    return true;
  }
  function disegnaLista(fine) {
    const righe = [];
    let i = pausa ? Math.min(voci.length - 1, fine + 40) : voci.length - 1;
    for (; i >= 0 && righe.length < 300; i--) {
      const v = voci[i];
      if (!passa(v) && i !== cursore) continue;
      righe.push(`<div data-i="${i}" class="${pausa && i === cursore ? "sel" : ""}">${v.t.toFixed(3).padStart(9)} <span class="dir">${DIREZIONE[v.c]}</span>` +
        `${v.cmd >= 0 ? "0x" + v.cmd.toString(16).padStart(2, "0") : "    "} ${nomeComando(v).padEnd(20)} ${hex(v.dati.slice(0, 16))}${v.dati.length > 16 ? " …" : ""}  <span class="tenue">${v.cmd !== 0x15 ? descriviMessaggio(v) : ""}</span></div>`);
    }
    const l = $("lista");
    l.innerHTML = righe.join("");
    l.querySelectorAll("div").forEach(el => el.onclick = () => { pausa = true; cursore = +el.dataset.i; aggiornaPulsanti(); disegna(); });
    if (pausa) { const s = l.querySelector(".sel"); if (s) s.scrollIntoView({ block: "nearest" }); }
  }

  function passo(d) {
    if (!pausa) return;
    let i = cursore;
    do i += d; while (i >= 0 && i < voci.length && !passa(voci[i]));
    if (i >= 0 && i < voci.length) { cursore = i; disegna(); }
  }
  function aggiornaPulsanti() {
    $("pausa").textContent = pausa ? "Riprendi" : "Pausa";
    $("pausa").classList.toggle("attivo", pausa);
    $("indietro").disabled = $("avanti").disabled = !pausa;
  }

  async function scarica() {
    try {
      const r = await (await fetch("/storia?dopo=" + ultimo)).json();
      if (r.voci.length) {
        for (const x of r.voci) voci.push(analizza(x));
        ultimo = r.voci[r.voci.length - 1][0];
        if (voci.length > MAX) { const via = voci.length - MAX; voci.splice(0, via); if (pausa) cursore = Math.max(0, cursore - via); }
        if (!pausa) disegna();
      }
      if (r.voci.length >= 4000) return scarica();     // c'è dell'altro
    } catch (e) { /* il simulatore non risponde: ci pensa principale.js */ }
  }

  async function avvia() {
    campi = await (await fetch("/campi")).json();
    nomi = campi.nomi;
    const s = campi.messaggi.find(m => m.cmd === 0x15 && m.verso_gd32 === 0);
    descrSensori = s ? s.campi : [];
    $("pausa").onclick = () => { pausa = !pausa; cursore = voci.length - 1; aggiornaPulsanti(); disegna(); };
    $("indietro").onclick = () => passo(-1);
    $("avanti").onclick = () => passo(1);
    ["f-ripetizioni", "f-sensori", "f-lidar"].forEach(id => $(id).onchange = disegna);
    $("valori").ontoggle = disegna;
    window.addEventListener("keydown", e => {
      if (App.scheda !== "messaggi" || e.target.tagName === "INPUT") return;
      if (e.key === "ArrowLeft") { e.preventDefault(); passo(-1); }
      if (e.key === "ArrowRight") { e.preventDefault(); passo(1); }
      if (e.key === " ") { e.preventDefault(); $("pausa").click(); }
    });
    setInterval(scarica, 250);
  }
  return { avvia, disegna };
})();
