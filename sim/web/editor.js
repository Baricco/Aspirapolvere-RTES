// editor.js - Vista della stanza ed editor a oggetti: selezione, maniglie (sposta, ruota, misura), strumenti di disegno,
// oggetti pronti, pannello delle proprietà, tastiera, annulla/ripeti, invio della stanza al simulatore.
"use strict";

const App = {
  stato: null,                 // ultimo /stato del simulatore (principale.js)
  st: null,                    // la stanza (modello di stanza.js): la copia della pagina è quella che conta
  sel: null,                   // null | {k:"o", i} | {k:"robot"} | {k:"base"}
  strumento: "seleziona", libreria: "",
  scheda: "stanza",
  undo: [], redo: [],
  anteprimaRobot: null,        // posa del robot mentre lo si trascina
};

const Editor = (() => {
  const tela = document.getElementById("tela"), ctx = tela.getContext("2d");
  const V = { s: 100, ox: 0, oy: 0, adattata: false };
  const GRADI = Math.PI / 180;
  let col = {};                // colori dal CSS
  let trascina = null, panoramica = null, disegno = null, mouse = [0, 0], prima = null;

  const colore = n => getComputedStyle(document.documentElement).getPropertyValue(n).trim();
  const aSchermo = (x, y) => [V.ox + x * V.s, V.oy - y * V.s];
  const aMondo = (px, py) => [(px - V.ox) / V.s, (V.oy - py) / V.s];
  const cm = v => Math.round(v * 100) / 100;                   // scatto di 1 cm
  const avviso = (t, c) => { const a = document.getElementById("avviso"); a.textContent = t; a.className = c || ""; App.tAvviso = performance.now(); };

  // ------------------------------------------------------------------------------------------- sincronia ---
  let timer = null;
  function invia(riposiziona) {
    clearTimeout(timer);
    return new Promise(ok => {
      timer = setTimeout(async () => {
        const r = await fetch("/stanza" + (riposiziona ? "?riposiziona=1" : ""), { method: "POST", body: Stanza.scrivi(App.st) });
        if (!r.ok) avviso("stanza rifiutata dal simulatore: " + await r.text(), "err");
        ok(r.ok);
      }, riposiziona ? 0 : 120);
    });
  }
  async function comando(c) {
    const r = await fetch("/comando", { method: "POST", body: c });
    const t = await r.text();
    if (!r.ok) avviso(t, "err");
    return r.ok ? t : null;
  }
  async function ricarica() {
    App.st = Stanza.leggi(await (await fetch("/stanza")).text());
    App.sel = null; App.undo = []; App.redo = [];
    aggiornaPannello();
  }
  function istantanea() { return Stanza.scrivi(App.st); }
  function registraModifica(testoPrima) {            // dopo una modifica: annulla torna a testoPrima
    if (testoPrima === istantanea()) return;
    App.undo.push(testoPrima); if (App.undo.length > 200) App.undo.shift();
    App.redo = [];
    invia(); aggiornaPannello();
  }
  function modifica(f) { const p = istantanea(); f(); registraModifica(p); }
  function selezioneValida() {      // dopo annulla/ripeti la selezione resta, se quello che indica esiste ancora
    if (App.sel && ((App.sel.k === "o" && App.sel.i >= App.st.oggetti.length) || (App.sel.k === "base" && !App.st.base))) App.sel = null;
  }
  function annulla() { if (!App.undo.length) return; App.redo.push(istantanea()); App.st = Stanza.leggi(App.undo.pop()); selezioneValida(); invia(); aggiornaPannello(); }
  function ripeti() { if (!App.redo.length) return; App.undo.push(istantanea()); App.st = Stanza.leggi(App.redo.pop()); selezioneValida(); invia(); aggiornaPannello(); }

  // ----------------------------------------------------------------------------------------- selezione ---
  function membri() {                                  // indici degli oggetti mossi insieme (il gruppo)
    if (!App.sel || App.sel.k !== "o") return [];
    const o = App.st.oggetti[App.sel.i];
    if (!o) return [];
    if (!o.gruppo) return [App.sel.i];
    return App.st.oggetti.map((x, i) => x.gruppo === o.gruppo ? i : -1).filter(i => i >= 0);
  }
  function centroSel() {
    if (!App.sel) return null;
    if (App.sel.k === "robot") { const r = posaRobot(); return [r.x, r.y]; }
    if (App.sel.k === "base") return [App.st.base.x, App.st.base.y];
    const m = membri(), c = m.map(i => Stanza.centro(App.st.oggetti[i]));
    return [c.reduce((s, p) => s + p[0], 0) / c.length, c.reduce((s, p) => s + p[1], 0) / c.length];
  }
  function raggioSel() {                               // raggio di un cerchio che contiene la selezione
    if (!App.sel) return 0;
    if (App.sel.k === "robot") return App.stato ? App.stato.geometria.raggio : 0.175;
    if (App.sel.k === "base") return 0.2;
    const [cx, cy] = centroSel();
    let r = 0.05;
    for (const i of membri()) {
      const o = App.st.oggetti[i];
      for (const p of puntiContorno(o)) r = Math.max(r, Math.hypot(p[0] - cx, p[1] - cy));
    }
    return r;
  }
  function puntiContorno(o) {
    if (o.tipo === "vuoto") return o.punti;
    if (o.tipo === "cerchio") return [[o.x - o.raggio, o.y], [o.x + o.raggio, o.y], [o.x, o.y - o.raggio], [o.x, o.y + o.raggio]];
    const r = Stanza.comeRettangolo(o), a = r.angolo * GRADI, c = Math.cos(a), s = Math.sin(a);
    return [[1, 1], [-1, 1], [-1, -1], [1, -1]].map(([u, v]) => [r.x + (u * r.larghezza / 2) * c - (v * r.profondita / 2) * s,
                                                               r.y + (u * r.larghezza / 2) * s + (v * r.profondita / 2) * c]);
  }
  function posaRobot() {
    if (App.anteprimaRobot) return App.anteprimaRobot;
    const r = App.stato ? App.stato.robot : App.st.robot;
    return { x: r.x, y: r.y, angolo: r.dir !== undefined ? r.dir : r.angolo };
  }

  // maniglie della selezione, in coordinate della stanza
  function maniglie() {
    const m = [];
    if (!App.sel) return m;
    const off = 28 / V.s;
    if (App.sel.k === "robot") {
      const r = posaRobot(), R = raggioSel(), a = r.angolo * GRADI;
      m.push({ t: "ruota", x: r.x + Math.cos(a) * (R + off), y: r.y + Math.sin(a) * (R + off) });
      return m;
    }
    if (App.sel.k === "base") {
      const b = App.st.base, a = b.angolo * GRADI;
      m.push({ t: "ruota", x: b.x + Math.cos(a) * off * 1.5, y: b.y + Math.sin(a) * off * 1.5 });
      return m;
    }
    const mm = membri(), [cx, cy] = centroSel(), R = raggioSel();
    // maniglia di rotazione: sopra l'oggetto se si vede, altrimenti sotto, a destra o a sinistra
    const visibile = (x, y) => { const [px, py] = aSchermo(x, y); return px > 12 && py > 48 && px < tela.clientWidth - 12 && py < tela.clientHeight - 12; };
    const posti = [[cx, cy + R + off], [cx, cy - R - off], [cx + R + off, cy], [cx - R - off, cy]];
    const [hx, hy] = posti.find(p => visibile(...p)) || posti[0];
    m.push({ t: "ruota", x: hx, y: hy });
    if (mm.length !== 1) return m;
    const o = App.st.oggetti[mm[0]];
    if (o.tipo === "rettangolo") puntiContorno(o).forEach((p, k) => m.push({ t: "misura", x: p[0], y: p[1], k }));
    else if (o.tipo === "cerchio") m.push({ t: "raggio", x: o.x + o.raggio, y: o.y });
    else if (o.tipo === "muro") { m.push({ t: "capo", x: o.da[0], y: o.da[1], k: "da" }); m.push({ t: "capo", x: o.a[0], y: o.a[1], k: "a" }); }
    else if (o.tipo === "vuoto") o.punti.forEach((p, k) => m.push({ t: "vertice", x: p[0], y: p[1], k }));
    return m;
  }

  function colpito(x, y) {                             // cosa c'è sotto il punto: robot, base, oggetto più piccolo
    const r = posaRobot(), R = App.stato ? App.stato.geometria.raggio : 0.175;
    if (Math.hypot(x - r.x, y - r.y) <= R) return { k: "robot" };
    const b = App.st.base;
    if (b) {
      const [u, v] = Stanza.locale({ x: b.x, y: b.y, angolo: b.angolo }, x, y);
      if (u <= 0.01 && u >= -0.13 && Math.abs(v) <= 0.16) return { k: "base" };
    }
    let meglio = null, area = Infinity;
    App.st.oggetti.forEach((o, i) => {
      if (Stanza.dentro(o, x, y, 4 / V.s)) { const a = Stanza.area(o); if (a < area) { area = a; meglio = { k: "o", i }; } }
    });
    return meglio;
  }

  // ------------------------------------------------------------------------------------------ oggetti pronti ---
  let contaGruppi = 0;
  function nuovoGruppo(nome) {
    const usati = new Set(App.st.oggetti.map(o => o.gruppo));
    let g; do g = nome + (++contaGruppi); while (usati.has(g));
    return g;
  }
  function pronto(tipo, x, y) {
    const R = (nome, l, p, qa, qd = 0, gruppo = "") => ({ tipo: "rettangolo", nome, x, y, larghezza: l, profondita: p, angolo: 0, quota_da: qd, quota_a: qa, gruppo });
    const C = (nome, dx, dy, r, qa, gruppo) => ({ tipo: "cerchio", nome, x: cm(x + dx), y: cm(y + dy), raggio: r, quota_da: 0, quota_a: qa, gruppo });
    const gambe = (nome, l, p, r, qa, g) => [[-1, -1], [1, -1], [-1, 1], [1, 1]].map(([a, b]) => C(nome, a * (l / 2 - r - 0.02), b * (p / 2 - r - 0.02), r, qa, g));
    switch (tipo) {
      case "divano": return [R("divano", 2.0, 0.85, 0.45)];
      case "armadio": return [R("armadio", 1.0, 0.6, 2.0)];
      case "credenza": return [R("credenza", 1.2, 0.45, 0.85, 0.09)];
      case "soglia": return [R("soglia", 0.9, 0.05, 0.015)];
      case "cavo": return [R("gradino", 0.9, 0.05, 0.03)];
      case "pianta": return [C("pianta", 0, 0, 0.15, 1.0, "")];
      case "scala": return [{ tipo: "vuoto", nome: "scala", gruppo: "", quota_da: 0, quota_a: 0,
                               punti: [[x - 0.5, y - 0.4], [x + 0.5, y - 0.4], [x + 0.5, y + 0.4], [x - 0.5, y + 0.4]].map(p => p.map(cm)) }];
      case "tavolo": { const g = nuovoGruppo("tavolo"); return [R("piano", 1.2, 0.8, 0.75, 0.72, g), ...gambe("gamba", 1.2, 0.8, 0.025, 0.72, g)]; }
      case "sedia": { const g = nuovoGruppo("sedia"); return [R("seduta", 0.42, 0.42, 0.48, 0.45, g), ...gambe("gamba", 0.42, 0.42, 0.015, 0.45, g)]; }
      case "letto": { const g = nuovoGruppo("letto"); return [R("letto", 1.6, 2.0, 0.5, 0.12, g), ...gambe("gamba", 1.6, 2.0, 0.03, 0.12, g)]; }
    }
    return [];
  }

  // ------------------------------------------------------------------------------------------- mouse ---
  function punto(e) { const r = tela.getBoundingClientRect(); return [e.clientX - r.left, e.clientY - r.top]; }

  tela.addEventListener("contextmenu", e => e.preventDefault());
  tela.addEventListener("wheel", e => {
    e.preventDefault();
    const k = Math.exp(-e.deltaY * 0.0015), [px, py] = punto(e);
    V.ox = px - (px - V.ox) * k; V.oy = py - (py - V.oy) * k; V.s *= k;
  }, { passive: false });
  tela.addEventListener("dblclick", e => {
    const [x, y] = aMondo(...punto(e));
    if (!colpito(x, y)) { V.adattata = false; adatta(); }
  });

  tela.addEventListener("mousedown", e => {
    tela.focus();
    const [px, py] = punto(e), [x, y] = aMondo(px, py);
    if (e.button === 2 || e.button === 1) { panoramica = [px, py, V.ox, V.oy]; return; }
    if (App.libreria) {
      const lib = App.libreria;
      if (lib === "base") modifica(() => { App.st.base = { x: cm(x), y: cm(y), angolo: App.st.base ? App.st.base.angolo : 90 }; App.sel = { k: "base" }; });
      else modifica(() => { const n = App.st.oggetti.length; App.st.oggetti.push(...pronto(lib, cm(x), cm(y))); App.sel = { k: "o", i: n }; });
      App.libreria = ""; document.getElementById("libreria").value = "";
      return;
    }
    if (App.strumento !== "seleziona") { disegno = { da: [cm(x), cm(y)], a: [cm(x), cm(y)] }; return; }
    // maniglie della selezione
    for (const m of maniglie()) {
      const [mx, my] = aSchermo(m.x, m.y);
      if (Math.hypot(mx - px, my - py) <= 8) { iniziaTrascina(m, x, y); return; }
    }
    const c = colpito(x, y);
    App.sel = c;
    aggiornaPannello();
    if (c) iniziaTrascina({ t: "sposta" }, x, y);
  });

  function iniziaTrascina(m, x, y) {
    prima = istantanea();
    const centro = centroSel();
    trascina = { m, x0: x, y0: y, centro, ang0: Math.atan2(y - centro[1], x - centro[0]),
                 orig: JSON.parse(JSON.stringify(App.st)), robot0: posaRobot() };
  }

  window.addEventListener("mousemove", e => {
    const [px, py] = punto(e), [x, y] = aMondo(px, py);
    mouse = [x, y];
    document.getElementById("coordinate").textContent = `x ${x.toFixed(2)} m  y ${y.toFixed(2)} m`;
    if (panoramica) { V.ox = panoramica[2] + px - panoramica[0]; V.oy = panoramica[3] + py - panoramica[1]; return; }
    if (disegno) {
      let a = [cm(x), cm(y)];
      if (App.strumento === "muro" && e.shiftKey) {
        const d = Math.hypot(a[0] - disegno.da[0], a[1] - disegno.da[1]);
        const ang = Math.round(Math.atan2(a[1] - disegno.da[1], a[0] - disegno.da[0]) / (15 * GRADI)) * 15 * GRADI;
        a = [cm(disegno.da[0] + d * Math.cos(ang)), cm(disegno.da[1] + d * Math.sin(ang))];
      }
      disegno.a = a; return;
    }
    if (!trascina) return;
    const t = trascina, m = t.m, orig = t.orig;
    // si riparte sempre dallo stato di inizio trascinamento: gli spostamenti non accumulano errori
    App.st.oggetti = JSON.parse(JSON.stringify(orig.oggetti));
    App.st.base = JSON.parse(JSON.stringify(orig.base));
    const dx = cm(x - t.x0), dy = cm(y - t.y0);
    if (App.sel.k === "robot") {
      const r = t.robot0;
      if (m.t === "sposta") App.anteprimaRobot = { x: cm(r.x + dx), y: cm(r.y + dy), angolo: r.angolo };
      else App.anteprimaRobot = { x: r.x, y: r.y, angolo: angoloMouse(r.x, r.y, x, y, e.shiftKey) };
      return;
    }
    if (App.sel.k === "base") {
      const b = App.st.base;
      if (m.t === "sposta") { b.x = cm(orig.base.x + dx); b.y = cm(orig.base.y + dy); }
      else b.angolo = angoloMouse(b.x, b.y, x, y, e.shiftKey);
      return;
    }
    const mm = membri();
    if (m.t === "sposta") mm.forEach(i => Stanza.sposta(App.st.oggetti[i], dx, dy));
    else if (m.t === "ruota") {
      let d = (Math.atan2(y - t.centro[1], x - t.centro[0]) - t.ang0) / GRADI;
      if (e.shiftKey) d = Math.round(d / 15) * 15;
      mm.forEach(i => Stanza.ruota(App.st.oggetti[i], d, t.centro[0], t.centro[1]));
    } else {
      const o = App.st.oggetti[mm[0]];
      if (m.t === "misura") {
        const [u, v] = Stanza.locale(o, x, y);
        o.larghezza = Math.max(0.02, cm(2 * Math.abs(u))); o.profondita = Math.max(0.02, cm(2 * Math.abs(v)));
      } else if (m.t === "raggio") o.raggio = Math.max(0.01, cm(Math.hypot(x - o.x, y - o.y)));
      else if (m.t === "capo") o[m.k] = [cm(x), cm(y)];
      else if (m.t === "vertice") o.punti[m.k] = [cm(x), cm(y)];
    }
    aggiornaCampi();
  });
  function angoloMouse(cx, cy, x, y, scatto) {
    let a = Math.atan2(y - cy, x - cx) / GRADI;
    if (scatto) a = Math.round(a / 15) * 15;
    return Stanza.norm(a);
  }

  window.addEventListener("mouseup", async () => {
    panoramica = null;
    if (disegno) { creaDaDisegno(); disegno = null; return; }
    if (!trascina) return;
    const t = trascina; trascina = null;
    if (App.sel && App.sel.k === "robot") {
      const r = App.anteprimaRobot;
      App.anteprimaRobot = null;
      if (r && (r.x !== t.robot0.x || r.y !== t.robot0.y || r.angolo !== t.robot0.angolo))
        await comando(`sposta ${r.x} ${r.y} ${r.angolo}`);
      aggiornaPannello();
      return;
    }
    registraModifica(prima);
  });

  function creaDaDisegno() {
    const [x0, y0] = disegno.da, [x1, y1] = disegno.a, w = Math.abs(x1 - x0), h = Math.abs(y1 - y0);
    const s = App.strumento;
    if ((s === "muro" && Math.hypot(w, h) < 0.05) || (s !== "muro" && s !== "cerchio" && (w < 0.02 || h < 0.02))) return;
    modifica(() => {
      const n = App.st.oggetti.length;
      if (s === "muro") App.st.oggetti.push({ tipo: "muro", nome: "muro", gruppo: "", da: [x0, y0], a: [x1, y1], spessore: 0.1, quota_da: 0, quota_a: 2.5 });
      else if (s === "rettangolo") App.st.oggetti.push({ tipo: "rettangolo", nome: "mobile", gruppo: "", x: cm((x0 + x1) / 2), y: cm((y0 + y1) / 2), larghezza: w, profondita: h, angolo: 0, quota_da: 0, quota_a: 0.5 });
      else if (s === "cerchio") App.st.oggetti.push({ tipo: "cerchio", nome: "cerchio", gruppo: "", x: x0, y: y0, raggio: Math.max(0.01, cm(Math.hypot(w, h))), quota_da: 0, quota_a: 0.5 });
      else if (s === "vuoto") App.st.oggetti.push({ tipo: "vuoto", nome: "vuoto", gruppo: "", quota_da: 0, quota_a: 0,
                                                   punti: [[Math.min(x0, x1), Math.min(y0, y1)], [Math.max(x0, x1), Math.min(y0, y1)], [Math.max(x0, x1), Math.max(y0, y1)], [Math.min(x0, x1), Math.max(y0, y1)]] });
      App.sel = { k: "o", i: n };
    });
    scegliStrumento("seleziona");
  }

  // ------------------------------------------------------------------------------------------- tastiera ---
  window.addEventListener("keydown", e => {
    if (e.target.tagName === "INPUT" || e.target.tagName === "SELECT") return;
    if (App.scheda === "messaggi" && (e.key === "ArrowLeft" || e.key === "ArrowRight")) return;   // li usa l'ispettore
    const ctrl = e.ctrlKey || e.metaKey;
    if (ctrl && e.key.toLowerCase() === "z") { e.preventDefault(); e.shiftKey ? ripeti() : annulla(); return; }
    if (ctrl && e.key.toLowerCase() === "y") { e.preventDefault(); ripeti(); return; }
    if (ctrl && e.key.toLowerCase() === "d") { e.preventDefault(); duplica(); return; }
    if (e.key === "Escape") { App.sel = null; scegliStrumento("seleziona"); aggiornaPannello(); return; }
    if (e.key === "v" || e.key === "V") { scegliStrumento("seleziona"); return; }
    if (!App.sel) return;
    if (e.key === "Delete" || e.key === "Backspace") { e.preventDefault(); elimina(); return; }
    if (e.key === "r" || e.key === "R") { e.preventDefault(); ruotaSel(e.shiftKey ? -15 : 15); return; }
    const passo = e.shiftKey ? 0.1 : 0.01;
    const freccia = { ArrowLeft: [-passo, 0], ArrowRight: [passo, 0], ArrowUp: [0, passo], ArrowDown: [0, -passo] }[e.key];
    if (freccia) { e.preventDefault(); spostaSel(...freccia); }
  });

  async function spostaSel(dx, dy) {
    if (App.sel.k === "robot") { const r = posaRobot(); await comando(`sposta ${cm(r.x + dx)} ${cm(r.y + dy)} ${r.angolo}`); return; }
    modifica(() => {
      if (App.sel.k === "base") { App.st.base.x = cm(App.st.base.x + dx); App.st.base.y = cm(App.st.base.y + dy); }
      else membri().forEach(i => Stanza.sposta(App.st.oggetti[i], dx, dy));
    });
  }
  async function ruotaSel(g) {
    if (App.sel.k === "robot") { const r = posaRobot(); await comando(`sposta ${r.x} ${r.y} ${Stanza.norm(r.angolo + g)}`); return; }
    modifica(() => {
      if (App.sel.k === "base") App.st.base.angolo = Stanza.norm(App.st.base.angolo + g);
      else { const [cx, cy] = centroSel(); membri().forEach(i => Stanza.ruota(App.st.oggetti[i], g, cx, cy)); }
    });
  }
  function elimina() {
    if (!App.sel || App.sel.k === "robot") return;
    modifica(() => {
      if (App.sel.k === "base") App.st.base = null;
      else { const m = new Set(membri()); App.st.oggetti = App.st.oggetti.filter((_, i) => !m.has(i)); }
      App.sel = null;
    });
  }
  function duplica() {
    if (!App.sel || App.sel.k !== "o") return;
    modifica(() => {
      const mm = membri(), n = App.st.oggetti.length, g = App.st.oggetti[mm[0]].gruppo ? nuovoGruppo(App.st.oggetti[mm[0]].gruppo.replace(/\d+$/, "")) : "";
      for (const i of mm) {
        const c = JSON.parse(JSON.stringify(App.st.oggetti[i]));
        if (g) c.gruppo = g;
        Stanza.sposta(c, 0.1, -0.1);
        App.st.oggetti.push(c);
      }
      App.sel = { k: "o", i: n };
    });
  }

  // --------------------------------------------------------------------------------------- pannello ---
  const CAMPI = {
    rettangolo: [["nome", "nome", "t"], ["gruppo", "gruppo", "t"], ["x", "x (m)"], ["y", "y (m)"], ["angolo", "angolo (°)"],
                 ["larghezza", "larghezza (m)"], ["profondita", "profondità (m)"], ["quota_da", "da altezza (m)"], ["quota_a", "a altezza (m)"]],
    cerchio: [["nome", "nome", "t"], ["gruppo", "gruppo", "t"], ["x", "x (m)"], ["y", "y (m)"], ["raggio", "raggio (m)"],
              ["quota_da", "da altezza (m)"], ["quota_a", "a altezza (m)"]],
    muro: [["nome", "nome", "t"], ["gruppo", "gruppo", "t"], ["da0", "da x (m)"], ["da1", "da y (m)"], ["a0", "a x (m)"], ["a1", "a y (m)"],
           ["spessore", "spessore (m)"], ["quota_da", "da altezza (m)"], ["quota_a", "a altezza (m)"]],
    vuoto: [["nome", "nome", "t"], ["gruppo", "gruppo", "t"]],
  };
  function valore(o, k) { return k === "da0" ? o.da[0] : k === "da1" ? o.da[1] : k === "a0" ? o.a[0] : k === "a1" ? o.a[1] : o[k]; }
  function imposta(o, k, v) { if (k === "da0") o.da[0] = v; else if (k === "da1") o.da[1] = v; else if (k === "a0") o.a[0] = v; else if (k === "a1") o.a[1] = v; else o[k] = v; }

  function aggiornaPannello() {
    const p = document.getElementById("proprieta");
    if (!App.st) return;
    const riga = (k, et, v, tipo) => `<tr><td>${et}</td><td><input data-k="${k}" type="${tipo === "t" ? "text" : "number"}" step="0.01" value="${tipo === "t" ? (v || "") : (+v).toFixed(tipo === "a" ? 1 : 3)}"></td></tr>`;
    if (!App.sel) {
      p.innerHTML = `<p class="tenue">Niente selezionato: proprietà della stanza (fuori dal rettangolo vale "muro").</p><table>` +
        riga("s_nome", "nome", App.st.nome, "t") + riga("s_larghezza", "larghezza (m)", App.st.larghezza) + riga("s_profondita", "profondità (m)", App.st.profondita) + "</table>";
    } else if (App.sel.k === "robot") {
      const r = posaRobot();
      p.innerHTML = `<p><b>Robot</b> (posizione attuale nel simulatore)</p><table>` + riga("r_x", "x (m)", r.x) + riga("r_y", "y (m)", r.y) + riga("r_angolo", "direzione (°)", r.angolo, "a") +
        `</table><div class="riga"><button id="p-inizio">Usa come posizione iniziale</button></div>` +
        `<p class="tenue">Iniziale salvata: x ${App.st.robot.x}, y ${App.st.robot.y}, ${App.st.robot.angolo}°.</p>`;
      document.getElementById("p-inizio").onclick = () => modifica(() => { const r = posaRobot(); App.st.robot = { x: cm(r.x), y: cm(r.y), angolo: Math.round(r.angolo) }; });
    } else if (App.sel.k === "base") {
      const b = App.st.base;
      p.innerHTML = `<p><b>Base di ricarica</b>: x, y = centro del lato con i contatti; il robot entra all'indietro (contatti dietro).</p><table>` +
        riga("b_x", "x (m)", b.x) + riga("b_y", "y (m)", b.y) + riga("b_angolo", "verso dove guarda (°)", b.angolo, "a") + `</table>`;
    } else {
      const mm = membri(), o = App.st.oggetti[App.sel.i];
      const cat = Stanza.categoria(o);
      p.innerHTML = `<p><b>${o.tipo}</b>${o.gruppo ? ` nel gruppo <b>${o.gruppo}</b> (${mm.length} oggetti: si spostano insieme)` : ""}<br>` +
        `<span class="tenue">${Stanza.DESCR_CATEGORIA[cat]}</span></p><table>` +
        CAMPI[o.tipo].map(([k, et, t]) => riga(k, et, valore(o, k), t || (k === "angolo" ? "a" : ""))).join("") + `</table>` +
        (o.tipo === "vuoto" ? `<p class="tenue">Trascina i vertici per cambiare forma.</p>` : "") +
        (o.gruppo ? `<div class="riga"><button id="p-separa">Togli dal gruppo</button></div>` : "");
      const s = document.getElementById("p-separa");
      if (s) s.onclick = () => modifica(() => { o.gruppo = ""; });
    }
    p.querySelectorAll("input").forEach(inp => inp.addEventListener("change", () => applicaCampo(inp)));
    document.getElementById("annulla").disabled = !App.undo.length;
    document.getElementById("ripeti").disabled = !App.redo.length;
  }
  function aggiornaCampi() {           // durante il trascinamento aggiorna solo i numeri
    if (!App.sel || App.sel.k !== "o") return;
    const o = App.st.oggetti[App.sel.i];
    document.querySelectorAll("#proprieta input[type=number]").forEach(inp => {
      const v = valore(o, inp.dataset.k);
      if (v !== undefined) inp.value = (+v).toFixed(inp.dataset.k === "angolo" ? 1 : 3);
    });
  }
  async function applicaCampo(inp) {
    const k = inp.dataset.k, v = inp.type === "number" ? +inp.value : inp.value.trim().replace(/\s+/g, "_");
    if (inp.type === "number" && !isFinite(v)) return;
    if (k.startsWith("r_")) {
      const r = posaRobot(), n = { x: r.x, y: r.y, angolo: r.angolo };
      n[k.slice(2)] = v;
      await comando(`sposta ${n.x} ${n.y} ${n.angolo}`);
      return;
    }
    modifica(() => {
      if (k.startsWith("s_")) App.st[k.slice(2)] = v;
      else if (k.startsWith("b_")) App.st.base[k.slice(2)] = v;
      else {
        const o = App.st.oggetti[App.sel.i];
        if (k === "x" || k === "y") {                 // spostare un oggetto di un gruppo sposta il gruppo
          const d = v - o[k];
          membri().forEach(i => Stanza.sposta(App.st.oggetti[i], k === "x" ? d : 0, k === "y" ? d : 0));
        } else imposta(o, k, v);
      }
    });
  }

  // ------------------------------------------------------------------------------------------- disegno ---
  function adatta() {
    if (!App.st) return;
    const W = tela.clientWidth, H = tela.clientHeight, lw = App.st.larghezza, lh = App.st.profondita;
    V.s = Math.min(W / lw, (H - 60) / lh) * 0.92;
    V.ox = (W - lw * V.s) / 2; V.oy = H - (H - lh * V.s) / 2 + 10; V.adattata = true;
  }
  function percorso(o) {
    ctx.beginPath();
    if (o.tipo === "cerchio") { const [x, y] = aSchermo(o.x, o.y); ctx.arc(x, y, o.raggio * V.s, 0, 7); return; }
    const p = puntiContorno(o).map(q => aSchermo(q[0], q[1]));
    ctx.moveTo(...p[0]); for (const q of p.slice(1)) ctx.lineTo(...q); ctx.closePath();
  }
  const LED = { 0: null, 1: "#4fc3f7", 2: "#4fc3f7", 3: "#fdd835", 4: ["#fdd835", "r"], 5: null, 6: ["#fdd835", "l"], 7: "#e53935",
                8: ["#e53935", "v"], 9: null, 10: ["#ff8f00", "l"], 11: ["#4fc3f7", "l"], 12: ["#4fc3f7", "r"], 13: "#4fc3f7",
                14: "#4fc3f7", 15: "#4fc3f7", 16: ["#e53935", "l"], 17: "#e57373", 18: "#ff8f00" };
  function coloreLed(n) {
    let l = LED[n]; if (!l) return null;
    if (typeof l === "string") return l;
    const t = performance.now() / 1000;
    if (l[1] === "l" && t % 2 > 1) return null;
    if (l[1] === "v" && t % 0.4 > 0.2) return null;
    if (l[1] === "r") return l[0] + Math.round(80 + 175 * (0.5 + 0.5 * Math.sin(t * 3))).toString(16).padStart(2, "0");
    return l[0];
  }
  App.coloreLed = coloreLed;

  function disegna() {
    const dpr = window.devicePixelRatio || 1, W = tela.clientWidth, H = tela.clientHeight;
    if (tela.width !== Math.round(W * dpr) || tela.height !== Math.round(H * dpr)) { tela.width = Math.round(W * dpr); tela.height = Math.round(H * dpr); }
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    for (const n of ["sfondo", "pavimento", "griglia", "muro", "alto", "basso", "torretta", "sopra", "vuoto", "base", "robot", "robot-bordo", "lidar", "sel", "errore", "ok", "tenue", "accento"])
      col[n] = colore("--" + n);
    ctx.fillStyle = col.sfondo; ctx.fillRect(0, 0, W, H);
    if (!App.st) return requestAnimationFrame(disegna);
    if (!V.adattata) adatta();
    const st = App.st;
    // pavimento e griglia
    const [x0, y0] = aSchermo(0, st.profondita);
    ctx.fillStyle = col.pavimento; ctx.fillRect(x0, y0, st.larghezza * V.s, st.profondita * V.s);
    const passo = V.s > 250 ? 0.1 : 1;
    ctx.strokeStyle = col.griglia; ctx.lineWidth = 1; ctx.beginPath();
    for (let x = 0; x <= st.larghezza + 1e-9; x += passo) { const [a, b] = aSchermo(x, 0), [, d] = aSchermo(x, st.profondita); ctx.moveTo(a, b); ctx.lineTo(a, d); }
    for (let y = 0; y <= st.profondita + 1e-9; y += passo) { const [a, b] = aSchermo(0, y), [c] = aSchermo(st.larghezza, y); ctx.moveTo(a, b); ctx.lineTo(c, b); }
    ctx.stroke();
    // oggetti: prima i vuoti, poi dal più basso al più alto
    const ordine = st.oggetti.map((o, i) => i).sort((a, b) => (st.oggetti[a].tipo === "vuoto" ? -1 : st.oggetti[a].quota_a) - (st.oggetti[b].tipo === "vuoto" ? -1 : st.oggetti[b].quota_a));
    const selezionati = new Set(membri());
    for (const i of ordine) {
      const o = st.oggetti[i], cat = Stanza.categoria(o);
      percorso(o);
      ctx.setLineDash([]);
      if (cat === "sopra") { ctx.fillStyle = col.sopra + "33"; ctx.fill(); ctx.setLineDash([5, 4]); ctx.strokeStyle = col.sopra; ctx.lineWidth = 1.5; ctx.stroke(); }
      else if (cat === "vuoto") { ctx.fillStyle = col.vuoto + "aa"; ctx.fill(); }
      else if (cat === "torretta") { ctx.fillStyle = col.torretta + "88"; ctx.fill(); ctx.strokeStyle = col.torretta; ctx.lineWidth = 1.5; ctx.stroke(); }
      else { ctx.fillStyle = col[cat]; ctx.fill(); }
      if (selezionati.has(i)) { ctx.setLineDash([]); ctx.strokeStyle = col.sel; ctx.lineWidth = 2.5; ctx.stroke(); }
    }
    ctx.setLineDash([]);
    // base
    if (st.base) {
      const b = st.base, [bx, by] = aSchermo(b.x, b.y);
      ctx.save(); ctx.translate(bx, by); ctx.rotate(-b.angolo * GRADI);
      ctx.fillStyle = col.base; ctx.fillRect(-0.12 * V.s, -0.15 * V.s, 0.12 * V.s, 0.30 * V.s);
      ctx.fillStyle = "#ffd54f"; ctx.fillRect(-0.012 * V.s, -0.05 * V.s, 0.012 * V.s, 0.10 * V.s);
      if (App.sel && App.sel.k === "base") { ctx.strokeStyle = col.sel; ctx.lineWidth = 2.5; ctx.strokeRect(-0.12 * V.s, -0.15 * V.s, 0.12 * V.s, 0.30 * V.s); }
      ctx.restore();
    }
    // posizione iniziale
    { const r = st.robot, [ix, iy] = aSchermo(r.x, r.y), R = (App.stato ? App.stato.geometria.raggio : 0.175) * V.s;
      ctx.strokeStyle = col.tenue; ctx.setLineDash([4, 4]); ctx.lineWidth = 1; ctx.beginPath(); ctx.arc(ix, iy, R, 0, 7); ctx.stroke();
      ctx.beginPath(); ctx.moveTo(ix, iy); ctx.lineTo(ix + Math.cos(-r.angolo * GRADI) * R, iy + Math.sin(-r.angolo * GRADI) * R); ctx.stroke(); ctx.setLineDash([]); }
    // lidar e robot
    const s = App.stato;
    if (s) {
      ctx.fillStyle = col.lidar; const p = s.lidar.punti, r = Math.max(1.5, V.s * 0.01);
      for (let i = 0; i + 1 < p.length; i += 2) { const [a, b] = aSchermo(p[i], p[i + 1]); ctx.fillRect(a - r / 2, b - r / 2, r, r); }
      disegnaRobot(s);
    }
    // anteprima del disegno
    if (disegno) {
      ctx.strokeStyle = col.accento; ctx.lineWidth = 2; ctx.setLineDash([6, 4]);
      const [a, b] = aSchermo(...disegno.da), [c, d] = aSchermo(...disegno.a);
      ctx.beginPath();
      if (App.strumento === "muro") { ctx.moveTo(a, b); ctx.lineTo(c, d); }
      else if (App.strumento === "cerchio") ctx.arc(a, b, Math.hypot(c - a, d - b), 0, 7);
      else ctx.rect(Math.min(a, c), Math.min(b, d), Math.abs(c - a), Math.abs(d - b));
      ctx.stroke(); ctx.setLineDash([]);
    }
    // maniglie
    for (const m of maniglie()) {
      const [x, y] = aSchermo(m.x, m.y);
      ctx.fillStyle = col.pavimento; ctx.strokeStyle = col.sel; ctx.lineWidth = 2;
      ctx.beginPath();
      if (m.t === "ruota") {
        const [cx, cy] = aSchermo(...centroSel());
        ctx.save(); ctx.setLineDash([3, 3]); ctx.lineWidth = 1; ctx.moveTo(cx, cy); ctx.lineTo(x, y); ctx.stroke(); ctx.restore();
        ctx.beginPath(); ctx.arc(x, y, 7, 0, 7);
      } else ctx.rect(x - 5, y - 5, 10, 10);
      ctx.fill(); ctx.stroke();
    }
    requestAnimationFrame(disegna);
  }

  function disegnaRobot(s) {
    const r = posaRobot(), G = s.geometria, [rx, ry] = aSchermo(r.x, r.y), R = G.raggio * V.s;
    ctx.save(); ctx.translate(rx, ry); ctx.rotate(-r.angolo * GRADI);
    if (s.sollevato) { ctx.shadowColor = "rgba(0,0,0,.45)"; ctx.shadowBlur = 18; ctx.shadowOffsetY = 10; }
    ctx.fillStyle = col.robot; ctx.strokeStyle = col["robot-bordo"]; ctx.lineWidth = 1.5;
    ctx.beginPath(); ctx.arc(0, 0, R, 0, 7); ctx.fill(); ctx.stroke(); ctx.shadowColor = "transparent";
    // bumper: arco anteriore sinistro (y negativo sullo schermo = sinistra del robot) e destro
    const arco = G.bumper_arco * GRADI;
    ctx.lineWidth = Math.max(3, R * 0.1);
    ctx.strokeStyle = s.bumper[0] ? col.errore : col.tenue; ctx.beginPath(); ctx.arc(0, 0, R * 1.04, -arco, 0); ctx.stroke();
    ctx.strokeStyle = s.bumper[1] ? col.errore : col.tenue; ctx.beginPath(); ctx.arc(0, 0, R * 1.04, 0, arco); ctx.stroke();
    // contatti di carica (dietro)
    ctx.fillStyle = "#ffd54f"; ctx.fillRect(-R - 2, -R * 0.25, 4, R * 0.5);
    // anti-caduta
    G.cliff_angoli.forEach((a, k) => {
      const v = (s.cliff >> k) & 1, t = -a * GRADI, d = G.cliff_distanza * V.s;
      ctx.fillStyle = v ? col.vuoto : col.ok; ctx.beginPath(); ctx.arc(Math.cos(t) * d, Math.sin(t) * d, Math.max(2.5, R * 0.08), 0, 7); ctx.fill();
    });
    // torretta del lidar e direzione
    ctx.strokeStyle = s.incastrato ? col.torretta : col["robot-bordo"]; ctx.lineWidth = s.incastrato ? 3 : 1;
    ctx.beginPath(); ctx.arc(G.lidar[0] * V.s, -G.lidar[1] * V.s, G.torretta * V.s, 0, 7); ctx.stroke();
    ctx.strokeStyle = col["robot-bordo"]; ctx.lineWidth = 2; ctx.beginPath(); ctx.moveTo(0, 0); ctx.lineTo(R * 0.65, 0); ctx.stroke();
    const cl = coloreLed(s.led);
    ctx.fillStyle = cl || col.griglia; ctx.beginPath(); ctx.arc(R * 0.45, 0, Math.max(3, R * 0.1), 0, 7); ctx.fill();
    if (App.sel && App.sel.k === "robot") { ctx.strokeStyle = col.sel; ctx.lineWidth = 2.5; ctx.beginPath(); ctx.arc(0, 0, R * 1.12, 0, 7); ctx.stroke(); }
    if (s.caduto) { ctx.strokeStyle = col.errore; ctx.lineWidth = 4; ctx.beginPath(); ctx.moveTo(-R, -R); ctx.lineTo(R, R); ctx.moveTo(R, -R); ctx.lineTo(-R, R); ctx.stroke(); }
    ctx.restore();
  }

  // ------------------------------------------------------------------------------------------- comandi ---
  function scegliStrumento(s) {
    App.strumento = s;
    document.querySelectorAll("#barra button").forEach(b => b.classList.toggle("attivo", b.dataset.s === s));
    tela.style.cursor = s === "seleziona" ? "default" : "crosshair";
  }
  document.querySelectorAll("#barra button").forEach(b => b.onclick = () => scegliStrumento(b.dataset.s));
  document.getElementById("libreria").onchange = e => { App.libreria = e.target.value; if (App.libreria) { scegliStrumento("seleziona"); avviso("clic sulla stanza per mettere: " + e.target.selectedOptions[0].text); } };
  document.getElementById("annulla").onclick = annulla;
  document.getElementById("ripeti").onclick = ripeti;
  document.getElementById("duplica").onclick = duplica;
  document.getElementById("elimina").onclick = elimina;
  document.getElementById("riposiziona").onclick = () => comando(`sposta ${App.st.robot.x} ${App.st.robot.y} ${App.st.robot.angolo}`);
  document.getElementById("nuova").onclick = async () => {
    const d = prompt("Dimensioni della stanza in metri, larghezza x profondità:", "5 x 4"); if (!d) return;
    const m = d.match(/([\d.,]+)\D+([\d.,]+)/); if (!m) return;
    const p = istantanea();
    App.st = Stanza.nuova(+m[1].replace(",", "."), +m[2].replace(",", "."));
    App.sel = null; App.undo.push(p); App.redo = [];
    await invia(true); V.adattata = false; aggiornaPannello();
  };
  document.getElementById("salva").onclick = async () => {
    const n = prompt("Nome della stanza (lettere, cifre, - e _):", App.st.nome); if (!n) return;
    App.st.nome = n; await invia(true);
    const t = await comando("salva " + n); if (t) { avviso(t, "ok"); elencoStanze(); }
  };
  document.getElementById("carica").onclick = async () => {
    const n = document.getElementById("stanze").value; if (!n) return;
    if (await comando("carica " + n)) { await ricarica(); V.adattata = false; }
  };
  async function elencoStanze() {
    const l = await (await fetch("/stanze")).json(), s = document.getElementById("stanze");
    s.innerHTML = l.sort().map(n => `<option>${n}</option>`).join("");
  }
  function legenda() {
    const v = [["muro", "muro"], ["alto", "mobile dal pavimento (lidar + bumper)"], ["basso", "basso (solo bumper)"],
               ["torretta", "rialzato 7,5–9,5 cm (si incastra)"], ["sopra", "ci passa sotto/sopra"], ["vuoto", "vuoto"], ["lidar", "punti del lidar"]];
    document.getElementById("legenda").innerHTML = v.map(([c, t]) => `<span class="c" style="background:var(--${c})"></span>${t}`).join(" &nbsp; ");
  }

  async function avvia() {
    legenda(); elencoStanze(); await ricarica(); requestAnimationFrame(disegna);
    window.addEventListener("resize", () => { V.adattata = false; });
  }
  // aSchermo e maniglie servono anche alla prova automatica della pagina (tools/prova-pagina.mjs)
  return { avvia, ricarica, aggiornaPannello, comando, avviso, adatta: () => { V.adattata = false; }, aSchermo, maniglie, centroSel };
})();
