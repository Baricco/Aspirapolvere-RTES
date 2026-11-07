// visore.js - Visualizzatore delle missioni registrate sul robot (strumento di debug, vedi visore.py).
// NOME.map (tools/osserva-robot.c): record da double t, uint32 versione, uint32 asse delle strisce (1 = x), griglia di
// navigazione 256 x 256 (byte: 4 bit bassi stato, 4 alti sottocelle bloccate), maschera delle metà pulite (2 bit per cella).
// NOME.oss: righe "P t x y theta_gradi stato missione" ogni 200 ms. NOME.out: registro del programma ("driver  t  testo").
"use strict";

const N = 256, CELLA = 0.2, REC = 8 + 4 + 4 + N * N + N * N / 4;
// stati della navigazione, nell'ordine dell'enum di tasks/vacuum/navigation.c
const STATI = ["attesa", "esplorazione", "pulizia (strisce)", "giro dei bordi", "rientro", "aggancio", "segue percorso",
               "sblocco", "riposo", "riallineamento", "guasto", "giro di un bordo", "uscita dalla base", "attraversamento", "pausa"];
const COLORE_MISSIONE = { 1: "#1f77b4", 2: "#2ca02c", 3: "#ff7f0e", 4: "#9467bd", 8: "#7f7f7f", 0: "#9ca3af" };
const COLORI = { 1: [42, 98, 184, 70], 2: [214, 39, 40, 150], 3: [142, 36, 170, 150], 4: [230, 140, 20, 170] };
const VERDE = [30, 170, 70, 190];
const NOMI_CELLE = ["sconosciuta", "libera", "bloccata", "proibita", "troppo stretta"];
// zona proibita della base (tasks/vacuum/mapping.c, con l'origine nel robot agganciato) e punto di aggancio
const BASE = { x0: -0.365, x1: -0.105, y0: -0.22, y1: 0.22 }, AGGANCIO_X = 0.30, RAGGIO = 0.175;

const tela = document.getElementById("tela"), ctx = tela.getContext("2d");
const tempo = document.getElementById("tempo"), ora = document.getElementById("ora");
const barraEv = document.getElementById("eventi-barra"), ctxEv = barraEv.getContext("2d");
const diretta = document.getElementById("diretta");
let M = null;                       // missione caricata: { mappe: [{t, versione, asse, griglia, maschera}], pose: [...], eventi, messaggi, tmax }
const cacheImmagini = new Map();
let vista = { s: 60, ox: 0, oy: 0 }, t = 0, inMoto = false, ultimoFrame = 0, mouse = null;

// ------------------------------------------------------------------------------------------- caricamento ---
async function caricaElenco() {
  const l = await (await fetch("/missioni")).json(), sel = document.getElementById("missione");
  sel.innerHTML = l.map(n => `<option>${n}</option>`).join("");
  const voluta = new URLSearchParams(location.search).get("m");
  if (voluta && l.includes(voluta)) sel.value = voluta;
  sel.onchange = () => carica(sel.value);
  if (sel.value) carica(sel.value);
}

async function carica(nome) {
  M = { nome, mappe: [], pose: [], eventi: [], messaggi: [], tmax: 1, off: { map: 0, oss: 0, out: 0 }, ultima: null };
  cacheImmagini.clear();
  await aggiungi();
  // istante di partenza: ?t= nell'indirizzo, altrimenti la prima posizione registrata (in diretta: l'ultima)
  const tChiesto = new URLSearchParams(location.search).get("t");
  t = diretta.checked ? M.tmax : tChiesto !== null ? Math.min(+tChiesto, M.tmax) : (M.pose.length ? M.pose[0].t : 0);
  tempo.value = t;
  centra();
  disegnaEventi(); aggiorna();
}

// i byte nuovi di un file della missione, dal punto in cui si era arrivati (l'ultima griglia sempre intera)
async function prendi(tipo) {
  const da = tipo === "ultima" ? 0 : M.off[tipo];
  return new Uint8Array(await (await fetch(`/file?nome=${M.nome}&tipo=${tipo}&da=${da}`)).arrayBuffer());
}

// solo le righe intere (l'ultima riga può essere ancora a metà mentre la missione gira)
const decodifica = new TextDecoder();
function righeNuove(tipo, b) {
  const fine = b.lastIndexOf(10);
  if (fine < 0) return [];
  M.off[tipo] += fine + 1;
  return decodifica.decode(b.subarray(0, fine)).split("\n");
}

function leggiRecord(b, o) {
  const dv = new DataView(b.buffer, b.byteOffset + o, REC);
  return { t: dv.getFloat64(0, true), versione: dv.getUint32(8, true), asse: dv.getUint32(12, true),
           griglia: b.subarray(o + 16, o + 16 + N * N), maschera: b.subarray(o + 16 + N * N, o + REC) };
}

// aggiunge alla missione i dati nuovi dei file (al primo caricamento: tutto)
async function aggiungi() {
  const [bm, bo, bu, bl] = await Promise.all(["map", "oss", "out", "ultima"].map(prendi));
  const nrec = Math.floor(bm.length / REC);              // solo i record interi
  for (let k = 0; k < nrec; k++) M.mappe.push(leggiRecord(bm, k * REC));
  M.off.map += nrec * REC;
  for (const r of righeNuove("oss", bo)) {
    const p = r.trim().split(/\s+/);
    if (p[0] !== "P" || p.length < 7) continue;
    const q = { t: +p[1], x: +p[2], y: +p[3], th: +p[4], st: +p[5], mi: +p[6] }, prec = M.pose[M.pose.length - 1];
    q.strada = prec ? prec.strada + Math.hypot(q.x - prec.x, q.y - prec.y) : 0;   // strada percorsa cumulata
    M.pose.push(q);
  }
  for (const r of righeNuove("out", bu)) {
    const m = r.match(/^driver\s+([\d.]+)\s+(.*)$/);
    if (m) M.eventi.push({ t: +m[1], testo: m[2], tipo: tipoEvento(m[2]) });
    else if (/^(mappatura|aperiodico|comandi|navigazione):/.test(r)) M.messaggi.push(r);
  }
  if (bl.length >= REC) M.ultima = leggiRecord(bl, 0);
  M.tmax = Math.max(M.pose.length ? M.pose[M.pose.length - 1].t : 0, M.mappe.length ? M.mappe[M.mappe.length - 1].t : 0,
                    M.ultima ? M.ultima.t : 0, 1);
  tempo.max = M.tmax;
  document.getElementById("messaggi").textContent = M.messaggi.join("\n") || "—";
}

// la griglia da mostrare all'istante tt: l'istantanea più recente, o l'ultima griglia della diretta se è più nuova
function mappaA(tt) {
  const k = indiceFinoA(M.mappe, tt);
  if (M.ultima && M.ultima.t <= tt && (k < 0 || M.ultima.t > M.mappe[k].t)) return { m: M.ultima, chiave: "u" + M.ultima.t };
  return k >= 0 ? { m: M.mappe[k], chiave: k } : null;
}

function tipoEvento(s) {
  if (/GUASTO|SCADENZA/.test(s)) return "guasto";
  if (/aggancio: esito [23]/.test(s)) return "aggancio";
  if (/batteria/.test(s)) return "batteria";
  if (/PROVA/.test(s)) return "prova";
  if (/aggancio/.test(s)) return "aggancio-fase";
  return "altro";
}
const COLORE_EVENTO = { guasto: "#dc2626", aggancio: "#2563eb", batteria: "#d97706", prova: "#6b7280", "aggancio-fase": "#93c5fd", altro: "#cbd5e1" };

// ------------------------------------------------------------------------------------------- ricerca nel tempo ---
function indiceFinoA(arr, tt) {      // ultimo elemento con t <= tt (-1 se nessuno)
  let a = 0, b = arr.length - 1, r = -1;
  while (a <= b) { const m = (a + b) >> 1; if (arr[m].t <= tt) { r = m; a = m + 1; } else b = m - 1; }
  return r;
}

// ------------------------------------------------------------------------------------------- griglia ---
function immagine(m, k) {
  if (cacheImmagini.has(k)) return cacheImmagini.get(k);
  // dell'ultima griglia della diretta se ne tiene una sola
  if (typeof k === "string") for (const kk of [...cacheImmagini.keys()]) if (typeof kk === "string") cacheImmagini.delete(kk);
  const c = document.createElement("canvas");
  c.width = c.height = 2 * N;
  const c2 = c.getContext("2d"), img = c2.createImageData(2 * N, 2 * N), px = img.data;
  const pulita = (i, meta) => { const n = 2 * i + meta; return (m.maschera[n >> 3] >> (n & 7) & 1) === 1; };
  for (let i = 0; i < N * N; i++) {
    const v = m.griglia[i] & 15, bloccate = m.griglia[i] >> 4, x = i % N, y = (i / N) | 0;
    for (let k4 = 0; k4 < 4; k4++) {          // k4 = bit - 4: 0 alto-sx, 1 alto-dx, 2 basso-sx, 3 basso-dx (alto = y più alta)
      const p = ((2 * y + (k4 < 2 ? 1 : 0)) * 2 * N + 2 * x + (k4 & 1)) * 4;
      const meta = m.asse ? (k4 < 2 ? 1 : 0) : (k4 & 1);
      const col = v === 1 && (bloccate >> k4 & 1) ? COLORI[2] : v === 1 && pulita(i, meta) ? VERDE : COLORI[v];
      if (col) { px[p] = col[0]; px[p + 1] = col[1]; px[p + 2] = col[2]; px[p + 3] = col[3]; }
    }
  }
  c2.putImageData(img, 0, 0);
  cacheImmagini.set(k, c);
  return c;
}

function contaMappa(m) {
  const r = { libere: 0, parziali: 0, bloccate: 0, proibite: 0, strette: 0, pulite: 0, percorribili: 0 };
  for (let i = 0; i < N * N; i++) {
    const v = m.griglia[i] & 15;
    if (v === 1) { r.libere++; if (m.griglia[i] >> 4) r.parziali++; else r.percorribili += 2; }
    else if (v === 2) r.bloccate++; else if (v === 3) r.proibite++; else if (v === 4) r.strette++;
  }
  for (let j = 0; j < m.maschera.length; j++) { let b = m.maschera[j]; while (b) { r.pulite += b & 1; b >>= 1; } }
  return r;
}

// ------------------------------------------------------------------------------------------- disegno ---
function adatta() {
  const r = tela.getBoundingClientRect(), d = devicePixelRatio || 1;
  tela.width = r.width * d; tela.height = r.height * d;
  ctx.setTransform(d, 0, 0, d, 0, 0);
  const re = barraEv.getBoundingClientRect();
  barraEv.width = re.width * d; barraEv.height = re.height * d;
  ctxEv.setTransform(d, 0, 0, d, 0, 0);
}

function centra() {
  const r = tela.getBoundingClientRect();
  const i = M ? indiceFinoA(M.pose, t) : -1, p = i >= 0 ? M.pose[i] : { x: 0, y: 0 };
  vista.ox = r.width / 2 - p.x * vista.s; vista.oy = r.height / 2 + p.y * vista.s;
}

function disegna() {
  const r = tela.getBoundingClientRect();
  ctx.clearRect(0, 0, r.width, r.height);
  if (!M) return;
  ctx.save();
  ctx.translate(vista.ox, vista.oy);
  ctx.scale(vista.s, -vista.s);                  // metri, y verso l'alto
  // griglia dell'istantanea più recente
  const mp = mappaA(t);
  if (mp) {
    ctx.imageSmoothingEnabled = false;
    ctx.drawImage(immagine(mp.m, mp.chiave), -N / 2 * CELLA, -N / 2 * CELLA, N * CELLA, N * CELLA);
  }
  // reticolo delle celle a zoom alto
  if (vista.s > 90) {
    const x0 = Math.floor((-vista.ox) / vista.s / CELLA) - 1, x1 = Math.ceil((r.width - vista.ox) / vista.s / CELLA) + 1;
    const y0 = Math.floor((vista.oy - r.height) / vista.s / CELLA) - 1, y1 = Math.ceil(vista.oy / vista.s / CELLA) + 1;
    ctx.strokeStyle = "rgba(0,0,0,0.08)"; ctx.lineWidth = 1 / vista.s;
    ctx.beginPath();
    for (let x = x0; x <= x1; x++) { ctx.moveTo(x * CELLA, y0 * CELLA); ctx.lineTo(x * CELLA, y1 * CELLA); }
    for (let y = y0; y <= y1; y++) { ctx.moveTo(x0 * CELLA, y * CELLA); ctx.lineTo(x1 * CELLA, y * CELLA); }
    ctx.stroke();
  }
  // base e punto di aggancio
  ctx.fillStyle = "rgba(20,20,20,0.85)";
  ctx.fillRect(BASE.x0, BASE.y0, BASE.x1 - BASE.x0, BASE.y1 - BASE.y0);
  ctx.strokeStyle = "#111"; ctx.lineWidth = 2 / vista.s;
  ctx.beginPath(); ctx.arc(AGGANCIO_X, 0, 0.03, 0, 2 * Math.PI); ctx.stroke();
  // scia colorata per missione, fino all'istante t
  const ip = indiceFinoA(M.pose, t);
  ctx.lineWidth = 2.5 / vista.s; ctx.lineCap = "round";
  for (let i = 1; i <= ip; i++) {
    const a = M.pose[i - 1], b = M.pose[i];
    if (Math.hypot(b.x - a.x, b.y - a.y) > 0.5) continue;      // salti (posizione ritrovata dopo un sollevamento)
    ctx.strokeStyle = COLORE_MISSIONE[b.mi] || "#9ca3af";
    ctx.globalAlpha = i > ip - 300 ? 0.95 : 0.45;               // l'ultimo minuto più marcato
    ctx.beginPath(); ctx.moveTo(a.x, a.y); ctx.lineTo(b.x, b.y); ctx.stroke();
  }
  ctx.globalAlpha = 1;
  // robot
  if (ip >= 0) {
    const p = M.pose[ip], th = p.th * Math.PI / 180;
    ctx.fillStyle = p.st === 10 ? "rgba(220,38,38,0.55)" : "rgba(255,255,255,0.75)";
    ctx.strokeStyle = "#111"; ctx.lineWidth = 2 / vista.s;
    ctx.beginPath(); ctx.arc(p.x, p.y, RAGGIO, 0, 2 * Math.PI); ctx.fill(); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(p.x, p.y); ctx.lineTo(p.x + RAGGIO * Math.cos(th), p.y + RAGGIO * Math.sin(th)); ctx.stroke();
  }
  ctx.restore();
}

function disegnaEventi() {
  const r = barraEv.getBoundingClientRect();
  ctxEv.clearRect(0, 0, r.width, r.height);
  if (!M) return;
  // fasce delle missioni
  for (let i = 1; i < M.pose.length; i++) {
    const a = M.pose[i - 1], b = M.pose[i];
    ctxEv.fillStyle = COLORE_MISSIONE[b.mi] || "#9ca3af";
    ctxEv.fillRect(a.t / M.tmax * r.width, 0, Math.max(1, (b.t - a.t) / M.tmax * r.width), 6);
  }
  for (const e of M.eventi) {
    ctxEv.fillStyle = COLORE_EVENTO[e.tipo];
    const x = e.t / M.tmax * r.width;
    ctxEv.fillRect(x - (e.tipo === "guasto" ? 1.5 : 0.5), 7, e.tipo === "guasto" ? 3 : 1, 7);
  }
}

// ------------------------------------------------------------------------------------------- pannello ---
const riga = (a, b) => `<tr><td>${a}</td><td>${b}</td></tr>`;
const chip = c => `<span class="c" style="background:${c}"></span>`;
const fmt = (v, d = 1) => v.toFixed(d).replace(".", ",");

function aggiorna() {
  if (!M) return;
  ora.textContent = fmt(t) + " s";
  const ip = indiceFinoA(M.pose, t), p = ip >= 0 ? M.pose[ip] : null;
  document.getElementById("stato-grande").innerHTML = p
    ? `${chip(COLORE_MISSIONE[p.mi] || "#9ca3af")}${STATI[p.mi] || p.mi} → ${STATI[p.st] || p.st}` : "—";
  document.getElementById("adesso").innerHTML = p ? [
    riga("posizione creduta", `(${fmt(p.x, 2)}; ${fmt(p.y, 2)}) m, ${fmt(p.th, 0)}°`),
    riga("distanza dal punto di aggancio", fmt(Math.hypot(p.x - AGGANCIO_X, p.y), 2) + " m"),
    riga("strada percorsa", fmt(p.strada, 1) + " m"),
  ].join("") : "";
  const mp = mappaA(t);
  if (mp) {
    const m = mp.m, c = contaMappa(m);
    document.getElementById("mappa").innerHTML = [
      riga("istantanea", `${fmt(m.t, 0)} s, versione ${m.versione}, strisce lungo ${m.asse ? "x" : "y"}`),
      riga(`${chip(rgba(COLORI[1]))}celle libere`, `${c.libere} (${fmt(c.libere * 0.04)} m²), ${c.parziali} in parte`),
      riga(`${chip(rgba(COLORI[2]))}bloccate`, `${c.bloccate - 1020}`),
      riga(`${chip(rgba(COLORI[3]))}proibite`, c.proibite),
      riga(`${chip(rgba(COLORI[4]))}troppo strette`, c.strette),
      riga(`${chip(rgba(VERDE))}metà pulite`, `${c.pulite} (${fmt(c.pulite * 0.02)} m²)`),
      riga("pulito / metà delle celle tutte libere", c.percorribili ? fmt(100 * c.pulite / c.percorribili, 0) + "%" : "—"),
    ].join("");
  }
  // tempo per missione e ingressi negli stati fino a t
  const perMissione = {}, ingressi = {};
  for (let i = 1; i <= ip; i++) {
    const a = M.pose[i - 1], b = M.pose[i];
    perMissione[a.mi] = (perMissione[a.mi] || 0) + (b.t - a.t);
    if (b.st !== a.st) ingressi[b.st] = (ingressi[b.st] || 0) + 1;
  }
  document.getElementById("missioni").innerHTML = Object.keys(perMissione).sort((a, b) => perMissione[b] - perMissione[a])
    .map(mi => riga(`${chip(COLORE_MISSIONE[mi] || "#9ca3af")}${STATI[mi] || mi}`, fmt(perMissione[mi], 0) + " s")).join("") || riga("—", "");
  document.getElementById("stati").innerHTML = Object.keys(ingressi).sort((a, b) => ingressi[b] - ingressi[a])
    .map(st => riga(STATI[st] || st, ingressi[st])).join("") || riga("—", "");
  document.getElementById("lista-eventi").innerHTML = M.eventi.map((e, i) =>
    `<div data-i="${i}" class="${e.t > t ? "futuro" : ""} ${e.tipo === "guasto" ? "guasto" : ""}" title="${e.testo.replace(/"/g, "&quot;")}">` +
    `${chip(COLORE_EVENTO[e.tipo])}${fmt(e.t)} s &nbsp;${e.testo}</div>`).join("") || "—";
  aggiornaCella();
  disegna();
}

function rgba(c) { return `rgba(${c[0]},${c[1]},${c[2]},${(c[3] / 255).toFixed(2)})`; }

function aggiornaCella() {
  const el = document.getElementById("cella");
  const mp = M ? mappaA(t) : null;
  if (!mouse || !mp) { el.textContent = "—"; return; }
  const x = (mouse.x - vista.ox) / vista.s, y = (vista.oy - mouse.y) / vista.s;
  const cx = Math.floor(x / CELLA) + N / 2, cy = Math.floor(y / CELLA) + N / 2;
  if (cx < 0 || cy < 0 || cx >= N || cy >= N) { el.textContent = "fuori dalla mappa"; return; }
  const i = cy * N + cx, m = mp.m, v = m.griglia[i], q = v >> 4;
  const n0 = 2 * i, pul = h => (m.maschera[(n0 + h) >> 3] >> ((n0 + h) & 7) & 1) ? "pulita" : "sporca";
  const sq = b => (q >> b & 1) ? "X" : ".";
  el.innerHTML = `(${fmt(x, 2)}; ${fmt(y, 2)}) m — cella (${cx}, ${cy}) indice ${i}<br>` +
    `${NOMI_CELLE[v & 15] || v & 15}, quadranti ${sq(0)}${sq(1)}/${sq(2)}${sq(3)} (alto/basso, X = bloccato), metà ${pul(0)} e ${pul(1)}`;
}

// ------------------------------------------------------------------------------------------- comandi ---
tempo.oninput = () => { t = +tempo.value; aggiorna(); };
document.getElementById("play").onclick = () => suonaFerma();
document.getElementById("centra").onclick = () => { centra(); disegna(); };
function suonaFerma() {
  inMoto = !inMoto;
  document.getElementById("play").textContent = inMoto ? "❚❚" : "▶";
  if (inMoto) { if (M && t >= M.tmax) t = 0; ultimoFrame = performance.now(); requestAnimationFrame(passo); }
}
function passo(ora_) {
  if (!inMoto || !M) return;
  t = Math.min(M.tmax, t + (ora_ - ultimoFrame) / 1000 * +document.getElementById("velocita").value);
  ultimoFrame = ora_;
  tempo.value = t;
  aggiorna();
  if (t >= M.tmax) { suonaFerma(); return; }
  requestAnimationFrame(passo);
}
document.addEventListener("keydown", e => {
  if (e.target.tagName === "SELECT") return;
  if (e.code === "Space") { e.preventDefault(); suonaFerma(); }
  if (e.code === "ArrowRight") { t = Math.min(M.tmax, t + (e.shiftKey ? 10 : 1)); tempo.value = t; aggiorna(); }
  if (e.code === "ArrowLeft") { t = Math.max(0, t - (e.shiftKey ? 10 : 1)); tempo.value = t; aggiorna(); }
});
document.getElementById("lista-eventi").onclick = e => {
  const d = e.target.closest("div[data-i]");
  if (!d) return;
  t = M.eventi[+d.dataset.i].t; tempo.value = t; aggiorna();
};
barraEv.onclick = e => {
  if (!M) return;
  const r = barraEv.getBoundingClientRect();
  t = (e.clientX - r.left) / r.width * M.tmax; tempo.value = t; aggiorna();
};
let trascina = null;
tela.onmousedown = e => { trascina = { x: e.clientX, y: e.clientY, ox: vista.ox, oy: vista.oy }; tela.classList.add("trascina"); };
window.onmouseup = () => { trascina = null; tela.classList.remove("trascina"); };
tela.onmousemove = e => {
  const r = tela.getBoundingClientRect();
  mouse = { x: e.clientX - r.left, y: e.clientY - r.top };
  if (trascina) { vista.ox = trascina.ox + e.clientX - trascina.x; vista.oy = trascina.oy + e.clientY - trascina.y; disegna(); }
  aggiornaCella();
};
tela.onmouseleave = () => { mouse = null; aggiornaCella(); };
tela.onwheel = e => {
  e.preventDefault();
  const r = tela.getBoundingClientRect(), mx = e.clientX - r.left, my = e.clientY - r.top;
  const f = Math.exp(-e.deltaY * 0.0015), s = Math.min(600, Math.max(8, vista.s * f));
  vista.ox = mx - (mx - vista.ox) * s / vista.s; vista.oy = my - (my - vista.oy) * s / vista.s; vista.s = s;
  disegna();
};
window.onresize = () => { adatta(); disegnaEventi(); disegna(); };

// in diretta: ogni 0,5 s aggiunge i dati nuovi (tools/visore/diretta.sh li copia dal robot) e segue l'ultimo istante
diretta.checked = new URLSearchParams(location.search).get("diretta") === "1";
let aggiornando = false;
setInterval(async () => {
  if (!diretta.checked || !M || aggiornando) return;
  aggiornando = true;
  try { await aggiungi(); } finally { aggiornando = false; }
  if (inMoto) return;
  t = M.tmax; tempo.value = t;
  disegnaEventi(); aggiorna();
}, 500);

document.getElementById("legenda").innerHTML =
  Object.entries({ 1: "esplorazione", 3: "giro dei bordi", 2: "pulizia a strisce", 4: "rientro", 8: "riposo" })
    .map(([mi, n]) => `${chip(COLORE_MISSIONE[mi])}${n}`).join(" &nbsp; ") + "<br>" +
  [["libera", COLORI[1]], ["bloccata", COLORI[2]], ["proibita", COLORI[3]], ["troppo stretta", COLORI[4]], ["pulita", VERDE]]
    .map(([n, c]) => `${chip(rgba(c))}${n}`).join(" &nbsp; ") + "<br>" +
  Object.entries({ guasto: "guasto", aggancio: "aggancio riuscito/fallito", batteria: "batteria", prova: "prova" })
    .map(([k, n]) => `${chip(COLORE_EVENTO[k])}${n}`).join(" &nbsp; ") +
  "<br>Base nera, cerchio = punto di aggancio, robot bianco (rosso in guasto).";

adatta();
caricaElenco();
