// griglia.js - La griglia di navigazione del programma dei task, sopra la stanza vera (GET /mappa-robot a 4 Hz).
// La legge build/host/osserva dalla memoria del programma (sim/osserva.c); il simulatore dà in /stato l'origine
// dell'odometria del programma (la posizione vera del robot quando è partito). Griglia: 256 x 256 celle da 0,20 m,
// cella 128 = origine, indice = y * 256 + x (tasks/vacuum/mapping.h). Byte della cella: 4 bit bassi = stato (0 sconosciuta,
// 1 libera, 2 bloccata, 3 proibita, 4 troppo stretta, chiusa dalla navigazione dopo un passaggio fallito), 4 bit alti =
// i 4 punti a +-5 cm dal centro (bit 4 + k, k = 2 * "y alta" + "x alta"), solo nelle celle libere. Ogni cella si
// disegna come 2 x 2 pixel, uno per punto: in una cella libera i punti non liberi sono rossi. Una cella libera con la
// maschera a zero (mappa di prima) si disegna tutta libera.
"use strict";
const Griglia = (() => {
  const N = 256, CELLA = 0.2, TESTA = 128;
  const COLORI = { 1: [42, 98, 184, 55], 2: [214, 39, 40, 135], 3: [142, 36, 170, 135], 4: [230, 140, 20, 150] };   // RGBA
  const STATI = ["cerco la griglia", "griglia letta", "programma terminato", "errore"];
  const tela = document.createElement("canvas");
  tela.width = tela.height = 2 * N;
  const c2 = tela.getContext("2d"), img = c2.createImageData(2 * N, 2 * N);
  const casella = document.getElementById("griglia-robot"), info = document.getElementById("info-griglia");
  let dati = null;           // { pid, stato, versione, letture, msg, libere, parziali, bloccate, proibite, strette }
  let assente = true;

  try { casella.checked = localStorage.getItem("griglia-robot") !== "0"; } catch (e) { /* va bene anche senza */ }
  casella.onchange = () => { try { localStorage.setItem("griglia-robot", casella.checked ? "1" : "0"); } catch (e) { } aggiornaInfo(); };

  function leggi(b) {
    const dv = new DataView(b.buffer, b.byteOffset, b.byteLength);
    if (b.length < TESTA + N * N || String.fromCharCode(b[0], b[1], b[2], b[3]) !== "RTNM") return null;
    let fine = 20;
    while (fine < TESTA && b[fine]) fine++;
    const d = { pid: dv.getUint32(4, true), stato: dv.getUint32(8, true), versione: dv.getUint32(12, true),
                letture: dv.getUint32(16, true), msg: new TextDecoder().decode(b.subarray(20, fine)),
                libere: 0, parziali: 0, bloccate: 0, proibite: 0, strette: 0 };
    const px = img.data;
    for (let i = 0; i < N * N; i++) {
      const v = b[TESTA + i] & 15, punti = b[TESTA + i] >> 4, x = i % N, y = (i / N) | 0;
      if (v === 1) { d.libere++; if (punti && punti !== 15) d.parziali++; }
      else if (v === 2) d.bloccate++; else if (v === 3) d.proibite++; else if (v === 4) d.strette++;
      for (let k = 0; k < 4; k++) {        // riga dell'immagine = y della griglia (la si capovolge disegnando)
        const p = ((2 * y + (k >> 1)) * 2 * N + 2 * x + (k & 1)) * 4;
        const c = v === 1 && punti && !(punti >> k & 1) ? COLORI[2] : COLORI[v];
        if (c) { px[p] = c[0]; px[p + 1] = c[1]; px[p + 2] = c[2]; px[p + 3] = c[3]; } else px[p + 3] = 0;
      }
    }
    c2.putImageData(img, 0, 0);
    return d;
  }

  async function ciclo() {
    if (casella.checked) {
      try {
        const r = await fetch("/mappa-robot");
        if (r.ok) { dati = leggi(new Uint8Array(await r.arrayBuffer())); assente = false; }
        else { dati = null; assente = true; }
      } catch (e) { dati = null; assente = true; }
    }
    aggiornaInfo();
    setTimeout(ciclo, 250);
  }

  const chip = (c, t) => `<span class="c" style="background:rgba(${c[0]},${c[1]},${c[2]},${(c[3] / 255).toFixed(2)})"></span>${t}`;
  function aggiornaInfo() {
    if (!casella.checked) { info.hidden = true; return; }
    info.hidden = false;
    if (assente || !dati) {
      info.innerHTML = `<b>Griglia del robot</b>: nessun programma osservato.<br><span class="tenue">Avvialo con ` +
        `<code>./rtes.sh sim stanza.txt build/host/vacuum</code>.</span>`;
      return;
    }
    const legenda = `${chip(COLORI[1], "libera")} &nbsp; ${chip(COLORI[2], "bloccata")} &nbsp; ${chip(COLORI[3], "proibita")} &nbsp; ${chip(COLORI[4], "troppo stretta")}`;
    if (dati.stato === 1 || dati.stato === 2) {
      info.innerHTML = `<b>Griglia del robot</b>: versione ${dati.versione}` + (dati.stato === 2 ? " (programma terminato)" : "") +
        `<br>${dati.libere} libere` + (dati.parziali ? ` (${dati.parziali} solo in parte)` : "") +
        `, ${dati.bloccate} bloccate (1020 sono il bordo della mappa)` +
        (dati.proibite ? `, ${dati.proibite} proibite` : "") + (dati.strette ? `, ${dati.strette} troppo strette` : "") + `<br>${legenda}`;
    } else {
      info.innerHTML = `<b>Griglia del robot</b>: ${STATI[dati.stato] || "?"}` + (dati.msg ? `<br><span class="${dati.stato === 3 ? "err" : "tenue"}">${dati.msg}</span>` : "");
    }
  }

  // Chiamata da Editor.disegna: V è la vista (s pixel/m, ox e oy = schermo dell'origine della stanza).
  function disegna(ctx, V) {
    const s = App.stato, o = s && s.programma;
    if (!casella.checked || !dati || dati.stato === 0 || dati.stato === 3 || !o || o.pid !== dati.pid) return;
    ctx.save();
    ctx.translate(V.ox + o.x * V.s, V.oy - o.y * V.s);       // origine dell'odometria del programma
    ctx.scale(V.s, -V.s);                                     // metri, y in alto
    ctx.rotate(o.dir);                                        // asse x della griglia = direzione iniziale del robot
    ctx.imageSmoothingEnabled = false;
    ctx.drawImage(tela, -N / 2 * CELLA, -N / 2 * CELLA, N * CELLA, N * CELLA);
    ctx.restore();
  }

  aggiornaInfo();
  ciclo();
  return { disegna };
})();
