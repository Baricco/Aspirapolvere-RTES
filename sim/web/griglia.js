// griglia.js - La griglia di navigazione del programma dei task, sopra la stanza vera (GET /mappa-robot a 4 Hz).
// La legge build/host/osserva dalla memoria del programma (sim/osserva.c); il simulatore dà in /stato l'origine
// dell'odometria del programma (la posizione vera del robot quando è partito). Griglia: 256 x 256 celle da 0,20 m,
// cella 128 = origine, indice = y * 256 + x (tasks/vacuum/mapping.h). Valori: 0 sconosciuta, 1 libera, 2 bloccata,
// 3 proibita.
"use strict";
const Griglia = (() => {
  const N = 256, CELLA = 0.2, TESTA = 128;
  const COLORI = { 1: [42, 98, 184, 55], 2: [214, 39, 40, 135], 3: [142, 36, 170, 135] };   // RGBA
  const STATI = ["cerco la griglia", "griglia letta", "programma terminato", "errore"];
  const tela = document.createElement("canvas");
  tela.width = tela.height = N;
  const c2 = tela.getContext("2d"), img = c2.createImageData(N, N);
  const casella = document.getElementById("griglia-robot"), info = document.getElementById("info-griglia");
  let dati = null;           // { pid, stato, versione, letture, msg, libere, bloccate, proibite }
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
                libere: 0, bloccate: 0, proibite: 0 };
    const px = img.data;
    for (let i = 0; i < N * N; i++) {
      const v = b[TESTA + i], c = COLORI[v], p = i * 4;   // riga dell'immagine = y della griglia (la si capovolge disegnando)
      if (v === 1) d.libere++; else if (v === 2) d.bloccate++; else if (v === 3) d.proibite++;
      if (c) { px[p] = c[0]; px[p + 1] = c[1]; px[p + 2] = c[2]; px[p + 3] = c[3]; } else px[p + 3] = 0;
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
    const legenda = `${chip(COLORI[1], "libera")} &nbsp; ${chip(COLORI[2], "bloccata")} &nbsp; ${chip(COLORI[3], "proibita")}`;
    if (dati.stato === 1 || dati.stato === 2) {
      info.innerHTML = `<b>Griglia del robot</b>: versione ${dati.versione}` + (dati.stato === 2 ? " (programma terminato)" : "") +
        `<br>${dati.libere} libere, ${dati.bloccate} bloccate (1020 sono il bordo della mappa)` +
        (dati.proibite ? `, ${dati.proibite} proibite` : "") + `<br>${legenda}`;
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
