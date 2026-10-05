// stanza.js - Il modello della stanza nella pagina: lettura e scrittura del formato di sim/mondo.h, geometria per la
// selezione. Unità: metri; angoli in GRADI (come nel file); x a destra, y in alto.
"use strict";
const Stanza = (() => {
  const GRADI = Math.PI / 180;

  function nuova(larghezza, profondita) {
    const s = 0.1, w = larghezza, h = profondita;
    return {
      nome: "nuova", larghezza: w, profondita: h,
      robot: { x: w / 2, y: h / 2, angolo: 90 }, base: null,
      oggetti: [
        { tipo: "muro", da: [0, s / 2], a: [w, s / 2], spessore: s, quota_da: 0, quota_a: 2.5, nome: "muro", gruppo: "" },
        { tipo: "muro", da: [0, h - s / 2], a: [w, h - s / 2], spessore: s, quota_da: 0, quota_a: 2.5, nome: "muro", gruppo: "" },
        { tipo: "muro", da: [s / 2, s], a: [s / 2, h - s], spessore: s, quota_da: 0, quota_a: 2.5, nome: "muro", gruppo: "" },
        { tipo: "muro", da: [w - s / 2, s], a: [w - s / 2, h - s], spessore: s, quota_da: 0, quota_a: 2.5, nome: "muro", gruppo: "" },
      ],
    };
  }

  function leggi(testo) {
    const st = { nome: "senza nome", larghezza: 4, profondita: 3, robot: { x: 2, y: 1.5, angolo: 90 }, base: null, oggetti: [] };
    for (let riga of testo.split("\n")) {
      riga = riga.replace(/#.*/, "").trim();
      if (!riga) continue;
      const parti = riga.split(/\s+/), tipo = parti.shift(), kv = {}, punti = [];
      let inPunti = false;
      for (const p of parti) {
        const i = p.indexOf("=");
        if (i < 0) { if (inPunti) punti.push(p.split(",").map(Number)); continue; }
        const k = p.slice(0, i), v = p.slice(i + 1);
        inPunti = k === "punti";
        if (inPunti) { if (v) punti.push(v.split(",").map(Number)); } else kv[k] = v;
      }
      const n = k => kv[k] !== undefined ? +kv[k] : undefined;
      const coppia = k => kv[k] ? kv[k].split(",").map(Number) : undefined;
      if (tipo === "stanza") { st.nome = kv.nome || st.nome; st.larghezza = n("larghezza"); st.profondita = n("profondita"); }
      else if (tipo === "robot") st.robot = { x: n("x"), y: n("y"), angolo: n("angolo") || 0 };
      else if (tipo === "base") st.base = { x: n("x"), y: n("y"), angolo: n("angolo") || 0 };
      else {
        const o = { tipo, nome: kv.nome || tipo, gruppo: kv.gruppo || "", quota_da: n("quota_da") || 0 };
        if (tipo === "muro") Object.assign(o, { da: coppia("da"), a: coppia("a"), spessore: n("spessore") || 0.1, quota_a: n("quota_a") || 2.5 });
        else if (tipo === "rettangolo") Object.assign(o, { x: n("x"), y: n("y"), larghezza: n("larghezza"), profondita: n("profondita"), angolo: n("angolo") || 0, quota_a: n("quota_a") || 0.5 });
        else if (tipo === "cerchio") Object.assign(o, { x: n("x"), y: n("y"), raggio: n("raggio"), quota_a: n("quota_a") || 0.5 });
        else if (tipo === "vuoto") Object.assign(o, { punti, quota_a: 0 });
        else continue;
        st.oggetti.push(o);
      }
    }
    return st;
  }

  const f = v => (Math.round(v * 10000) / 10000).toString();

  function scrivi(st) {
    const r = ["# stanza del simulatore RTES (formato in sim/mondo.h)",
      `stanza nome=${st.nome.replace(/\s+/g, "_") || "stanza"} larghezza=${f(st.larghezza)} profondita=${f(st.profondita)}`,
      `robot x=${f(st.robot.x)} y=${f(st.robot.y)} angolo=${f(st.robot.angolo)}`];
    if (st.base) r.push(`base x=${f(st.base.x)} y=${f(st.base.y)} angolo=${f(st.base.angolo)}`);
    for (const o of st.oggetti) {
      let e = "";
      if (o.nome && o.nome !== o.tipo) e += ` nome=${o.nome.replace(/\s+/g, "_")}`;
      if (o.gruppo) e += ` gruppo=${o.gruppo.replace(/\s+/g, "_")}`;
      if (o.tipo === "muro") r.push(`muro${e} da=${f(o.da[0])},${f(o.da[1])} a=${f(o.a[0])},${f(o.a[1])} spessore=${f(o.spessore)} quota_da=${f(o.quota_da)} quota_a=${f(o.quota_a)}`);
      else if (o.tipo === "rettangolo") r.push(`rettangolo${e} x=${f(o.x)} y=${f(o.y)} larghezza=${f(o.larghezza)} profondita=${f(o.profondita)} angolo=${f(o.angolo)} quota_da=${f(o.quota_da)} quota_a=${f(o.quota_a)}`);
      else if (o.tipo === "cerchio") r.push(`cerchio${e} x=${f(o.x)} y=${f(o.y)} raggio=${f(o.raggio)} quota_da=${f(o.quota_da)} quota_a=${f(o.quota_a)}`);
      else if (o.tipo === "vuoto") r.push(`vuoto${e} punti=${o.punti.map(p => f(p[0]) + "," + f(p[1])).join(" ")}`);
    }
    return r.join("\n") + "\n";
  }

  // ---- geometria ----
  function centro(o) {
    if (o.tipo === "muro") return [(o.da[0] + o.a[0]) / 2, (o.da[1] + o.a[1]) / 2];
    if (o.tipo === "vuoto") {
      const n = o.punti.length;
      return [o.punti.reduce((s, p) => s + p[0], 0) / n, o.punti.reduce((s, p) => s + p[1], 0) / n];
    }
    return [o.x, o.y];
  }
  // un muro visto come rettangolo
  function comeRettangolo(o) {
    if (o.tipo !== "muro") return o;
    const [cx, cy] = centro(o), dx = o.a[0] - o.da[0], dy = o.a[1] - o.da[1];
    return { x: cx, y: cy, larghezza: Math.hypot(dx, dy), profondita: o.spessore, angolo: Math.atan2(dy, dx) / GRADI };
  }
  function locale(r, x, y) {
    const a = r.angolo * GRADI, dx = x - r.x, dy = y - r.y;
    return [dx * Math.cos(a) + dy * Math.sin(a), -dx * Math.sin(a) + dy * Math.cos(a)];
  }
  function dentroPoligono(p, x, y) {
    let d = false;
    for (let i = 0, j = p.length - 1; i < p.length; j = i++)
      if ((p[i][1] > y) !== (p[j][1] > y) && x < (p[j][0] - p[i][0]) * (y - p[i][1]) / (p[j][1] - p[i][1]) + p[i][0]) d = !d;
    return d;
  }
  function dentro(o, x, y, margine = 0) {
    if (o.tipo === "vuoto") return dentroPoligono(o.punti, x, y);
    if (o.tipo === "cerchio") return Math.hypot(x - o.x, y - o.y) <= o.raggio + margine;
    const r = comeRettangolo(o), [u, v] = locale(r, x, y);
    return Math.abs(u) <= r.larghezza / 2 + margine && Math.abs(v) <= r.profondita / 2 + margine;
  }
  // area approssimata (per scegliere l'oggetto più piccolo sotto il mouse)
  function area(o) {
    if (o.tipo === "cerchio") return Math.PI * o.raggio * o.raggio;
    if (o.tipo === "vuoto") { let a = 0; const p = o.punti; for (let i = 0, j = p.length - 1; i < p.length; j = i++) a += (p[j][0] + p[i][0]) * (p[j][1] - p[i][1]); return Math.abs(a / 2); }
    const r = comeRettangolo(o); return r.larghezza * r.profondita;
  }
  function sposta(o, dx, dy) {
    if (o.tipo === "muro") { o.da = [o.da[0] + dx, o.da[1] + dy]; o.a = [o.a[0] + dx, o.a[1] + dy]; }
    else if (o.tipo === "vuoto") o.punti = o.punti.map(p => [p[0] + dx, p[1] + dy]);
    else { o.x += dx; o.y += dy; }
  }
  // ruota di "gradi" intorno al punto (cx, cy)
  function ruota(o, gradi, cx, cy) {
    const a = gradi * GRADI, c = Math.cos(a), s = Math.sin(a);
    const rp = p => [cx + (p[0] - cx) * c - (p[1] - cy) * s, cy + (p[0] - cx) * s + (p[1] - cy) * c];
    if (o.tipo === "muro") { o.da = rp(o.da); o.a = rp(o.a); }
    else if (o.tipo === "vuoto") o.punti = o.punti.map(rp);
    else { [o.x, o.y] = rp([o.x, o.y]); if (o.tipo === "rettangolo") o.angolo = norm(o.angolo + gradi); }
  }
  const norm = g => { g = ((g + 180) % 360 + 360) % 360 - 180; return Math.round(g * 1e6) / 1e6; };

  // Categoria rispetto al robot (sim/robot.h): decide il colore e cosa succede
  const CORPO = 0.075, ROBOT = 0.095, LIDAR = 0.087;
  function categoria(o) {
    if (o.tipo === "vuoto") return "vuoto";
    if (o.quota_a < 0.02 || o.quota_da >= ROBOT) return "sopra";       // il robot ci passa (sopra o sotto)
    if (o.quota_da >= CORPO) return "torretta";                          // incastra la torretta, niente bumper
    if (o.quota_a < LIDAR) return "basso";                               // bumper, il lidar non lo vede
    return o.tipo === "muro" ? "muro" : "alto";                          // bumper e lidar
  }
  const DESCR_CATEGORIA = {
    muro: "muro: lo vede il lidar, preme il bumper",
    alto: "mobile dal pavimento: lo vede il lidar, preme il bumper",
    basso: "ostacolo basso: preme il bumper, il lidar NON lo vede",
    torretta: "mobile rialzato fra 7,5 e 9,5 cm: incastra la torretta del lidar, senza bumper",
    sopra: "il robot ci passa (sotto o sopra): il lidar non lo vede",
    vuoto: "vuoto (scala): lo vedono gli anti-caduta",
  };

  return { nuova, leggi, scrivi, centro, comeRettangolo, locale, dentro, area, sposta, ruota, norm, categoria, DESCR_CATEGORIA, GRADI };
})();
