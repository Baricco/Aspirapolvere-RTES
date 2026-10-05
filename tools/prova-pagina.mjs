// prova-pagina.mjs - Prova automatica della pagina del simulatore con un browser headless (Brave o Chromium), pilotato
// con il protocollo DevTools: clic, trascinamenti, tastiera, oggetti pronti, robot, ispettore in pausa.
// Non è dentro "make test" (serve il browser). Uso: node tools/prova-pagina.mjs [browser]
import { spawn } from "node:child_process";
import { mkdirSync, copyFileSync } from "node:fs";

const BROWSER = process.argv[2] || "/opt/brave-bin/brave";
const PORTA = 8097, DEBUG = 9333;
const dorm = ms => new Promise(r => setTimeout(r, ms));
let falliti = 0;
const controlla = (c, t) => { console.log(`  ${c ? "ok" : "FALLITO"}: ${t}`); if (!c) falliti++; };

mkdirSync("build/stanze-prova", { recursive: true });
copyFileSync("sim/stanze/soggiorno.txt", "build/stanze-prova/soggiorno.txt");
const sim = spawn("build/host/sim", ["--porta", String(PORTA), "--stanze", "build/stanze-prova", "--durata", "90", "build/stanze-prova/soggiorno.txt"], { stdio: "ignore" });
await dorm(500);
const br = spawn(BROWSER, ["--headless=new", "--disable-gpu", "--no-sandbox", `--remote-debugging-port=${DEBUG}`, "--window-size=1500,950",
                          "--user-data-dir=build/profilo-prova", `http://127.0.0.1:${PORTA}/`], { stdio: "ignore" });
let ws, id = 0;
const attese = new Map();
for (let i = 0; i < 50 && !ws; i++) {
  await dorm(200);
  try {
    const pagine = await (await fetch(`http://127.0.0.1:${DEBUG}/json`)).json();
    const p = pagine.find(x => x.type === "page");
    if (p) ws = new WebSocket(p.webSocketDebuggerUrl);
  } catch (e) { }
}
await new Promise(r => ws.onopen = r);
ws.onmessage = e => { const m = JSON.parse(e.data); if (attese.has(m.id)) { attese.get(m.id)(m); attese.delete(m.id); } };
const cdp = (method, params = {}) => new Promise(r => { const i = ++id; attese.set(i, r); ws.send(JSON.stringify({ id: i, method, params })); });
const val = async expr => (await cdp("Runtime.evaluate", { expression: expr, awaitPromise: true, returnByValue: true })).result.result.value;
const mouse = (type, x, y, extra = {}) => cdp("Input.dispatchMouseEvent", { type, x, y, button: "left", clickCount: 1, ...extra });
async function trascina(x0, y0, x1, y1, modifiers = 0) {
  await mouse("mousePressed", x0, y0, { modifiers });
  for (let k = 1; k <= 8; k++) await mouse("mouseMoved", x0 + (x1 - x0) * k / 8, y0 + (y1 - y0) * k / 8, { modifiers });
  await mouse("mouseReleased", x1, y1, { modifiers });
  await dorm(300);
}
const clic = async (x, y) => { await mouse("mousePressed", x, y); await mouse("mouseReleased", x, y); await dorm(200); };
const tasto = async (key, modifiers = 0, code = key) => {
  await cdp("Input.dispatchKeyEvent", { type: "keyDown", key, code, modifiers, windowsVirtualKeyCode: key.length === 1 ? key.toUpperCase().charCodeAt(0) : 0 });
  await cdp("Input.dispatchKeyEvent", { type: "keyUp", key, code, modifiers });
  await dorm(300);
};
const schermo = async (x, y) => val(`(() => { const r = tela.getBoundingClientRect(); const p = Editor.aSchermo(${x}, ${y}); return [p[0] + r.left, p[1] + r.top]; })()`);
const stanza = async () => val(`fetch("/stanza").then(r => r.text())`);
const ogg = async i => val(`App.st.oggetti[${i}]`);

await dorm(2500);
console.log("1. selezione e spostamento");
let [x, y] = await schermo(1.15, 3.4);                    // centro del divano
await clic(x, y);
controlla(await val(`App.sel && App.sel.k === "o" && App.st.oggetti[App.sel.i].nome === "divano"`), "clic sul divano lo seleziona");
const s = await val(`Editor.aSchermo(1, 0)[0] - Editor.aSchermo(0, 0)[0]`);   // pixel per metro
await trascina(x, y, x + 0.5 * s, y);
let d = await ogg(4);
controlla(Math.abs(d.x - 1.65) < 0.011, `trascinato di 50 cm (x = ${d.x})`);
controlla((await stanza()).includes("nome=divano x=1.65"), "la stanza nel simulatore è aggiornata");

console.log("2. tastiera: ruota, frecce, annulla, ripeti");
await tasto("r");
controlla((await ogg(4)).angolo === 15, "R ruota di 15°");
await tasto("ArrowUp", 8);                                 // Shift: 10 cm
controlla(Math.abs((await ogg(4)).y - 3.5) < 1e-6, "Shift+freccia su sposta di 10 cm");
await tasto("z", 2, "KeyZ"); await tasto("z", 2, "KeyZ");
d = await ogg(4);
controlla(d.angolo === 0 && Math.abs(d.y - 3.4) < 1e-6, "Ctrl+Z due volte annulla rotazione e spostamento");
await tasto("y", 2, "KeyY");
controlla((await ogg(4)).angolo === 15, "Ctrl+Y ripete la rotazione");

console.log("3. maniglia di rotazione con Shift (scatti di 15°)");
const m = await val(`Editor.maniglie().find(m => m.t === "ruota")`), c = await val(`Editor.centroSel()`);
const [mx, my] = await schermo(m.x, m.y), [cx, cy] = await schermo(c[0], c[1]);
const r = Math.hypot(mx - cx, my - cy);
const a0 = Math.atan2(-(my - cy), mx - cx), a1 = a0 + 1.2;                 // circa 69° in senso antiorario
await trascina(mx, my, cx + r * Math.cos(a1), cy - r * Math.sin(a1), 8);
d = await ogg(4);
controlla(d.angolo === 90, `angolo con scatto di 15°: 15 + 75 = ${d.angolo}°`);

console.log("4. oggetti pronti: tavolo in gruppo, spostato e cancellato insieme");
await val(`(() => { const s = document.getElementById("libreria"); s.value = "tavolo"; s.dispatchEvent(new Event("change")); })()`);
[x, y] = await schermo(2.0, 1.5);
const prima = await val(`App.st.oggetti.length`);
await clic(x, y);
const dopo = await val(`App.st.oggetti.length`);
controlla(dopo === prima + 5, `tavolo = piano + 4 gambe (${dopo - prima} oggetti)`);
const gruppo = await val(`App.st.oggetti[${prima}].gruppo`);
[x, y] = await schermo(2.0 - 0.55, 1.5 - 0.35);           // una gamba
await clic(x, y);
await trascina(x, y, x, y - 0.3 * s);
const gambe = await val(`App.st.oggetti.filter(o => o.gruppo === "${gruppo}").map(o => o.y)`);
controlla(Math.abs(gambe[0] - 1.8) < 0.011, `trascinando una gamba si sposta tutto il tavolo (piano a y = ${gambe[0]})`);
await tasto("Delete");
controlla(await val(`App.st.oggetti.length`) === prima, "Canc toglie tutto il gruppo");

console.log("5. robot: trascinato e ruotato con la tastiera");
const p0 = await val(`App.stato.robot`);
[x, y] = await schermo(p0.x, p0.y);
await clic(x, y);
controlla(await val(`App.sel && App.sel.k === "robot"`), "clic sul robot lo seleziona");
await trascina(x, y, x + 0.4 * s, y);
await dorm(400);
let p1 = await val(`App.stato.robot`);
controlla(Math.abs(p1.x - (p0.x + 0.4)) < 0.02, `robot spostato nel simulatore (x ${p0.x.toFixed(2)} -> ${p1.x.toFixed(2)})`);
await tasto("r");
await dorm(400);
p1 = await val(`App.stato.robot`);
controlla(Math.abs(((p1.dir - p0.dir + 540) % 360) - 180 - 15) < 1, `R ruota il robot di 15° (${p0.dir} -> ${p1.dir})`);

console.log("6. disegno di un muro con lo strumento");
await val(`document.querySelector('#barra button[data-s="muro"]').click()`);
const n0 = await val(`App.st.oggetti.length`);
[x, y] = await schermo(3.0, 1.0);
const [x2, y2] = await schermo(4.0, 1.0);
await trascina(x, y, x2, y2);
const muro = await val(`App.st.oggetti[App.st.oggetti.length - 1]`);
controlla(await val(`App.st.oggetti.length`) === n0 + 1 && muro.tipo === "muro" && Math.abs(muro.a[0] - 4.0) < 0.011, "muro disegnato da 3 a 4 m");

console.log("7. ispettore: pausa e passi");
await val(`document.querySelector('#schede button[data-scheda="messaggi"]').click()`);
await dorm(800);
controlla(await val(`document.querySelectorAll("#griglia .b").length`) === 96, "griglia di 96 byte");
await val(`document.getElementById("pausa").click()`);
await dorm(300);
const pos1 = await val(`document.getElementById("posizione").textContent`);
await tasto("ArrowLeft"); await tasto("ArrowLeft");
const pos2 = await val(`document.getElementById("posizione").textContent`);
controlla(pos1 && pos2 && pos1 !== pos2, `← va al messaggio precedente (${pos1.split(":")[0]} -> ${pos2.split(":")[0]})`);
await dorm(1000);
controlla(await val(`document.getElementById("posizione").textContent`) === pos2, "in pausa la vista resta ferma");
const sel = await val(`document.querySelector("#lista .sel") !== null`);
controlla(sel, "il messaggio scelto è evidenziato nella lista");

ws.close(); br.kill(); sim.kill();
console.log("PROVA PAGINA: " + (falliti ? `${falliti} CONTROLLI FALLITI` : "OK"));
process.exit(falliti ? 1 : 0);
