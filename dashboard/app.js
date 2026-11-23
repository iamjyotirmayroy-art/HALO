"use strict";
// HALO rescuer dashboard: real map + place search, mission launch, live swarm, victims & AI verification,
// drones, camera wall, manual takeover, alerts, layers, replay. Data comes from the local simulation.
const DATA = "../simulation/results/";
const COLORS = ["#4287f5", "#f5a623", "#9b59b6", "#2ecc71", "#e74c3c", "#1abcc4"];
const $ = id => document.getElementById(id);
const color = i => COLORS[i % COLORS.length];
const rgba = (hex, a) => `rgba(${[1, 3, 5].map(k => parseInt(hex.slice(k, k + 2), 16))},${a})`;
const esc = s => String(s ?? "").replace(/[&<>"]/g, c => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;" })[c]);

let S = null, ST = null, frames = null, idx = 0, playing = true, mode = "live", lastT = -1, lastAck = 0, lastSnap = 0;
let vfilter = "all", allOnline = true, wallKey = "", pilot = null, summaryShown = false, sawRunning = false, primed = false, routesKey = "";
const watch = new Set(), trailOff = new Set(), camMode = {}, seenAlerts = new Set(), trailLines = {};
let muted = false;
try { muted = localStorage.getItem("halo-muted") === "1"; } catch (e) { /* storage blocked: default on */ }

// ---------------- map ----------------
const map = L.map("map", { maxZoom: 21, zoomSnap: 0.25 }).setView([22.5, 79], 5);     // opens on India
const tiles = {
  "Street map": L.tileLayer("https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png",
    { maxZoom: 21, maxNativeZoom: 19, attribution: "© OpenStreetMap contributors" }),
  "Satellite": L.tileLayer("https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}",
    { maxZoom: 21, maxNativeZoom: 19, attribution: "Imagery © Esri" }),
};
tiles["Street map"].addTo(map);
const G = Object.fromEntries(["base", "hazards", "zones", "routes", "links", "trails", "heat", "dyn", "draw"].map(k => [k, L.layerGroup().addTo(map)]));
L.control.layers(tiles, { "☢ Hazards": G.hazards, "🧩 Zones & searched": G.zones, "🟢 Safe routes": G.routes,
  "📡 Radio links": G.links, "🛩 Drone trails": G.trails, "🔥 Thermal zones": G.heat },
  { position: "topright", collapsed: innerWidth < 900 }).addTo(map);
const legend = L.control({ position: "bottomleft" });
legend.onAdd = () => Object.assign(L.DomUtil.create("div", "legend"), { innerHTML:
  '<i style="background:#9aa3ad"></i>Building<br><i style="background:#ff8c1a"></i>Unstable structure<br>' +
  '<i style="background:#ffd400"></i>Radiation<br><i style="background:#8a6a4a"></i>Rubble<br>' +
  '<i style="background:#2ecc71"></i>Safe rescue route<br><i style="background:#ff4d4f"></i>Unsafe direct route<br>' +
  '<i style="background:#ff9a2e;border-radius:50%"></i>Rescue team' });

function ll(x, y) {   // sim metres -> lat/lon (the sim square is a scale model fitted onto the drawn area)
  const [lat0, lon0] = S.origin, k = S.map_scale;
  return [lat0 + y * k / 111320, lon0 + x * k / (111320 * Math.cos(lat0 * Math.PI / 180))];
}
const gridCanvas = document.createElement("canvas");
function gridImage(paint) {   // one pixel per grid cell, north up -> stretched over the area as an image overlay
  const n = S.n, g = gridCanvas.getContext("2d");
  gridCanvas.width = gridCanvas.height = n;
  for (let i = 0; i < n; i++) for (let j = 0; j < n; j++) {
    const c = paint(i, j);
    if (c) { g.fillStyle = c; g.fillRect(i, n - 1 - j, 1, 1); }
  }
  return gridCanvas.toDataURL();
}
const gridOverlay = (url, group) => L.imageOverlay(url, [ll(0, 0), ll(S.area, S.area)], { className: "px", interactive: false }).addTo(group);
const label = (xy, html, group, w = 90) => L.marker(ll(...xy), { interactive: false,
  icon: L.divIcon({ className: "lbl", html, iconSize: [w, 14], iconAnchor: [w / 2, 7] }) }).addTo(group);

let zoneOverlay = null, baseMarker = null;
function drawStatic() {
  Object.values(G).forEach(g => g.clearLayers());
  Object.keys(trailLines).forEach(k => delete trailLines[k]);
  zoneOverlay = null; routesKey = "";
  const cells = a => new Set(a.map(String)), tall = cells(S.tall), weak = cells(S.weak), rubble = cells(S.rubble);
  gridOverlay(gridImage((i, j) => tall.has(i + "," + j) ? "rgba(154,163,173,.85)" : weak.has(i + "," + j) ? "rgba(255,140,26,.45)"
    : rubble.has(i + "," + j) ? "rgba(138,106,74,.35)" : S.radiation[i][j] > .15 ? `rgba(255,212,0,${Math.min(.65, S.radiation[i][j] / 3 * .7)})` : null), G.hazards);
  L.circle(ll(...S.dead_zone.c), { radius: S.dead_zone.r * S.map_scale, color: "#b07cff", weight: 1.5, dashArray: "4 4", fillOpacity: .05 })
    .bindTooltip("No radio signal (debris shadow)").addTo(G.hazards);
  const flat = S.radiation.flat(), r = flat.indexOf(Math.max(...flat));
  label([(Math.floor(r / S.n) + .5) * S.cell, (r % S.n + .5) * S.cell], "☢ RADIATION", G.hazards);
  const outline = (S.poly || [[0, 0], [S.area, 0], [S.area, S.area], [0, S.area]]).map(p => ll(...p));
  L.polygon(outline, { color: "#fff", weight: 2, fill: false, dashArray: "6 4", interactive: false }).addTo(G.base);
  S.pads.forEach(p => L.circleMarker(ll(...p), { radius: 4, color: "#1a9be6", weight: 2, fillOpacity: .5, interactive: false }).addTo(G.base));
  label(S.base, "🚑 BASE", G.base, 60);
  if (baseMarker) { baseMarker.remove(); baseMarker = null; }
  map.fitBounds(L.latLngBounds([...outline, ll(...S.base)]), { padding: [20, 20] });
  legend.addTo(map);
}

const prioColor = p => p >= 70 ? "#ff4d4f" : p >= 50 ? "#f5a623" : "#ffd166";
function victimColor(v) {
  if (v.status.startsWith("DISMISSED")) return "#6e7681";
  if (v.reached_t) return "#3b8cff";
  if (v.status.includes("verify")) return "#ffd166";
  return v.source === "phone" ? "#ff6bf0" : "#2ecc71";
}
const signal = d => d.state === "TRAPPED" ? "🆘 SOS beacon" : !d.online ? "offline · autonomous" : d.hops > 1 ? `mesh relay ×${d.hops - 1}` : "online";
const humans = st => st.victims.filter(v => !v.status.startsWith("DISMISSED"));
const isLive = () => mode === "live" && ST && !ST.final;

function trailsOf(st) {   // live state carries trails; in replay they are rebuilt from earlier frames
  if (st.trails) return st.trails;
  return st.drones.map((d, i) => frames.slice(0, idx + 1).map(f => f.drones[i].pos));
}

// ---------------- render one state ----------------
function render(st) {
  ST = st;
  if (st.t === lastT) return;
  lastT = st.t;
  const H = humans(st);
  $("k-time").textContent = st.t.toFixed(1) + " s";
  $("k-cov").textContent = st.coverage.toFixed(0) + "%";
  $("k-found").textContent = `${H.filter(v => v.source === "phone" || !v.status.startsWith("POSSIBLE")).length} / ${S.n_victims}`;
  $("k-reached").textContent = st.victims.filter(v => v.reached_t).length;
  $("k-online").textContent = `${st.drones.filter(d => d.online && d.state !== "TRAPPED").length} / ${st.drones.length}`;

  // zones in each drone's colour, filled in as they are searched
  const url = gridImage((i, j) => {
    const k = i * S.n + j, z = st.zones ? st.zones[k] : ".", cov = st.covered[k] === "1";
    return z !== "." ? rgba(color(z.charCodeAt(0) - 48), cov ? .42 : .14) : cov ? "rgba(59,140,255,.2)" : null;
  });
  if (zoneOverlay) zoneOverlay.setUrl(url); else zoneOverlay = gridOverlay(url, G.zones);

  const rk = JSON.stringify(st.routes);
  if (rk !== routesKey) {
    routesKey = rk;
    G.routes.clearLayers();
    for (const [vid, r] of Object.entries(st.routes)) {
      if (r.direct.length) {
        L.polyline(r.direct.map(p => ll(...p)), { color: "#ff4d4f", weight: 2, dashArray: "5 5", opacity: .85 })
          .bindTooltip(`Direct route to ${vid} unsafe: ${esc(r.reason)}`).addTo(G.routes);
        label(r.direct[Math.floor(r.direct.length / 2)], `✖ ${esc(r.reason)}`, G.routes, 200);
      }
      if (r.safe.length) L.polyline([ll(...S.base), ll(...S.entry), ...r.safe.map(p => ll(...p))], { color: "#2ecc71", weight: 3, opacity: .85 })
        .bindTooltip(`Safe route to ${vid} · ${esc(r.status)}`).addTo(G.routes);
    }
  }

  G.heat.clearLayers();
  st.heat.forEach(h => {
    const v = st.victims.find(x => x.id === h.id), dismissed = v && v.status.startsWith("DISMISSED"), hot = Math.min(1, (h.temp - 25) / 15);
    [[1.0, .10], [0.65, .18], [0.38, .30], [0.16, .55]].forEach(([r, o]) => L.circle(ll(...h.pos), {
      radius: r * S.map_scale, stroke: false, interactive: false, className: h.checked ? "" : "sos-pulse",
      fillColor: dismissed ? "#6e7681" : r < .3 ? "#fff3b0" : r < .5 ? "#ffb000" : "#ff2a00",
      fillOpacity: dismissed ? o * .4 : o * (.6 + .4 * hot) }).addTo(G.heat));
    const tag = !h.checked || !v ? '<span style="color:#ffd166">🤖 AI checking…</span>'
      : dismissed ? '<span style="color:#9aa3ad">✖ not a person</span>'
      : `<span style="color:#7dffa1">✔ ${esc(v.label.replace("body part: ", ""))} ${Math.round(v.confidence * 100)}%</span>`;
    L.marker(ll(h.pos[0], h.pos[1] - .75), { interactive: false, icon: L.divIcon({ className: "lbl", iconSize: [140, 26], iconAnchor: [70, 0],
      html: `<div style="text-align:center">🔥 ${h.temp.toFixed(1)}°C<br>${tag}</div>` }) }).addTo(G.heat);
  });

  G.links.clearLayers();
  st.links.forEach(([a, b]) => L.polyline([ll(...a), ll(...b)], { color: "#c3a3ff", weight: 1, opacity: .6, interactive: false }).addTo(G.links));

  trailsOf(st).forEach((tr, i) => {
    const pts = trailOff.has(i) ? [] : tr.map(p => ll(p[0], p[1]));
    if (trailLines[i]) trailLines[i].setLatLngs(pts);
    else trailLines[i] = L.polyline(pts, { color: color(i), weight: 2, opacity: .85, interactive: false }).addTo(G.trails);
  });

  G.dyn.clearLayers();
  if (st.collapsed) { const b = S.weak_box;
    L.rectangle([ll(b[0], b[1]), ll(b[2], b[3])], { color: "#ff4d4f", weight: 2, fillOpacity: .25 }).bindTooltip("COLLAPSED in aftershock").addTo(G.dyn); }
  st.victims.forEach(v => {
    L.circleMarker(ll(...v.pos), { radius: 8, color: victimColor(v), weight: 3, fillOpacity: .35 })
      .bindTooltip(`${v.id} · ${esc(v.label)}<br>${esc(v.status)}<br>priority ${v.priority}`).on("click", () => openVictim(v.id)).addTo(G.dyn);
    label([v.pos[0] + .3, v.pos[1] + .3], v.id, G.dyn, 30);
  });
  st.sos.forEach(s => L.circleMarker(ll(...s.pos), { radius: 16, color: "#ff4d4f", weight: 3, fill: false, className: "sos-pulse" })
    .bindTooltip(`SOS · drone ${s.drone} trapped`).addTo(G.dyn));
  st.teams.forEach(t => {
    L.circleMarker(ll(...t.pos), { radius: 7, color: "#fff", weight: 2, fillColor: "#ff9a2e", fillOpacity: 1 })
      .bindTooltip(`Team ${t.name}: ${t.state}${t.victim ? " → " + t.victim : ""}`).addTo(G.dyn);
    label([t.pos[0], t.pos[1] - .45], t.name, G.dyn, 24);
  });
  st.drones.forEach(d => L.marker(ll(d.pos[0], d.pos[1]), { icon: L.divIcon({ className: "", iconSize: [22, 22],
    html: `<div class="dm ${d.online ? "" : "off"} ${d.state === "MANUAL" ? "manual" : ""}" style="background:${color(d.id)}">` +
          `<b style="transform:rotate(${90 - d.heading * 180 / Math.PI}deg)"></b>D${d.id}</div>` }) })
    .bindTooltip(`Drone ${d.id}: ${d.state} · ${d.battery.toFixed(0)}% · ${signal(d)}`).on("click", () => openDrone(d.id)).addTo(G.dyn));

  renderAlerts(st);
  $("drones").innerHTML = st.drones.map(d => {
    const b = d.battery, bc = b > 40 ? "#2ecc71" : b > 20 ? "#f5a623" : "#ff4d4f";
    return `<div class="drone ${d.state === "TRAPPED" ? "sos" : ""}" data-drone="${d.id}" style="border-left:3px solid ${color(d.id)}">
      <div class="top"><span>Drone ${d.id}</span><span style="color:${color(d.id)}">${d.state}</span></div>
      <div class="bar"><i style="width:${b}%;background:${bc}"></i></div><div class="vmeta">${b.toFixed(0)}% · ${signal(d)}</div></div>`;
  }).join("");
  $("teams").innerHTML = st.teams.map(t => `<div class="team"><b>${t.name}</b> <span class="vmeta">${t.state.replace("_", " ")}</span><br>
    <span class="vmeta">${t.victim ? "→ " + t.victim : "&nbsp;"} · saved ${t.rescued.length}</span></div>`).join("");
  renderVictims();
  $("feed").innerHTML = [...st.events].reverse().map(e => `<div class="lvl-${e.lvl}">[${e.t.toFixed(1).padStart(6)}s] ${esc(e.msg)}</div>`).join("");

  if (mode === "live") liveFeedback(st);
  if (!$("camwall").hidden) renderWall(st);
  if (pilot) renderPilot(st);
  sawRunning ||= !st.final;
  if (st.final && sawRunning && !summaryShown) { summaryShown = true; showSummary(st); }   // only after watching it run
}

function renderVictims() {
  const keep = { all: () => true, open: v => !v.reached_t && !v.status.startsWith("DISMISSED"),
    verify: v => v.status.includes("verify") && !v.reached_t, buried: v => v.source === "phone" }[vfilter];
  const vs = ST.victims.filter(keep).sort((a, b) => (a.reached_t ? 1 : 0) - (b.reached_t ? 1 : 0) || b.priority - a.priority);
  $("victims").innerHTML = vs.map(v => `<div class="victim" data-victim="${v.id}">
    ${v.photo ? `<div class="thumbs"><img src="${DATA + v.thermal}" alt="thermal ${v.id}" loading="lazy"><img src="${DATA + v.photo}" alt="photo ${v.id}" loading="lazy"></div>`
      : '<div class="noimg">📶 phone signal only</div>'}
    <div><div class="row1"><span>${v.id} · ${esc(v.label)}</span><span class="prio" style="background:${prioColor(v.priority)}">P${v.priority}</span></div>
      <div class="status" style="color:${victimColor(v)}">${v.reached_t ? `✔ Reached by ${v.team} (${v.response_s}s)` : esc(v.status)}</div>
      <div class="vmeta">${v.source === "phone" ? "📶 phone signal · likely buried" : `🤖 AI ${Math.round(v.confidence * 100)}% · 🔥 ${v.temp ?? "–"}°C`}
        ${v.team && !v.reached_t ? ` · team ${v.team}` : ""}${v.route_status && v.route_status.startsWith("REROUTED") ? " · rerouted" : ""}</div></div></div>`).join("")
    || `<div class="empty">${ST.victims.length ? "None in this filter" : "Searching…"}</div>`;
}

// ---------------- alerts: banner + sound for new ones ----------------
function renderAlerts(st) {
  const A = [];
  st.sos.forEach(s => A.push({ key: "sos" + s.drone, cls: "", lvl: "sos", title: `🆘 SOS · Drone ${s.drone} trapped`,
    text: `Last position (${s.pos[0]}, ${s.pos[1]}) at ${s.t}s · beacon active · zone re-assigned` }));
  st.heat.filter(h => !h.checked).forEach(h => A.push({ key: "heat" + h.id, cls: "heat", lvl: "heat",
    title: `🔥 Heat signal ${h.temp.toFixed(1)}°C`, text: `Drone ${h.drone} diving for a photo · 🤖 AI checking for a person or body part…` }));
  st.victims.filter(v => v.status.includes("verify") && v.photo && !v.reached_t).forEach(v => A.push({ key: "ver" + v.id, cls: "verify", lvl: "verify",
    title: `Verify ${v.id} · ${esc(v.label)} (${Math.round(v.confidence * 100)}% AI)`, text: "Look at the drone photo: person or not?", victim: v.id }));
  Object.entries(st.routes).filter(([, r]) => r.direct.length).forEach(([vid, r]) => A.push({ key: "route" + vid, cls: "route", lvl: "route",
    title: `Unsafe direct route to ${vid}`, text: `${esc(r.reason)} · team rerouted on a safe path` }));
  $("alerts").innerHTML = A.map(a => `<div class="alert ${a.cls}" ${a.victim ? `data-victim="${a.victim}"` : ""}><b>${a.title}</b><br><span class="vmeta">${a.text}</span></div>`).join("")
    || '<div class="empty">No alerts</div>';
  const fresh = A.filter(a => !seenAlerts.has(a.key));
  fresh.forEach(a => seenAlerts.add(a.key));
  if (primed && fresh.length) {
    const a = fresh.find(x => x.lvl === "sos") || fresh[0];
    banner(a.title, a.lvl);
    if (mode === "live") beep(a.lvl === "sos");
  }
  primed = true;
}
let bannerTimer = 0, audio = null;
function banner(text, lvl) {
  const b = $("banner");
  b.innerHTML = text; b.className = "banner " + (lvl === "sos" ? "" : "verify"); b.hidden = false;
  clearTimeout(bannerTimer); bannerTimer = setTimeout(() => b.hidden = true, 6000);
}
function beep(urgent) {
  if (muted) return;
  try {
    audio = audio || new AudioContext();
    (urgent ? [0, .25, .5] : [0]).forEach(t0 => {
      const o = audio.createOscillator(), g = audio.createGain(), t = audio.currentTime + t0;
      o.frequency.value = urgent ? 880 : 620; g.gain.setValueAtTime(.18, t); g.gain.exponentialRampToValueAtTime(.001, t + .2);
      o.connect(g).connect(audio.destination); o.start(t); o.stop(t + .2);
    });
  } catch (e) { /* audio unavailable: banner only */ }
}
$("btn-mute").onclick = () => {
  muted = !muted; $("btn-mute").textContent = muted ? "🔕" : "🔔";
  try { localStorage.setItem("halo-muted", muted ? "1" : "0"); } catch (e) { /* not persisted */ }
};
$("btn-mute").textContent = muted ? "🔕" : "🔔";

let toastTimer = 0;
function toast(html, ms = 3500) {
  const t = $("toast"); t.innerHTML = html; t.hidden = false;
  clearTimeout(toastTimer); toastTimer = setTimeout(() => t.hidden = true, ms);
}
function liveFeedback(st) {   // acknowledgements and snapshot results from the swarm
  if (st.ack > lastAck) { if (lastAck) toast("✔ Command carried out by the swarm"); lastAck = st.ack; }
  const s = st.snapshots[st.snapshots.length - 1];
  if (s && s.t !== lastSnap) {
    if (lastSnap) toast(`<b>📸 Drone ${s.drone}</b> · 🤖 ${esc(s.text)}<br><img src="${DATA + s.img}?${s.t}" style="width:260px;margin-top:6px;border-radius:6px">`, 7000);
    lastSnap = s.t;
  }
}

// ---------------- commands to the swarm ----------------
async function api(path, body, quiet) {
  if (!isLive()) { if (!quiet) toast("Commands only work during a live mission"); return null; }
  try {
    const r = await fetch("/api/" + path, { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(body) });
    const j = await r.json();
    if (!quiet) toast(j.ok ? "📡 Sent to the swarm…" : "⚠ " + esc(j.error));
    return j;
  } catch (e) { if (!quiet) toast("⚠ Dashboard server not reachable"); return null; }
}

function openModal(html) { $("modal-body").innerHTML = html; $("modal").hidden = false; }
$("modal-close").onclick = () => $("modal").hidden = true;

function openVictim(id) {
  const v = ST.victims.find(x => x.id === id);
  if (!v) return;
  const open = !v.reached_t && !v.team;
  openModal((v.photo ? `<div class="pair"><div><img src="${DATA + v.thermal}"><div class="vmeta">Thermal camera</div></div>
      <div><img src="${DATA + v.photo}"><div class="vmeta">Drone photo · AI boxes</div></div></div>
      <div class="vmeta" style="margin-top:8px">🤖 AI check: <b style="color:var(--text)">${esc(v.label)}</b> · ${Math.round(v.confidence * 100)}% confidence</div>
      <div class="bar" style="height:8px"><i style="width:${Math.round(v.confidence * 100)}%;background:${v.confidence >= .85 ? "#2ecc71" : v.confidence >= .5 ? "#f5a623" : "#ff4d4f"}"></i></div>`
    : '<div class="noimg" style="width:100%;height:120px">📶 Located by phone signal only. No thermal or visual contact (likely buried).</div>') +
    `<h2 style="margin:10px 0 4px">${v.id} · ${esc(v.label)}</h2><div class="status" style="color:${victimColor(v)}">${esc(v.status)}</div>
     <p class="vmeta">Priority <b>${v.priority}</b> · ${v.source === "phone" ? "phone signal" : `🔥 ${v.temp}°C`} · ${v.trapped ? "trapped" : "not trapped"}
     · found at ${v.t}s<br>Route: ${esc(v.route_status || "pending")}${v.route_note ? "<br>" + esc(v.route_note) : ""}<br>
     ${v.team ? `Team ${v.team} ${v.reached_t ? `reached at ${v.reached_t}s (${v.response_s}s after detection)` : "on the way"}` : "No team assigned yet"}</p>
     <div class="row">${v.photo ? `<button data-act="confirm" data-id="${v.id}">✔ Confirm person</button><button data-act="reject" data-id="${v.id}">✖ Not a person</button>` : ""}
     <button class="primary" data-act="team" data-id="${v.id}" ${open ? "" : "disabled"}>🚑 Send team</button></div>`);
  map.panTo(ll(...v.pos));
}

function openDrone(id) {
  const d = ST.drones[id];
  openModal(`<h2 style="margin:0 0 6px;color:${color(id)}">Drone ${id}</h2>
    <p>State <b>${d.state}</b>${d.recalled ? " · recalled" : ""} · battery <b>${d.battery.toFixed(0)}%</b> · ${signal(d)}<br>
    <span class="vmeta">Position (${d.pos[0]}, ${d.pos[1]}) · altitude ${d.pos[2]} m (scale model)</span></p>
    <div class="row"><button data-act="trail" data-id="${id}">${trailOff.has(id) ? "Show" : "Hide"} trail</button>
    <button data-act="watch" data-id="${id}">📹 Watch camera</button><button data-act="control" data-id="${id}">🎮 Take control</button>
    <button class="danger" data-act="recall" data-id="${id}">↩ Recall to base</button></div>`);
  map.panTo(ll(d.pos[0], d.pos[1]));
}

document.addEventListener("click", e => {
  const t = e.target.closest("[data-act],[data-victim],[data-drone]");
  if (!t) return;
  if (t.dataset.victim) return openVictim(t.dataset.victim);
  if (t.dataset.drone) return openDrone(+t.dataset.drone);
  const id = t.dataset.id, n = +id;
  ({
    confirm: () => api("verify", { victim: id, person: true }),
    reject: () => api("verify", { victim: id, person: false }),
    team: () => api("command", { type: "send_team", victim: id }),
    recall: () => api("command", { type: "recall", drone: n }),
    trail: () => { trailOff.has(n) ? trailOff.delete(n) : trailOff.add(n); lastT = -1; render(ST); openDrone(n); },
    watch: () => { allOnline = false; watch.clear(); watch.add(n); openWall(); },
    control: () => startPilot(n),
    snap: () => api("command", { type: "snapshot", drone: n }),
    locate: () => { closeWall(); const d = ST.drones[n]; map.flyTo(ll(d.pos[0], d.pos[1]), Math.max(map.getZoom(), 18)); openDrone(n); },
    mode: () => { camMode[n] = camMode[n] === "thermal" ? "normal" : "thermal"; wallKey = ""; renderWall(ST); },
  })[t.dataset.act]?.();
  if (["confirm", "reject", "team", "recall", "watch", "control"].includes(t.dataset.act)) $("modal").hidden = true;
});
$("vfilter").onclick = e => {
  const b = e.target.closest("button"); if (!b) return;
  vfilter = b.dataset.f;
  $("vfilter").querySelectorAll("button").forEach(x => x.classList.toggle("on", x === b));
  if (ST) renderVictims();
};

// ---------------- 📹 camera wall (watch only: never changes what the drones do) ----------------
const camSrc = (id, m) => `/api/cam/${id}${m === "thermal" ? "?mode=thermal" : ""}`;
function wallIds() {
  return allOnline ? ST.drones.filter(d => d.online).map(d => d.id) : [...watch].sort((a, b) => a - b);
}
function postWatch() { api("watch", { drones: $("camwall").hidden ? [] : wallIds() }, true); }
function openWall() {
  if (!ST) return toast("Start a mission first");
  if (mode !== "live") return toast("Live cameras are only available during a live mission");
  $("camwall").hidden = false; wallKey = ""; renderWall(ST);
}
function closeWall() {
  $("camwall").hidden = true; $("camgrid").innerHTML = ""; wallKey = ""; postWatch();
  document.body.removeAttribute("data-tab"); syncTabs();
}
$("btn-cams").onclick = openWall;
$("cams-close").onclick = closeWall;
$("campick").onclick = e => {
  const b = e.target.closest("button"); if (!b) return;
  if (b.dataset.cam === "all") { allOnline = !allOnline; watch.clear(); }
  else { allOnline = false; const n = +b.dataset.cam; watch.has(n) ? watch.delete(n) : watch.add(n); }
  wallKey = ""; renderWall(ST);
};
function renderWall(st) {
  $("campick").innerHTML = `<button data-cam="all" class="${allOnline ? "on" : ""}">All online</button>` + st.drones.map(d =>
    `<button data-cam="${d.id}" class="${!allOnline && watch.has(d.id) ? "on" : ""}" style="border-color:${color(d.id)}">D${d.id}</button>`).join("");
  const ids = wallIds(), key = ids.map(i => i + (camMode[i] || "")).join(",");
  const grid = $("camgrid");
  if (key !== wallKey) {   // (re)build tiles only when the selection changes; streams keep running
    wallKey = key;
    grid.className = "camgrid n" + (ids.length <= 1 ? 1 : ids.length === 2 ? 2 : ids.length <= 4 ? 4 : 6);
    grid.innerHTML = ids.map(i => `<div class="tile" id="tile${i}"><img src="${camSrc(i, camMode[i])}" alt="drone ${i} camera">
      <div class="hd"></div><div class="flag" hidden></div><div class="ft">
      <button data-act="mode" data-id="${i}">${camMode[i] === "thermal" ? "Normal" : "Thermal"}</button>
      <button data-act="snap" data-id="${i}">📸 Snapshot + AI check</button><button data-act="locate" data-id="${i}">📍 Show on map</button>
      <button data-act="control" data-id="${i}">🎮 Take control</button></div></div>`).join("") || '<div class="empty" style="padding:20px">No drone selected</div>';
    postWatch();
  }
  ids.forEach(i => {
    const d = st.drones[i], cam = (st.cams || {})[i], tile = $("tile" + i);
    if (!tile) return;
    tile.querySelector(".hd").innerHTML = `<b style="color:${color(i)}">● Drone ${i}</b><span>${d.state}</span><span class="sp"></span>
      ${cam && cam.heat ? `<span class="badge">🔥 HEAT ${cam.heat.toFixed(1)}°C</span>` : ""}
      ${cam && cam.people.length ? `<span class="badge ai">🤖 ${esc(cam.people.join(", "))}</span>` : ""}<span>🔋${d.battery.toFixed(0)}%</span><span class="vmeta">${signal(d)}</span>`;
    tile.classList.toggle("heat", !!(cam && cam.heat));
    const flag = tile.querySelector(".flag"), sos = d.state === "TRAPPED";
    flag.hidden = d.online && !sos && !!cam;
    flag.className = "flag" + (sos ? " sos" : "");
    flag.textContent = sos ? "🆘 SOS · TRAPPED" : !d.online ? "NO SIGNAL" : "CONNECTING…";
  });
}

// ---------------- 🎮 manual takeover (input is only a desired velocity: safety stays on) ----------------
const keys = new Set(), sticks = { move: [0, 0], alt: [0, 0] };
let pilotTimer = 0;
function startPilot(id) {
  if (!isLive()) return toast("Manual control only works during a live mission");
  closeWall();
  api("command", { type: "takeover", drone: id }, true);
  pilot = { id, t0: Date.now(), manual: false, mode: "normal", lastInput: Date.now() };
  $("pilot").hidden = false; $("pilot-title").textContent = `🎮 Manual control · Drone ${id}`;
  $("pilot-title").style.color = color(id);
  $("pilot-cam").src = camSrc(id, "normal"); $("pilot-mode").textContent = "Thermal";
  pilotTimer = setInterval(pilotTick, 100);   // 10 Hz stick updates
}
function endPilot(msg) {
  clearInterval(pilotTimer); pilot = null; keys.clear();
  $("pilot").hidden = true; $("pilot-cam").src = "";
  if (msg) toast(msg, 5000);
}
function axis(v) { return Math.abs(v) < .15 ? 0 : v; }
function pilotTick() {
  const k = c => keys.has(c) ? 1 : 0, pad = (navigator.getGamepads ? [...navigator.getGamepads()] : []).find(g => g);
  const ax = i => pad ? axis(pad.axes[i] || 0) : 0, clamp = v => Math.max(-1, Math.min(1, v));
  const c = { drone: pilot.id,
    vx: clamp(k("w") - k("s") - sticks.move[1] - ax(1)), vy: clamp(k("a") - k("d") - sticks.move[0] - ax(0)),
    vz: clamp(k("r") - k("f") - sticks.alt[1] - ax(3)), yaw_rate: clamp(k("q") - k("e") - sticks.alt[0] - ax(2)) };
  if (c.vx || c.vy || c.vz || c.yaw_rate) { pilot.lastInput = Date.now(); api("control", c, true); }
}
function renderPilot(st) {
  const d = st.drones[pilot.id];
  if (d.state === "MANUAL") pilot.manual = true;
  else if (pilot.manual) {
    const why = [...st.events].reverse().find(e => e.msg.includes(`Drone ${pilot.id} `));
    return endPilot(`Drone ${pilot.id} is autonomous again${why ? ": " + esc(why.msg) : ""}`);
  } else if (Date.now() - pilot.t0 > 6000) {
    const why = [...st.events].reverse().find(e => e.lvl === "warn" && e.msg.includes(`Drone ${pilot.id}`));
    return endPilot(`Takeover not accepted${why ? ": " + esc(why.msg) : ""}`);
  }
  const idle = (Date.now() - pilot.lastInput) / 1000;
  $("pilot-status").textContent = `${pilot.manual ? "You are flying" : "Waiting for the drone…"} · 🔋${d.battery.toFixed(0)}% · ${signal(d)}`
    + (pilot.manual && idle > .5 ? ` · auto-release in ${Math.max(0, 2 - idle).toFixed(1)} s without input` : "");
  $("pilot-warn").hidden = !d.blocked;
}
$("pilot-release").onclick = () => { if (pilot) api("command", { type: "release", drone: pilot.id }, true); endPilot("Control released: drone back to autonomous mode"); };
$("pilot-snap").onclick = () => pilot && api("command", { type: "snapshot", drone: pilot.id });
$("pilot-mode").onclick = () => {
  pilot.mode = pilot.mode === "thermal" ? "normal" : "thermal";
  $("pilot-cam").src = camSrc(pilot.id, pilot.mode); $("pilot-mode").textContent = pilot.mode === "thermal" ? "Normal" : "Thermal";
};
addEventListener("keydown", e => { if (pilot && !e.target.matches("input")) keys.add(e.key.toLowerCase()); });
addEventListener("keyup", e => keys.delete(e.key.toLowerCase()));
addEventListener("blur", () => keys.clear());
for (const [name, el] of [["move", $("stick-move")], ["alt", $("stick-alt")]]) {   // touch / mouse joysticks
  const knob = el.querySelector("i");
  const set = e => {
    const r = el.getBoundingClientRect(), R = r.width / 2;
    let x = (e.clientX - r.left - R) / R, y = (e.clientY - r.top - R) / R;
    const n = Math.hypot(x, y); if (n > 1) { x /= n; y /= n; }
    sticks[name] = [x, y]; knob.style.transform = `translate(${x * R * .6}px,${y * R * .6}px)`;
  };
  el.onpointerdown = e => { el.setPointerCapture(e.pointerId); set(e); };
  el.onpointermove = e => { if (el.hasPointerCapture(e.pointerId)) set(e); };
  el.onpointerup = el.onpointercancel = () => { sticks[name] = [0, 0]; knob.style.transform = ""; };
}

// ---------------- place search, rescue base, drawing the area, launch ----------------
$("search").onsubmit = async e => {
  e.preventDefault();
  const q = $("q").value.trim(); if (!q) return;
  try {
    const r = await fetch("https://nominatim.openstreetmap.org/search?format=json&limit=5&q=" + encodeURIComponent(q));
    const res = await r.json();
    $("results").innerHTML = res.map((p, i) => `<div data-i="${i}">${esc(p.display_name)}</div>`).join("") || "<div>No place found</div>";
    $("results").onclick = ev => {
      const p = res[ev.target.closest("[data-i]")?.dataset.i]; if (!p) return;
      const b = p.boundingbox.map(Number);
      map.flyToBounds([[b[0], b[2]], [b[1], b[3]]], { maxZoom: 17 });
      $("results").innerHTML = "";
    };
  } catch (err) { $("results").innerHTML = "<div>Search needs an internet connection</div>"; }
};

let placing = false, draw = null, area = null;
const drawStyle = { color: "#ffd166", weight: 2, dashArray: "6 4", fillOpacity: .08 };
$("btn-base").onclick = () => { placing = true; map.getContainer().classList.add("drawing"); toast("Click the map to place the rescue base"); };
$("btn-mission").onclick = () => { $("launch").hidden = false; };
$("launch-cancel").onclick = () => { $("launch").hidden = true; draw = area = null; G.draw.clearLayers(); map.getContainer().classList.remove("drawing"); map.doubleClickZoom.enable(); };
document.querySelectorAll("[data-draw]").forEach(b => b.onclick = () => {
  G.draw.clearLayers(); area = null; draw = { kind: b.dataset.draw, pts: [] };
  map.getContainer().classList.add("drawing"); map.doubleClickZoom.disable();
  $("draw-finish").hidden = draw.kind !== "poly"; $("launch-go").disabled = true;
  $("launch-hint").textContent = draw.kind === "rect" ? "Click two opposite corners of the area." : "Click each corner, then ✔ Finish (or double-click).";
});
const shape = pts => draw.kind === "rect" ? L.rectangle(L.latLngBounds(pts), drawStyle) : L.polygon(pts, drawStyle);
function preview(cur) {
  G.draw.clearLayers();
  const pts = cur ? [...draw.pts, cur] : draw.pts;
  if (pts.length > 1) shape(pts).addTo(G.draw);
  draw.pts.forEach(p => L.circleMarker(p, { radius: 4, color: "#ffd166" }).addTo(G.draw));
}
function areaM2(pts) {   // shoelace on a local flat projection
  const k = 111320 * Math.cos(pts[0].lat * Math.PI / 180);
  return Math.abs(pts.reduce((s, p, i) => { const q = pts[(i + 1) % pts.length];
    return s + (p.lng * k) * (q.lat * 111320) - (q.lng * k) * (p.lat * 111320); }, 0)) / 2;
}
function finishDraw() {
  const pts = draw.pts.filter((p, i) => !i || !p.equals(draw.pts[i - 1]));
  if (draw.kind === "rect" ? pts.length < 2 : pts.length < 3) return toast("Add more corners first");
  const b = L.latLngBounds(pts);
  area = draw.kind === "rect" ? [b.getSouthWest(), b.getSouthEast(), b.getNorthEast(), b.getNorthWest()] : pts;
  draw = null; G.draw.clearLayers(); L.polygon(area, drawStyle).addTo(G.draw);
  map.getContainer().classList.remove("drawing"); map.doubleClickZoom.enable();
  $("draw-finish").hidden = true; $("launch-go").disabled = false;
  const m2 = areaM2(area);
  $("launch-hint").textContent = `Area: ${m2 > 1e6 ? (m2 / 1e6).toFixed(2) + " km²" : Math.round(m2).toLocaleString() + " m²"}. Choose drones and teams, then Launch.`;
}
$("draw-finish").onclick = () => draw && finishDraw();
map.on("click", e => {
  if (placing) {
    placing = false; map.getContainer().classList.remove("drawing");
    if (baseMarker) baseMarker.remove();
    baseMarker = L.marker(e.latlng, { draggable: true, icon: L.divIcon({ className: "lbl", html: "🚑 BASE", iconSize: [60, 14] }) }).addTo(map);
    return toast("Rescue base placed (drag to move). The swarm launches from its nearest edge of the area.");
  }
  if (!draw) return;
  draw.pts.push(e.latlng);
  if (draw.kind === "rect" && draw.pts.length === 2) finishDraw(); else preview();
});
map.on("mousemove", e => draw && draw.pts.length && preview(e.latlng));
map.on("dblclick", () => draw && draw.kind === "poly" && finishDraw());
$("launch-go").onclick = async () => {
  try {
    const r = await fetch("/api/mission", { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify({
      area: area.map(p => [p.lat, p.lng]), base: baseMarker ? [baseMarker.getLatLng().lat, baseMarker.getLatLng().lng] : null,
      drones: +$("n-drones").value, teams: +$("n-teams").value }) });
    const j = await r.json();
    if (!j.ok) return toast("⚠ " + esc(j.error));
  } catch (e) { return toast("⚠ Dashboard server not reachable (run python dashboard/serve.py)"); }
  $("launch").hidden = true; area = null;
  stopReplay(); S = null; ST = null; lastT = -1; lastAck = lastSnap = 0; summaryShown = sawRunning = primed = false; seenAlerts.clear();
  setMode("🚀 LAUNCHING…", "live");
  toast("🚀 Swarm launching: the simulation is fitting the search area…", 5000);
};

// ---------------- mission summary ----------------
function showSummary(st) {
  const H = humans(st), reached = H.filter(v => v.reached_t).length, verified = st.victims.filter(v => v.photo && v.confidence < .5).length;
  const unsafe = Object.values(st.routes).filter(r => r.direct.length).length;
  openModal(`<h2 style="margin:0 0 8px">✅ Mission summary</h2><div class="kpis" style="flex-wrap:wrap">
    <div class="kpi"><b>${H.length} / ${S.n_victims}</b><span>Victims found</span></div><div class="kpi"><b>${reached}</b><span>Reached by teams</span></div>
    <div class="kpi"><b>${(st.t / 60).toFixed(1)} min</b><span>Mission time</span></div>
    <div class="kpi"><b>${S.area_m2 > 1e6 ? (S.area_m2 / 1e6).toFixed(2) + " km²" : S.area_m2.toLocaleString() + " m²"}</b><span>Area · ${st.coverage.toFixed(0)}% searched</span></div>
    <div class="kpi"><b>${st.sos.length + verified + unsafe}</b><span>Alerts</span></div></div>
    <p class="vmeta">${st.sos.length} drone SOS · ${verified} photos sent to a rescuer to verify · ${unsafe} unsafe direct routes rerouted ·
    ${S.n_drones} drones · ${S.n_teams} rescue teams · ${H.filter(v => v.source === "phone").length} buried victims found by phone signal</p>`);
}

// ---------------- data: live polling or replay ----------------
function setMode(text, cls) { const m = $("mode"); m.textContent = text; m.className = "pill " + cls; }
const staticKey = s => JSON.stringify([s.origin, s.poly, s.base, s.n_drones, s.map_scale]);
let progressAt = Date.now();
async function pollLive() {
  if (mode !== "live") return;
  try {
    const j = await (await fetch(DATA + "live.json?_=" + Date.now())).json();
    if (!S || staticKey(S) !== staticKey(j.static)) { S = j.static; lastT = -1; drawStatic(); }
    if (!ST || j.state.t !== ST.t) progressAt = Date.now();
    render(j.state);
    setMode(j.state.final ? "MISSION COMPLETE" : Date.now() - progressAt > 5000 ? "⏸ SIMULATION NOT RUNNING" : "● LIVE", "live");
  } catch (e) { if (!$("mode").textContent.startsWith("🚀")) setMode(S ? "WAITING FOR SIMULATION…" : "NO MISSION · draw an area", "live"); }
  setTimeout(pollLive, 500);
}
let replayTimer = 0;
async function startReplay() {
  const j = await (await fetch(DATA + "replay.json?_=" + Date.now())).json();
  if (!$("camwall").hidden) closeWall();
  if (pilot) endPilot();
  mode = "replay"; frames = j.frames; idx = 0; playing = true; lastT = -1; summaryShown = sawRunning = primed = false; seenAlerts.clear();
  S = j.static; drawStatic();
  setMode("REPLAY", "replay");
  ["btn-play", "speed", "scrub"].forEach(id => $(id).hidden = false);
  $("btn-replay").textContent = "Live";
  $("scrub").max = frames.length - 1;
  render(frames[0]); tick();
}
function stopReplay() {
  clearTimeout(replayTimer);
  ["btn-play", "speed", "scrub"].forEach(id => $(id).hidden = true);
  $("btn-replay").textContent = "Replay";
  if (mode !== "live") { mode = "live"; S = null; lastT = -1; pollLive(); }
}
function tick() {
  if (mode !== "replay") return;
  if (playing && idx < frames.length - 1) { idx++; render(frames[idx]); $("scrub").value = idx; }
  replayTimer = setTimeout(tick, 500 / +$("speed").value);
}
$("scrub").oninput = e => { idx = +e.target.value; lastT = -1; render(frames[idx]); };
$("btn-play").onclick = () => { playing = !playing; $("btn-play").textContent = playing ? "⏸" : "▶"; };
$("btn-replay").onclick = () => mode === "replay" ? stopReplay() : startReplay().catch(() => toast("No replay yet: run a mission first."));

// ---------------- phone layout: bottom tabs ----------------
function syncTabs() { document.querySelectorAll("#tabs button").forEach(b => b.classList.toggle("on", b.dataset.tab === document.body.dataset.tab)); }
$("tabs").onclick = e => {
  const b = e.target.closest("button"); if (!b) return;
  if (b.dataset.tab === "cams") return openWall();
  if (document.body.dataset.tab === b.dataset.tab) document.body.removeAttribute("data-tab"); else document.body.dataset.tab = b.dataset.tab;
  syncTabs();
};

if ("serviceWorker" in navigator) navigator.serviceWorker.register("sw.js").catch(() => { /* needs localhost or HTTPS */ });
if (new URLSearchParams(location.search).get("mode") === "replay") startReplay().catch(() => pollLive()); else pollLive();
