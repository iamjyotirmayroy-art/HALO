"use strict";
// HALO Command: rescuer dashboard. Map + place search, mission launch, live swarm, YOLOv8-verified victims,
// fleet, camera wall, manual takeover, alerts, layers and replay. Data comes from the local simulation.
const DATA = "../simulation/results/";
const COLORS = ["#4f8ef7", "#f5a524", "#a66cf0", "#22c58b", "#f0574a", "#22b8cf"];
const $ = id => document.getElementById(id);
const color = i => COLORS[i % COLORS.length];
const rgba = (hex, a) => `rgba(${[1, 3, 5].map(k => parseInt(hex.slice(k, k + 2), 16))},${a})`;
const esc = s => String(s ?? "").replace(/[&<>"]/g, c => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;" })[c]);
const icon = n => `<svg><use href="#i-${n}"/></svg>`;
const clock = t => `${String(Math.floor(t / 60)).padStart(2, "0")}:${String(Math.floor(t % 60)).padStart(2, "0")}`;

let S = null, ST = null, frames = null, idx = 0, playing = true, mode = "live", lastT = -1, lastAck = 0, lastSnap = 0;
let vfilter = "all", allOnline = true, wallKey = "", pilot = null, summaryShown = false, sawRunning = false, primed = false;
let routesKey = "", drawer = null, drawerKey = "";
const watch = new Set(), trailOff = new Set(), camMode = {}, seenAlerts = new Set(), trailLines = {};
let muted = false;
try { muted = localStorage.getItem("halo-muted") === "1"; } catch (e) { /* storage blocked: sound stays on */ }

// ---------------- map ----------------
const map = L.map("map", { maxZoom: 21, zoomSnap: 0.25, zoomControl: false }).setView([22.5, 79], 5);   // opens on India
L.control.zoom({ position: "bottomright" }).addTo(map);
const tiles = {
  "Dark": L.tileLayer("https://{s}.basemaps.cartocdn.com/dark_all/{z}/{x}/{y}{r}.png",
    { maxZoom: 21, maxNativeZoom: 19, attribution: "© OpenStreetMap contributors © CARTO" }),
  "Streets": L.tileLayer("https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png",
    { maxZoom: 21, maxNativeZoom: 19, attribution: "© OpenStreetMap contributors" }),
  "Satellite": L.tileLayer("https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}",
    { maxZoom: 21, maxNativeZoom: 19, attribution: "Imagery © Esri" }),
};
tiles.Dark.addTo(map);
map.createPane("grid").style.zIndex = 350;   // grid images (hazards, zones) stay under routes, trails and heat zones
const G = Object.fromEntries(["base", "hazards", "zones", "routes", "links", "trails", "heat", "dyn", "draw"].map(k => [k, L.layerGroup().addTo(map)]));
L.control.layers(tiles, { "Hazards": G.hazards, "Zones & searched area": G.zones, "Rescue routes": G.routes,
  "Radio links": G.links, "Drone trails": G.trails, "Thermal zones": G.heat }, { position: "topright" }).addTo(map);
const legend = L.control({ position: "bottomleft" });
legend.onAdd = () => Object.assign(L.DomUtil.create("div", "legend"), { innerHTML:
  '<i style="background:#8b95a3"></i>Building<br><i style="background:#ff8c1a"></i>Unstable structure<br>' +
  '<i style="background:#ffd400"></i>Radiation<br><i style="background:#8a6a4a"></i>Rubble<br>' +
  '<i style="background:#22c58b"></i>Safe route<br><i style="background:#f04438"></i>Unsafe direct route<br>' +
  '<i style="background:#ff8a1f;transform:rotate(45deg) scale(.8)"></i>Rescue team' });

function ll(x, y) {   // sim metres -> lat/lon (the simulated square is fitted onto the drawn area)
  const [lat0, lon0] = S.origin, k = S.map_scale;
  return [lat0 + y * k / 111320, lon0 + x * k / (111320 * Math.cos(lat0 * Math.PI / 180))];
}
const fmtLL = (x, y) => ll(x, y).map(v => v.toFixed(5)).join(", ");
const gridCanvas = document.createElement("canvas");
function gridImage(paint) {   // one pixel per grid cell, north up, stretched over the area as an image overlay
  const n = S.n, g = gridCanvas.getContext("2d");
  gridCanvas.width = gridCanvas.height = n;
  for (let i = 0; i < n; i++) for (let j = 0; j < n; j++) {
    const c = paint(i, j);
    if (c) { g.fillStyle = c; g.fillRect(i, n - 1 - j, 1, 1); }
  }
  return gridCanvas.toDataURL();
}
const gridOverlay = (url, group) => L.imageOverlay(url, [ll(0, 0), ll(S.area, S.area)], { className: "px", interactive: false, pane: "grid" }).addTo(group);
const label = (xy, html, group, w = 90) => L.marker(ll(...xy), { interactive: false,
  icon: L.divIcon({ className: "lbl", html, iconSize: [w, 14], iconAnchor: [w / 2, 7] }) }).addTo(group);

let zoneOverlay = null, baseMarker = null;
function drawStatic() {
  Object.values(G).forEach(g => g.clearLayers());
  Object.keys(trailLines).forEach(k => delete trailLines[k]);
  zoneOverlay = null; routesKey = "";
  const cells = a => new Set(a.map(String)), tall = cells(S.tall), weak = cells(S.weak), rubble = cells(S.rubble);
  gridOverlay(gridImage((i, j) => tall.has(i + "," + j) ? "rgba(139,149,163,.8)" : weak.has(i + "," + j) ? "rgba(255,140,26,.42)"
    : rubble.has(i + "," + j) ? "rgba(138,106,74,.35)" : S.radiation[i][j] > .15 ? `rgba(255,212,0,${Math.min(.6, S.radiation[i][j] / 3 * .65)})` : null), G.hazards);
  L.circle(ll(...S.dead_zone.c), { radius: S.dead_zone.r * S.map_scale, color: "#a78bfa", weight: 1.2, dashArray: "4 4", fillOpacity: .04 })
    .bindTooltip("No radio signal (debris shadow)").addTo(G.hazards);
  const flat = S.radiation.flat(), r = flat.indexOf(Math.max(...flat));
  label([(Math.floor(r / S.n) + .5) * S.cell, (r % S.n + .5) * S.cell], "RADIATION", G.hazards);
  const outline = (S.poly || [[0, 0], [S.area, 0], [S.area, S.area], [0, S.area]]).map(p => ll(...p));
  L.polygon(outline, { color: "#e7ebf0", weight: 1.5, fill: false, dashArray: "6 5", interactive: false }).addTo(G.base);
  S.pads.forEach(p => L.circleMarker(ll(...p), { radius: 3.5, color: "#3d8bfd", weight: 2, fillOpacity: .5, interactive: false }).addTo(G.base));
  label(S.base, "BASE", G.base, 40);
  if (baseMarker) { baseMarker.remove(); baseMarker = null; }
  map.fitBounds(L.latLngBounds([...outline, ll(...S.base)]), { padding: [40, 40] });
  legend.addTo(map);
}

// ---------------- status helpers ----------------
const humans = st => st.victims.filter(v => !v.status.startsWith("DISMISSED"));
const isLive = () => mode === "live" && ST && !ST.final;
const signal = d => d.state === "TRAPPED" ? "SOS beacon" : !d.online ? "Offline · autonomous" : d.hops > 1 ? `Mesh relay ×${d.hops - 1}` : "Online";
const linkIcon = d => `<span class="link ${d.state === "TRAPPED" ? "sos" : d.online ? "" : "off"}" title="${signal(d)}">${icon(d.state === "TRAPPED" ? "sos" : !d.online ? "signal-off" : d.hops > 1 ? "mesh" : "signal")}</span>`;
const STATE_CHIP = { SEARCH: "info", INVESTIGATE: "warn", MANUAL: "verify", TRAPPED: "crit" };
const STATE_TXT = { SEARCH: "Searching", INVESTIGATE: "Inspecting", MANUAL: "Manual", TRAPPED: "Trapped", RETURN: "Returning", CHARGING: "Charging", DONE: "Landed", IDLE: "Idle" };
const stateChip = s => `<span class="chip ${STATE_CHIP[s] || ""}">${STATE_TXT[s] || s}</span>`;
const battColor = b => b > 40 ? "#17b26a" : b > 20 ? "#f79009" : "#f04438";
const batt = b => `<span class="batt"><i><i style="width:${b}%;background:${battColor(b)}"></i></i>${b.toFixed(0)}%</span>`;
const prioClass = p => p >= 70 ? "p-high" : p >= 50 ? "p-mid" : "p-low";
const heading = d => ((450 - d.heading * 180 / Math.PI) % 360).toFixed(0);
function victimColor(v) {
  if (v.status.startsWith("DISMISSED")) return "#5b6573";
  if (v.reached_t) return "#3d8bfd";
  if (v.status.includes("verify")) return "#fdb022";
  return v.source === "phone" ? "#e879f9" : "#17b26a";
}
function victimChip(v) {
  if (v.status.startsWith("DISMISSED")) return '<span class="chip">Dismissed</span>';
  if (v.reached_t) return '<span class="chip ok">Rescued</span>';
  if (v.team) return '<span class="chip info">Team en route</span>';
  if (v.status.includes("verify")) return '<span class="chip verify">Verify</span>';
  return v.source === "phone" ? '<span class="chip mesh">Buried</span>' : '<span class="chip crit">Confirmed</span>';
}
const aiLine = v => v.source === "phone" ? "Phone signal only · likely buried" : `${esc(v.model || "AI")} ${Math.round(v.confidence * 100)}% · ${v.temp ?? "–"}°C`;

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
  $("k-time").textContent = clock(st.t);
  $("k-cov").textContent = st.coverage.toFixed(0) + "%";
  $("k-cov-bar").style.width = st.coverage + "%";
  $("k-found").textContent = `${H.filter(v => v.source === "phone" || !v.status.startsWith("POSSIBLE")).length} / ${S.n_victims}`;
  $("k-reached").textContent = st.victims.filter(v => v.reached_t).length;
  $("k-online").textContent = `${st.drones.filter(d => d.online && d.state !== "TRAPPED").length} / ${st.drones.length}`;
  $("k-teams").textContent = `${st.teams.filter(t => t.state !== "READY").length} / ${st.teams.length}`;
  if (mode === "replay") $("scrub-t").textContent = st.t.toFixed(1) + " s";

  // zones in each drone's colour, filled in as they are searched
  const url = gridImage((i, j) => {
    const k = i * S.n + j, z = st.zones ? st.zones[k] : ".", cov = st.covered[k] === "1";
    return z !== "." ? rgba(color(z.charCodeAt(0) - 48), cov ? .26 : .09) : cov ? "rgba(61,139,253,.12)" : null;
  });
  if (zoneOverlay) zoneOverlay.setUrl(url); else zoneOverlay = gridOverlay(url, G.zones);

  const rk = JSON.stringify(st.routes);
  if (rk !== routesKey) {
    routesKey = rk;
    G.routes.clearLayers();
    Object.entries(st.routes).forEach(([vid, r], k) => {
      if (r.direct.length) {
        L.polyline(r.direct.map(p => ll(...p)), { color: "#f04438", weight: 2, dashArray: "5 6", opacity: .85 })
          .bindTooltip(`Direct route to ${vid} unsafe: ${esc(r.reason)}`).addTo(G.routes);
        label(r.direct[Math.floor(r.direct.length * (k % 2 ? .3 : .7))], `✕ ${esc(r.reason)}`, G.routes, 200);   // staggered so labels don't overlap
      }
      if (r.safe.length) L.polyline([ll(...S.base), ll(...S.entry), ...r.safe.map(p => ll(...p))], { color: "#22c58b", weight: 3, opacity: .85 })
        .bindTooltip(`Safe route to ${vid} · ${esc(r.status)}`).addTo(G.routes);
    });
  }

  G.heat.clearLayers();
  st.heat.forEach(h => {
    const v = st.victims.find(x => x.id === h.id), dismissed = v && v.status.startsWith("DISMISSED"), hot = Math.min(1, (h.temp - 25) / 15);
    [[1.0, .09], [0.65, .16], [0.38, .28], [0.16, .5]].forEach(([r, o]) => L.circle(ll(...h.pos), {
      radius: r * S.map_scale, stroke: false, interactive: false, className: h.checked ? "" : "sos-pulse",
      fillColor: dismissed ? "#5b6573" : r < .3 ? "#fff3b0" : r < .5 ? "#ffb000" : "#ff3d00",
      fillOpacity: dismissed ? o * .4 : o * (.6 + .4 * hot) }).addTo(G.heat));
    const tag = !h.checked || !v ? '<span style="color:#fdc660">AI checking…</span>'
      : dismissed ? '<span style="color:#9aa3ad">not a person</span>'
      : `<span style="color:#5fd49b">${esc(v.label.replace("body part: ", ""))} ${Math.round(v.confidence * 100)}%</span>`;
    L.marker(ll(h.pos[0], h.pos[1] - .75), { interactive: false, icon: L.divIcon({ className: "lbl", iconSize: [150, 26], iconAnchor: [75, 0],
      html: `<div style="text-align:center">${h.temp.toFixed(1)}°C<br>${tag}</div>` }) }).addTo(G.heat);
  });

  G.links.clearLayers();
  st.links.forEach(([a, b]) => L.polyline([ll(...a), ll(...b)], { color: "#a78bfa", weight: 1, opacity: .55, interactive: false }).addTo(G.links));

  trailsOf(st).forEach((tr, i) => {
    const pts = trailOff.has(i) ? [] : tr.map(p => ll(p[0], p[1]));
    if (trailLines[i]) trailLines[i].setLatLngs(pts);
    else trailLines[i] = L.polyline(pts, { color: color(i), weight: 2, opacity: .85, interactive: false }).addTo(G.trails);
  });

  G.dyn.clearLayers();
  if (st.collapsed) { const b = S.weak_box;
    L.rectangle([ll(b[0], b[1]), ll(b[2], b[3])], { color: "#f04438", weight: 2, fillOpacity: .22 }).bindTooltip("Collapsed in aftershock").addTo(G.dyn); }
  st.victims.forEach(v => {
    L.marker(ll(...v.pos), { icon: L.divIcon({ className: "", iconSize: [16, 16], html: `<div class="vm" style="background:${victimColor(v)}"></div>` }) })
      .bindTooltip(`<b>${v.id}</b> · ${esc(v.label)}<br>${esc(v.status)} · P${v.priority}`).on("click", () => openVictim(v.id)).addTo(G.dyn);
    label([v.pos[0] + .32, v.pos[1] + .3], v.id, G.dyn, 30);
  });
  st.sos.forEach(s => L.circleMarker(ll(...s.pos), { radius: 16, color: "#f04438", weight: 3, fill: false, className: "sos-pulse" })
    .bindTooltip(`SOS · drone ${s.drone} trapped`).addTo(G.dyn));
  st.teams.forEach(t => L.marker(ll(...t.pos), { icon: L.divIcon({ className: "", iconSize: [13, 13], html: '<div class="tm"></div>' }) })
    .bindTooltip(`Team ${t.name} · ${t.state.replace("_", " ").toLowerCase()}${t.victim ? " → " + t.victim : ""}`).addTo(G.dyn));
  st.drones.forEach(d => L.marker(ll(d.pos[0], d.pos[1]), { icon: L.divIcon({ className: "", iconSize: [28, 28], html:
    `<div class="dm ${d.online ? "" : "off"} ${d.state === "MANUAL" ? "manual" : ""}"><svg viewBox="0 0 28 28" style="transform:rotate(${90 - d.heading * 180 / Math.PI}deg)">` +
    `<circle cx="14" cy="15" r="8" fill="${color(d.id)}"/><path d="M14 2.5 18.5 9h-9z" fill="#fff" stroke="none"/></svg><b>D${d.id}</b></div>` }) })
    .bindTooltip(`Drone ${d.id} · ${STATE_TXT[d.state] || d.state} · ${d.battery.toFixed(0)}% · ${signal(d)}`).on("click", () => openDrone(d.id)).addTo(G.dyn));

  renderAlerts(st);
  renderFunnel(H);
  $("teams").innerHTML = st.teams.map(t => `<div class="row">${icon("team")}<div class="grow"><div class="title">Team ${t.name}</div>
    <div class="sub">${t.victim ? "Assigned " + t.victim : "At base"} · rescued ${t.rescued.length}</div></div>
    <span class="chip ${t.state === "READY" ? "" : t.state === "ON_SITE" ? "ok" : "info"}">${t.state.replace("_", " ")}</span></div>`).join("");
  renderVictims();
  $("drones").innerHTML = st.drones.map(d => `<div class="row click" data-drone="${d.id}"><span class="dot" style="background:${color(d.id)}"></span>
    <div class="grow"><div class="title">Drone ${d.id}${d.recalled ? " · recalled" : ""}</div><div class="sub">${signal(d)}</div></div>
    ${linkIcon(d)}${batt(d.battery)}${stateChip(d.state)}</div>`).join("");
  $("c-fleet").textContent = st.drones.length;
  $("feed").innerHTML = [...st.events].reverse().map(e => `<div class="lvl-${e.lvl}"><time>${clock(e.t)}</time><span>${esc(e.msg)}</span></div>`).join("");

  if (drawer) refreshDrawer();
  if (mode === "live") liveFeedback(st);
  sawRunning ||= !st.final;
  if (st.final && sawRunning && !summaryShown) { summaryShown = true; showSummary(st); }   // only after watching it run
}

function renderFunnel(H) {
  const n = Math.max(S.n_victims, 1), steps = [
    ["Detected", H.length], ["Verified", H.filter(v => !v.status.includes("verify")).length],
    ["Dispatched", H.filter(v => v.team || v.reached_t).length], ["Rescued", H.filter(v => v.reached_t).length]];
  $("funnel").innerHTML = steps.map(([k, c]) => `<div><b>${c}</b><span>${k}</span><div class="bar"><i style="width:${100 * c / n}%"></i></div></div>`).join("");
}

function renderVictims() {
  const keep = { all: () => true, open: v => !v.reached_t && !v.status.startsWith("DISMISSED"),
    verify: v => v.status.includes("verify") && !v.reached_t, buried: v => v.source === "phone" }[vfilter];
  const dis = v => v.status.startsWith("DISMISSED") ? 1 : 0;
  const vs = ST.victims.filter(keep).sort((a, b) => dis(a) - dis(b) || (a.reached_t ? 1 : 0) - (b.reached_t ? 1 : 0) || b.priority - a.priority);
  $("c-victims").textContent = humans(ST).length;
  $("victims").innerHTML = vs.map(v => `<div class="row click ${dis(v) ? "dim" : ""}" data-victim="${v.id}">
    <span class="prio ${prioClass(v.priority)}">${v.priority}</span>
    <span class="thumbs">${v.photo ? `<img src="${DATA + v.thermal}" alt="" loading="lazy"><img src="${DATA + v.photo}" alt="" loading="lazy">` : `<span>${icon("signal")}</span>`}</span>
    <div class="grow"><div class="title">${v.id} · ${esc(v.label)}</div><div class="sub">${aiLine(v)}${v.team ? " · team " + v.team : ""}</div></div>
    ${victimChip(v)}</div>`).join("") || `<p class="empty">${ST.victims.length ? "None in this filter" : "Searching…"}</p>`;
}

// ---------------- alerts: banner + sound for new ones ----------------
function renderAlerts(st) {
  const A = [];
  st.sos.forEach(s => A.push({ key: "sos" + s.drone, cls: "", ic: "sos", title: `SOS · Drone ${s.drone} trapped`, pos: s.pos,
    text: `Last position ${fmtLL(...s.pos)} at ${clock(s.t)} · beacon active · zone re-assigned` }));
  st.heat.filter(h => !h.checked).forEach(h => A.push({ key: "heat" + h.id, cls: "heat", ic: "flame", pos: h.pos,
    title: `Heat signal ${h.temp.toFixed(1)}°C`, text: `Drone ${h.drone} photographing · AI checking for a person` }));
  st.victims.filter(v => v.status.includes("verify") && v.photo && !v.reached_t).forEach(v => A.push({ key: "ver" + v.id, cls: "verify", ic: "eye", victim: v.id,
    title: `Verify ${v.id} · ${esc(v.label)}`, text: `${esc(v.model || "AI")} ${Math.round(v.confidence * 100)}% · open the photo and decide` }));
  Object.entries(st.routes).filter(([, r]) => r.direct.length).forEach(([vid, r]) => A.push({ key: "route" + vid, cls: "route", ic: "route",
    title: `Unsafe direct route to ${vid}`, text: `${esc(r.reason)} · team rerouted on a safe path` }));
  $("c-alerts").textContent = A.length;
  $("alerts").innerHTML = A.map(a => `<div class="row alert ${a.cls} ${a.victim || a.pos ? "click" : ""}" ${a.victim ? `data-victim="${a.victim}"` : a.pos ? `data-pan="${a.pos}"` : ""}>
    ${icon(a.ic)}<div class="grow"><div class="title">${a.title}</div><div class="sub">${a.text}</div></div></div>`).join("") || '<p class="empty">No active alerts</p>';
  const fresh = A.filter(a => !seenAlerts.has(a.key));
  fresh.forEach(a => seenAlerts.add(a.key));
  if (primed && fresh.length) {
    const a = fresh.find(x => x.ic === "sos") || fresh[0];
    banner(a);
    if (mode === "live") beep(a.ic === "sos");
  }
  primed = true;
}
let bannerTimer = 0, audio = null;
function banner(a) {
  const b = $("banner");
  b.innerHTML = icon(a.ic) + `<span>${a.title}</span>`; b.className = "banner " + a.cls; b.hidden = false;
  clearTimeout(bannerTimer); bannerTimer = setTimeout(() => b.hidden = true, 6000);
}
function beep(urgent) {
  if (muted) return;
  try {
    audio = audio || new AudioContext();
    (urgent ? [0, .22, .44] : [0]).forEach(t0 => {
      const o = audio.createOscillator(), g = audio.createGain(), t = audio.currentTime + t0;
      o.frequency.value = urgent ? 880 : 660; g.gain.setValueAtTime(.16, t); g.gain.exponentialRampToValueAtTime(.001, t + .18);
      o.connect(g).connect(audio.destination); o.start(t); o.stop(t + .18);
    });
  } catch (e) { /* audio unavailable: banner only */ }
}
function setMute(m) {
  muted = m; $("btn-mute").innerHTML = icon(m ? "bell-off" : "bell");
  try { localStorage.setItem("halo-muted", m ? "1" : "0"); } catch (e) { /* not persisted */ }
}
$("btn-mute").onclick = () => setMute(!muted);
setMute(muted);

let toastTimer = 0;
function toast(html, ms = 3500) {
  const t = $("toast"); t.innerHTML = html; t.hidden = false;
  clearTimeout(toastTimer); toastTimer = setTimeout(() => t.hidden = true, ms);
}
function liveFeedback(st) {   // acknowledgements and snapshot results from the swarm
  if (st.ack > lastAck) { if (lastAck) toast("Command carried out by the swarm"); lastAck = st.ack; }
  const s = st.snapshots[st.snapshots.length - 1];
  if (s && s.t !== lastSnap) {
    if (lastSnap) toast(`<b>Snapshot · Drone ${s.drone}</b><br><span style="color:var(--text-2)">${esc(s.text)}</span><img src="${DATA + s.img}?${s.t}" alt="">`, 7000);
    lastSnap = s.t;
  }
}

// ---------------- commands to the swarm ----------------
async function api(path, body, quiet) {
  if (!isLive()) { if (!quiet) toast("Commands only work during a live mission"); return null; }
  try {
    const r = await fetch("/api/" + path, { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(body) });
    const j = await r.json();
    if (!quiet) toast(j.ok ? "Sent to the swarm…" : "Error: " + esc(j.error));
    return j;
  } catch (e) { if (!quiet) toast("Dashboard server not reachable"); return null; }
}

// ---------------- detail drawer (victim / drone) ----------------
function openDrawer(kind, id) { drawer = { kind, id }; drawerKey = ""; $("drawer").hidden = false; refreshDrawer(); }
function closeDrawer() { drawer = null; $("drawer").hidden = true; }
$("drawer-back").onclick = closeDrawer;
function refreshDrawer() {
  const obj = drawer.kind === "victim" ? ST.victims.find(v => v.id === drawer.id) : ST.drones[drawer.id];
  if (!obj) return;
  const key = JSON.stringify([obj, trailOff.has(drawer.id)]);
  if (key === drawerKey) return;   // unchanged: keep scroll position and images
  drawerKey = key;
  (drawer.kind === "victim" ? victimDrawer : droneDrawer)(obj);
}
function victimDrawer(v) {
  const open = !v.reached_t && !v.team, pct = Math.round(v.confidence * 100), cc = v.confidence >= .6 ? "#17b26a" : v.confidence >= .3 ? "#f79009" : "#f04438";
  $("drawer-title").innerHTML = `${v.id} · ${esc(v.label)}`;
  $("drawer-body").innerHTML = (v.photo ? `<div class="pair"><figure><img src="${DATA + v.thermal}" alt="Thermal image"><figcaption>Thermal camera</figcaption></figure>
      <figure><img src="${DATA + v.photo}" alt="Drone photo"><figcaption>Drone photo · detections</figcaption></figure></div>
      <div class="ai"><header>${icon("cpu")}AI verification<span class="chip">${esc(v.model || "AI")}</span></header>
      <div style="display:flex;justify-content:space-between"><span>${esc(v.label)}</span><b class="mono">${pct}%</b></div>
      <div class="conf"><i style="width:${pct}%;background:${cc}"></i></div>
      ${v.detections && v.detections.length ? `<div class="dets">${v.detections.map(d => `<span class="chip">${esc(d.label)} ${Math.round(d.conf * 100)}%</span>`).join("")}</div>`
        : '<p class="note">No person detected in the photo. The heat signature is kept, never discarded: a rescuer decides.</p>'}</div>`
    : `<div class="ai"><header>${icon("signal")}Phone signal only</header><p class="note">No thermal or visual contact: located from the victim's phone signal. Likely buried.</p></div>`) +
    `<dl class="kv"><dt>Status</dt><dd>${victimChip(v)} <span style="color:var(--text-2)">${esc(v.status)}</span></dd>
     <dt>Priority</dt><dd><b class="mono">${v.priority}</b> / 100</dd>
     ${v.temp != null ? `<dt>Temperature</dt><dd class="mono">${v.temp}°C</dd>` : ""}
     <dt>Trapped</dt><dd>${v.trapped ? "Yes" : "No"}</dd><dt>Detected</dt><dd class="mono">T+${clock(v.t)}</dd>
     <dt>Location</dt><dd class="mono">${fmtLL(...v.pos)}</dd>
     <dt>Route</dt><dd>${esc(v.route_status || "Pending")}${v.route_note ? `<p class="note">${esc(v.route_note)}</p>` : ""}</dd>
     <dt>Team</dt><dd>${v.team ? `${v.team} · ${v.reached_t ? `reached at T+${clock(v.reached_t)} (${v.response_s}s after detection)` : "on the way"}` : "Not assigned"}</dd></dl>`;
  $("drawer-actions").innerHTML = (v.photo ? `<button data-act="confirm" data-id="${v.id}">${icon("check")}Person</button><button data-act="reject" data-id="${v.id}">${icon("x")}Not a person</button>` : "")
    + `<button class="primary" data-act="team" data-id="${v.id}" ${open ? "" : "disabled"}>${icon("team")}Dispatch team</button>`;
}
function droneDrawer(d) {
  $("drawer-title").innerHTML = `<span class="dot" style="background:${color(d.id)};display:inline-block;margin-right:8px"></span>Drone ${d.id}`;
  $("drawer-body").innerHTML = `<dl class="kv"><dt>State</dt><dd>${stateChip(d.state)}${d.recalled ? ' <span class="chip">Recalled</span>' : ""}</dd>
    <dt>Battery</dt><dd>${batt(d.battery)}</dd><dt>Link</dt><dd>${linkIcon(d)} ${signal(d)}</dd>
    <dt>Position</dt><dd class="mono">${fmtLL(d.pos[0], d.pos[1])}</dd><dt>Altitude</dt><dd class="mono">${d.pos[2].toFixed(2)} m (scale model)</dd>
    <dt>Heading</dt><dd class="mono">${heading(d)}°</dd></dl>
    <p class="note">Watching the camera never changes what the drone does. Manual control keeps the anti-crash safety on.</p>`;
  $("drawer-actions").innerHTML = `<button data-act="watch" data-id="${d.id}">${icon("video")}Camera</button><button data-act="control" data-id="${d.id}">${icon("gamepad")}Control</button>
    <button data-act="trail" data-id="${d.id}">${trailOff.has(d.id) ? "Show" : "Hide"} trail</button><button class="danger" data-act="recall" data-id="${d.id}">${icon("home")}Recall</button>`;
}
function openVictim(id) { const v = ST.victims.find(x => x.id === id); if (!v) return; openDrawer("victim", id); map.panTo(ll(...v.pos)); }
function openDrone(id) { const d = ST.drones[id]; openDrawer("drone", id); map.panTo(ll(d.pos[0], d.pos[1])); }

document.addEventListener("click", e => {
  const t = e.target.closest("[data-act],[data-victim],[data-drone],[data-pan]");
  if (!t) return;
  if (t.dataset.victim) return openVictim(t.dataset.victim);
  if (t.dataset.drone) return openDrone(+t.dataset.drone);
  if (t.dataset.pan) return map.flyTo(ll(...t.dataset.pan.split(",").map(Number)), Math.max(map.getZoom(), 18));
  const id = t.dataset.id, n = +id;
  ({
    confirm: () => api("verify", { victim: id, person: true }),
    reject: () => api("verify", { victim: id, person: false }),
    team: () => api("command", { type: "send_team", victim: id }),
    recall: () => api("command", { type: "recall", drone: n }),
    trail: () => { trailOff.has(n) ? trailOff.delete(n) : trailOff.add(n); lastT = -1; render(ST); },
    watch: () => { allOnline = false; watch.clear(); watch.add(n); openWall(); },
    control: () => startPilot(n),
    snap: () => api("command", { type: "snapshot", drone: n }),
    locate: () => { closeWall(); const d = ST.drones[n]; map.flyTo(ll(d.pos[0], d.pos[1]), Math.max(map.getZoom(), 18)); openDrone(n); },
    mode: () => { camMode[n] = camMode[n] === "thermal" ? "normal" : "thermal"; wallKey = ""; renderWall(ST); },
  })[t.dataset.act]?.();
});
$("vfilter").onclick = e => {
  const b = e.target.closest("button"); if (!b) return;
  vfilter = b.dataset.f;
  $("vfilter").querySelectorAll("button").forEach(x => x.classList.toggle("on", x === b));
  if (ST) renderVictims();
};

// ---------------- panel tabs (desktop) / bottom sheet (phone) ----------------
function setTab(name) {
  ["overview", "victims", "fleet", "log"].forEach(k => $("tab-" + k).hidden = k !== name);
  $("tabs").querySelectorAll("button").forEach(b => b.classList.toggle("on", b.dataset.tab === name));
}
$("tabs").onclick = e => { const b = e.target.closest("button"); if (b) { closeDrawer(); setTab(b.dataset.tab); } };
$("mobnav").onclick = e => {
  const b = e.target.closest("button"); if (!b) return;
  if (b.dataset.tab === "cams") return openWall();
  if (b.dataset.tab === "map" || document.body.dataset.sheet === b.dataset.tab) document.body.removeAttribute("data-sheet");
  else { document.body.dataset.sheet = b.dataset.tab; closeDrawer(); setTab(b.dataset.tab); }
  $("mobnav").querySelectorAll("button").forEach(x => x.classList.toggle("on", x.dataset.tab === (document.body.dataset.sheet || "map")));
};

// ---------------- camera wall (watch only: never changes what the drones do) ----------------
// frames are polled as plain JPEGs (~4 fps): long-lived MJPEG streams would use up the browser's 6 connections per host
const camURL = (id, m) => `${DATA}cams/drone${id}${m === "thermal" ? "_thermal" : ""}.jpg?${Date.now()}`;
setInterval(() => document.querySelectorAll("img[data-cam]").forEach(img => {
  if (img.dataset.busy) return;
  img.dataset.busy = 1;
  const next = new Image();   // double-buffered: the tile keeps its last frame until the next one has loaded
  next.onload = () => { if (img.dataset.cam) img.src = next.src; delete img.dataset.busy; };
  next.onerror = () => delete img.dataset.busy;
  next.src = camURL(img.dataset.cam, img.dataset.mode);
}), 250);
const wallIds = () => allOnline ? ST.drones.filter(d => d.online).map(d => d.id) : [...watch].sort((a, b) => a - b);
function postWatch() { api("watch", { drones: $("camwall").hidden ? [] : wallIds() }, true); }
function openWall() {
  if (!ST) return toast("Start a mission first");
  if (mode !== "live") return toast("Live cameras are only available during a live mission");
  $("camwall").hidden = false; $("btn-cams").classList.add("on"); wallKey = ""; renderWall(ST);
}
function closeWall() { $("camwall").hidden = true; $("btn-cams").classList.remove("on"); $("camgrid").innerHTML = ""; wallKey = ""; postWatch(); }
$("btn-cams").onclick = () => $("camwall").hidden ? openWall() : closeWall();
$("cams-close").onclick = closeWall;
$("campick").onclick = e => {
  const b = e.target.closest("button"); if (!b) return;
  if (b.dataset.cam === "all") { allOnline = !allOnline; watch.clear(); }
  else { allOnline = false; const n = +b.dataset.cam; watch.has(n) ? watch.delete(n) : watch.add(n); }
  wallKey = ""; renderWall(ST);
};
function renderWall(st) {
  $("cam-clock").textContent = `T+${clock(st.t)} · ${new Date().toLocaleTimeString()}`;
  $("campick").innerHTML = `<button data-cam="all" class="${allOnline ? "on" : ""}">All online</button>` + st.drones.map(d =>
    `<button data-cam="${d.id}" class="${!allOnline && watch.has(d.id) ? "on" : ""}"><span class="dot" style="background:${color(d.id)}"></span>D${d.id}</button>`).join("");
  const ids = wallIds(), key = ids.map(i => i + (camMode[i] || "")).join(","), grid = $("camgrid");
  if (key !== wallKey) {   // rebuild tiles only when the selection changes
    wallKey = key;
    grid.className = "camgrid n" + (ids.length <= 1 ? 1 : ids.length === 2 ? 2 : ids.length <= 4 ? 4 : 6);
    grid.innerHTML = ids.map(i => `<div class="tile" id="tile${i}"><img data-cam="${i}" data-mode="${camMode[i] || "normal"}" alt="Drone ${i} camera">
      <div class="flag"></div><div class="hd"></div><div class="ft"></div><div class="acts">
      <button data-act="mode" data-id="${i}">${icon("thermo")}${camMode[i] === "thermal" ? "Normal" : "Thermal"}</button>
      <button data-act="snap" data-id="${i}">${icon("camera")}Snapshot + AI</button><button data-act="locate" data-id="${i}">${icon("crosshair")}Map</button>
      <button data-act="control" data-id="${i}">${icon("gamepad")}Control</button></div></div>`).join("") || '<p class="empty" style="padding:24px">No drone selected</p>';
    postWatch();
  }
  ids.forEach(i => {
    const d = st.drones[i], cam = (st.cams || {})[i], tile = $("tile" + i);
    if (!tile) return;
    tile.querySelector(".hd").innerHTML = `<b style="color:${color(i)}">CAM D${String(i).padStart(2, "0")}</b><span>${STATE_TXT[d.state] || d.state}</span><span class="spacer"></span>
      ${cam && cam.heat ? `<span class="badge">HEAT ${cam.heat.toFixed(1)}°C</span>` : ""}${cam && cam.people.length ? `<span class="badge ai">${esc(cam.people.join(" · "))}</span>` : ""}
      <span>${(camMode[i] || "normal").toUpperCase()}</span>`;
    tile.querySelector(".ft").innerHTML = `${batt(d.battery)}${linkIcon(d)}<span>${signal(d)}</span><span class="spacer"></span><span>ALT ${d.pos[2].toFixed(2)} m</span>`;
    tile.classList.toggle("heat", !!(cam && cam.heat));
    const flag = tile.querySelector(".flag"), sos = d.state === "TRAPPED", live = tile.querySelector("img").naturalWidth > 0;
    flag.hidden = d.online && !sos && live;
    flag.className = "flag" + (sos ? " sos" : live ? " dim" : "");
    flag.textContent = sos ? "SOS · TRAPPED" : !d.online ? "NO SIGNAL" : "CONNECTING";
  });
}

// ---------------- manual takeover (input is only a desired velocity: safety stays on) ----------------
const keys = new Set(), sticks = { move: [0, 0], alt: [0, 0] };
let pilotTimer = 0;
function startPilot(id) {
  if (!isLive()) return toast("Manual control only works during a live mission");
  if (!$("camwall").hidden) closeWall();
  api("command", { type: "takeover", drone: id }, true);
  pilot = { id, t0: Date.now(), manual: false, mode: "normal", lastInput: Date.now() };
  $("pilot").hidden = false; $("pilot-title").textContent = `MANUAL · D${String(id).padStart(2, "0")}`; $("pilot-title").style.color = color(id);
  Object.assign($("pilot-cam").dataset, { cam: id, mode: "normal" }); $("pilot-mode").innerHTML = icon("thermo") + "Thermal";
  pilotTimer = setInterval(pilotTick, 100);   // 10 Hz stick updates
}
function endPilot(msg) {
  clearInterval(pilotTimer); pilot = null; keys.clear();
  $("pilot").hidden = true; delete $("pilot-cam").dataset.cam; $("pilot-cam").removeAttribute("src");
  if (msg) toast(msg, 5000);
}
const axis = v => Math.abs(v) < .15 ? 0 : v;
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
  $("pilot-status").textContent = pilot.manual ? `ALT ${d.pos[2].toFixed(2)} m · HDG ${heading(d)}° · BAT ${d.battery.toFixed(0)}% · ${signal(d).toUpperCase()}`
    : "WAITING FOR DRONE…";
  $("pilot-idle").textContent = pilot.manual && idle > .5 ? `AUTO-RELEASE IN ${Math.max(0, 2 - idle).toFixed(1)} s WITHOUT INPUT` : "";
  $("pilot-warn").hidden = !d.blocked;
}
$("pilot-release").onclick = () => { if (pilot) api("command", { type: "release", drone: pilot.id }, true); endPilot("Control released: drone back to autonomous mode"); };
$("pilot-snap").onclick = () => pilot && api("command", { type: "snapshot", drone: pilot.id });
$("pilot-mode").onclick = () => {
  pilot.mode = pilot.mode === "thermal" ? "normal" : "thermal";
  $("pilot-cam").dataset.mode = pilot.mode; $("pilot-mode").innerHTML = icon("thermo") + (pilot.mode === "thermal" ? "Normal" : "Thermal");
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
    const res = await (await fetch("https://nominatim.openstreetmap.org/search?format=json&limit=5&q=" + encodeURIComponent(q))).json();
    $("results").innerHTML = res.map((p, i) => `<div data-i="${i}">${esc(p.display_name)}</div>`).join("") || "<div>No place found</div>";
    $("results").onclick = ev => {
      const p = res[ev.target.closest("[data-i]")?.dataset.i]; if (!p) return;
      const b = p.boundingbox.map(Number);
      map.flyToBounds([[b[0], b[2]], [b[1], b[3]]], { maxZoom: 17 });
      $("results").innerHTML = "";
    };
  } catch (err) { $("results").innerHTML = "<div>Place search needs an internet connection</div>"; }
};

let placing = false, draw = null, area = null;
const drawStyle = { color: "#3d8bfd", weight: 2, dashArray: "6 5", fillColor: "#3d8bfd", fillOpacity: .08 };
const crosshair = on => map.getContainer().classList.toggle("drawing", on);
const step = id => $(id).closest("li");
$("btn-base").onclick = () => { placing = true; crosshair(true); toast("Click the map to place the rescue base"); };
$("btn-mission").onclick = () => { $("launch").hidden = false; $("btn-mission").classList.add("on"); };
$("launch-cancel").onclick = () => {
  $("launch").hidden = true; $("btn-mission").classList.remove("on"); draw = area = null; G.draw.clearLayers(); crosshair(false); map.doubleClickZoom.enable();
};
document.querySelectorAll("[data-draw]").forEach(b => b.onclick = () => {
  G.draw.clearLayers(); area = null; draw = { kind: b.dataset.draw, pts: [] };
  document.querySelectorAll("[data-draw]").forEach(x => x.classList.toggle("on", x === b));
  crosshair(true); map.doubleClickZoom.disable();
  $("draw-finish").hidden = draw.kind !== "poly"; $("launch-go").disabled = true; step("launch-hint").classList.remove("done");
  $("launch-hint").textContent = draw.kind === "rect" ? "Click two opposite corners." : "Click each corner, then Finish (or double-click).";
});
const shape = pts => draw.kind === "rect" ? L.rectangle(L.latLngBounds(pts), drawStyle) : L.polygon(pts, drawStyle);
function preview(cur) {
  G.draw.clearLayers();
  const pts = cur ? [...draw.pts, cur] : draw.pts;
  if (pts.length > 1) shape(pts).addTo(G.draw);
  draw.pts.forEach(p => L.circleMarker(p, { radius: 4, color: "#3d8bfd", fillColor: "#fff", fillOpacity: 1, weight: 2 }).addTo(G.draw));
}
function areaM2(pts) {   // shoelace on a local flat projection
  const k = 111320 * Math.cos(pts[0].lat * Math.PI / 180);
  return Math.abs(pts.reduce((s, p, i) => { const q = pts[(i + 1) % pts.length];
    return s + (p.lng * k) * (q.lat * 111320) - (q.lng * k) * (p.lat * 111320); }, 0)) / 2;
}
const fmtArea = m2 => m2 > 1e6 ? (m2 / 1e6).toFixed(2) + " km²" : Math.round(m2).toLocaleString() + " m²";
function finishDraw() {
  const pts = draw.pts.filter((p, i) => !i || !p.equals(draw.pts[i - 1]));
  if (draw.kind === "rect" ? pts.length < 2 : pts.length < 3) return toast("Add more corners first");
  const b = L.latLngBounds(pts);
  area = draw.kind === "rect" ? [b.getSouthWest(), b.getSouthEast(), b.getNorthEast(), b.getNorthWest()] : pts;
  draw = null; G.draw.clearLayers(); L.polygon(area, drawStyle).addTo(G.draw);
  crosshair(false); map.doubleClickZoom.enable();
  $("draw-finish").hidden = true; $("launch-go").disabled = false;
  $("launch-hint").textContent = `Area set · ${fmtArea(areaM2(area))}`; step("launch-hint").classList.add("done");
}
$("draw-finish").onclick = () => draw && finishDraw();
map.on("click", e => {
  if (placing) {
    placing = false; crosshair(false);
    if (baseMarker) baseMarker.remove();
    baseMarker = L.marker(e.latlng, { draggable: true, icon: L.divIcon({ className: "lbl", html: "BASE", iconSize: [40, 14] }) }).addTo(map);
    $("base-hint").textContent = "Placed (drag to move). The swarm launches from the nearest edge of the area."; step("base-hint").classList.add("done");
    return toast("Rescue base placed");
  }
  if (!draw) return;
  draw.pts.push(e.latlng);
  if (draw.kind === "rect" && draw.pts.length === 2) finishDraw(); else preview();
});
map.on("mousemove", e => draw && draw.pts.length && preview(e.latlng));
map.on("dblclick", () => draw && draw.kind === "poly" && finishDraw());
$("launch-go").onclick = async () => {
  try {
    const j = await (await fetch("/api/mission", { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify({
      area: area.map(p => [p.lat, p.lng]), base: baseMarker ? [baseMarker.getLatLng().lat, baseMarker.getLatLng().lng] : null,
      drones: +$("n-drones").value, teams: +$("n-teams").value }) })).json();
    if (!j.ok) return toast("Error: " + esc(j.error));
  } catch (e) { return toast("Dashboard server not reachable (run python dashboard/serve.py)"); }
  $("launch").hidden = true; $("btn-mission").classList.remove("on"); area = null;
  stopReplay(); closeDrawer(); S = null; ST = null; lastT = -1; lastAck = lastSnap = 0; summaryShown = sawRunning = primed = false; seenAlerts.clear();
  setMode("LAUNCHING", "launch");
  toast("Swarm launching: fitting the simulation to the search area…", 5000);
};

// ---------------- mission summary ----------------
function showSummary(st) {
  const H = humans(st), reached = H.filter(v => v.reached_t).length, verified = st.victims.filter(v => v.photo && v.confidence < .5).length;
  const unsafe = Object.values(st.routes).filter(r => r.direct.length).length;
  $("modal-body").innerHTML = `<div class="summary"><h2>Mission complete</h2><p>${S.n_drones} drones · ${S.n_teams} rescue teams · ${fmtArea(S.area_m2)} searched</p><div class="grid">
    <div><b>${H.length} / ${S.n_victims}</b><span>Victims found</span></div><div><b>${reached}</b><span>Reached by teams</span></div>
    <div><b>${clock(st.t)}</b><span>Mission time</span></div><div><b>${st.coverage.toFixed(0)}%</b><span>Area searched</span></div>
    <div><b>${H.filter(v => v.source === "phone").length}</b><span>Found by phone signal</span></div><div><b>${st.sos.length + verified + unsafe}</b><span>Alerts</span></div></div>
    <p style="margin:14px 0 0">${st.sos.length} drone SOS · ${verified} photos verified by a rescuer · ${unsafe} unsafe direct routes rerouted</p></div>`;
  $("modal").hidden = false;
}
$("modal-close").onclick = () => $("modal").hidden = true;

// ---------------- data: live polling or replay ----------------
function setMode(text, cls) { const m = $("mode"); m.textContent = text; m.className = "state " + cls; }
const staticKey = s => JSON.stringify([s.origin, s.poly, s.base, s.n_drones, s.map_scale]);
let progressAt = Date.now();
async function pollLive() {
  if (mode !== "live") return;
  try {
    const j = await (await fetch(DATA + "live.json?_=" + Date.now())).json();
    if (!S || staticKey(S) !== staticKey(j.static)) { S = j.static; lastT = -1; drawStatic(); }
    if (!ST || j.state.t !== ST.t) progressAt = Date.now();
    render(j.state);
    if (!$("camwall").hidden) renderWall(j.state);   // cameras and the pilot panel refresh on every poll
    if (pilot) renderPilot(j.state);
    const stale = Date.now() - progressAt > 5000;
    setMode(j.state.final ? "COMPLETE" : stale ? "NOT RUNNING" : "LIVE", j.state.final ? "done" : stale ? "stale" : "live");
  } catch (e) { if (!$("mode").classList.contains("launch")) setMode("STANDBY", "standby"); }
  setTimeout(pollLive, 500);
}
let replayTimer = 0;
async function startReplay() {
  const j = await (await fetch(DATA + "replay.json?_=" + Date.now())).json();
  if (!$("camwall").hidden) closeWall();
  if (pilot) endPilot();
  closeDrawer();
  mode = "replay"; frames = j.frames; idx = 0; playing = true; lastT = -1; summaryShown = sawRunning = primed = false; seenAlerts.clear();
  S = j.static; drawStatic();
  setMode("REPLAY", "replay"); $("timeline").hidden = false; $("btn-replay").classList.add("on");
  $("btn-play").innerHTML = icon("pause"); $("scrub").max = frames.length - 1;
  render(frames[0]); tick();
}
function stopReplay() {
  clearTimeout(replayTimer); $("timeline").hidden = true; $("btn-replay").classList.remove("on");
  if (mode !== "live") { mode = "live"; S = null; lastT = -1; sawRunning = primed = false; seenAlerts.clear(); closeDrawer(); pollLive(); }
}
function tick() {
  if (mode !== "replay") return;
  if (playing && idx < frames.length - 1) { idx++; render(frames[idx]); $("scrub").value = idx; }
  replayTimer = setTimeout(tick, 500 / +$("speed").value);
}
$("scrub").oninput = e => { idx = +e.target.value; lastT = -1; render(frames[idx]); };
$("btn-play").onclick = () => { playing = !playing; $("btn-play").innerHTML = icon(playing ? "pause" : "play"); };
$("replay-exit").onclick = stopReplay;
$("btn-replay").onclick = () => mode === "replay" ? stopReplay() : startReplay().catch(() => toast("No replay yet: run a mission first."));

addEventListener("pagehide", () => navigator.sendBeacon("/api/watch", JSON.stringify({ drones: [] })));   // stop rendering for a closed tab
if ("serviceWorker" in navigator) navigator.serviceWorker.register("sw.js").catch(() => { /* needs localhost or HTTPS */ });
if (new URLSearchParams(location.search).get("mode") === "replay") startReplay().catch(() => pollLive()); else pollLive();
