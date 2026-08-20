/* LoopBack — round-trip run planner.
 *
 * Loop generation, default (OSRM, no key): place a circle of waypoints whose
 * circumference matches the target distance, snap them to the road network,
 * then iteratively rescale the circle until the routed distance lands within
 * tolerance. Candidate loops passing through user-placed "avoid" markers are
 * penalised and re-rolled.
 *
 * With an OpenRouteService key (Settings): uses the foot-walking profile's
 * native round_trip generator with hard avoid_polygons around avoid markers.
 */

"use strict";

const OSRM_BASE = "https://router.project-osrm.org/route/v1/foot/";
const ORS_BASE = "https://api.openrouteservice.org/v2/directions/foot-walking/geojson";
const AVOID_RADIUS_M = 45; // a loop passing this close to an avoid marker counts as a hit

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

let map, startMarker, routeLine, rejoinLine, userMarker, accuracyCircle;
let start = null;          // {lat, lng}
let route = null;          // {distance, coords, cumDist, steps, provider}
let avoidPoints = [];      // [{pos: {lat,lng}, marker}]
let avoidMode = false;
let watchId = null;
let simTimer = null;
let voiceOn = true;
let nav = null;            // {idx, preAnnounced, rejoin, offRouteSince, ...}
let orsKey = localStorage.getItem("loopback-ors-key") || "";
let navTimer = null;       // updates the elapsed-time display each second
let wakeLock = null;       // keeps the phone screen on during a run
let pendingRun = null;     // stats awaiting Save/Discard on the summary panel
let viewedRunLine = null;  // polyline of a saved run being viewed
let viewedRunId = null;

const $ = (id) => document.getElementById(id);

// ---------------------------------------------------------------------------
// Geometry helpers
// ---------------------------------------------------------------------------

const R_EARTH = 6371000;
const toRad = (d) => (d * Math.PI) / 180;
const toDeg = (r) => (r * 180) / Math.PI;

function haversine(a, b) {
  const dLat = toRad(b.lat - a.lat);
  const dLng = toRad(b.lng - a.lng);
  const s =
    Math.sin(dLat / 2) ** 2 +
    Math.cos(toRad(a.lat)) * Math.cos(toRad(b.lat)) * Math.sin(dLng / 2) ** 2;
  return 2 * R_EARTH * Math.asin(Math.sqrt(s));
}

// Point reached by travelling `dist` metres from `p` on bearing `deg`.
function destinationPoint(p, deg, dist) {
  const br = toRad(deg);
  const dr = dist / R_EARTH;
  const lat1 = toRad(p.lat);
  const lng1 = toRad(p.lng);
  const lat2 = Math.asin(
    Math.sin(lat1) * Math.cos(dr) + Math.cos(lat1) * Math.sin(dr) * Math.cos(br)
  );
  const lng2 =
    lng1 +
    Math.atan2(
      Math.sin(br) * Math.sin(dr) * Math.cos(lat1),
      Math.cos(dr) - Math.sin(lat1) * Math.sin(lat2)
    );
  return { lat: toDeg(lat2), lng: toDeg(lng2) };
}

// Distance from point p to segment a-b, using a local flat approximation
// (fine at running scales).
function pointToSegment(p, a, b) {
  const kx = Math.cos(toRad(p.lat)) * 111320; // metres per degree of longitude
  const ky = 110540;                          // metres per degree of latitude
  const ax = (a.lng - p.lng) * kx, ay = (a.lat - p.lat) * ky;
  const bx = (b.lng - p.lng) * kx, by = (b.lat - p.lat) * ky;
  const dx = bx - ax, dy = by - ay;
  const lenSq = dx * dx + dy * dy;
  let t = lenSq ? -(ax * dx + ay * dy) / lenSq : 0;
  t = Math.max(0, Math.min(1, t));
  const cx = ax + t * dx, cy = ay + t * dy;
  return { dist: Math.hypot(cx, cy), t };
}

// Nearest point on a polyline: distance off it + distance along it.
function locateOnCoords(p, coords, cumDist) {
  let best = { dist: Infinity, along: 0 };
  for (let i = 0; i < coords.length - 1; i++) {
    const seg = pointToSegment(p, coords[i], coords[i + 1]);
    if (seg.dist < best.dist) {
      const segLen = cumDist[i + 1] - cumDist[i];
      best = { dist: seg.dist, along: cumDist[i] + seg.t * segLen };
    }
  }
  return best;
}

const locateOnRoute = (p) => locateOnCoords(p, route.coords, route.cumDist);

function pointAtAlong(along) {
  const { coords, cumDist } = route;
  if (along <= 0) return coords[0];
  if (along >= cumDist.at(-1)) return coords.at(-1);
  let i = 1;
  while (cumDist[i] < along) i++;
  const t = (along - cumDist[i - 1]) / (cumDist[i] - cumDist[i - 1] || 1);
  return {
    lat: coords[i - 1].lat + (coords[i].lat - coords[i - 1].lat) * t,
    lng: coords[i - 1].lng + (coords[i].lng - coords[i - 1].lng) * t,
  };
}

function buildCumDist(coords) {
  const cumDist = [0];
  for (let i = 1; i < coords.length; i++) {
    cumDist.push(cumDist[i - 1] + haversine(coords[i - 1], coords[i]));
  }
  return cumDist;
}

// ---------------------------------------------------------------------------
// Formatting
// ---------------------------------------------------------------------------

const fmtKm = (m) => (m / 1000).toFixed(2) + " km";

function fmtMetres(m) {
  if (m >= 950) return (m / 1000).toFixed(1) + " km";
  if (m >= 100) return Math.round(m / 50) * 50 + " m";
  return Math.max(10, Math.round(m / 10) * 10) + " m";
}

const lowerFirst = (s) => s.charAt(0).toLowerCase() + s.slice(1);

function fmtElapsed(sec) {
  sec = Math.max(0, Math.round(sec));
  const h = Math.floor(sec / 3600);
  const m = Math.floor((sec % 3600) / 60);
  const s = sec % 60;
  return h
    ? `${h}:${String(m).padStart(2, "0")}:${String(s).padStart(2, "0")}`
    : `${m}:${String(s).padStart(2, "0")}`;
}

function fmtPace(distanceM, elapsedS) {
  if (distanceM < 100) return "—";
  return fmtElapsed(elapsedS / (distanceM / 1000)) + " /km";
}

function fmtDate(iso) {
  const d = new Date(iso);
  const p = (n) => String(n).padStart(2, "0");
  return `${p(d.getDate())}/${p(d.getMonth() + 1)}/${d.getFullYear()} ${p(d.getHours())}:${p(d.getMinutes())}`;
}

// ---------------------------------------------------------------------------
// Saved runs (localStorage)
// ---------------------------------------------------------------------------

const RUNS_KEY = "loopback-runs";

function loadRuns() {
  try {
    return JSON.parse(localStorage.getItem(RUNS_KEY)) || [];
  } catch {
    return [];
  }
}

function persistRuns(runs) {
  localStorage.setItem(RUNS_KEY, JSON.stringify(runs.slice(0, 100)));
  updateRunsBadge();
}

function updateRunsBadge() {
  const n = loadRuns().length;
  const badge = $("runs-count");
  badge.hidden = n === 0;
  badge.textContent = n;
}

const MODIFIER_TEXT = {
  "sharp left": "sharp left", left: "left", "slight left": "slightly left",
  "sharp right": "sharp right", right: "right", "slight right": "slightly right",
  straight: "straight on", uturn: "around (U-turn)",
};

function stepInstruction(step) {
  const name = step.name ? ` onto ${step.name}` : "";
  const mod = MODIFIER_TEXT[step.maneuver.modifier] || "";
  switch (step.maneuver.type) {
    case "depart":
      return step.name ? `Head off along ${step.name}` : "Head off";
    case "arrive":
      return "You have arrived back at your start point";
    case "roundabout":
    case "rotary": {
      const exit = step.maneuver.exit;
      return exit
        ? `At the roundabout, take exit ${exit}${name}`
        : `Go around the roundabout${name}`;
    }
    case "continue":
    case "new name":
      return mod && mod !== "straight on" ? `Bear ${mod}${name}` : `Continue${name}`;
    case "merge":
      return `Merge ${mod}${name}`;
    case "fork":
      return `At the fork, keep ${mod}${name}`;
    case "end of road":
      return `At the end of the road, turn ${mod}${name}`;
    default:
      if (mod === "around (U-turn)") return `Turn around${name}`;
      if (mod === "straight on") return `Go straight on${name}`;
      return mod ? `Turn ${mod}${name}` : `Continue${name}`;
  }
}

// ---------------------------------------------------------------------------
// Routing providers
// ---------------------------------------------------------------------------

async function fetchRoute(points) {
  const coords = points.map((p) => `${p.lng.toFixed(6)},${p.lat.toFixed(6)}`).join(";");
  const url = `${OSRM_BASE}${coords}?overview=full&geometries=geojson&steps=true&continue_straight=true`;
  const res = await fetch(url);
  if (!res.ok) throw new Error(`Routing server error (${res.status})`);
  const data = await res.json();
  if (data.code !== "Ok" || !data.routes?.length) {
    throw new Error("No route found here — try a different start point");
  }
  return data.routes[0];
}

// Fraction of the route on numbered main roads (A-roads weighted fully,
// B-roads at 40%) — used to prefer quieter loops.
function busyWeight(refOrName) {
  if (/(^|[^A-Za-z0-9])A\d+/.test(refOrName)) return 1;
  if (/(^|[^A-Za-z0-9])B\d+/.test(refOrName)) return 0.4;
  return 0;
}

function parseRoute(osrmRoute) {
  const coords = osrmRoute.geometry.coordinates.map(([lng, lat]) => ({ lat, lng }));
  const cumDist = buildCumDist(coords);
  const steps = [];
  let busyM = 0;
  for (const leg of osrmRoute.legs) {
    for (const s of leg.steps) {
      busyM += busyWeight(`${s.ref || ""} ${s.name || ""}`) * (s.distance || 0);
    }
    for (const s of leg.steps) {
      // Intermediate waypoint "arrive"/"depart" pairs are artefacts of the
      // loop's shaping points, not real turns — drop them.
      if (s.maneuver.type === "arrive" && leg !== osrmRoute.legs.at(-1)) continue;
      if (s.maneuver.type === "depart" && leg !== osrmRoute.legs[0]) continue;
      const loc = { lat: s.maneuver.location[1], lng: s.maneuver.location[0] };
      steps.push({
        loc,
        along: locateOnCoords(loc, coords, cumDist).along,
        instruction: stepInstruction(s),
        type: s.maneuver.type,
      });
    }
  }
  return {
    distance: osrmRoute.distance, coords, cumDist, steps,
    busyFrac: osrmRoute.distance ? busyM / osrmRoute.distance : 0,
    provider: "OSRM",
  };
}

// Total ascent in metres, from the free Open-Meteo elevation API (no key).
async function fetchAscent(coords) {
  const n = Math.min(80, coords.length);
  const idxs = Array.from({ length: n }, (_, i) =>
    Math.floor((i * (coords.length - 1)) / (n - 1))
  );
  const lats = idxs.map((i) => coords[i].lat.toFixed(5)).join(",");
  const lngs = idxs.map((i) => coords[i].lng.toFixed(5)).join(",");
  const res = await fetch(
    `https://api.open-meteo.com/v1/elevation?latitude=${lats}&longitude=${lngs}`
  );
  if (!res.ok) throw new Error("elevation unavailable");
  const e = (await res.json()).elevation;
  // light smoothing, then sum the positive deltas
  const sm = e.map((v, i) =>
    i > 0 && i < e.length - 1 ? (e[i - 1] + v + e[i + 1]) / 3 : v
  );
  let ascent = 0;
  for (let i = 1; i < sm.length; i++) {
    const d = sm[i] - sm[i - 1];
    if (d > 0.5) ascent += d;
  }
  return Math.round(ascent);
}

// --- OpenRouteService (optional, key required) -----------------------------

function avoidMultiPolygon() {
  const d = 0.0005; // ~50 m square around each avoid marker
  return {
    type: "MultiPolygon",
    coordinates: avoidPoints.map(({ pos }) => [[
      [pos.lng - d, pos.lat - d], [pos.lng + d, pos.lat - d],
      [pos.lng + d, pos.lat + d], [pos.lng - d, pos.lat + d],
      [pos.lng - d, pos.lat - d],
    ]]),
  };
}

async function orsRoundTrip(origin, targetM, seed) {
  const body = {
    coordinates: [[origin.lng, origin.lat]],
    instructions: true,
    options: { round_trip: { length: targetM, points: 4, seed } },
  };
  if (avoidPoints.length) body.options.avoid_polygons = avoidMultiPolygon();
  const res = await fetch(ORS_BASE, {
    method: "POST",
    headers: { Authorization: orsKey, "Content-Type": "application/json" },
    body: JSON.stringify(body),
  });
  if (res.status === 401 || res.status === 403) {
    throw new Error("OpenRouteService rejected the API key — check it in Settings");
  }
  if (!res.ok) throw new Error(`OpenRouteService error (${res.status})`);
  const data = await res.json();
  const feat = data.features?.[0];
  if (!feat) throw new Error("OpenRouteService returned no route here");
  return feat;
}

function parseOrsRoute(feat) {
  const coords = feat.geometry.coordinates.map(([lng, lat]) => ({ lat, lng }));
  const cumDist = buildCumDist(coords);
  const steps = [];
  let busyM = 0;
  for (const seg of feat.properties.segments) {
    for (const s of seg.steps) {
      busyM += busyWeight(s.name || "") * (s.distance || 0);
      const loc = coords[s.way_points[0]];
      steps.push({
        loc,
        along: cumDist[s.way_points[0]],
        instruction:
          s.type === 10 ? "You have arrived back at your start point" : s.instruction,
        type: s.type === 10 ? "arrive" : "",
      });
    }
  }
  const distance = feat.properties.summary.distance;
  return {
    distance, coords, cumDist, steps,
    busyFrac: distance ? busyM / distance : 0,
    provider: "OpenRouteService (walking)",
  };
}

// ---------------------------------------------------------------------------
// Loop generation
// ---------------------------------------------------------------------------

function countAvoidHits(coords) {
  let hits = 0;
  for (const { pos } of avoidPoints) {
    for (let i = 0; i < coords.length - 1; i++) {
      if (pointToSegment(pos, coords[i], coords[i + 1]).dist < AVOID_RADIUS_M) {
        hits++;
        break;
      }
    }
  }
  return hits;
}

async function osrmLoop(origin, targetM, tolerance, bearing) {
  let radius = targetM / (2 * Math.PI);
  let best = null;

  for (let attempt = 0; attempt < 6; attempt++) {
    const centre = destinationPoint(origin, bearing, radius);
    const backBearing = (bearing + 180) % 360;
    // Three shaping waypoints spread around the circle from the start's position.
    const waypoints = [90, 180, 270].map((offset) =>
      destinationPoint(centre, (backBearing + offset) % 360, radius)
    );

    let osrmRoute;
    try {
      osrmRoute = await fetchRoute([origin, ...waypoints, origin]);
    } catch (err) {
      if (attempt === 0) throw err; // no route at all from here
      break;
    }

    const parsed = parseRoute(osrmRoute);
    const err = Math.abs(parsed.distance - targetM) / targetM;
    const hits = countAvoidHits(parsed.coords);
    const score = err + hits * 10; // any avoid hit outweighs distance accuracy
    if (!best || score < best.score) best = { parsed, err, hits, score };
    if (err <= tolerance && hits === 0) break;
    if (hits > 0) break; // this direction clips an avoided road — re-roll bearing

    // Rescale the circle towards the target, damped and clamped so road-network
    // quirks don't make it oscillate.
    let factor = targetM / parsed.distance;
    factor = Math.max(0.55, Math.min(1.8, 1 + (factor - 1) * 0.8));
    radius *= factor;
  }

  if (!best) throw new Error("Could not build a loop from here");
  return best;
}

// Gather candidate loops, then pick the winner on a combined score:
// distance accuracy first, then avoid-marker hits, quiet roads, and the
// terrain preference (flat/hilly) using real elevation data.
async function generateLoopRoute(origin, targetM, tolerance) {
  const terrain = $("terrain").value; // flat | any | hilly
  const candidates = [];
  let lastError = null;

  if (orsKey) {
    const firstSeed = Math.floor(Math.random() * 1000);
    for (let i = 0; i < 4; i++) {
      try {
        const parsed = parseOrsRoute(await orsRoundTrip(origin, targetM, firstSeed + i * 37));
        const err = Math.abs(parsed.distance - targetM) / targetM;
        candidates.push({ parsed, err, hits: 0 });
        if (err <= tolerance && terrain === "any") break;
      } catch (err) {
        lastError = err;
      }
    }
  } else {
    const first = Math.random() * 360;
    const nBearings = avoidPoints.length ? 6 : 3;
    for (let i = 0; i < nBearings; i++) {
      const bearing = (first + (360 / nBearings) * i) % 360;
      try {
        const result = await osrmLoop(origin, targetM, tolerance, bearing);
        candidates.push(result);
        // With no terrain preference, a clean quiet in-tolerance loop is
        // good enough — stop searching.
        if (terrain === "any" && result.err <= tolerance && result.hits === 0 &&
            result.parsed.busyFrac < 0.1) break;
      } catch (err) {
        lastError = err;
      }
    }
  }

  if (!candidates.length) throw lastError || new Error("Could not build a loop from here");

  // Elevation is a rate-limited free service: with no terrain preference only
  // the winner needs it (for display); with a preference, every candidate does.
  if (terrain !== "any") {
    for (const c of candidates) {
      try {
        c.ascent = await fetchAscent(c.parsed.coords);
      } catch {
        c.ascent = null; // service busy — terrain scoring just no-ops
      }
    }
  }

  for (const c of candidates) {
    let score =
      (c.err <= tolerance ? c.err * 0.5 : 1 + c.err * 2) +
      (c.hits || 0) * 10 +
      (c.parsed.busyFrac || 0) * 0.3;
    if (c.ascent != null && terrain !== "any") {
      const perKm = Math.min(40, c.ascent / (c.parsed.distance / 1000));
      score += terrain === "flat" ? perKm * 0.012 : -perKm * 0.012;
    }
    c.score = score;
  }
  candidates.sort((a, b) => a.score - b.score);

  const winner = candidates[0];
  if (winner.ascent === undefined) {
    try {
      winner.ascent = await fetchAscent(winner.parsed.coords);
    } catch {
      winner.ascent = null;
    }
  }
  winner.terrainSkipped = terrain !== "any" && candidates.every((c) => c.ascent == null);
  return winner;
}

// ---------------------------------------------------------------------------
// UI
// ---------------------------------------------------------------------------

function toast(msg, ms = 3500) {
  const el = $("toast");
  el.textContent = msg;
  el.hidden = false;
  clearTimeout(el._t);
  el._t = setTimeout(() => (el.hidden = true), ms);
}

function speak(text) {
  if (!voiceOn || !("speechSynthesis" in window)) return;
  speechSynthesis.cancel();
  const u = new SpeechSynthesisUtterance(text);
  u.lang = "en-GB";
  u.rate = 1.0;
  speechSynthesis.speak(u);
}

function setStart(latlng) {
  start = { lat: latlng.lat, lng: latlng.lng };
  if (!startMarker) {
    startMarker = L.marker(start, { title: "Start / finish" }).addTo(map);
  } else {
    startMarker.setLatLng(start);
  }
  $("btn-generate").disabled = false;
  $("hint").textContent = "Start point set. Choose a distance and generate your loop.";
  clearRoute();
}

function addAvoidPoint(latlng) {
  const pos = { lat: latlng.lat, lng: latlng.lng };
  const marker = L.circleMarker(pos, {
    radius: 9, color: "#b91c1c", weight: 2, fillColor: "#ef4444", fillOpacity: 0.75,
  }).addTo(map);
  marker.bindTooltip("Avoided — tap to remove", { direction: "top" });
  const entry = { pos, marker };
  marker.on("click", () => {
    map.removeLayer(marker);
    avoidPoints = avoidPoints.filter((e) => e !== entry);
    updateAvoidHint();
  });
  avoidPoints.push(entry);
  updateAvoidHint();
}

function updateAvoidHint() {
  if (avoidMode) {
    $("hint").textContent = avoidPoints.length
      ? `${avoidPoints.length} road${avoidPoints.length > 1 ? "s" : ""} marked to avoid. Regenerate to apply. Tap a marker to remove it.`
      : "Avoid mode: tap any road you don't want the route to use.";
  }
}

function clearRoute() {
  if (routeLine) { map.removeLayer(routeLine); routeLine = null; }
  route = null;
  $("btn-variation").disabled = true;
  $("btn-start").disabled = true;
  $("route-summary").hidden = true;
}

function drawRoute() {
  if (routeLine) map.removeLayer(routeLine);
  routeLine = L.polyline(route.coords, {
    color: "#4f46e5", weight: 5, opacity: 0.85,
  }).addTo(map);
  map.fitBounds(routeLine.getBounds(), { padding: [40, 40] });
}

async function onGenerate() {
  if (!start) return;
  const targetM = parseFloat($("distance").value) * 1000;
  const tolerance = parseFloat($("tolerance").value);
  if (!(targetM > 0)) { toast("Enter a distance first"); return; }

  const btn = $("btn-generate");
  btn.disabled = true;
  btn.innerHTML = '<span class="btn-icon">…</span>Working';
  try {
    const { parsed, err, hits, ascent, terrainSkipped } = await generateLoopRoute(start, targetM, tolerance);
    route = parsed;
    route.ascent = ascent ?? null;
    drawRoute();
    const summary = $("route-summary");
    summary.hidden = false;
    const bits = [`<strong>${fmtKm(route.distance)}</strong> loop`];
    bits.push(
      err <= tolerance
        ? `within ±${Math.round(tolerance * 100)}% ✅`
        : `${Math.round(err * 100)}% off target — closest found; try another or widen the tolerance`
    );
    if (route.ascent != null) bits.push(`↗ ${route.ascent} m climb`);
    if (terrainSkipped) bits.push("⚠ elevation service busy — terrain preference skipped");
    if (route.busyFrac >= 0.1) bits.push(`⚠ ${Math.round(route.busyFrac * 100)}% on main roads`);
    else bits.push("quiet roads ✅");
    if (hits > 0) bits.push(`⚠ couldn't fully dodge ${hits} avoided road${hits > 1 ? "s" : ""}`);
    else if (avoidPoints.length) bits.push("avoided roads dodged ✅");
    bits.push(`<span class="provider">via ${route.provider}</span>`);
    summary.innerHTML = bits.join(" · ");
    $("btn-variation").disabled = false;
    $("btn-start").disabled = false;
  } catch (err) {
    toast(err.message || "Route generation failed — try again");
  } finally {
    btn.disabled = false;
    btn.innerHTML = '<span class="btn-icon">✦</span>Generate route';
  }
}

// ---------------------------------------------------------------------------
// Live navigation
// ---------------------------------------------------------------------------

function updateUserMarker(pos, accuracy) {
  if (!userMarker) {
    userMarker = L.circleMarker(pos, {
      radius: 8, color: "#fff", weight: 2, fillColor: "#4f46e5", fillOpacity: 1,
    }).addTo(map);
    accuracyCircle = L.circle(pos, {
      radius: accuracy || 0, color: "#4f46e5", weight: 1, opacity: 0.3,
      fillOpacity: 0.08,
    }).addTo(map);
  } else {
    userMarker.setLatLng(pos);
    accuracyCircle.setLatLng(pos);
    accuracyCircle.setRadius(accuracy || 0);
  }
}

// Advance through a guidance step list; returns "finished" when its last step
// (an "arrive") has been announced.
function advanceSteps(guide, pos, announcePrefix = "") {
  while (guide.idx < guide.steps.length) {
    const step = guide.steps[guide.idx];
    const d = haversine(pos, step.loc);

    if (d > 45) {
      $("nav-instruction").textContent = step.instruction;
      $("nav-distance").textContent = fmtMetres(d);
      if (!guide.preAnnounced && d < 160) {
        speak(`${announcePrefix}In ${fmtMetres(d).replace(" m", " metres")}, ${lowerFirst(step.instruction)}`);
        guide.preAnnounced = true;
      }
      return "ongoing";
    }

    speak(step.instruction);
    $("nav-instruction").textContent = step.instruction;
    $("nav-distance").textContent = "Now";
    guide.idx++;
    guide.preAnnounced = false;
    if (step.type === "arrive" || guide.idx >= guide.steps.length) return "finished";
    return "ongoing";
  }
  return "finished";
}

async function requestRejoin(pos, along) {
  if (nav.rejoinFetching) return;
  const now = Date.now();
  if (now - (nav.lastRejoinFetch || 0) < 30000) return;
  nav.rejoinFetching = true;
  nav.lastRejoinFetch = now;
  try {
    const target = pointAtAlong(Math.min(route.distance - 1, along + 150));
    const parsed = parseRoute(await fetchRoute([pos, target]));
    // Swap the connector's "arrive" for a rejoin message.
    const last = parsed.steps.at(-1);
    if (last) { last.instruction = "Rejoin your loop"; last.type = ""; }
    if (nav.rejoin?.line) map.removeLayer(nav.rejoin.line);
    nav.rejoin = {
      steps: parsed.steps, idx: 0, preAnnounced: false,
      line: L.polyline(parsed.coords, {
        color: "#f59e0b", weight: 5, opacity: 0.9, dashArray: "8 8",
      }).addTo(map),
    };
    speak("Rerouting. " + (parsed.steps[0]?.instruction || "Follow the orange line back to your loop."));
  } catch {
    // Server hiccup — the 30 s cooldown will let us retry.
  } finally {
    nav.rejoinFetching = false;
  }
}

function clearRejoin(announce) {
  if (!nav?.rejoin) return;
  if (nav.rejoin.line) map.removeLayer(nav.rejoin.line);
  nav.rejoin = null;
  if (announce) speak("Back on route.");
}

function onPositionUpdate(pos, accuracy) {
  updateUserMarker(pos, accuracy);
  if (!nav || !route) return;
  map.panTo(pos, { animate: true });

  // Progress along the loop
  const onRoute = locateOnRoute(pos);
  const done = onRoute.along;
  nav.maxAlong = Math.max(nav.maxAlong || 0, done);
  announceKmSplit();
  const remaining = Math.max(0, route.distance - done);
  $("nav-done").textContent = fmtKm(done) + " done";
  $("nav-remaining").textContent = fmtKm(remaining) + " to go";
  $("nav-progress-fill").style.width =
    Math.min(100, (done / route.distance) * 100) + "%";

  // Off-route handling (ignore poor GPS fixes)
  const offRoute = onRoute.dist > Math.max(50, (accuracy || 0) * 1.5);
  $("nav-offroute").hidden = !offRoute;

  if (offRoute) {
    if (!nav.offRouteSince) nav.offRouteSince = Date.now();
    // Give the runner a moment to correct themselves, then fetch a way back.
    if (nav.rejoin || Date.now() - nav.offRouteSince > 8000 || onRoute.dist > 120) {
      if (!nav.rejoin) requestRejoin(pos, onRoute.along);
      if (nav.rejoin) advanceSteps(nav.rejoin, pos, "");
    }
    return;
  }

  nav.offRouteSince = null;
  if (nav.rejoin) {
    clearRejoin(true);
    // Resume the main guidance at the first turn ahead of where we rejoined.
    const ahead = route.steps.findIndex((s) => s.along > onRoute.along + 10);
    nav.idx = ahead === -1 ? route.steps.length - 1 : ahead;
    nav.preAnnounced = false;
  }

  if (advanceSteps(nav, pos) === "finished") finishRun();
}

async function acquireWakeLock() {
  try {
    if ("wakeLock" in navigator) wakeLock = await navigator.wakeLock.request("screen");
  } catch {
    // Not supported or denied — the run still works, the screen just may dim.
  }
}

function releaseWakeLock() {
  wakeLock?.release?.();
  wakeLock = null;
}

function announceKmSplit() {
  const km = Math.floor(nav.maxAlong / 1000);
  if (km <= (nav.lastKmAnnounced || 0)) return;
  nav.lastKmAnnounced = km;
  const elapsedS = (Date.now() - nav.startedAt) / 1000;
  const paceS = Math.round(elapsedS / (nav.maxAlong / 1000));
  const m = Math.floor(paceS / 60), s = paceS % 60;
  speak(`${km} kilometre${km > 1 ? "s" : ""} done. Average pace ${m} minutes ${s ? s : ""} per kilometre.`);
}

function startRun() {
  if (!route) return;
  nav = {
    steps: route.steps, idx: 0, preAnnounced: false, rejoin: null,
    offRouteSince: null, startedAt: Date.now(), maxAlong: 0, usedSim: false,
    lastKmAnnounced: 0,
  };
  acquireWakeLock();
  $("plan-panel").hidden = true;
  $("summary-panel").hidden = true;
  $("nav-panel").hidden = false;
  $("nav-time").textContent = "0:00";
  navTimer = setInterval(() => {
    if (nav) $("nav-time").textContent = fmtElapsed((Date.now() - nav.startedAt) / 1000);
  }, 1000);
  $("nav-instruction").textContent = route.steps[0]?.instruction || "Head off";
  $("nav-distance").textContent = "—";
  speak(`Starting your ${fmtKm(route.distance).replace(" km", " kilometre")} loop. ${route.steps[0]?.instruction || "Head off"}.`);

  if ("geolocation" in navigator) {
    watchId = navigator.geolocation.watchPosition(
      (p) => onPositionUpdate(
        { lat: p.coords.latitude, lng: p.coords.longitude },
        p.coords.accuracy
      ),
      (err) => toast(`GPS problem: ${err.message}. You can use 🧪 Simulate to preview.`),
      { enableHighAccuracy: true, maximumAge: 1000, timeout: 15000 }
    );
  } else {
    toast("No GPS available — use 🧪 Simulate to preview the guidance.");
  }
}

function finishRun() {
  speak("Loop complete. Nice work — you are back where you started.");
  endRun(true);
}

// Tear down tracking; show the summary card if the run covered any distance,
// otherwise drop straight back to planning.
function endRun(completed) {
  if (!nav) return;
  const elapsedS = (Date.now() - nav.startedAt) / 1000;
  const distanceM = completed ? route.distance : nav.maxAlong || 0;
  const usedSim = nav.usedSim;

  if (watchId !== null) { navigator.geolocation.clearWatch(watchId); watchId = null; }
  if (simTimer) { clearInterval(simTimer); simTimer = null; $("btn-simulate").innerHTML = '<span class="btn-icon">🧪</span>Simulate'; }
  if (navTimer) { clearInterval(navTimer); navTimer = null; }
  releaseWakeLock();
  clearRejoin(false);
  nav = null;
  $("nav-panel").hidden = true;
  if (!completed) speechSynthesis?.cancel();

  if (distanceM < 50) {
    $("plan-panel").hidden = false;
    return;
  }

  pendingRun = {
    date: new Date().toISOString(),
    distanceM: Math.round(distanceM),
    elapsedS: Math.round(elapsedS),
    completed,
    sim: usedSim,
    climbM: route.ascent ?? null,
    coords: route.coords.map((c) => [
      Math.round(c.lat * 1e5) / 1e5,
      Math.round(c.lng * 1e5) / 1e5,
    ]),
  };
  $("summary-title").textContent = completed ? "🏁 Loop complete" : "Run ended";
  $("sum-dist").textContent = fmtKm(distanceM);
  $("sum-time").textContent = fmtElapsed(elapsedS);
  $("sum-pace").textContent = fmtPace(distanceM, elapsedS);
  $("sum-climb").textContent = route.ascent != null ? `${route.ascent} m` : "—";
  $("summary-panel").hidden = false;
}

function closeSummary(save) {
  if (save && pendingRun) {
    const runs = loadRuns();
    runs.unshift({ id: crypto.randomUUID(), ...pendingRun });
    persistRuns(runs);
    toast("Run saved 💾");
  }
  pendingRun = null;
  $("summary-panel").hidden = true;
  $("plan-panel").hidden = false;
}

// Simulated run — moves a phantom runner along the route so the guidance can be
// previewed without leaving the house.
function toggleSimulation() {
  if (simTimer) {
    clearInterval(simTimer);
    simTimer = null;
    $("btn-simulate").innerHTML = '<span class="btn-icon">🧪</span>Simulate';
    return;
  }
  if (watchId !== null) { navigator.geolocation.clearWatch(watchId); watchId = null; }
  if (nav) nav.usedSim = true;
  let along = 0;
  const speed = 30; // m per tick — brisk, so previews are quick
  $("btn-simulate").innerHTML = '<span class="btn-icon">⏸</span>Pause';
  simTimer = setInterval(() => {
    if (!nav) { clearInterval(simTimer); simTimer = null; return; }
    along += speed;
    if (along >= route.cumDist.at(-1)) {
      onPositionUpdate(route.coords.at(-1), 5);
      if (simTimer) { clearInterval(simTimer); simTimer = null; }
      return;
    }
    onPositionUpdate(pointAtAlong(along), 5);
  }, 700);
}

// ---------------------------------------------------------------------------
// Wiring
// ---------------------------------------------------------------------------

function locateMe() {
  if (!("geolocation" in navigator)) {
    toast("Geolocation is not available in this browser");
    return;
  }
  const btn = $("btn-locate");
  btn.innerHTML = '<span class="btn-icon">📍</span>Locating…';
  navigator.geolocation.getCurrentPosition(
    (p) => {
      const pos = { lat: p.coords.latitude, lng: p.coords.longitude };
      map.setView(pos, 15);
      setStart(pos);
      btn.innerHTML = '<span class="btn-icon">📍</span>My location';
    },
    (err) => {
      toast(`Could not get your location: ${err.message}. Tap the map instead.`);
      btn.innerHTML = '<span class="btn-icon">📍</span>My location';
    },
    { enableHighAccuracy: true, timeout: 12000 }
  );
}

// ---------------------------------------------------------------------------
// Saved runs UI
// ---------------------------------------------------------------------------

// GPX export with timestamps spread along the track by distance, so imports
// into Strava/Garmin carry the run's real duration and pace.
function runToGpx(run) {
  const coords = run.coords.map(([lat, lng]) => ({ lat, lng }));
  const cum = buildCumDist(coords);
  // Partial runs stored the whole planned loop — trim to the distance covered.
  let end = cum.findIndex((d) => d > run.distanceM);
  if (end === -1) end = coords.length;
  const used = coords.slice(0, Math.max(2, end));
  const usedCum = cum.slice(0, Math.max(2, end));
  const totalLen = usedCum.at(-1) || 1;
  const startT = new Date(run.date).getTime() - run.elapsedS * 1000;
  const pts = used
    .map((c, i) => {
      const t = new Date(startT + run.elapsedS * 1000 * (usedCum[i] / totalLen));
      return `      <trkpt lat="${c.lat}" lon="${c.lng}"><time>${t.toISOString()}</time></trkpt>`;
    })
    .join("\n");
  return `<?xml version="1.0" encoding="UTF-8"?>
<gpx version="1.1" creator="LoopBack" xmlns="http://www.topografix.com/GPX/1/1">
  <trk>
    <name>LoopBack run ${fmtDate(run.date)}</name>
    <trkseg>
${pts}
    </trkseg>
  </trk>
</gpx>`;
}

function downloadRunGpx(run) {
  const blob = new Blob([runToGpx(run)], { type: "application/gpx+xml" });
  const a = document.createElement("a");
  a.href = URL.createObjectURL(blob);
  a.download = `loopback-${run.date.slice(0, 10)}-${(run.distanceM / 1000).toFixed(1)}km.gpx`;
  a.click();
  setTimeout(() => URL.revokeObjectURL(a.href), 5000);
}

// Records & totals shown at the top of My runs; simulated runs are excluded.
function renderRunStats(runs) {
  const real = runs.filter((r) => !r.sim);
  const now = new Date();
  const monday = new Date(now);
  monday.setHours(0, 0, 0, 0);
  monday.setDate(now.getDate() - ((now.getDay() + 6) % 7));
  const weekM = real
    .filter((r) => new Date(r.date) >= monday)
    .reduce((s, r) => s + r.distanceM, 0);
  const totalM = real.reduce((s, r) => s + r.distanceM, 0);
  const eligible = real.filter((r) => r.distanceM >= 1000);
  const best = eligible.length
    ? eligible.reduce((a, b) => (a.elapsedS / a.distanceM < b.elapsedS / b.distanceM ? a : b))
    : null;
  const longest = real.length ? Math.max(...real.map((r) => r.distanceM)) : 0;
  $("stat-week").textContent = (weekM / 1000).toFixed(1) + " km";
  $("stat-total").textContent = (totalM / 1000).toFixed(1) + " km";
  $("stat-best").textContent = best ? fmtPace(best.distanceM, best.elapsedS).replace(" /km", "") : "—";
  $("stat-longest").textContent = longest ? fmtKm(longest) : "—";
}

function clearViewedRun() {
  if (viewedRunLine) { map.removeLayer(viewedRunLine); viewedRunLine = null; }
  viewedRunId = null;
}

function viewRun(run) {
  if (viewedRunId === run.id) { clearViewedRun(); renderRuns(); return; }
  clearViewedRun();
  viewedRunLine = L.polyline(run.coords, {
    color: "#7c3aed", weight: 5, opacity: 0.85, dashArray: "1 8",
    lineCap: "round",
  }).addTo(map);
  map.fitBounds(viewedRunLine.getBounds(), { padding: [40, 40] });
  viewedRunId = run.id;
  renderRuns();
}

function deleteRun(run) {
  if (!confirm(`Delete this run? ${fmtKm(run.distanceM)} on ${fmtDate(run.date)}`)) return;
  if (viewedRunId === run.id) clearViewedRun();
  persistRuns(loadRuns().filter((r) => r.id !== run.id));
  renderRuns();
  toast("Run deleted");
}

function renderRuns() {
  const list = $("runs-list");
  const runs = loadRuns();
  renderRunStats(runs);
  list.innerHTML = "";
  if (!runs.length) {
    list.innerHTML = '<div class="runs-empty">No runs saved yet — finish a loop and hit 💾 Save run.</div>';
    return;
  }
  for (const run of runs) {
    const item = document.createElement("div");
    item.className = "run-item";

    const meta = document.createElement("div");
    meta.className = "run-meta";
    meta.innerHTML =
      `<strong>${fmtKm(run.distanceM)}</strong> · ${fmtElapsed(run.elapsedS)} · ${fmtPace(run.distanceM, run.elapsedS)}` +
      (run.climbM != null ? ` · ↗${run.climbM} m` : "") +
      (run.completed ? "" : " · partial") +
      (run.sim ? '<span class="sim-tag">sim</span>' : "") +
      `<br><span class="run-date">${fmtDate(run.date)}</span>`;

    const actions = document.createElement("div");
    actions.className = "run-actions";

    const view = document.createElement("button");
    view.className = "icon-btn" + (viewedRunId === run.id ? " viewing" : "");
    view.title = viewedRunId === run.id ? "Hide route" : "View route on map";
    view.textContent = "👁";
    view.addEventListener("click", () => viewRun(run));

    const gpx = document.createElement("button");
    gpx.className = "icon-btn";
    gpx.title = "Download GPX (for Strava, Garmin, etc.)";
    gpx.textContent = "⬇";
    gpx.addEventListener("click", () => downloadRunGpx(run));

    const del = document.createElement("button");
    del.className = "icon-btn danger";
    del.title = "Delete run";
    del.textContent = "🗑";
    del.addEventListener("click", () => deleteRun(run));

    actions.append(view, gpx, del);
    item.append(meta, actions);
    list.appendChild(item);
  }
}

function openRuns() {
  renderRuns();
  $("plan-panel").hidden = true;
  $("runs-panel").hidden = false;
}

function closeRuns() {
  clearViewedRun();
  $("runs-panel").hidden = true;
  $("plan-panel").hidden = false;
}

function toggleAvoidMode() {
  avoidMode = !avoidMode;
  $("btn-avoid").setAttribute("aria-pressed", String(avoidMode));
  if (avoidMode) {
    updateAvoidHint();
  } else {
    $("hint").textContent = start
      ? "Start point set. Choose a distance and generate your loop."
      : "Tap the map to set your start point, or use My location.";
  }
}

function init() {
  map = L.map("map", { zoomControl: false }).setView([53.5, -2.0], 6); // UK
  L.control.zoom({ position: "bottomright" }).addTo(map);
  L.tileLayer("https://tile.openstreetmap.org/{z}/{x}/{y}.png", {
    maxZoom: 19,
    attribution: '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors',
  }).addTo(map);

  map.on("click", (e) => {
    if (nav) return;
    if (avoidMode) addAvoidPoint(e.latlng);
    else setStart(e.latlng);
  });

  $("ors-key").value = orsKey;
  $("ors-key").addEventListener("change", (e) => {
    orsKey = e.target.value.trim();
    localStorage.setItem("loopback-ors-key", orsKey);
    toast(orsKey ? "OpenRouteService key saved — walking profile enabled" : "Key removed — using OSRM");
  });

  $("btn-locate").addEventListener("click", locateMe);
  $("btn-avoid").addEventListener("click", toggleAvoidMode);
  $("btn-generate").addEventListener("click", onGenerate);
  $("btn-variation").addEventListener("click", onGenerate);
  $("btn-start").addEventListener("click", startRun);
  $("btn-stop").addEventListener("click", () => endRun(false));
  $("btn-simulate").addEventListener("click", toggleSimulation);
  $("btn-save-run").addEventListener("click", () => closeSummary(true));
  $("btn-discard-run").addEventListener("click", () => closeSummary(false));
  $("btn-runs").addEventListener("click", openRuns);
  $("btn-runs-back").addEventListener("click", closeRuns);
  $("btn-help").addEventListener("click", () => {
    $("plan-panel").hidden = true;
    $("help-panel").hidden = false;
  });
  $("btn-help-back").addEventListener("click", () => {
    $("help-panel").hidden = true;
    $("plan-panel").hidden = false;
  });
  // Wake locks are released when the tab is backgrounded — take it back when
  // the runner returns to the app mid-run.
  document.addEventListener("visibilitychange", () => {
    if (nav && document.visibilityState === "visible") acquireWakeLock();
  });
  updateRunsBadge();
  $("btn-mute").addEventListener("click", () => {
    voiceOn = !voiceOn;
    if (!voiceOn) speechSynthesis?.cancel();
    $("btn-mute").innerHTML = voiceOn
      ? '<span class="btn-icon">🔊</span>Voice'
      : '<span class="btn-icon">🔇</span>Muted';
  });

  if ("serviceWorker" in navigator) {
    navigator.serviceWorker.register("./sw.js").catch(() => {
      // e.g. file:// or an unsupported browser — the app still works online
    });
  }
}

init();
