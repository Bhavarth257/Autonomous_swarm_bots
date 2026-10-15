// Rescue Mission Control: live view and operator control of the hexapod team.
'use strict';

const COLORS = ['#ff8c1a', '#33aaff', '#d24dff', '#33cc66', '#e6c619'];
const STATE_COLORS = {
  idle: '#6e7681', exploring: '#33aaff', returning: '#e6c619', done: '#3fb950', stopped: '#da3633',
  manual: '#ffb84d', 'operator goal': '#ffb84d', offline: '#484f58',
};
const KIND_COLORS = { visible: '#ff4d4f', buried: '#ff9f1a', hazard: '#3d8bff' };

const $ = (id) => document.getElementById(id);
let world = null;        // static info: building, spawns, extras, robots
let state = null;        // live state from the server
let selected = null;     // selected robot
let mode = 'select';
const layers = { map: true, rooms: true, trails: true, radio: false, goals: true, victims: true };
const view = { scale: 20, ox: 0, oy: 0, fitted: false };
let mapImg = new Image();       // the map being drawn
let mapMeta = null;             // its geometry
let mapLoading = false;
let mapVersion = null;
let ws = null;
let reqId = 0;

// ---------------------------------------------------------------- connection
function connect() {
  ws = new WebSocket(`ws://${location.host}/ws`);
  ws.onopen = () => $('conn').classList.add('ok');
  ws.onclose = () => { $('conn').classList.remove('ok'); setTimeout(connect, 1500); };
  ws.onmessage = (ev) => {
    const msg = JSON.parse(ev.data);
    if (msg.type === 'world') onWorld(msg);
    else if (msg.type === 'state') onState(msg);
    else if (msg.type === 'reply') onReply(msg);
  };
}
function send(obj) {
  if (ws && ws.readyState === WebSocket.OPEN) ws.send(JSON.stringify({ req: ++reqId, ...obj }));
}
function toast(text, ms = 3500) {
  const t = $('toast');
  t.textContent = text;
  t.classList.remove('hidden');
  clearTimeout(toast.timer);
  toast.timer = setTimeout(() => t.classList.add('hidden'), ms);
}
function onReply(msg) {
  if (!msg.ok) toast(`⚠ ${msg.error || 'command failed'}`, 5000);
  else if (msg.cmd === 'spawn_victim') toast(`Victim placed (${msg.id})`);
  else if (msg.path) toast(`Saved: ${msg.path}`, 6000);
}

// ---------------------------------------------------------------- world / state
function robotColor(r) { return COLORS[world.robots.indexOf(r) % COLORS.length]; }

const ROBOT_NAMES = {wheelleg: 'wheel-leg robots', rover: 'rovers', hexapod: 'hexapods'};

function onWorld(w) {
  world = w;
  $('subtitle').textContent = `seed ${w.seed} · ${w.building.cols}×${w.building.rows} rooms · ` +
    `${w.difficulty} · ${w.robots.length} ${ROBOT_NAMES[w.robot_type] || 'robots'}`;
  if (!selected) selected = w.robots[0];
  buildRobotCards();
  buildCams();
  fit();
}

function onState(s) {
  state = s;
  const t = s.mission_time;
  $('kpi-time').textContent = `${String(Math.floor(t / 60)).padStart(2, '0')}:${String(Math.floor(t % 60)).padStart(2, '0')}`;
  $('kpi-cov').textContent = `${Math.round(100 * s.coverage)} %`;
  $('kpi-vic').textContent = s.victims.length;
  const active = Object.values(s.robots).filter((r) => ['exploring', 'returning', 'manual', 'operator goal'].includes(r.state));
  $('kpi-act').textContent = `${active.length}/${Object.keys(s.robots).length}`;
  if (s.map && s.map.version !== mapVersion && !mapLoading) {
    // Load the new map in the background and swap it in when complete (double buffer):
    // swapping the src of the drawn image would leave nothing to draw while it loads.
    const meta = s.map;
    const next = new Image();
    mapLoading = true;
    next.onload = () => { mapImg = next; mapMeta = meta; mapVersion = meta.version; mapLoading = false; };
    next.onerror = () => { mapLoading = false; };
    next.src = `/api/map.png?v=${meta.version}`;
  }
  updateRobotCards();
  updateVictims();
  updateTimeline();
  drawCharts();
  updateTeleopPanel();
}

// ---------------------------------------------------------------- robot cards
function buildRobotCards() {
  const box = $('robots');
  box.innerHTML = '';
  for (const r of world.robots) {
    const el = document.createElement('div');
    el.className = 'robot';
    el.id = `card-${r}`;
    el.innerHTML = `
      <div class="head"><div class="name"><span class="swatch" style="background:${robotColor(r)}"></span>${r}</div>
        <span><span class="chip health" id="health-${r}" hidden></span> <span class="chip" id="chip-${r}">offline</span></span></div>
      <div class="stats">
        <span>position</span><span id="pos-${r}">–</span>
        <span>speed</span><span id="spd-${r}">–</span>
        <span>walked</span><span id="dst-${r}">–</span>
        <span>explored</span><span id="exp-${r}">–</span>
        <span>radio</span><span id="rad-${r}">–</span>
      </div>
      <div class="msg" id="msg-${r}"></div>
      <div class="btns">
        <button data-r="${r}" data-c="start" title="start / resume exploring">▶</button>
        <button data-r="${r}" data-c="stop" title="stop">■</button>
        <button data-r="${r}" data-c="return_home" title="return to the entrance">⌂</button>
        <button data-r="${r}" data-c="takeover" class="take" title="take manual control">✋ Take over</button>
      </div>`;
    el.addEventListener('click', (ev) => {
      const b = ev.target.closest('button');
      if (b) {
        ev.stopPropagation();
        send({ cmd: b.dataset.c, robot: b.dataset.r });
        if (b.dataset.c === 'takeover') selectRobot(b.dataset.r);
        return;
      }
      selectRobot(r);
    });
    box.appendChild(el);
  }
  selectRobot(selected);
}

function selectRobot(r) {
  selected = r;
  for (const x of world.robots) $(`card-${x}`).classList.toggle('selected', x === r);
  $('sel-label').textContent = r ? `selected: ${r}` : '';
}

function updateRobotCards() {
  for (const r of world.robots) {
    const s = state.robots[r];
    if (!s) continue;
    const chip = $(`chip-${r}`);
    chip.textContent = s.state;
    chip.style.background = (STATE_COLORS[s.state] || '#484f58') + '33';
    chip.style.color = STATE_COLORS[s.state] || '#c9d1d9';
    const hb = $(`health-${r}`);
    const bad = s.health && s.health !== 'ok';
    hb.hidden = !bad;
    if (bad) {
      hb.textContent = s.health;
      hb.className = `chip health ${s.health}`;
      hb.title = s.health_reason || '';
    }
    $(`pos-${r}`).textContent = s.pose ? `${s.pose[0].toFixed(1)}, ${s.pose[1].toFixed(1)} m` : '–';
    $(`spd-${r}`).textContent = `${s.speed.toFixed(2)} m/s`;
    $(`dst-${r}`).textContent = `${s.distance.toFixed(1)} m`;
    $(`exp-${r}`).textContent = `${Math.round(s.explored)} m²`;
    $(`rad-${r}`).textContent = s.peers.length ? s.peers.join(', ') : 'no contact';
    $(`msg-${r}`).textContent = s.message;
  }
}

// ---------------------------------------------------------------- cameras
function buildCams() {
  const box = $('cams');
  box.innerHTML = '';
  // one column per robot: camera row, then thermal row
  for (const [kind, label] of [['rgb', 'camera'], ['thermal', 'thermal']]) {
    for (const r of world.robots) {
      const el = document.createElement('div');
      el.className = 'cam';
      el.innerHTML = `<img alt=""><span class="tag" style="color:${robotColor(r)}">${r} · ${label}</span>`;
      el.addEventListener('click', () => openModal(r, kind, `${r} ${label}`));
      box.appendChild(el);
      pollFrames(el.querySelector('img'), r, kind, 250);
    }
  }
}
// Poll the newest frame (one request at a time per tile, no connection held open).
function pollFrames(img, r, kind, period, active = () => true) {
  const next = () => {
    if (!active()) return;
    const t = new Image();
    t.onload = () => { img.src = t.src; setTimeout(next, period); };
    t.onerror = () => setTimeout(next, 1000);         // no frame yet (204) or server busy
    t.src = `/api/frame/${r}/${kind}.jpg?t=${Date.now()}`;
  };
  next();
}
let modalOpen = false;
function openModal(r, kind, caption) {
  modalOpen = true;
  $('modal-cap').textContent = caption;
  $('modal').classList.remove('hidden');
  pollFrames($('modal-img'), r, kind, 120, () => modalOpen);
}
$('modal').addEventListener('click', () => { modalOpen = false; $('modal').classList.add('hidden'); });

// ---------------------------------------------------------------- victims
function updateVictims() {
  const box = $('victims');
  const list = state.victims;
  if (!list.length) { box.innerHTML = '<div class="empty">No victims reported yet</div>'; return; }
  const key = list.map((v) => `${v.n}:${v.kind}:${v.confidence}`).join('|');
  if (box.dataset.key === key) return;
  box.dataset.key = key;
  box.innerHTML = '';
  for (const v of list) {
    const el = document.createElement('div');
    el.className = 'victim';
    const s = v.sightings || {};
    el.innerHTML = `<img src="/api/snapshot/${v.n}.jpg" onerror="retryImg(this)" alt="">
      <div><div class="vt" style="color:${KIND_COLORS[v.kind]}">V${v.n} · ${v.kind}</div>
        <div class="vd">(${v.x.toFixed(1)}, ${v.y.toFixed(1)}) m · found by ${v.found_by || '?'} at ${Math.round(v.first_seen || 0)} s</div>
        <div class="vd">camera ${s.rgb || 0} · thermal ${s.thermal || 0} sightings</div></div>
      <div><div class="conf"><div style="width:${Math.round(100 * v.confidence)}%"></div></div>
        <div class="vd" style="text-align:right">${Math.round(100 * v.confidence)}%</div></div>`;
    el.addEventListener('click', () => { centerOn(v.x, v.y); showVictimPopup(v); });
    box.appendChild(el);
  }
}

// Snapshots can appear a moment after the victim does: retry quietly.
function retryImg(img) {
  img.style.visibility = 'hidden';
  if ((img.dataset.tries = (Number(img.dataset.tries) || 0) + 1) > 20) return;
  setTimeout(() => { img.style.visibility = ''; img.src = img.src.split('?')[0] + `?t=${Date.now()}`; }, 3000);
}
window.retryImg = retryImg;

function showVictimPopup(v) {
  const p = $('popup');
  const s = v.sightings || {};
  p.innerHTML = `<span class="close">✕</span>
    <img src="/api/snapshot/${v.n}.jpg" onerror="this.style.display='none'" alt="">
    <h4 style="color:${KIND_COLORS[v.kind]}">V${v.n} · ${v.kind} victim</h4>
    <div class="row"><span>position</span><b>(${v.x.toFixed(1)}, ${v.y.toFixed(1)}) m</b></div>
    <div class="row"><span>confidence</span><b>${Math.round(100 * v.confidence)} %</b></div>
    <div class="row"><span>found by</span><b>${v.found_by || '?'} at ${Math.round(v.first_seen || 0)} s</b></div>
    <div class="row"><span>evidence</span><b>camera ${s.rgb || 0} · thermal ${s.thermal || 0}</b></div>
    <div class="row"><span>max temperature</span><b>${v.max_temp_c != null ? v.max_temp_c + ' °C' : '–'}</b></div>`;
  const [sx, sy] = toScreen(v.x, v.y);
  const c = $('map');
  p.style.left = `${Math.min(Math.max(10, sx + 14), c.clientWidth - 300)}px`;
  p.style.top = `${Math.min(Math.max(50, sy - 60), c.clientHeight - 330)}px`;
  p.classList.remove('hidden');
  p.querySelector('.close').onclick = () => p.classList.add('hidden');
}

// ---------------------------------------------------------------- timeline
function updateTimeline() {
  const box = $('timeline');
  const evs = state.events;
  const key = evs.length ? `${evs.length}:${evs[evs.length - 1].text}` : '0';
  if (box.dataset.key === key) return;
  box.dataset.key = key;
  box.innerHTML = '';
  for (const e of evs) {
    const el = document.createElement('div');
    el.className = `ev ${e.kind}`;
    const m = Math.floor(e.t / 60), s = Math.floor(e.t % 60);
    el.innerHTML = `<span class="t">${String(m).padStart(2, '0')}:${String(s).padStart(2, '0')}</span><span class="x"></span>`;
    el.querySelector('.x').textContent = e.text;
    box.prepend(el);
  }
}

// ---------------------------------------------------------------- charts
function fitCanvas(c) {
  const r = c.getBoundingClientRect();
  const dpr = window.devicePixelRatio || 1;
  if (c.width !== Math.round(r.width * dpr) || c.height !== Math.round(r.height * dpr)) {
    c.width = Math.round(r.width * dpr);
    c.height = Math.round(r.height * dpr);
  }
  const ctx = c.getContext('2d');
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  return [ctx, r.width, r.height];
}

function lineChart(canvas, t, series, yMax, unit) {
  const [ctx, w, h] = fitCanvas(canvas);
  ctx.clearRect(0, 0, w, h);
  const L = 34, R = 8, T = 8, B = 18;
  const tMax = Math.max(60, t.length ? t[t.length - 1] : 60);
  ctx.strokeStyle = '#2a313c';
  ctx.fillStyle = '#8b949e';
  ctx.font = '10px sans-serif';
  ctx.lineWidth = 1;
  for (let i = 0; i <= 4; i++) {
    const y = T + (h - T - B) * (1 - i / 4);
    ctx.beginPath(); ctx.moveTo(L, y); ctx.lineTo(w - R, y); ctx.stroke();
    ctx.fillText(`${Math.round(yMax * i / 4)}${unit}`, 2, y + 3);
  }
  for (let i = 0; i <= 4; i++) {
    const x = L + (w - L - R) * i / 4;
    ctx.fillText(`${Math.round(tMax * i / 4 / 60)}m`, x - 6, h - 4);
  }
  for (const s of series) {
    ctx.strokeStyle = s.color;
    ctx.lineWidth = 2;
    ctx.beginPath();
    s.values.forEach((v, i) => {
      const x = L + (w - L - R) * t[i] / tMax;
      const y = T + (h - T - B) * (1 - Math.min(v, yMax) / yMax);
      i ? ctx.lineTo(x, y) : ctx.moveTo(x, y);
    });
    ctx.stroke();
    if (s.label && s.values.length) {
      ctx.fillStyle = s.color;
      ctx.fillText(s.label, w - R - ctx.measureText(s.label).width, T + 10 + 12 * series.indexOf(s));
    }
  }
}

function drawCharts() {
  const h = state.history;
  if (!h || !h.t.length) return;
  const vMax = Math.max(6, ...h.victims);
  // victims scaled onto the 0-100 axis so both fit one chart
  lineChart($('chart-cov'), h.t, [
    { values: h.coverage, color: '#3fb950', label: 'explored %' },
    { values: h.victims.map((v) => 100 * v / vMax), color: '#ff4d4f', label: `victims (of ${vMax})` },
  ], 100, '');
  const dMax = Math.max(20, ...Object.values(h.distance).flat());
  lineChart($('chart-dist'), h.t, world.robots.map((r) => ({ values: h.distance[r], color: robotColor(r), label: r })),
    Math.ceil(dMax / 20) * 20, 'm');
}

// ---------------------------------------------------------------- map
const canvas = $('map');
function toScreen(x, y) { return [view.ox + x * view.scale, view.oy - y * view.scale]; }
function toWorld(sx, sy) { return [(sx - view.ox) / view.scale, (view.oy - sy) / view.scale]; }

function fit() {
  if (!world) return;
  const r = canvas.getBoundingClientRect();
  const W = world.building.width, H = world.building.height + 4.5;
  view.scale = Math.min((r.width - 40) / W, (r.height - 110) / H);
  view.ox = (r.width - W * view.scale) / 2;
  view.oy = 60 + (world.building.height + 0.5) * view.scale;
}
function centerOn(x, y) {
  const r = canvas.getBoundingClientRect();
  view.ox = r.width / 2 - x * view.scale;
  view.oy = r.height / 2 + y * view.scale;
}
function zoom(f, cx, cy) {
  const [wx, wy] = toWorld(cx, cy);
  view.scale = Math.max(5, Math.min(200, view.scale * f));
  view.ox = cx - wx * view.scale;
  view.oy = cy + wy * view.scale;
}

function draw() {
  requestAnimationFrame(draw);
  if (!world) return;
  const [ctx, w, h] = fitCanvas(canvas);
  ctx.fillStyle = '#0b0f14';
  ctx.fillRect(0, 0, w, h);
  const s = view.scale;
  const B = world.building;
  const cw = B.width / B.cols, rh = B.height / B.rows;

  // building footprint and room grid (plan the rescuers have)
  let [x0, y0] = toScreen(0, B.height);
  ctx.fillStyle = '#10151c';
  ctx.fillRect(x0, y0, B.width * s, B.height * s);
  // combined robot maps
  if (layers.map && mapMeta && mapImg.naturalWidth) {
    const m = mapMeta;
    const [mx, my] = toScreen(m.x0, m.y0 + m.h * m.res);
    ctx.imageSmoothingEnabled = false;
    ctx.drawImage(mapImg, mx, my, m.w * m.res * s, m.h * m.res * s);
  }
  // hard-mode rooms: smoke and dark
  for (const e of world.extras || []) {
    let ex0, ey0, ex1, ey1;
    if (e.type === 'smoke') { ex0 = e.x - e.sx / 2; ex1 = e.x + e.sx / 2; ey0 = e.y - e.sy / 2; ey1 = e.y + e.sy / 2; }
    else { ex0 = e.room[0] * cw; ex1 = ex0 + cw; ey0 = e.room[1] * rh; ey1 = ey0 + rh; }
    const [a, b] = toScreen(ex0, ey1);
    ctx.save();
    ctx.beginPath(); ctx.rect(a, b, (ex1 - ex0) * s, (ey1 - ey0) * s); ctx.clip();
    ctx.strokeStyle = e.type === 'smoke' ? '#8fa3c855' : '#00000088';
    ctx.lineWidth = 2;
    for (let k = -h; k < w + h; k += 12) { ctx.beginPath(); ctx.moveTo(k, 0); ctx.lineTo(k + h, h); ctx.stroke(); }
    ctx.restore();
    ctx.fillStyle = e.type === 'smoke' ? '#9fb0d0' : '#8b949e';
    ctx.font = '11px sans-serif';
    ctx.fillText(e.type === 'smoke' ? 'smoke' : 'dark (roof)', a + 4, b + 13);
  }
  // who searches which room (swarm room book): owner's colour, fainter once done
  if (layers.rooms && state && state.rooms) {
    for (const room of state.rooms) {
      const [rx0, ry0, rx1, ry1] = room.rect;
      const [a, b] = toScreen(rx0, ry1);
      const col = world.robots.includes(room.owner) ? robotColor(room.owner) : '#888888';
      ctx.fillStyle = col + (room.done ? '0d' : '24');
      ctx.fillRect(a, b, (rx1 - rx0) * s, (ry1 - ry0) * s);
      ctx.fillStyle = col + 'cc';
      ctx.font = '11px sans-serif';
      ctx.textAlign = 'right';
      ctx.fillText(room.owner + (room.done ? ' ✓' : ''), a + (rx1 - rx0) * s - 4, b + (ry1 - ry0) * s - 4);
      ctx.textAlign = 'left';
    }
  }
  ctx.strokeStyle = '#30363d';
  ctx.setLineDash([4, 6]);
  ctx.lineWidth = 1;
  for (let i = 1; i < B.cols; i++) { const [a, b] = toScreen(i * cw, B.height); ctx.beginPath(); ctx.moveTo(a, b); ctx.lineTo(a, b + B.height * s); ctx.stroke(); }
  for (let j = 1; j < B.rows; j++) { const [a, b] = toScreen(0, j * rh); ctx.beginPath(); ctx.moveTo(a, b); ctx.lineTo(a + B.width * s, b); ctx.stroke(); }
  ctx.setLineDash([]);
  ctx.strokeStyle = '#58606b';
  ctx.lineWidth = 1.5;
  ctx.strokeRect(x0, y0, B.width * s, B.height * s);
  const en = world.entrance;
  const [ea, eb] = toScreen(en.x - en.width / 2, 0);
  ctx.strokeStyle = '#3fb950'; ctx.lineWidth = 4;
  ctx.beginPath(); ctx.moveTo(ea, eb); ctx.lineTo(ea + en.width * s, eb); ctx.stroke();
  ctx.fillStyle = '#3fb950'; ctx.font = '11px sans-serif'; ctx.fillText('entrance', ea, eb + 16);

  if (!state) return;
  const robots = state.robots;
  // radio range and links
  if (layers.radio) {
    for (const r of world.robots) {
      const p = robots[r] && robots[r].pose;
      if (!p) continue;
      const [a, b] = toScreen(p[0], p[1]);
      ctx.fillStyle = robotColor(r) + '12'; ctx.strokeStyle = robotColor(r) + '55'; ctx.lineWidth = 1;
      ctx.beginPath(); ctx.arc(a, b, state.radio_range * s, 0, 2 * Math.PI); ctx.fill(); ctx.stroke();
    }
  }
  for (const [ra, rb] of state.links) {
    const pa = robots[ra] && robots[ra].pose, pb = robots[rb] && robots[rb].pose;
    if (!pa || !pb) continue;
    const [a1, b1] = toScreen(pa[0], pa[1]), [a2, b2] = toScreen(pb[0], pb[1]);
    ctx.strokeStyle = '#3fb950aa'; ctx.lineWidth = 2; ctx.setLineDash([3, 5]);
    ctx.beginPath(); ctx.moveTo(a1, b1); ctx.lineTo(a2, b2); ctx.stroke(); ctx.setLineDash([]);
  }
  // trails and goals
  for (const r of world.robots) {
    const rs = robots[r];
    if (!rs) continue;
    if (layers.trails && rs.trail.length > 1) {
      ctx.strokeStyle = robotColor(r) + 'bb'; ctx.lineWidth = 2;
      ctx.beginPath();
      rs.trail.forEach(([x, y], i) => { const [a, b] = toScreen(x, y); i ? ctx.lineTo(a, b) : ctx.moveTo(a, b); });
      ctx.stroke();
    }
    if (layers.goals && rs.goal && rs.pose) {
      const [a1, b1] = toScreen(rs.pose[0], rs.pose[1]), [a2, b2] = toScreen(rs.goal[0], rs.goal[1]);
      ctx.strokeStyle = robotColor(r); ctx.lineWidth = 1.2; ctx.setLineDash([5, 4]);
      ctx.beginPath(); ctx.moveTo(a1, b1); ctx.lineTo(a2, b2); ctx.stroke(); ctx.setLineDash([]);
      ctx.beginPath(); ctx.arc(a2, b2, 6, 0, 2 * Math.PI); ctx.stroke();
      ctx.beginPath(); ctx.arc(a2, b2, 2, 0, 2 * Math.PI); ctx.fillStyle = robotColor(r); ctx.fill();
    }
  }
  // victims and hazards
  if (layers.victims) {
    for (const v of state.victims.concat(state.hazards.map((x) => ({ ...x, kind: 'hazard' })))) {
      const [a, b] = toScreen(v.x, v.y);
      const c = KIND_COLORS[v.kind] || '#aaa';
      ctx.fillStyle = c + '33';
      ctx.beginPath(); ctx.arc(a, b, 13, 0, 2 * Math.PI); ctx.fill();
      ctx.fillStyle = c; ctx.strokeStyle = '#fff'; ctx.lineWidth = 1.5;
      ctx.beginPath(); ctx.arc(a, b, 7, 0, 2 * Math.PI); ctx.fill(); ctx.stroke();
      ctx.fillStyle = '#fff'; ctx.font = 'bold 11px sans-serif';
      ctx.fillText(v.kind === 'hazard' ? 'HOT' : `V${v.n}`, a + 10, b - 8);
    }
  }
  // robots
  for (const r of world.robots) {
    const rs = robots[r];
    if (!rs || !rs.pose) continue;
    const [a, b] = toScreen(rs.pose[0], rs.pose[1]);
    const yaw = rs.pose[2];
    const R = Math.max(8, 0.35 * s);
    ctx.save();
    ctx.translate(a, b);
    if (r === selected) {
      ctx.strokeStyle = '#ffffffaa'; ctx.lineWidth = 2;
      ctx.beginPath(); ctx.arc(0, 0, R + 6, 0, 2 * Math.PI); ctx.stroke();
    }
    ctx.strokeStyle = STATE_COLORS[rs.state] || '#888'; ctx.lineWidth = 3;
    ctx.beginPath(); ctx.arc(0, 0, R + 2, 0, 2 * Math.PI); ctx.stroke();
    ctx.rotate(-yaw);
    ctx.fillStyle = robotColor(r);
    ctx.beginPath();
    for (let k = 0; k < 6; k++) {
      const ang = Math.PI / 6 + k * Math.PI / 3;
      ctx[k ? 'lineTo' : 'moveTo'](R * Math.cos(ang), R * Math.sin(ang));
    }
    ctx.closePath(); ctx.fill();
    ctx.fillStyle = '#fff';
    ctx.beginPath(); ctx.moveTo(R + 5, 0); ctx.lineTo(R - 3, -4); ctx.lineTo(R - 3, 4); ctx.closePath(); ctx.fill();
    ctx.restore();
    ctx.fillStyle = '#fff'; ctx.font = 'bold 12px sans-serif';
    ctx.fillText(r, a + R + 6, b + R + 8);
  }
  // placement preview
  if (mode !== 'select' && hover) {
    const [a, b] = toScreen(hover[0], hover[1]);
    ctx.strokeStyle = mode === 'place' ? '#ff4d4f' : robotColor(selected);
    ctx.lineWidth = 2;
    ctx.beginPath(); ctx.arc(a, b, 10, 0, 2 * Math.PI); ctx.stroke();
    ctx.fillStyle = '#fff'; ctx.font = '11px sans-serif';
    ctx.fillText(mode === 'place' ? `place ${$('place-kind').value} victim` : `send ${selected} here`, a + 14, b + 4);
  }
}

// map interaction
let drag = null;
let hover = null;
canvas.addEventListener('mousedown', (e) => { drag = { x: e.offsetX, y: e.offsetY, ox: view.ox, oy: view.oy, moved: false }; });
window.addEventListener('mouseup', (e) => {
  if (drag && !drag.moved && e.target === canvas) onMapClick(e.offsetX, e.offsetY);
  drag = null;
});
canvas.addEventListener('mousemove', (e) => {
  hover = toWorld(e.offsetX, e.offsetY);
  if (!drag) return;
  const dx = e.offsetX - drag.x, dy = e.offsetY - drag.y;
  if (Math.abs(dx) + Math.abs(dy) > 3) drag.moved = true;
  view.ox = drag.ox + dx; view.oy = drag.oy + dy;
});
canvas.addEventListener('mouseleave', () => { hover = null; });
canvas.addEventListener('wheel', (e) => { e.preventDefault(); zoom(e.deltaY < 0 ? 1.15 : 1 / 1.15, e.offsetX, e.offsetY); }, { passive: false });
$('zoom-in').onclick = () => { const r = canvas.getBoundingClientRect(); zoom(1.3, r.width / 2, r.height / 2); };
$('zoom-out').onclick = () => { const r = canvas.getBoundingClientRect(); zoom(1 / 1.3, r.width / 2, r.height / 2); };
$('zoom-fit').onclick = fit;
window.addEventListener('resize', () => { if (!view.moved) fit(); });

function onMapClick(sx, sy) {
  const [x, y] = toWorld(sx, sy);
  if (mode === 'goto') {
    if (!selected) return toast('Select a robot first');
    send({ cmd: 'goto', robot: selected, x, y });
    toast(`${selected} → (${x.toFixed(1)}, ${y.toFixed(1)}); it will hold there until you resume autonomy`);
    return;
  }
  if (mode === 'place') {
    const B = world.building;
    if (x < 0.3 || y < 0.3 || x > B.width - 0.3 || y > B.height - 0.3) return toast('Place victims inside the building');
    const kind = $('place-kind').value;
    send({ cmd: 'spawn_victim', x, y, posture: kind === 'sitting' ? 'sitting' : 'lying', buried: kind === 'buried' });
    toast('Placing victim…');
    return;
  }
  // select: robots first, then victims
  if (state) {
    for (const r of world.robots) {
      const p = state.robots[r] && state.robots[r].pose;
      if (p && Math.hypot(p[0] - x, p[1] - y) < Math.max(0.6, 14 / view.scale)) { selectRobot(r); return; }
    }
    for (const v of state.victims) {
      if (Math.hypot(v.x - x, v.y - y) < Math.max(0.7, 14 / view.scale)) { showVictimPopup(v); return; }
    }
  }
  $('popup').classList.add('hidden');
}

// toolbar
for (const b of document.querySelectorAll('#modes .tool')) {
  b.addEventListener('click', () => {
    mode = b.dataset.mode;
    for (const x of document.querySelectorAll('#modes .tool')) x.classList.toggle('active', x === b);
    canvas.className = `mode-${mode}`;
    $('hint').textContent = { select: 'Drag to pan · scroll to zoom · click a robot or victim',
      goto: `Click where ${selected || 'the selected robot'} should go (it holds there; Resume autonomy to continue)`,
      place: 'Click inside the building to place the victim in the running simulation' }[mode];
  });
}
for (const cb of document.querySelectorAll('#layers input')) cb.addEventListener('change', () => { layers[cb.dataset.layer] = cb.checked; });
for (const b of document.querySelectorAll('.actions [data-cmd]')) b.addEventListener('click', () => send({ cmd: b.dataset.cmd, robot: 'all' }));
$('btn-report').onclick = () => send({ cmd: 'save_report' });
$('btn-scenario').onclick = () => send({ cmd: 'save_scenario' });

// ---------------------------------------------------------------- teleop
const tele = { vx: 0, vy: 0, wz: 0, keys: new Set(), stick: null };
function updateTeleopPanel() {
  const rs = selected && state && state.robots[selected];
  const manual = rs && (rs.state === 'manual' || rs.state === 'operator goal');
  $('teleop').classList.toggle('hidden', !manual);
  $('teleop-robot').textContent = selected || '';
}
$('btn-resume').onclick = () => selected && send({ cmd: 'resume', robot: selected });
$('teleop-speed').oninput = () => { $('teleop-speed-val').textContent = Number($('teleop-speed').value).toFixed(2); };

window.addEventListener('keydown', (e) => {
  if (e.target.tagName === 'INPUT' || e.target.tagName === 'SELECT') return;
  const k = e.key.toLowerCase();
  if ('wasdqe'.includes(k) && k.length === 1) tele.keys.add(k);
});
window.addEventListener('keyup', (e) => tele.keys.delete(e.key.toLowerCase()));
window.addEventListener('blur', () => tele.keys.clear());

const joy = $('joystick');
function stickFrom(e) {
  const r = joy.getBoundingClientRect();
  const x = (e.clientX - r.left - r.width / 2) / (r.width / 2), y = (e.clientY - r.top - r.height / 2) / (r.height / 2);
  const n = Math.hypot(x, y);
  return n > 1 ? [x / n, y / n] : [x, y];
}
joy.addEventListener('pointerdown', (e) => { joy.setPointerCapture(e.pointerId); tele.stick = stickFrom(e); });
joy.addEventListener('pointermove', (e) => { if (tele.stick) tele.stick = stickFrom(e); });
joy.addEventListener('pointerup', () => { tele.stick = null; });
function drawJoystick() {
  const ctx = joy.getContext('2d');
  ctx.clearRect(0, 0, 150, 150);
  ctx.strokeStyle = '#30363d';
  ctx.beginPath(); ctx.arc(75, 75, 60, 0, 2 * Math.PI); ctx.stroke();
  ctx.beginPath(); ctx.moveTo(75, 15); ctx.lineTo(75, 135); ctx.moveTo(15, 75); ctx.lineTo(135, 75); ctx.stroke();
  const [x, y] = tele.stick || [0, 0];
  ctx.fillStyle = tele.stick ? '#3d8bff' : '#484f58';
  ctx.beginPath(); ctx.arc(75 + x * 55, 75 + y * 55, 18, 0, 2 * Math.PI); ctx.fill();
}
setInterval(() => {
  drawJoystick();
  if ($('teleop').classList.contains('hidden') || !selected) return;
  const v = Number($('teleop-speed').value);
  let vx = 0, vy = 0, wz = 0;
  if (tele.stick) { vx = -tele.stick[1] * v; wz = -tele.stick[0] * 1.0; }
  const K = tele.keys;
  if (K.has('w')) vx = v;
  if (K.has('s')) vx = -Math.min(v, 0.3);
  if (K.has('a')) wz = 0.8;
  if (K.has('d')) wz = -0.8;
  if (K.has('q')) vy = Math.min(v, 0.25);
  if (K.has('e')) vy = -Math.min(v, 0.25);
  const active = vx || vy || wz;
  if (active || tele.wasActive) send({ cmd: 'teleop', robot: selected, vx, vy, wz });
  tele.wasActive = Boolean(active);
}, 100);

connect();
draw();
