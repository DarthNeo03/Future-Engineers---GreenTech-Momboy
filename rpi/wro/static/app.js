/* Panel de depuracion WRO Future Engineers 2026 - JS sin dependencias */
'use strict';

const $  = (s) => document.querySelector(s);
const $$ = (s) => Array.from(document.querySelectorAll(s));

const api = {
  async status()      { return (await fetch('/api/status')).json(); },
  async schema()      { return (await fetch('/api/schema')).json(); },
  async setCfg(o)     { return (await fetch('/api/config', {method:'POST',
                          headers:{'Content-Type':'application/json'},
                          body:JSON.stringify(o)})).json(); },
  async cmd(o)        { return (await fetch('/api/command', {method:'POST',
                          headers:{'Content-Type':'application/json'},
                          body:JSON.stringify(o)})).json(); },
  async reset(keys)   { return (await fetch('/api/config/reset', {method:'POST',
                          headers:{'Content-Type':'application/json'},
                          body:JSON.stringify({keys})})).json(); },
};

/* ===================== video ===================== */
$$('#viewTabs button').forEach(b => b.onclick = () => {
  $$('#viewTabs button').forEach(x => x.classList.remove('on'));
  b.classList.add('on');
  $('#stream').src = '/stream.mjpg?view=' + b.dataset.view + '&t=' + Date.now();
});

/* ===================== modo / armado ===================== */
let state = {mode:'open', armed:false};

$$('.mode').forEach(b => b.onclick = async () => {
  await api.cmd({cmd:'mode', mode:b.dataset.mode});
});
$('#btnArm').onclick  = () => api.cmd({cmd:'arm'});
$('#btnStop').onclick = () => api.cmd({cmd:'disarm'});

document.addEventListener('keydown', e => {
  if (e.target.tagName === 'INPUT') return;
  if (e.code === 'Space') { e.preventDefault(); api.cmd({cmd:'disarm'}); }
});

/* ===================== calibracion de inclinacion ===================== */
$('#btnCal').onclick = async () => {
  const d = parseFloat($('#calDist').value) || 400;
  $('#calMsg').textContent = 'calibrando...';
  await api.cmd({cmd:'calibrate_pitch', distance_mm:d});
};

/* ===================== telemetria ===================== */
function pill(txt, cls) { return `<span class="pill ${cls||''}">${txt}</span>`; }
function row(k, v, cls) { return `<div class="k">${k}</div><div class="v ${cls||''}">${v}</div>`; }
const n = (v, d) => (v === null || v === undefined) ? '&mdash;'
        : (typeof v === 'number' ? v.toFixed(d === undefined ? 0 : d) : v);

async function tick() {
  let s;
  try { s = await api.status(); } catch (e) { return; }

  state.mode = s.mode; state.armed = s.armed;
  $$('.mode').forEach(b => b.classList.toggle('on', b.dataset.mode === s.mode));
  $('#btnArm').classList.toggle('on', s.armed);
  $('#btnArm').textContent = s.armed ? 'ARMADO' : 'ARMAR';
  $('#joystick').classList.toggle('hidden', s.mode !== 'manual');

  const c = s.ctrl, e = s.esp, v = s.vision;
  $('#pills').innerHTML =
      pill(`cam <b>${n(s.cam_fps,1)}</b> fps`, s.cam_ok ? 'ok' : 'bad')
    + pill(`lazo <b>${n(s.loop_hz,1)}</b> Hz`)
    + pill(`ESP32 <b>${e.connected ? 'OK' : 'OFF'}</b>`, e.connected ? 'ok' : 'bad')
    + pill(`<b>${c.state}</b>`, s.armed ? 'ok' : '')
    + pill(`sentido <b>${c.direction_txt}</b>`, c.direction ? 'ok' : 'warn')
    + pill(`vuelta <b>${c.laps}</b> · esq <b>${c.corners}</b>`)
    + pill(`t <b>${n(c.elapsed,1)}</b> s`)
    + (e.watchdog ? pill('WATCHDOG', 'bad') : '')
    + (s.error ? pill(s.error, 'bad') : '');

  $('#tState').innerHTML =
      row('estado', c.state)
    + row('mensaje', c.note || s.msg)
    + row('sentido', `${c.direction_txt} (${c.dir_source})`)
    + row('rumbo objetivo', n(c.target_yaw,1) + '&deg;')
    + row('error de rumbo', n(c.head_err,1) + '&deg;',
          Math.abs(c.head_err) > 25 ? 'warn' : '')
    + row('correccion lateral', n(c.head_corr,1) + '&deg;')
    + row('deriva corregida', n(c.yaw_bias,2) + '&deg;')
    + row('objetivo lateral', n(c.target_lat) + ' mm')
    + row('error lateral', n(c.lat_err) + ' mm')
    + row('direccion', n(c.steer) + ' %')
    + row('velocidad', n(c.speed) + ' %');

  const L = v.left, R = v.right;
  $('#tVision').innerHTML =
      row('puntos de contorno', v.n_points, v.n_points < 40 ? 'warn' : 'ok')
    + row('tramos', v.segments)
    + row('umbral usado', v.thresh)
    + row('fila ROI', v.roi_top)
    + row('frente', n(v.front) + ' mm')
    + row('frente (min)', n(v.front_min) + ' mm', v.front_min < 250 ? 'bad' : '')
    + row('muro izq.', L ? `${n(L.dist)} mm / ${n(L.angle,1)}&deg; q${n(L.q,2)}` : '&mdash;')
    + row('fin izq.', L && L.end !== null ? n(L.end) + ' mm' : '&mdash;')
    + row('muro der.', R ? `${n(R.dist)} mm / ${n(R.angle,1)}&deg; q${n(R.q,2)}` : '&mdash;')
    + row('fin der.', R && R.end !== null ? n(R.end) + ' mm' : '&mdash;')
    + row('ancho pasillo', v.corridor_check
        ? `${n(v.corridor)} mm (ref ${v.corridor_check.ref}, ${v.corridor_check.err_pct > 0 ? '+' : ''}${v.corridor_check.err_pct}%)`
        : n(v.corridor) + ' mm',
        v.corridor_check ? v.corridor_check.level : '')
    + row('interior', n(c.d_inner) + ' mm')
    + row('exterior', n(c.d_outer) + ' mm', c.d_outer !== null && c.d_outer < 200 ? 'bad' : '')
    + row('fin muro interior', n(c.inner_end) + ' mm');

  const lineTxt = ['-', 'NARANJA', 'AZUL'][e.line] || '-';
  $('#tEsp').innerHTML =
      row('puerto', e.port || '&mdash;', e.connected ? 'ok' : 'bad')
    + row('yaw', n(e.yaw,2) + '&deg;')
    + row('vel. angular', n(e.gz,1) + ' &deg;/s')
    + row('aceleracion', n(e.accel,2) + ' g')
    + row('linea actual', lineTxt, e.line ? 'ok' : '')
    + row('naranja / azul', `${e.n_orange} / ${e.n_blue}`)
    + row('r g b', e.rgb.map(x => x.toFixed(2)).join(' '))
    + row('C (luz)', e.c)
    + row('boton', e.button ? 'PULSADO' : '-')
    + row('edad telemetria', n(e.age,2) + ' s', e.age > 0.5 ? 'bad' : '')
    + row('pilares', s.pillars.length
        ? s.pillars.map(p => `${p.color[0].toUpperCase()}${p.x}`).join(' ') : '&mdash;');

  if (s.cal) $('#calMsg').textContent = s.cal;
}
setInterval(tick, 220); tick();

/* ===================== joystick ===================== */
(function () {
  const pad = $('#pad'), knob = $('#knob');
  let active = false, sx = 0, sy = 0;
  const keys = {};

  function show() {
    const mx = parseFloat($('#jsMax').value);
    $('#jsSteer').textContent = Math.round(sx * 100);
    $('#jsSpeed').textContent = Math.round(sy * mx);
    knob.style.left = (50 + sx * 42) + '%';
    knob.style.top  = (50 - sy * 42) + '%';
  }
  function fromEvent(ev) {
    const r = pad.getBoundingClientRect();
    const t = ev.touches ? ev.touches[0] : ev;
    sx = Math.max(-1, Math.min(1, ((t.clientX - r.left) / r.width  - 0.5) * 2));
    sy = Math.max(-1, Math.min(1, (0.5 - (t.clientY - r.top) / r.height) * 2));
    if (Math.abs(sx) < 0.08) sx = 0;
    if (Math.abs(sy) < 0.08) sy = 0;
    show();
  }
  const down = e => { active = true; fromEvent(e); e.preventDefault(); };
  const move = e => { if (active) { fromEvent(e); e.preventDefault(); } };
  const up   = () => { active = false; sx = sy = 0; show(); };

  pad.addEventListener('mousedown', down);
  window.addEventListener('mousemove', move);
  window.addEventListener('mouseup', up);
  pad.addEventListener('touchstart', down, {passive:false});
  pad.addEventListener('touchmove', move, {passive:false});
  window.addEventListener('touchend', up);

  $('#jsMax').oninput = () => { $('#jsMaxV').textContent = $('#jsMax').value; show(); };

  document.addEventListener('keydown', e => {
    if (e.target.tagName === 'INPUT') return;
    if (['ArrowUp','ArrowDown','ArrowLeft','ArrowRight'].includes(e.key)) {
      keys[e.key] = true; e.preventDefault();
    }
  });
  document.addEventListener('keyup', e => { keys[e.key] = false; });

  // Envio periodico: 20 Hz. El firmware frena solo si dejan de llegar comandos.
  setInterval(() => {
    if (state.mode !== 'manual') return;
    let x = sx, y = sy;
    if (!active) {
      x = (keys.ArrowLeft ? 1 : 0) - (keys.ArrowRight ? 1 : 0);
      y = (keys.ArrowUp   ? 1 : 0) - (keys.ArrowDown  ? 1 : 0);
      if (x || y) { sx = x; sy = y; show(); }
      else if (sx || sy) { sx = sy = 0; show(); }
    }
    const mx = parseFloat($('#jsMax').value);
    api.cmd({cmd:'manual', steer: x * 100, speed: y * mx});
  }, 50);
})();

/* ===================== panel de calibracion ===================== */
let timers = {};
function push(key, val) {
  clearTimeout(timers[key]);
  timers[key] = setTimeout(async () => {
    const o = {}; o[key] = val;
    await api.setCfg(o);
    $('#cfgMsg').textContent = key + ' = ' + val;
  }, 140);
}

function buildParam(p, value) {
  const d = document.createElement('div');
  d.className = 'p';
  d.dataset.key = p.key;
  d.dataset.adv = p.advanced ? '1' : '0';
  d.dataset.search = (p.key + ' ' + p.label + ' ' + p.help).toLowerCase();

  const lb = document.createElement('div');
  lb.className = 'lb';
  lb.innerHTML = `${p.label}${p.target === 'esp32' ? ' <span class="esp">&#9679;</span>' : ''}
                  <span class="kk">${p.key}</span>`;
  d.appendChild(lb);

  let mid, box;
  if (p.kind === 'bool') {
    mid = document.createElement('div');
    box = document.createElement('input');
    box.type = 'checkbox'; box.checked = !!value;
    box.onchange = () => push(p.key, box.checked);
    mid.appendChild(box);
    d.appendChild(mid);
    d.appendChild(document.createElement('div'));
  } else if (p.kind === 'choice') {
    mid = document.createElement('div');
    box = document.createElement('select');
    p.choices.forEach(c => {
      const o = document.createElement('option');
      o.value = c; o.textContent = c; if (c === value) o.selected = true;
      box.appendChild(o);
    });
    box.onchange = () => push(p.key, box.value);
    mid.appendChild(box);
    d.appendChild(mid);
    d.appendChild(document.createElement('div'));
  } else if (p.kind === 'text') {
    mid = document.createElement('div');
    box = document.createElement('input');
    box.type = 'text'; box.value = value; box.style.width = '160px';
    box.onchange = () => push(p.key, box.value);
    mid.appendChild(box);
    d.appendChild(mid);
    d.appendChild(document.createElement('div'));
  } else {
    const step = p.step || (p.kind === 'int' ? 1 : 0.01);
    const sl = document.createElement('input');
    sl.type = 'range';
    sl.min = p.lo !== null ? p.lo : 0;
    sl.max = p.hi !== null ? p.hi : 100;
    sl.step = step; sl.value = value;
    const num = document.createElement('input');
    num.type = 'number'; num.step = step; num.value = value;
    if (p.lo !== null) num.min = p.lo;
    if (p.hi !== null) num.max = p.hi;
    sl.oninput = () => { num.value = sl.value; push(p.key, parseFloat(sl.value)); d.classList.add('changed'); };
    num.onchange = () => { sl.value = num.value; push(p.key, parseFloat(num.value)); d.classList.add('changed'); };
    d.appendChild(sl);
    d.appendChild(num);
  }

  const qm = document.createElement('div');
  qm.className = 'qm'; qm.textContent = '?';
  const help = document.createElement('div');
  help.className = 'help hidden';
  help.textContent = p.help + (p.target === 'esp32'
      ? '  [se envia al ESP32 al instante]' : '');
  qm.onclick = () => help.classList.toggle('hidden');
  d.appendChild(qm);
  d.appendChild(help);
  return d;
}

async function buildConfig() {
  const {params, values} = await api.schema();
  const groups = {};
  params.forEach(p => (groups[p.group] = groups[p.group] || []).push(p));
  const wrap = $('#cfgGroups');
  wrap.innerHTML = '';
  // Orden numerico por el prefijo del grupo ("2." antes que "10.")
  const gnum = g => parseInt(g, 10) || 999;
  Object.keys(groups).sort((a, b) => gnum(a) - gnum(b)).forEach((g, i) => {
    const det = document.createElement('details');
    if (i < 3) det.open = true;
    const sum = document.createElement('summary');
    sum.textContent = g;
    det.appendChild(sum);
    const list = document.createElement('div');
    list.className = 'plist';
    groups[g].forEach(p => list.appendChild(buildParam(p, values[p.key])));
    det.appendChild(list);
    wrap.appendChild(det);
  });
  applyFilter();
}

function applyFilter() {
  const q = $('#filter').value.trim().toLowerCase();
  const adv = $('#showAdv').checked;
  $$('#cfgGroups .p').forEach(el => {
    const okA = adv || el.dataset.adv === '0';
    const okQ = !q || el.dataset.search.includes(q);
    el.style.display = (okA && okQ) ? '' : 'none';
  });
  if (q) $$('#cfgGroups details').forEach(d => d.open = true);
}
$('#filter').oninput = applyFilter;
$('#showAdv').onchange = applyFilter;

$('#btnSave').onclick = async () => {
  await api.cmd({cmd:'save'});
  $('#cfgMsg').textContent = 'config.json guardado';
};
$('#btnDefaults').onclick = async () => {
  if (!confirm('¿Restaurar TODOS los parametros a los valores por defecto?')) return;
  await api.reset(null);
  await buildConfig();
  $('#cfgMsg').textContent = 'valores por defecto restaurados';
};

buildConfig();
