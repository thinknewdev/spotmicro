TEMPLATE = r'''<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1, user-scalable=no">
<title>SpotMicro Control</title>
<style>
  :root {
    --bg: #101318; --panel: #1a1f27; --edge: #2a3140;
    --text: #e6ebf2; --dim: #8b95a6; --accent: #4c8dff;
    --good: #3ecf8e; --warn: #ffb347; --bad: #ff5d6c;
  }
  * { box-sizing: border-box; margin: 0; padding: 0; -webkit-tap-highlight-color: transparent; }
  body {
    background: var(--bg); color: var(--text);
    font-family: -apple-system, "Segoe UI", Roboto, "DejaVu Sans", sans-serif;
    min-height: 100vh; display: flex; flex-direction: column; align-items: center;
    padding: 12px; gap: 12px;
  }
  .wrap { width: 100%; max-width: 1160px; display: flex; flex-direction: column; gap: 12px; }
  header { display: flex; align-items: center; gap: 10px; }
  header h1 { font-size: 20px; font-weight: 700; }
  #state {
    margin-left: auto; font-size: 13px; font-weight: 700; letter-spacing: .06em;
    padding: 4px 14px; border-radius: 999px; background: var(--panel); border: 1px solid var(--edge);
  }
  #state.STAND { color: var(--good); } #state.WALK, #state.CRAWL { color: var(--accent); }
  #state.SIT, #state.DANCE, #state.CALPOSE { color: var(--warn); } #state.REST { color: var(--dim); }

  .panel { background: var(--panel); border: 1px solid var(--edge); border-radius: 14px; overflow: hidden; }
  .cams { display: grid; gap: 12px; grid-template-columns: 1fr; }
  @media (min-width: 860px) { .cams { grid-template-columns: 1fr 1fr; } }
  .cams iframe { width: 100%; aspect-ratio: 16/9; border: 0; display: block; background: #000; }
  .cams .label {
    font-size: 11px; color: var(--dim); text-transform: uppercase; letter-spacing: .1em;
    padding: 6px 12px; border-top: 1px solid var(--edge);
  }

  .stats { display: grid; grid-template-columns: 1.4fr 1fr 1fr 1fr; }
  .stat { padding: 10px 8px; text-align: center; border-right: 1px solid var(--edge); }
  .stat:last-child { border-right: 0; }
  .stat .label { font-size: 11px; color: var(--dim); text-transform: uppercase; letter-spacing: .08em; }
  .stat .value { font-size: 22px; font-weight: 700; font-variant-numeric: tabular-nums; }
  #distance.near { color: var(--bad); } #distance.close { color: var(--warn); }

  .controls { display: grid; gap: 12px; grid-template-columns: 1fr; }
  @media (min-width: 860px) { .controls { grid-template-columns: 1fr 1fr; } }
  .section { padding: 12px 14px 14px; }
  .section h2 {
    font-size: 11px; color: var(--dim); text-transform: uppercase;
    letter-spacing: .1em; font-weight: 600; margin-bottom: 10px;
  }
  button {
    font: inherit; font-weight: 600; color: var(--text);
    background: #232a36; border: 1px solid var(--edge); border-radius: 12px;
    padding: 12px 6px; font-size: 14px; cursor: pointer; touch-action: manipulation;
    display: flex; flex-direction: column; align-items: center; gap: 4px;
    transition: transform .06s, background .12s;
  }
  button:active { transform: scale(.96); background: var(--accent); }
  button.busy { opacity: .4; pointer-events: none; }
  button svg { width: 22px; height: 22px; fill: currentColor; }
  .dpad {
    display: grid; gap: 8px;
    grid-template-columns: repeat(3, 1fr);
    grid-template-areas: ". fwd ." "left stop right" ". back .";
  }
  #b-fwd { grid-area: fwd; } #b-back { grid-area: back; }
  #b-left { grid-area: left; } #b-right { grid-area: right; } #b-stand2 { grid-area: stop; }
  #b-fwd, #b-back { color: var(--accent); }
  .poses { display: grid; grid-template-columns: repeat(3, 1fr); gap: 8px; margin-bottom: 12px; }
  .poses .stand { color: var(--good); } .poses .rest { color: var(--dim); }
  .tricks { display: grid; grid-template-columns: repeat(3, 1fr); gap: 8px; }
  .tricks button { color: var(--warn); }
  .legs { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; }
  .leg { background: #151a22; border: 1px solid var(--edge); border-radius: 10px; padding: 8px 10px; }
  .leg h3 { font-size: 11px; color: var(--dim); font-weight: 600; text-transform: uppercase; letter-spacing: .08em; margin-bottom: 6px; }
  .jrow { display: grid; grid-template-columns: 24px 1fr 34px; align-items: center; gap: 6px; margin: 3px 0; }
  .jrow .jl { font-size: 10px; color: var(--dim); }
  .jrow .jv { font-size: 11px; font-variant-numeric: tabular-nums; text-align: right; }
  .track { height: 8px; background: #0d1117; border-radius: 4px; position: relative; overflow: hidden; }
  .track .pin { position: absolute; top: 0; bottom: 0; width: 4px; border-radius: 2px; background: var(--accent); transition: left .12s linear; }
  .track.sh .pin { background: var(--warn); } .track.kn .pin { background: var(--good); }
  #b-stop {
    flex-direction: row; gap: 6px; padding: 8px 16px; font-weight: 700;
    color: #fff; background: var(--bad); border-color: var(--bad); border-radius: 999px;
  }
  #b-stop svg { width: 14px; height: 14px; }
  #b-stop:active { background: #d9414f; }
  #toast {
    position: fixed; bottom: 16px; left: 50%; transform: translateX(-50%);
    background: var(--panel); border: 1px solid var(--edge); border-radius: 10px;
    padding: 8px 16px; font-size: 14px; opacity: 0; transition: opacity .25s; pointer-events: none;
  }
</style>
</head>
<body>
<div class="wrap">
  <header>
    <svg width="26" height="26" viewBox="0 0 24 24" fill="var(--accent)"><path d="M4.5 9.5 2 12l2.5 2.5L6 13h2v5l-2 3h3l1.5-2.5L12 21h3l-2-3v-5h4l1 2h2l1-3.5L20 8h-6V6l2-2h-4l-1.5 2L9 4H6l2 3v2.5z"/></svg>
    <h1>SpotMicro</h1>
    <span id="state">…</span>
    <button id="b-stop" onclick="cmd('stop')">
      <svg viewBox="0 0 24 24"><path d="M12 2L2 22h20zM11 9h2v7h-2zm0 8h2v2h-2z" fill="none"/><rect x="6" y="6" width="12" height="12" rx="2"/></svg>STOP</button>
  </header>

  <div class="cams">
    <div class="panel"><iframe id="cam" allow="autoplay" title="robot camera"></iframe><div class="label">Robot camera</div></div>
    <div class="panel"><iframe id="webcam" allow="autoplay" title="external camera"></iframe><div class="label">Room camera</div></div>
  </div>

  <div class="panel section" id="servopanel">
    <h2>Servos <span id="servostate" style="float:right;color:var(--dim);text-transform:none;letter-spacing:0"></span></h2>
    <div class="legs">
      <div class="leg" id="leg-front_left"><h3>Front Left</h3></div>
      <div class="leg" id="leg-front_right"><h3>Front Right</h3></div>
      <div class="leg" id="leg-rear_left"><h3>Rear Left</h3></div>
      <div class="leg" id="leg-rear_right"><h3>Rear Right</h3></div>
    </div>
  </div>

  <div class="panel">
    <div class="stats">
      <div class="stat"><div class="label">Proximity</div><div class="value" id="distance">—</div></div>
      <div class="stat"><div class="label">Roll</div><div class="value" id="roll">—</div></div>
      <div class="stat"><div class="label">Pitch</div><div class="value" id="pitch">—</div></div>
      <div class="stat"><div class="label">Power</div><div class="value" id="power">—</div></div>
    </div>
  </div>

  <div class="controls">
    <div class="panel section">
      <h2>Move</h2>
      <div class="dpad">
        <button id="b-fwd" onclick="cmd('walk',{cycles:6})">
          <svg viewBox="0 0 24 24"><path d="M12 4l8 10H4z"/></svg>Forward</button>
        <button id="b-left" onclick="cmd('left',{cycles:6})">
          <svg viewBox="0 0 24 24"><path d="M4 12l10-8v16z"/></svg>Turn Left</button>
        <button id="b-stand2" onclick="cmd('stand')">
          <svg viewBox="0 0 24 24"><rect x="7" y="7" width="10" height="10" rx="2"/></svg>Stand</button>
        <button id="b-right" onclick="cmd('right',{cycles:6})">
          <svg viewBox="0 0 24 24"><path d="M20 12L10 4v16z"/></svg>Turn Right</button>
        <button id="b-back" onclick="cmd('walk',{cycles:6,dir:-1})">
          <svg viewBox="0 0 24 24"><path d="M12 20L4 10h16z"/></svg>Backward</button>
      </div>
    </div>
    <div class="panel section">
      <h2>Pose</h2>
      <div class="poses">
        <button class="stand" onclick="cmd('stand')">
          <svg viewBox="0 0 24 24"><path d="M5 18V8h14v10h-2v-6H7v6z"/></svg>Stand</button>
        <button onclick="cmd('sit')">
          <svg viewBox="0 0 24 24"><path d="M5 18v-4l6-6h8v4h-6l-4 6z"/></svg>Sit</button>
        <button class="rest" onclick="cmd('rest')">
          <svg viewBox="0 0 24 24"><rect x="4" y="13" width="16" height="5" rx="2"/></svg>Rest</button>
      </div>
      <h2>Tricks</h2>
      <div class="tricks">
        <button onclick="cmd('wave')">
          <svg viewBox="0 0 24 24"><path d="M7 20v-8L5 8l2-3 3 5v10zm7-16l3 2-2 4 3 2-4 8-3-2 2-4-3-2z"/></svg>Wave</button>
        <button onclick="cmd('dance')">
          <svg viewBox="0 0 24 24"><circle cx="12" cy="5" r="2"/><path d="M8 22l2-7-2-4 4-2 4 2-2 4 2 7h-2l-2-6-2 6z"/></svg>Dance</button>
        <button onclick="cmd('dance',{move:'bounce'})">
          <svg viewBox="0 0 24 24"><path d="M12 3l4 5H8zm0 18l-4-5h8zM8 11h8v2H8z"/></svg>Bounce</button>
        <button onclick="cmd('dance',{move:'sway'})">
          <svg viewBox="0 0 24 24"><path d="M3 12l5-4v3h8V8l5 4-5 4v-3H8v3z"/></svg>Sway</button>
        <button onclick="cmd('dance',{move:'rock'})">
          <svg viewBox="0 0 24 24"><path d="M4 16l4-8 4 5 4-9 4 12z"/></svg>Rock</button>
        <button onclick="cmd('hop')">
          <svg viewBox="0 0 24 24"><path d="M12 3l5 6h-3v6h-4V9H7zM5 19h14v2H5z"/></svg>Hop</button>
        <button onclick="cmd('dance',{move:'groove'})">
          <svg viewBox="0 0 24 24"><path d="M9 18V6l10-2v11a3 3 0 11-2-2.8V7.5L11 9v9a3 3 0 11-2-2.8z"/></svg>Groove</button>
      </div>
    </div>
  </div>
</div>
<div id="toast"></div>

<script>
  __HOST_LINE__
  document.getElementById('cam').src = `http://${ROBOT}:8889/cam/`;
  document.getElementById('webcam').src = `http://${ROBOT}:8889/webcam/`;

  const toast = (msg) => {
    const el = document.getElementById('toast');
    el.textContent = msg; el.style.opacity = 1;
    clearTimeout(el._t); el._t = setTimeout(() => el.style.opacity = 0, 1800);
  };

  async function cmd(name, params = {}) {
    const q = new URLSearchParams(params).toString();
    try {
      const r = await fetch(`http://${ROBOT}:5000/motion/${name}${q ? '?' + q : ''}`, { method: 'POST' });
      const d = await r.json();
      toast(d.error ? `error: ${d.error}` : `sent: ${d.cmd}${d.move && d.move !== 'full' ? ' ' + d.move : ''}`);
    } catch (e) { toast('robot unreachable'); }
  }

  const fmt = (v, u) => (v === null || v === undefined) ? '—' : `${(+v).toFixed(1)}${u}`;

  async function poll() {
    try {
      const s = await (await fetch(`http://${ROBOT}:5000/motion/status`)).json();
      const st = document.getElementById('state');
      st.textContent = s.state || '?'; st.className = s.state || '';
      document.getElementById('roll').textContent = fmt(s.roll, '°');
      document.getElementById('pitch').textContent = fmt(s.pitch, '°');
      const pw = document.getElementById('power');
      pw.textContent = (s.throttled === '0x0') ? 'OK' : s.throttled;
      pw.style.color = (s.throttled === '0x0') ? 'var(--good)' : 'var(--bad)';
      const busy = (s.state === 'WALK' || s.state === 'CRAWL' || s.state === 'DANCE');
      for (const id of ['b-fwd', 'b-back', 'b-left', 'b-right'])
        document.getElementById(id).classList.toggle('busy', busy);
    } catch (e) { document.getElementById('state').textContent = 'offline'; }
    try {
      const p = await (await fetch(`http://${ROBOT}:5000/proximity`)).json();
      const el = document.getElementById('distance');
      if (p.distance_cm == null) { el.textContent = '—'; el.className = 'value'; }
      else {
        el.textContent = `${p.distance_cm.toFixed(0)} cm`;
        el.className = 'value' + (p.distance_cm < 15 ? ' near' : p.distance_cm < 40 ? ' close' : '');
      }
    } catch (e) {}
  }
  poll();
  setInterval(poll, 1000);

  // live servo quadrant view
  const LEGS = {
    front_left:  [["Sh","front_shoulder_left","sh"], ["Th","front_leg_left","th"], ["Kn","front_feet_left","kn"]],
    front_right: [["Sh","front_shoulder_right","sh"], ["Th","front_leg_right","th"], ["Kn","front_feet_right","kn"]],
    rear_left:   [["Sh","rear_shoulder_left","sh"], ["Th","rear_leg_left","th"], ["Kn","rear_feet_left","kn"]],
    rear_right:  [["Sh","rear_shoulder_right","sh"], ["Th","rear_leg_right","th"], ["Kn","rear_feet_right","kn"]],
  };
  for (const [leg, joints] of Object.entries(LEGS)) {
    const el = document.getElementById('leg-' + leg);
    for (const [label, name, cls] of joints) {
      el.insertAdjacentHTML('beforeend',
        `<div class="jrow"><span class="jl">${label}</span>` +
        `<div class="track ${cls}"><div class="pin" id="pin-${name}"></div></div>` +
        `<span class="jv" id="val-${name}">—</span></div>`);
    }
  }
  async function pollServos() {
    try {
      const d = await (await fetch(`http://${ROBOT}:5000/servos`)).json();
      const s = d.servos || {};
      document.getElementById('servostate').textContent =
        Object.keys(s).length ? '' : 'released';
      for (const joints of Object.values(LEGS)) {
        for (const [, name] of joints) {
          const v = s[name];
          const pin = document.getElementById('pin-' + name);
          const val = document.getElementById('val-' + name);
          if (v === undefined) { val.textContent = '—'; pin.style.left = '48%'; pin.style.opacity = .25; }
          else { val.textContent = Math.round(v) + '°'; pin.style.opacity = 1;
                 pin.style.left = `calc(${(v / 180) * 100}% - 2px)`; }
        }
      }
    } catch (e) {}
  }
  setInterval(pollServos, 300);
</script>
</body>
</html>
'''

local = TEMPLATE.replace('__HOST_LINE__', "const ROBOT = '192.168.1.104';")
pi = TEMPLATE.replace('__HOST_LINE__', "const ROBOT = location.hostname;")
open('/home/thinknewdev/Development/SpotMicroAI/control-panel/index.html', 'w').write(local)
open('/home/thinknewdev/Development/SpotMicroAI/robot-snapshot/spotmicro_flask/static/control.html', 'w').write(pi)
print("both panels rebuilt from one template")
