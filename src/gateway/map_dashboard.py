"""Map Dashboard — web UI for SLAM control and map management.

Served at GET /dashboard by GatewayModule.
Calls existing REST APIs: /api/v1/maps, /api/v1/health, /api/v1/state.
Zero external dependencies — pure HTML/CSS/JS.
"""


def generate_dashboard_html() -> str:
    return '''<!DOCTYPE html>
<html lang="en"><head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>LingTu — Map Dashboard</title>
<style>
:root {
  --bg: #0d1117; --bg2: #161b22; --bg3: #21262d; --border: #30363d;
  --text: #e6edf3; --text2: #8b949e; --accent: #00ff88; --accent2: #238636;
  --warn: #d29922; --danger: #f85149; --blue: #58a6ff; --purple: #bc8cff;
  --radius: 10px; --shadow: 0 2px 12px rgba(0,0,0,0.4);
}
* { margin:0; padding:0; box-sizing:border-box; }
body { font-family: -apple-system, 'Segoe UI', Roboto, monospace; background:var(--bg); color:var(--text); min-height:100vh; }

/* Header */
.header { background:var(--bg2); border-bottom:1px solid var(--border); padding:16px 24px; display:flex; align-items:center; justify-content:space-between; }
.header h1 { font-size:20px; font-weight:600; }
.header h1 span { color:var(--accent); }
.header .status-dot { display:inline-block; width:8px; height:8px; border-radius:50%; margin-right:8px; }
.header .status-dot.on { background:var(--accent); box-shadow:0 0 8px var(--accent); }
.header .status-dot.off { background:var(--danger); }

/* Layout */
.container { display:grid; grid-template-columns:320px 1fr; gap:0; min-height:calc(100vh - 60px); }
.sidebar { background:var(--bg2); border-right:1px solid var(--border); padding:20px; overflow-y:auto; }
.main { padding:20px; overflow-y:auto; }

/* Cards */
.card { background:var(--bg3); border:1px solid var(--border); border-radius:var(--radius); padding:16px; margin-bottom:16px; }
.card h3 { font-size:14px; color:var(--text2); text-transform:uppercase; letter-spacing:1px; margin-bottom:12px; }

/* SLAM Control */
.slam-mode { font-size:18px; font-weight:600; margin-bottom:12px; }
.slam-mode .mode-tag { display:inline-block; padding:4px 10px; border-radius:6px; font-size:13px; font-weight:500; }
.slam-mode .mode-tag.mapping { background:rgba(0,255,136,0.15); color:var(--accent); }
.slam-mode .mode-tag.nav { background:rgba(88,166,255,0.15); color:var(--blue); }
.slam-mode .mode-tag.stopped { background:rgba(248,81,73,0.15); color:var(--danger); }

.service-row { display:flex; align-items:center; padding:6px 0; font-size:13px; }
.service-row .dot { width:6px; height:6px; border-radius:50%; margin-right:8px; flex-shrink:0; }
.service-row .dot.active { background:var(--accent); }
.service-row .dot.inactive { background:var(--text2); }
.service-row .name { flex:1; color:var(--text2); }
.service-row .state { font-size:12px; }

.switch-btns { display:flex; gap:8px; margin-top:14px; }
.btn { border:none; padding:8px 16px; border-radius:6px; font-size:13px; font-weight:500; cursor:pointer; transition:all 0.2s; }
.btn:hover { transform:translateY(-1px); }
.btn:active { transform:translateY(0); }
.btn-primary { background:var(--accent2); color:#fff; }
.btn-primary:hover { background:#2ea043; }
.btn-blue { background:rgba(88,166,255,0.15); color:var(--blue); border:1px solid rgba(88,166,255,0.3); }
.btn-blue:hover { background:rgba(88,166,255,0.25); }
.btn-danger { background:rgba(248,81,73,0.1); color:var(--danger); border:1px solid rgba(248,81,73,0.3); }
.btn-danger:hover { background:rgba(248,81,73,0.2); }
.btn-sm { padding:5px 10px; font-size:12px; }
.btn:disabled { opacity:0.4; cursor:not-allowed; transform:none; }

/* Robot State */
.robot-stats { display:grid; grid-template-columns:1fr 1fr; gap:8px; }
.stat { text-align:center; padding:8px; background:var(--bg); border-radius:6px; }
.stat .val { font-size:18px; font-weight:600; color:var(--accent); }
.stat .label { font-size:11px; color:var(--text2); margin-top:2px; }

/* Map List */
.map-grid { display:grid; grid-template-columns:repeat(auto-fill, minmax(280px, 1fr)); gap:16px; }
.map-card { background:var(--bg2); border:1px solid var(--border); border-radius:var(--radius); padding:16px; transition:all 0.2s; position:relative; }
.map-card:hover { border-color:var(--accent); box-shadow:var(--shadow); }
.map-card.active { border-color:var(--accent); }
.map-card.active::before { content:'ACTIVE'; position:absolute; top:10px; right:10px; background:var(--accent); color:var(--bg); padding:2px 8px; border-radius:4px; font-size:10px; font-weight:700; }
.map-card .name { font-size:15px; font-weight:600; margin-bottom:6px; word-break:break-all; }
.map-card .meta { font-size:12px; color:var(--text2); margin-bottom:10px; }
.map-card .meta span { margin-right:12px; }
.map-card .actions { display:flex; gap:6px; flex-wrap:wrap; }

/* 3D Viewer */
.viewer-frame { width:100%; height:400px; border:1px solid var(--border); border-radius:var(--radius); background:var(--bg); }

/* Toast */
.toast { position:fixed; bottom:20px; right:20px; padding:12px 20px; border-radius:8px; font-size:13px; z-index:1000; transform:translateY(100px); opacity:0; transition:all 0.3s; }
.toast.show { transform:translateY(0); opacity:1; }
.toast.success { background:var(--accent2); color:#fff; }
.toast.error { background:var(--danger); color:#fff; }
.toast.info { background:var(--bg3); color:var(--text); border:1px solid var(--border); }

/* Loading */
.spinner { display:inline-block; width:14px; height:14px; border:2px solid var(--text2); border-top-color:var(--accent); border-radius:50%; animation:spin 0.6s linear infinite; }
@keyframes spin { to { transform:rotate(360deg); } }

/* Responsive */
@media (max-width: 768px) {
  .container { grid-template-columns:1fr; }
  .sidebar { border-right:none; border-bottom:1px solid var(--border); }
  .map-grid { grid-template-columns:1fr; }
}
</style>
</head>
<body>

<div class="header">
  <h1><span>灵途</span> LingTu Dashboard</h1>
  <div><span class="status-dot" id="hdrDot"></span><span id="hdrStatus">connecting...</span></div>
</div>

<div class="container">
  <div class="sidebar">
    <!-- SLAM Control -->
    <div class="card">
      <h3>SLAM Control</h3>
      <div class="slam-mode" id="slamMode">loading...</div>
      <div id="slamServices"></div>
      <div class="switch-btns">
        <button class="btn btn-primary" onclick="slamSwitch('fastlio2')" id="btnMapping">Mapping</button>
        <button class="btn btn-blue" onclick="slamSwitch('localizer')" id="btnNav">Navigation</button>
        <button class="btn btn-danger btn-sm" onclick="slamSwitch('stop')" id="btnStop">Stop</button>
      </div>
    </div>

    <!-- Robot State -->
    <div class="card">
      <h3>Robot State</h3>
      <div class="robot-stats">
        <div class="stat"><div class="val" id="posX">-</div><div class="label">X (m)</div></div>
        <div class="stat"><div class="val" id="posY">-</div><div class="label">Y (m)</div></div>
        <div class="stat"><div class="val" id="slamHz">-</div><div class="label">SLAM Hz</div></div>
        <div class="stat"><div class="val" id="mapPts">-</div><div class="label">Map Points</div></div>
      </div>
    </div>

    <!-- Quick Actions -->
    <div class="card">
      <h3>Quick Actions</h3>
      <div style="display:flex;flex-direction:column;gap:8px">
        <button class="btn btn-primary" onclick="saveMap()" id="btnSave">Save Current Map</button>
        <button class="btn btn-blue" onclick="openViewer()" id="btnViewer">Open 3D Viewer</button>
      </div>
    </div>
  </div>

  <div class="main">
    <div style="display:flex;justify-content:space-between;align-items:center;margin-bottom:16px">
      <h2>Maps</h2>
      <button class="btn btn-sm btn-blue" onclick="refreshMaps()">Refresh</button>
    </div>
    <div class="map-grid" id="mapGrid">
      <div class="card" style="text-align:center;color:var(--text2)"><span class="spinner"></span> Loading maps...</div>
    </div>

    <!-- 3D Preview -->
    <div style="margin-top:24px">
      <h2 style="margin-bottom:12px">3D Preview</h2>
      <iframe src="/map/viewer" class="viewer-frame" id="viewerFrame"></iframe>
    </div>
  </div>
</div>

<div class="toast" id="toast"></div>

<script>
const API = '';

function toast(msg, type='info') {
  const t = document.getElementById('toast');
  t.textContent = msg;
  t.className = 'toast ' + type + ' show';
  setTimeout(() => t.classList.remove('show'), 3000);
}

async function api(path, opts={}) {
  try {
    const r = await fetch(API + path, opts);
    return await r.json();
  } catch(e) {
    console.error(path, e);
    return null;
  }
}

// SLAM Status
async function refreshSlam() {
  const h = await api('/api/v1/health');
  if (!h) { document.getElementById('hdrDot').className='status-dot off'; return; }
  document.getElementById('hdrDot').className='status-dot on';
  document.getElementById('hdrStatus').textContent='online';

  // Try to get SLAM service status via systemd (through a dedicated endpoint or health)
  const gw = h.gateway || {};
  const pts = gw.map_points || 0;
  document.getElementById('mapPts').textContent = pts > 1000 ? (pts/1000).toFixed(0)+'K' : pts;

  // Get robot position
  const st = await api('/api/v1/state');
  if (st && st.odometry) {
    document.getElementById('posX').textContent = (st.odometry.x||0).toFixed(2);
    document.getElementById('posY').textContent = (st.odometry.y||0).toFixed(2);
  }
}

function updateSlamUI(services) {
  // services: {lidar:'running', slam:'running', slam_pgo:'running', localizer:'stopped'}
  const el = document.getElementById('slamServices');
  let html = '';
  for (const [name, state] of Object.entries(services)) {
    const active = state === 'running' || state === 'active';
    html += '<div class="service-row">' +
      '<div class="dot '+(active?'active':'inactive')+'"></div>' +
      '<span class="name">'+name+'</span>' +
      '<span class="state" style="color:'+(active?'var(--accent)':'var(--text2)')+'">'+state+'</span></div>';
  }
  el.innerHTML = html;

  const modeEl = document.getElementById('slamMode');
  if (services.slam_pgo === 'running' || services.slam_pgo === 'active') {
    modeEl.innerHTML = '<span class="mode-tag mapping">MAPPING</span>';
    document.getElementById('btnMapping').disabled = true;
    document.getElementById('btnNav').disabled = false;
  } else if (services.localizer === 'running' || services.localizer === 'active') {
    modeEl.innerHTML = '<span class="mode-tag nav">NAVIGATION</span>';
    document.getElementById('btnMapping').disabled = false;
    document.getElementById('btnNav').disabled = true;
  } else {
    modeEl.innerHTML = '<span class="mode-tag stopped">STOPPED</span>';
    document.getElementById('btnMapping').disabled = false;
    document.getElementById('btnNav').disabled = false;
  }
}

async function slamSwitch(mode) {
  toast('Switching to ' + mode + '...', 'info');
  const r = await api('/api/v1/slam/switch', {
    method:'POST',
    headers:{'Content-Type':'application/json'},
    body: JSON.stringify({profile: mode})
  });
  if (r && r.success) {
    toast('Switched to ' + mode, 'success');
    setTimeout(refreshAll, 2000);
  } else {
    toast('Switch failed: ' + (r?.message||'unknown'), 'error');
  }
}

// Maps
let currentMaps = [];
let activeMap = '';

async function refreshMaps() {
  const r = await api('/api/v1/maps', {
    method:'POST',
    headers:{'Content-Type':'application/json'},
    body: JSON.stringify({action:'list'})
  });
  if (!r) return;

  const maps = r.maps || r.data?.maps || [];
  activeMap = r.active || r.data?.active || '';
  currentMaps = maps;
  renderMaps(maps);
}

function renderMaps(maps) {
  const grid = document.getElementById('mapGrid');
  if (!maps || maps.length === 0) {
    grid.innerHTML = '<div class="card" style="text-align:center;color:var(--text2)">No maps found. Start mapping to create one.</div>';
    return;
  }
  grid.innerHTML = maps.map(m => {
    const name = m.name || m;
    const isActive = name === activeMap || m.active;
    const size = m.size || '';
    const patches = m.patches || 0;
    return '<div class="map-card'+(isActive?' active':'')+'">' +
      '<div class="name">'+name+'</div>' +
      '<div class="meta"><span>'+size+'</span>'+(patches?'<span>'+patches+' KF</span>':'')+'</div>' +
      '<div class="actions">' +
        (!isActive ? '<button class="btn btn-primary btn-sm" onclick="setActive(\''+name+'\')">Set Active</button>' : '') +
        '<button class="btn btn-blue btn-sm" onclick="viewMap(\''+name+'\')">View 3D</button>' +
        '<button class="btn btn-danger btn-sm" onclick="deleteMap(\''+name+'\')">Delete</button>' +
      '</div></div>';
  }).join('');
}

async function setActive(name) {
  toast('Setting active: ' + name, 'info');
  const r = await api('/api/v1/maps', {
    method:'POST', headers:{'Content-Type':'application/json'},
    body: JSON.stringify({action:'set_active', name:name})
  });
  if (r && (r.success !== false)) {
    toast(name + ' is now active', 'success');
    refreshMaps();
  } else {
    toast('Failed: '+(r?.error||'unknown'), 'error');
  }
}

async function deleteMap(name) {
  if (!confirm('Delete map "'+name+'"?')) return;
  const r = await api('/api/v1/maps', {
    method:'POST', headers:{'Content-Type':'application/json'},
    body: JSON.stringify({action:'delete', name:name})
  });
  if (r && (r.success !== false)) {
    toast('Deleted: ' + name, 'success');
    refreshMaps();
  } else {
    toast('Failed: '+(r?.error||'unknown'), 'error');
  }
}

function viewMap(name) {
  document.getElementById('viewerFrame').src = '/map/viewer?map=' + encodeURIComponent(name);
}

async function saveMap() {
  const name = prompt('Map name:', 'map_' + new Date().toISOString().slice(0,16).replace(/[T:]/g,'_'));
  if (!name) return;
  toast('Saving map "'+name+'"... (may take 10-30s)', 'info');
  document.getElementById('btnSave').disabled = true;
  const r = await api('/api/v1/maps', {
    method:'POST', headers:{'Content-Type':'application/json'},
    body: JSON.stringify({action:'save', name:name})
  });
  document.getElementById('btnSave').disabled = false;
  if (r && (r.success !== false)) {
    toast('Map saved: ' + name, 'success');
    refreshMaps();
  } else {
    toast('Save failed: '+(r?.error||r?.message||'unknown'), 'error');
  }
}

function openViewer() {
  window.open('/map/viewer', '_blank');
}

// Poll
async function refreshAll() {
  refreshSlam();
  // SLAM service status via health or dedicated endpoint
  try {
    const r = await api('/api/v1/slam/status');
    if (r && r.services) updateSlamUI(r.services);
  } catch(e) {
    // Fallback: just show unknown
    updateSlamUI({lidar:'?', slam:'?', slam_pgo:'?', localizer:'?'});
  }
}

// Init
refreshAll();
refreshMaps();
setInterval(refreshSlam, 5000);
setInterval(() => api('/api/v1/slam/status').then(r => { if(r&&r.services) updateSlamUI(r.services); }), 3000);
</script>
</body></html>'''
