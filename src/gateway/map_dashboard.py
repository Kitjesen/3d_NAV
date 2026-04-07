"""Map Dashboard V2 — professional robotics control UI.

Reference design: dark theme, left sidebar (map library + telemetry),
right panel (3D point cloud viewer), bottom status bar.
"""


def generate_dashboard_html() -> str:
    return '''<!DOCTYPE html>
<html lang="zh-CN"><head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>LingTu — Control Dashboard</title>
<style>
@import url('https://fonts.googleapis.com/css2?family=Inter:wght@400;500;600;700&family=JetBrains+Mono:wght@400;500&display=swap');
:root {
  --bg: #0a0e17; --bg2: #0f1520; --bg3: #151c2b; --bg4: #1a2235;
  --border: #1e2a3a; --border2: #2a3a4e;
  --text: #e2e8f0; --text2: #8892a4; --text3: #5a6478;
  --accent: #00ff88; --accent2: #00cc6a; --accent-dim: rgba(0,255,136,0.1);
  --blue: #3b82f6; --red: #ef4444; --orange: #f59e0b; --purple: #8b5cf6;
  --radius: 8px; --radius2: 12px;
}
* { margin:0; padding:0; box-sizing:border-box; }
body { font-family:'Inter','PingFang SC','Microsoft YaHei',sans-serif; background:var(--bg); color:var(--text); height:100vh; overflow:hidden; display:flex; flex-direction:column; }

/* Top Bar */
.topbar { height:48px; background:var(--bg2); border-bottom:1px solid var(--border); display:flex; align-items:center; padding:0 20px; justify-content:space-between; flex-shrink:0; }
.topbar-left { display:flex; align-items:center; gap:16px; }
.logo { display:flex; align-items:center; gap:8px; }
.logo-icon { width:28px; height:28px; border-radius:6px; background:var(--accent); display:flex; align-items:center; justify-content:center; font-weight:700; color:var(--bg); font-size:14px; }
.logo-text { font-weight:600; font-size:15px; }
.logo-text span { color:var(--accent); }
.status-badge { font-size:11px; padding:3px 10px; border-radius:20px; font-weight:500; letter-spacing:0.5px; }
.status-badge.on { background:rgba(0,255,136,0.15); color:var(--accent); }
.status-badge.off { background:rgba(239,68,68,0.15); color:var(--red); }
.topbar-tabs { display:flex; gap:0; }
.topbar-tabs button { background:none; border:none; color:var(--text2); padding:12px 20px; font-size:13px; cursor:pointer; border-bottom:2px solid transparent; font-family:inherit; }
.topbar-tabs button.active { color:var(--accent); border-bottom-color:var(--accent); }
.topbar-right { display:flex; align-items:center; gap:12px; }

/* Main Layout */
.main { flex:1; display:flex; overflow:hidden; }

/* Left Sidebar */
.sidebar { width:320px; background:var(--bg2); border-right:1px solid var(--border); display:flex; flex-direction:column; flex-shrink:0; }
.sidebar-section { padding:16px; border-bottom:1px solid var(--border); }
.sidebar-section h3 { font-size:11px; color:var(--text3); text-transform:uppercase; letter-spacing:1.5px; margin-bottom:12px; font-weight:600; }

/* Mode Tabs */
.mode-tabs { display:flex; gap:0; background:var(--bg3); border-radius:var(--radius); overflow:hidden; }
.mode-tabs button { flex:1; border:none; padding:10px 16px; font-size:13px; font-weight:500; cursor:pointer; font-family:inherit; transition:all 0.2s; background:transparent; color:var(--text2); }
.mode-tabs button.active { background:var(--accent); color:var(--bg); }
.mode-tabs button:hover:not(.active) { background:var(--bg4); }

/* Map List */
.map-list { flex:1; overflow-y:auto; padding:0; }
.map-item { padding:14px 16px; border-bottom:1px solid var(--border); cursor:pointer; transition:background 0.15s; }
.map-item:hover { background:var(--bg3); }
.map-item.active { border-left:3px solid var(--accent); background:var(--bg3); }
.map-item-header { display:flex; justify-content:space-between; align-items:center; margin-bottom:4px; }
.map-item-name { font-weight:600; font-size:13px; }
.map-item-badge { font-size:10px; padding:2px 8px; border-radius:10px; background:var(--accent); color:var(--bg); font-weight:600; }
.map-item-meta { font-size:11px; color:var(--text3); }
.map-item-actions { display:flex; gap:4px; margin-top:8px; }
.map-btn { border:none; padding:6px 14px; border-radius:6px; font-size:11px; font-weight:500; cursor:pointer; font-family:inherit; transition:all 0.15s; }
.map-btn.primary { background:var(--accent); color:var(--bg); }
.map-btn.primary:hover { background:var(--accent2); }
.map-btn.ghost { background:var(--bg4); color:var(--text2); border:1px solid var(--border); }
.map-btn.ghost:hover { border-color:var(--text2); }
.map-btn.danger { background:transparent; color:var(--red); border:1px solid rgba(239,68,68,0.3); }
.map-btn.danger:hover { background:rgba(239,68,68,0.1); }
.map-btn.icon { width:32px; height:32px; padding:0; display:flex; align-items:center; justify-content:center; border-radius:6px; background:var(--bg4); color:var(--text2); border:1px solid var(--border); }

/* Telemetry */
.telemetry { padding:16px; border-top:1px solid var(--border); }
.telemetry h3 { font-size:11px; color:var(--text3); text-transform:uppercase; letter-spacing:1.5px; margin-bottom:10px; font-weight:600; }
.tele-grid { display:grid; grid-template-columns:1fr 1fr; gap:8px; }
.tele-item { background:var(--bg3); border-radius:var(--radius); padding:10px; }
.tele-item .label { font-size:10px; color:var(--text3); margin-bottom:2px; }
.tele-item .value { font-size:18px; font-weight:600; color:var(--accent); font-family:'JetBrains Mono',monospace; }

/* Bottom Actions */
.sidebar-actions { padding:12px 16px; border-top:1px solid var(--border); display:flex; flex-direction:column; gap:8px; }
.action-btn { border:none; padding:10px; border-radius:var(--radius); font-size:13px; font-weight:500; cursor:pointer; font-family:inherit; display:flex; align-items:center; justify-content:center; gap:6px; transition:all 0.15s; }
.action-btn.save { background:var(--accent); color:var(--bg); }
.action-btn.save:hover { background:var(--accent2); }
.action-btn.viewer { background:var(--bg3); color:var(--text); border:1px solid var(--border); }
.action-btn.viewer:hover { border-color:var(--accent); }
.action-btn.estop { background:var(--red); color:white; font-weight:700; font-size:14px; padding:12px; border-radius:var(--radius2); }
.action-btn.estop:hover { background:#dc2626; }

/* Right Panel - 3D Viewer */
.viewer-panel { flex:1; display:flex; flex-direction:column; position:relative; }
.viewer-header { position:absolute; top:12px; left:16px; z-index:10; display:flex; gap:8px; align-items:center; }
.viewer-tag { padding:4px 12px; border-radius:20px; font-size:11px; font-weight:500; backdrop-filter:blur(8px); }
.viewer-tag.live { background:rgba(0,255,136,0.2); color:var(--accent); border:1px solid rgba(0,255,136,0.3); }
.viewer-tag.info { background:rgba(255,255,255,0.1); color:var(--text2); border:1px solid rgba(255,255,255,0.1); }
.viewer-frame { flex:1; border:none; width:100%; height:100%; }

/* Bottom Stats */
.viewer-stats { position:absolute; bottom:16px; left:16px; right:16px; display:flex; gap:12px; z-index:10; }
.stat-card { flex:1; background:rgba(15,21,32,0.85); backdrop-filter:blur(8px); border:1px solid var(--border); border-radius:var(--radius); padding:12px 16px; text-align:center; }
.stat-card .label { font-size:10px; color:var(--text3); margin-bottom:4px; }
.stat-card .value { font-size:20px; font-weight:700; font-family:'JetBrains Mono',monospace; }
.stat-card .value.green { color:var(--accent); }
.stat-card .value.blue { color:var(--blue); }
.stat-card .value.orange { color:var(--orange); }
.stat-card .unit { font-size:11px; color:var(--text3); margin-left:2px; }

/* Viewer Controls (right side) */
.viewer-controls { position:absolute; right:16px; top:50%; transform:translateY(-50%); display:flex; flex-direction:column; gap:8px; z-index:10; }
.vc-btn { width:40px; height:40px; border-radius:var(--radius); background:rgba(15,21,32,0.85); border:1px solid var(--border); color:var(--text2); cursor:pointer; display:flex; align-items:center; justify-content:center; font-size:18px; backdrop-filter:blur(8px); }
.vc-btn:hover { border-color:var(--accent); color:var(--accent); }

/* Bottom Bar */
.bottombar { height:32px; background:var(--bg2); border-top:1px solid var(--border); display:flex; align-items:center; padding:0 20px; font-size:11px; color:var(--text3); gap:24px; flex-shrink:0; font-family:'JetBrains Mono',monospace; }
.bottombar .dot { width:6px; height:6px; border-radius:50%; display:inline-block; margin-right:4px; }
.bottombar .dot.green { background:var(--accent); }

/* Toast */
.toast { position:fixed; bottom:48px; right:20px; padding:10px 18px; border-radius:var(--radius); font-size:12px; z-index:1000; transform:translateY(80px); opacity:0; transition:all 0.3s; backdrop-filter:blur(8px); }
.toast.show { transform:translateY(0); opacity:1; }
.toast.ok { background:rgba(0,204,106,0.9); color:#fff; }
.toast.err { background:rgba(239,68,68,0.9); color:#fff; }
.toast.info { background:rgba(15,21,32,0.9); color:var(--text); border:1px solid var(--border); }

.spinner { display:inline-block; width:12px; height:12px; border:2px solid var(--text3); border-top-color:var(--accent); border-radius:50%; animation:sp 0.6s linear infinite; }
@keyframes sp { to{transform:rotate(360deg);} }
</style>
</head>
<body>

<!-- Top Bar -->
<div class="topbar">
  <div class="topbar-left">
    <div class="logo">
      <div class="logo-icon">LT</div>
      <div class="logo-text"><span>LingTu</span> Dashboard</div>
    </div>
    <span class="status-badge on" id="sysBadge">SYSTEM ONLINE</span>
  </div>
  <div class="topbar-tabs">
    <button class="active">Telemetry</button>
    <button>Operations</button>
  </div>
  <div class="topbar-right">
    <span style="font-size:12px;color:var(--text3)" id="connInfo">--</span>
  </div>
</div>

<!-- Main -->
<div class="main">
  <!-- Sidebar -->
  <div class="sidebar">
    <div class="sidebar-section">
      <div class="mode-tabs">
        <button class="active" id="btnSlam" onclick="doSwitch('fastlio2')">SLAM 建图</button>
        <button id="btnNav" onclick="doSwitch('localizer')">导航巡航</button>
      </div>
    </div>

    <div class="sidebar-section" style="padding-bottom:8px">
      <div style="display:flex;justify-content:space-between;align-items:center">
        <h3>本地地图库</h3>
        <span style="font-size:10px;color:var(--accent);border:1px solid var(--accent);padding:1px 8px;border-radius:10px" id="mapCount">0</span>
      </div>
    </div>

    <div class="map-list" id="mapList">
      <div style="padding:20px;text-align:center;color:var(--text3)"><span class="spinner"></span></div>
    </div>

    <div class="telemetry">
      <h3>实时遥测数据</h3>
      <div class="tele-grid">
        <div class="tele-item"><div class="label">X 坐标</div><div class="value" id="tX">--</div></div>
        <div class="tele-item"><div class="label">Y 坐标</div><div class="value" id="tY">--</div></div>
        <div class="tele-item"><div class="label">频率</div><div class="value" id="tHz">--</div></div>
        <div class="tele-item"><div class="label">SLAM 帧</div><div class="value" id="tFrames">--</div></div>
      </div>
    </div>

    <div class="sidebar-actions">
      <button class="action-btn save" onclick="saveMap()" id="btnSave"><svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M19 21H5a2 2 0 01-2-2V5a2 2 0 012-2h11l5 5v11a2 2 0 01-2 2z"/><polyline points="17,21 17,13 7,13 7,21"/><polyline points="7,3 7,8 15,8"/></svg> 保存地图</button>
      <button class="action-btn viewer" onclick="showLive()"><svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M23 12l-2.44-2.78.34-3.68-3.61-.82-1.89-3.18L12 3 8.6 1.54 6.71 4.72l-3.61.81.34 3.68L1 12l2.44 2.78-.34 3.69 3.61.82 1.89 3.18L12 21l3.4 1.46 1.89-3.18 3.61-.82-.34-3.68L23 12z"/></svg> 刷新 3D 预览</button>
      <button class="action-btn estop" onclick="doSwitch('stop')"><svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.5"><circle cx="12" cy="12" r="10"/><line x1="15" y1="9" x2="9" y2="15"/><line x1="9" y1="9" x2="15" y2="15"/></svg> EMERGENCY STOP</button>
    </div>
  </div>

  <!-- 3D Viewer -->
  <div class="viewer-panel">
    <div class="viewer-header">
      <span class="viewer-tag live" id="vTag">3D 实时渲染</span>
      <span class="viewer-tag info" id="vInfo">LIDAR: ENABLED</span>
    </div>
    <iframe src="/map/viewer" class="viewer-frame" id="vFrame"></iframe>
    <div class="viewer-controls">
      <button class="vc-btn" onclick="showLive()" title="刷新">&#8635;</button>
    </div>
    <div class="viewer-stats">
      <div class="stat-card"><div class="label">SLAM 频率</div><div class="value green" id="sHz">--</div></div>
      <div class="stat-card"><div class="label">SLAM 模式</div><div class="value blue" id="sMode">--</div></div>
      <div class="stat-card"><div class="label">总运行时间</div><div class="value orange" id="sTime">--</div></div>
      <div class="stat-card"><div class="label">退化事件</div><div class="value green" id="sDegen">0</div></div>
    </div>
  </div>
</div>

<!-- Bottom Bar -->
<div class="bottombar">
  <span><span class="dot green" id="bDot"></span>SLAM ACTIVE</span>
  <span id="bLatency">延迟 --</span>
  <span style="flex:1"></span>
  <span id="bUUID">UUID: --</span>
  <span>VER: 2.1.0</span>
</div>

<div class="toast" id="toast"></div>

<script>
const startTime = Date.now();
function T(m,t='info'){const e=document.getElementById('toast');e.textContent=m;e.className='toast '+t+' show';setTimeout(()=>e.classList.remove('show'),3000);}
async function F(u,o={}){try{const c=new AbortController();const t=setTimeout(()=>c.abort(),5000);o.signal=c.signal;const r=await fetch(u,o);clearTimeout(t);return await r.json();}catch(e){return null;}}

// Uptime
function uptime(){const s=Math.floor((Date.now()-startTime)/1000);const h=Math.floor(s/3600);const m=Math.floor((s%3600)/60);const ss=s%60;return String(h).padStart(2,'0')+':'+String(m).padStart(2,'0')+':'+String(ss).padStart(2,'0');}

// Poll
async function poll(){
  const t0=Date.now();
  const h=await F('/api/v1/health');
  const lat=Date.now()-t0;
  document.getElementById('bLatency').textContent='延迟 '+lat+'ms';
  if(!h){document.getElementById('sysBadge').className='status-badge off';document.getElementById('sysBadge').textContent='OFFLINE';return;}
  document.getElementById('sysBadge').className='status-badge on';
  document.getElementById('sysBadge').textContent='SYSTEM ONLINE';
  document.getElementById('tHz').textContent=(h.slam_hz||0).toFixed(1);
  document.getElementById('tFrames').textContent=h.map_points||0;
  document.getElementById('sHz').textContent=(h.slam_hz||0).toFixed(1)+'Hz';
  document.getElementById('sTime').textContent=uptime();

  const st=await F('/api/v1/state');
  if(st&&st.odometry){
    document.getElementById('tX').textContent=(st.odometry.x||0).toFixed(2)+'m';
    document.getElementById('tY').textContent=(st.odometry.y||0).toFixed(2)+'m';
  }

  const s=await F('/api/v1/slam/status');
  if(s){
    const m=s.mode;
    document.getElementById('sMode').textContent=m==='fastlio2'?'建图':m==='localizer'?'导航':'停止';
    document.getElementById('btnSlam').className=m==='fastlio2'?'active':'';
    document.getElementById('btnNav').className=m==='localizer'?'active':'';
    document.getElementById('bDot').className='dot '+(m!=='stopped'?'green':'');
  }
}

async function doSwitch(p){
  const names={fastlio2:'建图模式',localizer:'导航模式',stop:'停止'};
  T('切换到'+names[p]+'...','info');
  const r=await F('/api/v1/slam/switch',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({profile:p})});
  if(r&&r.success){T('已切换: '+names[p],'ok');setTimeout(poll,2000);}
  else T('切换失败: '+(r?.message||'未知'),'err');
}

// Maps
let activeName='';
async function loadMaps(){
  const r=await F('/api/v1/slam/maps');
  if(!r||!r.maps)return;
  const maps=r.maps.filter(m=>m.has_pcd);
  activeName=r.active||'';
  document.getElementById('mapCount').textContent='TOTAL: '+maps.length;
  const el=document.getElementById('mapList');
  if(!maps.length){el.innerHTML='<div style="padding:20px;text-align:center;color:var(--text3)">暂无地图</div>';return;}
  el.innerHTML=maps.map(m=>{
    const isAct=m.name===activeName||m.active;
    return '<div class="map-item'+(isAct?' active':'')+'">' +
      '<div class="map-item-header"><span class="map-item-name">'+m.name.toUpperCase()+'</span>'+(isAct?'<span class="map-item-badge">正在使用</span>':'')+'</div>' +
      '<div class="map-item-meta">'+(m.size||'')+' '+(m.patches?m.patches+' 关键帧':'')+'</div>' +
      '<div class="map-item-actions">' +
        (isAct?'<button class="map-btn primary" onclick="viewSaved(&quot;'+m.name+'&quot;)">3D 预览</button>':'<button class="map-btn ghost" onclick="setAct(&quot;'+m.name+'&quot;)">激活地图</button>') +
        '<button class="map-btn icon" onclick="renameMap(&quot;'+m.name+'&quot;)" title="重命名"><svg width="12" height="12" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M11 4H4a2 2 0 00-2 2v14a2 2 0 002 2h14a2 2 0 002-2v-7"/><path d="M18.5 2.5a2.121 2.121 0 013 3L12 15l-4 1 1-4 9.5-9.5z"/></svg></button>' +
        '<button class="map-btn icon" onclick="delMap(&quot;'+m.name+'&quot;)" title="删除" style="color:var(--red)"><svg width="12" height="12" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><polyline points="3,6 5,6 21,6"/><path d="M19 6v14a2 2 0 01-2 2H7a2 2 0 01-2-2V6m3 0V4a2 2 0 012-2h4a2 2 0 012 2v2"/></svg></button>' +
      '</div></div>';
  }).join('');
}

function showLive(){
  document.getElementById('vFrame').src='/map/viewer?t='+Date.now();
  document.getElementById('vTag').textContent='3D 实时渲染';
}
function viewSaved(name){
  document.getElementById('vFrame').src='/map/viewer?map='+encodeURIComponent(name);
  document.getElementById('vTag').textContent='已保存: '+name;
}

async function setAct(n){
  T('激活地图: '+n,'info');
  const r=await F('/api/v1/map/activate',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({name:n})});
  if(r&&r.success){T(n+' 已激活','ok');loadMaps();}
  else T('激活失败: '+(r?.message||'未知'),'err');
}
async function renameMap(n){
  const nn=prompt('新名称:',n);
  if(!nn||nn===n)return;
  const r=await F('/api/v1/map/rename',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({old_name:n,new_name:nn})});
  if(r&&r.success){T('已重命名','ok');loadMaps();}
  else T('重命名失败','err');
}
async function delMap(n){
  if(!confirm('确定删除 "'+n+'" ?'))return;
  const r=await F('/api/v1/maps',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({action:'delete',name:n})});
  if(r&&r.success!==false){T('已删除','ok');loadMaps();}
  else T('删除失败','err');
}
async function saveMap(){
  const n=prompt('地图名称:','map_'+new Date().toISOString().slice(0,16).replace(/[T:]/g,'_'));
  if(!n)return;
  T('保存中 (约10-30秒)...','info');
  document.getElementById('btnSave').disabled=true;
  const r=await F('/api/v1/map/save',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({name:n})});
  document.getElementById('btnSave').disabled=false;
  if(r&&r.success){T('已保存: '+n+' ('+r.size+')','ok');loadMaps();}
  else T('保存失败: '+(r?.errors?.join(', ')||'SLAM 未运行'),'err');
}

// Init
poll(); loadMaps();
setInterval(poll, 3000);
setInterval(()=>{document.getElementById('sTime').textContent=uptime();},1000);
</script>
</body></html>'''
