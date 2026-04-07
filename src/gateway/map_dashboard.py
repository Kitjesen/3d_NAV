"""Map Dashboard V3 — collapsible panels, camera PIP, robot status.

Professional robotics control UI with:
- Collapsible sidebar sections (map library, robot status, telemetry)
- Camera picture-in-picture (top-right, from /ws/teleop JPEG stream)
- Clean dark theme, SVG icons only, Inter + JetBrains Mono
"""


def generate_dashboard_html() -> str:
    return '''<!DOCTYPE html>
<html lang="zh-CN"><head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>LingTu — Control Dashboard</title>
<style>
@import url('https://fonts.googleapis.com/css2?family=Inter:wght@400;500;600&family=JetBrains+Mono:wght@400;500&display=swap');
:root {
  --bg: #0a0e17; --bg2: #0f1520; --bg3: #151c2b; --bg4: #1a2235;
  --border: #1e2a3a;
  --text: #e2e8f0; --text2: #8892a4; --text3: #5a6478;
  --accent: #00ff88; --accent2: #00cc6a;
  --blue: #3b82f6; --red: #ef4444; --orange: #f59e0b;
  --radius: 6px;
}
* { margin:0; padding:0; box-sizing:border-box; }
body { font-family:'Inter','PingFang SC','Microsoft YaHei',sans-serif; background:var(--bg); color:var(--text); height:100vh; overflow:hidden; display:flex; flex-direction:column; font-size:12px; -webkit-font-smoothing:antialiased; }

/* Top Bar */
.topbar { height:36px; background:var(--bg2); border-bottom:1px solid var(--border); display:flex; align-items:center; padding:0 14px; justify-content:space-between; flex-shrink:0; }
.topbar-left { display:flex; align-items:center; gap:12px; }
.logo { display:flex; align-items:center; gap:6px; font-weight:500; font-size:12px; }
.logo svg { color:var(--accent); }
.logo span { color:var(--accent); }
.badge { font-size:10px; padding:2px 8px; border-radius:10px; font-weight:500; }
.badge.on { background:rgba(0,255,136,0.12); color:var(--accent); }
.badge.off { background:rgba(239,68,68,0.12); color:var(--red); }
.topbar-right { font-size:10px; color:var(--text3); font-family:'JetBrains Mono',monospace; }

/* Main */
.main { flex:1; display:flex; overflow:hidden; }

/* Sidebar */
.sidebar { width:260px; background:var(--bg2); border-right:1px solid var(--border); display:flex; flex-direction:column; flex-shrink:0; }

/* Mode Tabs */
.mode-section { padding:8px 10px; border-bottom:1px solid var(--border); }
.mode-tabs { display:flex; background:var(--bg3); border-radius:var(--radius); overflow:hidden; }
.mode-tabs button { flex:1; border:none; padding:6px; font-size:11px; font-weight:500; cursor:pointer; font-family:inherit; transition:all 0.15s; background:transparent; color:var(--text3); }
.mode-tabs button.active { background:var(--accent); color:var(--bg); }

/* Collapsible Panel */
.panel { border-bottom:1px solid var(--border); }
.panel-header { display:flex; align-items:center; justify-content:space-between; padding:8px 10px; cursor:pointer; user-select:none; }
.panel-header:hover { background:var(--bg3); }
.panel-header h3 { font-size:10px; color:var(--text3); text-transform:uppercase; letter-spacing:1px; font-weight:500; }
.panel-header .count { font-size:9px; color:var(--accent); border:1px solid rgba(0,255,136,0.3); padding:1px 6px; border-radius:8px; }
.panel-header .arrow { color:var(--text3); font-size:10px; transition:transform 0.2s; }
.panel-header .arrow.collapsed { transform:rotate(-90deg); }
.panel-body { overflow:hidden; transition:max-height 0.25s ease; }
.panel-body.collapsed { max-height:0 !important; }

/* Map List */
.map-list { overflow-y:auto; max-height:200px; scrollbar-width:thin; scrollbar-color:var(--border) transparent; }
.map-list::-webkit-scrollbar { width:3px; }
.map-list::-webkit-scrollbar-track { background:transparent; }
.map-list::-webkit-scrollbar-thumb { background:var(--border); border-radius:2px; }
.map-item { padding:6px 10px; border-bottom:1px solid rgba(30,42,58,0.5); transition:background 0.1s; }
.map-item:hover { background:var(--bg3); }
.map-item.active { border-left:2px solid var(--accent); background:rgba(0,255,136,0.04); }
.map-item-top { display:flex; justify-content:space-between; align-items:center; }
.map-item-name { font-weight:500; font-size:11px; }
.map-item-badge { font-size:9px; padding:1px 6px; border-radius:8px; background:var(--accent); color:var(--bg); font-weight:600; }
.map-item-meta { font-size:10px; color:var(--text3); margin-top:1px; }
.map-item-actions { display:flex; gap:3px; margin-top:4px; }
.mbtn { border:none; padding:3px 8px; border-radius:3px; font-size:9px; font-weight:500; cursor:pointer; font-family:inherit; transition:all 0.1s; }
.mbtn.primary { background:var(--accent); color:var(--bg); }
.mbtn.ghost { background:var(--bg4); color:var(--text3); border:1px solid var(--border); }
.mbtn.ghost:hover { color:var(--text2); border-color:var(--text3); }
.mbtn.icon { width:22px; height:22px; padding:0; display:flex; align-items:center; justify-content:center; background:var(--bg4); color:var(--text3); border:1px solid var(--border); border-radius:3px; }
.mbtn.icon:hover { color:var(--text); }

/* Robot Status */
.status-grid { display:grid; grid-template-columns:1fr 1fr; gap:4px; padding:6px 10px; }
.status-item { display:flex; align-items:center; gap:6px; padding:4px 6px; background:var(--bg3); border-radius:4px; }
.status-item svg { color:var(--text3); flex-shrink:0; }
.status-item .sl { font-size:9px; color:var(--text3); }
.status-item .sv { font-size:11px; font-weight:500; color:var(--accent); font-family:'JetBrains Mono',monospace; }

/* Telemetry */
.tele-grid { display:grid; grid-template-columns:1fr 1fr; gap:4px; padding:6px 10px; }
.tele-item { background:var(--bg3); border-radius:4px; padding:5px 6px; }
.tele-item .tl { font-size:9px; color:var(--text3); }
.tele-item .tv { font-size:14px; font-weight:500; color:var(--accent); font-family:'JetBrains Mono',monospace; }

/* Actions */
.actions { padding:8px 10px; border-top:1px solid var(--border); display:flex; flex-direction:column; gap:5px; margin-top:auto; }
.abtn { border:none; padding:6px; border-radius:var(--radius); font-size:11px; font-weight:500; cursor:pointer; font-family:inherit; display:flex; align-items:center; justify-content:center; gap:4px; transition:all 0.1s; }
.abtn.save { background:var(--accent); color:var(--bg); }
.abtn.save:hover { background:var(--accent2); }
.abtn.refresh { background:var(--bg3); color:var(--text2); border:1px solid var(--border); }
.abtn.refresh:hover { border-color:var(--accent); }
.abtn.estop { background:rgba(239,68,68,0.12); color:var(--red); border:1px solid rgba(239,68,68,0.25); font-weight:600; }

/* Viewer */
.viewer-panel { flex:1; position:relative; background:var(--bg); }
.viewer-frame { width:100%; height:100%; border:none; }

/* Camera PIP */
.camera-pip { position:absolute; top:12px; right:12px; width:240px; height:160px; border-radius:var(--radius); overflow:hidden; border:1px solid var(--border); background:#000; z-index:10; box-shadow:0 4px 20px rgba(0,0,0,0.5); }
.camera-pip img { width:100%; height:100%; object-fit:cover; }
.camera-pip .pip-label { position:absolute; bottom:4px; left:6px; font-size:9px; color:var(--accent); background:rgba(0,0,0,0.6); padding:1px 6px; border-radius:3px; }
.camera-pip .pip-close { position:absolute; top:4px; right:6px; background:rgba(0,0,0,0.6); border:none; color:var(--text2); cursor:pointer; width:18px; height:18px; border-radius:3px; font-size:10px; display:flex; align-items:center; justify-content:center; }

/* Viewer Tags */
.vtags { position:absolute; top:12px; left:12px; display:flex; gap:6px; z-index:10; }
.vtag { padding:3px 10px; border-radius:12px; font-size:10px; font-weight:500; backdrop-filter:blur(6px); }
.vtag.live { background:rgba(0,255,136,0.15); color:var(--accent); border:1px solid rgba(0,255,136,0.2); }
.vtag.info { background:rgba(255,255,255,0.06); color:var(--text3); border:1px solid rgba(255,255,255,0.08); }

/* Bottom Stats */
.bstats { position:absolute; bottom:12px; left:12px; right:264px; display:flex; gap:8px; z-index:10; }
.bstat { flex:1; background:rgba(15,21,32,0.8); backdrop-filter:blur(6px); border:1px solid var(--border); border-radius:4px; padding:6px 10px; text-align:center; }
.bstat .bl { font-size:8px; color:var(--text3); }
.bstat .bv { font-size:14px; font-weight:500; font-family:'JetBrains Mono',monospace; }
.bstat .bv.green { color:var(--accent); }
.bstat .bv.blue { color:var(--blue); }
.bstat .bv.orange { color:var(--orange); }

/* Bottom Bar */
.bottombar { height:24px; background:var(--bg2); border-top:1px solid var(--border); display:flex; align-items:center; padding:0 14px; font-size:9px; color:var(--text3); gap:16px; font-family:'JetBrains Mono',monospace; }
.dot { width:5px; height:5px; border-radius:50%; display:inline-block; margin-right:3px; }
.dot.g { background:var(--accent); }

/* Toast */
.toast { position:fixed; bottom:36px; right:16px; padding:8px 14px; border-radius:var(--radius); font-size:11px; z-index:1000; transform:translateY(60px); opacity:0; transition:all 0.3s; }
.toast.show { transform:translateY(0); opacity:1; }
.toast.ok { background:rgba(0,204,106,0.9); color:#fff; }
.toast.err { background:rgba(239,68,68,0.9); color:#fff; }
.toast.info { background:rgba(15,21,32,0.9); color:var(--text); border:1px solid var(--border); }
</style>
</head>
<body>

<div class="topbar">
  <div class="topbar-left">
    <div class="logo"><svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><polygon points="12 2 22 8.5 22 15.5 12 22 2 15.5 2 8.5 12 2"/><line x1="12" y1="22" x2="12" y2="15.5"/><polyline points="22 8.5 12 15.5 2 8.5"/></svg> <span>LingTu</span> Dashboard</div>
    <span class="badge on" id="sysBadge">ONLINE</span>
  </div>
  <div class="topbar-right" id="connInfo">--</div>
</div>

<div class="main">
  <div class="sidebar">
    <div class="mode-section">
      <div class="mode-tabs">
        <button class="active" id="btnSlam" onclick="doSwitch('fastlio2')">SLAM 建图</button>
        <button id="btnNav" onclick="doSwitch('localizer')">导航巡航</button>
      </div>
    </div>

    <!-- Map Library (collapsible) -->
    <div class="panel">
      <div class="panel-header" onclick="toggle('maps')">
        <h3>地图库</h3>
        <div style="display:flex;align-items:center;gap:6px"><span class="count" id="mapCount">0</span><span class="arrow" id="maps_arrow">▾</span></div>
      </div>
      <div class="panel-body" id="maps_body" style="max-height:220px">
        <div class="map-list" id="mapList"><div style="padding:12px;text-align:center;color:var(--text3);font-size:10px">加载中...</div></div>
      </div>
    </div>

    <!-- Robot Status (collapsible) -->
    <div class="panel">
      <div class="panel-header" onclick="toggle('robot')">
        <h3>机器人状态</h3>
        <span class="arrow" id="robot_arrow">▾</span>
      </div>
      <div class="panel-body" id="robot_body" style="max-height:120px">
        <div class="status-grid">
          <div class="status-item"><svg width="12" height="12" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><rect x="1" y="6" width="18" height="12" rx="2"/><line x1="23" y1="13" x2="23" y2="11"/></svg><div><div class="sl">电池</div><div class="sv" id="sBat">--</div></div></div>
          <div class="status-item"><svg width="12" height="12" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M14 14.76V3.5a2.5 2.5 0 00-5 0v11.26a4.5 4.5 0 105 0z"/></svg><div><div class="sl">温度</div><div class="sv" id="sTemp">--</div></div></div>
          <div class="status-item"><svg width="12" height="12" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M5 12.55a11 11 0 0114.08 0"/><path d="M1.42 9a16 16 0 0121.16 0"/><path d="M8.53 16.11a6 6 0 016.95 0"/><line x1="12" y1="20" x2="12" y2="20"/></svg><div><div class="sl">WiFi</div><div class="sv" id="sWifi">--</div></div></div>
          <div class="status-item"><svg width="12" height="12" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><circle cx="12" cy="12" r="10"/><polyline points="12 6 12 12 16 14"/></svg><div><div class="sl">运行</div><div class="sv" id="sUp">--</div></div></div>
        </div>
      </div>
    </div>

    <!-- Telemetry (collapsible) -->
    <div class="panel">
      <div class="panel-header" onclick="toggle('tele')">
        <h3>遥测数据</h3>
        <span class="arrow" id="tele_arrow">▾</span>
      </div>
      <div class="panel-body" id="tele_body" style="max-height:100px">
        <div class="tele-grid">
          <div class="tele-item"><div class="tl">X 坐标</div><div class="tv" id="tX">--</div></div>
          <div class="tele-item"><div class="tl">Y 坐标</div><div class="tv" id="tY">--</div></div>
          <div class="tele-item"><div class="tl">频率</div><div class="tv" id="tHz">--</div></div>
          <div class="tele-item"><div class="tl">SLAM 帧</div><div class="tv" id="tF">--</div></div>
        </div>
      </div>
    </div>

    <div class="actions">
      <button class="abtn save" onclick="saveMap()" id="btnSave"><svg width="12" height="12" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M19 21H5a2 2 0 01-2-2V5a2 2 0 012-2h11l5 5v11a2 2 0 01-2 2z"/><polyline points="17,21 17,13 7,13 7,21"/><polyline points="7,3 7,8 15,8"/></svg> 保存地图</button>
      <button class="abtn refresh" onclick="showLive()"><svg width="12" height="12" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><polyline points="23 4 23 10 17 10"/><path d="M20.49 15a9 9 0 11-2.12-9.36L23 10"/></svg> 刷新 3D 预览</button>
      <button class="abtn estop" onclick="doSwitch('stop')"><svg width="12" height="12" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.5"><circle cx="12" cy="12" r="10"/><line x1="15" y1="9" x2="9" y2="15"/><line x1="9" y1="9" x2="15" y2="15"/></svg> EMERGENCY STOP</button>
    </div>
  </div>

  <div class="viewer-panel">
    <div class="vtags">
      <span class="vtag live" id="vTag">3D 实时渲染</span>
      <span class="vtag info">LIDAR ON</span>
    </div>
    <iframe src="/map/viewer" class="viewer-frame" id="vFrame"></iframe>

    <!-- Camera PIP -->
    <div class="camera-pip" id="cameraPip">
      <img id="camImg" alt="">
      <span class="pip-label">CAMERA</span>
      <button class="pip-close" onclick="toggleCam()">x</button>
    </div>

    <div class="bstats">
      <div class="bstat"><div class="bl">SLAM 频率</div><div class="bv green" id="sHz">--</div></div>
      <div class="bstat"><div class="bl">模式</div><div class="bv blue" id="sMode">--</div></div>
      <div class="bstat"><div class="bl">运行时间</div><div class="bv orange" id="sTime">00:00:00</div></div>
      <div class="bstat"><div class="bl">退化</div><div class="bv green" id="sDegen">0</div></div>
    </div>
  </div>
</div>

<div class="bottombar">
  <span><span class="dot g" id="bDot"></span>SLAM ACTIVE</span>
  <span id="bLat">延迟 --</span>
  <span style="flex:1"></span>
  <span>VER 2.1.0</span>
</div>

<div class="toast" id="toast"></div>

<script>
const t0=Date.now();
function T(m,t='info'){const e=document.getElementById('toast');e.textContent=m;e.className='toast '+t+' show';setTimeout(()=>e.classList.remove('show'),2500);}
async function F(u,o={}){try{const c=new AbortController();const t=setTimeout(()=>c.abort(),5000);o.signal=c.signal;const r=await fetch(u,o);clearTimeout(t);return await r.json();}catch(e){return null;}}

// Collapse
function toggle(id){
  const b=document.getElementById(id+'_body');
  const a=document.getElementById(id+'_arrow');
  b.classList.toggle('collapsed');
  a.classList.toggle('collapsed');
}

// Uptime
function up(){const s=Math.floor((Date.now()-t0)/1000);return String(s/3600|0).padStart(2,'0')+':'+String((s%3600)/60|0).padStart(2,'0')+':'+String(s%60).padStart(2,'0');}

// Camera snapshot polling (cross-RMW via subprocess)
let camVisible=true;
function fetchCam(){
  if(!camVisible)return;
  const img=document.getElementById('camImg');
  img.src='/api/v1/camera/snapshot?t='+Date.now();
}
function toggleCam(){
  const p=document.getElementById('cameraPip');
  camVisible=!camVisible;
  p.style.display=camVisible?'block':'none';
}
fetchCam();
setInterval(fetchCam, 2000);

// Poll
async function poll(){
  const pt=Date.now();
  const h=await F('/api/v1/health');
  document.getElementById('bLat').textContent='延迟 '+(Date.now()-pt)+'ms';
  if(!h){document.getElementById('sysBadge').className='badge off';document.getElementById('sysBadge').textContent='OFFLINE';return;}
  document.getElementById('sysBadge').className='badge on';document.getElementById('sysBadge').textContent='ONLINE';
  document.getElementById('tHz').textContent=(h.slam_hz||0).toFixed(1);
  document.getElementById('tF').textContent=h.map_points||0;
  document.getElementById('sHz').textContent=(h.slam_hz||0).toFixed(1)+'Hz';
  document.getElementById('sTime').textContent=up();

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
  }
  // Robot status (placeholder — real values from driver when available)
  document.getElementById('sUp').textContent=up();
}

async function doSwitch(p){
  const n={fastlio2:'建图',localizer:'导航',stop:'停止'};
  T('切换: '+n[p],'info');
  const r=await F('/api/v1/slam/switch',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({profile:p})});
  if(r&&r.success){T('已切换','ok');setTimeout(poll,2000);}else T('失败','err');
}

// Maps
async function loadMaps(){
  const r=await F('/api/v1/slam/maps');
  if(!r||!r.maps)return;
  const maps=r.maps.filter(m=>m.has_pcd);
  const act=r.active||'';
  document.getElementById('mapCount').textContent=maps.length;
  const el=document.getElementById('mapList');
  if(!maps.length){el.innerHTML='<div style="padding:12px;text-align:center;color:var(--text3);font-size:10px">暂无地图</div>';return;}
  el.innerHTML=maps.map(m=>{
    const a=m.name===act||m.active;
    return '<div class="map-item'+(a?' active':'')+'"><div class="map-item-top"><span class="map-item-name">'+m.name+'</span>'+(a?'<span class="map-item-badge">使用中</span>':'')+'</div>'+
      '<div class="map-item-meta">'+(m.size||'')+' '+(m.patches?m.patches+' KF':'')+'</div>'+
      '<div class="map-item-actions">'+
        (a?'<button class="mbtn primary" onclick="viewSaved(&quot;'+m.name+'&quot;)">3D</button>':'<button class="mbtn ghost" onclick="setAct(&quot;'+m.name+'&quot;)">激活</button>')+
        '<button class="mbtn icon" onclick="renameMap(&quot;'+m.name+'&quot;)"><svg width="10" height="10" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M11 4H4a2 2 0 00-2 2v14a2 2 0 002 2h14a2 2 0 002-2v-7"/><path d="M18.5 2.5a2.121 2.121 0 013 3L12 15l-4 1 1-4 9.5-9.5z"/></svg></button>'+
        '<button class="mbtn icon" onclick="delMap(&quot;'+m.name+'&quot;)" style="color:var(--red)"><svg width="10" height="10" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><polyline points="3,6 5,6 21,6"/><path d="M19 6v14a2 2 0 01-2 2H7a2 2 0 01-2-2V6m3 0V4a2 2 0 012-2h4a2 2 0 012 2v2"/></svg></button>'+
      '</div></div>';
  }).join('');
}

function showLive(){document.getElementById('vFrame').src='/map/viewer?t='+Date.now();document.getElementById('vTag').textContent='3D 实时渲染';}
function viewSaved(n){document.getElementById('vFrame').src='/map/viewer?map='+encodeURIComponent(n);document.getElementById('vTag').textContent='已保存: '+n;}

async function setAct(n){
  T('激活: '+n,'info');
  const r=await F('/api/v1/map/activate',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({name:n})});
  if(r&&r.success){T('已激活','ok');loadMaps();}else T('激活失败','err');
}
async function renameMap(n){
  const nn=prompt('新名称:',n);if(!nn||nn===n)return;
  const r=await F('/api/v1/map/rename',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({old_name:n,new_name:nn})});
  if(r&&r.success){T('已重命名','ok');loadMaps();}else T('失败','err');
}
async function delMap(n){
  if(!confirm('删除 "'+n+'" ?'))return;
  const r=await F('/api/v1/maps',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({action:'delete',name:n})});
  if(r&&r.success!==false){T('已删除','ok');loadMaps();}else T('失败','err');
}
async function saveMap(){
  const n=prompt('地图名称:','map_'+new Date().toISOString().slice(0,16).replace(/[T:]/g,'_'));if(!n)return;
  T('保存中...','info');document.getElementById('btnSave').disabled=true;
  const r=await F('/api/v1/map/save',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({name:n})});
  document.getElementById('btnSave').disabled=false;
  if(r&&r.success){T('已保存: '+r.size,'ok');loadMaps();}else T('保存失败','err');
}

poll();loadMaps();
setInterval(poll,3000);
setInterval(()=>{document.getElementById('sTime').textContent=up();document.getElementById('sUp').textContent=up();},1000);
</script>
</body></html>'''
