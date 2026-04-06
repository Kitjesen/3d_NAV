"""Map Dashboard — web UI for SLAM control and map management.

Served at GET /dashboard by GatewayModule.
Calls existing REST APIs: /api/v1/maps, /api/v1/health, /api/v1/state.
Zero external dependencies — pure HTML/CSS/JS.
"""


def generate_dashboard_html() -> str:
    return '''<!DOCTYPE html>
<html lang="zh-CN"><head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>灵途 — 地图控制台</title>
<style>
:root {
  --bg: #0d1117; --bg2: #161b22; --bg3: #21262d; --border: #30363d;
  --text: #e6edf3; --text2: #8b949e; --accent: #00ff88; --accent2: #238636;
  --warn: #d29922; --danger: #f85149; --blue: #58a6ff; --purple: #bc8cff;
  --radius: 10px; --shadow: 0 2px 12px rgba(0,0,0,0.4);
}
* { margin:0; padding:0; box-sizing:border-box; }
body { font-family: -apple-system, 'PingFang SC', 'Microsoft YaHei', 'Segoe UI', monospace; background:var(--bg); color:var(--text); min-height:100vh; }

.header { background:var(--bg2); border-bottom:1px solid var(--border); padding:16px 24px; display:flex; align-items:center; justify-content:space-between; }
.header h1 { font-size:20px; font-weight:600; }
.header h1 span { color:var(--accent); }
.header .conn { font-size:13px; color:var(--text2); }
.header .dot { display:inline-block; width:8px; height:8px; border-radius:50%; margin-right:6px; }
.header .dot.on { background:var(--accent); box-shadow:0 0 8px var(--accent); }
.header .dot.off { background:var(--danger); }

.container { display:grid; grid-template-columns:300px 1fr; gap:0; min-height:calc(100vh - 60px); }
.sidebar { background:var(--bg2); border-right:1px solid var(--border); padding:16px; overflow-y:auto; }
.main { padding:20px; overflow-y:auto; }

.card { background:var(--bg3); border:1px solid var(--border); border-radius:var(--radius); padding:14px; margin-bottom:14px; }
.card h3 { font-size:12px; color:var(--text2); text-transform:uppercase; letter-spacing:1.5px; margin-bottom:10px; }

.slam-mode { font-size:16px; font-weight:600; margin-bottom:10px; }
.mode-tag { display:inline-block; padding:3px 10px; border-radius:6px; font-size:13px; font-weight:500; }
.mode-tag.mapping { background:rgba(0,255,136,0.15); color:var(--accent); }
.mode-tag.nav { background:rgba(88,166,255,0.15); color:var(--blue); }
.mode-tag.stopped { background:rgba(248,81,73,0.15); color:var(--danger); }

.svc-row { display:flex; align-items:center; padding:4px 0; font-size:12px; }
.svc-row .d { width:6px; height:6px; border-radius:50%; margin-right:8px; }
.svc-row .d.on { background:var(--accent); }
.svc-row .d.off { background:var(--text2); }
.svc-row .n { flex:1; color:var(--text2); }

.switch-btns { display:flex; gap:6px; margin-top:12px; }
.btn { border:none; padding:7px 14px; border-radius:6px; font-size:12px; font-weight:500; cursor:pointer; transition:all 0.15s; }
.btn:hover { transform:translateY(-1px); }
.btn:active { transform:translateY(0); }
.btn:disabled { opacity:0.35; cursor:not-allowed; transform:none; }
.btn-g { background:var(--accent2); color:#fff; }
.btn-g:hover { background:#2ea043; }
.btn-b { background:rgba(88,166,255,0.15); color:var(--blue); border:1px solid rgba(88,166,255,0.25); }
.btn-b:hover { background:rgba(88,166,255,0.25); }
.btn-r { background:rgba(248,81,73,0.1); color:var(--danger); border:1px solid rgba(248,81,73,0.25); }
.btn-r:hover { background:rgba(248,81,73,0.2); }
.btn-sm { padding:4px 10px; font-size:11px; }

.stats { display:grid; grid-template-columns:1fr 1fr; gap:6px; }
.st { text-align:center; padding:8px 4px; background:var(--bg); border-radius:6px; }
.st .v { font-size:18px; font-weight:700; color:var(--accent); }
.st .l { font-size:10px; color:var(--text2); margin-top:2px; }

.map-grid { display:grid; grid-template-columns:repeat(auto-fill, minmax(260px, 1fr)); gap:14px; }
.mc { background:var(--bg2); border:1px solid var(--border); border-radius:var(--radius); padding:14px; transition:all 0.15s; position:relative; }
.mc:hover { border-color:var(--accent); box-shadow:var(--shadow); }
.mc.act { border-color:var(--accent); }
.mc.act::before { content:'\\5F53\\524D\\4F7F\\7528'; position:absolute; top:8px; right:8px; background:var(--accent); color:var(--bg); padding:2px 8px; border-radius:4px; font-size:10px; font-weight:700; }
.mc .nm { font-size:14px; font-weight:600; margin-bottom:4px; word-break:break-all; }
.mc .mt { font-size:11px; color:var(--text2); margin-bottom:8px; }
.mc .mt span { margin-right:10px; }
.mc .acts { display:flex; gap:5px; flex-wrap:wrap; }

.empty-state { text-align:center; padding:40px; color:var(--text2); }
.empty-state p { margin-top:8px; font-size:13px; }

.viewer-frame { width:100%; height:420px; border:1px solid var(--border); border-radius:var(--radius); background:var(--bg); }

.toast { position:fixed; bottom:20px; right:20px; padding:10px 18px; border-radius:8px; font-size:12px; z-index:1000; transform:translateY(80px); opacity:0; transition:all 0.3s; }
.toast.show { transform:translateY(0); opacity:1; }
.toast.ok { background:var(--accent2); color:#fff; }
.toast.err { background:var(--danger); color:#fff; }
.toast.info { background:var(--bg3); color:var(--text); border:1px solid var(--border); }

.spinner { display:inline-block; width:14px; height:14px; border:2px solid var(--text2); border-top-color:var(--accent); border-radius:50%; animation:sp 0.6s linear infinite; }
@keyframes sp { to { transform:rotate(360deg); } }

@media (max-width: 768px) {
  .container { grid-template-columns:1fr; }
  .sidebar { border-right:none; border-bottom:1px solid var(--border); }
  .map-grid { grid-template-columns:1fr; }
}
</style>
</head>
<body>

<div class="header">
  <h1><span>灵途</span> 地图控制台</h1>
  <div class="conn"><span class="dot" id="hDot"></span><span id="hSt">连接中...</span></div>
</div>

<div class="container">
  <div class="sidebar">
    <div class="card">
      <h3>SLAM 控制</h3>
      <div class="slam-mode" id="slamMode"><span class="spinner"></span></div>
      <div id="slamSvc"></div>
      <div class="switch-btns">
        <button class="btn btn-g" onclick="doSwitch('fastlio2')" id="bMap">建图</button>
        <button class="btn btn-b" onclick="doSwitch('localizer')" id="bNav">导航</button>
        <button class="btn btn-r btn-sm" onclick="doSwitch('stop')" id="bStop">停止</button>
      </div>
    </div>

    <div class="card">
      <h3>机器人状态</h3>
      <div class="stats">
        <div class="st"><div class="v" id="pX">-</div><div class="l">X (m)</div></div>
        <div class="st"><div class="v" id="pY">-</div><div class="l">Y (m)</div></div>
        <div class="st"><div class="v" id="hz">-</div><div class="l">SLAM 频率</div></div>
        <div class="st"><div class="v" id="pts">-</div><div class="l">地图点数</div></div>
      </div>
    </div>

    <div class="card">
      <h3>快捷操作</h3>
      <div style="display:flex;flex-direction:column;gap:6px">
        <button class="btn btn-g" onclick="saveMap()" id="bSave">保存当前地图</button>
        <button class="btn btn-b" onclick="window.open('/map/viewer','_blank')">打开 3D 查看器</button>
      </div>
    </div>
  </div>

  <div class="main">
    <div style="display:flex;justify-content:space-between;align-items:center;margin-bottom:14px">
      <h2>地图列表</h2>
      <button class="btn btn-sm btn-b" onclick="loadMaps()">刷新</button>
    </div>
    <div class="map-grid" id="mGrid">
      <div class="empty-state"><span class="spinner"></span><p>加载中...</p></div>
    </div>

    <div style="margin-top:24px">
      <h2 style="margin-bottom:10px">3D 预览</h2>
      <iframe src="/map/viewer" class="viewer-frame" id="vFrame"></iframe>
    </div>
  </div>
</div>

<div class="toast" id="toast"></div>

<script>
function T(msg,t='info'){const e=document.getElementById('toast');e.textContent=msg;e.className='toast '+t+' show';setTimeout(()=>e.classList.remove('show'),3000);}
async function F(u,o={}){
  try{
    const c=new AbortController();
    const t=setTimeout(()=>c.abort(),5000);
    o.signal=c.signal;
    const r=await fetch(u,o);
    clearTimeout(t);
    return await r.json();
  }catch(e){console.warn('API error:',u,e.message);return null;}
}

// SLAM status
async function pollSlam(){
  const h=await F('/api/v1/health');
  if(!h){document.getElementById('hDot').className='dot off';document.getElementById('hSt').textContent='离线';return;}
  document.getElementById('hDot').className='dot on';
  document.getElementById('hSt').textContent='在线';
  const gw=h.gateway||{};
  const p=gw.map_points||0;
  document.getElementById('pts').textContent=p>1000?(p/1000|0)+'K':p;

  const st=await F('/api/v1/state');
  if(st&&st.odometry){
    document.getElementById('pX').textContent=(st.odometry.x||0).toFixed(2);
    document.getElementById('pY').textContent=(st.odometry.y||0).toFixed(2);
  }

  const s=await F('/api/v1/slam/status');
  if(!s)return;
  const el=document.getElementById('slamSvc');
  let html='';
  const svcs=s.services||{};
  for(const[n,v]of Object.entries(svcs)){
    const on=v==='running'||v==='active';
    html+='<div class="svc-row"><div class="d '+(on?'on':'off')+'"></div><span class="n">'+n+'</span><span style="color:'+(on?'var(--accent)':'var(--text2)')+';font-size:11px">'+v+'</span></div>';
  }
  el.innerHTML=html;

  const m=document.getElementById('slamMode');
  if(s.mode==='fastlio2'){m.innerHTML='<span class="mode-tag mapping">建图模式</span>';document.getElementById('bMap').disabled=true;document.getElementById('bNav').disabled=false;}
  else if(s.mode==='localizer'){m.innerHTML='<span class="mode-tag nav">导航模式</span>';document.getElementById('bMap').disabled=false;document.getElementById('bNav').disabled=true;}
  else{m.innerHTML='<span class="mode-tag stopped">已停止</span>';document.getElementById('bMap').disabled=false;document.getElementById('bNav').disabled=false;}
}

async function doSwitch(p){
  const names={fastlio2:'建图模式',localizer:'导航模式',stop:'停止'};
  T('切换到'+names[p]+'...','info');
  const r=await F('/api/v1/slam/switch',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({profile:p})});
  if(r&&r.success){T('已切换: '+names[p],'ok');setTimeout(pollSlam,2000);}
  else T('切换失败: '+(r?.message||'未知错误'),'err');
}

// Maps — try API first, fallback to /api/v1/slam/maps
let activeName='';

async function loadMaps(){
  // Primary: filesystem scan (has size, patches, active info)
  let r=await F('/api/v1/slam/maps');
  let maps=[], active='';

  if(r&&r.maps){
    maps=r.maps.filter(m=>m.has_pcd); active=r.active||'';
  } else {
    // Fallback: MapManagerModule API
    r=await F('/api/v1/maps',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({action:'list'})});
    if(r&&r.maps) { maps=r.maps; active=r.active||''; }
  }

  activeName=active;
  const g=document.getElementById('mGrid');

  if(!maps||maps.length===0){
    g.innerHTML='<div class="empty-state"><p>暂无地图。开始建图后保存即可。</p></div>';
    return;
  }

  g.innerHTML=maps.map(m=>{
    const name=typeof m==='string'?m:(m.name||'');
    const isAct=name===active||(m&&m.active);
    const size=m.size||'';
    const kf=m.patches||m.keyframes||0;
    return '<div class="mc'+(isAct?' act':'')+'">' +
      '<div class="nm">'+name+'</div>' +
      '<div class="mt">'+(size?'<span>'+size+'</span>':'')+(kf?'<span>'+kf+' 关键帧</span>':'')+'</div>' +
      '<div class="acts">' +
        (!isAct?'<button class="btn btn-g btn-sm" onclick="setAct(&quot;'+name+'&quot;)">设为当前</button>':'')+
        '<button class="btn btn-b btn-sm" onclick="document.getElementById(&quot;vFrame&quot;).src=&quot;/map/viewer?map='+encodeURIComponent(name)+'&quot;">3D 查看</button>'+
        '<button class="btn btn-sm" style="background:rgba(188,140,255,0.15);color:var(--purple);border:1px solid rgba(188,140,255,0.25)" onclick="renameMap(&quot;'+name+'&quot;)">重命名</button>'+
        '<button class="btn btn-r btn-sm" onclick="delMap(&quot;'+name+'&quot;)">删除</button>'+
      '</div></div>';
  }).join('');
}

async function setAct(n){
  T('设置当前地图: '+n,'info');
  const r=await F('/api/v1/maps',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({action:'set_active',name:n})});
  if(r&&r.success!==false){T(n+' 已设为当前地图','ok');loadMaps();}
  else T('失败: '+(r?.error||'未知'),'err');
}

async function renameMap(n){
  const nn=prompt('新名称:',n);
  if(!nn||nn===n)return;
  T('重命名: '+n+' → '+nn,'info');
  const r=await F('/api/v1/map/rename',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({old_name:n,new_name:nn})});
  if(r&&r.success){T('已重命名: '+nn,'ok');loadMaps();}
  else T('重命名失败: '+(r?.message||'未知'),'err');
}

async function delMap(n){
  if(!confirm('确定删除地图 "'+n+'" ?'))return;
  const r=await F('/api/v1/maps',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({action:'delete',name:n})});
  if(r&&r.success!==false){T('已删除: '+n,'ok');loadMaps();}
  else T('删除失败: '+(r?.error||'未知'),'err');
}

async function saveMap(){
  const n=prompt('地图名称:','map_'+new Date().toISOString().slice(0,16).replace(/[T:]/g,'_'));
  if(!n)return;
  T('正在保存 "'+n+'" (约10-30秒)...','info');
  document.getElementById('bSave').disabled=true;
  const r=await F('/api/v1/map/save',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({name:n})});
  document.getElementById('bSave').disabled=false;
  if(r&&r.success){T('已保存: '+n+' ('+r.size+')','ok');loadMaps();}
  else T('保存失败: '+(r?.errors?.join(', ')||r?.message||'SLAM 未运行'),'err');
}

// Init + polling (independent, non-blocking)
try{pollSlam();}catch(e){console.error('pollSlam init:',e);}
try{loadMaps();}catch(e){console.error('loadMaps init:',e);}
setInterval(()=>{try{pollSlam();}catch(e){}},4000);
</script>
</body></html>'''
