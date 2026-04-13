"""Generate a self-contained demo HTML for the LingTu map viewer (new design) and open it."""
import numpy as np, tempfile, os, subprocess, base64

rng = np.random.default_rng(42)
fx = rng.uniform(-8, 8, 3000); fy = rng.uniform(-6, 6, 3000); fz = rng.uniform(-0.05, 0.05, 3000)
wx = np.concatenate([rng.uniform(-8,-7.7,400), rng.uniform(7.7,8,400), rng.uniform(-8,8,400), rng.uniform(-8,8,400)])
wy = np.concatenate([rng.uniform(-6,6,400), rng.uniform(-6,6,400), rng.uniform(-6,-5.7,400), rng.uniform(5.7,6,400)])
wz = rng.uniform(0, 1.5, 1600)
pts = np.column_stack([np.concatenate([fx,wx]), np.concatenate([fy,wy]), np.concatenate([fz,wz])])

n = len(pts)
z = pts[:,2]; zmin, zmax = float(z.min()), float(z.max())
cx, cy = float(pts[:,0].mean()), float(pts[:,1].mean())
coords = ",".join(f"{pts[i,0]:.3f},{pts[i,1]:.3f},{pts[i,2]:.3f}" for i in range(n))
rx, ry, ryaw = 1.2, -0.8, 0.4

# Build a tiny fake costmap (base64 encoded)
cm_cols = 60
cm_grid = np.zeros((cm_cols, cm_cols), dtype=np.uint8)
for r in range(cm_cols):
    for c in range(cm_cols):
        wall = (r<3 or r>cm_cols-4 or c<3 or c>cm_cols-4)
        obs  = ((r-20)**2+(c-15)**2<25) or ((r-35)**2+(c-40)**2<16)
        inf_ = ((r-20)**2+(c-15)**2<64) or ((r-35)**2+(c-40)**2<49)
        if wall or obs: cm_grid[r,c] = 100
        elif inf_:      cm_grid[r,c] = 40
cm_b64 = base64.b64encode(cm_grid.tobytes()).decode()

html = f"""<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<title>灵途 · LingTu — Demo</title>
<style>
*{{box-sizing:border-box;margin:0;padding:0;}}
body{{background:#05060d;overflow:hidden;font-family:-apple-system,BlinkMacSystemFont,"SF Pro Display","Helvetica Neue",sans-serif;}}
.glass{{background:rgba(4,12,35,0.78);border:1px solid rgba(40,90,200,0.30);backdrop-filter:blur(22px);-webkit-backdrop-filter:blur(22px);border-radius:14px;}}
#hud{{position:fixed;top:16px;left:16px;padding:14px 18px;min-width:210px;z-index:50;}}
.lbl{{font-size:9px;letter-spacing:2.5px;color:rgba(80,140,255,0.55);text-transform:uppercase;margin-bottom:8px;}}
.row{{font-size:12.5px;color:#8ab4ff;letter-spacing:.4px;line-height:2.0;font-variant-numeric:tabular-nums;}}
.row b{{color:#5599ff;font-weight:600;}}
#ctrlBar{{position:fixed;bottom:22px;left:50%;transform:translateX(-50%);display:flex;align-items:center;gap:10px;padding:9px 16px;z-index:50;white-space:nowrap;}}
#missionTxt{{font-size:11.5px;color:rgba(100,155,255,0.8);letter-spacing:.5px;min-width:160px;}}
.sep{{width:1px;height:22px;background:rgba(50,100,200,0.22);}}
.btn{{background:rgba(20,50,130,0.55);border:1px solid rgba(55,110,240,0.45);color:#7aacff;font-size:10.5px;letter-spacing:.8px;padding:6px 15px;border-radius:8px;cursor:pointer;font-family:inherit;transition:all .16s;outline:none;}}
.btn:hover{{background:rgba(40,85,190,0.70);color:#c0d4ff;border-color:rgba(90,150,255,0.65);}}
.btn.stop{{background:rgba(110,20,20,0.55);border-color:rgba(220,50,50,0.45);color:#ff9999;}}
.btn.stop:hover{{background:rgba(160,30,30,0.70);color:#ffcccc;}}
.btn.active{{background:rgba(120,20,20,0.55);border-color:rgba(220,60,60,0.50);color:#ffaaaa;}}
#hint{{position:fixed;top:16px;left:50%;transform:translateX(-50%);font-size:10px;color:rgba(50,100,200,0.42);letter-spacing:1.8px;pointer-events:none;z-index:40;}}
#toast{{position:fixed;bottom:78px;left:50%;transform:translateX(-50%);padding:9px 22px;font-size:11.5px;letter-spacing:.5px;display:none;pointer-events:none;z-index:200;color:#7ab8ff;}}
.dot{{width:6px;height:6px;border-radius:50%;background:#ff4444;display:inline-block;margin-right:8px;flex-shrink:0;}}
.dot.live{{background:#44cc88;box-shadow:0 0 7px #44cc8899;}}
</style>
</head>
<body>
<script src="https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js"></script>
<script src="https://cdn.jsdelivr.net/npm/three@0.128.0/examples/js/controls/OrbitControls.js"></script>
<script src="https://cdn.jsdelivr.net/npm/three@0.128.0/examples/js/loaders/STLLoader.js"></script>

<div id="hud" class="glass">
  <div class="lbl">灵途 · 导航状态 [DEMO]</div>
  <div class="row">X&nbsp;<b id="hx">{rx:.2f}</b>&nbsp;&nbsp;Y&nbsp;<b id="hy">{ry:.2f}</b></div>
  <div class="row">模式&nbsp;<b id="hmode">自主</b></div>
  <div class="row" id="hmission" style="font-size:11px;color:rgba(80,130,220,0.60);">演示模式 — 机器人未连接</div>
</div>

<div id="ctrlBar" class="glass">
  <span class="dot live" id="dot"></span>
  <span id="missionTxt">待机</span>
  <div class="sep"></div>
  <button class="btn stop" onclick="showToast('■ 紧急停止 [Demo]')">■&nbsp;停止</button>
  <button class="btn" id="expBtn" onclick="toggleExplore()">▶&nbsp;探索</button>
</div>

<div id="hint">单击地图发送导航目标 · 拖拽旋转 · 滚轮缩放 · [DEMO]</div>
<div id="toast" class="glass"></div>

<script>
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x05060d);
scene.fog = new THREE.FogExp2(0x05060d, 0.0055);
const camera = new THREE.PerspectiveCamera(50, innerWidth/innerHeight, 0.1, 600);
camera.position.set({cx+5:.1f}, {cy-28:.1f}, 28);
camera.up.set(0,0,1);
const renderer = new THREE.WebGLRenderer({{antialias:true}});
renderer.setSize(innerWidth,innerHeight);
renderer.setPixelRatio(Math.min(devicePixelRatio,2));
renderer.toneMapping = THREE.ACESFilmicToneMapping;
renderer.toneMappingExposure = 1.1;
document.body.appendChild(renderer.domElement);
const controls = new THREE.OrbitControls(camera,renderer.domElement);
controls.target.set({cx:.1f},{cy:.1f},1.5);
controls.enableDamping=true; controls.dampingFactor=0.06;
controls.minDistance=2; controls.maxDistance=250;
controls.update();

// Lighting
scene.add(new THREE.AmbientLight(0x1a2a60,2.5));
const dl=new THREE.DirectionalLight(0x5588ff,2.0); dl.position.set(4,-6,10); scene.add(dl);
const dl2=new THREE.DirectionalLight(0x2244aa,0.8); dl2.position.set(-4,4,3); scene.add(dl2);

// Point cloud — cold blue height gradient
(function(){{
  const geo=new THREE.BufferGeometry();
  const pos=new Float32Array({n*3}); const col=new Float32Array({n*3});
  const pts=[{coords}];
  const zmin={zmin:.3f},zmax={zmax:.3f},zr=zmax-zmin||1;
  for(let i=0;i<{n};i++){{
    pos[i*3]=pts[i*3];pos[i*3+1]=pts[i*3+1];pos[i*3+2]=pts[i*3+2];
    const t=(pts[i*3+2]-zmin)/zr;
    if(t<0.5){{const s=t*2;col[i*3]=0.04*(1-s);col[i*3+1]=0.10*(1-s)+0.33*s;col[i*3+2]=0.24*(1-s)+0.78*s;}}
    else{{const s=(t-0.5)*2;col[i*3]=0;col[i*3+1]=0.33*(1-s)+0.90*s;col[i*3+2]=0.78*(1-s)+1.0*s;}}
  }}
  geo.setAttribute('position',new THREE.BufferAttribute(pos,3));
  geo.setAttribute('color',new THREE.BufferAttribute(col,3));
  scene.add(new THREE.Points(geo,new THREE.PointsMaterial({{size:0.035,vertexColors:true,sizeAttenuation:true,transparent:true,opacity:0.88}})));
}})();

// Ground grid
const grid=new THREE.GridHelper(100,100,0x0c1830,0x070e1d);
grid.rotation.x=Math.PI/2; grid.position.set({cx:.1f},{cy:.1f},{zmin:.2f}-0.01); scene.add(grid);

// Robot group
const robotGroup=new THREE.Group();
robotGroup.position.set({rx:.3f},{ry:.3f},0); robotGroup.rotation.z={ryaw:.4f};
scene.add(robotGroup);
// Fallback wireframe body
const fb=new THREE.Mesh(new THREE.BoxGeometry(0.62,0.36,0.18),
  new THREE.MeshBasicMaterial({{color:0x2255cc,transparent:true,opacity:0.55,wireframe:true}}));
fb.position.set(0,0,0.40); robotGroup.add(fb);
// STL meshes (will 404 gracefully in demo)
if(typeof THREE.STLLoader!=='undefined'){{
  const loader=new THREE.STLLoader();
  const mb=new THREE.MeshPhongMaterial({{color:0xb8cdff,specular:0x4466bb,shininess:55,transparent:true,opacity:0.92}});
  const ml=new THREE.MeshPhongMaterial({{color:0x8aaae8,specular:0x223388,shininess:38,transparent:true,opacity:0.88}});
  function load(url,mat,px,py,pz){{loader.load(url,function(g){{g.computeVertexNormals();const m=new THREE.Mesh(g,mat.clone());m.position.set(px,py,pz);robotGroup.add(m);fb.visible=false;}},undefined,()=>{{}});}}
  const bz=0.40;
  load('/robot/meshes/base_link.STL',mb,0,0,bz);
  load('/robot/meshes/fr_hip_Link.STL',ml,0.2395,-0.0646,bz+0.0807);
  load('/robot/meshes/fl_hip_Link.STL',ml,0.2395,0.0654,bz+0.0807);
  load('/robot/meshes/rr_hip_Link.STL',ml,-0.2395,-0.0662,bz+0.0807);
  load('/robot/meshes/rl_hip_Link.STL',ml,-0.2395,0.0638,bz+0.0807);
}}

// Pulsing ring
const ringMat=new THREE.MeshBasicMaterial({{color:0x3377ff,transparent:true,opacity:0.55,side:THREE.DoubleSide}});
const ringMesh=new THREE.Mesh(new THREE.RingGeometry(0.38,0.52,56),ringMat);
ringMesh.rotation.x=Math.PI/2; ringMesh.position.set({rx:.3f},{ry:.3f},0.015); scene.add(ringMesh);

// Goal marker
const goalGroup=new THREE.Group(); goalGroup.visible=false;
const gS=new THREE.Mesh(new THREE.SphereGeometry(0.13,20,20),new THREE.MeshBasicMaterial({{color:0x00e5ff,transparent:true,opacity:0.88}}));
gS.position.z=0.13; goalGroup.add(gS);
const gR1=new THREE.Mesh(new THREE.RingGeometry(0.18,0.30,48),new THREE.MeshBasicMaterial({{color:0x00e5ff,transparent:true,opacity:0.55,side:THREE.DoubleSide}}));
gR1.rotation.x=Math.PI/2; goalGroup.add(gR1);
const gR2=new THREE.Mesh(new THREE.RingGeometry(0.34,0.46,48),new THREE.MeshBasicMaterial({{color:0x00e5ff,transparent:true,opacity:0.22,side:THREE.DoubleSide}}));
gR2.rotation.x=Math.PI/2; goalGroup.add(gR2);
scene.add(goalGroup);

// Trajectory — rolling ring-buffer with fade
const TM=2000;
const tPos=new Float32Array(TM*3); const tCol=new Float32Array(TM*3);
const tGeo=new THREE.BufferGeometry();
tGeo.setAttribute('position',new THREE.BufferAttribute(tPos,3));
tGeo.setAttribute('color',new THREE.BufferAttribute(tCol,3));
tGeo.setDrawRange(0,0);
scene.add(new THREE.Line(tGeo,new THREE.LineBasicMaterial({{vertexColors:true,transparent:true,opacity:0.9}})));
let tN=0;

// Mock demo trajectory — spiral
(function(){{
  for(let i=0;i<300;i++){{
    const tt=i/300*Math.PI*4;
    const ox={cx:.1f}+Math.cos(tt)*3.5, oy={cy:.1f}+Math.sin(tt)*2;
    if(tN<TM){{tPos[tN*3]=ox;tPos[tN*3+1]=oy;tPos[tN*3+2]=0.08;tN++;}}
  }}
  const n2=Math.min(tN,TM);
  for(let i=0;i<n2;i++){{const f=i/n2;tCol[i*3]=0.04+0.22*f;tCol[i*3+1]=0.20+0.47*f;tCol[i*3+2]=0.80+0.20*f;}}
  tGeo.attributes.position.needsUpdate=true; tGeo.attributes.color.needsUpdate=true;
  tGeo.setDrawRange(0,tN);
}})();

// Planned path (green)
const pathGeo=new THREE.BufferGeometry();
(function(){{
  const a=[];
  for(let i=0;i<25;i++){{const t=i/24;a.push({rx:.2f}+t*5,{ry:.2f}+t*3,0.05);}}
  pathGeo.setAttribute('position',new THREE.BufferAttribute(new Float32Array(a),3));
}})();
scene.add(new THREE.Line(pathGeo,new THREE.LineBasicMaterial({{color:0x00ffaa,transparent:true,opacity:0.72}})));

// Costmap (mock, pre-built)
(function(){{
  const cols={cm_cols},res=0.2,ox=-6.0,oy=-5.0,sz=cols*res;
  const canvas=document.createElement('canvas'); canvas.width=cols; canvas.height=cols;
  const ctx=canvas.getContext('2d');
  const b64='{cm_b64}';
  const bin=atob(b64); const arr=new Uint8Array(bin.length);
  for(let i=0;i<bin.length;i++) arr[i]=bin.charCodeAt(i);
  const img=ctx.createImageData(cols,cols);
  for(let i=0;i<cols*cols;i++){{
    const v=arr[i]; if(v<=0){{img.data[i*4+3]=0;continue;}}
    img.data[i*4]=v>50?215:175; img.data[i*4+1]=v>50?42:85;
    img.data[i*4+2]=v>50?28:28; img.data[i*4+3]=Math.min(205,v*2+55);
  }}
  ctx.putImageData(img,0,0);
  const tex=new THREE.CanvasTexture(canvas);
  const mesh=new THREE.Mesh(new THREE.PlaneGeometry(sz,sz),
    new THREE.MeshBasicMaterial({{map:tex,transparent:true,opacity:0.50,side:THREE.DoubleSide,depthWrite:false}}));
  mesh.position.set(ox+sz/2,oy+sz/2,0.01); scene.add(mesh);
}})();

// UI helpers
const $toast=document.getElementById('toast');
const $mission=document.getElementById('missionTxt');
const $expBtn=document.getElementById('expBtn');
let _exploring=false;
function showToast(msg,ms){{$toast.textContent=msg;$toast.style.display='block';clearTimeout($toast._t);$toast._t=setTimeout(()=>{{$toast.style.display='none';}},ms||3000);}}
function toggleExplore(){{
  _exploring=!_exploring;
  $expBtn.textContent=_exploring?'■ 停止探索':'▶ 探索';
  $expBtn.className=_exploring?'btn active':'btn';
  $mission.textContent=_exploring?'自主探索中...':'待机';
  showToast(_exploring?'自主探索启动 [Demo]':'探索已停止 [Demo]');
}}

// Click-to-navigate
const ray=new THREE.Raycaster(); const mv=new THREE.Vector2();
const gp=new THREE.Plane(new THREE.Vector3(0,0,1),0); let _md=null;
renderer.domElement.addEventListener('mousedown',e=>{{_md={{x:e.clientX,y:e.clientY}};}});
renderer.domElement.addEventListener('mouseup',e=>{{
  if(!_md)return; const dx=e.clientX-_md.x,dy=e.clientY-_md.y; _md=null;
  if(dx*dx+dy*dy>25)return;
  mv.x=(e.clientX/innerWidth)*2-1; mv.y=-(e.clientY/innerHeight)*2+1;
  ray.setFromCamera(mv,camera); const tgt=new THREE.Vector3();
  if(!ray.ray.intersectPlane(gp,tgt))return;
  goalGroup.position.set(tgt.x,tgt.y,0); goalGroup.visible=true;
  showToast('导航目标 → ('+tgt.x.toFixed(2)+', '+tgt.y.toFixed(2)+')  [Demo]');
  $mission.textContent='导航中 → ('+tgt.x.toFixed(1)+', '+tgt.y.toFixed(1)+')';
}});

// Animation loop
let _t=0;
(function _loop(){{
  requestAnimationFrame(_loop); _t+=0.016;
  ringMat.opacity=0.35+0.25*(0.5+0.5*Math.sin(_t*2.8));
  ringMesh.scale.setScalar(1.0+0.07*Math.sin(_t*2.8));
  robotGroup.position.x={rx:.3f}+0.008*Math.sin(_t*0.7);  // subtle drift in demo
  robotGroup.position.y={ry:.3f}+0.008*Math.cos(_t*0.5);
  ringMesh.position.x=robotGroup.position.x;
  ringMesh.position.y=robotGroup.position.y;
  if(goalGroup.visible){{
    const gpp=0.5+0.5*Math.sin(_t*4.5);
    gR1.material.opacity=0.28+0.27*gpp;
    gR2.scale.setScalar(1.0+0.18*gpp);
    gR2.material.opacity=0.18*(1-gpp);
  }}
  controls.update(); renderer.render(scene,camera);
}})();
addEventListener('resize',()=>{{camera.aspect=innerWidth/innerHeight;camera.updateProjectionMatrix();renderer.setSize(innerWidth,innerHeight);}});
</script>
</body>
</html>"""

tmp = tempfile.NamedTemporaryFile(delete=False, suffix='.html', mode='w', encoding='utf-8')
tmp.write(html)
tmp.close()
print(f"Generated: {tmp.name}")
subprocess.Popen(['cmd', '/c', 'start', '', tmp.name])
