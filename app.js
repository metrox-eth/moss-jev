import {ReplayPlayer} from './player.mjs';
import * as T from 'three';
import {GLTFLoader} from './replay/assets/GLTFLoader.js';
const $=id=>document.getElementById(id);
let sceneName='can',modelReady=false,toastTimer,legacyName=null;
const scene=new T.Scene();scene.fog=new T.FogExp2(0x0c1520,.45);
let renderer;
try{renderer=new T.WebGLRenderer({antialias:true,alpha:true});}catch(e){$('phase').textContent='3D UNAVAILABLE';$('action').textContent='Enable browser graphics acceleration, then reload.';throw e;}
renderer.setPixelRatio(Math.min(devicePixelRatio,2));renderer.shadowMap.enabled=true;renderer.shadowMap.type=T.PCFSoftShadowMap;
renderer.outputColorSpace=T.SRGBColorSpace;renderer.toneMapping=T.ACESFilmicToneMapping;renderer.toneMappingExposure=1.2;
$('viewport').appendChild(renderer.domElement);
const camera=new T.PerspectiveCamera(38,1,.01,20);camera.up.set(0,0,1);
const center=new T.Vector3(.075,0,.15);let yaw=-.95,elevation=.6,distance=1.04;
function updateCamera(){camera.position.set(center.x+distance*Math.cos(yaw)*Math.cos(elevation),center.y+distance*Math.sin(yaw)*Math.cos(elevation),center.z+distance*Math.sin(elevation));camera.lookAt(center);}
updateCamera();
scene.add(new T.HemisphereLight(0xcbeaf4,0x243343,2.2));
const light=new T.DirectionalLight(0xffe2cf,3);light.position.set(.2,-.8,2);light.castShadow=true;
light.shadow.mapSize.set(2048,2048);Object.assign(light.shadow.camera,{left:-1,right:1,top:1,bottom:-1,near:.1,far:5});light.shadow.bias=-.0002;scene.add(light);
const rim=new T.DirectionalLight(0x6cf8c7,1.2);rim.position.set(-1,1,.6);scene.add(rim);
const mat=(c,metalness=.12)=>new T.MeshStandardMaterial({color:c,metalness,roughness:.55});
function mesh(geo,material,parent=scene){const m=new T.Mesh(geo,material);m.castShadow=true;m.receiveShadow=true;parent.add(m);return m;}
const ground=mesh(new T.BoxGeometry(1.7,1.3,.025),mat(0x172735));ground.position.set(.05,0,-.014);
const grid=new T.GridHelper(1.6,32,0x3b5563,0x263d4b);grid.rotation.x=Math.PI/2;grid.position.z=.0003;scene.add(grid);
const edge=new T.LineSegments(new T.EdgesGeometry(ground.geometry),new T.LineBasicMaterial({color:0x486575}));edge.position.copy(ground.position);scene.add(edge);
const nodes={},jawMeshes={left:[],right:[]},jawCenters={};const physicsMeshes={};
for(const [name,color,size] of [['palm',0x389985,[.11,.034,.016]],['gripper_mount',0x283d48,[.036,.034,.036]],['pad_left',0x55c9ad,[.008,.030,.048]],['pad_right',0x55c9ad,[.008,.030,.048]]])physicsMeshes[name]=mesh(new T.BoxGeometry(...size),mat(color));
const cg=new T.CylinderGeometry(.033,.033,.115,48);cg.rotateX(Math.PI/2);
physicsMeshes.can=mesh(cg,mat(0xff947e,.55));
for(const z of [-.0575,.0575]){const cap=mesh(new T.CylinderGeometry(.0333,.0333,.0015,48).rotateX(Math.PI/2),mat(0xc7d1d5,.8),physicsMeshes.can);cap.position.z=z;}
for(const z of [-.0575,.0575]){const rim=mesh(new T.TorusGeometry(.032,.0012,8,48),mat(0xc7d1d5,.8),physicsMeshes.can);rim.position.z=z;}
const tab=mesh(new T.TorusGeometry(.006,.0015,8,20),mat(0xc7d1d5,.8),physicsMeshes.can);tab.scale.y=1.5;tab.position.set(0,.003,.059);
const ring=mesh(new T.TorusGeometry(.048,.0012,6,48),new T.MeshBasicMaterial({color:0x6cf8c7}));ring.position.set(.315,0,.0016);
const tracks=new T.Group(),roverVisual=new T.Group();scene.add(tracks,roverVisual);const treadMeshes=[];
for(const side of [-1,1])for(let i=0;i<68;i++){
 const t=mesh(new T.BoxGeometry(.0085,.036,.008),mat(0x2e3c44),tracks);
 treadMeshes.push({mesh:t,side,index:i});
}
function moveTracks(x){
 tracks.position.x=x;roverVisual.position.x=x;
 const L=.5+2*Math.PI*.036;
 for(const {mesh:t,side,index} of treadMeshes){
 const s=((index/68*L-x)%L+L)%L;let xx,z,a;
 if(s<.25){xx=-.125+s;z=.036;a=0;}else if(s<.25+Math.PI*.036){a=Math.PI/2-(s-.25)/.036;xx=.125+.036*Math.cos(a);z=.036*Math.sin(a);a-=Math.PI/2;}
 else if(s<.5+Math.PI*.036){xx=.125-(s-.25-Math.PI*.036);z=-.036;a=0;}
 else{a=-Math.PI/2-(s-.5-Math.PI*.036)/.036;xx=-.125+.036*Math.cos(a);z=.036*Math.sin(a);a-=Math.PI/2;}
 t.position.set(xx,side*.121,z+.041);t.rotation.y=-a;
 }
}
moveTracks(0);
new GLTFLoader().load('./replay/assets/moss.glb',g=>{
 g.scene.traverse(o=>{
  if(o.name.startsWith('arm_'))nodes[o.name.slice(4)]=o;
 if(['grip_133','grip_159'].includes(o.name))jawMeshes.left.push(o);
 if(['grip_134','grip_160'].includes(o.name))jawMeshes.right.push(o);
 if(o.name==='grip_159'||o.name==='grip_160'){
  o.geometry.computeBoundingBox();jawCenters[o.name==='grip_159'?'left':'right']=o.geometry.boundingBox.getCenter(new T.Vector3());
 }
  if(o.isMesh){o.castShadow=true;o.receiveShadow=true;}
 });scene.add(g.scene);for(const child of [...g.scene.children])if(!child.name.startsWith('arm_'))roverVisual.attach(child);modelReady=true;
 for(const n of ['palm','pad_left','pad_right','gripper_mount'])physicsMeshes[n].visible=false;
},undefined,()=>toast('The CAD model could not load. Refresh to retry.'));

const player=new ReplayPlayer();let loading=false,loadVersion=0,fetchController=null,loadError='',lastUIKey='',lastTime=performance.now();
const cache=new Map();
const labels={reposition:'Move closer',approach:'Approach the can',align:'Align the gripper',close:'Grasp',lift:'Lift',retry:'Try again',clear:'Clear the rim',carry:'Move over the bin',release:'Let go',home:'Return the arm',stop:'Collected'};
const descriptions={can:'Pick up a full-size can and drop it into the bin.',miss:'The gripper starts misaligned. Jev uses feedback to adjust and try again.',far:'The can is too far away. MOSS drives closer, then picks it up.'};
function toast(message){$('toast').textContent=message;$('toast').hidden=false;clearTimeout(toastTimer);toastTimer=setTimeout(()=>$('toast').hidden=true,5000);}
function controls(){
 const ready=!!player.data&&modelReady&&!loading&&!legacyName;
 $('run').disabled=!ready||player.running;$('pause').disabled=!ready||!player.running;$('stop').disabled=!ready||player.time===0;
 $('reset').disabled=!ready;$('seek').disabled=!ready;
 $('run').textContent=loading?'LOADING…':player.running?'PLAYING…':player.time>0&&player.time<player.data?.duration?'RESUME →':'PLAY REPLAY →';
 const duration=player.data?.duration||0;
 $('seek').value=duration?Math.round(player.time/duration*1000):0;
 $('seek').setAttribute('aria-valuetext',`${player.time.toFixed(1)} of ${duration.toFixed(1)} seconds`);
 $('playback-time').textContent=`${player.time.toFixed(1)} / ${duration.toFixed(1)} s`;
}
function updateUI(sample){
 controls();if(!sample||legacyName)return;
 const o=sample.observation,d=player.data.decisions[sample.decision_index];
 const key=`${sample.decision_index}/${player.running}/${o.finger_contacts}/${o.settled_in_bin}/${o.moving}/${o.reposition_notice}/${loading}/${loadError}`;
 $('reach-modal').hidden=!o.reposition_notice||(!player.running&&player.time===0);
 if(key===lastUIKey)return;lastUIKey=key;
 $('phase').textContent=loading?'LOADING SCENARIO':loadError?'REPLAY UNAVAILABLE':o.settled_in_bin?'COLLECTED':player.running?'REPLAY · NO API CALLS':player.time>0?'PAUSED':'READY TO REPLAY';
 $('action').textContent=loading?'Loading recorded motion…':o.settled_in_bin?'The can is safely in the bin.':d?(d.action==='reposition'?'MOSS is repositioning':labels[d.action]||d.action):'Choose a scenario and press Play replay.';
 $('contacts').textContent=`${o.finger_contacts} / 2`;
 $('calls').textContent=`${Math.max(0,sample.decision_index+1)} recorded decisions`;
 $('decision').textContent=d?labels[d.action]||d.action:'Your call. Its next move.';
 $('latency').textContent=d?`${d.latency_ms} ms`:'—';
 $('probabilities').replaceChildren(...Object.entries(d?.probabilities||{}).sort((a,b)=>b[1]-a[1]).slice(0,4).map(([name,value])=>{
   const row=document.createElement('div');row.className='prob';const label=document.createElement('span');label.textContent=labels[name]||name;
   const bar=document.createElement('div');bar.className='bar';const fill=document.createElement('i');fill.style.width=`${Math.max(0,Math.min(100,Number(value)*100))}%`;bar.append(fill);
   const number=document.createElement('b');number.textContent=`${Math.round(value*100)}%`;row.append(label,bar,number);return row;
 }));
 const recent=player.data.decisions.slice(0,sample.decision_index+1).slice(-4).reverse().map(d=>'Jev → '+(labels[d.action]||d.action));
 if(o.settled_in_bin)recent.unshift('Collected and settled in the bin.');
 $('events').replaceChildren(...(recent.length?recent:['Recorded run ready. No API calls.']).map(message=>{const li=document.createElement('li');li.textContent=message;li.className='jev';return li;}));
}
async function selectModern(name){
 const version=++loadVersion;fetchController?.abort();fetchController=new AbortController();player.reset();player.data=null;loading=true;loadError='';lastUIKey='';legacyName=null;sceneName=name;
 document.body.classList.remove('legacy-view');$('legacy-frame').hidden=true;$('legacy-frame').src='about:blank';
 $('version-label').textContent='V0.2 · PHYSICS REPLAY';$('scenario-description').textContent=descriptions[name];$('reach-modal').hidden=true;$('error').hidden=true;
 $('phase').textContent='LOADING SCENARIO';$('action').textContent='Loading recorded motion…';
 document.querySelectorAll('[data-scene]').forEach(b=>{b.classList.toggle('selected',b.dataset.scene===name);b.setAttribute('aria-pressed',String(b.dataset.scene===name));});
 document.querySelectorAll('[data-legacy]').forEach(b=>{b.classList.remove('selected');b.setAttribute('aria-pressed','false');});
 distance=name==='far'?1.3:1.04;center.x=name==='far'?.23:.075;updateCamera();controls();
 try{
  let data=cache.get(name);
  if(!data){const response=await fetch(new URL(`./recordings/${name}.json`,import.meta.url),{signal:fetchController.signal});if(!response.ok)throw new Error(`Recording unavailable (${response.status})`);data=await response.json();cache.set(name,data);}
  if(version!==loadVersion)return;
  player.load(data);loading=false;updateUI(player.pair().a);
 }catch(error){
  if(version!==loadVersion||error.name==='AbortError')return;
  loading=false;loadError=error.message;$('phase').textContent='REPLAY UNAVAILABLE';$('action').textContent='Select the scenario again to retry.';$('error').hidden=false;$('error-title').textContent='COULD NOT LOAD REPLAY';$('error-text').textContent=error.message;controls();
 }
}
$('run').onclick=()=>{player.play();lastTime=performance.now();lastUIKey='';};
$('pause').onclick=()=>player.pause();
$('stop').onclick=$('reset').onclick=()=>{player.reset();lastUIKey='';};
$('seek').oninput=()=>{player.seek(Number($('seek').value)/1000*player.data.duration);lastUIKey='';};
document.querySelectorAll('[data-scene]').forEach(b=>b.onclick=()=>selectModern(b.dataset.scene));
document.querySelectorAll('[data-legacy]').forEach(b=>b.onclick=()=>{
 ++loadVersion;fetchController?.abort();loading=false;player.pause();legacyName=b.dataset.legacy;
 document.body.classList.add('legacy-view');$('legacy-frame').hidden=false;$('legacy-frame').src=`./replay/?embed=1&mission=${encodeURIComponent(legacyName)}`;
 $('version-label').textContent='V0.1 · RECORDED DEMO';$('scenario-description').textContent='The original demos: recorded Jev target choices and scripted motion.';
 document.querySelectorAll('[data-legacy]').forEach(x=>{x.classList.toggle('selected',x===b);x.setAttribute('aria-pressed',String(x===b));});
 document.querySelectorAll('[data-scene]').forEach(x=>{x.classList.remove('selected');x.setAttribute('aria-pressed','false');});
 $('reach-modal').hidden=true;controls();
});
function zoom(f){distance=T.MathUtils.clamp(distance*f,.48,1.9);updateCamera();}
$('zoom-in').onclick=()=>zoom(.88);$('zoom-out').onclick=()=>zoom(1.14);$('camera-reset').onclick=()=>{yaw=-.95;elevation=.6;distance=sceneName==='far'?1.3:1.04;updateCamera();};
const pointers=new Map();let gesture=null;
function pointState(){const p=[...pointers.values()];return p.length>1?{x:(p[0].x+p[1].x)/2,y:(p[0].y+p[1].y)/2,d:Math.hypot(p[0].x-p[1].x,p[0].y-p[1].y)}:p[0]?{...p[0],d:0}:null;}
renderer.domElement.onpointerdown=e=>{pointers.set(e.pointerId,{x:e.clientX,y:e.clientY});gesture=pointState();renderer.domElement.setPointerCapture(e.pointerId);};
renderer.domElement.onpointermove=e=>{
 if(!pointers.has(e.pointerId))return;pointers.set(e.pointerId,{x:e.clientX,y:e.clientY});const next=pointState();
 if(gesture){if(next.d&&gesture.d)zoom(gesture.d/next.d);else{yaw-=(next.x-gesture.x)*.006;elevation=T.MathUtils.clamp(elevation+(next.y-gesture.y)*.005,.12,1.35);updateCamera();}}
 gesture=next;
};
renderer.domElement.onpointerup=renderer.domElement.onpointercancel=e=>{pointers.delete(e.pointerId);gesture=pointState();};
renderer.domElement.addEventListener('wheel',e=>{e.preventDefault();zoom(Math.exp(e.deltaY*.001));},{passive:false});
new ResizeObserver(()=>{const r=$('viewport').getBoundingClientRect();if(r.width>0&&r.height>0){camera.aspect=r.width/r.height;camera.updateProjectionMatrix();renderer.setSize(r.width,r.height);}}).observe($('viewport'));
const m4=new T.Matrix4(),q1=new T.Quaternion(),q2=new T.Quaternion(),p1=new T.Vector3(),p2=new T.Vector3();
function quat(wxyz){return new T.Quaternion(wxyz[1],wxyz[2],wxyz[3],wxyz[0]);}
function drawPair({a,b,t}){
 for(const [name,pose] of Object.entries(b.transforms.bodies)){
  const node=nodes[name];if(!node)continue;const prev=a.transforms.bodies[name]||pose;
  node.matrixAutoUpdate=true;node.position.fromArray(prev).lerp(p2.fromArray(pose),t);node.quaternion.copy(quat(prev.slice(3))).slerp(quat(pose.slice(3)),t);
 }
 for(const [name,g] of Object.entries(b.transforms.geoms)){
  const node=physicsMeshes[name],prev=a.transforms.geoms[name]||g;
  p1.fromArray(prev.p);p2.fromArray(g.p);node.position.copy(p1).lerp(p2,t);
  const rotation=r=>m4.set(r[0],r[1],r[2],0,r[3],r[4],r[5],0,r[6],r[7],r[8],0,0,0,0,1);
  q1.setFromRotationMatrix(rotation(prev.r));q2.setFromRotationMatrix(rotation(g.r));node.quaternion.copy(q1).slerp(q2,t);
 }
 moveTracks(T.MathUtils.lerp(a.observation.base_x_m,b.observation.base_x_m,t));
 const frame=nodes.gripper_frame_link;
 if(frame)for(const side of ['left','right']){
  const target=physicsMeshes['pad_'+side].position.clone().sub(frame.position).applyQuaternion(frame.quaternion.clone().invert());
  if(jawCenters[side])for(const jaw of jawMeshes[side])jaw.position.copy(target).sub(jawCenters[side]);
 }
 ring.visible=!a.observation.held&&!a.observation.in_bin;ring.position.set(physicsMeshes.can.position.x,physicsMeshes.can.position.y,.0016);
}
document.addEventListener('visibilitychange',()=>{if(document.hidden)player.pause();lastTime=performance.now();});
function render(now){
 requestAnimationFrame(render);const dt=Math.min(.1,Math.max(0,(now-lastTime)/1000));lastTime=now;
 if(legacyName)return;
 player.advance(dt);const pair=player.pair();if(pair){drawPair(pair);updateUI(pair.a);}else controls();
 renderer.render(scene,camera);
}
requestAnimationFrame(render);selectModern('can');
