'use strict';
(function(){
const {W,H,IH,SC,C,clamp,M}=B;
const $=id=>document.getElementById(id);
const cv=$('cv'),ctx=cv.getContext('2d');cv.width=W*SC;cv.height=H*SC;
const iv=$('inner'),ic=iv.getContext('2d');iv.width=W*SC;iv.height=IH*SC;
const sim=B.sim={id:1,variant:'a',params:{},items:[],vehs:[],t:0,seed:4,rng:null,paused:false,speed:1,trails:true,labels:true,stat:{hits:0}};
const model=()=>M[sim.id];

/* ---------- state ---------- */
function loadModel(id,variant){
  sim.id=id;const m=model();
  sim.variant=m.variants.some(v=>v.k===variant)?variant:m.variants[0].k;
  sim.params={};m.params.forEach(p=>sim.params[p.id]=p.val);
  sim.speed=m.speed||1;$('speed').value=sim.speed;$('speedV').textContent=sim.speed+'×';
  buildUI();resetAll(false);syncURL();
}
// hash = vehicle + variant, ?seed= = start; both survive a reload and can be linked from a slide
function syncURL(withSeed){
  try{
    const u=new URL(location.href);
    if(withSeed||u.searchParams.has('seed'))u.searchParams.set('seed',sim.seed);
    u.hash='#'+sim.id+(sim.variant!==model().variants[0].k?sim.variant:'');
    history.replaceState(null,'',u);
  }catch(e){}
}
function resetAll(newSeed){
  if(newSeed){let s;do s=1+Math.floor(Math.random()*9999);while(s===sim.seed);sim.seed=s;syncURL(true)}
  $('seedV').textContent='run '+sim.seed;
  sim.rng=B.mulberry32(sim.seed*7919+sim.id*31);
  sim.t=0;sim.stat={hits:0};
  sim.items=model().items(sim.variant,sim).map(o=>({...o}));
  sim.vehs=[];model().init(sim);
}
function rebuildVehicles(){sim.rng=B.mulberry32(sim.seed*7919+sim.id*31);sim.stat={hits:0};sim.vehs=[];model().init(sim)}

/* ---------- ui ---------- */
const rail=$('rail');
Object.keys(M).forEach(k=>{const li=document.createElement('li');li.innerHTML=`<b>${k}</b><span>${M[k].name}</span>`;li.onclick=()=>loadModel(+k);rail.appendChild(li)});
function railIntoView(){
  const li=rail.children[sim.id-1];
  if(li&&rail.scrollWidth>rail.clientWidth+1)rail.scrollLeft=li.offsetLeft-(rail.clientWidth-li.offsetWidth)/2;
}
function buildUI(){
  const m=model();
  [...rail.children].forEach((li,i)=>li.classList.toggle('on',i+1===sim.id));
  railIntoView();
  $('vTitle').innerHTML=`Vehicle ${sim.id}<br><span>${m.name}</span>`;
  const vb=$('variants');vb.innerHTML='';
  if(m.variants.length>1)m.variants.forEach(v=>{const b=document.createElement('button');b.textContent=sim.id+v.k;b.className=v.k===sim.variant?'on':'';
    b.onclick=()=>{sim.variant=v.k;buildUI();resetAll(false);syncURL()};vb.appendChild(b)});
  const cur=m.variants.find(v=>v.k===sim.variant);
  $('vText').textContent=cur.text;$('vSummary').textContent=cur.summary||'';
  $('vNote').textContent=m.note||'';
  const box=$('controls');box.innerHTML='';
  m.params.forEach(p=>{
    if(p.show&&!p.show(sim.variant))return;
    const d=document.createElement('div');d.className='ctl';const fmt=p.fmt||(x=>x);
    d.innerHTML=`<div><label for="p_${p.id}">${p.label}</label><output>${fmt(sim.params[p.id])}</output></div><input id="p_${p.id}" type="range" min="${p.min}" max="${p.max}" step="${p.step}" value="${sim.params[p.id]}">`;
    const inp=d.querySelector('input'),out=d.querySelector('output');
    inp.oninput=()=>{sim.params[p.id]=+inp.value;out.textContent=fmt(+inp.value);if(p.reset)rebuildVehicles()};
    box.appendChild(d);
  });
  $('bAdd').style.display=$('bDel').style.display=m.addKind?'':'none';
  fit();
}
$('bPause').onclick=()=>{sim.paused=!sim.paused;$('bPause').textContent=sim.paused?'Play':'Pause';$('bPause').classList.toggle('on',sim.paused)};
$('bReset').onclick=()=>resetAll(false);
$('bShuffle').onclick=()=>resetAll(true);
$('bTrails').onclick=()=>{sim.trails=!sim.trails;$('bTrails').classList.toggle('on',sim.trails)};
$('speed').oninput=e=>{sim.speed=+e.target.value;$('speedV').textContent=sim.speed+'×'};
function addSource(x,y){
  const m=model();if(!m.addKind||sim.items.length>=10)return;
  const r=sim.rng;x=x??60+r()*(W-120);y=y??60+r()*(H-120);
  m.add?m.add(sim,x,y):sim.items.push({kind:m.addKind,x,y,power:1});
}
$('bAdd').onclick=()=>addSource();
$('bDel').onclick=()=>{if(sim.items.length>1)sim.items.pop()};
function download(blob,name){const a=document.createElement('a');a.href=URL.createObjectURL(blob);a.download=name;a.click();setTimeout(()=>URL.revokeObjectURL(a.href),2000)}
const fname=()=>`braitenberg-v${sim.id}${sim.variant}-run${sim.seed}`;
const both=()=>$('recMode').value!=='stage';
/* Stage and panel stacked on white, 1920 wide: stage 1920x1080, a thin rule, panel 1920x500. */
const RULE=4;
function composeSize(){return[cv.width,cv.height+RULE+Math.round(iv.height*cv.width/iv.width)]}
function compose(c){
  const x=c.getContext('2d'),w=c.width,ih=c.height-cv.height-RULE;
  x.fillStyle='#fff';x.fillRect(0,0,w,c.height);
  x.drawImage(cv,0,0,w,cv.height);
  x.fillStyle=C.ink;x.fillRect(0,cv.height+RULE/2-1,w,2);
  x.drawImage(iv,0,cv.height+RULE,w,ih);
}
function mkComposite(){const c=document.createElement('canvas');[c.width,c.height]=composeSize();return c}
$('bPng').onclick=()=>{
  if(!both()){cv.toBlob(b=>download(b,fname()+'-stage.png'));return}
  const c=mkComposite();compose(c);c.toBlob(b=>download(b,fname()+'.png'));
};
let rec=null,recC=null,recT0=0,recSec=-1;
function recLabel(){
  const n=Math.floor((performance.now()-recT0)/1000);if(n===recSec)return;recSec=n;
  $('bRec').textContent='Stop '+Math.floor(n/60)+':'+String(n%60).padStart(2,'0');
}
$('bRec').onclick=()=>{
  if(rec){rec.stop();return}
  if(!window.MediaRecorder||!cv.captureStream){alert('Recording needs Chrome, Edge or Firefox.');return}
  const mime=['video/webm;codecs=vp9','video/webm;codecs=vp8','video/webm','video/mp4'].find(t=>MediaRecorder.isTypeSupported(t))||'';
  const whole=both(),sfx=whole?'':'-stage';
  if(whole){recC=mkComposite();compose(recC)}
  const chunks=[],r=rec=new MediaRecorder((whole?recC:cv).captureStream(30),{mimeType:mime,videoBitsPerSecond:10e6});
  r.ondataavailable=e=>e.data.size&&chunks.push(e.data);
  r.onstop=()=>{
    const blob=new Blob(chunks,{type:mime});B.lastClip={size:blob.size,type:mime};
    if(blob.size<5000)alert('The clip is empty. Keep this tab visible while recording.');
    else download(blob,fname()+sfx+'.'+(mime.includes('mp4')?'mp4':'webm'));
    rec=recC=null;$('bRec').textContent='Record clip';$('bRec').classList.remove('on');$('recMode').disabled=false;
  };
  r.start(1000);recT0=performance.now();recSec=-1;recLabel();$('bRec').classList.add('on');$('recMode').disabled=true;
};
/* ---------- present mode: stage as large as the window allows, always 16:9 ---------- */
const bar=$('bar'),inh=document.querySelector('.inh'),sum=$('vSummary');
function fit(){
  const cl=document.body.classList,root=document.documentElement.style;
  if(!cl.contains('present')){root.removeProperty('--pw');root.removeProperty('--barh');return}
  const bh=bar.offsetHeight,inside=cl.contains('inside'),pad=12;
  const k=9/16+(inside?IH/W:0),extra=4+sum.offsetHeight+12+(inside?inh.offsetHeight+6:0);
  const pw=Math.floor(Math.max(200,Math.min(innerWidth-2*pad,(innerHeight-pad-bh-8-extra)/k)));
  root.setProperty('--barh',bh+'px');root.setProperty('--pw',pw+'px');
}
function setPresent(on){
  const cl=document.body.classList;cl.toggle('present',on);
  $('bPresent').textContent=on?'Exit present':'Present';$('bPresent').classList.toggle('on',on);fit();
}
function setInside(on){document.body.classList.toggle('inside',on);$('bInside').classList.toggle('on',on);fit()}
$('bPresent').onclick=()=>setPresent(!document.body.classList.contains('present'));
$('bInside').onclick=()=>setInside(!document.body.classList.contains('inside'));
addEventListener('resize',()=>{fit();railIntoView()});
if(window.ResizeObserver){const ro=new ResizeObserver(fit);ro.observe(bar);ro.observe(sum)}
addEventListener('keydown',e=>{
  if(e.ctrlKey||e.metaKey||e.altKey)return;
  const t=e.target;
  if(t.tagName==='SELECT'||t.tagName==='TEXTAREA'||(t.tagName==='INPUT'&&t.type!=='range'))return;
  if(t.tagName==='INPUT'&&e.key.startsWith('Arrow'))return;   // arrows belong to a focused slider
  const pres=document.body.classList.contains('present');
  if(e.key===' '){e.preventDefault();$('bPause').click()}
  else if(e.key==='r'||e.key==='R')resetAll(false);
  else if(e.key==='p'||e.key==='P')setPresent(!pres);
  else if(e.key==='Escape'&&pres)setPresent(false);
  else if((e.key==='i'||e.key==='I')&&pres)setInside(!document.body.classList.contains('inside'));
  else if(e.key==='ArrowRight'&&sim.id<14)loadModel(sim.id+1);
  else if(e.key==='ArrowLeft'&&sim.id>1)loadModel(sim.id-1);
});
// a focused button must not also fire on Space (Firefox clicks on keyup); sliders let go of focus after a drag
addEventListener('keyup',e=>{if(e.key===' '&&e.target.tagName!=='INPUT'&&e.target.tagName!=='SELECT')e.preventDefault()});
addEventListener('pointerup',e=>{if(e.target.type==='range')e.target.blur()});
// dragging sources
let drag=null;
const toWorld=e=>{const r=cv.getBoundingClientRect();return[(e.clientX-r.left)/r.width*W,(e.clientY-r.top)/r.height*H]};
const hit=(x,y)=>{let best=null,bd=26;for(const o of sim.items){if(o.fixed||o.gone)continue;const d=Math.hypot(o.x-x,o.y-y);if(d<bd){bd=d;best=o}}return best};
cv.addEventListener('pointerdown',e=>{const[x,y]=toWorld(e);const o=hit(x,y);if(o){drag=o;cv.setPointerCapture(e.pointerId)}});
cv.addEventListener('pointermove',e=>{const[x,y]=toWorld(e);if(drag){drag.x=clamp(x,12,W-12);drag.y=clamp(y,12,H-12);if(drag.grp!=null&&model().onDrag)model().onDrag(sim,drag)}cv.style.cursor=drag?'grabbing':hit(x,y)?'grab':'default'});
cv.addEventListener('pointerup',()=>drag=null);
cv.addEventListener('dblclick',e=>{const[x,y]=toWorld(e);if(!hit(x,y))addSource(x,y)});

/* ---------- rendering ---------- */
function draw(){
  const c=ctx,m=model();c.setTransform(SC,0,0,SC,0,0);
  c.fillStyle='#fff';c.fillRect(0,0,W,H);
  if(m.renderBack)m.renderBack(c,sim);
  for(const o of sim.items)if(o.kind==='heat')B.drawItem(c,o,sim);
  if(sim.trails)B.drawTrails(c,sim);
  for(const o of sim.items)if(o.kind!=='heat')B.drawItem(c,o,sim);
  if(m.render)m.render(c,sim);
  for(const v of sim.vehs)B.drawVeh(c,v);
  if(m.renderTop)m.renderTop(c,sim);
  c.textAlign='left';c.textBaseline='alphabetic';c.fillStyle=C.ink;c.font='600 14px system-ui';
  c.fillText(`Vehicle ${sim.id}${m.variants.length>1?sim.variant:''}`,14,24);
  const lines=m.hud?m.hud(sim):[];c.fillStyle=C.mute;c.font='12px system-ui';
  lines.forEach((l,i)=>c.fillText(l,14,H-12-(lines.length-1-i)*17));
  c.textAlign='right';c.fillText(sim.t.toFixed(0)+' s',W-12,H-12);
  // inner panel
  ic.setTransform(SC,0,0,SC,0,0);ic.fillStyle='#fff';ic.fillRect(0,0,W,IH);
  m.inner(ic,sim,W,IH);
  if(recC)compose(recC);
  if(rec)recLabel();
}
let last=performance.now(),acc=0;
function frame(now){
  const dt=Math.min(0.05,(now-last)/1000);last=now;
  if(!sim.paused){
    acc+=dt*sim.speed;let n=0;
    while(acc>=1/60&&n<400){sim.t+=1/60;model().tick(sim,1/60);acc-=1/60;n++}
    if(n>=400)acc=0;
  }
  draw();requestAnimationFrame(frame);
}
B.step=(secs)=>{for(let i=0;i<secs*60;i++){sim.t+=1/60;model().tick(sim,1/60)}};
B.load=loadModel;B.reset=resetAll;B.draw=draw;
/* ---------- boot ---------- */
const q=new URLSearchParams(location.search);
if(q.has('embed'))document.body.classList.add('embed');
if(q.has('seed')){const n=parseInt(q.get('seed'),10);if(Number.isFinite(n))sim.seed=n}
const parse=()=>{const h=(location.hash||'').match(/^#(\d{1,2})([a-z]?)/);return h&&M[h[1]]?[+h[1],h[2]||null]:[+(q.get('v')||1),q.get('var')]};
loadModel(...parse());
if(q.has('speed')){sim.speed=+q.get('speed');$('speed').value=sim.speed;$('speedV').textContent=sim.speed+'×'}
addEventListener('hashchange',()=>{const[id,v]=parse();if(id!==sim.id||(v&&v!==sim.variant))loadModel(id,v)});
if(q.has('present'))setPresent(true);
requestAnimationFrame(frame);
})();
