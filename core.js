'use strict';
/* Core: physics, sensing, drawing helpers. Models register themselves in B.M[n]. */
const B=window.B={M:{}};
B.W=960;B.H=540;B.IH=250;B.SC=2;
const TAU=B.TAU=Math.PI*2, VMAX=95, TRACK=28;
const C=B.C={ink:'#1c1c1a',mute:'#8b8a85',rule:'#d9d6cd',red:'#c4301c',blue:'#2a5d9f',olive:'#5b6b2e',paper:'#fbfaf6',faint:'#eeece5'};
const clamp=B.clamp=(x,a=0,b=1)=>x<a?a:x>b?b:x;
B.rad=d=>d*Math.PI/180;
B.wrapA=a=>{a%=TAU;if(a>Math.PI)a-=TAU;if(a<-Math.PI)a+=TAU;return a};
B.mulberry32=function(a){return function(){a|=0;a=a+0x6D2B79F5|0;let t=Math.imul(a^a>>>15,1|a);t=t+Math.imul(t^t>>>7,61|t)^t;return((t^t>>>14)>>>0)/4294967296}};
B.gauss=r=>{let u=0;while(!u)u=r();const v=r();return Math.sqrt(-2*Math.log(u))*Math.cos(TAU*v)};
const W=B.W,H=B.H;

/* ---------- sensing ---------- */
// Stimulus of one kind at a point. Falls off as 1/(1+(d/R)^2). sharp>0: cos^sharp cone, sharp<0: cardioid, 0: omni.
B.sense=function(px,py,dir,kind,sim,sharp=1,R=120){
  let s=0;
  for(const o of sim.items){
    if(o.kind!==kind||o.off||o.gone)continue;
    const dx=o.x-px,dy=o.y-py,d2=dx*dx+dy*dy,Rr=o.R||R;
    let f=1;
    if(sharp>0){const c=Math.cos(Math.atan2(dy,dx)-dir);f=c>0?Math.pow(c,sharp):0}
    else if(sharp<0){const c=Math.cos(Math.atan2(dy,dx)-dir);f=Math.pow((1+c)/2,-sharp)}
    s+=(o.power||1)*f/(1+d2/(Rr*Rr));
  }
  return s;
};
// local body coords -> world
B.toWorld=function(v,lx,ly){const c=Math.cos(v.th),s=Math.sin(v.th);return[v.x+c*lx-s*ly,v.y+s*lx+c*ly]};
// left/right sensor pair at the front, each tilted by +-phi from the heading (left = -y on screen)
B.sens2=function(v,sim,kind,phi,sharp=1,R=120){
  const out=[];
  for(const side of[-1,1]){const [x,y]=B.toWorld(v,22,side*10);out.push(B.sense(x,y,v.th+side*phi,kind,sim,sharp,R))}
  return out;
};
B.sens1=function(v,sim,kind,sharp=0,R=120){const [x,y]=B.toWorld(v,22,0);return B.sense(x,y,v.th,kind,sim,sharp,R)};

/* ---------- vehicle ---------- */
B.mkVeh=(x,y,th)=>({x,y,th,vl:0,vr:0,mL:0,mR:0,sL:0,sR:0,age:0,trail:[],ta:0,cool:0,wires:null,single:false,phi:0.7});
// Motor outputs mL,mR in 0..1. Faster left wheel turns the vehicle to the right (clockwise on screen).
B.drive=function(v,mL,mR,dt){
  v.mL=mL;v.mR=mR;
  const sp=(mL+mR)/2*VMAX,om=(mL-mR)*VMAX/TRACK;
  v.th+=om*dt;v.x+=Math.cos(v.th)*sp*dt;v.y+=Math.sin(v.th)*sp*dt;
};
// Walls. m is the hard margin (the vehicle centre never goes closer). Inside a soft zone of WALL_ZONE px in front of it
// the heading bends away from the wall, more the deeper the vehicle is: a curve whose radius shrinks from infinity at the edge of the zone
// to 1/WALL_K px at the margin. The bend is per distance driven, so fast and slow vehicles follow the same curve and a vehicle at rest does not turn;
// below WALL_V px/s the distance counts a little more (sqrt law), so a crawling vehicle still gets away from the wall.
// Only if that was not enough (the vehicle reaches the margin) is the position clamped and, if it still points into the wall, the heading reflected.
const WALL_ZONE=45,WALL_K=1/20,WALL_V=40;
B.bounce=function(v,m=22){
  const Z=WALL_ZONE;
  const nx=clamp((m+Z-v.x)/Z)-clamp((m+Z-(W-v.x))/Z),ny=clamp((m+Z-v.y)/Z)-clamp((m+Z-(H-v.y))/Z),s=Math.min(1,Math.hypot(nx,ny));   // s: penetration 0..1, (nx,ny): direction away from the wall(s)
  if(s>0){
    const sp=Math.abs((v.mL+v.mR)/2||0)*VMAX;   // speed in px/s
    v.th+=WALL_K*s*(sp>=WALL_V?sp:Math.sqrt(WALL_V*sp))/60*clamp(B.wrapA(Math.atan2(ny,nx)-v.th)*1.5,-1,1);
  }
  if(v.x<m){v.x=m;if(Math.cos(v.th)<0)v.th=Math.PI-v.th}
  if(v.x>W-m){v.x=W-m;if(Math.cos(v.th)>0)v.th=Math.PI-v.th}
  if(v.y<m){v.y=m;if(Math.sin(v.th)<0)v.th=-v.th}
  if(v.y>H-m){v.y=H-m;if(Math.sin(v.th)>0)v.th=-v.th}
};
B.trail=function(v,dt){v.ta+=dt;if(v.ta>0.05){v.ta=0;v.trail.push(v.x,v.y);if(v.trail.length>500)v.trail.splice(0,2)}};
B.touch=function(sim,v,kind,r=20){
  v.cool-=1/60;let hit=null;
  for(const o of sim.items){if(o.kind!==kind||o.off||o.gone)continue;
    if(Math.hypot(o.x-v.x,o.y-v.y)<r&&v.cool<=0){sim.stat.hits++;v.cool=1.2;o.flash=0.5;hit=o}}
  return hit;
};
B.spawn=function(sim,n){const r=sim.rng,a=[];for(let i=0;i<n;i++)a.push(B.mkVeh(60+r()*(W-120),60+r()*(H-120),r()*TAU));return a};

/* ---------- drawing: stage ---------- */
const KIND={light:'light',heat:'heat',oxy:'oxygen',org:'organic matter',tone:'tone'};
B.drawItem=function(c,o,sim){
  if(o.gone)return;
  const lab=sim.labels&&KIND[o.kind]&&!o.nolabel;
  if(o.kind==='heat'){
    const R=o.R||120;c.strokeStyle='rgba(196,48,28,.45)';c.lineWidth=1.2;c.setLineDash([5,5]);
    for(const k of[1,2]){c.beginPath();c.arc(o.x,o.y,R*k,0,TAU);c.stroke()}
    c.setLineDash([]);c.fillStyle=C.red;c.beginPath();c.arc(o.x,o.y,5,0,TAU);c.fill();
    if(o.zone){c.strokeStyle=C.red;c.lineWidth=1.6;c.setLineDash([2,4]);c.beginPath();c.arc(o.x,o.y,o.zone,0,TAU);c.stroke();c.setLineDash([])}
    if(lab){c.fillStyle=C.red;c.font='12px system-ui';c.textAlign='center';c.fillText('heat',o.x,o.y+16)}
  }else if(o.kind==='light'){
    const on=!o.off;
    c.strokeStyle=C.ink;c.lineWidth=1.4;c.setLineDash(on?[]:[2,3]);
    for(let i=0;i<8;i++){const a=i*TAU/8;c.beginPath();c.moveTo(o.x+Math.cos(a)*9,o.y+Math.sin(a)*9);c.lineTo(o.x+Math.cos(a)*14,o.y+Math.sin(a)*14);c.stroke()}
    c.fillStyle=on?C.ink:'#fff';c.beginPath();c.arc(o.x,o.y,6,0,TAU);c.fill();c.stroke();c.setLineDash([]);
    if(o.flash>0){c.strokeStyle=C.red;c.lineWidth=1.5;c.beginPath();c.arc(o.x,o.y,14+o.flash*50,0,TAU);c.stroke()}
    if(lab&&on){c.fillStyle=C.mute;c.font='12px system-ui';c.textAlign='center';c.fillText('light',o.x,o.y+27)}
  }else if(o.kind==='oxy'){
    c.fillStyle='#fff';c.strokeStyle=C.blue;c.lineWidth=1.5;c.beginPath();c.arc(o.x,o.y,9,0,TAU);c.fill();c.stroke();
    c.fillStyle=C.blue;c.font='10px system-ui';c.textAlign='center';c.textBaseline='middle';c.fillText('O₂',o.x,o.y+.5);c.textBaseline='alphabetic';
    if(lab){c.font='12px system-ui';c.fillText('oxygen',o.x,o.y+22)}
  }else if(o.kind==='org'){
    c.fillStyle=C.olive;for(const[dx,dy]of[[-5,-3],[4,-4],[0,4],[7,3],[-6,5]]){c.beginPath();c.arc(o.x+dx,o.y+dy,3,0,TAU);c.fill()}
    if(o.flash>0){c.strokeStyle=C.red;c.lineWidth=1.5;c.beginPath();c.arc(o.x,o.y,12+o.flash*40,0,TAU);c.stroke()}
    if(lab){c.font='12px system-ui';c.textAlign='center';c.fillText('organic',o.x,o.y+22)}
  }else if(o.kind==='tone'){
    const on=!o.off;c.strokeStyle=on?C.ink:C.rule;c.lineWidth=1.6;
    for(const k of[10,17,24]){c.beginPath();c.arc(o.x,o.y,k,-0.9,0.9);c.stroke()}
    c.fillStyle=on?C.ink:C.rule;c.fillRect(o.x-9,o.y-4,8,8);
    if(o.name){c.fillStyle=C.mute;c.font='12px system-ui';c.textAlign='center';c.fillText(o.name,o.x,o.y+38)}
  }
  if(o.flash>0)o.flash=Math.max(0,o.flash-1/60);
};
B.drawVeh=function(c,v){
  c.save();c.translate(v.x,v.y);c.rotate(v.th);if(v.sc)c.scale(v.sc,v.sc);
  const A=a=>clamp(a);
  // wires drawn inside the body: solid = excitatory, dashed = inhibitory, red = active
  if(v.wires){
    for(const e of v.wires){
      const y0=v.single?0:(e.a==='L'?-10:10),y1=v.single?0:(e.b==='L'?-12:12);
      c.strokeStyle=A(e.act)>0.12?C.red:C.mute;c.lineWidth=1+A(e.act)*2.2;c.setLineDash(e.sign<0?[3,3]:[]);
      c.beginPath();c.moveTo(18,y0);c.lineTo(-13,y1);c.stroke();c.setLineDash([]);
    }
  }
  for(const s of[-1,1]){
    const m=v.single?(v.mL+v.mR)/2:(s<0?v.mL:v.mR);
    c.fillStyle=C.ink;c.globalAlpha=.3+.7*A(m);c.fillRect(-21,s>0?13:-18,13,5);c.globalAlpha=1;
  }
  c.fillStyle=v.fill||'#fff';c.strokeStyle=C.ink;c.lineWidth=1.7;c.beginPath();c.roundRect(-18,-13,40,26,5);c.fill();c.stroke();
  // redraw wires on top of the white body so they stay visible
  if(v.wires){
    for(const e of v.wires){
      const y0=v.single?0:(e.a==='L'?-10:10),y1=v.single?0:(e.b==='L'?-12:12);
      c.strokeStyle=A(e.act)>0.12?C.red:'#9a9993';c.lineWidth=1+A(e.act)*2.2;c.setLineDash(e.sign<0?[3,3]:[]);
      c.beginPath();c.moveTo(19,y0);c.lineTo(-13,y1);c.stroke();c.setLineDash([]);
    }
  }
  const sens=v.single?[[0,v.sL]]:[[-1,v.sL],[1,v.sR]];
  for(const[s,val]of sens){
    c.strokeStyle=C.ink;c.lineWidth=1.3;
    if(!v.single){c.beginPath();c.moveTo(22,s*10);c.lineTo(22+Math.cos(s*v.phi)*8,s*10+Math.sin(s*v.phi)*8);c.stroke()}
    c.fillStyle='#fff';c.beginPath();c.arc(22,s*10,3.6,0,TAU);c.fill();c.stroke();
    c.globalAlpha=A(val);c.fillStyle=C.red;c.beginPath();c.arc(22,s*10,3.6,0,TAU);c.fill();c.globalAlpha=1;
  }
  c.restore();
};
B.drawTrails=function(c,sim){
  c.lineWidth=1.3;
  for(const v of sim.vehs){const t=v.trail,n=t.length/2;if(n<3)continue;
    for(let q=0;q<3;q++){const i0=Math.floor(n*q/3),i1=Math.min(n-1,Math.floor(n*(q+1)/3)+1);
      c.strokeStyle=v.tc||C.ink;c.globalAlpha=.12+.16*q;c.beginPath();c.moveTo(t[2*i0],t[2*i0+1]);for(let i=i0+1;i<=i1;i++)c.lineTo(t[2*i],t[2*i+1]);c.stroke()}
    c.globalAlpha=1}
};

/* ---------- drawing: diagrams for the inner panel ---------- */
const nodeShape=(c,n,x,y,R,hw)=>{
  c.beginPath();
  if(n.shape==='unit'){c.moveTo(x,y-R);c.lineTo(x+R,y);c.lineTo(x,y+R);c.lineTo(x-R,y);c.closePath()}
  else if(n.shape==='out')c.rect(x-hw,y-R*.85,hw*2,R*1.7);   // hw: half width, wider than the square when the label needs it
  else c.arc(x,y,R,0,TAU);
};
// graph: nodes {id,x,y,label,act,shape:'in'|'unit'|'out',sub}, edges {a,b,sign,act,w,type:'syn'|'mnem'|'ergo',curve,label}
B.graph=function(c,x0,y0,w,h,g){
  const by={};g.nodes.forEach(n=>by[n.id]=n);const R=g.R||15;
  const P=n=>[x0+n.x*w,y0+n.y*h];
  c.lineCap='round';c.textBaseline='middle';
  for(const e of g.edges){
    const a=by[e.a],b=by[e.b];if(!a||!b)continue;
    let[ax,ay]=P(a),[bx,by2]=P(b);const act=clamp(e.act||0),wt=e.w==null?1:Math.min(1,Math.abs(e.w));
    const col=act>0.1?C.red:C.mute,lw=e.type==='syn'||!e.type?1.4+wt*1.6:0.8+wt*4;
    c.strokeStyle=col;c.fillStyle=col;c.setLineDash(e.sign<0?[4,3]:[]);c.lineWidth=lw;
    if(a===b){c.beginPath();c.arc(ax,ay-R-8,9,0.7*Math.PI,2.3*Math.PI);c.stroke();c.setLineDash([]);continue}
    const dx=bx-ax,dy=by2-ay,d=Math.hypot(dx,dy),ux=dx/d,uy=dy/d,nx=-uy,ny=ux,cu=e.curve||0;
    const sx=ax+ux*(R+1),sy=ay+uy*(R+1),ex=bx-ux*(R+3),ey=by2-uy*(R+3);
    const mx=(sx+ex)/2+nx*cu,my=(sy+ey)/2+ny*cu;
    c.beginPath();c.moveTo(sx,sy);
    if(e.type==='mnem'){ // two parallel strands, no arrowhead: connects both ways
      c.lineTo(ex,ey);c.stroke();c.setLineDash([]);c.strokeStyle='#fff';c.lineWidth=Math.max(0.5,lw-2.2);c.beginPath();c.moveTo(sx,sy);c.lineTo(ex,ey);c.stroke();
    }else{
      if(cu)c.quadraticCurveTo(mx,my,ex,ey);else c.lineTo(ex,ey);c.stroke();c.setLineDash([]);
      const ang=cu?Math.atan2(ey-my,ex-mx):Math.atan2(ey-sy,ex-sx);
      c.beginPath();c.moveTo(ex+Math.cos(ang)*3,ey+Math.sin(ang)*3);c.lineTo(ex-Math.cos(ang-.4)*9,ey-Math.sin(ang-.4)*9);c.lineTo(ex-Math.cos(ang+.4)*9,ey-Math.sin(ang+.4)*9);c.closePath();c.fill();
    }
    c.setLineDash([]);
    const lx=(sx+ex)/2+nx*(cu/2),ly=(sy+ey)/2+ny*(cu/2);
    const txt=e.label!=null?e.label:(e.sign?(e.sign>0?'+':'−'):'');
    if(txt!==''){c.font=(e.label!=null?'13':'bold 15')+'px system-ui';const bw=Math.max(24,c.measureText(txt).width+8);c.fillStyle='#fff';c.fillRect(lx-bw/2,ly-8,bw,16);c.fillStyle=col;c.textAlign='center';c.fillText(txt,lx,ly+.5)}
  }
  for(const n of g.nodes){
    const[x,y]=P(n);const act=clamp(n.act||0);
    c.font='600 14px system-ui';const hw=Math.max(R*.85,c.measureText(n.label||'').width/2+4);
    c.fillStyle='#fff';c.strokeStyle=C.ink;c.lineWidth=1.6;nodeShape(c,n,x,y,R,hw);c.fill();
    c.globalAlpha=act;c.fillStyle=C.red;nodeShape(c,n,x,y,R,hw);c.fill();c.globalAlpha=1;nodeShape(c,n,x,y,R,hw);c.stroke();
    c.fillStyle=act>0.55?'#fff':C.ink;c.textAlign='center';c.fillText(n.label||'',x,y+.5);
    if(n.sub){c.fillStyle=C.mute;c.font='13px system-ui';c.textBaseline='alphabetic';c.fillText(n.sub,x,y+R+15);c.textBaseline='middle'}
  }
  if(g.caption){c.fillStyle=C.mute;c.font='13px system-ui';c.textAlign='left';c.textBaseline='alphabetic';c.fillText(g.caption,x0,y0+h+2)}
  c.textBaseline='alphabetic';
};
B.curve=function(c,x0,y0,w,h,f,marks,xl,yl){
  c.strokeStyle=C.ink;c.lineWidth=1;c.beginPath();c.moveTo(x0,y0);c.lineTo(x0,y0+h);c.lineTo(x0+w,y0+h);c.stroke();
  c.lineWidth=2;c.beginPath();for(let i=0;i<=100;i++){const s=i/100,x=x0+s*w,y=y0+h-clamp(f(s))*h;i?c.lineTo(x,y):c.moveTo(x,y)}c.stroke();
  for(const m of marks||[]){const s=clamp(m),x=x0+s*w,y=y0+h-clamp(f(s))*h;c.strokeStyle=C.red;c.fillStyle=C.red;c.lineWidth=1;c.setLineDash([3,3]);c.beginPath();c.moveTo(x,y0+h);c.lineTo(x,y);c.stroke();c.setLineDash([]);c.beginPath();c.arc(x,y,3.5,0,TAU);c.fill()}
  c.fillStyle=C.mute;c.font='13px system-ui';c.textAlign='right';c.fillText(xl||'stimulus',x0+w,y0+h+16);c.textAlign='left';c.fillText(yl||'motor speed',x0,y0-6);
};
B.bars=function(c,x0,y0,w,h,arr,col,label,mx=1,marks){
  const n=arr.length,bw=w/n;c.fillStyle=C.faint;c.fillRect(x0,y0,w,h);
  c.fillStyle=col||C.ink;for(let k=0;k<n;k++){const v=clamp(arr[k]/mx)*h;if(v>0.5)c.fillRect(x0+k*bw,y0+h-v,Math.max(1,bw-1),v)}
  c.fillStyle=C.mute;c.font='13px system-ui';c.textAlign='left';c.fillText(label,x0,y0-4);
  if(marks)for(const m of marks){c.strokeStyle=C.red;c.lineWidth=1.5;const x=x0+(m+.5)*bw;c.beginPath();c.moveTo(x,y0);c.lineTo(x,y0+h);c.stroke()}
};
B.text=function(c,s,x,y,col,size=13,align='left',font='system-ui'){c.fillStyle=col||C.mute;c.font=size+'px '+font;c.textAlign=align;c.fillText(s,x,y)};
