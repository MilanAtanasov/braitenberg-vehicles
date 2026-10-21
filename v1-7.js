'use strict';
(function(){
const {W,H,C,clamp,rad,gauss,M,TAU}=B;
const nP=(mx,v)=>({id:'n',label:'Vehicles',min:1,max:mx,step:1,val:v,reset:true});
const phiP=v=>({id:'phi',label:'Sensor angle',min:10,max:80,step:1,val:v,fmt:x=>x+'°'});
const gainP=(v,mx=5)=>({id:'gain',label:'Gain',min:0.5,max:mx,step:0.1,val:v,fmt:x=>x.toFixed(1)});
const pairWires=(v,cross,sign)=>cross?[{a:'L',b:'R',sign,act:v.sL},{a:'R',b:'L',sign,act:v.sR}]:[{a:'L',b:'L',sign,act:v.sL},{a:'R',b:'R',sign,act:v.sR}];
function net22(v,cross,sign,lab){
  const nodes=[{id:'SL',x:.08,y:.2,label:'L',act:v.sL,shape:'in',sub:lab},{id:'SR',x:.08,y:.8,label:'R',act:v.sR,shape:'in'},
    {id:'ML',x:.92,y:.2,label:'L',act:v.mL,shape:'out',sub:'motor'},{id:'MR',x:.92,y:.8,label:'R',act:v.mR,shape:'out'}];
  const e=cross?[['SL','MR',v.sL],['SR','ML',v.sR]]:[['SL','ML',v.sL],['SR','MR',v.sR]];
  return{nodes,edges:e.map(([a,b,act])=>({a,b,sign,act}))};
}
function inner1to4(c,sim,w,h,g,curve){
  const cap=g.caption;g.caption=null;B.graph(c,30,24,400,170,g);if(cap)B.text(c,cap,30,200,C.mute,13);
  if(curve)B.curve(c,540,34,360,150,curve.f,curve.marks,curve.xl,curve.yl);
}

/* ---------------- 1 ---------------- */
M[1]={name:'Getting around',
 variants:[
  {k:'a',text:'One sensor feeds one motor. The more heat it senses, the faster the motor runs. The vehicle can only go straight ahead, but small disturbances (here a random push) bend its course. It crosses warm regions quickly and lingers in cool ones.',summary:'Runs faster in heat, so it crosses warm regions quickly.'},
  {k:'b',text:'For comparison, the wire is inhibitory. The more heat, the slower the motor. The book only brings in inhibitory wires with Vehicle 3. The vehicle slows down in warm regions and collects there.',summary:'Slows down in heat and collects in warm regions.'}],
 params:[nP(20,6),{id:'gain',label:'Sensitivity',min:0.5,max:4,step:0.1,val:2,fmt:x=>x.toFixed(1)},{id:'noise',label:'Disturbance',min:0,max:2,step:0.05,val:0.7,fmt:x=>x.toFixed(2)}],
 items:()=>[{kind:'heat',x:280,y:300,R:140},{kind:'heat',x:680,y:200,R:140}],
 init(sim){sim.vehs=B.spawn(sim,sim.params.n);sim.vehs.forEach(v=>v.single=true)},
 tick(sim,dt){
  const P=sim.params,inh=sim.variant==='b';let hot=0,sp=0;
  sim.vehs.forEach(v=>{
   const s=B.sens1(v,sim,'heat',0,140);
   const m=inh?clamp(1-P.gain*0.5*s,0.03,1):clamp(0.08+P.gain*0.5*s,0,1);
   v.sL=v.sR=s;v.th+=P.noise*Math.sqrt(dt)*gauss(sim.rng);v.wires=[{a:'L',b:'L',sign:inh?-1:1,act:s}];
   B.drive(v,m,m,dt);B.bounce(v);B.trail(v,dt);if(s>0.5)hot++;sp+=m;
  });
  sim.stat.hot=hot/sim.vehs.length;sim.stat.sp=sp/sim.vehs.length;
 },
 hud:s=>[`mean speed ${((s.stat.sp||0)*100|0)} %`],
 inner(c,sim,w,h){
  const v=sim.vehs[0]||{},inh=sim.variant==='b',g=sim.params.gain;
  const f=s=>inh?clamp(1-g*0.5*s,0.03,1):clamp(0.08+g*0.5*s,0,1);
  inner1to4(c,sim,w,h,{caption:'sensor → motor',nodes:[{id:'S',x:.12,y:.5,label:'S',act:v.sL,shape:'in',sub:'heat'},{id:'M',x:.88,y:.5,label:'M',act:v.mL,shape:'out',sub:'motor'}],edges:[{a:'S',b:'M',sign:inh?-1:1,act:v.sL}]},{f,marks:[v.sL||0],xl:'heat at the sensor',yl:'motor speed'});
 }};

/* ---------------- 2 ---------------- */
M[2]={name:'Fear and aggression',addKind:'light',
 variants:[
  {k:'a',text:'Two sensors, two motors. Each sensor drives the motor on its own side through an excitatory wire, so more light means a faster motor. The sensor nearer the source drives its motor harder, so the vehicle turns away from the source and runs faster the closer it is. Braitenberg reads this as fear.',summary:'Flees the light and runs faster the closer it gets.'},
  {k:'b',text:'Same parts, crossed wires. The sensor nearer the source drives the motor on the far side, so the vehicle turns toward the source and speeds up as it comes closer. It runs into the source, which Braitenberg calls aggression.',summary:'Charges at the light and speeds up as it closes in.'}],
 params:[nP(6,3),phiP(40),gainP(2.5)],
 items:()=>[{kind:'light',x:480,y:270}],
 init(sim){sim.vehs=B.spawn(sim,sim.params.n);sim.vehs.forEach(v=>v.phi=rad(sim.params.phi))},
 tick(sim,dt){
  const P=sim.params,cross=sim.variant==='b';
  sim.vehs.forEach(v=>{
   v.phi=rad(P.phi);const[sL,sR]=B.sens2(v,sim,'light',v.phi);v.sL=sL;v.sR=sR;
   const a=clamp(0.1+P.gain*sL),b=clamp(0.1+P.gain*sR);
   v.wires=pairWires(v,cross,1);B.drive(v,cross?b:a,cross?a:b,dt);B.bounce(v);B.trail(v,dt);B.touch(sim,v,'light');
  });
 },
 hud:s=>s.variant==='b'?[`hits ${s.stat.hits}`]:[],
 inner(c,sim,w,h){
  const v=sim.vehs[0]||{},cross=sim.variant==='b',g=sim.params.gain;
  const g1=net22(v,cross,1,'light');g1.caption=cross?'crossed, excitatory':'same side, excitatory';
  inner1to4(c,sim,w,h,g1,{f:s=>clamp(0.1+g*s),marks:[v.sL||0,v.sR||0],xl:'light at the sensor',yl:'motor speed'});
 }};

/* ---------------- 3 ---------------- */
const K3={heat:{cross:0,sign:1,name:'heat',acts:'flees from heat'},light:{cross:1,sign:1,name:'light',acts:'charges at lights'},oxy:{cross:1,sign:-1,name:'oxygen',acts:'slows near it, then moves on'},org:{cross:0,sign:-1,name:'organic',acts:'stays at it until it is used up'}};
// 3c inner panel: one row per sensor pair (L and R sensor), wires to the left and right motor, wiring named on the right
function inner3c(c,v){
  const rows=['heat','light','oxy','org'],ry=i=>36+i*58,Rn=12,sx=150,mx=540,mw=44,mTop=[22,136],mH=92,lines=[];
  c.textBaseline='middle';
  rows.forEach((kind,i)=>{
    const q=K3[kind],s=(v.s3&&v.s3[kind])||[0,0],y=ry(i);
    for(const side of[0,1]){const to=q.cross?1-side:side;
      lines.push({x0:sx+Rn,y0:y+(side?14:-14),x1:mx-3,y1:mTop[to]+13+i*22,act:s[side],sign:q.sign})}
  });
  lines.sort((a,b)=>(a.act>0.1)-(b.act>0.1));
  for(const e of lines){
    const col=e.act>0.1?C.red:C.mute;c.strokeStyle=col;c.fillStyle=col;c.lineWidth=1.4+clamp(e.act)*2.2;c.setLineDash(e.sign<0?[5,4]:[]);
    c.beginPath();c.moveTo(e.x0,e.y0);c.lineTo(e.x1,e.y1);c.stroke();c.setLineDash([]);
    const a=Math.atan2(e.y1-e.y0,e.x1-e.x0);
    c.beginPath();c.moveTo(e.x1+Math.cos(a)*2,e.y1+Math.sin(a)*2);c.lineTo(e.x1-Math.cos(a-.4)*10,e.y1-Math.sin(a-.4)*10);c.lineTo(e.x1-Math.cos(a+.4)*10,e.y1-Math.sin(a+.4)*10);c.closePath();c.fill();
  }
  const node=(x,y,act,txt,sq)=>{
    c.fillStyle='#fff';c.strokeStyle=C.ink;c.lineWidth=1.6;
    const path=()=>{c.beginPath();if(sq)c.rect(x,y,mw,mH);else c.arc(x,y,Rn,0,TAU)};
    path();c.fill();c.globalAlpha=clamp(act||0);c.fillStyle=C.red;path();c.fill();c.globalAlpha=1;path();c.stroke();
    c.fillStyle=(act||0)>0.55?'#fff':C.ink;c.font='600 14px system-ui';c.textAlign='center';c.fillText(txt,sq?x+mw/2:x,sq?y+mH/2:y+.5);
  };
  rows.forEach((kind,i)=>{
    const q=K3[kind],s=(v.s3&&v.s3[kind])||[0,0],y=ry(i);
    node(sx,y-14,s[0],'L');node(sx,y+14,s[1],'R');
    c.textAlign='left';c.fillStyle=C.ink;c.font='600 15px system-ui';c.fillText(q.name,22,y);
    c.font='600 14px system-ui';c.fillText((q.cross?'crossed':'same side')+', '+(q.sign>0?'excitatory':'inhibitory'),670,y-9);
    c.fillStyle=C.mute;c.font='13px system-ui';c.fillText(q.acts,670,y+10);
  });
  node(mx,mTop[0],v.mL,'L',true);node(mx,mTop[1],v.mR,'R',true);
  c.fillStyle=C.mute;c.font='13px system-ui';c.textAlign='left';c.fillText('motor',mx+mw+8,mTop[0]+mH/2);c.fillText('motor',mx+mw+8,mTop[1]+mH/2);
  c.textBaseline='alphabetic';
}
M[3]={name:'Love',addKind:'light',
 variants:[
  {k:'a',text:'Two sensors, two motors and same-side wires, but now inhibitory. More light means a slower motor. The vehicle turns toward the source, slows down as it gets closer and stops in front of it. Braitenberg calls this love.',summary:'Turns toward the light, slows down and stops in front of it.'},
  {k:'b',text:'Crossed and inhibitory. The vehicle turns away from the source, slowly while it is near, then speeds off. This is the explorer, which keeps moving on to look for other sources.',summary:'Turns away from the light and moves on to look elsewhere.'},
  {k:'c',text:'Four pairs of sensors, for heat, light, oxygen and organic matter, each wired differently. The vehicle flees heat, charges at lights and stays at organic matter, while oxygen slows it down for a moment before it moves on. Lamps break when it hits them. Oxygen and organic matter are used up after it has stayed a few seconds, and they reappear elsewhere.',summary:'Flees heat, charges at lights, slows near oxygen and stays at organic matter.'}],
 params:[nP(6,2),phiP(40),gainP(1.6,3)],
 items:v=>v==='c'?[{kind:'heat',x:300,y:150,R:80},{kind:'heat',x:720,y:400,R:80},{kind:'light',x:200,y:400},{kind:'light',x:760,y:140},{kind:'oxy',x:480,y:120},{kind:'oxy',x:480,y:420},{kind:'org',x:130,y:230},{kind:'org',x:830,y:300}]:[{kind:'light',x:480,y:270}],
 add(sim,x,y){const c3=sim.variant==='c';sim.items.push({kind:c3?['light','oxy','org','heat'][sim.items.length%4]:'light',x:c3?clamp(x,110,W-110):x,y:c3?clamp(y,110,H-110):y,R:c3&&sim.items.length%4===3?80:undefined})},
 init(sim){sim.vehs=B.spawn(sim,sim.variant==='c'?1:sim.params.n);sim.vehs.forEach(v=>v.phi=rad(sim.params.phi))},
 tick(sim,dt){
  const P=sim.params,k=sim.variant;
  sim.vehs.forEach(v=>{
   v.phi=rad(P.phi);let mL,mR;
   if(k==='c'){
    mL=0.55;mR=0.55;v.s3={};
    for(const kind in K3){const q=K3[kind],[a,b]=B.sens2(v,sim,kind,v.phi,-1,110);v.s3[kind]=[a,b];
     const lL=q.cross?b:a,lR=q.cross?a:b;mL+=q.sign*P.gain*lL;mR+=q.sign*P.gain*lR}
    v.sL=Math.max(...Object.values(v.s3).map(x=>x[0]));v.sR=Math.max(...Object.values(v.s3).map(x=>x[1]));
   }else{
    const[sL,sR]=B.sens2(v,sim,'light',v.phi,-1);v.sL=sL;v.sR=sR;
    const a=1-P.gain*sL,b=1-P.gain*sR;mL=k==='a'?a:b;mR=k==='a'?b:a;v.wires=pairWires(v,k==='b',-1);
   }
   B.drive(v,clamp(mL),clamp(mR),dt);B.bounce(v);B.trail(v,dt);
   if(k==='c'){
    v.th+=0.3*Math.sqrt(dt)*gauss(sim.rng); // small disturbances, as in Vehicle 1
    // lamps break when hit or circled for long; oxygen and organic matter are used up while the vehicle is parked
    v.park=(v.mL+v.mR)/2<0.12?(v.park||0)+dt:0;
    v.wall=(v.x<32||v.x>W-32||v.y<32||v.y>H-32)?(v.wall||0)+dt:0;
    if(v.wall>2.5){v.th=Math.atan2(H/2-v.y,W/2-v.x)+(sim.rng()-.5)*1.2;v.wall=0} // stuck along a wall: turn back into the room
    let near=null,nd=1e9;
    for(const o of sim.items){
     if(o.gone)continue;const d=Math.hypot(o.x-v.x,o.y-v.y);
     if(o.kind==='light'){o.eat=d<90?(o.eat||0)+dt:Math.max(0,(o.eat||0)-dt);if(d<28||o.eat>6){o.gone=true;o.eat=0;o.back=sim.t+4}}
     else if((o.kind==='oxy'||o.kind==='org')&&d<nd){nd=d;near=o}
    }
    if(v.park>3&&near){near.gone=true;near.back=sim.t+4;v.park=0}
   }
  });
  if(k==='c')for(const o of sim.items)if(o.gone&&o.back!=null&&sim.t>=o.back){ // it reappears somewhere else
   const v=sim.vehs[0],r=sim.rng;let x,y,t=0;
   do{x=110+r()*(W-220);y=110+r()*(H-220);t++}while(t<30&&(Math.hypot(x-v.x,y-v.y)<250||sim.items.some(q=>q!==o&&!q.gone&&Math.hypot(q.x-x,q.y-y)<90)));
   o.x=x;o.y=y;o.gone=false;o.back=null;o.flash=.5;
  }
 },
 hud:()=>[],
 inner(c,sim,w,h){
  const v=sim.vehs[0]||{},k=sim.variant,g=sim.params.gain;
  if(k==='c'){inner3c(c,v);return}
  const g1=net22(v,k==='b',-1,'light');g1.caption=k==='b'?'crossed, inhibitory':'same side, inhibitory';
  inner1to4(c,sim,w,h,g1,{f:s=>clamp(1-g*s),marks:[v.sL||0,v.sR||0],xl:'light at the sensor',yl:'motor speed'});
 }};

/* ---------------- 4 ---------------- */
const resp4=(s,s0,wd)=>0.2+0.8*Math.exp(-Math.pow((s-s0)/wd,2));
M[4]={name:'Values and special tastes',addKind:'light',
 variants:[
  {k:'a',text:'Motor speed rises with the stimulus up to a maximum and falls again beyond it, so the response curve is not monotonic. With crossed wires the vehicle is drawn toward the source until the stimulus gets too strong, then it is turned away. Here it ends up circling the source at a preferred distance.',summary:'Drawn to the light, it circles at a preferred distance.'},
  {k:'b',text:'Threshold devices sit between the sensors and the motors. Below the threshold the motors idle and above it they run flat out. The vehicle ignores a weak stimulus and charges once the stimulus is strong enough.',summary:'Ignores weak stimuli and charges at lights once they are strong enough.'}],
 params:[nP(4,2),phiP(35),{id:'s0',label:'Preferred stimulus',min:0.15,max:0.6,step:0.01,val:0.35,fmt:x=>x.toFixed(2),show:v=>v==='a'},{id:'wd',label:'Response width',min:0.1,max:0.6,step:0.01,val:0.28,fmt:x=>x.toFixed(2),show:v=>v==='a'},{id:'thr',label:'Threshold',min:0.05,max:0.8,step:0.01,val:0.3,fmt:x=>x.toFixed(2),show:v=>v==='b'}],
 items:v=>v==='a'?[{kind:'light',x:480,y:270}]:[{kind:'light',x:240,y:150},{kind:'light',x:700,y:190},{kind:'light',x:520,y:430}],
 init(sim){sim.vehs=B.spawn(sim,sim.variant==='b'?1:sim.params.n);sim.vehs.forEach(v=>{v.phi=rad(sim.params.phi);v.bias=0;v.on=[0,0]});if(sim.variant==='b'){const v=sim.vehs[0];v.x=120;v.y=430;v.th=-0.5}},
 tick(sim,dt){
  const P=sim.params;
  sim.vehs.forEach(v=>{
   v.phi=rad(P.phi);
   if(sim.variant==='a'){
    const[sL,sR]=B.sens2(v,sim,'light',v.phi,-0.8);v.sL=sL;v.sR=sR;
    B.drive(v,resp4(sR,P.s0,P.wd),resp4(sL,P.s0,P.wd),dt);v.wires=pairWires(v,true,1);
   }else{
    const[sL,sR]=B.sens2(v,sim,'light',v.phi,1,120);v.sL=sL;v.sR=sR;
    v.on=[sL>P.thr?1:0,sR>P.thr?1:0];
    v.bias+=(-v.bias*0.6)*dt+0.1*Math.sqrt(dt)*gauss(sim.rng);
    B.drive(v,clamp(0.2+v.bias+0.8*v.on[1]),clamp(0.2-v.bias+0.8*v.on[0]),dt);v.wires=pairWires(v,true,1);B.touch(sim,v,'light');
   }
   B.bounce(v);B.trail(v,dt);
  });
 },
 hud:s=>s.variant==='b'?[`lights reached ${s.stat.hits}`]:[],
 inner(c,sim,w,h){
  const v=sim.vehs[0]||{},P=sim.params;
  if(sim.variant==='a'){
   const g=net22(v,true,1,'light');g.caption='crossed, response not monotonic';
   inner1to4(c,sim,w,h,g,{f:s=>resp4(s,P.s0,P.wd),marks:[v.sL||0,v.sR||0],xl:'light at the sensor',yl:'motor speed'});
  }else{
   const o=v.on||[0,0];
   const g={caption:'sensor → threshold device → motor (crossed)',nodes:[
    {id:'SL',x:.08,y:.2,label:'L',act:v.sL,shape:'in'},{id:'SR',x:.08,y:.8,label:'R',act:v.sR,shape:'in'},
    {id:'TL',x:.5,y:.2,label:'T',act:o[0],shape:'unit'},{id:'TR',x:.5,y:.8,label:'T',act:o[1],shape:'unit'},
    {id:'ML',x:.92,y:.2,label:'L',act:v.mL,shape:'out'},{id:'MR',x:.92,y:.8,label:'R',act:v.mR,shape:'out'}],
    edges:[{a:'SL',b:'TL',sign:1,act:o[0]},{a:'SR',b:'TR',sign:1,act:o[1]},{a:'TL',b:'MR',sign:1,act:o[0]},{a:'TR',b:'ML',sign:1,act:o[1]}]};
   inner1to4(c,sim,w,h,g,{f:s=>s>P.thr?1:0.2,marks:[v.sL||0,v.sR||0],xl:'light at the sensor',yl:'motor speed'});
  }
 }};

/* ---------------- 5 ---------------- */
const HZ=0.35;
const flashLit=(o,t)=>{const p=o.pat,u=t%p.period;return p.starts.some(s=>u>=s&&u<s+p.len)};
// 5d: put beacons A and B 170-260 px ahead of the vehicle, close enough together in angle (at most 60 degrees)
// that both stay in view while it turns to face either of them
function place5(sim,v,again){
  const r=sim.rng,A=sim.items.find(o=>o.label==='A'),Bc=sim.items.find(o=>o.label==='B');
  if(!A||!Bc)return;
  v.n.scene=sim.t;
  for(let t=0;t<120;t++){
    const c=v.th+(r()*2-1)*0.9,h=0.2+r()*0.3,s=r()<.5?1:-1,a1=c+s*h,a2=c-s*h,d1=170+r()*80,d2=170+r()*90;
    const xa=v.x+Math.cos(a1)*d1,ya=v.y+Math.sin(a1)*d1,xb=v.x+Math.cos(a2)*d2,yb=v.y+Math.sin(a2)*d2;
    if(xa>70&&xa<W-70&&xb>70&&xb<W-70&&ya>70&&ya<H-90&&yb>70&&yb<H-90){A.x=xa;A.y=ya;Bc.x=xb;Bc.y=yb;return}
  }
  v.th=Math.atan2(H/2-v.y,W/2-v.x);if(!again)place5(sim,v,true);
}
M[5]={name:'Logic',addKind:'light',
 variants:[
  {k:'a',text:'Threshold devices can be combined into logic. Here the light pathway only fires when the heat sensor is active too (AND). The vehicle goes for lights inside the warm area, marked by the dotted circle, and ignores the others.',summary:'Goes only for lights inside the warm area.'},
  {k:'b',text:'The heat sensor inhibits the light pathway (AND NOT). The vehicle goes for lights outside the warm area and ignores the others.',summary:'Goes only for lights outside the warm area.'},
  {k:'c',text:'A threshold device wired back onto itself keeps firing after its input has stopped. That is a memory. When the blinking light is off, the vehicle keeps driving where it was heading. Here the memory fades after the set span.',summary:'Keeps driving the same way for a while after the light goes off.'},
  {k:'d',text:'Threshold devices in a chain can count. The vehicle turns toward a flashing beacon but only drives off after the third flash in a row, and then the count starts again. Beacon A flashes three times per cycle and beacon B twice, so it only ever goes to A.',summary:'Turns to a flashing beacon but drives off only after the third flash.'}],
 params:[{id:'thr',label:'Threshold',min:0.05,max:0.8,step:0.01,val:0.3,fmt:x=>x.toFixed(2)},
  {id:'tau',label:'Memory span',min:0.2,max:6,step:0.1,val:1.5,fmt:x=>x.toFixed(1)+' s',show:v=>v==='c'},
  {id:'blink',label:'Blink period',min:1,max:8,step:0.5,val:4,fmt:x=>x.toFixed(1)+' s',show:v=>v==='c'}],
 items:v=>v==='c'?[{kind:'light',x:640,y:270}]
  :v==='d'?[{kind:'light',x:560,y:400,R:320,nolabel:true,pat:{starts:[2.3,3,3.7],len:0.35,period:8.3},label:'A'},{kind:'light',x:560,y:160,R:320,nolabel:true,pat:{starts:[0,0.7],len:0.35,period:8.3},label:'B'}]
  :[{kind:'heat',x:300,y:260,R:110,zone:110*Math.sqrt(1/HZ-1)},{kind:'light',x:240,y:210},{kind:'light',x:700,y:160},{kind:'light',x:660,y:420},{kind:'light',x:340,y:440}],
 init(sim){
  const v=sim.variant==='c'?B.mkVeh(470,270,0):sim.variant==='d'?B.mkVeh(300,290,0):B.mkVeh(120,420,-0.5);
  v.bias=0;v.n={aL:0,aR:0,g:0,M:0,eL:0,eR:0,cnt:0,last:-9,go:0,T:0,reloc:0,scene:0};v.phi=rad(35);sim.vehs=[v];sim.stat.A=0;sim.stat.B=0;
  if(sim.variant==='d')place5(sim,v);
 },
 tick(sim,dt){
  const P=sim.params,k=sim.variant,v=sim.vehs[0],n=v.n;
  for(const o of sim.items){if(k==='c')o.off=(sim.t%P.blink)>P.blink*0.45;if(o.pat)o.off=!flashLit(o,sim.t)}
  const[sL,sR]=B.sens2(v,sim,'light',v.phi);v.sL=sL;v.sR=sR;
  const sH=B.sense(v.x,v.y,0,'heat',sim,0);v.sH=sH;
  const pa=n.aL||n.aR;
  n.aL=sL>P.thr?1:0;n.aR=sR>P.thr?1:0;n.g=sH>HZ?1:0;
  let eL=n.aL,eR=n.aR,fwd=0;
  if(k==='a'){eL*=n.g;eR*=n.g}
  if(k==='b'){eL*=1-n.g;eR*=1-n.g}
  if(k==='c'){n.M=(n.aL||n.aR)?1:n.M*Math.exp(-dt/P.tau);if(!n.aL&&!n.aR&&n.M>0.15)fwd=0.55}
  if(k==='d'){
   // the vehicle waits, turns toward a flashing beacon and only drives off after the third flash in a row
   const rise=(n.aL||n.aR)&&!pa;
   if(rise){if(sim.t-n.last>1.3)n.cnt=0;n.cnt++;n.last=sim.t;if(n.cnt>=3){n.go=4.2;n.cnt=0}}
   const goPrev=n.go;n.go=Math.max(0,n.go-dt);
   if(goPrev>0&&n.go===0)n.reloc=0.8; // it missed: set up a new scene
   if(sim.t-n.scene>25&&!n.reloc&&n.go===0)place5(sim,v); // no go for a long time (e.g. a beacon out of view): new scene
   const st=clamp(4*(sR-sL),-1,1),base=n.go>0?0.85:0;
   n.eL=n.aL;n.eR=n.aR;
   B.drive(v,clamp(base+Math.max(st,0)),clamp(base+Math.max(-st,0)),dt);B.bounce(v);B.trail(v,dt);
   for(const o of sim.items){
    if(o.cd>0)o.cd-=dt;
    if(o.label&&Math.hypot(o.x-v.x,o.y-v.y)<26&&!(o.cd>0)){sim.stat[o.label]++;o.cd=3;o.flash=0.5;n.go=0;n.reloc=0.8}
   }
   if(n.reloc>0&&(n.reloc-=dt)<=0){n.reloc=0;place5(sim,v)}
   return;
  }
  n.eL=eL;n.eR=eR;
  v.bias+=(-v.bias*0.6)*dt+0.1*Math.sqrt(dt)*gauss(sim.rng);
  B.drive(v,clamp(0.26+v.bias+0.75*eR+fwd),clamp(0.26-v.bias+0.75*eL+fwd),dt);B.bounce(v);B.trail(v,dt);
  if(k==='c'){const l=sim.items[0];if(Math.hypot(l.x-v.x,l.y-v.y)<22){sim.stat.hits++;l.flash=0.5;n.M=0;const a=v.th+(sim.rng()*2-1)*0.8;l.x=clamp(v.x+Math.cos(a)*200,75,W-75);l.y=clamp(v.y+Math.sin(a)*200,75,H-75)}}
  else B.touch(sim,v,'light');
 },
 render(c,sim){if(sim.variant==='d')for(const o of sim.items){B.text(c,o.label==='A'?'A flashes 3 times':'B flashes 2 times',o.x,o.y+34,C.mute,12,'center')}},
 hud(s){const n=s.vehs[0].n;
  if(s.variant==='d')return[`count ${n.go>0?3:n.cnt} of 3`,`reached A ${s.stat.A}   reached B ${s.stat.B}`];
  if(s.variant==='c')return[`memory ${(n.M*100|0)} %`,`lights reached ${s.stat.hits}`];
  return[`lights reached ${s.stat.hits}`]},
 inner(c,sim,w,h){
  const k=sim.variant,v=sim.vehs[0],n=v.n;
  const SL={id:'SL',x:.06,y:.2,label:'L',act:v.sL,shape:'in',sub:'light'},SR={id:'SR',x:.06,y:.62,label:'R',act:v.sR,shape:'in'};
  const ML={id:'ML',x:.94,y:.2,label:'L',act:v.mL,shape:'out',sub:'motor'},MR={id:'MR',x:.94,y:.62,label:'R',act:v.mR,shape:'out'};
  let g;
  const base=[{a:'SL',b:'TL',sign:1,act:n.aL},{a:'SR',b:'TR',sign:1,act:n.aR},{a:'TL',b:'MR',sign:1,act:n.eL},{a:'TR',b:'ML',sign:1,act:n.eR}];
  if(k==='a'||k==='b'){
   g={caption:k==='a'?'heat unit H lets the light units pass (AND)':'heat unit H blocks the light units (AND NOT)',nodes:[SL,SR,{id:'TL',x:.5,y:.2,label:'T',act:n.eL,shape:'unit'},{id:'TR',x:.5,y:.62,label:'T',act:n.eR,shape:'unit'},ML,MR,
    {id:'SH',x:.06,y:.98,label:'H',act:clamp(v.sH),shape:'in',sub:'heat'},{id:'G',x:.3,y:.98,label:k==='a'?'&':'¬',act:n.g,shape:'unit'}],
    edges:[...base,{a:'SH',b:'G',sign:1,act:n.g},{a:'G',b:'TL',sign:k==='a'?1:-1,act:n.g,curve:20},{a:'G',b:'TR',sign:k==='a'?1:-1,act:n.g}]};
  }else if(k==='c'){
   g={caption:'T units steer while the light is on, and the self-excited unit M keeps both motors running',nodes:[SL,SR,{id:'TL',x:.35,y:.2,label:'T',act:n.aL,shape:'unit'},{id:'TR',x:.35,y:.62,label:'T',act:n.aR,shape:'unit'},{id:'M',x:.6,y:.98,label:'M',act:n.M,shape:'unit'},ML,MR],
    edges:[{a:'SL',b:'TL',sign:1,act:n.aL},{a:'SR',b:'TR',sign:1,act:n.aR},{a:'TL',b:'MR',sign:1,act:n.aL},{a:'TR',b:'ML',sign:1,act:n.aR},{a:'TL',b:'M',sign:1,act:n.aL},{a:'TR',b:'M',sign:1,act:n.aR},{a:'M',b:'M',sign:1,act:n.M},{a:'M',b:'ML',sign:1,act:n.M>.15?1:0},{a:'M',b:'MR',sign:1,act:n.M>.15?1:0}]};
  }else{
   g={caption:'T detects a flash, units 1 to 3 count flashes in a row, and the third starts both motors and resets the count',nodes:[SL,SR,{id:'T',x:.26,y:.4,label:'T',act:(n.aL||n.aR)?1:0,shape:'unit'},
    {id:'C1',x:.42,y:.4,label:'1',act:n.cnt>=1?1:0,shape:'unit'},{id:'C2',x:.56,y:.4,label:'2',act:n.cnt>=2?1:0,shape:'unit'},{id:'C3',x:.7,y:.4,label:'3',act:n.go>0?1:0,shape:'unit'},ML,MR],
    edges:[{a:'SL',b:'T',sign:1,act:n.aL},{a:'SR',b:'T',sign:1,act:n.aR},{a:'T',b:'C1',sign:1,act:n.cnt>=1?1:0},{a:'C1',b:'C2',sign:1,act:n.cnt>=2?1:0},{a:'C2',b:'C3',sign:1,act:n.go>0?1:0},{a:'C3',b:'ML',sign:1,act:n.go>0?1:0},{a:'C3',b:'MR',sign:1,act:n.go>0?1:0}]};
  }
  const cap=g.caption;g.caption=null;B.graph(c,40,26,w-80,150,g);B.text(c,cap,40,238,C.mute,13);
 }};

/* ---------------- 6 ---------------- */
const SCOL={love:'#c4301c',fear:'#b9b7ae',aggression:'#1c1c1a',explorer:'#6e9ad0'};
const SNAME={love:'same side, inhibitory (love)',fear:'same side, excitatory (fear)',aggression:'crossed, excitatory (aggression)',explorer:'crossed, inhibitory (explorer)'};
const TB={x0:50,y0:44,x1:W-50,y1:H-44};
const strat=g=>{const u=(g.a+g.d)/2,x=(g.b+g.c)/2;return Math.abs(u)>=Math.abs(x)?(u<0?'love':'fear'):(x<0?'explorer':'aggression')};
const randG=r=>{const w=()=>(r()*2-1)*2;return{a:w(),b:w(),c:w(),d:w(),bs:0.05+r()*0.45}};
const mutG=(g,m,r)=>{const f=(x,s)=>x+gauss(r)*m*s;return{a:clamp(f(g.a,1),-3,3),b:clamp(f(g.b,1),-3,3),c:clamp(f(g.c,1),-3,3),d:clamp(f(g.d,1),-3,3),bs:clamp(f(g.bs,0.3),0,0.7)}};
function newV6(sim,g){
 const r=sim.rng,v=B.mkVeh(TB.x0+60+r()*(TB.x1-TB.x0-120),TB.y0+60+r()*(TB.y1-TB.y0-120),r()*TAU);
 v.g=g;v.e=1;v.s=strat(g);v.fill=SCOL[v.s];v.sc=0.6;v.phi=rad(40);return v;
}
M[6]={name:'Selection',speed:4,addKind:'light',
 variants:[{k:'a',text:'Vehicles with random wiring run on a table with food. A vehicle that falls off the edge is replaced by a copy of one still running, and here the same happens when one starves or grows old. The copy is not exact, so the wiring is never designed, only selected. The fill colour shows the wiring type.',summary:'Vehicles that fall off, starve or grow old are replaced by mutated copies.'}],
 params:[{id:'pop',label:'Population',min:6,max:40,step:1,val:18,reset:true},{id:'mut',label:'Mutation',min:0.02,max:0.8,step:0.01,val:0.25,fmt:x=>x.toFixed(2)}],
 items:()=>[{kind:'light',x:310,y:270,nolabel:true},{kind:'light',x:650,y:270,nolabel:true}],
 init(sim){sim.ev={deaths:[],ghosts:[],births:0};for(let i=0;i<sim.params.pop;i++)sim.vehs.push(newV6(sim,randG(sim.rng)))},
 tick(sim,dt){
  const ev=sim.ev,r=sim.rng;
  for(let i=0;i<sim.vehs.length;i++){
   const v=sim.vehs[i],g=v.g,[sL,sR]=B.sens2(v,sim,'light',v.phi);v.sL=sL;v.sR=sR;
   B.drive(v,clamp(g.bs+g.a*sL+g.b*sR),clamp(g.bs+g.c*sL+g.d*sR),dt);B.trail(v,dt);v.age+=dt;
   let fed=false;for(const o of sim.items)if(Math.hypot(o.x-v.x,o.y-v.y)<100)fed=true;
   v.e=Math.min(1.5,v.e+dt*(fed?0.35:-0.04));
   let why=null;
   if(v.x<TB.x0||v.x>TB.x1||v.y<TB.y0||v.y>TB.y1)why='fell';else if(v.e<=0)why='starved';else if(v.age>70)why='old';
   if(why){
    ev.deaths.push(v.age);ev.births++;ev.ghosts.push({x:clamp(v.x,TB.x0,TB.x1),y:clamp(v.y,TB.y0,TB.y1),t:0});
    let best=null,bf=-1;for(let k=0;k<3;k++){const q=sim.vehs[(r()*sim.vehs.length)|0];if(q===v)continue;const f=q.e+q.age/80;if(f>bf){bf=f;best=q}}
    sim.vehs[i]=newV6(sim,best?mutG(best.g,sim.params.mut,r):randG(r));
   }
  }
  ev.ghosts=ev.ghosts.filter(g=>(g.t+=dt)<2.5);
 },
 avg(sim){const n=sim.ev.deaths.slice(-25);return n.length?n.reduce((a,b)=>a+b,0)/n.length:0},
 hud(s){const c={love:0,fear:0,aggression:0,explorer:0};s.vehs.forEach(v=>c[v.s]++);
  return[`replaced ${s.ev.births}   the last 25 survived ${M[6].avg(s).toFixed(0)} s on average`,`love ${c.love}   fear ${c.fear}   aggression ${c.aggression}   explorer ${c.explorer}`]},
 renderBack(c,sim){
  c.fillStyle=C.faint;c.fillRect(0,0,W,H);c.fillStyle='#fff';c.strokeStyle=C.ink;c.lineWidth=1.5;c.beginPath();c.rect(TB.x0,TB.y0,TB.x1-TB.x0,TB.y1-TB.y0);c.fill();c.stroke();
 },
 render(c,sim){for(const g of sim.ev.ghosts){c.save();c.globalAlpha=1-g.t/2.5;c.strokeStyle=C.red;c.lineWidth=1.8;c.beginPath();c.moveTo(g.x-6,g.y-6);c.lineTo(g.x+6,g.y+6);c.moveTo(g.x+6,g.y-6);c.lineTo(g.x-6,g.y+6);c.stroke();c.restore()}},
 inner(c,sim,w,h){
  const n=sim.vehs.length||1,m={a:0,b:0,c:0,d:0};sim.vehs.forEach(v=>{for(const k in m)m[k]+=v.g[k]/n});
  B.graph(c,30,24,330,170,{nodes:[{id:'SL',x:.1,y:.2,label:'L',act:0,shape:'in'},{id:'SR',x:.1,y:.8,label:'R',act:0,shape:'in'},{id:'ML',x:.9,y:.2,label:'L',act:0,shape:'out'},{id:'MR',x:.9,y:.8,label:'R',act:0,shape:'out'}],
   edges:[{a:'SL',b:'ML',w:m.a/1.5,sign:Math.sign(m.a)||1},{a:'SR',b:'MR',w:m.d/1.5,sign:Math.sign(m.d)||1},{a:'SL',b:'MR',w:m.c/1.5,sign:Math.sign(m.c)||1},{a:'SR',b:'ML',w:m.b/1.5,sign:Math.sign(m.b)||1}].map(e=>({...e,act:0}))});
  B.text(c,'average wiring of the population (line thickness = weight)',30,200,C.mute,13);
  // survival of each dead vehicle
  const d=sim.ev.deaths,x0=480,y0=34,pw=430,ph=146,N=Math.max(60,d.length);
  c.strokeStyle=C.ink;c.lineWidth=1;c.beginPath();c.moveTo(x0,y0);c.lineTo(x0,y0+ph);c.lineTo(x0+pw,y0+ph);c.stroke();
  B.text(c,'survival time of each replaced vehicle (dots), moving average (line)',x0-30,y0-12,C.mute,13);B.text(c,'70 s',x0-6,y0+8,C.mute,13,'right');B.text(c,'0',x0-6,y0+ph,C.mute,13,'right');
  const px=i=>x0+i/(N-1)*pw,py=y=>y0+ph-Math.min(y,70)/70*ph,st=Math.max(0,d.length-N);
  c.fillStyle=C.mute;for(let i=st;i<d.length;i++)c.fillRect(px(i-st)-1.5,py(d[i])-1.5,3,3);
  if(d.length>1){c.strokeStyle=C.red;c.lineWidth=2;c.beginPath();for(let i=st;i<d.length;i++){let s=0,k=0;for(let j=Math.max(0,i-14);j<=i;j++){s+=d[j];k++}i===st?c.moveTo(px(0),py(s/k)):c.lineTo(px(i-st),py(s/k))}c.stroke()}
 }};

/* ---------------- 7: mnemotrix ---------------- */
// A respawned lamp + organic matter: at least 110 px from the walls, 200 px from the vehicle and 150 px from every other group's lamp.
// Tries up to 40 random spots and keeps the best one if none meets all the demands (score: the worst distance as a fraction of what is asked).
function placeGroup(sim,g,v){
  const r=sim.rng,lamps=sim.items.filter(o=>o.kind==='light'&&o.grp!=null&&o.grp!==g);
  let x=0,y=0,best=-1;
  for(let t=0;t<40;t++){
    const px=110+r()*(W-220-26),py=110+r()*(H-220);   // the organic matter sits 26 px right of the lamp
    const sc=Math.min(Math.hypot(px-v.x,py-v.y)/200,...lamps.map(o=>Math.hypot(px-o.x,py-o.y)/150));
    if(sc>best){best=sc;x=px;y=py}
    if(sc>=1)break;
  }
  for(const o of sim.items)if(o.grp===g){o.x=x+(o.kind==='org'?26:0);o.y=y;o.gone=false}
}
M[7]={name:'Concepts',addKind:'light',
 variants:[{k:'a',text:'A mnemotrix wire joins two threshold devices. Its resistance is high at first and drops when both devices are active together, then slowly recovers. While it is low, activity in one device tends to excite the other. Here a lamp usually sits next to the organic matter the vehicle goes for, and after a few meetings a lamp alone draws it.',summary:'Learns that a lamp means food and starts to follow lamps alone.'}],
 params:[{id:'rate',label:'Learning rate',min:0.02,max:0.4,step:0.01,val:0.12,fmt:x=>x.toFixed(2)},{id:'tau',label:'Forgetting time',min:10,max:300,step:5,val:120,fmt:x=>x+' s'}],
 items:()=>[{kind:'light',x:230,y:150,grp:0},{kind:'org',x:256,y:150,grp:0},{kind:'light',x:720,y:140,grp:1},{kind:'org',x:746,y:140,grp:1},{kind:'light',x:600,y:420,grp:2},{kind:'org',x:626,y:420,grp:2},
  {kind:'light',x:150,y:420,lone:true},{kind:'light',x:860,y:340,lone:true}],
 init(sim){const v=B.mkVeh(480,270,0.2);v.phi=rad(40);sim.vehs=[v];sim.w=0;sim.hist=[];sim.respawn=[];sim.stat.eaten=0;sim.stat.lone=0},
 tick(sim,dt){
  const P=sim.params,v=sim.vehs[0];
  const[lL,lR]=B.sens2(v,sim,'light',v.phi,1,110),[oL,oR]=B.sens2(v,sim,'org',v.phi,1,110);
  v.lL=lL;v.lR=lR;v.oL=oL;v.oR=oR;v.sL=Math.max(lL,oL);v.sR=Math.max(lR,oR);
  const aL=clamp(Math.max(lL,lR)*3),aO=clamp(Math.max(oL,oR)*3);v.aL=aL;v.aO=aO;
  sim.w=clamp(sim.w+dt*(P.rate*aL*aO*(1-sim.w)-sim.w/P.tau));
  const eL=oL+sim.w*lL,eR=oR+sim.w*lR;
  B.drive(v,clamp(0.22+1.6*eR),clamp(0.22+1.6*eL),dt);B.bounce(v);B.trail(v,dt);
  for(const o of sim.items){
   if(o.kind==='org'&&!o.gone&&Math.hypot(o.x-v.x,o.y-v.y)<22){o.gone=true;o.flash=.5;sim.stat.eaten++;sim.respawn.push({g:o.grp,t:sim.t+4})}
   if(o.lone&&!(o.cd>0)&&Math.hypot(o.x-v.x,o.y-v.y)<26){sim.stat.lone++;o.cd=4;o.flash=.5}if(o.cd>0)o.cd-=dt;
  }
  sim.respawn=sim.respawn.filter(q=>{if(sim.t>=q.t){placeGroup(sim,q.g,v);return false}return true});
  if(Math.floor(sim.t*2)!==Math.floor((sim.t-dt)*2)){sim.hist.push(sim.w);if(sim.hist.length>400)sim.hist.shift()}
 },
 onDrag(sim,o){for(const q of sim.items)if(q!==o&&q.grp===o.grp){q.x=o.x+(q.kind==='org'?26:0);q.y=o.y}},
 hud:s=>[`mnemotrix strength ${(s.w*100|0)} %   organic matter eaten ${s.stat.eaten}   lone lamps visited ${s.stat.lone}`],
 inner(c,sim,w,h){
  const v=sim.vehs[0],wt=sim.w;
  B.text(c,'U1 and U2 are threshold devices. The double line is the mnemotrix, strength '+wt.toFixed(2),30,214,C.mute,13);
  B.graph(c,30,20,520,176,{R:16,nodes:[
   {id:'SL',x:.06,y:.15,label:'S',act:Math.max(v.lL,v.lR),shape:'in',sub:'light'},{id:'SO',x:.06,y:.8,label:'S',act:Math.max(v.oL,v.oR),shape:'in',sub:'organic'},
   {id:'U1',x:.4,y:.15,label:'U1',act:v.aL,shape:'unit'},{id:'U2',x:.4,y:.8,label:'U2',act:Math.max(v.aO,wt*v.aL),shape:'unit'},
   {id:'M',x:.92,y:.5,label:'M',act:(v.mL+v.mR)/2,shape:'out',sub:'motors'}],
   edges:[{a:'SL',b:'U1',sign:1,act:v.aL},{a:'SO',b:'U2',sign:1,act:v.aO},{a:'U1',b:'U2',type:'mnem',w:wt,act:wt*v.aL,label:''},{a:'U2',b:'M',sign:1,act:Math.max(v.aO,wt*v.aL),curve:10}]});
  const x0=620,y0=34,pw=300,ph=140;c.strokeStyle=C.ink;c.lineWidth=1;c.beginPath();c.moveTo(x0,y0);c.lineTo(x0,y0+ph);c.lineTo(x0+pw,y0+ph);c.stroke();
  B.text(c,'mnemotrix strength over time',x0,y0-10,C.mute,13);B.text(c,'1',x0-6,y0+8,C.mute,13,'right');B.text(c,'0',x0-6,y0+ph,C.mute,13,'right');
  c.strokeStyle=C.red;c.lineWidth=2;c.beginPath();sim.hist.forEach((q,i)=>{const x=x0+i/399*pw,y=y0+ph-q*ph;i?c.lineTo(x,y):c.moveTo(x,y)});c.stroke();
 }};
})();
