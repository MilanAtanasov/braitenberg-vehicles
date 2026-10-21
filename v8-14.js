'use strict';
(function(){
const {W,H,C,clamp,rad,gauss,wrapA,M,TAU}=B;
const phi40=rad(40);
function moveGroup(sim,g,x,y){for(const o of sim.items)if(o.grp===g){o.x=x+(o.ox||0);o.y=y+(o.oy||0)}}
function farSpot(sim,v,d=200,m=70){const r=sim.rng;let x,y,t=0;do{x=m+r()*(W-2*m);y=m+r()*(H-2*m);t++}while(t<40&&Math.hypot(x-v.x,y-v.y)<d);return[x,y]}
function gotoXY(v,tx,ty,dt,sp){
  const e=wrapA(Math.atan2(ty-v.y,tx-v.x)-v.th),turn=clamp(e*1.5,-1,1),s=sp*clamp(Math.hypot(tx-v.x,ty-v.y)/40);
  B.drive(v,clamp(s+0.55*turn),clamp(s-0.55*turn),dt);
}
// extra turn (added to the left wheel, taken from the right) that bends a vehicle away from a wall it is approaching
function wallTurn(v,m=160){const d=Math.min(v.x,W-v.x,v.y,H-v.y);return d<m?0.35*clamp(wrapA(Math.atan2(H/2-v.y,W/2-v.x)-v.th),-1,1)*(1-d/m):0}
// slow wavering search path that also bends away from the walls; returns [mL,mR]
function wander(sim,v,base=0.3,wm=1){
  const wob=wm*(0.05*Math.sin(sim.t*TAU/22)+0.02*Math.sin(sim.t*TAU/7.3)),t=wallTurn(v);
  return[base+wob+t,base-wob-t];
}
// a point d0..d1 px in front of the vehicle (any direction if that does not fit), at least m px from every wall; the lamp and the food beside it
function spotAhead(sim,v,d0,d1,m){
  const r=sim.rng;let x=0,y=0;
  for(let t=0;t<40;t++){const a=v.th+(t<25?(r()*2-1)*0.9:(r()*2-1)*Math.PI),d=d0+r()*(d1-d0);x=v.x+Math.cos(a)*d;y=v.y+Math.sin(a)*d;if(x>=m&&x<=W-m-30&&y>=m&&y<=H-m)return[x,y]}
  return[clamp(x,m,W-m-30),clamp(y,m,H-m)];
}
const dot=(c,x,y,r,col)=>{c.fillStyle=col;c.beginPath();c.arc(x,y,r,0,TAU);c.fill()};

/* ---------------- 8: neighbourhood, things, movement ---------------- */
const N8=64,FOV8=rad(170);
M[8]={name:'Space, things and movements',
 variants:[
  {k:'a',text:'Each retina photocell feeds its own threshold device, and they form a map of the visual field. Lateral inhibition lets an active unit suppress its neighbours, so a blur becomes one thing at one place. A delay line between neighbours makes a movement detector for each direction. The vehicle ignores lamps that stand still and turns to those that move, walks toward them briefly and stops to watch again.',summary:'Ignores lamps that stand still and turns to the ones that move.'},
  {k:'b',text:'Same vehicle, but without lateral inhibition. The map keeps the whole blur of each lamp, so the movement detectors respond all along it, even for lamps that stand still. The vehicle turns to the brightest lamp, moving or not.',summary:'Turns to the brightest lamp, whether it moves or not.'}],
 params:[{id:'delay',label:'Detector delay',min:0.1,max:0.8,step:0.05,val:0.3,fmt:x=>x.toFixed(2)+' s'}],
 // the lamps that stand still are the brighter ones, so a vehicle that only looks at brightness would stare at them
 items:()=>[{kind:'light',x:190,y:150,power:2},{kind:'light',x:760,y:110,power:2},{kind:'light',x:800,y:420,power:2},
  {kind:'light',fixed:true,mv:{r:170,w:0.33,ph:0.5},nolabel:true},{kind:'light',fixed:true,mv:{r:215,w:-0.26,ph:3.2},nolabel:true}],
 init(sim){const v=B.mkVeh(480,270,-0.3);v.sac=null;v.hold=0;v.go=0;v.phi=0;sim.vehs=[v];sim.uh=[];sim.stat.sac=0;sim.stat.mv=0;sim.stat.st=0;sim.r8={I:new Float32Array(N8),u:new Float32Array(N8),dR:new Float32Array(N8),dL:new Float32Array(N8),mx:0}},
 tick(sim,dt){
  const v=sim.vehs[0],q=sim.r8,{I,u,dR,dL}=q,P=sim.params,inh=sim.variant==='a';
  for(const o of sim.items)if(o.mv){o.x=480+o.mv.r*Math.cos(o.mv.ph+o.mv.w*sim.t);o.y=270+o.mv.r*Math.sin(o.mv.ph+o.mv.w*sim.t)}
  I.fill(0);
  for(const o of sim.items){
   const dx=o.x-v.x,dy=o.y-v.y,d=Math.hypot(dx,dy),a=wrapA(Math.atan2(dy,dx)-v.th);if(Math.abs(a)>FOV8/2+0.1)continue;
   const pos=(a+FOV8/2)/FOV8*(N8-1),amp=(o.power||1)/(1+d*d/(260*260));
   for(let k=Math.max(0,Math.floor(pos-5));k<=Math.min(N8-1,Math.ceil(pos+5));k++)I[k]+=amp*Math.exp(-((k-pos)**2)/(2*2.2*2.2));
  }
  for(let k=0;k<N8;k++){let isMax=I[k]>0.06;if(inh)for(let j=Math.max(0,k-5);j<=Math.min(N8-1,k+5);j++)if(I[j]>I[k]||(I[j]===I[k]&&j<k))isMax=false;u[k]=(inh?isMax:I[k]>0.06)?I[k]:0}
  dR.fill(0);dL.fill(0);q.mx=0;
  const L=Math.round(P.delay*60);
  if(v.sac){
   // a saccade: the vehicle turns on the spot; the map is not read while it turns (the whole scene sweeps over the retina)
   v.sac.p+=dt/0.35;const e=Math.min(1,v.sac.p),s=e*e*(3-2*e);v.th=v.sac.from+v.sac.d*s;
   if(e>=1){
    v.sac=null;v.hold=P.delay+0.2;
    // then a short slow walk toward the lamp it faces, stopping well short of it and of the walls
    let dm=1e9;for(const o of sim.items){const dx=o.x-v.x,dy=o.y-v.y;if(Math.abs(wrapA(Math.atan2(dy,dx)-v.th))<0.25)dm=Math.min(dm,Math.hypot(dx,dy))}
    // (it stays within 110 px of the middle of the arena, so the lamps keep going round it)
    v.go=clamp((dm-70)/33,0,1.8);
    for(let t=0.2;t<=v.go;t+=0.2)if(Math.hypot(v.x+Math.cos(v.th)*33*t-480,v.y+Math.sin(v.th)*33*t-270)>110){v.go=t-0.2;break}
   }
   sim.uh=[];
  }else if(v.go>0){
   v.go-=dt;B.drive(v,0.35,0.35,dt);B.trail(v,dt);sim.uh=[];
  }else{
   v.mL=v.mR=0;
   sim.uh.push(Float32Array.from(u));if(sim.uh.length>L+1)sim.uh.shift();
   if(v.hold>0){v.hold-=dt;sim.uh=[]}
   else if(sim.uh.length>L){
    const past=sim.uh[0];let best=0,bk=0;
    for(let k=0;k<N8;k++)for(let d=1;d<=3;d++){
     if(k+d<N8){const m=Math.min(past[k],u[k+d]);if(m>dR[k])dR[k]=m;if(m>best){best=m;bk=k+d/2}}
     if(k-d>=0){const m=Math.min(past[k],u[k-d]);if(m>dL[k])dL[k]=m;if(m>best){best=m;bk=k-d/2}}
    }
    q.mx=best;
    if(best>0.1){
     const a=-FOV8/2+bk/(N8-1)*FOV8;v.sac={from:v.th,d:a,p:0};sim.stat.sac++;
     // which lamp is it turning to?
     let mvd=0.2,std=0.2;for(const o of sim.items){const e=Math.abs(wrapA(Math.atan2(o.y-v.y,o.x-v.x)-(v.th+a)));if(o.mv)mvd=Math.min(mvd,e);else std=Math.min(std,e)}
     if(mvd<0.2)sim.stat.mv++;else if(std<0.2)sim.stat.st++;
    }
   }
  }
  B.bounce(v);
 },
 render(c,sim){
  const v=sim.vehs[0];c.strokeStyle=C.rule;c.lineWidth=1;c.setLineDash([3,4]);
  for(const o of sim.items)if(o.mv){c.beginPath();c.arc(480,270,o.mv.r,0,TAU);c.stroke()}
  c.setLineDash([]);c.fillStyle='rgba(28,28,26,.04)';c.beginPath();c.moveTo(v.x,v.y);c.arc(v.x,v.y,330,v.th-FOV8/2,v.th+FOV8/2);c.closePath();c.fill();
  c.strokeStyle=C.rule;c.stroke();
 },
 hud:s=>[`turns ${s.stat.sac}   toward a moving lamp ${s.stat.mv}   toward a still lamp ${s.stat.st}`],
 inner(c,sim,w,h){
  const q=sim.r8,x0=150,pw=w-190,rh=34,gap=12;
  const rows=[['photocells',q.I,C.ink,1],['map of things',q.u,C.ink,1],['moving right',q.dR,C.red,1],['moving left',q.dL,C.red,1]];
  rows.forEach(([lab,arr,col,mx],i)=>{const y=12+i*(rh+gap);B.bars(c,x0,y,pw,rh,arr,col,'',mx);B.text(c,lab,x0-10,y+rh/2+4,C.ink,13,'right')});
  B.text(c,'left',x0,h-8,C.mute,13);B.text(c,'right',x0+pw,h-8,C.mute,13,'right');B.text(c,'visual field',x0+pw/2,h-8,C.mute,13,'center');
 }};

/* ---------------- 9: symmetry and periodicity ---------------- */
const N9=96,FOV9=rad(90),K9=40,NL9=24,LAG9=4,BLOB9=3;
// dots of one row, as offsets (px) across the line of sight; the row is turned to face the vehicle
const ROWS={per:[-60,-30,0,30,60],sym:[-72,-60,-30,30,60,72],asym:[-68,-56,-30,28,44,72]};
const ROWN={per:'periodic',sym:'symmetric',asym:'irregular'};
// where a row that has been reached turns up again: 120 px from the walls, far from the vehicle, away from the other rows
function rowSpot(sim,v,self){
  const r=sim.rng;let best=null,bs=-1;
  for(let t=0;t<40;t++){
   const x=120+r()*(W-240),y=120+r()*(H-240);
   let s=Math.min(Math.hypot(x-v.x,y-v.y)-260,0)*3;
   for(const o of sim.items)if(o!==self)s+=Math.min(Math.hypot(x-o.x,y-o.y)-170,0);
   if(s===0)return[x,y];
   if(best===null||s>bs){best=[x,y];bs=s}
  }
  return best;
}
M[9]={name:'Shapes',addKind:'row',
 variants:[
  {k:'a',text:'Detectors can pick out properties of shape. A symmetry detector is a row of threshold devices that fires when the pattern on the retina is mirror-symmetric about its position. The vehicle turns toward the objects the detector responds to and so ends up facing them. An evenly spaced row is symmetric too.',summary:'Turns toward rows of dots that are mirror-symmetric.'},
  {k:'b',text:'A periodicity detector fires when the pattern repeats at some spacing. Evenly spaced rows are periodic, and the symmetric row with uneven gaps is not.',summary:'Turns toward evenly spaced rows of dots.'}],
 params:[{id:'ts',label:'Detector threshold',min:0.2,max:0.95,step:0.01,val:0.6,fmt:x=>x.toFixed(2)}],
 items:()=>[['per',200,150],['sym',720,150],['asym',180,400],['per',760,400],['sym',480,420],['asym',500,125]].map(([t,x,y])=>({kind:'row',type:t,x,y,off:false})),
 add(sim,x,y){sim.items.push({kind:'row',type:['per','sym','asym'][sim.items.length%3],x,y})},
 init(sim){const v=B.mkVeh(480,270,0.3);v.phi=phi40;sim.vehs=[v];sim.stat.per=sim.stat.sym=sim.stat.asym=0;sim.r9={I:new Float32Array(N9),R:new Float32Array(N9),T:new Float32Array(N9),S:new Float32Array(N9),Pd:new Float32Array(NL9),sm:0,sk:-1,pm:0,pk:-1,pl:0,loc:-1,hold:0,mem:null,aim:null,aimT:0}},
 tick(sim,dt){
  const v=sim.vehs[0],q=sim.r9,{I,R,T,S,Pd}=q,P=sim.params;I.fill(0);
  for(const o of sim.items){
   const dx0=o.x-v.x,dy0=o.y-v.y,d0=Math.hypot(dx0,dy0)||1;o.em=[];
   for(const off of ROWS[o.type]){const ex=o.x-dy0/d0*off,ey=o.y+dx0/d0*off;o.em.push([ex,ey]);
    const dx=ex-v.x,dy=ey-v.y,d=Math.hypot(dx,dy),a=wrapA(Math.atan2(dy,dx)-v.th);if(Math.abs(a)>FOV9/2+0.1)continue;
    const pos=(a+FOV9/2)/FOV9*(N9-1),amp=1/(1+d*d/(300*300));
    for(let k=Math.max(0,Math.floor(pos-4));k<=Math.min(N9-1,Math.ceil(pos+4));k++)I[k]+=amp*Math.exp(-((k-pos)**2)/(2*1.1*1.1));}
   if(Math.hypot(dx0,dy0)<30&&q.aim===o&&sim.t-q.aimT<2){sim.stat[o.type]++;const[x,y]=rowSpot(sim,v,o);o.x=x;o.y=y;q.hold=0;q.aim=null}   // arrived at the row it was heading for; rows it merely passes are left alone
  }
  // threshold devices: every bright spot becomes one blob of about equal height, whatever its distance (contrast-normalised)
  let mx=0;for(let k=0;k<N9;k++)if(I[k]>mx)mx=I[k];
  const ref=Math.max(mx,0.55);for(let k=0;k<N9;k++)R[k]=clamp((I[k]/ref-0.15)/0.5);
  // symmetry detectors, one for every position (also between two photocells): how much of the pattern on the left is matched by its mirror image on the right,
  // to within one photocell. A solid bar is trivially symmetric, so the overlap expected by chance (from how filled the pattern is) is subtracted.
  let first=-1,last=-1,tot=0;for(let k=0;k<N9;k++){tot+=R[k];if(R[k]>0.3){if(first<0)first=k;last=k}}
  const at=i=>i>=0&&i<N9?R[i]:0,near=i=>Math.max(at(i-1),at(i),at(i+1));
  let sm=0,sk=-1;S.fill(0);
  if(first>=0)for(let m=2*first;m<=2*last;m++){
   let inter=0,uni=0,filled=0,ext=0;
   for(let dd=m&1;dd<=2*K9;dd+=2){const i=(m-dd)/2,j=(m+dd)/2;if(i<first-1&&j>last+1)break;const a=at(i),b=at(j);
    if(dd===0){inter+=a;uni+=a;filled+=a}else{inter+=0.5*(Math.min(a,near(j))+Math.min(b,near(i)));uni+=a>b?a:b;filled+=a+b;if(a>0.3||b>0.3)ext=dd/2}}
   const matched=(2*inter-((m&1)?0:R[m>>1]))/BLOB9;   // spots on the two sides that have a partner
   let sc=0;
   if(uni>0.5){
    const rho=ext>0?clamp(filled/(2*ext)):1,chance=rho/(2-rho),lo=Math.max(0.45,chance+0.25),hi=Math.max(lo+0.1,Math.min(1,lo+0.4));
    sc=clamp((inter/uni-lo)/(hi-lo))*clamp((matched-2)/2);
   }
   const c=m>>1;if(sc>S[c])S[c]=sc;
   if(sc>sm+1e-6){sm=sc;sk=m/2}
  }
  if(sk>=0){let n=0,d=0;for(let c=Math.max(0,Math.ceil(sk-3));c<=Math.min(N9-1,sk+3);c++)if(S[c]>=sm-0.05){n+=S[c]*c;d+=S[c]}sk=n/d}
  // periodicity detector for each spacing: three spots at equal distances, with a small tolerance. A crowded retina
  // contains such triples by chance, so the chance level (from how much of the pattern is filled) is subtracted.
  const fill=first<0?0:clamp(tot/(last-first+1));
  const triple=(L,out)=>{
   const w=Math.floor(L/12),w3=w+(L>=8?1:0);let s=0;
   for(let k=0;k+2*L-w3<N9;k++){
    if(R[k]<=0){if(out)out[k]=0;continue}
    let a=0,b=0;
    for(let j=-w;j<=w;j++){const i=k+L+j;if(i>=0&&i<N9&&R[i]>a)a=R[i]}
    for(let j=-w3;j<=w3;j++){const i=k+2*L+j;if(i>=0&&i<N9&&R[i]>b)b=R[i]}
    const m=Math.min(R[k],a,b);s+=m;if(out)out[k]=m;
   }
   return s;
  };
  let pm=0,pl=-1;
  for(let l=0;l<NL9;l++){
   const L=l+LAG9,w=Math.floor(L/12),w3=w+(L>=8?1:0),base=Math.min(1,fill*(BLOB9+2*w)/BLOB9)*Math.min(1,fill*(BLOB9+2*w3)/BLOB9);
   Pd[l]=tot>2*BLOB9&&base<0.4?clamp((triple(L)/tot-base)/(0.5-base)):0;
   if(Pd[l]>pm){pm=Pd[l];pl=l}
  }
  // where the periodic pattern is: centre of the triples at the best spacing
  let pk=-1;
  if(pl>=0){
   const L=pl+LAG9;T.fill(0);triple(L,T);
   let best=-1,bv=0;for(let c=0;c<N9;c++){let a=0;for(let k=0;k<N9;k++)if(T[k]>0)a+=T[k]*Math.exp(-((k+L-c)**2)/(2*(1.5*L)**2));if(a>bv){bv=a;best=c}}
   let n=0,d=0;for(let k=0;k<N9;k++)if(T[k]>0&&Math.abs(k+L-best)<=1.5*L){n+=T[k]*(k+L);d+=T[k]}
   pk=d>0?n/d:best;
  }
  q.sm=sm;q.sk=sk;q.pm=pm;q.pk=pk;q.pl=pl;
  const useSym=sim.variant==='a',val=useSym?sm:pm,loc=useSym?sk:pk;q.loc=val>P.ts?loc:-1;
  if(q.loc>=0){const a=v.th-FOV9/2+q.loc/(N9-1)*FOV9;let best=null,bd=0.2;for(const o of sim.items){const e=Math.abs(wrapA(Math.atan2(o.y-v.y,o.x-v.x)-a));if(e<bd){bd=e;best=o}}if(best){q.aim=best;q.aimT=sim.t}}
  // nothing detected: a slow, wavering search; something detected: the side nearer to it is driven less, and the vehicle speeds up
  let[mL,mR]=wander(sim,v,0.4,2);
  if(q.loc>=0){const w=q.loc/(N9-1),g=1.6;mL=0.25+g*(w+0.35);mR=0.25+g*((1-w)+0.35);q.hold=1.2;q.mem=[clamp(mL),clamp(mR)]}
  else if(q.hold>0){q.hold-=dt;[mL,mR]=q.mem}   // close to a row it no longer fits on the retina: keep going for a moment
  B.drive(v,clamp(mL),clamp(mR),dt);B.bounce(v);B.trail(v,dt);
 },
 render(c,sim){
  const v=sim.vehs[0];c.fillStyle='rgba(28,28,26,.04)';c.beginPath();c.moveTo(v.x,v.y);c.arc(v.x,v.y,330,v.th-FOV9/2,v.th+FOV9/2);c.closePath();c.fill();c.strokeStyle=C.rule;c.stroke();
  for(const o of sim.items){if(!o.em)continue;
   c.strokeStyle=C.rule;c.lineWidth=1;c.beginPath();c.moveTo(o.em[0][0],o.em[0][1]);for(const e of o.em)c.lineTo(e[0],e[1]);c.stroke();
   for(const e of o.em){dot(c,e[0],e[1],4.5,C.ink)}
   const lo=o.em.reduce((a,e)=>e[1]>a[1]?e:a,o.em[0]);B.text(c,ROWN[o.type],lo[0],Math.min(lo[1]+18,H-6),C.mute,12,'center')}   // name under the lower end, clear of the dots
  const q=sim.r9;if(q.loc>=0){const a=v.th-FOV9/2+q.loc/(N9-1)*FOV9;c.strokeStyle=C.red;c.lineWidth=1.5;c.setLineDash([5,5]);c.beginPath();c.moveTo(v.x,v.y);c.lineTo(v.x+Math.cos(a)*200,v.y+Math.sin(a)*200);c.stroke();c.setLineDash([])}
 },
 hud:s=>[`drove to periodic ${s.stat.per}   symmetric ${s.stat.sym}   irregular ${s.stat.asym}`],
 inner(c,sim,w,h){
  const q=sim.r9,x0=170,pw=w-220,rh=34,gap=12,ts=sim.params.ts,a=sim.variant==='a';
  const row=(i,lab,arr,wd,col,marks)=>{const y=12+i*(rh+gap);B.bars(c,x0,y,wd,rh,arr,col,'',1,marks);B.text(c,lab,x0-10,y+rh/2+4,C.ink,13,'right');return y};
  row(0,'retina',q.I,pw,C.ink);
  row(1,'threshold devices',q.R,pw,C.ink);
  row(2,'symmetry detectors',q.S,pw,C.red,a&&q.sk>=0&&q.sm>ts?[Math.round(q.sk)]:null);
  const y3=row(3,'periodicity detectors',q.Pd,pw*0.5,C.red);
  B.text(c,'by spacing, narrow to wide',x0,y3+rh+16,C.mute,13);
  const x1=x0+pw*0.5+30;
  B.text(c,`strongest symmetry ${q.sm.toFixed(2)}`,x1,y3+12,a?C.red:C.ink,13);
  B.text(c,`strongest periodicity ${q.pm.toFixed(2)}`,x1,y3+30,a?C.ink:C.red,13);
  B.text(c,`threshold ${ts.toFixed(2)}, the red value steers`,x1,y3+48,C.mute,13);
 }};

/* ---------------- shared by 10 and 11: learning wires ---------------- */
const F10=['light','heat','oxy','org'],F10N=['light','heat','oxygen','organic'];
// the four items lie within ~40 px, so they carry no labels of their own; the model draws one label "thing" per group instead
function thing(g,x,y){return[{kind:'light',x:x,y:y,grp:g,ox:0,oy:0,lab:1,nolabel:true},{kind:'heat',x:x+38,y:y-6,R:55,grp:g,ox:38,oy:-6,nolabel:true},{kind:'oxy',x:x-34,y:y+10,grp:g,ox:-34,oy:10,nolabel:true},{kind:'org',x:x+2,y:y-34,grp:g,ox:2,oy:-34,nolabel:true}]}
M[10]={name:'Getting ideas',addKind:null,
 variants:[{k:'a',text:'Many mnemotrix wires join the units for the features of one thing. Light, warmth, oxygen and organic matter keep turning up together. After many such meetings any one of them, a lone lamp for example, also activates the others, including the organic matter the vehicle goes for.',summary:'Learns that light, warmth, oxygen and organic matter belong together.'}],
 params:[{id:'rate',label:'Learning rate',min:0.02,max:0.4,step:0.01,val:0.1,fmt:x=>x.toFixed(2)},{id:'tau',label:'Forgetting time',min:20,max:400,step:10,val:200,fmt:x=>x+' s'}],
 items:()=>[...thing(0,230,150),...thing(1,720,140),...thing(2,600,420),{kind:'light',x:150,y:420,lone:true},{kind:'heat',x:860,y:340,R:55,lone:true},{kind:'oxy',x:480,y:490,lone:true}],
 onDrag(sim,o){moveGroup(sim,o.grp,o.x-o.ox,o.y-o.oy)},
 render(c,sim){
  // the label stays as long as any item of the group is present (the lamp never goes, only the organic matter)
  for(const g of new Set(sim.items.filter(o=>o.grp!=null).map(o=>o.grp))){
   const its=sim.items.filter(o=>o.grp===g),lamp=its.find(o=>o.kind==='light');
   if(lamp&&its.some(o=>!o.gone))B.text(c,'thing',lamp.x,lamp.y+46,C.mute,12,'center');
  }
 },
 init(sim){const v=B.mkVeh(480,270,0.2);v.phi=phi40;sim.vehs=[v];sim.Wm=[0,1,2,3].map(()=>[0,0,0,0]);sim.a=[0,0,0,0];sim.sd=[0,0,0,0];sim.respawn=[];sim.stat.eaten=0;sim.stat.lone=0},
 tick(sim,dt){
  const P=sim.params,v=sim.vehs[0],Wm=sim.Wm,sL=[],sR=[],a=[];
  F10.forEach((k,i)=>{const[l,r]=B.sens2(v,sim,k,v.phi,-1,110);sL[i]=l;sR[i]=r;a[i]=clamp(Math.max(l,r)*3)});
  sim.a=a;
  for(let i=0;i<4;i++)for(let j=i+1;j<4;j++){const d=dt*(P.rate*a[i]*a[j]*(1-Wm[i][j])-Wm[i][j]/P.tau);Wm[i][j]=Wm[j][i]=clamp(Wm[i][j]+d)}
  let eL=sL[3],eR=sR[3];sim.sd=a.map((x,j)=>{let m=x;for(let i=0;i<4;i++)if(i!==j)m=Math.max(m,Wm[i][j]*a[i]);return m});
  for(let i=0;i<3;i++){eL=Math.max(eL,sL[3]+Wm[i][3]*sL[i]);eR=Math.max(eR,sR[3]+Wm[i][3]*sR[i])}
  v.sL=Math.max(...sL);v.sR=Math.max(...sR);
  const wt=wallTurn(v);
  B.drive(v,clamp(0.22+1.2*eR+wt),clamp(0.22+1.2*eL-wt),dt);B.bounce(v);B.trail(v,dt);
  for(const o of sim.items){
   if(o.kind==='org'&&!o.gone&&Math.hypot(o.x-v.x,o.y-v.y)<45){o.gone=true;o.flash=.5;sim.stat.eaten++;sim.respawn.push({g:o.grp,t:sim.t+4})}
   if(o.lone&&!(o.cd>0)&&Math.hypot(o.x-v.x,o.y-v.y)<28){sim.stat.lone++;o.cd=4;o.flash=.5}if(o.cd>0)o.cd-=dt;
  }
  sim.respawn=sim.respawn.filter(q=>{if(sim.t>=q.t){const[x,y]=farSpot(sim,v);moveGroup(sim,q.g,x,y);for(const o of sim.items)if(o.grp===q.g)o.gone=false;return false}return true});
 },
 hud:s=>[`organic matter eaten ${s.stat.eaten}   lone features visited ${s.stat.lone}`],
 inner(c,sim,w,h){
  // three blocks, nothing beyond x = 920: the matrix (120-300), the sensor bars (380-600), the awareness bars (680-900)
  const x0=130,y0=46,cs=44,COL=['light','heat','O₂','organic'];
  B.text(c,'mnemotrix strengths',x0,y0-18,C.mute,13);
  F10N.forEach((n,i)=>B.text(c,n,x0-8,y0+i*cs+cs/2+4,C.ink,13,'right'));
  COL.forEach((n,i)=>B.text(c,n,x0+i*cs+cs/2-1,y0+4*cs+16,C.ink,13,'center'));
  for(let i=0;i<4;i++)for(let j=0;j<4;j++){
   const val=i===j?0:sim.Wm[i][j];c.fillStyle=C.faint;c.fillRect(x0+j*cs,y0+i*cs,cs-2,cs-2);c.fillStyle=C.red;c.globalAlpha=val;c.fillRect(x0+j*cs,y0+i*cs,cs-2,cs-2);c.globalAlpha=1;
   if(i!==j)B.text(c,val.toFixed(2),x0+j*cs+cs/2-1,y0+i*cs+cs/2+5,val>0.55?'#fff':C.ink,13,'center')}
  const bar=(bx,title,arr,col)=>{
   B.text(c,title,bx,y0-18,C.mute,13);
   F10N.forEach((n,i)=>{const y=y0+i*cs;B.text(c,n,bx,y+cs/2+4,C.ink,13);
    c.fillStyle=C.faint;c.fillRect(bx+65,y+8,155,cs-20);c.fillStyle=col;c.fillRect(bx+65,y+8,155*clamp(arr[i]),cs-20)});
  };
  bar(380,'what the sensors report',sim.a,C.ink);
  bar(680,'what it is aware of, via the wires',sim.sd,C.red);
 }};

/* ---------------- 11: ergotrix ---------------- */
M[11]={name:'Rules and regularities',addKind:null,
 variants:[{k:'a',text:'An ergotrix wire conducts one way only and grows stronger when its input unit is active shortly before its output unit. Here a lamp flashes and about a second later food appears at the same place. The vehicle learns lamp → food. The reverse wire food → lamp stays at zero, and so does the symmetrical mnemotrix, because the two are never active together.',summary:'Learns that food follows the lamp and waits for it there.'}],
 params:[{id:'rate',label:'Learning rate',min:0.1,max:2,step:0.05,val:0.6,fmt:x=>x.toFixed(2)}],
 items:()=>[{kind:'light',x:300,y:200,off:true,fixed:true},{kind:'org',x:326,y:200,gone:true,fixed:true}],
 init(sim){const v=B.mkVeh(480,270,0.1);v.phi=phi40;sim.vehs=[v];sim.E={sf:0,fs:0,m:0};sim.tr={S:0,F:0};sim.cyc={st:'wait',t:1,t0:0};v.until=0;v.place=null;sim.lat=[];sim.stat.eaten=0},
 tick(sim,dt){
  const P=sim.params,v=sim.vehs[0],cy=sim.cyc,lamp=sim.items[0],food=sim.items[1],E=sim.E,tr=sim.tr;
  // world: lamp, 1 s pause, food
  if(cy.st==='wait'&&sim.t>=cy.t){const[x,y]=spotAhead(sim,v,150,210,130);lamp.x=x;lamp.y=y;food.x=lamp.x+26;food.y=lamp.y;lamp.off=false;cy.st='lamp';cy.t=sim.t+1}
  else if(cy.st==='lamp'&&sim.t>=cy.t){lamp.off=true;cy.st='gap';cy.t=sim.t+1}
  else if(cy.st==='gap'&&sim.t>=cy.t){food.gone=false;cy.st='food';cy.t=sim.t+7;cy.t0=sim.t}
  else if(cy.st==='food'&&(food.gone||sim.t>=cy.t)){food.gone=true;cy.st='wait';cy.t=sim.t+3.5}
  const[lL,lR]=B.sens2(v,sim,'light',v.phi,-1,110),[oL,oR]=B.sens2(v,sim,'org',v.phi,-1,110);
  const aS=clamp(Math.max(lL,lR)*3),aF=clamp(Math.max(oL,oR)*3);
  tr.S=Math.max(aS,tr.S*Math.exp(-dt/1.2));tr.F=Math.max(aF,tr.F*Math.exp(-dt/1.2));
  const tS=tr.S>0.3?tr.S:0,tF=tr.F>0.3?tr.F:0;
  E.sf=clamp(E.sf+dt*P.rate*tS*aF*(1-E.sf)*(aS<0.5?1:0));
  E.fs=clamp(E.fs+dt*P.rate*tF*aS*(1-E.fs)*(aF<0.5?1:0));
  E.m=clamp(E.m+dt*P.rate*aS*aF*(1-E.m));
  v.aS=aS;v.aF=aF;v.sL=Math.max(lL,oL);v.sR=Math.max(lR,oR);
  if(aS>0.3&&lamp.off===false){v.place=[lamp.x+13,lamp.y];v.until=sim.t+4}
  if(E.sf>0.3&&v.until>sim.t&&food.gone&&v.place)gotoXY(v,v.place[0],v.place[1],dt,0.9);   // expects food at the lamp's place
  else{const[bl,br]=wander(sim,v,0.22,0.7);B.drive(v,clamp(bl+1.6*oR),clamp(br+1.6*oL),dt)}
  B.bounce(v);B.trail(v,dt);
  if(!food.gone&&Math.hypot(food.x-v.x,food.y-v.y)<22){food.gone=true;food.flash=.5;sim.stat.eaten++;sim.lat.push(sim.t-cy.t0);if(sim.lat.length>8)sim.lat.shift()}
 },
 hud(s){const l=s.lat.length?s.lat.reduce((a,b)=>a+b)/s.lat.length:0;return[`food eaten ${s.stat.eaten}   mean wait (last 8) ${l.toFixed(1)} s`]},
 inner(c,sim,w,h){
  const E=sim.E,v=sim.vehs[0];
  B.graph(c,30,22,560,170,{caption:'S is the lamp unit and F the food unit. Arrows are ergotrix wires (one direction), the plain line is a mnemotrix.',nodes:[
   {id:'S',x:.12,y:.5,label:'S',act:v.aS,shape:'unit',sub:'lamp'},{id:'F',x:.88,y:.5,label:'F',act:v.aF,shape:'unit',sub:'food'}],
   edges:[{a:'S',b:'F',type:'ergo',w:E.sf,act:E.sf>0.05?v.aS:0,label:E.sf.toFixed(2),curve:-26},{a:'F',b:'S',type:'ergo',w:E.fs,act:0,label:E.fs.toFixed(2),curve:-26},{a:'S',b:'F',type:'mnem',w:E.m,act:0,label:E.m.toFixed(2)}]});
  B.text(c,'S → F  '+E.sf.toFixed(2),640,60,C.ink,14);B.text(c,'F → S  '+E.fs.toFixed(2),640,90,C.ink,14);B.text(c,'mnemotrix  '+E.m.toFixed(2),640,120,C.ink,14);
 }};

/* ---------------- 12: trains of thought ---------------- */
const IDEA=[{g:[.5,.5],n:'ahead'},{g:[.3,.62],n:'left'},{g:[.5,.5],n:'ahead'},{g:[.62,.3],n:'right'},{g:[.7,.7],n:'fast'},{g:[.1,.65],n:'sharp left'},{g:[.5,.5],n:'ahead'},{g:[.65,.35],n:'right'}];
const LINK=[[0,1,1.4],[1,2,1.4],[2,3,1.4],[2,5,1.25],[3,4,1.4],[4,0,1.4],[5,6,1.4],[6,7,1.4],[7,0,1.4],[3,6,0.9],[1,3,0.9],[5,7,0.9]];
M[12]={name:'Trains of thought',
 variants:[
  {k:'a',text:'Ideas are units joined in a chain by ergotrix wires, and each is tied to a movement. A global control raises or lowers all thresholds at once. When the threshold is high the outside world is ignored and activity runs along the chain, one idea after the next, a train of thought. The epilepsy inhibitor, a negative feedback from the total activity, keeps just one idea going.',summary:'Runs through a chain of ideas, one movement after another.'},
  {k:'b',text:'Without the epilepsy inhibitor each active idea sets off its successors, and they stay on. The activity spreads along the wires until every unit is on and the vehicle twitches. After a while the units tire and it starts again (not in the book).',summary:'Without the inhibitor the activity spreads until every unit is on.'}],
 params:[{id:'th',label:'Global threshold',min:0.4,max:1.6,step:0.05,val:0.7,fmt:x=>x.toFixed(2)}],
 items:()=>[{kind:'tone',x:820,y:90,off:true,fixed:true,name:'outside'}],
 init(sim){const v=B.mkVeh(480,270,0.2);sim.vehs=[v];sim.act=Array(8).fill(0);sim.lvl=Array(8).fill(0);sim.ct=0;sim.silent=0;sim.full=0;sim.rest=0;sim.cNext=0;sim.cm=[0.5,0.5];sim.stat.thoughts=0;sim.stat.seizures=0},
 tick(sim,dt){
  const v=sim.vehs[0],P=sim.params,tone=sim.items[0],r=sim.rng,inh=sim.variant==='a';
  if(tone.until&&sim.t>tone.until){tone.off=true;tone.until=0}
  // one step of activity: 0.45 s per idea with the inhibitor; 1 s without it, slow enough to watch the activity spread
  const step=inh?0.45:1.0;sim.ct+=dt;
  if(sim.ct>=step){
   sim.ct=0;const a=sim.act,x=Array(8).fill(0);
   for(const[i,j,w]of LINK)x[j]+=w*a[i];
   const n0=a.reduce((p,q)=>p+q,0);
   if(sim.rest>0){sim.rest-=step;x.fill(0)}   // worn out: nothing can start for a while
   else if(!n0){sim.silent+=step;
    if(sim.silent>4){tone.off=false;tone.until=sim.t+0.6;x[0]+=1.0;sim.silent=0}
    else if(sim.silent>2&&r()<0.3&&P.th>1.0)x[Math.floor(r()*8)]+=1.6}
   else sim.silent=0;
   for(let j=0;j<8;j++)x[j]+=gauss(r)*0.06;
   let cand=[];for(let j=0;j<8;j++)if(x[j]>P.th)cand.push(j);
   let na;
   if(inh){if(cand.length>1){cand.sort((p,q)=>x[q]-x[p]);cand=[cand[0]]}na=Array(8).fill(0);cand.forEach(j=>na[j]=1)}
   else{na=a.map((q,j)=>q||cand.includes(j)?1:0)}   // no inhibitor: units that are on stay on, and recruit their successors
   if(cand.length&&!n0)sim.stat.thoughts++;
   if(!inh){
    const n=na.reduce((p,q)=>p+q,0);
    if(n===8){sim.full+=step;if(sim.full>=6){na=Array(8).fill(0);sim.full=0;sim.rest=3;sim.stat.seizures++}}else sim.full=0;
   }
   sim.act=na;
  }
  for(let i=0;i<8;i++)sim.lvl[i]=sim.act[i]?1:Math.max(0,sim.lvl[i]-dt/0.6);
  const on=sim.act.map((q,i)=>q?i:-1).filter(i=>i>=0);
  let mL=0.12,mR=0.12;
  if(on.length>3){
   // all the movements at once: new random wheel speeds a few times a second
   if(sim.t>=sim.cNext){sim.cm=[r(),r()];sim.cNext=sim.t+0.25}
   [mL,mR]=sim.cm;sim.chaos=true;
  }else{
   sim.chaos=false;
   if(on.length){mL=on.reduce((s,i)=>s+IDEA[i].g[0],0)/on.length;mR=on.reduce((s,i)=>s+IDEA[i].g[1],0)/on.length}
   const t=wallTurn(v);mL+=t;mR-=t;
  }
  B.drive(v,clamp(mL),clamp(mR),dt);B.bounce(v);B.trail(v,dt);v.sL=v.sR=0;
  tone.name=tone.off?'outside':'outside stimulus';   // the label under the tone marker says so while the tone is on
 },
 render(c,sim){
  // the idea that is active now, next to the vehicle; many at once: one common label; none: nothing
  const v=sim.vehs[0],on=sim.act.map((q,i)=>q?i:-1).filter(i=>i>=0);
  const s=on.length>3?'all units active':[...new Set(on.map(i=>IDEA[i].n))].join(' + ');
  if(s)B.text(c,s,v.x,v.y-26,C.ink,13,'center');
 },
 hud(s){
  const n=s.act.reduce((p,q)=>p+q,0);
  if(s.variant==='a')return[s.chaos?'all units active':n?'idea '+(s.act.indexOf(1)+1)+' active':'quiet'];
  return[n===8?'all 8 units active':n?`${n} of 8 units active`:s.rest>0?'the units are worn out':'quiet'];
 },
 inner(c,sim,w,h){
  const nodes=IDEA.map((q,i)=>{const a=-Math.PI/2+i*TAU/8;return{id:'i'+i,x:.5+0.37*(h/w)*1.55*Math.cos(a),y:.5+0.36*Math.sin(a),label:String(i+1),act:sim.lvl[i],shape:'unit',sub:null}});
  const edges=LINK.map(([i,j,wt])=>({a:'i'+i,b:'i'+j,type:'ergo',w:wt/2,act:sim.lvl[i]&&sim.lvl[j]?1:sim.lvl[i]*0.4,curve:wt<1?-14:0}));
  B.graph(c,150,12,330,h-34,{R:13,nodes,edges,caption:''});
  B.text(c,'ideas, each tied to a movement:',540,32,C.mute,13);
  IDEA.forEach((q,i)=>B.text(c,(i+1)+'  '+q.n,540+(i%2)*150,58+Math.floor(i/2)*22,sim.act[i]?C.red:C.ink,13));
  const n=sim.act.reduce((p,q)=>p+q,0);
  B.text(c,'global threshold '+sim.params.th.toFixed(2)+(sim.variant==='a'?'   inhibitor on':'   inhibitor off'),540,166,C.ink,13);
  B.text(c,'outside stimulus passes if the threshold is below 1.0',540,188,C.mute,13);
  if(sim.variant==='b')B.text(c,`${n} of 8 units active`+(sim.rest>0?'   worn out':''),540,214,n===8?C.red:C.ink,13);
 }};

/* ---------------- 13: foresight ---------------- */
const NS=48,CX=480,CY=275,RO=165,DELAY13=54;   // both vehicles see the lamp 54 frames = 0.9 s late
M[13]={name:'Foresight',
 variants:[
  {k:'a',text:'Ergotrix wires have learned which place the lamp reaches after which. The red vehicle heads for where the predictor expects the lamp, and the grey one for where it saw it. Both see the lamp 0.9 s late, and if expectation and input disagree, the sensors win. After the lamp reverses, the red vehicle is no better than the grey one until the wires are learned again.',summary:'The predicting vehicle aims where the lamp will be, the reacting one where it was.'},
  {k:'b',text:'A fast ergotrix works as short-term memory. A strongly unpleasant event, the bang, activates an evaluator D, which makes the predictor replay the two events before it, tone A then tone B. One bang is enough for the vehicle to expect it and to run when it hears A or B.',summary:'After one bang it expects it and runs when it hears the tones.'},
  {k:'c',text:'The same as 13b but without the evaluator. The two wires grow only a little with each bang, so the vehicle needs many repetitions before it runs in time.',summary:'Needs many bangs before it learns to run when it hears the tones.'}],
 params:[{id:'ahead',label:'Look ahead',min:0,max:6,step:1,val:4,fmt:x=>x+' steps',show:v=>v==='a'}],
 items:v=>v==='a'?[{kind:'light',x:CX+RO,y:CY,fixed:true,nolabel:true}]:[{kind:'tone',x:260,y:130,off:true,fixed:true,name:'tone A'},{kind:'tone',x:700,y:130,off:true,fixed:true,name:'tone B'}],
 init(sim){
  if(sim.variant==='a'){
   const p=B.mkVeh(CX-60,CY+150,0);p.phi=phi40;p.fill=C.red;const r=B.mkVeh(CX+60,CY+160,2);r.phi=phi40;r.tc=C.mute;sim.vehs=[p,r];
   sim.ang=0;sim.angH=[0];sim.dir=1;sim.flip=50;sim.E=Array.from({length:NS},()=>new Float32Array(NS));sim.cur=0;sim.exp=-1;sim.conf=0;sim.hops=0;sim.dist=[[],[]];sim.stat.mis=0;
  }else{
   const v=B.mkVeh(480,400,-1.2);v.phi=phi40;v.flee=0;sim.vehs=[v];sim.bE={ab:0,bp:0};sim.tr={A:0,B:0};sim.cy={t:2,fired:[0,0,0]};sim.D=0;sim.stat.hurt=0;sim.stat.safe=0;sim.hurtFlash=0;v.bias=0;
  }
 },
 tick(sim,dt){
  if(sim.variant==='a')return tickA(sim,dt);
  const v=sim.vehs[0],cy=sim.cy,E=sim.bE,tr=sim.tr,tA=sim.items[0],tB=sim.items[1],useD=sim.variant==='b';
  const u=sim.t-cy.t;
  tA.off=!(u>=0&&u<0.6);tB.off=!(u>=1.2&&u<1.8);
  const aA=!tA.off?1:0,aB=!tB.off?1:0;
  tr.A=Math.max(aA,tr.A*Math.exp(-dt/1.6));tr.B=Math.max(aB,tr.B*Math.exp(-dt/1.6));
  const learn=0.08,D=0.55;
  if(aB&&!cy.fired[1]){cy.fired[1]=1;E.ab+=learn*tr.A*(1-E.ab)}
  // expectation of the bang, from the wires
  let expP=0;if(aA&&E.ab*E.bp>expP)expP=E.ab*E.bp;if(aB&&E.bp>expP)expP=E.bp;
  if(expP>0.4&&v.flee<=0&&sim.t-cy.t<2.4)v.flee=3.2;
  if(u>=2.4&&!cy.fired[2]){
   cy.fired[2]=1;sim.hurtFlash=0.5;
   if(v.flee>0)sim.stat.safe++;else sim.stat.hurt++;
   E.bp=clamp(E.bp+learn*tr.B*(1-E.bp));
   if(useD){sim.D=1;E.ab=clamp(E.ab+D*(1-E.ab));E.bp=clamp(E.bp+D*(1-E.bp))}
  }
  if(u>=6.5){cy.t=sim.t+0.5;cy.fired=[0,0,0]}
  sim.D=Math.max(0,sim.D-dt*1.2);sim.hurtFlash=Math.max(0,sim.hurtFlash-dt);v.flee=Math.max(0,v.flee-dt);
  sim.exp=expP;
  v.bias+=(-v.bias*0.6)*dt+0.12*Math.sqrt(dt)*gauss(sim.rng);
  const wt=wallTurn(v);
  if(v.flee>0)B.drive(v,clamp(1+wt),clamp(1-wt),dt);else B.drive(v,clamp(0.2+v.bias+wt),clamp(0.2-v.bias-wt),dt);
  B.bounce(v);B.trail(v,dt);
 },
 render(c,sim){
  if(sim.variant==='a'){
   c.strokeStyle=C.rule;c.lineWidth=1;c.setLineDash([3,4]);c.beginPath();c.arc(CX,CY,RO,0,TAU);c.stroke();c.setLineDash([]);
   {const sx=CX+RO*Math.cos(sim.angH[0]),sy=CY+RO*Math.sin(sim.angH[0]);c.strokeStyle=C.mute;c.lineWidth=1.4;c.setLineDash([3,3]);c.beginPath();c.arc(sx,sy,8,0,TAU);c.stroke();c.setLineDash([]);B.text(c,'as seen 0.9 s ago',sx+13,sy+4,C.mute,12,'left')}
   if(sim.tgt){c.strokeStyle=C.red;c.lineWidth=1.6;c.setLineDash([4,3]);c.beginPath();c.arc(sim.tgt[0],sim.tgt[1],9,0,TAU);c.stroke();c.setLineDash([]);B.text(c,sim.hops>0?'expected':'believes sensors',sim.tgt[0],sim.tgt[1]-15,C.red,12,'center')}
  }else{
   const v=sim.vehs[0];if(sim.hurtFlash>0){c.strokeStyle=C.red;c.lineWidth=2;for(let i=0;i<10;i++){const a=i*TAU/10;c.beginPath();c.moveTo(v.x+Math.cos(a)*24,v.y+Math.sin(a)*24);c.lineTo(v.x+Math.cos(a)*(34+sim.hurtFlash*30),v.y+Math.sin(a)*(34+sim.hurtFlash*30));c.stroke()}}
   if(v.flee>0)B.text(c,'runs',v.x,v.y-30,C.red,12,'center');
  }
 },
 renderTop(c,sim){if(sim.variant==='a')B.drawItem(c,sim.items[0],sim)},   // the lamp stays visible when a vehicle sits on it
 hud(s){
  if(s.variant==='a'){const m=i=>s.dist[i].length?s.dist[i].reduce((a,b)=>a+b)/s.dist[i].length:0,d=i=>Math.hypot(s.vehs[i].x-s.items[0].x,s.vehs[i].y-s.items[0].y);return[`red, predicting   ${d(0).toFixed(0)} px from the lamp   mean over 20 s ${m(0).toFixed(0)} px`,`grey, reacting   ${d(1).toFixed(0)} px from the lamp   mean over 20 s ${m(1).toFixed(0)} px`]}
  return[`bangs taken ${s.stat.hurt}   bangs avoided ${s.stat.safe}`]},
 inner(c,sim,w,h){
  if(sim.variant==='a'){
   const q=sim.dist;B.text(c,'sectors visited by the lamp, with the learned ergotrix wire to the next sector',40,24,C.mute,13);
   const cell=(w-80)/NS;for(let i=0;i<NS;i++){const j=sim.E[i].indexOf(Math.max(...sim.E[i]));const val=sim.E[i][j];
    c.fillStyle=C.faint;c.fillRect(40+i*cell,40,cell-2,34);c.fillStyle=C.red;c.globalAlpha=val;c.fillRect(40+i*cell,40,cell-2,34);c.globalAlpha=1;
    if(i===sim.cur){c.strokeStyle=C.ink;c.lineWidth=2;c.strokeRect(40+i*cell,38,cell-2,38)}}
   B.text(c,'1',40,94,C.mute,13);B.text(c,String(NS),40+NS*cell,94,C.mute,13,'right');
   B.text(c,`expectation confidence ${sim.conf.toFixed(2)}   mismatches with the sensors ${sim.stat.mis}`,40,130,C.ink,13);
   B.text(c,'The black frame marks the sector where the lamp is now. Red shows how strongly the next sector is expected.',40,152,C.mute,13);
   return;
  }
  const E=sim.bE,v=sim.vehs[0];
  B.graph(c,60,20,620,180,{caption:'A and B are tone units, P is pain, and D is the evaluator that replays A and B when P fires',nodes:[
   {id:'A',x:.08,y:.5,label:'A',act:sim.items[0].off?0:1,shape:'unit'},{id:'B',x:.4,y:.5,label:'B',act:sim.items[1].off?0:1,shape:'unit'},{id:'P',x:.72,y:.5,label:'P',act:sim.hurtFlash>0?1:0,shape:'unit'},
   {id:'D',x:.56,y:.02,label:'D',act:sim.D,shape:'unit'},{id:'R',x:.94,y:.5,label:'run',act:v.flee>0?1:0,shape:'out'}],
   edges:[{a:'A',b:'B',type:'ergo',w:E.ab,act:E.ab>0.05?1:0,label:E.ab.toFixed(2)},{a:'B',b:'P',type:'ergo',w:E.bp,act:E.bp>0.05?1:0,label:E.bp.toFixed(2)},{a:'P',b:'D',sign:1,act:sim.D},{a:'P',b:'R',sign:1,act:0,curve:0},
   ...(sim.variant==='b'?[]:[])]});
  B.text(c,'expected bang '+sim.exp.toFixed(2)+(sim.exp>0.4?'  → runs':''),720,70,C.ink,13);
  B.text(c,sim.variant==='b'?'evaluator D on':'evaluator D off',720,100,C.mute,13);
 }};
function tickA(sim,dt){
  const p=sim.vehs[0],r=sim.vehs[1],lamp=sim.items[0];
  if(sim.t>sim.flip){sim.dir*=-1;sim.flip=sim.t+70+sim.rng()*10}
  sim.ang+=sim.dir*0.38*dt;lamp.x=CX+RO*Math.cos(sim.ang);lamp.y=CY+RO*Math.sin(sim.ang);
  // both vehicles perceive the lamp 0.9 s late
  sim.angH.push(sim.ang);if(sim.angH.length>DELAY13+1)sim.angH.shift();const seen=sim.angH[0];
  const sec=Math.floor(((seen%TAU)+TAU)%TAU/TAU*NS),E=sim.E;
  if(sec!==sim.cur){
   const wrong=sim.exp>=0&&sec!==sim.exp;
   if(wrong){sim.conf=0;sim.stat.mis++;E[sim.cur][sim.exp]*=0.2}else sim.conf=Math.min(1,sim.conf+0.34);
   E[sim.cur][sec]+=0.6*(1-E[sim.cur][sec]);
   sim.cur=sec;
   let best=-1,bv=0.3;for(let k=0;k<NS;k++)if(E[sec][k]>bv){bv=E[sec][k];best=k}sim.exp=best;
  }
  let s=sec,hops=0;const ahead=sim.conf>=0.5?sim.params.ahead:0;
  for(let i=0;i<ahead;i++){let best=-1,bv=0.3;for(let k=0;k<NS;k++)if(E[s][k]>bv){bv=E[s][k];best=k}if(best<0)break;s=best;hops++}
  sim.hops=hops;
  const tang=(s+0.5)/NS*TAU,tx=CX+RO*Math.cos(tang),ty=CY+RO*Math.sin(tang);sim.tgt=[tx,ty];
  gotoXY(p,tx,ty,dt,1);B.bounce(p);B.trail(p,dt);
  gotoXY(r,CX+RO*Math.cos(seen),CY+RO*Math.sin(seen),dt,1);B.bounce(r);B.trail(r,dt);
  [p,r].forEach((v,i)=>{if(Math.floor(sim.t*2)!==Math.floor((sim.t-dt)*2)){sim.dist[i].push(Math.hypot(v.x-lamp.x,v.y-lamp.y));if(sim.dist[i].length>40)sim.dist[i].shift()}});
}

/* ---------------- 14: egotism and optimism ---------------- */
const GOOD=[190,150],BAD=[770,150],HOME=[480,430];
M[14]={name:'Egotism and optimism',
 variants:[{k:'a',text:'When several next states are about equally likely, the vehicle takes the most pleasant one. After the tone, food may turn up at the left or a heat source at the right. The predictor has counted how often each came. The vehicle goes to the left whenever the two are within the tolerance of each other, however the earlier trials went.',summary:'Goes to the pleasant side whenever both outcomes are about equally likely.'}],
 params:[{id:'pg',label:'Chance of food',min:0.1,max:0.9,step:0.05,val:0.5,fmt:x=>Math.round(x*100)+' %'},{id:'eps',label:'Tolerance',min:0,max:0.6,step:0.05,val:0.1,fmt:x=>x.toFixed(2)}],
 items:()=>[{kind:'tone',x:480,y:100,off:true,fixed:true,name:'tone'},{kind:'org',x:GOOD[0],y:GOOD[1],gone:true,fixed:true},{kind:'heat',x:BAD[0],y:BAD[1],R:50,gone:true,fixed:true}],
 init(sim){const v=B.mkVeh(HOME[0],HOME[1],-1.57);sim.vehs=[v];sim.cnt={g:1,b:1};sim.ph={st:'home',t:1.5};sim.choice=null;sim.stat.good=0;sim.stat.bad=0;sim.stat.trials=0;sim.last=''},
 tick(sim,dt){
  const v=sim.vehs[0],ph=sim.ph,P=sim.params,tone=sim.items[0],food=sim.items[1],heat=sim.items[2],c=sim.cnt;
  const pg=c.g/(c.g+c.b),pb=1-pg;
  if(ph.st==='home'){gotoXY(v,HOME[0],HOME[1],dt,0.8);if(sim.t>ph.t){tone.off=false;ph.st='cue';ph.t=sim.t+0.8;
    const pmax=Math.max(pg,pb),cand=[];if(pg>=pmax-P.eps)cand.push('good');if(pb>=pmax-P.eps)cand.push('bad');
    sim.choice=cand.includes('good')?'good':'bad';sim.cand=cand}}
  else if(ph.st==='cue'){gotoXY(v,HOME[0],HOME[1],dt,0.8);if(sim.t>ph.t){tone.off=true;ph.st='go'}}
  else if(ph.st==='go'){const T=sim.choice==='good'?GOOD:BAD;gotoXY(v,T[0]+(sim.choice==='good'?30:0),T[1]+50,dt,0.9);
    if(Math.hypot(v.x-T[0],v.y-T[1]-50)<40||(!ph.t2&&(ph.t2=sim.t+4))&&false){}
    if(!ph.arr&&Math.hypot(v.x-(T[0]+(sim.choice==='good'?30:0)),v.y-(T[1]+50))<25){ph.arr=sim.t}
    if(ph.arr&&sim.t>ph.arr+1.2){
     const good=sim.rng()<P.pg;if(good){food.gone=false;c.g++}else{heat.gone=false;c.b++}sim.stat.trials++;
     const hit=(good&&sim.choice==='good')||(!good&&sim.choice==='bad');
     sim.last=good?(hit?'food, reached':'food, missed'):(hit?'heat, burnt':'heat, avoided');
     if(good&&hit)sim.stat.good++;if(!good&&hit)sim.stat.bad++;
     if(good&&hit)food.flash=.5;ph.st='out';ph.t=sim.t+2.5;ph.arr=0}}
  else if(ph.st==='out'){if(sim.t>ph.t){food.gone=true;heat.gone=true;ph.st='home';ph.t=sim.t+2.5}}
  B.bounce(v);B.trail(v,dt);
 },
 render(c,sim){
  B.text(c,'food (pleasant)',GOOD[0],GOOD[1]+86,C.olive,12,'center');B.text(c,'heat (unpleasant)',BAD[0],BAD[1]+86,C.red,12,'center');
  c.strokeStyle=C.rule;c.lineWidth=1;c.setLineDash([3,4]);for(const T of[GOOD,BAD]){c.beginPath();c.arc(T[0]+(T===GOOD?30:0),T[1]+50,18,0,TAU);c.stroke()}c.setLineDash([]);
 },
 hud:s=>[`trials ${s.stat.trials}   food reached ${s.stat.good}   burnt ${s.stat.bad}`+(s.last?`   last trial ${s.last}`:'')],
 inner(c,sim,w,h){
  const cn=sim.cnt,pg=cn.g/(cn.g+cn.b),pb=1-pg,eps=sim.params.eps;
  B.text(c,'predicted next states after the tone',40,24,C.mute,13);
  const rows=[['food at the left',pg,'pleasant  +1',C.olive],['heat at the right',pb,'unpleasant  −1',C.red]];
  rows.forEach(([n,p,pl,col],i)=>{const y=44+i*58;B.text(c,n,40,y+16,C.ink,13);c.fillStyle=C.faint;c.fillRect(190,y,300,22);c.fillStyle=col;c.fillRect(190,y,300*p,22);B.text(c,Math.round(p*100)+' % of past trials',500,y+16,C.ink,13);B.text(c,pl,660,y+16,C.mute,13)});
  const pmax=Math.max(pg,pb);
  B.text(c,`candidates are the next states within ${eps.toFixed(2)} of the most likely one → ${(pg>=pmax-eps?'food ':'')}${(pb>=pmax-eps?'heat':'')}`,40,176,C.ink,13);
  B.text(c,`the vehicle picks the most pleasant candidate → ${sim.choice==='good'?'goes left':sim.choice==='bad'?'goes right':'–'}`,40,200,C.red,13);
 }};
})();
