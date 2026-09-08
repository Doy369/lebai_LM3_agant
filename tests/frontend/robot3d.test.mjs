import test from 'node:test';
import assert from 'node:assert/strict';
import {readFile} from 'node:fs/promises';
import {number, parseTrajectory, degToRad, radToDeg, mmToM, zeroPose} from '../../robot_system/web/static/robot3d/validation.js';
import {Vector3, Group, Quaternion, Box3} from '../../robot_system/web/static/robot3d/vendor/three.module.js';
import {GLTFLoader} from '../../robot_system/web/static/robot3d/vendor/GLTFLoader.js';
import {scene_T_base, base_T_model, base_T_tcp, optical_T_threeCamera, matrixFromRows} from '../../robot_system/web/static/robot3d/coordinates.js';
import {JointRig} from '../../robot_system/web/static/robot3d/joints.js';
import {LatestResource, inspectGLB, disposeGraph} from '../../robot_system/web/static/robot3d/model-loader.js';
import {PoseState} from '../../robot_system/web/static/robot3d/state.js';
import {decodeFeedback, FeedbackSync} from '../../robot_system/web/static/robot3d/sync.js';
import {DEFAULT_CONFIG, validateConfig, saveConfig, loadConfig} from '../../robot_system/web/static/robot3d/config.js';
const close = (a,b) => assert.ok(Math.abs(a-b)<1e-6,`${a} != ${b}`);
const fixture = (timestamp=1000) => ({success:true,data:{is_connected:true,dry_run:true,joint_unit:'radian',feedback_time_source:'server_read_completion',feedback_read_at_unix_ms:timestamp,kin_data:{actual_joint_pose:[1,2,3,4,5,6]}}});
test('boundary conversions and strict numeric inputs', () => {
  close(degToRad(180),Math.PI); close(radToDeg(Math.PI/2),90); close(mmToM(350),.35);
  for(const v of ['', ' ', 'Infinity', Infinity, NaN, null, true, [], '0x10','1abc']) assert.throws(()=>number(v,'test',-180,180));
  assert.throws(()=>number('181','test',-180,180)); assert.equal(number('-180','test',-180,180),-180);
});
test('trajectory rejects empty values, malformed segments, duplicates, range errors before playback', () => {
  for(const s of ['', '关节1:', '关节1:Infinity;', '关节1:181;关节1:0;', '关节7:3;', '夹爪:-1;', '关节1:1,关节1:2;', '关节1:1;;关节1:0']) assert.throws(()=>parseTrajectory(s,1000),s);
  assert.throws(()=>parseTrajectory('关节1:0;',0));
  close(parseTrajectory('关节1:90,夹爪:100;',1000)[0].joints[0],Math.PI/2);
});
test('Base Z-up → scene Y-up; model centering is not a physical transform', () => {
  const p=new Vector3(1,2,3).applyMatrix4(scene_T_base); close(p.x,1);close(p.y,3);close(p.z,-2);
  p.applyMatrix4(scene_T_base.clone().invert());close(p.y,2);
  const m=scene_T_base.clone().multiply(base_T_model);close(m.determinant(),1);
  const z=new Vector3(0,0,-1).applyMatrix4(optical_T_threeCamera);close(z.z,1);
});
test('TCP uses Rz Ry Rx and matrix validator rejects reflection/scaling/nonfinite', () => {
  const m=base_T_tcp({x:1,y:2,z:3,rx:0,ry:0,rz:Math.PI/2});
  const p=new Vector3(1,0,0).applyMatrix4(m);close(p.x,1);close(p.y,3);close(p.z,3);
  for(const a of [2,-1,NaN]) assert.throws(()=>matrixFromRows([[a,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]));
});
test('joint mapping composes imported rotation and preserves translation and scale', () => {
  const root=new Group();let parent=root;const nodes=[];
  for(let i=1;i<=6;i++){const n=new Group();n.name=`Joint${i}`;n.position.set(.1,.2,.3);n.scale.set(2,2,2);parent.add(n);parent=n;nodes.push(n);}
  nodes[0].quaternion.setFromAxisAngle(new Vector3(1,0,0),.7);
  const original=nodes[0].quaternion.clone();const rig=new JointRig({scene:root,animations:[]});
  rig.apply({joints:[Math.PI/2,0,0,0,0,0],gripper:0});
  const expected=original.clone().multiply(new Quaternion().setFromAxisAngle(new Vector3(0,1,0),Math.PI/2));
  close(Math.abs(expected.dot(nodes[0].quaternion)),1); assert.deepEqual(nodes[0].position.toArray(),[.1,.2,.3]);assert.equal(nodes[0].scale.x,2);
  rig.apply(zeroPose());close(Math.abs(original.dot(nodes[0].quaternion)),1);
});
test('real bundled GLB: hierarchy, six gripper tracks, motion isolated from arm', async () => {
  const b=await readFile(new URL('../../robot_system/web/static/robot3d/assets/Lebai_LM3.glb',import.meta.url));
  const buffer=b.buffer.slice(b.byteOffset,b.byteOffset+b.byteLength); const j=inspectGLB(buffer);assert.equal(j.nodes.length,73);
  const gltf=await new GLTFLoader().parseAsync(buffer,'');const rig=new JointRig(gltf);
  assert.equal(rig.clip.tracks.length,6);close(rig.endTimeSec,.833333333);
  const p={joints:[.2,-.6,.1,.2,.3,.4],gripper:0};rig.apply(p);
  const q=rig.joints.map(n=>n.node.quaternion.clone());const claw=gltf.scene.getObjectByName('polySurface36').quaternion.clone();
  rig.apply({...p,gripper:1});rig.joints.forEach((n,i)=>close(Math.abs(q[i].dot(n.node.quaternion)),1));
  assert.ok(Math.abs(claw.dot(gltf.scene.getObjectByName('polySurface36').quaternion))<.999);
  assert.ok(new Box3().setFromObject(gltf.scene).getSize(new Vector3()).length()>.5);
  rig.dispose();disposeGraph(gltf.scene);
});
test('single writer modes, independent numeric interpolation, reliable stop and reset', () => {
  const s=new PoseState();s.play(parseTrajectory('关节1:90;关节1:0;',1000),100);
  close(s.current(600).joints[0],Math.PI/4);s.player.stop();close(s.current(1700).joints[0],Math.PI/4);
  s.reset();assert.deepEqual(s.current(1800),zeroPose());
  s.play(parseTrajectory('关节1:90;',1000),2000);s.setMode('feedback');assert.equal(s.player.running,false);
  assert.equal(s.current(2500),null);assert.throws(()=>s.setManual(zeroPose()));assert.throws(()=>s.play([],0));
  s.acceptFeedback([4,0,0,0,0,0]);assert.equal(s.current(3000).joints[0],4); // feedback is never slider-clamped
  s.setMode('manual');assert.deepEqual(s.current(4000),zeroPose());
});
test('trajectory reaches exact final target and finishes across skipped frames', () => {
  const s=new PoseState();s.play(parseTrajectory('关节1:90;关节1:-90,夹爪:100;',1000),0);
  close(s.current(20000).joints[0],-Math.PI/2);assert.equal(s.manual.gripper,1);assert.equal(s.player.running,false);
});
test('feedback validates schema, stale age, timestamps and dry-run provenance', () => {
  const s=decodeFeedback(fixture(),1200,1100);assert.equal(s.source,'simulated');assert.equal(s.ageMs,200);
  assert.throws(()=>decodeFeedback(fixture(),1200,1100,1000));assert.throws(()=>decodeFeedback(fixture(5000),1200,1100));
  for(const key of ['is_connected','kin_data']){const f=fixture();delete f.data[key];assert.throws(()=>decodeFeedback(f,1200,1100));}
  const old=fixture();delete old.data.feedback_read_at_unix_ms;assert.equal(decodeFeedback(old,1200,1100).timed,false);
});
test('feedback watchdog ages frozen samples and shows disconnect independently of pose', () => {
  let now=1000;const sync=new FeedbackSync({read:async()=>fixture(),onSample:()=>{},onStatus:()=>{},now:()=>now});
  sync.active=true;sync.sample=decodeFeedback(fixture(),1000,900);assert.equal(sync.status().kind,'simulated');
  now=5000;assert.equal(sync.status().kind,'stale');sync.error='offline';assert.equal(sync.status().kind,'disconnected');sync.stop();
});
test('late feedback cannot write after leaving mode; request is aborted', async () => {
  let resolve, signal, writes=0;
  const sync=new FeedbackSync({read:s=>{signal=s;return new Promise(r=>resolve=r);},onSample:()=>writes++,onStatus:()=>{}});
  sync.start();sync.stop();resolve(fixture());await new Promise(r=>setImmediate(r));assert.equal(writes,0);assert.equal(signal.aborted,true);
});
test('latest model wins, failed replacement keeps old, dispose invalidates pending load', async () => {
  const disposed=[];const slot=new LatestResource(v=>disposed.push(v));await slot.replace(async()=> 'original');
  await assert.rejects(slot.replace(async()=>{throw Error('bad');}));assert.equal(slot.current,'original');
  let slow;const pending=slot.replace(()=>new Promise(r=>slow=r));await slot.replace(async()=> 'new');slow('old-request');
  assert.equal(await pending,false);assert.equal(slot.current,'new');assert.deepEqual(disposed,['original','old-request']);
  let late;const closing=slot.replace(()=>new Promise(r=>late=r));slot.dispose();late('after-close');await closing;
  assert.equal(slot.current,null);assert.deepEqual(disposed.slice(-2),['new','after-close']);
});
test('GLB rejects text glTF, truncated chunks and external resources before parsing', () => {
  assert.throws(()=>inspectGLB(new TextEncoder().encode('{}').buffer));
  const data=new TextEncoder().encode(JSON.stringify({asset:{version:'2.0'},buffers:[{uri:'model.bin'}]}));
  const b=new ArrayBuffer(20+data.length),v=new DataView(b);[0x46546c67,2,b.byteLength,data.length,0x4e4f534a].forEach((n,i)=>v.setUint32(i*4,n,true));new Uint8Array(b,20).set(data);
  assert.throws(()=>inspectGLB(b),/外部/);v.setUint32(12,999999,true);assert.throws(()=>inspectGLB(b),/长度/);
});
test('versioned config roundtrip is validated and cannot claim camera calibration', () => {
  let raw;const storage={setItem:(k,v)=>raw=v,getItem:()=>raw};saveConfig(DEFAULT_CONFIG,storage);assert.deepEqual(loadConfig(storage),DEFAULT_CONFIG);
  for(const mutate of [c=>c.version=2,c=>c.units.length='mm',c=>c.camera.mode='calibrated',c=>c.target.sizeM[0]=0,c=>c.preview.joints[0]=Infinity,c=>c.camera.targetBaseM=c.camera.positionBaseM]) {
    const c=structuredClone(DEFAULT_CONFIG);mutate(c);assert.throws(()=>validateConfig(c));
  }
});
