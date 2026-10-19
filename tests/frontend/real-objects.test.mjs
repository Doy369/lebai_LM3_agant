import test from 'node:test';
import assert from 'node:assert/strict';
import { readFile } from 'node:fs/promises';
import { parseRealObjects, realObjectsToBodies } from '../../robot_system/web/static/robot3d/real-objects.js';
import { LivePhysics } from '../../robot_system/web/static/robot3d/live-physics.js';
import { Group } from '../../robot_system/web/static/robot3d/vendor/three.module.js';
import { GLTFLoader } from '../../robot_system/web/static/robot3d/vendor/GLTFLoader.js';
import { JointRig, UPRIGHT_REFERENCE_JOINTS_RAD } from '../../robot_system/web/static/robot3d/joints.js';
import { scene_T_base, base_T_model } from '../../robot_system/web/static/robot3d/coordinates.js';
import { RobotSolids } from '../../robot_system/web/static/robot3d/solid-collisions.js';
import { VirtualGrasp } from '../../robot_system/web/static/robot3d/virtual-grasp.js';
import { planVirtualMotion } from '../../robot_system/web/static/robot3d/virtual-motion.js';
import { disposeGraph } from '../../robot_system/web/static/robot3d/model-loader.js';
import { GripperCamera } from '../../robot_system/web/static/robot3d/gripper-camera.js';

const sample = JSON.parse(await readFile(new URL('../../robot_system/web/static/robot3d/examples/real-objects-sample.json', import.meta.url), 'utf8'));
const near = (a, b) => assert.ok(Math.abs(a - b) < 1e-8, `${a} != ${b}`);

test('single-image detection keeps object label and color in the virtual body', () => {
  const capture = parseRealObjects({...sample, source: 'real-camera', objects: [{...sample.objects[0], label: '香蕉', color: '#efc52d', sizeM: [.15, .08, .04]}]});
  const body = realObjectsToBodies(capture)[0];
  assert.equal(body.label, '香蕉'); assert.equal(body.color, '#efc52d');
  assert.deepEqual(body.sizeM, [.15, .08, .04]);
  assert.throws(() => parseRealObjects({...sample, objects: [{...sample.objects[0], color: 'javascript:alert(1)'}]}), /颜色/);
});

test('raw camera plan uses measured surface point, never compensated flange point', () => {
  const plan = {success: true, surface_point_base_m: {x: -.3, y: -.24, z: .04}, point_base_m: {x: -.3, y: -.24, z: .225}, decision: {selected_object_name: '绿色物块'}};
  const value = parseRealObjects({success: true, data: plan});
  assert.deepEqual(value.objects[0].positionBaseM, [-.3, -.24, .04]); assert.equal(value.source, 'import');
  assert.throws(() => parseRealObjects({...plan, surface_point_base_m: undefined}), /执行点/);
  assert.throws(() => parseRealObjects({...plan, success: false}), /执行点/);
});

test('units, frame, dimensions, ids and untrusted photo paths are validated', () => {
  for (const change of [{units: 'mm'}, {frame: 'camera'}, {objects: []}, {objects: [...sample.objects, ...sample.objects]},
    {objects: [{...sample.objects[0], positionBaseM: [NaN, 0, 0]}]}, {objects: [{...sample.objects[0], sizeM: [.001, .04, .04]}]}]) {
    assert.throws(() => parseRealObjects({...sample, ...change}));
  }
  const value = parseRealObjects({...sample, snapshotUrl: 'https://example.com/a.jpg', annotatedUrl: '/captures/../../server.py'});
  assert.equal(value.snapshotUrl, null); assert.equal(value.annotatedUrl, null);
  const path = '/captures/' + 'a'.repeat(32) + '-snapshot.jpg';
  assert.equal(parseRealObjects({...sample, snapshotUrl: path}).snapshotUrl, path);
});

test('table support and measured top-surface modes make height assumptions explicit', () => {
  const value = parseRealObjects(sample);
  const table = realObjectsToBodies(value, {sizeM: [.04, .05, .06], tableTopM: .1})[0];
  assert.deepEqual(table.position.slice(0, 2), [-.3, -.24]); near(table.position[2], .1301);
  const measured = realObjectsToBodies(value, {sizeM: [.04, .04, .06], heightMode: 'measured'})[0]; near(measured.position[2], .01);
  const center = parseRealObjects({...sample, objects: [{...sample.objects[0], pointKind: 'center', sizeM: [.03, .04, .05], yawRad: Math.PI / 2}]});
  const body = realObjectsToBodies(center, {heightMode: 'measured'})[0]; near(body.position[2], .04); near(body.rotation[2], Math.SQRT1_2);
  assert.deepEqual(body.sizeM, [.03, .04, .05]); assert.deepEqual(value.objects[0].positionBaseM, [-.3, -.24, .04]);
});

test('imported real-object definitions preserve old scene on collisions and survive reset', () => {
  const world = new LivePhysics(); world.setEnvironment({groundZ: -.075, table: {position: [0, 0, -.035], size: [1.2, 1, .07]}});
  const bodies = realObjectsToBodies(parseRealObjects(sample), {sizeM: [.04, .04, .04]});
  world.replaceObjects(bodies); const before = world.objectDefinitions(true);
  assert.throws(() => world.replaceObjects([{...bodies[0], position: [-.3, -.24, -.03]}]), /桌面|地面/);
  assert.deepEqual(world.objectDefinitions(true), before); world.reset(); assert.deepEqual(world.objectDefinitions(true), before);
});

test('real-object import can approach and grasp through the actual GLB rig', async () => {
  const buffer = await readFile(new URL('../../robot_system/web/static/robot3d/assets/Lebai_LM3.glb', import.meta.url));
  const gltf = await new GLTFLoader().parseAsync(buffer.buffer.slice(buffer.byteOffset, buffer.byteOffset + buffer.byteLength), '');
  const base = new Group(); base.applyMatrix4(scene_T_base); const model = new Group(); model.applyMatrix4(base_T_model); base.add(model); model.add(gltf.scene);
  const rig = new JointRig(gltf), camera = new GripperCamera(base); camera.bind(rig);
  try {
    const world = new LivePhysics(); world.setEnvironment({groundZ: -.075, table: {position: [0, 0, -.035], size: [1.2, 1, .07]}});
    const home = {joints: [...UPRIGHT_REFERENCE_JOINTS_RAD], gripper: 1}; rig.apply(home);
    const solids = new RobotSolids(rig, base, world); world.robotBoxes = solids.boxes();
    world.replaceObjects(realObjectsToBodies(parseRealObjects(sample), {sizeM: [.04, .04, .04]}));
    const target = world.bodies[0].position;
    const segments = await planVirtualMotion(rig, base, solids, home, [[...target.slice(0, 2), target[2] + .12], [...target]], {firstJointMove: true});
    let pose = structuredClone(home);
    for (const segment of segments) { pose = {...pose, joints: pose.joints.map((q, i) => segment.joints[i] ?? q), gripper: segment.gripper}; rig.apply(pose); }
    const grasp = new VirtualGrasp(world); grasp.update(rig, base, pose, 0); grasp.arm('R1');
    pose = {...pose, gripper: 0}; rig.apply(pose); grasp.update(rig, base, pose, 1020);
    assert.equal(world.heldId, 'R1'); assert.deepEqual(world.bodies[0].sizeM, [.04, .04, .04]);
  } finally { camera.dispose(); rig.dispose(); disposeGraph(gltf.scene); }
});
