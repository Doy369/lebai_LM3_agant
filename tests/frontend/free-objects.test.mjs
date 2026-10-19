import test from 'node:test';
import assert from 'node:assert/strict';
import { readFile } from 'node:fs/promises';
import { Group, PerspectiveCamera, Quaternion, Vector3 } from '../../robot_system/web/static/robot3d/vendor/three.module.js';
import { GLTFLoader } from '../../robot_system/web/static/robot3d/vendor/GLTFLoader.js';
import { JointRig, UPRIGHT_REFERENCE_JOINTS_RAD } from '../../robot_system/web/static/robot3d/joints.js';
import { disposeGraph } from '../../robot_system/web/static/robot3d/model-loader.js';
import { scene_T_base, base_T_model } from '../../robot_system/web/static/robot3d/coordinates.js';
import { LivePhysics } from '../../robot_system/web/static/robot3d/live-physics.js';
import { RobotSolids, bodyBox, boxContact, boxExtents } from '../../robot_system/web/static/robot3d/solid-collisions.js';
import { PhysicsView } from '../../robot_system/web/static/robot3d/physics-view.js';
import { VirtualGrasp, gripperOpeningM } from '../../robot_system/web/static/robot3d/virtual-grasp.js';
import { virtualToolMatrix } from '../../robot_system/web/static/robot3d/virtual-tool.js';
import { planVirtualMotion, transferWaypoints } from '../../robot_system/web/static/robot3d/virtual-motion.js';
import { pickTablePoint, freeTableObject, tableObject, placementTarget } from '../../robot_system/web/static/robot3d/placement.js';
import { validateObjects } from '../../robot_system/web/static/robot3d/objects.js';
import { DEFAULT_CONFIG, validateConfig } from '../../robot_system/web/static/robot3d/config.js';

const spec = () => ({label: '绿色测试物块', color: '#35bca9', mass: .4, sizeM: [.04, .04, .04]});
const table = {position: [0, 0, -.035], size: [1.2, 1, .07]};
const near = (a, b, tolerance = 1e-7) => assert.ok(Math.abs(a - b) < tolerance, `${a} != ${b}`);
const home = () => ({joints: [...UPRIGHT_REFERENCE_JOINTS_RAD], gripper: 1});
function worldAtTable() {
  const world = new LivePhysics();
  world.setEnvironment({groundZ: -.075, table, obstacles: [{position: [.3, 0, .04], size: [.08, .08, .08], label: '橙色固定物块'}],
    initialPositions: [[.264, -.16, .03], [.36, .16, .03]]}); world.reset(); return world;
}

test('custom size, mass, color and stable ids survive reset; deletion and empty scenes remain valid', () => {
  const world = worldAtTable(), body = world.addBody(freeTableObject(world, spec()));
  assert.equal(body.id, 'C1'); assert.equal(world.bodies.length, 3); assert.deepEqual(boxExtents(bodyBox(body)), [.02, .02, .02]);
  const initial = [...body.position]; body.position[0] += .02; world.reset();
  assert.deepEqual(world.bodies.find(b => b.id === 'C1').position, initial);
  world.configure({massB: 2, gravity: 3}); near(world.bodies.find(b => b.id === 'C1').mass, .4);
  world.updateBody('C1', {sizeM: [.05, .04, .08], mass: 2}); near(world.bodies.find(b => b.id === 'C1').position[2], .04);
  assert.throws(() => world.updateBody('C1', {sizeM: [0, .04, .04]}));
  const exported = world.snapshot(); exported.bodies[2].sizeM[0] = 5; near(world.bodies[2].sizeM[0], .05);
  world.attach('C1'); assert.throws(() => world.removeBody('C1'), /松开/); assert.throws(() => world.updateBody('C1', {mass: 1}), /松开/);
  world.releaseHeld(); world.beginDrag('C1'); world.removeBody('C1'); assert.equal(world.drag, null);
  world.removeBody('A'); world.removeBody('B'); world.step(); world.reset(); assert.equal(world.bodies.length, 0);
});

test('all custom pairs collide at actual sizes, including minimum-size bodies at maximum throw speed', () => {
  const world = new LivePhysics({gravity: 0, restitution: 1, friction: 0});
  world.replaceObjects(['C1', 'C2', 'C3'].map((id, i) => ({...spec(), id, sizeM: [.02, .02, .02], position: [.08 * i, 0, .3]})));
  world.bodies[1].velocity[0] = 3; world.bodies[2].velocity[0] = -3;
  for (let i = 0; i < 8; i++) world.step();
  const hit = world.lastImpact; assert.equal(hit.kind, 'blocks'); assert.equal(hit.first, 'C2'); assert.equal(hit.second, 'C3');
  assert.ok(world.bodies[1].velocity[0] < 0); assert.ok(world.bodies[2].velocity[0] > 0);
  near(hit.totalMomentumBefore[0], hit.totalMomentumAfter[0]);
  assert.equal(boxContact(bodyBox(world.bodies[1]), bodyBox(world.bodies[2]), 1e-6), null);
});

test('rendered custom volumes follow their dimensions and disappear after removal', () => {
  const world = worldAtTable(), view = new PhysicsView(new Group());
  const body = world.addBody(freeTableObject(world, spec())); view.update(world.snapshot(), 'live');
  const mesh = view.blocks.find(b => b.id === body.id).mesh;
  assert.deepEqual(mesh.scale.toArray(), body.sizeM); assert.equal(mesh.material.color.getHexString(), '35bca9');
  world.removeBody('A'); world.removeBody(body.id); view.update(world.snapshot(), 'live');
  assert.deepEqual(view.blocks.map(b => b.id), ['B']); assert.equal(mesh.parent, null); view.dispose();
});

test('table ray picking respects the Base transform and full object footprint', () => {
  const base = new Group(); base.applyMatrix4(scene_T_base); base.updateMatrixWorld(true);
  const target = base.localToWorld(new Vector3(-.3, -.2, 0)), camera = new PerspectiveCamera(45, 1, .01, 10);
  camera.position.copy(target).add(new Vector3(0, .8, .5)); camera.lookAt(target);
  const rect = {left: 20, top: 10, width: 400, height: 400}, point = pickTablePoint(camera, base, table, rect, 220, 210);
  point.forEach((v, i) => near(v, [-.3, -.2, 0][i]));
  assert.equal(pickTablePoint(camera, base, table, rect, -1, 200), null);
  assert.throws(() => tableObject(worldAtTable(), spec(), [.599, 0, 0]), /完整空间/);
  const world = worldAtTable(); assert.throws(() => world.addBody(tableObject(world, spec(), [.3, 0, 0])), /占用/);
});

test('custom objects roundtrip in scene JSON; invalid import leaves existing objects intact', () => {
  const world = worldAtTable(); world.addBody(freeTableObject(world, spec()));
  const config = validateConfig({...DEFAULT_CONFIG, objects: world.objectDefinitions(true)});
  assert.deepEqual(validateConfig(JSON.parse(JSON.stringify(config))), config);
  const before = world.objectDefinitions(true);
  assert.throws(() => world.replaceObjects([{...before[0], position: [.3, 0, .03]}]), /占用/);
  assert.deepEqual(world.objectDefinitions(true), before);
  assert.throws(() => validateObjects([before[0], before[0]]), /重复/);
});

test('actual robot grips a new 4 cm body, checks its carried volume, places at an arbitrary picked point and releases', async () => {
  const bytes = await readFile(new URL('../../robot_system/web/static/robot3d/assets/Lebai_LM3.glb', import.meta.url));
  const gltf = await new GLTFLoader().parseAsync(bytes.buffer.slice(bytes.byteOffset, bytes.byteOffset + bytes.byteLength), '');
  const rig = new JointRig(gltf), base = new Group(), model = new Group();
  base.applyMatrix4(scene_T_base); model.applyMatrix4(base_T_model); base.add(model); model.add(gltf.scene);
  const world = worldAtTable(), solids = new RobotSolids(rig, base, world), grasp = new VirtualGrasp(world);
  let p = home(); rig.apply(p); world.robotBoxes = solids.boxes();
  const body = world.addBody(freeTableObject(world, spec())), start = [...body.position];
  async function travel(points, firstJointMove = false) {
    const segments = await planVirtualMotion(rig, base, solids, p, points, {firstJointMove,
      displayPose: q => grasp.displayPose(q), heldAt: q => grasp.heldAt(rig, base, q)});
    for (const segment of segments) {
      const next = {joints: p.joints.map((q, i) => segment.joints[i] ?? q), gripper: segment.gripper};
      const swept = solids.sweep(grasp.displayPose(p), grasp.displayPose(next), q => grasp.heldAt(rig, base, q));
      assert.equal(swept.blocked, false, swept.reason); p = next;
      rig.apply(p); rig.apply(grasp.update(rig, base, p, 1000));
    }
  }
  try {
    grasp.update(rig, base, p, 0);
    await travel([[start[0], start[1], start[2] + .16], [start[0], start[1], start[2] + .006]], true);
    grasp.arm(body.id); p = {...p, gripper: 0}; rig.apply(p); rig.apply(grasp.update(rig, base, p, 1020));
    assert.equal(world.heldId, body.id); assert.ok(gripperOpeningM(rig) >= .04 && gripperOpeningM(rig) < .042);
    assert.deepEqual(grasp.heldAt(rig, base, p).sizeM, [.04, .04, .04]);
    await travel([[start[0], start[1], start[2] + .18]]);
    world.robotBoxes = solids.boxes();
    const point = [-.32, -.24, 0], target = placementTarget(world, body, point, grasp.attachment);
    assert.throws(() => placementTarget(world, body, [.3, 0, 0], grasp.attachment), /占用/);
    const current = new Vector3().setFromMatrixPosition(virtualToolMatrix(rig, base)).toArray();
    await travel(transferWaypoints(current, ...target.toolPoint));
    p = {...p, gripper: 1}; rig.apply(p); grasp.update(rig, base, p, 2020); assert.equal(world.heldId, null);
    world.robotBoxes = solids.boxes(); for (let i = 0; i < 300; i++) world.step();
    near(body.position[0], point[0], .007); near(body.position[1], point[1], .007);
    near(body.position[2], boxExtents(bodyBox(body))[2], .0001);
  } finally { rig.dispose(); disposeGraph(gltf.scene); }
});
