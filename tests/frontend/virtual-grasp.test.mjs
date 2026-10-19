import test from 'node:test';
import assert from 'node:assert/strict';
import { readFile } from 'node:fs/promises';
import { Group, Vector3 } from '../../robot_system/web/static/robot3d/vendor/three.module.js';
import { GLTFLoader } from '../../robot_system/web/static/robot3d/vendor/GLTFLoader.js';
import { JointRig, UPRIGHT_REFERENCE_JOINTS_RAD } from '../../robot_system/web/static/robot3d/joints.js';
import { disposeGraph } from '../../robot_system/web/static/robot3d/model-loader.js';
import { scene_T_base, base_T_model } from '../../robot_system/web/static/robot3d/coordinates.js';
import { solveVirtualIK, virtualToolMatrix } from '../../robot_system/web/static/robot3d/virtual-tool.js';
import { VirtualGrasp, gripperOpeningM } from '../../robot_system/web/static/robot3d/virtual-grasp.js';
import { LivePhysics } from '../../robot_system/web/static/robot3d/live-physics.js';
import { bodyBox, boxExtents } from '../../robot_system/web/static/robot3d/solid-collisions.js';

const bytes = await readFile(new URL('../../robot_system/web/static/robot3d/assets/Lebai_LM3.glb', import.meta.url));
async function setup() {
  const gltf = await new GLTFLoader().parseAsync(bytes.buffer.slice(bytes.byteOffset, bytes.byteOffset + bytes.byteLength), '');
  const rig = new JointRig(gltf), base = new Group(), model = new Group();
  base.applyMatrix4(scene_T_base); model.applyMatrix4(base_T_model); base.add(model); model.add(gltf.scene);
  const world = new LivePhysics();
  world.setEnvironment({groundZ: -.075, table: {position: [0, 0, -.035], size: [1.2, 1, .07]},
    initialPositions: [[.264, -.16, .03], [.36, .16, .03]]}); world.reset();
  return {rig, base, world, dispose() {rig.dispose(); disposeGraph(gltf.scene);}};
}
const point = (rig, base) => new Vector3().setFromMatrixPosition(virtualToolMatrix(rig, base));
const initial = () => ({joints: [0, 0, 0, 0, 0, 0], gripper: 1});

test('actual GLB reaches both blocks and transfer waypoints; failed IK restores the original pose', async () => {
  const {rig, base, dispose} = await setup(); let p = {joints: [...UPRIGHT_REFERENCE_JOINTS_RAD], gripper: 1};
  try {
    for (const target of [[.264, -.16, .194], [.264, -.16, .034], [.264, -.16, .214], [-.276, -.16, .246], [-.276, -.16, .046], [.36, .16, .034]]) {
      p = solveVirtualIK(rig, base, target, p); rig.apply(p);
      assert.ok(point(rig, base).distanceTo(new Vector3(...target)) < .006);
      assert.ok(new Vector3(0, 0, 1).transformDirection(virtualToolMatrix(rig, base)).z > .98);
    }
    const before = point(rig, base).clone();
    assert.throws(() => solveVirtualIK(rig, base, [3, 0, 1], p), /够不到/);
    assert.ok(point(rig, base).distanceTo(before) < 1e-10);
  } finally { dispose(); }
});

test('close captures a nearby block, fingers stop at its width, carry follows the tool and open restores gravity', async () => {
  const {rig, base, world, dispose} = await setup(), grasp = new VirtualGrasp(world);
  try {
    let p = solveVirtualIK(rig, base, [.264, -.16, .034], initial(), {align: true}); rig.apply(p); grasp.update(rig, base, p, 0);
    p = {...p, gripper: 0}; rig.apply(p); const display = grasp.update(rig, base, p, 20);
    assert.equal(world.heldId, 'A'); assert.ok(display.gripper > .5 && display.gripper < 1);
    rig.apply(display); assert.ok(gripperOpeningM(rig) >= .06 && gripperOpeningM(rig) < .0615);
    assert.equal(world.beginDrag('A'), false);
    const before = [...world.bodies[0].position]; for (let i = 0; i < 300; i++) world.step();
    assert.deepEqual(world.bodies[0].position, before);
    p = solveVirtualIK(rig, base, [.264, -.16, .214], p, {align: true}); rig.apply(p); grasp.update(rig, base, p, 1020);
    assert.ok(world.bodies[0].position[2] > .20);
    p = {...p, joints: [...p.joints]}; p.joints[0] -= .25; rig.apply(p); grasp.update(rig, base, p, 2020);
    assert.ok(Math.abs(world.bodies[0].position[0] - before[0]) > .01);
    assert.ok(Math.abs(world.bodies[0].rotation[2]) > .01);
    grasp.update(rig, base, p, 3020); p = {...p, gripper: 1}; rig.apply(p); grasp.update(rig, base, p, 3040);
    assert.equal(world.heldId, null);
    for (let i = 0; i < 700; i++) world.step();
    assert.ok(Math.abs(world.bodies[0].position[2] - boxExtents(bodyBox(world.bodies[0]))[2]) < 1e-7);
  } finally { dispose(); }
});

test('closing away from a block never captures it; resetting cancels a held constraint', async () => {
  const {rig, base, world, dispose} = await setup(), grasp = new VirtualGrasp(world);
  try {
    let p = initial(); rig.apply(p); grasp.update(rig, base, p, 0);
    p = {...p, gripper: 0}; rig.apply(p); grasp.update(rig, base, p, 20);
    assert.equal(world.heldId, null); assert.match(grasp.message, /没有物块/);
    p = solveVirtualIK(rig, base, [.36, .16, .034], {...p, gripper: 1}, {align: true}); rig.apply(p); grasp.update(rig, base, p, 40);
    p = {...p, gripper: 0}; rig.apply(p); grasp.update(rig, base, p, 60); assert.equal(world.heldId, 'B');
    world.reset(); grasp.update(rig, base, p, 80);
    assert.equal(world.heldId, null); assert.equal(grasp.attachment, null);
  } finally { dispose(); }
});

test('a held body acts as a kinematic contact without moving under gravity or drag', () => {
  const world = new LivePhysics({massA: .05, massB: 10, gravity: 0});
  world.bodies[0].position = [0, 0, .1]; world.bodies[1].position = [.05, 0, .1];
  assert.equal(world.attach('A'), true); assert.equal(world.attach('B'), false);
  world.step(); assert.deepEqual(world.bodies[0].position, [0, 0, .1]);
  assert.ok(world.bodies[1].position[0] >= .06 - 1e-9);
  const snapshot = world.snapshot(); snapshot.bodies[0].rotation[0] = 10;
  assert.equal(world.bodies[0].rotation[0], 0);
  world.releaseHeld(); assert.equal(world.beginDrag('A'), true);
});
