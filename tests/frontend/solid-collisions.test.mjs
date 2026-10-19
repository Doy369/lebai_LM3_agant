import test from 'node:test';
import assert from 'node:assert/strict';
import { readFile } from 'node:fs/promises';
import { Group, Quaternion, Vector3 } from '../../robot_system/web/static/robot3d/vendor/three.module.js';
import { GLTFLoader } from '../../robot_system/web/static/robot3d/vendor/GLTFLoader.js';
import { JointRig, UPRIGHT_REFERENCE_JOINTS_RAD } from '../../robot_system/web/static/robot3d/joints.js';
import { disposeGraph } from '../../robot_system/web/static/robot3d/model-loader.js';
import { scene_T_base, base_T_model } from '../../robot_system/web/static/robot3d/coordinates.js';
import { RobotSolids, solidBox, bodyBox, boxContact, boxExtents } from '../../robot_system/web/static/robot3d/solid-collisions.js';
import { planVirtualMotion, transferWaypoints } from '../../robot_system/web/static/robot3d/virtual-motion.js';
import { VirtualGrasp } from '../../robot_system/web/static/robot3d/virtual-grasp.js';
import { LivePhysics } from '../../robot_system/web/static/robot3d/live-physics.js';

const bytes = await readFile(new URL('../../robot_system/web/static/robot3d/assets/Lebai_LM3.glb', import.meta.url));
async function setup() {
  const gltf = await new GLTFLoader().parseAsync(bytes.buffer.slice(bytes.byteOffset, bytes.byteOffset + bytes.byteLength), '');
  const rig = new JointRig(gltf), base = new Group(), model = new Group();
  base.applyMatrix4(scene_T_base); model.applyMatrix4(base_T_model); base.add(model); model.add(gltf.scene);
  const world = new LivePhysics();
  world.setEnvironment({groundZ: -.075, table: {position: [0, 0, -.035], size: [1.2, 1, .07]},
    obstacles: [{position: [.3, 0, .04], size: [.08, .08, .08], label: '橙色固定物块'}],
    initialPositions: [[.264, -.16, .03], [.36, .16, .03]]}); world.reset();
  return {rig, base, world, solids: new RobotSolids(rig, base, world), dispose() {rig.dispose(); disposeGraph(gltf.scene);}};
}
const home = () => ({joints: [...UPRIGHT_REFERENCE_JOINTS_RAD], gripper: 1});

test('oriented cube volume collides at rotated corners and floor support uses its actual extent', () => {
  const q = new Quaternion().setFromAxisAngle(new Vector3(0, 1, 0), Math.PI / 4).toArray();
  const a = solidBox([0, 0, .1], [.06, .06, .06], q), b = solidBox([.07, 0, .1], [.06, .06, .06]);
  assert.ok(boxContact(a, b), 'a rotated corner reaches beyond the unrotated 3 cm half-width');
  const w = new LivePhysics(); w.bodies[0].rotation = q; w.bodies[0].position = [-.16, 0, .2];
  for (let i = 0; i < 600; i++) w.step();
  assert.ok(Math.abs(w.bodies[0].position[2] - boxExtents(bodyBox(w.bodies[0]))[2]) < 1e-7);
});

test('large joint jumps stop at the first collision, including a path with two valid endpoints', async () => {
  const {solids, dispose} = await setup();
  try {
    const p = home(); assert.equal(solids.check(p), null);
    const endpoint = structuredClone(p); endpoint.joints[1] = Math.PI;
    assert.equal(solids.check(endpoint), null, 'endpoint alone would miss the table crossing');
    const result = solids.sweep(p, endpoint);
    assert.equal(result.blocked, true); assert.match(result.reason, /桌面/); assert.equal(solids.check(result.pose), null);
    const retreat = solids.sweep(result.pose, p); assert.equal(retreat.blocked, false, 'one can leave the boundary');
    const fold = structuredClone(p); fold.joints[2] = Math.PI;
    const folded = solids.sweep(p, fold); assert.equal(folded.blocked, true); assert.match(folded.reason, /自身碰撞死区/);
    assert.equal(solids.check(folded.pose), null);
    const limit = structuredClone(p); limit.joints[0] = Math.PI + .1;
    assert.match(solids.check(limit), /限位/);
    const wrist = structuredClone(p); wrist.joints[4] = solids.rig.joints[4].zeroOffsetRad;
    assert.match(solids.check(wrist, null, {wrist: true}), /奇异死区/);
  } finally { dispose(); }
});

test('a nearby cube offset across the finger gap cannot be captured through a finger', async () => {
  const {rig, base, solids, world, dispose} = await setup(), grasp = new VirtualGrasp(world);
  try {
    const initial = home(), segments = await planVirtualMotion(rig, base, solids, initial, [[.264, -.16, .194], [.264, -.16, .036]], {firstJointMove: true});
    const last = segments.at(-1), p = {joints: initial.joints.map((q, i) => last.joints[i] ?? q), gripper: 1};
    world.bodies[0].position[1] += .01; rig.apply(p); grasp.update(rig, base, p, 0);
    rig.apply({...p, gripper: 0}); grasp.update(rig, base, {...p, gripper: 0}, 20);
    assert.equal(world.heldId, null); assert.match(grasp.message, /未对准/);
  } finally { dispose(); }
});

test('dragging a cube into the robot base or fixed orange block never leaves intersecting volumes', async () => {
  const {rig, solids, world, dispose} = await setup();
  try {
    rig.apply(home()); world.robotBoxes = solids.boxes();
    for (const target of [[0, 0, .03], [.3, 0, .03]]) {
      world.beginDrag('A'); world.setDragTarget(target);
      for (let i = 0; i < 350; i++) {
        world.step(); const cube = bodyBox(world.bodies[0]);
        assert.ok(!world.robotBoxes.some(box => boxContact(box, cube, .0001)));
        assert.equal(boxContact(solidBox([.3, 0, .04], [.08, .08, .08]), cube, .0001), null);
      }
      world.endDrag();
    }
  } finally { dispose(); }
});

test('both cubes can be approached, gripped, lifted and carried along fully validated paths', async () => {
  const {rig, base, solids, world, dispose} = await setup(); const grasp = new VirtualGrasp(world);
  try {
    for (const id of ['A', 'B']) {
      world.reset(); grasp.reset(); let p = home(); rig.apply(p); grasp.update(rig, base, p, 0);
      const body = world.bodies.find(b => b.id === id), [x, y] = body.position;
      const travel = async (points, firstJointMove = false) => {
        let segments;
        try { segments = await planVirtualMotion(rig, base, solids, p, points, {firstJointMove,
          displayPose: q => grasp.displayPose(q), heldAt: q => grasp.heldAt(rig, base, q)}); }
        catch (error) { throw Error(`${id} at ${JSON.stringify(points)}: ${error.message}`, {cause: error}); }
        for (const segment of segments) {
          const next = {joints: p.joints.map((q, i) => segment.joints[i] ?? q), gripper: segment.gripper};
          const sweep = solids.sweep(grasp.displayPose(p), grasp.displayPose(next), q => grasp.heldAt(rig, base, q));
          assert.equal(sweep.blocked, false, sweep.reason); p = next;
          rig.apply(p); rig.apply(grasp.update(rig, base, p, 1000));
          assert.equal(solids.check(grasp.displayPose(p), grasp.heldAt(rig, base, p)), null);
        }
      };
      await travel([[x, y, .194], [x, y, .036]], true);
      p = {...p, gripper: 0}; rig.apply(p); rig.apply(grasp.update(rig, base, p, 1020));
      assert.equal(world.heldId, id);
      await travel([[x, y, .216]]);
      const collision = solids.check(grasp.displayPose(p), {...grasp.heldAt(rig, base, p), position: [x, y, 0]});
      assert.match(collision, /桌面/, 'a held cube must not bypass static contacts');
      await travel(transferWaypoints([x, y, .216], -.276, -.16, .046));
      p = {...p, gripper: 1}; rig.apply(p); grasp.update(rig, base, p, 2020); assert.equal(world.heldId, null);
      for (let i = 0; i < 500; i++) world.step();
      assert.ok(Math.abs(body.position[2] - boxExtents(bodyBox(body))[2]) < 1e-7);
    }
  } finally { dispose(); }
});
