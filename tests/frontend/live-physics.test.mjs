import test from 'node:test';
import assert from 'node:assert/strict';
import {Group, Vector3, PerspectiveCamera, Ray, Plane, Mesh, BoxGeometry, MeshBasicMaterial} from '../../robot_system/web/static/robot3d/vendor/three.module.js';
import {LivePhysics, LIVE_STEP_S} from '../../robot_system/web/static/robot3d/live-physics.js';
import {PhysicsView} from '../../robot_system/web/static/robot3d/physics-view.js';
import {pickPhysicsBlock, dragTarget} from '../../robot_system/web/static/robot3d/physics-drag.js';
import {scene_T_base} from '../../robot_system/web/static/robot3d/coordinates.js';
import {pickArmJoint, dragJointAngle} from '../../robot_system/web/static/robot3d/arm-drag.js';

const near = (a, b, e = 1e-8) => assert.ok(Math.abs(a - b) < e, `${a} != ${b}`);
const run = (w, n) => { for (let i = 0; i < n; i++) w.step(); };

test('live physics starts at rest, stable on the floor with no repeated resting impacts', () => {
  const w = new LivePhysics(); run(w, 500);
  assert.equal(w.running, true); assert.equal(w.impactCount, 0);
  for (const b of w.bodies) { near(b.position[2], .03); assert.deepEqual(b.velocity, [0, 0, 0]); }
});

test('mouse drag applies a force and mass affects acceleration rather than teleporting', () => {
  const light = new LivePhysics(), heavy = new LivePhysics({massA: 2});
  for (const w of [light, heavy]) { w.beginDrag('A'); w.setDragTarget([-.15, 0, .03]); w.step(); }
  near(light.bodies[0].acceleration[0], 1.8); near(heavy.bodies[0].acceleration[0], .9);
  assert.ok(light.bodies[0].position[0] < -.159);
  near(light.snapshot().bodies[0].momentum[0], light.bodies[0].mass * light.bodies[0].velocity[0]);
  assert.ok(light.bodies[0].dragForce[0] > 0);
});

test('release retains velocity and the block continues moving; pause prevents drag', () => {
  const w = new LivePhysics({gravity: 0, friction: 0}); w.beginDrag('A'); w.setDragTarget([-.08, 0, .03]); run(w, 10);
  const v = w.bodies[0].velocity[0], x = w.bodies[0].position[0]; assert.ok(v > 0);
  w.endDrag(); w.step(); near(w.bodies[0].velocity[0], v); assert.ok(w.bodies[0].position[0] > x);
  assert.deepEqual(w.bodies[0].dragForce, [0, 0, 0]);
  w.setRunning(false); const before = w.snapshot(); w.advance(.1); assert.deepEqual(w.snapshot(), before);
  assert.equal(w.beginDrag('A'), false); w.reset(); assert.equal(w.running, true);
});

test('lifting through mouse force and releasing produces gravity and platform impulse', () => {
  const w = new LivePhysics(); w.beginDrag('A'); w.setDragTarget([-.16, 0, .3]); run(w, 120);
  assert.ok(w.bodies[0].position[2] > .20); w.endDrag(); w.step(); near(w.bodies[0].acceleration[2], -9.81);
  for (let i = 0; i < 600 && !w.lastImpact; i++) w.step();
  assert.equal(w.lastImpact.kind, 'floor'); assert.ok(w.lastImpact.impulseOnFirstNs[2] > 0);
  assert.equal(w.running, true, 'live impact must not pause');
});

test('3D contact impulse conserves momentum and elastic kinetic energy with unequal masses', () => {
  for (const axis of [0, 1, 2]) {
    const w = new LivePhysics({massA: 1, massB: 2, gravity: 0, friction: 0, restitution: 1});
    w.bodies[0].position = [0, 0, .3]; w.bodies[1].position = [0, 0, .3];
    w.bodies[0].position[axis] -= .031; w.bodies[1].position[axis] += .031;
    w.bodies[0].velocity[axis] = 1; w.step(); const hit = w.lastImpact;
    assert.equal(hit.kind, 'blocks'); near(hit.after[0].velocity[axis], -1 / 3); near(hit.after[1].velocity[axis], 2 / 3);
    near(hit.totalMomentumBefore[axis], hit.totalMomentumAfter[axis]); near(hit.kineticBeforeJ, hit.kineticAfterJ);
  }
});

test('blocks collide and can travel beyond the removed cage; floor friction reduces speed', () => {
  const w = new LivePhysics({gravity: 0, restitution: .8, friction: 0}); w.bodies[0].velocity[0] = 3;
  run(w, 150); assert.ok(w.impactCount >= 1);
  assert.ok(w.bodies[1].position[0] > 1);
  const f = new LivePhysics(); f.bodies[0].velocity[1] = .1; run(f, 20); near(f.bodies[0].velocity[1], 0);
});

test('live mass changes keep position and velocity; targets validate and snapshots are isolated', () => {
  const w = new LivePhysics(); w.bodies[0].velocity[0] = .4; const x = w.bodies[0].position[0];
  w.configure({massA: 2}); near(w.snapshot().bodies[0].momentum[0], .8); near(w.bodies[0].position[0], x);
  assert.throws(() => w.configure({massA: -1})); near(w.config.massA, 2);
  w.beginDrag('A'); assert.throws(() => w.setDragTarget([NaN, 0, 0])); w.setDragTarget([10, -10, -10]);
  assert.deepEqual(w.drag.target, [10, -10, .03]);
  const snap = w.snapshot(); snap.bodies[0].position[0] = 8; snap.bodies[0].velocity[0] = 8;
  near(w.bodies[0].position[0], x); near(w.bodies[0].velocity[0], .4);
});

test('held block cannot push another through the scene table, including extreme mass ratios', () => {
  for (const [massA, massB] of [[1, 1], [.05, 10], [10, .05]]) {
    const w = new LivePhysics({massA, massB});
    w.setEnvironment({table: {position: [.4, 0, .15], size: [.2, .4, .3]}});
    w.beginDrag('A'); w.setDragTarget([.6, 0, .03]);
    for (let i = 0; i < 1000; i++) {
      w.step();
      for (const b of w.bodies) assert.ok(b.position[0] <= .27 + 1e-8);
      assert.ok(w.bodies[1].position[0] - w.bodies[0].position[0] >= .06 - 1e-8);
    }
  }
});

test('blocks rest on the shared table, leave the table freely and land on the shared ground', () => {
  const w = new LivePhysics();
  w.setEnvironment({groundZ: -.075, table: {position: [0, 0, -.035], size: [1.2, 1, .07]},
    initialPositions: [[.3, -.15, .03], [.3, .15, .03]]}); w.reset(); run(w, 300);
  near(w.bodies[0].position[2], .03); assert.equal(w.impactCount, 0);
  w.beginDrag('A'); w.setDragTarget([.85, -.15, .03]); run(w, 180); w.endDrag();
  assert.ok(w.bodies[0].position[0] > .65);
  run(w, 400); near(w.bodies[0].position[2], -.045); assert.equal(w.lastImpact.second, '地面');
});

test('ground-level movement hits the side of the shared table instead of passing through it', () => {
  const w = new LivePhysics({friction: 0});
  w.setEnvironment({groundZ: -.075, table: {position: [0, 0, -.035], size: [1.2, 1, .07]},
    initialPositions: [[.8, -.15, -.045], [.3, .15, .03]]}); w.reset();
  w.bodies[0].velocity[0] = -1; run(w, 100);
  assert.ok(w.bodies[0].position[0] >= .63); assert.equal(w.lastImpact.second, '桌面');
});

test('screen ray picks the visible block under scene_T_base, empty space does not intercept orbit', () => {
  const base = new Group(); base.applyMatrix4(scene_T_base);
  const w = new LivePhysics(), view = new PhysicsView(base); view.root.visible = true; view.update(w.snapshot(), 'live');
  base.updateMatrixWorld(true); const p = view.blocks[0].mesh.getWorldPosition(new Vector3());
  const camera = new PerspectiveCamera(45, 1, .01, 10); camera.position.copy(p).add(new Vector3(0, .5, .5)); camera.lookAt(p);
  const rect = {left: 20, top: 10, width: 400, height: 400};
  const picked = pickPhysicsBlock(camera, view, rect, 220, 210); assert.equal(picked.hit.object.name, 'physics-block-A');
  assert.equal(pickPhysicsBlock(camera, view, rect, 21, 11), null);
  view.root.visible = false; assert.equal(pickPhysicsBlock(camera, view, rect, 220, 210), null); view.dispose();
});

test('drag plane maps camera ray into local Base coordinates and preserves pick offset', () => {
  const base = new Group(); base.applyMatrix4(scene_T_base); const root = new Group(); root.position.set(.95, -.1, 0); base.add(root);
  base.updateMatrixWorld(true);
  const local = new Vector3(.1, .02, .2), world = root.localToWorld(local.clone());
  const normal = new Vector3(0, 1, 0).transformDirection(root.matrixWorld);
  const plane = new Plane().setFromNormalAndCoplanarPoint(normal, world);
  const ray = new Ray(world.clone().addScaledVector(normal, 1), normal.clone().negate());
  const target = dragTarget(ray, plane, root, new Vector3(.01, 0, 0));
  target.forEach((v, i) => near(v, [.09, .02, .2][i]));
});

test('dragging an arm link picks its owning joint and local angles stay bounded', () => {
  const root = new Group(), joint = new Group(), child = new Group(); root.add(joint); joint.add(child);
  const mesh = new Mesh(new BoxGeometry(.1, .1, .1), new MeshBasicMaterial()); child.add(mesh);
  const rig = {root, joints: [{node: joint}]}, camera = new PerspectiveCamera(45, 1, .01, 10);
  camera.position.set(0, 0, 1); camera.lookAt(0, 0, 0);
  const rect = {left: 0, top: 0, width: 400, height: 400};
  assert.equal(pickArmJoint(camera, rig, rect, 200, 200), 0);
  assert.equal(pickArmJoint(camera, rig, rect, 0, 0), null);
  near(dragJointAngle(0, 100, 0), .8); near(dragJointAngle(0, 100, 0, true), .2);
  near(dragJointAngle(0, 1000, 0), Math.PI); near(dragJointAngle(0, -1000, 0), -Math.PI);
  mesh.geometry.dispose(); mesh.material.dispose();
});
