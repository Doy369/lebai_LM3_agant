import test from 'node:test';
import assert from 'node:assert/strict';
import {Group, Vector3} from '../../robot_system/web/static/robot3d/vendor/three.module.js';
import {SimplePhysics, validatePhysicsConfig, PHYSICS_STEP_S, BLOCK_SIDE_M} from '../../robot_system/web/static/robot3d/simple-physics.js';
import {PhysicsView} from '../../robot_system/web/static/robot3d/physics-view.js';
import {scene_T_base} from '../../robot_system/web/static/robot3d/coordinates.js';

const near = (a, b, tolerance = 1e-8) => assert.ok(Math.abs(a - b) < tolerance, `${a} != ${b}`);
function untilImpact(world) {
  world.start();
  for (let i = 0; i < 3000 && !world.lastImpact; i++) world.advance(PHYSICS_STEP_S);
  assert.ok(world.lastImpact, 'expected a collision');
  return world.lastImpact;
}

test('free fall follows z=z0-gt²/2, v=-gt and p=mv, independent of frame batching', () => {
  const a = new SimplePhysics({mode: 'drop', height: .6, gravity: 2, massA: 2});
  const b = new SimplePhysics(a.config); a.start(); b.start();
  for (let i = 0; i < 60; i++) a.advance(PHYSICS_STEP_S);
  for (let i = 0; i < 5; i++) b.advance(.05);
  near(a.time, .25); near(a.bodies[0].position[2], .03 + .6 - .5 * 2 * .25 ** 2);
  near(a.bodies[0].velocity[2], -.5); near(a.snapshot().bodies[0].momentum[2], -1);
  near(a.snapshot().bodies[0].acceleration[2], -2);
  near(a.bodies[0].position[2], b.bodies[0].position[2]);
});

test('floor rebound uses restitution and external impulse equals delta momentum', () => {
  const world = new SimplePhysics({mode: 'drop', gravity: 9.81, height: .35, massA: 2, restitution: .5});
  const hit = untilImpact(world); const beforeV = -Math.sqrt(2 * 9.81 * .35);
  assert.equal(hit.kind, 'floor'); near(hit.before[0].velocity[2], beforeV);
  near(hit.after[0].velocity[2], -beforeV * .5);
  near(hit.impulseOnFirstNs[2], 2 * (-beforeV * .5 - beforeV));
  assert.notDeepEqual(hit.totalMomentumBefore, hit.totalMomentumAfter);
  assert.equal(world.running, false); near(world.bodies[0].position[2], BLOCK_SIDE_M / 2);
});

test('elastic head-on collision conserves momentum and energy for unequal masses', () => {
  const world = new SimplePhysics({massA: 1, massB: 2, speed: 1, restitution: 1});
  const hit = untilImpact(world);
  assert.equal(hit.kind, 'blocks'); near(hit.after[0].velocity[0], -1 / 3); near(hit.after[1].velocity[0], 2 / 3);
  near(hit.totalMomentumBefore[0], hit.totalMomentumAfter[0]); near(hit.kineticBeforeJ, hit.kineticAfterJ);
  near(hit.impulseOnFirstNs[0], -4 / 3);
});

test('inelastic impact keeps momentum, decreases energy, and zero restitution matches velocities', () => {
  for (const restitution of [0, .3, .8]) {
    const world = new SimplePhysics({massA: 2, massB: .5, speed: 1.5, restitution});
    const hit = untilImpact(world);
    near(hit.totalMomentumBefore[0], hit.totalMomentumAfter[0]);
    assert.ok(hit.kineticAfterJ < hit.kineticBeforeJ);
    near(hit.after[1].velocity[0] - hit.after[0].velocity[0], restitution * 1.5);
  }
});

test('swept collisions do not tunnel at max speed and wall impacts use external impulse', () => {
  const world = new SimplePhysics({speed: 3, restitution: 1});
  world.start(); world.step(.1);
  assert.equal(world.lastImpact.kind, 'blocks');
  near(world.time, (.10 - (-.18) - .06) / 3);
  near(world.bodies[1].position[0] - world.bodies[0].position[0], BLOCK_SIDE_M);
  world.start();
  for (let i = 0; i < 500 && world.lastImpact.kind !== 'wall'; i++) world.advance(PHYSICS_STEP_S);
  assert.equal(world.lastImpact.kind, 'wall');
  assert.notDeepEqual(world.lastImpact.totalMomentumBefore, world.lastImpact.totalMomentumAfter);
});

test('resting floor contact does not chatter; pause/reset and large frame gaps stay bounded', () => {
  const world = new SimplePhysics({mode: 'drop', restitution: 0, pauseOnImpact: false});
  world.start(); for (let i = 0; i < 600; i++) world.advance(PHYSICS_STEP_S);
  assert.equal(world.impactCount, 1); near(world.bodies[0].velocity[2], 0);
  near(world.snapshot().bodies[0].acceleration[2], 0);
  world.pause(); const before = world.snapshot(); world.advance(10); assert.deepEqual(world.snapshot(), before);
  world.reset({mode: 'drop', gravity: 0}); world.start(); world.advance(10); assert.ok(world.time <= .100001);
  near(world.bodies[0].velocity[2], 0);
  world.reset(); assert.equal(world.running, false); assert.equal(world.lastImpact, null);
});

test('parameters reject invalid units/ranges and exported snapshots do not mutate simulation', () => {
  for (const c of [{massA: 0}, {gravity: NaN}, {speed: Infinity}, {restitution: 1.1}, {mode: 'unknown'}, {massB: '1'}]) {
    assert.throws(() => validatePhysicsConfig(c));
  }
  const world = new SimplePhysics(); const exported = world.snapshot(); exported.bodies[0].position[0] = 100;
  near(world.bodies[0].position[0], -.18);
});

test('physics view places blocks in Base coordinates, highlights drag and disposes cleanly', () => {
  const scene = new Group(), base = new Group(); base.applyMatrix4(scene_T_base); scene.add(base);
  const world = new SimplePhysics({mode: 'drop'}), view = new PhysicsView(base);
  view.root.visible = true; view.update(world.snapshot(), 'drop'); scene.updateMatrixWorld(true);
  const point = view.blocks[0].mesh.getWorldPosition(new Vector3());
  near(point.x, 0); near(point.y, .38); near(point.z, 0);
  assert.equal(view.blocks[1].mesh.visible, false);
  view.update({...world.snapshot(), dragId: 'A'}, 'drop'); assert.equal(view.blocks[0].mesh.material.emissive.getHex(), 0x254b35);
  view.dispose(); assert.equal(view.root.parent, null);
});
