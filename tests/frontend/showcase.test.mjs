import test from 'node:test';
import assert from 'node:assert/strict';
import { ShowcasePlayer, SHOWCASE_DURATION_MS } from '../../robot_system/web/static/robot3d/showcase.js';
import { zeroPose } from '../../robot_system/web/static/robot3d/validation.js';
import { UPRIGHT_REFERENCE_JOINTS_RAD } from '../../robot_system/web/static/robot3d/joints.js';

test('presentation plays all motions, drops once, finishes at home and can be stopped', () => {
  const player = new ShowcasePlayer(); player.start(zeroPose(), 0, {loop: false});
  const cues = [], labels = new Set(); let sample;
  for (let ms = 0; ms <= SHOWCASE_DURATION_MS; ms += 20) {
    sample = player.tick(ms); cues.push(...sample.cues); labels.add(sample.label);
    assert.ok(sample.pose.joints.every(q => Number.isFinite(q) && Math.abs(q) <= Math.PI));
  }
  assert.deepEqual(cues, ['reset', 'drop']); assert.equal(labels.size, 10);
  assert.equal(sample.completed, true); assert.equal(sample.progress, 1);
  assert.deepEqual(sample.pose.joints, [...UPRIGHT_REFERENCE_JOINTS_RAD]);
  assert.equal(player.tick(SHOWCASE_DURATION_MS + 20), null);
  player.start(zeroPose(), 0); player.stop(); assert.equal(player.tick(2000), null);
});

test('pause freezes the whole timeline and resume does not jump across elapsed wall time', () => {
  const player = new ShowcasePlayer(); player.start(zeroPose(), 0);
  player.tick(20); const before = player.tick(40); player.setPaused(true);
  const paused = player.tick(60000);
  assert.deepEqual(paused.pose, before.pose); assert.equal(paused.progress, before.progress);
  assert.deepEqual(paused.cues, []); player.setPaused(false);
  assert.deepEqual(player.tick(120000).pose, before.pose);
  assert.ok(player.tick(120020).progress > before.progress);
});

test('loop resets physics once per cycle and preserves a continuous home pose', () => {
  const player = new ShowcasePlayer(); player.start(zeroPose(), 0);
  const cues = []; let previous, next;
  for (let ms = 0; ms < SHOWCASE_DURATION_MS; ms += 20) {
    previous = player.tick(ms); cues.push(...previous.cues);
  }
  next = player.tick(SHOWCASE_DURATION_MS); cues.push(...next.cues);
  assert.equal(next.cycle, 2); assert.equal(next.completed, false);
  assert.deepEqual(next.pose.joints, [...UPRIGHT_REFERENCE_JOINTS_RAD]);
  assert.ok(Math.max(...previous.pose.joints.map((q, i) => Math.abs(q - next.pose.joints[i]))) < .001);
  for (let ms = SHOWCASE_DURATION_MS + 20; ms < SHOWCASE_DURATION_MS * 2; ms += 20) cues.push(...player.tick(ms).cues);
  assert.deepEqual(cues, ['reset', 'drop', 'reset', 'drop']);
});
