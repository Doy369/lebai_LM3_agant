import { pose, degToRad } from './validation.js';
import { UPRIGHT_REFERENCE_JOINTS_RAD } from './joints.js';

const home = () => ({joints: [...UPRIGHT_REFERENCE_JOINTS_RAD], gripper: 0});
function target(delta, gripper = 0) {
  return {joints: home().joints.map((q, i) => q + degToRad(delta[i] ?? 0)), gripper};
}
const STEPS = [
  {label: '准备姿态', durationMs: 2000, to: target([])},
  {label: '向左转动', durationMs: 2800, to: target([35, 10])},
  {label: '向右转动', durationMs: 2800, to: target([-35, 10])},
  {label: '伸展机械臂', durationMs: 3000, to: target([20, 28, -20, 12])},
  {label: '转动手腕', durationMs: 2800, to: target([20, 28, -20, 32, 40, 45])},
  {label: '夹爪张开', durationMs: 1200, to: target([20, 28, -20, 32, 40, 45], 1)},
  {label: '夹爪闭合', durationMs: 1200, to: target([20, 28, -20, 32, 40, 45])},
  {label: '夹爪再次张开', durationMs: 1200, to: target([20, 28, -20, 32, 40, 45], 1)},
  {label: '物块下落与反弹', durationMs: 3200, to: target([15, 12, -10], 1), cue: 'drop'},
  {label: '返回起点', durationMs: 3000, to: target([])},
];
export const SHOWCASE_DURATION_MS = STEPS.reduce((sum, step) => sum + step.durationMs, 0);

// Local presentation clock: no controller, API request or compiled-plan event.
export class ShowcasePlayer {
  constructor() { this.stop(); }
  start(initial, nowMs, {loop = true} = {}) {
    this.initial = pose(initial); this.lastMs = nowMs; this.elapsedMs = 0;
    this.active = true; this.paused = false; this.loop = loop; this.cycle = 1;
    this.fired = new Set(); this.pendingReset = true;
  }
  setPaused(value) { this.paused = Boolean(value); this.lastMs = null; }
  stop() { this.active = false; this.paused = false; this.lastMs = null; }
  tick(nowMs) {
    if (!this.active) return null;
    const cues = [];
    if (!this.paused) {
      // Match the physics world's maximum frame step; background time is not replayed.
      if (this.lastMs != null) this.elapsedMs += Math.min(50, Math.max(0, nowMs - this.lastMs));
      this.lastMs = nowMs;
      if (this.elapsedMs >= SHOWCASE_DURATION_MS) {
        if (this.loop) {
          this.elapsedMs -= SHOWCASE_DURATION_MS; this.cycle++;
          this.initial = home(); this.fired.clear(); this.pendingReset = true;
        } else { this.elapsedMs = SHOWCASE_DURATION_MS; this.active = false; }
      }
      if (this.pendingReset) { cues.push('reset'); this.pendingReset = false; }
    }
    let start = 0, from = this.initial, current = STEPS.at(-1), progress = 1;
    for (let index = 0; index < STEPS.length; index++) {
      const step = STEPS[index];
      if (!this.paused && this.elapsedMs >= start && step.cue && !this.fired.has(index)) {
        cues.push(step.cue); this.fired.add(index);
      }
      if (this.elapsedMs < start + step.durationMs || index === STEPS.length - 1) {
        current = step; progress = Math.min(1, Math.max(0, (this.elapsedMs - start) / step.durationMs)); break;
      }
      start += step.durationMs; from = step.to;
    }
    const eased = progress * progress * (3 - 2 * progress);
    const currentPose = eased === 1 ? structuredClone(current.to) : eased === 0 ? structuredClone(from)
      : {joints: from.joints.map((q, i) => q + (current.to.joints[i] - q) * eased),
        gripper: from.gripper + (current.to.gripper - from.gripper) * eased};
    return {pose: currentPose,
      label: current.label, progress: this.elapsedMs / SHOWCASE_DURATION_MS, cycle: this.cycle,
      paused: this.paused, completed: !this.active, cues};
  }
}
