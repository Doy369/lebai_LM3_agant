import { pose } from './validation.js';
export class TrajectoryPlayer {
  constructor() { this.stop(); }
  start(initial, segments, nowMs) {
    let previous = pose(initial);
    let at = 0;
    this.segments = segments.map(s => {
      const next = pose({joints: previous.joints.map((q, i) => s.joints[i] ?? q), gripper: s.gripper ?? previous.gripper});
      const result = {from: previous, to: next, start: at, duration: s.durationMs};
      previous = next; at += s.durationMs;
      return result;
    });
    this.startMs = nowMs; this.running = true;
  }
  sample(nowMs) {
    if (!this.running) return null;
    const elapsed = Math.max(0, nowMs - this.startMs);
    const s = this.segments.find(s => elapsed < s.start + s.duration) ?? this.segments.at(-1);
    const t = Math.min(1, (elapsed - s.start) / s.duration);
    if (elapsed >= this.segments.at(-1).start + this.segments.at(-1).duration) this.running = false;
    return {joints: s.from.joints.map((q, i) => q + (s.to.joints[i] - q) * t),
      gripper: s.from.gripper + (s.to.gripper - s.from.gripper) * t};
  }
  stop() { this.running = false; this.segments = []; }
}
