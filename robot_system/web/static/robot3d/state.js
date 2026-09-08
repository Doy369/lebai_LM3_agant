import { pose, zeroPose, vector } from './validation.js';
import { TrajectoryPlayer } from './trajectory.js';
export class PoseState {
  constructor() { this.mode = 'manual'; this.manual = zeroPose(); this.feedback = null; this.player = new TrajectoryPlayer(); }
  setMode(mode) {
    if (!['manual', 'feedback'].includes(mode)) throw Error('未知姿态模式');
    this.player.stop(); this.mode = mode;
  }
  setManual(value) {
    if (this.mode !== 'manual') throw Error('实时反馈模式不接受手动姿态');
    const next = pose(value); this.player.stop(); this.manual = next;
  }
  acceptFeedback(joints) { this.feedback = {joints: vector(joints, 6, '反馈关节(rad)'), gripper: 0}; }
  play(segments, nowMs) {
    if (this.mode !== 'manual') throw Error('请先切换到手动预览');
    this.player.start(this.manual, segments, nowMs);
  }
  reset() { this.setManual(zeroPose()); }
  current(nowMs) {
    if (this.mode === 'feedback') return this.feedback;
    const next = this.player.sample(nowMs);
    if (next) this.manual = next;
    return this.manual;
  }
}
