import { vector } from './validation.js';
export function decodeFeedback(payload, receivedMs, requestedMs, previousTimestamp = null) {
  const d = payload?.data;
  if (payload?.success !== true || !d || d.is_connected !== true) throw Error('反馈断开或连接状态未知');
  if (d.joint_unit !== undefined && d.joint_unit !== 'radian') throw Error('反馈关节单位无效');
  const joints = vector(d.kin_data?.actual_joint_pose, 6, '反馈关节(rad)');
  const timestamp = d.feedback_read_at_unix_ms;
  const timed = typeof timestamp === 'number' && Number.isFinite(timestamp);
  if (timed && (timestamp > receivedMs + 1000 || (previousTimestamp !== null && timestamp <= previousTimestamp))) throw Error('反馈时间戳倒退、重复或时钟不一致');
  const ageMs = timed ? Math.max(receivedMs - timestamp, receivedMs - requestedMs, 0) : 0;
  return {joints, receivedMs, timestamp: timed ? timestamp : null, ageMs,
    source: d.dry_run === true ? 'simulated' : d.dry_run === false ? 'robot' : 'unknown',
    timed: timed && d.feedback_time_source === 'server_read_completion'};
}
export class FeedbackSync {
  constructor({read, onSample, onStatus, now = Date.now, intervalMs = 1000, staleMs = 3500, timeoutMs = 3000}) {
    Object.assign(this, {read, onSample, onStatus, now, intervalMs, staleMs, timeoutMs});
    this.generation = 0; this.active = false; this.sample = null; this.error = null; this.hasPose = false;
  }
  status() {
    if (!this.active) return {kind: 'manual', text: '手动预览 · 不读取实时姿态'};
    const s = this.sample;
    const retained = this.hasPose ? '保留最后姿态' : '暂无有效反馈姿态';
    if (this.error) return {kind: 'disconnected', text: `反馈断开：${this.error} · ${retained}`};
    if (!s) return {kind: 'waiting', text: '等待读取 · 暂无反馈姿态'};
    const age = s.ageMs + Math.max(0, this.now() - s.receivedMs);
    if (age > this.staleMs) return {kind: 'stale', text: `反馈过期 ${Math.round(age)} ms · ${retained}`};
    const prefix = s.source === 'simulated' ? '模拟反馈（Dry Run）' : s.source === 'robot' ? '真机只读反馈' : '来源未知';
    return {kind: s.source === 'simulated' ? 'simulated' : s.timed ? 'recent' : 'untimed',
      text: `${prefix} · ${s.timed ? '服务端读取完成' : '无采样时间，实时性未验证'} · ${Math.round(age)} ms`};
  }
  start() {
    this.stop(); this.active = true; this.sample = null; this.error = null; this.hasPose = false;
    this.watchdog = setInterval(() => this.onStatus(this.status()), 250);
    void this.poll(this.generation);
  }
  async poll(generation) {
    if (!this.active || generation !== this.generation) return;
    const controller = new AbortController(); this.controller = controller;
    const requestedMs = this.now();
    const timeout = setTimeout(() => controller.abort(), this.timeoutMs);
    try {
      const payload = await this.read(controller.signal);
      if (controller.signal.aborted) throw Error('请求超时');
      if (!this.active || generation !== this.generation) return;
      const next = decodeFeedback(payload, this.now(), requestedMs, this.sample?.timestamp);
      this.sample = next; this.error = null;
      if (next.ageMs <= this.staleMs) { this.onSample(next.joints); this.hasPose = true; }
    } catch (e) {
      if (this.active && generation === this.generation) this.error = e.name === 'AbortError' ? '请求超时' : e.message;
    } finally {
      clearTimeout(timeout);
      if (this.active && generation === this.generation) {
        this.onStatus(this.status()); this.timer = setTimeout(() => this.poll(generation), this.intervalMs);
      }
    }
  }
  stop() {
    this.active = false; this.generation++; this.controller?.abort();
    clearTimeout(this.timer); clearInterval(this.watchdog);
  }
}
