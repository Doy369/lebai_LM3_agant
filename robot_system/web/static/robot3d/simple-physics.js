// Educational translational model. Base axes: X/Y horizontal, Z up; SI units.
// Head-on AABB blocks, no rotation, no robot contact or gripper dynamics.
export const PHYSICS_DEFAULTS = Object.freeze({
  mode: 'collision', massA: 1, massB: 1, speed: .8, height: .35,
  gravity: 9.81, restitution: .8, pauseOnImpact: true,
});
export const PHYSICS_STEP_S = 1 / 240;
export const BLOCK_SIDE_M = .06;
const HALF = BLOCK_SIDE_M / 2;
const LIMIT = .4 - HALF;
const momentum = body => body.velocity.map(v => body.mass * v);
const energy = body => .5 * body.mass * body.velocity.reduce((sum, v) => sum + v * v, 0);

function finite(value, label, low, high) {
  if (typeof value !== 'number' || !Number.isFinite(value) || value < low || value > high) {
    throw Error(`${label} 必须在 ${low}～${high} 之间`);
  }
  return value;
}
export function validatePhysicsConfig(value) {
  const c = {...PHYSICS_DEFAULTS, ...value};
  if (!['drop', 'collision'].includes(c.mode)) throw Error('物理场景无效');
  if (typeof c.pauseOnImpact !== 'boolean') throw Error('碰撞暂停选项无效');
  return {
    mode: c.mode, massA: finite(c.massA, '物块 A 质量(kg)', .05, 10),
    massB: finite(c.massB, '物块 B 质量(kg)', .05, 10),
    speed: finite(c.speed, '初速度(m/s)', 0, 3),
    height: finite(c.height, '下落高度(m)', .08, .6),
    gravity: finite(c.gravity, '重力(m/s²)', 0, 20),
    restitution: finite(c.restitution, '弹性系数', 0, 1), pauseOnImpact: c.pauseOnImpact,
  };
}

export class SimplePhysics {
  constructor(config = PHYSICS_DEFAULTS) { this.reset(config); }
  reset(config = this.config) {
    this.config = validatePhysicsConfig(config); this.time = 0; this.accumulator = 0;
    this.running = false; this.lastImpact = null; this.impactCount = 0;
    const drop = this.config.mode === 'drop';
    this.bodies = [{
      id: 'A', mass: this.config.massA, position: [drop ? 0 : -.18, 0, drop ? HALF + this.config.height : HALF],
      velocity: [drop ? 0 : this.config.speed, 0, 0],
    }];
    if (!drop) this.bodies.push({id: 'B', mass: this.config.massB, position: [.10, 0, HALF], velocity: [0, 0, 0]});
    return this.snapshot();
  }
  start() { this.running = true; }
  pause() { this.running = false; this.accumulator = 0; }
  recordImpact(kind, first, second, before) {
    const after = this.bodies.map(body => ({id: body.id, velocity: [...body.velocity], momentum: momentum(body), kineticJ: energy(body)}));
    const firstBefore = before.find(body => body.id === first);
    const firstAfter = after.find(body => body.id === first);
    const involved = second === 'B' ? ['A', 'B'] : [first];
    const sum = (list, field) => list.filter(body => involved.includes(body.id)).reduce((total, body) => {
      body[field].forEach((value, axis) => { total[axis] += value; }); return total;
    }, [0, 0, 0]);
    this.lastImpact = {
      kind, first, second, atTimeS: this.time, before, after,
      impulseOnFirstNs: firstAfter.momentum.map((p, i) => p - firstBefore.momentum[i]),
      totalMomentumBefore: sum(before, 'momentum'), totalMomentumAfter: sum(after, 'momentum'),
      kineticBeforeJ: before.filter(body => involved.includes(body.id)).reduce((sum, b) => sum + b.kineticJ, 0),
      kineticAfterJ: after.filter(body => involved.includes(body.id)).reduce((sum, b) => sum + b.kineticJ, 0),
    };
    this.impactCount++;
    if (this.config.pauseOnImpact) this.running = false;
  }
  captureBodies() {
    return this.bodies.map(body => ({id: body.id, velocity: [...body.velocity], momentum: momentum(body), kineticJ: energy(body)}));
  }
  step(dt = PHYSICS_STEP_S) {
    finite(dt, '积分步长(s)', 0, .1);
    if (this.config.mode === 'drop') {
      const body = this.bodies[0], g = this.config.gravity;
      if (body.position[2] <= HALF + 1e-10 && Math.abs(body.velocity[2]) < .03) {
        body.position[2] = HALF; body.velocity[2] = 0; this.time += dt; return;
      }
      const height = body.position[2] - HALF, vz = body.velocity[2];
      const hitTime = g > 0 ? (vz + Math.sqrt(vz * vz + 2 * g * Math.max(0, height))) / g
        : vz < 0 ? -height / vz : Infinity;
      if (hitTime >= 0 && hitTime <= dt) {
        body.velocity[2] -= g * hitTime; body.position[2] = HALF; this.time += hitTime;
        const before = this.captureBodies(); body.velocity[2] *= -this.config.restitution;
        if (Math.abs(body.velocity[2]) < .03) body.velocity[2] = 0;
        this.recordImpact('floor', 'A', '演示平台', before);
        // Stop at the event so displayed before/after values are the actual contact state.
        return;
      }
      body.position[2] += vz * dt - .5 * g * dt * dt; body.velocity[2] -= g * dt;
      this.time += dt; return;
    }
    // Sweep to the earliest contact instead of resolving penetrations after a frame.
    let remaining = dt;
    for (let iteration = 0; iteration < 8 && remaining > 1e-12; iteration++) {
      const [a, b] = this.bodies;
      let hit = null, at = remaining + 1;
      const relative = a.velocity[0] - b.velocity[0];
      if (relative > 1e-12) {
        const t = Math.max(0, b.position[0] - a.position[0] - BLOCK_SIDE_M) / relative;
        if (t <= remaining) { at = t; hit = {kind: 'blocks', body: a}; }
      }
      for (const body of this.bodies) {
        const v = body.velocity[0];
        if (Math.abs(v) < 1e-12) continue;
        const t = ((v > 0 ? LIMIT : -LIMIT) - body.position[0]) / v;
        if (t >= -1e-10 && Math.max(0, t) < at && t <= remaining) {
          at = Math.max(0, t); hit = {kind: 'wall', body};
        }
      }
      const travel = hit ? at : remaining;
      for (const body of this.bodies) body.position[0] += body.velocity[0] * travel;
      this.time += travel; remaining -= travel;
      if (!hit) break;
      const before = this.captureBodies();
      if (hit.kind === 'blocks') {
        const impulse = (1 + this.config.restitution) * relative / (1 / a.mass + 1 / b.mass);
        a.velocity[0] -= impulse / a.mass; b.velocity[0] += impulse / b.mass;
        this.recordImpact('blocks', 'A', 'B', before);
      } else {
        hit.body.velocity[0] *= -this.config.restitution;
        this.recordImpact('wall', hit.body.id, '演示侧墙', before);
      }
      if (!this.running && this.config.pauseOnImpact) return;
    }
  }
  advance(frameSeconds) {
    finite(frameSeconds, '帧时长(s)', 0, 3600);
    if (!this.running) return this.snapshot();
    // Background-tab gaps are discarded; there is no burst of catch-up simulation.
    this.accumulator += Math.min(frameSeconds, .1);
    while (this.running && this.accumulator + 1e-12 >= PHYSICS_STEP_S) {
      this.step(); this.accumulator -= PHYSICS_STEP_S;
    }
    if (!this.running) this.accumulator = 0;
    return this.snapshot();
  }
  snapshot() {
    return {
      timeS: this.time, running: this.running, impactCount: this.impactCount,
      bodies: this.bodies.map(body => {
        const airborne = this.config.mode === 'drop' && (body.position[2] > HALF + 1e-9 || body.velocity[2] > 0);
        return {...body, position: [...body.position], velocity: [...body.velocity], momentum: momentum(body),
          acceleration: [0, 0, airborne ? -this.config.gravity : 0], kineticJ: energy(body)};
      }),
      lastImpact: this.lastImpact ? structuredClone(this.lastImpact) : null,
    };
  }
}
