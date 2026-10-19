import { BLOCK_SIDE_M } from './simple-physics.js';
import { bodyBox, solidBox, boxContact, boxExtents } from './solid-collisions.js?v=20261007-solid4';
import { bodySize, bodyLabel, bodyColor, validateObject, validateObjects, MAX_OBJECTS } from './objects.js';

const HALF = BLOCK_SIDE_M / 2;
export const LIVE_STEP_S = 1 / 240;
export const LIVE_DEFAULTS = Object.freeze({massA: 1, massB: 1, gravity: 9.81, restitution: .5, friction: .15});
const length = v => Math.hypot(...v);
const momentum = b => b.velocity.map(v => b.mass * v);
const capture = bodies => bodies.map(b => ({id: b.id, velocity: [...b.velocity], momentum: momentum(b),
  kineticJ: .5 * b.mass * b.velocity.reduce((sum, v) => sum + v * v, 0)}));

export function liveConfig(value = {}) {
  const c = {...LIVE_DEFAULTS, ...value};
  for (const [key, lo, hi] of [['massA', .05, 10], ['massB', .05, 10], ['gravity', 0, 20], ['restitution', 0, 1], ['friction', 0, 1]]) {
    if (typeof c[key] !== 'number' || !Number.isFinite(c[key]) || c[key] < lo || c[key] > hi) throw Error(`${key} 必须在 ${lo}～${hi} 之间`);
  }
  return c;
}

// Mouse spring is an external force. Integration and contacts use SI units in Base axes.
export class LivePhysics {
  constructor(config) {
    this.config = liveConfig(config); this.environment = {groundZ: 0, table: null, obstacles: []}; this.initialPositions = null;
    this.robotBoxes = []; this.customObjects = null; this.nextBodyId = 1; this.objectRevision = 0; this.reset();
  }
  setEnvironment({groundZ = 0, table = null, obstacles = [], initialPositions = null}) {
    if (!Number.isFinite(groundZ)) throw Error('地面高度无效');
    if (table && (!['position', 'size'].every(key => Array.isArray(table[key]) && table[key].length === 3 && table[key].every(Number.isFinite))
      || table.size.some(v => v <= 0))) throw Error('桌面几何无效');
    if (initialPositions && (initialPositions.length !== 2 || initialPositions.some(p => p.length !== 3 || !p.every(Number.isFinite)))) throw Error('物块初始位置无效');
    if (!Array.isArray(obstacles) || obstacles.some(o => !['position', 'size'].every(k => Array.isArray(o[k]) && o[k].length === 3 && o[k].every(Number.isFinite)) || o.size.some(v => v <= 0))) throw Error('固定物块几何无效');
    this.environment = {groundZ, table: table ? structuredClone(table) : null, obstacles: structuredClone(obstacles)};
    if (initialPositions) this.initialPositions = structuredClone(initialPositions);
  }
  supportHeight(body) {
    const {groundZ, table} = this.environment;
    const extent = boxExtents(bodyBox(body));
    if (table) {
      const top = table.position[2] + table.size[2] / 2;
      if ([0, 1].every(i => Math.abs(body.position[i] - table.position[i]) < table.size[i] / 2 + extent[i] - 1e-8)
        && body.position[2] >= top + extent[2] - 1e-6) return Math.max(groundZ, top);
    }
    return groundZ;
  }
  moveWithContacts(body, axis, distance) {
    if (body.id === this.heldId) return 0;
    const extent = boxExtents(bodyBox(body));
    const start = body.position[axis]; let end = start + distance;
    if (axis === 2) end = Math.max(this.environment.groundZ + extent[2], end);
    const table = this.environment.table;
    if (table && [0, 1, 2].filter(i => i !== axis).every(i => Math.abs(body.position[i] - table.position[i]) < table.size[i] / 2 + extent[i] - 1e-8)) {
      const low = table.position[axis] - table.size[axis] / 2 - extent[axis], high = table.position[axis] + table.size[axis] / 2 + extent[axis];
      if (start <= low + 1e-8 && distance > 0) end = Math.min(end, low);
      if (start >= high - 1e-8 && distance < 0) end = Math.max(end, high);
    }
    body.position[axis] = end; return Math.abs(end - start);
  }
  configure(config) {
    this.config = liveConfig({...this.config, ...config});
    for (const id of ['A', 'B']) {
      const key = id === 'A' ? 'massA' : 'massB'; if (!(key in config)) continue;
      const mass = this.config[key];
      const body = this.bodies.find(b => b.id === id); if (body) body.mass = mass;
      const saved = this.customObjects?.find(b => b.id === id); if (saved) saved.mass = mass;
    }
  }
  defaultObjects() {
    return ['A', 'B'].map((id, i) => ({id, label: bodyLabel({id}), color: bodyColor({id}), mass: i ? this.config.massB : this.config.massA,
      sizeM: [BLOCK_SIDE_M, BLOCK_SIDE_M, BLOCK_SIDE_M], rotation: [0, 0, 0, 1],
      position: this.initialPositions ? [...this.initialPositions[i]] : [i ? .12 : -.16, 0, this.environment.groundZ + HALF]}));
  }
  objectDefinitions(current = false) {
    return this.bodies.map(b => validateObject({id: b.id, label: bodyLabel(b), color: bodyColor(b), mass: b.mass,
      sizeM: bodySize(b), position: current ? b.position : b.initialPosition, rotation: current ? b.rotation : b.initialRotation}));
  }
  occupancyReason(object, ignoreId = null) {
    const box = bodyBox(object);
    if (object.position[2] - boxExtents(box)[2] < this.environment.groundZ - 1e-6) return '物块不能穿过地面';
    const table = this.environment.table;
    if (table && boxContact(box, solidBox(table.position, table.size), .00001)) return '物块与桌面相交';
    for (const obstacle of this.environment.obstacles) if (boxContact(box, solidBox(obstacle.position, obstacle.size), .00001)) return `位置被${obstacle.label ?? '固定物块'}占用`;
    if (this.robotBoxes.some(part => boxContact(box, part, .00001))) return '位置被机械臂占用';
    for (const b of this.bodies) if (b.id !== ignoreId && boxContact(box, bodyBox(b), .00001)) return `位置被${bodyLabel(b)}占用`;
    return null;
  }
  addBody(value) {
    if (this.bodies.length >= MAX_OBJECTS) throw Error(`场景最多支持 ${MAX_OBJECTS} 个物块`);
    let id; do { id = `C${this.nextBodyId++}`; } while (this.bodies.some(b => b.id === id));
    const object = validateObject({...value, id});
    const reason = this.occupancyReason(object); if (reason) throw Error(reason);
    this.customObjects = [...this.objectDefinitions(), object];
    const body = this.makeBody(object); this.bodies.push(body); this.objectRevision++; return body;
  }
  updateBody(id, values) {
    const body = this.bodies.find(b => b.id === id); if (!body) throw Error('请选择已有物块');
    if (this.heldId === id) throw Error('请先松开夹持物块，再修改尺寸或质量');
    const object = validateObject({...this.objectDefinitions(true).find(b => b.id === id), ...values, id});
    const top = this.environment.table && this.environment.table.position[2] + this.environment.table.size[2] / 2;
    const onTable = top != null && body.position[2] - boxExtents(bodyBox(body))[2] >= top - .001
      && [0, 1].every(i => Math.abs(body.position[i] - this.environment.table.position[i]) < this.environment.table.size[i] / 2);
    object.position[2] = Math.max(object.position[2], (onTable ? top : this.environment.groundZ) + boxExtents(bodyBox(object))[2]);
    const reason = this.occupancyReason(object, id); if (reason) throw Error(reason);
    this.customObjects = this.objectDefinitions().map(b => b.id === id ? object : b);
    Object.assign(body, this.makeBody(object)); this.objectRevision++;
    if (id === 'A' || id === 'B') this.config[id === 'A' ? 'massA' : 'massB'] = body.mass;
    if (this.drag?.id === id) this.endDrag(); return body;
  }
  removeBody(id) {
    if (this.heldId === id) throw Error('请先松开夹持物块，再删除');
    if (!this.bodies.some(b => b.id === id)) throw Error('请选择已有物块');
    this.customObjects = this.objectDefinitions().filter(b => b.id !== id);
    if (this.drag?.id === id) this.endDrag(); this.bodies = this.bodies.filter(b => b.id !== id); this.objectRevision++;
  }
  replaceObjects(values, {check = true} = {}) {
    const objects = validateObjects(values), previous = this.bodies;
    try {
      this.bodies = [];
      for (const object of objects) {
        const reason = check && this.occupancyReason(object); if (reason) throw Error(reason);
        this.bodies.push(this.makeBody(object));
      }
    } catch (error) { this.bodies = previous; throw error; }
    this.customObjects = objects; this.reset();
    for (const id of ['A', 'B']) {
      const body = this.bodies.find(b => b.id === id); if (body) this.config[id === 'A' ? 'massA' : 'massB'] = body.mass;
    }
  }
  makeBody(object) {
    return {...structuredClone(object), initialPosition: [...object.position], initialRotation: [...object.rotation],
      velocity: [0, 0, 0], acceleration: [0, 0, 0], dragForce: [0, 0, 0]};
  }
  reset() {
    this.time = 0; this.accumulator = 0; this.running = true; this.drag = null;
    this.lastImpact = null; this.impactCount = 0; this.heldId = null;
    this.bodies = (this.customObjects ?? this.defaultObjects()).map(object => this.makeBody(object)); this.objectRevision++;
  }
  setRunning(value) { this.running = Boolean(value); this.accumulator = 0; if (!value) this.endDrag(); }
  beginDrag(id) {
    if (!this.running || id === this.heldId) return false;
    const body = this.bodies.find(b => b.id === id); if (!body) return false;
    this.drag = {id, target: [...body.position]}; return true;
  }
  setDragTarget(position) {
    if (!this.drag) return;
    if (!Array.isArray(position) || position.length !== 3 || !position.every(Number.isFinite)) throw Error('拖拽目标坐标无效');
    const body = this.bodies.find(b => b.id === this.drag.id);
    this.drag.target = [position[0], position[1], Math.max(this.environment.groundZ + boxExtents(bodyBox(body))[2], position[2])];
  }
  endDrag() { this.drag = null; this.bodies?.forEach(b => { b.dragForce = [0, 0, 0]; }); }
  attach(id) {
    if (this.heldId || !this.bodies.some(body => body.id === id)) return false;
    this.endDrag(); this.heldId = id;
    const body = this.bodies.find(body => body.id === id);
    body.velocity = [0, 0, 0]; body.acceleration = [0, 0, 0]; return true;
  }
  moveHeld(position, rotation, seconds = 0) {
    if (!this.heldId) return;
    if (!Array.isArray(position) || position.length !== 3 || !position.every(Number.isFinite)
      || !Array.isArray(rotation) || rotation.length !== 4 || !rotation.every(Number.isFinite)
      || !Number.isFinite(seconds) || seconds < 0) throw Error('夹取物块的姿态无效');
    const body = this.bodies.find(body => body.id === this.heldId);
    body.velocity = position.map((v, i) => seconds > 0 ? (v - body.position[i]) / seconds : 0);
    const speed = length(body.velocity); if (speed > 3) body.velocity = body.velocity.map(v => v * 3 / speed);
    body.position = [...position]; body.rotation = [...rotation]; body.acceleration = [0, 0, 0];
  }
  releaseHeld() { const id = this.heldId; this.heldId = null; return id; }
  impact(kind, first, second, before) {
    const after = capture(this.bodies), ids = kind === 'blocks' ? [first, second] : [first];
    const sum = list => list.filter(b => ids.includes(b.id)).reduce((p, b) => p.map((v, i) => v + b.momentum[i]), [0, 0, 0]);
    const a = before.find(b => b.id === first), b = after.find(b => b.id === first);
    this.lastImpact = {kind, first, second, atTimeS: this.time, before, after,
      impulseOnFirstNs: b.momentum.map((v, i) => v - a.momentum[i]),
      totalMomentumBefore: sum(before), totalMomentumAfter: sum(after),
      kineticBeforeJ: before.filter(b => ids.includes(b.id)).reduce((v, b) => v + b.kineticJ, 0),
      kineticAfterJ: after.filter(b => ids.includes(b.id)).reduce((v, b) => v + b.kineticJ, 0)};
    this.impactCount++;
  }
  step(dt = LIVE_STEP_S) {
    if (!Number.isFinite(dt) || dt <= 0 || dt > LIVE_STEP_S) throw Error('实时积分步长无效');
    const minimumSide = Math.min(.06, ...this.bodies.flatMap(bodySize));
    const steps = Math.ceil(dt / Math.min(LIVE_STEP_S, minimumSide / 12) - 1e-10);
    if (steps > 1) { for (let i = 0; i < steps; i++) this.step(dt / steps); return; }
    this.time += dt;
    for (const b of this.bodies) {
      if (b.id === this.heldId) continue;
      const a = [0, 0, -this.config.gravity], force = [0, 0, 0];
      if (this.drag?.id === b.id) {
        // A damped spring lets a held body push other bodies rather than teleport through them.
        for (let axis = 0; axis < 3; axis++) force[axis] = 180 * (this.drag.target[axis] - b.position[axis]) - 12 * b.velocity[axis];
        force[2] += b.mass * this.config.gravity;
        for (let axis = 0; axis < 3; axis++) a[axis] += force[axis] / b.mass;
      } else if (b.position[2] <= this.supportHeight(b) + boxExtents(bodyBox(b))[2] + 1e-7 && b.velocity[2] <= 0) {
        a[2] = 0;
        const speed = Math.hypot(b.velocity[0], b.velocity[1]);
        if (speed > 0) {
          const brake = Math.min(this.config.friction * this.config.gravity, speed / dt);
          a[0] = -brake * b.velocity[0] / speed; a[1] = -brake * b.velocity[1] / speed;
        }
      }
      const before = [...b.velocity];
      for (let axis = 0; axis < 3; axis++) b.velocity[axis] += a[axis] * dt;
      // Limit interactive throw speed to keep collision substeps below one block width.
      const speed = length(b.velocity); if (speed > 3) b.velocity = b.velocity.map(v => v * 3 / speed);
      b.acceleration = b.velocity.map((v, i) => (v - before[i]) / dt);
      b.dragForce = force;
      for (let axis = 0; axis < 3; axis++) b.position[axis] += b.velocity[axis] * dt;
    }
    // Contacts are with the shared scene's ground/table and other blocks, with no side cage.
    const tableBox = this.environment.table && solidBox(this.environment.table.position, this.environment.table.size);
    for (let pass = 0; pass < 6; pass++) {
      for (const b of this.bodies) {
        if (b.id === this.heldId) continue;
        const minimumZ = this.environment.groundZ + boxExtents(bodyBox(b))[2];
        if (b.position[2] < minimumZ) { b.position[2] = minimumZ; this.staticImpact(b, 2, 1, '地面'); }
        for (const obstacle of [...(tableBox ? [tableBox] : []), ...this.environment.obstacles.map(o => ({...solidBox(o.position, o.size), label: o.label ?? '固定物块'})), ...this.robotBoxes]) {
          const contact = boxContact(obstacle, bodyBox(b));
          if (!contact) continue;
          const n = contact.normal.toArray();
          for (let i = 0; i < 3; i++) b.position[i] += n[i] * (contact.depth + 1e-8);
          const normalSpeed = b.velocity.reduce((v, x, i) => v + x * n[i], 0);
          if (normalSpeed < 0) {
            const beforeContact = capture(this.bodies);
            const bounce = Math.abs(normalSpeed) < .08 ? 0 : this.config.restitution;
            for (let i = 0; i < 3; i++) b.velocity[i] -= (1 + bounce) * normalSpeed * n[i];
            if (Math.abs(normalSpeed) >= .08) this.impact(n[2] > .5 ? 'floor' : 'wall', b.id, obstacle === tableBox ? '桌面' : obstacle.label ?? '机械臂', beforeContact);
          }
        }
      }
      for (let ai = 0; ai < this.bodies.length; ai++) for (let bi = ai + 1; bi < this.bodies.length; bi++) {
      const a = this.bodies[ai], b = this.bodies[bi], contact = boxContact(bodyBox(a), bodyBox(b), -1e-9);
      if (!contact) continue;
      const n = contact.normal.toArray(), axis = n.map(Math.abs).indexOf(Math.max(...n.map(Math.abs))), sign = n[axis] >= 0 ? 1 : -1;
      const invA = a.id === this.heldId ? 0 : 1 / a.mass,
        invB = b.id === this.heldId ? 0 : 1 / b.mass, inv = invA + invB;
      const correction = Math.max(0, contact.depth);
      for (let i = 0; i < 3; i++) {
        this.moveWithContacts(a, i, -n[i] * correction * invA / inv);
        this.moveWithContacts(b, i, n[i] * correction * invB / inv);
      }
      const remaining = boxContact(bodyBox(a), bodyBox(b));
      if (remaining) {
        let missing = remaining.depth;
        missing -= this.moveWithContacts(a, axis, -sign * missing);
        this.moveWithContacts(b, axis, sign * Math.max(0, missing));
      }
      const relative = n.reduce((v, x, i) => v + x * (a.velocity[i] - b.velocity[i]), 0);
      if (relative <= 1e-10) continue;
      const before = capture(this.bodies), impulse = (1 + (relative < .08 ? 0 : this.config.restitution)) * relative / inv;
      for (let i = 0; i < 3; i++) { a.velocity[i] -= n[i] * impulse * invA; b.velocity[i] += n[i] * impulse * invB; }
      if (relative >= .08) this.impact('blocks', a.id, b.id, before);
      }
    }
  }
  staticImpact(body, axis, sign, object) {
    if (body.velocity[axis] * sign >= 0) return;
    const speed = Math.abs(body.velocity[axis]), before = capture(this.bodies);
    body.velocity[axis] = speed < .08 ? 0 : -this.config.restitution * body.velocity[axis];
    if (speed >= .08) this.impact(axis === 2 && sign > 0 ? 'floor' : 'wall', body.id, object, before);
  }
  advance(seconds) {
    if (!Number.isFinite(seconds) || seconds < 0) throw Error('帧时间无效');
    if (!this.running) return;
    this.accumulator += Math.min(seconds, .05);
    while (this.accumulator + 1e-12 >= LIVE_STEP_S) { this.step(); this.accumulator -= LIVE_STEP_S; }
  }
  snapshot() {
    const captured = capture(this.bodies);
    return {timeS: this.time, running: this.running, dragId: this.drag?.id ?? null, heldId: this.heldId, impactCount: this.impactCount,
      totalMomentum: captured.reduce((p, b) => p.map((v, i) => v + b.momentum[i]), [0, 0, 0]),
      bodies: this.bodies.map((b, i) => ({...b, sizeM: [...b.sizeM], initialPosition: [...b.initialPosition], initialRotation: [...b.initialRotation], position: [...b.position], rotation: [...b.rotation], velocity: [...b.velocity],
        acceleration: [...b.acceleration], dragForce: [...b.dragForce], ...captured[i]})),
      lastImpact: this.lastImpact ? structuredClone(this.lastImpact) : null};
  }
}
