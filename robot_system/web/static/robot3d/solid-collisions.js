import { Box3, BoxGeometry, EdgesGeometry, Group, LineBasicMaterial, LineSegments, Matrix4, Quaternion, Vector3 } from './vendor/three.module.js';
import { bodySize } from './objects.js';

const AXES = [new Vector3(1, 0, 0), new Vector3(0, 1, 0), new Vector3(0, 0, 1)];
export function solidBox(position, size, rotation = [0, 0, 0, 1]) {
  const q = new Quaternion(...rotation).normalize();
  return {center: new Vector3(...position), half: size.map(v => v / 2), axes: AXES.map(a => a.clone().applyQuaternion(q))};
}
export function bodyBox(body) { return solidBox(body.position, bodySize(body), body.rotation); }
export function boxRadius(box, axis) { return box.axes.reduce((r, a, i) => r + box.half[i] * Math.abs(a.dot(axis)), 0); }
export function boxExtents(box) { return AXES.map(a => boxRadius(box, a)); }

// Separating-axis test on oriented, solid boxes. The normal points from a to b.
// Touching faces are allowed; positive depth means volume intersection.
export function boxContact(a, b, tolerance = 1e-9) {
  const delta = b.center.clone().sub(a.center);
  if (delta.lengthSq() > (Math.hypot(...a.half) + Math.hypot(...b.half)) ** 2) return null;
  let depth = Infinity, normal;
  const axes = [...a.axes, ...b.axes];
  for (const x of a.axes) for (const y of b.axes) {
    const cross = x.clone().cross(y); if (cross.lengthSq() > 1e-12) axes.push(cross.normalize());
  }
  for (const axis of axes) {
    const distance = delta.dot(axis), overlap = boxRadius(a, axis) + boxRadius(b, axis) - Math.abs(distance);
    if (overlap <= tolerance) return null;
    if (overlap < depth) { depth = overlap; normal = axis.clone().multiplyScalar(distance >= 0 ? 1 : -1); }
  }
  return {depth, normal};
}

// Slice long CAD meshes before taking bounds. A single box around a bent link
// fills empty corners and falsely labels the normal wrist mounting as a collision.
function meshBounds(mesh) {
  const whole = mesh.geometry.boundingBox, size = whole.getSize(new Vector3()), scale = new Vector3().setFromMatrixScale(mesh.matrixWorld);
  const lengths = size.toArray().map((v, i) => v * scale.getComponent(i));
  const axis = lengths.indexOf(Math.max(...lengths));
  const count = /^ARM/.test(mesh.name) ? Math.max(1, Math.min(24, Math.ceil(lengths[axis] / .02))) : 1;
  if (count === 1) return [whole.clone()];
  const attribute = mesh.geometry.attributes.position, index = mesh.geometry.index, boxes = [];
  const vertex = i => new Vector3().fromBufferAttribute(attribute, index ? index.getX(i) : i);
  for (let slice = 0; slice < count; slice++) {
    const low = whole.min.getComponent(axis) + size.getComponent(axis) * slice / count;
    const high = whole.min.getComponent(axis) + size.getComponent(axis) * (slice + 1) / count, box = new Box3();
    for (let t = 0; t < (index?.count ?? attribute.count); t += 3) {
      const points = [vertex(t), vertex(t + 1), vertex(t + 2)];
      for (let k = 0; k < 3; k++) {
        const a = points[k], b = points[(k + 1) % 3], x = a.getComponent(axis), y = b.getComponent(axis);
        if (x >= low && x <= high) box.expandByPoint(a);
        for (const plane of [low, high]) if ((x < plane && y > plane) || (x > plane && y < plane))
          box.expandByPoint(a.clone().lerp(b, (plane - x) / (y - x)));
      }
    }
    if (!box.isEmpty()) boxes.push(box);
  }
  return boxes;
}

export class RobotSolids {
  constructor(rig, base, world) {
    this.rig = rig; this.base = base; this.world = world; this.parts = [];
    base.updateWorldMatrix(true, true);
    rig.root.traverse(mesh => {
      if (!mesh.isMesh) return;
      let ancestor = mesh, joint = 0;
      while (ancestor) {
        const match = /^Joint([1-6])$/.exec(ancestor.name);
        if (match) { joint = Number(match[1]); break; } ancestor = ancestor.parent;
      }
      mesh.geometry.computeBoundingBox();
      const bounds = mesh.geometry.boundingBox;
      if (!bounds || bounds.isEmpty()) return;
      for (const piece of meshBounds(mesh)) this.parts.push({mesh, joint, center: piece.getCenter(new Vector3()), half: piece.getSize(new Vector3()).multiplyScalar(.5).toArray(),
        label: /^ARM/.test(mesh.name) ? (joint ? `连杆 ${joint}` : '底座') : '夹爪 / 相机支架',
        finger: /polySurface(4[24567]|5[239]|60)_/.test(mesh.name)});
    });
  }
  boxes() {
    this.base.updateWorldMatrix(true, true);
    const inverse = this.base.matrixWorld.clone().invert();
    return this.parts.map(part => {
      const m = new Matrix4().multiplyMatrices(inverse, part.mesh.matrixWorld), scale = new Vector3().setFromMatrixScale(m);
      const axes = [0, 1, 2].map(i => new Vector3().setFromMatrixColumn(m, i).normalize());
      return {...part, center: part.center.clone().applyMatrix4(m), half: part.half.map((v, i) => v * scale.getComponent(i)), axes};
    });
  }
  check(p, heldBody = null, {blocks = true, wrist = false} = {}) {
    if (p.joints.some(q => !Number.isFinite(q) || Math.abs(q) > Math.PI + 1e-8)) return '关节预览限位';
    this.rig.apply(p); const boxes = this.boxes(), environment = this.world.environment;
    const inverse = this.base.matrixWorld.clone().invert();
    const mounts = this.rig.joints.map(j => new Vector3().setFromMatrixPosition(j.node.matrixWorld).applyMatrix4(inverse));
    if (wrist) {
      const a = new Vector3(0, 0, 1).transformDirection(this.rig.joints[3].node.matrixWorld);
      const b = new Vector3(0, 0, 1).transformDirection(this.rig.joints[5].node.matrixWorld);
      if (Math.abs(a.dot(b)) > .999) return '腕部接近奇异死区，请换个姿态';
    }
    const table = environment.table && solidBox(environment.table.position, environment.table.size);
    const obstacles = (environment.obstacles ?? []).map(o => ({...solidBox(o.position, o.size), label: o.label ?? '固定物块'}));
    for (let i = 0; i < boxes.length; i++) {
      const a = boxes[i];
      // The fixed pedestal is mounted on the table; all movable pieces remain solid.
      if (a.joint && a.center.z - boxRadius(a, AXES[2]) < environment.groundZ - .0002) return `${a.label} ↔ 地面`;
      if (a.joint && table && boxContact(a, table, .0002)) return `${a.label} ↔ 桌面`;
      for (const obstacle of obstacles) if (boxContact(a, obstacle, .0002)) return `${a.label} ↔ ${obstacle.label}`;
      for (let j = i + 1; j < boxes.length; j++) {
        const b = boxes[j];
        // Adjacent link housings overlap intentionally at the shared revolute joint.
        if (a.joint === b.joint) continue;
        if (Math.abs(a.joint - b.joint) === 1) {
          const mount = mounts[Math.max(a.joint, b.joint) - 1];
          if ([a, b].every(part => part.center.distanceTo(mount) - Math.hypot(...part.half) < .08)) continue;
        }
        if (boxContact(a, b, .0002)) return `${a.label} ↔ ${b.label}（自身碰撞死区）`;
      }
      if (blocks) for (const body of this.world.bodies) {
        const held = body.id === this.world.heldId, candidate = held ? heldBody ?? body : body;
        // Only the two terminal finger faces may contact a grasped cube.
        if (held && a.finger) continue;
        if (boxContact(a, bodyBox(candidate), .0003)) return `${a.label} ↔ 物块 ${body.id}`;
      }
    }
    if (heldBody) {
      const box = bodyBox(heldBody);
      if (box.center.z - boxRadius(box, AXES[2]) < environment.groundZ - .0002) return '夹持物块 ↔ 地面';
      if (table && boxContact(box, table, .0002)) return '夹持物块 ↔ 桌面';
      for (const obstacle of obstacles) if (boxContact(box, obstacle, .0002)) return `夹持物块 ↔ ${obstacle.label}`;
      for (const body of this.world.bodies) if (body.id !== this.world.heldId && boxContact(box, bodyBox(body), .0002)) return `夹持物块 ↔ 物块 ${body.id}`;
    }
    return null;
  }
  // Sweep every attempted movement, including large slider jumps and delayed frames.
  sweep(from, to, heldAt = () => null, options) {
    const delta = to.joints.map((q, i) => q - from.joints[i]);
    const count = Math.max(1, Math.ceil(Math.max(...delta.map(Math.abs)) / .008), Math.ceil(Math.abs(to.gripper - from.gripper) / .025));
    const interpolate = t => ({joints: delta.map((d, i) => from.joints[i] + d * t), gripper: from.gripper + (to.gripper - from.gripper) * t});
    let previous = 0, reason = null;
    for (let i = 1; i <= count; i++) {
      const t = i / count, candidate = interpolate(t); reason = this.check(candidate, heldAt(candidate), options);
      if (reason) {
        let low = previous, high = t;
        for (let k = 0; k < 8; k++) {
          const middle = (low + high) / 2, p = interpolate(middle);
          if (this.check(p, heldAt(p), options)) high = middle; else low = middle;
        }
        const accepted = interpolate(low); this.rig.apply(accepted);
        return {pose: accepted, blocked: true, reason};
      }
      previous = t;
    }
    this.rig.apply(to); return {pose: to, blocked: false, reason: null};
  }
}

export class SolidVolumeView {
  constructor(base) {
    this.root = new Group(); this.root.visible = false; this.root.name = 'local-solid-volumes'; base.add(this.root);
    const cube = new BoxGeometry(1, 1, 1); this.geometry = new EdgesGeometry(cube); cube.dispose();
    this.material = new LineBasicMaterial({color: 0x0ba5a8, transparent: true, opacity: .7}); this.lines = [];
  }
  update(boxes, visible) {
    this.root.visible = visible; if (!visible) return;
    while (this.lines.length < boxes.length) { const line = new LineSegments(this.geometry, this.material); this.lines.push(line); this.root.add(line); }
    this.lines.forEach((line, i) => {
      line.visible = i < boxes.length; if (!line.visible) return;
      const box = boxes[i]; line.position.copy(box.center); line.scale.set(...box.half.map(v => v * 2));
      line.quaternion.setFromRotationMatrix(new Matrix4().makeBasis(...box.axes));
    });
  }
  dispose() { this.root.removeFromParent(); this.geometry.dispose(); this.material.dispose(); }
}
