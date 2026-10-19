import { Box3, Matrix4, Quaternion, Vector3 } from './vendor/three.module.js';
import { virtualToolMatrix } from './virtual-tool.js?v=20261007-solid4';
import { BLOCK_SIDE_M } from './simple-physics.js';
import { bodyBox, boxRadius } from './solid-collisions.js?v=20261007-solid4';
import { bodyLabel, bodySize } from './objects.js';

export const GRASP_RADIUS_M = .035;
export const blockName = (id, world) => bodyLabel(world?.bodies.find(b => b.id === id) ?? {id});

function padBounds(rig, name) {
  const node = rig.root.getObjectByName(name), box = new Box3();
  const inverse = rig.joints[5].node.matrixWorld.clone().invert();
  node?.traverse(mesh => {
    if (!mesh.geometry) return;
    const transform = new Matrix4().multiplyMatrices(inverse, mesh.matrixWorld);
    const vertices = mesh.geometry.attributes.position, point = new Vector3();
    for (let i = 0; i < vertices.count; i++) box.expandByPoint(point.fromBufferAttribute(vertices, i).applyMatrix4(transform));
  });
  return box;
}
export function gripperOpeningM(rig) {
  const left = padBounds(rig, 'polySurface45'), right = padBounds(rig, 'polySurface47');
  return left.isEmpty() || right.isEmpty() ? null : right.min.y - left.max.y;
}
export function contactGripperRatio(rig, p, width = BLOCK_SIDE_M) {
  let low = 0, high = 1;
  try {
    for (let i = 0; i < 10; i++) {
      const gripper = (low + high) / 2; rig.apply({...p, gripper});
      const opening = gripperOpeningM(rig);
      if (opening == null) return .96;
      if (opening < width) low = gripper; else high = gripper;
    }
    return high;
  } finally { rig.apply(p); }
}

// A proximity-gated kinematic constraint for browser interaction, not force closure.
export class VirtualGrasp {
  constructor(world) { this.world = world; this.attachment = null; this.armed = false; this.lastMs = null; this.message = '选物块后点击“靠近物块”，再闭合夹爪'; }
  arm(id = null) { this.targetId = id; this.armed = true; }
  release() {
    const id = this.world.releaseHeld(); this.attachment = null;
    if (id) this.message = `已松开${blockName(id, this.world)} · 物块继续受重力影响`;
  }
  reset() { this.release(); this.armed = false; this.targetId = null; this.lastMs = null; }
  update(rig, base, p, nowMs) {
    const tool = virtualToolMatrix(rig, base), center = new Vector3().setFromMatrixPosition(tool);
    if (this.attachment?.id !== this.world.heldId) this.attachment = null;
    if (p.gripper >= .99) { this.release(); this.armed = true; }
    else if (this.armed && !this.attachment) {
      this.armed = false;
      const candidates = this.world.bodies.filter(body => this.world.drag?.id !== body.id && (!this.targetId || body.id === this.targetId))
        .map(body => ({body, distance: center.distanceTo(new Vector3(...body.position))})).sort((a, b) => a.distance - b.distance);
      const nearest = candidates[0];
      const offset = nearest && new Vector3(...nearest.body.position).applyMatrix4(tool.clone().invert());
      const width = nearest && 2 * (boxRadius(bodyBox(nearest.body), new Vector3(0, 1, 0).transformDirection(tool)) + Math.abs(offset.y));
      rig.apply({...p, gripper: 1}); const maximumOpening = gripperOpeningM(rig); rig.apply(p);
      const fits = maximumOpening != null && width <= maximumOpening + .0001;
      if (nearest?.distance <= GRASP_RADIUS_M && fits && this.world.attach(nearest.body.id)) {
        this.attachment = {id: nearest.body.id,
          offset,
          rotation: new Quaternion().setFromRotationMatrix(tool).invert().multiply(new Quaternion(...nearest.body.rotation)),
          ratio: contactGripperRatio(rig, p, width)};
        this.message = `已夹住${blockName(nearest.body.id, this.world)} · 可以抬高、移动或拖动关节`;
      } else this.message = nearest?.distance <= GRASP_RADIUS_M ? '物块未对准夹爪开口 · 请重新靠近或调整方向' : '夹爪附近没有物块 · 请先靠近物块再闭合';
    }
    if (this.attachment) {
      const position = this.attachment.offset.clone().applyMatrix4(tool);
      const rotation = new Quaternion().setFromRotationMatrix(tool).multiply(this.attachment.rotation).normalize();
      this.world.moveHeld(position.toArray(), rotation.toArray(), this.lastMs == null ? 0 : Math.max(0, (nowMs - this.lastMs) / 1000));
    }
    this.lastMs = nowMs;
    return this.attachment ? {...p, gripper: Math.max(p.gripper, this.attachment.ratio)} : p;
  }
  heldAt(rig, base, p) {
    if (!this.attachment || this.world.heldId !== this.attachment.id) return null;
    rig.apply(p); const tool = virtualToolMatrix(rig, base);
    const body = this.world.bodies.find(b => b.id === this.attachment.id);
    return {id: this.attachment.id, sizeM: [...bodySize(body)], position: this.attachment.offset.clone().applyMatrix4(tool).toArray(),
      rotation: new Quaternion().setFromRotationMatrix(tool).multiply(this.attachment.rotation).normalize().toArray()};
  }
  displayPose(p) { return this.attachment ? {...p, gripper: Math.max(p.gripper, this.attachment.ratio)} : p; }
}
