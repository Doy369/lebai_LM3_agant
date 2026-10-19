import { Matrix4, Plane, Raycaster, Vector2, Vector3 } from './vendor/three.module.js';
import { bodyBox, boxExtents } from './solid-collisions.js';

// Pick the real table's top plane in Base coordinates, independent of camera view.
export function pickTablePoint(camera, base, table, rect, x, y) {
  if (!table || rect.width <= 0 || rect.height <= 0 || x < rect.left || x > rect.left + rect.width || y < rect.top || y > rect.top + rect.height) return null;
  camera.updateMatrixWorld(); base.updateWorldMatrix(true, false);
  const caster = new Raycaster();
  caster.setFromCamera(new Vector2((x - rect.left) / rect.width * 2 - 1, 1 - (y - rect.top) / rect.height * 2), camera);
  const ray = caster.ray.clone().applyMatrix4(new Matrix4().copy(base.matrixWorld).invert());
  const top = table.position[2] + table.size[2] / 2;
  const point = ray.intersectPlane(new Plane(new Vector3(0, 0, 1), -top), new Vector3());
  if (!point || [0, 1].some(i => Math.abs(point.getComponent(i) - table.position[i]) > table.size[i] / 2)) return null;
  return point.toArray();
}

export function tableObject(world, spec, point, gap = 0) {
  const table = world.environment.table;
  if (!table) throw Error('当前场景没有可选桌面');
  const object = {...spec, position: [...point], rotation: spec.rotation ?? [0, 0, 0, 1]};
  const extents = boxExtents(bodyBox(object));
  if ([0, 1].some(i => Math.abs(point[i] - table.position[i]) + extents[i] > table.size[i] / 2 - .002)) throw Error('请选择桌面内部，给物块留出完整空间');
  object.position[2] = table.position[2] + table.size[2] / 2 + extents[2] + gap;
  return object;
}

export function freeTableObject(world, spec) {
  const table = world.environment.table;
  if (!table) throw Error('当前场景没有桌面');
  for (const x of [.22, .30, .15, .38, .46, -.23, -.34, .07, -.46]) for (const y of [-.28, .28, -.4, .4, -.15, .15, 0]) {
    try {
      const object = tableObject(world, spec, [table.position[0] + table.size[0] * x, table.position[1] + table.size[1] * y, 0]);
      if (!world.occupancyReason(object)) return object;
    } catch { /* Try another table location. */ }
  }
  throw Error('桌面没有足够的空位，请删除物块或选择其他位置');
}

export function placementTarget(world, body, point, attachment) {
  // At the planner's aligned tool pose, the saved relative rotation/offset are in Base axes.
  const object = tableObject(world, {...body, rotation: attachment.rotation.toArray()}, point, .01);
  const reason = world.occupancyReason(object, body.id); if (reason) throw Error(reason);
  return {object, toolPoint: object.position.map((v, i) => v - attachment.offset.getComponent(i))};
}
