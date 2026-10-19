import { Matrix4, Quaternion, Vector3 } from './vendor/three.module.js';
import { pose, vector } from './validation.js';
import { UPRIGHT_REFERENCE_JOINTS_RAD } from './joints.js';

// Display grasp point between the bundled GLB's finger pads, in Joint6 coordinates.
// This is a browser interaction frame, not a calibrated physical TCP.
export const VIRTUAL_TOOL_OFFSET = Object.freeze([0, .0003, -.175]);
export function virtualToolMatrix(rig, base) {
  base.updateWorldMatrix(true, false); rig.root.updateWorldMatrix(true, true);
  return new Matrix4().multiplyMatrices(base.matrixWorld.clone().invert(), rig.joints[5].node.matrixWorld)
    .multiply(new Matrix4().makeTranslation(...VIRTUAL_TOOL_OFFSET));
}

function linearSolve(matrix, values) {
  const rows = matrix.map((row, i) => [...row, values[i]]), n = rows.length;
  for (let i = 0; i < n; i++) {
    let pivot = i; for (let j = i + 1; j < n; j++) if (Math.abs(rows[j][i]) > Math.abs(rows[pivot][i])) pivot = j;
    if (Math.abs(rows[pivot][i]) < 1e-12) return null;
    [rows[i], rows[pivot]] = [rows[pivot], rows[i]];
    const divisor = rows[i][i]; for (let k = i; k <= n; k++) rows[i][k] /= divisor;
    for (let j = 0; j < n; j++) if (j !== i) {
      const factor = rows[j][i]; for (let k = i; k <= n; k++) rows[j][k] -= factor * rows[i][k];
    }
  }
  return rows.map(row => row[n]);
}

export function solveVirtualIK(rig, base, target, initial, {vertical = true, align = false, accept = () => true} = {}) {
  const goal = new Vector3(...vector(target, 3, '末端位置')), original = pose(initial);
  const measure = joints => {
    rig.apply({joints, gripper: original.gripper}); const m = virtualToolMatrix(rig, base);
    const position = new Vector3().setFromMatrixPosition(m), axis = new Vector3(0, 0, 1).transformDirection(m);
    if (align) {
      const q = new Quaternion().setFromRotationMatrix(m).normalize();
      if (q.w < 0) q.set(-q.x, -q.y, -q.z, -q.w);
      const sine = Math.hypot(q.x, q.y, q.z), factor = sine > 1e-9 ? .16 * 2 * Math.atan2(sine, q.w) / sine : .32;
      return [...position.toArray(), q.x * factor, q.y * factor, q.z * factor];
    }
    return [...position.toArray(), ...(vertical ? axis.toArray().map(v => v * .16) : [])];
  };
  const desired = [...goal.toArray(), ...(align ? [0, 0, 0] : vertical ? [0, 0, .16] : [])];
  const seeds = [original.joints, [0, 0, 0, 0, 0, 0], [...UPRIGHT_REFERENCE_JOINTS_RAD],
    [3, -1.7, 2.1, -2, -.78, 0], [.1, -2, -2.1, -.6, 2.35, 0],
    [0, -.7, -1.2, -1.4, .8, 0], [Math.atan2(goal.y, goal.x), -2, 1.2, -.8, -.6, 0]];
  let best = null;
  try {
    for (const seed of seeds) {
      let joints = [...seed], seedBest = null;
      for (let iteration = 0; iteration < 180; iteration++) {
        const actual = measure(joints), error = desired.map((v, i) => v - actual[i]);
        const positionError = Math.hypot(...error.slice(0, 3));
        const directionError = Math.hypot(...error.slice(3));
        const cost = Math.hypot(...error);
        const candidate = {joints: [...joints], gripper: original.gripper};
        if (!seedBest || cost < seedBest.cost) seedBest = {joints: [...joints], cost, positionError, directionError};
        if (positionError < .0003 && directionError < (align ? .0003 : .008)) {
          if (accept(candidate)) return candidate;
          // This branch converged into a forbidden volume; try another branch.
          break;
        }
        const columns = joints.map((q, index) => {
          const probe = [...joints], delta = q > Math.PI - .003 ? -.002 : .002;
          probe[index] += delta; const sampled = measure(probe);
          return sampled.map((v, row) => (v - actual[row]) / delta);
        });
        const normal = error.map((_, i) => error.map((_, j) => columns.reduce((sum, column) => sum + column[i] * column[j], 0) + (i === j ? .0004 : 0)));
        const solved = linearSolve(normal, error); if (!solved) break;
        const delta = columns.map(column => column.reduce((sum, v, i) => sum + v * solved[i], 0));
        const scale = Math.min(1, .18 / Math.max(...delta.map(Math.abs), .18));
        joints = joints.map((q, i) => Math.max(-Math.PI, Math.min(Math.PI, q + delta[i] * scale)));
      }
      if (seedBest?.positionError < .006 && seedBest.directionError < (align ? .002 : .025)
        && (!best || seedBest.cost < best.cost) && accept({joints: seedBest.joints, gripper: original.gripper})) best = seedBest;
    }
    if (best?.positionError < .006 && best.directionError < .025) return {joints: best.joints, gripper: original.gripper};
    throw Error('这个位置暂时够不到或处于碰撞死区，请把物块拖近一些或换个位置');
  } finally { rig.apply(original); }
}
