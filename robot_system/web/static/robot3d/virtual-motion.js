import { Vector3 } from './vendor/three.module.js';
import { solveVirtualIK, virtualToolMatrix } from './virtual-tool.js?v=20261007-solid4';

export function transferWaypoints(current, x, y, z) {
  const height = Math.max(current[2], z + .25, .30), side = current[1] >= 0 ? .30 : -.30;
  // Go around the pedestal on the current side. Crossing to the opposite side
  // first can require a ±pi joint wrap or sweep the wrist into the base.
  return [[current[0], current[1], height], [current[0], side, height],
    [x, side, height], [x, y, height], [x, y, z]];
}

// Preflight the exact interpolated joint segments, with a carried cube included.
export async function planVirtualMotion(rig, base, solids, initial, points, {
  firstJointMove = false, displayPose = p => p, heldAt = () => null, yieldControl = null,
} = {}) {
  let current = structuredClone(initial), work = 0; const segments = [];
  try {
    for (let index = 0; index < points.length; index++) {
      rig.apply(current); const start = new Vector3().setFromMatrixPosition(virtualToolMatrix(rig, base)), target = new Vector3(...points[index]);
      const count = index === 0 && firstJointMove ? 1 : Math.max(1, Math.ceil(start.distanceTo(target) / .004));
      const duration = points.length > 1 ? (index ? 1000 : 1400) : 900;
      for (let step = 1; step <= count; step++) {
        const previous = current, goal = start.clone().lerp(target, step / count).toArray();
        current = solveVirtualIK(rig, base, goal, current, {align: true, accept: candidate => {
          const display = displayPose(candidate);
          if (solids.check(display, heldAt(candidate), {wrist: true})) return false;
          return !solids.sweep(displayPose(previous), display, heldAt).blocked;
        }});
        segments.push({joints: Object.fromEntries(current.joints.map((q, i) => [i, q])), gripper: initial.gripper, durationMs: duration / count});
        if (yieldControl && ++work % 8 === 0) {
          rig.apply(displayPose(initial)); await yieldControl();
        }
      }
    }
    return segments;
  } finally { rig.apply(displayPose(initial)); }
}
