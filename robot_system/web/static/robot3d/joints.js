import { Quaternion, Vector3, AnimationClip, AnimationMixer, LoopOnce } from './vendor/three.module.js';

// Captured from robot 172.16.2.162 on 2026-09-13 while the operator confirmed
// that the physical arm was in the upright pose represented by the GLB import.
// This is a one-pose zero alignment. Joint signs still require one-at-a-time
// motion checks before the mapping can be called fully calibrated.
export const UPRIGHT_REFERENCE_JOINTS_RAD = Object.freeze([
  0.004793689962142629,
  -1.4965900061809287,
  0.3151371781112564,
  -1.576452880950225,
  0.39308257689569553,
  -0.006231796950785417,
]);

// Model-space joint deltas at the captured reference pose. Joint3 comes from
// the operator-provided side view (upper link about 22 degrees left of vertical).
// Joint5 and Joint6 keep the SDK angles instead of being cancelled by the
// single-pose reference; this agrees with the independent wrist axes in Lebai's
// official lm3_macro.xacro and the operator-provided wrist close-up.
export const PHOTO_REFERENCE_MODEL_DELTA_RAD = Object.freeze([
  0,
  0,
  22 * Math.PI / 180,
  0,
  UPRIGHT_REFERENCE_JOINTS_RAD[4],
  UPRIGHT_REFERENCE_JOINTS_RAD[5],
]);

// Axes come from the dtsci reference implementation. Joint1 and Joint2 were
// verified positive by isolated operator motion on 2026-09-13. Joint5 was
// verified opposite to the imported model axis, so its direction is -1.
// Offsets keep the confirmed reference snapshot unchanged when a sign changes.
export const JOINT_DIRECTIONS = Object.freeze([1, 1, 1, 1, -1, 1]);
export const JOINT_MAPPING = ['y', 'z', 'z', 'z', 'y', 'z'].map((axis, i) => ({
  name: `Joint${i + 1}`,
  axis,
  direction: JOINT_DIRECTIONS[i],
  zeroOffsetRad: PHOTO_REFERENCE_MODEL_DELTA_RAD[i] - JOINT_DIRECTIONS[i] * UPRIGHT_REFERENCE_JOINTS_RAD[i],
  calibration: 'operator_motion_joint1_joint2_joint5_directions_verified_others_pending',
}));
export const GRIPPER_TRACK_NODES = ['polySurface36', 'polySurface42', 'polySurface38', 'polySurface40', 'polySurface49', 'polySurface50'];
export class JointRig {
  constructor(gltf) {
    this.root = gltf.scene;
    this.joints = JOINT_MAPPING.map(mapping => {
      const found = []; this.root.traverse(n => { if (n.name === mapping.name) found.push(n); });
      if (found.length !== 1) throw Error(`模型需要唯一节点 ${mapping.name}`);
      const node = found[0];
      return { ...mapping, node, initialQuaternion: node.quaternion.clone(), axisVector: new Vector3(...({x:[1,0,0],y:[0,1,0],z:[0,0,1]}[mapping.axis])) };
    });
    this.joints.slice(1).forEach((joint, i) => {
      let n = joint.node.parent;
      while (n && n !== this.joints[i].node) n = n.parent;
      if (!n) throw Error(`关节链层级不兼容：${joint.name}`);
    });
    const raw = gltf.animations.find(c => c.name === 'Take 001');
    const tracks = raw?.tracks.filter(t => GRIPPER_TRACK_NODES.some(name => t.name === `${name}.quaternion`)) ?? [];
    // Strict allowlist: no arm transform may be written by the gripper mixer.
    if (tracks.length === 6 && tracks.every(t => t.times.length > 20)) {
      this.clip = new AnimationClip('gripper-preview', raw.duration, tracks.map(t => t.clone()));
      this.startTimeSec = tracks[0].times[0]; this.endTimeSec = tracks[0].times[20];
      this.mixer = new AnimationMixer(this.root);
      this.action = this.mixer.clipAction(this.clip);
      this.action.setLoop(LoopOnce, 1); this.action.clampWhenFinished = true;
      this.action.play(); this.action.paused = true;
    }
  }
  apply({joints, gripper}) {
    if (this.action) {
      this.action.time = this.startTimeSec + gripper * (this.endTimeSec - this.startTimeSec);
      this.mixer.update(0);
    }
    this.joints.forEach((j, i) => {
      // Imported local transform followed by rotation about its local joint axis.
      const delta = new Quaternion().setFromAxisAngle(j.axisVector, j.direction * joints[i] + j.zeroOffsetRad);
      j.node.quaternion.copy(j.initialQuaternion).multiply(delta);
    });
    this.root.updateMatrixWorld(true);
  }
  dispose() { this.mixer?.stopAllAction(); this.mixer?.uncacheRoot(this.root); }
}
