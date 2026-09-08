import { Quaternion, Vector3, AnimationClip, AnimationMixer, LoopOnce } from './vendor/three.module.js';
// Axes from dtsci reference; positive signs and zero offsets remain UNCALIBRATED.
export const JOINT_MAPPING = ['y', 'z', 'z', 'z', 'y', 'z'].map((axis, i) => ({
  name: `Joint${i + 1}`, axis, direction: 1, zeroOffsetRad: 0, calibration: 'pending',
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
