import { PerspectiveCamera, Group, Mesh, BoxGeometry, CylinderGeometry, MeshStandardMaterial, Matrix4, Quaternion, Vector3 } from './vendor/three.module.js';
import { VIRTUAL_TOOL_OFFSET } from './virtual-tool.js';
import { number, vector, radToDeg } from './validation.js';
import { disposeGraph } from './model-loader.js';

export const DEFAULT_GRIPPER_CAMERA = Object.freeze({
  offsetToolM: Object.freeze([.06, 0, .05]), targetToolM: Object.freeze([0, 0, -.025]),
  fovRad: 60 * Math.PI / 180, showMount: true,
});

export function validateGripperCamera(value = DEFAULT_GRIPPER_CAMERA) {
  const offsetToolM = vector(value.offsetToolM, 3, '夹爪相机安装位置(m)', -.25, .25);
  const targetToolM = vector(value.targetToolM, 3, '夹爪相机朝向点(m)', -.5, .5);
  const direction = new Vector3(...targetToolM).sub(new Vector3(...offsetToolM));
  if (direction.length() < .001) throw Error('夹爪相机位置不能与朝向点重合');
  if (Math.abs(direction.normalize().y) > .9999) throw Error('夹爪相机朝向不能与图像上方向平行');
  if (typeof value.showMount !== 'boolean') throw Error('夹爪相机显示选项无效');
  return {offsetToolM, targetToolM, fovRad: number(value.fovRad, '夹爪相机视场角(rad)', Math.PI / 6, Math.PI * 110 / 180), showMount: value.showMount};
}

// A fixed mounting transform: it follows wrist rotation, never tracks a world-space object.
export function gripperCameraLocalMatrix(value = DEFAULT_GRIPPER_CAMERA) {
  const config = validateGripperCamera(value), eye = new Vector3(...config.offsetToolM), target = new Vector3(...config.targetToolM);
  const rotation = new Quaternion().setFromRotationMatrix(new Matrix4().lookAt(eye, target, new Vector3(0, 1, 0)));
  return new Matrix4().makeTranslation(...VIRTUAL_TOOL_OFFSET).multiply(new Matrix4().compose(eye, rotation, new Vector3(1, 1, 1)));
}

export class GripperCamera {
  constructor(scene) {
    this.scene = scene; this.rig = null; this.camera = new PerspectiveCamera(60, 16 / 9, .003, 30);
    this.root = new Group(); this.root.name = 'gripper-camera-mount'; this.root.position.fromArray(VIRTUAL_TOOL_OFFSET); this.root.visible = false;
    this.helper = new Group(); this.helper.name = 'gripper-camera-housing';
    const body = new Mesh(new BoxGeometry(.03, .022, .016), new MeshStandardMaterial({color: 0x148f9e, roughness: .55}));
    const lens = new Mesh(new CylinderGeometry(.006, .006, .014, 16), new MeshStandardMaterial({color: 0x102733, roughness: .25}));
    lens.rotation.x = Math.PI / 2; lens.position.z = -.014; this.helper.add(body, lens);
    this.bracket = new Mesh(new CylinderGeometry(.0025, .0025, 1, 8), new MeshStandardMaterial({color: 0x50626d}));
    this.root.add(this.helper, this.bracket); scene.add(this.root); this.configure(DEFAULT_GRIPPER_CAMERA);
  }
  configure(value) {
    const config = validateGripperCamera(value), eye = new Vector3(...config.offsetToolM), target = new Vector3(...config.targetToolM);
    this.config = config; this.helper.position.copy(eye);
    this.helper.quaternion.setFromRotationMatrix(new Matrix4().lookAt(eye, target, new Vector3(0, 1, 0)));
    const anchor = new Vector3(0, 0, .11), direction = eye.clone().sub(anchor), length = direction.length();
    this.bracket.position.copy(anchor).add(eye).multiplyScalar(.5);
    this.bracket.quaternion.setFromUnitVectors(new Vector3(0, 1, 0), length > 1e-9 ? direction.normalize() : new Vector3(0, 1, 0));
    this.bracket.scale.y = Math.max(.001, length);
    this.camera.fov = radToDeg(config.fovRad); this.camera.updateProjectionMatrix();
    this.root.visible = Boolean(this.rig && config.showMount); this.update();
  }
  bind(rig) {
    this.unbind(); if (!rig) return;
    this.rig = rig; rig.joints[5].node.add(this.root); this.root.position.fromArray(VIRTUAL_TOOL_OFFSET);
    this.root.visible = this.config.showMount; this.update();
  }
  unbind() {
    this.root.removeFromParent(); this.scene.add(this.root); this.root.visible = false; this.rig = null;
  }
  update() {
    if (!this.rig) return false;
    this.helper.updateWorldMatrix(true, false);
    this.helper.matrixWorld.decompose(this.camera.position, this.camera.quaternion, this.camera.scale);
    this.camera.updateMatrixWorld(true); return true;
  }
  dispose() { this.rig = null; this.root.removeFromParent(); disposeGraph(this.root); }
}
