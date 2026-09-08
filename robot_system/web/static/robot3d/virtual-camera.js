import { PerspectiveCamera, Vector3, Group, Mesh, BoxGeometry, CylinderGeometry, MeshStandardMaterial } from './vendor/three.module.js';
import { scene_T_base } from './coordinates.js';
import { radToDeg } from './validation.js';
export class VirtualCamera {
  constructor(scene) {
    this.camera = new PerspectiveCamera(50, 16 / 9, .01, 30);
    this.helper = new Group(); this.helper.name = 'schematic_camera_marker';
    const body = new Mesh(new BoxGeometry(.06, .04, .035), new MeshStandardMaterial({color: 0x148f9e}));
    const lens = new Mesh(new CylinderGeometry(.013, .013, .025, 12), new MeshStandardMaterial({color: 0x183440}));
    lens.rotation.x = Math.PI / 2; lens.position.z = -.025;
    this.helper.add(body, lens); scene.add(this.helper);
  }
  configure(config) {
    this.camera.position.copy(new Vector3(...config.positionBaseM).applyMatrix4(scene_T_base));
    this.camera.up.set(0, 1, 0);
    this.camera.lookAt(new Vector3(...config.targetBaseM).applyMatrix4(scene_T_base));
    this.camera.fov = radToDeg(config.fovRad); this.camera.updateProjectionMatrix(); this.camera.updateMatrixWorld();
    this.helper.position.copy(this.camera.position); this.helper.quaternion.copy(this.camera.quaternion);
  }
  dispose() {
    this.helper.removeFromParent();
    this.helper.traverse(n => { n.geometry?.dispose(); n.material?.dispose(); });
  }
}
