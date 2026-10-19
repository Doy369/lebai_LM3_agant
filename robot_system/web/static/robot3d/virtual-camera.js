import { PerspectiveCamera, Vector3, Group, Mesh, BoxGeometry, CylinderGeometry, MeshStandardMaterial } from './vendor/three.module.js';
import { scene_T_base } from './coordinates.js';
import { radToDeg } from './validation.js';
import { projectionMatrixFromIntrinsics, scaledIntrinsics, scene_T_calibratedCamera } from './camera-calibration.js';
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
    if (config.mode === 'calibrated') {
      const cameraMatrix = scene_T_calibratedCamera(config.baseTCamera);
      cameraMatrix.decompose(this.camera.position, this.camera.quaternion, this.camera.scale);
      const intrinsics = scaledIntrinsics(config.cameraMatrix, config.imageSizeWh, config.renderSizeWh);
      this.camera.near = config.nearM; this.camera.far = config.farM;
      this.camera.projectionMatrix.copy(projectionMatrixFromIntrinsics(intrinsics, config.nearM, config.farM));
      this.camera.projectionMatrixInverse.copy(this.camera.projectionMatrix).invert();
      this.helper.name = 'calibrated_camera_marker_pending_physical_validation';
    } else {
      this.camera.position.copy(new Vector3(...config.positionBaseM).applyMatrix4(scene_T_base));
      this.camera.up.set(0, 1, 0);
      this.camera.lookAt(new Vector3(...config.targetBaseM).applyMatrix4(scene_T_base));
      this.camera.fov = radToDeg(config.fovRad); this.camera.updateProjectionMatrix();
      this.helper.name = 'schematic_camera_marker';
    }
    this.camera.updateMatrixWorld(true);
    this.helper.position.copy(this.camera.position); this.helper.quaternion.copy(this.camera.quaternion);
  }
  dispose() {
    this.helper.removeFromParent();
    this.helper.traverse(n => { n.geometry?.dispose(); n.material?.dispose(); });
  }
}
