import * as T from './vendor/three.module.js';
import { OrbitControls } from './vendor/OrbitControls.js';
import { scene_T_base, base_T_model } from './coordinates.js';
import { VirtualCamera } from './virtual-camera.js';
import { GripperCamera, DEFAULT_GRIPPER_CAMERA } from './gripper-camera.js';
import { disposeGraph } from './model-loader.js';
import { CollisionOverlay } from './collision-overlay.js';
export class RobotScene {
  constructor(mainHost, cameraHost, onFrame) {
    this.mainHost = mainHost; this.cameraHost = cameraHost;
    this.scene = new T.Scene(); this.scene.background = new T.Color('#eaf0f3');
    // Keep the last frame available when the embedded browser captures the canvas.
    this.renderer = new T.WebGLRenderer({antialias: true, preserveDrawingBuffer: true});
    this.renderer.setPixelRatio(Math.min(devicePixelRatio, 2)); mainHost.append(this.renderer.domElement);
    // One WebGL context, a separate virtual-camera render target copied into its canvas.
    this.cameraCanvas = document.createElement('canvas'); this.cameraCanvas.width = 640; this.cameraCanvas.height = 360;
    this.cameraContext = this.cameraCanvas.getContext('2d'); cameraHost.append(this.cameraCanvas);
    this.cameraTarget = new T.WebGLRenderTarget(640, 360);
    this.cameraTarget.texture.colorSpace = T.SRGBColorSpace;
    this.pixels = new Uint8Array(640 * 360 * 4); this.cameraImage = this.cameraContext.createImageData(640, 360);
    this.camera = new T.PerspectiveCamera(45, 1, .01, 50);
    this.controls = new OrbitControls(this.camera, this.renderer.domElement); this.controls.enableDamping = true;
    this.base = new T.Group(); this.base.name = 'scene_T_base'; this.base.applyMatrix4(scene_T_base); this.scene.add(this.base);
    this.model = new T.Group(); this.model.name = 'base_T_model_UNCALIBRATED'; this.model.applyMatrix4(base_T_model); this.base.add(this.model);
    this.scene.add(new T.HemisphereLight(0xffffff, 0x677981, 2.4));
    const light = new T.DirectionalLight(0xffffff, 3); light.position.set(2, 3, 2); this.scene.add(light);
    this.base.add(new T.AxesHelper(.25));
    this.groundZ = -.075;
    const ground = new T.Mesh(new T.PlaneGeometry(20, 20), new T.MeshStandardMaterial({color: 0xe2e8eb, roughness: 1}));
    ground.position.z = this.groundZ; this.base.add(ground);
    const grid = new T.GridHelper(10, 100, 0x879ca5, 0xcad4da); grid.position.y = this.groundZ + .001; this.scene.add(grid);
    const makeBox = color => { const mesh = new T.Mesh(new T.BoxGeometry(1, 1, 1), new T.MeshStandardMaterial({color, roughness: .7})); this.base.add(mesh); return mesh; };
    this.table = makeBox(0xb4bdc4); this.target = makeBox(0xe9953e);
    this.collision = new CollisionOverlay(this.base);
    this.virtual = new VirtualCamera(this.scene); this.gripperVirtual = new GripperCamera(this.scene); this.cameraMode = 'gripper'; this.resetView();
    this.observer = new ResizeObserver(() => this.resize()); this.observer.observe(mainHost); this.resize();
    this.onFrame = onFrame; this.lastCameraMs = -Infinity;
    this.animate = ms => {
      if (this.closed) return;
      this.onFrame(ms); this.gripperVirtual.update(); this.controls.update();
      if (ms - this.lastCameraMs >= 100) {
        this.lastCameraMs = ms;
        const mounted = this.cameraMode === 'gripper' && this.gripperVirtual.rig && this.model.visible;
        const mountVisible = this.gripperVirtual.root.visible, fixedVisible = this.virtual.helper.visible;
        const overlays = [];
        this.scene.traverse(node => {
          if (node.type === 'AxesHelper' || ['virtual-grasp-point', 'placement-target', 'local-solid-volumes'].includes(node.name)) overlays.push([node, node.visible]);
        });
        for (const [node] of overlays) node.visible = false;
        this.virtual.helper.visible = false; if (mounted) this.gripperVirtual.root.visible = false;
        try {
          this.renderer.setRenderTarget(this.cameraTarget); this.renderer.render(this.scene, mounted ? this.gripperVirtual.camera : this.virtual.camera);
          this.renderer.readRenderTargetPixels(this.cameraTarget, 0, 0, 640, 360, this.pixels);
          for (let y = 0; y < 360; y++) this.cameraImage.data.set(this.pixels.subarray(y * 2560, (y + 1) * 2560), (359 - y) * 2560);
          this.cameraContext.putImageData(this.cameraImage, 0, 0);
        } finally {
          this.renderer.setRenderTarget(null); this.virtual.helper.visible = fixedVisible; this.gripperVirtual.root.visible = mountVisible;
          for (const [node, visible] of overlays) node.visible = visible;
        }
      }
      this.renderer.render(this.scene, this.camera); this.frame = requestAnimationFrame(this.animate);
    };
    this.frame = requestAnimationFrame(this.animate);
  }
  configure(config) {
    for (const key of ['table', 'target']) { this[key].position.fromArray(config[key].positionBaseM); this[key].scale.fromArray(config[key].sizeM); }
    this.virtual.configure(config.camera);
    this.gripperVirtual.configure(config.gripperCamera ?? DEFAULT_GRIPPER_CAMERA); this.cameraMode = config.cameraView ?? 'fixed'; this.lastCameraMs = -Infinity;
    this.onPhysicsEnvironmentChange?.();
  }
  setCollisionVisualization(data) { this.collision.setData(data); }
  setCollisionVisible(visible) { this.collision.setEnabled(visible); }
  updateCollisionVisualization(rig, sampleIndex = null, pinClosest = false) {
    this.collision.update(rig, sampleIndex, pinClosest);
  }
  resetView() { this.camera.position.set(1.25, .9, 1.35); this.controls.target.set(0, .4, 0); this.controls.update(); }
  focusPhysics(root) {
    const center = this.base.localToWorld(new T.Vector3(this.table.position.x, this.table.position.y,
      this.table.position.z + this.table.scale.z / 2 + .3));
    this.camera.up.set(0, 1, 0); this.controls.target.copy(center);
    this.camera.position.copy(center).add(new T.Vector3(1.15, .8, 1.2)); this.controls.update();
  }
    focusObjects(root) {
      const box = new T.Box3().setFromObject(root);
      if (box.isEmpty()) return this.focusPhysics(root);
      const center = box.getCenter(new T.Vector3());
      const distance = Math.max(.65, box.getSize(new T.Vector3()).length() * 1.8);
      this.camera.up.set(0, 1, 0); this.controls.target.copy(center);
      this.camera.position.copy(center).add(new T.Vector3(-distance, distance * .8, distance)); this.controls.update();
    }
  sideView() { this.camera.position.set(0, .45, 1.65); this.camera.up.set(0, 1, 0); this.controls.target.set(0, .45, 0); this.controls.update(); }
  topView() { this.camera.position.set(0, 1.4, -.15); this.camera.up.set(0, 0, 1); this.controls.target.set(0, .4, -.15); this.controls.update(); }
  frameModel() {
    const box = new T.Box3().setFromObject(this.model);
    if (!box.isEmpty()) { const center = box.getCenter(new T.Vector3()); this.controls.target.copy(center); this.camera.position.copy(center).add(new T.Vector3(1.25, .7, 1.35)); this.controls.update(); }
  }
  resize() {
    const w = Math.max(1, this.mainHost.clientWidth), h = Math.max(1, this.mainHost.clientHeight);
    this.renderer.setSize(w, h, false); this.camera.aspect = w / h; this.camera.updateProjectionMatrix();
  }
  dispose() {
    this.closed = true; cancelAnimationFrame(this.frame); this.observer.disconnect(); this.controls.dispose(); this.virtual.dispose(); this.gripperVirtual.dispose();
    this.cameraTarget.dispose(); this.collision.dispose(); disposeGraph(this.scene); this.renderer.dispose(); this.renderer.forceContextLoss();
    this.renderer.domElement.remove(); this.cameraCanvas.remove();
  }
}
