import test from 'node:test';
import assert from 'node:assert/strict';
import { readFile } from 'node:fs/promises';
import { Group, Matrix4, Vector3, Raycaster } from '../../robot_system/web/static/robot3d/vendor/three.module.js';
import { GLTFLoader } from '../../robot_system/web/static/robot3d/vendor/GLTFLoader.js';
import { JointRig, UPRIGHT_REFERENCE_JOINTS_RAD } from '../../robot_system/web/static/robot3d/joints.js';
import { disposeGraph } from '../../robot_system/web/static/robot3d/model-loader.js';
import { scene_T_base, base_T_model } from '../../robot_system/web/static/robot3d/coordinates.js';
import { GripperCamera, DEFAULT_GRIPPER_CAMERA, gripperCameraLocalMatrix, validateGripperCamera } from '../../robot_system/web/static/robot3d/gripper-camera.js';
import { DEFAULT_CONFIG, validateConfig } from '../../robot_system/web/static/robot3d/config.js';
import { LivePhysics } from '../../robot_system/web/static/robot3d/live-physics.js';
import { PhysicsView } from '../../robot_system/web/static/robot3d/physics-view.js';
import { VirtualGrasp } from '../../robot_system/web/static/robot3d/virtual-grasp.js';
import { RobotSolids } from '../../robot_system/web/static/robot3d/solid-collisions.js';
import { planVirtualMotion, transferWaypoints } from '../../robot_system/web/static/robot3d/virtual-motion.js';
import { virtualToolMatrix } from '../../robot_system/web/static/robot3d/virtual-tool.js';
import { freeTableObject } from '../../robot_system/web/static/robot3d/placement.js';

function sameMatrix(a, b) { a.elements.forEach((v, i) => assert.ok(Math.abs(v - b.elements[i]) < 1e-8, `matrix element ${i}`)); }

test('mounting follows wrist translation and rotation with a constant local transform; rebind and hide preserve optics', () => {
  const scene = new Group(), parent = new Group(), wrist = new Group(); scene.add(parent); parent.add(wrist);
  parent.position.set(1, .2, -.4); parent.rotation.y = .4;
  const camera = new GripperCamera(scene), rig = {joints: Array.from({length: 6}, () => ({node: wrist}))};
  try {
    camera.bind(rig); const initial = camera.camera.matrixWorld.clone();
    wrist.position.set(.1, .4, -.2); wrist.rotation.set(.3, -.5, .8); camera.update();
    assert.ok(initial.elements.some((v, i) => Math.abs(v - camera.camera.matrixWorld.elements[i]) > .01));
    sameMatrix(camera.camera.matrixWorld, wrist.matrixWorld.clone().multiply(gripperCameraLocalMatrix()));
    const pose = camera.camera.matrixWorld.clone(); camera.configure({...camera.config, fovRad: Math.PI / 2, showMount: false});
    assert.equal(camera.root.visible, false); assert.equal(camera.camera.fov, 90); sameMatrix(pose, camera.camera.matrixWorld);
    camera.unbind(); assert.equal(camera.update(), false); assert.equal(camera.root.parent, scene);
    camera.bind(rig); sameMatrix(camera.camera.matrixWorld, wrist.matrixWorld.clone().multiply(gripperCameraLocalMatrix(camera.config)));
  } finally { camera.dispose(); assert.equal(camera.root.parent, null); }
});

test('camera settings roundtrip with old scenes still valid; malformed geometry and view modes reject', () => {
  const c = validateConfig({...DEFAULT_CONFIG, cameraView: 'gripper', gripperCamera: DEFAULT_GRIPPER_CAMERA});
  assert.deepEqual(validateConfig(JSON.parse(JSON.stringify(c))), c);
  assert.deepEqual(validateConfig(DEFAULT_CONFIG), DEFAULT_CONFIG);
  for (const change of [v => v.fovRad = 0, v => v.offsetToolM[0] = Infinity, v => v.targetToolM = [...v.offsetToolM], v => v.showMount = 'true']) {
    const invalid = structuredClone(DEFAULT_GRIPPER_CAMERA); change(invalid); assert.throws(() => validateGripperCamera(invalid));
  }
  assert.throws(() => validateConfig({...c, cameraView: 'unknown'}), /摄像头视角/);
});

test('actual mounted camera sees the grasp target without the robot hiding it; camera housing participates in solid geometry', async () => {
  const bytes = await readFile(new URL('../../robot_system/web/static/robot3d/assets/Lebai_LM3.glb', import.meta.url));
  const gltf = await new GLTFLoader().parseAsync(bytes.buffer.slice(bytes.byteOffset, bytes.byteOffset + bytes.byteLength), '');
  const rig = new JointRig(gltf), scene = new Group(), base = new Group(), model = new Group();
  base.applyMatrix4(scene_T_base); model.applyMatrix4(base_T_model); scene.add(base); base.add(model); model.add(gltf.scene);
  const world = new LivePhysics();
  world.setEnvironment({groundZ: -.075, table: {position: [0, 0, -.035], size: [1.2, 1, .07]},
    obstacles: [{position: [.3, 0, .04], size: [.08, .08, .08], label: '橙色固定物块'}], initialPositions: [[.264, -.16, .03], [.36, .16, .03]]}); world.reset();
  let p = {joints: [...UPRIGHT_REFERENCE_JOINTS_RAD], gripper: 1}; rig.apply(p);
  const camera = new GripperCamera(scene); camera.bind(rig);
  const solids = new RobotSolids(rig, base, world), grasp = new VirtualGrasp(world), view = new PhysicsView(base);
  try {
    assert.equal(solids.check(p), null); assert.ok(solids.parts.some(part => part.mesh.parent === camera.helper));
    world.robotBoxes = solids.boxes();
    const added = world.addBody(freeTableObject(world, {label: '绿色物块', color: '#35bca9', mass: 1, sizeM: [.04, .04, .04]}));
    for (const id of ['A', 'B', added.id]) {
      world.reset(); grasp.reset(); p = {joints: [...UPRIGHT_REFERENCE_JOINTS_RAD], gripper: 1}; rig.apply(p); grasp.update(rig, base, p, 0);
      const body = world.bodies.find(b => b.id === id), start = [...body.position];
      async function travel(points, firstJointMove = false) {
        const segments = await planVirtualMotion(rig, base, solids, p, points, {firstJointMove,
          displayPose: q => grasp.displayPose(q), heldAt: q => grasp.heldAt(rig, base, q)});
        for (const segment of segments) {
          p = {joints: p.joints.map((q, i) => segment.joints[i] ?? q), gripper: segment.gripper};
          rig.apply(p); rig.apply(grasp.update(rig, base, p, 1000));
        }
        camera.update(); sameMatrix(camera.camera.matrixWorld, rig.joints[5].node.matrixWorld.clone().multiply(gripperCameraLocalMatrix()));
      }
      await travel([[start[0], start[1], start[2] + .16], [start[0], start[1], start[2] + .006]], true);
      const center = base.localToWorld(new Vector3(...body.position)), projected = center.clone().project(camera.camera);
      assert.ok(Math.abs(projected.x) < 1 && Math.abs(projected.y) < 1 && projected.z > -1 && projected.z < 1, `${id} must be in the camera image`);
      view.update(world.snapshot(), 'live'); base.updateMatrixWorld(true);
      const meshes = [];
      rig.root.traverse(node => {
        if (!node.isMesh) return;
        let ancestor = node; while (ancestor && ancestor !== camera.root) ancestor = ancestor.parent;
        if (!ancestor) meshes.push(node);
      });
      const ray = new Raycaster(camera.camera.position, center.clone().sub(camera.camera.position).normalize(), .003, 30);
      const hit = ray.intersectObjects([...meshes, ...view.blocks.map(b => b.mesh)], false)[0];
      assert.equal(hit?.object.name, `physics-block-${id}`, 'the target is visible through the real camera viewpoint');
      grasp.arm(id); p = {...p, gripper: 0}; rig.apply(p); rig.apply(grasp.update(rig, base, p, 1020)); assert.equal(world.heldId, id);
      await travel([[start[0], start[1], start[2] + .18]]);
      const current = new Vector3().setFromMatrixPosition(virtualToolMatrix(rig, base)).toArray();
      await travel(transferWaypoints(current, -.276, -.16, .07));
      p = {...p, gripper: 1}; rig.apply(p); grasp.update(rig, base, p, 2020); assert.equal(world.heldId, null);
    }
  } finally { view.dispose(); camera.dispose(); rig.dispose(); disposeGraph(gltf.scene); }
});
