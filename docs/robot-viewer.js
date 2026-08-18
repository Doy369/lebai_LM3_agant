import * as THREE from "three";
import { OrbitControls } from "./vendor/three/addons/controls/OrbitControls.js";
import { GLTFLoader } from "./vendor/three/addons/loaders/GLTFLoader.js";

const canvas = document.getElementById("robot3dCanvas");

if (canvas) {
  startRobotViewer(canvas).catch((error) => {
    console.error("虚拟机械臂初始化失败", error);
    setViewerStatus("error", "模型加载失败");
    const message = document.getElementById("robotModelMessage");
    if (message) message.textContent = String(error?.message || error);
  });
}

async function startRobotViewer(targetCanvas) {
  const scene = new THREE.Scene();
  scene.background = new THREE.Color(0x07131a);

  const camera = new THREE.PerspectiveCamera(38, 1, 0.01, 100);
  camera.position.set(1.15, 0.85, 1.25);

  const renderer = new THREE.WebGLRenderer({
    canvas: targetCanvas,
    antialias: true,
    alpha: false,
  });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
  renderer.outputColorSpace = THREE.SRGBColorSpace;
  renderer.toneMapping = THREE.ACESFilmicToneMapping;
  renderer.toneMappingExposure = 1.05;
  renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;

  const controls = new OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true;
  controls.dampingFactor = 0.07;
  controls.minDistance = 0.35;
  controls.maxDistance = 5;
  controls.autoRotateSpeed = 1.25;

  scene.add(new THREE.HemisphereLight(0xc7efff, 0x18232a, 2.1));

  const keyLight = new THREE.DirectionalLight(0xffffff, 3.2);
  keyLight.position.set(2.5, 3.5, 2.2);
  keyLight.castShadow = true;
  scene.add(keyLight);

  const rimLight = new THREE.DirectionalLight(0x44d9ff, 1.4);
  rimLight.position.set(-2.2, 1.6, -1.8);
  scene.add(rimLight);

  const grid = new THREE.GridHelper(2.4, 24, 0x1a8fa8, 0x173946);
  grid.material.opacity = 0.42;
  grid.material.transparent = true;
  scene.add(grid);

  const ground = new THREE.Mesh(
    new THREE.CircleGeometry(1.2, 64),
    new THREE.MeshStandardMaterial({ color: 0x081a22, roughness: 0.92, metalness: 0.08 }),
  );
  ground.rotation.x = -Math.PI / 2;
  ground.position.y = -0.002;
  ground.receiveShadow = true;
  scene.add(ground);

  const loader = new GLTFLoader();
  const modelUrl = new URL("./models/Lebai_LM3.glb", import.meta.url).href;
  const gltf = await loader.loadAsync(modelUrl);
  const robotRoot = gltf.scene;
  scene.add(robotRoot);

  robotRoot.traverse((object) => {
    if (!object.isMesh) return;
    object.castShadow = true;
    object.receiveShadow = true;
  });

  const jointAxes = [
    new THREE.Vector3(0, 1, 0),
    new THREE.Vector3(1, 0, 0),
    new THREE.Vector3(1, 0, 0),
    new THREE.Vector3(0, 1, 0),
    new THREE.Vector3(1, 0, 0),
    new THREE.Vector3(0, 1, 0),
  ];
  const joints = jointAxes.map((axis, index) => {
    const node = robotRoot.getObjectByName(`Joint${index + 1}`);
    return node
      ? { node, axis, restQuaternion: node.quaternion.clone() }
      : null;
  });

  const missingJoints = joints
    .map((joint, index) => (joint ? null : `J${index + 1}`))
    .filter(Boolean);
  if (missingJoints.length) {
    throw new Error(`模型缺少关节节点：${missingJoints.join("、")}`);
  }

  const fitModel = () => {
    robotRoot.updateMatrixWorld(true);
    const box = new THREE.Box3().setFromObject(robotRoot);
    const size = box.getSize(new THREE.Vector3());
    const center = box.getCenter(new THREE.Vector3());
    const maxSize = Math.max(size.x, size.y, size.z, 0.01);
    const distance = maxSize / (2 * Math.tan(THREE.MathUtils.degToRad(camera.fov / 2)));
    controls.target.copy(center);
    camera.position.copy(center).add(new THREE.Vector3(distance * 0.85, distance * 0.55, distance * 0.95));
    camera.near = Math.max(distance / 100, 0.005);
    camera.far = Math.max(distance * 20, 20);
    camera.updateProjectionMatrix();
    controls.update();
  };

  const jointInputs = Array.from(document.querySelectorAll("[data-virtual-joint]"));
  const followToggle = document.getElementById("followRobotJointsToggle");
  let currentPose = [0, 0, 0, 0, 0, 0];

  const applyJointPose = (pose, source = "virtual") => {
    if (!Array.isArray(pose) || pose.length < 6) return;
    currentPose = pose.slice(0, 6).map((value) => Number(value) || 0);
    joints.forEach((joint, index) => {
      const delta = new THREE.Quaternion().setFromAxisAngle(joint.axis, currentPose[index]);
      joint.node.quaternion.copy(joint.restQuaternion).multiply(delta);
    });
    jointInputs.forEach((input, index) => {
      input.value = String(currentPose[index]);
      const value = document.querySelector(`[data-virtual-joint-value="${index}"]`);
      if (value) value.textContent = `${currentPose[index].toFixed(2)} rad`;
    });
    const syncPill = document.getElementById("robotTwinSyncPill");
    if (syncPill) {
      syncPill.className = `status-pill ${source === "robot" ? "success" : "neutral"}`;
      syncPill.textContent = source === "robot" ? "已同步机器人关节" : "虚拟姿态预览";
    }
  };

  jointInputs.forEach((input) => {
    input.addEventListener("input", () => {
      if (followToggle) followToggle.checked = false;
      const nextPose = jointInputs.map((item) => Number(item.value));
      applyJointPose(nextPose, "virtual");
    });
  });

  document.getElementById("fitRobotViewButton")?.addEventListener("click", fitModel);
  document.getElementById("zeroRobotPoseButton")?.addEventListener("click", () => {
    if (followToggle) followToggle.checked = false;
    applyJointPose([0, 0, 0, 0, 0, 0], "virtual");
  });
  document.getElementById("robotAutoRotateToggle")?.addEventListener("change", (event) => {
    controls.autoRotate = Boolean(event.target.checked);
  });

  window.addEventListener("lebai:robot-status", (event) => {
    if (!followToggle?.checked) return;
    const pose = event.detail?.kin_data?.actual_joint_pose;
    if (Array.isArray(pose) && pose.length >= 6) applyJointPose(pose, "robot");
  });
  followToggle?.addEventListener("change", () => {
    if (!followToggle.checked) return;
    const pose = window.__lebaiRobotStatus?.kin_data?.actual_joint_pose;
    if (Array.isArray(pose) && pose.length >= 6) applyJointPose(pose, "robot");
  });

  const initialRobotPose = window.__lebaiRobotStatus?.kin_data?.actual_joint_pose;
  const hasInitialRobotPose = Boolean(
    followToggle?.checked && Array.isArray(initialRobotPose) && initialRobotPose.length >= 6,
  );

  const resize = () => {
    const width = Math.max(targetCanvas.clientWidth, 1);
    const height = Math.max(targetCanvas.clientHeight, 1);
    if (targetCanvas.width === Math.round(width * renderer.getPixelRatio()) &&
        targetCanvas.height === Math.round(height * renderer.getPixelRatio())) return;
    renderer.setSize(width, height, false);
    camera.aspect = width / height;
    camera.updateProjectionMatrix();
  };
  const resizeObserver = new ResizeObserver(resize);
  resizeObserver.observe(targetCanvas);

  fitModel();
  applyJointPose(hasInitialRobotPose ? initialRobotPose : currentPose, hasInitialRobotPose ? "robot" : "virtual");
  setViewerStatus("success", "Lebai LM3 模型已加载");
  const message = document.getElementById("robotModelMessage");
  if (message) message.textContent = "拖动视角可旋转，滚轮可缩放；关节滑块仅改变虚拟模型。";

  const clock = new THREE.Clock();
  const render = () => {
    resize();
    controls.update(clock.getDelta());
    renderer.render(scene, camera);
    requestAnimationFrame(render);
  };
  render();
}

function setViewerStatus(kind, text) {
  const pill = document.getElementById("robotModelStatusPill");
  if (!pill) return;
  pill.className = `status-pill ${kind}`;
  pill.textContent = text;
}
