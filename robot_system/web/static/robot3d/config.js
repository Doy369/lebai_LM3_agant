import { vector, number, pose, zeroPose } from './validation.js';
import { validateCalibratedCameraConfig } from './camera-calibration.js';
import { validateObjects } from './objects.js';
import { validateGripperCamera } from './gripper-camera.js';
export const DEFAULT_CONFIG = {
  version: 1, units: {length: 'm', angle: 'rad', time: 'ms'},
  model: 'Lebai_LM3',
  table: {positionBaseM: [0, 0, -0.035], sizeM: [1.2, 1, 0.07]},
  target: {positionBaseM: [.3, 0, .04], sizeM: [.08, .08, .08]},
  camera: {mode: 'schematic', positionBaseM: [1.1, -1.1, .9], targetBaseM: [.1, 0, .4], fovRad: Math.PI * 50 / 180},
  preview: zeroPose(),
};
function box(value, name) {
  return {positionBaseM: vector(value?.positionBaseM, 3, `${name}位置(m)`, -5, 5), sizeM: vector(value?.sizeM, 3, `${name}尺寸(m)`, .001, 5)};
}
export function validateConfig(c) {
  if (c?.version !== 1) throw Error('不支持的场景版本，仅接受 version=1');
  if (c.units?.length !== 'm' || c.units?.angle !== 'rad' || c.units?.time !== 'ms') throw Error('场景单位必须为 m / rad / ms');
  if (c.model !== 'Lebai_LM3') throw Error('场景配置只支持内置 Lebai_LM3，外部 GLB 不持久化');
  if (c.cameraView !== undefined && !['fixed', 'gripper'].includes(c.cameraView)) throw Error('摄像头视角必须为 fixed 或 gripper');
  let camera;
  if (c.camera?.mode === 'calibrated') camera = validateCalibratedCameraConfig(c.camera);
  else if (c.camera?.mode === 'schematic') {
    const positionBaseM = vector(c.camera.positionBaseM, 3, '相机位置(m)', -5, 5);
    const targetBaseM = vector(c.camera.targetBaseM, 3, '相机目标(m)', -5, 5);
    if (Math.hypot(...positionBaseM.map((v, i) => v - targetBaseM[i])) < .001) throw Error('相机位置不能与观察目标重合');
    camera = {mode: 'schematic', positionBaseM, targetBaseM, fovRad: number(c.camera.fovRad, '相机视场角(rad)', Math.PI / 18, Math.PI * 2 / 3)};
  } else throw Error('相机模式必须为 schematic 或 calibrated');
  return {version: 1, units: {...DEFAULT_CONFIG.units}, model: c.model,
    table: box(c.table, '桌面'), target: box(c.target, '目标物'),
    camera, preview: pose(c.preview), ...(c.objects === undefined ? {} : {objects: validateObjects(c.objects)}),
    ...(c.gripperCamera === undefined ? {} : {gripperCamera: validateGripperCamera(c.gripperCamera)}),
    ...(c.cameraView === undefined ? {} : {cameraView: c.cameraView})};
}
const KEY = 'lebai.robot3d.scene.v1';
export function saveConfig(config, storage = localStorage) { storage.setItem(KEY, JSON.stringify(validateConfig(config))); }
export function loadConfig(storage = localStorage) {
  const raw = storage.getItem(KEY);
  if (!raw) throw Error('当前浏览器没有已保存的场景');
  return validateConfig(JSON.parse(raw));
}
