import { vector, number, pose, zeroPose } from './validation.js';
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
  if (c.camera?.mode !== 'schematic') throw Error('尚无实测核对记录，不能启用真实相机对齐');
  const positionBaseM = vector(c.camera.positionBaseM, 3, '相机位置(m)', -5, 5);
  const targetBaseM = vector(c.camera.targetBaseM, 3, '相机目标(m)', -5, 5);
  if (Math.hypot(...positionBaseM.map((v, i) => v - targetBaseM[i])) < .001) throw Error('相机位置不能与观察目标重合');
  return {version: 1, units: {...DEFAULT_CONFIG.units}, model: c.model,
    table: box(c.table, '桌面'), target: box(c.target, '目标物'),
    camera: {mode: 'schematic', positionBaseM, targetBaseM, fovRad: number(c.camera.fovRad, '相机视场角(rad)', Math.PI / 18, Math.PI * 2 / 3)}, preview: pose(c.preview)};
}
const KEY = 'lebai.robot3d.scene.v1';
export function saveConfig(config, storage = localStorage) { storage.setItem(KEY, JSON.stringify(validateConfig(config))); }
export function loadConfig(storage = localStorage) {
  const raw = storage.getItem(KEY);
  if (!raw) throw Error('当前浏览器没有已保存的场景');
  return validateConfig(JSON.parse(raw));
}
