import { BLOCK_SIDE_M } from './simple-physics.js';

export const MAX_OBJECTS = 20;
export const bodySize = body => body.sizeM ?? [BLOCK_SIDE_M, BLOCK_SIDE_M, BLOCK_SIDE_M];
export const bodyLabel = body => body?.label || (body?.id === 'A' ? '蓝色物块 A' : body?.id === 'B' ? '紫色物块 B' : `物块 ${body?.id ?? ''}`);
export const bodyColor = body => body?.color ?? (body?.id === 'B' ? '#9164d7' : '#167eaa');

export function validateObject(value) {
  if (!value || typeof value.id !== 'string' || !/^[A-Za-z][A-Za-z0-9_-]{0,31}$/.test(value.id)) throw Error('物块编号无效');
  if (typeof value.label !== 'string' || !value.label.trim() || value.label.trim().length > 40) throw Error('物块名称需要 1～40 个字符');
  if (typeof value.color !== 'string' || !/^#[0-9a-f]{6}$/i.test(value.color)) throw Error('物块颜色无效');
  if (!Number.isFinite(value.mass) || value.mass < .05 || value.mass > 10) throw Error('物块质量必须为 0.05～10 kg');
  const vector = (v, count, low, high, label) => {
    if (!Array.isArray(v) || v.length !== count || v.some(n => typeof n !== 'number' || !Number.isFinite(n) || n < low || n > high)) throw Error(`${label}无效`);
    return [...v];
  };
  const sizeM = vector(value.sizeM, 3, .02, .2, '物块尺寸（2～20 cm）');
  const position = vector(value.position, 3, -5, 5, '物块位置');
  const rotation = vector(value.rotation ?? [0, 0, 0, 1], 4, -1, 1, '物块旋转');
  if (Math.abs(Math.hypot(...rotation) - 1) > .0001) throw Error('物块旋转必须为单位四元数');
  return {id: value.id, label: value.label.trim(), color: value.color.toLowerCase(), mass: value.mass, sizeM, position, rotation};
}

export function validateObjects(values) {
  if (!Array.isArray(values) || values.length > MAX_OBJECTS) throw Error(`场景最多支持 ${MAX_OBJECTS} 个物块`);
  const objects = values.map(validateObject);
  if (new Set(objects.map(o => o.id)).size !== objects.length) throw Error('物块编号不能重复');
  return objects;
}
