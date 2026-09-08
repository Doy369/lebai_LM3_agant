// All conversions live at boundaries; internal lengths m, angles rad, time ms.
export const degToRad = value => value * Math.PI / 180;
export const radToDeg = value => value * 180 / Math.PI;
export const mmToM = value => value / 1000;
export function number(value, name, min = -Infinity, max = Infinity) {
  if ((typeof value !== 'number' && typeof value !== 'string') ||
      (typeof value === 'string' && (!value.trim() || !/^[+-]?(?:\d+\.?\d*|\.\d+)(?:e[+-]?\d+)?$/i.test(value.trim())))) {
    throw Error(`${name}：请输入非空数值`);
  }
  const n = Number(value);
  if (!Number.isFinite(n)) throw Error(`${name}：必须是有限数值`);
  if (n < min || n > max) throw Error(`${name}：必须在 ${min}～${max} 之间`);
  return n;
}
export function vector(value, length, name, min = -Infinity, max = Infinity) {
  if (!Array.isArray(value) || value.length !== length) throw Error(`${name}：需要 ${length} 个数值`);
  return value.map((v, i) => {
    if (typeof v !== 'number') throw Error(`${name}[${i}]：需要数值类型`);
    return number(v, `${name}[${i}]`, min, max);
  });
}
export function pose(value, previewLimits = true) {
  return { joints: vector(value?.joints, 6, '关节(rad)', previewLimits ? -Math.PI : -Infinity, previewLimits ? Math.PI : Infinity),
    gripper: number(value?.gripper, '夹爪动画比例', 0, 1) };
}
export const zeroPose = () => ({joints: [0, 0, 0, 0, 0, 0], gripper: 0});

// Each semicolon-separated segment is a target, not an instruction to the robot.
export function parseTrajectory(text, durationMs) {
  const duration = number(durationMs, '每段时长(ms)', 100, 60000);
  if (typeof text !== 'string' || !text.trim()) throw Error('动作序列不能为空');
  const segments = text.trim().replace(/;\s*$/, '').split(';');
  if (segments.length > 100) throw Error('最多支持 100 段预览');
  return segments.map((segment, index) => {
    const target = { joints: {}, durationMs: duration };
    const seen = new Set();
    for (const token of segment.split(',')) {
      const match = token.trim().match(/^(关节[1-6]|夹爪)\s*:\s*(.*)$/);
      if (!match) throw Error(`第 ${index + 1} 段格式错误：${token}`);
      const [, key, raw] = match;
      if (seen.has(key)) throw Error(`第 ${index + 1} 段重复：${key}`);
      seen.add(key);
      const value = number(raw, key, key === '夹爪' ? 0 : -180, key === '夹爪' ? 100 : 180);
      if (key === '夹爪') target.gripper = value / 100;
      else target.joints[Number(key.slice(-1)) - 1] = degToRad(value);
    }
    return target;
  });
}
