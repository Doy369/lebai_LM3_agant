import { validateObjects, MAX_OBJECTS } from './objects.js';

const point = (value, label) => {
  const array = Array.isArray(value) ? value : value && [value.x, value.y, value.z];
  if (!array || array.length !== 3 || array.some(n => typeof n !== 'number' || !Number.isFinite(n) || Math.abs(n) > 5)) throw Error(`${label}需要三个以米为单位的有效数值`);
  return [...array];
};
const photo = value => typeof value === 'string' && /^\/captures\/[a-f0-9]{32}-(snapshot|annotated)\.jpg$/.test(value) ? value : null;

// point_base_m in the old camera plan is the compensated flange command, not an object center.
export function parseRealObjects(payload) {
  if (!payload || typeof payload !== 'object') throw Error('识别结果必须是 JSON 对象');
  if (payload.schema !== 'virtual-grasp-real-objects/v1') {
    const plan = payload.data ?? payload;
    if (plan.success !== true || !plan.surface_point_base_m) throw Error('需要相机原始规划中的 surface_point_base_m；执行点和旧世界状态不能当作物体位置');
    payload = {schema: 'virtual-grasp-real-objects/v1', source: 'import', units: 'm', frame: 'robot-base', snapshotId: plan.snapshot_id,
      objects: [{id: 'R1', label: plan.decision?.selected_object_name ?? '识别物体', positionBaseM: plan.surface_point_base_m, pointKind: 'surface'}]};
  }
  if (payload.units !== 'm' || payload.frame !== 'robot-base') throw Error('识别结果必须使用机械臂基座坐标系 robot-base，长度单位 m');
  if (!['real-camera', 'sample', 'import'].includes(payload.source)) throw Error('识别结果来源无效');
  if (!Array.isArray(payload.objects) || !payload.objects.length || payload.objects.length > MAX_OBJECTS) throw Error(`需要 1～${MAX_OBJECTS} 个识别物体`);
  const objects = payload.objects.map((o, i) => {
    if (!o || !['surface', 'center'].includes(o.pointKind)) throw Error('物体测量点类型需要 surface 或 center');
    const id = o.id ?? `R${i + 1}`, label = o.label ?? '识别物体';
    if (typeof id !== 'string' || !/^[A-Za-z][A-Za-z0-9_-]{0,31}$/.test(id)) throw Error('识别物体编号无效');
    if (typeof label !== 'string' || !label.trim() || label.trim().length > 40) throw Error('识别物体名称需要 1～40 个字符');
    const sizeM = o.sizeM == null ? null : point(o.sizeM, '物体尺寸');
    if (sizeM?.some(n => n < .02 || n > .2)) throw Error('当前虚拟物块尺寸支持 2～20 cm');
    const yawRad = o.yawRad ?? 0;
    if (typeof yawRad !== 'number' || !Number.isFinite(yawRad) || Math.abs(yawRad) > Math.PI) throw Error('物体朝向需要 -π～π 弧度');
    const color = o.color ?? null;
    if (color !== null && (typeof color !== 'string' || !/^#[0-9a-f]{6}$/i.test(color))) throw Error('识别物体颜色无效');
    return {id, label: label.trim(), positionBaseM: point(o.positionBaseM, '物体测量点'), pointKind: o.pointKind, sizeM, yawRad, color};
  });
  if (new Set(objects.map(o => o.id)).size !== objects.length) throw Error('识别物体编号不能重复');
  return {source: payload.source, snapshotId: String(payload.snapshotId ?? '').slice(0, 160),
    capturedAt: typeof payload.capturedAt === 'string' && Number.isFinite(Date.parse(payload.capturedAt)) ? payload.capturedAt : null,
    objects, snapshotUrl: photo(payload.snapshotUrl), annotatedUrl: photo(payload.annotatedUrl)};
}

export function realObjectsToBodies(capture, {sizeM = [.06, .06, .06], tableTopM = 0, heightMode = 'table', mass = 1, color = '#35bca9'} = {}) {
  if (!['table', 'measured'].includes(heightMode)) throw Error('物体高度模式无效');
  if (typeof tableTopM !== 'number' || !Number.isFinite(tableTopM) || Math.abs(tableTopM) > 5) throw Error('桌面高度无效');
  return validateObjects(capture.objects.map(o => {
    const size = o.sizeM ?? sizeM, position = [...o.positionBaseM];
    if (heightMode === 'table') position[2] = tableTopM + size[2] / 2 + .0001;
    else if (o.pointKind === 'surface') position[2] -= size[2] / 2;
    return {id: o.id, label: o.label, sizeM: [...size], position, mass, color: o.color ?? color,
      rotation: [0, 0, Math.sin(o.yawRad / 2), Math.cos(o.yawRad / 2)]};
  }));
}
