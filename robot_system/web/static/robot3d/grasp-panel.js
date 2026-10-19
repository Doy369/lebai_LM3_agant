import { Mesh, SphereGeometry, RingGeometry, MeshBasicMaterial, Vector3, DoubleSide } from './vendor/three.module.js';
import { virtualToolMatrix } from './virtual-tool.js?v=20261007-solid4';
import { VirtualGrasp, blockName, GRASP_RADIUS_M } from './virtual-grasp.js?v=20261007-solid4';
import { UPRIGHT_REFERENCE_JOINTS_RAD } from './joints.js';
import { planVirtualMotion, transferWaypoints } from './virtual-motion.js?v=20261007-solid4';
import { bodyLabel, bodySize, bodyColor, MAX_OBJECTS } from './objects.js';
import { bodyBox, boxExtents } from './solid-collisions.js';
import { pickTablePoint, tableObject, freeTableObject, placementTarget } from './placement.js';
import { number } from './validation.js';

export function mountGraspPanel(host, scene, physics, {getRig, getPose, getSolids, prepare, applyPose, play, stop, moving, interactive, signal}) {
  const world = physics.world, grasp = new VirtualGrasp(world);
  host.innerHTML = `<section class="r3-grasp"><div class="r3-toolbar"><strong>网页夹取控制</strong><button id="r3-grasp-enter">进入夹取控制</button><button id="r3-grasp-stop" disabled>停止夹取移动</button><label><input id="r3-grasp-volumes" type="checkbox">显示实体体积</label></div>
    <fieldset id="r3-grasp-controls" class="r3-grasp-controls"><div class="r3-toolbar">
      <label>夹取目标 <select id="r3-grasp-target" aria-label="夹取目标"></select></label>
      <button id="r3-grasp-add">添加物块</button><button id="r3-grasp-delete">删除所选</button>
      <button id="r3-grasp-approach">靠近物块</button><button id="r3-grasp-close">闭合并夹取</button><button id="r3-grasp-lift">抬高物块</button>
      <button id="r3-grasp-place">放到左侧</button><button id="r3-grasp-open">张开并松开</button><button id="r3-grasp-reset">复位夹取场景</button></div>
      <div class="r3-toolbar"><button id="r3-grasp-pick-place" aria-pressed="false">选择放置点</button><button id="r3-grasp-place-picked" disabled>搬到指定点并松开</button><button id="r3-grasp-cancel-pick" hidden>取消选点</button><span id="r3-grasp-place-point" class="r3-caption">尚未选择放置点</span></div>
      <div class="r3-toolbar"><label>末端步长 <select id="r3-grasp-step" aria-label="末端步长"><option value=".03">3 cm</option><option value=".01">1 cm</option><option value=".05">5 cm</option></select></label>
      ${['+X', '-X', '+Y', '-Y', '+Z', '-Z'].map((label, i) => `<button id="r3-grasp-jog-${i}">末端 ${label}</button>`).join('')}</div></fieldset>
    <details class="r3-object-settings"><summary>物块设置 · <span id="r3-grasp-count"></span></summary>
      <fieldset id="r3-grasp-object-controls"><div class="r3-object-inputs">
        <label>物块名称<input id="r3-grasp-object-name" maxlength="40" value="新物块"></label>
        <label>长度 X(cm)<input id="r3-grasp-size-x" type="number" min="2" max="20" step="1" value="6"></label>
        <label>宽度 Y(cm)<input id="r3-grasp-size-y" type="number" min="2" max="20" step="1" value="6"></label>
        <label>高度 Z(cm)<input id="r3-grasp-size-z" type="number" min="2" max="20" step="1" value="6"></label>
        <label>物块质量(kg)<input id="r3-grasp-object-mass" type="number" min=".05" max="10" step=".05" value="1"></label>
        <label>物块颜色<input id="r3-grasp-object-color" type="color" value="#35bca9"></label></div>
        <div class="r3-toolbar"><button id="r3-grasp-pick-add" aria-pressed="false">选择新增位置</button><button id="r3-grasp-update">应用到所选物块</button><button id="r3-grasp-load-object">读取所选属性</button></div>
      </fieldset><p id="r3-grasp-add-point" class="r3-caption">添加时自动寻找桌面空位；也可先选择新增位置。</p>
      <p class="r3-caption">尺寸、颜色与质量用于新增物块，也可应用到所选物块。大物块可能超出夹爪开口；夹持时请先松开再修改或删除。</p></details>
    <p id="r3-grasp-state" class="r3-caption" role="status"></p>
    <p class="r3-caption">靠近 → 闭合夹取 → 抬高 / 移动 → 张开放下。拖动关节碰到实体或自身死区会停住；勾选“显示实体体积”可查看碰撞边界。全部操作在网页中完成。</p></section>`;
  const el = key => host.querySelector(`#r3-grasp-${key}`);
  const marker = new Mesh(new SphereGeometry(.006, 12, 8), new MeshBasicMaterial({color: 0x00afc7, depthTest: false}));
  marker.renderOrder = 5; marker.visible = false; marker.name = 'virtual-grasp-point'; scene.base.add(marker);
  const destination = new Mesh(new RingGeometry(.018, .028, 32), new MeshBasicMaterial({color: 0x10a86b, side: DoubleSide, depthTest: false}));
  destination.name = 'placement-target'; destination.renderOrder = 6; destination.visible = false; scene.base.add(destination);
  let motionLabel = '', planning = false, revision = 0, objectsRevision = -1, placePoint = null, addPoint = null, completion = null, controlsBeforePick = true;
  const name = id => blockName(id, world);
  const selected = () => world.bodies.find(body => body.id === el('target').value);
  const listen = (key, callback) => el(key).addEventListener('click', async () => {
    try { await callback(); } catch (error) { grasp.message = error.message; motionLabel = ''; }
  }, {signal});
  function refreshObjects(selectId) {
    if (objectsRevision === world.objectRevision && !selectId) return;
    const id = selectId ?? el('target').value;
    el('target').replaceChildren(...world.bodies.map(body => {
      const option = document.createElement('option'); option.value = body.id; option.textContent = `${bodyLabel(body)} (${body.id})`; return option;
    }));
    if (world.bodies.some(b => b.id === id)) el('target').value = id;
    grasp.targetId = el('target').value || null;
    el('count').textContent = `${world.bodies.length} / ${MAX_OBJECTS} 个物块`; objectsRevision = world.objectRevision;
  }
  function objectForm() {
    return {label: el('object-name').value.trim() || '新物块', sizeM: ['x', 'y', 'z'].map(axis => number(el(`size-${axis}`).value, '物块尺寸(cm)', 2, 20) / 100),
      mass: number(el('object-mass').value, '物块质量(kg)', .05, 10), color: el('object-color').value};
  }
  function loadObject() {
    const body = selected(); if (!body) throw Error('请先选择物块');
    el('object-name').value = bodyLabel(body); el('object-mass').value = body.mass; el('object-color').value = bodyColor(body);
    ['x', 'y', 'z'].forEach((axis, i) => { el(`size-${axis}`).value = (bodySize(body)[i] * 100).toFixed(1); });
  }
  function endPicking() {
    if (!scene.pointerMode) return;
    scene.pointerMode = null; scene.controls.enabled = controlsBeforePick; scene.renderer.domElement.style.cursor = '';
    el('pick-place').setAttribute('aria-pressed', 'false'); el('pick-add').setAttribute('aria-pressed', 'false'); el('cancel-pick').hidden = true;
  }
  function beginPicking(mode) {
    if (scene.pointerMode === mode) { endPicking(); grasp.message = '已取消选点'; return; }
    if (moving() || planning) throw Error('请先停止机械臂移动');
    prepare(); endPicking(); physics.releaseDrag(); controlsBeforePick = scene.controls.enabled; scene.controls.enabled = false;
    scene.pointerMode = mode; scene.renderer.domElement.style.cursor = 'crosshair'; el('cancel-pick').hidden = false;
    el(mode === 'place' ? 'pick-place' : 'pick-add').setAttribute('aria-pressed', 'true');
    if (mode === 'place') { placePoint = null; el('place-point').textContent = '请点击桌面上的放置位置'; }
    else { addPoint = null; el('add-point').textContent = '请点击桌面上的新增位置'; }
    destination.visible = false; grasp.message = '选点模式 · 点击桌面空位，或点击“取消选点”退出';
  }
  listen('pick-place', () => beginPicking('place')); listen('pick-add', () => beginPicking('add'));
  listen('cancel-pick', () => { endPicking(); grasp.message = '已取消选点'; });
  listen('add', () => {
    if (world.heldId) throw Error('请先松开物块，再添加');
    const spec = objectForm(), object = addPoint ? tableObject(world, spec, addPoint) : freeTableObject(world, spec);
    const body = world.addBody(object); physics.refresh(); refreshObjects(body.id); endPicking(); addPoint = null;
    el('add-point').textContent = '添加时自动寻找桌面空位；也可先选择新增位置。';
    grasp.message = `已添加${bodyLabel(body)} · 已选为夹取目标，可直接拖动或靠近夹取`;
  });
  listen('delete', () => {
    const body = selected(); if (!body) throw Error('请先选择物块');
    world.removeBody(body.id); physics.refresh(); refreshObjects(); grasp.message = `已删除${bodyLabel(body)}`;
  });
  listen('load-object', loadObject);
  listen('update', () => {
    const body = selected(); if (!body) throw Error('请先选择物块');
    world.updateBody(body.id, objectForm()); physics.refresh(); refreshObjects(body.id); grasp.message = '所选物块的尺寸、质量和颜色已更新';
  });
  el('target').addEventListener('change', () => { grasp.targetId = el('target').value || null; }, {signal});
  const canvas = scene.renderer.domElement;
  canvas.addEventListener('pointerdown', event => {
    if (!scene.pointerMode || event.button !== 0 || !event.isPrimary) return;
    event.preventDefault(); event.stopImmediatePropagation();
    try {
      const point = pickTablePoint(scene.camera, scene.base, world.environment.table, canvas.getBoundingClientRect(), event.clientX, event.clientY);
      if (!point) throw Error('请点击桌面内部');
      const held = world.bodies.find(b => b.id === world.heldId), body = held ?? selected();
      const spec = scene.pointerMode === 'add' ? objectForm() : body ?? objectForm();
      const candidate = tableObject(world, spec, point, .002);
      const reason = world.occupancyReason(candidate, scene.pointerMode === 'place' ? held?.id : null); if (reason) throw Error(reason);
      const text = `X ${(point[0] * 100).toFixed(1)} cm · Y ${(point[1] * 100).toFixed(1)} cm`;
      if (scene.pointerMode === 'place') { placePoint = point; el('place-point').textContent = `放置点：${text}`; }
      else { addPoint = point; el('add-point').textContent = `新增位置：${text} · 点击“添加物块”`; }
      destination.position.fromArray(point); destination.position.z += .002; destination.visible = true; endPicking();
      grasp.message = held ? '放置点已选择 · 点击“搬到指定点并松开”' : '桌面位置已选择 · 可以添加物块或先完成夹取';
    } catch (error) { grasp.message = error.message; }
  }, {capture: true, signal});
  window.addEventListener('keydown', event => { if (event.key === 'Escape' && scene.pointerMode) { endPicking(); grasp.message = '已取消选点'; } }, {signal});
  function minimumHeight(x, y) {
    const {groundZ, table} = world.environment;
    const body = world.bodies.find(b => b.id === world.heldId) ?? selected(), extent = body ? boxExtents(bodyBox(body)) : [.03, .03, .03];
    return Math.max(groundZ, table && [x, y].every((v, i) => Math.abs(v - table.position[i]) < table.size[i] / 2 + extent[i])
      ? table.position[2] + table.size[2] / 2 : groundZ) + extent[2] + .006;
  }
  async function move(points, label, gripper = getPose().gripper) {
    endPicking(); completion = null; prepare(); const rig = getRig(), original = {...getPose(), gripper}, ticket = ++revision;
    planning = true; motionLabel = '正在检查路径与实体碰撞…';
    // First travel at clearance height, then follow short Cartesian waypoints.
    // Linear interpolation of just two joint poses can curve the fingers into a cube.
    try {
      const targets = points.map(point => [point[0], point[1], Math.max(point[2], minimumHeight(point[0], point[1]))]);
      const segments = await planVirtualMotion(rig, scene.base, getSolids(), original, targets, {
        firstJointMove: label.startsWith('正在靠近'), displayPose: p => grasp.displayPose(p), heldAt: p => grasp.heldAt(rig, scene.base, p),
        yieldControl: async () => {
          await new Promise(resolve => requestAnimationFrame(resolve));
          if (ticket !== revision || signal?.aborted) throw Error('已取消路径规划');
        },
      });
      if (ticket !== revision || signal?.aborted) return false;
      motionLabel = label; play(segments); return true;
    } finally { planning = false; }
  }
  function resetScene() {
    completion = null; endPicking(); placePoint = null; addPoint = null; destination.visible = false;
    el('place-point').textContent = '尚未选择放置点'; el('add-point').textContent = '添加时自动寻找桌面空位；也可先选择新增位置。';
    prepare(); grasp.reset(); physics.resetObjects(); refreshObjects();
    applyPose({joints: [...UPRIGHT_REFERENCE_JOINTS_RAD], gripper: 1}, {reset: true}); physics.focus();
    grasp.message = '场景已复位 · 选择物块后点击“靠近物块”';
  }
  listen('enter', () => {
    prepare(); applyPose({...getPose(), gripper: 1}); physics.focus();
    grasp.message = '已进入夹取控制 · 先靠近物块，再闭合夹爪';
  });
  listen('reset', resetScene);
  listen('stop', () => { revision++; completion = null; endPicking(); stop(); motionLabel = ''; grasp.message = world.heldId ? `已停止移动 · 仍夹持${name(world.heldId)}` : '已停止移动 · 可以继续操作'; });
  listen('approach', async () => {
    const body = world.bodies.find(body => body.id === el('target').value);
    if (!body) throw Error('请先添加或选择一个物块');
    grasp.targetId = body.id;
    const p = [...body.position];
    if (await move([[p[0], p[1], p[2] + .16], [p[0], p[1], p[2] + .004]], `正在靠近${name(body.id)}`, 1)) grasp.message = '已靠近物块 · 点击“闭合并夹取”';
  });
  listen('close', () => {
    endPicking(); prepare(); grasp.arm(el('target').value || null); motionLabel = '正在闭合夹爪'; play([{joints: {}, gripper: 0, durationMs: 450}]);
  });
  listen('open', () => {
    completion = null; endPicking(); prepare(); motionLabel = '正在张开夹爪'; play([{joints: {}, gripper: 1, durationMs: 450}]);
  });
  listen('lift', () => {
    const position = new Vector3().setFromMatrixPosition(virtualToolMatrix(getRig(), scene.base)).toArray();
    position[2] += .18; return move([position], '正在抬高物块');
  });
  listen('place', async () => {
    const table = world.environment.table, current = new Vector3().setFromMatrixPosition(virtualToolMatrix(getRig(), scene.base));
    const x = (table?.position[0] ?? 0) - (table?.size[0] ?? 1.2) * .23, y = (table?.position[1] ?? 0) - (table?.size[1] ?? 1) * .16;
    const z = minimumHeight(x, y) + .01;
    if (await move(transferWaypoints(current.toArray(), x, y, z), '正在移动到左侧放置位置')) grasp.message = '已到左侧放置位置 · 点击“张开并松开”放下物块';
  });
  listen('place-picked', async () => {
    const body = world.bodies.find(b => b.id === world.heldId);
    if (!body || !grasp.attachment) throw Error('请先夹住物块');
    if (!placePoint) throw Error('请先选择放置点');
    const {toolPoint} = placementTarget(world, body, placePoint, grasp.attachment);
    const current = new Vector3().setFromMatrixPosition(virtualToolMatrix(getRig(), scene.base)).toArray();
    if (await move(transferWaypoints(current, ...toolPoint), '正在搬运到指定放置点')) {
      completion = () => { motionLabel = '正在松开物块'; play([{joints: {}, gripper: 1, durationMs: 450}]); };
      grasp.message = '已到指定放置点 · 正在松开物块';
    }
  });
  for (let index = 0; index < 6; index++) listen(`jog-${index}`, () => {
    const point = new Vector3().setFromMatrixPosition(virtualToolMatrix(getRig(), scene.base)).toArray();
    point[Math.floor(index / 2)] += (index % 2 ? -1 : 1) * Number(el('step').value);
    if (point[2] < minimumHeight(point[0], point[1]) - .0001) throw Error('已到桌面 / 地面边界，不能继续下降');
    return move([point], '末端移动中');
  });
  return {
    resetScene,
    refreshScene() {
      refreshObjects(); placePoint = null; addPoint = null; destination.visible = false;
      el('place-point').textContent = '尚未选择放置点'; el('add-point').textContent = '添加时自动寻找桌面空位；也可先选择新增位置。';
      grasp.message = '场景配置已应用 · 选择物块后可以继续夹取';
    },
    cancelMotion() { revision++; completion = null; endPicking(); motionLabel = ''; },
    release() { revision++; completion = null; endPicking(); grasp.reset(); },
    planning() { return planning; },
    showVolumes() { return el('volumes').checked; },
    collisionPose(p) { return grasp.displayPose(p); },
    heldAt(p) { return grasp.heldAt(getRig(), scene.base, p); },
    blocked(reason) { completion = null; motionLabel = ''; grasp.message = `已停在边界 · ${reason}，请向反方向移动`; },
    clearBlocked() {
      if (grasp.message.startsWith('已停在边界')) grasp.message = world.heldId ? `已夹住${name(world.heldId)} · 可以继续移动` : '已退出实体边界 · 可以继续操作';
    },
    tick(now, p) {
      const rig = getRig(), available = Boolean(rig && p && interactive());
      refreshObjects();
      if (completion && !moving() && !planning) { const done = completion; completion = null; done(); }
      el('enter').disabled = !rig || planning; el('controls').disabled = !available || moving() || planning;
      el('object-controls').disabled = !available || moving() || planning || Boolean(world.heldId);
      el('stop').disabled = !available || !(moving() || planning);
      const held = Boolean(world.heldId);
      el('target').disabled = held; el('add').disabled = held || world.bodies.length >= MAX_OBJECTS; el('delete').disabled = held || !selected();
      el('approach').disabled = held || !selected(); el('close').disabled = held || !selected();
      el('lift').disabled = !held; el('place').disabled = !held;
      el('place-picked').disabled = !held || !placePoint; el('update').disabled = !selected(); el('load-object').disabled = !selected();
      marker.visible = available;
      let displayed = p;
      if (available) {
        displayed = grasp.update(rig, scene.base, p, now);
        const tool = new Vector3().setFromMatrixPosition(virtualToolMatrix(rig, scene.base)); marker.position.copy(tool);
        const near = world.bodies.some(body => tool.distanceTo(new Vector3(...body.position)) <= GRASP_RADIUS_M);
        marker.material.color.setHex(held || near ? 0x27b76e : 0x00afc7);
      } else if (world.heldId) grasp.reset();
      const status = !rig ? '请等待内置模型加载，或恢复内置模型' : !available ? '点击“进入夹取控制”后即可操作' : moving() || planning ? motionLabel || '机械臂移动中' : grasp.message;
      if (el('state').textContent !== status) el('state').textContent = status;
      host.dataset.held = world.heldId ?? ''; return displayed;
    },
    dispose() { revision++; completion = null; endPicking(); grasp.reset(); for (const mesh of [marker, destination]) { mesh.geometry.dispose(); mesh.material.dispose(); mesh.removeFromParent(); } },
  };
}
