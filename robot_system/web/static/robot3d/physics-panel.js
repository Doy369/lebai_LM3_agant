import { LivePhysics } from './live-physics.js?v=20261007-solid4';
import { PhysicsView } from './physics-view.js?v=20261007-solid4';
import { mountPhysicsDrag } from './physics-drag.js?v=20261007-showcase';
import { number } from './validation.js';

export function mountPhysicsPanel(host, scene, {signal} = {}) {
  host.innerHTML = `
    <details class="r3-physics" open><summary>网页自由演示 · 物理场景</summary>
      <p class="r3-caption">物体直接放在机械臂场景中。左键拖动物体，松开后自然运动；拖动机械臂连杆调整关节；拖空白处旋转视角。默认按屏幕方向自由移动物体，按住 Shift 可抬起。全部交互在网页内运行。</p>
      <fieldset id="r3-phys-controls" class="r3-physics-controls"><div class="r3-physics-inputs">
        <label>物体拖动方向<select id="r3-phys-plane" class="r3-panel-input"><option value="free">自由移动（随视角）</option><option value="horizontal">水平移动（X/Y）</option><option value="vertical">抬起 / 下落（X/Z）</option></select></label>
        <label>A 质量(kg)<input id="r3-phys-massA" class="r3-panel-input" type="number" min=".05" max="10" step=".05" value="1"></label>
        <label>B 质量(kg)<input id="r3-phys-massB" class="r3-panel-input" type="number" min=".05" max="10" step=".05" value="1"></label>
        <label>重力(m/s²)<input id="r3-phys-gravity" class="r3-panel-input" type="number" min="0" max="20" step=".1" value="9.81"></label>
        <label>弹性系数 e<input id="r3-phys-restitution" class="r3-panel-input" type="number" min="0" max="1" step=".1" value=".5"></label>
        <label>摩擦系数<input id="r3-phys-friction" class="r3-panel-input" type="number" min="0" max="1" step=".05" value=".15"></label>
      </div>
      <div class="r3-toolbar"><button id="r3-phys-pause">暂停物理</button><button id="r3-phys-reset">重置物体</button><button id="r3-phys-focus">观察整个场景</button>
        <label><input id="r3-phys-show" type="checkbox" checked>显示物理物块</label></div></fieldset>
      <p id="r3-phys-status" class="r3-caption" role="status"></p>
      <p class="r3-caption">参数修改立即生效。物体可以移出桌面并落到地面；机械臂关节与夹爪动画也可通过原有滑块调整。</p>
      <p id="r3-phys-error" class="r3-error" role="alert"></p>
    </details>`;
  const el = key => host.querySelector(`#r3-phys-${key}`);
  const world = new LivePhysics(), view = new PhysicsView(scene.base);
  function configureEnvironment() {
    const position = scene.table.position.toArray(), size = scene.table.scale.toArray();
    const top = Math.max(scene.groundZ, position[2] + size[2] / 2);
    world.setEnvironment({groundZ: scene.groundZ, table: {position, size}, obstacles: [
      {position: scene.target.position.toArray(), size: scene.target.scale.toArray(), label: '橙色固定物块'}], initialPositions:
      [position[0] + size[0] * .22, position[0] + size[0] * .30].map((x, i) => [x, position[1] + (i ? .16 : -.16) * size[1], top + .03])});
  }
  scene.onPhysicsEnvironmentChange = configureEnvironment; configureEnvironment(); world.reset();
  let lastMs = null, enabled = true, closed = false, showcaseActive = false;
  view.root.visible = true;
  function listen(node, event, callback) {
    node.addEventListener(event, () => {
      try { callback(); el('error').textContent = ''; }
      catch (error) { el('error').textContent = error.message; }
    }, {signal});
  }
  function show(value) {
    enabled = value; el('show').checked = value; view.root.visible = value;
  }
  function focus() { show(true); scene.focusPhysics(view.root); }
  function render() {
    const s = world.snapshot(); view.update(s, 'live');
    for (const id of ['A', 'B']) {
      const input = el(id === 'A' ? 'massA' : 'massB'), body = world.bodies.find(b => b.id === id); input.disabled = !body;
      if (body && document.activeElement !== input) input.value = body.mass;
    }
    const status = !enabled ? '已隐藏并暂停' : s.heldId ? `机械臂正在夹持物块 ${s.heldId}` : !s.running ? '已暂停物理' : s.dragId ? `正在拖动物块 ${s.dragId}` : '物理世界运行中 · 左键拖动物块';
    if (el('status').textContent !== status) el('status').textContent = status;
    el('pause').textContent = s.running ? '暂停物理' : '继续物理';
  }
  const drag = mountPhysicsDrag(scene, view, world, {mode: () => el('plane').value, onChange: () => render(), enabled: () => !showcaseActive && !scene.pointerMode && !scene.motionBusy});
  for (const name of ['massA', 'massB', 'gravity', 'restitution', 'friction']) {
    listen(el(name), 'change', () => { world.configure({[name]: number(el(name).value, name)}); render(); });
  }
  listen(el('plane'), 'change', () => { drag.release(); render(); });
  listen(el('pause'), 'click', () => {
    drag.release(); world.setRunning(!world.running); if (world.running) show(true); lastMs = null; render();
  });
  listen(el('reset'), 'click', () => {
    drag.release(); world.reset(); show(true); lastMs = null; render();
  });
  listen(el('focus'), 'click', focus);
  listen(el('show'), 'change', () => {
    drag.release(); show(el('show').checked); world.setRunning(enabled); lastMs = null; render();
  });
  document.addEventListener('visibilitychange', () => {
    if (document.hidden) { drag.release(); world.setRunning(false); lastMs = null; render(); }
  }, {signal});
  render();
  const query = new URLSearchParams(location.search);
  if (query.get('physicsDemo') === '1' || query.get('physics') === '1') focus();
  return {
    tick(now) {
      if (closed) return;
      if (enabled && lastMs != null) world.advance(Math.max(0, (now - lastMs) / 1000));
      lastMs = now; if (enabled) render();
    },
    focus,
    focusObjects() { show(true); scene.focusObjects(view.root); },
    world,
    refresh() { render(); },
    releaseDrag() { drag.release(); },
    ensureRunning() { show(true); world.setRunning(true); lastMs = null; },
    resetObjects() { drag.release(); world.reset(); show(true); lastMs = null; render(); },
    beginShowcase() {
      drag.release(); showcaseActive = true; el('controls').disabled = true;
      world.reset(); show(true); lastMs = null; render();
    },
    showcaseCue(cue) {
      if (cue === 'reset') world.reset();
      if (cue === 'drop') {
        world.reset();
        world.bodies.forEach((body, i) => { body.position[2] += i ? .28 : .42; });
      }
      lastMs = null; render();
    },
    pauseShowcase(paused) { world.setRunning(!paused); lastMs = null; render(); },
    endShowcase() {
      showcaseActive = false; el('controls').disabled = false;
      world.setRunning(true); lastMs = null; render();
    },
    dispose() {
      closed = true; drag.dispose(); world.setRunning(false); view.dispose();
      if (scene.onPhysicsEnvironmentChange === configureEnvironment) scene.onPhysicsEnvironmentChange = null;
    },
  };
}
