import { PoseState } from './state.js';
import { FeedbackSync } from './sync.js';
import { number, degToRad, radToDeg, parseTrajectory } from './validation.js';
import { ModelLoader } from './model-loader.js?v=20260913-joint5';
import { RobotScene } from './scene.js?v=20261013-image-objects2';
import { DEFAULT_CONFIG, validateConfig, saveConfig, loadConfig } from './config.js';
import { calibratedCameraConfigFromApi } from './camera-calibration.js';
import { compiledPlanToTrajectory } from './compiled-plan.js';
import { syntheticCollisionDemo } from './collision-overlay.js';
import { mountPhysicsPanel } from './physics-panel.js?v=20261013-image-objects2';
import { mountArmDrag } from './arm-drag.js';
import { ShowcasePlayer, SHOWCASE_DURATION_MS } from './showcase.js';
import { mountGraspPanel } from './grasp-panel.js?v=20261007-solid4';
import { RobotSolids, SolidVolumeView, bodyBox, solidBox } from './solid-collisions.js?v=20261007-solid4';
import { UPRIGHT_REFERENCE_JOINTS_RAD } from './joints.js';
import { LivePhysics } from './live-physics.js';
import { DEFAULT_GRIPPER_CAMERA } from './gripper-camera.js';
import { mountRealObjectsPanel } from './real-objects-panel.js?v=20261013-image-objects';

const DEFAULT_MODEL = new URL('./assets/Lebai_LM3.glb', import.meta.url);
export function mountRobot3D(root, {readStatus, readCalibration, captureObjects, readCameraObjects, getCameraCommand, feedbackIntervalMs = 100}) {
  const state = new PoseState(); let config = validateConfig({...DEFAULT_CONFIG, cameraView: 'gripper'});
  const showcase = new ShowcasePlayer();
  let scene, loader, physics, armDrag, graspPanel, externalModel = false, closed = false, modelRequest = 0, lastReadoutMs = 0, activeCompiledPlanId = null;
  let activeCollisionPreview = null, collisionPinnedSampleIndex = null, collisionVisible = true;
  let solids = null, solidView = null, acceptedPose = null;
  let realObjects;
  const autoCollisionDemo = new URLSearchParams(location.search).get('collisionDemo') === '1';
  let autoCollisionDemoLoaded = false;
  const lifetime = new AbortController();
  root.innerHTML = `
    <div class="r3-head"><div><p class="panel-kicker">Virtual workspace / 只读姿态</p><h2>机械臂三维预览</h2></div>
      <label>姿态来源 <select id="r3-mode"><option value="manual">手动预览</option><option value="feedback">实时反馈</option></select></label></div>
    <p class="r3-notice">当前真机姿态已按现场照片完成单点对齐；Joint1、Joint2、Joint5 方向已由现场运动确认，其余关节及 Base 与模型的对应关系仍待逐轴实测。±180°仅为预览输入范围；夹爪百分比仅表示动画进度。碰撞层使用待实测胶囊体/AABB，只能显示后端近似验证结果。</p>
    <div id="r3-sync" class="r3-status" role="status">手动预览 · 不读取实时姿态</div>
    <div class="r3-showcase"><div class="r3-toolbar"><button id="r3-showcase-play" disabled>播放展示动画</button><button id="r3-showcase-pause" disabled>暂停展示</button><button id="r3-showcase-stop" disabled>停止展示</button>
      <label><input id="r3-showcase-loop" type="checkbox" checked>循环播放</label></div>
      <p id="r3-showcase-state" class="r3-caption" role="status">约 ${Math.round(SHOWCASE_DURATION_MS / 1000)} 秒 · 关节运动、夹爪开合、物块下落 · 可随时旋转视角</p></div>
    <div id="r3-grasp-host"></div>
    <div class="r3-views"><div><div id="r3-main" class="r3-main" aria-label="自由观察三维画面"><div id="r3-showcase-overlay" class="r3-showcase-overlay" hidden><strong id="r3-showcase-phase"></strong><progress id="r3-showcase-progress" max="1" value="0" aria-label="展示动画进度"></progress></div></div>
      <p id="r3-object-caption" class="r3-caption" hidden></p><p class="r3-caption">左键拖动连杆 / 物体 · 拖空白处旋转 · 滚轮缩放 · 右键平移。连杆拖动在手动预览且未播放动画时启用。</p></div>
      <div><div class="r3-toolbar r3-camera-tools"><label>摄像头视角<select id="r3-camera-view"><option value="gripper">夹爪相机</option><option value="fixed">固定相机</option></select></label><button id="r3-camera-zoom">放大画面</button></div>
      <div id="r3-camera" class="r3-camera" aria-label="虚拟摄像头画面"></div><p id="r3-camera-caption" class="r3-caption" role="status">夹爪摄像头加载中…</p>
      <div class="r3-camera-options"><label>夹爪相机视场角(°)<input id="r3-camera-fov" type="range" min="30" max="110" step="1" value="60"><output id="r3-camera-fov-value">60°</output></label><label><input id="r3-camera-mount" type="checkbox" checked>显示夹爪摄像头</label></div>
      <pre id="r3-readout" class="r3-readout">等待模型</pre></div></div>
    <dialog id="r3-camera-dialog" class="r3-camera-dialog" aria-labelledby="r3-camera-dialog-title"><div class="r3-toolbar"><strong id="r3-camera-dialog-title">夹爪摄像头</strong><button id="r3-camera-close">关闭相机画面</button></div><div id="r3-camera-large" class="r3-camera"></div><p class="r3-caption">实时虚拟摄像头画面 · 按 Esc 关闭</p></dialog>
    <div class="r3-toolbar"><button id="r3-free-mode">网页自由演示</button><button id="r3-view-reset">视角复位</button><button id="r3-side-view">校准侧视</button><button id="r3-top-view">校准俯视</button><button id="r3-frame">居中观察</button>
      <button id="r3-builtin">恢复内置模型</button><button id="r3-load-camera">加载标定相机</button><label>替换预览模型 <input id="r3-file" type="file" accept=".glb"></label></div>
    <p id="r3-model-status" class="r3-caption" role="status">模型加载中…</p>
    <p id="r3-free-status" class="r3-caption" role="status">网页自由演示 · 拖动连杆调整关节，Shift 可精细调整；滑块可控制关节与夹爪动画。</p>
    <p id="r3-solid-status" class="r3-caption" role="status">实体碰撞约束加载中…</p>
    <div id="r3-real-host"></div>
    <fieldset id="r3-manual" class="r3-joints">${Array.from({length:7}, (_, i) => {
      const label = i < 6 ? `关节${i + 1} (°)` : '夹爪动画 (%)';
      return `<div class="r3-joint"><label for="r3-number-${i}">${label}</label><input id="r3-number-${i}" type="text" inputmode="decimal" value="0"><input id="r3-range-${i}" aria-label="${label}滑块" type="range" min="${i < 6 ? -180 : 0}" max="${i < 6 ? 180 : 100}" step="1" value="0"></div>`;
    }).join('')}</fieldset>
    <div class="r3-trajectory"><label>预览动作序列（角度 / 百分比）<textarea id="r3-trajectory" class="r3-panel-input" rows="2">关节1:30,关节2:-30,夹爪:100;关节1:0,关节2:0,夹爪:0;</textarea></label>
      <label>每段时长(ms)<input id="r3-duration" class="r3-panel-input" type="text" inputmode="decimal" value="2000"></label></div>
    <div class="r3-toolbar"><button id="r3-play">播放预览</button><button id="r3-stop">停止预览</button><button id="r3-reset">重置预览</button><span id="r3-play-state">已停止</span></div>
    <p id="r3-plan-status" class="r3-caption">尚未接收统一抓取计划。</p>
    <section id="r3-collision-panel" class="r3-collision-panel" data-kind="idle" aria-live="polite">
      <div class="r3-collision-head"><strong>轨迹碰撞层</strong><span id="r3-collision-state">等待碰撞验证报告</span></div>
      <div class="r3-collision-grid">
        <span>最小间隙 <b id="r3-clearance">—</b></span><span>最近对象 <b id="r3-pair">—</b></span>
        <span>阶段/样本 <b id="r3-collision-sample">—</b></span><span>场景状态 <b id="r3-collision-scene">—</b></span>
      </div>
      <div class="r3-toolbar"><button id="r3-demo-collision">加载离线碰撞示例</button><button id="r3-focus-collision" disabled>定位最小间隙姿态</button><button id="r3-toggle-collision" disabled>隐藏碰撞层</button></div>
      <p class="r3-caption">青色：连杆胶囊体；橙色：最近对象与间隙线；红色：当前样本发生近似相交。数值为后端报告结果。</p>
    </section>
    <div id="r3-physics-host"></div>
    <div id="r3-error" class="r3-error" role="alert"></div>
    <details class="r3-config"><summary>场景配置 · 保存与恢复</summary><p class="r3-caption">位置和尺寸使用 Base 坐标系与米，关节使用弧度。可修改桌面、目标物、示意摄像头与手动姿态。保存不会保存令牌、反馈或本地上传模型。</p>
      <textarea id="r3-config" class="r3-panel-input" aria-label="场景配置 JSON" rows="12"></textarea>
      <div class="r3-toolbar"><button id="r3-apply">应用配置</button><button id="r3-save">保存当前场景</button><button id="r3-restore">恢复已保存场景</button><button id="r3-export">导出 JSON</button><label>导入 JSON <input id="r3-import" type="file" accept=".json"></label></div>
      <p id="r3-config-status" role="status"></p></details>`;
  const el = id => root.querySelector(`#r3-${id}`);
  const showError = e => { el('error').textContent = e?.message ?? ''; };
  const on = (node, event, fn) => node.addEventListener(event, e => {
    Promise.resolve().then(() => fn(e)).catch(showError);
  }, {signal: lifetime.signal});
  function renderCollisionPanel(data) {
    activeCollisionPreview = data; collisionPinnedSampleIndex = null; collisionVisible = true;
    scene?.setCollisionVisualization(data); scene?.setCollisionVisible(true);
    el('toggle-collision').textContent = '隐藏碰撞层';
    el('toggle-collision').disabled = !data;
    el('focus-collision').disabled = !data?.focusJoints;
    if (!data) {
      el('collision-panel').dataset.kind = 'idle'; el('collision-state').textContent = '当前计划没有碰撞验证明细';
      for (const id of ['clearance', 'pair', 'collision-sample', 'collision-scene']) el(id).textContent = '—';
      return;
    }
    const rejected = data.status === 'collision';
    el('collision-panel').dataset.kind = rejected ? 'collision' : 'approximate';
    el('collision-state').textContent = rejected ? '检测到近似相交 / 已拒绝执行' : '近似模型未发现相交 / 仍需复核';
    el('clearance').textContent = data.minimumClearanceM == null ? '—' : `${(data.minimumClearanceM * 1000).toFixed(1)} mm`;
    el('pair').textContent = data.closestPair?.join(' ↔ ') ?? '—';
    el('collision-sample').textContent = data.closestSampleIndex == null ? '—' : `${data.closestPhase ?? 'unknown'} / ${data.closestSampleIndex}`;
    el('collision-scene').textContent = data.sceneStatus;
  }
  const sync = new FeedbackSync({read: readStatus, intervalMs: feedbackIntervalMs, onSample: joints => state.acceptFeedback(joints), onStatus: s => {
    el('sync').textContent = s.text; el('sync').dataset.kind = s.kind;
  }});
  const inputs = Array.from({length:7}, (_, i) => [el(`number-${i}`), el(`range-${i}`)]);
  function displayInputs(force = false) {
    const values = [...state.manual.joints.map(radToDeg), state.manual.gripper * 100];
    inputs.forEach(([n, r], i) => {
      if (!force && n.getAttribute('aria-invalid') === 'true') return;
      if (force || document.activeElement !== n) n.value = values[i].toFixed(1);
      r.value = values[i]; n.removeAttribute('aria-invalid');
    });
  }
  function setMode(mode) {
    stopShowcase();
    graspPanel?.cancelMotion(); activeCompiledPlanId = null;
    if (mode === 'feedback') graspPanel?.release();
    armDrag?.release();
    state.setMode(mode); el('mode').value = mode;
    acceptedPose = null;
    el('manual').disabled = mode !== 'manual'; el('play').disabled = mode !== 'manual'; el('reset').disabled = mode !== 'manual';
    if (mode === 'feedback') { state.feedback = null; sync.start(); }
    else { sync.stop(); el('sync').textContent = sync.status().text; el('sync').dataset.kind = 'manual'; displayInputs(true); }
    showError(null);
  }
  function stopShowcase(completed = false) {
    if (!showcase.active && el('showcase-stop').disabled) return;
    showcase.stop(); physics?.endShowcase();
    el('showcase-pause').disabled = true; el('showcase-pause').textContent = '暂停展示';
    el('showcase-stop').disabled = true; el('showcase-play').textContent = '播放展示动画';
    el('showcase-overlay').hidden = true;
    el('showcase-state').textContent = completed ? '展示完成 · 可以继续拖动机械臂和物块' : '展示已停止 · 可以继续自由操作';
    el('free-status').textContent = '网页自由演示 · 拖动连杆调整关节，Shift 可精细调整；滑块可控制关节与夹爪动画。';
    displayInputs(true);
  }
  function startShowcase() {
    if (!loader.current) throw Error('请等待机械臂模型加载完成');
    if (!loader.current.rig.action) throw Error('当前模型没有夹爪动画，请恢复内置模型');
    graspPanel?.release(); activeCompiledPlanId = null; setMode('manual'); renderCollisionPanel(null); armDrag.release();
    physics.focus(); physics.beginShowcase();
    showcase.start(state.manual, performance.now(), {loop: el('showcase-loop').checked});
    el('showcase-pause').disabled = false; el('showcase-stop').disabled = false;
    el('showcase-play').textContent = '重新播放展示'; el('showcase-overlay').hidden = false;
    el('free-status').textContent = '展示动画播放中 · 可旋转或缩放视角；停止展示后可拖动连杆和物体。';
    el('plan-status').textContent = '正在播放网页展示动画。';
    showError(null);
  }
  try {
    scene = new RobotScene(el('main'), el('camera'), now => {
      scene.motionBusy = state.player.running || showcase.active || Boolean(graspPanel?.planning()) || state.mode !== 'manual';
      const presentation = showcase.tick(now);
      if (presentation) {
        for (const cue of presentation.cues) physics?.showcaseCue(cue);
        state.setManual(presentation.pose);
        const label = `${presentation.paused ? '已暂停' : '第 ' + presentation.cycle + ' 轮'} · ${presentation.label}`;
        if (el('showcase-phase').textContent !== label) el('showcase-phase').textContent = label;
        el('showcase-progress').value = presentation.progress;
        const status = `${label} · ${Math.round(presentation.progress * 100)}%`;
        if (el('showcase-state').textContent !== status) el('showcase-state').textContent = status;
        if (presentation.completed) stopShowcase(true);
      }
      const wasPlaying = state.player.running;
      let p = state.current(now), blockedMotion = false;
      if (solids && p && state.mode === 'manual') {
        const wanted = graspPanel.collisionPose(p), from = graspPanel.collisionPose(acceptedPose ?? p);
        const changed = wanted.gripper !== from.gripper || wanted.joints.some((q, i) => q !== from.joints[i]);
        if (changed) {
          const result = solids.sweep(from, wanted, candidate => graspPanel.heldAt(candidate));
          if (result.blocked) {
            blockedMotion = true; stopShowcase(); state.player.stop();
            const interrupted = activeCompiledPlanId; activeCompiledPlanId = null;
            p = {...result.pose, gripper: physics.world.heldId ? p.gripper : result.pose.gripper}; state.setManual(p);
            graspPanel.blocked(result.reason); el('solid-status').textContent = `实体边界已拦截：${result.reason} · 向反方向移动可退出`;
            el('solid-status').dataset.blocked = 'true'; displayInputs(true);
            if (interrupted) el('plan-status').textContent = `统一计划 ${interrupted} 的虚拟播放因实体碰撞中止，未标记预演完成。`;
          } else {
            graspPanel.clearBlocked();
            el('solid-status').textContent = '实体碰撞已启用 · 连杆 / 夹爪 / 物块 / 桌面 / 地面 · 死区自动停住';
            el('solid-status').dataset.blocked = 'false';
          }
        }
        acceptedPose = structuredClone(p);
      } else if (p) acceptedPose = structuredClone(p);
      if (loader?.current && p) loader.current.rig.apply(p);
      const displayed = graspPanel?.tick(now, p);
      if (loader?.current && displayed && displayed !== p) loader.current.rig.apply(displayed);
      if (solids) {
        physics.world.robotBoxes = solids.boxes();
        solidView.update([...physics.world.robotBoxes, ...physics.world.bodies.map(bodyBox),
          solidBox(physics.world.environment.table.position, physics.world.environment.table.size),
          ...physics.world.environment.obstacles.map(o => solidBox(o.position, o.size))], graspPanel.showVolumes());
      } else solidView?.update([], false);
      physics?.tick(now);
      if (loader?.current) {
        const sampleIndex = state.player.activeMeta?.sampleIndex ?? collisionPinnedSampleIndex;
        scene.updateCollisionVisualization(loader.current.rig, sampleIndex, collisionPinnedSampleIndex != null);
      }
      // Never show a retained manual pose as if it had come from feedback.
      scene.model.visible = Boolean(p);
      el('manual').disabled = state.mode !== 'manual' || state.player.running || showcase.active || graspPanel?.planning();
      el('play').disabled = state.mode !== 'manual' || showcase.active || graspPanel?.planning();
      el('reset').disabled = state.mode !== 'manual' || showcase.active;
      el('play-state').textContent = showcase.active ? (showcase.paused ? '展示已暂停' : '展示动画播放中') : state.player.running ? '预览播放中' : '已停止';
      if (wasPlaying && !state.player.running && !blockedMotion) {
        displayInputs();
        if (activeCompiledPlanId) {
          const completedPlanId = activeCompiledPlanId; activeCompiledPlanId = null;
          const collisionText = activeCollisionPreview?.status === 'collision' ? '检测到近似相交，执行已拒绝' : '近似未相交，仍需复核';
          el('plan-status').textContent = `统一计划 ${completedPlanId} 已完成虚拟播放；${collisionText}。`;
          window.dispatchEvent(new CustomEvent('lebai:compiled-plan-preview-complete', {detail: {planId: completedPlanId}}));
        }
      }
      if (now - lastReadoutMs > 100) {
        lastReadoutMs = now;
        cameraCaption();
        if (state.mode === 'manual' && (state.player.running || showcase.active)) displayInputs();
        el('readout').textContent = p ? `姿态来源：${state.mode === 'manual' ? '手动/轨迹预览' : '只读接口'}\n关节(rad)：${p.joints.map(q => q.toFixed(4)).join(', ')}\n夹爪：${state.mode === 'feedback' ? '无反馈，显示动画零位' : physics?.world.heldId ? `夹持物块 ${physics.world.heldId} · 按物块宽度停住` : (p.gripper*100).toFixed(1)+'% 动画'}\n模型映射：Joint1/2 同向，Joint5 反向；Joint3/4/6 待核对` : '暂无反馈姿态，机械臂暂不显示';
      }
    });
    loader = new ModelLoader(scene.model); scene.configure(config);
    physics = mountPhysicsPanel(el('physics-host'), scene, {signal: lifetime.signal});
    solidView = new SolidVolumeView(scene.base);
    graspPanel = mountGraspPanel(el('grasp-host'), scene, physics, {
      signal: lifetime.signal,
      getRig: () => externalModel ? null : loader.current?.rig,
      getPose: () => state.manual,
      getSolids: () => solids,
      moving: () => state.player.running,
      interactive: () => state.mode === 'manual' && !showcase.active && !activeCompiledPlanId && !closed,
      prepare: () => {
        if (!loader.current || externalModel) throw Error('请先恢复内置机械臂模型');
        graspPanel?.cancelMotion();
        activeCompiledPlanId = null; stopShowcase();
        if (state.mode !== 'manual') setMode('manual');
        state.player.stop(); armDrag?.release(); renderCollisionPanel(null); physics.ensureRunning();
        el('plan-status').textContent = '网页夹取控制 · 动作仅作用于虚拟机械臂和物块。';
      },
      applyPose: (p, {reset = false} = {}) => {
        if (reset && solids) {
          const reason = solids.check(p); if (reason) throw Error(`复位位置被占用：${reason}`);
          acceptedPose = structuredClone(p);
          el('solid-status').dataset.blocked = 'false';
          el('solid-status').textContent = '实体碰撞已启用 · 连杆 / 夹爪 / 物块 / 桌面 / 地面 · 死区自动停住';
        }
        state.setManual(p); displayInputs(true);
      },
      play: segments => { state.play(segments, performance.now()); showError(null); },
      stop: () => { state.player.stop(); displayInputs(true); },
    });
    armDrag = mountArmDrag(scene, {
      getRig: () => loader.current?.rig,
      enabled: () => state.mode === 'manual' && !state.player.running && !showcase.active && !graspPanel?.planning() && !scene.pointerMode && !closed,
      getPose: () => state.manual,
      setPose: next => {
        activeCompiledPlanId = null; collisionPinnedSampleIndex = null;
        state.setManual(next); displayInputs(true); showError(null);
      },
      onStatus: text => { el('free-status').textContent = text; },
    });
  } catch (e) { armDrag?.dispose(); graspPanel?.dispose(); physics?.dispose(); solidView?.dispose(); scene?.dispose(); sync.stop(); lifetime.abort(); throw e; }

  async function loadModel(source, isExternal) {
    graspPanel.release();
    stopShowcase();
    armDrag.release();
    state.player.stop(); const request = ++modelRequest;
    scene.gripperVirtual.unbind();
    el('model-status').textContent = '模型加载中…'; showError(null);
    try {
      if (!await loader.load(source) || closed || request !== modelRequest) return;
      externalModel = isExternal; scene.gripperVirtual.bind(isExternal ? null : loader.current.rig); scene.frameModel();
      solids = isExternal ? null : new RobotSolids(loader.current.rig, scene.base, physics.world);
      if (solids) {
        if (solids.check(state.manual)) state.setManual({joints: [...UPRIGHT_REFERENCE_JOINTS_RAD], gripper: 1});
        acceptedPose = structuredClone(state.manual);
        el('solid-status').textContent = '实体碰撞已启用 · 连杆 / 夹爪 / 物块 / 桌面 / 地面 · 死区自动停住';
      } else { acceptedPose = null; physics.world.robotBoxes = []; el('solid-status').textContent = '外部模型未建立实体约束，请恢复内置模型'; }
      if (request === 1 && ['physicsDemo', 'physics'].some(key => new URLSearchParams(location.search).get(key) === '1')) physics.focus();
      if (isExternal) renderCollisionPanel(null);
      el('model-status').textContent = `已加载${isExternal ? '外部 GLB（映射待核对）' : 'Lebai_LM3'} · Joint1～Joint6 已检查 · ${loader.current.rig.action ? '6 条夹爪专用轨道' : '无兼容夹爪动画'}。仅支持自包含 GLB 2.0；不支持外部 glTF 资源及压缩扩展。`;
      inputs[6].forEach(n => { n.disabled = !loader.current.rig.action; });
      el('showcase-play').disabled = !loader.current.rig.action;
      if (request === 1 && new URLSearchParams(location.search).get('grasp') === '1') graspPanel.resetScene();
      else if (request === 1 && new URLSearchParams(location.search).get('showcase') === '1') startShowcase();
      if (!isExternal && autoCollisionDemo && !autoCollisionDemoLoaded) {
        autoCollisionDemoLoaded = true; showSyntheticCollisionDemo();
      }
    } catch (e) {
      if (closed || request !== modelRequest) return;
      scene.gripperVirtual.bind(externalModel ? null : loader.current?.rig);
      el('model-status').textContent = loader.current ? '加载失败，保留原模型' : '模型未加载，请恢复内置模型或选择兼容 GLB'; showError(e);
    }
  }
  function cameraCaption() {
    const gripper = scene.cameraMode === 'gripper', mounted = scene.gripperVirtual.rig && scene.model.visible;
    const text = gripper ? (mounted ? `夹爪摄像头 · ${state.mode === 'feedback' ? '随有效反馈姿态更新' : '随虚拟夹爪移动与转动'} · 640×360 / 10 fps` : '夹爪相机暂无有效姿态 · 当前显示固定相机')
      : `固定摄像头 · ${config.camera.mode === 'calibrated' ? '已加载标定参数，物理对齐待现场验证' : '示意视角'} · 640×360 / 10 fps`;
    if (el('camera-caption').textContent !== text) el('camera-caption').textContent = text;
    el('camera-dialog-title').textContent = gripper ? '夹爪摄像头 · 随动视角' : '固定摄像头 · 场景视角';
  }
  function displayCameraControls() {
    el('camera-view').value = scene.cameraMode;
    const fov = radToDeg(scene.gripperVirtual.config.fovRad); el('camera-fov').value = fov; el('camera-fov-value').textContent = `${Math.round(fov)}°`;
    el('camera-mount').checked = scene.gripperVirtual.config.showMount; cameraCaption();
  }
  on(el('camera-view'), 'change', () => { scene.cameraMode = el('camera-view').value; scene.lastCameraMs = -Infinity; cameraCaption(); });
  on(el('camera-fov'), 'input', () => {
    scene.gripperVirtual.configure({...scene.gripperVirtual.config, fovRad: degToRad(number(el('camera-fov').value, '视场角(°)', 30, 110))});
    el('camera-fov-value').textContent = `${el('camera-fov').value}°`; scene.lastCameraMs = -Infinity;
  });
  on(el('camera-mount'), 'change', () => scene.gripperVirtual.configure({...scene.gripperVirtual.config, showMount: el('camera-mount').checked}));
  on(el('camera-zoom'), 'click', () => { el('camera-dialog').showModal(); el('camera-large').append(scene.cameraCanvas); cameraCaption(); });
  on(el('camera-close'), 'click', () => el('camera-dialog').close());
  on(el('camera-dialog'), 'close', () => el('camera').append(scene.cameraCanvas));
  inputs.forEach(([n, r], i) => {
    const change = value => {
      try {
        const v = number(value, i < 6 ? `关节${i+1}(°)` : '夹爪(%)', i < 6 ? -180 : 0, i < 6 ? 180 : 100);
        const next = structuredClone(state.manual);
        if (i < 6) next.joints[i] = degToRad(v); else next.gripper = v / 100;
        // Preserve the focused text draft (e.g. "0." while typing "0.5").
        collisionPinnedSampleIndex = null; state.setManual(next); n.removeAttribute('aria-invalid'); displayInputs();
        if (!inputs.some(([input]) => input.getAttribute('aria-invalid') === 'true')) showError(null);
      } catch (e) { n.setAttribute('aria-invalid', 'true'); showError(e); }
    };
    on(n, 'input', () => change(n.value)); on(r, 'input', () => change(r.value));
    on(n, 'blur', () => displayInputs());
  });
  on(el('mode'), 'change', () => setMode(el('mode').value));
  for (const [id, event] of [['apiBaseInput', 'change'], ['controlTokenInput', 'input']]) {
    const node = document.getElementById(id);
    if (node) on(node, event, () => { if (state.mode === 'feedback') setMode('feedback'); });
  }
  on(el('view-reset'), 'click', () => scene.resetView());
  on(el('showcase-play'), 'click', startShowcase);
  on(el('showcase-pause'), 'click', () => {
    showcase.setPaused(!showcase.paused); physics.pauseShowcase(showcase.paused);
    el('showcase-pause').textContent = showcase.paused ? '继续展示' : '暂停展示';
  });
  on(el('showcase-stop'), 'click', () => stopShowcase());
  on(el('showcase-loop'), 'change', () => { showcase.loop = el('showcase-loop').checked; });
  document.addEventListener('visibilitychange', () => {
    if (document.hidden && showcase.active) {
      showcase.setPaused(true); physics.pauseShowcase(true); el('showcase-pause').textContent = '继续展示';
    }
  }, {signal: lifetime.signal});
  on(el('free-mode'), 'click', () => {
    activeCompiledPlanId = null; setMode('manual'); renderCollisionPanel(null); displayInputs(true); physics.focus();
    el('free-status').textContent = '网页自由演示 · 拖动连杆或物体；动作仅作用于浏览器模型。';
    el('plan-status').textContent = '网页自由演示未绑定抓取执行计划。';
  });
  on(el('side-view'), 'click', () => scene.sideView());
  on(el('top-view'), 'click', () => scene.topView());
  on(el('frame'), 'click', () => scene.frameModel());
  on(el('builtin'), 'click', () => loadModel(DEFAULT_MODEL, false));
  on(el('load-camera'), 'click', async () => {
    if (typeof readCalibration !== 'function') throw Error('当前页面没有提供标定读取接口');
    const payload = await readCalibration(lifetime.signal);
    const camera = calibratedCameraConfigFromApi(payload, [640, 360]);
    apply({...snapshot(), camera, cameraView: 'fixed'});
    el('config-status').textContent = '已加载相机内外参；物理像素对齐仍待验证';
  });
  window.addEventListener('lebai:compiled-plan', event => {
    try {
      if (!loader.current) throw Error('机械臂模型尚未加载，不能确认统一计划预演完成');
      if (externalModel) throw Error('碰撞报告仅绑定内置 Lebai_LM3，请先恢复内置模型');
      const packet = event.detail?.plan ? event.detail : {plan: event.detail, trajectoryValidation: null};
      const preview = compiledPlanToTrajectory(packet.plan, packet.trajectoryValidation);
      graspPanel.release();
      setMode('manual');
      state.setManual({joints: preview.startJoints, gripper: 1});
      activeCompiledPlanId = preview.planId;
      renderCollisionPanel(preview.collisionVisualization);
      state.play(preview.segments, performance.now());
      const clearance = preview.collisionVisualization?.minimumClearanceM;
      el('plan-status').textContent = `正在播放 ${preview.planId}；碰撞=${preview.collisionStatus}${clearance == null ? '' : `，最小间隙=${(clearance * 1000).toFixed(1)}mm`}，相机对齐=${preview.cameraAlignment}。`;
      showError(null);
    } catch (error) {
      activeCompiledPlanId = null;
      el('plan-status').textContent = `统一计划无法预演：${error.message}`;
      showError(error);
    }
  }, {signal: lifetime.signal});
  on(el('focus-collision'), 'click', () => {
    if (!activeCollisionPreview?.focusJoints) throw Error('当前碰撞报告没有可定位的采样姿态');
    stopShowcase();
    activeCompiledPlanId = null; state.player.stop();
    state.setManual({joints: activeCollisionPreview.focusJoints, gripper: state.manual.gripper});
    collisionPinnedSampleIndex = activeCollisionPreview.closestSampleIndex;
    displayInputs(true);
    scene.updateCollisionVisualization(loader.current?.rig, collisionPinnedSampleIndex, true);
    el('plan-status').textContent = `已定位 ${activeCollisionPreview.closestStepId ?? 'plan_start'} 的最小间隙姿态；继续完整预演需重新执行安全探测。`;
  });
  function showSyntheticCollisionDemo() {
    if (!loader.current || externalModel) throw Error('请先加载内置 Lebai_LM3 模型');
    const demo = syntheticCollisionDemo(); setMode('manual');
    state.setManual({joints: demo.focusJoints, gripper: 0});
    renderCollisionPanel(demo); collisionPinnedSampleIndex = demo.closestSampleIndex;
    displayInputs(true); scene.updateCollisionVisualization(loader.current.rig, collisionPinnedSampleIndex, true);
    el('plan-status').textContent = '正在显示离线合成碰撞示例：link_3 与 synthetic_fixture 近似相交；该示例没有执行锁，不会调用真机。';
  }
  on(el('demo-collision'), 'click', showSyntheticCollisionDemo);
  on(el('toggle-collision'), 'click', () => {
    collisionVisible = !collisionVisible; scene.setCollisionVisible(collisionVisible);
    el('toggle-collision').textContent = collisionVisible ? '隐藏碰撞层' : '显示碰撞层';
  });
  on(el('file'), 'change', () => { const file = el('file').files[0]; el('file').value = ''; if (file) return loadModel(file, true); });
  on(el('play'), 'click', () => {
    if (inputs.some(([n]) => n.getAttribute('aria-invalid') === 'true')) throw Error('请先修正关节输入');
    const segments = parseTrajectory(el('trajectory').value, el('duration').value);
    if (segments.some(s => s.gripper !== undefined) && !loader.current?.rig.action) throw Error('当前模型没有兼容夹爪动画');
    graspPanel.cancelMotion(); activeCompiledPlanId = null; collisionPinnedSampleIndex = null; state.play(segments, performance.now()); showError(null);
  });
  on(el('stop'), 'click', () => { graspPanel.cancelMotion(); stopShowcase(); activeCompiledPlanId = null; collisionPinnedSampleIndex = null; state.player.stop(); displayInputs(); });
  on(el('reset'), 'click', () => { activeCompiledPlanId = null; collisionPinnedSampleIndex = null; graspPanel.resetScene(); displayInputs(true); showError(null); });
  const snapshot = () => {
    if (externalModel) throw Error('保存场景前请恢复内置模型；外部模型文件不持久化');
    if (physics.world.heldId || state.player.running || graspPanel.planning() || showcase.active) throw Error('请先松开夹持物块并停止移动，再保存或应用场景');
    return validateConfig({...config, preview: state.manual, objects: physics.world.objectDefinitions(true), cameraView: scene.cameraMode, gripperCamera: scene.gripperVirtual.config});
  };
  function apply(c) {
    const valid = validateConfig(c); // Validate everything before changing any state.
    if (externalModel || !solids) throw Error('应用场景前请恢复内置模型');
    if (physics.world.heldId || graspPanel.planning() || state.player.running || showcase.active) throw Error('请先松开夹持物块并停止移动，再应用场景');
    const trial = new LivePhysics(physics.world.config), position = valid.table.positionBaseM, size = valid.table.sizeM;
    trial.setEnvironment({groundZ: scene.groundZ, table: {position, size}, obstacles: [{position: valid.target.positionBaseM, size: valid.target.sizeM, label: '橙色固定物块'}],
      initialPositions: [.22, .30].map((x, i) => [position[0] + size[0] * x, position[1] + (i ? .16 : -.16) * size[1], position[2] + size[2] / 2 + .03])});
    const oldMount = scene.gripperVirtual.config;
    try {
      scene.gripperVirtual.configure(valid.gripperCamera ?? DEFAULT_GRIPPER_CAMERA); loader.current.rig.apply(valid.preview);
      const trialSolids = new RobotSolids(loader.current.rig, scene.base, trial); trial.robotBoxes = trialSolids.boxes(); trial.replaceObjects(valid.objects ?? trial.defaultObjects());
      const reason = trialSolids.check(valid.preview); if (reason) throw Error(`配置姿态处于碰撞区域：${reason}`);
    } finally { scene.gripperVirtual.configure(oldMount); loader.current.rig.apply(graspPanel.collisionPose(state.current(performance.now()) ?? state.manual)); }
    graspPanel.release(); activeCompiledPlanId = null; collisionPinnedSampleIndex = null;
    setMode('manual'); state.setManual(valid.preview); config = valid; scene.configure(config); renderCollisionPanel(null);
    physics.world.replaceObjects(trial.objectDefinitions(true), {check: false}); loader.current.rig.apply(valid.preview); physics.refresh(); graspPanel.refreshScene();
    acceptedPose = structuredClone(valid.preview);
    el('object-caption').hidden = !valid.objects?.length;
    el('object-caption').textContent = `当前虚拟物体：${(valid.objects ?? []).map(o => o.label).join('、')}`;
    displayInputs(true); el('config').value = JSON.stringify(config, null, 2); showError(null);
    displayCameraControls();
    window.dispatchEvent(new CustomEvent('lebai:virtual-scene-changed'));
    el('config-status').textContent = '配置已应用，已切换到手动预览';
  }
  on(el('apply'), 'click', () => apply(JSON.parse(el('config').value)));
  on(el('save'), 'click', () => { const c = snapshot(); saveConfig(c); el('config').value = JSON.stringify(c,null,2); el('config-status').textContent = '当前场景已保存到此浏览器'; showError(null); });
  on(el('restore'), 'click', () => apply(loadConfig()));
  const urls = new Set(), urlTimers = new Set();
  on(el('export'), 'click', () => {
    const url = URL.createObjectURL(new Blob([JSON.stringify(snapshot(), null, 2)], {type: 'application/json'})); urls.add(url);
    const a = document.createElement('a'); a.href = url; a.download = 'lebai-scene-v1.json'; a.click();
    const timer = setTimeout(() => { URL.revokeObjectURL(url); urls.delete(url); urlTimers.delete(timer); }, 1000); urlTimers.add(timer);
  });
  let importRevision = 0;
  on(el('import'), 'change', async () => {
    const revision = ++importRevision, file = el('import').files[0]; el('import').value = '';
    if (!file) return; if (file.size > 100000) throw Error('场景 JSON 不可超过 100 KB');
    const text = await file.text(); if (!closed && revision === importRevision) apply(JSON.parse(text));
  });
  realObjects = mountRealObjectsPanel(el('real-host'), {signal: lifetime.signal, captureCurrent: captureObjects, readLatest: readCameraObjects, getInitialCommand: getCameraCommand,
    getTableTop: () => config.table.positionBaseM[2] + config.table.sizeM[2] / 2,
    applyObjects: (objects, {tableTopM}) => {
      if (state.mode !== 'manual') setMode('manual');
      const current = snapshot(), table = structuredClone(current.table); table.positionBaseM[2] = tableTopM - table.sizeM[2] / 2;
      for (const axis of [0, 1]) {
        const half = Math.max(table.sizeM[axis] / 2, ...objects.map(o => Math.abs(o.position[axis] - table.positionBaseM[axis]) + o.sizeM[axis] / 2 + .05));
        table.sizeM[axis] = Math.min(5, half * 2);
      }
      apply({...current, table, objects}); physics.releaseDrag(); armDrag.release(); physics.world.setRunning(false); physics.refresh(); physics.focusObjects();
    },
  });
  el('config').value = JSON.stringify(config, null, 2);
  renderCollisionPanel(null);
  displayCameraControls();
  const dispose = () => {
    if (closed) return; closed = true; importRevision++; modelRequest++; lifetime.abort(); sync.stop(); state.player.stop();
    showcase.stop();
    realObjects.dispose(); armDrag.dispose(); graspPanel.dispose(); physics.dispose(); solidView.dispose(); scene.gripperVirtual.unbind(); loader.dispose(); scene.dispose(); urls.forEach(URL.revokeObjectURL); urlTimers.forEach(clearTimeout);
  };
  window.addEventListener('pagehide', e => {
    dispose();
    // Only a cached document needs this one-shot restore listener.
    if (e.persisted) window.addEventListener('pageshow', () => location.reload(), {once: true});
  }, {signal: lifetime.signal});
  void loadModel(DEFAULT_MODEL, false);
  return {dispose};
}
