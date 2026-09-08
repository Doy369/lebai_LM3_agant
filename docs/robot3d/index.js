import { PoseState } from './state.js';
import { FeedbackSync } from './sync.js';
import { number, degToRad, radToDeg, parseTrajectory } from './validation.js';
import { ModelLoader } from './model-loader.js';
import { RobotScene } from './scene.js';
import { DEFAULT_CONFIG, validateConfig, saveConfig, loadConfig } from './config.js';

const DEFAULT_MODEL = new URL('./assets/Lebai_LM3.glb', import.meta.url);
export function mountRobot3D(root, {readStatus}) {
  const state = new PoseState(); let config = validateConfig(DEFAULT_CONFIG);
  let scene, loader, externalModel = false, closed = false, modelRequest = 0, lastReadoutMs = 0;
  const lifetime = new AbortController();
  root.innerHTML = `
    <div class="r3-head"><div><p class="panel-kicker">Virtual workspace / 只读姿态</p><h2>机械臂三维预览</h2></div>
      <label>姿态来源 <select id="r3-mode"><option value="manual">手动预览</option><option value="feedback">实时反馈</option></select></label></div>
    <p class="r3-notice">关节轴、方向、零位、Base 与模型的对应关系待实测校准。±180°仅为预览输入范围；夹爪百分比仅表示动画进度，不是开口距离。预演不验证碰撞、接触或抓取成功。</p>
    <div id="r3-sync" class="r3-status" role="status">手动预览 · 不读取实时姿态</div>
    <div class="r3-views"><div><div id="r3-main" class="r3-main" aria-label="自由观察三维画面"></div>
      <p class="r3-caption">拖动旋转 · 滚轮缩放 · 右键平移；居中仅调整观察相机，保留模型导入变换。坐标轴：X 红 / Y 绿 / Z 蓝。</p></div>
      <div><div id="r3-camera" class="r3-camera" aria-label="虚拟摄像头画面"></div><p class="r3-caption">虚拟摄像头 · 示意视角 · 640×360 / 10 fps<br>独立于自由观察相机。尚未启用真实相机对齐。</p>
      <pre id="r3-readout" class="r3-readout">等待模型</pre></div></div>
    <div class="r3-toolbar"><button id="r3-view-reset">视角复位</button><button id="r3-frame">居中观察</button>
      <button id="r3-builtin">恢复内置模型</button><label>替换预览模型 <input id="r3-file" type="file" accept=".glb"></label></div>
    <p id="r3-model-status" class="r3-caption" role="status">模型加载中…</p>
    <fieldset id="r3-manual" class="r3-joints">${Array.from({length:7}, (_, i) => {
      const label = i < 6 ? `关节${i + 1} (°)` : '夹爪动画 (%)';
      return `<div class="r3-joint"><label for="r3-number-${i}">${label}</label><input id="r3-number-${i}" type="text" inputmode="decimal" value="0"><input id="r3-range-${i}" aria-label="${label}滑块" type="range" min="${i < 6 ? -180 : 0}" max="${i < 6 ? 180 : 100}" step="1" value="0"></div>`;
    }).join('')}</fieldset>
    <div class="r3-trajectory"><label>预览动作序列（角度 / 百分比）<textarea id="r3-trajectory" class="r3-panel-input" rows="2">关节1:30,关节2:-30,夹爪:100;关节1:0,关节2:0,夹爪:0;</textarea></label>
      <label>每段时长(ms)<input id="r3-duration" class="r3-panel-input" type="text" inputmode="decimal" value="2000"></label></div>
    <div class="r3-toolbar"><button id="r3-play">播放预览</button><button id="r3-stop">停止预览</button><button id="r3-reset">重置预览</button><span id="r3-play-state">已停止</span></div>
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
  const sync = new FeedbackSync({read: readStatus, onSample: joints => state.acceptFeedback(joints), onStatus: s => {
    el('sync').textContent = s.text; el('sync').dataset.kind = s.kind;
  }});
  const inputs = Array.from({length:7}, (_, i) => [el(`number-${i}`), el(`range-${i}`)]);
  function displayInputs(force = false) {
    const values = [...state.manual.joints.map(radToDeg), state.manual.gripper * 100];
    inputs.forEach(([n, r], i) => {
      if (!force && n.getAttribute('aria-invalid') === 'true') return;
      if (document.activeElement !== n) n.value = values[i].toFixed(1);
      r.value = values[i]; n.removeAttribute('aria-invalid');
    });
  }
  function setMode(mode) {
    state.setMode(mode); el('mode').value = mode;
    el('manual').disabled = mode !== 'manual'; el('play').disabled = mode !== 'manual'; el('reset').disabled = mode !== 'manual';
    if (mode === 'feedback') { state.feedback = null; sync.start(); }
    else { sync.stop(); el('sync').textContent = sync.status().text; el('sync').dataset.kind = 'manual'; displayInputs(true); }
    showError(null);
  }
  try {
    scene = new RobotScene(el('main'), el('camera'), now => {
      const wasPlaying = state.player.running;
      const p = state.current(now);
      if (loader?.current && p) loader.current.rig.apply(p);
      // Never show a retained manual pose as if it had come from feedback.
      scene.model.visible = Boolean(p);
      el('manual').disabled = state.mode !== 'manual' || state.player.running;
      el('play-state').textContent = state.player.running ? '预览播放中' : '已停止';
      if (wasPlaying && !state.player.running) displayInputs();
      if (now - lastReadoutMs > 100) {
        lastReadoutMs = now;
        if (state.mode === 'manual' && state.player.running) displayInputs();
        el('readout').textContent = p ? `姿态来源：${state.mode === 'manual' ? '手动/轨迹预览' : '只读接口'}\n关节(rad)：${p.joints.map(q => q.toFixed(4)).join(', ')}\n夹爪：${state.mode === 'feedback' ? '无反馈，显示动画零位' : (p.gripper*100).toFixed(1)+'% 动画'}\n模型映射：待校准` : '暂无反馈姿态，机械臂暂不显示';
      }
    });
    loader = new ModelLoader(scene.model); scene.configure(config);
  } catch (e) { scene?.dispose(); sync.stop(); lifetime.abort(); throw e; }

  async function loadModel(source, isExternal) {
    state.player.stop(); const request = ++modelRequest;
    el('model-status').textContent = '模型加载中…'; showError(null);
    try {
      if (!await loader.load(source) || closed || request !== modelRequest) return;
      externalModel = isExternal; scene.frameModel();
      el('model-status').textContent = `已加载${isExternal ? '外部 GLB（映射待核对）' : 'Lebai_LM3'} · Joint1～Joint6 已检查 · ${loader.current.rig.action ? '6 条夹爪专用轨道' : '无兼容夹爪动画'}。仅支持自包含 GLB 2.0；不支持外部 glTF 资源及压缩扩展。`;
      inputs[6].forEach(n => { n.disabled = !loader.current.rig.action; });
    } catch (e) {
      if (closed || request !== modelRequest) return;
      el('model-status').textContent = loader.current ? '加载失败，保留原模型' : '模型未加载，请恢复内置模型或选择兼容 GLB'; showError(e);
    }
  }
  inputs.forEach(([n, r], i) => {
    const change = value => {
      try {
        const v = number(value, i < 6 ? `关节${i+1}(°)` : '夹爪(%)', i < 6 ? -180 : 0, i < 6 ? 180 : 100);
        const next = structuredClone(state.manual);
        if (i < 6) next.joints[i] = degToRad(v); else next.gripper = v / 100;
        // Preserve the focused text draft (e.g. "0." while typing "0.5").
        state.setManual(next); n.removeAttribute('aria-invalid'); displayInputs();
        if (!inputs.some(([input]) => input.getAttribute('aria-invalid') === 'true')) showError(null);
      } catch (e) { n.setAttribute('aria-invalid', 'true'); showError(e); }
    };
    on(n, 'input', () => change(n.value)); on(r, 'input', () => change(r.value));
  });
  on(el('mode'), 'change', () => setMode(el('mode').value));
  for (const [id, event] of [['apiBaseInput', 'change'], ['controlTokenInput', 'input']]) {
    const node = document.getElementById(id);
    if (node) on(node, event, () => { if (state.mode === 'feedback') setMode('feedback'); });
  }
  on(el('view-reset'), 'click', () => scene.resetView());
  on(el('frame'), 'click', () => scene.frameModel());
  on(el('builtin'), 'click', () => loadModel(DEFAULT_MODEL, false));
  on(el('file'), 'change', () => { const file = el('file').files[0]; el('file').value = ''; if (file) return loadModel(file, true); });
  on(el('play'), 'click', () => {
    if (inputs.some(([n]) => n.getAttribute('aria-invalid') === 'true')) throw Error('请先修正关节输入');
    const segments = parseTrajectory(el('trajectory').value, el('duration').value);
    if (segments.some(s => s.gripper !== undefined) && !loader.current?.rig.action) throw Error('当前模型没有兼容夹爪动画');
    state.play(segments, performance.now()); showError(null);
  });
  on(el('stop'), 'click', () => { state.player.stop(); displayInputs(); });
  on(el('reset'), 'click', () => { state.reset(); displayInputs(true); showError(null); });
  const snapshot = () => {
    if (externalModel) throw Error('保存场景前请恢复内置模型；外部模型文件不持久化');
    return validateConfig({...config, preview: state.manual});
  };
  function apply(c) {
    const valid = validateConfig(c); // Validate everything before changing any state.
    if (externalModel) throw Error('应用场景前请恢复内置模型');
    setMode('manual'); state.setManual(valid.preview); config = valid; scene.configure(config);
    displayInputs(true); el('config').value = JSON.stringify(config, null, 2); showError(null);
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
  el('config').value = JSON.stringify(config, null, 2);
  const dispose = () => {
    if (closed) return; closed = true; importRevision++; modelRequest++; lifetime.abort(); sync.stop(); state.player.stop();
    loader.dispose(); scene.dispose(); urls.forEach(URL.revokeObjectURL); urlTimers.forEach(clearTimeout);
  };
  window.addEventListener('pagehide', e => {
    dispose();
    // Only a cached document needs this one-shot restore listener.
    if (e.persisted) window.addEventListener('pageshow', () => location.reload(), {once: true});
  }, {signal: lifetime.signal});
  void loadModel(DEFAULT_MODEL, false);
  return {dispose};
}
