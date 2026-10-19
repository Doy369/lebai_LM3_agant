import { parseRealObjects, realObjectsToBodies } from './real-objects.js?v=20261013-image-objects';
import { number } from './validation.js';

export function mountRealObjectsPanel(host, {signal, getTableTop, applyObjects, captureCurrent, readLatest, getInitialCommand}) {
  host.innerHTML = `<details class="standalone-section r3-real-section" open><summary>真实物体 → 虚拟场景</summary>
    <p class="r3-caption">拍摄一张图片，本地识别物体并显示到虚拟场景。位置来自 RGB-D 和现有标定，形状用长方体近似，高度默认 4 cm。</p>
    <div class="r3-toolbar"><label>识别目标<input id="r3-real-command" maxlength="500" placeholder="例如：识别绿色物块"></label><button id="r3-real-capture">识别当前画面物体</button><button id="r3-real-latest">读取最近识别结果</button><label>导入识别 JSON<input id="r3-real-file" type="file" accept=".json"></label><button id="r3-real-sample">载入测试数据</button></div>
    <fieldset id="r3-real-inputs" class="r3-object-inputs">
      <label>默认长度 X(cm)<input id="r3-real-x" type="number" min="2" max="20" step=".1" value="4"></label>
      <label>默认宽度 Y(cm)<input id="r3-real-y" type="number" min="2" max="20" step=".1" value="4"></label>
      <label>默认高度 Z(cm)<input id="r3-real-z" type="number" min="2" max="20" step=".1" value="4"></label>
      <label>桌面高度 Z(m)<input id="r3-real-top" type="number" min="-5" max="5" step=".001" value="0"></label>
      <label>物体高度<select id="r3-real-height"><option value="table">放在桌面上</option><option value="measured">测量高度（表面按顶面估计）</option></select></label>
      <label>默认质量(kg)<input id="r3-real-mass" type="number" min=".05" max="10" step=".05" value="1"></label>
    </fieldset>
    <div class="r3-toolbar"><label><input id="r3-real-auto" type="checkbox" checked>识别后自动显示到虚拟场景</label><button id="r3-real-apply" disabled>生成虚拟物块</button><span class="r3-caption">替换当前物块列表 · 仅操作网页模型</span></div>
    <p id="r3-real-status" class="r3-caption" role="status">尚未读取识别结果。没有设备时可载入测试数据验证流程。</p>
    <div id="r3-real-results" class="r3-real-results" hidden><figure id="r3-real-photo-wrap" hidden><img id="r3-real-photo" alt="实体相机识别照片"><figcaption>后端最近识别快照</figcaption></figure><div id="r3-real-info"></div></div>
    <p id="r3-real-error" class="r3-error" role="alert"></p>
  </details>`;
  const el = key => host.querySelector(`#r3-real-${key}`);
  let capture = null, revision = 0, closed = false;
  el('top').value = getTableTop();
  el('command').value = '全部可识别物体';
  const status = text => { el('status').textContent = text; };
  const listen = (key, event, callback) => el(key).addEventListener(event, async () => {
    try { el('error').textContent = ''; await callback(); }
    catch (error) { if (!closed) { el('error').textContent = error.message; status(capture ? '操作未完成，保留此前读取的结果与当前场景' : '尚未读取有效识别结果'); } }
  }, {signal});
  function display(value, cameraResult = null) {
    capture = parseRealObjects(value);
    if (cameraResult) { capture.source = 'real-camera'; capture.capturedAt = cameraResult.capturedAt ?? null; }
    el('apply').disabled = false; el('results').hidden = false;
    const source = {'real-camera': '实体相机结果', sample: '测试数据（非真实拍摄）', import: '导入的相机规划'}[capture.source];
    status(`${source} · ${capture.objects.length} 个物体${capture.capturedAt ? ' · ' + new Date(capture.capturedAt).toLocaleString() : ''} · 待生成`);
    const table = document.createElement('table'), head = document.createElement('tr');
    for (const text of ['物体', '测量点 X / Y / Z(m)', '尺寸来源']) { const th = document.createElement('th'); th.textContent = text; head.append(th); }
    table.append(head);
    for (const o of capture.objects) {
      const row = document.createElement('tr');
      for (const text of [o.label, o.positionBaseM.map(n => n.toFixed(4)).join(' / '), o.sizeM ? '识别数据提供' : '使用上方填写尺寸']) { const td = document.createElement('td'); td.textContent = text; row.append(td); }
      table.append(row);
    }
    const note = document.createElement('p'); note.className = 'r3-caption';
    note.textContent = '位置使用机械臂基座坐标。默认仅使用测量点 X/Y，物块底面贴合填写的桌面高度；尺寸、质量与未提供的朝向属于仿真假设。';
    el('info').replaceChildren(table, note);
    const url = cameraResult?.photoUrl ?? capture.annotatedUrl ?? capture.snapshotUrl;
    el('photo-wrap').hidden = !url;
    if (url) el('photo').src = url; else el('photo').removeAttribute('src');
  }
  listen('capture', 'click', async () => {
    if (typeof captureCurrent !== 'function') throw Error('当前页面没有相机识别接口');
    const command = el('command').value.trim(); if (!command) throw Error('请填写识别目标');
    const ticket = ++revision; el('capture').disabled = true; status('正在拍摄并识别物体…');
    try {
      const result = await captureCurrent(command, signal);
      if (!closed && ticket === revision) { display(result.plan, result); if (el('auto').checked) generate(); }
    } finally { if (!closed) el('capture').disabled = false; }
  });
  listen('latest', 'click', async () => {
    const ticket = ++revision, result = await readLatest?.();
    if (!result) throw Error('本次页面尚无有效识别结果，请先识别当前画面，或使用控制台的“抓取当前画面目标并规划”');
    if (!closed && ticket === revision) display(result.plan, result);
  });
  listen('file', 'change', async () => {
    const ticket = ++revision, file = el('file').files[0]; el('file').value = ''; if (!file) return;
    if (file.size > 100000) throw Error('识别 JSON 不可超过 100 KB');
    const text = await file.text(); if (!closed && ticket === revision) display(JSON.parse(text));
  });
  listen('sample', 'click', async () => {
    const ticket = ++revision, response = await fetch(new URL('./examples/real-objects-sample.json', import.meta.url), {signal});
    if (!response.ok) throw Error('测试数据读取失败'); const value = await response.json();
    if (!closed && ticket === revision) display(value);
  });
  function generate() {
    if (!capture) throw Error('请先读取识别结果');
    const tableTopM = number(el('top').value, '桌面高度(m)', -5, 5), heightMode = el('height').value;
    const bodies = realObjectsToBodies(capture, {sizeM: ['x', 'y', 'z'].map(key => number(el(key).value, '默认尺寸(cm)', 2, 20) / 100),
      tableTopM, heightMode, mass: number(el('mass').value, '默认质量(kg)', .05, 10)});
    applyObjects(bodies, {tableTopM});
    status(`已生成 ${bodies.length} 个${capture.source === 'sample' ? '测试' : '识别'}物块 · ${heightMode === 'table' ? '按桌面支撑放置' : '按测量高度估计'} · 物理已暂停，可选择目标试抓`);
  }
  listen('apply', 'click', generate);
  el('photo').addEventListener('error', () => { if (!closed) { el('photo-wrap').hidden = true; el('error').textContent = '识别照片读取失败，物体数据仍可使用'; } }, {signal});
  return {dispose() { closed = true; revision++; }};
}
