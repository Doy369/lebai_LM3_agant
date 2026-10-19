import * as T from './vendor/three.module.js';

export function pickArmJoint(camera, rig, rect, x, y) {
  if (!rig || rect.width <= 0 || rect.height <= 0) return null;
  camera.updateMatrixWorld(); rig.root.updateWorldMatrix(true, true);
  const ray = new T.Raycaster();
  ray.setFromCamera(new T.Vector2((x - rect.left) / rect.width * 2 - 1, 1 - (y - rect.top) / rect.height * 2), camera);
  for (const hit of ray.intersectObject(rig.root, true)) {
    let node = hit.object;
    while (node) {
      const index = rig.joints.findIndex(j => j.node === node);
      if (index >= 0) return index;
      if (node === rig.root) break;
      node = node.parent;
    }
  }
  return null;
}
export function dragJointAngle(start, dx, dy, fine = false) {
  return Math.max(-Math.PI, Math.min(Math.PI, start + (dx - dy) * (fine ? .002 : .008)));
}

// Local pose writer only: no controller, API, feedback or compiled-plan calls.
export function mountArmDrag(scene, {getRig, enabled, getPose, setPose, onStatus}) {
  const canvas = scene.renderer.domElement, lifetime = new AbortController();
  const options = {capture: true, signal: lifetime.signal};
  let held = null, previousControls = true;
  function release() {
    if (!held) return;
    const pointerId = held.pointerId; held = null;
    scene.controls.enabled = previousControls; canvas.style.cursor = '';
    if (canvas.hasPointerCapture(pointerId)) canvas.releasePointerCapture(pointerId);
    onStatus('网页自由演示 · 拖动连杆调整关节，Shift 可精细调整');
  }
  canvas.addEventListener('pointerdown', event => {
    if (!enabled() || !event.isPrimary || event.button !== 0 || held) return;
    const rig = getRig(), index = pickArmJoint(scene.camera, rig, canvas.getBoundingClientRect(), event.clientX, event.clientY);
    if (index == null) return;
    held = {rig, index, x: event.clientX, y: event.clientY, pose: structuredClone(getPose()), pointerId: event.pointerId};
    previousControls = scene.controls.enabled; scene.controls.enabled = false;
    canvas.setPointerCapture(event.pointerId); canvas.style.cursor = 'ew-resize';
    event.preventDefault(); event.stopImmediatePropagation(); onStatus(`正在调整关节 ${index + 1} · 网页模型`);
  }, options);
  canvas.addEventListener('pointermove', event => {
    if (held) {
      if (!enabled() || getRig() !== held.rig) { release(); return; }
      if (event.pointerId !== held.pointerId) return;
      const next = structuredClone(held.pose);
      next.joints[held.index] = dragJointAngle(held.pose.joints[held.index], event.clientX - held.x, event.clientY - held.y, event.shiftKey);
      setPose(next); event.preventDefault(); event.stopImmediatePropagation();
    } else if (enabled() && pickArmJoint(scene.camera, getRig(), canvas.getBoundingClientRect(), event.clientX, event.clientY) != null) {
      canvas.style.cursor = 'ew-resize';
    }
  }, options);
  for (const type of ['pointerup', 'pointercancel', 'lostpointercapture']) {
    canvas.addEventListener(type, event => {
      if (held && event.pointerId === held.pointerId) { release(); event.stopImmediatePropagation(); }
    }, options);
  }
  window.addEventListener('blur', release, {signal: lifetime.signal});
  return {release, dispose() { release(); lifetime.abort(); }};
}
