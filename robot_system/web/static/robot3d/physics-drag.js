import * as T from './vendor/three.module.js';

export function pickPhysicsBlock(camera, view, rect, x, y) {
  if (!view.root.visible || rect.width <= 0 || rect.height <= 0) return null;
  camera.updateMatrixWorld(); view.root.updateWorldMatrix(true, true);
  const ray = new T.Raycaster();
  ray.setFromCamera(new T.Vector2((x - rect.left) / rect.width * 2 - 1, 1 - (y - rect.top) / rect.height * 2), camera);
  const hit = ray.intersectObjects(view.blocks.map(b => b.mesh).filter(mesh => mesh.visible), false)[0];
  return hit ? {hit, ray} : null;
}

export function dragTarget(ray, plane, root, offset) {
  const point = ray.intersectPlane(plane, new T.Vector3());
  return point ? root.worldToLocal(point).sub(offset).toArray() : null;
}

export function mountPhysicsDrag(scene, view, world, {mode, onChange, enabled = () => true}) {
  const canvas = scene.renderer.domElement, listeners = new AbortController();
  let held = null, previousControls = true;
  function release() {
    if (!held) return;
    const id = held.pointerId; held = null; world.endDrag();
    scene.controls.enabled = previousControls; canvas.style.cursor = 'grab';
    if (canvas.hasPointerCapture(id)) canvas.releasePointerCapture(id);
    onChange();
  }
  const options = {capture: true, signal: listeners.signal};
  canvas.addEventListener('pointerdown', event => {
    if (!event.isPrimary || event.button !== 0 || held || !world.running || !enabled()) return;
    const picked = pickPhysicsBlock(scene.camera, view, canvas.getBoundingClientRect(), event.clientX, event.clientY);
    if (!picked) return;
    const body = world.bodies.find(b => `physics-block-${b.id}` === picked.hit.object.name);
    if (!body || !world.beginDrag(body.id)) return;
    const position = view.root.localToWorld(new T.Vector3(...body.position));
    const direction = event.shiftKey ? 'vertical' : mode();
    const normal = direction === 'free' ? scene.camera.getWorldDirection(new T.Vector3())
      : new T.Vector3(...(direction === 'vertical' ? [0, 1, 0] : [0, 0, 1])).transformDirection(view.root.matrixWorld);
    const plane = new T.Plane().setFromNormalAndCoplanarPoint(normal, position);
    const first = picked.ray.ray.intersectPlane(plane, new T.Vector3());
    if (!first) { world.endDrag(); return; }
    const offset = view.root.worldToLocal(first).sub(new T.Vector3(...body.position));
    held = {pointerId: event.pointerId, plane, offset};
    previousControls = scene.controls.enabled; scene.controls.enabled = false;
    canvas.setPointerCapture(event.pointerId); canvas.style.cursor = 'grabbing';
    event.preventDefault(); event.stopImmediatePropagation(); onChange();
  }, options);
  canvas.addEventListener('pointermove', event => {
    if (held) {
      if (event.pointerId !== held.pointerId) return;
      const rect = canvas.getBoundingClientRect(), ray = new T.Raycaster();
      ray.setFromCamera(new T.Vector2((event.clientX - rect.left) / rect.width * 2 - 1,
        1 - (event.clientY - rect.top) / rect.height * 2), scene.camera);
      const target = dragTarget(ray.ray, held.plane, view.root, held.offset);
      if (target) world.setDragTarget(target);
      event.preventDefault(); event.stopImmediatePropagation(); onChange();
    } else if (world.running && enabled()) {
      canvas.style.cursor = pickPhysicsBlock(scene.camera, view, canvas.getBoundingClientRect(), event.clientX, event.clientY) ? 'grab' : '';
    }
  }, options);
  for (const type of ['pointerup', 'pointercancel', 'lostpointercapture']) {
    canvas.addEventListener(type, event => {
      if (held && event.pointerId === held.pointerId) { release(); event.stopImmediatePropagation(); }
    }, options);
  }
  window.addEventListener('blur', release, {signal: listeners.signal});
  return {release, dispose() { release(); listeners.abort(); canvas.style.cursor = ''; }};
}
