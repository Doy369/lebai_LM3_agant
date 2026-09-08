import { GLTFLoader } from './vendor/GLTFLoader.js';
import { LoadingManager } from './vendor/three.module.js';
import { JointRig } from './joints.js';

export function disposeGraph(root) {
  const disposed = new Set();
  const dispose = item => { if (item && !disposed.has(item)) { disposed.add(item); item.dispose?.(); } };
  root?.traverse(node => {
    dispose(node.geometry);
    for (const material of Array.isArray(node.material) ? node.material : [node.material]) {
      if (!material) continue;
      Object.values(material).filter(v => v?.isTexture).forEach(texture => {
        if (!disposed.has(texture)) texture.source?.data?.close?.();
        dispose(texture);
      });
      dispose(material);
    }
  });
}
export function inspectGLB(buffer) {
  if (!(buffer instanceof ArrayBuffer) || buffer.byteLength < 20 || buffer.byteLength > 50 * 1024 * 1024) throw Error('GLB 文件无效或超过 50 MiB');
  const v = new DataView(buffer);
  if (v.getUint32(0, true) !== 0x46546c67 || v.getUint32(4, true) !== 2 || v.getUint32(8, true) !== buffer.byteLength || v.getUint32(16, true) !== 0x4e4f534a) throw Error('仅支持 glTF 2.0 二进制 GLB');
  const length = v.getUint32(12, true);
  if (length + 20 > buffer.byteLength) throw Error('GLB JSON 长度无效');
  const json = JSON.parse(new TextDecoder().decode(new Uint8Array(buffer, 20, length)));
  if ([...(json.buffers ?? []), ...(json.images ?? [])].some(r => r.uri && !r.uri.startsWith('data:'))) throw Error('只支持自包含 GLB；不接受外部 bin/纹理资源');
  if ((json.extensionsRequired ?? []).some(e => !['KHR_materials_specular', 'KHR_materials_unlit'].includes(e))) throw Error('模型含不支持的必需扩展（不支持 Draco/KTX2/Meshopt）');
  return json;
}

// Commit only the newest successful candidate; failure leaves the old resource alive.
export class LatestResource {
  constructor(dispose) { this.disposeResource = dispose; this.revision = 0; this.current = null; this.closed = false; }
  async replace(create) {
    if (this.closed) return false;
    const revision = ++this.revision;
    let candidate;
    try { candidate = await create(); }
    catch (e) { if (revision !== this.revision || this.closed) return false; throw e; }
    if (revision !== this.revision || this.closed) { this.disposeResource(candidate); return false; }
    const old = this.current; this.current = candidate;
    if (old) this.disposeResource(old);
    return true;
  }
  dispose() { this.closed = true; this.revision++; if (this.current) this.disposeResource(this.current); this.current = null; }
}

export class ModelLoader {
  constructor(parent) {
    this.parent = parent; this.controller = null;
    this.slot = new LatestResource(value => { value.rig.dispose(); value.gltf.scene.removeFromParent(); disposeGraph(value.gltf.scene); });
  }
  async load(source) {
    this.controller?.abort(); const controller = new AbortController(); this.controller = controller;
    const changed = await this.slot.replace(async () => {
      let buffer;
      if (source instanceof File) {
        if (!/\.glb$/i.test(source.name) || source.size > 50 * 1024 * 1024) throw Error('请选择不超过 50 MiB 的自包含 .glb');
        buffer = await source.arrayBuffer();
      } else {
        const response = await fetch(source, {signal: controller.signal});
        if (!response.ok) throw Error(`模型读取失败 HTTP ${response.status}`);
        buffer = await response.arrayBuffer();
      }
      inspectGLB(buffer);
      const manager = new LoadingManager();
      const temporaryUrls = new Set();
      manager.setURLModifier(url => {
        if (!/^(blob:|data:)/.test(url)) throw Error('拒绝 GLB 外部资源');
        if (url.startsWith('blob:')) temporaryUrls.add(url);
        return url;
      });
      let gltf;
      try { gltf = await new GLTFLoader(manager).parseAsync(buffer, ''); }
      finally { temporaryUrls.forEach(url => URL.revokeObjectURL(url)); }
      try { return {gltf, rig: new JointRig(gltf)}; }
      catch (e) { disposeGraph(gltf.scene); throw e; }
    });
    if (changed) this.parent.add(this.slot.current.gltf.scene);
    return changed;
  }
  get current() { return this.slot.current; }
  dispose() { this.controller?.abort(); this.slot.dispose(); }
}
