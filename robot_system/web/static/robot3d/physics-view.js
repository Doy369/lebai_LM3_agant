import * as T from './vendor/three.module.js';
import { disposeGraph } from './model-loader.js';
import { bodySize, bodyColor } from './objects.js';

export class PhysicsView {
  constructor(base) {
    this.root = new T.Group(); this.root.name = 'interactive-block-physics';
    this.root.visible = false; base.add(this.root);
    this.blocks = ['A', 'B'].map(id => this.createBlock({id}));
  }
  createBlock(body) {
    const mesh = new T.Mesh(new T.BoxGeometry(1, 1, 1), new T.MeshStandardMaterial({color: bodyColor(body), roughness: .65}));
    mesh.name = `physics-block-${body.id}`; mesh.scale.fromArray(bodySize(body)); this.root.add(mesh); return {id: body.id, mesh};
  }
  update(snapshot, mode) {
    const ids = new Set(snapshot.bodies.map(b => b.id));
    if (mode === 'drop') { ids.add('A'); ids.add('B'); }
    this.blocks = this.blocks.filter(block => {
      if (ids.has(block.id)) return true;
      disposeGraph(block.mesh); block.mesh.removeFromParent(); return false;
    });
    for (const body of snapshot.bodies) if (!this.blocks.some(b => b.id === body.id)) this.blocks.push(this.createBlock(body));
    this.blocks.forEach(({id, mesh}) => {
      const body = snapshot.bodies.find(b => b.id === id); mesh.visible = Boolean(body);
      if (body) { mesh.position.fromArray(body.position); mesh.scale.fromArray(bodySize(body)); mesh.material.color.set(bodyColor(body)); mesh.quaternion.fromArray(body.rotation ?? [0, 0, 0, 1]); }
      mesh.material.emissive.setHex(body && (snapshot.dragId === body.id || snapshot.heldId === body.id) ? 0x254b35 : 0);
    });
  }
  dispose() { disposeGraph(this.root); this.root.removeFromParent(); }
}
