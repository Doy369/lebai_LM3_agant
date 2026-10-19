import * as T from './vendor/three.module.js';
import {number, vector} from './validation.js';

const LINK_NAMES = Array.from({length: 6}, (_, index) => `link_${index}`);

function integer(value, name, minimum = -1, maximum = 5000) {
  const parsed = number(value, name, minimum, maximum);
  if (!Number.isInteger(parsed)) throw Error(`${name} 必须是整数`);
  return parsed;
}

function identifier(value, name) {
  if (typeof value !== 'string' || !value.trim() || value.length > 160) throw Error(`${name} 无效`);
  return value;
}

export function collisionVisualizationFromReport(report, samples, startJoints) {
  if (report == null) return null;
  if (!report || typeof report !== 'object' || Array.isArray(report) || report.schema_version !== 1) {
    throw Error('仅支持碰撞验证报告 version=1');
  }
  if (!['collision', 'clear_approximate'].includes(report.status)) throw Error('碰撞验证报告状态无效');
  if (report.geometry_status !== 'approximate_unverified') throw Error('碰撞几何置信状态无效');
  const linkRadiiM = vector(report.link_radii_m, 6, '连杆胶囊半径(m)', .001, .5);
  const safetyMarginM = number(report.safety_margin_m, '碰撞安全余量(m)', 0, .5);
  if (!Array.isArray(report.obstacles) || report.obstacles.length > 101) throw Error('碰撞障碍物列表无效');
  const obstacles = report.obstacles.map((item, index) => {
    if (!item || typeof item !== 'object' || Array.isArray(item)) throw Error(`障碍物 ${index} 无效`);
    return {
      objectId: identifier(item.object_id, `障碍物 ${index} ID`),
      centerBaseM: vector(item.center_base_m, 3, `障碍物 ${index} 中心(m)`, -5, 5),
      sizeXyzM: vector(item.size_xyz_m, 3, `障碍物 ${index} 尺寸(m)`, .001, 5),
      source: identifier(item.source, `障碍物 ${index} 来源`),
    };
  });
  const knownNames = new Set([...LINK_NAMES, ...obstacles.map(item => item.objectId)]);
  if (!Array.isArray(report.contacts) || report.contacts.length > 200) throw Error('碰撞接触记录无效');
  const contacts = report.contacts.map((item, index) => {
    if (!item || typeof item !== 'object' || Array.isArray(item)) throw Error(`接触记录 ${index} 无效`);
    const first = identifier(item.first, `接触记录 ${index} 第一对象`);
    const second = identifier(item.second, `接触记录 ${index} 第二对象`);
    if (!knownNames.has(first) || !knownNames.has(second)) throw Error(`接触记录 ${index} 引用了未知对象`);
    return {
      sampleIndex: integer(item.sample_index, `接触记录 ${index} 样本号`),
      stepId: identifier(item.step_id, `接触记录 ${index} 步骤`),
      phase: identifier(item.phase, `接触记录 ${index} 阶段`),
      first, second,
      clearanceM: number(item.clearance_m, `接触记录 ${index} 间隙(m)`, -2, 5),
    };
  });
  let closestPair = null, closestPointsBaseM = null, closestSampleIndex = null, focusJoints = null;
  if (report.closest_pair != null) {
    if (!Array.isArray(report.closest_pair) || report.closest_pair.length !== 2) throw Error('最近对象对无效');
    closestPair = report.closest_pair.map((value, index) => identifier(value, `最近对象 ${index}`));
    if (closestPair.some(value => !knownNames.has(value))) throw Error('最近对象对引用了未知对象');
  }
  if (report.closest_points_base_m != null) {
    if (!Array.isArray(report.closest_points_base_m) || report.closest_points_base_m.length !== 2) throw Error('最近点坐标无效');
    closestPointsBaseM = report.closest_points_base_m.map((point, index) => vector(point, 3, `最近点 ${index}(m)`, -5, 5));
  }
  if (report.closest_sample_index != null) {
    closestSampleIndex = integer(report.closest_sample_index, '最近点样本号');
    if (closestSampleIndex === -1) focusJoints = vector(startJoints, 6, '计划起点关节(rad)', -Math.PI, Math.PI);
    else {
      const sample = samples.find(item => item?.sample_index === closestSampleIndex);
      if (!sample) throw Error('碰撞报告最近点样本不存在于轨迹报告');
      focusJoints = vector(sample.joints_rad, 6, '最近点关节(rad)', -Math.PI, Math.PI);
    }
  }
  return {
    reportId: identifier(report.report_id, '碰撞报告 ID'),
    status: report.status,
    geometryStatus: report.geometry_status,
    sceneStatus: identifier(report.scene_status, '碰撞场景状态'),
    modelId: identifier(report.model_id, '碰撞模型 ID'),
    linkRadiiM,
    safetyMarginM,
    minimumClearanceM: report.minimum_clearance_m == null ? null : number(report.minimum_clearance_m, '最小间隙(m)', -2, 5),
    closestPair,
    closestPointsBaseM,
    closestSampleIndex,
    closestStepId: report.closest_step_id == null ? null : identifier(report.closest_step_id, '最近点步骤'),
    closestPhase: report.closest_phase == null ? null : identifier(report.closest_phase, '最近点阶段'),
    focusJoints,
    obstacles,
    contacts,
  };
}

export function syntheticCollisionDemo() {
  return {
    reportId: 'synthetic_collision_demo_no_execution',
    status: 'collision',
    geometryStatus: 'approximate_unverified',
    sceneStatus: 'synthetic_offline_demo',
    modelId: 'lebai_glb_capsules_v1',
    linkRadiiM: [.075, .065, .06, .055, .05, .045],
    safetyMarginM: .01,
    minimumClearanceM: -.007046869663566227,
    closestPair: ['link_3', 'synthetic_fixture'],
    closestPointsBaseM: [
      [-.25798005741545177, .14673592280178743, .4534533740384919],
      [-.26, .14, .453],
    ],
    closestSampleIndex: 5,
    closestStepId: '02_approach_pick',
    closestPhase: 'approach_pick',
    focusJoints: [0, -1.1, 1.25, 0, 1.3, 0],
    obstacles: [
      {objectId: 'default_table', centerBaseM: [0, 0, -.035], sizeXyzM: [1.2, 1, .07], source: 'configured_default'},
      {objectId: 'synthetic_fixture', centerBaseM: [-.25, .15, .463], sizeXyzM: [.02, .02, .02], source: 'synthetic_demo'},
    ],
    contacts: [{
      sampleIndex: 5, stepId: '02_approach_pick', phase: 'approach_pick',
      first: 'link_3', second: 'synthetic_fixture', clearanceM: -.007046869663566227,
    }],
  };
}

function disposeObject(root) {
  root.traverse(object => {
    object.geometry?.dispose?.();
    const materials = Array.isArray(object.material) ? object.material : object.material ? [object.material] : [];
    materials.forEach(material => material.dispose?.());
  });
  root.clear();
}

function capsulePart(material, geometry) {
  const mesh = new T.Mesh(geometry, material);
  mesh.renderOrder = 5;
  return mesh;
}

export class CollisionOverlay {
  constructor(base) {
    this.base = base;
    this.root = new T.Group(); this.root.name = 'collision-validation-overlay';
    this.links = new T.Group(); this.links.name = 'collision-link-capsules';
    this.boxes = new T.Group(); this.boxes.name = 'collision-obstacle-boxes';
    this.root.add(this.links, this.boxes); this.base.add(this.root);
    this.minimumLine = new T.Line(
      new T.BufferGeometry(),
      new T.LineBasicMaterial({color: 0xffb020, depthTest: false, transparent: true, opacity: .95}),
    );
    this.minimumLine.name = 'minimum-clearance-line'; this.minimumLine.visible = false;
    this.minimumLine.renderOrder = 8; this.root.add(this.minimumLine);
    this.linkObjects = []; this.boxObjects = new Map(); this.data = null; this.enabled = true;
  }
  setData(data) {
    disposeObject(this.links); disposeObject(this.boxes); this.linkObjects = []; this.boxObjects.clear();
    this.data = data; this.minimumLine.visible = false;
    if (!data) return;
    data.linkRadiiM.forEach((radius, index) => {
      const material = new T.MeshBasicMaterial({color: 0x16a3b6, transparent: true, opacity: .20, depthWrite: false});
      const group = new T.Group(); group.name = `collision-link_${index}`; group.userData.material = material;
      const cylinder = capsulePart(material, new T.CylinderGeometry(1, 1, 1, 16, 1, true));
      const start = capsulePart(material, new T.SphereGeometry(1, 12, 8));
      const end = capsulePart(material, new T.SphereGeometry(1, 12, 8));
      cylinder.scale.set(radius, 1, radius); start.scale.setScalar(radius); end.scale.setScalar(radius);
      group.add(cylinder, start, end); group.userData.parts = {cylinder, start, end};
      this.links.add(group); this.linkObjects.push(group);
    });
    data.obstacles.forEach(obstacle => {
      const group = new T.Group(); group.name = `collision-box-${obstacle.objectId}`;
      group.position.fromArray(obstacle.centerBaseM); group.scale.fromArray(obstacle.sizeXyzM);
      const fillMaterial = new T.MeshBasicMaterial({color: 0xffb020, transparent: true, opacity: .10, depthWrite: false});
      const edgeMaterial = new T.LineBasicMaterial({color: 0xd38400, transparent: true, opacity: .75});
      const geometry = new T.BoxGeometry(1, 1, 1);
      const fill = new T.Mesh(geometry, fillMaterial); fill.renderOrder = 3;
      const edges = new T.LineSegments(new T.EdgesGeometry(geometry), edgeMaterial); edges.renderOrder = 4;
      group.add(fill, edges); group.userData.materials = {fillMaterial, edgeMaterial};
      this.boxes.add(group); this.boxObjects.set(obstacle.objectId, group);
    });
    if (data.closestPointsBaseM) {
      this.minimumLine.geometry.setFromPoints(data.closestPointsBaseM.map(point => new T.Vector3(...point)));
    }
    this.root.visible = this.enabled;
  }
  setEnabled(enabled) { this.enabled = Boolean(enabled); this.root.visible = this.enabled; }
  update(rig, sampleIndex = null, pinClosest = false) {
    if (!this.data || !rig || !this.enabled) return;
    this.base.updateWorldMatrix(true, true); rig.root.updateWorldMatrix(true, true);
    const origins = [new T.Vector3(0, 0, 0), ...rig.joints.map(joint => {
      const point = joint.node.getWorldPosition(new T.Vector3());
      return this.base.worldToLocal(point);
    })];
    this.linkObjects.forEach((group, index) => {
      const from = origins[index], to = origins[index + 1], direction = to.clone().sub(from);
      const length = direction.length(), parts = group.userData.parts;
      parts.cylinder.position.copy(from).add(to).multiplyScalar(.5);
      parts.cylinder.scale.y = Math.max(length, 1e-6);
      parts.cylinder.quaternion.setFromUnitVectors(new T.Vector3(0, 1, 0), direction.normalize());
      parts.start.position.copy(from); parts.end.position.copy(to);
    });
    const activeContacts = this.data.contacts.filter(contact => contact.sampleIndex === sampleIndex);
    const highlighted = new Set(activeContacts.flatMap(contact => [contact.first, contact.second]));
    const atClosest = pinClosest || sampleIndex === this.data.closestSampleIndex;
    if (atClosest && this.data.closestPair) this.data.closestPair.forEach(name => highlighted.add(name));
    const collision = activeContacts.length > 0;
    this.linkObjects.forEach((group, index) => {
      const material = group.userData.material;
      const selected = highlighted.has(`link_${index}`);
      material.color.setHex(selected ? (collision ? 0xe52e2e : 0xffa000) : 0x16a3b6);
      material.opacity = selected ? .58 : .20;
    });
    this.boxObjects.forEach((group, name) => {
      const selected = highlighted.has(name), materials = group.userData.materials;
      materials.fillMaterial.color.setHex(selected ? (collision ? 0xe52e2e : 0xffa000) : 0xffb020);
      materials.edgeMaterial.color.setHex(selected ? (collision ? 0xc91414 : 0xd87b00) : 0xd38400);
      materials.fillMaterial.opacity = selected ? .28 : .10;
    });
    this.minimumLine.visible = Boolean(atClosest && this.data.closestPointsBaseM);
    this.minimumLine.material.color.setHex(this.data.minimumClearanceM <= this.data.safetyMarginM ? 0xe52e2e : 0xffa000);
  }
  dispose() {
    this.minimumLine.geometry.dispose(); this.minimumLine.material.dispose();
    disposeObject(this.links); disposeObject(this.boxes); this.root.removeFromParent(); this.data = null;
  }
}
