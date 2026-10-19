import {number, vector} from './validation.js';
import {collisionVisualizationFromReport} from './collision-overlay.js';

const PHASES = [
  'prepare_open', 'approach_pick', 'descend_pick', 'grasp', 'retreat_pick',
  'transfer', 'descend_place', 'release', 'retreat_place',
];
const MOVE_ACTIONS = new Set(['movej', 'movel']);
const GRIPPER = {gripper_open: 1, gripper_close: 0};

export function compiledPlanToTrajectory(value, trajectoryValidation = null) {
  if (!value || typeof value !== 'object' || Array.isArray(value)) throw Error('统一抓取计划必须是对象');
  if (value.schema_version !== 1) throw Error('仅支持统一抓取计划 version=1');
  if (typeof value.plan_id !== 'string' || !/^grasp_[0-9a-f]{24}$/.test(value.plan_id)) throw Error('统一抓取计划 ID 无效');
  const startJoints = vector(value.start_joints_rad, 6, '计划起始关节(rad)', -Math.PI, Math.PI);
  if (!Array.isArray(value.steps) || value.steps.length !== PHASES.length) throw Error('统一抓取计划必须包含九个阶段');
  if (Date.now() > number(value.expires_at_unix_ms, '计划过期时间', 1)) throw Error('统一抓取计划已过期');

  let reportSamples = null;
  if (trajectoryValidation != null) {
    if (!trajectoryValidation || typeof trajectoryValidation !== 'object' || trajectoryValidation.plan_id !== value.plan_id) {
      throw Error('轨迹验证报告与统一抓取计划不匹配');
    }
    if (trajectoryValidation.kinematic_status !== 'pass' || !Array.isArray(trajectoryValidation.samples) || trajectoryValidation.samples.length > 5000) {
      throw Error('轨迹验证报告没有通过或采样数量无效');
    }
    reportSamples = trajectoryValidation.samples;
  }

  const seen = new Set(), segments = [];
  value.steps.forEach((step, index) => {
    if (!step || typeof step !== 'object' || Array.isArray(step)) throw Error(`计划第 ${index + 1} 步无效`);
    if (step.phase !== PHASES[index]) throw Error(`计划第 ${index + 1} 步阶段顺序错误`);
    if (typeof step.step_id !== 'string' || !step.step_id || seen.has(step.step_id)) throw Error('计划步骤 ID 缺失或重复');
    seen.add(step.step_id);
    const durationMs = number(step.duration_hint_sec, `${step.step_id} 时长`, .1, 60) * 1000;
    if (MOVE_ACTIONS.has(step.action)) {
      const endpoint = vector(step.target_joints_rad, 6, `${step.step_id} 关节(rad)`, -Math.PI, Math.PI);
      const samples = reportSamples?.filter(sample => sample.step_id === step.step_id) ?? [];
      if (reportSamples && samples.length === 0) throw Error(`${step.step_id} 缺少连续轨迹样本`);
      if (samples.length) {
        let previousProgress = 0;
        samples.forEach((sample, sampleIndex) => {
          if (sample.phase !== step.phase || sample.action !== step.action) throw Error(`${step.step_id} 采样阶段不匹配`);
          const progress = number(sample.progress, `${step.step_id} 样本进度`, Number.EPSILON, 1);
          if (progress <= previousProgress) throw Error(`${step.step_id} 样本进度必须递增`);
          previousProgress = progress;
          segments.push({
            joints: vector(sample.joints_rad, 6, `${step.step_id} 样本关节(rad)`, -Math.PI, Math.PI),
            durationMs: durationMs / samples.length,
            meta: {
              sampleIndex: Number.isInteger(sample.sample_index) ? sample.sample_index : null,
              stepId: step.step_id,
              phase: step.phase,
            },
          });
          if (sampleIndex === samples.length - 1 && Math.abs(progress - 1) > 1e-9) throw Error(`${step.step_id} 最后样本必须到达进度 1`);
        });
      } else segments.push({joints: endpoint, durationMs});
      return;
    }
    if (!(step.action in GRIPPER) || step.target_joints_rad != null || step.target_tcp != null) {
      throw Error(`${step.step_id} 动作或目标无效`);
    }
    segments.push({joints: {}, gripper: GRIPPER[step.action], durationMs});
  });
  const validation = value.validation;
  if (!validation || validation.ik !== 'pass' || validation.workspace !== 'pass' || validation.joint_limits !== 'pass') {
    throw Error('统一抓取计划没有通过 IK、工作空间和关节限位检查');
  }
  return {
    planId: value.plan_id,
    startJoints,
    segments,
    collisionStatus: String(trajectoryValidation?.collision_status || validation.collision || 'unknown'),
    cameraAlignment: String(validation.camera_alignment || 'pending'),
    collisionVisualization: collisionVisualizationFromReport(
      trajectoryValidation?.collision_validation ?? null,
      reportSamples ?? [],
      startJoints,
    ),
  };
}
