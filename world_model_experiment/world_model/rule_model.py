from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, Iterable, List

from .types import (
    DecisionAction,
    DecisionResult,
    GraspActionCandidate,
    TransitionPrediction,
    WorldState,
)


def _clamp(value: float) -> float:
    return max(0.0, min(1.0, float(value)))


@dataclass
class RuleWorldModelConfig:
    uncertainty_weights: Dict[str, float] = field(
        default_factory=lambda: {"semantic": 0.30, "depth": 0.25, "calibration": 0.20, "motion": 0.25}
    )
    success_weights: Dict[str, float] = field(
        default_factory=lambda: {"semantic": 0.25, "depth": 0.20, "calibration": 0.15, "motion": 0.40}
    )
    reobserve_uncertainty_threshold: float = 0.45
    probe_uncertainty_threshold: float = 0.22
    execute_success_threshold: float = 0.72
    collision_reject_threshold: float = 0.35
    minimum_workspace_margin_m: float = 0.015
    minimum_clearance_m: float = 0.010
    minimum_joint_margin_rad: float = 0.080
    minimum_singularity_margin_rad: float = 0.100

    @classmethod
    def from_json(cls, path: str | Path) -> "RuleWorldModelConfig":
        with Path(path).open("r", encoding="utf-8") as file:
            payload = json.load(file)
        return cls(**payload)

    def __post_init__(self) -> None:
        for weights_name in ("uncertainty_weights", "success_weights"):
            weights = getattr(self, weights_name)
            required = {"semantic", "depth", "calibration", "motion"}
            if set(weights) != required:
                raise ValueError(f"{weights_name} 必须且只能包含 {sorted(required)}。")
            if any(float(value) < 0.0 for value in weights.values()):
                raise ValueError(f"{weights_name} 不能包含负权重。")
            total = sum(float(value) for value in weights.values())
            if abs(total - 1.0) > 1e-6:
                raise ValueError(f"{weights_name} 权重之和必须为 1，当前为 {total}。")


class RuleWorldModel:
    """可解释的 V0 动作条件世界模型，后续可被学习模型替换。"""

    def __init__(self, config: RuleWorldModelConfig | None = None) -> None:
        self.config = config or RuleWorldModelConfig()

    def predict(self, state: WorldState, action: GraspActionCandidate) -> TransitionPrediction:
        target = state.object_by_id(action.target_track_id)
        reasons: List[str] = []

        semantic_uncertainty = _clamp(1.0 - target.semantic_confidence)
        depth_uncertainty = _clamp(1.0 - target.depth_confidence)
        calibration_uncertainty = 1.0 if not state.calibration.valid else _clamp(
            state.calibration.drift_score
            + min(state.calibration.translation_uncertainty_m / 0.020, 1.0) * 0.35
            + min(state.calibration.rotation_uncertainty_deg / 2.0, 1.0) * 0.20
        )

        motion_penalties = []
        if not action.ik_reachable or not target.reachable:
            motion_penalties.append(1.0)
            reasons.append("目标或候选动作不可达。")
        motion_penalties.extend(
            [
                1.0 - min(max(action.workspace_margin_m, 0.0) / max(self.config.minimum_workspace_margin_m, 1e-6), 1.0),
                1.0 - min(max(action.minimum_clearance_m, 0.0) / max(self.config.minimum_clearance_m, 1e-6), 1.0),
                1.0 - min(max(action.joint_margin_rad, 0.0) / max(self.config.minimum_joint_margin_rad, 1e-6), 1.0),
                1.0 - min(max(action.singularity_margin_rad, 0.0) / max(self.config.minimum_singularity_margin_rad, 1e-6), 1.0),
            ]
        )
        motion_uncertainty = _clamp(max(motion_penalties))

        uncertainty_components = {
            "semantic": semantic_uncertainty,
            "depth": depth_uncertainty,
            "calibration": calibration_uncertainty,
            "motion": motion_uncertainty,
        }
        total_uncertainty = _clamp(
            sum(self.config.uncertainty_weights[name] * value for name, value in uncertainty_components.items())
        )

        motion_confidence = 1.0 - motion_uncertainty
        success_components = {
            "semantic": target.semantic_confidence,
            "depth": target.depth_confidence,
            "calibration": 1.0 - calibration_uncertainty,
            "motion": motion_confidence,
        }
        success_probability = _clamp(
            sum(self.config.success_weights[name] * value for name, value in success_components.items())
            * (1.0 - 0.35 * total_uncertainty)
        )

        collision_probability = _clamp(
            max(
                1.0 if not action.ik_reachable else 0.0,
                1.0 - min(max(action.minimum_clearance_m, 0.0) / max(self.config.minimum_clearance_m, 1e-6), 1.0),
                1.0 - min(max(action.workspace_margin_m, 0.0) / max(self.config.minimum_workspace_margin_m, 1e-6), 1.0),
            )
        )
        slip_probability = _clamp(
            0.35 * (1.0 - target.depth_confidence)
            + 0.25 * min(abs(action.grasp_position_base_m.x - target.position_base_m.x) / 0.030, 1.0)
            + 0.25 * min(abs(action.grasp_position_base_m.y - target.position_base_m.y) / 0.030, 1.0)
            + 0.15 * min(target.position_std_m.z / 0.020, 1.0)
        )
        expected_motion_cost = _clamp(action.path_length_m / 1.5)

        if semantic_uncertainty > 0.35:
            reasons.append("语义目标一致性偏低。")
        if depth_uncertainty > 0.35:
            reasons.append("目标深度质量偏低。")
        if calibration_uncertainty > 0.35:
            reasons.append("标定状态或外参稳定性不足。")
        if collision_probability >= self.config.collision_reject_threshold:
            reasons.append("候选动作碰撞或越界风险过高。")

        return TransitionPrediction(
            candidate_id=action.candidate_id,
            success_probability=success_probability,
            collision_probability=collision_probability,
            slip_probability=slip_probability,
            total_uncertainty=total_uncertainty,
            semantic_uncertainty=semantic_uncertainty,
            depth_uncertainty=depth_uncertainty,
            calibration_uncertainty=calibration_uncertainty,
            motion_uncertainty=motion_uncertainty,
            expected_motion_cost=expected_motion_cost,
            reasons=reasons,
        )

    def decide(self, state: WorldState, candidates: Iterable[GraspActionCandidate]) -> DecisionResult:
        candidate_list = list(candidates)
        if not candidate_list:
            return DecisionResult(DecisionAction.REJECT, None, 0.0, "没有可评价的抓取候选。", None)
        if not state.robot.connected or not state.robot.can_move or state.robot.emergency_stopped:
            return DecisionResult(DecisionAction.REJECT, None, 0.0, "机器人状态不允许运动。", None)
        if not state.calibration.valid:
            return DecisionResult(DecisionAction.REJECT, None, 0.0, "标定状态无效，禁止规划真机动作。", None)

        predictions = [(candidate, self.predict(state, candidate)) for candidate in candidate_list]
        ranked = sorted(
            predictions,
            key=lambda pair: (
                pair[1].success_probability
                - 0.45 * pair[1].collision_probability
                - 0.20 * pair[1].slip_probability
                - 0.20 * pair[1].total_uncertainty
                - 0.05 * pair[1].expected_motion_cost
            ),
            reverse=True,
        )
        selected, prediction = ranked[0]
        score = _clamp(
            prediction.success_probability
            - 0.45 * prediction.collision_probability
            - 0.20 * prediction.slip_probability
            - 0.20 * prediction.total_uncertainty
            - 0.05 * prediction.expected_motion_cost
        )

        if prediction.collision_probability >= self.config.collision_reject_threshold:
            decision = DecisionAction.REJECT
            reason = "所有高分候选仍存在不可接受的碰撞或越界风险。"
        elif prediction.total_uncertainty >= self.config.reobserve_uncertainty_threshold:
            decision = DecisionAction.REOBSERVE
            reason = "综合不确定性过高，需要重新获取RGB-D观测。"
        elif (
            prediction.total_uncertainty >= self.config.probe_uncertainty_threshold
            or prediction.success_probability < self.config.execute_success_threshold
        ):
            decision = DecisionAction.PROBE
            reason = "候选可达，但应先执行低风险安全探测。"
        else:
            decision = DecisionAction.EXECUTE
            reason = "候选通过规则世界模型和硬状态检查。"

        return DecisionResult(decision, selected.candidate_id, score, reason, prediction)
