from __future__ import annotations

import hashlib
import json
import math
import time
from dataclasses import dataclass
from typing import Any, Callable, Dict, List, Mapping, Optional, Sequence, Tuple

from .compiled_plan import CartesianTarget, CompiledGraspPlan
from .collision_validation import CollisionValidationReport


IKSolver = Callable[[Dict[str, float], Sequence[float]], Sequence[float]]
JointValidator = Callable[[Sequence[float], str], None]
WorkspaceValidator = Callable[[CartesianTarget, str], None]


def _lerp(a: float, b: float, progress: float) -> float:
    return float(a) + (float(b) - float(a)) * progress


def _angle_delta(a: float, b: float) -> float:
    return math.atan2(math.sin(float(b) - float(a)), math.cos(float(b) - float(a)))


def _interpolate_tcp(start: CartesianTarget, end: CartesianTarget, progress: float) -> CartesianTarget:
    return CartesianTarget(
        x=_lerp(start.x, end.x, progress),
        y=_lerp(start.y, end.y, progress),
        z=_lerp(start.z, end.z, progress),
        rz=start.rz + _angle_delta(start.rz, end.rz) * progress,
        ry=start.ry + _angle_delta(start.ry, end.ry) * progress,
        rx=start.rx + _angle_delta(start.rx, end.rx) * progress,
    )


@dataclass(frozen=True)
class TrajectoryValidationConfig:
    max_joint_sample_step_rad: float = 0.05
    max_linear_sample_step_m: float = 0.005
    max_angular_sample_step_rad: float = math.radians(2.0)
    max_adjacent_joint_jump_rad: float = 0.35
    max_total_samples: int = 5000

    def __post_init__(self) -> None:
        for name in (
            "max_joint_sample_step_rad", "max_linear_sample_step_m",
            "max_angular_sample_step_rad", "max_adjacent_joint_jump_rad",
        ):
            value = float(getattr(self, name))
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} 必须是大于 0 的有限数值。")
        if not isinstance(self.max_total_samples, int) or self.max_total_samples < 9:
            raise ValueError("max_total_samples 必须是不小于 9 的整数。")

    def to_dict(self) -> Dict[str, Any]:
        return {
            "max_joint_sample_step_rad": self.max_joint_sample_step_rad,
            "max_linear_sample_step_m": self.max_linear_sample_step_m,
            "max_angular_sample_step_rad": self.max_angular_sample_step_rad,
            "max_adjacent_joint_jump_rad": self.max_adjacent_joint_jump_rad,
            "max_total_samples": self.max_total_samples,
        }


@dataclass(frozen=True)
class TrajectorySample:
    sample_index: int
    step_id: str
    phase: str
    action: str
    progress: float
    joints_rad: Tuple[float, ...]
    tcp: Optional[CartesianTarget]

    def to_dict(self) -> Dict[str, Any]:
        return {
            "sample_index": self.sample_index,
            "step_id": self.step_id,
            "phase": self.phase,
            "action": self.action,
            "progress": self.progress,
            "joints_rad": list(self.joints_rad),
            "tcp": self.tcp.to_dict() if self.tcp else None,
        }


@dataclass(frozen=True)
class SegmentValidation:
    step_id: str
    phase: str
    action: str
    sample_count: int
    path_length_m: Optional[float]
    max_joint_sample_delta_rad: float
    intermediate_ik: str
    workspace_path: str

    def to_dict(self) -> Dict[str, Any]:
        return {
            "step_id": self.step_id,
            "phase": self.phase,
            "action": self.action,
            "sample_count": self.sample_count,
            "path_length_m": self.path_length_m,
            "max_joint_sample_delta_rad": self.max_joint_sample_delta_rad,
            "intermediate_ik": self.intermediate_ik,
            "workspace_path": self.workspace_path,
        }


@dataclass(frozen=True)
class TrajectoryValidationReport:
    report_id: str
    plan_id: str
    kinematic_status: str
    collision_status: str
    gate_recommendation: str
    checked_at_unix_ms: int
    config: TrajectoryValidationConfig
    segments: Tuple[SegmentValidation, ...]
    samples: Tuple[TrajectorySample, ...]
    issues: Tuple[str, ...]
    collision_validation: Optional[CollisionValidationReport] = None
    schema_version: int = 2

    @classmethod
    def build(
        cls,
        *,
        plan_id: str,
        kinematic_status: str,
        config: TrajectoryValidationConfig,
        segments: Sequence[SegmentValidation],
        samples: Sequence[TrajectorySample],
        issues: Sequence[str],
        checked_at_unix_ms: Optional[int] = None,
    ) -> "TrajectoryValidationReport":
        if kinematic_status not in {"pass", "reject"}:
            raise ValueError("kinematic_status 必须是 pass 或 reject。")
        provisional = cls(
            report_id="pending",
            plan_id=str(plan_id),
            kinematic_status=kinematic_status,
            collision_status="unknown",
            gate_recommendation="reject" if kinematic_status == "reject" else "reobserve",
            checked_at_unix_ms=int(checked_at_unix_ms if checked_at_unix_ms is not None else time.time() * 1000),
            config=config,
            segments=tuple(segments),
            samples=tuple(samples),
            issues=tuple(str(issue) for issue in issues),
        )
        return cls(**{**provisional.__dict__, "report_id": provisional.expected_report_id()})

    def with_collision_validation(
        self,
        collision_validation: CollisionValidationReport,
    ) -> "TrajectoryValidationReport":
        if collision_validation.plan_id != self.plan_id:
            raise ValueError("碰撞验证报告与轨迹验证报告的 plan_id 不一致。")
        collision_status = "collision" if collision_validation.status == "collision" else "unknown"
        recommendation = "reject" if self.kinematic_status == "reject" or collision_status == "collision" else "reobserve"
        merged_issues = tuple(self.issues) + tuple(
            issue for issue in collision_validation.issues if issue not in self.issues
        )
        provisional = TrajectoryValidationReport(
            report_id="pending",
            plan_id=self.plan_id,
            kinematic_status=self.kinematic_status,
            collision_status=collision_status,
            gate_recommendation=recommendation,
            checked_at_unix_ms=self.checked_at_unix_ms,
            config=self.config,
            segments=self.segments,
            samples=self.samples,
            issues=merged_issues,
            collision_validation=collision_validation,
        )
        return TrajectoryValidationReport(
            **{**provisional.__dict__, "report_id": provisional.expected_report_id()}
        )

    @classmethod
    def from_dict(cls, payload: Mapping[str, Any]) -> "TrajectoryValidationReport":
        expected = {
            "schema_version", "report_id", "plan_id", "kinematic_status", "collision_status",
            "gate_recommendation", "checked_at_unix_ms", "config", "segments", "samples", "issues",
            "collision_validation",
        }
        if set(payload) != expected:
            raise ValueError("轨迹验证报告字段不匹配。")
        config = TrajectoryValidationConfig(**dict(payload["config"]))
        segments = tuple(SegmentValidation(**item) for item in payload["segments"])
        samples = tuple(
            TrajectorySample(
                sample_index=int(item["sample_index"]),
                step_id=str(item["step_id"]),
                phase=str(item["phase"]),
                action=str(item["action"]),
                progress=float(item["progress"]),
                joints_rad=tuple(float(value) for value in item["joints_rad"]),
                tcp=CartesianTarget.from_dict(item["tcp"]) if isinstance(item.get("tcp"), Mapping) else None,
            )
            for item in payload["samples"]
        )
        report = cls(
            schema_version=int(payload["schema_version"]),
            report_id=str(payload["report_id"]),
            plan_id=str(payload["plan_id"]),
            kinematic_status=str(payload["kinematic_status"]),
            collision_status=str(payload["collision_status"]),
            gate_recommendation=str(payload["gate_recommendation"]),
            checked_at_unix_ms=int(payload["checked_at_unix_ms"]),
            config=config,
            segments=segments,
            samples=samples,
            issues=tuple(str(issue) for issue in payload["issues"]),
            collision_validation=(
                CollisionValidationReport.from_dict(payload["collision_validation"])
                if isinstance(payload["collision_validation"], Mapping)
                else None
            ),
        )
        if report.collision_validation and report.collision_validation.plan_id != report.plan_id:
            raise ValueError("碰撞验证报告与轨迹验证报告的 plan_id 不一致。")
        if report.report_id != report.expected_report_id():
            raise ValueError("轨迹验证报告内容与 report_id 哈希不一致。")
        if report.plan_id == "" or report.kinematic_status not in {"pass", "reject"}:
            raise ValueError("轨迹验证报告状态无效。")
        if report.collision_status not in {"unknown", "collision"}:
            raise ValueError("轨迹验证报告的碰撞状态无效。")
        expected_gate = (
            "reject"
            if report.kinematic_status == "reject" or report.collision_status == "collision"
            else "reobserve"
        )
        if report.gate_recommendation != expected_gate:
            raise ValueError("轨迹验证报告的门控建议与验证状态不一致。")
        if report.collision_validation:
            expected_collision = (
                "collision" if report.collision_validation.status == "collision" else "unknown"
            )
            if report.collision_status != expected_collision:
                raise ValueError("碰撞验证明细与轨迹验证报告状态不一致。")
        return report

    def expected_report_id(self) -> str:
        payload = self.to_dict()
        payload.pop("report_id", None)
        canonical = json.dumps(payload, ensure_ascii=False, sort_keys=True, separators=(",", ":"), allow_nan=False)
        return "trajectory_" + hashlib.sha256(canonical.encode("utf-8")).hexdigest()[:24]

    def to_dict(self) -> Dict[str, Any]:
        return {
            "schema_version": self.schema_version,
            "report_id": self.report_id,
            "plan_id": self.plan_id,
            "kinematic_status": self.kinematic_status,
            "collision_status": self.collision_status,
            "gate_recommendation": self.gate_recommendation,
            "checked_at_unix_ms": self.checked_at_unix_ms,
            "config": self.config.to_dict(),
            "segments": [segment.to_dict() for segment in self.segments],
            "samples": [sample.to_dict() for sample in self.samples],
            "issues": list(self.issues),
            "collision_validation": (
                self.collision_validation.to_dict() if self.collision_validation else None
            ),
        }


def validate_compiled_trajectory(
    plan: CompiledGraspPlan,
    *,
    ik_solver: IKSolver,
    joint_validator: JointValidator,
    workspace_validator: WorkspaceValidator,
    config: Optional[TrajectoryValidationConfig] = None,
) -> TrajectoryValidationReport:
    """对 MoveJ/MoveL 全段采样；整机几何碰撞仍保守标记 unknown。"""
    cfg = config or TrajectoryValidationConfig()
    samples: List[TrajectorySample] = []
    segments: List[SegmentValidation] = []
    issues: List[str] = []
    previous_joints = tuple(plan.start_joints_rad)
    previous_tcp: Optional[CartesianTarget] = None

    def add_sample(step: Any, progress: float, joints: Sequence[float], tcp: Optional[CartesianTarget]) -> None:
        nonlocal previous_joints
        resolved = tuple(float(value) for value in joints)
        if len(resolved) != 6 or any(not math.isfinite(value) for value in resolved):
            raise ValueError(f"{step.step_id} 返回了无效关节解。")
        joint_validator(resolved, f"{step.step_id}_sample_{len(samples)}")
        jump = max(abs(current - previous) for current, previous in zip(resolved, previous_joints))
        if jump > cfg.max_adjacent_joint_jump_rad:
            raise ValueError(
                f"{step.step_id} 相邻采样关节跳变过大: {jump:.4f}rad > {cfg.max_adjacent_joint_jump_rad:.4f}rad"
            )
        samples.append(TrajectorySample(
            sample_index=len(samples), step_id=step.step_id, phase=step.phase, action=step.action,
            progress=float(progress), joints_rad=resolved, tcp=tcp,
        ))
        if len(samples) > cfg.max_total_samples:
            raise ValueError(f"轨迹采样数超过上限 {cfg.max_total_samples}。")
        previous_joints = resolved

    try:
        for step in plan.steps:
            if step.action.startswith("gripper_"):
                segments.append(SegmentValidation(
                    step.step_id, step.phase, step.action, 0, 0.0, 0.0, "not_applicable", "not_applicable",
                ))
                continue
            assert step.target_joints_rad is not None and step.target_tcp is not None
            segment_start_index = len(samples)
            segment_max_jump = 0.0
            if step.action == "movej":
                max_delta = max(abs(target - start) for target, start in zip(step.target_joints_rad, previous_joints))
                count = max(1, int(math.ceil(max_delta / cfg.max_joint_sample_step_rad)))
                start_joints = previous_joints
                for index in range(1, count + 1):
                    progress = index / count
                    joints = tuple(_lerp(start, target, progress) for start, target in zip(start_joints, step.target_joints_rad))
                    before = previous_joints
                    add_sample(step, progress, joints, step.target_tcp if index == count else None)
                    segment_max_jump = max(segment_max_jump, max(abs(a - b) for a, b in zip(previous_joints, before)))
                segments.append(SegmentValidation(
                    step.step_id, step.phase, step.action, len(samples) - segment_start_index,
                    None, segment_max_jump, "endpoint_only", "endpoint_only",
                ))
            else:
                if previous_tcp is None:
                    raise ValueError(f"{step.step_id} 缺少前一 TCP，无法验证 MoveL 连续路径。")
                dx, dy, dz = step.target_tcp.x - previous_tcp.x, step.target_tcp.y - previous_tcp.y, step.target_tcp.z - previous_tcp.z
                path_length = math.sqrt(dx * dx + dy * dy + dz * dz)
                max_angle = max(
                    abs(_angle_delta(previous_tcp.rz, step.target_tcp.rz)),
                    abs(_angle_delta(previous_tcp.ry, step.target_tcp.ry)),
                    abs(_angle_delta(previous_tcp.rx, step.target_tcp.rx)),
                )
                count = max(
                    1,
                    int(math.ceil(path_length / cfg.max_linear_sample_step_m)),
                    int(math.ceil(max_angle / cfg.max_angular_sample_step_rad)),
                )
                start_tcp = previous_tcp
                for index in range(1, count + 1):
                    progress = index / count
                    tcp = _interpolate_tcp(start_tcp, step.target_tcp, progress)
                    workspace_validator(tcp, f"{step.step_id}_sample_{index}")
                    joints = ik_solver(tcp.to_dict(), previous_joints)
                    before = previous_joints
                    add_sample(step, progress, joints, tcp)
                    segment_max_jump = max(segment_max_jump, max(abs(a - b) for a, b in zip(previous_joints, before)))
                segments.append(SegmentValidation(
                    step.step_id, step.phase, step.action, len(samples) - segment_start_index,
                    path_length, segment_max_jump, "pass", "pass",
                ))
            previous_tcp = step.target_tcp
    except Exception as exc:
        issues.append(str(exc))
        return TrajectoryValidationReport.build(
            plan_id=plan.plan_id,
            kinematic_status="reject",
            config=cfg,
            segments=segments,
            samples=samples,
            issues=issues,
        )

    issues.append("连续运动学采样完成；几何碰撞结论需由调用方附加。")
    return TrajectoryValidationReport.build(
        plan_id=plan.plan_id,
        kinematic_status="pass",
        config=cfg,
        segments=segments,
        samples=samples,
        issues=issues,
    )
