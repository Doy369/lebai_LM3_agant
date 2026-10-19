from __future__ import annotations

import hashlib
import json
import math
import time
from dataclasses import dataclass
from typing import Any, Dict, Mapping, Optional, Sequence, Tuple

from robot_system.llm.types import GraspPose


SCHEMA_VERSION = 1
EXPECTED_PHASES = (
    "prepare_open",
    "approach_pick",
    "descend_pick",
    "grasp",
    "retreat_pick",
    "transfer",
    "descend_place",
    "release",
    "retreat_place",
)
MOVE_ACTIONS = {"movej", "movel"}
GRIPPER_ACTIONS = {"gripper_open", "gripper_close"}


def _finite(value: Any, name: str) -> float:
    number = float(value)
    if not math.isfinite(number):
        raise ValueError(f"{name} 必须是有限数值。")
    return number


def _joints(values: Sequence[Any], name: str) -> Tuple[float, ...]:
    if not isinstance(values, (list, tuple)) or len(values) != 6:
        raise ValueError(f"{name} 必须包含 6 个关节弧度值。")
    return tuple(_finite(value, f"{name}[{index}]") for index, value in enumerate(values))


@dataclass(frozen=True)
class CartesianTarget:
    """Base 坐标系中的 TCP 目标，角度顺序与乐白接口一致。"""

    x: float
    y: float
    z: float
    rz: float
    ry: float
    rx: float

    def __post_init__(self) -> None:
        for name in ("x", "y", "z", "rz", "ry", "rx"):
            object.__setattr__(self, name, _finite(getattr(self, name), name))

    @classmethod
    def from_grasp_pose(cls, pose: GraspPose) -> "CartesianTarget":
        return cls(
            x=pose.position.x,
            y=pose.position.y,
            z=pose.position.z,
            rz=pose.yaw,
            ry=pose.pitch,
            rx=pose.roll,
        )

    @classmethod
    def from_dict(cls, payload: Mapping[str, Any]) -> "CartesianTarget":
        expected = {"x", "y", "z", "rz", "ry", "rx"}
        if set(payload) != expected:
            raise ValueError(f"target_tcp 字段必须为 {sorted(expected)}。")
        return cls(**{name: payload[name] for name in expected})

    def to_dict(self) -> Dict[str, float]:
        return {"x": self.x, "y": self.y, "z": self.z, "rz": self.rz, "ry": self.ry, "rx": self.rx}


@dataclass(frozen=True)
class CompiledPlanStep:
    step_id: str
    phase: str
    action: str
    duration_hint_sec: float
    target_tcp: Optional[CartesianTarget] = None
    target_joints_rad: Optional[Tuple[float, ...]] = None

    def __post_init__(self) -> None:
        if not self.step_id.strip():
            raise ValueError("计划步骤缺少 step_id。")
        if self.phase not in EXPECTED_PHASES:
            raise ValueError(f"未知抓取阶段: {self.phase}")
        if self.action not in MOVE_ACTIONS | GRIPPER_ACTIONS:
            raise ValueError(f"未知计划动作: {self.action}")
        object.__setattr__(self, "duration_hint_sec", _finite(self.duration_hint_sec, "duration_hint_sec"))
        if self.duration_hint_sec <= 0:
            raise ValueError("duration_hint_sec 必须大于 0。")
        if self.action in MOVE_ACTIONS:
            if self.target_tcp is None or self.target_joints_rad is None:
                raise ValueError(f"运动步骤 {self.step_id} 必须同时包含 TCP 和关节目标。")
            object.__setattr__(self, "target_joints_rad", _joints(self.target_joints_rad, "target_joints_rad"))
        elif self.target_tcp is not None or self.target_joints_rad is not None:
            raise ValueError(f"夹爪步骤 {self.step_id} 不能包含运动目标。")

    @classmethod
    def move(
        cls,
        step_id: str,
        phase: str,
        action: str,
        pose: GraspPose,
        joints: Sequence[float],
        duration_hint_sec: float,
    ) -> "CompiledPlanStep":
        return cls(
            step_id=step_id,
            phase=phase,
            action=action,
            duration_hint_sec=duration_hint_sec,
            target_tcp=CartesianTarget.from_grasp_pose(pose),
            target_joints_rad=_joints(joints, "target_joints_rad"),
        )

    @classmethod
    def from_dict(cls, payload: Mapping[str, Any]) -> "CompiledPlanStep":
        expected = {"step_id", "phase", "action", "duration_hint_sec", "target_tcp", "target_joints_rad"}
        if set(payload) != expected:
            raise ValueError(f"计划步骤字段不匹配: {sorted(set(payload) ^ expected)}")
        target_tcp = payload["target_tcp"]
        target_joints = payload["target_joints_rad"]
        return cls(
            step_id=str(payload["step_id"]),
            phase=str(payload["phase"]),
            action=str(payload["action"]),
            duration_hint_sec=payload["duration_hint_sec"],
            target_tcp=CartesianTarget.from_dict(target_tcp) if isinstance(target_tcp, Mapping) else None,
            target_joints_rad=_joints(target_joints, "target_joints_rad") if target_joints is not None else None,
        )

    def to_dict(self) -> Dict[str, Any]:
        return {
            "step_id": self.step_id,
            "phase": self.phase,
            "action": self.action,
            "duration_hint_sec": self.duration_hint_sec,
            "target_tcp": self.target_tcp.to_dict() if self.target_tcp else None,
            "target_joints_rad": list(self.target_joints_rad) if self.target_joints_rad else None,
        }


@dataclass(frozen=True)
class PlanValidation:
    ik: str = "pass"
    workspace: str = "pass"
    joint_limits: str = "pass"
    collision: str = "unknown"
    camera_alignment: str = "pending"

    def __post_init__(self) -> None:
        allowed = {"pass", "fail", "unknown", "pending"}
        for name in ("ik", "workspace", "joint_limits", "collision", "camera_alignment"):
            if getattr(self, name) not in allowed:
                raise ValueError(f"无效验证状态 {name}={getattr(self, name)}")
        if "fail" in (self.ik, self.workspace, self.joint_limits):
            raise ValueError("存在失败的基础验证时不能生成可执行计划。")

    @classmethod
    def from_dict(cls, payload: Mapping[str, Any]) -> "PlanValidation":
        expected = {"ik", "workspace", "joint_limits", "collision", "camera_alignment"}
        if set(payload) != expected:
            raise ValueError("validation 字段不完整。")
        return cls(**{key: str(payload[key]) for key in expected})

    def to_dict(self) -> Dict[str, str]:
        return {
            "ik": self.ik,
            "workspace": self.workspace,
            "joint_limits": self.joint_limits,
            "collision": self.collision,
            "camera_alignment": self.camera_alignment,
        }


@dataclass(frozen=True)
class CompiledGraspPlan:
    plan_id: str
    snapshot_id: str
    target_object_id: str
    calibration_fingerprint: str
    model_version: str
    tool_version: str
    start_joints_rad: Tuple[float, ...]
    steps: Tuple[CompiledPlanStep, ...]
    validation: PlanValidation
    created_at_unix_ms: int
    expires_at_unix_ms: int
    schema_version: int = SCHEMA_VERSION

    def __post_init__(self) -> None:
        if self.schema_version != SCHEMA_VERSION:
            raise ValueError(f"不支持的计划版本: {self.schema_version}")
        object.__setattr__(self, "start_joints_rad", _joints(self.start_joints_rad, "start_joints_rad"))
        object.__setattr__(self, "steps", tuple(self.steps))
        if tuple(step.phase for step in self.steps) != EXPECTED_PHASES:
            raise ValueError("抓取计划必须严格包含接近、下降、夹紧、抬升、搬运、放置和撤离九阶段。")
        if len({step.step_id for step in self.steps}) != len(self.steps):
            raise ValueError("计划步骤 ID 必须唯一。")
        if self.created_at_unix_ms <= 0 or self.expires_at_unix_ms <= self.created_at_unix_ms:
            raise ValueError("计划有效期无效。")
        for name in ("snapshot_id", "target_object_id", "calibration_fingerprint", "model_version", "tool_version"):
            if not str(getattr(self, name)).strip():
                raise ValueError(f"计划缺少 {name}。")

    @classmethod
    def build(
        cls,
        *,
        snapshot_id: str,
        target_object_id: str,
        calibration_fingerprint: str,
        start_joints_rad: Sequence[float],
        steps: Sequence[CompiledPlanStep],
        model_version: str = "Lebai_LM3",
        tool_version: str = "software_tcp_v1",
        validation: Optional[PlanValidation] = None,
        created_at_unix_ms: Optional[int] = None,
        ttl_ms: int = 30_000,
    ) -> "CompiledGraspPlan":
        created = int(created_at_unix_ms if created_at_unix_ms is not None else time.time() * 1000)
        if ttl_ms <= 0:
            raise ValueError("ttl_ms 必须大于 0。")
        provisional = cls(
            plan_id="pending",
            snapshot_id=str(snapshot_id),
            target_object_id=str(target_object_id),
            calibration_fingerprint=str(calibration_fingerprint),
            model_version=str(model_version),
            tool_version=str(tool_version),
            start_joints_rad=_joints(start_joints_rad, "start_joints_rad"),
            steps=tuple(steps),
            validation=validation or PlanValidation(),
            created_at_unix_ms=created,
            expires_at_unix_ms=created + int(ttl_ms),
        )
        return cls(**{**provisional.__dict__, "plan_id": provisional.expected_plan_id()})

    @classmethod
    def from_dict(cls, payload: Mapping[str, Any]) -> "CompiledGraspPlan":
        expected = {
            "schema_version", "plan_id", "snapshot_id", "target_object_id", "calibration_fingerprint",
            "model_version", "tool_version", "start_joints_rad", "steps", "validation",
            "created_at_unix_ms", "expires_at_unix_ms",
        }
        if set(payload) != expected:
            raise ValueError(f"计划字段不匹配: {sorted(set(payload) ^ expected)}")
        if not isinstance(payload["steps"], list) or not isinstance(payload["validation"], Mapping):
            raise ValueError("计划 steps 或 validation 类型无效。")
        plan = cls(
            schema_version=int(payload["schema_version"]),
            plan_id=str(payload["plan_id"]),
            snapshot_id=str(payload["snapshot_id"]),
            target_object_id=str(payload["target_object_id"]),
            calibration_fingerprint=str(payload["calibration_fingerprint"]),
            model_version=str(payload["model_version"]),
            tool_version=str(payload["tool_version"]),
            start_joints_rad=_joints(payload["start_joints_rad"], "start_joints_rad"),
            steps=tuple(CompiledPlanStep.from_dict(item) for item in payload["steps"]),
            validation=PlanValidation.from_dict(payload["validation"]),
            created_at_unix_ms=int(payload["created_at_unix_ms"]),
            expires_at_unix_ms=int(payload["expires_at_unix_ms"]),
        )
        if plan.plan_id != plan.expected_plan_id():
            raise ValueError("计划内容与 plan_id 哈希不一致，拒绝执行。")
        return plan

    def expected_plan_id(self) -> str:
        payload = self.to_dict()
        payload.pop("plan_id", None)
        canonical = json.dumps(payload, ensure_ascii=False, sort_keys=True, separators=(",", ":"), allow_nan=False)
        return "grasp_" + hashlib.sha256(canonical.encode("utf-8")).hexdigest()[:24]

    def assert_not_expired(self, now_unix_ms: Optional[int] = None) -> None:
        now = int(now_unix_ms if now_unix_ms is not None else time.time() * 1000)
        if now > self.expires_at_unix_ms:
            raise PermissionError(f"统一抓取计划已过期: plan_id={self.plan_id}")

    def to_dict(self) -> Dict[str, Any]:
        return {
            "schema_version": self.schema_version,
            "plan_id": self.plan_id,
            "snapshot_id": self.snapshot_id,
            "target_object_id": self.target_object_id,
            "calibration_fingerprint": self.calibration_fingerprint,
            "model_version": self.model_version,
            "tool_version": self.tool_version,
            "start_joints_rad": list(self.start_joints_rad),
            "steps": [step.to_dict() for step in self.steps],
            "validation": self.validation.to_dict(),
            "created_at_unix_ms": self.created_at_unix_ms,
            "expires_at_unix_ms": self.expires_at_unix_ms,
        }
