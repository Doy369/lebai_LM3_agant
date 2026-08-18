from __future__ import annotations

import math
import time
import uuid
from dataclasses import asdict, dataclass, field
from enum import Enum
from typing import Any, Dict, List, Optional, Tuple


def _assert_probability(value: float, label: str) -> None:
    if not math.isfinite(value) or not 0.0 <= value <= 1.0:
        raise ValueError(f"{label} 必须是 [0, 1] 内的有限值，当前为 {value!r}。")


@dataclass(frozen=True)
class Position3D:
    x: float
    y: float
    z: float

    def __post_init__(self) -> None:
        if not all(math.isfinite(value) for value in (self.x, self.y, self.z)):
            raise ValueError("三维位置不能包含 NaN 或无穷值。")


@dataclass
class WorldObjectState:
    track_id: str
    name: str
    position_base_m: Position3D
    semantic_confidence: float
    depth_confidence: float
    bbox_xyxy: Tuple[int, int, int, int]
    center_uv: Tuple[float, float]
    color: Optional[str] = None
    size_xyz_m: Optional[Position3D] = None
    yaw_rad: float = 0.0
    position_std_m: Position3D = field(default_factory=lambda: Position3D(0.005, 0.005, 0.008))
    reachable: bool = True
    visible: bool = True
    grasped: bool = False
    last_seen_at: float = field(default_factory=time.time)
    metadata: Dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not self.track_id.strip() or not self.name.strip():
            raise ValueError("物体 track_id 和 name 不能为空。")
        _assert_probability(self.semantic_confidence, "semantic_confidence")
        _assert_probability(self.depth_confidence, "depth_confidence")
        x1, y1, x2, y2 = self.bbox_xyxy
        if x2 <= x1 or y2 <= y1:
            raise ValueError(f"目标框不合法: {self.bbox_xyxy}")
        if min(self.position_std_m.x, self.position_std_m.y, self.position_std_m.z) < 0.0:
            raise ValueError("位置标准差不能为负数。")


@dataclass
class RobotState:
    joints_rad: Tuple[float, ...]
    tcp_pose: Tuple[float, float, float, float, float, float]
    robot_state: str
    can_move: bool
    connected: bool
    emergency_stopped: bool = False
    gripper_opening: Optional[float] = None
    metadata: Dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        if len(self.joints_rad) != 6 or len(self.tcp_pose) != 6:
            raise ValueError("当前实验假定六轴机械臂，关节和 TCP 位姿都必须包含 6 个数值。")
        if not all(math.isfinite(value) for value in (*self.joints_rad, *self.tcp_pose)):
            raise ValueError("机器人状态不能包含 NaN 或无穷值。")


@dataclass
class CalibrationState:
    fingerprint: str
    valid: bool
    translation_uncertainty_m: float
    rotation_uncertainty_deg: float
    drift_score: float = 0.0

    def __post_init__(self) -> None:
        if self.translation_uncertainty_m < 0.0 or self.rotation_uncertainty_deg < 0.0:
            raise ValueError("标定不确定性不能为负数。")
        _assert_probability(self.drift_score, "drift_score")


@dataclass
class TaskState:
    user_command: str
    target_track_id: Optional[str]
    retry_count: int = 0
    max_retries: int = 2

    def __post_init__(self) -> None:
        if not self.user_command.strip():
            raise ValueError("用户指令不能为空。")
        if self.retry_count < 0 or self.max_retries < 0:
            raise ValueError("重试次数不能为负数。")


@dataclass
class WorldState:
    objects: List[WorldObjectState]
    robot: RobotState
    calibration: CalibrationState
    task: TaskState
    state_id: str = field(default_factory=lambda: uuid.uuid4().hex)
    timestamp: float = field(default_factory=time.time)
    rgb_snapshot_id: Optional[str] = None
    depth_snapshot_id: Optional[str] = None
    metadata: Dict[str, Any] = field(default_factory=dict)

    def object_by_id(self, track_id: str) -> WorldObjectState:
        for item in self.objects:
            if item.track_id == track_id:
                return item
        raise KeyError(f"世界状态中不存在目标 {track_id!r}。")

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass
class GraspActionCandidate:
    candidate_id: str
    target_track_id: str
    grasp_position_base_m: Position3D
    roll_rad: float
    pitch_rad: float
    yaw_rad: float
    pre_grasp_offset_m: float
    gripper_opening: float
    ik_reachable: bool
    workspace_margin_m: float
    joint_margin_rad: float
    singularity_margin_rad: float
    minimum_clearance_m: float
    path_length_m: float
    source: str = "bbox_depth"
    metadata: Dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        numeric = (
            self.roll_rad,
            self.pitch_rad,
            self.yaw_rad,
            self.pre_grasp_offset_m,
            self.gripper_opening,
            self.workspace_margin_m,
            self.joint_margin_rad,
            self.singularity_margin_rad,
            self.minimum_clearance_m,
            self.path_length_m,
        )
        if not all(math.isfinite(value) for value in numeric):
            raise ValueError("候选动作不能包含 NaN 或无穷值。")
        if min(self.pre_grasp_offset_m, self.path_length_m) < 0.0:
            raise ValueError("预抓取距离和轨迹长度不能为负数。")


@dataclass
class TransitionPrediction:
    candidate_id: str
    success_probability: float
    collision_probability: float
    slip_probability: float
    total_uncertainty: float
    semantic_uncertainty: float
    depth_uncertainty: float
    calibration_uncertainty: float
    motion_uncertainty: float
    expected_motion_cost: float
    reasons: List[str] = field(default_factory=list)

    def __post_init__(self) -> None:
        for label in (
            "success_probability",
            "collision_probability",
            "slip_probability",
            "total_uncertainty",
            "semantic_uncertainty",
            "depth_uncertainty",
            "calibration_uncertainty",
            "motion_uncertainty",
            "expected_motion_cost",
        ):
            _assert_probability(float(getattr(self, label)), label)

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


class DecisionAction(str, Enum):
    EXECUTE = "execute"
    PROBE = "probe"
    REOBSERVE = "reobserve"
    REJECT = "reject"


@dataclass
class DecisionResult:
    action: DecisionAction
    selected_candidate_id: Optional[str]
    score: float
    reason: str
    prediction: Optional[TransitionPrediction]

    def to_dict(self) -> Dict[str, Any]:
        payload = asdict(self)
        payload["action"] = self.action.value
        if self.prediction is not None:
            payload["prediction"] = self.prediction.to_dict()
        return payload


@dataclass
class TrialOutcome:
    success: bool
    failure_type: Optional[str] = None
    target_selected_correctly: Optional[bool] = None
    target_lifted: Optional[bool] = None
    target_placed: Optional[bool] = None
    human_verified: bool = False
    notes: str = ""

    def __post_init__(self) -> None:
        if self.success and self.failure_type:
            raise ValueError("成功试验不能同时包含 failure_type。")

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)
