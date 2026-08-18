from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Any, Dict, List, Optional


@dataclass
class Pose3D:
    """统一描述三维位置。"""

    x: float
    y: float
    z: float

    def to_dict(self) -> Dict[str, float]:
        return {"x": self.x, "y": self.y, "z": self.z}


@dataclass
class GraspPose:
    """
    描述建议抓取位姿。

    约定：
    1. `position` 使用机械臂 Base 坐标系。
    2. `roll/pitch/yaw` 使用弧度。
    3. `approach_vector` 用于表达工具靠近目标的方向。
    """

    position: Pose3D
    roll: float = 0.0
    pitch: float = 3.141592653589793
    yaw: float = 0.0
    approach_vector: Pose3D = field(default_factory=lambda: Pose3D(0.0, 0.0, -1.0))
    pre_grasp_offset_m: float = 0.08

    def to_dict(self) -> Dict[str, Any]:
        return {
            "position": self.position.to_dict(),
            "roll": self.roll,
            "pitch": self.pitch,
            "yaw": self.yaw,
            "approach_vector": self.approach_vector.to_dict(),
            "pre_grasp_offset_m": self.pre_grasp_offset_m,
        }


@dataclass
class ObjectCandidate:
    """
    视觉模块提供给大模型或规则引擎的候选物体。
    """

    object_id: str
    name: str
    position: Pose3D
    confidence: float
    display_name: Optional[str] = None
    color: Optional[str] = None
    size_xyz_m: Optional[Pose3D] = None
    grasp_pose: Optional[GraspPose] = None
    tags: List[str] = field(default_factory=list)
    metadata: Dict[str, Any] = field(default_factory=dict)

    def to_prompt_dict(self) -> Dict[str, Any]:
        payload: Dict[str, Any] = {
            "object_id": self.object_id,
            "name": self.name,
            "display_name": self.display_name or self.name,
            "position_base_m": self.position.to_dict(),
            "confidence": self.confidence,
            "color": self.color,
            "tags": list(self.tags),
            "metadata": dict(self.metadata),
        }
        if self.size_xyz_m is not None:
            payload["size_xyz_m"] = self.size_xyz_m.to_dict()
        if self.grasp_pose is not None:
            payload["grasp_pose_hint"] = self.grasp_pose.to_dict()
        return payload


@dataclass
class AgentDecision:
    """候选物体模式下的大模型决策结果。"""

    success: bool
    selected_object_id: Optional[str]
    selected_object_name: Optional[str]
    grasp_pose: Optional[GraspPose]
    reason: str
    execution_summary: str
    raw_model_response: Optional[str] = None
    matched_filters: List[str] = field(default_factory=list)
    candidate_count: int = 0
    source: str = "rule"
    metadata: Dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> Dict[str, Any]:
        payload = asdict(self)
        if self.grasp_pose is not None:
            payload["grasp_pose"] = self.grasp_pose.to_dict()
        return payload


@dataclass
class ImageTargetPlan:
    """
    基于当前图像的单次目标规划结果。

    这个结果只负责圈出目标与解释原因，真正的 3D 坐标和抓取位姿
    由深度与标定模块进一步计算。
    """

    success: bool
    selected_object_name: Optional[str]
    reason: str
    bbox_xyxy: Optional[List[int]]
    center_uv: Optional[List[int]]
    confidence: float = 0.0
    raw_model_response: Optional[str] = None
    source: str = "qwen"
    metadata: Dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass
class AgentRuntimeConfig:
    """
    Qwen 代理配置。

    默认使用百炼 DashScope 的 OpenAI 兼容接口。
    """

    api_key: Optional[str]
    model: str = "qwen3.5-plus"
    base_url: str = "https://dashscope.aliyuncs.com/compatible-mode/v1"
    timeout_sec: float = 30.0
    temperature: float = 0.1
    top_p: float = 0.8
    max_tokens: int = 700
    enable_model: bool = True
    enable_rule_fallback: bool = True
