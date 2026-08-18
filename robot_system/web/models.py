from __future__ import annotations

from typing import Any, Dict, List, Literal, Optional, Tuple

from pydantic import BaseModel, ConfigDict, Field, model_validator


class SafeBaseModel(BaseModel):
    model_config = ConfigDict(allow_inf_nan=False, extra="forbid")


class Pose3DModel(SafeBaseModel):
    """Simple 3D point."""

    x: float
    y: float
    z: float


class GraspPoseModel(SafeBaseModel):
    """Robot grasp or place pose."""

    position: Pose3DModel
    roll: float = 0.0
    pitch: float = 3.141592653589793
    yaw: float = 0.0
    approach_vector: Pose3DModel = Field(default_factory=lambda: Pose3DModel(x=0.0, y=0.0, z=-1.0))
    pre_grasp_offset_m: float = 0.08


class Detection2DRequest(SafeBaseModel):
    """2D detection payload used by the debug pipeline."""

    object_id: str = Field(..., description="Unique candidate ID")
    name: str = Field(..., description="Detected class name, for example apple or cup")
    u: float = Field(..., ge=0.0, description="Pixel u coordinate")
    v: float = Field(..., ge=0.0, description="Pixel v coordinate")
    depth: float = Field(..., gt=0.0, description="Depth at the pixel")
    confidence: float = Field(..., ge=0.0, le=1.0, description="Detection confidence")
    display_name: Optional[str] = Field(default=None, description="Human friendly display name")
    color: Optional[str] = Field(default=None, description="Color hint")
    depth_unit: str = Field(default="m", description="Depth unit, m or mm")
    yaw_rad: float = Field(default=0.0, description="In-plane yaw hint")
    tags: List[str] = Field(default_factory=list, description="Extra tags")
    metadata: Dict[str, Any] = Field(default_factory=dict, description="Additional detector metadata")
    size_xyz_m: Optional[Tuple[float, float, float]] = Field(default=None, description="Estimated object size in meters")


class PlanPickRequest(SafeBaseModel):
    """Decision-only request using precomputed detections."""

    user_command: str = Field(..., min_length=1, max_length=500, description="Natural-language pick instruction")
    detections: List[Detection2DRequest] = Field(default_factory=list, max_length=100, description="Vision candidates")
    scene_context: Dict[str, Any] = Field(default_factory=dict, description="Scene context")


class CameraPlanRequest(SafeBaseModel):
    """Single-shot planning request from the current RGB+Depth frame."""

    user_command: str = Field(..., min_length=1, max_length=500, description="Natural-language pick instruction")
    scene_context: Dict[str, Any] = Field(default_factory=dict, description="Scene context")


class SafetyProbeRequestModel(SafeBaseModel):
    """Debug safety-probe request using precomputed detections."""

    user_command: str = Field(..., min_length=1, max_length=500, description="Natural-language pick instruction")
    detections: List[Detection2DRequest] = Field(default_factory=list, max_length=100, description="Vision candidates")
    scene_context: Dict[str, Any] = Field(default_factory=dict, description="Scene context")
    connect_robot: bool = Field(default=True, description="Whether to connect and execute on the robot")
    descend_clearance_m: float = Field(default=0.05, ge=0.0, description="Probe clearance above the target")
    hover_offset_m: Optional[float] = Field(default=None, ge=0.0, description="Probe hover offset")


class ExecutePickRequest(SafeBaseModel):
    """Debug full-pick request using precomputed detections."""

    user_command: str = Field(..., min_length=1, max_length=500, description="Natural-language pick instruction")
    detections: List[Detection2DRequest] = Field(default_factory=list, max_length=100, description="Vision candidates")
    scene_context: Dict[str, Any] = Field(default_factory=dict, description="Scene context")
    execute_motion: bool = Field(default=True, description="Whether to send motion commands")
    drop_pose_override: Optional[GraspPoseModel] = Field(default=None, description="Override the default place pose")
    pick_pre_grasp_offset_m: Optional[float] = Field(default=None, ge=0.0, description="Override pick hover offset")
    pick_post_grasp_offset_m: float = Field(default=0.10, ge=0.0, description="Pick retreat offset")
    place_pre_place_offset_m: Optional[float] = Field(default=None, ge=0.0, description="Override place hover offset")
    place_post_place_offset_m: float = Field(default=0.10, ge=0.0, description="Place retreat offset")


class CameraGraspRequest(SafeBaseModel):
    """Camera-based probe or pick request."""

    user_command: str = Field(..., min_length=1, max_length=500, description="Natural-language pick instruction")
    mode: Literal["probe", "pick"] = Field(default="probe", description="Whether to execute a safety probe or a full pick")
    probe_lock_id: Optional[str] = Field(default=None, description="Lock ID returned by the latest successful safety probe")
    scene_context: Dict[str, Any] = Field(default_factory=dict, description="Scene context")
    descend_clearance_m: float = Field(default=0.05, ge=0.0, description="Probe clearance above the target")
    hover_offset_m: Optional[float] = Field(default=0.08, ge=0.0, description="Hover offset before probe or pick")
    drop_pose_override: Optional[GraspPoseModel] = Field(default=None, description="Override the default place pose")
    pick_pre_grasp_offset_m: Optional[float] = Field(default=None, ge=0.0, description="Override pick hover offset")
    pick_post_grasp_offset_m: float = Field(default=0.10, ge=0.0, description="Pick retreat offset")
    place_pre_place_offset_m: Optional[float] = Field(default=None, ge=0.0, description="Override place hover offset")
    place_post_place_offset_m: float = Field(default=0.10, ge=0.0, description="Place retreat offset")


class AutoCalibrationRunRequest(SafeBaseModel):
    """Automatic calibration request."""

    sample_count: int = Field(default=20, ge=8, le=40, description="Planned pose count")
    min_success_samples: int = Field(default=12, ge=6, le=40, description="Minimum valid samples before solving")
    settle_sec: float = Field(default=0.8, ge=0.0, le=5.0, description="Wait time after each move")
    execute_motion: bool = Field(default=True, description="Whether to move the real robot")
    run_calibration: bool = Field(default=True, description="Whether to run the calibration solver after sampling")
    auto_start_system: bool = Field(default=False, description="Whether to call start_sys before the run")
    session_name: Optional[str] = Field(default=None, description="Optional output session name")

    @model_validator(mode="after")
    def validate_success_target(self) -> "AutoCalibrationRunRequest":
        if self.min_success_samples > self.sample_count:
            raise ValueError("min_success_samples 不能大于 sample_count。")
        return self


class JogTCPRequest(SafeBaseModel):
    """Small TCP jog request."""

    dx: float = Field(default=0.0, ge=-0.05, le=0.05)
    dy: float = Field(default=0.0, ge=-0.05, le=0.05)
    dz: float = Field(default=0.0, ge=-0.05, le=0.05)
    drz: float = Field(default=0.0, ge=-0.20, le=0.20)
    dry: float = Field(default=0.0, ge=-0.20, le=0.20)
    drx: float = Field(default=0.0, ge=-0.20, le=0.20)
    linear: bool = True
    label: str = "web_jog_tcp"


class GenericMessageResponse(SafeBaseModel):
    """Generic API response wrapper."""

    success: bool
    message: str
    data: Dict[str, Any] = Field(default_factory=dict)
