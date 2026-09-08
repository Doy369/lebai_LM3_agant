from __future__ import annotations

import base64
import copy
import functools
import hashlib
import json
import math
import os
import subprocess
import sys
import threading
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional

import cv2
import numpy as np

from robot_system.calibration import AutoCalibrationConfig, AutoCalibrationRunner
from robot_system.control import (
    GripperConfig,
    LebaiController,
    LebaiControllerConfig,
    MotionProfile,
    PickPlaceRequest,
    SafetyProbeRequest,
    WorkspaceBox,
)
from robot_system.llm import QwenPickAgent
from robot_system.llm.types import GraspPose, ImageTargetPlan, Pose3D
from robot_system.pipeline import PickExecutor, PickExecutorConfig
from robot_system.vision import CalibrationBundle, Detection2D, OrbbecCameraConfig, OrbbecDepthCamera, VisionTransformer

from .camera_feed import CameraFeed, CameraFeedConfig
from .models import (
    AutoCalibrationRunRequest,
    CameraGraspRequest,
    CameraPlanRequest,
    Detection2DRequest,
    ExecutePickRequest,
    GraspPoseModel,
    JogTCPRequest,
    PlanPickRequest,
    Pose3DModel,
    SafetyProbeRequestModel,
)


def _parse_bool(value: Optional[str], default: bool) -> bool:
    if value is None:
        return default
    return value.strip().lower() not in {"0", "false", "no", "off"}


def _parse_float_env(name: str, default: float) -> float:
    raw = os.getenv(name)
    if raw is None or raw.strip() == "":
        return default
    return float(raw)


def _parse_int_env(name: str, default: int) -> int:
    raw = os.getenv(name)
    if raw is None or raw.strip() == "":
        return default
    return int(raw)


def _parse_home_joint_pose(raw_value: Optional[str]) -> Optional[tuple[float, float, float, float, float, float]]:
    if raw_value is None or raw_value.strip() == "":
        return None
    values = [float(item.strip()) for item in raw_value.split(",") if item.strip()]
    if len(values) != 6:
        raise ValueError("LEBAI_HOME_JOINT_POSE 必须是 6 个以逗号分隔的关节角。")
    return tuple(values)  # type: ignore[return-value]


def _parse_cors_origins(raw_value: Optional[str]) -> List[str]:
    if raw_value is None or raw_value.strip() == "":
        return []
    return [item.strip() for item in raw_value.split(",") if item.strip()]


def _parse_real_motion_confirmation(value: Optional[str]) -> bool:
    return bool(value and value.strip().upper() == "YES")


def _exclusive_motion(method: Any) -> Any:
    """Serialize all ordinary motion operations for one Web service instance."""

    @functools.wraps(method)
    def wrapper(self: "RobotWebService", *args: Any, **kwargs: Any) -> Any:
        if not self._motion_lock.acquire(blocking=False):
            raise PermissionError("已有机械臂运动任务正在执行，请等待完成或先执行停止运动。")
        try:
            return method(self, *args, **kwargs)
        finally:
            self._motion_lock.release()

    return wrapper


def _safe_sdk_call(obj: Any, method_name: str) -> Any:
    method = getattr(obj, method_name, None)
    if not callable(method):
        return None
    try:
        return method()
    except BaseException as exc:
        return f"<error: {exc}>"


def _build_default_drop_pose() -> GraspPose:
    return GraspPose(
        position=Pose3D(-0.30, -0.22, 0.15),
        roll=0.0,
        pitch=math.pi,
        yaw=0.0,
        approach_vector=Pose3D(0.0, 0.0, -1.0),
        pre_grasp_offset_m=0.10,
    )


def _parse_drop_pose_from_env() -> GraspPose:
    default = _build_default_drop_pose()
    return GraspPose(
        position=Pose3D(
            _parse_float_env("LEBAI_DROP_X", default.position.x),
            _parse_float_env("LEBAI_DROP_Y", default.position.y),
            _parse_float_env("LEBAI_DROP_Z", default.position.z),
        ),
        roll=_parse_float_env("LEBAI_DROP_ROLL", default.roll),
        pitch=_parse_float_env("LEBAI_DROP_PITCH", default.pitch),
        yaw=_parse_float_env("LEBAI_DROP_YAW", default.yaw),
        approach_vector=Pose3D(
            _parse_float_env("LEBAI_DROP_APPROACH_X", default.approach_vector.x),
            _parse_float_env("LEBAI_DROP_APPROACH_Y", default.approach_vector.y),
            _parse_float_env("LEBAI_DROP_APPROACH_Z", default.approach_vector.z),
        ),
        pre_grasp_offset_m=_parse_float_env("LEBAI_DROP_PRE_GRASP_OFFSET_M", default.pre_grasp_offset_m),
    )


@dataclass
class WebBackendConfig:
    """Web 后端运行配置。"""

    app_title: str = "Lebai Robot Web API"
    app_version: str = "0.2.0"
    calibration_file: Path = Path("biaoding/eye_to_hand_result.json")
    robot_ip: str = "127.0.0.1"
    dry_run: bool = True
    cors_origins: List[str] = field(default_factory=list)
    web_token: Optional[str] = None
    allow_real_motion: bool = False
    allow_debug_motion: bool = False
    probe_lock_ttl_sec: float = 30.0
    probe_lock_joint_tolerance_rad: float = 0.15
    workspace: WorkspaceBox = field(default_factory=WorkspaceBox)
    motion: MotionProfile = field(default_factory=MotionProfile)
    home_joint_pose: Optional[tuple[float, float, float, float, float, float]] = None
    runtime_state_file: Path = Path("biaoding/web_runtime_state.json")
    preview_camera: CameraFeedConfig = field(default_factory=CameraFeedConfig)
    orbbec_camera: OrbbecCameraConfig = field(default_factory=OrbbecCameraConfig)
    drop_pose: GraspPose = field(default_factory=_build_default_drop_pose)
    default_probe_descend_clearance_m: float = 0.05
    default_probe_hover_offset_m: float = 0.08
    tool_surface_offset_m: float = 0.185

    def __post_init__(self) -> None:
        if not self.dry_run and not self.allow_real_motion:
            raise RuntimeError(
                "真机模式需要显式设置 LEBAI_ALLOW_REAL_MOTION=YES；未确认时后端拒绝启动。"
            )
        if not self.dry_run and not self.web_token:
            raise RuntimeError(
                "真机模式需要设置 LEBAI_WEB_TOKEN，用于保护机械臂控制接口。"
            )
        if self.probe_lock_ttl_sec <= 0.0:
            raise ValueError("probe_lock_ttl_sec 必须大于 0。")
        if self.probe_lock_joint_tolerance_rad <= 0.0:
            raise ValueError("probe_lock_joint_tolerance_rad 必须大于 0。")

    @classmethod
    def from_env(cls) -> "WebBackendConfig":
        return cls(
            app_title=os.getenv("LEBAI_WEB_TITLE", "Lebai Robot Web API"),
            app_version=os.getenv("LEBAI_WEB_VERSION", "0.2.0"),
            calibration_file=Path(os.getenv("LEBAI_WEB_CALIBRATION_FILE", "biaoding/eye_to_hand_result.json")),
            robot_ip=os.getenv("LEBAI_ROBOT_IP", "127.0.0.1"),
            dry_run=_parse_bool(os.getenv("LEBAI_DRY_RUN"), True),
            cors_origins=_parse_cors_origins(os.getenv("LEBAI_WEB_CORS_ORIGINS")),
            web_token=(os.getenv("LEBAI_WEB_TOKEN") or "").strip() or None,
            allow_real_motion=_parse_real_motion_confirmation(os.getenv("LEBAI_ALLOW_REAL_MOTION")),
            allow_debug_motion=_parse_bool(os.getenv("LEBAI_ALLOW_DEBUG_MOTION"), False),
            probe_lock_ttl_sec=_parse_float_env("LEBAI_PROBE_LOCK_TTL_SEC", 30.0),
            probe_lock_joint_tolerance_rad=_parse_float_env(
                "LEBAI_PROBE_LOCK_JOINT_TOLERANCE_RAD", 0.15
            ),
            workspace=WorkspaceBox(
                x_min=_parse_float_env("LEBAI_WS_X_MIN", -0.75),
                x_max=_parse_float_env("LEBAI_WS_X_MAX", -0.15),
                y_min=_parse_float_env("LEBAI_WS_Y_MIN", -0.45),
                y_max=_parse_float_env("LEBAI_WS_Y_MAX", 0.45),
                z_min=_parse_float_env("LEBAI_WS_Z_MIN", 0.02),
                z_max=_parse_float_env("LEBAI_WS_Z_MAX", 0.55),
            ),
            motion=MotionProfile(
                joint_acc=_parse_float_env("LEBAI_JOINT_ACC", 0.6),
                joint_vel=_parse_float_env("LEBAI_JOINT_VEL", 0.35),
                linear_acc=_parse_float_env("LEBAI_LINEAR_ACC", 0.18),
                linear_vel=_parse_float_env("LEBAI_LINEAR_VEL", 0.08),
                move_time=_parse_float_env("LEBAI_MOVE_TIME", 0.0),
                blend_radius=_parse_float_env("LEBAI_BLEND_RADIUS", 0.0),
            ),
            home_joint_pose=_parse_home_joint_pose(os.getenv("LEBAI_HOME_JOINT_POSE")),
            runtime_state_file=Path(os.getenv("LEBAI_WEB_RUNTIME_STATE_FILE", "biaoding/web_runtime_state.json")),
            preview_camera=CameraFeedConfig(
                camera_id=_parse_int_env("LEBAI_CAMERA_ID", 0),
                backend_mode=os.getenv("LEBAI_CAMERA_BACKEND", "auto"),
                frame_width=_parse_int_env("LEBAI_CAMERA_WIDTH", 1280),
                frame_height=_parse_int_env("LEBAI_CAMERA_HEIGHT", 720),
                jpeg_quality=_parse_int_env("LEBAI_CAMERA_JPEG_QUALITY", 90),
                read_retry_count=_parse_int_env("LEBAI_CAMERA_READ_RETRY_COUNT", 2),
                warmup_delay_sec=_parse_float_env("LEBAI_CAMERA_WARMUP_DELAY_SEC", 0.12),
            ),
            orbbec_camera=OrbbecCameraConfig(
                color_width=_parse_int_env("LEBAI_ORBBEC_COLOR_WIDTH", 1280),
                color_height=_parse_int_env("LEBAI_ORBBEC_COLOR_HEIGHT", 720),
                color_fps=_parse_int_env("LEBAI_ORBBEC_COLOR_FPS", 30),
                depth_width=_parse_int_env("LEBAI_ORBBEC_DEPTH_WIDTH", 848),
                depth_height=_parse_int_env("LEBAI_ORBBEC_DEPTH_HEIGHT", 480),
                depth_fps=_parse_int_env("LEBAI_ORBBEC_DEPTH_FPS", 30),
                align_mode=os.getenv("LEBAI_ORBBEC_ALIGN_MODE", "filter"),
                enable_frame_sync=_parse_bool(os.getenv("LEBAI_ORBBEC_ENABLE_FRAME_SYNC"), True),
                wait_timeout_ms=_parse_int_env("LEBAI_ORBBEC_WAIT_TIMEOUT_MS", 1500),
                jpeg_quality=_parse_int_env("LEBAI_ORBBEC_JPEG_QUALITY", 90),
                depth_neighborhood_window=_parse_int_env("LEBAI_ORBBEC_DEPTH_WINDOW", 7),
            ),
            drop_pose=_parse_drop_pose_from_env(),
            default_probe_descend_clearance_m=_parse_float_env("LEBAI_DEFAULT_PROBE_DESCEND_CLEARANCE_M", 0.05),
            default_probe_hover_offset_m=_parse_float_env("LEBAI_DEFAULT_PROBE_HOVER_OFFSET_M", 0.08),
            tool_surface_offset_m=_parse_float_env("LEBAI_TOOL_SURFACE_OFFSET_M", 0.185),
        )


@dataclass
class ProbeLockState:
    """In-memory lock created by the latest successful safety probe."""

    probe_lock_id: str
    created_at: float
    snapshot_id: str
    plan_data: Dict[str, Any]
    execution_target_pose: Dict[str, Any]
    execution_target_joints: List[float]
    robot_joint_snapshot: Optional[List[float]]
    calibration_fingerprint: str


class RobotWebService:
    """Web 层服务对象。"""

    def __init__(self, config: Optional[WebBackendConfig] = None) -> None:
        self.config = config or WebBackendConfig.from_env()
        self.preview_camera = CameraFeed(self.config.preview_camera)
        self.depth_camera = OrbbecDepthCamera(self.config.orbbec_camera)
        self._vision_transformer: Optional[VisionTransformer] = None
        self._llm_agent: Optional[QwenPickAgent] = None
        self._latest_plan_image_jpeg: Optional[bytes] = None
        self._latest_snapshot_jpeg: Optional[bytes] = None
        self._latest_snapshot_id: Optional[str] = None
        self._latest_probe_lock: Optional[ProbeLockState] = None
        self._camera_io_lock = threading.Lock()
        self._auto_calibration_lock = threading.Lock()
        self._motion_lock = threading.Lock()
        self._probe_lock_guard = threading.Lock()
        self._last_auto_calibration_result: Optional[Dict[str, Any]] = None
        self._runtime_state = self._load_runtime_state()
        saved_home = self._runtime_state.get("home_joint_pose")
        if self.config.home_joint_pose is None and isinstance(saved_home, list) and len(saved_home) == 6:
            self.config.home_joint_pose = tuple(float(value) for value in saved_home)  # type: ignore[assignment]
        elif self.config.home_joint_pose is not None:
            self._runtime_state["home_joint_pose"] = [float(value) for value in self.config.home_joint_pose]

    def health(self) -> Dict[str, Any]:
        calibration_exists = self.config.calibration_file.exists()
        return {
            "success": True,
            "service": self.config.app_title,
            "version": self.config.app_version,
            "dry_run": self.config.dry_run,
            "robot_ip": self.config.robot_ip,
            "calibration_file": str(self.config.calibration_file),
            "calibration_file_exists": calibration_exists,
        }

    def calibration_status(self) -> Dict[str, Any]:
        if not self.config.calibration_file.exists():
            return {"success": False, "message": f"未找到标定文件: {self.config.calibration_file}", "data": {}}

        bundle = CalibrationBundle.from_file(self.config.calibration_file)
        metadata = bundle.metadata
        return {
            "success": True,
            "message": "标定文件加载成功。",
            "data": {
                "path": str(self.config.calibration_file),
                "image_size_wh": metadata.get("image_size_wh"),
                "mode": metadata.get("mode"),
                "camera_mount": metadata.get("camera_mount"),
                "target_mount": metadata.get("target_mount"),
                "board": metadata.get("board"),
                "selected_hand_eye_method": metadata.get("hand_eye", {}).get("selected_method"),
                "samples": metadata.get("samples", {}),
                "camera_matrix": bundle.camera_matrix.tolist(),
                "dist_coeffs": bundle.dist_coeffs.reshape(-1).tolist(),
                "base_T_camera": bundle.base_T_camera.tolist(),
            },
        }

    def auto_calibration_plan(self) -> Dict[str, Any]:
        runner = AutoCalibrationRunner(
            controller=self._build_motion_controller(),
            camera=self.depth_camera,
            config=AutoCalibrationConfig(),
        )
        return {
            "success": True,
            "message": "自动标定计划已生成。",
            "data": {
                **runner.preview_plan(),
                "robot_ip": self.config.robot_ip,
                "workspace": asdict(self.config.workspace),
                "last_result": self._last_auto_calibration_result,
            },
        }

    @_exclusive_motion
    def run_auto_calibration(self, request: AutoCalibrationRunRequest) -> Dict[str, Any]:
        if not self._auto_calibration_lock.acquire(blocking=False):
            raise RuntimeError("已有自动标定任务正在运行，请等待当前任务结束。")

        controller = self._build_motion_controller()
        auto_config = AutoCalibrationConfig(
            data_root=Path("calib_data"),
            output_dir=self.config.calibration_file.parent,
            session_name=request.session_name,
            sample_count=int(request.sample_count),
            min_success_samples=int(request.min_success_samples),
            settle_sec=float(request.settle_sec),
            execute_motion=bool(request.execute_motion),
            auto_start_system=bool(request.auto_start_system),
            run_calibration=bool(request.run_calibration),
        )
        runner = AutoCalibrationRunner(
            controller=controller,
            camera=self.depth_camera,
            config=auto_config,
        )
        try:
            with self._camera_io_lock:
                self.preview_camera.close()
                result = runner.run()
            self._vision_transformer = None
            self._last_auto_calibration_result = result
            return {
                "success": bool(result.get("success")),
                "message": result.get("message") or "自动标定完成。",
                "data": result,
            }
        finally:
            self._auto_calibration_lock.release()

    def camera_status(self) -> Dict[str, Any]:
        if self.depth_camera.sdk_available:
            preview_status = {
                "stream_ready": True,
                "stream_url": "/api/camera/frame.jpg",
                "stream_type": "rgb",
                "camera_id": self.config.preview_camera.camera_id,
                "backend_mode": self.config.preview_camera.backend_mode,
                "requested_size_wh": [self.config.orbbec_camera.color_width, self.config.orbbec_camera.color_height],
                "last_error": None,
                "open_info": {},
                "source": "orbbec_sdk_on_demand",
                "image_source_for_grasp": "orbbec_sdk_on_demand",
                "note": "页面预览与当前画面抓取共用 Orbbec SDK 单帧抓取。",
            }
        else:
            with self._camera_io_lock:
                preview_status = self.preview_camera.get_status()
            preview_status["source"] = "opencv_videocapture"
            preview_status["image_source_for_grasp"] = "opencv_videocapture"
        depth_status: Dict[str, Any] = {
            "sdk_available": self.depth_camera.sdk_available,
            "stream_ready": False,
            "open_on_demand": True,
            "align_mode": self.config.orbbec_camera.align_mode,
            "requested_color_wh": [
                self.config.orbbec_camera.color_width,
                self.config.orbbec_camera.color_height,
            ],
            "requested_depth_wh": [
                self.config.orbbec_camera.depth_width,
                self.config.orbbec_camera.depth_height,
            ],
            "last_error": None if self.depth_camera.sdk_available else "未安装 pyorbbecsdk，无法读取真实 RGB+Depth。",
            "note": "页面实时预览默认走普通 RGB。只有点击“当前画面规划/抓取”时，才会按需打开 Orbbec 深度链路。",
        }

        return {
            "success": True,
            "message": "相机状态读取完成。",
            "data": {
                "preview": preview_status,
                "depth_camera": depth_status,
                "latest_snapshot_id": self._latest_snapshot_id,
                "latest_plan_image_url": "/api/camera/latest-plan.jpg",
            },
        }

    def orbbec_self_test(self) -> Dict[str, Any]:
        config_payload = asdict(self.config.orbbec_camera)
        result: Dict[str, Any] = {
            "sdk_available": self.depth_camera.sdk_available,
            "orbbec_config": config_payload,
            "subprocess_returncode": None,
            "subprocess_stderr": "",
            "device_count": 0,
            "devices": [],
            "enumeration": {"success": False, "error": None},
            "snapshot_test": {"success": False, "error": None},
        }

        if not self.depth_camera.sdk_available:
            message = "未安装 pyorbbecsdk，无法执行 Orbbec 自检。"
            result["enumeration"]["error"] = message
            result["snapshot_test"]["error"] = message
            return {"success": False, "message": message, "data": result}

        script = f"""
import json
from robot_system.vision import OrbbecCameraConfig, OrbbecDepthCamera

payload = {{
    "sdk_available": True,
    "device_count": 0,
    "devices": [],
    "enumeration": {{"success": False, "error": None}},
    "snapshot_test": {{"success": False, "error": None}},
}}

try:
    from pyorbbecsdk import Context
    ctx = Context()
    device_list = ctx.query_devices()
    payload["device_count"] = int(device_list.get_count())
    payload["enumeration"]["success"] = True
    for index in range(payload["device_count"]):
        try:
            dev = device_list.get_device_by_index(index)
            info = dev.get_device_info()
            payload["devices"].append({{
                "index": index,
                "name": info.get_name(),
                "pid": info.get_pid(),
                "serial_number": info.get_serial_number(),
            }})
        except BaseException as exc:
            payload["devices"].append({{"index": index, "error": str(exc)}})
except BaseException as exc:
    payload["enumeration"]["error"] = str(exc)
    print(json.dumps(payload, ensure_ascii=False))
    raise SystemExit(0)

camera = OrbbecDepthCamera(OrbbecCameraConfig(**json.loads({json.dumps(json.dumps(config_payload, ensure_ascii=False))})))
try:
    snapshot = camera.capture_snapshot()
    center_u = snapshot.image_size_wh[0] // 2
    center_v = snapshot.image_size_wh[1] // 2
    payload["snapshot_test"] = {{
        "success": True,
        "error": None,
        "snapshot_id": snapshot.snapshot_id,
        "timestamp_ms": snapshot.timestamp_ms,
        "image_size_wh": list(snapshot.image_size_wh),
        "depth_shape": list(snapshot.depth_m.shape),
        "center_uv": [center_u, center_v],
        "center_depth_m": snapshot.get_depth_at_pixel(center_u, center_v),
        "open_info": dict(getattr(camera, "_open_info", {{}})),
    }}
except BaseException as exc:
    payload["snapshot_test"]["error"] = str(exc)
finally:
    try:
        camera.close()
    except BaseException:
        pass

print(json.dumps(payload, ensure_ascii=False))
"""

        with self._camera_io_lock:
            self.preview_camera.close()
            self.depth_camera.close()
            completed = subprocess.run(
                [sys.executable, "-c", script],
                cwd=str(Path.cwd()),
                capture_output=True,
                timeout=25,
            )
        result["subprocess_returncode"] = completed.returncode
        stdout = self._decode_subprocess_output(completed.stdout)
        stderr = self._decode_subprocess_output(completed.stderr)
        result["subprocess_stderr"] = stderr.strip()

        if stdout.strip():
            lines = [line for line in stdout.splitlines() if line.strip()]
            for line in reversed(lines):
                try:
                    parsed = json.loads(line)
                    if isinstance(parsed, dict):
                        result.update(parsed)
                        break
                except json.JSONDecodeError:
                    continue

        success = bool(result.get("enumeration", {}).get("success")) and bool(result.get("snapshot_test", {}).get("success"))
        if success:
            return {"success": True, "message": "Orbbec 自检通过，已成功抓取一帧 RGB+Depth。", "data": result}

        if completed.returncode != 0 and not result["snapshot_test"].get("error"):
            result["snapshot_test"]["error"] = stderr.strip() or "Orbbec 自检子进程异常退出。"
        return {"success": False, "message": "Orbbec 自检失败，请查看返回的枚举和抓帧诊断信息。", "data": result}

    def camera_frame_jpeg(self) -> bytes:
        with self._camera_io_lock:
            if self.depth_camera.sdk_available:
                self.preview_camera.close()
                self.depth_camera.close()
                try:
                    return self.depth_camera.capture_preview_jpeg()
                finally:
                    self.depth_camera.close()
            return self.preview_camera.capture_frame_jpeg()

    def latest_plan_frame_jpeg(self) -> bytes:
        if self._latest_plan_image_jpeg is not None:
            return self._latest_plan_image_jpeg
        if self._latest_snapshot_jpeg is not None:
            return self._latest_snapshot_jpeg
        raise FileNotFoundError("尚未生成基于当前画面的抓取规划。")
    def robot_status(self) -> Dict[str, Any]:
        controller = self._build_status_controller()
        try:
            controller.connect()
            status = controller.get_robot_status_summary()
            kin_data = controller.get_kin_data()
            # Server read completion time, not a hardware acquisition timestamp.
            feedback_read_at_unix_ms = time.time_ns() // 1_000_000
            saved_home_joint_pose = self._effective_home_joint_pose()
            return {
                "success": True,
                "message": "机器人状态读取成功。",
                "data": {
                    "dry_run": self.config.dry_run,
                    "robot_ip": self.config.robot_ip,
                    "workspace": asdict(self.config.workspace),
                    "motion": asdict(self.config.motion),
                    "home_joint_pose": saved_home_joint_pose,
                    "home_tcp_pose": self._runtime_state.get("home_tcp_pose"),
                    "teach_mode_requested": bool(self._runtime_state.get("teach_mode_requested", False)),
                    "runtime_state_file": str(self.config.runtime_state_file),
                    "robot_state": status.get("robot_state"),
                    "can_move": status.get("can_move"),
                    "estop_reason": status.get("estop_reason"),
                    "requires_manual_enable": status.get("requires_manual_enable"),
                    "is_connected": status.get("is_connected"),
                    "kin_data": kin_data,
                    "feedback_read_at_unix_ms": feedback_read_at_unix_ms,
                    "feedback_time_source": "server_read_completion",
                    "joint_unit": "radian",
                    "command_log": controller.command_log,
                },
            }
        finally:
            controller.disconnect()

    def plan_pick(self, request: PlanPickRequest) -> Dict[str, Any]:
        executor = self._build_executor()
        result = executor.execute_from_detections(
            user_command=request.user_command,
            detections=self._convert_detections(request.detections),
            scene_context=dict(request.scene_context),
            execute_motion=False,
        )
        return {
            "success": result.success,
            "message": result.message,
            "data": {
                "mode": "debug_detections",
                "user_command": result.user_command,
                "candidates": result.candidates,
                "decision": result.decision,
                "execution": result.execution,
            },
        }

    def plan_from_camera(self, request: CameraPlanRequest) -> Dict[str, Any]:
        plan_data = self._plan_from_camera_internal(
            user_command=request.user_command,
            scene_context=dict(request.scene_context),
        )
        return {"success": plan_data["success"], "message": plan_data["message"], "data": plan_data}

    def world_model_status(self) -> Dict[str, Any]:
        """返回结构化世界模型实验的只读状态，不连接任何设备。"""
        from world_model_experiment.world_model import OfflinePreflight

        project_root = Path(__file__).resolve().parents[2]
        experiment_root = project_root / "world_model_experiment"
        preflight = OfflinePreflight(project_root, experiment_root).run()

        latest_state = None
        latest_state_path = None
        for name in (
            "latest_camera_world_state.json",
            "latest_integrated_state.json",
            "latest_world_state.json",
        ):
            candidate = experiment_root / "data" / name
            if not candidate.exists():
                continue
            try:
                latest_state = json.loads(candidate.read_text(encoding="utf-8"))
                latest_state_path = str(candidate)
                break
            except Exception:
                continue

        trials_root = experiment_root / "trials"
        recent_trials: List[Dict[str, Any]] = []
        if trials_root.exists():
            trial_dirs = sorted(
                (item for item in trials_root.iterdir() if item.is_dir()),
                key=lambda item: item.stat().st_mtime,
                reverse=True,
            )[:8]
            recent_trials = [
                {
                    "name": item.name,
                    "updated_at": item.stat().st_mtime,
                    "has_world_state": (item / "world_state_before.json").exists(),
                    "has_decision": (item / "decision.json").exists(),
                    "has_outcome": (item / "outcome.json").exists(),
                }
                for item in trial_dirs
            ]

        return {
            "success": True,
            "message": "结构化世界模型实验状态读取成功。",
            "data": {
                "experiment_root": str(experiment_root),
                "preflight": preflight.to_dict(),
                "latest_state_path": latest_state_path,
                "latest_state": latest_state,
                "recent_trials": recent_trials,
                "runtime_safety": {
                    "backend_dry_run": self.config.dry_run,
                    "real_motion_configured": bool(self.config.allow_real_motion and not self.config.dry_run),
                    "world_model_plan_sends_motion": False,
                    "probe_lock_required_for_pick": True,
                },
            },
        }

    def world_model_plan_from_camera(self, request: CameraPlanRequest) -> Dict[str, Any]:
        """真实RGB-D感知＋世界模型Dry-run评价；不会连接或驱动机械臂。"""
        from world_model_experiment.world_model import (
            DryRunCandidateGenerator,
            ExistingProjectAdapter,
            RuleWorldModel,
            RuleWorldModelConfig,
            TrialLogger,
            WorldStateStore,
        )

        plan_data = self._plan_from_camera_internal(
            user_command=request.user_command,
            scene_context={
                **dict(request.scene_context),
                "experiment": "structured_world_model",
                "motion_allowed": False,
            },
        )
        if not plan_data.get("success"):
            return {
                "success": False,
                "message": plan_data.get("message") or "当前画面规划失败。",
                "data": {
                    "mode": "world_model_camera_only",
                    "motion_sent": False,
                    "source_plan": plan_data,
                },
            }

        project_root = Path(__file__).resolve().parents[2]
        experiment_root = project_root / "world_model_experiment"
        controller = LebaiController(
            LebaiControllerConfig(
                robot_ip="world-model-dry-run-only",
                workspace=self.config.workspace,
                motion=self.config.motion,
                dry_run=True,
                auto_start_system=False,
                gripper=GripperConfig(init_on_connect=False),
            )
        )
        adapter = ExistingProjectAdapter(self.config.calibration_file)
        state = adapter.build_state_from_camera_plan(request.user_command, plan_data, controller)
        candidates = DryRunCandidateGenerator(controller).generate(state)
        model_config = RuleWorldModelConfig.from_json(experiment_root / "config" / "default.json")
        model = RuleWorldModel(model_config)
        decision = model.decide(state, candidates)

        evaluations = []
        for candidate in candidates:
            prediction = model.predict(state, candidate)
            evaluations.append(
                {
                    "candidate": asdict(candidate),
                    "prediction": prediction.to_dict(),
                    "selected": candidate.candidate_id == decision.selected_candidate_id,
                }
            )

        logger = TrialLogger(experiment_root / "trials")
        trial_dir = logger.start_trial("web_world_model")
        logger.record_planning(trial_dir, state, candidates, decision)
        logger.write_json(trial_dir, "source_camera_plan.json", plan_data)
        if self._latest_snapshot_jpeg is not None:
            (trial_dir / "snapshot.jpg").write_bytes(self._latest_snapshot_jpeg)
        if self._latest_plan_image_jpeg is not None:
            (trial_dir / "annotated_plan.jpg").write_bytes(self._latest_plan_image_jpeg)
        WorldStateStore(experiment_root / "data" / "latest_camera_world_state.json").save(state)

        return {
            "success": True,
            "message": "真实画面已完成世界状态构建和候选风险评价；未发送机械臂动作。",
            "data": {
                "mode": "world_model_camera_only",
                "motion_sent": False,
                "kinematics_mode": "controller_dry_run",
                "trial_id": trial_dir.name,
                "state_id": state.state_id,
                "target": asdict(state.objects[0]),
                "calibration": asdict(state.calibration),
                "candidate_count": len(candidates),
                "candidates": evaluations,
                "world_model_decision": decision.to_dict(),
                "source_plan": plan_data,
            },
        }

    @_exclusive_motion
    def safety_probe(self, request: SafetyProbeRequestModel) -> Dict[str, Any]:
        if request.connect_robot and not self.config.dry_run and not self.config.allow_debug_motion:
            raise PermissionError(
                "真机 JSON 调试探测默认禁用；如确需使用，请显式设置 LEBAI_ALLOW_DEBUG_MOTION=1。"
            )
        executor = self._build_executor()
        result = executor.safety_probe_from_detections(
            user_command=request.user_command,
            detections=self._convert_detections(request.detections),
            scene_context=dict(request.scene_context),
            connect_robot=request.connect_robot,
            descend_clearance_m=request.descend_clearance_m,
            hover_offset_m=request.hover_offset_m,
        )
        return {
            "success": result.success,
            "message": result.message,
            "data": {
                "mode": "debug_detections_probe",
                "user_command": result.user_command,
                "candidates": result.candidates,
                "decision": result.decision,
                "execution": result.execution,
            },
        }

    @_exclusive_motion
    def execute_pick(self, request: ExecutePickRequest) -> Dict[str, Any]:
        if request.execute_motion and not self.config.dry_run and not self.config.allow_debug_motion:
            raise PermissionError(
                "真机 JSON 调试抓取默认禁用；如确需使用，请显式设置 LEBAI_ALLOW_DEBUG_MOTION=1。"
            )
        executor = self._build_executor()
        result = executor.execute_from_detections(
            user_command=request.user_command,
            detections=self._convert_detections(request.detections),
            scene_context=dict(request.scene_context),
            execute_motion=request.execute_motion,
            drop_pose_override=self._convert_grasp_pose(request.drop_pose_override) if request.drop_pose_override else None,
            pick_pre_grasp_offset_m=request.pick_pre_grasp_offset_m,
            pick_post_grasp_offset_m=request.pick_post_grasp_offset_m,
            place_pre_place_offset_m=request.place_pre_place_offset_m,
            place_post_place_offset_m=request.place_post_place_offset_m,
        )
        return {
            "success": result.success,
            "message": result.message,
            "data": {
                "mode": "debug_detections_pick",
                "user_command": result.user_command,
                "candidates": result.candidates,
                "decision": result.decision,
                "execution": result.execution,
            },
        }

    @_exclusive_motion
    def grasp_from_camera(self, request: CameraGraspRequest) -> Dict[str, Any]:
        if request.mode == "pick":
            return self._pick_from_probe_lock(request)

        plan_data = self._plan_from_camera_internal(
            user_command=request.user_command,
            scene_context=dict(request.scene_context),
        )
        if not plan_data["success"]:
            return {"success": False, "message": plan_data["message"], "data": plan_data}

        grasp_pose = self._grasp_pose_from_dict(plan_data["grasp_pose"])

        controller = self._build_motion_controller()
        try:
            controller.connect()
            controller.assert_robot_can_move()

            execution = controller.safety_probe(
                SafetyProbeRequest(
                    target_pose=grasp_pose,
                    label=f"probe_{plan_data['snapshot_id']}",
                    hover_offset_m=request.hover_offset_m,
                    descend_clearance_m=request.descend_clearance_m,
                )
            )
            message = "已基于当前画面执行安全探测。"
            lock = self._store_probe_lock(plan_data=plan_data, execution=execution)
            plan_data["probe_lock_id"] = lock.probe_lock_id
            plan_data["execution_source"] = "fresh_snapshot"
            plan_data["lock_snapshot_id"] = lock.snapshot_id

            plan_data["execution"] = execution
            plan_data["robot_status_preflight"] = controller.get_robot_status_summary()
            return {"success": True, "message": message, "data": plan_data}
        finally:
            controller.disconnect()

    def emergency_stop(self) -> Dict[str, Any]:
        controller = self._build_motion_controller()
        try:
            controller.connect()
            controller.emergency_stop()
            return {"success": True, "message": "已触发急停/停运动指令。", "data": {"command_log": controller.command_log}}
        finally:
            controller.disconnect()

    def stop_motion(self) -> Dict[str, Any]:
        controller = self._build_motion_controller()
        try:
            controller.connect()
            controller.stop_motion()
            return {"success": True, "message": "已下发停止运动指令。", "data": {"command_log": controller.command_log}}
        finally:
            controller.disconnect()

    @_exclusive_motion
    def open_gripper(self) -> Dict[str, Any]:
        controller = self._build_motion_controller()
        try:
            controller.connect()
            controller.open_gripper()
            return {"success": True, "message": "夹爪打开完成。", "data": {"command_log": controller.command_log}}
        finally:
            controller.disconnect()

    @_exclusive_motion
    def close_gripper(self) -> Dict[str, Any]:
        controller = self._build_motion_controller()
        try:
            controller.connect()
            controller.close_gripper()
            return {"success": True, "message": "夹爪闭合完成。", "data": {"command_log": controller.command_log}}
        finally:
            controller.disconnect()

    @_exclusive_motion
    def reset_robot(self) -> Dict[str, Any]:
        controller = self._build_motion_controller()
        try:
            controller.connect()
            controller.assert_robot_can_move()
            result = controller.reset_to_home()
            return {"success": True, "message": "机器人已回到 Home 位。", "data": result}
        finally:
            controller.disconnect()

    @_exclusive_motion
    def jog_tcp(self, request: JogTCPRequest) -> Dict[str, Any]:
        controller = self._build_motion_controller()
        try:
            controller.connect()
            controller.assert_robot_can_move()
            result = controller.jog_tcp(
                dx=request.dx,
                dy=request.dy,
                dz=request.dz,
                drz=request.drz,
                dry=request.dry,
                drx=request.drx,
                linear=request.linear,
                label=request.label,
            )
            return {"success": True, "message": "TCP 点动执行完成。", "data": result}
        finally:
            controller.disconnect()

    @_exclusive_motion
    def start_robot_system(self) -> Dict[str, Any]:
        controller = self._build_motion_controller()
        try:
            controller.connect()
            before_status = controller.get_robot_status_summary()
            if before_status.get("can_move"):
                return {
                    "success": True,
                    "message": "机器人已经处于可运动状态。",
                    "data": {
                        "before_status": before_status,
                        "after_status": before_status,
                        "command_log": controller.command_log,
                    },
                }

            start_error: Optional[str] = None
            try:
                controller.start_system()
            except RuntimeError as exc:
                start_error = str(exc)

            after_status = self._wait_until_robot_ready(controller)
            if not after_status.get("can_move"):
                details = []
                if after_status.get("robot_state"):
                    details.append(f"robot_state={after_status['robot_state']}")
                if after_status.get("estop_reason"):
                    details.append(f"estop_reason={after_status['estop_reason']}")
                if start_error:
                    details.append(f"start_error={start_error}")
                message = "启动机械臂后仍未进入可运动状态，请先确认乐白官方界面已使能且无报警。"
                if details:
                    message += " " + "; ".join(details)
                raise PermissionError(message)

            return {
                "success": True,
                "message": "机械臂启动完成，当前已允许运动。",
                "data": {
                    "before_status": before_status,
                    "after_status": after_status,
                    "start_error": start_error,
                    "command_log": controller.command_log,
                },
            }
        finally:
            controller.disconnect()

    @_exclusive_motion
    def record_home_pose(self) -> Dict[str, Any]:
        controller = self._build_status_controller()
        try:
            controller.connect()
            snapshot = controller.capture_home_snapshot()
            joint_pose = [float(value) for value in snapshot["joint_pose"]]
            if len(joint_pose) != 6:
                raise RuntimeError(f"当前关节位姿格式异常，无法保存为 Home: {joint_pose}")

            self.config.home_joint_pose = tuple(joint_pose)  # type: ignore[assignment]
            self._runtime_state["home_joint_pose"] = joint_pose
            self._runtime_state["home_tcp_pose"] = snapshot["tcp_pose"]
            self._runtime_state["home_recorded_at"] = time.time()
            self._save_runtime_state()

            return {
                "success": True,
                "message": "已将当前位置记录为 Home。",
                "data": {
                    "home_joint_pose": joint_pose,
                    "home_tcp_pose": snapshot["tcp_pose"],
                    "runtime_state_file": str(self.config.runtime_state_file),
                    "command_log": controller.command_log,
                },
            }
        finally:
            controller.disconnect()

    @_exclusive_motion
    def enter_teach_mode(self) -> Dict[str, Any]:
        controller = self._build_motion_controller()
        try:
            controller.connect()
            controller.assert_robot_can_move()
            controller.enter_teach_mode()
            self._runtime_state["teach_mode_requested"] = True
            self._runtime_state["teach_mode_updated_at"] = time.time()
            self._save_runtime_state()
            return {
                "success": True,
                "message": "已进入示教模式。",
                "data": {
                    "teach_mode_requested": True,
                    "robot_status": controller.get_robot_status_summary(),
                    "command_log": controller.command_log,
                },
            }
        finally:
            controller.disconnect()

    @_exclusive_motion
    def exit_teach_mode(self) -> Dict[str, Any]:
        controller = self._build_motion_controller()
        try:
            controller.connect()
            controller.exit_teach_mode()
            self._runtime_state["teach_mode_requested"] = False
            self._runtime_state["teach_mode_updated_at"] = time.time()
            self._save_runtime_state()
            return {
                "success": True,
                "message": "已退出示教模式。",
                "data": {
                    "teach_mode_requested": False,
                    "robot_status": controller.get_robot_status_summary(),
                    "command_log": controller.command_log,
                },
            }
        finally:
            controller.disconnect()

    def runtime_config(self) -> Dict[str, Any]:
        return {
            "success": True,
            "message": "后端配置读取成功。",
            "data": {
                "app_title": self.config.app_title,
                "app_version": self.config.app_version,
                "robot_ip": self.config.robot_ip,
                "dry_run": self.config.dry_run,
                "calibration_file": str(self.config.calibration_file),
                "runtime_state_file": str(self.config.runtime_state_file),
                "cors_origins": list(self.config.cors_origins),
                "control_auth_required": bool(self.config.web_token),
                "allow_real_motion": self.config.allow_real_motion,
                "allow_debug_motion": self.config.allow_debug_motion,
                "probe_lock_ttl_sec": self.config.probe_lock_ttl_sec,
                "probe_lock_joint_tolerance_rad": self.config.probe_lock_joint_tolerance_rad,
                "workspace": asdict(self.config.workspace),
                "motion": asdict(self.config.motion),
                "home_joint_pose": self._effective_home_joint_pose(),
                "home_tcp_pose": self._runtime_state.get("home_tcp_pose"),
                "teach_mode_requested": bool(self._runtime_state.get("teach_mode_requested", False)),
                "preview_camera": asdict(self.config.preview_camera),
                "orbbec_camera": asdict(self.config.orbbec_camera),
                "drop_pose": self.config.drop_pose.to_dict(),
                "default_probe_descend_clearance_m": self.config.default_probe_descend_clearance_m,
                "default_probe_hover_offset_m": self.config.default_probe_hover_offset_m,
                "tool_surface_offset_m": self.config.tool_surface_offset_m,
                "qwen_model": self._build_llm_agent().config.model,
                "requires_manual_enable": not self.config.dry_run,
            },
        }

    def _pick_from_probe_lock(self, request: CameraGraspRequest) -> Dict[str, Any]:
        lock = self._consume_probe_lock(request.probe_lock_id)
        plan_data = self._copy_probe_lock_plan_data(lock)
        controller = self._build_motion_controller()
        try:
            controller.connect()
            controller.assert_robot_can_move()
            if lock.robot_joint_snapshot is not None:
                current_joints = controller.get_actual_joint_pose()
                max_joint_delta = max(
                    abs(float(current) - float(expected))
                    for current, expected in zip(current_joints, lock.robot_joint_snapshot)
                )
                if max_joint_delta > self.config.probe_lock_joint_tolerance_rad:
                    raise PermissionError(
                        "机械臂位置已偏离安全探测结束位置，探测锁已失效。"
                        f" max_joint_delta={max_joint_delta:.4f}rad"
                    )
            execution = controller.pick_and_place_from_locked_pick(
                PickPlaceRequest(
                    pick_pose=self._grasp_pose_from_dict(lock.execution_target_pose),
                    place_pose=self._convert_grasp_pose(request.drop_pose_override) if request.drop_pose_override else self.config.drop_pose,
                    label=f"pick_{lock.snapshot_id}",
                    pre_grasp_offset_m=request.pick_pre_grasp_offset_m,
                    post_grasp_offset_m=request.pick_post_grasp_offset_m,
                    pre_place_offset_m=request.place_pre_place_offset_m,
                    post_place_offset_m=request.place_post_place_offset_m,
                ),
                pick_pose=self._grasp_pose_from_dict(lock.execution_target_pose),
                pick_joints=lock.execution_target_joints,
            )
            plan_data["execution"] = execution
            plan_data["probe_lock_id"] = lock.probe_lock_id
            plan_data["execution_source"] = "probe_lock"
            plan_data["lock_snapshot_id"] = lock.snapshot_id
            plan_data["robot_status_preflight"] = controller.get_robot_status_summary()
            return {"success": True, "message": "已复用最近一次安全探测执行完整抓取。", "data": plan_data}
        finally:
            controller.disconnect()

    def _store_probe_lock(self, plan_data: Dict[str, Any], execution: Dict[str, Any]) -> ProbeLockState:
        target_pose = execution.get("target_pose")
        target_joints = execution.get("target_joints")
        robot_joint_snapshot = execution.get("retreat_joints")
        snapshot_id = str(plan_data.get("snapshot_id") or "")
        if not isinstance(target_pose, dict) or not isinstance(target_joints, list) or not snapshot_id:
            raise RuntimeError("安全探测结果缺少可复用的目标位姿，无法创建 probe lock。")

        probe_lock = ProbeLockState(
            probe_lock_id=f"probe_lock_{snapshot_id}_{int(time.time() * 1000)}",
            created_at=time.time(),
            snapshot_id=snapshot_id,
            plan_data=copy.deepcopy(plan_data),
            execution_target_pose=copy.deepcopy(target_pose),
            execution_target_joints=[float(value) for value in target_joints],
            robot_joint_snapshot=(
                [float(value) for value in robot_joint_snapshot]
                if isinstance(robot_joint_snapshot, list) and len(robot_joint_snapshot) == 6
                else None
            ),
            calibration_fingerprint=self._calibration_fingerprint(),
        )
        with self._probe_lock_guard:
            self._latest_probe_lock = probe_lock
        return probe_lock

    def _copy_probe_lock_plan_data(self, lock: ProbeLockState) -> Dict[str, Any]:
        plan_data = copy.deepcopy(lock.plan_data)
        plan_data["probe_lock_id"] = lock.probe_lock_id
        plan_data["lock_snapshot_id"] = lock.snapshot_id
        return plan_data

    def _require_probe_lock(self, probe_lock_id: Optional[str]) -> ProbeLockState:
        with self._probe_lock_guard:
            return self._validate_probe_lock_unlocked(probe_lock_id)

    def _consume_probe_lock(self, probe_lock_id: Optional[str]) -> ProbeLockState:
        with self._probe_lock_guard:
            lock = self._validate_probe_lock_unlocked(probe_lock_id)
            self._latest_probe_lock = None
            return lock

    def _validate_probe_lock_unlocked(self, probe_lock_id: Optional[str]) -> ProbeLockState:
        lock = self._latest_probe_lock
        if not probe_lock_id or lock is None or lock.probe_lock_id != probe_lock_id:
            raise PermissionError("安全探测锁无效或已被消费。请重新执行当前画面安全探测后再抓取。")
        age_sec = time.time() - lock.created_at
        if age_sec > self.config.probe_lock_ttl_sec:
            self._latest_probe_lock = None
            raise PermissionError(
                f"安全探测锁已过期（{age_sec:.1f}s > {self.config.probe_lock_ttl_sec:.1f}s），请重新探测。"
            )
        if lock.calibration_fingerprint != self._calibration_fingerprint():
            self._latest_probe_lock = None
            raise PermissionError("标定文件已发生变化，安全探测锁已失效，请重新探测。")
        return lock

    def _calibration_fingerprint(self) -> str:
        path = self.config.calibration_file
        if not path.exists():
            return "missing"
        digest = hashlib.sha256()
        with path.open("rb") as file:
            for chunk in iter(lambda: file.read(1024 * 1024), b""):
                digest.update(chunk)
        return digest.hexdigest()

    def _invalidate_probe_lock(self) -> None:
        with self._probe_lock_guard:
            self._latest_probe_lock = None

    def export_example_payload(self) -> Dict[str, Any]:
        example = PlanPickRequest(
            user_command="抓取红色的苹果",
            detections=[
                Detection2DRequest(
                    object_id="obj_apple_red_01",
                    name="apple",
                    display_name="红苹果",
                    color="red",
                    u=660.0,
                    v=410.0,
                    depth=0.36,
                    depth_unit="m",
                    confidence=0.98,
                    yaw_rad=0.15,
                    tags=["fruit", "苹果"],
                    metadata={"detector": "demo"},
                    size_xyz_m=(0.075, 0.075, 0.072),
                )
            ],
            scene_context={"robot_state": "idle", "source": "web_demo"},
        )
        return {"success": True, "message": "接口样例生成成功。", "data": self._model_dump(example)}

    def export_camera_example_payload(self) -> Dict[str, Any]:
        example = CameraPlanRequest(
            user_command="抓取红色杯子",
            scene_context={"source": "web_demo", "note": "当前画面单帧规划"},
        )
        return {"success": True, "message": "当前画面抓取样例生成成功。", "data": self._model_dump(example)}

    def _default_runtime_state(self) -> Dict[str, Any]:
        return {
            "home_joint_pose": list(self.config.home_joint_pose) if self.config.home_joint_pose else None,
            "home_tcp_pose": None,
            "teach_mode_requested": False,
            "home_recorded_at": None,
            "teach_mode_updated_at": None,
        }

    def _load_runtime_state(self) -> Dict[str, Any]:
        default_state = self._default_runtime_state()
        path = self.config.runtime_state_file
        if not path.exists():
            return default_state
        try:
            payload = json.loads(path.read_text(encoding="utf-8"))
        except Exception:
            return default_state
        if not isinstance(payload, dict):
            return default_state
        state = dict(default_state)
        state.update(payload)
        return state

    def _save_runtime_state(self) -> None:
        path = self.config.runtime_state_file
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(self._runtime_state, ensure_ascii=False, indent=2), encoding="utf-8")

    def _effective_home_joint_pose(self) -> Optional[List[float]]:
        if self.config.home_joint_pose is not None:
            return [float(value) for value in self.config.home_joint_pose]
        saved = self._runtime_state.get("home_joint_pose")
        if isinstance(saved, list) and len(saved) == 6:
            return [float(value) for value in saved]
        return None

    def _decode_subprocess_output(self, payload: Any) -> str:
        if payload is None:
            return ""
        if isinstance(payload, str):
            return payload
        if isinstance(payload, bytes):
            for encoding in ("utf-8", "gbk", sys.getdefaultencoding()):
                try:
                    return payload.decode(encoding)
                except Exception:
                    continue
            return payload.decode("utf-8", errors="replace")
        return str(payload)

    def _wait_until_robot_ready(
        self,
        controller: LebaiController,
        timeout_sec: float = 8.0,
        interval_sec: float = 0.4,
    ) -> Dict[str, Any]:
        deadline = time.time() + timeout_sec
        last_status = controller.get_robot_status_summary()
        while time.time() < deadline:
            if last_status.get("can_move"):
                return last_status
            time.sleep(interval_sec)
            last_status = controller.get_robot_status_summary()
        return last_status

    def _build_motion_controller(self) -> LebaiController:
        return LebaiController(
            LebaiControllerConfig(
                robot_ip=self.config.robot_ip,
                workspace=self.config.workspace,
                motion=self.config.motion,
                dry_run=self.config.dry_run,
                auto_start_system=False,
                gripper=GripperConfig(init_on_connect=False),
                home_joint_pose=self.config.home_joint_pose,
            )
        )

    def _build_status_controller(self) -> LebaiController:
        return LebaiController(
            LebaiControllerConfig(
                robot_ip=self.config.robot_ip,
                workspace=self.config.workspace,
                motion=self.config.motion,
                dry_run=self.config.dry_run,
                auto_start_system=False,
                gripper=GripperConfig(init_on_connect=False),
                home_joint_pose=self.config.home_joint_pose,
            )
        )

    def _build_executor(self) -> PickExecutor:
        controller_config = LebaiControllerConfig(
            robot_ip=self.config.robot_ip,
            workspace=self.config.workspace,
            motion=self.config.motion,
            dry_run=self.config.dry_run,
            auto_start_system=False,
            gripper=GripperConfig(init_on_connect=False),
            home_joint_pose=self.config.home_joint_pose,
        )
        return PickExecutor.from_defaults(
            calibration_file=self.config.calibration_file,
            controller_config=controller_config,
            config=PickExecutorConfig(
                calibration_file=self.config.calibration_file,
                drop_pose=self.config.drop_pose,
            ),
        )

    def _build_vision_transformer(self) -> VisionTransformer:
        if self._vision_transformer is None:
            self._vision_transformer = VisionTransformer.from_calibration_file(self.config.calibration_file)
        return self._vision_transformer

    def _build_llm_agent(self) -> QwenPickAgent:
        if self._llm_agent is None:
            self._llm_agent = QwenPickAgent.from_env()
        return self._llm_agent

    def _generate_depth_candidates(self, snapshot: Any) -> List[Dict[str, Any]]:
        depth = np.asarray(snapshot.depth_m, dtype=np.float32)
        valid_mask = np.isfinite(depth) & (depth > 0.0)
        if int(valid_mask.sum()) < 1000:
            return []

        fill_value = float(np.median(depth[valid_mask]))
        filled = depth.copy()
        filled[~valid_mask] = fill_value
        filled[filled <= 0.0] = fill_value

        background = cv2.morphologyEx(
            filled,
            cv2.MORPH_CLOSE,
            np.ones((61, 61), np.uint8),
        )
        diff = background - filled
        mask = (diff > 0.015).astype(np.uint8) * 255
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        mask = cv2.dilate(mask, np.ones((19, 19), np.uint8), iterations=1)

        num_labels, _, stats, _ = cv2.connectedComponentsWithStats(
            (mask > 0).astype(np.uint8),
            8,
        )
        image_h, image_w = depth.shape[:2]
        border_margin = 30
        candidates: List[Dict[str, Any]] = []
        for label in range(1, num_labels):
            x, y, w, h, area = [int(v) for v in stats[label]]
            if area < 800:
                continue
            if x <= border_margin or y <= border_margin:
                continue
            if x + w >= image_w - border_margin or y + h >= image_h - border_margin:
                continue
            if w < 20 or h < 20:
                continue
            if area > 35000:
                continue

            fill_ratio = float(area) / float(max(w * h, 1))
            if fill_ratio < 0.10:
                continue

            bbox = [x, y, x + w, y + h]
            sampled = snapshot.sample_depth_in_bbox(tuple(bbox))
            if sampled is None:
                continue

            candidate_id = f"C{len(candidates) + 1}"
            center_u = int(round(sampled["u"]))
            center_v = int(round(sampled["v"]))
            candidates.append(
                {
                    "candidate_id": candidate_id,
                    "bbox_xyxy": bbox,
                    "center_uv": [center_u, center_v],
                    "area_px": int(area),
                    "fill_ratio": fill_ratio,
                    "depth_hint_m": float(sampled["depth_m"]),
                }
            )

        candidates.sort(key=lambda item: item["area_px"], reverse=True)
        return candidates[:8]

    def _annotate_candidates_for_qwen(
        self,
        snapshot: Any,
        candidates: List[Dict[str, Any]],
    ) -> bytes:
        image = snapshot.color_bgr.copy()
        for candidate in candidates:
            x1, y1, x2, y2 = [int(v) for v in candidate["bbox_xyxy"]]
            label = str(candidate["candidate_id"])
            cv2.rectangle(image, (x1, y1), (x2, y2), (36, 255, 180), 2)
            cv2.putText(
                image,
                label,
                (x1 + 4, max(y1 - 8, 20)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 255),
                2,
                cv2.LINE_AA,
            )
            center_u, center_v = [int(v) for v in candidate["center_uv"]]
            cv2.circle(image, (center_u, center_v), 5, (0, 180, 255), -1)

        ok, encoded = cv2.imencode(
            ".jpg",
            image,
            [int(cv2.IMWRITE_JPEG_QUALITY), int(self.config.orbbec_camera.jpeg_quality)],
        )
        if not ok:
            raise RuntimeError("候选框标注图编码失败。")
        return encoded.tobytes()

    def _enrich_depth_candidates(
        self,
        snapshot: Any,
        candidates: List[Dict[str, Any]],
    ) -> List[Dict[str, Any]]:
        enriched: List[Dict[str, Any]] = []
        for item in candidates:
            enriched_item = dict(item)
            enriched_item["dominant_color"] = self._infer_candidate_color(
                snapshot=snapshot,
                bbox_xyxy=item["bbox_xyxy"],
            )
            enriched.append(enriched_item)
        return enriched

    def _infer_candidate_color(
        self,
        snapshot: Any,
        bbox_xyxy: List[int],
    ) -> Optional[str]:
        x1, y1, x2, y2 = [int(v) for v in bbox_xyxy]
        image_h, image_w = snapshot.color_bgr.shape[:2]
        x1 = max(0, min(x1, image_w - 1))
        y1 = max(0, min(y1, image_h - 1))
        x2 = max(x1 + 1, min(x2, image_w))
        y2 = max(y1 + 1, min(y2, image_h))

        crop = snapshot.color_bgr[y1:y2, x1:x2]
        if crop.size == 0:
            return None

        mean_bgr = crop.reshape(-1, 3).mean(axis=0).astype(np.uint8)
        hsv = cv2.cvtColor(np.asarray([[mean_bgr]], dtype=np.uint8), cv2.COLOR_BGR2HSV)[0, 0]
        hue = int(hsv[0])
        sat = int(hsv[1])
        val = int(hsv[2])

        if val < 50:
            return "black"
        if sat < 35 and val > 185:
            return "white"
        if sat < 35:
            return None
        if hue < 10 or hue >= 170:
            return "red"
        if hue < 22:
            return "orange"
        if hue < 35:
            return "yellow"
        if hue < 85:
            return "green"
        if hue < 130:
            return "blue"
        if hue < 155:
            return "purple"
        return "red"

    def _plan_with_local_candidates(
        self,
        user_command: str,
        candidates: List[Dict[str, Any]],
        model_error: Optional[str] = None,
    ) -> ImageTargetPlan:
        normalized_command = self._normalize_text(user_command)
        filtered = list(candidates)
        matched_filters: List[str] = []

        color_aliases = {
            "red": ["红", "红色", "red"],
            "yellow": ["黄", "黄色", "yellow"],
            "green": ["绿", "绿色", "green"],
            "blue": ["蓝", "蓝色", "blue"],
            "white": ["白", "白色", "white"],
            "black": ["黑", "黑色", "black"],
            "orange": ["橙", "橙色", "orange"],
            "purple": ["紫", "紫色", "purple"],
        }
        desired_colors = [
            color
            for color, aliases in color_aliases.items()
            if any(self._normalize_text(alias) in normalized_command for alias in aliases)
        ]
        if desired_colors:
            color_filtered = [
                item for item in filtered
                if item.get("dominant_color") in desired_colors
            ]
            if color_filtered:
                filtered = color_filtered
                matched_filters.append(f"color={desired_colors[0]}")

        if "最近" in user_command or "nearest" in normalized_command or "closest" in normalized_command:
            selected = min(filtered, key=lambda item: float(item.get("depth_hint_m", 999.0)))
            matched_filters.append("nearest")
        elif "最左" in user_command or "leftmost" in normalized_command:
            selected = min(filtered, key=lambda item: int(item["center_uv"][0]))
            matched_filters.append("leftmost")
        elif "最右" in user_command or "rightmost" in normalized_command:
            selected = max(filtered, key=lambda item: int(item["center_uv"][0]))
            matched_filters.append("rightmost")
        else:
            selected = max(
                filtered,
                key=lambda item: (float(item.get("area_px", 0)), -float(item.get("depth_hint_m", 999.0))),
            )
            matched_filters.append("largest_prominent_blob")

        selected_color = selected.get("dominant_color")
        selected_name = f"{selected_color} target" if selected_color else "candidate target"
        reason = "Qwen 当前不可用，已回退到本地深度候选规划。"
        if matched_filters:
            reason += f" 规则: {', '.join(matched_filters)}。"
        if model_error:
            reason += f" 模型错误: {model_error}"

        return ImageTargetPlan(
            success=True,
            selected_object_name=selected_name,
            reason=reason,
            bbox_xyxy=list(selected["bbox_xyxy"]),
            center_uv=list(selected["center_uv"]),
            confidence=0.45 if desired_colors else 0.35,
            source="local-candidate",
            metadata={
                "candidates": candidates,
                "selected_candidate_id": str(selected["candidate_id"]),
                "matched_filters": matched_filters,
                "model_error": model_error,
            },
        )

    def _normalize_text(self, text: str) -> str:
        return "".join(str(text).strip().lower().split())

    def _plan_with_depth_candidates(
        self,
        user_command: str,
        snapshot: Any,
        scene_context: Dict[str, Any],
    ) -> Optional[ImageTargetPlan]:
        candidates = self._generate_depth_candidates(snapshot)
        if not candidates:
            return None
        candidates = self._enrich_depth_candidates(snapshot, candidates)

        agent = self._build_llm_agent()
        if not (agent.config.enable_model and agent.config.api_key):
            return self._plan_with_local_candidates(
                user_command=user_command,
                candidates=candidates,
                model_error="未配置 QWEN_API_KEY / DASHSCOPE_API_KEY",
            )

        try:
            annotated_jpeg = self._annotate_candidates_for_qwen(snapshot, candidates)
            image_b64 = base64.b64encode(annotated_jpeg).decode("ascii")
            instruction = {
                "task": user_command,
                "scene_context": scene_context,
                "candidates": candidates,
                "requirements": [
                    "只能从给定的 candidate_id 中选择",
                    "不要重新发明新的 bbox",
                    "如果没有合适目标，返回 success=false",
                ],
                "output_schema": {
                    "success": "bool",
                    "selected_candidate_id": "string or null",
                    "selected_object_name": "string or null",
                    "reason": "string",
                    "confidence": "float",
                },
            }
            messages = [
                {
                    "role": "system",
                    "content": (
                        "你是机器人抓取系统的候选目标选择代理。"
                        "图像里已经画出了候选框编号，你只能在这些候选框里选最符合指令的目标。"
                        "不要输出 Markdown，只输出严格 JSON。"
                    ),
                },
                {
                    "role": "user",
                    "content": [
                        {
                            "type": "text",
                            "text": json.dumps(instruction, ensure_ascii=False, indent=2),
                        },
                        {
                            "type": "image_url",
                            "image_url": {"url": f"data:image/jpeg;base64,{image_b64}"},
                        },
                    ],
                },
            ]
            raw_text = agent._post_chat_completion(
                messages=messages,
                response_format={"type": "json_object"},
            )
            payload = agent._extract_json(raw_text)
            success = bool(payload.get("success"))
            selected_candidate_id = payload.get("selected_candidate_id")
            selected_object_name = payload.get("selected_object_name")
            reason = str(payload.get("reason", "")).strip() or "Qwen 未提供原因。"
            confidence = float(payload.get("confidence", 0.0) or 0.0)

            candidate_map = {item["candidate_id"]: item for item in candidates}
            selected_candidate = (
                candidate_map.get(str(selected_candidate_id))
                if selected_candidate_id is not None
                else None
            )
            if success and selected_candidate is None:
                raise ValueError(f"Qwen 返回了未知 candidate_id: {selected_candidate_id}")

            if not success or selected_candidate is None:
                return ImageTargetPlan(
                    success=False,
                    selected_object_name=None if selected_object_name is None else str(selected_object_name),
                    reason=reason,
                    bbox_xyxy=None,
                    center_uv=None,
                    confidence=confidence,
                    raw_model_response=raw_text,
                    source="qwen-candidate",
                    metadata={"candidates": candidates},
                )

            return ImageTargetPlan(
                success=True,
                selected_object_name=None if selected_object_name is None else str(selected_object_name),
                reason=reason,
                bbox_xyxy=list(selected_candidate["bbox_xyxy"]),
                center_uv=list(selected_candidate["center_uv"]),
                confidence=confidence,
                raw_model_response=raw_text,
                source="qwen-candidate",
                metadata={"candidates": candidates, "selected_candidate_id": str(selected_candidate_id)},
            )
        except Exception as exc:
            return self._plan_with_local_candidates(
                user_command=user_command,
                candidates=candidates,
                model_error=str(exc),
            )

    def _plan_from_camera_internal(
        self,
        user_command: str,
        scene_context: Optional[Dict[str, Any]] = None,
    ) -> Dict[str, Any]:
        if not self.config.calibration_file.exists():
            raise FileNotFoundError(f"未找到标定文件: {self.config.calibration_file}")

        snapshot = self._capture_snapshot_for_plan()
        snapshot_jpeg = snapshot.jpeg_bytes(self.config.orbbec_camera.jpeg_quality)
        self._latest_snapshot_jpeg = snapshot_jpeg
        self._latest_snapshot_id = snapshot.snapshot_id

        base_scene_context = dict(scene_context or {})
        candidate_stage_failure: Optional[Dict[str, Any]] = None
        image_plan = self._plan_with_depth_candidates(
            user_command=user_command,
            snapshot=snapshot,
            scene_context=base_scene_context,
        )
        if image_plan is None:
            image_plan = self._plan_image_target_with_qwen(
                user_command=user_command,
                image_bytes=snapshot_jpeg,
                image_size_wh=snapshot.image_size_wh,
                scene_context=base_scene_context,
            )
        elif image_plan.source == "qwen-candidate" and not image_plan.success:
            candidate_stage_failure = {
                "reason": image_plan.reason,
                "candidate_count": len(image_plan.metadata.get("candidates", [])),
                "candidates": image_plan.metadata.get("candidates", []),
            }
            fallback_scene_context = dict(base_scene_context)
            fallback_scene_context["candidate_stage_failure"] = {
                "reason": image_plan.reason,
                "candidate_count": len(image_plan.metadata.get("candidates", [])),
            }
            fallback_scene_context["retry_instruction"] = (
                "候选框阶段没有把目标可靠圈出来。"
                "请忽略候选框约束，直接在整张图中重新定位最符合指令的目标物体本体。"
                "不要框到机械臂、鼠标、笔记本或桌面背景。"
            )
            image_plan = self._plan_image_target_with_qwen(
                user_command=user_command,
                image_bytes=snapshot_jpeg,
                image_size_wh=snapshot.image_size_wh,
                scene_context=fallback_scene_context,
            )
        result = self._evaluate_camera_plan(snapshot=snapshot, image_plan=image_plan)
        result["retry_attempted"] = False
        if candidate_stage_failure is not None:
            result["candidate_stage_failure"] = candidate_stage_failure

        if (image_plan.source == "qwen-candidate" and image_plan.success) or result["success"] or not result.get("retryable", False):
            return result

        retry_scene_context = self._build_retry_scene_context(
            base_scene_context=base_scene_context,
            image_plan=image_plan,
            result=result,
        )
        retry_plan = self._plan_image_target_with_qwen(
            user_command=user_command,
            image_bytes=snapshot_jpeg,
            image_size_wh=snapshot.image_size_wh,
            scene_context=retry_scene_context,
        )
        retry_result = self._evaluate_camera_plan(snapshot=snapshot, image_plan=retry_plan)
        retry_result["retry_attempted"] = True
        retry_result["previous_failure"] = {
            "message": result["message"],
            "bbox_xyxy": result["bbox_xyxy"],
            "center_uv": result["center_uv"],
        }
        if retry_result["success"]:
            retry_result["message"] = "Qwen 二次重定位后，已基于当前画面生成抓取规划。"
        return retry_result

    def _evaluate_camera_plan(
        self,
        snapshot: Any,
        image_plan: ImageTargetPlan,
    ) -> Dict[str, Any]:
        if not image_plan.success:
            self._latest_plan_image_jpeg = self._annotate_snapshot(snapshot, None, None, image_plan.reason)
            return {
                "success": False,
                "message": image_plan.reason,
                "snapshot_id": snapshot.snapshot_id,
                "latest_plan_image_url": "/api/camera/latest-plan.jpg",
                "image_size_wh": list(snapshot.image_size_wh),
                "decision": image_plan.to_dict(),
                "bbox_xyxy": image_plan.bbox_xyxy,
                "center_uv": image_plan.center_uv,
                "depth_m": None,
                "point_base_m": None,
                "grasp_pose": None,
                "execution": None,
                "retryable": True,
            }

        if self._bbox_touches_image_border(image_plan=image_plan, image_size_wh=snapshot.image_size_wh):
            self._latest_plan_image_jpeg = self._annotate_snapshot(
                snapshot,
                image_plan.bbox_xyxy,
                tuple(image_plan.center_uv) if image_plan.center_uv is not None else None,
                "边界框贴边",
            )
            return {
                "success": False,
                "message": "目标框紧贴图像边界，定位结果不稳定，已拒绝下发机械臂动作。",
                "snapshot_id": snapshot.snapshot_id,
                "latest_plan_image_url": "/api/camera/latest-plan.jpg",
                "image_size_wh": list(snapshot.image_size_wh),
                "decision": image_plan.to_dict(),
                "bbox_xyxy": image_plan.bbox_xyxy,
                "center_uv": image_plan.center_uv,
                "depth_m": None,
                "depth_source": None,
                "point_base_m": None,
                "grasp_pose": None,
                "execution": None,
                "retryable": True,
            }

        center_u, center_v = self._resolve_center(image_plan)
        depth_m = None
        depth_source = "center"
        if image_plan.bbox_xyxy is not None:
            sampled = snapshot.sample_depth_in_bbox(tuple(image_plan.bbox_xyxy))
            if sampled is not None:
                center_u = int(round(sampled["u"]))
                center_v = int(round(sampled["v"]))
                depth_m = float(sampled["depth_m"])
                depth_source = "bbox_grid_near_surface"

        if depth_m is None:
            depth_m = snapshot.get_depth_at_pixel(center_u, center_v)
            depth_source = "center"
        if depth_m is None:
            depth_m = snapshot.find_valid_depth_neighborhood(
                center_u,
                center_v,
                window_size=self.config.orbbec_camera.depth_neighborhood_window,
            )
            depth_source = "neighborhood_median"

        if depth_m is None:
            self._latest_plan_image_jpeg = self._annotate_snapshot(
                snapshot,
                image_plan.bbox_xyxy,
                (center_u, center_v),
                "深度无效",
            )
            return {
                "success": False,
                "message": "目标框中心附近没有有效深度，已拒绝下发机械臂动作。",
                "snapshot_id": snapshot.snapshot_id,
                "latest_plan_image_url": "/api/camera/latest-plan.jpg",
                "image_size_wh": list(snapshot.image_size_wh),
                "decision": image_plan.to_dict(),
                "bbox_xyxy": image_plan.bbox_xyxy,
                "center_uv": [center_u, center_v],
                "depth_m": None,
                "depth_source": depth_source,
                "point_base_m": None,
                "grasp_pose": None,
                "execution": None,
                "retryable": True,
            }

        transformer = self._build_vision_transformer()
        transformer.assert_image_size(snapshot.image_size_wh)
        point_base = transformer.pixel_to_base(
            u=center_u,
            v=center_v,
            depth=depth_m,
            depth_unit="m",
        )
        grasp_pose = self._build_executable_grasp_pose(point_base)
        command_point = grasp_pose.position
        workspace_ok = self.config.workspace.contains(command_point)
        self._latest_plan_image_jpeg = self._annotate_snapshot(
            snapshot,
            image_plan.bbox_xyxy,
            (center_u, center_v),
            image_plan.selected_object_name or "target",
        )

        if not workspace_ok:
            return {
                "success": False,
                "message": (
                    "已识别到目标表面点，但换算后的机械臂执行位姿仍超出工作空间，已拒绝下发动作。"
                    f" command_point=({command_point.x:.4f}, {command_point.y:.4f}, {command_point.z:.4f})"
                ),
                "snapshot_id": snapshot.snapshot_id,
                "latest_plan_image_url": "/api/camera/latest-plan.jpg",
                "image_size_wh": list(snapshot.image_size_wh),
                "decision": image_plan.to_dict(),
                "bbox_xyxy": image_plan.bbox_xyxy,
                "center_uv": [center_u, center_v],
                "depth_m": float(depth_m),
                "depth_source": depth_source,
                "surface_point_base_m": point_base.to_dict(),
                "point_base_m": command_point.to_dict(),
                "grasp_pose": grasp_pose.to_dict(),
                "workspace_ok": False,
                "workspace": asdict(self.config.workspace),
                "execution": None,
                "retryable": True,
            }

        return {
            "success": True,
            "message": "已基于当前画面生成抓取规划。",
            "snapshot_id": snapshot.snapshot_id,
            "latest_plan_image_url": "/api/camera/latest-plan.jpg",
            "image_size_wh": list(snapshot.image_size_wh),
            "decision": image_plan.to_dict(),
            "bbox_xyxy": image_plan.bbox_xyxy,
            "center_uv": [center_u, center_v],
            "depth_m": float(depth_m),
            "depth_source": depth_source,
            "surface_point_base_m": point_base.to_dict(),
            "point_base_m": command_point.to_dict(),
            "grasp_pose": grasp_pose.to_dict(),
            "workspace_ok": True,
            "workspace": asdict(self.config.workspace),
            "execution": None,
            "retryable": False,
        }

    def _build_executable_grasp_pose(self, surface_point: Pose3D) -> GraspPose:
        """
        把视觉识别到的“目标表面点”换算成机械臂真正要执行的位姿。
        当前现场 `actual_tcp_pose == actual_flange_pose`，说明机器人侧尚未配置 TCP，
        因此这里先在软件侧补一个“夹爪末端表面点 -> 法兰执行点”的固定补偿距离。
        """
        approach_vector = Pose3D(0.0, 0.0, -1.0)
        approach = self._normalize_pose_vector(approach_vector)
        offset_m = max(float(self.config.tool_surface_offset_m), 0.0)
        command_point = Pose3D(
            x=surface_point.x - approach.x * offset_m,
            y=surface_point.y - approach.y * offset_m,
            z=surface_point.z - approach.z * offset_m,
        )
        return GraspPose(
            position=command_point,
            roll=0.0,
            pitch=math.pi,
            yaw=0.0,
            approach_vector=approach_vector,
            pre_grasp_offset_m=self.config.default_probe_hover_offset_m,
        )

    def _normalize_pose_vector(self, vector: Pose3D) -> Pose3D:
        length = math.sqrt(vector.x * vector.x + vector.y * vector.y + vector.z * vector.z)
        if length < 1e-9:
            raise ValueError("向量长度不能为 0。")
        return Pose3D(vector.x / length, vector.y / length, vector.z / length)

    def _bbox_touches_image_border(
        self,
        image_plan: Any,
        image_size_wh: Any,
        border_margin_px: int = 2,
    ) -> bool:
        if image_plan.bbox_xyxy is None or image_size_wh is None or len(image_size_wh) != 2:
            return False
        width = int(image_size_wh[0])
        height = int(image_size_wh[1])
        x1, y1, x2, y2 = [int(v) for v in image_plan.bbox_xyxy]
        return (
            x1 <= border_margin_px
            or y1 <= border_margin_px
            or x2 >= width - border_margin_px
            or y2 >= height - border_margin_px
        )

    def _build_retry_scene_context(
        self,
        base_scene_context: Dict[str, Any],
        image_plan: ImageTargetPlan,
        result: Dict[str, Any],
    ) -> Dict[str, Any]:
        retry_context = dict(base_scene_context)
        retry_context["previous_failure"] = {
            "message": result["message"],
            "bbox_xyxy": result.get("bbox_xyxy"),
            "center_uv": result.get("center_uv"),
            "selected_object_name": image_plan.selected_object_name,
        }
        retry_context["robot_workspace"] = asdict(self.config.workspace)
        retry_context["retry_instruction"] = (
            "上一次定位结果不可用。请重新定位到目标物体本体，不要框到桌面、背景或物体外的空白区域。"
            "如果没有把握，请返回 success=false。"
        )
        return retry_context

    def _capture_snapshot_for_plan(self) -> Any:
        last_error: Optional[Exception] = None
        for attempt in range(2):
            try:
                with self._camera_io_lock:
                    # 规划/抓取前先释放网页 RGB 预览，再独占 Orbbec SDK。
                    self.preview_camera.close()
                    self.depth_camera.close()
                    snapshot = self.depth_camera.capture_snapshot()
                    self.depth_camera.close()
                    return snapshot
            except RuntimeError as exc:
                last_error = exc
                time.sleep(0.2 + attempt * 0.2)

        assert last_error is not None
        message = str(last_error)
        if "0xc00d3704" in message.lower() or "mft" in message.lower() or "资源不足" in message:
            raise RuntimeError(
                "当前画面抓取规划失败：Orbbec RGB+Depth 开流失败。"
                "原因是深度链在抓帧时仍然遇到 Windows MFT 资源占用。"
                "本次请求已自动释放网页预览后重试，但仍未成功。"
                "请关闭其它占用 Gemini 的程序后重试。"
            ) from last_error
        raise RuntimeError(f"当前画面抓取规划失败：{message}") from last_error

    def _plan_image_target_with_qwen(
        self,
        user_command: str,
        image_bytes: bytes,
        image_size_wh: Any,
        scene_context: Dict[str, Any],
    ) -> ImageTargetPlan:
        agent = self._build_llm_agent()
        if not (agent.config.enable_model and agent.config.api_key):
            raise RuntimeError("未配置 QWEN_API_KEY / DASHSCOPE_API_KEY，无法执行当前画面抓取规划。")

        raw_text = agent._post_chat_completion(
            messages=self._build_camera_plan_messages(
                user_command=user_command,
                image_bytes=image_bytes,
                image_mime_type="image/jpeg",
                image_size_wh=image_size_wh,
                scene_context=scene_context,
            ),
            response_format={"type": "json_object"},
        )
        payload = agent._extract_json(raw_text)
        plan = self._sanitize_image_plan_payload(payload=payload, image_size_wh=image_size_wh)
        plan.raw_model_response = raw_text
        return plan

    def _build_camera_plan_messages(
        self,
        user_command: str,
        image_bytes: bytes,
        image_mime_type: str,
        image_size_wh: Any,
        scene_context: Dict[str, Any],
    ) -> List[Dict[str, Any]]:
        image_b64 = base64.b64encode(image_bytes).decode("ascii")
        system_prompt = (
            "你是机器人抓取系统的视觉定位代理。"
            "你会看到一张当前 RGB 图像，以及一条中文抓取指令。"
            "你的任务是只根据图像内容，找出最应该抓取的一个目标物体。"
            "你必须返回严格 JSON，不要输出 Markdown，不要输出额外解释。"
            "bbox_xyxy 必须紧贴目标物体本体，不能把桌面、背景或大面积空白区域框进去。"
            "center_uv 必须落在目标物体内部，而不是背景上。"
            "如果 scene_context 中存在 previous_failure，说明上一次定位错了，你必须重新定位，不能重复之前的错误框。"
            "如果没有把握，请返回 success=false，而不是猜测背景区域。"
        )
        instruction = {
            "task": user_command,
            "image_size_wh": image_size_wh,
            "scene_context": scene_context,
            "requirements": [
                "只返回严格 JSON",
                "bbox_xyxy 使用当前整张图像的像素坐标 [x1, y1, x2, y2]",
                "center_uv 使用目标物体内部的像素中心 [u, v]",
                "bbox 必须完整落在图像范围内",
                "如果目标不存在或定位不确定，success=false 且 bbox_xyxy/center_uv 设为 null",
            ],
            "output_schema": {
                "success": "bool",
                "selected_object_name": "string or null",
                "reason": "string",
                "bbox_xyxy": "[int, int, int, int] or null",
                "center_uv": "[int, int] or null",
                "confidence": "float",
            },
        }
        return [
            {"role": "system", "content": system_prompt},
            {
                "role": "user",
                "content": [
                    {
                        "type": "text",
                        "text": json.dumps(instruction, ensure_ascii=False, indent=2),
                    },
                    {
                        "type": "image_url",
                        "image_url": {"url": f"data:{image_mime_type};base64,{image_b64}"},
                    },
                ],
            },
        ]

    def _sanitize_image_plan_payload(
        self,
        payload: Dict[str, Any],
        image_size_wh: Any,
    ) -> ImageTargetPlan:
        success = bool(payload.get("success"))
        reason = str(payload.get("reason", "")).strip() or "Qwen 未提供原因。"
        selected_object_name = payload.get("selected_object_name")
        confidence = float(payload.get("confidence", 0.0) or 0.0)

        bbox_xyxy: Optional[List[int]] = None
        center_uv: Optional[List[int]] = None
        bbox_clamped = False
        center_clamped = False

        bbox = payload.get("bbox_xyxy")
        if bbox is not None:
            if not isinstance(bbox, (list, tuple)) or len(bbox) != 4:
                raise ValueError(f"Qwen 返回的 bbox_xyxy 非法: {bbox}")
            bbox_xyxy = [int(round(float(item))) for item in bbox]

        center = payload.get("center_uv")
        if center is not None:
            if not isinstance(center, (list, tuple)) or len(center) != 2:
                raise ValueError(f"Qwen 返回的 center_uv 非法: {center}")
            center_uv = [int(round(float(center[0]))), int(round(float(center[1])))]

        if image_size_wh is not None and len(image_size_wh) == 2:
            width = int(image_size_wh[0])
            height = int(image_size_wh[1])
            if bbox_xyxy is not None:
                x1, y1, x2, y2 = bbox_xyxy
                clamped_bbox = [
                    max(0, min(x1, width - 1)),
                    max(0, min(y1, height - 1)),
                    max(0, min(x2, width)),
                    max(0, min(y2, height)),
                ]
                bbox_clamped = clamped_bbox != bbox_xyxy
                bbox_xyxy = clamped_bbox
                x1, y1, x2, y2 = bbox_xyxy
                if x2 <= x1 or y2 <= y1:
                    raise ValueError(
                        f"Qwen 返回的 bbox_xyxy 在裁剪到图像范围后仍然无效: bbox={bbox_xyxy}, image={width}x{height}"
                    )
            if center_uv is not None:
                u, v = center_uv
                clamped_center = [
                    max(0, min(u, width - 1)),
                    max(0, min(v, height - 1)),
                ]
                center_clamped = clamped_center != center_uv
                center_uv = clamped_center

        if center_uv is None and bbox_xyxy is not None:
            x1, y1, x2, y2 = bbox_xyxy
            center_uv = [int(round((x1 + x2) / 2.0)), int(round((y1 + y2) / 2.0))]

        if bbox_clamped or center_clamped:
            reason = (
                "Qwen 返回的目标框或中心点超出了图像边界。"
                "这通常表示模型没有稳定定位到目标，本次规划已安全拒绝。"
            )
            success = False

        return ImageTargetPlan(
            success=success,
            selected_object_name=None if selected_object_name is None else str(selected_object_name),
            reason=reason,
            bbox_xyxy=bbox_xyxy,
            center_uv=center_uv,
            confidence=confidence,
            metadata={
                "bbox_was_clamped": bbox_clamped,
                "center_was_clamped": center_clamped,
            },
        )
    def _resolve_center(self, image_plan: Any) -> tuple[int, int]:
        if image_plan.center_uv is not None:
            return int(image_plan.center_uv[0]), int(image_plan.center_uv[1])
        if image_plan.bbox_xyxy is None:
            raise ValueError("Qwen 返回成功，但没有提供 bbox_xyxy / center_uv。")
        x1, y1, x2, y2 = image_plan.bbox_xyxy
        return int(round((x1 + x2) / 2.0)), int(round((y1 + y2) / 2.0))

    def _annotate_snapshot(
        self,
        snapshot: Any,
        bbox_xyxy: Optional[List[int]],
        center_uv: Optional[tuple[int, int]],
        label: Optional[str],
    ) -> bytes:
        image = snapshot.color_bgr.copy()
        if bbox_xyxy is not None:
            x1, y1, x2, y2 = [int(value) for value in bbox_xyxy]
            cv2.rectangle(image, (x1, y1), (x2, y2), (36, 255, 180), 2)
        if center_uv is not None:
            cv2.circle(image, (int(center_uv[0]), int(center_uv[1])), 6, (0, 180, 255), -1)
        if label:
            cv2.putText(
                image,
                str(label),
                (18, 34),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.9,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )
        ok, encoded = cv2.imencode(".jpg", image, [int(cv2.IMWRITE_JPEG_QUALITY), 92])
        if not ok:
            raise RuntimeError("标注后的快照编码失败。")
        return encoded.tobytes()

    def _convert_detections(self, detections: List[Detection2DRequest]) -> List[Detection2D]:
        return [
            Detection2D(
                object_id=item.object_id,
                name=item.name,
                display_name=item.display_name,
                color=item.color,
                u=float(item.u),
                v=float(item.v),
                depth=float(item.depth),
                depth_unit=item.depth_unit,
                confidence=float(item.confidence),
                yaw_rad=float(item.yaw_rad),
                tags=list(item.tags),
                metadata=dict(item.metadata),
                size_xyz_m=item.size_xyz_m,
            )
            for item in detections
        ]

    def _model_dump(self, model: Any) -> Dict[str, Any]:
        if hasattr(model, "model_dump"):
            return model.model_dump()
        if hasattr(model, "dict"):
            return model.dict()
        return json.loads(json.dumps(model))

    def _convert_pose3d(self, model: Pose3DModel) -> Pose3D:
        return Pose3D(x=float(model.x), y=float(model.y), z=float(model.z))

    def _grasp_pose_from_dict(self, payload: Dict[str, Any]) -> GraspPose:
        return GraspPose(
            position=Pose3D(
                x=float(payload["position"]["x"]),
                y=float(payload["position"]["y"]),
                z=float(payload["position"]["z"]),
            ),
            roll=float(payload["roll"]),
            pitch=float(payload["pitch"]),
            yaw=float(payload["yaw"]),
            approach_vector=Pose3D(
                x=float(payload["approach_vector"]["x"]),
                y=float(payload["approach_vector"]["y"]),
                z=float(payload["approach_vector"]["z"]),
            ),
            pre_grasp_offset_m=float(payload["pre_grasp_offset_m"]),
        )

    def _convert_grasp_pose(self, model: GraspPoseModel) -> GraspPose:
        return GraspPose(
            position=self._convert_pose3d(model.position),
            roll=float(model.roll),
            pitch=float(model.pitch),
            yaw=float(model.yaw),
            approach_vector=self._convert_pose3d(model.approach_vector),
            pre_grasp_offset_m=float(model.pre_grasp_offset_m),
        )

    def shutdown(self) -> None:
        self.preview_camera.close()
        self.depth_camera.close()
