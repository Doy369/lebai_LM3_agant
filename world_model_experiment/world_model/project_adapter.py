from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
from typing import Any, Dict, Optional, Sequence, Tuple

from robot_system.control import LebaiController
from robot_system.llm.types import ObjectCandidate

from .types import (
    CalibrationState,
    Position3D,
    RobotState,
    TaskState,
    WorldObjectState,
    WorldState,
)


def _clamp_probability(value: float) -> float:
    return max(0.0, min(1.0, float(value)))


class ExistingProjectAdapter:
    """把现有抓取项目的规划结果转换为实验世界状态。"""

    def __init__(self, calibration_file: str | Path) -> None:
        self.calibration_file = Path(calibration_file)

    def calibration_state(self) -> CalibrationState:
        raw = self.calibration_file.read_bytes()
        payload = json.loads(raw.decode("utf-8"))
        hand_eye = payload.get("hand_eye", {})
        diagnostics = hand_eye.get("selected_method_diagnostics", {})

        translation_std = diagnostics.get("translation_std_m", [])
        valid_translation_std = [
            abs(float(value))
            for value in translation_std
            if isinstance(value, (int, float)) and math.isfinite(float(value))
        ]
        translation_uncertainty_m = max(valid_translation_std, default=0.010)
        rotation_uncertainty_deg = float(diagnostics.get("rotation_consistency_deg_mean", 1.0))
        if not math.isfinite(rotation_uncertainty_deg) or rotation_uncertainty_deg < 0.0:
            rotation_uncertainty_deg = 1.0

        return CalibrationState(
            fingerprint=hashlib.sha256(raw).hexdigest(),
            valid=True,
            translation_uncertainty_m=translation_uncertainty_m,
            rotation_uncertainty_deg=rotation_uncertainty_deg,
            drift_score=0.0,
        )

    def robot_state(self, controller: LebaiController) -> RobotState:
        if not controller.config.dry_run:
            raise PermissionError("初步实验适配器只允许使用 dry_run=True 的控制器。")

        status = controller.get_robot_status_summary()
        kin_data = controller.get_kin_data()
        joints = self._float_tuple(kin_data.get("actual_joint_pose"), length=6, label="actual_joint_pose")
        tcp = kin_data.get("actual_tcp_pose")
        if not isinstance(tcp, dict):
            raise ValueError("get_kin_data 缺少 actual_tcp_pose 对象。")
        tcp_pose = tuple(float(tcp.get(key, 0.0)) for key in ("x", "y", "z", "rx", "ry", "rz"))

        return RobotState(
            joints_rad=joints,
            tcp_pose=tcp_pose,  # type: ignore[arg-type]
            robot_state=str(status.get("robot_state") or "unknown"),
            can_move=bool(status.get("can_move")),
            connected=bool(status.get("is_connected")),
            emergency_stopped=bool(status.get("estop_reason")),
            metadata={"dry_run": True, "status": dict(status)},
        )

    def object_from_candidate(self, candidate: ObjectCandidate) -> WorldObjectState:
        metadata = dict(candidate.metadata)
        center = metadata.get("pixel_center", {})
        center_uv = self._center_uv(center)
        bbox = self._bbox(metadata.get("bbox_xyxy"), center_uv)
        depth_confidence = self._depth_confidence(metadata, source=str(metadata.get("depth_source", "candidate")))
        position_std = self._position_std(metadata)
        return WorldObjectState(
            track_id=candidate.object_id,
            name=candidate.name,
            color=candidate.color,
            position_base_m=Position3D(candidate.position.x, candidate.position.y, candidate.position.z),
            semantic_confidence=_clamp_probability(candidate.confidence),
            depth_confidence=depth_confidence,
            bbox_xyxy=bbox,
            center_uv=center_uv,
            size_xyz_m=(
                Position3D(candidate.size_xyz_m.x, candidate.size_xyz_m.y, candidate.size_xyz_m.z)
                if candidate.size_xyz_m is not None
                else None
            ),
            yaw_rad=float(candidate.grasp_pose.yaw) if candidate.grasp_pose is not None else 0.0,
            position_std_m=position_std,
            reachable=True,
            metadata={"adapter_source": "ObjectCandidate", **metadata},
        )

    def object_from_camera_plan(
        self,
        plan_data: Dict[str, Any],
        track_id: Optional[str] = None,
    ) -> WorldObjectState:
        if not bool(plan_data.get("success")):
            raise ValueError("失败的相机规划不能转换为可抓取世界物体。")
        point = plan_data.get("point_base_m")
        decision = plan_data.get("decision")
        if not isinstance(point, dict) or not isinstance(decision, dict):
            raise ValueError("相机规划缺少 point_base_m 或 decision。")

        center_uv = self._center_uv(plan_data.get("center_uv"))
        bbox = self._bbox(plan_data.get("bbox_xyxy"), center_uv)
        name = str(decision.get("selected_object_name") or "unknown_object")
        confidence = _clamp_probability(float(decision.get("confidence", 0.5)))
        depth_source = str(plan_data.get("depth_source") or "unknown")
        depth_metadata = plan_data.get("depth_quality")
        metadata: Dict[str, Any] = {
            "adapter_source": "camera_plan",
            "snapshot_id": plan_data.get("snapshot_id"),
            "depth_m": plan_data.get("depth_m"),
            "depth_source": depth_source,
            "depth_quality": depth_metadata if isinstance(depth_metadata, dict) else {},
        }

        return WorldObjectState(
            track_id=track_id or f"camera_{plan_data.get('snapshot_id') or 'target'}",
            name=name,
            position_base_m=Position3D(float(point["x"]), float(point["y"]), float(point["z"])),
            semantic_confidence=confidence,
            depth_confidence=self._depth_confidence(metadata, source=depth_source),
            bbox_xyxy=bbox,
            center_uv=center_uv,
            position_std_m=self._position_std(metadata),
            reachable=bool(plan_data.get("workspace_ok", True)),
            metadata=metadata,
        )

    def build_state_from_camera_plan(
        self,
        user_command: str,
        plan_data: Dict[str, Any],
        controller: LebaiController,
    ) -> WorldState:
        target = self.object_from_camera_plan(plan_data)
        return WorldState(
            objects=[target],
            robot=self.robot_state(controller),
            calibration=self.calibration_state(),
            task=TaskState(user_command=user_command, target_track_id=target.track_id),
            rgb_snapshot_id=str(plan_data.get("snapshot_id") or "") or None,
            depth_snapshot_id=str(plan_data.get("snapshot_id") or "") or None,
            metadata={"adapter": "ExistingProjectAdapter", "dry_run": True},
        )

    @staticmethod
    def _float_tuple(value: Any, length: int, label: str) -> Tuple[float, ...]:
        if not isinstance(value, Sequence) or isinstance(value, (str, bytes)) or len(value) != length:
            raise ValueError(f"{label} 必须包含 {length} 个数值。")
        result = tuple(float(item) for item in value)
        if not all(math.isfinite(item) for item in result):
            raise ValueError(f"{label} 不能包含 NaN 或无穷值。")
        return result

    @staticmethod
    def _center_uv(value: Any) -> Tuple[float, float]:
        if isinstance(value, dict):
            pair = (value.get("u"), value.get("v"))
        elif isinstance(value, Sequence) and not isinstance(value, (str, bytes)) and len(value) == 2:
            pair = (value[0], value[1])
        else:
            raise ValueError("规划结果缺少合法的 center_uv/pixel_center。")
        center = (float(pair[0]), float(pair[1]))
        if not all(math.isfinite(item) for item in center):
            raise ValueError("center_uv 不能包含 NaN 或无穷值。")
        return center

    @staticmethod
    def _bbox(value: Any, center_uv: Tuple[float, float]) -> Tuple[int, int, int, int]:
        if isinstance(value, Sequence) and not isinstance(value, (str, bytes)) and len(value) == 4:
            bbox = tuple(int(round(float(item))) for item in value)
            if bbox[2] > bbox[0] and bbox[3] > bbox[1]:
                return bbox  # type: ignore[return-value]
        u, v = center_uv
        return (int(round(u - 5)), int(round(v - 5)), int(round(u + 5)), int(round(v + 5)))

    @staticmethod
    def _depth_confidence(metadata: Dict[str, Any], source: str) -> float:
        quality = metadata.get("depth_quality")
        if isinstance(quality, dict):
            valid_ratio = _clamp_probability(float(quality.get("valid_ratio", 0.8)))
            std_m = max(float(quality.get("std_m", quality.get("depth_std_m", 0.010))), 0.0)
            return _clamp_probability(0.75 * valid_ratio + 0.25 * (1.0 - min(std_m / 0.030, 1.0)))
        fallback = {
            "bbox_grid_near_surface": 0.85,
            "neighborhood_median": 0.70,
            "center": 0.65,
            "candidate": 0.75,
        }
        return fallback.get(source, 0.60)

    @staticmethod
    def _position_std(metadata: Dict[str, Any]) -> Position3D:
        raw = metadata.get("position_std_m")
        if isinstance(raw, dict):
            return Position3D(max(float(raw.get("x", 0.005)), 0.0), max(float(raw.get("y", 0.005)), 0.0), max(float(raw.get("z", 0.008)), 0.0))
        depth_quality = metadata.get("depth_quality")
        z_std = 0.008
        if isinstance(depth_quality, dict):
            z_std = max(float(depth_quality.get("std_m", depth_quality.get("depth_std_m", z_std))), 0.001)
        return Position3D(max(0.003, z_std * 0.6), max(0.003, z_std * 0.6), z_std)
