from __future__ import annotations

import importlib.util
import json
import math
import os
import tempfile
import time
from dataclasses import asdict, dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

import cv2
import numpy as np

from robot_system.control import LebaiController, WorkspaceBox
from robot_system.llm.types import Pose3D
from robot_system.vision import CalibrationBundle, OrbbecDepthCamera


@dataclass(frozen=True)
class AutoCalibrationPoseDelta:
    """Relative TCP pose perturbation used by the automatic calibration scan."""

    dx: float = 0.0
    dy: float = 0.0
    dz: float = 0.0
    drx: float = 0.0
    dry: float = 0.0
    drz: float = 0.0
    label: str = "pose"

    def to_dict(self) -> Dict[str, float | str]:
        return asdict(self)


@dataclass
class AutoCalibrationConfig:
    """
    Automatic eye-to-hand calibration settings.

    The robot starts from the current operator-approved pose. Each sample is a
    small local perturbation around that pose, which keeps the board in the
    camera view and avoids broad blind robot motion.
    """

    data_root: Path = Path("calib_data")
    output_dir: Path = Path("biaoding")
    session_name: Optional[str] = None
    sample_count: int = 20
    min_success_samples: int = 12
    settle_sec: float = 0.8
    execute_motion: bool = True
    auto_start_system: bool = False
    run_calibration: bool = True
    board_inner_corners: Tuple[int, int] = (8, 11)
    keep_rejected_images: bool = True
    pose_deltas: List[AutoCalibrationPoseDelta] = field(default_factory=list)


class AutoCalibrationRunner:
    """Run safe automatic data collection and then reuse 02_calibrate.py."""

    def __init__(
        self,
        controller: LebaiController,
        camera: OrbbecDepthCamera,
        config: Optional[AutoCalibrationConfig] = None,
    ) -> None:
        self.controller = controller
        self.camera = camera
        self.config = config or AutoCalibrationConfig()

    def build_default_pose_deltas(self, sample_count: Optional[int] = None) -> List[AutoCalibrationPoseDelta]:
        requested = int(sample_count or self.config.sample_count)
        base_pattern = [
            AutoCalibrationPoseDelta(label="center"),
            AutoCalibrationPoseDelta(dx=0.025, dry=0.08, label="x_plus_pitch"),
            AutoCalibrationPoseDelta(dx=-0.025, dry=-0.08, label="x_minus_pitch"),
            AutoCalibrationPoseDelta(dy=0.025, drx=-0.08, label="y_plus_roll"),
            AutoCalibrationPoseDelta(dy=-0.025, drx=0.08, label="y_minus_roll"),
            AutoCalibrationPoseDelta(dz=0.020, drz=0.10, label="z_plus_yaw"),
            AutoCalibrationPoseDelta(dz=-0.015, drz=-0.10, label="z_minus_yaw"),
            AutoCalibrationPoseDelta(dx=0.035, dy=0.020, drx=0.06, dry=0.06, label="xy_plus_tilt"),
            AutoCalibrationPoseDelta(dx=-0.035, dy=-0.020, drx=-0.06, dry=-0.06, label="xy_minus_tilt"),
            AutoCalibrationPoseDelta(dx=0.020, dy=-0.030, drx=0.10, label="diag_a"),
            AutoCalibrationPoseDelta(dx=-0.020, dy=0.030, dry=-0.10, label="diag_b"),
            AutoCalibrationPoseDelta(dx=0.045, dz=0.010, drz=-0.12, label="wide_x_plus"),
            AutoCalibrationPoseDelta(dx=-0.045, dz=0.010, drz=0.12, label="wide_x_minus"),
            AutoCalibrationPoseDelta(dy=0.045, dz=0.005, drx=-0.12, label="wide_y_plus"),
            AutoCalibrationPoseDelta(dy=-0.045, dz=0.005, drx=0.12, label="wide_y_minus"),
            AutoCalibrationPoseDelta(dx=0.025, dy=0.025, dz=0.020, dry=0.12, label="high_a"),
            AutoCalibrationPoseDelta(dx=-0.025, dy=-0.025, dz=0.020, drx=-0.12, label="high_b"),
            AutoCalibrationPoseDelta(dx=0.030, dy=-0.025, dz=-0.010, drx=0.08, dry=-0.08, label="low_a"),
            AutoCalibrationPoseDelta(dx=-0.030, dy=0.025, dz=-0.010, drx=-0.08, dry=0.08, label="low_b"),
            AutoCalibrationPoseDelta(drx=0.14, dry=-0.10, drz=0.08, label="tilt_combo"),
        ]
        if requested <= len(base_pattern):
            return base_pattern[:requested]

        deltas = list(base_pattern)
        while len(deltas) < requested:
            index = len(deltas)
            angle = (index - len(base_pattern) + 1) * math.pi / 4.0
            radius = 0.025 + 0.004 * ((index - len(base_pattern)) % 3)
            deltas.append(
                AutoCalibrationPoseDelta(
                    dx=radius * math.cos(angle),
                    dy=radius * math.sin(angle),
                    dz=0.010 * math.sin(angle * 0.5),
                    drx=0.08 * math.sin(angle),
                    dry=0.08 * math.cos(angle),
                    drz=0.06 * math.sin(angle * 1.5),
                    label=f"extra_{index:02d}",
                )
            )
        return deltas

    def preview_plan(self) -> Dict[str, Any]:
        deltas = self.config.pose_deltas or self.build_default_pose_deltas()
        return {
            "sample_count": len(deltas),
            "min_success_samples": self.config.min_success_samples,
            "settle_sec": self.config.settle_sec,
            "execute_motion": self.config.execute_motion,
            "run_calibration": self.config.run_calibration,
            "board_inner_corners": list(self.config.board_inner_corners),
            "pose_deltas": [delta.to_dict() for delta in deltas],
        }

    def run(self) -> Dict[str, Any]:
        deltas = self.config.pose_deltas or self.build_default_pose_deltas()
        if int(self.config.min_success_samples) > len(deltas):
            raise ValueError(
                f"min_success_samples={self.config.min_success_samples} 不能大于计划样本数 {len(deltas)}。"
            )
        session_dir = self._create_session_dir()
        rejected_dir = session_dir / "rejected"
        rejected_dir.mkdir(parents=True, exist_ok=True)

        report: Dict[str, Any] = {
            "success": False,
            "session_dir": str(session_dir),
            "started_at": datetime.now().isoformat(timespec="seconds"),
            "config": self._jsonable_config(),
            "samples": [],
            "rejected": [],
            "calibration": None,
        }

        self.controller.connect()
        try:
            if self.config.auto_start_system:
                self.controller.start_system()
            self.controller.assert_robot_can_move()

            start_pose = self.controller.get_actual_tcp_pose()
            start_joints = self.controller.get_actual_joint_pose()
            report["start_pose"] = start_pose
            report["start_joints"] = [float(value) for value in start_joints]

            valid_index = 0
            reference_joints: Sequence[float] = start_joints
            for attempt_index, delta in enumerate(deltas):
                target_pose = self._apply_delta(start_pose, delta)
                attempt: Dict[str, Any] = {
                    "attempt_index": attempt_index,
                    "delta": delta.to_dict(),
                    "target_pose": target_pose,
                }

                try:
                    joints = self._validate_target_pose(target_pose, reference_joints, label=f"auto_calib_{attempt_index:03d}")
                    attempt["target_joints"] = [float(value) for value in joints]

                    if self.config.execute_motion:
                        self.controller.move_joint(joints)
                        time.sleep(max(float(self.config.settle_sec), 0.0))
                        actual_pose = self.controller.get_actual_tcp_pose()
                        actual_joints = self.controller.get_actual_joint_pose()
                        reference_joints = actual_joints
                    else:
                        actual_pose = target_pose
                        actual_joints = joints

                    frame = self.camera.capture_snapshot().color_bgr
                    found, corners = self._detect_chessboard(frame)
                    if not found:
                        attempt["reason"] = "chessboard_not_found"
                        self._store_rejected(rejected_dir, attempt_index, frame, attempt)
                        report["rejected"].append(attempt)
                        continue

                    sample_id = f"{valid_index:03d}"
                    image_name = f"img_{sample_id}.png"
                    pose_name = f"pose_{sample_id}.json"
                    image_path = session_dir / image_name
                    pose_path = session_dir / pose_name
                    if not cv2.imwrite(str(image_path), frame):
                        raise RuntimeError(f"写入标定图像失败: {image_path}")
                    pose_record = {
                        "sample_id": sample_id,
                        "image_file": image_name,
                        "robot_pose": self._normalize_robot_pose(actual_pose),
                        "T_base_to_tcp": self._pose_to_homogeneous(actual_pose).tolist(),
                        "actual_joint_pose": [float(value) for value in actual_joints],
                        "auto_calibration": {
                            "attempt_index": attempt_index,
                            "delta": delta.to_dict(),
                            "target_pose": target_pose,
                            "target_joints": attempt["target_joints"],
                            "chessboard_corner_count": int(len(corners) if corners is not None else 0),
                        },
                    }
                    with pose_path.open("w", encoding="utf-8") as file:
                        json.dump(pose_record, file, ensure_ascii=False, indent=2)

                    sample_payload = {
                        **attempt,
                        "sample_id": sample_id,
                        "image_file": image_name,
                        "pose_file": pose_name,
                        "actual_pose": pose_record["robot_pose"],
                        "accepted": True,
                    }
                    report["samples"].append(sample_payload)
                    valid_index += 1
                except Exception as exc:
                    attempt["reason"] = str(exc)
                    report["rejected"].append(attempt)

            report["used_count"] = len(report["samples"])
            if report["used_count"] < int(self.config.min_success_samples):
                raise RuntimeError(
                    f"有效自动标定样本不足: {report['used_count']} / {self.config.min_success_samples}. "
                    "请把标定板重新放回画面中央，或缩小自动姿态扰动范围后再试。"
                )

            if self.config.run_calibration:
                report["calibration"] = self._run_calibration(session_dir)

            report["success"] = True
            report["message"] = "自动标定完成。"
            return report
        finally:
            report["finished_at"] = datetime.now().isoformat(timespec="seconds")
            self._write_report(session_dir, report)
            try:
                self.camera.close()
            finally:
                self.controller.disconnect()

    def _create_session_dir(self) -> Path:
        name = self.config.session_name
        if not name:
            name = "auto_" + datetime.now().strftime("%Y%m%d_%H%M%S")
        safe_name = "".join(char if char.isalnum() or char in {"_", "-"} else "_" for char in name)
        session_dir = Path(self.config.data_root) / safe_name
        session_dir.mkdir(parents=True, exist_ok=False)
        return session_dir

    def _validate_target_pose(
        self,
        target_pose: Dict[str, float],
        reference_joints: Sequence[float],
        label: str,
    ) -> List[float]:
        self.controller.config.workspace.assert_contains(
            Pose3D(target_pose["x"], target_pose["y"], target_pose["z"]),
            label,
        )
        joints = self.controller.solve_ik(target_pose, reference_joints)
        check_limits = getattr(self.controller, "_check_joint_limits", None)
        if callable(check_limits):
            check_limits(joints, label)
        check_singularity = getattr(self.controller, "_check_singularity_risk", None)
        if callable(check_singularity):
            check_singularity(joints, label)
        return [float(value) for value in joints]

    def _apply_delta(self, base_pose: Dict[str, float], delta: AutoCalibrationPoseDelta) -> Dict[str, float]:
        return {
            "x": float(base_pose["x"]) + float(delta.dx),
            "y": float(base_pose["y"]) + float(delta.dy),
            "z": float(base_pose["z"]) + float(delta.dz),
            "rz": float(base_pose["rz"]) + float(delta.drz),
            "ry": float(base_pose["ry"]) + float(delta.dry),
            "rx": float(base_pose["rx"]) + float(delta.drx),
        }

    def _detect_chessboard(self, frame_bgr: np.ndarray) -> Tuple[bool, Optional[np.ndarray]]:
        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        pattern = tuple(int(value) for value in self.config.board_inner_corners)

        found = False
        corners: Optional[np.ndarray] = None
        if hasattr(cv2, "findChessboardCornersSB"):
            found, corners = cv2.findChessboardCornersSB(gray, pattern, None)
        if not found:
            found, corners = cv2.findChessboardCorners(
                gray,
                pattern,
                flags=cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE,
            )
            if found and corners is not None:
                criteria = (
                    cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
                    30,
                    0.001,
                )
                corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        return bool(found), corners

    def _store_rejected(
        self,
        rejected_dir: Path,
        attempt_index: int,
        frame: np.ndarray,
        attempt: Dict[str, Any],
    ) -> None:
        if not self.config.keep_rejected_images:
            return
        image_name = f"rejected_{attempt_index:03d}.png"
        json_name = f"rejected_{attempt_index:03d}.json"
        cv2.imwrite(str(rejected_dir / image_name), frame)
        with (rejected_dir / json_name).open("w", encoding="utf-8") as file:
            json.dump(attempt, file, ensure_ascii=False, indent=2)
        attempt["rejected_image_file"] = str(Path("rejected") / image_name)
        attempt["rejected_json_file"] = str(Path("rejected") / json_name)

    def _run_calibration(self, session_dir: Path) -> Dict[str, Any]:
        project_root = Path(__file__).resolve().parents[2]
        script_path = project_root / "02_calibrate.py"
        spec = importlib.util.spec_from_file_location("auto_calibrate_script", script_path)
        if spec is None or spec.loader is None:
            raise RuntimeError(f"无法加载标定脚本: {script_path}")

        output_dir = Path(self.config.output_dir)
        output_dir.mkdir(parents=True, exist_ok=True)
        output_path = output_dir / "eye_to_hand_result.json"
        with tempfile.TemporaryDirectory(prefix=".auto_calibration_", dir=str(output_dir)) as temp_dir:
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            module.DATA_DIR = Path(session_dir)
            module.OUTPUT_DIR = Path(temp_dir)
            return_code = int(module.main_v2())
            staged_output = Path(temp_dir) / "eye_to_hand_result.json"
            if return_code != 0:
                raise RuntimeError(f"02_calibrate.py 标定失败，return_code={return_code}。")
            if not staged_output.exists():
                raise RuntimeError("标定脚本返回成功，但没有生成 eye_to_hand_result.json。")
            CalibrationBundle.from_file(staged_output)
            os.replace(staged_output, output_path)
        return {
            "return_code": return_code,
            "output_path": str(output_path),
            "output_exists": True,
        }

    def _write_report(self, session_dir: Path, report: Dict[str, Any]) -> None:
        try:
            with (session_dir / "auto_calibration_report.json").open("w", encoding="utf-8") as file:
                json.dump(report, file, ensure_ascii=False, indent=2)
        except Exception:
            pass

    def _jsonable_config(self) -> Dict[str, Any]:
        payload = asdict(self.config)
        payload["data_root"] = str(self.config.data_root)
        payload["output_dir"] = str(self.config.output_dir)
        payload["board_inner_corners"] = list(self.config.board_inner_corners)
        payload["pose_deltas"] = [delta.to_dict() for delta in self.config.pose_deltas]
        return payload

    def _normalize_robot_pose(self, pose: Dict[str, Any]) -> Dict[str, float]:
        return {
            "x": float(pose["x"]),
            "y": float(pose["y"]),
            "z": float(pose["z"]),
            "rz": float(pose["rz"]),
            "ry": float(pose["ry"]),
            "rx": float(pose["rx"]),
        }

    def _pose_to_homogeneous(self, pose: Dict[str, Any]) -> np.ndarray:
        normalized = self._normalize_robot_pose(pose)
        transform = np.eye(4, dtype=np.float64)
        transform[:3, :3] = self._rotation_matrix_from_zyx_euler(
            normalized["rz"],
            normalized["ry"],
            normalized["rx"],
        )
        transform[:3, 3] = np.array(
            [normalized["x"], normalized["y"], normalized["z"]],
            dtype=np.float64,
        )
        return transform

    def _rotation_matrix_from_zyx_euler(self, rz: float, ry: float, rx: float) -> np.ndarray:
        cz, sz = np.cos(rz), np.sin(rz)
        cy, sy = np.cos(ry), np.sin(ry)
        cx, sx = np.cos(rx), np.sin(rx)
        rot_z = np.array([[cz, -sz, 0.0], [sz, cz, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)
        rot_y = np.array([[cy, 0.0, sy], [0.0, 1.0, 0.0], [-sy, 0.0, cy]], dtype=np.float64)
        rot_x = np.array([[1.0, 0.0, 0.0], [0.0, cx, -sx], [0.0, sx, cx]], dtype=np.float64)
        return rot_z @ rot_y @ rot_x
