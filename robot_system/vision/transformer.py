from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np
import cv2

from robot_system.llm.types import GraspPose, ObjectCandidate, Pose3D


@dataclass
class CalibrationBundle:
    """封装视觉坐标转换所需的标定数据。"""

    camera_matrix: np.ndarray
    dist_coeffs: np.ndarray
    base_T_camera: np.ndarray
    camera_T_base: np.ndarray
    metadata: Dict[str, Any] = field(default_factory=dict)

    @classmethod
    def from_file(cls, path: str | Path) -> "CalibrationBundle":
        """从 Eye-to-Hand 标定求解器导出的 JSON 中读取标定数据。"""
        calibration_path = Path(path)
        with calibration_path.open("r", encoding="utf-8") as file:
            payload = json.load(file)

        camera_matrix = np.asarray(
            payload["camera_intrinsic"]["camera_matrix"],
            dtype=np.float64,
        ).reshape(3, 3)
        dist_coeffs = np.asarray(
            payload["camera_intrinsic"]["dist_coeffs"],
            dtype=np.float64,
        ).reshape(-1, 1)
        base_T_camera = np.asarray(
            payload["hand_eye"]["base_T_camera"],
            dtype=np.float64,
        ).reshape(4, 4)
        camera_T_base = np.asarray(
            payload["hand_eye"]["camera_T_base"],
            dtype=np.float64,
        ).reshape(4, 4)

        arrays = {
            "camera_matrix": camera_matrix,
            "dist_coeffs": dist_coeffs,
            "base_T_camera": base_T_camera,
            "camera_T_base": camera_T_base,
        }
        for name, value in arrays.items():
            if not np.all(np.isfinite(value)):
                raise ValueError(f"标定字段 {name} 包含 NaN 或无穷值。")
        if camera_matrix[0, 0] <= 0.0 or camera_matrix[1, 1] <= 0.0:
            raise ValueError("相机焦距 fx/fy 必须大于 0。")
        expected_last_row = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
        if not np.allclose(base_T_camera[3], expected_last_row, atol=1e-8):
            raise ValueError("base_T_camera 不是合法的齐次变换矩阵。")
        if not np.allclose(camera_T_base[3], expected_last_row, atol=1e-8):
            raise ValueError("camera_T_base 不是合法的齐次变换矩阵。")
        if not np.allclose(base_T_camera @ camera_T_base, np.eye(4), atol=1e-5):
            raise ValueError("base_T_camera 与 camera_T_base 不是互逆矩阵。")
        rotation = base_T_camera[:3, :3]
        if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-4) or not np.isclose(
            np.linalg.det(rotation), 1.0, atol=1e-4
        ):
            raise ValueError("base_T_camera 的旋转部分不是合法旋转矩阵。")

        return cls(
            camera_matrix=camera_matrix,
            dist_coeffs=dist_coeffs,
            base_T_camera=base_T_camera,
            camera_T_base=camera_T_base,
            metadata=payload,
        )


@dataclass
class Detection2D:
    """
    视觉模块给出的二维检测结果。

    约定：
    1. `u, v` 是彩色图像中的像素坐标。
    2. `depth` 是与该像素对应的深度值。
    3. `depth_unit` 支持 `"m"` / `"mm"`。
    4. `yaw_rad` 可作为平面内抓取朝向提示。
    """

    object_id: str
    name: str
    u: float
    v: float
    depth: float
    confidence: float
    display_name: Optional[str] = None
    color: Optional[str] = None
    depth_unit: str = "m"
    yaw_rad: float = 0.0
    tags: List[str] = field(default_factory=list)
    metadata: Dict[str, Any] = field(default_factory=dict)
    size_xyz_m: Optional[Tuple[float, float, float]] = None


class VisionTransformer:
    """
    模块一：视觉与坐标转换核心类。

    提供能力：
    1. 从标定文件读取相机内参与 `base_T_camera`
    2. 像素 + 深度 -> 相机坐标
    3. 相机坐标 -> Base 坐标
    4. Base 点 -> 相机坐标 / 像素平面
    5. 把 2D 检测结果直接转换成模块二可用的 `ObjectCandidate`
    """

    def __init__(self, calibration: CalibrationBundle) -> None:
        self.calibration = calibration
        self.fx = float(calibration.camera_matrix[0, 0])
        self.fy = float(calibration.camera_matrix[1, 1])
        self.cx = float(calibration.camera_matrix[0, 2])
        self.cy = float(calibration.camera_matrix[1, 2])

    @classmethod
    def from_calibration_file(cls, path: str | Path) -> "VisionTransformer":
        return cls(CalibrationBundle.from_file(path))

    def pixel_to_camera(
        self,
        u: float,
        v: float,
        depth: float,
        depth_unit: str = "m",
    ) -> Pose3D:
        """
        把像素坐标与深度值投影到相机坐标系。

        数学关系：
            X = (u - cx) / fx * Z
            Y = (v - cy) / fy * Z
            Z = depth
        """
        z_m = self._normalize_depth(depth, depth_unit)
        pixel = np.asarray([[[float(u), float(v)]]], dtype=np.float64)
        normalized = cv2.undistortPoints(
            pixel,
            self.calibration.camera_matrix,
            self.calibration.dist_coeffs,
        ).reshape(2)
        x_m = float(normalized[0]) * z_m
        y_m = float(normalized[1]) * z_m
        return Pose3D(x=x_m, y=y_m, z=z_m)

    def camera_to_base(self, point_camera: Pose3D) -> Pose3D:
        """把相机坐标系中的三维点转换到机械臂 Base 坐标系。"""
        homogeneous = np.array(
            [point_camera.x, point_camera.y, point_camera.z, 1.0],
            dtype=np.float64,
        )
        transformed = self.calibration.base_T_camera @ homogeneous
        return Pose3D(
            x=float(transformed[0]),
            y=float(transformed[1]),
            z=float(transformed[2]),
        )

    def base_to_camera(self, point_base: Pose3D) -> Pose3D:
        """把 Base 坐标系中的三维点反向变换到相机坐标系。"""
        homogeneous = np.array(
            [point_base.x, point_base.y, point_base.z, 1.0],
            dtype=np.float64,
        )
        transformed = self.calibration.camera_T_base @ homogeneous
        return Pose3D(
            x=float(transformed[0]),
            y=float(transformed[1]),
            z=float(transformed[2]),
        )

    def camera_to_pixel(self, point_camera: Pose3D) -> Tuple[float, float, float]:
        """把相机坐标系三维点投影回像素平面，返回 `(u, v, depth_m)`。"""
        if point_camera.z <= 0.0:
            raise ValueError("该点位于相机后方或深度非正，无法投影到图像平面。")

        object_point = np.asarray(
            [[[point_camera.x, point_camera.y, point_camera.z]]],
            dtype=np.float64,
        )
        projected, _ = cv2.projectPoints(
            object_point,
            np.zeros(3, dtype=np.float64),
            np.zeros(3, dtype=np.float64),
            self.calibration.camera_matrix,
            self.calibration.dist_coeffs,
        )
        u, v = projected.reshape(2)
        return float(u), float(v), float(point_camera.z)

    def assert_image_size(self, image_size_wh: Sequence[int]) -> None:
        expected = self.calibration.metadata.get("image_size_wh")
        if not isinstance(expected, (list, tuple)) or len(expected) != 2:
            raise ValueError("标定文件缺少合法的 image_size_wh。")
        actual = [int(image_size_wh[0]), int(image_size_wh[1])]
        expected_size = [int(expected[0]), int(expected[1])]
        if actual != expected_size:
            raise ValueError(
                f"当前图像分辨率与标定不一致: actual={actual}, calibration={expected_size}。"
            )

    def base_to_pixel(self, point_base: Pose3D) -> Tuple[float, float, float]:
        """把 Base 点一步反投影到图像像素平面，返回 `(u, v, depth_m)`。"""
        point_camera = self.base_to_camera(point_base)
        return self.camera_to_pixel(point_camera)

    def pixel_to_base(
        self,
        u: float,
        v: float,
        depth: float,
        depth_unit: str = "m",
    ) -> Pose3D:
        """一站式完成像素坐标到 Base 坐标系的转换。"""
        point_camera = self.pixel_to_camera(u=u, v=v, depth=depth, depth_unit=depth_unit)
        return self.camera_to_base(point_camera)

    def build_candidate_from_detection(
        self,
        detection: Detection2D,
        pre_grasp_offset_m: float = 0.08,
        top_down_pitch_rad: float = np.pi,
    ) -> ObjectCandidate:
        """把 2D 检测结果转换成模块二可直接消费的候选目标。"""
        position_base = self.pixel_to_base(
            u=detection.u,
            v=detection.v,
            depth=detection.depth,
            depth_unit=detection.depth_unit,
        )

        grasp_pose = GraspPose(
            position=position_base,
            roll=0.0,
            pitch=float(top_down_pitch_rad),
            yaw=float(detection.yaw_rad),
            approach_vector=Pose3D(0.0, 0.0, -1.0),
            pre_grasp_offset_m=pre_grasp_offset_m,
        )

        size_xyz_m = None
        if detection.size_xyz_m is not None:
            size_xyz_m = Pose3D(
                x=float(detection.size_xyz_m[0]),
                y=float(detection.size_xyz_m[1]),
                z=float(detection.size_xyz_m[2]),
            )

        metadata = {
            **detection.metadata,
            "pixel_center": {"u": detection.u, "v": detection.v},
            "depth_m": self._normalize_depth(detection.depth, detection.depth_unit),
        }

        return ObjectCandidate(
            object_id=detection.object_id,
            name=detection.name,
            display_name=detection.display_name,
            position=position_base,
            confidence=float(detection.confidence),
            color=detection.color,
            size_xyz_m=size_xyz_m,
            grasp_pose=grasp_pose,
            tags=list(detection.tags),
            metadata=metadata,
        )

    def build_candidates_from_detections(
        self,
        detections: Sequence[Detection2D],
        pre_grasp_offset_m: float = 0.08,
    ) -> List[ObjectCandidate]:
        """批量把 2D 检测结果转换成模块二使用的候选物体。"""
        return [
            self.build_candidate_from_detection(
                detection=item,
                pre_grasp_offset_m=pre_grasp_offset_m,
            )
            for item in detections
        ]

    def _normalize_depth(self, depth: float, depth_unit: str) -> float:
        unit = depth_unit.strip().lower()
        if unit == "m":
            value = float(depth)
            if not np.isfinite(value) or value <= 0.0:
                raise ValueError("depth 必须是大于 0 的有限值。")
            return value
        if unit == "mm":
            value = float(depth) / 1000.0
            if not np.isfinite(value) or value <= 0.0:
                raise ValueError("depth 必须是大于 0 的有限值。")
            return value
        raise ValueError("depth_unit 只允许 'm' 或 'mm'")
