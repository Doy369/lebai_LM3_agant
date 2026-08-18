"""
眼在手外（Eye-to-Hand）手眼标定脚本。

适用场景：
1. 相机固定在机器人工作区外部；
2. 标定板刚性固定在夹爪/工具上；
3. 已经通过 01_collect_data.py 采集了图像和对应的机器人 TCP 位姿。

脚本输出：
1. 相机内参 camera_matrix / dist_coeffs
2. 机械臂基坐标到相机坐标的齐次矩阵 base_T_camera
   其含义是：p_base = base_T_camera @ p_camera
3. 若干诊断指标，帮助判断标定结果是否可信
"""

from __future__ import annotations

import configparser
import json
import math
import os
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import cv2
import numpy as np


# =========================
# 可按现场情况修改的配置
# =========================
DATA_DIR = Path("calib_data")
OUTPUT_DIR = Path("biaoding")

# 棋盘格参数。
# 注意：OpenCV 的 CHESSBOARD_SIZE 需要填写“内角点数量”，不是格子数量。
# 如果你的板子是 9 格 * 12 格，那么内角点通常应为 8 * 11。
BOARD_DIMENSION_MODE = "inner_corners"  # "squares" 或 "inner_corners"
BOARD_GRID_SHAPE = (8, 11)  # 若 mode="squares"，表示格子数；若为 "inner_corners"，表示内角点数
SQUARE_SIZE_M = 0.003

if BOARD_DIMENSION_MODE == "squares":
    if BOARD_GRID_SHAPE[0] < 2 or BOARD_GRID_SHAPE[1] < 2:
        raise ValueError("当 BOARD_DIMENSION_MODE='squares' 时，棋盘格边长方向至少要有 2 个格子。")
    CHESSBOARD_SIZE = (BOARD_GRID_SHAPE[0] - 1, BOARD_GRID_SHAPE[1] - 1)
elif BOARD_DIMENSION_MODE == "inner_corners":
    CHESSBOARD_SIZE = BOARD_GRID_SHAPE
else:
    raise ValueError("BOARD_DIMENSION_MODE 只允许 'squares' 或 'inner_corners'")

# 实际上建议至少 12~20 张姿态差异足够大的样本。
MIN_VALID_SAMPLES = 8

# 兼容旧采集脚本中仅保存 6 维列表位姿的情况。
# 可选值：
# 1. "x_y_z_rx_ry_rz": 对应你最早版本 01_collect_data.py 中 dict 分支保存顺序
# 2. "x_y_z_rz_ry_rx": 对应部分 SDK / Lua 风格数组顺序
LEGACY_POSE_LIST_ORDER = "x_y_z_rx_ry_rz"

# 相机内参来源：
# 1. "preset": 使用相机厂家/工具导出的内参，推荐用于 Gemini 这类已有工厂标定的相机
# 2. "estimate": 从棋盘图像重新估计内参
INTRINSIC_SOURCE = "preset"

# 这里建议放 Gemini 工具导出的“与你实际采图流一致”的内参。
# 如果你采集的是 RGB 图像，就放 RGB 内参；如果采集的是 IR/深度图像，就放对应流的内参。
# 当前默认指向你放在工作目录下的 Orbbec 导出文件。
PRESET_INTRINSICS_PATH = Path(
    os.getenv("LEBAI_CAMERA_INTRINSICS_FILE", "CameraParam_Orbbec.example.ini")
)
PRESET_INTRINSIC_STREAM_NAME = "color"

# 自动坏帧剔除与翻转嫌疑诊断配置。
# 这一步不会修改原始采集数据，只会在标定阶段跳过明显坏帧，并把嫌疑样本写入结果文件。
AUTO_REJECT_REPROJECTION_OUTLIERS = True
REPROJECTION_ERROR_THRESHOLD_MIN_PX = 0.20
REPROJECTION_ERROR_THRESHOLD_MAX_PX = 1.00
REPROJECTION_ERROR_MAD_SCALE = 3.5
ROTATION_FLIP_SUSPECT_THRESHOLD_DEG = 120.0
ROTATION_OUTLIER_REPORT_TOPK = 5


def rotation_matrix_from_zyx_euler(rz: float, ry: float, rx: float) -> np.ndarray:
    """根据乐白常见的 Z-Y-X 欧拉角定义构造旋转矩阵。"""
    cz, sz = np.cos(rz), np.sin(rz)
    cy, sy = np.cos(ry), np.sin(ry)
    cx, sx = np.cos(rx), np.sin(rx)

    rot_z = np.array(
        [[cz, -sz, 0.0], [sz, cz, 0.0], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )
    rot_y = np.array(
        [[cy, 0.0, sy], [0.0, 1.0, 0.0], [-sy, 0.0, cy]],
        dtype=np.float64,
    )
    rot_x = np.array(
        [[1.0, 0.0, 0.0], [0.0, cx, -sx], [0.0, sx, cx]],
        dtype=np.float64,
    )
    return rot_z @ rot_y @ rot_x


def pose_to_homogeneous(pose: Dict[str, float]) -> np.ndarray:
    """把 Base -> TCP 位姿转成 4x4 齐次矩阵。"""
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = rotation_matrix_from_zyx_euler(
        pose["rz"], pose["ry"], pose["rx"]
    )
    transform[:3, 3] = np.array([pose["x"], pose["y"], pose["z"]], dtype=np.float64)
    return transform


def invert_transform(transform: np.ndarray) -> np.ndarray:
    """刚体变换求逆。"""
    rotation = transform[:3, :3]
    translation = transform[:3, 3]
    inverted = np.eye(4, dtype=np.float64)
    inverted[:3, :3] = rotation.T
    inverted[:3, 3] = -rotation.T @ translation
    return inverted


def homogeneous_from_rt(rotation: np.ndarray, translation: np.ndarray) -> np.ndarray:
    """由 R / t 构造 4x4 齐次矩阵。"""
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = rotation
    transform[:3, 3] = np.asarray(translation, dtype=np.float64).reshape(3)
    return transform


def normalize_pose_dict(raw_pose: Dict[str, Any]) -> Dict[str, float]:
    """兼容常见 dict 字段命名。"""
    normalized = {
        "x": raw_pose.get("x", raw_pose.get("X")),
        "y": raw_pose.get("y", raw_pose.get("Y")),
        "z": raw_pose.get("z", raw_pose.get("Z")),
        "rz": raw_pose.get("rz", raw_pose.get("Rz")),
        "ry": raw_pose.get("ry", raw_pose.get("Ry")),
        "rx": raw_pose.get("rx", raw_pose.get("Rx")),
    }
    if any(value is None for value in normalized.values()):
        raise ValueError(f"位姿字段不完整：{raw_pose}")
    return {key: float(value) for key, value in normalized.items()}


def parse_pose_record(raw_record: Any) -> Tuple[np.ndarray, Dict[str, Any]]:
    """
    兼容两种采集格式：
    1. 新格式：保存完整字典，包含 robot_pose / T_base_to_tcp
    2. 旧格式：仅保存 [x, y, z, rx, ry, rz]
    """
    if isinstance(raw_record, dict):
        if "T_base_to_tcp" in raw_record:
            matrix = np.asarray(raw_record["T_base_to_tcp"], dtype=np.float64)
            if matrix.shape != (4, 4):
                raise ValueError("T_base_to_tcp 不是 4x4 矩阵。")
            return matrix, raw_record

        if "robot_pose" in raw_record:
            pose = normalize_pose_dict(raw_record["robot_pose"])
            return pose_to_homogeneous(pose), raw_record

        if {"x", "y", "z"} <= set(raw_record.keys()) or {"X", "Y", "Z"} <= set(raw_record.keys()):
            pose = normalize_pose_dict(raw_record)
            return pose_to_homogeneous(pose), raw_record

    if isinstance(raw_record, list) and len(raw_record) == 6:
        # 兼容早期只保存 6 维列表的格式。这里必须显式声明顺序，
        # 否则一旦把 [rz, ry, rx] 错当成 [rx, ry, rz]，手眼结果会整体错误。
        if LEGACY_POSE_LIST_ORDER == "x_y_z_rx_ry_rz":
            x, y, z, rx, ry, rz = raw_record
        elif LEGACY_POSE_LIST_ORDER == "x_y_z_rz_ry_rx":
            x, y, z, rz, ry, rx = raw_record
        else:
            raise ValueError(
                "LEGACY_POSE_LIST_ORDER 配置非法，"
                "只允许 'x_y_z_rx_ry_rz' 或 'x_y_z_rz_ry_rx'"
            )

        pose = {
            "x": float(x),
            "y": float(y),
            "z": float(z),
            "rz": float(rz),
            "ry": float(ry),
            "rx": float(rx),
        }
        return pose_to_homogeneous(pose), {
            "legacy_pose_list": raw_record,
            "legacy_pose_list_order": LEGACY_POSE_LIST_ORDER,
            "robot_pose": pose,
        }

    raise ValueError(f"不支持的 pose 文件格式：{raw_record}")


def find_corresponding_image(pose_path: Path, pose_record: Dict[str, Any]) -> Path:
    """根据 pose 文件找到对应图像。"""
    if isinstance(pose_record, dict) and pose_record.get("image_file"):
        image_path = pose_path.parent / pose_record["image_file"]
        if image_path.exists():
            return image_path

    suffix = pose_path.stem.split("_")[-1]
    candidates = [
        pose_path.parent / f"img_{suffix}.png",
        pose_path.parent / f"img_{suffix}.jpg",
        pose_path.parent / "images" / f"img_{suffix}.png",
        pose_path.parent / "images" / f"img_{suffix}.jpg",
    ]
    for image_path in candidates:
        if image_path.exists():
            return image_path

    raise FileNotFoundError(f"找不到与 {pose_path.name} 对应的图像文件。")


def load_samples(data_dir: Path) -> List[Dict[str, Any]]:
    pose_files = sorted(
        data_dir.glob("pose_*.json"),
        key=lambda path: int(path.stem.split("_")[-1]),
    )
    if not pose_files:
        raise FileNotFoundError(f"{data_dir} 下没有找到 pose_*.json")

    samples = []
    for pose_path in pose_files:
        with pose_path.open("r", encoding="utf-8") as file:
            raw_record = json.load(file)

        base_to_tcp, parsed_record = parse_pose_record(raw_record)
        image_path = find_corresponding_image(pose_path, parsed_record)
        samples.append(
            {
                "sample_id": pose_path.stem.split("_")[-1],
                "pose_path": str(pose_path),
                "image_path": str(image_path),
                "T_base_to_tcp": base_to_tcp,
                "pose_record": parsed_record,
            }
        )
    return samples


def _try_extract_matrix_from_dict(data: Dict[str, Any], candidate_keys: List[str]) -> Optional[np.ndarray]:
    """从 JSON 字典里提取矩阵/数组字段。"""
    for key in candidate_keys:
        if key not in data:
            continue
        value = data[key]
        if isinstance(value, dict) and "data" in value:
            value = value["data"]
        try:
            array = np.asarray(value, dtype=np.float64)
        except Exception:
            continue
        if array.size > 0:
            return array
    return None


def resolve_preset_intrinsics_path(intrinsics_path: Path) -> Path:
    """
    解析预设内参文件路径。

    若显式配置的路径不存在，会尝试在当前目录自动发现唯一的 Orbbec CameraParam 文件。
    """
    if intrinsics_path.exists():
        return intrinsics_path

    candidates = sorted(Path(".").glob("CameraParam_*.ini"))
    if len(candidates) == 1:
        return candidates[0]

    if len(candidates) > 1:
        raise FileNotFoundError(
            "未找到配置指定的预设内参文件，且当前目录存在多个 CameraParam_*.ini，"
            "请把 PRESET_INTRINSICS_PATH 指向具体文件。"
        )

    raise FileNotFoundError(f"预设内参文件不存在：{intrinsics_path}")


def load_preset_intrinsics(intrinsics_path: Path) -> Tuple[np.ndarray, np.ndarray, Dict[str, Any]]:
    """
    读取厂家/工具导出的相机内参。

    当前支持：
    1. JSON
    2. OpenCV YAML / YML / XML（通过 FileStorage 读取）
    """
    intrinsics_path = resolve_preset_intrinsics_path(intrinsics_path)

    suffix = intrinsics_path.suffix.lower()
    camera_matrix: Optional[np.ndarray] = None
    dist_coeffs: Optional[np.ndarray] = None
    meta: Dict[str, Any] = {
        "source": "preset",
        "path": str(intrinsics_path),
        "stream_name": PRESET_INTRINSIC_STREAM_NAME,
    }

    if suffix == ".json":
        with intrinsics_path.open("r", encoding="utf-8") as file:
            raw = json.load(file)

        if isinstance(raw, dict):
            nested_candidates = [
                raw,
                raw.get("camera_intrinsic", {}),
                raw.get("intrinsics", {}),
                raw.get("color", {}),
                raw.get("depth", {}),
            ]

            for candidate in nested_candidates:
                if not isinstance(candidate, dict):
                    continue

                matrix = _try_extract_matrix_from_dict(
                    candidate,
                    ["camera_matrix", "K", "cameraMatrix", "intrinsic_matrix"],
                )
                coeffs = _try_extract_matrix_from_dict(
                    candidate,
                    ["dist_coeffs", "distCoeffs", "distortion_coefficients", "D", "coeffs"],
                )

                if matrix is None and all(key in candidate for key in ("fx", "fy", "cx", "cy")):
                    matrix = np.array(
                        [
                            [float(candidate["fx"]), 0.0, float(candidate["cx"])],
                            [0.0, float(candidate["fy"]), float(candidate["cy"])],
                            [0.0, 0.0, 1.0],
                        ],
                        dtype=np.float64,
                    )

                if matrix is not None:
                    camera_matrix = np.asarray(matrix, dtype=np.float64).reshape(3, 3)
                if coeffs is not None:
                    dist_coeffs = np.asarray(coeffs, dtype=np.float64).reshape(-1, 1)

                if camera_matrix is not None:
                    break

    elif suffix in {".yaml", ".yml", ".xml"}:
        storage = cv2.FileStorage(str(intrinsics_path), cv2.FILE_STORAGE_READ)
        if not storage.isOpened():
            raise RuntimeError(f"无法打开 OpenCV FileStorage 文件：{intrinsics_path}")

        try:
            for key in ("camera_matrix", "K", "cameraMatrix", "intrinsic_matrix"):
                node = storage.getNode(key)
                if not node.empty():
                    matrix = node.mat()
                    if matrix is not None:
                        camera_matrix = np.asarray(matrix, dtype=np.float64).reshape(3, 3)
                        break

            for key in ("dist_coeffs", "distCoeffs", "distortion_coefficients", "D", "coeffs"):
                node = storage.getNode(key)
                if not node.empty():
                    coeffs = node.mat()
                    if coeffs is not None:
                        dist_coeffs = np.asarray(coeffs, dtype=np.float64).reshape(-1, 1)
                        break
        finally:
            storage.release()
    elif suffix == ".ini":
        parser = configparser.ConfigParser()
        parser.read(intrinsics_path, encoding="utf-8")

        stream_name = PRESET_INTRINSIC_STREAM_NAME.strip().lower()
        if stream_name == "color":
            intrinsic_section = "ColorIntrinsic"
            distortion_section = "ColorDistortion"
        elif stream_name == "depth":
            intrinsic_section = "DepthIntrinsic"
            distortion_section = "DepthDistortion"
        else:
            raise ValueError("PRESET_INTRINSIC_STREAM_NAME 只允许 'color' 或 'depth'")

        if intrinsic_section not in parser:
            raise ValueError(f"INI 文件中缺少节：[{intrinsic_section}]")

        intrinsic = parser[intrinsic_section]
        fx = float(intrinsic["fx"])
        fy = float(intrinsic["fy"])
        cx = float(intrinsic["cx"])
        cy = float(intrinsic["cy"])
        camera_matrix = np.array(
            [
                [fx, 0.0, cx],
                [0.0, fy, cy],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        )

        meta["image_size_wh_from_file"] = [
            int(float(intrinsic.get("width", "0"))),
            int(float(intrinsic.get("height", "0"))),
        ]
        meta["orbbec_sections"] = {
            "intrinsic": intrinsic_section,
            "distortion": distortion_section,
        }

        if distortion_section in parser:
            distortion = parser[distortion_section]
            dist_coeffs = np.array(
                [
                    float(distortion.get("k1", 0.0)),
                    float(distortion.get("k2", 0.0)),
                    float(distortion.get("p1", 0.0)),
                    float(distortion.get("p2", 0.0)),
                    float(distortion.get("k3", 0.0)),
                    float(distortion.get("k4", 0.0)),
                    float(distortion.get("k5", 0.0)),
                    float(distortion.get("k6", 0.0)),
                ],
                dtype=np.float64,
            ).reshape(-1, 1)

        if "D2CTransformParam" in parser:
            d2c = parser["D2CTransformParam"]
            rotation = np.array(
                [
                    [float(d2c.get("rot0", 1.0)), float(d2c.get("rot1", 0.0)), float(d2c.get("rot2", 0.0))],
                    [float(d2c.get("rot3", 0.0)), float(d2c.get("rot4", 1.0)), float(d2c.get("rot5", 0.0))],
                    [float(d2c.get("rot6", 0.0)), float(d2c.get("rot7", 0.0)), float(d2c.get("rot8", 1.0))],
                ],
                dtype=np.float64,
            )
            translation = np.array(
                [
                    float(d2c.get("trans0", 0.0)),
                    float(d2c.get("trans1", 0.0)),
                    float(d2c.get("trans2", 0.0)),
                ],
                dtype=np.float64,
            )
            meta["depth_to_color_transform"] = {
                "rotation": rotation.tolist(),
                "translation": translation.tolist(),
            }
    else:
        raise ValueError("预设内参文件只支持 .json / .yaml / .yml / .xml / .ini")

    if camera_matrix is None:
        raise ValueError(
            "未能从预设内参文件中解析出 camera_matrix。"
            "请确认文件里包含 camera_matrix / K / fx fy cx cy 等字段。"
        )

    if dist_coeffs is None:
        dist_coeffs = np.zeros((5, 1), dtype=np.float64)
        meta["dist_coeffs_defaulted_to_zero"] = True
    else:
        meta["dist_coeffs_defaulted_to_zero"] = False

    return camera_matrix, dist_coeffs, meta


def build_chessboard_object_points() -> np.ndarray:
    """构造棋盘格角点在标定板坐标系下的三维坐标。"""
    cols, rows = CHESSBOARD_SIZE
    object_points = np.zeros((rows * cols, 3), dtype=np.float32)
    grid = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    object_points[:, :2] = grid
    object_points *= float(SQUARE_SIZE_M)
    return object_points


def detect_chessboard_corners(image: np.ndarray) -> Tuple[bool, Optional[np.ndarray]]:
    """检测棋盘格角点。"""
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    found = False
    corners = None
    if hasattr(cv2, "findChessboardCornersSB"):
        found, corners = cv2.findChessboardCornersSB(gray, CHESSBOARD_SIZE, None)
    if not found:
        found, corners = cv2.findChessboardCorners(
            gray,
            CHESSBOARD_SIZE,
            flags=cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE,
        )
        if found:
            criteria = (
                cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
                30,
                0.001,
            )
            corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

    return found, corners


def collect_observations(samples: List[Dict[str, Any]]) -> Tuple[List[Dict[str, Any]], Tuple[int, int]]:
    """
    从采集数据中抽取可用于标定的观测。

    返回：
    1. valid_samples：带 image_points / object_points 的有效样本列表
    2. image_size：OpenCV 约定的 (width, height)
    """
    object_points = build_chessboard_object_points()
    valid_samples: List[Dict[str, Any]] = []
    image_size: Optional[Tuple[int, int]] = None

    for sample in samples:
        image = cv2.imread(sample["image_path"])
        if image is None:
            print(f"[WARN] 图像读取失败，跳过：{sample['image_path']}")
            continue

        if image_size is None:
            image_size = (int(image.shape[1]), int(image.shape[0]))
        elif image_size != (int(image.shape[1]), int(image.shape[0])):
            raise ValueError(
                "检测到混合分辨率样本："
                f"首张图像为 {image_size}，当前图像为 {(int(image.shape[1]), int(image.shape[0]))}，"
                "请保证所有标定图像分辨率一致。"
            )

        found, corners = detect_chessboard_corners(image)
        if not found or corners is None:
            print(f"[WARN] 未检测到棋盘格，跳过样本 {sample['sample_id']}")
            continue

        valid_samples.append(
            {
                **sample,
                "object_points": object_points.copy(),
                "image_points": corners.reshape(-1, 2).astype(np.float32),
            }
        )

    if image_size is None:
        raise RuntimeError("所有图像都读取失败，无法继续标定。")

    return valid_samples, image_size


def calibrate_camera_intrinsics(
    valid_samples: List[Dict[str, Any]],
    image_size: Tuple[int, int],
) -> Tuple[float, np.ndarray, np.ndarray, List[np.ndarray], List[np.ndarray], List[float]]:
    """用所有有效视图先标定相机内参，再拿每帧的外参供手眼标定使用。"""
    object_points = [sample["object_points"] for sample in valid_samples]
    image_points = [sample["image_points"] for sample in valid_samples]

    rms, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        object_points,
        image_points,
        image_size,
        None,
        None,
    )

    per_view_errors = []
    for obj_pts, img_pts, rvec, tvec in zip(object_points, image_points, rvecs, tvecs):
        reprojected, _ = cv2.projectPoints(obj_pts, rvec, tvec, camera_matrix, dist_coeffs)
        reprojected = reprojected.reshape(-1, 2)
        error = np.linalg.norm(img_pts - reprojected, axis=1).mean()
        per_view_errors.append(float(error))

    return rms, camera_matrix, dist_coeffs, rvecs, tvecs, per_view_errors


def solve_target_poses_with_known_intrinsics(
    valid_samples: List[Dict[str, Any]],
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
) -> Tuple[List[Dict[str, Any]], List[np.ndarray], List[np.ndarray], List[float]]:
    """
    在内参已知的情况下，对每一帧单独执行 PnP，求得 target -> camera 外参。
    """
    solved_samples: List[Dict[str, Any]] = []
    rvecs: List[np.ndarray] = []
    tvecs: List[np.ndarray] = []
    per_view_errors: List[float] = []

    for sample in valid_samples:
        success, rvec, tvec = cv2.solvePnP(
            sample["object_points"],
            sample["image_points"],
            camera_matrix,
            dist_coeffs,
            flags=cv2.SOLVEPNP_ITERATIVE,
        )
        if not success:
            print(f"[WARN] solvePnP 失败，跳过样本 {sample['sample_id']}")
            continue

        if hasattr(cv2, "solvePnPRefineLM"):
            rvec, tvec = cv2.solvePnPRefineLM(
                sample["object_points"],
                sample["image_points"],
                camera_matrix,
                dist_coeffs,
                rvec,
                tvec,
            )

        reprojected, _ = cv2.projectPoints(
            sample["object_points"],
            rvec,
            tvec,
            camera_matrix,
            dist_coeffs,
        )
        reprojected = reprojected.reshape(-1, 2)
        error = np.linalg.norm(sample["image_points"] - reprojected, axis=1).mean()

        solved_samples.append(sample)
        rvecs.append(np.asarray(rvec, dtype=np.float64).reshape(3, 1))
        tvecs.append(np.asarray(tvec, dtype=np.float64).reshape(3, 1))
        per_view_errors.append(float(error))

    if len(solved_samples) < MIN_VALID_SAMPLES:
        raise RuntimeError(
            "已知内参模式下，成功求出 PnP 外参的样本太少："
            f"{len(solved_samples)} < {MIN_VALID_SAMPLES}"
        )

    return solved_samples, rvecs, tvecs, per_view_errors


def annotate_samples_with_reprojection_errors(
    valid_samples: List[Dict[str, Any]],
    per_view_errors: List[float],
) -> None:
    """把单帧重投影误差回写到样本对象，方便后续诊断。"""
    for sample, error in zip(valid_samples, per_view_errors):
        sample["pnp_reprojection_error_px"] = float(error)


def compute_reprojection_error_threshold(per_view_errors: List[float]) -> Dict[str, float]:
    """
    使用“中位数 + k * MAD”的稳健统计阈值自动识别坏帧。
    同时再限制到一个保守的上下界，避免阈值过宽或过窄。
    """
    if not per_view_errors:
        return {
            "threshold_px": REPROJECTION_ERROR_THRESHOLD_MAX_PX,
            "median_px": 0.0,
            "mad_px": 0.0,
            "robust_sigma_px": 0.0,
        }

    errors = np.asarray(per_view_errors, dtype=np.float64)
    median = float(np.median(errors))
    mad = float(np.median(np.abs(errors - median)))
    robust_sigma = float(1.4826 * mad)
    dynamic_threshold = median + REPROJECTION_ERROR_MAD_SCALE * robust_sigma
    threshold = min(
        REPROJECTION_ERROR_THRESHOLD_MAX_PX,
        max(REPROJECTION_ERROR_THRESHOLD_MIN_PX, dynamic_threshold),
    )
    return {
        "threshold_px": float(threshold),
        "median_px": median,
        "mad_px": mad,
        "robust_sigma_px": robust_sigma,
    }


def filter_reprojection_outliers(
    valid_samples: List[Dict[str, Any]],
    rvecs: List[np.ndarray],
    tvecs: List[np.ndarray],
    per_view_errors: List[float],
) -> Tuple[List[Dict[str, Any]], List[np.ndarray], List[np.ndarray], List[float], Dict[str, Any]]:
    """
    自动剔除单帧重投影误差明显偏大的样本。
    这类样本通常对应角点误检、模糊、部分遮挡或姿态求解不稳定。
    """
    annotate_samples_with_reprojection_errors(valid_samples, per_view_errors)

    if not AUTO_REJECT_REPROJECTION_OUTLIERS:
        return valid_samples, rvecs, tvecs, per_view_errors, {
            "enabled": False,
            "threshold_px": None,
            "median_px": None,
            "mad_px": None,
            "robust_sigma_px": None,
            "rejected_count": 0,
            "rejected_sample_ids": [],
            "rejected_samples": [],
        }

    threshold_info = compute_reprojection_error_threshold(per_view_errors)
    threshold_px = threshold_info["threshold_px"]

    keep_indices: List[int] = []
    rejected_samples: List[Dict[str, Any]] = []
    for index, (sample, error) in enumerate(zip(valid_samples, per_view_errors)):
        if float(error) <= threshold_px:
            keep_indices.append(index)
            continue

        rejected_samples.append(
            {
                "sample_id": sample["sample_id"],
                "image_path": sample["image_path"],
                "pose_path": sample["pose_path"],
                "reprojection_error_px": float(error),
            }
        )

    if not rejected_samples:
        return valid_samples, rvecs, tvecs, per_view_errors, {
            "enabled": True,
            **threshold_info,
            "rejected_count": 0,
            "rejected_sample_ids": [],
            "rejected_samples": [],
        }

    if len(keep_indices) < MIN_VALID_SAMPLES:
        print(
            "[WARN] 按单帧重投影误差自动剔除后，可用样本数不足；"
            "为避免把数据筛得过狠，本次保留全部样本，只输出坏帧诊断。"
        )
        return valid_samples, rvecs, tvecs, per_view_errors, {
            "enabled": True,
            **threshold_info,
            "rejected_count": 0,
            "rejected_sample_ids": [],
            "rejected_samples": [],
            "rejection_skipped_due_to_min_samples": True,
            "would_reject_samples": rejected_samples,
        }

    filtered_samples = [valid_samples[index] for index in keep_indices]
    filtered_rvecs = [rvecs[index] for index in keep_indices]
    filtered_tvecs = [tvecs[index] for index in keep_indices]
    filtered_errors = [per_view_errors[index] for index in keep_indices]

    return filtered_samples, filtered_rvecs, filtered_tvecs, filtered_errors, {
        "enabled": True,
        **threshold_info,
        "rejected_count": len(rejected_samples),
        "rejected_sample_ids": [item["sample_id"] for item in rejected_samples],
        "rejected_samples": rejected_samples,
    }


def validate_preset_intrinsics_for_images(
    image_size: Tuple[int, int],
    intrinsic_meta: Dict[str, Any],
) -> None:
    """
    校验预设内参与当前采集图像分辨率是否匹配。
    """
    preset_size = intrinsic_meta.get("image_size_wh_from_file")
    if not preset_size:
        return

    preset_size_tuple = (int(preset_size[0]), int(preset_size[1]))
    if image_size != preset_size_tuple:
        raise ValueError(
            "预设内参分辨率与采集图像分辨率不一致："
            f"内参文件={preset_size_tuple}, 采集图像={image_size}。"
            "请确认你使用的是同一路视频流的内参。"
        )


def available_hand_eye_methods() -> Dict[str, int]:
    """列出当前 OpenCV 编译版本支持的手眼标定方法。"""
    candidates = {
        "Tsai": getattr(cv2, "CALIB_HAND_EYE_TSAI", None),
        "Park": getattr(cv2, "CALIB_HAND_EYE_PARK", None),
        "Horaud": getattr(cv2, "CALIB_HAND_EYE_HORAUD", None),
        "Andreff": getattr(cv2, "CALIB_HAND_EYE_ANDREFF", None),
        "Daniilidis": getattr(cv2, "CALIB_HAND_EYE_DANIILIDIS", None),
    }
    return {name: value for name, value in candidates.items() if value is not None}


def rotation_distance_deg(rotation_a: np.ndarray, rotation_b: np.ndarray) -> float:
    """计算两个旋转之间的测地角度差（度）。"""
    relative = rotation_a @ rotation_b.T
    trace_value = np.clip((np.trace(relative) - 1.0) * 0.5, -1.0, 1.0)
    angle_rad = math.acos(trace_value)
    return math.degrees(angle_rad)


def choose_rotation_medoid(rotations: List[np.ndarray]) -> np.ndarray:
    """选取“总角距离最小”的旋转作为代表旋转，避免做复杂旋转平均。"""
    if len(rotations) == 1:
        return rotations[0]

    costs = []
    for idx, rotation_i in enumerate(rotations):
        cost = 0.0
        for jdx, rotation_j in enumerate(rotations):
            if idx == jdx:
                continue
            cost += rotation_distance_deg(rotation_i, rotation_j)
        costs.append(cost)
    return rotations[int(np.argmin(costs))]


def evaluate_eye_to_hand_candidate(
    base_to_camera: np.ndarray,
    valid_samples: List[Dict[str, Any]],
    target_to_camera_transforms: List[np.ndarray],
) -> Dict[str, Any]:
    """
    利用“标定板固定在夹爪上”这一先验做一致性检查。

    如果 base_to_camera 合理，那么：
        gripper_T_target_i = gripper_T_base_i @ base_T_camera @ (camera <- target)_i
    在所有样本里应当接近常量。
    """
    gripper_to_target_transforms = []

    for sample, target_to_camera in zip(valid_samples, target_to_camera_transforms):
        gripper_to_base = invert_transform(sample["T_base_to_tcp"])
        gripper_to_target = gripper_to_base @ base_to_camera @ target_to_camera
        gripper_to_target_transforms.append(gripper_to_target)

    translations = np.stack(
        [transform[:3, 3] for transform in gripper_to_target_transforms],
        axis=0,
    )
    rotations = [transform[:3, :3] for transform in gripper_to_target_transforms]
    rotation_medoid = choose_rotation_medoid(rotations)

    translation_mean = translations.mean(axis=0)
    translation_std = translations.std(axis=0)
    rotation_errors_deg = [
        rotation_distance_deg(rotation, rotation_medoid) for rotation in rotations
    ]
    per_sample_consistency = []
    for sample, transform, rotation_error_deg in zip(
        valid_samples,
        gripper_to_target_transforms,
        rotation_errors_deg,
    ):
        per_sample_consistency.append(
            {
                "sample_id": sample["sample_id"],
                "image_path": sample["image_path"],
                "pose_path": sample["pose_path"],
                "reprojection_error_px": sample.get("pnp_reprojection_error_px"),
                "rotation_error_deg": float(rotation_error_deg),
                "translation_xyz_m": transform[:3, 3].astype(float).tolist(),
                "is_flip_suspect": bool(
                    float(rotation_error_deg) >= ROTATION_FLIP_SUSPECT_THRESHOLD_DEG
                ),
            }
        )

    per_sample_consistency_sorted = sorted(
        per_sample_consistency,
        key=lambda item: item["rotation_error_deg"],
        reverse=True,
    )
    flip_suspect_samples = [
        item for item in per_sample_consistency_sorted if item["is_flip_suspect"]
    ]

    representative_gripper_to_target = np.eye(4, dtype=np.float64)
    representative_gripper_to_target[:3, :3] = rotation_medoid
    representative_gripper_to_target[:3, 3] = translation_mean

    # 评分越低越好。这里把平移标准差换算到毫米，再与角度误差做一个简单合成。
    score = float(np.linalg.norm(translation_std) * 1000.0 + np.mean(rotation_errors_deg))

    return {
        "score": score,
        "gripper_T_target_mean": representative_gripper_to_target,
        "translation_mean_m": translation_mean.tolist(),
        "translation_std_m": translation_std.tolist(),
        "rotation_consistency_deg_mean": float(np.mean(rotation_errors_deg)),
        "rotation_consistency_deg_max": float(np.max(rotation_errors_deg)),
        "flip_suspect_samples": flip_suspect_samples,
        "worst_rotation_samples": per_sample_consistency_sorted[:ROTATION_OUTLIER_REPORT_TOPK],
    }


def solve_eye_to_hand(
    valid_samples: List[Dict[str, Any]],
    rvecs: List[np.ndarray],
    tvecs: List[np.ndarray],
) -> Tuple[str, np.ndarray, Dict[str, Any]]:
    """
    对眼在手外场景求解 base_T_camera。

    关键数学关系：
    1. 机器人原始读数通常是 Base -> TCP，记作 base_T_gripper
    2. OpenCV 的 calibrateHandEye 在眼在手外时，需要把求解器第一组绝对位姿
       按照 gripper_T_base 的链路提供给它（即 base_T_gripper 的逆）
    3. 这样求得的结果在数学上对应 base_T_camera

    注意：
    这里故意没有直接使用变量名 R_gripper2base 的语义，而是按“对求解器提供哪组绝对位姿”
    来写，目的是避免眼在手外时最常见的方向误用。
    """
    solver_robot_rotations = []
    solver_robot_translations = []
    target_to_camera_rotations = []
    target_to_camera_translations = []
    target_to_camera_transforms = []

    for sample, rvec, tvec in zip(valid_samples, rvecs, tvecs):
        target_to_camera_rotation, _ = cv2.Rodrigues(rvec)
        target_to_camera = homogeneous_from_rt(target_to_camera_rotation, tvec)

        gripper_to_base = invert_transform(sample["T_base_to_tcp"])

        solver_robot_rotations.append(gripper_to_base[:3, :3])
        solver_robot_translations.append(gripper_to_base[:3, 3].reshape(3, 1))
        target_to_camera_rotations.append(target_to_camera_rotation)
        target_to_camera_translations.append(np.asarray(tvec, dtype=np.float64).reshape(3, 1))
        target_to_camera_transforms.append(target_to_camera)

    method_results = {}
    for method_name, method_flag in available_hand_eye_methods().items():
        try:
            estimated_rotation, estimated_translation = cv2.calibrateHandEye(
                solver_robot_rotations,
                solver_robot_translations,
                target_to_camera_rotations,
                target_to_camera_translations,
                method=method_flag,
            )
            base_to_camera = homogeneous_from_rt(estimated_rotation, estimated_translation)
            if not np.isfinite(base_to_camera).all():
                raise ValueError("求解结果包含 NaN/Inf。")

            diagnostics = evaluate_eye_to_hand_candidate(
                base_to_camera=base_to_camera,
                valid_samples=valid_samples,
                target_to_camera_transforms=target_to_camera_transforms,
            )
            method_results[method_name] = {
                "base_T_camera": base_to_camera,
                **diagnostics,
            }
        except Exception as exc:
            print(f"[WARN] 手眼方法 {method_name} 求解失败：{exc}")

    if not method_results:
        raise RuntimeError("所有 OpenCV 手眼标定方法都失败了。")

    best_method_name = min(
        method_results,
        key=lambda name: method_results[name]["score"],
    )
    return best_method_name, method_results[best_method_name]["base_T_camera"], method_results


def save_results(
    output_dir: Path,
    image_size: Tuple[int, int],
    rms: Optional[float],
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
    detected_valid_sample_count: int,
    valid_samples: List[Dict[str, Any]],
    per_view_errors: List[float],
    best_method_name: str,
    best_base_to_camera: np.ndarray,
    all_method_results: Dict[str, Any],
    intrinsic_meta: Dict[str, Any],
    reprojection_filter_info: Dict[str, Any],
) -> Path:
    """把标定结果和诊断信息规范保存到 JSON。"""
    output_dir.mkdir(parents=True, exist_ok=True)

    camera_to_base = invert_transform(best_base_to_camera)
    methods_payload = {}
    for method_name, method_result in all_method_results.items():
        methods_payload[method_name] = {
            "score": method_result["score"],
            "base_T_camera": method_result["base_T_camera"].tolist(),
            "camera_T_base": invert_transform(method_result["base_T_camera"]).tolist(),
            "translation_mean_m": method_result["translation_mean_m"],
            "translation_std_m": method_result["translation_std_m"],
            "rotation_consistency_deg_mean": method_result["rotation_consistency_deg_mean"],
            "rotation_consistency_deg_max": method_result["rotation_consistency_deg_max"],
            "gripper_T_target_mean": method_result["gripper_T_target_mean"].tolist(),
            "flip_suspect_samples": method_result.get("flip_suspect_samples", []),
            "worst_rotation_samples": method_result.get("worst_rotation_samples", []),
        }

    result = {
        "mode": "eye_to_hand",
        "camera_mount": "static",
        "target_mount": "rigidly_attached_to_gripper",
        "image_size_wh": [int(image_size[0]), int(image_size[1])],
        "board": {
            "type": "chessboard",
            "dimension_mode": BOARD_DIMENSION_MODE,
            "grid_shape": [int(BOARD_GRID_SHAPE[0]), int(BOARD_GRID_SHAPE[1])],
            "inner_corners": [int(CHESSBOARD_SIZE[0]), int(CHESSBOARD_SIZE[1])],
            "square_size_m": float(SQUARE_SIZE_M),
        },
        "robot_pose_convention": {
            "frame": "base_to_tcp",
            "translation_unit": "meter",
            "rotation_unit": "radian",
            "euler_order": "RzRyRx",
            "rotation_matrix_formula": "R = Rz(rz) @ Ry(ry) @ Rx(rx)",
            "legacy_pose_list_order": LEGACY_POSE_LIST_ORDER,
        },
        "camera_intrinsic": {
            "source": INTRINSIC_SOURCE,
            "stream_name": PRESET_INTRINSIC_STREAM_NAME if INTRINSIC_SOURCE == "preset" else "estimated_from_images",
            "rms_reprojection_error": None if rms is None else float(rms),
            "camera_matrix": camera_matrix.tolist(),
            "dist_coeffs": dist_coeffs.reshape(-1).tolist(),
            "meta": intrinsic_meta,
        },
        "hand_eye": {
            "selected_method": best_method_name,
            "base_T_camera": best_base_to_camera.tolist(),
            "camera_T_base": camera_to_base.tolist(),
            "selected_method_diagnostics": methods_payload[best_method_name],
            "all_methods": methods_payload,
        },
        "samples": {
            "detected_valid_count_before_filter": int(detected_valid_sample_count),
            "used_count": len(valid_samples),
            "used_sample_ids": [sample["sample_id"] for sample in valid_samples],
            "per_view_reprojection_error_px": {
                sample["sample_id"]: error
                for sample, error in zip(valid_samples, per_view_errors)
            },
            "auto_reprojection_filter": reprojection_filter_info,
        },
        "usage_note": {
            "base_T_camera_definition": "该矩阵把相机坐标系中的点变换到机械臂 Base 坐标系：p_base = base_T_camera @ p_camera",
            "calibrateHandEye_input_note": "眼在手外时，求解器使用的是 gripper_T_base（由 base_T_tcp 求逆得到）与 camera_T_target（即 PnP 求得的 target -> camera 变换）。",
        },
    }

    output_path = output_dir / "eye_to_hand_result.json"
    with output_path.open("w", encoding="utf-8") as file:
        json.dump(result, file, indent=2, ensure_ascii=False)
    return output_path


def main_v2() -> int:
    try:
        samples = load_samples(DATA_DIR)
    except Exception as exc:
        print(f"[ERROR] 读取采集数据失败：{exc}")
        return 1

    print(f"[INFO] 共读取到 {len(samples)} 组采集样本。")

    try:
        valid_samples, image_size = collect_observations(samples)
    except Exception as exc:
        print(f"[ERROR] 提取棋盘格角点失败：{exc}")
        return 1

    print(f"[INFO] 有效棋盘格样本数：{len(valid_samples)}")
    if len(valid_samples) < MIN_VALID_SAMPLES:
        print(
            f"[ERROR] 有效样本数不足，当前只有 {len(valid_samples)} 组，"
            f"建议至少 {MIN_VALID_SAMPLES} 组。"
        )
        return 1

    detected_valid_sample_count = len(valid_samples)

    try:
        if INTRINSIC_SOURCE == "preset":
            camera_matrix, dist_coeffs, intrinsic_meta = load_preset_intrinsics(
                PRESET_INTRINSICS_PATH
            )
            validate_preset_intrinsics_for_images(image_size, intrinsic_meta)
            valid_samples, rvecs, tvecs, per_view_errors = solve_target_poses_with_known_intrinsics(
                valid_samples=valid_samples,
                camera_matrix=camera_matrix,
                dist_coeffs=dist_coeffs,
            )
            rms = None
        elif INTRINSIC_SOURCE == "estimate":
            intrinsic_meta = {
                "source": "estimate",
                "path": None,
                "stream_name": "estimated_from_images",
            }
            rms, camera_matrix, dist_coeffs, rvecs, tvecs, per_view_errors = calibrate_camera_intrinsics(
                valid_samples=valid_samples,
                image_size=image_size,
            )
        else:
            raise ValueError("INTRINSIC_SOURCE 只允许 'preset' 或 'estimate'")

        (
            valid_samples,
            rvecs,
            tvecs,
            per_view_errors,
            reprojection_filter_info,
        ) = filter_reprojection_outliers(
            valid_samples=valid_samples,
            rvecs=rvecs,
            tvecs=tvecs,
            per_view_errors=per_view_errors,
        )

        if reprojection_filter_info.get("enabled"):
            print(
                "[INFO] PnP 单帧重投影误差自动筛帧阈值："
                f"{reprojection_filter_info['threshold_px']:.3f} px "
                f"(median={reprojection_filter_info['median_px']:.3f}, "
                f"MAD={reprojection_filter_info['mad_px']:.3f})"
            )
            rejected_samples = reprojection_filter_info.get("rejected_samples", [])
            if rejected_samples:
                rejected_summary = ", ".join(
                    f"{item['sample_id']}({item['reprojection_error_px']:.3f}px)"
                    for item in rejected_samples
                )
                print(f"[WARN] 自动剔除高重投影误差样本：{rejected_summary}")

        if (
            INTRINSIC_SOURCE == "estimate"
            and reprojection_filter_info.get("rejected_count", 0) > 0
        ):
            print("[INFO] 已剔除坏帧，正在基于剩余样本重新估计相机内参与每帧外参。")
            rms, camera_matrix, dist_coeffs, rvecs, tvecs, per_view_errors = calibrate_camera_intrinsics(
                valid_samples=valid_samples,
                image_size=image_size,
            )
            annotate_samples_with_reprojection_errors(valid_samples, per_view_errors)
            reprojection_filter_info["recomputed_intrinsics_after_filter"] = True

        best_method_name, best_base_to_camera, all_method_results = solve_eye_to_hand(
            valid_samples=valid_samples,
            rvecs=rvecs,
            tvecs=tvecs,
        )
        best_method_diagnostics = all_method_results[best_method_name]
        flip_suspect_samples = best_method_diagnostics.get("flip_suspect_samples", [])
        if flip_suspect_samples:
            flip_summary = ", ".join(
                f"{item['sample_id']}({item['rotation_error_deg']:.1f}deg)"
                for item in flip_suspect_samples[:ROTATION_OUTLIER_REPORT_TOPK]
            )
            print(
                "[WARN] 检测到棋盘翻转/姿态异常嫌疑样本。"
                "这通常意味着角点顺序不一致、标定板未刚性固定，或个别样本姿态求解异常："
                f"{flip_summary}"
            )

        output_path = save_results(
            output_dir=OUTPUT_DIR,
            image_size=image_size,
            rms=rms,
            camera_matrix=camera_matrix,
            dist_coeffs=dist_coeffs,
            detected_valid_sample_count=detected_valid_sample_count,
            valid_samples=valid_samples,
            per_view_errors=per_view_errors,
            best_method_name=best_method_name,
            best_base_to_camera=best_base_to_camera,
            all_method_results=all_method_results,
            intrinsic_meta=intrinsic_meta,
            reprojection_filter_info=reprojection_filter_info,
        )
    except Exception as exc:
        print(f"[ERROR] 标定失败：{exc}")
        return 1

    print("\n=== 标定完成 ===")
    if rms is None:
        print("[INFO] 相机内参来源：Gemini/厂家预设内参")
    else:
        print(f"[INFO] 相机内参 RMS 重投影误差: {rms:.4f} px")
    print(f"[INFO] 选择的手眼方法: {best_method_name}")
    print(f"[INFO] 结果已保存到: {output_path}")
    print("[INFO] base_T_camera 含义：p_base = base_T_camera @ p_camera")
    return 0


def main() -> int:
    return main_v2()

    try:
        samples = load_samples(DATA_DIR)
    except Exception as exc:
        print(f"[ERROR] 读取采集数据失败：{exc}")
        return 1

    print(f"[INFO] 共读取到 {len(samples)} 组采集样本。")

    try:
        valid_samples, image_size = collect_observations(samples)
    except Exception as exc:
        print(f"[ERROR] 提取棋盘格角点失败：{exc}")
        return 1

    print(f"[INFO] 有效棋盘格样本数：{len(valid_samples)}")
    if len(valid_samples) < MIN_VALID_SAMPLES:
        print(
            f"[ERROR] 有效样本数不足，当前只有 {len(valid_samples)} 组，"
            f"建议至少 {MIN_VALID_SAMPLES} 组。"
        )
        return 1

    try:
        if INTRINSIC_SOURCE == "preset":
            camera_matrix, dist_coeffs, intrinsic_meta = load_preset_intrinsics(PRESET_INTRINSICS_PATH)
            validate_preset_intrinsics_for_images(image_size, intrinsic_meta)
            valid_samples, rvecs, tvecs, per_view_errors = solve_target_poses_with_known_intrinsics(
                valid_samples=valid_samples,
                camera_matrix=camera_matrix,
                dist_coeffs=dist_coeffs,
            )
            rms = None
        elif INTRINSIC_SOURCE == "estimate":
            intrinsic_meta = {
                "source": "estimate",
                "path": None,
                "stream_name": "estimated_from_images",
            }
            rms, camera_matrix, dist_coeffs, rvecs, tvecs, per_view_errors = calibrate_camera_intrinsics(
                valid_samples=valid_samples,
                image_size=image_size,
            )
        else:
            raise ValueError("INTRINSIC_SOURCE 只允许 'preset' 或 'estimate'")

        best_method_name, best_base_to_camera, all_method_results = solve_eye_to_hand(
            valid_samples=valid_samples,
            rvecs=rvecs,
            tvecs=tvecs,
        )
        output_path = save_results(
            output_dir=OUTPUT_DIR,
            image_size=image_size,
            rms=rms,
            camera_matrix=camera_matrix,
            dist_coeffs=dist_coeffs,
            valid_samples=valid_samples,
            per_view_errors=per_view_errors,
            best_method_name=best_method_name,
            best_base_to_camera=best_base_to_camera,
            all_method_results=all_method_results,
            intrinsic_meta=intrinsic_meta,
        )
    except Exception as exc:
        print(f"[ERROR] 标定失败：{exc}")
        return 1

    print("\n=== 标定完成 ===")
    if rms is None:
        print("[INFO] 相机内参来源：Gemini/厂家预设内参")
    else:
        print(f"[INFO] 相机内参 RMS 重投影误差: {rms:.4f} px")
    print(f"[INFO] 选择的手眼方法: {best_method_name}")
    print(f"[INFO] 结果已保存到: {output_path}")
    print("[INFO] base_T_camera 含义：p_base = base_T_camera @ p_camera")
    return 0


if __name__ == "__main__":
    sys.exit(main())
