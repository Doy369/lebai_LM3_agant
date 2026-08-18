"""
眼在手外（Eye-to-Hand）手眼标定数据采集脚本。

场景约定：
1. 相机固定在机器人工作区外部；
2. 标定板刚性安装在夹爪/工具坐标系上；
3. 机器人返回的是 TCP 相对 Base 的位姿。

重要提醒：
1. 标定板必须“刚性固定”在夹爪上，不能只是放在夹爪上方或松动贴附。
2. 乐白官方文档中，笛卡尔姿态通常采用 Z-Y-X 欧拉角顺序：
   R = Rz(rz) @ Ry(ry) @ Rx(rx)
   请不要把 (rx, ry, rz) 当成 Rodrigues 旋转向量直接丢给 cv2.Rodrigues。
"""

from __future__ import annotations

import json
import os
import sys
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, Optional, Tuple

import cv2
import numpy as np

try:
    from pygrabber.dshow_graph import FilterGraph  # type: ignore
except ImportError:
    FilterGraph = None

try:
    import lebai_sdk  # type: ignore
except ImportError:
    lebai_sdk = None

try:
    from lebai import LebaiRobot  # type: ignore
except ImportError:
    LebaiRobot = None


# =========================
# 可按现场情况修改的配置
# =========================
ROBOT_IP = "127.0.0.1"
CAMERA_ID = 0
CAMERA_PREFERRED_NAME = "Orbbec Gemini 335 RGB Camera"
SAVE_DIR = Path("calib_data")

# 是否按设备名优先匹配摄像头。你已经确认 CAMERA_ID=0 可用时，建议关闭，
# 避免不同 backend / 枚举方式带来额外变量。
USE_CAMERA_NAME_MATCH = False

# 相机打开后端：
# 1. "auto": 等价于 cv2.VideoCapture(CAMERA_ID)，最接近你手工测试时的行为
# 2. "dshow": 强制 DirectShow
# 3. "msmf": 强制 Media Foundation
CAMERA_BACKEND_MODE = "auto"

# 需要与后续使用的相机内参严格匹配。
# 你当前导出的 Orbbec 文件对应彩色流为 1280x720。
FRAME_WIDTH = 1280
FRAME_HEIGHT = 720

# 棋盘格参数。OpenCV 这里要填“内角点数”，不是格子数。
# 你的板子如果是 9 格 * 12 格，那么内角点应为 8 * 11。
BOARD_DIMENSION_MODE = "squares"  # "squares" 或 "inner_corners"
BOARD_GRID_SHAPE = (9, 12)
if BOARD_DIMENSION_MODE == "squares":
    CHESSBOARD_SIZE = (BOARD_GRID_SHAPE[0] - 1, BOARD_GRID_SHAPE[1] - 1)
else:
    CHESSBOARD_SIZE = BOARD_GRID_SHAPE

# 为了避免保存无法用于标定的样本，默认要求画面里能检测到棋盘格。
REQUIRE_CHESSBOARD_FOR_SAVE = True


def rotation_matrix_from_zyx_euler(rz: float, ry: float, rx: float) -> np.ndarray:
    """根据乐白常见的 Z-Y-X 欧拉角约定构造旋转矩阵。"""
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
    """
    将乐白 TCP 位姿转成 Base -> TCP 的 4x4 齐次矩阵。

    pose 字段约定：
    {
        "x": m, "y": m, "z": m,
        "rz": rad, "ry": rad, "rx": rad
    }
    """
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = rotation_matrix_from_zyx_euler(
        pose["rz"], pose["ry"], pose["rx"]
    )
    transform[:3, 3] = np.array([pose["x"], pose["y"], pose["z"]], dtype=np.float64)
    return transform


def normalize_lebai_pose(raw_pose: Any) -> Dict[str, float]:
    """
    统一把 SDK 返回值规范成带字段名的字典，避免后续误用姿态顺序。

    已兼容两种常见输入：
    1. dict: {"x", "y", "z", "rz", "ry", "rx"} 或大小写变体
    2. list/tuple: [x, y, z, rz, ry, rx]
    """
    if isinstance(raw_pose, dict):
        normalized = {
            "x": raw_pose.get("x", raw_pose.get("X")),
            "y": raw_pose.get("y", raw_pose.get("Y")),
            "z": raw_pose.get("z", raw_pose.get("Z")),
            "rz": raw_pose.get("rz", raw_pose.get("Rz")),
            "ry": raw_pose.get("ry", raw_pose.get("Ry")),
            "rx": raw_pose.get("rx", raw_pose.get("Rx")),
        }
        if any(value is None for value in normalized.values()):
            raise ValueError(f"SDK 返回的位姿字段不完整：{raw_pose}")
        return {key: float(value) for key, value in normalized.items()}

    if isinstance(raw_pose, (list, tuple)) and len(raw_pose) == 6:
        x, y, z, rz, ry, rx = raw_pose
        return {
            "x": float(x),
            "y": float(y),
            "z": float(z),
            "rz": float(rz),
            "ry": float(ry),
            "rx": float(rx),
        }

    raise TypeError(f"不支持的位姿格式：{type(raw_pose)!r}, 内容: {raw_pose}")


def draw_detection_overlay(frame: np.ndarray) -> Tuple[np.ndarray, bool]:
    """尝试检测棋盘格，只用于采集预览与样本质量把关。"""
    preview = frame.copy()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

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
            term = (
                cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
                30,
                0.001,
            )
            corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), term)

    if found and corners is not None:
        cv2.drawChessboardCorners(preview, CHESSBOARD_SIZE, corners, found)
        cv2.putText(
            preview,
            "Chessboard: OK",
            (30, 90),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.9,
            (0, 255, 0),
            2,
        )
    else:
        cv2.putText(
            preview,
            "Chessboard: NOT FOUND",
            (30, 90),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.9,
            (0, 0, 255),
            2,
        )

    return preview, found


class LebaiRobotClient:
    """
    对乐白 SDK 做一个很薄的封装。

    说明：
    1. 当前环境里没有安装 SDK，因此这里只保持与官方常见接口风格兼容；
    2. 如果你的现场 SDK 方法名略有差异，只需要改这里，不影响标定主逻辑。
    """

    def __init__(self, ip: str) -> None:
        self.ip = ip
        self.robot: Optional[Any] = None
        self.backend: Optional[str] = None

    def connect(self) -> None:
        errors = []

        if lebai_sdk is not None:
            try:
                if hasattr(lebai_sdk, "init"):
                    lebai_sdk.init()
                self.robot = lebai_sdk.connect(self.ip, False)
                if hasattr(self.robot, "start_sys"):
                    self.robot.start_sys()
                self.backend = "lebai_sdk"
                return
            except Exception as exc:  # pragma: no cover - 依赖现场 SDK
                errors.append(f"lebai_sdk.connect 失败: {exc}")

        if LebaiRobot is not None:
            try:
                self.robot = LebaiRobot(self.ip)
                if hasattr(self.robot, "start_sys"):
                    self.robot.start_sys()
                self.backend = "lebai"
                return
            except Exception as exc:  # pragma: no cover - 依赖现场 SDK
                errors.append(f"LebaiRobot 连接失败: {exc}")

        joined = " | ".join(errors) if errors else "未找到可用的乐白 Python SDK。"
        raise RuntimeError(joined)

    def get_actual_tcp_pose(self) -> Dict[str, float]:
        if self.robot is None:
            raise RuntimeError("机械臂尚未连接。")

        if hasattr(self.robot, "get_actual_tcp_pose"):
            return normalize_lebai_pose(self.robot.get_actual_tcp_pose())

        if hasattr(self.robot, "get_kin_data"):
            kin_data = self.robot.get_kin_data()
            if isinstance(kin_data, dict) and "actual_tcp_pose" in kin_data:
                return normalize_lebai_pose(kin_data["actual_tcp_pose"])

        raise RuntimeError("当前 SDK 对象没有可用的 TCP 位姿读取接口。")

    def close(self) -> None:
        if self.robot is None:
            return

        for method_name in ("close", "disconnect", "stop_sys"):
            method = getattr(self.robot, method_name, None)
            if callable(method):
                try:
                    method()
                except Exception:
                    pass


def enumerate_windows_cameras() -> Dict[int, str]:
    """
    通过 DirectShow 枚举 Windows 摄像头名称。

    需要安装 pygrabber；如果没有安装，则返回空字典。
    """
    if os.name != "nt" or FilterGraph is None:
        return {}

    try:
        graph = FilterGraph()
        devices = graph.get_input_devices()
        return {index: name for index, name in enumerate(devices)}
    except Exception:
        return {}


def resolve_camera_candidates() -> list[int]:
    """
    生成相机候选索引。

    优先按名称匹配 Orbbec RGB，相机名无法解析时再回退到 CAMERA_ID。
    """
    if not USE_CAMERA_NAME_MATCH:
        return [CAMERA_ID]

    candidates = []
    devices = enumerate_windows_cameras()
    if devices:
        print("[INFO] 当前系统视频设备：")
        for index, name in devices.items():
            print(f"  - [{index}] {name}")
            if CAMERA_PREFERRED_NAME.lower() in name.lower():
                candidates.append(index)
    elif CAMERA_PREFERRED_NAME:
        print(
            "[WARN] 当前环境无法按设备名枚举摄像头。"
            "如果打开的不是 Gemini，请安装 pygrabber，或者手动把 CAMERA_ID 改成 Gemini 的索引。"
        )

    if CAMERA_ID not in candidates:
        candidates.append(CAMERA_ID)
    return candidates


def resolve_backend_candidates() -> list[Optional[int]]:
    """根据配置决定 VideoCapture 的打开方式。"""
    mode = CAMERA_BACKEND_MODE.strip().lower()
    if mode == "auto":
        return [None]
    if mode == "dshow":
        return [cv2.CAP_DSHOW]
    if mode == "msmf":
        return [cv2.CAP_MSMF]
    raise ValueError("CAMERA_BACKEND_MODE 只允许 'auto' / 'dshow' / 'msmf'")


def open_camera(camera_id: int) -> cv2.VideoCapture:
    """按优先级尝试打开 USB 相机。"""
    tried = []
    for candidate_id in resolve_camera_candidates():
        for backend in resolve_backend_candidates():
            if backend is None:
                cap = cv2.VideoCapture(candidate_id)
                tried.append(f"id={candidate_id}, backend=auto")
            else:
                cap = cv2.VideoCapture(candidate_id, backend)
                tried.append(f"id={candidate_id}, backend={backend}")
            if not cap.isOpened():
                cap.release()
                continue

            cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

            actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            print(
                f"[INFO] 已打开相机 id={candidate_id}, backend={backend if backend is not None else 'auto'}, "
                f"实际分辨率={actual_width}x{actual_height}"
            )
            return cap

    raise RuntimeError(f"无法打开目标相机，尝试过: {', '.join(tried)}")


def write_session_meta(save_dir: Path) -> None:
    """保存本次采集的约定，方便后续标定脚本做一致性检查。"""
    meta = {
        "created_at": datetime.now().isoformat(timespec="seconds"),
        "mode": "eye_to_hand",
        "camera_mount": "static",
        "target_mount": "rigidly_attached_to_gripper",
        "robot_pose_format": {
            "frame": "base_to_tcp",
            "translation_unit": "meter",
            "rotation_unit": "radian",
            "euler_order": "RzRyRx",
            "rotation_matrix_formula": "R = Rz(rz) @ Ry(ry) @ Rx(rx)",
        },
        "chessboard_size": list(CHESSBOARD_SIZE),
        "require_chessboard_for_save": REQUIRE_CHESSBOARD_FOR_SAVE,
    }
    with (save_dir / "session_meta.json").open("w", encoding="utf-8") as file:
        json.dump(meta, file, indent=2, ensure_ascii=False)


def next_sample_index(save_dir: Path) -> int:
    existing = sorted(save_dir.glob("pose_*.json"))
    if not existing:
        return 0
    last_index = max(int(path.stem.split("_")[-1]) for path in existing)
    return last_index + 1


def save_sample(
    save_dir: Path,
    sample_index: int,
    frame: np.ndarray,
    pose: Dict[str, float],
    chessboard_found: bool,
) -> None:
    sample_id = f"{sample_index:03d}"
    image_path = save_dir / f"img_{sample_id}.png"
    pose_path = save_dir / f"pose_{sample_id}.json"

    if not cv2.imwrite(str(image_path), frame):
        raise RuntimeError(f"图像写入失败：{image_path}")

    record = {
        "sample_id": sample_id,
        "timestamp": datetime.now().isoformat(timespec="milliseconds"),
        "image_file": image_path.name,
        "robot_pose": pose,
        "T_base_to_tcp": pose_to_homogeneous(pose).tolist(),
        "chessboard_found_in_preview": chessboard_found,
        "image_width": int(frame.shape[1]),
        "image_height": int(frame.shape[0]),
    }
    with pose_path.open("w", encoding="utf-8") as file:
        json.dump(record, file, indent=2, ensure_ascii=False)


def main() -> int:
    SAVE_DIR.mkdir(parents=True, exist_ok=True)
    write_session_meta(SAVE_DIR)

    robot = LebaiRobotClient(ROBOT_IP)
    try:
        robot.connect()
        print(f"[OK] 机械臂已连接，SDK backend: {robot.backend}")
    except Exception as exc:
        print(f"[ERROR] 机械臂连接失败：{exc}")
        return 1

    try:
        cap = open_camera(CAMERA_ID)
        print(f"[OK] 相机已打开，ID = {CAMERA_ID}")
    except Exception as exc:
        print(f"[ERROR] 相机打开失败：{exc}")
        robot.close()
        return 1

    print("\n--- 采集说明 ---")
    print("1. 这是眼在手外采集：相机固定不动，标定板刚性固定在夹爪上。")
    print("2. 请让标定板覆盖视野中的不同位置、距离和倾角，不要只在一个平面内平移。")
    print("3. 按 S 保存当前图像和机器人位姿；按 Q 退出。")
    print("4. 如果画面提示 'Chessboard: NOT FOUND'，该帧默认不会保存。")

    sample_index = next_sample_index(SAVE_DIR)

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print("[ERROR] 相机读取失败，采集结束。")
                break

            preview, chessboard_found = draw_detection_overlay(frame)
            cv2.putText(
                preview,
                f"Saved: {sample_index}",
                (30, 50),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                (0, 255, 0),
                2,
            )
            cv2.imshow("Eye-to-Hand Calibration Capture", preview)

            key = cv2.waitKey(30) & 0xFF
            if key in (ord("q"), ord("Q")):
                break

            if key not in (ord("s"), ord("S")):
                continue

            if REQUIRE_CHESSBOARD_FOR_SAVE and not chessboard_found:
                print("[WARN] 当前帧未检测到棋盘格，已跳过保存。")
                continue

            try:
                pose = robot.get_actual_tcp_pose()
                save_sample(SAVE_DIR, sample_index, frame, pose, chessboard_found)
                print(f"[OK] 已保存样本 {sample_index:03d}")
                sample_index += 1
            except Exception as exc:
                print(f"[ERROR] 保存样本失败：{exc}")
                continue
    finally:
        cap.release()
        cv2.destroyAllWindows()
        robot.close()

    print("采集结束。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
