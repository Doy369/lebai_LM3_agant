from __future__ import annotations

import threading
import time
from dataclasses import asdict, dataclass
from typing import Any, Dict, Optional

import cv2


@dataclass
class CameraFeedConfig:
    """
    Web 预览使用的普通 RGB 相机配置。

    设计约束：
    1. 保持 `cv2.VideoCapture(camera_id)` 行为，兼容当前机器上的 RGB 预览。
    2. 每次请求后立即释放设备，避免长期占用 Gemini 的 RGB 流。
    3. 读取失败时允许自动重试一次，便于现场联调。
    """

    camera_id: int = 0
    backend_mode: str = "auto"
    frame_width: int = 1280
    frame_height: int = 720
    jpeg_quality: int = 90
    read_retry_count: int = 2
    warmup_delay_sec: float = 0.12


class CameraFeed:
    """
    基于 OpenCV 的轻量 RGB 预览管理器。

    注意：
    这里不负责真实深度链路，只用于页面 RGB 预览。
    """

    def __init__(self, config: Optional[CameraFeedConfig] = None) -> None:
        self.config = config or CameraFeedConfig()
        self._lock = threading.Lock()
        self._capture: Optional[cv2.VideoCapture] = None
        self._last_error: Optional[str] = None
        self._last_open_info: Dict[str, Any] = {}

    def get_status(self) -> Dict[str, Any]:
        with self._lock:
            ready = self._ensure_open_locked()
            payload = {
                "stream_ready": ready,
                "stream_url": "/api/camera/frame.jpg",
                "stream_type": "rgb",
                "camera_id": self.config.camera_id,
                "backend_mode": self.config.backend_mode,
                "requested_size_wh": [self.config.frame_width, self.config.frame_height],
                "last_error": self._last_error,
                "open_info": dict(self._last_open_info),
            }
            # 状态探测完成后立即释放设备，避免占用 Gemini 的 RGB 通道。
            self._release_locked()
            return payload

    def capture_frame_jpeg(self) -> bytes:
        with self._lock:
            try:
                frame = self._read_frame_locked()
                ok, encoded = cv2.imencode(
                    ".jpg",
                    frame,
                    [int(cv2.IMWRITE_JPEG_QUALITY), int(self.config.jpeg_quality)],
                )
                if not ok:
                    raise RuntimeError("相机帧编码为 JPEG 失败。")
                return encoded.tobytes()
            finally:
                # 预览接口每次请求后都释放设备，避免和 Orbbec SDK 抢占资源。
                self._release_locked()

    def close(self) -> None:
        with self._lock:
            self._release_locked()

    def _read_frame_locked(self) -> Any:
        if not self._ensure_open_locked():
            raise RuntimeError(self._last_error or "无法打开相机。")

        retries = max(int(self.config.read_retry_count), 1)
        last_error = "未知相机读取错误"
        for _ in range(retries):
            assert self._capture is not None
            ok, frame = self._capture.read()
            if ok and frame is not None:
                self._last_error = None
                self._last_open_info["last_frame_ts"] = time.time()
                return frame

            last_error = "相机返回空帧。"
            self._release_locked()
            if self._ensure_open_locked():
                continue
            last_error = self._last_error or last_error

        self._last_error = last_error
        raise RuntimeError(last_error)

    def _ensure_open_locked(self) -> bool:
        if self._capture is not None and self._capture.isOpened():
            return True

        self._release_locked()
        self._last_error = None
        self._last_open_info = {}
        tried_labels: list[str] = []

        for backend in self._backend_candidates():
            tried_labels.append(self._backend_label(backend))
            capture = self._open_capture(backend)
            if capture is None:
                continue

            self._capture = capture
            time.sleep(self.config.warmup_delay_sec)
            self._last_open_info = {
                "camera_id": self.config.camera_id,
                "backend_mode": self.config.backend_mode,
                "backend_value": backend,
                "backend_label": self._backend_label(backend),
                "actual_size_wh": [
                    int(capture.get(cv2.CAP_PROP_FRAME_WIDTH)),
                    int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT)),
                ],
                "opened_at": time.time(),
            }
            return True

        if self._last_error is None:
            self._last_error = f"无法打开目标相机，已尝试 backend: {', '.join(tried_labels)}"
        return False

    def _open_capture(self, backend: Optional[int]) -> Optional[cv2.VideoCapture]:
        try:
            if backend is None:
                capture = cv2.VideoCapture(self.config.camera_id)
            else:
                capture = cv2.VideoCapture(self.config.camera_id, backend)
        except Exception as exc:
            self._last_error = f"cv2.VideoCapture 打开异常: {exc}"
            return None

        if not capture.isOpened():
            capture.release()
            backend_label = self._backend_label(backend)
            self._last_error = f"无法打开相机: camera_id={self.config.camera_id}, backend={backend_label}"
            return None

        capture.set(cv2.CAP_PROP_FRAME_WIDTH, float(self.config.frame_width))
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, float(self.config.frame_height))
        return capture

    def _backend_candidates(self) -> list[Optional[int]]:
        mode = self.config.backend_mode.strip().lower()
        if mode == "auto":
            return [None, cv2.CAP_DSHOW, cv2.CAP_MSMF]
        if mode == "dshow":
            return [cv2.CAP_DSHOW]
        if mode == "msmf":
            return [cv2.CAP_MSMF]
        raise ValueError("camera backend 只允许 'auto' / 'dshow' / 'msmf'")

    def _backend_label(self, backend: Optional[int]) -> str:
        if backend is None:
            return "auto"
        if backend == cv2.CAP_DSHOW:
            return "dshow"
        if backend == cv2.CAP_MSMF:
            return "msmf"
        return str(backend)

    def _release_locked(self) -> None:
        if self._capture is not None:
            try:
                self._capture.release()
            except Exception:
                pass
        self._capture = None

    def to_dict(self) -> Dict[str, Any]:
        return {
            "config": asdict(self.config),
            "last_error": self._last_error,
            "last_open_info": dict(self._last_open_info),
        }
