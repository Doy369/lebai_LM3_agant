from __future__ import annotations

import threading
import time
from dataclasses import asdict, dataclass
from typing import Any, Dict, Iterable, Optional, Tuple

import cv2
import numpy as np

try:
    from pyorbbecsdk import (  # type: ignore
        AlignFilter,
        Config,
        OBAlignMode,
        OBFormat,
        OBFrameAggregateOutputMode,
        OBSensorType,
        OBStreamType,
        Pipeline,
    )
except ImportError:
    AlignFilter = None
    Config = None
    OBAlignMode = None
    OBFormat = None
    OBFrameAggregateOutputMode = None
    OBSensorType = None
    OBStreamType = None
    Pipeline = None


@dataclass
class OrbbecCameraConfig:
    """
    Orbbec 单帧 RGB+Depth 抓取配置。

    这里直接把默认 profile 固定到现场已验证可工作的组合：
    - Color: 1280x720 MJPG 30fps
    - Depth: 848x480 Y16 30fps
    """

    color_width: int = 1280
    color_height: int = 720
    color_fps: int = 30
    depth_width: int = 848
    depth_height: int = 480
    depth_fps: int = 30
    align_mode: str = "filter"
    enable_frame_sync: bool = True
    wait_timeout_ms: int = 1500
    jpeg_quality: int = 90
    depth_neighborhood_window: int = 7


@dataclass
class OrbbecSnapshot:
    """单次 RGB + 对齐深度快照。"""

    snapshot_id: str
    timestamp_ms: int
    color_bgr: np.ndarray
    depth_m: np.ndarray

    @property
    def image_size_wh(self) -> Tuple[int, int]:
        height, width = self.color_bgr.shape[:2]
        return int(width), int(height)

    def get_depth_at_pixel(self, u: int, v: int) -> Optional[float]:
        height, width = self.depth_m.shape[:2]
        if u < 0 or v < 0 or u >= width or v >= height:
            return None
        value = float(self.depth_m[v, u])
        if not np.isfinite(value) or value <= 0.0:
            return None
        return value

    def find_valid_depth_neighborhood(self, u: int, v: int, window_size: int = 7) -> Optional[float]:
        radius = max(int(window_size), 1) // 2
        height, width = self.depth_m.shape[:2]
        x1 = max(u - radius, 0)
        x2 = min(u + radius + 1, width)
        y1 = max(v - radius, 0)
        y2 = min(v + radius + 1, height)
        region = self.depth_m[y1:y2, x1:x2]
        if region.size == 0:
            return None
        valid = region[np.isfinite(region) & (region > 0.0)]
        if valid.size == 0:
            return None
        return float(np.median(valid))

    def sample_depth_in_bbox(
        self,
        bbox_xyxy: Tuple[int, int, int, int],
        grid_size: int = 5,
        inner_margin_ratio: float = 0.18,
        near_percentile: float = 30.0,
    ) -> Optional[Dict[str, float]]:
        """
        在目标框内部多点采样深度。

        这样做比“只取 bbox 中心点”更稳，尤其当中心点刚好落在反光区、阴影区、
        目标边缘，或者框里混入了一部分桌面背景时。
        """
        x1, y1, x2, y2 = [int(v) for v in bbox_xyxy]
        height, width = self.depth_m.shape[:2]
        x1 = max(0, min(x1, width - 1))
        y1 = max(0, min(y1, height - 1))
        x2 = max(0, min(x2, width))
        y2 = max(0, min(y2, height))
        if x2 <= x1 or y2 <= y1:
            return None

        box_w = x2 - x1
        box_h = y2 - y1
        if box_w < 6 or box_h < 6:
            return None

        margin_x = max(2, int(round(box_w * float(inner_margin_ratio))))
        margin_y = max(2, int(round(box_h * float(inner_margin_ratio))))
        sx1 = min(max(x1 + margin_x, x1), x2 - 1)
        sy1 = min(max(y1 + margin_y, y1), y2 - 1)
        sx2 = max(min(x2 - margin_x, x2), sx1 + 1)
        sy2 = max(min(y2 - margin_y, y2), sy1 + 1)

        xs = np.linspace(sx1, sx2 - 1, num=max(int(grid_size), 2))
        ys = np.linspace(sy1, sy2 - 1, num=max(int(grid_size), 2))

        candidates = []
        center_u = (x1 + x2) * 0.5
        center_v = (y1 + y2) * 0.5
        for v in ys:
            for u in xs:
                uu = int(round(float(u)))
                vv = int(round(float(v)))
                depth = self.get_depth_at_pixel(uu, vv)
                if depth is None:
                    continue
                candidates.append(
                    {
                        "u": float(uu),
                        "v": float(vv),
                        "depth_m": float(depth),
                        "center_distance_px": float(np.hypot(uu - center_u, vv - center_v)),
                    }
                )

        if not candidates:
            return None

        depth_values = np.asarray([item["depth_m"] for item in candidates], dtype=np.float64)
        near_threshold = float(np.percentile(depth_values, float(near_percentile)))
        preferred = [item for item in candidates if item["depth_m"] <= near_threshold + 1e-6]
        if not preferred:
            preferred = candidates

        best = min(
            preferred,
            key=lambda item: (item["center_distance_px"], item["depth_m"]),
        )
        return {
            "u": float(best["u"]),
            "v": float(best["v"]),
            "depth_m": float(best["depth_m"]),
            "sample_count": float(len(candidates)),
            "near_threshold_m": near_threshold,
        }

    def jpeg_bytes(self, quality: int = 90) -> bytes:
        ok, encoded = cv2.imencode(".jpg", self.color_bgr, [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)])
        if not ok:
            raise RuntimeError("Orbbec RGB 帧编码为 JPEG 失败。")
        return encoded.tobytes()


class OrbbecDepthCamera:
    """
    基于 Orbbec 官方 Python SDK 的单帧相机适配层。

    目标：
    1. 提供 RGB + 对齐深度快照；
    2. 失败时给出明确、可操作的错误原因；
    3. 使用现场已验证可工作的 profile 组合。
    """

    def __init__(self, config: Optional[OrbbecCameraConfig] = None) -> None:
        self.config = config or OrbbecCameraConfig()
        self._lock = threading.Lock()
        self._pipeline: Optional[Any] = None
        self._align_filter: Optional[Any] = None
        self._last_error: Optional[str] = None
        self._open_info: Dict[str, Any] = {}
        self._snapshot_seq = 0

    @property
    def sdk_available(self) -> bool:
        return Pipeline is not None and Config is not None and OBSensorType is not None and OBFormat is not None

    def get_status(self) -> Dict[str, Any]:
        return {
            "sdk_available": self.sdk_available,
            "stream_ready": self._pipeline is not None,
            "align_mode": self.config.align_mode,
            "requested_color_wh": [self.config.color_width, self.config.color_height],
            "requested_depth_wh": [self.config.depth_width, self.config.depth_height],
            "last_error": self._last_error,
            "open_info": dict(self._open_info),
        }

    def capture_snapshot(self) -> OrbbecSnapshot:
        with self._lock:
            return self._capture_snapshot_locked()

    def capture_preview_jpeg(self) -> bytes:
        snapshot = self.capture_snapshot()
        return snapshot.jpeg_bytes(self.config.jpeg_quality)

    def close(self) -> None:
        with self._lock:
            self._release_locked()

    def _capture_snapshot_locked(self) -> OrbbecSnapshot:
        if not self.sdk_available:
            raise RuntimeError("未安装 pyorbbecsdk，无法读取真实 RGB+Depth。")

        if not self._ensure_open_locked():
            raise RuntimeError(self._last_error or "无法打开 Orbbec 深度相机。")

        assert self._pipeline is not None
        frames = self._pipeline.wait_for_frames(int(self.config.wait_timeout_ms))
        if frames is None:
            raise RuntimeError("等待 Orbbec 帧超时，没有拿到 RGB+Depth 数据。")

        if self._align_filter is not None:
            frames = self._align_filter.process(frames)
            if hasattr(frames, "as_frame_set"):
                frames = frames.as_frame_set()
            if frames is None:
                raise RuntimeError("Orbbec 对齐滤波失败，未拿到有效帧。")

        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()
        if color_frame is None or depth_frame is None:
            raise RuntimeError("当前帧缺少彩色或深度数据，无法继续。")

        color_bgr = self._frame_to_bgr_image(color_frame)
        if color_bgr is None:
            raise RuntimeError("无法把 Orbbec 彩色帧转换成 BGR 图像。")

        depth_w = int(depth_frame.get_width())
        depth_h = int(depth_frame.get_height())
        depth_scale = float(depth_frame.get_depth_scale())
        depth_data = np.frombuffer(depth_frame.get_data(), dtype=np.uint16).reshape((depth_h, depth_w))
        depth_m = self._depth_to_meters(depth_data, depth_scale)

        color_h, color_w = color_bgr.shape[:2]
        if (depth_w, depth_h) != (color_w, color_h):
            raise RuntimeError(
                f"Orbbec 深度帧没有对齐到 RGB。color={color_w}x{color_h}, depth={depth_w}x{depth_h}。"
            )

        self._snapshot_seq += 1
        timestamp_ms = int(time.time() * 1000)
        snapshot_id = f"snapshot_{timestamp_ms}_{self._snapshot_seq:04d}"
        self._open_info["last_snapshot_id"] = snapshot_id
        self._open_info["last_snapshot_ts"] = timestamp_ms
        return OrbbecSnapshot(
            snapshot_id=snapshot_id,
            timestamp_ms=timestamp_ms,
            color_bgr=color_bgr,
            depth_m=depth_m,
        )

    def _ensure_open_locked(self) -> bool:
        if self._pipeline is not None:
            return True

        if not self.sdk_available:
            self._last_error = "未安装 pyorbbecsdk，无法读取真实 RGB+Depth。"
            return False

        try:
            pipeline = Pipeline()
            config = Config()

            color_profile = self._select_stream_profile(
                pipeline=pipeline,
                sensor_type=OBSensorType.COLOR_SENSOR,
                width=self.config.color_width,
                height=self.config.color_height,
                fps=self.config.color_fps,
                preferred_formats=[OBFormat.MJPG, OBFormat.RGB, OBFormat.BGR],
            )
            depth_profile = self._select_stream_profile(
                pipeline=pipeline,
                sensor_type=OBSensorType.DEPTH_SENSOR,
                width=self.config.depth_width,
                height=self.config.depth_height,
                fps=self.config.depth_fps,
                preferred_formats=[OBFormat.Y16],
            )

            config.enable_stream(color_profile)
            config.enable_stream(depth_profile)

            if OBFrameAggregateOutputMode is not None:
                try:
                    config.set_frame_aggregate_output_mode(OBFrameAggregateOutputMode.FULL_FRAME_REQUIRE)
                except Exception:
                    pass

            align_mode = self.config.align_mode.strip().lower()
            if align_mode in {"hw", "sw", "none"} and OBAlignMode is not None:
                if align_mode == "hw":
                    config.set_align_mode(OBAlignMode.HW_MODE)
                elif align_mode == "sw":
                    config.set_align_mode(OBAlignMode.SW_MODE)
                else:
                    config.set_align_mode(OBAlignMode.DISABLE)

            if self.config.enable_frame_sync:
                try:
                    pipeline.enable_frame_sync()
                except Exception:
                    pass

            pipeline.start(config)

            align_filter = None
            if align_mode in {"filter", "auto"} and AlignFilter is not None and OBStreamType is not None:
                align_filter = AlignFilter(align_to_stream=OBStreamType.COLOR_STREAM)

            self._pipeline = pipeline
            self._align_filter = align_filter
            self._last_error = None
            self._open_info = {
                "align_mode": align_mode,
                "config": asdict(self.config),
                "opened_at": time.time(),
                "sdk_available": True,
                "selected_color_profile": self._profile_to_dict(color_profile),
                "selected_depth_profile": self._profile_to_dict(depth_profile),
            }
            return True
        except Exception as exc:
            self._release_locked()
            self._last_error = self._normalize_error_message(exc)
            return False

    def _select_stream_profile(
        self,
        pipeline: Any,
        sensor_type: Any,
        width: int,
        height: int,
        fps: int,
        preferred_formats: Iterable[Any],
    ) -> Any:
        profile_list = pipeline.get_stream_profile_list(sensor_type)
        if profile_list is None:
            raise RuntimeError(f"无法获取传感器 profile 列表: {sensor_type}")

        profiles = list(self._iter_profiles(profile_list))
        if not profiles:
            raise RuntimeError(f"传感器 {sensor_type} 没有可用的 stream profile。")

        exact_matches = [
            profile
            for profile in profiles
            if self._profile_matches(profile, width=width, height=height, fps=fps, preferred_formats=preferred_formats)
        ]
        if exact_matches:
            return exact_matches[0]

        # 如果没有严格匹配，优先选同分辨率、同格式、最接近 fps 的 profile。
        scored = sorted(
            profiles,
            key=lambda profile: self._profile_score(
                profile=profile,
                width=width,
                height=height,
                fps=fps,
                preferred_formats=preferred_formats,
            ),
        )
        if scored:
            return scored[0]

        get_default = getattr(profile_list, "get_default_video_stream_profile", None)
        if callable(get_default):
            profile = get_default()
            if profile is not None:
                return profile

        raise RuntimeError(f"无法为传感器 {sensor_type} 选择有效 profile。")

    @staticmethod
    def _depth_to_meters(depth_data: np.ndarray, depth_scale: float) -> np.ndarray:
        if not np.isfinite(depth_scale) or depth_scale <= 0.0:
            raise RuntimeError(f"Orbbec 返回了非法 depth_scale: {depth_scale}")
        # Orbbec get_depth_scale() 把原始深度值换算为毫米；这里始终再换算为米。
        return depth_data.astype(np.float32) * float(depth_scale) / 1000.0

    def _iter_profiles(self, profile_list: Any) -> Iterable[Any]:
        get_count = getattr(profile_list, "get_count", None)
        get_by_index = getattr(profile_list, "get_stream_profile_by_index", None)
        if not callable(get_count) or not callable(get_by_index):
            return []
        count = int(get_count())
        return [get_by_index(index) for index in range(count)]

    def _profile_matches(
        self,
        profile: Any,
        width: int,
        height: int,
        fps: int,
        preferred_formats: Iterable[Any],
    ) -> bool:
        profile_width = self._safe_profile_attr(profile, "get_width")
        profile_height = self._safe_profile_attr(profile, "get_height")
        profile_fps = self._safe_profile_attr(profile, "get_fps")
        profile_format = self._safe_profile_attr(profile, "get_format")

        if width > 0 and profile_width != width:
            return False
        if height > 0 and profile_height != height:
            return False
        if fps > 0 and profile_fps != fps:
            return False
        if preferred_formats and profile_format not in preferred_formats:
            return False
        return True

    def _profile_score(
        self,
        profile: Any,
        width: int,
        height: int,
        fps: int,
        preferred_formats: Iterable[Any],
    ) -> tuple[int, int, int, int]:
        profile_width = self._safe_profile_attr(profile, "get_width")
        profile_height = self._safe_profile_attr(profile, "get_height")
        profile_fps = self._safe_profile_attr(profile, "get_fps")
        profile_format = self._safe_profile_attr(profile, "get_format")

        format_penalty = 0 if profile_format in set(preferred_formats) else 1
        width_penalty = abs((profile_width or 0) - width) if width > 0 else 0
        height_penalty = abs((profile_height or 0) - height) if height > 0 else 0
        fps_penalty = abs((profile_fps or 0) - fps) if fps > 0 else 0
        return (format_penalty, width_penalty + height_penalty, fps_penalty, -(profile_fps or 0))

    def _safe_profile_attr(self, profile: Any, getter_name: str) -> Optional[Any]:
        getter = getattr(profile, getter_name, None)
        if not callable(getter):
            return None
        try:
            return getter()
        except Exception:
            return None

    def _profile_to_dict(self, profile: Any) -> Dict[str, Any]:
        return {
            "width": self._safe_profile_attr(profile, "get_width"),
            "height": self._safe_profile_attr(profile, "get_height"),
            "fps": self._safe_profile_attr(profile, "get_fps"),
            "format": str(self._safe_profile_attr(profile, "get_format")),
        }

    def _frame_to_bgr_image(self, frame: Any) -> Optional[np.ndarray]:
        width = int(frame.get_width())
        height = int(frame.get_height())
        data = np.frombuffer(frame.get_data(), dtype=np.uint8)
        color_format = frame.get_format()

        if OBFormat is None:
            return None

        if color_format == OBFormat.RGB:
            image = data.reshape((height, width, 3))
            return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        if color_format == OBFormat.BGR:
            return data.reshape((height, width, 3))
        if color_format == OBFormat.MJPG:
            return cv2.imdecode(data, cv2.IMREAD_COLOR)
        if color_format == OBFormat.YUYV:
            image = data.reshape((height, width, 2))
            return cv2.cvtColor(image, cv2.COLOR_YUV2BGR_YUY2)
        if color_format == OBFormat.UYVY:
            image = data.reshape((height, width, 2))
            return cv2.cvtColor(image, cv2.COLOR_YUV2BGR_UYVY)
        if color_format == OBFormat.NV12:
            image = data.reshape((height * 3 // 2, width))
            return cv2.cvtColor(image, cv2.COLOR_YUV2BGR_NV12)
        if color_format == OBFormat.NV21:
            image = data.reshape((height * 3 // 2, width))
            return cv2.cvtColor(image, cv2.COLOR_YUV2BGR_NV21)
        if color_format == OBFormat.I420:
            image = data.reshape((height * 3 // 2, width))
            return cv2.cvtColor(image, cv2.COLOR_YUV2BGR_I420)
        return None

    def _normalize_error_message(self, exc: Exception) -> str:
        message = str(exc)
        lowered = message.lower()
        if "0xc00d3704" in lowered or "mft" in lowered:
            return (
                "打开 Orbbec RGB+Depth 失败：Windows Media Foundation 返回 0xc00d3704，"
                "表示相机流资源不足或被其它进程占用。常见原因是网页 RGB 预览、Orbbec Viewer、"
                "或其它相机程序仍在占用 Gemini。"
            )
        return f"打开 Orbbec 深度相机失败: {message}"

    def _release_locked(self) -> None:
        if self._pipeline is not None:
            try:
                self._pipeline.stop()
            except Exception:
                pass
        self._pipeline = None
        self._align_filter = None
