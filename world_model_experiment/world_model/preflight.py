from __future__ import annotations

import importlib.util
import json
import os
import shutil
import tempfile
import time
from dataclasses import asdict, dataclass, field
from enum import Enum
from pathlib import Path
from typing import Any, Dict, List

from robot_system.vision import CalibrationBundle
from robot_system.web import WebBackendConfig

from .rule_model import RuleWorldModelConfig


class CheckLevel(str, Enum):
    PASS = "pass"
    WARN = "warn"
    FAIL = "fail"


@dataclass
class CheckResult:
    name: str
    level: CheckLevel
    message: str
    data: Dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> Dict[str, Any]:
        payload = asdict(self)
        payload["level"] = self.level.value
        return payload


@dataclass
class PreflightReport:
    generated_at: float
    checks: List[CheckResult]
    ready_for_camera_capture: bool
    ready_for_robot_readonly: bool
    ready_for_real_motion: bool = False

    @property
    def failed(self) -> bool:
        return any(item.level == CheckLevel.FAIL for item in self.checks)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "generated_at": self.generated_at,
            "failed": self.failed,
            "ready_for_camera_capture": self.ready_for_camera_capture,
            "ready_for_robot_readonly": self.ready_for_robot_readonly,
            "ready_for_real_motion": False,
            "checks": [item.to_dict() for item in self.checks],
            "note": "离线预检永远不会授权真实运动；真机运动仍需现场逐级确认。",
        }

    def save(self, path: str | Path) -> Path:
        output = Path(path)
        output.parent.mkdir(parents=True, exist_ok=True)
        with output.open("w", encoding="utf-8") as file:
            json.dump(self.to_dict(), file, ensure_ascii=False, indent=2)
        return output


class OfflinePreflight:
    """不连接相机和机械臂的上机前静态检查。"""

    def __init__(
        self,
        project_root: str | Path,
        experiment_root: str | Path,
        minimum_free_disk_gb: float = 2.0,
    ) -> None:
        self.project_root = Path(project_root)
        self.experiment_root = Path(experiment_root)
        self.minimum_free_disk_gb = float(minimum_free_disk_gb)

    def run(self) -> PreflightReport:
        checks: List[CheckResult] = []
        checks.append(self._check_calibration())
        checks.append(self._check_rule_config())
        checks.append(self._check_workspace_config())
        checks.append(self._check_output_directory())
        checks.append(self._check_disk())

        core_available = self._module_available("numpy") and self._module_available("cv2")
        checks.append(
            CheckResult(
                "core_dependencies",
                CheckLevel.PASS if core_available else CheckLevel.FAIL,
                "NumPy/OpenCV 可用。" if core_available else "缺少 NumPy 或 OpenCV。",
            )
        )

        camera_sdk = self._module_available("pyorbbecsdk")
        checks.append(
            CheckResult(
                "orbbec_sdk",
                CheckLevel.PASS if camera_sdk else CheckLevel.WARN,
                "pyorbbecsdk 可用。" if camera_sdk else "当前解释器未发现 pyorbbecsdk；明天需改用已安装SDK的虚拟环境。",
            )
        )
        robot_sdk = self._module_available("lebai_sdk")
        checks.append(
            CheckResult(
                "lebai_sdk",
                CheckLevel.PASS if robot_sdk else CheckLevel.WARN,
                "lebai_sdk 可用。" if robot_sdk else "当前解释器未发现 lebai_sdk；无法进行机器人只读或真机测试。",
            )
        )

        qwen_key = bool(os.getenv("QWEN_API_KEY") or os.getenv("DASHSCOPE_API_KEY"))
        checks.append(
            CheckResult(
                "qwen_api_key",
                CheckLevel.PASS if qwen_key else CheckLevel.WARN,
                "Qwen API Key 已配置（未输出密钥内容）。" if qwen_key else "未配置 QWEN_API_KEY/DASHSCOPE_API_KEY。",
            )
        )
        checks.append(self._check_motion_environment())

        hard_failure = any(item.level == CheckLevel.FAIL for item in checks)
        return PreflightReport(
            generated_at=time.time(),
            checks=checks,
            ready_for_camera_capture=not hard_failure and camera_sdk and qwen_key,
            ready_for_robot_readonly=not hard_failure and robot_sdk,
            ready_for_real_motion=False,
        )

    def _check_calibration(self) -> CheckResult:
        path = self.project_root / "biaoding" / "eye_to_hand_result.json"
        try:
            bundle = CalibrationBundle.from_file(path)
            metadata = bundle.metadata
            samples = metadata.get("samples", {})
            return CheckResult(
                "calibration",
                CheckLevel.PASS,
                "标定文件、齐次矩阵和相机内参校验通过。",
                {
                    "path": str(path),
                    "image_size_wh": metadata.get("image_size_wh"),
                    "selected_method": metadata.get("hand_eye", {}).get("selected_method"),
                    "used_sample_count": samples.get("used_count"),
                },
            )
        except Exception as exc:
            return CheckResult("calibration", CheckLevel.FAIL, f"标定检查失败: {exc}", {"path": str(path)})

    def _check_rule_config(self) -> CheckResult:
        path = self.experiment_root / "config" / "default.json"
        try:
            RuleWorldModelConfig.from_json(path)
            return CheckResult("world_model_config", CheckLevel.PASS, "世界模型权重和阈值配置通过。", {"path": str(path)})
        except Exception as exc:
            return CheckResult("world_model_config", CheckLevel.FAIL, f"世界模型配置失败: {exc}", {"path": str(path)})

    @staticmethod
    def _check_workspace_config() -> CheckResult:
        try:
            config = WebBackendConfig(dry_run=True, allow_real_motion=False)
            workspace = config.workspace
            valid = workspace.x_min < workspace.x_max and workspace.y_min < workspace.y_max and workspace.z_min < workspace.z_max
            if not valid:
                raise ValueError("工作空间上下界顺序错误。")
            return CheckResult("workspace", CheckLevel.PASS, "默认工作空间边界有效。", asdict(workspace))
        except Exception as exc:
            return CheckResult("workspace", CheckLevel.FAIL, f"工作空间配置失败: {exc}")

    def _check_output_directory(self) -> CheckResult:
        target = self.experiment_root / "trials"
        try:
            target.mkdir(parents=True, exist_ok=True)
            with tempfile.NamedTemporaryFile(prefix=".write_test_", dir=str(target), delete=True):
                pass
            return CheckResult("trial_output", CheckLevel.PASS, "试验日志目录可写。", {"path": str(target)})
        except Exception as exc:
            return CheckResult("trial_output", CheckLevel.FAIL, f"试验日志目录不可写: {exc}", {"path": str(target)})

    def _check_disk(self) -> CheckResult:
        try:
            usage = shutil.disk_usage(self.experiment_root)
            free_gb = usage.free / (1024**3)
            level = CheckLevel.PASS if free_gb >= self.minimum_free_disk_gb else CheckLevel.WARN
            return CheckResult(
                "free_disk",
                level,
                f"实验盘剩余空间 {free_gb:.2f} GB。",
                {"free_gb": free_gb, "minimum_free_gb": self.minimum_free_disk_gb},
            )
        except Exception as exc:
            return CheckResult("free_disk", CheckLevel.WARN, f"无法读取磁盘空间: {exc}")

    @staticmethod
    def _check_motion_environment() -> CheckResult:
        dry_run = (os.getenv("LEBAI_DRY_RUN") or "").strip()
        allow_motion = (os.getenv("LEBAI_ALLOW_REAL_MOTION") or "").strip().upper()
        if allow_motion == "YES" or dry_run == "0":
            return CheckResult(
                "motion_environment",
                CheckLevel.WARN,
                "当前会话存在真机相关环境变量。进行相机只读采集前应新开终端并设置 LEBAI_DRY_RUN=1。",
                {"LEBAI_DRY_RUN": dry_run or "unset", "LEBAI_ALLOW_REAL_MOTION": "set" if allow_motion else "unset"},
            )
        return CheckResult(
            "motion_environment",
            CheckLevel.PASS,
            "当前会话未启用真实运动。",
            {"LEBAI_DRY_RUN": dry_run or "unset", "LEBAI_ALLOW_REAL_MOTION": "unset"},
        )

    @staticmethod
    def _module_available(name: str) -> bool:
        try:
            return importlib.util.find_spec(name) is not None
        except Exception:
            return False
