"""机器人项目核心包。"""

from typing import Any

from .llm import QwenPickAgent
from .pipeline import PickExecutor, PickExecutorConfig
from .vision import CalibrationBundle, Detection2D, VisionTransformer

__all__ = [
    "CalibrationBundle",
    "Detection2D",
    "PickExecutor",
    "PickExecutorConfig",
    "QwenPickAgent",
    "RobotWebService",
    "VisionTransformer",
    "WebBackendConfig",
    "create_app",
]


def __getattr__(name: str) -> Any:
    """Load optional Web dependencies only when a Web symbol is requested."""
    if name in {"RobotWebService", "WebBackendConfig", "create_app"}:
        from .web import RobotWebService, WebBackendConfig, create_app

        values = {
            "RobotWebService": RobotWebService,
            "WebBackendConfig": WebBackendConfig,
            "create_app": create_app,
        }
        return values[name]
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
