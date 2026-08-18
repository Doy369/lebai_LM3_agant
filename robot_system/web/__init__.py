"""模块四：Web GUI 后端接口。"""

from .app import create_app
from .service import RobotWebService, WebBackendConfig

__all__ = [
    "create_app",
    "RobotWebService",
    "WebBackendConfig",
]
