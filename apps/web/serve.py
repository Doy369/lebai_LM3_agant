"""
模块四后端启动入口。

运行方式：
    lebai-web

默认配置：
1. 读取环境变量中的机器人 IP、dry_run、工作空间、Home 位等参数；
2. 默认监听 127.0.0.1:8001；
3. 默认使用 `biaoding/eye_to_hand_result.json` 作为标定文件。
"""

from __future__ import annotations

import os

from robot_system.web import create_app


app = create_app()


def main() -> None:
    try:
        import uvicorn
    except ImportError as exc:
        raise SystemExit(
            "未安装 uvicorn。请先安装 Web 依赖，例如：pip install fastapi uvicorn pydantic"
        ) from exc

    host = os.getenv("LEBAI_WEB_HOST", "127.0.0.1")
    port = int(os.getenv("LEBAI_WEB_PORT", "8001"))
    uvicorn.run(app, host=host, port=port, reload=False)


if __name__ == "__main__":
    main()
