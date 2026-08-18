"""
模块二演示入口。

运行方式：
    lebai-qwen-demo

可选环境变量：
    QWEN_API_KEY / DASHSCOPE_API_KEY
    QWEN_MODEL
    QWEN_BASE_URL
"""

from __future__ import annotations

import json

from robot_system.llm import QwenPickAgent
from robot_system.llm.qwen_agent import build_demo_candidates


def main() -> None:
    agent = QwenPickAgent.from_env()
    decision = agent.plan_pick(
        user_command="抓取红色的苹果",
        candidates=build_demo_candidates(),
        scene_context={
            "robot_state": "idle",
            "camera_name": "Orbbec Gemini 335 RGB Camera",
            "notes": "这是模块二的本地演示，不直接驱动机械臂。",
        },
    )
    print(json.dumps(decision.to_dict(), ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
