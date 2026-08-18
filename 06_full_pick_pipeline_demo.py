"""
模块一 + 模块二 + 模块三完整闭环演示。

默认使用 dry-run，不会真实驱动机械臂。
"""

from __future__ import annotations

from dataclasses import asdict
import json

from robot_system.control import LebaiControllerConfig
from robot_system.pipeline import PickExecutor
from robot_system.vision import Detection2D


def build_demo_detections() -> list[Detection2D]:
    return [
        Detection2D(
            object_id="obj_apple_red_01",
            name="apple",
            display_name="红苹果",
            color="red",
            u=660.0,
            v=410.0,
            depth=0.36,
            depth_unit="m",
            confidence=0.98,
            yaw_rad=0.15,
            tags=["fruit", "苹果", "redapple"],
            metadata={"detector": "demo"},
            size_xyz_m=(0.075, 0.075, 0.072),
        ),
        Detection2D(
            object_id="obj_banana_01",
            name="banana",
            display_name="香蕉",
            color="yellow",
            u=530.0,
            v=335.0,
            depth=0.34,
            depth_unit="m",
            confidence=0.93,
            yaw_rad=-0.35,
            tags=["fruit", "香蕉", "banana"],
            metadata={"detector": "demo"},
            size_xyz_m=(0.12, 0.035, 0.03),
        ),
    ]


def main() -> None:
    executor = PickExecutor.from_defaults(
        controller_config=LebaiControllerConfig(
            robot_ip="127.0.0.1",
            dry_run=True,
        )
    )
    result = executor.execute_from_detections(
        user_command="抓取红色的苹果",
        detections=build_demo_detections(),
        scene_context={
            "robot_state": "idle",
            "camera_frame": "color",
            "pipeline_mode": "demo_dry_run",
        },
        execute_motion=True,
    )
    print(json.dumps(asdict(result), ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
