"""
真机首测建议脚本：安全探测模式。

默认仍使用 dry-run，避免误动作。
确认工位安全后，再把 `dry_run=False`。
"""

from __future__ import annotations

from dataclasses import asdict
import json

from robot_system.control import LebaiControllerConfig, MotionProfile, WorkspaceBox
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
    ]


def main() -> None:
    controller_config = LebaiControllerConfig(
        robot_ip="127.0.0.1",
        dry_run=True,
        motion=MotionProfile(
            joint_acc=0.20,
            joint_vel=0.12,
            linear_acc=0.05,
            linear_vel=0.03,
        ),
        workspace=WorkspaceBox(
            x_min=-0.75,
            x_max=-0.15,
            y_min=-0.45,
            y_max=0.45,
            z_min=0.05,
            z_max=0.45,
        ),
    )

    executor = PickExecutor.from_defaults(controller_config=controller_config)
    result = executor.safety_probe_from_detections(
        user_command="抓取红色的苹果",
        detections=build_demo_detections(),
        scene_context={
            "robot_state": "idle",
            "probe_mode": "real_robot_safety_probe",
        },
        connect_robot=True,
        descend_clearance_m=0.05,
    )
    print(json.dumps(asdict(result), ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
