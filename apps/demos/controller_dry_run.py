"""
模块三演示入口。

默认采用 dry-run，不会真正驱动机械臂。
运行方式：
    lebai-controller-demo
"""

from __future__ import annotations

import json

from robot_system.control import LebaiController, LebaiControllerConfig, PickPlaceRequest
from robot_system.llm.types import GraspPose, Pose3D


def main() -> None:
    controller = LebaiController(
        LebaiControllerConfig(
            robot_ip="127.0.0.1",
            dry_run=True,
        )
    )
    controller.connect()

    pick_pose = GraspPose(
        position=Pose3D(-0.45, 0.12, 0.13),
        roll=0.0,
        pitch=3.141592653589793,
        yaw=0.15,
        approach_vector=Pose3D(0.0, 0.0, -1.0),
        pre_grasp_offset_m=0.08,
    )
    place_pose = GraspPose(
        position=Pose3D(-0.32, -0.18, 0.14),
        roll=0.0,
        pitch=3.141592653589793,
        yaw=0.0,
        approach_vector=Pose3D(0.0, 0.0, -1.0),
        pre_grasp_offset_m=0.08,
    )

    result = controller.pick_and_place(
        PickPlaceRequest(
            pick_pose=pick_pose,
            place_pose=place_pose,
            label="demo_pick_and_place",
        )
    )
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
