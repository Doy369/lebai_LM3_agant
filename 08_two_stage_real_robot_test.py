"""
两阶段真机测试脚本。

阶段说明：
1. probe: 只做安全探测，不闭合夹爪，不接触物体
2. pick: 运行完整 pick_and_place

推荐流程：
1. 先保持 TEST_STAGE = "probe"，并使用 dry_run=True 验证
2. 再把 dry_run=False 做真机探测
3. 探测位置确认正确后，切换 TEST_STAGE = "pick"
"""

from __future__ import annotations

from dataclasses import asdict
import json

from robot_system.control import LebaiControllerConfig, MotionProfile, WorkspaceBox
from robot_system.pipeline import PickExecutor
from robot_system.vision import Detection2D


# =========================
# 现场调试时优先改这里
# =========================
TEST_STAGE = "probe"  # 可选: "probe" / "pick"
DRY_RUN = True
ROBOT_IP = "127.0.0.1"

# 首次真机建议保持较慢速度
MOTION_PROFILE = MotionProfile(
    joint_acc=0.20,
    joint_vel=0.12,
    linear_acc=0.05,
    linear_vel=0.03,
)

# 必须按你的真实工位重新确认
WORKSPACE = WorkspaceBox(
    x_min=-0.75,
    x_max=-0.15,
    y_min=-0.45,
    y_max=0.45,
    z_min=0.05,
    z_max=0.45,
)

# probe 阶段时，机械臂会下降到目标上方这么高的位置后停止
PROBE_DESCEND_CLEARANCE_M = 0.05


def build_demo_detections() -> list[Detection2D]:
    """
    演示检测结果。

    现场接入真实视觉后，把这里替换成你的检测输出即可。
    """
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
        robot_ip=ROBOT_IP,
        dry_run=DRY_RUN,
        motion=MOTION_PROFILE,
        workspace=WORKSPACE,
    )
    executor = PickExecutor.from_defaults(controller_config=controller_config)

    scene_context = {
        "robot_state": "idle",
        "test_stage": TEST_STAGE,
        "dry_run": DRY_RUN,
    }

    detections = build_demo_detections()
    if TEST_STAGE == "probe":
        result = executor.safety_probe_from_detections(
            user_command="抓取红色的苹果",
            detections=detections,
            scene_context=scene_context,
            connect_robot=True,
            descend_clearance_m=PROBE_DESCEND_CLEARANCE_M,
        )
    elif TEST_STAGE == "pick":
        result = executor.execute_from_detections(
            user_command="抓取红色的苹果",
            detections=detections,
            scene_context=scene_context,
            execute_motion=True,
        )
    else:
        raise ValueError("TEST_STAGE 只允许 'probe' 或 'pick'")

    print(json.dumps(asdict(result), ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
