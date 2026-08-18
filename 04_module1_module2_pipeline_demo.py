"""
模块一 + 模块二联调演示。

功能：
1. 读取 `biaoding/eye_to_hand_result.json`
2. 将示例 2D 检测结果转换为 Base 坐标系候选目标
3. 调用模块二 Qwen Agent 输出最终抓取决策

运行方式：
    python 04_module1_module2_pipeline_demo.py
"""

from __future__ import annotations

import json
from pathlib import Path

from robot_system.llm import QwenPickAgent
from robot_system.vision import Detection2D, VisionTransformer


CALIBRATION_FILE = Path("biaoding/eye_to_hand_result.json")


def build_demo_detections() -> list[Detection2D]:
    """
    构造几条示例检测结果。

    说明：
    1. 这里的 u/v/depth 是演示数据；
    2. 接入真实检测器后，只需把同结构的数据替换进来即可。
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
            metadata={"detector": "demo", "bbox_xyxy": [618, 370, 704, 452]},
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
            metadata={"detector": "demo", "bbox_xyxy": [470, 290, 600, 380]},
            size_xyz_m=(0.12, 0.035, 0.03),
        ),
        Detection2D(
            object_id="obj_cup_blue_01",
            name="cup",
            display_name="蓝色杯子",
            color="blue",
            u=760.0,
            v=300.0,
            depth=0.41,
            depth_unit="m",
            confidence=0.88,
            yaw_rad=0.0,
            tags=["cup", "杯子", "container"],
            metadata={"detector": "demo", "bbox_xyxy": [720, 250, 805, 375]},
            size_xyz_m=(0.07, 0.07, 0.10),
        ),
    ]


def main() -> None:
    transformer = VisionTransformer.from_calibration_file(CALIBRATION_FILE)
    detections = build_demo_detections()
    candidates = transformer.build_candidates_from_detections(detections)

    agent = QwenPickAgent.from_env()
    decision = agent.plan_pick(
        user_command="抓取红色的苹果",
        candidates=candidates,
        scene_context={
            "robot_state": "idle",
            "camera_frame": "color",
            "base_frame": "lebai_base",
            "calibration_file": str(CALIBRATION_FILE),
        },
    )

    payload = {
        "detections": [
            {
                "object_id": item.object_id,
                "name": item.name,
                "display_name": item.display_name,
                "color": item.color,
                "pixel_uv": {"u": item.u, "v": item.v},
                "depth_m": item.depth if item.depth_unit == "m" else item.depth / 1000.0,
            }
            for item in detections
        ],
        "candidates_base_frame": [item.to_prompt_dict() for item in candidates],
        "decision": decision.to_dict(),
    }
    print(json.dumps(payload, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
