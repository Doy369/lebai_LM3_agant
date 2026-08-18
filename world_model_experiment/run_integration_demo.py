from __future__ import annotations

import json
import sys
from pathlib import Path


EXPERIMENT_ROOT = Path(__file__).resolve().parent
PROJECT_ROOT = EXPERIMENT_ROOT.parent
for path in (str(EXPERIMENT_ROOT), str(PROJECT_ROOT)):
    if path not in sys.path:
        sys.path.insert(0, path)

from robot_system.control import LebaiController, LebaiControllerConfig  # noqa: E402
from robot_system.vision import VisionTransformer  # noqa: E402
from world_model import (  # noqa: E402
    DryRunCandidateGenerator,
    ExistingProjectAdapter,
    RuleWorldModel,
    RuleWorldModelConfig,
    TrialLogger,
    TrialOutcome,
    WorldStateStore,
)


def build_synthetic_camera_plan(calibration_file: Path) -> dict:
    """用真实标定矩阵和合成像素深度生成规划数据，不访问相机。"""
    center_uv = [640, 360]
    depth_m = 0.45
    transformer = VisionTransformer.from_calibration_file(calibration_file)
    point = transformer.pixel_to_base(center_uv[0], center_uv[1], depth_m, depth_unit="m")
    return {
        "success": True,
        "snapshot_id": "synthetic_calibrated_snapshot_001",
        "image_size_wh": [1280, 720],
        "decision": {
            "success": True,
            "selected_object_name": "实验杯",
            "confidence": 0.90,
            "source": "synthetic_offline_input",
        },
        "bbox_xyxy": [560, 260, 720, 520],
        "center_uv": center_uv,
        "depth_m": depth_m,
        "depth_source": "bbox_grid_near_surface",
        "depth_quality": {"valid_ratio": 0.92, "std_m": 0.006},
        "point_base_m": point.to_dict(),
        "workspace_ok": True,
    }


def main() -> None:
    calibration_file = PROJECT_ROOT / "biaoding" / "eye_to_hand_result.json"
    controller = LebaiController(LebaiControllerConfig(robot_ip="dry-run-only", dry_run=True))
    adapter = ExistingProjectAdapter(calibration_file)
    plan_data = build_synthetic_camera_plan(calibration_file)
    state = adapter.build_state_from_camera_plan("抓取实验杯", plan_data, controller)

    candidates = DryRunCandidateGenerator(controller).generate(state)
    config = RuleWorldModelConfig.from_json(EXPERIMENT_ROOT / "config" / "default.json")
    decision = RuleWorldModel(config).decide(state, candidates)

    state_path = WorldStateStore(EXPERIMENT_ROOT / "data" / "latest_integrated_state.json").save(state)
    logger = TrialLogger(EXPERIMENT_ROOT / "trials")
    trial_dir = logger.start_trial("integrated_dry_run")
    logger.record_planning(trial_dir, state, candidates, decision)
    logger.write_json(trial_dir, "source_camera_plan.json", plan_data)
    logger.record_outcome(
        trial_dir,
        TrialOutcome(success=True, notes="真实标定＋合成RGB-D输入＋控制器Dry-run，不是真机抓取结果。"),
    )

    print(json.dumps({"candidate_count": len(candidates), "decision": decision.to_dict()}, ensure_ascii=False, indent=2))
    print(f"集成世界状态: {state_path}")
    print(f"集成试验日志: {trial_dir}")
    print("安全提示: 未打开相机、未连接机械臂、未发送运动命令。")


if __name__ == "__main__":
    main()
