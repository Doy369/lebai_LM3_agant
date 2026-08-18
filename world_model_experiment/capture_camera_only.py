from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path


EXPERIMENT_ROOT = Path(__file__).resolve().parent
PROJECT_ROOT = EXPERIMENT_ROOT.parent
for path in (str(EXPERIMENT_ROOT), str(PROJECT_ROOT)):
    if path not in sys.path:
        sys.path.insert(0, path)

from robot_system.control import LebaiController, LebaiControllerConfig  # noqa: E402
from robot_system.web import RobotWebService, WebBackendConfig  # noqa: E402
from robot_system.web.models import CameraPlanRequest  # noqa: E402
from world_model import (  # noqa: E402
    DryRunCandidateGenerator,
    ExistingProjectAdapter,
    RuleWorldModel,
    RuleWorldModelConfig,
    TrialLogger,
    WorldStateStore,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="真实RGB-D＋Qwen只读规划采集；永远使用机械臂Dry-run。")
    parser.add_argument("--command", required=True, help="自然语言目标，例如：抓取红色杯子。")
    parser.add_argument(
        "--capture",
        action="store_true",
        help="显式允许打开真实相机。未提供时程序只打印安全说明。",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not args.capture:
        print("未提供 --capture：没有打开相机。")
        print("确认相机安装稳定且没有其它程序占用后，再显式提供 --capture。")
        return 2
    if not (os.getenv("QWEN_API_KEY") or os.getenv("DASHSCOPE_API_KEY")):
        print("未配置 QWEN_API_KEY/DASHSCOPE_API_KEY，停止采集。")
        return 2

    calibration_file = PROJECT_ROOT / "biaoding" / "eye_to_hand_result.json"
    backend_config = WebBackendConfig(
        calibration_file=calibration_file,
        robot_ip="camera-only-no-robot",
        dry_run=True,
        allow_real_motion=False,
        allow_debug_motion=False,
        web_token=None,
        runtime_state_file=EXPERIMENT_ROOT / "data" / "camera_only_runtime.json",
    )
    service = RobotWebService(backend_config)
    logger = TrialLogger(EXPERIMENT_ROOT / "trials")
    trial_dir = logger.start_trial("camera_only")
    try:
        result = service.plan_from_camera(
            CameraPlanRequest(
                user_command=args.command,
                scene_context={"experiment": "structured_world_model", "motion_allowed": False},
            )
        )
        logger.write_json(trial_dir, "raw_camera_plan.json", result)
        if service._latest_snapshot_jpeg is not None:
            (trial_dir / "snapshot.jpg").write_bytes(service._latest_snapshot_jpeg)
        if service._latest_plan_image_jpeg is not None:
            (trial_dir / "annotated_plan.jpg").write_bytes(service._latest_plan_image_jpeg)

        if not result.get("success"):
            print(json.dumps(result, ensure_ascii=False, indent=2))
            print(f"规划失败日志已保存: {trial_dir}")
            return 3

        plan_data = result["data"]
        controller = LebaiController(LebaiControllerConfig(robot_ip="dry-run-only", dry_run=True))
        adapter = ExistingProjectAdapter(calibration_file)
        state = adapter.build_state_from_camera_plan(args.command, plan_data, controller)
        candidates = DryRunCandidateGenerator(controller).generate(state)
        model_config = RuleWorldModelConfig.from_json(EXPERIMENT_ROOT / "config" / "default.json")
        decision = RuleWorldModel(model_config).decide(state, candidates)
        logger.record_planning(trial_dir, state, candidates, decision)
        WorldStateStore(EXPERIMENT_ROOT / "data" / "latest_camera_world_state.json").save(state)

        summary = {
            "success": True,
            "motion_sent": False,
            "candidate_count": len(candidates),
            "world_model_decision": decision.to_dict(),
            "trial_dir": str(trial_dir),
        }
        logger.write_json(trial_dir, "camera_only_summary.json", summary)
        print(json.dumps(summary, ensure_ascii=False, indent=2))
        print("安全提示: 只打开了相机；机械臂控制器为Dry-run，未连接机器人、未发送运动命令。")
        return 0
    finally:
        service.preview_camera.close()
        service.depth_camera.close()


if __name__ == "__main__":
    raise SystemExit(main())
