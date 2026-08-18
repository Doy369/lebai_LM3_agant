from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path

from robot_system.calibration import AutoCalibrationConfig, AutoCalibrationRunner
from robot_system.control import LebaiController
from robot_system.vision import OrbbecDepthCamera


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run automatic Eye-to-Hand calibration.")
    parser.add_argument("--robot-ip", default=os.getenv("LEBAI_ROBOT_IP", "127.0.0.1"))
    parser.add_argument("--samples", type=int, default=20)
    parser.add_argument("--min-success", type=int, default=12)
    parser.add_argument("--settle-sec", type=float, default=0.8)
    parser.add_argument("--data-root", default="calib_data")
    parser.add_argument("--output-dir", default="biaoding")
    parser.add_argument("--session-name", default=None)
    parser.add_argument("--dry-run", action="store_true", help="Do not connect real robot motion.")
    parser.add_argument("--no-motion", action="store_true", help="Validate/capture without moving through the pose plan.")
    parser.add_argument("--no-calibrate", action="store_true", help="Only collect images and poses.")
    parser.add_argument("--auto-start", action="store_true", help="Call start_sys before moving.")
    parser.add_argument(
        "--allow-real-motion",
        action="store_true",
        help="Required confirmation when LEBAI_DRY_RUN=0.",
    )
    parser.add_argument("--plan-only", action="store_true", help="Print the planned perturbations and exit.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    os.environ["LEBAI_ROBOT_IP"] = str(args.robot_ip)
    if args.dry_run:
        os.environ["LEBAI_DRY_RUN"] = "1"

    controller = LebaiController.from_env()
    motion_requested = not args.no_motion and not args.plan_only
    if not controller.config.dry_run and motion_requested and not args.allow_real_motion:
        print(
            "[ERROR] 真机自动标定必须额外提供 --allow-real-motion；未确认时拒绝运动。",
            file=sys.stderr,
        )
        return 2
    camera = OrbbecDepthCamera()
    config = AutoCalibrationConfig(
        data_root=Path(args.data_root),
        output_dir=Path(args.output_dir),
        session_name=args.session_name,
        sample_count=int(args.samples),
        min_success_samples=int(args.min_success),
        settle_sec=float(args.settle_sec),
        execute_motion=not bool(args.no_motion),
        auto_start_system=bool(args.auto_start),
        run_calibration=not bool(args.no_calibrate),
    )
    runner = AutoCalibrationRunner(controller=controller, camera=camera, config=config)

    if args.plan_only:
        print(json.dumps(runner.preview_plan(), ensure_ascii=False, indent=2))
        return 0

    try:
        result = runner.run()
    except Exception as exc:
        print(f"[ERROR] 自动标定失败: {exc}", file=sys.stderr)
        return 1

    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result.get("success") else 1


if __name__ == "__main__":
    sys.exit(main())
