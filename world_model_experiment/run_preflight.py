from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path


EXPERIMENT_ROOT = Path(__file__).resolve().parent
PROJECT_ROOT = EXPERIMENT_ROOT.parent
for path in (str(EXPERIMENT_ROOT), str(PROJECT_ROOT)):
    if path not in sys.path:
        sys.path.insert(0, path)

from world_model import OfflinePreflight  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="世界模型实验真机前离线预检（不连接任何设备）。")
    parser.add_argument(
        "--report",
        type=Path,
        default=EXPERIMENT_ROOT / "reports" / "latest_preflight.json",
        help="JSON报告保存路径。",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    report = OfflinePreflight(PROJECT_ROOT, EXPERIMENT_ROOT).run()
    dated_report = args.report.with_name(f"preflight_{time.strftime('%Y%m%d_%H%M%S')}.json")
    report.save(args.report)
    report.save(dated_report)
    print(json.dumps(report.to_dict(), ensure_ascii=False, indent=2))
    print(f"预检报告: {args.report}")
    print("安全提示: 本程序没有打开相机、连接机械臂或发送运动命令。")
    return 1 if report.failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
