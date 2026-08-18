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

from robot_system.control import GripperConfig, LebaiController, LebaiControllerConfig  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="机械臂只读状态检查：连接后只读取状态和运动学数据。")
    parser.add_argument("--robot-ip", default="127.0.0.1")
    parser.add_argument("--confirm", help="必须精确填写 READ_ONLY。")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.confirm != "READ_ONLY":
        print("确认文本不正确；没有连接机械臂。请使用 --confirm READ_ONLY。")
        return 2

    controller = LebaiController(
        LebaiControllerConfig(
            robot_ip=args.robot_ip,
            dry_run=False,
            auto_start_system=False,
            stop_system_on_disconnect=False,
            gripper=GripperConfig(init_on_connect=False),
        )
    )
    report = {
        "timestamp": time.time(),
        "robot_ip": args.robot_ip,
        "read_only": True,
        "commands_forbidden": ["start_sys", "movej", "movel", "set_claw", "stop_sys"],
    }
    try:
        controller.connect()
        report["status"] = controller.get_robot_status_summary()
        report["kinematics"] = controller.get_kin_data()
        report["success"] = True
    except Exception as exc:
        report["success"] = False
        report["error"] = str(exc)
    finally:
        controller.disconnect()

    output = EXPERIMENT_ROOT / "reports" / f"robot_readonly_{time.strftime('%Y%m%d_%H%M%S')}.json"
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2))
    print(f"只读报告: {output}")
    print("本程序未调用使能、运动、夹爪或停机接口。")
    return 0 if report.get("success") else 1


if __name__ == "__main__":
    raise SystemExit(main())
