from __future__ import annotations

import json
import math
import sys
from pathlib import Path


EXPERIMENT_ROOT = Path(__file__).resolve().parent
PROJECT_ROOT = EXPERIMENT_ROOT.parent
for path in (str(EXPERIMENT_ROOT), str(PROJECT_ROOT)):
    if path not in sys.path:
        sys.path.insert(0, path)

from world_model import (  # noqa: E402
    CalibrationState,
    GraspActionCandidate,
    Position3D,
    RobotState,
    RuleWorldModel,
    RuleWorldModelConfig,
    TaskState,
    TrialLogger,
    TrialOutcome,
    WorldObjectState,
    WorldState,
    WorldStateStore,
)


def build_demo_state() -> WorldState:
    target = WorldObjectState(
        track_id="object_red_cup_001",
        name="cup",
        color="red",
        position_base_m=Position3D(-0.43, 0.08, 0.055),
        semantic_confidence=0.92,
        depth_confidence=0.88,
        bbox_xyxy=(420, 205, 585, 510),
        center_uv=(502.5, 357.5),
        size_xyz_m=Position3D(0.075, 0.075, 0.105),
        position_std_m=Position3D(0.003, 0.003, 0.005),
    )
    return WorldState(
        objects=[target],
        robot=RobotState(
            joints_rad=(0.1, -0.8, 1.1, 0.2, -1.2, 0.0),
            tcp_pose=(-0.35, 0.0, 0.30, 0.0, math.pi, 0.0),
            robot_state="idle",
            can_move=True,
            connected=True,
            gripper_opening=1.0,
        ),
        calibration=CalibrationState(
            fingerprint="demo-horaud-20-samples",
            valid=True,
            translation_uncertainty_m=0.0011,
            rotation_uncertainty_deg=0.331,
            drift_score=0.02,
        ),
        task=TaskState(user_command="抓取红色杯子", target_track_id=target.track_id),
        rgb_snapshot_id="demo_rgb_001",
        depth_snapshot_id="demo_depth_001",
    )


def build_demo_candidates() -> list[GraspActionCandidate]:
    return [
        GraspActionCandidate(
            candidate_id="candidate_center",
            target_track_id="object_red_cup_001",
            grasp_position_base_m=Position3D(-0.43, 0.08, 0.055),
            roll_rad=0.0,
            pitch_rad=math.pi,
            yaw_rad=0.0,
            pre_grasp_offset_m=0.08,
            gripper_opening=0.65,
            ik_reachable=True,
            workspace_margin_m=0.07,
            joint_margin_rad=0.55,
            singularity_margin_rad=0.35,
            minimum_clearance_m=0.025,
            path_length_m=0.42,
        ),
        GraspActionCandidate(
            candidate_id="candidate_edge_risky",
            target_track_id="object_red_cup_001",
            grasp_position_base_m=Position3D(-0.40, 0.08, 0.055),
            roll_rad=0.0,
            pitch_rad=math.pi,
            yaw_rad=math.pi / 2.0,
            pre_grasp_offset_m=0.06,
            gripper_opening=0.65,
            ik_reachable=True,
            workspace_margin_m=0.004,
            joint_margin_rad=0.05,
            singularity_margin_rad=0.07,
            minimum_clearance_m=0.006,
            path_length_m=0.37,
            source="offset_candidate",
        ),
    ]


def main() -> None:
    config = RuleWorldModelConfig.from_json(EXPERIMENT_ROOT / "config" / "default.json")
    model = RuleWorldModel(config)
    state = build_demo_state()
    candidates = build_demo_candidates()
    decision = model.decide(state, candidates)

    state_path = WorldStateStore(EXPERIMENT_ROOT / "data" / "latest_world_state.json").save(state)
    logger = TrialLogger(EXPERIMENT_ROOT / "trials")
    trial_dir = logger.start_trial("offline_demo")
    logger.record_planning(trial_dir, state, candidates, decision)
    logger.record_outcome(
        trial_dir,
        TrialOutcome(success=True, target_selected_correctly=True, human_verified=False, notes="离线演示，不代表真机结果。"),
    )

    print(json.dumps(decision.to_dict(), ensure_ascii=False, indent=2))
    print(f"世界状态已保存: {state_path}")
    print(f"试验日志已保存: {trial_dir}")
    print("安全提示: 本演示未连接机械臂，也未发送任何运动命令。")


if __name__ == "__main__":
    main()
