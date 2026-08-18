from __future__ import annotations

import math
import ast
import sys
import tempfile
import unittest
from pathlib import Path


EXPERIMENT_ROOT = Path(__file__).resolve().parents[1]
PROJECT_ROOT = EXPERIMENT_ROOT.parent
for import_path in (str(EXPERIMENT_ROOT), str(PROJECT_ROOT)):
    if import_path not in sys.path:
        sys.path.insert(0, import_path)

from robot_system.control import LebaiController, LebaiControllerConfig  # noqa: E402

from world_model import (  # noqa: E402
    CalibrationState,
    DecisionAction,
    DryRunCandidateGenerator,
    ExistingProjectAdapter,
    OfflinePreflight,
    GraspActionCandidate,
    Position3D,
    RobotState,
    RuleWorldModel,
    TaskState,
    TrialLogger,
    TrialOutcome,
    WorldObjectState,
    WorldState,
    WorldStateStore,
)


def build_state(*, semantic: float = 0.95, depth: float = 0.90, calibration_valid: bool = True) -> WorldState:
    return WorldState(
        objects=[
            WorldObjectState(
                track_id="target-1",
                name="cup",
                position_base_m=Position3D(-0.42, 0.05, 0.06),
                semantic_confidence=semantic,
                depth_confidence=depth,
                bbox_xyxy=(10, 20, 80, 120),
                center_uv=(45.0, 70.0),
            )
        ],
        robot=RobotState(
            joints_rad=(0.0, 0.1, 0.2, 0.3, 0.4, 0.5),
            tcp_pose=(-0.3, 0.0, 0.3, 0.0, math.pi, 0.0),
            robot_state="idle",
            can_move=True,
            connected=True,
        ),
        calibration=CalibrationState(
            fingerprint="test-calibration",
            valid=calibration_valid,
            translation_uncertainty_m=0.001,
            rotation_uncertainty_deg=0.2,
            drift_score=0.01,
        ),
        task=TaskState(user_command="抓取杯子", target_track_id="target-1"),
    )


def build_candidate(*, safe: bool = True) -> GraspActionCandidate:
    return GraspActionCandidate(
        candidate_id="safe" if safe else "unsafe",
        target_track_id="target-1",
        grasp_position_base_m=Position3D(-0.42, 0.05, 0.06),
        roll_rad=0.0,
        pitch_rad=math.pi,
        yaw_rad=0.0,
        pre_grasp_offset_m=0.08,
        gripper_opening=0.6,
        ik_reachable=safe,
        workspace_margin_m=0.08 if safe else 0.0,
        joint_margin_rad=0.5 if safe else 0.0,
        singularity_margin_rad=0.4 if safe else 0.0,
        minimum_clearance_m=0.03 if safe else 0.0,
        path_length_m=0.4,
    )


class RuleWorldModelTests(unittest.TestCase):
    def test_safe_candidate_can_execute(self) -> None:
        result = RuleWorldModel().decide(build_state(), [build_candidate(safe=True)])
        self.assertEqual(result.action, DecisionAction.EXECUTE)
        self.assertEqual(result.selected_candidate_id, "safe")
        self.assertIsNotNone(result.prediction)
        self.assertGreater(result.prediction.success_probability, 0.72)  # type: ignore[union-attr]

    def test_unreachable_candidate_is_rejected(self) -> None:
        result = RuleWorldModel().decide(build_state(), [build_candidate(safe=False)])
        self.assertEqual(result.action, DecisionAction.REJECT)
        self.assertGreaterEqual(result.prediction.collision_probability, 0.35)  # type: ignore[union-attr]

    def test_invalid_calibration_is_hard_rejection(self) -> None:
        result = RuleWorldModel().decide(build_state(calibration_valid=False), [build_candidate()])
        self.assertEqual(result.action, DecisionAction.REJECT)
        self.assertIsNone(result.selected_candidate_id)

    def test_low_perception_confidence_requests_reobservation(self) -> None:
        result = RuleWorldModel().decide(
            build_state(semantic=0.10, depth=0.10),
            [build_candidate()],
        )
        self.assertEqual(result.action, DecisionAction.REOBSERVE)


class PersistenceTests(unittest.TestCase):
    def test_state_and_trial_log_are_written(self) -> None:
        state = build_state()
        candidate = build_candidate()
        decision = RuleWorldModel().decide(state, [candidate])
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            store = WorldStateStore(root / "data" / "state.json")
            path = store.save(state)
            self.assertTrue(path.exists())
            self.assertEqual(store.load_dict()["state_id"], state.state_id)

            logger = TrialLogger(root / "trials")
            trial_dir = logger.start_trial("unit_test")
            logger.record_planning(trial_dir, state, [candidate], decision)
            logger.record_outcome(trial_dir, TrialOutcome(success=True))
            self.assertTrue((trial_dir / "world_state_before.json").exists())
            self.assertTrue((trial_dir / "decision.json").exists())
            self.assertTrue((trial_dir / "outcome.json").exists())


class ExistingProjectIntegrationTests(unittest.TestCase):
    def setUp(self) -> None:
        self.calibration_file = PROJECT_ROOT / "biaoding" / "eye_to_hand_result.json"
        self.controller = LebaiController(LebaiControllerConfig(robot_ip="dry-run-only", dry_run=True))

    def test_camera_plan_is_adapted_and_candidates_are_generated(self) -> None:
        adapter = ExistingProjectAdapter(self.calibration_file)
        plan_data = {
            "success": True,
            "snapshot_id": "unit-snapshot",
            "decision": {"selected_object_name": "cup", "confidence": 0.91},
            "bbox_xyxy": [100, 120, 220, 300],
            "center_uv": [160, 210],
            "depth_m": 0.45,
            "depth_source": "bbox_grid_near_surface",
            "depth_quality": {"valid_ratio": 0.95, "std_m": 0.004},
            "point_base_m": {"x": -0.45, "y": 0.05, "z": 0.10},
            "workspace_ok": True,
        }
        state = adapter.build_state_from_camera_plan("抓取杯子", plan_data, self.controller)
        candidates = DryRunCandidateGenerator(self.controller).generate(state)
        decision = RuleWorldModel().decide(state, candidates)

        self.assertEqual(state.objects[0].name, "cup")
        self.assertEqual(len(candidates), 10)
        self.assertTrue(any(candidate.ik_reachable for candidate in candidates))
        self.assertIn(decision.action, {DecisionAction.EXECUTE, DecisionAction.PROBE})

    def test_adapter_rejects_non_dry_run_controller(self) -> None:
        adapter = ExistingProjectAdapter(self.calibration_file)
        controller = LebaiController(LebaiControllerConfig(robot_ip="not-connected", dry_run=False))
        with self.assertRaises(PermissionError):
            adapter.robot_state(controller)

    def test_failed_camera_plan_cannot_become_world_object(self) -> None:
        adapter = ExistingProjectAdapter(self.calibration_file)
        with self.assertRaises(ValueError):
            adapter.object_from_camera_plan({"success": False})


class RealMachinePreparationTests(unittest.TestCase):
    def test_offline_preflight_never_authorizes_motion(self) -> None:
        report = OfflinePreflight(PROJECT_ROOT, EXPERIMENT_ROOT).run()
        self.assertFalse(report.failed)
        self.assertFalse(report.ready_for_real_motion)
        self.assertTrue(any(item.name == "calibration" for item in report.checks))

    def test_camera_only_script_hardcodes_dry_run(self) -> None:
        source = (EXPERIMENT_ROOT / "capture_camera_only.py").read_text(encoding="utf-8")
        tree = ast.parse(source)
        dry_run_values = [
            keyword.value.value
            for node in ast.walk(tree)
            if isinstance(node, ast.Call)
            for keyword in node.keywords
            if keyword.arg == "dry_run" and isinstance(keyword.value, ast.Constant)
        ]
        self.assertTrue(dry_run_values)
        self.assertNotIn(False, dry_run_values)
        self.assertNotIn("safety_probe(", source)
        self.assertNotIn("pick_and_place(", source)

    def test_robot_readonly_script_calls_only_read_methods(self) -> None:
        source = (EXPERIMENT_ROOT / "robot_readonly_check.py").read_text(encoding="utf-8")
        tree = ast.parse(source)
        controller_calls = {
            node.func.attr
            for node in ast.walk(tree)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
            and isinstance(node.func.value, ast.Name)
            and node.func.value.id == "controller"
        }
        self.assertEqual(
            controller_calls,
            {"connect", "get_robot_status_summary", "get_kin_data", "disconnect"},
        )


if __name__ == "__main__":
    unittest.main()
