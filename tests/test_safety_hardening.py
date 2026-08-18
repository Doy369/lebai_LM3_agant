from __future__ import annotations

import ast
import copy
import json
import tempfile
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np
from fastapi.testclient import TestClient
from pydantic import ValidationError

from robot_system.calibration import AutoCalibrationConfig, AutoCalibrationRunner
from robot_system.control import LebaiController, LebaiControllerConfig
from robot_system.llm.types import Pose3D
from robot_system.vision import OrbbecDepthCamera, VisionTransformer
from robot_system.web import WebBackendConfig, create_app
from robot_system.web.models import AutoCalibrationRunRequest, CameraGraspRequest
from robot_system.web.service import RobotWebService

from tests.test_probe_lock_flow import PLAN_DATA, PROBE_EXECUTION


PROJECT_ROOT = Path(__file__).resolve().parents[1]


class SafeDefaultsTests(unittest.TestCase):
    def test_demo_scripts_do_not_default_to_real_motion(self) -> None:
        for relative_path in (
            Path("apps/demos/controller_dry_run.py"),
            Path("apps/demos/full_pick_pipeline.py"),
            Path("apps/robot/safety_probe.py"),
        ):
            tree = ast.parse((PROJECT_ROOT / relative_path).read_text(encoding="utf-8"))
            dry_run_values = [
                keyword.value.value
                for node in ast.walk(tree)
                if isinstance(node, ast.Call)
                for keyword in node.keywords
                if keyword.arg == "dry_run" and isinstance(keyword.value, ast.Constant)
            ]
            self.assertTrue(dry_run_values, str(relative_path))
            self.assertNotIn(False, dry_run_values, str(relative_path))

        tree = ast.parse(
            (PROJECT_ROOT / "apps" / "robot" / "two_stage_pick.py").read_text(encoding="utf-8")
        )
        assignments = {
            node.targets[0].id: node.value.value
            for node in tree.body
            if isinstance(node, ast.Assign)
            and len(node.targets) == 1
            and isinstance(node.targets[0], ast.Name)
            and isinstance(node.value, ast.Constant)
        }
        self.assertEqual(assignments["TEST_STAGE"], "probe")
        self.assertIs(assignments["DRY_RUN"], True)

    def test_real_motion_requires_explicit_confirmation_and_token(self) -> None:
        with self.assertRaises(RuntimeError):
            WebBackendConfig(dry_run=False, web_token="token")
        with self.assertRaises(RuntimeError):
            WebBackendConfig(dry_run=False, allow_real_motion=True)
        config = WebBackendConfig(
            dry_run=False,
            allow_real_motion=True,
            web_token="token",
        )
        self.assertFalse(config.dry_run)


class WebAuthenticationTests(unittest.TestCase):
    def test_control_route_requires_token(self) -> None:
        config = WebBackendConfig(dry_run=True, web_token="secret-token")
        with TestClient(create_app(config)) as client:
            unauthorized = client.post("/api/robot/stop-motion")
            self.assertEqual(unauthorized.status_code, 401)
            authorized = client.post(
                "/api/robot/stop-motion",
                headers={"X-Lebai-Token": "secret-token"},
            )
            self.assertEqual(authorized.status_code, 200)


class ProbeLockHardeningTests(unittest.TestCase):
    def build_service(self, ttl_sec: float = 30.0) -> RobotWebService:
        self.tempdir = tempfile.TemporaryDirectory()
        return RobotWebService(
            WebBackendConfig(
                dry_run=True,
                probe_lock_ttl_sec=ttl_sec,
                runtime_state_file=Path(self.tempdir.name) / "state.json",
            )
        )

    def tearDown(self) -> None:
        tempdir = getattr(self, "tempdir", None)
        if tempdir is not None:
            tempdir.cleanup()

    def test_camera_pick_without_probe_lock_is_rejected_before_planning(self) -> None:
        service = self.build_service()
        service._plan_from_camera_internal = Mock(side_effect=AssertionError("must not replan"))
        with self.assertRaises(PermissionError):
            service.grasp_from_camera(CameraGraspRequest(user_command="抓取目标", mode="pick"))
        service._plan_from_camera_internal.assert_not_called()

    def test_probe_lock_expires(self) -> None:
        service = self.build_service(ttl_sec=0.01)
        lock = service._store_probe_lock(copy.deepcopy(PLAN_DATA), copy.deepcopy(PROBE_EXECUTION))
        lock.created_at = time.time() - 1.0
        with self.assertRaises(PermissionError):
            service._consume_probe_lock(lock.probe_lock_id)
        self.assertIsNone(service._latest_probe_lock)

    def test_probe_lock_is_consumed_atomically_once(self) -> None:
        service = self.build_service()
        lock = service._store_probe_lock(copy.deepcopy(PLAN_DATA), copy.deepcopy(PROBE_EXECUTION))
        consumed = service._consume_probe_lock(lock.probe_lock_id)
        self.assertEqual(consumed.probe_lock_id, lock.probe_lock_id)
        with self.assertRaises(PermissionError):
            service._consume_probe_lock(lock.probe_lock_id)

    def test_probe_lock_is_invalidated_when_calibration_changes(self) -> None:
        service = self.build_service()
        calibration_copy = Path(self.tempdir.name) / "calibration.json"
        calibration_copy.write_text(
            (PROJECT_ROOT / "biaoding" / "eye_to_hand_result.json").read_text(encoding="utf-8"),
            encoding="utf-8",
        )
        service.config.calibration_file = calibration_copy
        lock = service._store_probe_lock(copy.deepcopy(PLAN_DATA), copy.deepcopy(PROBE_EXECUTION))
        payload = json.loads(calibration_copy.read_text(encoding="utf-8"))
        payload["test_revision"] = 2
        calibration_copy.write_text(json.dumps(payload), encoding="utf-8")
        with self.assertRaises(PermissionError):
            service._consume_probe_lock(lock.probe_lock_id)

    def test_motion_lock_rejects_parallel_ordinary_motion(self) -> None:
        service = self.build_service()
        self.assertTrue(service._motion_lock.acquire(blocking=False))
        try:
            with self.assertRaises(PermissionError):
                service.open_gripper()
        finally:
            service._motion_lock.release()


class ValidationAndTimeoutTests(unittest.TestCase):
    def test_auto_calibration_minimum_cannot_exceed_sample_count(self) -> None:
        with self.assertRaises(ValidationError):
            AutoCalibrationRunRequest(sample_count=8, min_success_samples=12)

    def test_depth_scale_is_always_converted_from_mm_to_m(self) -> None:
        raw = np.asarray([[6000]], dtype=np.uint16)
        depth_m = OrbbecDepthCamera._depth_to_meters(raw, 0.1)
        self.assertAlmostEqual(float(depth_m[0, 0]), 0.6, places=6)

    def test_motion_timeout_stops_robot(self) -> None:
        fake_robot = SimpleNamespace(
            is_connected=lambda: True,
            get_motion_state=lambda motion_id: "RUNNING",
        )
        fake_robot.stop_move = Mock()
        controller = LebaiController(
            LebaiControllerConfig(
                robot_ip="127.0.0.1",
                dry_run=False,
                wait_motion_timeout_sec=0.1,
            )
        )
        controller.robot = fake_robot
        with self.assertRaises(TimeoutError):
            controller.wait_motion(42)
        fake_robot.stop_move.assert_called_once_with()

    def test_calibration_rejects_wrong_image_size(self) -> None:
        transformer = VisionTransformer.from_calibration_file(
            PROJECT_ROOT / "biaoding" / "eye_to_hand_result.json"
        )
        transformer.assert_image_size((1280, 720))
        with self.assertRaises(ValueError):
            transformer.assert_image_size((640, 480))

    def test_distorted_projection_round_trip(self) -> None:
        transformer = VisionTransformer.from_calibration_file(
            PROJECT_ROOT / "biaoding" / "eye_to_hand_result.json"
        )
        camera_point = Pose3D(0.18, 0.08, 0.65)
        u, v, depth_m = transformer.camera_to_pixel(camera_point)
        recovered = transformer.pixel_to_camera(u, v, depth_m)
        self.assertAlmostEqual(recovered.x, camera_point.x, places=6)
        self.assertAlmostEqual(recovered.y, camera_point.y, places=6)
        self.assertAlmostEqual(recovered.z, camera_point.z, places=6)

    def test_failed_auto_calibration_preserves_existing_result(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            output_dir = Path(temp_dir) / "output"
            output_dir.mkdir()
            existing = output_dir / "eye_to_hand_result.json"
            existing.write_text("old calibration", encoding="utf-8")
            runner = AutoCalibrationRunner(
                controller=Mock(),
                camera=Mock(),
                config=AutoCalibrationConfig(output_dir=output_dir),
            )
            with patch(
                "robot_system.calibration.auto_calibrator.calibration_solver.main_v2",
                return_value=1,
            ):
                with self.assertRaises(RuntimeError):
                    runner._run_calibration(Path(temp_dir))
            self.assertEqual(existing.read_text(encoding="utf-8"), "old calibration")


if __name__ == "__main__":
    unittest.main()
