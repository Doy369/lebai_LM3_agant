from __future__ import annotations

import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from robot_system.web import WebBackendConfig, create_app
from robot_system.web.models import CameraPlanRequest
from robot_system.web.service import RobotWebService


class WorldModelWebIntegrationTests(unittest.TestCase):
    def build_service(self, runtime_state_file: Path) -> RobotWebService:
        return RobotWebService(
            WebBackendConfig(
                dry_run=True,
                runtime_state_file=runtime_state_file,
            )
        )

    def test_routes_are_registered(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            app = create_app(
                WebBackendConfig(
                    dry_run=True,
                    runtime_state_file=Path(temp_dir) / "state.json",
                )
            )
        routes = {route.path for route in app.routes}
        self.assertIn("/api/experiment/world-model/status", routes)
        self.assertIn("/api/experiment/world-model/plan-from-camera", routes)

    def test_status_declares_world_model_planning_read_only(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            service = self.build_service(Path(temp_dir) / "state.json")
            response = service.world_model_status()

        self.assertTrue(response["success"])
        safety = response["data"]["runtime_safety"]
        self.assertFalse(safety["world_model_plan_sends_motion"])
        self.assertTrue(safety["probe_lock_required_for_pick"])

    def test_camera_plan_uses_dry_run_kinematics_and_sends_no_motion(self) -> None:
        fake_plan = {
            "success": True,
            "message": "camera plan ready",
            "snapshot_id": "world-model-test-frame",
            "decision": {
                "selected_object_name": "cup",
                "confidence": 0.92,
            },
            "bbox_xyxy": [100, 100, 220, 260],
            "center_uv": [160, 180],
            "depth_m": 0.45,
            "depth_source": "bbox_grid_near_surface",
            "depth_quality": {"valid_ratio": 0.96, "std_m": 0.004},
            "point_base_m": {"x": -0.45, "y": 0.05, "z": 0.10},
            "workspace_ok": True,
        }

        with tempfile.TemporaryDirectory() as temp_dir:
            service = self.build_service(Path(temp_dir) / "state.json")
            with patch.object(
                service,
                "_plan_from_camera_internal",
                return_value=fake_plan,
            ):
                response = service.world_model_plan_from_camera(
                    CameraPlanRequest(user_command="抓取杯子")
                )

        self.assertTrue(response["success"])
        data = response["data"]
        self.assertFalse(data["motion_sent"])
        self.assertEqual(data["kinematics_mode"], "controller_dry_run")
        self.assertGreater(data["candidate_count"], 0)
        self.assertIn("world_model_decision", data)


if __name__ == "__main__":
    unittest.main()
