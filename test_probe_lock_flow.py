from __future__ import annotations

import copy
import tempfile
import unittest
from pathlib import Path
from unittest.mock import Mock

from fastapi import HTTPException

from robot_system.control import LebaiController, LebaiControllerConfig, PickPlaceRequest
from robot_system.llm.types import GraspPose, Pose3D
from robot_system.web.app import _call_service
from robot_system.web.models import CameraGraspRequest
from robot_system.web.service import RobotWebService, WebBackendConfig


PLAN_DATA = {
    "success": True,
    "message": "已基于当前画面生成抓取规划。",
    "snapshot_id": "snapshot_test_001",
    "latest_plan_image_url": "/api/camera/latest-plan.jpg",
    "image_size_wh": [1280, 720],
    "decision": {"success": True, "selected_object_name": "yellow target"},
    "bbox_xyxy": [522, 247, 629, 462],
    "center_uv": [592, 354],
    "depth_m": 0.6359999775886536,
    "depth_source": "bbox_grid_near_surface",
    "surface_point_base_m": {"x": -0.5451, "y": 0.2320, "z": -0.0666},
    "point_base_m": {"x": -0.5451, "y": 0.2320, "z": 0.1183},
    "grasp_pose": {
        "position": {"x": -0.5451, "y": 0.2320, "z": 0.1183},
        "roll": 0.0,
        "pitch": 3.141592653589793,
        "yaw": 0.0,
        "approach_vector": {"x": 0.0, "y": 0.0, "z": -1.0},
        "pre_grasp_offset_m": 0.08,
    },
    "workspace_ok": True,
    "workspace": {"x_min": -0.75, "x_max": -0.15, "y_min": -0.45, "y_max": 0.45, "z_min": 0.02, "z_max": 0.55},
    "execution": None,
    "retryable": False,
}

PROBE_EXECUTION = {
    "success": True,
    "label": "probe_snapshot_test_001",
    "mode": "safety_probe",
    "command_log": [],
    "target_pose": {
        "position": {"x": -0.5451, "y": 0.2320, "z": 0.1183},
        "roll": 0.0,
        "pitch": 3.141592653589793,
        "yaw": -1.5707963267948966,
        "approach_vector": {"x": 0.0, "y": 0.0, "z": -1.0},
        "pre_grasp_offset_m": 0.08,
    },
    "target_joints": [-0.60, -0.42, 0.93, -2.08, -1.57, -0.60],
}

PICK_EXECUTION = {
    "success": True,
    "label": "pick_snapshot_test_001",
    "pick_pose_source": "probe_lock",
    "command_log": [],
    "pick_pose": copy.deepcopy(PROBE_EXECUTION["target_pose"]),
    "pick_pose_joints": copy.deepcopy(PROBE_EXECUTION["target_joints"]),
}


class FakeMotionController:
    def __init__(self, *, probe_execution=None, pick_execution=None):
        self._probe_execution = copy.deepcopy(probe_execution)
        self._pick_execution = copy.deepcopy(pick_execution)
        self.locked_pick_calls = []
        self.probe_calls = []

    def connect(self) -> None:
        return None

    def disconnect(self) -> None:
        return None

    def assert_robot_can_move(self) -> None:
        return None

    def safety_probe(self, request):
        self.probe_calls.append(request)
        return copy.deepcopy(self._probe_execution)

    def pick_and_place_from_locked_pick(self, request, pick_pose, pick_joints):
        self.locked_pick_calls.append(
            {
                "request": request,
                "pick_pose": pick_pose,
                "pick_joints": [float(value) for value in pick_joints],
            }
        )
        return copy.deepcopy(self._pick_execution)

    def pick_and_place(self, request):
        return copy.deepcopy(self._pick_execution)

    def get_robot_status_summary(self):
        return {
            "dry_run": True,
            "is_connected": True,
            "robot_state": "IDLE",
            "can_move": True,
            "estop_reason": "NONE",
            "requires_manual_enable": False,
        }


class ProbeLockServiceTests(unittest.TestCase):
    def build_service(self) -> RobotWebService:
        self.tempdir = tempfile.TemporaryDirectory()
        config = WebBackendConfig(
            calibration_file=Path("biaoding/eye_to_hand_result.json"),
            runtime_state_file=Path(self.tempdir.name) / "web_runtime_state.json",
            dry_run=True,
        )
        return RobotWebService(config)

    def tearDown(self) -> None:
        tempdir = getattr(self, "tempdir", None)
        if tempdir is not None:
            tempdir.cleanup()

    def test_probe_success_creates_probe_lock(self) -> None:
        service = self.build_service()
        controller = FakeMotionController(probe_execution=PROBE_EXECUTION)
        service._plan_from_camera_internal = Mock(return_value=copy.deepcopy(PLAN_DATA))
        service._build_motion_controller = Mock(return_value=controller)

        response = service.grasp_from_camera(CameraGraspRequest(user_command="抓取香蕉", mode="probe"))

        self.assertTrue(response["success"])
        self.assertEqual(response["data"]["execution_source"], "fresh_snapshot")
        self.assertEqual(response["data"]["lock_snapshot_id"], PLAN_DATA["snapshot_id"])
        self.assertTrue(response["data"]["probe_lock_id"])
        self.assertEqual(service._latest_probe_lock.probe_lock_id, response["data"]["probe_lock_id"])
        self.assertEqual(len(controller.probe_calls), 1)

    def test_pick_with_probe_lock_skips_replanning_and_invalidates_lock(self) -> None:
        service = self.build_service()
        lock = service._store_probe_lock(copy.deepcopy(PLAN_DATA), copy.deepcopy(PROBE_EXECUTION))
        planner = Mock(side_effect=AssertionError("locked pick should not replan"))
        controller = FakeMotionController(pick_execution=PICK_EXECUTION)
        service._plan_from_camera_internal = planner
        service._build_motion_controller = Mock(return_value=controller)

        response = service.grasp_from_camera(
            CameraGraspRequest(user_command="抓取香蕉", mode="pick", probe_lock_id=lock.probe_lock_id)
        )

        planner.assert_not_called()
        self.assertTrue(response["success"])
        self.assertEqual(response["data"]["execution_source"], "probe_lock")
        self.assertEqual(response["data"]["lock_snapshot_id"], PLAN_DATA["snapshot_id"])
        self.assertEqual(controller.locked_pick_calls[0]["pick_joints"], PROBE_EXECUTION["target_joints"])
        self.assertIsNone(service._latest_probe_lock)

    def test_invalid_probe_lock_maps_to_http_409(self) -> None:
        service = self.build_service()
        service._build_motion_controller = Mock(side_effect=AssertionError("invalid lock should fail before robot connect"))

        with self.assertRaises(HTTPException) as context:
            _call_service(
                lambda: service.grasp_from_camera(
                    CameraGraspRequest(user_command="抓取香蕉", mode="pick", probe_lock_id="probe_lock_stale")
                )
            )

        self.assertEqual(context.exception.status_code, 409)


class LockedPickControllerTests(unittest.TestCase):
    def test_locked_pick_reuses_pick_pose_and_only_resolves_place_pose(self) -> None:
        controller = LebaiController(LebaiControllerConfig(robot_ip="127.0.0.1", dry_run=True))
        pick_pose = GraspPose(
            position=Pose3D(-0.45, 0.05, 0.20),
            roll=0.0,
            pitch=3.141592653589793,
            yaw=-1.5707963267948966,
            approach_vector=Pose3D(0.0, 0.0, -1.0),
            pre_grasp_offset_m=0.08,
        )
        place_pose = GraspPose(
            position=Pose3D(-0.30, -0.22, 0.15),
            roll=0.0,
            pitch=3.141592653589793,
            yaw=0.0,
            approach_vector=Pose3D(0.0, 0.0, -1.0),
            pre_grasp_offset_m=0.10,
        )
        place_joints = [0.10, -0.60, 1.00, -1.40, 1.57, 0.20]
        locked_pick_joints = [-0.60, -0.42, 0.93, -2.08, -1.57, -0.60]
        request = PickPlaceRequest(pick_pose=pick_pose, place_pose=place_pose, label="pick_lock_test")

        controller.resolve_grasp_pose = Mock(return_value=(place_pose, place_joints))
        controller.resolve_locked_grasp_pose = Mock(
            side_effect=[
                (pick_pose, locked_pick_joints),
                (pick_pose, locked_pick_joints),
                (place_pose, place_joints),
                (place_pose, place_joints),
            ]
        )
        controller.open_gripper = Mock()
        controller.move_joint = Mock()
        controller.move_linear = Mock()
        controller.close_gripper = Mock()

        result = controller.pick_and_place_from_locked_pick(request, pick_pose=pick_pose, pick_joints=locked_pick_joints)

        controller.resolve_grasp_pose.assert_called_once_with(place_pose, "place_pose")
        self.assertEqual(result["pick_pose_source"], "probe_lock")
        self.assertEqual(result["pick_pose_joints"], locked_pick_joints)
        self.assertEqual(controller.move_linear.call_args_list[0].args[0]["rz"], pick_pose.yaw)


if __name__ == "__main__":
    unittest.main()
