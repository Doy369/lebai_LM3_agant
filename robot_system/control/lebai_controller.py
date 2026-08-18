from __future__ import annotations

import math
import os
import time
from dataclasses import asdict, dataclass, field
from typing import Any, Dict, List, Optional, Sequence, Tuple

from robot_system.llm.types import GraspPose, Pose3D

try:
    import lebai_sdk  # type: ignore
except ImportError:
    lebai_sdk = None


class RobotMotionNotReadyError(PermissionError):
    """机器人当前状态不允许运动时抛出的异常。"""


@dataclass
class WorkspaceBox:
    """机械臂工作空间包围盒。"""

    x_min: float = -0.75
    x_max: float = -0.15
    y_min: float = -0.45
    y_max: float = 0.45
    z_min: float = 0.02
    z_max: float = 0.55

    def contains(self, point: Pose3D) -> bool:
        return (
            self.x_min <= point.x <= self.x_max
            and self.y_min <= point.y <= self.y_max
            and self.z_min <= point.z <= self.z_max
        )

    def assert_contains(self, point: Pose3D, label: str) -> None:
        if not self.contains(point):
            raise ValueError(
                f"{label} 超出工作空间限制: "
                f"point=({point.x:.4f}, {point.y:.4f}, {point.z:.4f}), "
                f"workspace={asdict(self)}"
            )


@dataclass
class MotionProfile:
    """统一管理关节运动与直线运动参数。"""

    joint_acc: float = 0.6
    joint_vel: float = 0.35
    linear_acc: float = 0.18
    linear_vel: float = 0.08
    move_time: float = 0.0
    blend_radius: float = 0.0


@dataclass
class GripperConfig:
    """夹爪控制参数。"""

    open_force: int = 30
    open_amplitude: int = 100
    close_force: int = 45
    close_amplitude: int = 0
    init_on_connect: bool = False
    wait_after_action_sec: float = 0.8


@dataclass
class PickPlaceRequest:
    """标准抓取流程请求。"""

    pick_pose: GraspPose
    place_pose: GraspPose
    label: str = "pick_and_place"
    pre_grasp_offset_m: Optional[float] = None
    post_grasp_offset_m: float = 0.10
    pre_place_offset_m: Optional[float] = None
    post_place_offset_m: float = 0.10


@dataclass
class SafetyProbeRequest:
    """真机低风险探测请求。"""

    target_pose: GraspPose
    label: str = "safety_probe"
    hover_offset_m: Optional[float] = None
    descend_clearance_m: float = 0.04
    retreat_offset_m: float = 0.10
    open_gripper_first: bool = True


@dataclass
class LebaiControllerConfig:
    """
    Lebai 控制器配置。

    默认采用“手动使能”模式：
    1. Web 和自动化流程不自动 `start_sys`
    2. 需要先在乐白官方界面手动上使能与清报警
    """

    robot_ip: str
    workspace: WorkspaceBox = field(default_factory=WorkspaceBox)
    motion: MotionProfile = field(default_factory=MotionProfile)
    gripper: GripperConfig = field(default_factory=GripperConfig)
    dry_run: bool = True
    auto_start_system: bool = False
    stop_system_on_disconnect: bool = False
    min_manipulation: float = 0.01
    singularity_joint5_threshold_rad: float = 0.10
    joint_limit_abs_rad: float = 6.10
    joint6_limit_abs_rad: float = 9.50
    wait_motion_timeout_sec: float = 60.0
    home_joint_pose: Optional[Tuple[float, float, float, float, float, float]] = None


class LebaiController:
    """
    Lebai 机械臂安全控制器。

    核心约束：
    1. 运动前先做工作空间校验。
    2. 运动前先做 IK 可达性预检。
    3. 运动前检查当前机器人状态是否允许运动。
    4. Web 状态查询只读，不触发 start_sys/stop_sys。
    """

    def __init__(self, config: LebaiControllerConfig) -> None:
        self.config = config
        self.robot: Optional[Any] = None
        self.command_log: List[Dict[str, Any]] = []

    @classmethod
    def from_env(cls) -> "LebaiController":
        robot_ip = os.getenv("LEBAI_ROBOT_IP", "127.0.0.1")
        dry_run = os.getenv("LEBAI_DRY_RUN", "1") != "0"
        return cls(LebaiControllerConfig(robot_ip=robot_ip, dry_run=dry_run))

    def connect(self) -> None:
        """连接机器人，但默认不自动 start_sys。"""
        if self.config.dry_run:
            self._log("connect", {"robot_ip": self.config.robot_ip, "dry_run": True})
            return

        if self.robot is not None and self._safe_is_connected():
            return

        if lebai_sdk is None:
            raise RuntimeError("未安装 lebai_sdk，无法连接真实机械臂。")

        lebai_sdk.init()
        try:
            self.robot = lebai_sdk.connect(self.config.robot_ip, False)
        except Exception as exc:
            self.robot = None
            raise RuntimeError(
                f"连接机械臂失败: robot_ip={self.config.robot_ip}, error={exc}. "
                "这通常表示机器人端 SDK 服务未就绪、被其他客户端占用、正在重启，"
                "或远端在握手后立即关闭了连接。"
            ) from exc
        self._log("connect", {"robot_ip": self.config.robot_ip, "dry_run": False})

        if self.config.auto_start_system:
            self.start_system()

        if self.config.gripper.init_on_connect:
            self.init_gripper(force=True)

    def disconnect(self) -> None:
        """
        断开机器人连接。

        注意：这里默认不再自动 `stop_sys`，避免 Web 轮询或动作结束时把
        机器人系统反复拉起/关闭，表现成“不断重启”。
        """
        if self.config.dry_run:
            self._log("disconnect", {"dry_run": True})
            return

        if self.robot is None:
            return

        if self.config.stop_system_on_disconnect:
            try:
                self.stop_system()
            except Exception:
                pass

        for method_name in ("disconnect", "close"):
            method = getattr(self.robot, method_name, None)
            if callable(method):
                try:
                    method()
                except Exception:
                    pass
        self.robot = None
        self._log("disconnect", {"dry_run": False})

    def reconnect(self) -> None:
        if self.config.dry_run:
            self._log("reconnect", {"dry_run": True})
            return

        self._log("reconnect", {"robot_ip": self.config.robot_ip})
        try:
            self.disconnect()
        except Exception:
            self.robot = None
        self.connect()

    def start_system(self) -> None:
        self._log("start_system", {})
        self._call_robot_method(("start_sys",))

    def stop_system(self) -> None:
        self._log("stop_system", {})
        self._call_robot_method(("stop_sys",))

    def enter_teach_mode(self) -> None:
        self.assert_robot_can_move()
        self._log("enter_teach_mode", {})
        self._call_robot_method(("teach_mode",))

    def exit_teach_mode(self) -> None:
        self._log("exit_teach_mode", {})
        self._call_robot_method(("end_teach_mode",))

    def emergency_stop(self) -> None:
        self._log("emergency_stop", {})
        for candidates in (("stop_move",), ("estop", "emergency_stop")):
            try:
                self._call_robot_method(candidates)
                return
            except Exception:
                continue
        if not self.config.dry_run:
            raise RuntimeError("未找到可用的急停/停运动接口。")

    def stop_motion(self) -> None:
        self._log("stop_motion", {})
        self._call_robot_method(("stop_move",))

    def get_robot_state(self) -> Optional[str]:
        if self.config.dry_run:
            return "dry_run"
        try:
            state = self._call_robot_method(("get_robot_state",))
        except RuntimeError as exc:
            if not self._is_connection_closed_error(exc):
                raise
            self.reconnect()
            state = self._call_robot_method(("get_robot_state",))
        return None if state is None else str(state)

    def get_estop_reason(self) -> Optional[str]:
        if self.config.dry_run:
            return None
        return self._optional_robot_method(("get_estop_reason",))

    def can_move(self, robot_state: Optional[str] = None) -> bool:
        if self.config.dry_run:
            return True
        state = robot_state or self.get_robot_state()
        if not state:
            return False
        result = self._optional_robot_method(("can_move",), state)
        return bool(result) if result is not None else False

    def get_robot_status_summary(self) -> Dict[str, Any]:
        if self.config.dry_run:
            summary = {
                "dry_run": True,
                "is_connected": True,
                "robot_state": "dry_run",
                "can_move": True,
                "estop_reason": None,
                "requires_manual_enable": False,
            }
            self._log("get_robot_status_summary", {"mock": summary})
            return summary

        robot_state = self.get_robot_state()
        can_move = self.can_move(robot_state)
        estop_reason = self.get_estop_reason()
        summary = {
            "dry_run": False,
            "is_connected": self._safe_is_connected(),
            "robot_state": robot_state,
            "can_move": can_move,
            "estop_reason": estop_reason,
            "requires_manual_enable": not can_move,
        }
        self._log("get_robot_status_summary", summary)
        return summary

    def assert_robot_can_move(self) -> Dict[str, Any]:
        """
        运动前统一检查。

        这里不再帮 Web 自动 start_sys，而是明确要求现场先在官方界面手动使能。
        """
        summary = self.get_robot_status_summary()
        if summary["can_move"]:
            return summary

        message = "机器人当前不允许运动。请先在乐白官方界面手动使能，并确认急停已释放、报警已清除。"
        details: List[str] = []
        if summary.get("robot_state"):
            details.append(f"robot_state={summary['robot_state']}")
        if summary.get("estop_reason"):
            details.append(f"estop_reason={summary['estop_reason']}")
        if details:
            message += " " + "; ".join(details)
        raise RobotMotionNotReadyError(message)

    def get_kin_data(self) -> Dict[str, Any]:
        if self.config.dry_run:
            mock = {
                "actual_joint_pose": [0.0, -1.0, 1.0, 0.0, 1.57, 0.0],
                "actual_tcp_pose": {"x": -0.45, "y": 0.0, "z": 0.25, "rz": 0.0, "ry": 0.0, "rx": 3.14},
            }
            self._log("get_kin_data", {"mock": mock})
            return mock

        try:
            data = self._call_robot_method(("get_kin_data",))
        except RuntimeError as exc:
            if not self._is_connection_closed_error(exc):
                raise
            self._log("get_kin_data_retry_after_reconnect", {"reason": str(exc)})
            self.reconnect()
            data = self._call_robot_method(("get_kin_data",))
        if not isinstance(data, dict):
            raise RuntimeError(f"get_kin_data 返回格式异常: {data}")
        return data

    def get_actual_joint_pose(self) -> List[float]:
        kin_data = self.get_kin_data()
        joints = kin_data.get("actual_joint_pose")
        if joints is None:
            raise RuntimeError("未在运动学反馈中拿到 actual_joint_pose。")
        return [float(value) for value in joints]

    def get_actual_tcp_pose(self) -> Dict[str, float]:
        kin_data = self.get_kin_data()
        pose = kin_data.get("actual_tcp_pose")
        if pose is None:
            raise RuntimeError("未在运动学反馈中拿到 actual_tcp_pose。")
        if isinstance(pose, dict):
            return {
                "x": float(pose["x"]),
                "y": float(pose["y"]),
                "z": float(pose["z"]),
                "rz": float(pose["rz"]),
                "ry": float(pose["ry"]),
                "rx": float(pose["rx"]),
            }
        if isinstance(pose, (list, tuple)) and len(pose) == 6:
            x, y, z, rz, ry, rx = pose
            return {
                "x": float(x),
                "y": float(y),
                "z": float(z),
                "rz": float(rz),
                "ry": float(ry),
                "rx": float(rx),
            }
        raise RuntimeError(f"actual_tcp_pose 返回格式异常: {pose}")

    def capture_home_snapshot(self) -> Dict[str, Any]:
        snapshot = {
            "joint_pose": self.get_actual_joint_pose(),
            "tcp_pose": self.get_actual_tcp_pose(),
        }
        self._log("capture_home_snapshot", snapshot)
        return snapshot

    def init_gripper(self, force: bool = False) -> None:
        self._log("init_gripper", {"force": force})
        self._call_robot_method(("init_claw",), force)

    def open_gripper(self) -> None:
        self.assert_robot_can_move()
        self.init_gripper(force=False)
        payload = {
            "force": self.config.gripper.open_force,
            "amplitude": self.config.gripper.open_amplitude,
        }
        self._log("open_gripper", payload)
        self._call_robot_method(("set_claw",), payload["force"], payload["amplitude"])
        self._sleep(self.config.gripper.wait_after_action_sec)

    def close_gripper(self) -> None:
        self.assert_robot_can_move()
        self.init_gripper(force=False)
        payload = {
            "force": self.config.gripper.close_force,
            "amplitude": self.config.gripper.close_amplitude,
        }
        self._log("close_gripper", payload)
        self._call_robot_method(("set_claw",), payload["force"], payload["amplitude"])
        self._sleep(self.config.gripper.wait_after_action_sec)

    def solve_ik(
        self,
        pose: Dict[str, float],
        reference_joints: Optional[Sequence[float]] = None,
    ) -> List[float]:
        if self.config.dry_run:
            self._log("solve_ik", {"pose": pose, "reference_joints": reference_joints, "mock": True})
            return [0.0, -1.1, 1.25, 0.0, 1.30, pose["rz"]]

        if reference_joints is None:
            reference_joints = self.get_actual_joint_pose()

        result = self._call_robot_method(("kinematics_inverse",), pose, list(reference_joints))
        joints = self._normalize_ik_result(result)
        if not joints:
            raise RuntimeError(f"IK 失败，目标位姿不可达: {pose}")
        joints = self._unwrap_ik_solution(joints, reference_joints)
        return joints

    def move_joint(self, target: Sequence[float] | Dict[str, float]) -> Any:
        self.assert_robot_can_move()
        motion = self.config.motion
        payload = {
            "target": list(target) if not isinstance(target, dict) else target,
            "a": motion.joint_acc,
            "v": motion.joint_vel,
            "t": motion.move_time,
            "r": motion.blend_radius,
        }
        self._log("move_joint", payload)
        motion_id = self._call_robot_method(("movej",), payload["target"], payload["a"], payload["v"], payload["t"], payload["r"])
        self.wait_motion(motion_id)
        return motion_id

    def move_linear(self, target_pose: Dict[str, float]) -> Any:
        self.assert_robot_can_move()
        motion = self.config.motion
        payload = {
            "target_pose": target_pose,
            "a": motion.linear_acc,
            "v": motion.linear_vel,
            "t": motion.move_time,
            "r": 0.0,
        }
        self._log("move_linear", payload)
        motion_id = self._call_robot_method(
            ("movel", "move_p"),
            payload["target_pose"],
            payload["a"],
            payload["v"],
            payload["t"],
            payload["r"],
        )
        self.wait_motion(motion_id)
        return motion_id

    def wait_motion(self, motion_id: Any = 0) -> None:
        if self.config.dry_run:
            self._log("wait_motion", {"motion_id": motion_id, "mock": True})
            return
        timeout_sec = max(float(self.config.wait_motion_timeout_sec), 0.1)
        deadline = time.monotonic() + timeout_sec
        while True:
            state = str(self._call_robot_method(("get_motion_state",), motion_id)).strip().upper()
            self._log("motion_state", {"motion_id": motion_id, "state": state})
            if state == "FINISHED":
                return
            if state not in {"WAIT", "RUNNING"}:
                raise RuntimeError(f"运动 {motion_id} 返回未知状态: {state}")
            if time.monotonic() >= deadline:
                try:
                    self.stop_motion()
                finally:
                    raise TimeoutError(
                        f"运动 {motion_id} 超过 {timeout_sec:.1f}s 未完成，已下发 stop_move。"
                    )
            time.sleep(0.05)

    def move_to_grasp_pose(
        self,
        pose: GraspPose,
        label: str = "target_pose",
        linear: bool = True,
    ) -> Dict[str, Any]:
        resolved_pose, resolved_joints = self.resolve_grasp_pose(pose, label)
        robot_pose = self._pose_to_robot_dict(resolved_pose)
        if linear:
            motion_id = self.move_linear(robot_pose)
        else:
            motion_id = self.move_joint(resolved_joints)
        result = {
            "success": True,
            "label": label,
            "linear": linear,
            "motion_id": motion_id,
            "target_pose": resolved_pose.to_dict(),
            "target_joints": [float(value) for value in resolved_joints],
            "command_log": self.command_log,
        }
        self._log("move_to_grasp_pose_complete", {"label": label, "linear": linear})
        return result

    def reset_to_home(self) -> Dict[str, Any]:
        if self.config.home_joint_pose is None:
            raise ValueError("未配置 home_joint_pose，无法执行 reset_to_home。")

        target_joints = [float(value) for value in self.config.home_joint_pose]
        self._check_joint_limits(target_joints, "home_joint_pose")
        motion_id = self.move_joint(target_joints)
        result = {
            "success": True,
            "label": "reset_to_home",
            "motion_id": motion_id,
            "target_joints": target_joints,
            "command_log": self.command_log,
        }
        self._log("reset_to_home_complete", {"target_joints": target_joints})
        return result

    def jog_tcp(
        self,
        dx: float = 0.0,
        dy: float = 0.0,
        dz: float = 0.0,
        drz: float = 0.0,
        dry: float = 0.0,
        drx: float = 0.0,
        linear: bool = True,
        label: str = "jog_tcp",
    ) -> Dict[str, Any]:
        current_pose = self.get_actual_tcp_pose()
        target_pose = GraspPose(
            position=Pose3D(
                x=float(current_pose["x"]) + float(dx),
                y=float(current_pose["y"]) + float(dy),
                z=float(current_pose["z"]) + float(dz),
            ),
            roll=float(current_pose["rx"]) + float(drx),
            pitch=float(current_pose["ry"]) + float(dry),
            yaw=float(current_pose["rz"]) + float(drz),
            approach_vector=Pose3D(0.0, 0.0, -1.0),
        )
        result = self.move_to_grasp_pose(
            pose=target_pose,
            label=label,
            linear=linear,
        )
        result["delta"] = {
            "dx": dx,
            "dy": dy,
            "dz": dz,
            "drz": drz,
            "dry": dry,
            "drx": drx,
        }
        result["current_pose"] = current_pose
        return result

    def pick_and_place(self, request: PickPlaceRequest) -> Dict[str, Any]:
        pick_pose, pick_joints = self.resolve_grasp_pose(request.pick_pose, "pick_pose")
        return self._pick_and_place_with_resolved_pick(
            request=request,
            pick_pose=pick_pose,
            pick_joints=pick_joints,
            pick_pose_source="resolved",
        )

    def pick_and_place_from_locked_pick(
        self,
        request: PickPlaceRequest,
        pick_pose: GraspPose,
        pick_joints: Sequence[float],
    ) -> Dict[str, Any]:
        locked_pick_joints = [float(value) for value in pick_joints]
        if len(locked_pick_joints) != 6:
            raise ValueError(f"Locked pick joints must contain 6 values, got {locked_pick_joints}")
        self.config.workspace.assert_contains(pick_pose.position, "pick_pose_locked")
        self._check_joint_limits(locked_pick_joints, "pick_pose_locked")
        self._check_singularity_risk(locked_pick_joints, "pick_pose_locked")
        return self._pick_and_place_with_resolved_pick(
            request=request,
            pick_pose=pick_pose,
            pick_joints=locked_pick_joints,
            pick_pose_source="probe_lock",
        )

    def _pick_and_place_with_resolved_pick(
        self,
        request: PickPlaceRequest,
        pick_pose: GraspPose,
        pick_joints: Sequence[float],
        pick_pose_source: str,
    ) -> Dict[str, Any]:
        place_pose, place_joints = self.resolve_grasp_pose(request.place_pose, "place_pose")
        pick_hover = self._build_offset_pose(
            pick_pose,
            request.pre_grasp_offset_m
            if request.pre_grasp_offset_m is not None
            else pick_pose.pre_grasp_offset_m,
            reverse_approach=True,
        )
        pick_retreat = self._build_offset_pose(
            pick_pose,
            request.post_grasp_offset_m,
            reverse_approach=True,
        )
        place_hover = self._build_offset_pose(
            place_pose,
            request.pre_place_offset_m
            if request.pre_place_offset_m is not None
            else place_pose.pre_grasp_offset_m,
            reverse_approach=True,
        )
        place_retreat = self._build_offset_pose(
            place_pose,
            request.post_place_offset_m,
            reverse_approach=True,
        )

        pick_hover_pose, pick_hover_joints = self.resolve_locked_grasp_pose(
            pick_hover,
            "pick_hover",
            reference_joints=pick_joints,
        )
        pick_retreat_pose, pick_retreat_joints = self.resolve_locked_grasp_pose(
            pick_retreat,
            "pick_retreat",
            reference_joints=pick_joints,
        )
        place_hover_pose, place_hover_joints = self.resolve_locked_grasp_pose(
            place_hover,
            "place_hover",
            reference_joints=place_joints,
        )
        place_retreat_pose, place_retreat_joints = self.resolve_locked_grasp_pose(
            place_retreat,
            "place_retreat",
            reference_joints=place_joints,
        )

        self.open_gripper()
        self.move_joint(pick_hover_joints)
        self.move_linear(self._pose_to_robot_dict(pick_pose))
        self.close_gripper()
        self.move_linear(self._pose_to_robot_dict(pick_retreat_pose))
        self.move_joint(place_hover_joints)
        self.move_linear(self._pose_to_robot_dict(place_pose))
        self.open_gripper()
        self.move_linear(self._pose_to_robot_dict(place_retreat_pose))

        result = {
            "success": True,
            "label": request.label,
            "command_log": self.command_log,
            "pick_pose_source": pick_pose_source,
            "pick_pose": pick_pose.to_dict(),
            "place_pose": place_pose.to_dict(),
            "pick_pose_joints": [float(value) for value in pick_joints],
            "pick_hover_pose": pick_hover_pose.to_dict(),
            "pick_hover_joints": [float(value) for value in pick_hover_joints],
            "pick_retreat_pose": pick_retreat_pose.to_dict(),
            "pick_retreat_joints": [float(value) for value in pick_retreat_joints],
            "place_pose_joints": [float(value) for value in place_joints],
            "place_hover_pose": place_hover_pose.to_dict(),
            "place_hover_joints": [float(value) for value in place_hover_joints],
            "place_retreat_pose": place_retreat_pose.to_dict(),
            "place_retreat_joints": [float(value) for value in place_retreat_joints],
        }
        self._log("pick_and_place_complete", {"label": request.label})
        return result

    def safety_probe(self, request: SafetyProbeRequest) -> Dict[str, Any]:
        target_pose, target_joints = self.resolve_grasp_pose(request.target_pose, "probe_target")
        hover_pose = self._build_offset_pose(
            target_pose,
            request.hover_offset_m
            if request.hover_offset_m is not None
            else target_pose.pre_grasp_offset_m,
            reverse_approach=True,
        )
        near_target_pose = self._build_offset_pose(
            target_pose,
            request.descend_clearance_m,
            reverse_approach=True,
        )
        retreat_pose = self._build_offset_pose(
            target_pose,
            request.retreat_offset_m,
            reverse_approach=True,
        )

        hover_pose, hover_joints = self.resolve_locked_grasp_pose(
            hover_pose,
            "probe_hover",
            reference_joints=target_joints,
        )
        near_target_pose, near_target_joints = self.resolve_locked_grasp_pose(
            near_target_pose,
            "probe_near_target",
            reference_joints=target_joints,
        )
        retreat_pose, retreat_joints = self.resolve_locked_grasp_pose(
            retreat_pose,
            "probe_retreat",
            reference_joints=target_joints,
        )

        if request.open_gripper_first:
            self.open_gripper()
        self.move_joint(hover_joints)
        self.move_linear(self._pose_to_robot_dict(near_target_pose))
        self.move_linear(self._pose_to_robot_dict(retreat_pose))

        result = {
            "success": True,
            "label": request.label,
            "mode": "safety_probe",
            "command_log": self.command_log,
            "target_pose": target_pose.to_dict(),
            "target_joints": [float(value) for value in target_joints],
            "hover_pose": hover_pose.to_dict(),
            "hover_joints": [float(value) for value in hover_joints],
            "near_target_pose": near_target_pose.to_dict(),
            "near_target_joints": [float(value) for value in near_target_joints],
            "retreat_pose": retreat_pose.to_dict(),
            "retreat_joints": [float(value) for value in retreat_joints],
        }
        self._log("safety_probe_complete", {"label": request.label})
        return result

    def _build_offset_pose(
        self,
        grasp_pose: GraspPose,
        offset_m: float,
        reverse_approach: bool,
    ) -> GraspPose:
        direction = self._normalize_vector(grasp_pose.approach_vector)
        sign = -1.0 if reverse_approach else 1.0
        safe_offset_m = self._clamp_offset_to_workspace(
            start_pose=grasp_pose,
            direction=direction,
            requested_offset_m=float(offset_m),
            sign=sign,
        )
        position = Pose3D(
            x=grasp_pose.position.x + sign * direction.x * safe_offset_m,
            y=grasp_pose.position.y + sign * direction.y * safe_offset_m,
            z=grasp_pose.position.z + sign * direction.z * safe_offset_m,
        )
        return GraspPose(
            position=position,
            roll=grasp_pose.roll,
            pitch=grasp_pose.pitch,
            yaw=grasp_pose.yaw,
            approach_vector=grasp_pose.approach_vector,
            pre_grasp_offset_m=grasp_pose.pre_grasp_offset_m,
        )

    def _clamp_offset_to_workspace(
        self,
        start_pose: GraspPose,
        direction: Pose3D,
        requested_offset_m: float,
        sign: float,
        margin_m: float = 0.002,
    ) -> float:
        requested = max(float(requested_offset_m), 0.0)
        move = Pose3D(
            x=sign * direction.x,
            y=sign * direction.y,
            z=sign * direction.z,
        )

        limits: List[float] = []
        axis_specs = (
            (start_pose.position.x, move.x, self.config.workspace.x_min + margin_m, self.config.workspace.x_max - margin_m),
            (start_pose.position.y, move.y, self.config.workspace.y_min + margin_m, self.config.workspace.y_max - margin_m),
            (start_pose.position.z, move.z, self.config.workspace.z_min + margin_m, self.config.workspace.z_max - margin_m),
        )
        for current, delta, lower, upper in axis_specs:
            if abs(delta) < 1e-9:
                continue
            if delta > 0.0:
                allowed = (upper - current) / delta
            else:
                allowed = (lower - current) / delta
            if allowed >= 0.0:
                limits.append(float(allowed))

        if not limits:
            return requested

        max_allowed = max(0.0, min(limits))
        return min(requested, max_allowed)

    def _pose_to_robot_dict(self, pose: GraspPose) -> Dict[str, float]:
        return {
            "x": float(pose.position.x),
            "y": float(pose.position.y),
            "z": float(pose.position.z),
            "rz": float(pose.yaw),
            "ry": float(pose.pitch),
            "rx": float(pose.roll),
        }

    def _normalize_vector(self, vector: Pose3D) -> Pose3D:
        length = math.sqrt(vector.x * vector.x + vector.y * vector.y + vector.z * vector.z)
        if length < 1e-9:
            raise ValueError("approach_vector 长度不能为 0。")
        return Pose3D(vector.x / length, vector.y / length, vector.z / length)

    def _normalize_ik_result(self, result: Any) -> List[float]:
        if isinstance(result, dict):
            if "ok" in result and not bool(result["ok"]):
                return []
            if "joints" in result:
                joints = result["joints"]
            elif all(key in result for key in ("j1", "j2", "j3", "j4", "j5", "j6")):
                joints = [result[f"j{index}"] for index in range(1, 7)]
            else:
                joints = [value for key, value in sorted(result.items()) if key.startswith("j")]
            return [float(value) for value in joints]

        if isinstance(result, (list, tuple)) and len(result) >= 6:
            return [float(value) for value in result[:6]]

        return []

    def _check_joint_limits(self, joints: Sequence[float], label: str) -> None:
        for index, value in enumerate(joints, start=1):
            limit = self.config.joint6_limit_abs_rad if index == 6 else self.config.joint_limit_abs_rad
            if abs(float(value)) > limit:
                raise ValueError(
                    f"{label} 的 IK 解超出保守关节范围: joint{index}={value:.4f} rad, limit={limit:.4f} rad"
                )

    def _unwrap_ik_solution(
        self,
        joints: Sequence[float],
        reference_joints: Sequence[float],
    ) -> List[float]:
        normalized: List[float] = []
        two_pi = 2.0 * math.pi
        for index, value in enumerate(joints):
            reference = float(reference_joints[index]) if index < len(reference_joints) else float(value)
            candidates = [float(value) + two_pi * turns for turns in range(-3, 4)]
            limit = self.config.joint6_limit_abs_rad if index == 5 else self.config.joint_limit_abs_rad
            in_limit = [candidate for candidate in candidates if abs(candidate) <= limit]
            candidate_pool = in_limit if in_limit else candidates
            chosen = min(candidate_pool, key=lambda candidate: abs(candidate - reference))
            normalized.append(float(chosen))
        return normalized

    def validate_grasp_pose(self, grasp_pose: GraspPose, label: str) -> List[float]:
        _, joints = self.resolve_grasp_pose(grasp_pose, label)
        return joints

    def resolve_locked_grasp_pose(
        self,
        grasp_pose: GraspPose,
        label: str,
        reference_joints: Sequence[float],
    ) -> Tuple[GraspPose, List[float]]:
        """
        在保持当前 pose 姿态不变的前提下验证可达性。
        这用于同一段轨迹中的 hover / descend / retreat waypoint，
        避免它们各自重新选一遍 yaw/IK 分支而导致路径突然拧腕或绕远。
        """
        self.config.workspace.assert_contains(grasp_pose.position, label)
        joints = self.solve_ik(self._pose_to_robot_dict(grasp_pose), reference_joints)
        self._check_joint_limits(joints, label)
        self._check_singularity_risk(joints, label)
        return grasp_pose, joints

    def resolve_grasp_pose(self, grasp_pose: GraspPose, label: str) -> Tuple[GraspPose, List[float]]:
        self.config.workspace.assert_contains(grasp_pose.position, label)
        reference_joints = self.get_actual_joint_pose()
        current_tcp_pose = self.get_actual_tcp_pose()

        last_error: Optional[Exception] = None
        best: Optional[Tuple[Tuple[float, float, float, float], GraspPose, List[float]]] = None
        for candidate_pose in self._candidate_grasp_poses(grasp_pose, current_tcp_pose):
            try:
                joints = self.solve_ik(self._pose_to_robot_dict(candidate_pose), reference_joints)
                self._check_joint_limits(joints, label)
                self._check_singularity_risk(joints, label)
            except Exception as exc:
                last_error = exc
                continue

            occupancy = max(
                abs(float(joint)) / self._joint_limit_for_axis(index + 1)
                for index, joint in enumerate(joints)
            )
            joint6_occupancy = abs(float(joints[5])) / self._joint_limit_for_axis(6)
            yaw_delta = abs(self._wrap_angle(candidate_pose.yaw - grasp_pose.yaw))
            motion_delta = sum(abs(float(joint) - float(reference)) for joint, reference in zip(joints, reference_joints))
            score = (occupancy, joint6_occupancy, yaw_delta, motion_delta)
            if best is None or score < best[0]:
                best = (score, candidate_pose, joints)

        if best is not None:
            return best[1], best[2]
        if last_error is not None:
            raise last_error
        raise RuntimeError(f"{label} 未找到可行的抓取位姿。")

    def _candidate_grasp_poses(
        self,
        grasp_pose: GraspPose,
        current_tcp_pose: Dict[str, float],
    ) -> List[GraspPose]:
        yaw_candidates = [
            float(grasp_pose.yaw),
            float(current_tcp_pose.get("rz", grasp_pose.yaw)),
            float(current_tcp_pose.get("rz", grasp_pose.yaw)) + math.pi / 2.0,
            float(current_tcp_pose.get("rz", grasp_pose.yaw)) - math.pi / 2.0,
            float(current_tcp_pose.get("rz", grasp_pose.yaw)) + math.pi,
            float(grasp_pose.yaw) + math.pi / 2.0,
            float(grasp_pose.yaw) - math.pi / 2.0,
            float(grasp_pose.yaw) + math.pi,
        ]
        poses: List[GraspPose] = []
        seen: set[float] = set()
        for yaw in yaw_candidates:
            wrapped = round(self._wrap_angle(yaw), 6)
            if wrapped in seen:
                continue
            seen.add(wrapped)
            poses.append(
                GraspPose(
                    position=grasp_pose.position,
                    roll=grasp_pose.roll,
                    pitch=grasp_pose.pitch,
                    yaw=float(yaw),
                    approach_vector=grasp_pose.approach_vector,
                    pre_grasp_offset_m=grasp_pose.pre_grasp_offset_m,
                )
            )
        return poses

    def _joint_limit_for_axis(self, axis_index: int) -> float:
        return self.config.joint6_limit_abs_rad if axis_index == 6 else self.config.joint_limit_abs_rad

    def _wrap_angle(self, angle_rad: float) -> float:
        return math.atan2(math.sin(angle_rad), math.cos(angle_rad))

    def _check_singularity_risk(self, joints: Sequence[float], label: str) -> None:
        manipulation = self._measure_manipulation(joints)
        if manipulation is not None and manipulation < self.config.min_manipulation:
            raise ValueError(
                f"{label} 的灵活度过低，疑似接近奇异位姿或工作空间边界: "
                f"manipulation={manipulation:.6f}"
            )

        joint5 = float(joints[4])
        distance_to_zero = abs(joint5)
        distance_to_pi = min(abs(joint5 - math.pi), abs(joint5 + math.pi))
        if min(distance_to_zero, distance_to_pi) < self.config.singularity_joint5_threshold_rad:
            raise ValueError(
                f"{label} 疑似接近腕部奇异位姿，已拒绝执行: joint5={joint5:.4f} rad"
            )

    def _measure_manipulation(self, joints: Sequence[float]) -> Optional[float]:
        if self.config.dry_run:
            self._log("measure_manipulation", {"joints": list(joints), "mock": 0.2})
            return 0.2

        if self.robot is None:
            return None

        method = getattr(self.robot, "measure_manipulation", None)
        if not callable(method):
            return None

        try:
            value = method(list(joints))
        except Exception:
            return None
        try:
            return float(value)
        except Exception:
            return None

    def _optional_robot_method(self, method_names: Tuple[str, ...], *args: Any) -> Optional[Any]:
        try:
            return self._call_robot_method(method_names, *args)
        except Exception:
            return None

    def _call_robot_method(self, method_names: Tuple[str, ...], *args: Any) -> Any:
        if self.config.dry_run:
            self._log("dry_run_method", {"method_names": method_names, "args": args})
            return 1

        if self.robot is None:
            raise RuntimeError("机器人尚未连接。")

        last_error: Optional[Exception] = None
        saw_callable_method = False
        for method_name in method_names:
            method = getattr(self.robot, method_name, None)
            if callable(method):
                saw_callable_method = True
                try:
                    return method(*args)
                except Exception as exc:
                    last_error = exc
                    continue

        if saw_callable_method and last_error is not None:
            if self._is_connection_closed_error(last_error):
                self.robot = None
                raise RuntimeError(
                    f"调用机器人方法 {method_names} 时连接已断开: {last_error}。请重新连接后重试。"
                ) from last_error
            raise RuntimeError(
                f"调用机器人方法 {method_names} 失败，最近错误: {last_error}"
            ) from last_error

        raise RuntimeError(f"机器人对象不存在可用方法 {method_names}")

    def _safe_is_connected(self) -> bool:
        if self.robot is None:
            return False
        method = getattr(self.robot, "is_connected", None)
        if not callable(method):
            return True
        try:
            return bool(method())
        except Exception:
            return False

    def _is_connection_closed_error(self, error: BaseException) -> bool:
        text = str(error).lower()
        keywords = (
            "restart required",
            "closed i/o",
            "forcibly closed",
            "10054",
            "远程主机强迫关闭了一个现有的连接",
            "connection reset",
            "broken pipe",
        )
        return any(keyword in text for keyword in keywords)

    def _sleep(self, seconds: float) -> None:
        if self.config.dry_run:
            self._log("sleep", {"seconds": seconds, "mock": True})
            return
        time.sleep(seconds)

    def _log(self, action: str, payload: Dict[str, Any]) -> None:
        self.command_log.append({"action": action, "payload": payload, "ts": time.time()})
