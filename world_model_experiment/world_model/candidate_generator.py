from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Sequence, Tuple

from robot_system.control import LebaiController
from robot_system.llm.types import GraspPose, Pose3D

from .types import GraspActionCandidate, Position3D, WorldState


@dataclass
class CandidateGeneratorConfig:
    xy_offsets_m: Tuple[Tuple[float, float], ...] = (
        (0.0, 0.0),
        (0.010, 0.0),
        (-0.010, 0.0),
        (0.0, 0.010),
        (0.0, -0.010),
    )
    yaw_offsets_rad: Tuple[float, ...] = (0.0, math.pi / 2.0)
    pre_grasp_offset_m: float = 0.08
    gripper_opening: float = 0.65
    max_candidates: int = 10


class DryRunCandidateGenerator:
    """复用主项目位姿解析器和安全检查生成 Dry-run 抓取候选。"""

    def __init__(self, controller: LebaiController, config: CandidateGeneratorConfig | None = None) -> None:
        if not controller.config.dry_run:
            raise PermissionError("初步候选生成器只允许使用 dry_run=True 的控制器。")
        self.controller = controller
        self.config = config or CandidateGeneratorConfig()

    def generate(self, state: WorldState, target_track_id: str | None = None) -> List[GraspActionCandidate]:
        target_id = target_track_id or state.task.target_track_id
        if not target_id:
            raise ValueError("任务没有指定目标 track_id。")
        target = state.object_by_id(target_id)
        current_tcp = state.robot.tcp_pose
        candidates: List[GraspActionCandidate] = []

        for offset_index, (dx, dy) in enumerate(self.config.xy_offsets_m):
            for yaw_index, yaw_offset in enumerate(self.config.yaw_offsets_rad):
                if len(candidates) >= self.config.max_candidates:
                    return candidates

                requested_position = Position3D(
                    target.position_base_m.x + float(dx),
                    target.position_base_m.y + float(dy),
                    target.position_base_m.z,
                )
                requested_yaw = float(target.yaw_rad + yaw_offset)
                grasp_pose = GraspPose(
                    position=Pose3D(requested_position.x, requested_position.y, requested_position.z),
                    roll=0.0,
                    pitch=math.pi,
                    yaw=requested_yaw,
                    approach_vector=Pose3D(0.0, 0.0, -1.0),
                    pre_grasp_offset_m=self.config.pre_grasp_offset_m,
                )

                ik_reachable = True
                resolved_position = requested_position
                resolved_yaw = requested_yaw
                joints: Sequence[float] = ()
                error = None
                try:
                    resolved_pose, joints = self.controller.resolve_grasp_pose(
                        grasp_pose,
                        label=f"world_model_candidate_{offset_index}_{yaw_index}",
                    )
                    resolved_position = Position3D(
                        resolved_pose.position.x,
                        resolved_pose.position.y,
                        resolved_pose.position.z,
                    )
                    resolved_yaw = float(resolved_pose.yaw)
                except Exception as exc:
                    ik_reachable = False
                    error = str(exc)

                workspace_margin = self._workspace_margin(resolved_position)
                joint_margin = self._joint_margin(joints) if joints else 0.0
                singularity_margin = self._singularity_margin(joints) if joints else 0.0
                clearance = max(resolved_position.z - self.controller.config.workspace.z_min, 0.0)
                path_length = math.dist(current_tcp[:3], (resolved_position.x, resolved_position.y, resolved_position.z))
                source = "center" if dx == 0.0 and dy == 0.0 else "xy_offset"

                candidates.append(
                    GraspActionCandidate(
                        candidate_id=f"{target_id}_o{offset_index}_y{yaw_index}",
                        target_track_id=target_id,
                        grasp_position_base_m=resolved_position,
                        roll_rad=0.0,
                        pitch_rad=math.pi,
                        yaw_rad=resolved_yaw,
                        pre_grasp_offset_m=self.config.pre_grasp_offset_m,
                        gripper_opening=self.config.gripper_opening,
                        ik_reachable=ik_reachable,
                        workspace_margin_m=workspace_margin,
                        joint_margin_rad=joint_margin,
                        singularity_margin_rad=singularity_margin,
                        minimum_clearance_m=clearance,
                        path_length_m=path_length,
                        source=source,
                        metadata={"offset_xy_m": [dx, dy], "requested_yaw_rad": requested_yaw, "validation_error": error},
                    )
                )

        return candidates

    def _workspace_margin(self, position: Position3D) -> float:
        workspace = self.controller.config.workspace
        return min(
            position.x - workspace.x_min,
            workspace.x_max - position.x,
            position.y - workspace.y_min,
            workspace.y_max - position.y,
            position.z - workspace.z_min,
            workspace.z_max - position.z,
        )

    def _joint_margin(self, joints: Sequence[float]) -> float:
        margins = []
        for index, joint in enumerate(joints):
            limit = self.controller.config.joint6_limit_abs_rad if index == 5 else self.controller.config.joint_limit_abs_rad
            margins.append(limit - abs(float(joint)))
        return max(min(margins), 0.0)

    @staticmethod
    def _singularity_margin(joints: Sequence[float]) -> float:
        joint5 = float(joints[4])
        return min(abs(joint5), abs(joint5 - math.pi), abs(joint5 + math.pi))
