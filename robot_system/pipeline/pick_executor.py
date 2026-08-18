from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence

from robot_system.control import (
    LebaiController,
    LebaiControllerConfig,
    PickPlaceRequest,
    SafetyProbeRequest,
)
from robot_system.llm import QwenPickAgent
from robot_system.llm.types import AgentDecision, GraspPose, ObjectCandidate, Pose3D
from robot_system.vision import Detection2D, VisionTransformer


@dataclass
class PickExecutionResult:
    """统一描述模块一/二/三串联后的执行结果。"""

    success: bool
    user_command: str
    candidates: List[Dict[str, Any]]
    decision: Dict[str, Any]
    execution: Optional[Dict[str, Any]]
    message: str


@dataclass
class PickExecutorConfig:
    """
    抓取执行器配置。

    `drop_pose` 为默认放置位姿，当前采用一个固定回收点。
    """

    calibration_file: str | Path = Path("biaoding/eye_to_hand_result.json")
    drop_pose: GraspPose = field(
        default_factory=lambda: GraspPose(
            position=Pose3D(-0.30, -0.22, 0.15),
            roll=0.0,
            pitch=3.141592653589793,
            yaw=0.0,
            approach_vector=Pose3D(0.0, 0.0, -1.0),
            pre_grasp_offset_m=0.10,
        )
    )


class PickExecutor:
    """
    模块一/二/三编排器。

    输入：
    1. 自然语言抓取指令
    2. 视觉检测结果（像素 + 深度）

    输出：
    1. 候选目标列表
    2. 模块二的决策结果
    3. 模块三的执行结果（可为 dry-run）
    """

    def __init__(
        self,
        vision_transformer: VisionTransformer,
        llm_agent: QwenPickAgent,
        controller: LebaiController,
        config: Optional[PickExecutorConfig] = None,
    ) -> None:
        self.vision_transformer = vision_transformer
        self.llm_agent = llm_agent
        self.controller = controller
        self.config = config or PickExecutorConfig()

    @classmethod
    def from_defaults(
        cls,
        calibration_file: str | Path = Path("biaoding/eye_to_hand_result.json"),
        controller_config: Optional[LebaiControllerConfig] = None,
        config: Optional[PickExecutorConfig] = None,
    ) -> "PickExecutor":
        vision = VisionTransformer.from_calibration_file(calibration_file)
        agent = QwenPickAgent.from_env()
        controller = LebaiController(controller_config or LebaiControllerConfig(robot_ip="127.0.0.1", dry_run=True))
        executor_config = config or PickExecutorConfig(calibration_file=calibration_file)
        return cls(
            vision_transformer=vision,
            llm_agent=agent,
            controller=controller,
            config=executor_config,
        )

    def execute_from_detections(
        self,
        user_command: str,
        detections: Sequence[Detection2D],
        scene_context: Optional[Dict[str, Any]] = None,
        execute_motion: bool = True,
        drop_pose_override: Optional[GraspPose] = None,
        pick_pre_grasp_offset_m: Optional[float] = None,
        pick_post_grasp_offset_m: float = 0.10,
        place_pre_place_offset_m: Optional[float] = None,
        place_post_place_offset_m: float = 0.10,
    ) -> PickExecutionResult:
        """
        从视觉检测结果出发，执行完整抓取流程。
        """
        candidates = self.vision_transformer.build_candidates_from_detections(list(detections))
        return self.execute_from_candidates(
            user_command=user_command,
            candidates=candidates,
            scene_context=scene_context,
            execute_motion=execute_motion,
            drop_pose_override=drop_pose_override,
            pick_pre_grasp_offset_m=pick_pre_grasp_offset_m,
            pick_post_grasp_offset_m=pick_post_grasp_offset_m,
            place_pre_place_offset_m=place_pre_place_offset_m,
            place_post_place_offset_m=place_post_place_offset_m,
        )

    def execute_from_candidates(
        self,
        user_command: str,
        candidates: Sequence[ObjectCandidate],
        scene_context: Optional[Dict[str, Any]] = None,
        execute_motion: bool = True,
        drop_pose_override: Optional[GraspPose] = None,
        pick_pre_grasp_offset_m: Optional[float] = None,
        pick_post_grasp_offset_m: float = 0.10,
        place_pre_place_offset_m: Optional[float] = None,
        place_post_place_offset_m: float = 0.10,
    ) -> PickExecutionResult:
        candidate_list = list(candidates)
        decision = self.llm_agent.plan_pick(
            user_command=user_command,
            candidates=candidate_list,
            scene_context=scene_context or {},
        )

        if not decision.success or decision.grasp_pose is None:
            return PickExecutionResult(
                success=False,
                user_command=user_command,
                candidates=[item.to_prompt_dict() for item in candidate_list],
                decision=decision.to_dict(),
                execution=None,
                message=decision.reason,
            )

        execution_result = None
        if execute_motion:
            self.controller.connect()
            try:
                execution_result = self.controller.pick_and_place(
                    PickPlaceRequest(
                        pick_pose=decision.grasp_pose,
                        place_pose=drop_pose_override or self.config.drop_pose,
                        label=f"pick_{decision.selected_object_id or 'unknown'}",
                        pre_grasp_offset_m=pick_pre_grasp_offset_m,
                        post_grasp_offset_m=pick_post_grasp_offset_m,
                        pre_place_offset_m=place_pre_place_offset_m,
                        post_place_offset_m=place_post_place_offset_m,
                    )
                )
            finally:
                self.controller.disconnect()

        message = (
            f"已为指令“{user_command}”选中目标 {decision.selected_object_name or decision.selected_object_id}。"
        )
        if execute_motion:
            message += " 抓取流程已提交给机械臂控制模块。"
        else:
            message += " 当前仅完成决策与规划，未执行机械臂动作。"

        return PickExecutionResult(
            success=True,
            user_command=user_command,
            candidates=[item.to_prompt_dict() for item in candidate_list],
            decision=decision.to_dict(),
            execution=execution_result,
            message=message,
        )

    def safety_probe_from_detections(
        self,
        user_command: str,
        detections: Sequence[Detection2D],
        scene_context: Optional[Dict[str, Any]] = None,
        connect_robot: bool = True,
        descend_clearance_m: float = 0.04,
        hover_offset_m: Optional[float] = None,
    ) -> PickExecutionResult:
        """
        从视觉检测结果出发，执行一次低风险真机探测。

        适合首次上真机时验证：
        1. 像素到 Base 坐标链路是否正确；
        2. 机械臂是否能移动到目标上方；
        3. 抓取朝向是否大致正确。
        """
        candidates = self.vision_transformer.build_candidates_from_detections(list(detections))
        decision = self.llm_agent.plan_pick(
            user_command=user_command,
            candidates=candidates,
            scene_context=scene_context or {},
        )

        if not decision.success or decision.grasp_pose is None:
            return PickExecutionResult(
                success=False,
                user_command=user_command,
                candidates=[item.to_prompt_dict() for item in candidates],
                decision=decision.to_dict(),
                execution=None,
                message=decision.reason,
            )

        execution_result = None
        if connect_robot:
            self.controller.connect()
            try:
                execution_result = self.controller.safety_probe(
                    SafetyProbeRequest(
                        target_pose=decision.grasp_pose,
                        label=f"probe_{decision.selected_object_id or 'unknown'}",
                        descend_clearance_m=descend_clearance_m,
                        hover_offset_m=hover_offset_m,
                    )
                )
            finally:
                self.controller.disconnect()

        return PickExecutionResult(
            success=True,
            user_command=user_command,
            candidates=[item.to_prompt_dict() for item in candidates],
            decision=decision.to_dict(),
            execution=execution_result,
            message=(
                f"已为指令“{user_command}”生成安全探测动作，"
                f"目标为 {decision.selected_object_name or decision.selected_object_id}。"
            ),
        )
