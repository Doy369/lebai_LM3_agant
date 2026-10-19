"""统一抓取计划的数据结构。"""

from .compiled_plan import CartesianTarget, CompiledGraspPlan, CompiledPlanStep, PlanValidation
from .collision_validation import (
    ApproximateCollisionGeometry,
    AxisAlignedBox,
    CollisionContact,
    CollisionValidationReport,
    validate_model_collision,
)
from .trajectory_validation import (
    SegmentValidation,
    TrajectorySample,
    TrajectoryValidationConfig,
    TrajectoryValidationReport,
    validate_compiled_trajectory,
)

__all__ = [
    "CartesianTarget",
    "ApproximateCollisionGeometry",
    "AxisAlignedBox",
    "CollisionContact",
    "CollisionValidationReport",
    "CompiledGraspPlan",
    "CompiledPlanStep",
    "PlanValidation",
    "SegmentValidation",
    "TrajectorySample",
    "TrajectoryValidationConfig",
    "TrajectoryValidationReport",
    "validate_model_collision",
    "validate_compiled_trajectory",
]
