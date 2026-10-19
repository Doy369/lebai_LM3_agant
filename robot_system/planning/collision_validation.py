from __future__ import annotations

import hashlib
import json
import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Iterable, Mapping, Optional, Sequence, Tuple

from .compiled_plan import CompiledGraspPlan


Vector3 = Tuple[float, float, float]
Matrix3 = Tuple[Vector3, Vector3, Vector3]
DEFAULT_GEOMETRY_FILE = Path(__file__).with_name("collision_geometry.json")


def _vector(value: Any, name: str, *, positive: bool = False) -> Vector3:
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise ValueError(f"{name} 必须包含 3 个数值。")
    result = tuple(float(item) for item in value)
    if any(not math.isfinite(item) for item in result):
        raise ValueError(f"{name} 必须是有限数值。")
    if positive and any(item <= 0 for item in result):
        raise ValueError(f"{name} 的三个尺寸必须大于 0。")
    return result  # type: ignore[return-value]


def _dot(a: Vector3, b: Vector3) -> float:
    return sum(x * y for x, y in zip(a, b))


def _sub(a: Vector3, b: Vector3) -> Vector3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _add(a: Vector3, b: Vector3) -> Vector3:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _scale(a: Vector3, factor: float) -> Vector3:
    return (a[0] * factor, a[1] * factor, a[2] * factor)


def _mat_mul(a: Matrix3, b: Matrix3) -> Matrix3:
    bt = tuple(zip(*b))
    return tuple(tuple(_dot(row, col) for col in bt) for row in a)  # type: ignore[return-value]


def _mat_vec(matrix: Matrix3, value: Vector3) -> Vector3:
    return tuple(_dot(row, value) for row in matrix)  # type: ignore[return-value]


def _axis_rotation(axis: str, angle: float) -> Matrix3:
    c, s = math.cos(angle), math.sin(angle)
    if axis == "x":
        return ((1.0, 0.0, 0.0), (0.0, c, -s), (0.0, s, c))
    if axis == "y":
        return ((c, 0.0, s), (0.0, 1.0, 0.0), (-s, 0.0, c))
    if axis == "z":
        return ((c, -s, 0.0), (s, c, 0.0), (0.0, 0.0, 1.0))
    raise ValueError(f"未知关节轴: {axis}")


def _closest_segment_segment(
    a0: Vector3, a1: Vector3, b0: Vector3, b1: Vector3,
) -> Tuple[float, Vector3, Vector3]:
    """Shortest distance and centerline points between two finite 3D segments."""
    u, v, w = _sub(a1, a0), _sub(b1, b0), _sub(a0, b0)
    aa, bb, cc, dd, ee = _dot(u, u), _dot(u, v), _dot(v, v), _dot(u, w), _dot(v, w)
    denominator = aa * cc - bb * bb
    small = 1e-12
    s_num, s_den = denominator, denominator
    t_num, t_den = denominator, denominator
    if denominator < small:
        s_num, s_den, t_num, t_den = 0.0, 1.0, ee, cc
    else:
        s_num, t_num = bb * ee - cc * dd, aa * ee - bb * dd
        if s_num < 0.0:
            s_num, t_num, t_den = 0.0, ee, cc
        elif s_num > s_den:
            s_num, t_num, t_den = s_den, ee + bb, cc
    if t_num < 0.0:
        t_num = 0.0
        if -dd < 0.0:
            s_num = 0.0
        elif -dd > aa:
            s_num = s_den
        else:
            s_num, s_den = -dd, aa
    elif t_num > t_den:
        t_num = t_den
        if -dd + bb < 0.0:
            s_num = 0.0
        elif -dd + bb > aa:
            s_num = s_den
        else:
            s_num, s_den = -dd + bb, aa
    sc = 0.0 if abs(s_num) < small else s_num / s_den
    tc = 0.0 if abs(t_num) < small else t_num / t_den
    first = _add(a0, _scale(u, sc))
    second = _add(b0, _scale(v, tc))
    delta = _sub(first, second)
    return math.sqrt(max(0.0, _dot(delta, delta))), first, second


def _point_aabb_distance_sq(point: Vector3, minimum: Vector3, maximum: Vector3) -> float:
    total = 0.0
    for value, low, high in zip(point, minimum, maximum):
        if value < low:
            total += (low - value) ** 2
        elif value > high:
            total += (value - high) ** 2
    return total


def _closest_segment_aabb(
    a: Vector3, b: Vector3, minimum: Vector3, maximum: Vector3,
) -> Tuple[float, Vector3, Vector3]:
    """Exact distance and closest points for a segment and an axis-aligned box."""
    direction = _sub(b, a)
    breakpoints = {0.0, 1.0}
    for start, delta, low, high in zip(a, direction, minimum, maximum):
        if abs(delta) > 1e-12:
            breakpoints.add(min(1.0, max(0.0, (low - start) / delta)))
            breakpoints.add(min(1.0, max(0.0, (high - start) / delta)))
    ordered = sorted(breakpoints)
    candidates = set(ordered)
    for left, right in zip(ordered, ordered[1:]):
        midpoint = (left + right) * 0.5
        numerator = 0.0
        denominator = 0.0
        for start, delta, low, high in zip(a, direction, minimum, maximum):
            value = start + delta * midpoint
            boundary: Optional[float] = low if value < low else high if value > high else None
            if boundary is not None:
                numerator += delta * (start - boundary)
                denominator += delta * delta
        if denominator > 0.0:
            candidates.add(min(right, max(left, -numerator / denominator)))
    best_t = min(
        candidates,
        key=lambda t: _point_aabb_distance_sq(_add(a, _scale(direction, t)), minimum, maximum),
    )
    segment_point = _add(a, _scale(direction, best_t))
    box_point = tuple(
        min(high, max(low, value))
        for value, low, high in zip(segment_point, minimum, maximum)
    )
    delta = _sub(segment_point, box_point)  # type: ignore[arg-type]
    return math.sqrt(max(0.0, _dot(delta, delta))), segment_point, box_point  # type: ignore[arg-type]


def _capsule_surface_points(
    first: Vector3,
    second: Vector3,
    first_radius: float,
    second_radius: float = 0.0,
) -> Tuple[Vector3, Vector3]:
    delta = _sub(second, first)
    distance = math.sqrt(max(0.0, _dot(delta, delta)))
    if distance <= 1e-12:
        return first, second
    direction = _scale(delta, 1.0 / distance)
    return _add(first, _scale(direction, first_radius)), _sub(second, _scale(direction, second_radius))


@dataclass(frozen=True)
class AxisAlignedBox:
    object_id: str
    center_base_m: Vector3
    size_xyz_m: Vector3
    source: str

    @property
    def minimum(self) -> Vector3:
        return tuple(c - size * 0.5 for c, size in zip(self.center_base_m, self.size_xyz_m))  # type: ignore[return-value]

    @property
    def maximum(self) -> Vector3:
        return tuple(c + size * 0.5 for c, size in zip(self.center_base_m, self.size_xyz_m))  # type: ignore[return-value]

    def to_dict(self) -> Dict[str, Any]:
        return {
            "object_id": self.object_id,
            "center_base_m": list(self.center_base_m),
            "size_xyz_m": list(self.size_xyz_m),
            "source": self.source,
        }


@dataclass(frozen=True)
class ApproximateCollisionGeometry:
    model_id: str
    geometry_status: str
    kinematic_mapping_status: str
    joint_axes: Tuple[str, ...]
    joint_directions: Tuple[int, ...]
    joint_zero_offsets_rad: Tuple[float, ...]
    joint_local_translations_model_m: Tuple[Vector3, ...]
    link_radii_m: Tuple[float, ...]
    safety_margin_m: float
    table: AxisAlignedBox
    table_ignored_links: Tuple[str, ...]
    self_collision_ignored_pairs: Tuple[Tuple[str, str], ...]
    source: str

    @classmethod
    def load(cls, path: Path = DEFAULT_GEOMETRY_FILE) -> "ApproximateCollisionGeometry":
        payload = json.loads(path.read_text(encoding="utf-8"))
        if int(payload.get("schema_version", 0)) != 1:
            raise ValueError("碰撞几何配置 schema_version 不受支持。")
        table = payload["table"]
        result = cls(
            model_id=str(payload["model_id"]),
            geometry_status=str(payload["geometry_status"]),
            kinematic_mapping_status=str(payload["kinematic_mapping_status"]),
            joint_axes=tuple(str(value) for value in payload["joint_axes"]),
            joint_directions=tuple(int(value) for value in payload["joint_directions"]),
            joint_zero_offsets_rad=tuple(float(value) for value in payload["joint_zero_offsets_rad"]),
            joint_local_translations_model_m=tuple(
                _vector(value, f"joint_local_translations_model_m[{index}]")
                for index, value in enumerate(payload["joint_local_translations_model_m"])
            ),
            link_radii_m=tuple(float(value) for value in payload["link_radii_m"]),
            safety_margin_m=float(payload["safety_margin_m"]),
            table=AxisAlignedBox(
                object_id=str(table["object_id"]),
                center_base_m=_vector(table["center_base_m"], "table.center_base_m"),
                size_xyz_m=_vector(table["size_xyz_m"], "table.size_xyz_m", positive=True),
                source="configured_default",
            ),
            table_ignored_links=tuple(str(value) for value in payload["table_ignored_links"]),
            self_collision_ignored_pairs=tuple(
                tuple(sorted((str(pair[0]), str(pair[1]))))
                for pair in payload["self_collision_ignored_pairs"]
            ),
            source=str(payload["source"]),
        )
        counts = {
            len(result.joint_axes), len(result.joint_directions), len(result.joint_zero_offsets_rad),
            len(result.joint_local_translations_model_m), len(result.link_radii_m),
        }
        if counts != {6} or any(axis not in {"x", "y", "z"} for axis in result.joint_axes):
            raise ValueError("碰撞几何配置必须包含一致的六关节参数。")
        if result.safety_margin_m < 0 or any(radius <= 0 for radius in result.link_radii_m):
            raise ValueError("碰撞半径必须大于 0，安全余量不能为负数。")
        return result

    def link_capsules(self, joints_rad: Sequence[float]) -> Tuple[Tuple[str, Vector3, Vector3, float], ...]:
        if len(joints_rad) != 6:
            raise ValueError("碰撞正运动学需要 6 个关节角。")
        # base_T_model = Rx(+pi/2), matching robot3d/coordinates.js.
        rotation: Matrix3 = ((1.0, 0.0, 0.0), (0.0, 0.0, -1.0), (0.0, 1.0, 0.0))
        position: Vector3 = (0.0, 0.0, 0.0)
        origins = [position]
        for index, (translation, axis, direction, offset) in enumerate(zip(
            self.joint_local_translations_model_m,
            self.joint_axes,
            self.joint_directions,
            self.joint_zero_offsets_rad,
        )):
            position = _add(position, _mat_vec(rotation, translation))
            origins.append(position)
            angle = direction * float(joints_rad[index]) + offset
            rotation = _mat_mul(rotation, _axis_rotation(axis, angle))
        return tuple(
            (f"link_{index}", origins[index], origins[index + 1], self.link_radii_m[index])
            for index in range(6)
        )


@dataclass(frozen=True)
class CollisionContact:
    sample_index: int
    step_id: str
    phase: str
    first: str
    second: str
    clearance_m: float

    def to_dict(self) -> Dict[str, Any]:
        return {
            "sample_index": self.sample_index,
            "step_id": self.step_id,
            "phase": self.phase,
            "first": self.first,
            "second": self.second,
            "clearance_m": self.clearance_m,
        }


@dataclass(frozen=True)
class CollisionValidationReport:
    report_id: str
    plan_id: str
    status: str
    geometry_status: str
    scene_status: str
    model_id: str
    checked_at_unix_ms: int
    checked_sample_count: int
    obstacle_count: int
    safety_margin_m: float
    minimum_clearance_m: Optional[float]
    closest_pair: Optional[Tuple[str, str]]
    closest_points_base_m: Optional[Tuple[Vector3, Vector3]]
    closest_sample_index: Optional[int]
    closest_step_id: Optional[str]
    closest_phase: Optional[str]
    link_radii_m: Tuple[float, ...]
    obstacles: Tuple[AxisAlignedBox, ...]
    contacts: Tuple[CollisionContact, ...]
    issues: Tuple[str, ...]
    schema_version: int = 1

    @classmethod
    def from_dict(cls, payload: Mapping[str, Any]) -> "CollisionValidationReport":
        expected = {
            "schema_version", "report_id", "plan_id", "status", "geometry_status", "scene_status",
            "model_id", "checked_at_unix_ms", "checked_sample_count", "obstacle_count", "safety_margin_m",
            "minimum_clearance_m", "closest_pair", "closest_points_base_m", "closest_sample_index",
            "closest_step_id", "closest_phase", "link_radii_m", "obstacles", "contacts", "issues",
        }
        if set(payload) != expected:
            raise ValueError("碰撞验证报告字段不匹配。")
        closest = payload["closest_pair"]
        closest_points = payload["closest_points_base_m"]
        report = cls(
            schema_version=int(payload["schema_version"]),
            report_id=str(payload["report_id"]),
            plan_id=str(payload["plan_id"]),
            status=str(payload["status"]),
            geometry_status=str(payload["geometry_status"]),
            scene_status=str(payload["scene_status"]),
            model_id=str(payload["model_id"]),
            checked_at_unix_ms=int(payload["checked_at_unix_ms"]),
            checked_sample_count=int(payload["checked_sample_count"]),
            obstacle_count=int(payload["obstacle_count"]),
            safety_margin_m=float(payload["safety_margin_m"]),
            minimum_clearance_m=None if payload["minimum_clearance_m"] is None else float(payload["minimum_clearance_m"]),
            closest_pair=None if closest is None else (str(closest[0]), str(closest[1])),
            closest_points_base_m=(
                None if closest_points is None else (
                    _vector(closest_points[0], "closest_points_base_m[0]"),
                    _vector(closest_points[1], "closest_points_base_m[1]"),
                )
            ),
            closest_sample_index=(
                None if payload["closest_sample_index"] is None else int(payload["closest_sample_index"])
            ),
            closest_step_id=None if payload["closest_step_id"] is None else str(payload["closest_step_id"]),
            closest_phase=None if payload["closest_phase"] is None else str(payload["closest_phase"]),
            link_radii_m=tuple(float(value) for value in payload["link_radii_m"]),
            obstacles=tuple(AxisAlignedBox(
                object_id=str(item["object_id"]),
                center_base_m=_vector(item["center_base_m"], "obstacle.center_base_m"),
                size_xyz_m=_vector(item["size_xyz_m"], "obstacle.size_xyz_m", positive=True),
                source=str(item["source"]),
            ) for item in payload["obstacles"]),
            contacts=tuple(CollisionContact(
                sample_index=int(item["sample_index"]),
                step_id=str(item["step_id"]),
                phase=str(item["phase"]),
                first=str(item["first"]),
                second=str(item["second"]),
                clearance_m=float(item["clearance_m"]),
            ) for item in payload["contacts"]),
            issues=tuple(str(item) for item in payload["issues"]),
        )
        if report.status not in {"collision", "clear_approximate"}:
            raise ValueError("碰撞验证报告状态无效。")
        if len(report.link_radii_m) != 6 or any(value <= 0 or not math.isfinite(value) for value in report.link_radii_m):
            raise ValueError("碰撞验证报告的连杆半径无效。")
        if report.obstacle_count != len(report.obstacles):
            raise ValueError("碰撞验证报告的障碍物数量不一致。")
        if report.report_id != report.expected_report_id():
            raise ValueError("碰撞验证报告内容与 report_id 哈希不一致。")
        return report

    def expected_report_id(self) -> str:
        payload = self.to_dict()
        payload.pop("report_id", None)
        canonical = json.dumps(payload, ensure_ascii=False, sort_keys=True, separators=(",", ":"), allow_nan=False)
        return "collision_" + hashlib.sha256(canonical.encode("utf-8")).hexdigest()[:24]

    def to_dict(self) -> Dict[str, Any]:
        return {
            "schema_version": self.schema_version,
            "report_id": self.report_id,
            "plan_id": self.plan_id,
            "status": self.status,
            "geometry_status": self.geometry_status,
            "scene_status": self.scene_status,
            "model_id": self.model_id,
            "checked_at_unix_ms": self.checked_at_unix_ms,
            "checked_sample_count": self.checked_sample_count,
            "obstacle_count": self.obstacle_count,
            "safety_margin_m": self.safety_margin_m,
            "minimum_clearance_m": self.minimum_clearance_m,
            "closest_pair": list(self.closest_pair) if self.closest_pair else None,
            "closest_points_base_m": (
                [list(point) for point in self.closest_points_base_m]
                if self.closest_points_base_m else None
            ),
            "closest_sample_index": self.closest_sample_index,
            "closest_step_id": self.closest_step_id,
            "closest_phase": self.closest_phase,
            "link_radii_m": list(self.link_radii_m),
            "obstacles": [obstacle.to_dict() for obstacle in self.obstacles],
            "contacts": [contact.to_dict() for contact in self.contacts],
            "issues": list(self.issues),
        }


def _scene_boxes(
    scene_context: Optional[Mapping[str, Any]],
    geometry: ApproximateCollisionGeometry,
    target_object_id: str,
) -> Tuple[Tuple[AxisAlignedBox, ...], str, Tuple[str, ...]]:
    boxes = [geometry.table]
    issues = []
    raw = None if scene_context is None else scene_context.get("collision_obstacles", scene_context.get("obstacles"))
    if raw is None:
        return tuple(boxes), "table_only_incomplete", (
            "场景未提供 collision_obstacles，当前仅检查默认桌面。",
        )
    if not isinstance(raw, list):
        return tuple(boxes), "invalid_obstacles", ("scene_context.obstacles 必须是数组，已忽略。",)
    for index, item in enumerate(raw[:100]):
        if not isinstance(item, Mapping):
            issues.append(f"obstacles[{index}] 不是对象，已忽略。")
            continue
        object_id = str(item.get("object_id") or item.get("id") or f"scene_obstacle_{index}")
        if object_id == target_object_id:
            continue
        center = item.get("center_base_m", item.get("position_base_m"))
        size = item.get("size_xyz_m", item.get("size_m"))
        try:
            boxes.append(AxisAlignedBox(
                object_id=object_id,
                center_base_m=_vector(center, f"obstacles[{index}].center_base_m"),
                size_xyz_m=_vector(size, f"obstacles[{index}].size_xyz_m", positive=True),
                source="scene_context",
            ))
        except (TypeError, ValueError) as exc:
            issues.append(str(exc))
    status = "provided_obstacles_unverified" if len(boxes) > 1 else "provided_but_empty"
    return tuple(boxes), status, tuple(issues)


def validate_model_collision(
    plan: CompiledGraspPlan,
    samples: Iterable[Any],
    *,
    scene_context: Optional[Mapping[str, Any]] = None,
    geometry: Optional[ApproximateCollisionGeometry] = None,
    checked_at_unix_ms: Optional[int] = None,
) -> CollisionValidationReport:
    """Check sampled arm capsules against boxes and non-adjacent arm links.

    The current GLB mapping and radii are explicitly approximate. Intersections reject a
    plan, while a clear result remains evidence for re-observation rather than safety proof.
    """
    model = geometry or ApproximateCollisionGeometry.load()
    boxes, scene_status, scene_issues = _scene_boxes(scene_context, model, plan.target_object_id)
    all_samples = [
        type("StartSample", (), {
            "sample_index": -1, "step_id": "plan_start", "phase": "plan_start",
            "joints_rad": plan.start_joints_rad,
        })()
    ] + list(samples)
    minimum = math.inf
    closest: Optional[Tuple[str, str]] = None
    closest_points: Optional[Tuple[Vector3, Vector3]] = None
    closest_sample: Optional[Any] = None
    contacts = []
    for sample in all_samples:
        capsules = model.link_capsules(sample.joints_rad)
        for link_name, start, end, radius in capsules:
            for box in boxes:
                if box.object_id == model.table.object_id and link_name in model.table_ignored_links:
                    continue
                distance, centerline_point, box_point = _closest_segment_aabb(
                    start, end, box.minimum, box.maximum,
                )
                clearance = distance - radius
                if clearance < minimum:
                    minimum, closest = clearance, (link_name, box.object_id)
                    closest_points = _capsule_surface_points(centerline_point, box_point, radius)
                    closest_sample = sample
                if clearance <= model.safety_margin_m:
                    contacts.append(CollisionContact(
                        sample_index=int(sample.sample_index), step_id=str(sample.step_id), phase=str(sample.phase),
                        first=link_name, second=box.object_id, clearance_m=clearance,
                    ))
        for first_index, first in enumerate(capsules):
            for second_index in range(first_index + 2, len(capsules)):
                second = capsules[second_index]
                if tuple(sorted((first[0], second[0]))) in model.self_collision_ignored_pairs:
                    continue
                distance, first_centerline, second_centerline = _closest_segment_segment(
                    first[1], first[2], second[1], second[2],
                )
                clearance = distance - first[3] - second[3]
                if clearance < minimum:
                    minimum, closest = clearance, (first[0], second[0])
                    closest_points = _capsule_surface_points(
                        first_centerline, second_centerline, first[3], second[3],
                    )
                    closest_sample = sample
                if clearance <= model.safety_margin_m:
                    contacts.append(CollisionContact(
                        sample_index=int(sample.sample_index), step_id=str(sample.step_id), phase=str(sample.phase),
                        first=first[0], second=second[0], clearance_m=clearance,
                    ))
    status = "collision" if contacts else "clear_approximate"
    issues = list(scene_issues)
    issues.append(
        "检测到近似几何相交，计划必须拒绝并复核模型参数。"
        if contacts else
        "近似模型未发现相交；因连杆半径、工具几何和部分关节方向未实测，碰撞状态仍为 unknown。"
    )
    provisional = CollisionValidationReport(
        report_id="pending",
        plan_id=plan.plan_id,
        status=status,
        geometry_status=model.geometry_status,
        scene_status=scene_status,
        model_id=model.model_id,
        checked_at_unix_ms=int(checked_at_unix_ms if checked_at_unix_ms is not None else time.time() * 1000),
        checked_sample_count=len(all_samples),
        obstacle_count=len(boxes),
        safety_margin_m=model.safety_margin_m,
        minimum_clearance_m=None if minimum == math.inf else minimum,
        closest_pair=closest,
        closest_points_base_m=closest_points,
        closest_sample_index=(None if closest_sample is None else int(closest_sample.sample_index)),
        closest_step_id=(None if closest_sample is None else str(closest_sample.step_id)),
        closest_phase=(None if closest_sample is None else str(closest_sample.phase)),
        link_radii_m=model.link_radii_m,
        obstacles=boxes,
        contacts=tuple(contacts[:200]),
        issues=tuple(issues),
    )
    return CollisionValidationReport(**{**provisional.__dict__, "report_id": provisional.expected_report_id()})


__all__ = [
    "ApproximateCollisionGeometry",
    "AxisAlignedBox",
    "CollisionContact",
    "CollisionValidationReport",
    "validate_model_collision",
]
