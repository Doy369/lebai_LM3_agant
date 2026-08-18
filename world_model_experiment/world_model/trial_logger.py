from __future__ import annotations

import json
import re
import time
import uuid
from dataclasses import asdict, is_dataclass
from enum import Enum
from pathlib import Path
from typing import Any, Dict, Optional

from .types import DecisionResult, GraspActionCandidate, TrialOutcome, WorldState


def _json_default(value: Any) -> Any:
    if isinstance(value, Enum):
        return value.value
    if is_dataclass(value):
        return asdict(value)
    raise TypeError(f"无法序列化 {type(value).__name__}。")


class TrialLogger:
    """按试验目录记录状态、候选、预测和人工核验结果。"""

    def __init__(self, root: str | Path) -> None:
        self.root = Path(root)

    def start_trial(self, prefix: str = "trial") -> Path:
        safe_prefix = re.sub(r"[^A-Za-z0-9_-]+", "_", prefix).strip("_") or "trial"
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        trial_dir = self.root / f"{safe_prefix}_{timestamp}_{uuid.uuid4().hex[:8]}"
        trial_dir.mkdir(parents=True, exist_ok=False)
        return trial_dir

    def write_json(self, trial_dir: str | Path, filename: str, payload: Any) -> Path:
        path = Path(trial_dir) / filename
        if path.parent.resolve() != Path(trial_dir).resolve():
            raise ValueError("试验日志文件必须直接写入当前试验目录。")
        with path.open("w", encoding="utf-8") as file:
            json.dump(payload, file, ensure_ascii=False, indent=2, default=_json_default)
        return path

    def record_planning(
        self,
        trial_dir: str | Path,
        state: WorldState,
        candidates: list[GraspActionCandidate],
        decision: DecisionResult,
    ) -> None:
        self.write_json(trial_dir, "world_state_before.json", state)
        self.write_json(trial_dir, "candidates.json", candidates)
        self.write_json(trial_dir, "decision.json", decision.to_dict())

    def record_outcome(
        self,
        trial_dir: str | Path,
        outcome: TrialOutcome,
        state_after: Optional[WorldState] = None,
    ) -> None:
        self.write_json(trial_dir, "outcome.json", outcome.to_dict())
        if state_after is not None:
            self.write_json(trial_dir, "world_state_after.json", state_after)
