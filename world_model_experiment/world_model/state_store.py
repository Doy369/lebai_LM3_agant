from __future__ import annotations

import json
import os
import tempfile
from pathlib import Path
from typing import Any, Dict

from .types import WorldState


class WorldStateStore:
    """以原子替换方式保存最新世界状态，避免中断产生半截 JSON。"""

    def __init__(self, path: str | Path) -> None:
        self.path = Path(path)

    def save(self, state: WorldState) -> Path:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        payload = state.to_dict()
        handle, temp_name = tempfile.mkstemp(prefix=f".{self.path.name}.", suffix=".tmp", dir=str(self.path.parent))
        try:
            with os.fdopen(handle, "w", encoding="utf-8") as file:
                json.dump(payload, file, ensure_ascii=False, indent=2)
                file.flush()
                os.fsync(file.fileno())
            os.replace(temp_name, self.path)
        except Exception:
            try:
                os.unlink(temp_name)
            except OSError:
                pass
            raise
        return self.path

    def load_dict(self) -> Dict[str, Any]:
        with self.path.open("r", encoding="utf-8") as file:
            payload = json.load(file)
        if not isinstance(payload, dict):
            raise ValueError("世界状态文件根节点必须是 JSON 对象。")
        return payload
