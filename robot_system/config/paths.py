"""Central path resolution for entry points and bundled repository assets."""

from __future__ import annotations

import os
from pathlib import Path


def repository_root() -> Path:
    """Return the repository root without depending on the current directory."""

    return Path(__file__).resolve().parents[2]


def resolve_from_workspace(
    value: str | Path,
    *,
    workspace_env: str = "LEBAI_WORKSPACE_ROOT",
) -> Path:
    """Resolve runtime data relative to an explicit workspace or the current directory."""

    candidate = Path(value).expanduser()
    if candidate.is_absolute():
        return candidate.resolve()
    workspace = Path(os.getenv(workspace_env, str(Path.cwd()))).expanduser().resolve()
    return (workspace / candidate).resolve()
