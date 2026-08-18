from __future__ import annotations

import os
import runpy
import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parent
LEGACY_SITE_PACKAGES = Path(
    os.getenv(
        "LEBAI_LEGACY_SITE_PACKAGES",
        str(PROJECT_ROOT / ".venv" / "Lib" / "site-packages"),
    )
)
BACKEND_ENTRY = PROJECT_ROOT / "09_module4_fastapi_backend.py"


def main() -> None:
    # Keep the current interpreter's packages first, then expose legacy SDKs.
    if LEGACY_SITE_PACKAGES.exists():
        legacy_path = str(LEGACY_SITE_PACKAGES)
        if legacy_path not in sys.path:
            sys.path.append(legacy_path)

    runpy.run_path(str(BACKEND_ENTRY), run_name="__main__")


if __name__ == "__main__":
    main()
