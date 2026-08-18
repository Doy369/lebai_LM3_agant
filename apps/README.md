# Application entry points

This directory contains user-facing commands only. Reusable perception, planning,
calibration, control, and Web behavior belongs in `robot_system/`.

- `calibration/`: data collection, solver adapters, validation, and automatic calibration;
- `demos/`: offline or Dry-run demonstrations;
- `robot/`: explicitly safety-gated robot workflows;
- `web/`: FastAPI service startup.

The numbered scripts in the repository root are temporary compatibility wrappers.
New integrations should use the commands declared in `pyproject.toml`.
