"""Command-line adapter for the Eye-to-Hand calibration solver."""

from robot_system.calibration.eye_to_hand_solver import main, main_v2

__all__ = ["main", "main_v2"]


if __name__ == "__main__":
    raise SystemExit(main())
