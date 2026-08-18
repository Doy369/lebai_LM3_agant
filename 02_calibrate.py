"""Compatibility wrapper for the original Eye-to-Hand solver command."""

from apps.calibration.solve_eye_to_hand import main, main_v2

__all__ = ["main", "main_v2"]


if __name__ == "__main__":
    raise SystemExit(main())
