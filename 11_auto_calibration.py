"""Compatibility wrapper for the original automatic calibration command."""

from apps.calibration.auto_calibrate import main


if __name__ == "__main__":
    raise SystemExit(main())
