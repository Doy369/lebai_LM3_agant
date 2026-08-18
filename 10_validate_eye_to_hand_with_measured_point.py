"""Compatibility wrapper for the original calibration validation command."""

from apps.calibration.validate_eye_to_hand import main


if __name__ == "__main__":
    raise SystemExit(main())
