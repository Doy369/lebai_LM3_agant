"""Compatibility wrapper for the original calibration collection command."""

from apps.calibration.collect_data import main


if __name__ == "__main__":
    raise SystemExit(main())
