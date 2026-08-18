from __future__ import annotations

import importlib
import os
import tempfile
import tomllib
import unittest
from pathlib import Path
from unittest.mock import patch

from robot_system.config import repository_root, resolve_from_workspace


PROJECT_ROOT = Path(__file__).resolve().parents[1]


class ProjectStructureTests(unittest.TestCase):
    def test_console_entry_points_are_importable(self) -> None:
        payload = tomllib.loads((PROJECT_ROOT / "pyproject.toml").read_text(encoding="utf-8"))
        scripts = payload["project"]["scripts"]
        self.assertGreaterEqual(len(scripts), 10)
        for command, target in scripts.items():
            module_name, attribute_name = target.split(":", maxsplit=1)
            module = importlib.import_module(module_name)
            self.assertTrue(callable(getattr(module, attribute_name)), command)

    def test_legacy_entry_points_remain_as_small_wrappers(self) -> None:
        wrappers = {
            "01_collect_data.py": "apps.calibration.collect_data",
            "02_calibrate.py": "apps.calibration.solve_eye_to_hand",
            "03_module2_qwen_demo.py": "apps.demos.qwen_planning",
            "04_module1_module2_pipeline_demo.py": "apps.demos.vision_language_pipeline",
            "05_module3_controller_demo.py": "apps.demos.controller_dry_run",
            "06_full_pick_pipeline_demo.py": "apps.demos.full_pick_pipeline",
            "07_real_robot_safety_probe_demo.py": "apps.robot.safety_probe",
            "08_two_stage_real_robot_test.py": "apps.robot.two_stage_pick",
            "09_module4_fastapi_backend.py": "apps.web.serve",
            "10_validate_eye_to_hand_with_measured_point.py": "apps.calibration.validate_eye_to_hand",
            "11_auto_calibration.py": "apps.calibration.auto_calibrate",
        }
        for filename, target in wrappers.items():
            content = (PROJECT_ROOT / filename).read_text(encoding="utf-8")
            self.assertIn(target, content, filename)
            self.assertLess(len(content.splitlines()), 20, filename)

        compatibility_launcher = (PROJECT_ROOT / "start_backend_compat.py").read_text(
            encoding="utf-8"
        )
        self.assertNotIn("09_module4_fastapi_backend.py", compatibility_launcher)
        self.assertNotIn("runpy", compatibility_launcher)
        real_robot_launcher = (PROJECT_ROOT / "start_backend_real_robot.ps1").read_text(
            encoding="utf-8"
        )
        self.assertIn("-m apps.web.serve", real_robot_launcher)

        for source_file in (PROJECT_ROOT / "robot_system").rglob("*.py"):
            self.assertNotIn("from apps", source_file.read_text(encoding="utf-8"), source_file)

    def test_repository_and_workspace_paths_are_centralized(self) -> None:
        self.assertEqual(repository_root(), PROJECT_ROOT)
        with tempfile.TemporaryDirectory() as temp_dir:
            with patch.dict(os.environ, {"LEBAI_WORKSPACE_ROOT": temp_dir}):
                expected = (Path(temp_dir) / "runtime" / "state.json").resolve()
                self.assertEqual(resolve_from_workspace("runtime/state.json"), expected)


if __name__ == "__main__":
    unittest.main()
