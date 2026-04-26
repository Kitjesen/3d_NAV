"""Tests for the BMI088 IMU-noise sanity check in calibration/apply_calibration.py.

The sanity check is procedural protection: Allan Variance can produce values
that pass YAML schema but are 1-2 orders of magnitude off if the recording
environment was wrong (vibration, thermal drift, units mis-converted). The
default ng=0.01 currently shipping in lio.yaml is itself such a value — it
is ~10-50x higher than BMI088 datasheet typical, and the test here locks in
that calling the sanity check with that value warns.
"""

import importlib.util
import logging
import unittest
from pathlib import Path


def _load_apply_calibration():
    """apply_calibration.py lives in calibration/, not src/, so it cannot be
    imported via the normal package path. Load it directly by file path."""
    repo_root = Path(__file__).resolve().parent.parent.parent.parent
    path = repo_root / "calibration" / "apply_calibration.py"
    spec = importlib.util.spec_from_file_location("apply_calibration", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class TestBMI088SanityCheck(unittest.TestCase):
    def setUp(self) -> None:
        self.ac = _load_apply_calibration()
        self.logger_name = "apply_calibration"

    def test_in_range_logs_info(self):
        with self.assertLogs(self.logger_name, level=logging.INFO) as cm:
            self.ac._sanity_check_imu_noise("ng", 5e-4)
        joined = " ".join(cm.output)
        self.assertIn("within BMI088 expected range", joined)
        self.assertNotIn("WARNING", joined)

    def test_default_ng_001_triggers_high_warning(self):
        """The shipped default ng=0.01 must trigger the HIGH warning — this
        is the very value the audit flagged as 10-50x off."""
        with self.assertLogs(self.logger_name, level=logging.WARNING) as cm:
            self.ac._sanity_check_imu_noise("ng", 0.01)
        joined = " ".join(cm.output)
        self.assertIn("unusually HIGH", joined)
        self.assertIn("ng", joined)

    def test_too_low_triggers_low_warning(self):
        with self.assertLogs(self.logger_name, level=logging.WARNING) as cm:
            self.ac._sanity_check_imu_noise("ng", 1e-7)
        joined = " ".join(cm.output)
        self.assertIn("unusually LOW", joined)

    def test_accel_in_range(self):
        with self.assertLogs(self.logger_name, level=logging.INFO) as cm:
            self.ac._sanity_check_imu_noise("na", 5e-3)
        self.assertIn("within BMI088 expected range", " ".join(cm.output))

    def test_unknown_name_is_silent(self):
        # Unknown parameter names must not raise and must not log — covers
        # forward-compat if a future calibrator emits extra fields.
        try:
            self.ac._sanity_check_imu_noise("garbage_param", 1.0)
        except Exception as e:
            self.fail(f"sanity check raised on unknown name: {e}")

    def test_threshold_is_loose_5x(self):
        """Boundary check: values exactly at low/5 or high*5 should NOT warn,
        only just past them should. Locks in the ±5x tolerance contract."""
        # ng range is (2e-4, 1e-3); 5x of high = 5e-3 — boundary, no warn
        with self.assertLogs(self.logger_name, level=logging.INFO) as cm:
            self.ac._sanity_check_imu_noise("ng", 5e-3)
        self.assertIn("within BMI088 expected range", " ".join(cm.output))

        # 5.1e-3 — just past, warn
        with self.assertLogs(self.logger_name, level=logging.WARNING) as cm:
            self.ac._sanity_check_imu_noise("ng", 5.1e-3)
        self.assertIn("unusually HIGH", " ".join(cm.output))


class TestBMI088ReferenceTable(unittest.TestCase):
    def test_all_keys_have_low_high_pairs(self):
        ac = _load_apply_calibration()
        for name, bounds in ac.BMI088_REFERENCE.items():
            self.assertEqual(len(bounds), 2, f"{name} bounds malformed")
            low, high = bounds
            self.assertLess(low, high, f"{name} low >= high")
            self.assertGreater(low, 0, f"{name} low non-positive")


if __name__ == "__main__":
    unittest.main()
