import importlib.util
from pathlib import Path
import unittest


_PATH = Path(__file__).parents[1] / "scripts" / "decompress_rgbd_node.py"
_SPEC = importlib.util.spec_from_file_location("decompress_rgbd_node", _PATH)
assert _SPEC is not None and _SPEC.loader is not None
_MODULE = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_MODULE)


class QualifiedPairGateTest(unittest.TestCase):
    def test_runner_exposes_dataset_specific_sync_envelope(self):
        runner = (Path(__file__).parents[1] / "scripts" / "run_orbslam_rgbd.sh").read_text()
        self.assertIn("DECOMP_SYNC_MAX_DELTA_MS", runner)
        self.assertIn("sync_max_delta_ms", runner)

    def test_pairing_slop_is_the_same_qualified_envelope(self):
        self.assertEqual(_MODULE.sync_slop_seconds(2.0), 0.002)

    def test_accepts_monotonic_hardware_like_pair(self):
        accepted, reason = _MODULE.qualified_pair_gate(
            1_000_000_000, 1_001_000_000, None, max_delta_ms=2.0
        )
        self.assertTrue(accepted)
        self.assertEqual(reason, "qualified")

    def test_rejects_approximate_pair_outside_hardware_envelope(self):
        accepted, reason = _MODULE.qualified_pair_gate(
            1_000_000_000, 1_033_000_000, None, max_delta_ms=2.0
        )
        self.assertFalse(accepted)
        self.assertEqual(reason, "stamp_delta")

    def test_rejects_nonincreasing_color_timestamp(self):
        accepted, reason = _MODULE.qualified_pair_gate(
            1_000_000_000, 1_000_500_000, 1_000_000_000, max_delta_ms=2.0
        )
        self.assertFalse(accepted)
        self.assertEqual(reason, "nonmonotonic")


if __name__ == "__main__":
    unittest.main()
