import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "lib"))
from place_fail import attempt, enabled, landing  # noqa: E402


class PlaceFailTests(unittest.TestCase):
    def test_success_rates_follow_kind(self):
        f = {"place_p": 0.95, "retry_q": 0.5, "takeover_r": 0.8, "place_fail_alpha": 0.0}
        ok = lambda kind: sum(attempt(1, f"L{i}", 1, kind, f, 2.0)[0] for i in range(4000)) / 4000  # noqa: E731
        self.assertAlmostEqual(ok("first"), 0.95, delta=0.02)
        self.assertAlmostEqual(ok("retry"), 0.5, delta=0.03)
        self.assertAlmostEqual(ok("takeover"), 0.8, delta=0.03)

    def test_failed_attempt_time_is_clipped_and_centered(self):
        f = {"place_p": 0.0, "retry_q": 0.0, "takeover_r": 0.0, "place_fail_alpha": 0.0}
        ts = [attempt(1, f"L{i}", 1, "first", f, 2.0)[1] for i in range(4000)]
        self.assertTrue(all(0.1 <= t <= 2.0 for t in ts))
        self.assertAlmostEqual(sum(ts) / len(ts), 1.0, delta=0.05)

    def test_skew_moves_failure_time_later(self):
        f = {"place_p": 0.0, "retry_q": 0.0, "takeover_r": 0.0, "place_fail_alpha": 4.0}
        ts = [attempt(1, f"L{i}", 1, "first", f, 2.0)[1] for i in range(4000)]
        self.assertGreater(sum(ts) / len(ts), 1.2)

    def test_success_takes_act_s_and_draw_is_deterministic(self):
        f = {"place_p": 1.0, "retry_q": 1.0, "takeover_r": 1.0, "place_fail_alpha": 0.0}
        self.assertEqual(attempt(3, "L1", 2, "first", f, 2.0)[:2], (True, 2.0))
        g = dict(f, place_p=0.5)
        self.assertEqual(attempt(3, "L9", 4, "first", g, 2.0), attempt(3, "L9", 4, "first", g, 2.0))

    def test_landing_picks_a_free_neighbor_or_none(self):
        occ = {"r0s01": None, "r0s02": "X", "r0s03": None}
        self.assertIn(landing("r0s02", occ, 0.3), ("r0s01", "r0s03"))
        self.assertNotEqual(landing("r0s02", occ, 0.3), landing("r0s02", occ, 0.7))
        self.assertEqual(landing("r0s02", {"r0s01": "A", "r0s02": "X", "r0s03": None}, 0.3), "r0s03")
        self.assertIsNone(landing("r0s02", {"r0s01": "A", "r0s02": "X", "r0s03": "B"}, 0.3))
        self.assertIsNone(landing("r0s00", {"r0s00": "X", "r0s01": "A"}, 0.3))   # end of face: no r0s-1

    def test_enabled_only_with_place_p(self):
        self.assertFalse(enabled({"place_error_p": 0.1}))
        self.assertTrue(enabled({"place_p": 1.0}))


if __name__ == "__main__":
    unittest.main()
