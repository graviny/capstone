import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "lib"))
from record_quality import RecordQuality


class RecordQualityTest(unittest.TestCase):
    def test_busy_does_not_count_as_correction(self):
        q = RecordQuality()
        q.sample(0, set(), set())
        q.sample(10, {"A"}, set())
        q.sample(20, {"A"}, {"A"})
        self.assertEqual(q.summary(20)["corrected"], 0)
        self.assertEqual(q.summary(20)["open_at_end"], 1)
        q.sample(30, set(), {"A"})
        self.assertEqual(q.summary(40)["mean_time_to_correct_s"], 20)
        self.assertEqual(q.summary(40)["settled_mismatch_mean"], 0.25)

    def test_final_sample_keeps_unresolved_episode(self):
        q = RecordQuality()
        q.sample(10, {"A"}, set())
        q.sample(15, {"A"}, set())
        result = q.summary(15)
        self.assertEqual(result["open_at_end"], 1)
        self.assertEqual(result["mean_open_age_s"], 5)
        self.assertIsNone(result["mean_time_to_correct_s"])
