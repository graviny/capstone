"""Tests for the lightweight simulator lib/lite_sim.py (no Webots needed).

    python -m unittest discover -s tests -v        (from webots/)
"""
import json
import os
import shutil
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "lib"))

from lite_sim import LiteSim  # noqa: E402
from nav import GridMap  # noqa: E402
from wes_runner import WES  # noqa: E402
import sched_learned  # noqa: E402
import sched_verify  # noqa: E402

CFG = json.loads((ROOT / "scenario.json").read_text(encoding="utf-8"))
GRID, FIELDS = GridMap(CFG), {}


class LiteSimTest(unittest.TestCase):
    def run_once(self, seed=0, duration=900.0, module=sched_verify, robots=None):
        cfg = {**CFG, "seed": seed, "robots": CFG["robots"][:robots]}
        tmp = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, tmp, ignore_errors=True)
        with mock.patch.dict(os.environ, {"VERIFY_POLICY": "event", "ASSIGN_WEIGHTS": str(Path(tmp) / "none.json")}):
            wes = WES(cfg, scheduler=module.Scheduler(cfg), log_dir=tmp)
        return LiteSim(cfg, wes, grid=GRID, fields=FIELDS).run(duration)   # run() closes the WMS log

    def test_right_of_way(self):
        def sim(mode):
            cfg = {**CFG, "nav": {**CFG["nav"], "traffic_priority": mode}}
            return LiteSim(cfg, wes=None, grid=GRID, fields=FIELDS)
        pick = {"task_id": "PK", "priority": 1, "deadline": 900.0}
        putaway = {"task_id": "PA", "priority": 2, "deadline": None}
        s = sim("task")
        r = s.robots
        r[0].update(task=None)                                   # idle: last
        r[1].update(task=dict(putaway), phase="go_src")          # empty, low priority
        r[2].update(task=dict(pick), phase="go_src")             # empty, urgent
        r[3].update(task=dict(putaway), phase="go_dst", holding="LPN")   # loaded: first
        order = sorted(range(4), key=lambda k: s.right_of_way(r[k]))
        self.assertEqual(order, [3, 2, 1, 0])
        s = sim("id")
        for k in range(4):
            s.robots[k].update(r[k])
        self.assertEqual(sorted(range(4), key=lambda k: s.right_of_way(s.robots[k])), [0, 1, 2, 3])

    def test_learned_scheduler_keeps_the_wms_consistent(self):
        # Heuristic weights (no weight file), few robots so busy robots win tasks too.
        s = self.run_once(module=sched_learned, robots=3)
        self.assertGreater(s["units_shipped"], 0)
        counts = s["wes_stats"]["counts"]
        self.assertFalse({k: v for k, v in counts.items() if k.startswith("anomaly_") or "reject" in k})

    def test_same_seed_same_result(self):
        a, b = self.run_once(), self.run_once()
        self.assertEqual(json.dumps(a, sort_keys=True), json.dumps(b, sort_keys=True))

    def test_runs_the_warehouse(self):
        s = self.run_once()
        self.assertGreater(s["units_shipped"], 0)
        self.assertGreater(s["events"]["TASK_COMPLETED"], 0)
        self.assertEqual(s["mis_shipped"], 0)
        counts = s["wes_stats"]["counts"]
        # The WMS accepted every decision and saw nothing inconsistent.
        self.assertFalse({k: v for k, v in counts.items() if k.startswith("anomaly_") or "reject" in k})
        self.assertTrue(all(0.0 <= b <= 1.0 for b in s["robot_busy_share"].values()))


if __name__ == "__main__":
    unittest.main()
