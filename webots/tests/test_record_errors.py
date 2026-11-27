"""Record errors at t=0 (tools/add_record_errors.py, lib/wms.py). No Webots needed.

    python -m unittest tests.test_record_errors -v        (from webots/)
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
sys.path.insert(0, str(ROOT / "tools"))

from add_record_errors import add_record_errors  # noqa: E402
from lite_sim import LiteSim  # noqa: E402
from nav import GridMap  # noqa: E402
from wes_runner import WES  # noqa: E402
from wms import MockWMS  # noqa: E402
import sched_verify  # noqa: E402

CFG = json.loads((ROOT / "scenario.json").read_text(encoding="utf-8"))
GRID, FIELDS = GridMap(CFG), {}
STOCKED = {st["slot"] for st in CFG["initial_stock"]}
KEYS = ("order_lead_all_s", "unit_lead_s", "order_lead_p90_s", "units_shipped", "units_released",
        "missed_cutoffs", "events", "record_quality")


def run_lite(cfg):
    tmp = tempfile.mkdtemp()
    try:
        with mock.patch.dict(os.environ, {"VERIFY_POLICY": "event"}):
            wes = WES(cfg, scheduler=sched_verify.Scheduler(cfg), log_dir=tmp)
        return LiteSim(cfg, wes, grid=GRID, fields=FIELDS).run()
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


class RecordErrorsTest(unittest.TestCase):
    def test_deterministic_and_level_zero_is_empty(self):
        a = add_record_errors(CFG, 0.1, "mixed", 7)
        self.assertEqual(a, add_record_errors(CFG, 0.1, "mixed", 7))
        self.assertNotEqual(a["initial_record_errors"], add_record_errors(CFG, 0.1, "mixed", 8)["initial_record_errors"])
        z = add_record_errors(CFG, 0.0, "mixed", 7)
        self.assertEqual(z["initial_record_errors"], [])
        self.assertEqual(z["initial_record_errors_meta"]["k"], 0)

    def test_counts_and_slot_conditions(self):
        n = len(STOCKED)
        for mix, level in (("ghost", 0.2), ("hidden", 0.2), ("swap", 0.2), ("mixed", 0.11)):
            out = add_record_errors(CFG, level, mix, 3)
            e = out["initial_record_errors"]
            k = round(level * n)
            self.assertEqual(out["initial_record_errors_meta"]["k"], k)
            self.assertEqual(len(e), k)
            self.assertEqual(len({x["slot"] for x in e}), k)            # no slot twice
            types = [x["type"] for x in e]
            want = {"ghost": {"ghost": k}, "hidden": {"hidden": k}, "swap": {"swap": k},
                    "mixed": {"ghost": k - k // 2, "hidden": k // 2}}[mix]
            self.assertEqual({t: types.count(t) for t in set(types)}, want)
            sku_at = {st["slot"]: st["sku"] for st in CFG["initial_stock"]}
            for x in e:
                if x["type"] == "ghost":
                    self.assertNotIn(x["slot"], STOCKED)
                    self.assertIsNotNone(x["record_sku"])
                else:
                    self.assertIn(x["slot"], STOCKED)
                if x["type"] == "swap":
                    self.assertNotEqual(x["record_sku"], sku_at[x["slot"]])
            self.assertEqual(out["initial_stock"], CFG["initial_stock"])   # physical side untouched

    def test_wrong_slots_look_like_right_ones(self):
        for mix in ("mixed", "swap"):
            out = add_record_errors(CFG, 0.2, mix, 5)
            rec = MockWMS(out).view.snapshot()["records"]
            fields = lambda sid: (rec[sid]["version"], rec[sid]["updated_t"], rec[sid]["observed_t"])
            wrong = {x["slot"]: x for x in out["initial_record_errors"]}
            normal_stocked = {fields(s) for s in STOCKED - set(wrong)}
            normal_empty = {fields(s["id"]) for s in CFG["slots"]
                            if s["kind"] == "rack" and s["id"] not in STOCKED | set(wrong)}
            for sid, x in wrong.items():
                self.assertIn(fields(sid), normal_empty if x["type"] == "hidden" else normal_stocked, x)
                self.assertEqual(rec[sid]["item"], x["record_sku"])

    def test_full_episode_with_many_errors(self):
        s = run_lite(add_record_errors(CFG, 0.20, "mixed", 11))
        self.assertGreater(s["units_shipped"], 0)
        self.assertNotIn("INVARIANT_VIOLATION", s["diagnostics"])
        self.assertGreater(s["events"].get("SHORT_PICK", 0) + s["events"].get("DESTINATION_OCCUPIED", 0), 0)

    def test_record_slips_put_the_record_next_to_the_carton(self):
        cfg = {**CFG, "record_slip_p": 1.0}
        tmp = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, tmp, ignore_errors=True)
        with mock.patch.dict(os.environ, {"VERIFY_POLICY": "event"}):
            wes = WES(cfg, scheduler=sched_verify.Scheduler(cfg), log_dir=tmp)
        sim = LiteSim(cfg, wes, grid=GRID, fields=FIELDS)
        s = sim.run(1200.0)
        slips = [x for x in wes.wms.record_slips if not x["skipped"]]
        self.assertGreater(len(slips), 0)
        self.assertEqual(s["record_slips"]["injected"], len(slips))
        slots = {x["id"]: x for x in CFG["slots"]}
        for x in slips:
            a, b = slots[x["actual"]], slots[x["recorded"]]
            self.assertEqual(b["kind"], "rack")
            self.assertNotEqual(x["actual"], x["recorded"])
        self.assertGreater(sum(slots[x["recorded"]]["face"] == slots[x["actual"]]["face"] for x in slips), 0)
        counts = s["wes_stats"]["counts"]
        self.assertFalse({k: v for k, v in counts.items() if k.startswith("anomaly_") or "reject" in k})
        self.assertGreater(s["record_quality"]["settled_mismatch_mean"], 0)

    def test_record_slips_off_by_default(self):
        s = run_lite(CFG)
        self.assertEqual(s["record_slips"], {"injected": 0, "skipped": 0})

    def test_level_zero_changes_nothing(self):
        before, after = run_lite(CFG), run_lite(add_record_errors(CFG, 0.0, "mixed", 1))
        self.assertEqual({k: before[k] for k in KEYS}, {k: after[k] for k in KEYS})


if __name__ == "__main__":
    unittest.main()
