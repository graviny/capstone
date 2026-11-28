"""The workload generator must not silently discard busy-day waves at the horizon."""
import json
import random
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
from gen_world import audit_workload, build_orders, plan_stock  # noqa: E402


class OrderReleaseTest(unittest.TestCase):
    def test_busy_day_all_waves_fit_without_artificial_gap(self):
        skus = json.loads((ROOT / "data" / "skus.json").read_text(encoding="utf-8"))["skus"]
        base = dict(date="2023-03-22", order_sample=1.0, max_line_qty=6,
                    time_scale=0.0, release_frac=0.6, duration=3600.0, warmup=30.0)
        waves, _, meta = build_orders(SimpleNamespace(**base, wave_interval=0.0), skus)
        self.assertGreater(len(waves), 40)
        self.assertLessEqual(waves[-1]["release_t"], 3600.0)
        self.assertEqual(meta["wave_interval_s"], 0.0)
        self.assertEqual(meta["order_timing_version"], "compressed-source-v2")

        legacy, _, legacy_meta = build_orders(SimpleNamespace(**base, wave_interval=90.0), skus)
        self.assertGreater(legacy[-1]["release_t"], 3600.0)
        self.assertEqual(sum(w["release_t"] <= 3600 for w in legacy), 40)
        self.assertEqual(legacy_meta["order_timing_version"], "min-gap-legacy-v1")

    def test_busy_day_supply_shortfall_is_reported(self):
        skus = json.loads((ROOT / "data" / "skus.json").read_text(encoding="utf-8"))["skus"]
        scenario = json.loads((ROOT / "scenario.json").read_text(encoding="utf-8"))
        racks = [slot for slot in scenario["slots"] if slot["kind"] == "rack"]
        args = SimpleNamespace(date="2023-03-22", order_sample=1.0, max_line_qty=6,
                               time_scale=0.0, release_frac=0.6, duration=3600.0,
                               warmup=30.0, wave_interval=0.0, initial_cover=0.5,
                               initial_fill=0.6, stock_layout="abc", inbound_lead=240.0,
                               safety_stock=0.15, inbound_scale=1.0,
                               truck_cartons=12, truck_first=60.0, truck_min_gap=120.0)
        waves, _, _ = build_orders(args, skus)
        stock, trucks, _ = plan_stock(args, skus, waves, racks, random.Random(49))
        audit = audit_workload(waves, stock, trucks, args.duration)
        self.assertEqual(audit["released_units"], sum(ln["qty"] for w in waves for ln in w["lines"]))
        self.assertEqual(audit["waves_after_horizon"], 0)
        self.assertEqual(audit["unservable_units_lower_bound"],
                         audit["released_units"] - audit["supply_upper_bound_units"])
        self.assertGreater(audit["unservable_units_lower_bound"], 0)

    def test_target_keeps_whole_orders_and_all_skus(self):
        skus = json.loads((ROOT / "data" / "skus.json").read_text(encoding="utf-8"))["skus"]
        source = json.loads((ROOT / "data" / "orders_2023-03-22.json").read_text(encoding="utf-8"))
        self.assertIsNone(json.loads((ROOT / "data" / "skus.json").read_text(encoding="utf-8"))["top_sku"])
        args = SimpleNamespace(date="2023-03-22", order_sample=1.0, target_units=100,
                               max_line_qty=0, time_scale=0.0, release_frac=0.6,
                               duration=3600.0, warmup=30.0, wave_interval=0.0)
        waves, _, meta = build_orders(args, skus)
        selected = {ln["order"] for w in waves for ln in w["lines"]}
        original = [ln for w in source["waves"] for ln in w["lines"] if ln["qty"] > 0]
        actual = [ln for w in waves for ln in w["lines"]]
        self.assertEqual(actual, [ln for ln in original if ln["order"] in selected])
        self.assertLessEqual(meta["units"], 100)
        self.assertEqual(meta["units_capped"], 0)
        self.assertEqual(meta["order_selection"], "whole-order-hash-cap-v1")


if __name__ == "__main__":
    unittest.main()
