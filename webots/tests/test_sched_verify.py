"""Unit tests for the experiment scheduler lib/sched_verify.py (no Webots needed).

    python -m unittest discover -s tests -v        (from webots/)
"""
import os
import shutil
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "lib"))
sys.path.insert(0, str(ROOT / "tests"))

from wes_runner import WES  # noqa: E402
import sched_verify  # noqa: E402
from test_wms import Feed, robots_report, small_cfg  # noqa: E402


class Run:
    """A runner driven like the supervisor drives it."""

    def __init__(self, cfg, tmp, **env):
        with mock.patch.dict(os.environ, {k: str(v) for k, v in env.items()}):
            self.sched = sched_verify.Scheduler(cfg)
        self.wes = WES(cfg, scheduler=self.sched, log_dir=tmp)
        self.ev, self.sent = Feed(), []

    def emit(self, kind, t, **kw):
        self.wes.on_event(self.ev(kind, t=t, **kw))

    def step(self, t, **rep):
        out = self.wes.decide(t, robots_report(self.wes.wms, **rep))
        self.sent += out
        return out

    def verifies(self):
        return {tid: t for tid, t in self.wes.wms.tasks.items() if t["task_type"] == "VERIFY"}


def order_a(run):
    run.emit("ORDER_RELEASED", 2.0, wave="W1", lines=[
        {"order": "O1", "customer": "C1", "region": "R1", "sku": "A", "qty": 1}])
    run.step(2.0)
    run.emit("SHORT_PICK", 10.0, robot_id=1, task_id="PK-O1-A", slot="r0s00", item_id="A", observed="EMPTY")


class SchedVerifyTest(unittest.TestCase):
    def new_run(self, cfg, **env):
        tmp = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, tmp, ignore_errors=True)
        run = Run(cfg, tmp, **env)
        self.addCleanup(run.wes.stats)          # closes the WMS log (runs before the rmtree)
        return run

    def test_none_creates_nothing(self):
        run = self.new_run(small_cfg(), VERIFY_POLICY="none")
        self.assertEqual(run.wes.name, "wms-verify-none")
        order_a(run)
        for t in (11.0, 12.0, 13.0):
            run.step(t, r1={"task": "PK-O1-A"})
        self.assertEqual(run.verifies(), {})

    def test_event_checks_neighbors_with_the_patrol(self):
        run = self.new_run(small_cfg(), VERIFY_POLICY="event", VERIFY_NEIGHBORS=2)
        order_a(run)
        run.step(11.0, r1={"task": "PK-O1-A"})
        vf = run.verifies()
        # Closest neighbors first; r0s00 is at the end of the face, so only one side.
        self.assertEqual([t["destination"] for t in vf.values()], ["r0s01", "r0s02"])
        run.step(12.0, r1={"task": "PK-O1-A"})
        assigned = [d for d in run.sent if d["action"] == "EXECUTE" and d["task_id"] in vf]
        self.assertEqual([(d["robot_id"], d["task"]["destination"]) for d in assigned], [(0, "r0s01")])
        # The patrol reports: the record of r0s01 is corrected, the task is done.
        run.emit("OBSERVATION_RESULT", 20.0, robot_id=0, task_id=assigned[0]["task_id"], slot="r0s01",
                 observed="EMPTY", source="verify")
        run.emit("TASK_COMPLETED", 20.0, robot_id=0, task_id=assigned[0]["task_id"], task_type="VERIFY",
                 slot="r0s01", item_id="EMPTY")
        self.assertIsNone(run.wes.wms.rec["r0s01"]["item"])
        self.assertEqual(run.verifies()[assigned[0]["task_id"]]["status"], "COMPLETED")

    def test_without_a_patrol_a_transport_checks(self):
        cfg = small_cfg()
        cfg["robots"] = [r for r in cfg["robots"] if r["role"] != "patrol"]
        run = self.new_run(cfg, VERIFY_POLICY="event", VERIFY_NEIGHBORS=1)
        order_a(run)
        run.step(11.0, r1={"task": "PK-O1-A"})
        run.step(12.0, r1={"task": "PK-O1-A"})
        vf = run.verifies()
        assigned = [d for d in run.sent if d["action"] == "EXECUTE" and d["task_id"] in vf]
        self.assertEqual([d["robot_id"] for d in assigned], [2])

    def test_destination_occupied_checks_that_slot(self):
        run = self.new_run(small_cfg(), VERIFY_POLICY="event")
        run.emit("TRUCK_ARRIVED", 1.0, truck="T1", asn=[{"lpn": "IN1", "item": "C"}])
        run.emit("INBOUND_STAGED", 2.0, slot="B0", lpn="IN1", item="C")
        run.step(2.0)
        run.emit("DESTINATION_OCCUPIED", 11.0, robot_id=2, task_id="PA-IN1", slot="r0s02", item_id="C",
                 observed="UNKNOWN")
        run.step(11.0, r2={"task": "PA-IN1", "holding": "C"})
        run.step(12.0, r2={"task": "PA-IN1", "holding": "C"})
        self.assertIn("r0s02", [t["destination"] for t in run.verifies().values()])

    def test_periodic_sweeps_and_caps_open_tasks(self):
        run = self.new_run(small_cfg(), VERIFY_POLICY="periodic", VERIFY_PERIOD_S=10, VERIFY_MAX_OPEN=2)
        r0 = {"task": "busy"}            # keep the patrol busy so the checks stay open
        for t in range(0, 61, 5):
            run.step(float(t), r0=r0)
        vf = run.verifies()
        self.assertEqual([t["destination"] for t in vf.values()], ["r0s00", "r0s01"])


if __name__ == "__main__":
    unittest.main()
