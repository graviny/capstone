"""Unit tests for the Mock WMS and the WES runner (no Webots needed).

    python -m unittest discover -s tests -v        (from webots/)
"""
import json
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "lib"))
sys.path.insert(0, str(ROOT / "tools"))

from wms import MockWMS, UNKNOWN  # noqa: E402
from wes_runner import WES  # noqa: E402
from sched_standin import Scheduler  # noqa: E402
import replay_wms  # noqa: E402


def small_cfg():
    """One rack face with 6 slots, 2 buffer zones, 1 exception slot, a 2-slot lane."""
    slots = [{"id": f"r0s{k:02d}", "kind": "rack", "face": "r0s", "pos": [float(k), 0.0], "approach": [float(k), -0.5]}
             for k in range(6)]
    slots += [{"id": "B0", "kind": "buffer", "pos": [-3.0, 2.0], "approach": [-3.0, 1.5]},
              {"id": "B1", "kind": "buffer", "pos": [-2.0, 2.0], "approach": [-2.0, 1.5]},
              {"id": "X0", "kind": "exception", "pos": [-3.0, 3.0], "approach": [-3.0, 2.5]},
              {"id": "LR1-0", "kind": "outbound", "lane": "R1", "pos": [9.0, 1.0], "approach": [8.5, 1.0]},
              {"id": "LR1-1", "kind": "outbound", "lane": "R1", "pos": [9.0, 1.4], "approach": [8.5, 1.4]}]
    return {
        "seed": 0, "slots": slots,
        "skus": [{"sku": "A", "abc": "A"}, {"sku": "B", "abc": "B"}, {"sku": "C", "abc": "C"}],
        "initial_stock": [{"slot": "r0s00", "sku": "A", "lpn": "ST1"}, {"slot": "r0s01", "sku": "B", "lpn": "ST2"}],
        "robots": [{"id": 0, "role": "patrol", "home": [0.0, 5.0]},
                   {"id": 1, "role": "transport", "home": [1.0, 5.0]},
                   {"id": 2, "role": "transport", "home": [8.0, 5.0]}],
        "outbound": {"date": "test", "lanes": {"R1": {"slots": ["LR1-0", "LR1-1"]}}, "cutoff_period_s": 900.0,
                     "center": [9.0, 1.0]},
        "faults": {"place_error_p": 0.0}, "patrol": {"idle_sweep": False},
    }


class Feed:
    """Events with ids and times, like the supervisor's."""

    def __init__(self):
        self.n = 0

    def __call__(self, kind, t=0.0, **kw):
        self.n += 1
        return {"event_id": f"E{self.n:06d}", "t": t, "type": kind, **kw}


def robots_report(wms, **override):
    """The robots list the supervisor passes to decide(); override: {id: {...}}."""
    out = []
    for r in wms.robots.values():
        rep = {"id": r["id"], "role": r["role"], "pos": r["pos"], "task": None, "holding": None, "phase": None}
        rep.update(override.get(f"r{r['id']}", {}))
        out.append(rep)
    return out


class WMSTest(unittest.TestCase):
    def setUp(self):
        self.w = MockWMS(small_cfg())
        self.ev = Feed()

    def execute(self, tid, robot, source=None, destination=None):
        d = {"robot_id": robot, "task_id": tid, "action": "EXECUTE"}
        d.update({k: v for k, v in (("source", source), ("destination", destination)) if v is not None})
        return self.w.apply_decision(d)

    def stage(self, slot, lpn, item, t=1.0):
        self.w.apply_event(self.ev("TRUCK_ARRIVED", t=t, truck="T1", asn=[{"lpn": lpn, "item": item}]))
        self.w.apply_event(self.ev("INBOUND_STAGED", t=t, slot=slot, lpn=lpn, item=item))

    # ---- records and inbound ----
    def test_initial_stock(self):
        self.assertEqual(self.w.rec["r0s00"]["item"], "A")
        self.assertEqual(self.w.view.stock("A"), 1)
        snap = self.w.record_snapshot()
        self.assertEqual(len(snap), 6)                    # rack slots only
        self.assertIsNone(snap["r0s02"])

    def test_staged_carton_gets_putaway(self):
        self.stage("B0", "IN1", "C")
        task = self.w.tasks["PA-IN1"]
        self.assertEqual((task["task_type"], task["status"], task["source"], task["lpn"]),
                         ("PUTAWAY", "PENDING", "B0", "IN1"))
        self.assertEqual(self.w.asn["IN1"]["status"], "STAGED")
        self.assertEqual(self.w.rec["B0"]["item"], "C")
        # The WMS stores it: class C goes to the free rack slot farthest from the outbound side.
        self.assertEqual(task["destination"], "r0s02")
        self.assertEqual(self.w.rec["r0s02"]["reserved_by"], "PA-IN1")

    def test_unread_carton_verify_then_putaway(self):
        self.w.apply_event(self.ev("INBOUND_READ_FAIL", t=2.0, slot="X0"))
        self.assertEqual(self.w.rec["X0"]["item"], UNKNOWN)
        self.assertEqual(self.w.tasks["VF-X0"]["status"], "PENDING")
        self.assertIsNotNone(self.execute("VF-X0", 0, destination="X0"))
        self.w.apply_event(self.ev("OBSERVATION_RESULT", t=9.0, robot_id=0, task_id="VF-X0", slot="X0",
                                   observed="C", source="verify"))
        self.assertNotIn("PA-X0", self.w.tasks)         # not before the VERIFY is done
        self.w.apply_event(self.ev("TASK_COMPLETED", t=9.0, robot_id=0, task_id="VF-X0", task_type="VERIFY",
                                   slot="X0", item_id="C"))
        self.assertEqual(self.w.tasks["VF-X0"]["status"], "COMPLETED")
        self.assertEqual(self.w.tasks["PA-X0"]["item_id"], "C")
        self.assertEqual(self.w.rec["X0"]["item"], "C")
        self.assertEqual(self.w.exceptions, [])          # identifying is not a discrepancy

    def test_unreadable_verify_is_retried(self):
        self.w.apply_event(self.ev("INBOUND_READ_FAIL", t=2.0, slot="X0"))
        self.execute("VF-X0", 0, destination="X0")
        self.w.apply_event(self.ev("OBSERVATION_RESULT", t=9.0, robot_id=0, task_id="VF-X0", slot="X0",
                                   observed="UNREADABLE", source="verify"))
        self.w.apply_event(self.ev("TASK_COMPLETED", t=9.0, robot_id=0, task_id="VF-X0", task_type="VERIFY",
                                   slot="X0", item_id="UNREADABLE"))
        self.assertEqual(self.w.tasks["VF-X0-2"]["status"], "PENDING")

    # ---- orders and picking ----
    def test_order_release_creates_picks_with_deadline(self):
        self.w.apply_event(self.ev("ORDER_RELEASED", t=100.0, wave="W1", lines=[
            {"order": "O1", "customer": "C1", "region": "R1", "sku": "A", "qty": 2}]))
        ids = [t for t in self.w.tasks if t.startswith("PK-")]
        self.assertEqual(ids, ["PK-O1-A", "PK-O1-A-2"])
        self.assertEqual(self.w.tasks["PK-O1-A"]["deadline"], 900.0)
        self.assertEqual(self.w.orders["O1"]["units"], 2)

    def pick_flow(self):
        self.w.apply_event(self.ev("ORDER_RELEASED", t=10.0, wave="W1", lines=[
            {"order": "O1", "customer": "C1", "region": "R1", "sku": "A", "qty": 1}]))
        sent = self.execute("PK-O1-A", 2, source="r0s00", destination="LR1-0")
        self.assertEqual(sent["task"]["order"], "O1")     # the robot needs the order for the lane tag
        self.assertEqual(sent["task"]["lpn"], "ST1")
        return sent

    def test_execute_reserves_and_robot_reports(self):
        self.pick_flow()
        task = self.w.tasks["PK-O1-A"]
        self.assertEqual(task["status"], "ASSIGNED")
        self.assertEqual(self.w.rec["r0s00"]["reserved_by"], "PK-O1-A")
        self.assertEqual(self.w.view.idle_robots("transport")[0]["id"], 1)
        self.w.update_robots(11.0, robots_report(self.w, r2={"task": "PK-O1-A", "phase": "go_src"}))
        self.assertEqual(task["status"], "IN_PROGRESS")
        self.w.update_robots(20.0, robots_report(self.w, r2={"task": "PK-O1-A", "holding": "A"}))
        self.assertTrue(task["picked"])
        self.assertEqual(self.w.rec["r0s00"]["reserved_by"], "PK-O1-A")   # until completed

    def test_pick_completion_and_shipping(self):
        self.pick_flow()
        self.w.apply_event(self.ev("TASK_COMPLETED", t=30.0, robot_id=2, task_id="PK-O1-A", task_type="PICK",
                                   item_id="A", slot="LR1-0"))
        self.assertIsNone(self.w.rec["r0s00"]["item"])
        self.assertEqual(self.w.rec["LR1-0"]["item"], "A")
        self.assertIsNone(self.w.rec["r0s00"]["reserved_by"])
        self.assertIsNone(self.w.robots[2]["assigned"])
        self.assertEqual(self.w.orders["O1"]["picked"], 1)
        self.w.apply_event(self.ev("OUTBOUND_DEPARTED", t=40.0, region="R1", cartons=[
            {"slot": "LR1-0", "lpn": "ST1", "item": "A", "order": "O1"}]))
        self.assertIsNone(self.w.rec["LR1-0"]["item"])
        self.assertEqual(self.w.orders["O1"]["status"], "SHIPPED")
        self.assertFalse(self.w.lane_truck["R1"])

    def test_truck_leaves_before_the_completion_report(self):
        """The robot places the carton, the truck leaves at once, then the robot's report arrives."""
        self.pick_flow()
        self.w.apply_event(self.ev("OUTBOUND_DEPARTED", t=30.0, region="R1", cartons=[
            {"slot": "LR1-0", "lpn": "ST1", "item": "A", "order": "O1"}]))
        self.w.apply_event(self.ev("TASK_COMPLETED", t=30.1, robot_id=2, task_id="PK-O1-A", task_type="PICK",
                                   item_id="A", slot="LR1-0"))
        self.assertIsNone(self.w.rec["LR1-0"]["item"])                      # already gone
        self.assertIsNone(self.w.rec["r0s00"]["item"])
        self.assertEqual(self.w.orders["O1"]["status"], "SHIPPED")
        self.assertNotIn("anomaly_SHIPPED_NOT_AS_RECORDED", self.w.counts)

    def test_duplicate_and_late_completion_change_nothing(self):
        self.pick_flow()
        ev = self.ev("TASK_COMPLETED", t=30.0, robot_id=2, task_id="PK-O1-A", task_type="PICK", item_id="A",
                     slot="LR1-0")
        self.assertTrue(self.w.apply_event(ev))
        version = self.w.rec["LR1-0"]["version"]
        self.assertFalse(self.w.apply_event(ev))                        # same event id
        self.w.apply_event({**ev, "event_id": "E999999"})                  # same report again
        self.assertEqual(self.w.rec["LR1-0"]["version"], version)
        self.assertEqual(self.w.orders["O1"]["picked"], 1)
        self.assertEqual(self.w.counts["anomaly_DUPLICATE_EVENT"], 1)
        self.assertEqual(self.w.counts["anomaly_LATE_COMPLETION"], 1)

    # ---- exceptions ----
    def test_short_pick_reroute_and_requeue(self):
        self.w.rec["r0s03"]["item"] = "A"                                  # a second recorded A
        self.pick_flow()
        self.w.apply_event(self.ev("SHORT_PICK", t=20.0, robot_id=2, task_id="PK-O1-A", slot="r0s00",
                                   item_id="A", observed="EMPTY"))
        task = self.w.tasks["PK-O1-A"]
        self.assertEqual((task["status"], task["hold_reason"]), ("ON_HOLD", "SHORT_PICK"))
        self.assertIsNone(self.w.rec["r0s00"]["item"])
        self.assertIsNone(self.w.rec["r0s00"]["reserved_by"])
        self.assertEqual(self.w.exceptions[-1]["type"], "SHORT_PICK")
        sent = self.w.apply_decision({"robot_id": 2, "task_id": "PK-O1-A", "action": "REROUTE",
                                      "source": "r0s03", "destination": "LR1-0"})
        self.assertEqual((sent["source"], sent["destination"]), ("r0s03", "LR1-0"))
        self.assertEqual(task["status"], "IN_PROGRESS")
        self.assertEqual(self.w.rec["r0s03"]["reserved_by"], "PK-O1-A")
        self.w.apply_event(self.ev("SHORT_PICK", t=30.0, robot_id=2, task_id="PK-O1-A", slot="r0s03",
                                   item_id="A", observed="EMPTY"))
        self.w.apply_decision({"robot_id": 2, "task_id": "PK-O1-A", "action": "REASSIGN"})
        self.assertEqual((task["status"], task["source"], task["destination"]), ("PENDING", None, None))
        self.assertIsNone(self.w.rec["LR1-0"]["reserved_by"])
        self.assertIsNone(self.w.robots[2]["assigned"])

    def test_destination_occupied(self):
        self.stage("B0", "IN1", "C")                                       # WMS slot: r0s02
        self.assertIsNotNone(self.execute("PA-IN1", 1))
        reroute = {"robot_id": 1, "task_id": "PA-IN1", "action": "REROUTE", "source": "B0", "destination": "r0s05"}
        self.assertIsNone(self.w.apply_decision(dict(reroute)))            # not before an exception
        self.assertEqual(self.w.counts["rejected_DESTINATION_SET_BY_WMS"], 1)
        self.w.apply_event(self.ev("DESTINATION_OCCUPIED", t=50.0, robot_id=1, task_id="PA-IN1", slot="r0s02",
                                   item_id="C", observed="B"))
        self.assertEqual(self.w.rec["r0s02"]["item"], "B")
        self.assertEqual(self.w.tasks["PA-IN1"]["status"], "ON_HOLD")
        self.w.apply_decision({"robot_id": 1, "task_id": "PA-IN1", "action": "WAIT"})
        self.assertEqual(self.w.tasks["PA-IN1"]["hold_reason"], "WAIT")
        self.assertIsNone(self.w.apply_decision({**reroute, "destination": "r0s01"}))
        self.assertEqual(self.w.counts["rejected_DESTINATION_NOT_EMPTY"], 1)
        self.assertIsNotNone(self.w.apply_decision(dict(reroute)))
        self.assertEqual(self.w.rec["r0s05"]["reserved_by"], "PA-IN1")

    def test_unreachable_destination_keeps_record(self):
        self.stage("B0", "IN1", "C")
        self.execute("PA-IN1", 1)
        self.w.apply_event(self.ev("DESTINATION_OCCUPIED", t=50.0, robot_id=1, task_id="PA-IN1", slot="r0s02",
                                   item_id="C", observed="UNREACHABLE"))
        self.assertIsNone(self.w.rec["r0s02"]["item"])
        self.assertEqual(self.w.tasks["PA-IN1"]["status"], "ON_HOLD")

    def test_putaway_waits_for_a_free_slot(self):
        for k in range(2, 6):
            self.w.rec[f"r0s{k:02d}"]["item"] = "B"                         # racks full in the record
        self.stage("B0", "IN1", "C")
        task = self.w.tasks["PA-IN1"]
        self.assertIsNone(task["destination"])
        self.assertIsNone(self.execute("PA-IN1", 1, source="B0"))
        self.assertEqual(self.w.counts["rejected_NO_PUTAWAY_SLOT_YET"], 1)
        self.pick_flow()                                                    # takes the A out of r0s00
        self.w.apply_event(self.ev("TASK_COMPLETED", t=30.0, robot_id=2, task_id="PK-O1-A", task_type="PICK",
                                   item_id="A", slot="LR1-0"))
        self.assertEqual(task["destination"], "r0s00")                     # the slot that freed up
        self.assertIsNotNone(self.execute("PA-IN1", 1))
        self.w.apply_decision({"robot_id": 1, "task_id": "PA-IN1", "action": "REASSIGN"})
        self.assertEqual((task["status"], task["destination"]), ("PENDING", "r0s00"))   # keeps its slot
        self.assertEqual(self.w.rec["r0s00"]["reserved_by"], "PA-IN1")

    def test_observation_discrepancy(self):
        self.w.apply_event(self.ev("OBSERVATION_RESULT", t=5.0, robot_id=0, slot="r0s02", observed="A",
                                   source="patrol"))
        self.assertEqual(self.w.rec["r0s02"]["item"], "A")
        self.assertEqual(self.w.exceptions[-1]["type"], "RECORD_DISCREPANCY")
        self.assertEqual(self.w.rec["r0s02"]["observed_t"], 5.0)

    def test_restaged_buffer_survives_old_completion(self):
        """A carton staged where the previous one was picked up must keep its record when the
        previous putaway completes (even with the same SKU)."""
        self.stage("B0", "IN1", "C", t=1.0)                                # WMS slot: r0s02
        self.execute("PA-IN1", 1)
        self.stage("B0", "IN2", "C", t=5.0)                                # IN1 was taken from B0
        self.assertTrue(self.w.tasks["PA-IN1"]["picked"])
        self.assertEqual(self.w.tasks["PA-IN2"]["destination"], "r0s03")
        self.assertIsNotNone(self.execute("PA-IN2", 2))
        self.w.apply_event(self.ev("TASK_COMPLETED", t=9.0, robot_id=1, task_id="PA-IN1", task_type="PUTAWAY",
                                   item_id="C", slot="r0s02"))
        self.assertEqual(self.w.rec["B0"]["item"], "C")                     # IN2 still there
        self.assertEqual(self.w.rec["B0"]["reserved_by"], "PA-IN2")
        self.assertEqual(self.w.rec["r0s02"]["item"], "C")

    # ---- failures and decision checks ----
    def test_task_failed_then_requeued(self):
        self.pick_flow()
        self.w.apply_event(self.ev("TASK_FAILED", t=20.0, robot_id=2, task_id="PK-O1-A", reason="PICK_too_far"))
        task = self.w.tasks["PK-O1-A"]
        self.assertEqual((task["status"], task["fail_reason"]), ("FAILED", "PICK_too_far"))
        self.assertIsNone(self.w.robots[2]["assigned"])
        self.assertIsNone(self.w.rec["r0s00"]["reserved_by"])
        self.w.apply_decision({"robot_id": None, "task_id": "PK-O1-A", "action": "REASSIGN"})
        self.assertEqual(task["status"], "PENDING")

    def test_busy_failure_only_frees_that_task(self):
        self.pick_flow()
        self.w.apply_event(self.ev("TASK_FAILED", t=20.0, robot_id=1, task_id="PK-O1-A", reason="BUSY"))
        self.assertEqual(self.w.tasks["PK-O1-A"]["status"], "ASSIGNED")    # robot 1 never had it

    def test_cancel_refused_by_robot(self):
        self.pick_flow()
        self.w.apply_decision({"robot_id": 2, "task_id": "PK-O1-A", "action": "CANCEL"})
        self.w.apply_event(self.ev("TASK_FAILED", t=21.0, robot_id=2, task_id="PK-O1-A", reason="HOLDING_CARTON",
                                   item_id="A"))
        task = self.w.tasks["PK-O1-A"]
        self.assertEqual((task["status"], task["robot_id"]), ("ON_HOLD", 2))
        self.assertEqual(self.w.robots[2]["assigned"], "PK-O1-A")

    def test_rejected_decisions(self):
        self.w.apply_event(self.ev("ORDER_RELEASED", t=10.0, wave="W1", lines=[
            {"order": "O1", "customer": "C1", "region": "R1", "sku": "A", "qty": 2}]))
        self.stage("B0", "IN1", "C")
        cases = [
            ({"robot_id": 0, "task_id": "PK-O1-A", "action": "EXECUTE", "source": "r0s00", "destination": "LR1-0"},
             "ROLE"),
            ({"robot_id": 2, "task_id": "PK-O1-A", "action": "EXECUTE", "source": "r0s01", "destination": "LR1-0"},
             "SOURCE_NOT_RECORDED"),
            ({"robot_id": 2, "task_id": "PA-IN1", "action": "EXECUTE", "source": "B0", "destination": "r0s04"},
             "DESTINATION_SET_BY_WMS"),
            ({"robot_id": 2, "task_id": "NOPE", "action": "EXECUTE"}, "UNKNOWN_TASK"),
            ({"robot_id": 2, "task_id": "PK-O1-A", "action": "JUMP"}, "UNKNOWN_ACTION"),
        ]
        for d, why in cases:
            self.assertIsNone(self.w.apply_decision(d), why)
            self.assertEqual(self.w.counts.get(f"rejected_{why}"), 1, why)
        self.assertIsNotNone(self.execute("PK-O1-A", 2, source="r0s00", destination="LR1-0"))
        again = [
            ({"robot_id": 1, "task_id": "PK-O1-A-2", "action": "EXECUTE", "source": "r0s00", "destination": "LR1-1"},
             "SLOT_RESERVED"),
            ({"robot_id": 2, "task_id": "PA-IN1", "action": "EXECUTE", "source": "B0", "destination": "r0s04"},
             "ROBOT_BUSY"),
            ({"robot_id": 1, "task_id": "PK-O1-A", "action": "EXECUTE", "source": "r0s00", "destination": "LR1-0"},
             "TASK_ASSIGNED"),
            ({"robot_id": 1, "task_id": "PK-O1-A", "action": "CANCEL"}, "NOT_THIS_ROBOT"),
        ]
        for d, why in again:
            self.assertIsNone(self.w.apply_decision(d), why)
            self.assertEqual(self.w.counts.get(f"rejected_{why}"), 1, why)
        self.w.update_robots(20.0, robots_report(self.w, r2={"task": "PK-O1-A", "holding": "A"}))
        self.assertIsNone(self.w.apply_decision({"robot_id": 2, "task_id": "PK-O1-A", "action": "CANCEL"}))
        self.assertEqual(self.w.counts.get("rejected_ROBOT_HOLDING_CARTON"), 1)

    def test_create_verify(self):
        sent = self.w.apply_decision({"robot_id": None, "action": "CREATE",
                                      "task": {"task_type": "VERIFY", "destination": "r0s02", "reason": "S2"}})
        self.assertEqual(sent["task_id"], "VF-W0001")
        task = self.w.tasks["VF-W0001"]
        self.assertEqual((task["status"], task["destination"], task["reason"]), ("PENDING", "r0s02", "S2"))
        self.assertIsNone(self.w.apply_decision({"robot_id": None, "action": "CREATE",
                                                 "task": {"task_type": "PICK", "item_id": "A"}}))


class RunnerTest(unittest.TestCase):
    """The runner with the stand-in scheduler, driven like the supervisor drives a WES,
    then rebuilt from the event log."""

    def test_flow_and_replay(self):
        cfg = small_cfg()
        with tempfile.TemporaryDirectory() as tmp:
            wes = WES(cfg, scheduler=Scheduler(cfg), log_dir=tmp)
            self.assertEqual(wes.name, "wms-standin")
            ev, lines = Feed(), []

            def emit(e):
                lines.append(e)
                wes.on_event(e)

            def step(t, **rep):
                for d in wes.decide(t, robots_report(wes.wms, **rep)):
                    lines.append({"t": t, "kind": "DECISION", **d})

            emit(ev("TRUCK_ARRIVED", t=1.0, truck="T1", asn=[{"lpn": "IN1", "item": "C"}]))
            emit(ev("INBOUND_STAGED", t=2.0, slot="B0", lpn="IN1", item="C"))
            emit(ev("ORDER_RELEASED", t=2.0, wave="W1", lines=[
                {"order": "O1", "customer": "C1", "region": "R1", "sku": "A", "qty": 1}]))
            step(2.0)
            sent = {d["task_id"]: d for d in lines if d.get("kind") == "DECISION"}
            # The PICK goes first (priority 1) to the transport nearest to r0s00, the PUTAWAY of
            # a class C carton to the free rack slot farthest from the outbound side.
            self.assertEqual(sent["PK-O1-A"]["robot_id"], 1)
            self.assertEqual(sent["PK-O1-A"]["task"]["destination"], "LR1-0")
            self.assertEqual(sent["PA-IN1"]["robot_id"], 2)
            self.assertEqual(sent["PA-IN1"]["task"]["destination"], "r0s02")
            step(3.0, r1={"task": "PK-O1-A"}, r2={"task": "PA-IN1"})
            emit(ev("SHORT_PICK", t=10.0, robot_id=1, task_id="PK-O1-A", slot="r0s00", item_id="A",
                    observed="EMPTY"))
            emit(ev("DESTINATION_OCCUPIED", t=11.0, robot_id=2, task_id="PA-IN1", slot="r0s02", item_id="C",
                    observed="B"))
            step(11.0, r1={"task": "PK-O1-A"}, r2={"task": "PA-IN1", "holding": "C"})
            acts = [(d["task_id"], d["action"], d.get("destination")) for d in lines if d.get("kind") == "DECISION"][2:]
            # No other A recorded: back to the queue. Occupied slot: the free slot nearest to it.
            self.assertEqual(acts, [("PK-O1-A", "REASSIGN", None), ("PA-IN1", "REROUTE", "r0s03")])
            emit(ev("TASK_COMPLETED", t=20.0, robot_id=2, task_id="PA-IN1", task_type="PUTAWAY", item_id="C",
                    slot="r0s03"))
            step(20.0)
            self.assertEqual(wes.wms.rec["r0s03"]["item"], "C")
            self.assertEqual(wes.wms.rec["r0s02"]["item"], "B")
            self.assertIsNone(wes.wms.rec["B0"]["item"])
            self.assertEqual(wes.wms.tasks["PK-O1-A"]["status"], "PENDING")     # backorder: no A recorded
            wes.stats()

            events = Path(tmp) / "events.jsonl"
            events.write_text("".join(json.dumps(x) + "\n" for x in lines), encoding="utf-8")
            report = replay_wms.check(cfg, Path(tmp))
            self.assertTrue(report["same"], report["diff"])


if __name__ == "__main__":
    unittest.main()
