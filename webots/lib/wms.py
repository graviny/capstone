"""Mock WMS: the warehouse's record of stock, orders and tasks.

The WMS keeps what the central system believes, never the simulator's ground truth:

  slot records   what each slot holds according to the record (SKU, None = empty,
                 "UNKNOWN" = occupied but not identified), with the carton's LPN when
                 known, a version that changes whenever the record changes, the time a
                 robot last looked at the slot, and the task that reserved it
  orders         released order lines, picked and shipped units, cutoff deadline
  inbound        ASN of each arrived truck, what was staged / could not be read
  tasks          PUTAWAY, PICK, VERIFY, RELOCATE with their status (see below)

It changes only through
  apply_event(ev)          an environment or robot event (see webots/README.md)
  apply_decision(d)        a WES decision, checked against the record first
  update_robots(t, robots) the robots' own reports (task started, carton picked up)

and it never decides who does what next: that is the WES (a scheduler module, see
wes_runner.py). The WMS creates the work that business events call for (an order line
needs a PICK, a staged carton needs a PUTAWAY, an unreadable carton needs a VERIFY);
the WES may ask for more with a CREATE decision (VERIFY / RELOCATE). Where a carton is
stored is the WMS's too: it gives each new PUTAWAY a free slot (choose_putaway_slot) and
reserves it; the WES may move it only after an exception or a WAIT (REROUTE while ON_HOLD).

Task status
  PENDING      waiting for the WES to assign it
  ASSIGNED     given to a robot (EXECUTE sent)
  IN_PROGRESS  the robot reports it is working on it
  ON_HOLD      the robot stopped: an exception (SHORT_PICK, ITEM_MISMATCH,
               DESTINATION_OCCUPIED) or a WAIT; needs a REROUTE / CANCEL / REASSIGN
  FAILED       the robot gave up (TASK_FAILED); needs a REASSIGN (back to PENDING) or CANCEL
  COMPLETED    done, the record was updated
  CANCELLED    dropped

Everything the WMS does is written to wms_log.jsonl, so a run can be rebuilt from its
events.jsonl (tools/replay_wms.py). Interface details: webots/docs/wms.md.
"""
import hashlib
import json
import math

PENDING, ASSIGNED, IN_PROGRESS, ON_HOLD = "PENDING", "ASSIGNED", "IN_PROGRESS", "ON_HOLD"
COMPLETED, FAILED, CANCELLED = "COMPLETED", "FAILED", "CANCELLED"
STATUSES = (PENDING, ASSIGNED, IN_PROGRESS, ON_HOLD, FAILED, COMPLETED, CANCELLED)
ACTIVE = (ASSIGNED, IN_PROGRESS, ON_HOLD)          # a robot is on the task
FINAL = (COMPLETED, CANCELLED)
UNKNOWN = "UNKNOWN"                                # occupied, item not identified
INBOUND = ("buffer", "exception")
TASK_TYPES = ("PUTAWAY", "PICK", "VERIFY", "RELOCATE")
MOVES = ("PUTAWAY", "PICK", "RELOCATE")            # tasks that carry a carton
PREFIX = {"PUTAWAY": "PA", "PICK": "PK", "VERIFY": "VF", "RELOCATE": "RL"}
PRIORITY = {"PICK": 1, "PUTAWAY": 2, "VERIFY": 2, "RELOCATE": 2}   # lower is more urgent
# Fields the WES picks when it assigns a task; cleared when the task goes back to PENDING.
# (A PUTAWAY's destination is the WMS's: see _slot_putaways.)
CHOSEN_AT_ASSIGN = {"PICK": ("source", "destination"), "PUTAWAY": (), "VERIFY": (), "RELOCATE": ()}
# Task fields a robot gets with EXECUTE.
PAYLOAD = ("task_id", "task_type", "item_id", "source", "destination", "priority", "deadline", "state_version",
           "order", "region", "wave", "lpn")
REJECT_PRINT_LIMIT = 20


def scenario_hash(cfg):
    return hashlib.sha256(json.dumps(cfg, sort_keys=True).encode()).hexdigest()[:16]


class MockWMS:
    def __init__(self, cfg, log_path=None, strict=True):
        """strict=False skips the checks on decisions (replaying decisions a run already accepted)."""
        self.strict = strict
        self.slots = {s["id"]: s for s in cfg["slots"]}            # static: kind, pos, approach, face / lane
        self.rec = {sid: {"item": None, "lpn": None, "version": 0, "updated_t": 0.0, "observed_t": None,
                          "reserved_by": None} for sid in self.slots}
        for st in cfg["initial_stock"]:
            self.rec[st["slot"]].update(item=st["sku"], lpn=st["lpn"], version=1)
        self.abc = {s["sku"]: s.get("abc", "C") for s in cfg["skus"]}
        self.robots = {r["id"]: {"id": r["id"], "role": r["role"], "pos": list(r.get("home", (0.0, 0.0))),
                                 "task": None, "holding": None, "phase": None, "assigned": None}
                       for r in cfg["robots"]}
        out = cfg["outbound"]
        self.lanes = {region: lane["slots"] for region, lane in out["lanes"].items()}
        self.lane_truck = {region: True for region in self.lanes}   # region truck at the dock
        self.cutoff_period = out.get("cutoff_period_s")
        self.out_xy = tuple(out.get("center", (0.0, 0.0)))
        self.tasks = {}                    # task id -> task (creation order)
        self.orders = {}
        self.asn = {}                      # lpn -> {item, truck, arrived_t, status, slot}
        self.exceptions = []               # what robots found that disagrees with the record
        self.counts = {}
        self.t = 0.0
        self.version = 0                   # bumps on every change of the record or of a task
        self._pending = set()
        self._unslotted = []               # PENDING putaways still without a destination
        self._reserved = {}                # task id -> slots it reserved
        self._ids = {}
        self._w_seq = 0
        self._queue_seq = 0
        self._seen = set()
        self._rejects_printed = 0
        self.view = WMSView(self)
        self._log = open(log_path, "w", encoding="utf-8") if log_path else None
        self.log("START", scenario=scenario_hash(cfg), slots=len(self.slots), robots=len(self.robots),
                 initial_stock=len(cfg["initial_stock"]))

    # ------------------------------------------------------------------ log
    def log(self, entry_type, **kw):
        if self._log:
            self._log.write(json.dumps({"t": round(self.t, 2), "type": entry_type, **kw}) + "\n")

    def count(self, key, n=1):
        self.counts[key] = self.counts.get(key, 0) + n

    def anomaly(self, kind, **kw):
        """Something that should not happen with a consistent record; kept for the post-run check."""
        self.count(f"anomaly_{kind}")
        self.log("ANOMALY", kind=kind, **kw)

    def flush(self):
        if self._log:
            self._log.flush()

    def close(self):
        if self._log and not self._log.closed:
            self.log("FINAL", **self.final_snapshot())
            self._log.close()

    # ---------------------------------------------------------------- slots
    def set_slot(self, sid, item, cause, lpn=None, arrival=False, observed=False, **ctx):
        """Change a slot record. arrival=True: a (new) carton was put there, so the version
        changes even when the SKU is the same as before. Returns True if the record changed."""
        r = self.rec[sid]
        if observed:
            r["observed_t"] = self.t
        if not arrival and r["item"] == item:
            if lpn and item is not None:
                r["lpn"] = lpn
            return False
        old = r["item"]
        r.update(item=item, lpn=lpn if item is not None else None, version=r["version"] + 1, updated_t=self.t)
        self.version += 1
        self.log("SLOT", slot=sid, old=old, new=item, lpn=r["lpn"], cause=cause, **ctx)
        return True

    def reserve(self, tid, sid):
        self.rec[sid]["reserved_by"] = tid
        self._reserved.setdefault(tid, set()).add(sid)

    def release(self, tid, sid=None):
        for s in ([sid] if sid else list(self._reserved.get(tid, ()))):
            if self.rec[s]["reserved_by"] == tid:
                self.rec[s]["reserved_by"] = None
            self._reserved.get(tid, set()).discard(s)

    def next_cutoff(self, t):
        if not self.cutoff_period:
            return None
        return (math.floor(t / self.cutoff_period) + 1) * self.cutoff_period

    def free_slots(self, kind="rack", exclude=()):
        """Slots of this kind that are empty in the record and not reserved (scenario order)."""
        return [sid for sid, r in self.rec.items() if r["item"] is None and r["reserved_by"] is None
                and self.slots[sid]["kind"] == kind and sid not in exclude]

    def choose_putaway_slot(self, item, near=None, exclude=()):
        """The storage rule: the free rack slot nearest to `near`, or by class (A near the
        outbound side, B in between, C far from it)."""
        free = self.free_slots("rack", exclude)
        if not free:
            return None
        pos = lambda s: self.slots[s]["pos"]                       # noqa: E731
        if near is not None:
            p = pos(near)
            return min(free, key=lambda s: math.hypot(pos(s)[0] - p[0], pos(s)[1] - p[1]))
        sign = {"A": 1, "B": 0.3, "C": -1}[self.abc.get(item, "C")]
        return min(free, key=lambda s: sign * math.hypot(pos(s)[0] - self.out_xy[0], pos(s)[1] - self.out_xy[1]))

    def _slot_putaways(self):
        """Give each PENDING putaway without a destination a free slot (oldest first) and reserve
        it. Runs after every event and decision, so a replay does the same."""
        for tid in list(self._unslotted):
            task = self.tasks[tid]
            if task["status"] != PENDING or task["destination"] is not None:
                self._unslotted.remove(tid)
                continue
            dst = self.choose_putaway_slot(task["item_id"])
            if dst is None:
                return                     # racks full (in the record): wait for a slot to free up
            self._unslotted.remove(tid)
            task.update(destination=dst, state_version=task["state_version"] + 1)
            self.reserve(tid, dst)
            self.version += 1
            self.log("PUTAWAY_SLOT", task_id=tid, item=task["item_id"], destination=dst)

    def _keep_putaway_slot(self, task):
        """A putaway back in the queue keeps its destination if that is still free."""
        dst = task["destination"]
        if dst and self.rec[dst]["item"] is None and self.rec[dst]["reserved_by"] in (None, task["task_id"]):
            self.reserve(task["task_id"], dst)
        else:
            task["destination"] = None
            self._unslotted.append(task["task_id"])

    # ---------------------------------------------------------------- tasks
    def _next_id(self, base):
        n = self._ids[base] = self._ids.get(base, 0) + 1
        return base if n == 1 else f"{base}-{n}"

    def new_task(self, task_type, item, cause, task_id=None, source=None, destination=None, priority=None,
                 deadline=None, **extra):
        tid = task_id or self._next_id(f"{PREFIX[task_type]}-{extra.get('lpn') or source or destination}")
        task = {"task_id": tid, "task_type": task_type, "item_id": item, "source": source,
                "destination": destination, "status": PENDING,
                "priority": PRIORITY[task_type] if priority is None else priority, "deadline": deadline,
                "state_version": 1, "created_t": self.t, "updated_t": self.t, "robot_id": None, "attempts": 0,
                "hold_reason": None, "fail_reason": None, "picked": False, "source_version": None,
                "queue_seq": self._enqueue(), **extra}
        self.tasks[tid] = task
        self._pending.add(tid)
        if task_type == "PUTAWAY" and destination is None:
            self._unslotted.append(tid)            # the WMS picks its slot (_slot_putaways)
        self.version += 1
        self.count(f"created_{task_type}")
        self.log("TASK_CREATED", task_id=tid, task_type=task_type, item=item, source=source,
                 destination=destination, cause=cause)
        return task

    def _enqueue(self):
        self._queue_seq += 1
        return self._queue_seq

    def set_status(self, task, status, cause, **kw):
        old = task["status"]
        if old == status:
            return
        tid = task["task_id"]
        task.update(status=status, updated_t=self.t, state_version=task["state_version"] + 1, **kw)
        self.version += 1
        if status == PENDING:
            task.update(queue_seq=self._enqueue(), picked=False, source_version=None, hold_reason=None)
            self._pending.add(tid)
            for f in CHOSEN_AT_ASSIGN[task["task_type"]]:
                task[f] = None
        else:
            self._pending.discard(tid)
        if status not in ACTIVE:
            self.release(tid)
            self._detach(task)
        if status == PENDING and task["task_type"] == "PUTAWAY":
            self._keep_putaway_slot(task)
        self.log("TASK_STATUS", task_id=tid, old=old, new=status, cause=cause, **kw)

    def _detach(self, task):
        rid = task["robot_id"]
        if rid is not None and self.robots[rid]["assigned"] == task["task_id"]:
            self.robots[rid]["assigned"] = None
        task["robot_id"] = None

    def _hold(self, task, reason):
        if task["status"] in ACTIVE:
            task["hold_reason"] = reason
            self.set_status(task, ON_HOLD, reason, hold_reason=reason)

    def _open_refs(self, sid):
        """Open tasks still about the carton in slot sid (not yet picked up from it)."""
        return [t for t in self.tasks.values() if t["status"] not in FINAL and not t["picked"]
                and (t["source"] == sid or (t["task_type"] == "VERIFY" and t["destination"] == sid))]

    def _inbound_work(self, sid, lpn=None):
        """A carton waiting in the inbound area needs a PUTAWAY, or a VERIFY if it is not identified."""
        item = self.rec[sid]["item"]
        if item == UNKNOWN:
            return self.new_task("VERIFY", None, "INBOUND_UNIDENTIFIED", destination=sid)
        if item is not None:
            return self.new_task("PUTAWAY", item, "INBOUND_IDENTIFIED", source=sid,
                                 lpn=lpn or self.rec[sid]["lpn"])
        return None

    def _inbound_followup(self, sid):
        if sid and self.slots[sid]["kind"] in INBOUND and not self._open_refs(sid):
            self._inbound_work(sid)

    def _carton_left(self, sid, cause):
        """A new carton arrived in sid, so tasks for the previous one have picked it up: their
        reservation of sid is over."""
        for t in self.tasks.values():
            if t["source"] == sid and t["status"] in ACTIVE:
                if not t["picked"]:
                    self._mark_picked(t, cause)
                self.release(t["task_id"], sid)
            elif t["source"] == sid and t["status"] in (PENDING, FAILED) and t["task_type"] == "PUTAWAY":
                self.anomaly("STALE_PUTAWAY", task_id=t["task_id"], slot=sid)

    def _mark_picked(self, task, cause):
        # The source stays reserved (and recorded) until the task completes, as the record only
        # changes on a confirmed result; a new carton arriving there ends it (_carton_left).
        task.update(picked=True, picked_t=self.t)
        self.log("TASK_PICKED", task_id=task["task_id"], source=task["source"], cause=cause)

    def _exception(self, kind, sid, task, observed, expected=None):
        rec = {"t": self.t, "type": kind, "slot": sid, "task_id": task["task_id"] if task else None,
               "expected": expected, "observed": observed}
        self.exceptions.append(rec)
        self.count(f"exception_{kind}")
        self.log("EXCEPTION", **{k: v for k, v in rec.items() if k != "t"})

    # --------------------------------------------------------------- events
    def apply_event(self, ev):
        """Apply one event. Returns False for an event already applied (same event_id)."""
        eid = ev.get("event_id")
        if eid is not None:
            if eid in self._seen:
                self.anomaly("DUPLICATE_EVENT", event_id=eid, event=ev.get("type"))
                return False
            self._seen.add(eid)
        self.t = max(self.t, ev.get("t", self.t))
        self.count(f"event_{ev['type']}")
        handler = getattr(self, f"_on_{ev['type'].lower()}", None)
        if handler:
            handler(ev)
        self._slot_putaways()
        return True

    def _task_of(self, ev):
        task = self.tasks.get(ev.get("task_id"))
        if task is None and ev.get("task_id"):
            self.anomaly("UNKNOWN_TASK", task_id=ev.get("task_id"), event=ev["type"], event_id=ev.get("event_id"))
        return task

    def _on_order_released(self, ev):
        for ln in ev["lines"]:
            oid = ln["order"]
            o = self.orders.get(oid)
            if o is None:
                o = self.orders[oid] = {"order": oid, "region": ln["region"], "customer": ln.get("customer"),
                                        "wave": ev.get("wave"), "released_t": self.t,
                                        "deadline": self.next_cutoff(self.t), "units": 0, "picked": 0,
                                        "shipped": 0, "status": "OPEN", "done_t": None}
            o["units"] += ln["qty"]
            for _ in range(ln["qty"]):
                self.new_task("PICK", ln["sku"], "ORDER_RELEASED", task_id=self._next_id(f"PK-{oid}-{ln['sku']}"),
                              deadline=o["deadline"], order=oid, region=ln["region"], wave=ev.get("wave"))
        self.log("ORDER_RELEASED", wave=ev.get("wave"), lines=len(ev["lines"]),
                 units=sum(ln["qty"] for ln in ev["lines"]))

    def _on_truck_arrived(self, ev):
        for c in ev.get("asn", []):
            self.asn[c["lpn"]] = {"item": c["item"], "truck": ev.get("truck"), "arrived_t": self.t,
                                  "status": "EXPECTED", "slot": None}
        self.log("ASN", truck=ev.get("truck"), cartons=len(ev.get("asn", [])))

    def _on_inbound_staged(self, ev):
        sid, lpn, item = ev["slot"], ev.get("lpn"), ev.get("item")
        self._carton_left(sid, "INBOUND_STAGED")
        self.set_slot(sid, item or UNKNOWN, "INBOUND_STAGED", lpn=lpn, arrival=True, event_id=ev.get("event_id"))
        if lpn:
            a = self.asn.get(lpn)
            if a is None:
                self.anomaly("NOT_IN_ASN", lpn=lpn, slot=sid)
            else:
                a.update(status="STAGED", slot=sid)
                if a["item"] != item:
                    self.anomaly("ASN_ITEM_MISMATCH", lpn=lpn, asn_item=a["item"], scanned=item)
        else:
            self.count("inbound_unread")
        self._inbound_work(sid, lpn)

    def _on_inbound_read_fail(self, ev):
        sid = ev["slot"]
        self._carton_left(sid, "INBOUND_READ_FAIL")
        self.set_slot(sid, UNKNOWN, "INBOUND_READ_FAIL", arrival=True, event_id=ev.get("event_id"))
        self._inbound_work(sid)

    def _on_outbound_truck_arrived(self, ev):
        self.lane_truck[ev["region"]] = True

    def _on_outbound_departed(self, ev):
        region = ev["region"]
        self.lane_truck[region] = False
        for c in ev.get("cartons", []):
            if self.rec[c["slot"]]["item"] != c["item"]:
                # Placed just before the truck left: its TASK_COMPLETED comes after this event.
                placing = next((t for t in self.tasks.values() if t["task_type"] == "PICK" and t["status"] in ACTIVE
                                and t["destination"] == c["slot"] and t["item_id"] == c["item"]
                                and t.get("order") == c.get("order")), None)
                if placing is not None:
                    placing["shipped_before_completion"] = True
                    self.count("shipped_before_completion")
                else:
                    self.anomaly("SHIPPED_NOT_AS_RECORDED", slot=c["slot"], recorded=self.rec[c["slot"]]["item"],
                                 shipped=c["item"], order=c.get("order"))
            o = self.orders.get(c.get("order"))
            if o is not None:
                o["shipped"] += 1
                if o["shipped"] >= o["units"] and o["status"] != "SHIPPED":
                    o.update(status="SHIPPED", done_t=self.t)
                    self.count("orders_shipped")
                    self.log("ORDER_SHIPPED", order=o["order"], lead_s=round(self.t - o["released_t"], 1))
        for sid in self.lanes[region]:          # the truck takes everything staged
            self.set_slot(sid, None, "OUTBOUND_DEPARTED", event_id=ev.get("event_id"))

    def _on_task_completed(self, ev):
        task = self._task_of(ev)
        if task is None:
            return
        if task["status"] not in ACTIVE:
            self.anomaly("LATE_COMPLETION", task_id=task["task_id"], status=task["status"],
                         event_id=ev.get("event_id"))
            return
        if task["task_type"] in MOVES:
            src, dst = task["source"], ev.get("slot") or task["destination"]
            if dst != task["destination"]:
                self.anomaly("COMPLETED_ELSEWHERE", task_id=task["task_id"], slot=dst, destination=task["destination"])
            if src and self.rec[src]["version"] == task["source_version"]:
                self.set_slot(src, None, "TASK_COMPLETED", task_id=task["task_id"])
            elif src:
                # Something new was recorded there after the robot took the carton: keep it.
                self.count("source_changed_before_completion")
            if not task.get("shipped_before_completion"):          # else it already left on the truck
                self.set_slot(dst, ev.get("item_id") or task["item_id"], "TASK_COMPLETED", lpn=task.get("lpn"),
                              arrival=True, task_id=task["task_id"])
            if task["task_type"] == "PICK" and task.get("order") in self.orders:
                self.orders[task["order"]]["picked"] += 1
        self.set_status(task, COMPLETED, "TASK_COMPLETED", done_t=self.t)
        if task["task_type"] == "VERIFY":
            self._inbound_followup(task["destination"])

    def _on_task_failed(self, ev):
        task = self._task_of(ev)
        if task is None:
            return
        rid, reason = ev.get("robot_id"), ev.get("reason")
        if reason == "HOLDING_CARTON":
            # The robot refused a CANCEL / REASSIGN: it still has the task and the carton.
            if task["status"] not in ACTIVE:
                self.anomaly("CANCEL_REFUSED_BY_ROBOT", task_id=task["task_id"], robot=rid, status=task["status"])
                if self.robots[rid]["assigned"] is None:
                    task["robot_id"] = rid
                    self.robots[rid]["assigned"] = task["task_id"]
                    task.update(picked=True, hold_reason="CANCEL_REFUSED")
                    self._pending.discard(task["task_id"])
                    self.set_status(task, ON_HOLD, "CANCEL_REFUSED", hold_reason="CANCEL_REFUSED")
            return
        if task["status"] in FINAL:
            self.anomaly("LATE_FAILURE", task_id=task["task_id"], status=task["status"], reason=reason)
            return
        if reason == "BUSY" and task["robot_id"] != rid:
            return                               # an EXECUTE the robot never took, already handled
        self.set_status(task, FAILED, "TASK_FAILED", fail_reason=reason)

    def _on_short_pick(self, ev):
        self._source_wrong(ev)

    def _on_item_mismatch(self, ev):
        self._source_wrong(ev)

    def _source_wrong(self, ev):
        """The robot looked at the source: the expected carton is not there."""
        task, sid, seen = self._task_of(ev), ev["slot"], ev.get("observed")
        old = self.rec[sid]["item"]
        new = None if seen in (None, "EMPTY") else (UNKNOWN if seen == "UNREADABLE" else seen)
        self.set_slot(sid, new, ev["type"], observed=seen != "UNREADABLE", task_id=ev.get("task_id"))
        self._exception(ev["type"], sid, task, seen, expected=ev.get("item_id") or old)
        if task is not None and task["status"] in ACTIVE:
            self.release(task["task_id"], sid)
            self._hold(task, ev["type"])

    def _on_destination_occupied(self, ev):
        task, sid, seen = self._task_of(ev), ev["slot"], ev.get("observed")
        if seen != "UNREACHABLE":               # unreachable says nothing about the slot
            self.set_slot(sid, UNKNOWN if seen in (None, "EMPTY", UNKNOWN) else seen, "DESTINATION_OCCUPIED",
                          observed=seen not in (None, UNKNOWN), task_id=ev.get("task_id"))
        self._exception("DESTINATION_OCCUPIED", sid, task, seen)
        if task is not None and task["status"] in ACTIVE:
            self.release(task["task_id"], sid)
            self._hold(task, "DESTINATION_OCCUPIED")

    def _on_observation_result(self, ev):
        sid, seen = ev["slot"], ev["observed"]
        task = self.tasks.get(ev.get("task_id"))
        if task is not None and task["status"] == ASSIGNED:
            self.set_status(task, IN_PROGRESS, "OBSERVATION_RESULT")
        if seen == "UNREADABLE":
            self.count("observations_unreadable")
            return
        old = self.rec[sid]["item"]
        new = None if seen == "EMPTY" else seen
        if self.set_slot(sid, new, "OBSERVATION", observed=True, task_id=ev.get("task_id"), by=ev.get("source")):
            if old != UNKNOWN:
                self._exception("RECORD_DISCREPANCY", sid, task, seen, expected=old)
        self._inbound_followup(sid)

    # ------------------------------------------------------------ decisions
    def apply_decision(self, d, trusted=False):
        """Check and apply one WES decision. Returns the message for the robot (EXECUTE gets the
        task payload), or None if the decision was rejected. trusted=True applies it unchecked."""
        action = d.get("action")
        fn = getattr(self, f"_do_{str(action).lower()}", None)
        if fn is None:
            return self.reject(d, "UNKNOWN_ACTION")
        task = self.tasks.get(d.get("task_id"))
        if action != "CREATE" and task is None:
            return self.reject(d, "UNKNOWN_TASK")
        if self.strict and not trusted:
            why = getattr(self, f"_check_{action.lower()}")(d, task)
            if why:
                return self.reject(d, why)
        self.count(f"decision_{action}")
        sent = fn(d, task)
        self._slot_putaways()
        return sent

    def reject(self, d, why):
        self.count("decisions_rejected")
        self.count(f"rejected_{why}")
        self.log("DECISION_REJECTED", reason=why, decision={k: v for k, v in d.items() if k != "task"})
        if self._rejects_printed < REJECT_PRINT_LIMIT:
            self._rejects_printed += 1
            print(f"[wms t={self.t:.0f}] rejected {d.get('action')} {d.get('task_id')} robot {d.get('robot_id')}: {why}")
        return None

    def _slots_of(self, d, task):
        """Source and destination of a decision: top-level fields, then its task payload, then the task."""
        payload = d.get("task") or {}
        src = d["source"] if "source" in d else payload.get("source", task["source"])
        dst = d["destination"] if "destination" in d else payload.get("destination", task["destination"])
        return src, dst

    def _check_slots(self, task, src, dst):
        ttype = task["task_type"]
        for s in (src, dst):
            if s is not None and s not in self.slots:
                return "NO_SUCH_SLOT"
        if dst is None or (ttype in MOVES and src is None):
            return "MISSING_SLOT"
        if ttype in MOVES and src == dst:
            return "SAME_SOURCE_DESTINATION"
        for s in (None if task["picked"] else src, dst):      # once picked up, the source no longer matters
            other = self.rec[s]["reserved_by"] if s else None
            if other not in (None, task["task_id"]):
                return "SLOT_RESERVED"
        return None

    def _check_move_records(self, task, src, dst, src_changed=True, dst_changed=True):
        if task["task_type"] not in MOVES:
            return None
        if src_changed and not task["picked"]:
            item = self.rec[src]["item"]
            if item != task["item_id"] and not (task["task_type"] == "RELOCATE" and item == UNKNOWN):
                return "SOURCE_NOT_RECORDED"
        if dst_changed and self.rec[dst]["item"] is not None:
            return "DESTINATION_NOT_EMPTY"
        return None

    def _check_execute(self, d, task):
        if task["status"] not in (PENDING, FAILED):
            return f"TASK_{task['status']}"
        r = self.robots.get(d.get("robot_id"))
        if r is None:
            return "NO_SUCH_ROBOT"
        if r["assigned"] is not None or r["task"] is not None:
            return "ROBOT_BUSY"
        if r["role"] == "patrol" and task["task_type"] != "VERIFY":
            return "ROLE"
        src, dst = self._slots_of(d, task)
        if task["task_type"] == "PUTAWAY":
            if task["destination"] is None:
                return "NO_PUTAWAY_SLOT_YET"       # racks full in the record
            if dst != task["destination"]:
                return "DESTINATION_SET_BY_WMS"
        return self._check_slots(task, src, dst) or self._check_move_records(task, src, dst)

    def _check_attached(self, d, task):
        if task["status"] not in ACTIVE:
            return f"TASK_{task['status']}"
        if d.get("robot_id") != task["robot_id"]:
            return "NOT_THIS_ROBOT"
        return None

    def _check_reroute(self, d, task):
        why = self._check_attached(d, task)
        if why:
            return why
        src, dst = self._slots_of(d, task)
        if task["task_type"] == "PUTAWAY" and dst != task["destination"] and task["status"] != ON_HOLD:
            return "DESTINATION_SET_BY_WMS"        # only an exception or a WAIT lets the WES move it
        return self._check_slots(task, src, dst) or self._check_move_records(
            task, src, dst, src_changed=src != task["source"], dst_changed=dst != task["destination"])

    def _check_wait(self, d, task):
        return self._check_attached(d, task)

    def _check_cancel(self, d, task):
        if task["status"] in FINAL:
            return f"TASK_{task['status']}"
        if task["status"] in ACTIVE:
            if d.get("robot_id") != task["robot_id"]:
                return "NOT_THIS_ROBOT"
            if self.robots[task["robot_id"]]["holding"]:
                return "ROBOT_HOLDING_CARTON"      # the robot would refuse it
        elif d.get("robot_id") is not None:
            return "TASK_NOT_ASSIGNED"
        return None

    def _check_reassign(self, d, task):
        return self._check_cancel(d, task)

    def _check_create(self, d, task):
        spec = d.get("task") or {}
        ttype = spec.get("task_type")
        if d.get("robot_id") is not None:
            return "CREATE_WITH_ROBOT"
        if ttype not in ("VERIFY", "RELOCATE"):
            return "CREATE_TYPE"
        for f in ("source", "destination"):
            if spec.get(f) is not None and spec[f] not in self.slots:
                return "NO_SUCH_SLOT"
        if ttype == "VERIFY" and spec.get("destination") is None:
            return "MISSING_SLOT"
        if ttype == "RELOCATE" and (spec.get("source") is None or not spec.get("item_id")):
            return "MISSING_SLOT"
        return None

    def _do_execute(self, d, task):
        src, dst = self._slots_of(d, task)
        rid, tid = d["robot_id"], task["task_id"]
        self._pending.discard(tid)
        task.update(source=src, destination=dst, robot_id=rid, attempts=task["attempts"] + 1, assigned_t=self.t,
                    picked=False, hold_reason=None, fail_reason=None,
                    source_version=self.rec[src]["version"] if src else None)
        if task["task_type"] == "PICK" and src:
            task["lpn"] = self.rec[src]["lpn"]
        for s in (src, dst):
            if s:
                self.reserve(tid, s)
        self.robots[rid]["assigned"] = tid
        self.set_status(task, ASSIGNED, "EXECUTE", robot_id=rid)
        return {**d, "task": {k: task.get(k) for k in PAYLOAD}}

    def _do_reroute(self, d, task):
        src, dst = self._slots_of(d, task)
        tid = task["task_id"]
        if src != task["source"]:
            if task["source"]:
                self.release(tid, task["source"])
            task["source_version"] = self.rec[src]["version"] if src else None
            if task["task_type"] == "PICK" and src:
                task["lpn"] = self.rec[src]["lpn"]
        if dst != task["destination"] and task["destination"]:
            self.release(tid, task["destination"])
        for s in (src if not task["picked"] else None, dst):
            if s:
                self.reserve(tid, s)
        self.log("TASK_REROUTED", task_id=tid, old_source=task["source"], source=src,
                 old_destination=task["destination"], destination=dst)
        task.update(source=src, destination=dst, state_version=task["state_version"] + 1, updated_t=self.t)
        self.version += 1
        if task["status"] == ON_HOLD:
            self.set_status(task, IN_PROGRESS, "REROUTE", hold_reason=None)
        return {**d, "source": src, "destination": dst}

    def _do_wait(self, d, task):
        self._hold(task, "WAIT")
        return d

    def _do_cancel(self, d, task):
        self.set_status(task, CANCELLED, "CANCEL")
        self._inbound_followup(task["source"] if task["task_type"] != "VERIFY" else task["destination"])
        return d

    def _do_reassign(self, d, task):
        self.set_status(task, PENDING, "REASSIGN")
        return d

    def _do_create(self, d, task):
        spec = d.get("task") or {}
        ttype = spec["task_type"]
        self._w_seq += 1
        extra = {k: v for k, v in spec.items() if k not in ("task_id", "task_type", "item_id", "source",
                                                              "destination", "priority", "deadline")}
        task = self.new_task(ttype, spec.get("item_id"), "WES_CREATE", task_id=f"{PREFIX[ttype]}-W{self._w_seq:04d}",
                             source=spec.get("source"), destination=spec.get("destination"),
                             priority=spec.get("priority"), deadline=spec.get("deadline"), **extra)
        return {**d, "task_id": task["task_id"]}

    # --------------------------------------------------------------- robots
    def update_robots(self, t, robots):
        """The robots' own reports, every step: which task they work on, what they carry."""
        self.t = max(self.t, t)
        for r in robots:
            rb = self.robots.get(r["id"])
            if rb is None:
                continue
            rb.update(pos=r["pos"], task=r.get("task"), holding=r.get("holding"), phase=r.get("phase"))
            task = self.tasks.get(rb["task"]) if rb["task"] else None
            if task is None or task["robot_id"] != rb["id"]:
                continue
            if task["status"] == ASSIGNED:
                self.set_status(task, IN_PROGRESS, "ROBOT")
            if rb["holding"] and not task["picked"] and task["task_type"] in MOVES:
                self._mark_picked(task, "ROBOT")

    # -------------------------------------------------------------- outputs
    def record_snapshot(self):
        """Rack slot -> SKU the record holds (None empty, "UNKNOWN"); the simulator compares it with the truth."""
        return {sid: r["item"] for sid, r in self.rec.items() if self.slots[sid]["kind"] == "rack"}

    def final_snapshot(self):
        return {"records": {sid: r["item"] for sid, r in self.rec.items()},
                "tasks": {tid: t["status"] for tid, t in self.tasks.items()},
                "orders": {oid: [o["picked"], o["shipped"]] for oid, o in self.orders.items()}}

    def stats(self):
        st = {"tasks": {}, "orders": {"released": len(self.orders),
                                      "shipped": sum(1 for o in self.orders.values() if o["status"] == "SHIPPED")},
              "inbound": {"asn_cartons": len(self.asn),
                          "staged_read": sum(1 for a in self.asn.values() if a["status"] == "STAGED")},
              "counts": dict(sorted(self.counts.items()))}
        for t in self.tasks.values():
            k = f"{t['task_type']}_{t['status']}"
            st["tasks"][k] = st["tasks"].get(k, 0) + 1
        st["tasks"] = dict(sorted(st["tasks"].items()))
        return st


class WMSView:
    """What a WES scheduler sees: the WMS's state and some queries on it. Read only; change
    things through decisions."""

    def __init__(self, wms):
        self._w = wms

    t = property(lambda self: self._w.t)
    version = property(lambda self: self._w.version)
    tasks = property(lambda self: self._w.tasks)          # task id -> task
    slots = property(lambda self: self._w.slots)          # slot id -> static info (kind, pos, ...)
    records = property(lambda self: self._w.rec)          # slot id -> record (item, lpn, version, ...)
    robots = property(lambda self: self._w.robots)        # robot id -> latest report + "assigned"
    orders = property(lambda self: self._w.orders)
    lanes = property(lambda self: self._w.lanes)          # region -> outbound lane slots
    lane_truck = property(lambda self: self._w.lane_truck)
    exceptions = property(lambda self: self._w.exceptions)
    out_xy = property(lambda self: self._w.out_xy)

    def task(self, tid):
        return self._w.tasks.get(tid)

    def pending_tasks(self):
        """PENDING tasks by (priority, creation time, time queued)."""
        tasks = self._w.tasks
        return sorted((tasks[tid] for tid in self._w._pending),
                      key=lambda t: (t["priority"], t["created_t"], t["queue_seq"]))

    def tasks_with(self, *statuses):
        return [t for t in self._w.tasks.values() if t["status"] in statuses]

    def idle_robots(self, role=None):
        return [r for r in self._w.robots.values() if r["assigned"] is None and r["task"] is None
                and (role is None or r["role"] == role)]

    def pos(self, sid):
        return self._w.slots[sid]["pos"]

    @staticmethod
    def dist(a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def item_at(self, sid):
        return self._w.rec[sid]["item"]

    def free_slots(self, kind="rack", exclude=()):
        """Slots of this kind that are empty in the record and not reserved."""
        return self._w.free_slots(kind, exclude)

    def slots_with(self, item, kind="rack", exclude=()):
        """Unreserved slots of this kind that hold the item according to the record."""
        return [sid for sid, r in self._w.rec.items() if r["item"] == item and r["reserved_by"] is None
                and self._w.slots[sid]["kind"] == kind and sid not in exclude]

    def stock(self, item):
        return sum(1 for sid, r in self._w.rec.items() if r["item"] == item and self._w.slots[sid]["kind"] == "rack")

    def lane_free_slot(self, region, exclude=()):
        for sid in self._w.lanes[region]:
            r = self._w.rec[sid]
            if r["item"] is None and r["reserved_by"] is None and sid not in exclude:
                return sid
        return None

    def putaway_slot(self, item, near=None, exclude=()):
        """The WMS's storage rule (the one it uses for new putaways): the free rack slot nearest
        to `near`, or by class (A near the outbound side, C far from it). For finding another
        slot when a putaway's destination turns out to be taken."""
        return self._w.choose_putaway_slot(item, near, exclude)

    def next_cutoff(self, t=None):
        return self._w.next_cutoff(self._w.t if t is None else t)

    def snapshot(self):
        """A JSON-ready copy of the state (for logging or an external solver)."""
        w = self._w
        return {"t": w.t, "version": w.version, "tasks": {k: dict(v) for k, v in w.tasks.items()},
                "records": {k: dict(v) for k, v in w.rec.items()},
                "robots": {k: dict(v) for k, v in w.robots.items()},
                "orders": {k: dict(v) for k, v in w.orders.items()}, "lane_truck": dict(w.lane_truck)}
