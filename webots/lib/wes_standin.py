"""Stand-in WMS + WES so the environment runs end to end.

This is NOT the research algorithm and not the team's Mock WMS. It keeps a
slot record, turns events into tasks and hands them out first-come first-served
to the nearest idle robot of the right role, and reacts to exceptions in the
simplest way (event-triggered replanning). Replace it by pointing the
WES_MODULE environment variable at another module with the same interface:

    class WES:
        name: str
        def __init__(self, scenario): ...
        def on_event(self, event: dict) -> None: ...
        def decide(self, t: float, robots: list[dict]) -> list[dict]:   # Decisions
        def record_snapshot(self) -> dict[str, str | None]:             # optional: slot -> SKU it believes

Task / Decision / event fields follow the team design document (section 6).
The WES only sees events; it never reads the simulator's ground truth.
"""
import math


class WES:
    name = "standin"

    def __init__(self, cfg):
        self.slots = {s["id"]: s for s in cfg["slots"]}
        self.roles = {r["id"]: r["role"] for r in cfg["robots"]}
        self.lanes = cfg["outbound"]["lanes"]
        self.abc = {s["sku"]: s["abc"] for s in cfg["skus"]}
        self.has_patrol = any(r == "patrol" for r in self.roles.values())
        self.record = {sid: None for sid in self.slots}       # what the system believes is in each slot
        for st in cfg["initial_stock"]:
            self.record[st["slot"]] = st["sku"]
        self.tasks = {}
        self.pending = []          # task ids waiting for a robot (or for stock / lane space)
        self.waiting = []          # task ids holding a carton until a lane frees up
        self.busy = {}             # robot id -> task id
        self.reserved = {}         # slot -> task id
        self.out = []
        self.seq = 0
        self.t = 0.0
        self.out_xy = tuple(cfg["outbound"].get("center", (8.7, 0.0)))

    # ---- helpers ----
    def new_task(self, task_type, item, priority, source=None, destination=None, **extra):
        self.seq += 1
        tid = f"T{self.seq:05d}"
        self.tasks[tid] = {"task_id": tid, "task_type": task_type, "item_id": item, "source": source,
                           "destination": destination, "status": "PENDING", "priority": priority,
                           "deadline": None, "state_version": 1, "created_t": self.t, **extra}
        self.pending.append(tid)
        return tid

    def dist(self, a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def free_rack(self, item, near=None):
        free = [s for s, v in self.record.items()
                if v is None and self.slots[s]["kind"] == "rack" and s not in self.reserved]
        if not free:
            return None
        if near is not None:
            p = self.slots[near]["pos"]
            return min(free, key=lambda s: self.dist(self.slots[s]["pos"], p))
        # Class-based storage: fast movers near the outbound side.
        sign = {"A": 1, "B": 0.3, "C": -1}[self.abc.get(item, "C")]
        return min(free, key=lambda s: sign * self.dist(self.slots[s]["pos"], self.out_xy))

    def source_of(self, item):
        cands = [s for s, v in self.record.items()
                 if v == item and self.slots[s]["kind"] == "rack" and s not in self.reserved]
        return min(cands, key=lambda s: self.dist(self.slots[s]["pos"], self.out_xy)) if cands else None

    def lane_slot(self, region):
        for s in self.lanes[region]["slots"]:
            if self.record[s] is None and s not in self.reserved:
                return s
        return None

    def release(self, tid):
        for s in [s for s, v in self.reserved.items() if v == tid]:
            del self.reserved[s]

    def free_robot(self, tid):
        for r in [r for r, v in self.busy.items() if v == tid]:
            del self.busy[r]

    def decision(self, robot, tid, action, **kw):
        self.out.append({"robot_id": robot, "task_id": tid, "action": action, **kw})

    def robot_of(self, tid):
        return next((r for r, v in self.busy.items() if v == tid), None)

    def back_to_pending(self, tid, status="PENDING"):
        task = self.tasks[tid]
        self.release(tid)
        self.free_robot(tid)
        task["status"] = status
        task["source"] = task["source"] if task["task_type"] != "PICK" else None
        if task["task_type"] == "PICK":
            task["destination"] = None
        if tid not in self.pending:
            self.pending.append(tid)

    # ---- events ----
    def on_event(self, ev):
        self.t = ev["t"]
        kind = ev["type"]
        task = self.tasks.get(ev.get("task_id"))
        if kind == "ORDER_RELEASED":
            for ln in ev["lines"]:
                for _ in range(ln["qty"]):
                    self.new_task("PICK", ln["sku"], 1, order=ln["order"], region=ln["region"], wave=ev["wave"])
        elif kind == "INBOUND_STAGED":
            self.record[ev["slot"]] = ev["item"] or "UNKNOWN"
            if ev["item"]:
                self.new_task("PUTAWAY", ev["item"], 2, source=ev["slot"])
            else:
                self.new_task("VERIFY", None, 2, destination=ev["slot"])
        elif kind == "INBOUND_READ_FAIL":
            self.record[ev["slot"]] = "UNKNOWN"
            self.new_task("VERIFY", None, 2, destination=ev["slot"])
        elif kind == "OBSERVATION_RESULT":
            slot = ev["slot"]
            if ev["observed"] == "UNREADABLE":
                # Nothing learned; an unidentified inbound carton gets another look.
                if self.slots[slot]["kind"] in ("buffer", "exception") and self.record[slot] == "UNKNOWN":
                    self.new_task("VERIFY", None, 2, destination=slot)
                return
            seen = None if ev["observed"] == "EMPTY" else ev["observed"]
            self.record[slot] = seen
            # An identified carton in the inbound area becomes a putaway. A VERIFY there was
            # asked for a newly arrived carton; any other sighting only counts if no putaway
            # is waiting for that slot yet.
            verify = self.tasks.get(ev.get("task_id"), {}).get("task_type") == "VERIFY"
            if seen and self.slots[slot]["kind"] in ("buffer", "exception") and (verify or not any(
                    t["source"] == slot and t["status"] in ("PENDING", "ASSIGNED") for t in self.tasks.values()
                    if t["task_type"] == "PUTAWAY")):
                self.new_task("PUTAWAY", seen, 2, source=slot)
        elif kind == "TASK_COMPLETED" and task:
            task["status"] = "DONE"
            if task["task_type"] in ("PUTAWAY", "RELOCATE", "PICK"):
                self.record[task["source"]] = None
                self.record[task["destination"]] = task["item_id"]
            self.release(task["task_id"])
            self.free_robot(task["task_id"])
        elif kind in ("SHORT_PICK", "ITEM_MISMATCH") and task:
            seen = ev.get("observed")
            self.record[ev["slot"]] = None if seen in (None, "EMPTY") else (
                "UNKNOWN" if seen == "UNREADABLE" else seen)
            tid = task["task_id"]
            robot = self.robot_of(tid)
            if task["task_type"] == "PUTAWAY":
                # The inbound carton is not what the scanner said: drop the task, put away what is there.
                task["status"] = "CANCELLED"
                self.release(tid)
                self.free_robot(tid)
                self.decision(robot, tid, "CANCEL")
                if seen in ("UNREADABLE", "UNKNOWN"):
                    self.new_task("VERIFY", None, 2, destination=ev["slot"])
                elif seen not in (None, "EMPTY"):
                    self.new_task("PUTAWAY", seen, 2, source=ev["slot"])
                return
            self.reserved.pop(ev["slot"], None)
            alt = self.source_of(task["item_id"])
            if alt:
                self.reserved[alt] = tid
                task["source"] = alt
                task["state_version"] += 1
                self.decision(robot, tid, "REROUTE", source=alt, destination=task["destination"])
            else:
                self.decision(robot, tid, "CANCEL")
                self.back_to_pending(tid, "BACKORDER")
        elif kind == "DESTINATION_OCCUPIED" and task:
            seen = ev.get("observed")
            if seen != "UNREACHABLE":         # unreachable says nothing about what is in the slot
                self.record[ev["slot"]] = "UNKNOWN" if seen in (None, "EMPTY") else seen
            tid = task["task_id"]
            self.reserved.pop(ev["slot"], None)
            new = (self.lane_slot(task["region"]) if task["task_type"] == "PICK"
                   else self.free_rack(task["item_id"], near=ev["slot"]))
            if new:
                self.reserved[new] = tid
                task["destination"] = new
                task["state_version"] += 1
                self.decision(self.robot_of(tid), tid, "REROUTE", source=task["source"], destination=new)
            else:
                self.decision(self.robot_of(tid), tid, "WAIT")
                self.waiting.append(tid)
        elif kind == "OUTBOUND_DEPARTED":
            # The truck takes everything staged: the whole lane is empty again.
            for s in self.lanes[ev["region"]]["slots"]:
                self.record[s] = None
        elif kind == "TASK_FAILED" and task:
            if task["status"] != "DONE":
                self.back_to_pending(task["task_id"])

    # ---- assignment ----
    def decide(self, t, robots):
        self.t = t
        for tid in list(self.waiting):
            task = self.tasks[tid]
            new = (self.lane_slot(task["region"]) if task["task_type"] == "PICK"
                   else self.free_rack(task["item_id"]))
            if new:
                self.waiting.remove(tid)
                self.reserved[new] = tid
                task["destination"] = new
                task["state_version"] += 1
                self.decision(self.robot_of(tid), tid, "REROUTE", source=task["source"], destination=new)

        idle = {r["id"]: r for r in robots if r["id"] not in self.busy and r["task"] is None}
        self.pending.sort(key=lambda tid: (self.tasks[tid]["priority"], self.tasks[tid]["created_t"]))
        for tid in list(self.pending):
            if not idle:
                break
            task = self.tasks[tid]
            role = "patrol" if task["task_type"] == "VERIFY" and self.has_patrol else "transport"
            cands = [r for r in idle.values() if r["role"] == role]
            if not cands:
                continue
            if task["task_type"] == "PICK":
                src, dst = self.source_of(task["item_id"]), self.lane_slot(task["region"])
                if src is None or dst is None:
                    continue                 # no recorded stock (backorder) or the lane is full
            elif task["task_type"] in ("PUTAWAY", "RELOCATE"):
                src, dst = task["source"], task["destination"] or self.free_rack(task["item_id"])
                if dst is None:
                    continue
            else:
                src, dst = None, task["destination"]
            at = self.slots[src or dst]["pos"]
            robot = min(cands, key=lambda r: self.dist(r["pos"], at))
            task.update(source=src, destination=dst, status="ASSIGNED")
            for s in (src, dst):
                if s:
                    self.reserved[s] = tid
            self.busy[robot["id"]] = tid
            del idle[robot["id"]]
            self.pending.remove(tid)
            self.decision(robot["id"], tid, "EXECUTE", task=dict(task))
        out, self.out = self.out, []
        return out

    def record_snapshot(self):
        return {s: v for s, v in self.record.items() if self.slots[s]["kind"] == "rack"}

    def stats(self):
        st = {}
        for task in self.tasks.values():
            k = f"{task['task_type']}_{task['status']}"
            st[k] = st.get(k, 0) + 1
        return st
