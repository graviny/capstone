"""The stand-in WES policy (lib/wes_standin.py) as a scheduler on top of the Mock WMS.

Temporary, to run the WMS end to end until the team's baseline policies exist. Same
choices as wes_standin:
  - PENDING tasks by (priority, creation time); each goes to the nearest idle robot of its
    role (VERIFY to a patrol robot if there is one, everything else to a transport robot)
  - PICK from the recorded slot nearest to the outbound side, into the first free slot of
    the order's lane; PUTAWAY into the slot the WMS gave it (wes_standin picked it here)
  - SHORT_PICK / ITEM_MISMATCH: a PUTAWAY is cancelled (the WMS makes new work for what is
    really there); a PICK is rerouted to another recorded slot, or goes back to the queue
  - DESTINATION_OCCUPIED: reroute to the nearest free slot (lane slot for a PICK), or WAIT
  - TASK_FAILED: back to the queue
"""

EXECUTE, REROUTE, WAIT, CANCEL, REASSIGN = "EXECUTE", "REROUTE", "WAIT", "CANCEL", "REASSIGN"


class Scheduler:
    name = "standin"

    def __init__(self, cfg):
        self.has_patrol = any(r["role"] == "patrol" for r in cfg["robots"])
        self.waiting = []          # tasks told to WAIT until a destination frees up

    def source_of(self, s, item, exclude=()):
        cands = s.slots_with(item, exclude=exclude)
        return min(cands, key=lambda sid: s.dist(s.pos(sid), s.out_xy)) if cands else None

    def destination_of(self, s, task, near=None, exclude=()):
        if task["task_type"] == "PICK":
            return s.lane_free_slot(task["region"], exclude)
        return s.putaway_slot(task["item_id"], near=near, exclude=exclude)

    def on_event(self, ev, s):
        task = s.task(ev.get("task_id"))
        if task is None:
            return []
        kind, rid, tid = ev["type"], task["robot_id"], task["task_id"]
        if kind in ("SHORT_PICK", "ITEM_MISMATCH") and task["status"] == "ON_HOLD":
            if task["task_type"] == "PUTAWAY":
                return [{"robot_id": rid, "task_id": tid, "action": CANCEL}]
            alt = self.source_of(s, task["item_id"])
            if alt:
                return [{"robot_id": rid, "task_id": tid, "action": REROUTE, "source": alt,
                         "destination": task["destination"]}]
            return [{"robot_id": rid, "task_id": tid, "action": REASSIGN}]
        if kind == "DESTINATION_OCCUPIED" and task["status"] == "ON_HOLD":
            new = self.destination_of(s, task, near=ev["slot"])
            if new:
                return [{"robot_id": rid, "task_id": tid, "action": REROUTE, "source": task["source"],
                         "destination": new}]
            self.waiting.append(tid)
            return [{"robot_id": rid, "task_id": tid, "action": WAIT}]
        if kind == "TASK_FAILED" and task["status"] == "FAILED":
            return [{"robot_id": None, "task_id": tid, "action": REASSIGN}]
        return []

    def plan(self, s):
        out, taken = [], set()
        for tid in list(self.waiting):
            task = s.task(tid)
            if task is None or task["status"] != "ON_HOLD":
                self.waiting.remove(tid)
                continue
            new = self.destination_of(s, task, exclude=taken)
            if new:
                self.waiting.remove(tid)
                taken.add(new)
                out.append({"robot_id": task["robot_id"], "task_id": tid, "action": REROUTE,
                            "source": task["source"], "destination": new})

        idle = {r["id"]: r for r in s.idle_robots()}
        for task in s.pending_tasks():
            if not idle:
                break
            role = "patrol" if task["task_type"] == "VERIFY" and self.has_patrol else "transport"
            cands = [r for r in idle.values() if r["role"] == role]
            if not cands:
                continue
            if task["task_type"] == "PICK":
                src = self.source_of(s, task["item_id"], exclude=taken)
                dst = s.lane_free_slot(task["region"], exclude=taken)
                if src is None or dst is None:
                    continue             # no recorded stock (backorder) or the lane is full
            elif task["task_type"] == "PUTAWAY":
                src, dst = task["source"], task["destination"]     # the WMS reserved it
                if dst is None:
                    continue                 # no free slot in the record yet
            elif task["task_type"] == "RELOCATE":
                src = task["source"]
                dst = task["destination"] or s.putaway_slot(task["item_id"], exclude=taken)
                if dst is None:
                    continue
            else:
                src, dst = None, task["destination"]
            at = s.pos(src or dst)
            robot = min(cands, key=lambda r: s.dist(r["pos"], at))
            taken.update(x for x in (src, dst) if x)
            del idle[robot["id"]]
            out.append({"robot_id": robot["id"], "task_id": task["task_id"], "action": EXECUTE,
                        "source": src, "destination": dst})
        return out
