"""Learned robot assignment: which robot does which task, scored by trained weights.

Same as sched_verify (VERIFY policy from VERIFY_POLICY, exception handling, source and
destination rules of sched_standin); only the assignment differs. Every step each pending
task is paired with each idle robot and with each busy robot (by where it will be free and
how far it still drives), a pair is scored w . features, and the best pairs are taken
greedily. When a busy robot wins a task, the task waits for it (nothing is sent; the next
steps decide again).

    WES_MODULE=wes_runner WES_SCHEDULER=sched_learned ASSIGN_WEIGHTS=models/assign_weights.json

Without ASSIGN_WEIGHTS (or the default file) it uses HEURISTIC: nearest robot, picks first.
Weights are trained in the lightweight simulator (tools/train_assign.py).
"""
import json
import os
from pathlib import Path

import sched_standin
import sched_verify

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_WEIGHTS = ROOT / "models" / "assign_weights.json"
FEATURES = ("dist", "busy", "pick", "putaway", "verify", "slack", "age", "truck_here", "lane_fill", "buffer_fill")
HEURISTIC = {"dist": -1.0, "busy": -1.0, "pick": 2.0}
MAX_TASKS = 30                     # pending tasks considered per step (oldest first)


def load_weights(path=None):
    """Weights from the file, or HEURISTIC when there is none (then the second value is False)."""
    path = Path(path or os.environ.get("ASSIGN_WEIGHTS") or DEFAULT_WEIGHTS)
    w = json.loads(path.read_text(encoding="utf-8"))["weights"] if path.exists() else HEURISTIC
    return [float(w.get(f, 0.0)) for f in FEATURES], path.exists()


class Assign(sched_standin.Scheduler):
    """Replaces sched_standin.plan; sched_verify.plan's super().plan(s) lands here (see Scheduler)."""

    def endpoints(self, s, task, taken):
        """Source and destination as sched_standin.plan picks them, or None if not possible now."""
        tt = task["task_type"]
        if tt == "PICK":
            src = self.source_of(s, task["item_id"], exclude=taken)
            dst = s.lane_free_slot(task["region"], exclude=taken)
            return (src, dst) if src and dst else None
        if tt == "PUTAWAY":
            return (task["source"], task["destination"]) if task["destination"] else None
        if tt == "RELOCATE":
            dst = task["destination"] or s.putaway_slot(task["item_id"], exclude=taken)
            return (task["source"], dst) if dst else None
        return (None, task["destination"])

    def task_features(self, s, task):
        tt, f = task["task_type"], dict.fromkeys(FEATURES, 0.0)
        f[{"PICK": "pick", "VERIFY": "verify"}.get(tt, "putaway")] = 1.0
        f["age"] = min((s.t - task["created_t"]) / 600.0, 2.0)
        if tt == "PICK":
            if task.get("deadline") is not None:
                f["slack"] = min(max((task["deadline"] - s.t) / 900.0, 0.0), 1.0)
            f["truck_here"] = float(s.lane_truck.get(task["region"], True))
            lane = s.lanes[task["region"]]
            f["lane_fill"] = sum(1 for x in lane if s.records[x]["item"] or s.records[x]["reserved_by"]) / len(lane)
        elif tt == "PUTAWAY":
            buf = [x for x, v in s.slots.items() if v["kind"] == "buffer"]
            f["buffer_fill"] = sum(1 for x in buf if s.records[x]["item"]) / max(len(buf), 1)
        return f

    def free_at(self, s, r):
        """Where a busy robot will be free and how far it still drives (m), or None."""
        task = s.task(r["task"] or r["assigned"])
        if task is None or task["status"] not in ("ASSIGNED", "IN_PROGRESS"):
            return None
        dst = s.pos(task["destination"])
        if task["source"] and not task["picked"] and not r["holding"]:
            return dst, s.dist(r["pos"], s.pos(task["source"])) + s.dist(s.pos(task["source"]), dst)
        return dst, s.dist(r["pos"], dst)

    def reroute_waiting(self, s):
        """sched_standin.plan's first part, unchanged: a task told to WAIT gets a destination
        when one frees up."""
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
                out.append({"robot_id": task["robot_id"], "task_id": tid, "action": "REROUTE",
                            "source": task["source"], "destination": new})
        return out, taken

    def plan(self, s):
        out, taken = self.reroute_waiting(s)
        wd, wb = self.w[0], self.w[1]
        idle = s.idle_robots()
        busy = [(r, *fa) for r in s.robots.values() if r not in idle for fa in [self.free_at(s, r)] if fa]
        cands = []
        for task in s.pending_tasks()[:MAX_TASKS]:
            ep = self.endpoints(s, task, taken)
            if ep is None:
                continue
            at = s.pos(ep[0] or ep[1])
            role = "patrol" if task["task_type"] == "VERIFY" and self.has_patrol else "transport"
            tf = self.task_features(s, task)
            base = sum(wi * tf[k] for wi, k in zip(self.w, FEATURES))
            cands += [(base + wd * s.dist(r["pos"], at) / 10.0, task, r["id"], False) for r in idle if r["role"] == role]
            cands += [(base + wd * (left + s.dist(end, at)) / 10.0 + wb, task, r["id"], True)
                      for r, end, left in busy if r["role"] == role]
        cands.sort(key=lambda c: -c[0])
        used_r, used_t = set(), set()
        for _, task, rid, is_busy in cands:
            if rid in used_r or task["task_id"] in used_t:
                continue
            ep = self.endpoints(s, task, taken)        # slots may have been taken by a better pair
            if ep is None:
                continue
            used_r.add(rid)
            used_t.add(task["task_id"])
            if not is_busy:                            # a busy robot's task waits for it
                taken.update(x for x in ep if x)
                out.append({"robot_id": rid, "task_id": task["task_id"], "action": "EXECUTE",
                            "source": ep[0], "destination": ep[1]})
        return out


class Scheduler(sched_verify.Scheduler, Assign):
    """MRO: Scheduler -> sched_verify -> Assign -> sched_standin, so sched_verify keeps its
    VERIFY work and hands the assignment to Assign.plan."""

    def __init__(self, cfg, weights=None):
        super().__init__(cfg)
        trained = True
        if weights is None:
            weights, trained = load_weights()
        self.w = list(weights)
        # Log folders: wms-learned-<policy> with trained weights, wms-heuristic-<policy> without.
        self.name = f"{'learned' if trained else 'heuristic'}-{self.policy}"
