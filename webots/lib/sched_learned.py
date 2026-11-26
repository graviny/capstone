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
import math
import os
from pathlib import Path

import sched_standin
import sched_verify

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_WEIGHTS = ROOT / "models" / "assign_weights.json"
FEATURES = ("dist", "busy", "pick", "putaway", "verify", "slack", "age", "truck_here", "lane_fill", "buffer_fill",
            "eta")               # eta: est. time (min) until the robot can be at the task, busy robots included
HEURISTIC = {"dist": -1.0, "busy": -1.0, "pick": 2.0}
MAX_TASKS = 30                     # candidate tasks per step: the oldest startable ones (window 'feasible')


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
            if not task["destination"] or task["destination"] in taken:
                return None            # another decision of this step already uses the slot
            return task["source"], task["destination"]
        if tt == "RELOCATE":
            dst = task["destination"] or s.putaway_slot(task["item_id"], exclude=taken)
            return (task["source"], dst) if dst else None
        if task["destination"] in taken:
            return None                # VERIFY: slot reserved by a decision of this step
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
        """Where a busy robot will be free, how far it still drives (m) and how many stops
        (pick/place/verify) it still makes, or None."""
        task = s.task(r["task"] or r["assigned"])
        if task is None or task["status"] not in ("ASSIGNED", "IN_PROGRESS"):
            return None
        dst = s.pos(task["destination"])
        if task["source"] and not task["picked"] and not r["holding"]:
            return dst, s.dist(r["pos"], s.pos(task["source"])) + s.dist(s.pos(task["source"]), dst), 2
        return dst, s.dist(r["pos"], dst), 1

    def leg_s(self, m, stops=0):
        """Estimated seconds for m metres in one leg plus `stops` handlings, with lite_sim's
        timing (path / speed + 2 s per leg; look + act per stop). Straight-line m, so a lower bound."""
        speed, stop_s = self.timing
        return m / speed + 2.0 + stops * stop_s

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

    def update_load_weights(self, s):
        """Load-adaptive weights: pending tasks per robot, smoothed (EMA, time constant `adapt_tc` s), picks the
        weight set. Uses only the queue length the WMS shows; no truth, no future information."""
        dt = 0.0 if self._pt is None else max(s.t - self._pt, 0.0)
        self._pt = s.t
        inst = len(s.pending_tasks()) / max(len(s.robots), 1)
        a = 1.0 - math.exp(-dt / self.adapt_tc) if self._ema is not None else 1.0
        self._ema = inst if self._ema is None else self._ema + a * (inst - self._ema)
        self.w = self.w_high if self._ema >= self.adapt_tau else self.w_low

    def plan(self, s):
        if getattr(self, "w_high", None) is not None:
            self.update_load_weights(s)
        out, taken = self.reroute_waiting(s)
        wd, wb = self.w[0], self.w[1]
        we = self.w[FEATURES.index("eta")] if len(self.w) > FEATURES.index("eta") else 0.0
        idle = s.idle_robots()
        res = self.reserved                             # commit: task id -> busy robot that will do it next
        if self.commit:
            pend = {t["task_id"] for t in s.pending_tasks()}
            for tid in [t for t in res if t not in pend]:
                del res[tid]
            for r in list(idle):                         # a robot that just got free runs its reserved task
                tid = next((t for t, x in res.items() if x == r["id"]), None)
                if tid is None:
                    continue
                del res[tid]
                ep = self.endpoints(s, s.task(tid), taken)
                if ep:                                   # else the reservation lapses and it is planned normally
                    idle.remove(r)
                    taken.update(x for x in ep if x)
                    out.append({"robot_id": r["id"], "task_id": tid, "action": "EXECUTE",
                                "source": ep[0], "destination": ep[1]})
        booked = set(res.values())
        busy = [(r, *fa) for r in s.robots.values() if r not in idle and r["id"] not in booked
                for fa in [self.free_at(s, r)] if fa]
        cands, role_of = [], {}
        for task in self.candidate_tasks(s, taken):
            if task["task_id"] in res or any(o["task_id"] == task["task_id"] for o in out):
                continue
            ep = self.endpoints(s, task, taken)
            if ep is None:
                continue
            at = s.pos(ep[0] or ep[1])
            role = "patrol" if task["task_type"] == "VERIFY" and self.has_patrol else "transport"
            role_of[task["task_id"]] = role
            tf = self.task_features(s, task)
            if getattr(self, 'age_mode', 'learned') == 'zero':
                tf['age'] = 0.0            # ablation: old tasks are not made less attractive
            base = sum(wi * tf[k] for wi, k in zip(self.w, FEATURES))
            max_wait = getattr(self, 'max_wait', None)
            if max_wait is not None and s.t - task["created_t"] > max_wait:
                base += 100.0              # starvation guard: overdue tasks outrank every learned score
            cands += [(base + wd * s.dist(r["pos"], at) / 10.0 + we * self.leg_s(s.dist(r["pos"], at)) / 60.0,
                       task, r["id"], False) for r in idle if r["role"] == role]
            cands += [(base + wd * (left + s.dist(end, at)) / 10.0 + wb
                       + we * (self.leg_s(left, stops) + self.leg_s(s.dist(end, at))) / 60.0, task, r["id"], True)
                      for r, end, left, stops in busy if r["role"] == role]
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
            if is_busy:                                # a busy robot's task waits for it
                # ... and with commit nobody else may take it; commit "idle": only when it beat a free idle robot
                if self.commit and (self.commit != "idle" or any(
                        r["id"] not in used_r and r["role"] == role_of[task["task_id"]] for r in idle)):
                    res[task["task_id"]] = rid
            else:
                taken.update(x for x in ep if x)
                out.append({"robot_id": rid, "task_id": task["task_id"], "action": "EXECUTE",
                            "source": ep[0], "destination": ep[1]})
        return out

    def candidate_tasks(self, s, taken=()):
        tasks = s.pending_tasks()
        limit = getattr(self, 'candidate_limit', MAX_TASKS)
        if getattr(self, 'window', 'feasible') == 'feasible':   # default since 2026-11-20; 'raw' = first MAX_TASKS
            # Bounded compute like MAX_TASKS, but the window counts only tasks that can be
            # started now, so infeasible tasks at the queue head cannot hide the others.
            picked = []
            for task in tasks:
                if self.endpoints(s, task, taken) is not None:
                    picked.append(task)
                    if limit is not None and len(picked) >= limit:
                        break
            return picked
        selected = tasks if limit is None else tasks[:limit]
        if not hasattr(self, 'candidate_audit'):
            self.candidate_audit = {'calls': 0, 'excluded_task_steps': {}, 'max_excluded_age_s': {}}
        audit = self.candidate_audit
        audit['calls'] += 1
        for task in tasks[len(selected):]:
            kind = task['task_type']
            audit['excluded_task_steps'][kind] = audit['excluded_task_steps'].get(kind, 0) + 1
            audit['max_excluded_age_s'][kind] = max(audit['max_excluded_age_s'].get(kind, 0), s.t - task['created_t'])
        return selected


class Scheduler(sched_verify.Scheduler, Assign):
    """MRO: Scheduler -> sched_verify -> Assign -> sched_standin, so sched_verify keeps its
    VERIFY work and hands the assignment to Assign.plan."""

    def __init__(self, cfg, weights=None):
        super().__init__(cfg)
        trained, weights_from_env = True, weights is None and bool(os.environ.get("ASSIGN_WEIGHTS"))
        if weights is None:
            weights, trained = load_weights()
        self.w = list(weights)
        # Commit (ASSIGN_COMMIT=1): a task won by a busy robot is reserved for it, so it is that robot's
        # next task. shortcut: a reservation only lapses when the task leaves PENDING or cannot start
        # when the robot is free; no timeout if the robot is held up for long.
        # ASSIGN_COMMIT=idle: reserve only when the busy robot won against an idle robot that was still free.
        self.commit = {"1": True, "idle": "idle"}.get(os.environ.get("ASSIGN_COMMIT", ""), False)
        self.reserved = {}
        nav, per = cfg["nav"], cfg["perception"]
        self.timing = (nav["speed_mps"], 2.0 + per["look_s"] + nav["act_s"])   # leg_s(): speed, s per stop
        self.w_low = self.w_high = None            # set both (and adapt_tau) for load-adaptive weights
        self.adapt_tau, self.adapt_tc, self._ema, self._pt = 3.5, 300.0, None, None
        # Log folders: wms-learned-<policy> with trained weights, wms-heuristic-<policy> without.
        self.name = f"{'learned' if trained else 'heuristic'}-{self.policy}"
        if trained and weights_from_env:               # two weight files can run side by side
            self.name = f"learned-{Path(os.environ['ASSIGN_WEIGHTS']).stem}-{self.policy}"
        if self.commit:
            self.name += "-commit" + ("-idle" if self.commit == "idle" else "")
