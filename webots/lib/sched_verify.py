"""Experiment scheduler: the stand-in policy plus a simple verification policy.

For the "do we need a patrol robot?" experiment (docs/experiments_patrol.md). Not the
team's baseline / proposed WES: it only gives every robot mix the SAME verification rule,
so that comparing mixes compares who does the checking, not whether anyone does.

    WES_MODULE=wes_runner WES_SCHEDULER=sched_verify VERIFY_POLICY=event

VERIFY_POLICY
  none      the stand-in as it is (exceptions are rerouted, nothing is checked)
  event     after a SHORT_PICK / ITEM_MISMATCH, VERIFY the slots next to it on the same face
            (a worker moves a carton up to worker_move_max_slots); after a
            DESTINATION_OCCUPIED, VERIFY that slot (its record is UNKNOWN)
  periodic  every VERIFY_PERIOD_S, VERIFY the rack slot checked longest ago (cycle count)
  both      event + periodic

Who checks: the stand-in gives a VERIFY to the nearest idle patrol robot, or to the nearest
idle transport robot when there is no patrol robot (PICKs go first, by priority).

Other settings (environment): VERIFY_NEIGHBORS (default: the scenario's
worker_move_max_slots), VERIFY_PERIOD_S (60), VERIFY_MAX_OPEN (4: at most this many open
VERIFY tasks from this policy), VERIFY_RECENT_S (120: skip a slot checked this recently).
"""
import os

import sched_standin

EXCEPTIONS = ("SHORT_PICK", "ITEM_MISMATCH")
FINAL = ("COMPLETED", "CANCELLED", "FAILED")


def _env(name, default, cast=float):
    v = os.environ.get(name)
    return cast(v) if v not in (None, "") else default


class Scheduler(sched_standin.Scheduler):
    def __init__(self, cfg):
        super().__init__(cfg)
        self.policy = os.environ.get("VERIFY_POLICY", "event").lower()
        if self.policy not in ("none", "event", "periodic", "both"):
            raise ValueError(f"VERIFY_POLICY: {self.policy}")
        self.name = f"verify-{self.policy}"
        self.neighbors = _env("VERIFY_NEIGHBORS", int(cfg.get("faults", {}).get("worker_move_max_slots", 3)), int)
        self.period = _env("VERIFY_PERIOD_S", 60.0)
        self.max_open = _env("VERIFY_MAX_OPEN", 4, int)
        self.recent = _env("VERIFY_RECENT_S", 120.0)

        # Rack faces in slot order, for neighbors and the cycle-count sweep.
        faces = {}
        for s in cfg["slots"]:
            if s["kind"] == "rack":
                faces.setdefault(s["face"], []).append(s["id"])
        self.face_of, self.index_of, self.faces = {}, {}, {}
        for f, ids in faces.items():
            ids.sort()
            self.faces[f] = ids
            for k, sid in enumerate(ids):
                self.face_of[sid], self.index_of[sid] = f, k
        self.sweep_order = [sid for f in sorted(self.faces) for sid in self.faces[f]]
        self.sweep_k = 0
        self.next_periodic = None
        self.wanted = []          # slots to check, oldest first
        self.mine = set()         # task ids of the VERIFY tasks this policy created
        self._n_seen = 0
        self.created = 0

    # ------------------------------------------------------------------ events
    def on_event(self, ev, s):
        out = super().on_event(ev, s)
        if self.policy in ("event", "both") and ev.get("slot") in self.face_of:
            sid = ev["slot"]
            if ev["type"] in EXCEPTIONS:
                self._want(self._neighbors(sid))
            elif ev["type"] == "DESTINATION_OCCUPIED" and ev.get("observed") != "UNREACHABLE":
                self._want([sid])
        return out

    def _neighbors(self, sid):
        ids, k = self.faces[self.face_of[sid]], self.index_of[sid]
        near = []
        for d in range(1, self.neighbors + 1):          # closest first
            for j in (k - d, k + d):
                if 0 <= j < len(ids):
                    near.append(ids[j])
        return near

    def _want(self, sids):
        for sid in sids:
            if sid not in self.wanted:
                self.wanted.append(sid)

    # -------------------------------------------------------------------- plan
    def plan(self, s):
        self._track(s)
        out = []
        open_tasks = {}
        for tid in list(self.mine):
            task = s.task(tid)
            if task is None or task["status"] in FINAL:
                self.mine.discard(tid)
            else:
                open_tasks[tid] = task
        # A pending check whose slot another task now uses: that robot looks at it anyway.
        for tid, task in open_tasks.items():
            r = s.records[task["destination"]]
            if task["status"] == "PENDING" and r["reserved_by"] not in (None, tid):
                out.append({"robot_id": None, "task_id": tid, "action": "CANCEL"})
        if out:
            return out         # the stand-in plans next step, without these tasks

        if self.policy in ("periodic", "both"):
            if self.next_periodic is None:
                self.next_periodic = s.t + self.period
            elif s.t >= self.next_periodic:
                self.next_periodic += self.period
                sid = self._next_sweep_slot(s, open_tasks)
                if sid:
                    self._want([sid])

        checking = {t["destination"] for t in open_tasks.values()}
        while self.wanted and len(open_tasks) + len(out) < self.max_open:
            sid = self.wanted.pop(0)
            r = s.records[sid]
            if sid in checking or r["reserved_by"] is not None:
                continue
            if r.get("observed_t") is not None and s.t - r["observed_t"] < self.recent:
                continue
            checking.add(sid)
            self.created += 1
            out.append({"robot_id": None, "task_id": None, "action": "CREATE",
                        "task": {"task_type": "VERIFY", "destination": sid, "policy": self.policy}})
        if out:
            return out         # the new tasks get created first; the stand-in assigns them next step

        return super().plan(s)

    def _next_sweep_slot(self, s, open_tasks):
        checking = {t["destination"] for t in open_tasks.values()}
        for _ in range(len(self.sweep_order)):
            sid = self.sweep_order[self.sweep_k]
            self.sweep_k = (self.sweep_k + 1) % len(self.sweep_order)
            r = s.records[sid]
            if sid in checking or sid in self.wanted or r["reserved_by"] is not None:
                continue
            if r.get("observed_t") is not None and s.t - r["observed_t"] < self.recent:
                continue
            return sid
        return None

    def _track(self, s):
        """The runner does not hand the new task ids back, so find this policy's tasks by their
        tag ("policy") among the tasks created since the last step."""
        if len(s.tasks) > self._n_seen:
            for tid in list(s.tasks)[self._n_seen:]:
                if s.tasks[tid].get("policy") == self.policy:
                    self.mine.add(tid)
            self._n_seen = len(s.tasks)
