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
import math
import os
import random

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
        # Which slot a periodic tick checks. sweep = fixed spatial order (the original rule); random,
        # oldest, value choose among recorded-occupied rack slots. Tick period/budget is the same for all.
        self.tick_rule = os.environ.get("VERIFY_TICK_RULE", "sweep").lower()
        if self.tick_rule not in ("sweep", "random", "oldest", "value"):
            raise ValueError(f"VERIFY_TICK_RULE: {self.tick_rule}")
        self.rate_source = os.environ.get("VERIFY_RATE_SOURCE", "online").lower()   # online | config
        self._cfg_rate_per_h = float(cfg.get("faults", {}).get("worker_move_per_h", 0)
                                     + cfg.get("faults", {}).get("worker_remove_per_h", 0))
        self._rng = random.Random(int(cfg.get("seed", 0)) * 7919 + 17)
        self.found = 0            # discrepancies seen by robots/checks (a lower bound of what happened)
        self.exposure = 0.0       # integral of recorded-occupied rack slots over time (slot-seconds)
        self._last_t = None
        self.pending_pred = {}    # slot -> (recorded item, p) of a tick check not yet seen
        self.calibration = []     # (p, mismatch 0/1) of each finished tick check

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
        if ev["type"] in EXCEPTIONS + ("DESTINATION_OCCUPIED",) and ev.get("observed") not in (None, "UNREACHABLE"):
            self.found += 1
        if ev["type"] == "OBSERVATION_RESULT" and ev.get("slot") in self.pending_pred:
            recorded, p = self.pending_pred.pop(ev["slot"])
            seen = ev.get("observed")
            if seen != "UNREADABLE":
                mismatch = int((None if seen == "EMPTY" else seen) != recorded)
                self.calibration.append((p, mismatch))
                self.found += mismatch
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
        if self._last_t is not None:
            self.exposure += (s.t - self._last_t) * sum(1 for sid in self.sweep_order
                                                        if s.records[sid]["item"] not in (None, "UNKNOWN"))
        self._last_t = s.t
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

    def _rate(self, n_occ):
        """Per-slot mismatch hazard (1/s). config: the scenario's worker S1+S2 rates spread over the occupied
        slots (known-rate reference, NOT deployable). online: Gamma(0.5, 3600*n) prior updated with the
        discrepancies found so far (underestimates: undetected ones are not counted)."""
        if self.rate_source == "config":
            return self._cfg_rate_per_h / 3600.0 / max(n_occ, 1)
        return (0.5 + self.found) / (3600.0 * max(n_occ, 1) + self.exposure / max(n_occ, 1))

    def _ranked_slot(self, s, open_tasks):
        checking = {t["destination"] for t in open_tasks.values()}
        demand = {}
        for t in s.tasks.values():
            if t["task_type"] == "PICK" and t["status"] not in FINAL:
                demand[t["item_id"]] = demand.get(t["item_id"], 0) + 1
        cands = []
        for sid in self.sweep_order:
            r = s.records[sid]
            if r["item"] is None or r["item"] == "UNKNOWN" or sid in checking or sid in self.wanted                     or r["reserved_by"] is not None:
                continue
            if r.get("observed_t") is not None and s.t - r["observed_t"] < self.recent:
                continue
            cands.append(sid)
        if not cands:
            return None
        n_occ = sum(1 for sid in self.sweep_order if s.records[sid]["item"] not in (None, "UNKNOWN"))
        lam = self._rate(n_occ)
        def age(sid):
            r = s.records[sid]
            return s.t - max(r["updated_t"], r["observed_t"] or 0.0)
        p = {sid: 1.0 - math.exp(-lam * age(sid)) for sid in cands}
        if self.tick_rule == "random":
            pick = self._rng.choice(cands)
        elif self.tick_rule == "oldest":
            pick = max(cands, key=lambda sid: (age(sid), sid))
        else:   # value: hazard x (1 + queued PICK demand for the recorded SKU, capped at 2)
            pick = max(cands, key=lambda sid: (p[sid] * (1 + min(demand.get(s.records[sid]["item"], 0), 2)), sid))
        self.pending_pred[pick] = (s.records[pick]["item"], p[pick])
        return pick

    def _next_sweep_slot(self, s, open_tasks):
        if self.tick_rule != "sweep":
            return self._ranked_slot(s, open_tasks)
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
