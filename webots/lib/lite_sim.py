"""Lightweight simulator: the Webots warehouse without physics, for fast runs (RL training,
parameter sweeps). It reads the same scenario.json, drives the same WES interface
(on_event / decide / record_snapshot / stats) with the same events and decisions, and writes
the same summary.json fields. So a scheduler trained or tuned here runs unchanged in Webots
(WES_MODULE=wes_runner WES_SCHEDULER=<module>), which is where results are checked.

What is the same as Webots (controllers/supervisor + agent): orders, inbound trucks and belt,
outbound docks and cutoffs, the worker faults (S1 move, S2 removal, S3 return into a robot's
destination), the robot's task phases (go, look, pick, go, look, place) and its reports.

What is simpler:
  robots   no wheels, no collision avoidance: a leg takes path length / speed * slowdown
           + leg_s + yield_s per robot on the move that has the right of way over this one
           (nav.traffic_priority: by task or by id, as agent.py). leg_s stands for turning,
           yield_s for giving way; calibrated against 47 Webots runs: slowdown 1.0, leg_s 2,
           yield_s 1 (docs/lite_sim.md)
  time     fixed steps of dt (0.5 s); a robot's phase still ends at its exact time
  looking  a robot always reads the label (no UNREADABLE)
  shortcut: S1 robot placement error, scanner failure and the patrol idle sweep are not
           modelled (all off by default); add them here if an experiment turns them on.

Usage (from webots/):
  python lib/lite_sim.py [--scheduler sched_verify --policy event] [--duration 3600]
                         [--seeds 0 1 2] [--slowdown 1.0 --leg-s 2 --yield-s 1]
In code (e.g. an RL scheduler):
  from lite_sim import LiteSim
  summary = LiteSim(cfg, wes).run()        # wes: wes_runner.WES(cfg, scheduler, log_dir)
"""
import argparse
from array import array
import heapq
import importlib
import json
import math
import os
import random
import sys
import time
import zlib
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "lib"))
from nav import NEIGHBORS, GridMap  # noqa: E402
from record_quality import RecordQuality
from research_metrics import FlowMetrics, order_metrics, task_metrics
from fault_tape import FaultTape, apply_worker_attempt

STATS_PERIOD_S = 10.0
IDLE_PARK_S = 20.0                 # agent.py: an idle robot drives home after this long
S3_DELAY_S = 4.0                   # supervisor.py: the returned carton lands 4 s after the pick
NO_TASK = [2, 9, 1e9]              # agent.py: right-of-way key of a robot without a task


def unit(*parts):
    """Same deterministic draw as supervisor.py (S3 is keyed by task and destination)."""
    return zlib.crc32("-".join(str(p) for p in parts).encode()) / 2 ** 32


def distance_field(grid, goal):
    """Grid steps from every reachable cell to goal (Dijkstra; same moves as GridMap.plan)."""
    occ, nx, ny = grid.occ, grid.nx, grid.ny

    def free(i, j):
        return 0 <= i < nx and 0 <= j < ny and not occ[i][j]

    dist, heap = {goal: 0.0}, [(0.0, goal)]
    while heap:
        d, (i, j) = heapq.heappop(heap)
        if d > dist[(i, j)]:
            continue
        for di, dj, w in NEIGHBORS:
            n = (i + di, j + dj)
            if not free(*n) or (di and dj and not (free(i + di, j) and free(i, j + dj))):
                continue
            if d + w < dist.get(n, math.inf):
                dist[n] = d + w
                heapq.heappush(heap, (d + w, n))
    return dist


class LiteSim:
    def __init__(self, cfg, wes, dt=0.5, slowdown=1.0, leg_s=2.0, yield_s=1.0, grid=None, fields=None, event_sink=None):
        """grid, fields: pass the same GridMap and dict to many episodes of one layout, so the
        distance fields are computed once."""
        self.cfg, self.wes, self.dt = cfg, wes, dt
        self.event_sink = event_sink
        self.slowdown, self.leg_s, self.yield_s = slowdown, leg_s, yield_s
        self.by_task = cfg["nav"].get("traffic_priority", "id") == "task"
        self.grid = grid or GridMap(cfg)
        self.fields = {} if fields is None else fields   # goal cell -> distance field
        self.slots = {s["id"]: s for s in cfg["slots"]}
        self.faults, self.belt_cfg, self.out_cfg = cfg["faults"], cfg["belt"], cfg["outbound"]
        self.nav, self.per = cfg["nav"], cfg["perception"]
        self.seed = cfg["seed"]
        self.worker_rng = random.Random(self.seed + 101)
        self.fault_tape = FaultTape(cfg) if cfg.get('fault_model') == 'marked-poisson-v1' else None
        self.flow_metrics = FlowMetrics()
        self.t = 0.0

        # Ground truth: which carton is where.
        self.sku_of = {c["lpn"]: c["sku"] for c in cfg["cartons"]}
        self.occupant = {s: None for s in self.slots}
        self.slot_of = {}
        for st in cfg["initial_stock"]:
            self.occupant[st["slot"]], self.slot_of[st["lpn"]] = st["lpn"], st["slot"]
        self.returns = [c["lpn"] for c in cfg["returns"]]
        self.lane_tag = {}
        self.s3_pending = []

        self.robots = {r["id"]: {"id": r["id"], "role": r["role"], "home": tuple(r["home"]), "pos": tuple(r["home"]),
                                 "task": None, "phase": None, "holding": None, "leg": None, "timer": None,
                                 "idle_since": 0.0, "parked": True, "busy_s": 0.0, "dist": 0.0}
                       for r in cfg["robots"]}
        self.task_src, self.task_dest = {}, {}

        self.waves = sorted(self.out_cfg["waves"], key=lambda w: w["release_t"])
        self.next_wave = 0
        self.orders = {}
        for w in self.waves:
            for ln in w["lines"]:
                o = self.orders.setdefault(ln["order"], {"units": 0, "shipped": 0, "released_t": w["release_t"],
                                                         "done_t": None})
                o["units"] += ln["qty"]
        self.trucks = sorted(cfg["inbound"]["trucks"], key=lambda tr: tr["arrive_t"])
        self.next_truck, self.dock, self.unload_at = 0, None, 0.0
        self.zones = self.belt_cfg["zones"]
        self.zone_x = {z: self.slots[z]["pos"][0] for z in self.zones}
        self.belt = []
        self.lanes = self.out_cfg["lanes"]
        self.lane_state = {r: {"present": True, "loading_until": None, "back_at": None,
                               "next_cutoff": self.out_cfg["cutoff_period_s"]} for r in self.lanes}
        self.quality = RecordQuality()
        self.n = {"events": 0, "released": 0, "shipped": 0, "mis_shipped": 0, "missed_cutoffs": 0,
                  "shipments": 0, "staged_in": 0}
        self.counts, self.truth_counts = {}, {}
        self.unit_wait = {}                # order -> release times of its units not shipped yet
        self.unit_leads = []               # release -> shipped, s, per shipped unit

    # ------------------------------------------------------------ helpers
    def gt_sku(self, sid):
        lpn = self.occupant[sid]
        return self.sku_of[lpn] if lpn else None

    def put(self, lpn, sid):
        self.occupant[sid], self.slot_of[lpn] = lpn, sid

    def take(self, lpn):
        sid = self.slot_of.pop(lpn, None)
        if sid:
            self.occupant[sid] = None

    def emit(self, ev, t=None):
        self.n["events"] += 1
        ev = {"event_id": f"E{self.n['events']:06d}", "t": round(self.t if t is None else t, 2), **ev}
        self.counts[ev["type"]] = self.counts.get(ev["type"], 0) + 1
        self.flow_metrics.observe(ev)
        if self.event_sink:
            self.event_sink(ev)
        self.wes.on_event(ev)

    def truth(self, kind):
        self.truth_counts[kind] = self.truth_counts.get(kind, 0) + 1

    def path_m(self, a, b):
        """Shortest grid path length from a to b (8-connected, octile: a bit longer than the
        pruned A* path the robots drive; slowdown absorbs it)."""
        g = self.grid.nearest_free(self.grid.to_cell(b), set())
        ny = self.grid.ny
        if g not in self.fields:   # flat array, not the dict: the large layout's fields overflow RAM per worker
            f = array("d", [math.inf]) * (self.grid.nx * ny)
            for (i, j), d in distance_field(self.grid, g).items():
                f[i * ny + j] = d
            self.fields[g] = f
        i, j = self.grid.nearest_free(self.grid.to_cell(a), set())
        d = self.fields[g][i * ny + j]
        return d * self.grid.res if d != math.inf else math.dist(a, b)

    # ------------------------------------------------------------- robots
    def where(self, r, t):
        """Position now; on a leg, along the straight line between its ends."""
        leg = r["leg"]
        if leg is None:
            return r["pos"]
        f = min(1.0, (t - leg["t0"]) / max(leg["t1"] - leg["t0"], 1e-9))
        return tuple(a + (b - a) * f for a, b in zip(leg["from"], leg["to"]))

    def stop(self, r, t):
        leg = r["leg"]
        if leg is not None:
            r["pos"] = self.where(r, t)
            r["dist"] += leg["m"] * min(1.0, (t - leg["t0"]) / max(leg["t1"] - leg["t0"], 1e-9))
            r["leg"] = None

    def right_of_way(self, r):
        """agent.py prio_key + id (lower goes first): by task, or by id in older scenarios."""
        task = r["task"]
        if not self.by_task or task is None or r["phase"] == "wait":
            k = NO_TASK
        else:
            dl = task.get("deadline")
            k = [0 if r["holding"] else 1, task.get("priority") or 9, dl if dl is not None else 1e9]
        return tuple(k) + (r["id"],)

    def drive(self, r, target, t):
        self.stop(r, t)
        m = self.path_m(r["pos"], target)
        # Giving way: yield_s for every robot on the move that has the right of way over this one.
        mine = self.right_of_way(r)
        ahead = sum(1 for o in self.robots.values()
                    if o is not r and o["leg"] and o["leg"]["t1"] > t and self.right_of_way(o) < mine)
        dur = m / self.nav["speed_mps"] * self.slowdown + self.leg_s + self.yield_s * ahead
        r["leg"] = {"from": r["pos"], "to": tuple(target), "t0": t, "t1": t + dur, "m": m}
        r["timer"] = t + dur

    def go_slot(self, r, sid, phase, t):
        r["phase"] = phase
        self.drive(r, self.slots[sid]["approach"], t)

    def finish(self, r, t):
        self.stop(r, t)
        r.update(task=None, phase=None, timer=None, parked=False, idle_since=t)

    def wait(self, r, t):
        self.stop(r, t)
        r.update(phase="wait", timer=None)

    def on_decision(self, d, t):
        r = self.robots[d["robot_id"]]
        action = d["action"]
        if action == "EXECUTE":
            if r["task"] is not None:
                self.emit({"type": "TASK_FAILED", "robot_id": r["id"], "task_id": d["task_id"], "reason": "BUSY"}, t)
                return
            r["task"] = task = dict(d["task"])
            if task["task_type"] == "VERIFY":
                self.go_slot(r, task["destination"], "go_verify", t)
            else:
                self.go_slot(r, task["source"], "go_src", t)
            return
        task = r["task"]
        if task is None or d["task_id"] != task["task_id"]:
            return
        if action == "REROUTE":
            task["source"] = d.get("source", task["source"])
            task["destination"] = d.get("destination", task["destination"])
            if r["holding"]:
                self.go_slot(r, task["destination"], "go_dst", t)
            else:
                self.go_slot(r, task["source"], "go_src", t)
        elif action == "WAIT":
            self.wait(r, t)
        elif action in ("CANCEL", "REASSIGN"):
            if r["holding"]:
                self.emit({"type": "TASK_FAILED", "robot_id": r["id"], "task_id": task["task_id"],
                           "reason": "HOLDING_CARTON", "item_id": self.sku_of[r["holding"]]}, t)
            else:
                self.finish(r, t)

    def phase_end(self, r, t):
        """What agent.py does when a phase is over (arrived, looked, picked, placed)."""
        task, phase, rid = r["task"], r["phase"], r["id"]
        if phase is None:                                          # parked
            self.stop(r, t)
            r["timer"] = None
            return
        if phase.startswith("go_"):
            self.stop(r, t)
            r["phase"] = phase.replace("go_", "look_")
            r["timer"] = t + self.per["look_s"]
            return
        sid = task["destination"] if phase in ("look_dst", "look_verify", "place") else task["source"]
        seen = self.gt_sku(sid) or "EMPTY"
        tid = task["task_id"]
        if phase == "look_verify":
            self.emit({"type": "OBSERVATION_RESULT", "robot_id": rid, "task_id": tid, "slot": sid, "observed": seen,
                       "source": "verify"}, t)
            self.emit({"type": "TASK_COMPLETED", "robot_id": rid, "task_id": tid, "task_type": "VERIFY", "slot": sid,
                       "item_id": seen}, t)
            self.finish(r, t)
        elif phase == "look_src":
            if seen == task["item_id"]:
                r.update(phase="pick", timer=t + self.nav["act_s"])
            else:
                self.emit({"type": "SHORT_PICK" if seen == "EMPTY" else "ITEM_MISMATCH", "robot_id": rid,
                           "task_id": tid, "slot": sid, "item_id": task["item_id"], "observed": seen}, t)
                self.wait(r, t)
        elif phase == "look_dst":
            if seen == "EMPTY":
                r.update(phase="place", timer=t + self.nav["act_s"])
            else:
                self.emit({"type": "DESTINATION_OCCUPIED", "robot_id": rid, "task_id": tid, "slot": sid,
                           "item_id": self.sku_of[r["holding"]], "observed": seen}, t)
                self.wait(r, t)
        elif phase == "pick":
            lpn = self.occupant[sid]
            if lpn is None:                    # taken between looking and picking
                self.emit({"type": "SHORT_PICK", "robot_id": rid, "task_id": tid, "slot": sid,
                           "item_id": task["item_id"], "observed": "EMPTY"}, t)
                self.wait(r, t)
                return
            self.take(lpn)
            r["holding"] = lpn
            dest = self.task_dest.get(tid)
            if dest and self.slots[dest]["kind"] == "rack":
                if self.fault_tape:
                    event = self.fault_tape.s3(lpn, t, self.faults['dest_taken_p'])
                    if event:
                        self.s3_pending.append((t + S3_DELAY_S, dest, event))
                elif unit(self.seed, tid, dest, "dest") < self.faults["dest_taken_p"]:
                    self.s3_pending.append((t + S3_DELAY_S, dest, tid))
            self.go_slot(r, task["destination"], "go_dst", t)
        elif phase == "place":
            if self.occupant[sid] is not None:
                self.emit({"type": "DESTINATION_OCCUPIED", "robot_id": rid, "task_id": tid, "slot": sid,
                           "item_id": self.sku_of[r["holding"]], "observed": "UNKNOWN"}, t)
                self.wait(r, t)
                return
            lpn = r["holding"]
            self.put(lpn, sid)
            if self.slots[sid]["kind"] == "outbound":
                self.lane_tag[lpn] = {"order": task.get("order"), "item": task["item_id"],
                                      "region": self.slots[sid]["lane"]}
            self.emit({"type": "TASK_COMPLETED", "robot_id": rid, "task_id": tid, "task_type": task["task_type"],
                       "item_id": self.sku_of[lpn], "slot": sid}, t)
            r["holding"] = None
            self.finish(r, t)

    # -------------------------------------------------------- environment
    def environment(self, t):
        """supervisor.py's loop body before the robots: orders, inbound, belt, outbound, faults."""
        while self.next_wave < len(self.waves) and self.waves[self.next_wave]["release_t"] <= t:
            w = self.waves[self.next_wave]
            self.next_wave += 1
            self.n["released"] += sum(ln["qty"] for ln in w["lines"])
            for ln in w["lines"]:
                self.unit_wait.setdefault(ln["order"], []).extend([w["release_t"]] * ln["qty"])
            self.emit({"type": "ORDER_RELEASED", "wave": w["wave"], "lines": w["lines"]})

        bc = self.belt_cfg
        if self.dock is None and self.next_truck < len(self.trucks) and self.trucks[self.next_truck]["arrive_t"] <= t:
            tr = self.trucks[self.next_truck]
            self.dock = {"truck": tr["truck"], "queue": list(tr["cartons"])}
            self.next_truck += 1
            self.unload_at = t
            self.emit({"type": "TRUCK_ARRIVED", "truck": tr["truck"],
                       "asn": [{"lpn": c["lpn"], "item": c["sku"]} for c in tr["cartons"]]})
        if self.dock is not None:
            entry_clear = all(b["x"] > bc["x_entry"] + bc["gap_m"] for b in self.belt)
            if self.dock["queue"] and t >= self.unload_at and entry_clear:
                c = self.dock["queue"].pop(0)
                self.belt.append({"lpn": c["lpn"], "x": bc["x_entry"]})
                self.unload_at = t + bc["unload_s"]
            if not self.dock["queue"]:
                self.emit({"type": "TRUCK_DEPARTED", "truck": self.dock["truck"]})
                self.dock = None

        # Belt: each carton heads for the east-most free zone ahead of it (supervisor.py).
        self.belt.sort(key=lambda b: -b["x"])
        claimed, prev_x, done = set(), None, []
        for b in self.belt:
            limit = bc["x_end"] if prev_x is None else prev_x - bc["gap_m"]
            free = [z for z in self.zones if self.zone_x[z] >= b["x"] - 0.25 and not self.occupant[z]
                    and z not in claimed]
            target = max(free, key=lambda z: self.zone_x[z]) if free else None
            if target:
                claimed.add(target)
                goal = min(max(self.zone_x[target], b["x"]), limit)
            else:
                goal = limit
            if goal > b["x"]:
                b["x"] = min(goal, b["x"] + bc["speed_mps"] * self.dt)
            prev_x = b["x"]
            if target and b["x"] >= self.zone_x[target] - 1e-6:
                self.put(b["lpn"], target)
                done.append(b)
                self.n["staged_in"] += 1
                self.emit({"type": "INBOUND_STAGED", "slot": target, "lpn": b["lpn"], "item": self.sku_of[b["lpn"]]})
        for b in done:
            self.belt.remove(b)

        oc = self.out_cfg
        for region, ls in self.lane_state.items():
            staged = [s for s in self.lanes[region]["slots"] if self.occupant[s]]
            cutoff_now = False
            if t >= ls["next_cutoff"]:
                if staged and not ls["present"]:
                    self.n["missed_cutoffs"] += 1
                cutoff_now = bool(staged) and ls["present"]
                ls["next_cutoff"] += oc["cutoff_period_s"]
            if ls["present"] and ls["loading_until"] is None and (len(staged) >= oc["dispatch_cartons"] or cutoff_now):
                ls["loading_until"] = t + oc["load_s"]
            if ls["loading_until"] is not None and t >= ls["loading_until"]:
                cartons = []
                for s in staged:
                    lpn = self.occupant[s]
                    tag = self.lane_tag.pop(lpn, {})
                    cartons.append({"slot": s, "lpn": lpn, "item": self.sku_of[lpn], "order": tag.get("order")})
                    if tag.get("item") == self.sku_of[lpn] and tag.get("region") == region and tag.get("order") in self.orders:
                        o = self.orders[tag["order"]]
                        o["shipped"] += 1
                        self.n["shipped"] += 1
                        if self.unit_wait.get(tag["order"]):
                            self.unit_leads.append(t - self.unit_wait[tag["order"]].pop(0))
                        if o["shipped"] >= o["units"]:
                            o["done_t"] = t
                    else:
                        self.n["mis_shipped"] += 1
                    self.take(lpn)
                self.n["shipments"] += 1
                ls.update(present=False, loading_until=None, back_at=t + oc["turnaround_s"])
                self.emit({"type": "OUTBOUND_DEPARTED", "region": region, "cartons": cartons})
            if not ls["present"] and t >= ls["back_at"]:
                ls["present"] = True
                self.emit({"type": "OUTBOUND_TRUCK_ARRIVED", "region": region})

        self.worker_events()
        for item in [p for p in self.s3_pending if p[0] <= t]:
            self.s3_pending.remove(item)
            _, dest, event = item
            if self.occupant[dest] is None and self.returns:
                self.put(self.returns.pop(0), dest)
                self.truth("S3_DEST_TAKEN")
                if self.fault_tape:
                    self.fault_tape.record(event, t, 'injected', dst=dest)
            elif self.fault_tape:
                self.fault_tape.record(event, t, 'skipped_occupied' if self.occupant[dest] else 'skipped_no_returns', dst=dest)

    def worker_events(self):
        if self.fault_tape:
            for event in self.fault_tape.due(self.t):
                result = apply_worker_attempt(event, self.t, self.slots, self.occupant, self.slot_of,
                    self.faults['worker_move_max_slots'], self.take, self.put, self.take, self.fault_tape)
                if result:
                    self.truth('S1_UNRECORDED_MOVE' if event['kind'] == 'S1' else 'S2_UNRECORDED_REMOVAL')
            return
        f, rng = self.faults, self.worker_rng
        rack_lpns = [l for l, s in self.slot_of.items() if self.slots[s]["kind"] == "rack"]
        if not rack_lpns:
            return
        if rng.random() < f["worker_move_per_h"] / 3600 * self.dt:
            lpn = rng.choice(rack_lpns)
            src = self.slot_of[lpn]
            idx, face = int(src[3:]), src[:3]
            k = f["worker_move_max_slots"]
            free = [f"{face}{idx + d:02d}" for d in range(-k, k + 1)
                    if d and self.occupant.get(f"{face}{idx + d:02d}", "x") is None]
            if free:
                self.take(lpn)
                self.put(lpn, rng.choice(free))
                self.truth("S1_UNRECORDED_MOVE")
        if rng.random() < f["worker_remove_per_h"] / 3600 * self.dt:
            self.take(rng.choice(rack_lpns))
            self.truth("S2_UNRECORDED_REMOVAL")

    # --------------------------------------------------------------- loop
    def step(self):
        t = self.t = self.t + self.dt
        self.environment(t)
        # Robot phases that ended during this step, in time order.
        while True:
            due = [r for r in self.robots.values() if r["timer"] is not None and r["timer"] <= t]
            if not due:
                break
            r = min(due, key=lambda r: (r["timer"], r["id"]))
            at, r["timer"] = r["timer"], None
            self.phase_end(r, at)
        for r in self.robots.values():
            if r["task"] is not None:
                r["busy_s"] += self.dt
            elif not r["parked"] and r["leg"] is None and t - r["idle_since"] >= IDLE_PARK_S:
                r["parked"] = True
                self.drive(r, r["home"], t)
        info = [{"id": r["id"], "role": r["role"], "pos": list(self.where(r, t)),
                 "task": r["task"]["task_id"] if r["task"] else None,
                 "holding": self.sku_of[r["holding"]] if r["holding"] else None, "phase": r["phase"]}
                for r in self.robots.values()]
        for d in self.wes.decide(t, info):
            if self.event_sink:
                self.event_sink({'t': round(t, 2), 'kind': 'DECISION', **d})
            if d.get("robot_id") is None:
                continue
            if d["action"] == "EXECUTE":
                self.task_dest[d["task_id"]] = d["task"].get("destination")
                self.task_src[d["task_id"]] = d["task"].get("source")
            elif d["action"] == "REROUTE":
                self.task_dest[d["task_id"]] = d.get("destination")
                self.task_src[d["task_id"]] = d.get("source", self.task_src.get(d["task_id"]))
            self.on_decision(d, t)
        if t + 1e-9 >= self.next_stats:
            self.record_quality(t)
            self.next_stats = t + STATS_PERIOD_S

    def record_quality(self, t):
        rec = self.wes.record_snapshot()
        wrong = {s for s, v in rec.items() if v != self.gt_sku(s)}
        busy = {x for r in self.robots.values() if r["task"]
                for x in (self.task_src.get(r["task"]["task_id"]), self.task_dest.get(r["task"]["task_id"])) if x}
        self.quality.sample(t, wrong, busy)

    def run(self, duration=None):
        duration = duration or self.cfg["episode"]["duration_s"]
        self.next_stats = 0.0
        while self.t + 1e-9 < duration:
            self.step()
        self.record_quality(self.t)
        return self.summary()

    def summary(self):
        t = self.t
        done = [o for o in self.orders.values() if o["done_t"] is not None]
        rec = self.wes.record_snapshot()
        # Censored leads: whatever is not shipped by the end counts until the end, so runs that
        # complete different orders are still compared on the same orders / units.
        released = [o for o in self.orders.values() if o["released_t"] <= t]
        lead_all = sorted((o["done_t"] if o["done_t"] is not None else t) - o["released_t"] for o in released)
        period = self.out_cfg["cutoff_period_s"]
        tard = [max(0.0, (o["done_t"] if o["done_t"] is not None else t)
                    - (o["released_t"] // period + 1) * period) for o in released]
        units = self.unit_leads + [t - r for rs in self.unit_wait.values() for r in rs]
        return {
            "duration_s": round(t, 1), "wes": self.wes.name, "simulator": "lite",
            "orders_released": sum(1 for o in self.orders.values() if o["released_t"] <= t),
            "orders_completed": len(done),
            "mean_order_lead_s": sum(o["done_t"] - o["released_t"] for o in done) / len(done) if done else None,
            "order_lead_all_s": sum(lead_all) / len(lead_all) if lead_all else None,
            "order_lead_p90_s": lead_all[int(0.9 * (len(lead_all) - 1))] if lead_all else None,
            "order_tardiness_s": sum(tard) / len(tard) if tard else None,
            "unit_lead_s": sum(units) / len(units) if units else None,
            "units_released": self.n["released"], "units_shipped": self.n["shipped"],
            "mis_shipped": self.n["mis_shipped"], "outbound_trucks": self.n["shipments"],
            "missed_cutoffs": self.n["missed_cutoffs"], "inbound_trucks_arrived": self.next_truck,
            "inbound_staged": self.n["staged_in"], "cartons_on_belt_at_end": len(self.belt),
            "events": self.counts, "faults_injected": dict(self.truth_counts), "diagnostics": {},
            "record_mismatch_at_end": sum(1 for s, v in rec.items() if v != self.gt_sku(s)),
            "record_quality": self.quality.summary(t),
            "research_metrics": {**self.flow_metrics.summary(t), **order_metrics(self.orders, t),
                                 'tasks': task_metrics(self.wes.wms.tasks, t)},
            "fault_tape": self.fault_tape.summary() if self.fault_tape else None,
            "robot_roles": {r["id"]: r["role"] for r in self.robots.values()},
            "robot_distance_m": {r["id"]: round(r["dist"], 1) for r in self.robots.values()},
            "robot_busy_share": {r["id"]: round(r["busy_s"] / t, 3) for r in self.robots.values()},
            "wes_stats": self.wes.stats(),
        }


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scenario", type=Path, default=ROOT / "scenario.json")
    ap.add_argument("--scheduler", default="sched_standin", help="scheduler module in lib/")
    ap.add_argument("--policy", help="VERIFY_POLICY for sched_verify (none, event, periodic, both)")
    ap.add_argument("--seeds", type=int, nargs="+", help="override the scenario seed (worker faults, S3 draws)")
    ap.add_argument("--duration", type=float)
    ap.add_argument("--dt", type=float, default=0.5)
    ap.add_argument("--slowdown", type=float, default=1.0, help="travel time factor (turning, waiting on others)")
    ap.add_argument("--leg-s", type=float, default=2.0, help="extra seconds per leg (turning, slowing down)")
    ap.add_argument("--yield-s", type=float, default=1.0, help="seconds per moving robot with the right of way")
    args = ap.parse_args()
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    if args.policy:
        os.environ["VERIFY_POLICY"] = args.policy
    from wes_runner import WES, log_dir_for

    base = json.loads(args.scenario.read_text(encoding="utf-8"))
    grid, fields = GridMap(base), {}
    for seed in args.seeds or [base["seed"]]:
        cfg = {**base, "seed": seed}
        sched = importlib.import_module(args.scheduler).Scheduler(cfg)
        name = f"wms-{sched.name}"
        log_dir = Path(str(log_dir_for(cfg, name)).rsplit("_", 1)[0] + "_lite")
        wes = WES(cfg, scheduler=sched, log_dir=log_dir)
        t0 = time.time()
        s = LiteSim(cfg, wes, dt=args.dt, slowdown=args.slowdown, leg_s=args.leg_s, yield_s=args.yield_s, grid=grid,
                      fields=fields).run(args.duration)
        s["tag"] = log_dir.name
        s["lite"] = {"dt": args.dt, "slowdown": args.slowdown, "leg_s": args.leg_s, "yield_s": args.yield_s}
        (log_dir / "summary.json").write_text(json.dumps(s, indent=2), encoding="utf-8")
        busy = [b for r, b in s["robot_busy_share"].items() if s["robot_roles"][r] == "transport"]
        lead = s["mean_order_lead_s"]
        print(f"seed {seed}: shipped {s['units_shipped']}/{s['units_released']}, lead {lead and round(lead)} s, "
              f"busy {sum(busy) / len(busy):.3f}, dist {sum(s['robot_distance_m'].values()):.0f} m, "
              f"SHORT_PICK {s['events'].get('SHORT_PICK', 0)}, DEST_OCC {s['events'].get('DESTINATION_OCCUPIED', 0)}, "
              f"missed {s['missed_cutoffs']}, mismatch {s['record_quality']['settled_mismatch_mean']} "
              f"| {time.time() - t0:.1f} s -> {log_dir.name}")


if __name__ == "__main__":
    main()
