# Robot Place Failure Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Robots sometimes put a rack carton one slot off; the robot retries twice, then the WMS records it and creates a mandatory fix task (RELOCATE) that another robot takes over, and a failed takeover quarantines the carton.

**Architecture:** One shared draw (`lib/place_fail.py`) decides each place attempt's outcome and duration from (seed, LPN, place count, attempt kind). The supervisor (Webots) and the lite sim both call it. The robot logic (agent.py / lite sim) does the check and the retries and reports `PLACE_FAILED`. The WMS records the failure, creates the fix task and quarantines.

**Tech Stack:** Python 3.11, Webots R2025a controllers, unittest.

**Spec:** `docs/superpowers/specs/2026-11-21-robot-place-failure-design.md`

## Global Constraints

- Only rack destinations (PUTAWAY, RELOCATE) can fail; outbound lanes and buffers never do.
- p = `faults.place_p`, q = `faults.retry_q`, r = `faults.takeover_r`, alpha = `faults.place_fail_alpha`. A scenario without `place_p` behaves exactly as today (no check, no failure). Experiment values: p = r = 0.95, q in {0.5, 0.8, 0.95}, alpha = 0.
- Failed attempt duration: skew-normal, location act_s/2, scale act_s/4, shape alpha, clipped to [0.1, act_s]. Success takes act_s.
- Same robot: at most 2 retries after the first try. Takeover robot: 1 try, no retry.
- Fix task: RELOCATE, source = where the carton landed, destination = the intended slot, `cause` = `"PLACE_FAILED"`; the WMS rejects CANCEL of it.
- Quarantine: the slot stays reserved by `"QUARANTINE"` forever, so `slots_with`, `free_slots`, `lane_free_slot` and `stock` skip it.
- Draws keyed `unit(seed, lpn, k, kind, ...)` with kind in `first | retry | takeover`, so every WES policy and both engines see the same outcome for the same attempt.
- `lib/wms.py` is 진우's file: the change is approved; tell 진우 what changed.
- **Do not start until the running Webots sweep (`wsweep2`) has finished** (about 22:40): every new Webots run loads `controllers/` and `lib/` fresh, so edits would leak into it.
- Commit only when the user asks.

## Review Focus

1. No free neighbor on either side: the place succeeds (no failure); the test in Task 1 pins it.
2. A fix RELOCATE whose robot is told REASSIGN: allowed (only CANCEL is refused); the WMS test in Task 2 pins it.
3. A failure while the truck/other robot already reserved the landed slot in the record: the WMS must still record the carton there and leave the other task to get DESTINATION_OCCUPIED; the WMS test in Task 2 pins it.
4. Order needs a quarantined carton: the PICK must never be routed to it (`slots_with` empty); the WMS test in Task 2 pins it.
5. Old scenarios (no `place_p`): identical lite-sim result to before; the lite test in Task 3 pins it.

---

### Task 1: Shared place-attempt draw

**Files:**
- Create: `lib/place_fail.py`
- Test: `tests/test_place_fail.py`

**Interfaces:**
- Produces: `attempt(seed, lpn, k, kind, faults, act_s) -> tuple[bool, float]`, returning (success, seconds). `kind` is `"first" | "retry" | "takeover"`. `enabled(faults) -> bool` is True iff `"place_p" in faults`. `landing(slot_id, occupant, u) -> str | None` is the free neighbor slot (same face, index +-1, order picked by `u < 0.5`), or None.
- `unit(*parts)` is copied with the same formula as `supervisor.py:95` (crc32 of the joined parts / 2**32).

- [ ] **Step 1: Write the failing tests**

```python
def test_success_rates_follow_kind(self):
    f = {"place_p": 0.95, "retry_q": 0.5, "takeover_r": 0.8, "place_fail_alpha": 0.0}
    ok = lambda kind: sum(attempt(1, f"L{i}", 1, kind, f, 2.0)[0] for i in range(4000)) / 4000
    self.assertAlmostEqual(ok("first"), 0.95, delta=0.02)
    self.assertAlmostEqual(ok("retry"), 0.5, delta=0.03)
    self.assertAlmostEqual(ok("takeover"), 0.8, delta=0.03)

def test_failed_attempt_time_is_clipped_and_centered(self):
    f = {"place_p": 0.0, "retry_q": 0.0, "takeover_r": 0.0, "place_fail_alpha": 0.0}
    ts = [attempt(1, f"L{i}", 1, "first", f, 2.0)[1] for i in range(4000)]
    self.assertTrue(all(0.1 <= t <= 2.0 for t in ts))
    self.assertAlmostEqual(sum(ts) / len(ts), 1.0, delta=0.05)

def test_success_takes_act_s_and_draw_is_deterministic(self):
    f = {"place_p": 1.0, "retry_q": 1.0, "takeover_r": 1.0, "place_fail_alpha": 0.0}
    self.assertEqual(attempt(3, "L1", 2, "first", f, 2.0), (True, 2.0))
    g = dict(f, place_p=0.5)
    self.assertEqual(attempt(3, "L9", 4, "first", g, 2.0), attempt(3, "L9", 4, "first", g, 2.0))

def test_landing_picks_a_free_neighbor_or_none(self):
    occ = {"r0s01": None, "r0s02": "X", "r0s03": None}
    self.assertIn(landing("r0s02", occ, 0.3), ("r0s01", "r0s03"))
    self.assertIsNone(landing("r0s02", {"r0s01": "A", "r0s02": "X", "r0s03": "B"}, 0.3))

def test_enabled_only_with_place_p(self):
    self.assertFalse(enabled({"place_error_p": 0.1}))
    self.assertTrue(enabled({"place_p": 1.0}))
```

- [ ] **Step 2: Run** `python -m unittest tests.test_place_fail -v` → FAIL (module missing).
- [ ] **Step 3: Implement `lib/place_fail.py`.** Success if `unit(seed, lpn, k, kind, "ok") < {first: place_p, retry: retry_q, takeover: takeover_r}[kind]`. Failure time: two normals by Box-Muller from `unit(..., "n1")`, `unit(..., "n2")` (clip u away from 0), skew-normal `z = d*|n0| + sqrt(1-d*d)*n1` with `d = a/sqrt(1+a*a)`, then `act_s/2 + act_s/4*z` clipped to [0.1, act_s]. `landing` uses the existing face/index naming (`sid[:3]`, `int(sid[3:])`) exactly as `supervisor.py:293-299`.
- [ ] **Step 4: Run** the tests → PASS.

### Task 2: WMS `PLACE_FAILED`, fix task, quarantine

**Files:**
- Modify: `lib/wms.py` (`PAYLOAD`, new `_on_place_failed`, `_check_cancel`, `WMSView.stock`, docstring)
- Test: `tests/test_wms.py` (new class `PlaceFailedTests`, using `small_cfg()`)

**Interfaces:**
- Consumes the robot event `{"type": "PLACE_FAILED", "robot_id", "task_id", "slot": <intended>, "actual": <landed>, "item_id", "takeover": bool}`.
- Produces: the fix task `RELOCATE` with `cause="PLACE_FAILED"`, `source=actual`, `destination=intended`, `lpn` copied, both slots reserved by it; `"cause"` added to `PAYLOAD` so robots receive it; counters `place_failed` and `quarantined`; `rec[sid]["reserved_by"] == "QUARANTINE"` for a quarantined slot.

- [ ] **Step 1: Write the failing tests** (drive a PUTAWAY of `A` to `r0s03` to IN_PROGRESS the way existing tests do, then apply the event):
  - `test_first_failure_records_and_creates_fix`: after `PLACE_FAILED(slot=r0s03, actual=r0s04, takeover=False)`, `rec["r0s04"]["item"] == "A"`, the PUTAWAY is `COMPLETED` with `misplaced` True, exactly one new RELOCATE exists with source `r0s04`, destination `r0s03`, cause `PLACE_FAILED`, status `PENDING`, and both `r0s03` and `r0s04` are `reserved_by` its id. No `anomaly_*` counter is set.
  - `test_fix_task_cannot_be_cancelled_but_can_be_reassigned`: `CANCEL` of the fix RELOCATE is rejected with `"FIX_TASK"`; `REASSIGN` passes `_check_reassign` (status-only reasons apply as today).
  - `test_takeover_failure_quarantines`: run the fix RELOCATE, then `PLACE_FAILED(slot=r0s03, actual=r0s02, takeover=True)`: `rec["r0s02"]` holds `A` with `reserved_by == "QUARANTINE"`, `r0s03` is released, `view.slots_with("A") == []` (with `r0s04` emptied by the move), `view.stock("A")` excludes it, counter `quarantined == 1`, and no new task is created.
  - `test_landing_slot_reserved_by_other_task`: reserve `r0s04` for another PUTAWAY first; the failure still records `A` at `r0s04` (the other task later gets DESTINATION_OCCUPIED as today).
  - `test_payload_carries_cause`: the EXECUTE payload of the fix task contains `"cause": "PLACE_FAILED"`.
- [ ] **Step 2: Run** `python -m unittest tests.test_wms -v` → the new tests FAIL.
- [ ] **Step 3: Implement.** `_on_place_failed` handles the source like `_on_task_completed` (lines 413-437: clear the source if its version is unchanged, `set_slot(actual, item, "PLACE_FAILED", lpn=..., arrival=True, observed=True)`), sets the task `COMPLETED` with `misplaced=True`, releases its reservations, then either creates and reserves the fix task (`takeover` False) or reserves `actual` for `"QUARANTINE"` and counts it (`takeover` True). `_check_cancel` returns `"FIX_TASK"` for a CANCEL of a task with `cause == "PLACE_FAILED"` (check `d["action"]`; REASSIGN shares the function). `stock` skips slots reserved by `"QUARANTINE"`. Add `PLACE_FAILED` to the docstring event list.
- [ ] **Step 4: Run** `python -m unittest discover tests` → all PASS (replay test included: `tools/replay_wms.py` must rebuild the same state; it replays events, so no change expected).

### Task 3: Lite sim place model

**Files:**
- Modify: `lib/lite_sim.py` (`phase_end` place branch ~line 307, robot state, a new `retry` path, docstring "shortcut" line 19)
- Test: `tests/test_lite_sim.py`

**Interfaces:**
- Consumes: `place_fail.attempt`, `place_fail.landing`, `place_fail.enabled` (Task 1); WMS `PLACE_FAILED` (Task 2).
- Produces: the same event sequence agent.py will produce (Task 4): place → check (`look_s`) → `TASK_COMPLETED`, or retry (go to landed slot, pick, go to destination, place) up to 2 times → `PLACE_FAILED`. Takeover attempt when `task.get("cause") == "PLACE_FAILED"`. Summary field `place_failures: {"first", "retry", "takeover", "quarantined"}`.

- [ ] **Step 1: Write the failing tests**
  - `test_place_model_off_is_unchanged`: a scenario with no `place_p` gives the same `summary` (shipped, order flow, task counts) as before the change. Freeze today's numbers for the test scenario by running it once before editing and pasting them as constants.
  - `test_failures_retry_then_fix`: with `place_p=0.0, retry_q=0.0, takeover_r=1.0`, every rack placement ends in `PLACE_FAILED` after 3 tries, a fix RELOCATE completes, `place_failures["first"]` equals the number of rack placements and `quarantined == 0`; WMS record quality at the end is 0 mismatches.
  - `test_takeover_failure_quarantines`: `place_p=retry_q=takeover_r=0.0` → `quarantined` equals the number of rack placements; no WMS anomaly or rejected decision.
  - `test_same_seed_same_failures`: two runs with `place_p=0.8, retry_q=0.5, takeover_r=0.8` give identical `place_failures` and identical `PLACE_FAILED` events.
- [ ] **Step 2: Run** `python -m unittest tests.test_lite_sim -v` → new tests FAIL.
- [ ] **Step 3: Implement.** When `enabled(faults)` and the destination is a rack slot, the place phase lasts `attempt(...)[1]` seconds instead of `act_s` (draw at phase start, `k` = per-LPN place count as `supervisor.py`'s `place_count`). On failure, `landing()` with `u = unit(seed, lpn, k, kind, "side")`; no free neighbor → treat as success. Then a check phase of `look_s`. Keep the per-robot try counter on the robot state and reset it in `finish`.
- [ ] **Step 4: Run** `python -m unittest discover tests` → all PASS.

### Task 4: Webots supervisor and agent

**Files:**
- Modify: `controllers/supervisor/supervisor.py` (`handle_act` place branch lines 281-309, a pending-act list processed in the main loop like `s3_pending` line 481, docstring fault list)
- Modify: `controllers/agent/agent.py` (look_dst → place at line ~299, ACT_RESULT place branch ~line 250, new phases)

**Interfaces:**
- ACT message gains `"attempt": "first" | "retry" | "takeover"`. With the model enabled the agent sends the place ACT when the place phase **starts**; the supervisor draws `attempt(...)`, holds the act for its seconds, then applies it (put at intended or at `landing()`) and replies `ACT_RESULT` with `"landed": <slot>`. Model disabled: today's flow (wait act_s, then ACT) unchanged.
- Truth log: `ROBOT_PLACE_FAIL` with lpn, intended, landed, kind, robot, task_id; summary `faults_injected` counts it.
- Agent phases: `check` (look_s after a place) → `TASK_COMPLETED` / `go_retry` (to the landed slot) → `pick` from the landed slot → `go_dst` → `look_dst` → `place` with `attempt="retry"`. After the 3rd failed try (or a failed takeover) the agent sends `PLACE_FAILED` (Task 2 fields) and finishes. Shortcut allowed: the robot learns the landed slot from `ACT_RESULT` instead of looking at the neighbors; mark it `# shortcut: robot is told where the carton landed, upgrade to looking at both neighbors if perception errors are modelled`.

- [ ] **Step 1: Smoke test the code without Webots:** `python -m py_compile controllers/supervisor/supervisor.py controllers/agent/agent.py` → no output.
- [ ] **Step 2: Implement** as in Interfaces, reusing `place_fail` and keeping the old `place_error_p` branch for scenarios without `place_p`.
- [ ] **Step 3: Webots check, model forced to fail.** `gen_world.py --plain --seed 3001 --date 2023-03-09 --fault-scale 0 --place-p 0.5 --retry-q 0.5 --takeover-r 0.5 --run-name pf_smoke` (Task 5 flags), patch `fault_model`/`fault_seed` as `scratchpad/webots_sweep.sh` does, run one batch with `sched_verify`. Expected in the log folder: `truth.jsonl` has `ROBOT_PLACE_FAIL`, `events.jsonl` has `PLACE_FAILED` with `takeover` both False and True, summary `wes_stats.counts` has `place_failed` and `quarantined` > 0, no `anomaly_*`, `record_quality` settled mismatch 0 at the end, no robot stalled > 120 s (`scratchpad/stalls.py`). Restore the default world afterwards (`backup/s42`).
- [ ] **Step 4: Webots check, lite agreement.** Run the same scenario in the lite sim; the multiset of `(lpn, kind, ok)` attempts in Webots must equal the lite sim's for the attempts both reached (same keys → same draws).

### Task 5: Scenario flags and docs

**Files:**
- Modify: `tools/gen_world.py` (args near line 305, `faults` dict line 553)
- Modify: `docs/lite_sim.md` (the "모델링하지 않은 것" line), `webots/README.md` or `docs/wms.md` event list (whichever lists robot events)

**Interfaces:**
- `--place-p` (default None = model off), `--retry-q`, `--takeover-r` (default 1.0), `--place-fail-alpha` (default 0.0). Written to `faults` only when `--place-p` is given; not multiplied by `--fault-scale` (they are success probabilities).

- [ ] **Step 1:** Implement; `python tools/gen_world.py --plain --seed 1 --date 2023-03-09 --fault-scale 0 --place-p 0.95 --retry-q 0.5 --run-name x` then check `scenario.json` faults has `place_p 0.95, retry_q 0.5, takeover_r 1.0` and every human rate 0. Restore the default world afterwards.
- [ ] **Step 2:** Update the docs: the new event, quarantine, the flags, and that human faults are off on this branch.

### Task 6: Webots experiment (q sensitivity)

**Files:**
- Create: `scratchpad/webots_place_sweep.sh` (from `scratchpad/webots_sweep2.sh`)

- [ ] **Step 1:** For q in 0.5, 0.8, 0.95 and loads 03-09, 03-22 half, 03-22 full: `--fault-scale 0 --place-p 0.95 --retry-q $q --takeover-r 0.95`, seed 3001, 8 robots, baseline (`sched_verify`) and mix (`sched_learned`) in parallel. Restore the default world at the end.
- [ ] **Step 2:** Report per condition: order flow (order_all), unit lead, shipped, missed, place failures by kind, fix-task first-assignment wait (`research_metrics.task_metrics`, RELOCATE), quarantined, max stall. Lite-sim numbers, if shown, are labelled as screens.
