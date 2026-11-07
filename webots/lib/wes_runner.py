"""WES module for the simulator: Mock WMS + a scheduler (the WES policy).

    WES_MODULE=wes_runner WES_SCHEDULER=sched_standin  (scheduler module in lib/)

The supervisor only knows the WES interface (on_event / decide / record_snapshot /
stats). This module splits it in two:

    event   -> wms.apply_event(ev)            the record and the tasks (Mock WMS)
            -> scheduler.on_event(ev, state)  optional: react at once (e.g. to SHORT_PICK)
    step    -> wms.update_robots(t, robots)
            -> scheduler.plan(state)          assign / reroute / wait ...
    each decision -> wms.apply_decision(d)    checked against the record; rejected ones
                                              are logged and never reach a robot

A scheduler is a module with

    class Scheduler:
        name = "my_policy"                    # goes into the log folder name (wms-my_policy)
        def __init__(self, cfg): ...          # scenario.json
        def plan(self, state): return [...]   # Decisions, every step
        def on_event(self, ev, state): return [...]   # optional

where state is the WMS view (lib/wms.py, WMSView). Details: webots/docs/wms.md.
The WMS writes wms_log.jsonl and a copy of scenario.json next to the simulator's logs
(or to WMS_LOG_DIR).
"""
import importlib
import json
import os
from pathlib import Path

from wms import MockWMS

ROOT = Path(__file__).resolve().parents[1]


def log_dir_for(cfg, name):
    """The folder the supervisor writes this run's logs to (same rule as supervisor.py)."""
    faults_on = any(v for k, v in cfg["faults"].items() if not k.endswith("max_slots"))
    sweep = "_sweep" if cfg.get("patrol", {}).get("idle_sweep") else ""
    run = f"_{cfg['run_name']}" if cfg.get("run_name") else ""
    mode = "batch" if os.environ.get("QUIT_AT_END") == "1" else "gui"
    tag = (f"{cfg['outbound']['date']}_{name}_seed{cfg['seed']}_{'faults' if faults_on else 'nofaults'}"
           f"{sweep}{run}_{mode}")
    return ROOT / "logs" / tag


class WES:
    def __init__(self, cfg, scheduler=None, log_dir=None):
        if scheduler is None:
            scheduler = importlib.import_module(os.environ.get("WES_SCHEDULER", "sched_standin")).Scheduler(cfg)
        self.sched = scheduler
        self.name = f"wms-{scheduler.name}"
        if log_dir is None:
            log_dir = Path(os.environ["WMS_LOG_DIR"]) if os.environ.get("WMS_LOG_DIR") else log_dir_for(cfg, self.name)
        Path(log_dir).mkdir(parents=True, exist_ok=True)
        # The scenario this run used: for tools/replay_wms.py and gen_world.py --config.
        (Path(log_dir) / "scenario.json").write_text(json.dumps(cfg), encoding="utf-8")
        self.wms = MockWMS(cfg, log_path=Path(log_dir) / "wms_log.jsonl")
        self.out = []

    def on_event(self, ev):
        if not self.wms.apply_event(ev):
            return
        react = getattr(self.sched, "on_event", None)
        if react:
            # Tagged with the event so a replay applies them at the same point (tools/replay_wms.py).
            self._accept(react(ev, self.wms.view), after=ev.get("event_id"))

    def decide(self, t, robots):
        self.wms.update_robots(t, robots)
        self._accept(self.sched.plan(self.wms.view))
        out, self.out = self.out, []
        return out

    def _accept(self, decisions, after=None):
        for d in decisions or ():
            if after:
                d = {**d, "after_event": after}
            sent = self.wms.apply_decision(d)
            if sent is not None:
                self.out.append(sent)

    def record_snapshot(self):
        self.wms.flush()
        return self.wms.record_snapshot()

    def stats(self):
        # Called once, at the end of the episode: write the final state too.
        self.wms.close()
        return self.wms.stats()
