"""Replay a Webots run in LiteSim and compare event timing and matched tasks.

Usage: python tools/diagnose_engine_gap.py logs/<webots-run> [--out report.json]
The Webots run must contain scenario.json, run_env.json, events.jsonl, and summary.json.
This is an engine diagnostic, not a held-out policy evaluation.
"""
import argparse
import importlib
import json
import os
import statistics as st
import sys
import tempfile
from collections import defaultdict
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "lib"))
from lite_sim import LiteSim  # noqa: E402
from wes_runner import WES  # noqa: E402
from validate_lite import POLICY_ENV  # noqa: E402


def task_times(events):
    started, finished, types, robots = {}, {}, {}, {}
    for ev in events:
        if ev.get("kind") == "DECISION" and ev.get("action") == "EXECUTE":
            tid = ev.get("task_id")
            started.setdefault(tid, ev["t"])
            types.setdefault(tid, (ev.get("task") or {}).get("task_type"))
            robots.setdefault(tid, ev.get("robot_id"))
        elif ev.get("type") == "TASK_COMPLETED":
            finished.setdefault(ev.get("task_id"), ev["t"])
    return {tid: (types.get(tid), t - started[tid], robots.get(tid))
            for tid, t in finished.items() if tid in started}


def describe(webots_events, lite_events):
    result = {"event_timing": {}}
    def order_flow(events):
        orders = {}
        for ev in events:
            if ev.get("type") == "ORDER_RELEASED":
                for line in ev["lines"]:
                    row = orders.setdefault(line["order"], {"released_t": ev["t"], "demand": 0, "shipped": 0,
                                                               "completed_t": None})
                    row["demand"] += line["qty"]
            elif ev.get("type") == "OUTBOUND_DEPARTED":
                for carton in ev["cartons"]:
                    if carton.get("order") in orders:
                        row = orders[carton["order"]]
                        row["shipped"] += 1
                        if row["shipped"] == row["demand"]:
                            row["completed_t"] = ev["t"]
        return orders
    w_orders, l_orders = order_flow(webots_events), order_flow(lite_events)
    result["orders"] = {oid: {"webots_completed_t": w_orders[oid]["completed_t"],
                               "lite_completed_t": l_orders[oid]["completed_t"],
                               "webots_minus_lite_s": round(w_orders[oid]["completed_t"] -
                                                            l_orders[oid]["completed_t"], 2)
                               if w_orders[oid]["completed_t"] is not None and
                                  l_orders[oid]["completed_t"] is not None else None}
                        for oid in sorted(w_orders.keys() & l_orders.keys())}
    assignments = {}
    for label, events in (("webots", webots_events), ("lite", lite_events)):
        assignments[label] = [(ev["t"], ev.get("task_id"), ev.get("robot_id")) for ev in events
                              if ev.get("kind") == "DECISION" and ev.get("action") == "EXECUTE"]
    prefix = 0
    for w, l in zip(assignments["webots"], assignments["lite"]):
        if w[1:] != l[1:]:
            break
        prefix += 1
    result["assignment_sequence"] = {
        "webots_count": len(assignments["webots"]), "lite_count": len(assignments["lite"]),
        "matching_prefix": prefix,
        "first_webots_difference": assignments["webots"][prefix] if prefix < len(assignments["webots"]) else None,
        "first_lite_difference": assignments["lite"][prefix] if prefix < len(assignments["lite"]) else None,
    }
    if prefix:
        early_ids = [tid for _, tid, _ in assignments["webots"][:prefix]]
        def completions(events):
            return {ev.get("task_id"): ev["t"] for ev in events
                    if ev.get("type") == "TASK_COMPLETED" and ev.get("task_id") in early_ids}
        wc, lc = completions(webots_events), completions(lite_events)
        result["assignment_sequence"]["matching_prefix_completions"] = [
            {"task_id": tid, "robot_id": rid, "webots_s": wc.get(tid), "lite_s": lc.get(tid)}
            for _, tid, rid in assignments["webots"][:prefix]]
    for label, events in (("webots", webots_events), ("lite", lite_events)):
        times = defaultdict(list)
        for ev in events:
            if ev.get("type"):
                times[ev["type"]].append(ev["t"])
        result["event_timing"][label] = {kind: {"count": len(ts), "first_s": min(ts),
                                                  "median_s": st.median(ts), "last_s": max(ts)}
                                          for kind, ts in sorted(times.items())}
    wt, lt = task_times(webots_events), task_times(lite_events)
    shared = sorted(set(wt) & set(lt))
    by_type = defaultdict(list)
    same_robot = 0
    for tid in shared:
        typ, wdur, wrid = wt[tid]
        _, ldur, lrid = lt[tid]
        same_robot += wrid == lrid
        if wrid != lrid:
            continue
        by_type[typ].append({"task_id": tid, "webots_s": round(wdur, 2),
                             "lite_s": round(ldur, 2), "lite_minus_webots_s": round(ldur - wdur, 2)})
    result["matched_tasks"] = {
        typ: {"n": len(rows), "median_webots_s": st.median(r["webots_s"] for r in rows),
              "median_lite_s": st.median(r["lite_s"] for r in rows),
              "median_difference_s": st.median(r["lite_minus_webots_s"] for r in rows),
              "mean_difference_s": round(st.mean(r["lite_minus_webots_s"] for r in rows), 2),
              "mean_abs_difference_s": round(st.mean(abs(r["lite_minus_webots_s"]) for r in rows), 2)}
        for typ, rows in by_type.items()}
    result["task_counts"] = {"webots_completed": len(wt), "lite_completed": len(lt),
                             "matched_task_id": len(shared), "matched_task_and_robot": same_robot}
    return result


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("run", type=Path)
    ap.add_argument("--out", type=Path)
    ap.add_argument("--lite-events", type=Path, help="optionally save replay events as JSONL")
    ap.add_argument("--until", type=float, help="compare flushed events through this simulation second")
    ap.add_argument("--leg-s", type=float, default=2.0)
    ap.add_argument("--yield-s", type=float, default=1.0)
    args = ap.parse_args()
    run = args.run
    cfg = json.loads((run / "scenario.json").read_text(encoding="utf-8"))
    env = json.loads((run / "run_env.json").read_text(encoding="utf-8"))
    saved = {key: os.environ.get(key) for key in POLICY_ENV}
    try:
        for key in POLICY_ENV:
            value = env.get(key)
            if value is None:
                os.environ.pop(key, None)
            else:
                os.environ[key] = value
        module = importlib.import_module(env.get("WES_SCHEDULER") or "sched_standin")
        lite_events = []
        with tempfile.TemporaryDirectory() as tmp:
            summary = LiteSim(cfg, WES(cfg, scheduler=module.Scheduler(cfg), log_dir=tmp),
                              leg_s=args.leg_s, yield_s=args.yield_s,
                              event_sink=lite_events.append).run(args.until)
    finally:
        for key, value in saved.items():
            if value is None:
                os.environ.pop(key, None)
            else:
                os.environ[key] = value
    if args.lite_events:
        args.lite_events.parent.mkdir(parents=True, exist_ok=True)
        args.lite_events.write_text("".join(json.dumps(ev, ensure_ascii=False) + "\n" for ev in lite_events),
                                    encoding="utf-8")
    webots_events = [json.loads(line) for line in (run / "events.jsonl").open(encoding="utf-8")]
    if args.until is not None:
        webots_events = [event for event in webots_events if event["t"] <= args.until]
    summary_path = run / "summary.json"
    webots_summary = json.loads(summary_path.read_text(encoding="utf-8")) if summary_path.exists() else {}
    report = {"scenario": str(run / "scenario.json"), "env": env,
              "lite_parameters": {"leg_s": args.leg_s, "yield_s": args.yield_s},
              "webots": {k: webots_summary.get(k) for k in ("units_released", "units_shipped", "orders_completed",
                                                              "inbound_staged", "missed_cutoffs", "diagnostics",
                                                              "record_slips", "record_mismatch_at_end")},
              "lite": {k: summary.get(k) for k in ("units_released", "units_shipped", "orders_completed",
                                                    "inbound_staged", "missed_cutoffs", "diagnostics",
                                                    "record_slips", "record_mismatch_at_end")},
              **describe(webots_events, lite_events)}
    if summary_path.exists() and args.until is None:
        from validate_lite import events_metrics
        report["paired_metrics"] = {"webots": events_metrics(webots_events, webots_summary),
                                    "lite": events_metrics(lite_events, summary)}
    output = json.dumps(report, ensure_ascii=False, indent=2)
    if args.out:
        args.out.write_text(output + "\n", encoding="utf-8")
    print(output)


if __name__ == "__main__":
    main()
