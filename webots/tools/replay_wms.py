"""Rebuild a run's Mock WMS from its events.jsonl and check it against the run's own final state.

The WMS changes only through events and decisions, and the supervisor logs both, so
feeding events.jsonl to a fresh MockWMS must end in the same records, task results and
order counts as the live run (wms_log.jsonl, FINAL line). Robot reports are not in the log:
they only tell IN_PROGRESS from ASSIGNED, which the check treats alike.

Usage (from webots/):
  python tools/replay_wms.py logs/<run folder> [--scenario scenario.json]
"""
import argparse
import json
import sys
from collections import Counter, defaultdict
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "lib"))
from wms import ASSIGNED, IN_PROGRESS, MockWMS, scenario_hash  # noqa: E402


def read_jsonl(path):
    with open(path, encoding="utf-8") as f:
        return [json.loads(line) for line in f if line.strip()]


def replay(cfg, events_path):
    """A MockWMS fed with the run's events and accepted decisions, in the run's order."""
    wms = MockWMS(cfg, strict=False)
    lines = read_jsonl(events_path)
    after = defaultdict(list)                  # decisions a scheduler made in reaction to an event
    for ln in lines:
        if ln.get("kind") == "DECISION" and ln.get("after_event"):
            after[ln["after_event"]].append(ln)

    def decide(ln):
        wms.t = max(wms.t, ln["t"])
        wms.apply_decision({k: v for k, v in ln.items() if k not in ("t", "kind")}, trusted=True)

    for ln in lines:
        if ln.get("kind") == "DECISION":
            if not ln.get("after_event"):
                decide(ln)
        else:
            wms.apply_event(ln)
            for d in after.get(ln.get("event_id"), ()):
                decide(d)
    return wms


def normalized(final):
    same = {IN_PROGRESS: ASSIGNED}
    return {"records": final["records"], "orders": final["orders"],
            "tasks": {tid: same.get(st, st) for tid, st in final["tasks"].items()}}


def check(cfg, log_dir):
    log_dir = Path(log_dir)
    wms = replay(cfg, log_dir / "events.jsonl")
    live = read_jsonl(log_dir / "wms_log.jsonl")
    start = next((x for x in live if x["type"] == "START"), {})
    final = next((x for x in reversed(live) if x["type"] == "FINAL"), None)
    report = {"wms": wms, "scenario_ok": start.get("scenario") == scenario_hash(cfg), "diff": {}}
    if final is None:
        report.update(same=False, diff={"FINAL": "no FINAL line (run did not reach the end?)"})
        return report
    a, b = normalized(final), normalized(wms.final_snapshot())
    for part in ("records", "tasks", "orders"):
        keys = set(a[part]) | set(b[part])
        bad = {k: (a[part].get(k), b[part].get(k)) for k in sorted(keys) if a[part].get(k) != b[part].get(k)}
        if bad:
            report["diff"][part] = bad
    report["same"] = not report["diff"]
    return report


def summary(wms):
    tasks = Counter(f"{t['task_type']}_{t['status']}" for t in wms.tasks.values())
    exc = Counter(e["type"] for e in wms.exceptions)
    anomalies = {k[len("anomaly_"):]: v for k, v in wms.counts.items() if k.startswith("anomaly_")}
    orders = wms.orders.values()
    shipped = [o for o in orders if o["status"] == "SHIPPED"]
    lead = sum(o["done_t"] - o["released_t"] for o in shipped) / len(shipped) if shipped else None
    lines = [
        f"t_end {wms.t:.0f} s",
        "tasks: " + ", ".join(f"{k} {v}" for k, v in sorted(tasks.items())),
        f"orders: released {len(wms.orders)}, shipped {len(shipped)}"
        + (f", mean lead {lead:.0f} s" if lead is not None else ""),
        f"inbound: ASN cartons {len(wms.asn)}, staged with label "
        f"{sum(1 for a in wms.asn.values() if a['status'] == 'STAGED')}, unread {wms.counts.get('inbound_unread', 0)}"
        f", read failures {wms.counts.get('event_INBOUND_READ_FAIL', 0)}",
        "exceptions: " + (", ".join(f"{k} {v}" for k, v in sorted(exc.items())) or "none"),
        "anomalies: " + (", ".join(f"{k} {v}" for k, v in sorted(anomalies.items())) or "none"),
    ]
    return "\n".join(lines)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("log_dir", type=Path)
    ap.add_argument("--scenario", type=Path, default=ROOT / "scenario.json")
    args = ap.parse_args()
    cfg = json.loads(args.scenario.read_text(encoding="utf-8"))
    report = check(cfg, args.log_dir)
    if not report["scenario_ok"]:
        print("warning: scenario.json differs from the one this run used (regenerated since?)")
    print(summary(report["wms"]))
    if report["same"]:
        print("replay: same final records, task results and order counts as the run")
    else:
        print("replay: DIFFERENT from the run")
        for part, bad in report["diff"].items():
            items = list(bad.items()) if isinstance(bad, dict) else [(part, bad)]
            print(f"  {part}: {len(items)} differ, e.g. " + "; ".join(f"{k}: run {v[0]} / replay {v[1]}"
                                                                     if isinstance(v, tuple) else f"{k}: {v}"
                                                                     for k, v in items[:5]))
        sys.exit(1)


if __name__ == "__main__":
    main()
