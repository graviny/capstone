"""Side-by-side summary of runs (summary.json and decision counts from events.jsonl).

Usage (from webots/):
  python tools/compare_runs.py logs/<run A> logs/<run B> [...]
"""
import argparse
import json
import sys
from collections import Counter
from pathlib import Path

ROWS = [
    ("orders released / completed", lambda s: f"{s['orders_released']} / {s['orders_completed']}"),
    ("mean order lead (s)", lambda s: f"{s['mean_order_lead_s']:.0f}" if s.get("mean_order_lead_s") else "-"),
    ("units released / shipped", lambda s: f"{s['units_released']} / {s['units_shipped']}"),
    ("mis-shipped", lambda s: s["mis_shipped"]),
    ("outbound trucks / missed cutoffs", lambda s: f"{s['outbound_trucks']} / {s['missed_cutoffs']}"),
    ("inbound trucks / staged", lambda s: f"{s['inbound_trucks_arrived']} / {s['inbound_staged']}"),
    ("cartons on belt at end", lambda s: s["cartons_on_belt_at_end"]),
    ("record mismatch at end", lambda s: s.get("record_mismatch_at_end")),
    ("faults S1 place / S1 move", lambda s: f"{s['faults_injected'].get('S1_PLACE_ERROR', 0)} / "
                                            f"{s['faults_injected'].get('S1_UNRECORDED_MOVE', 0)}"),
    ("faults S2 removal / S3 dest", lambda s: f"{s['faults_injected'].get('S2_UNRECORDED_REMOVAL', 0)} / "
                                              f"{s['faults_injected'].get('S3_DEST_TAKEN', 0)}"),
    ("scanner failures", lambda s: s["faults_injected"].get("SCANNER_FAIL", 0)),
    ("SHORT_PICK / ITEM_MISMATCH", lambda s: f"{s['events'].get('SHORT_PICK', 0)} / {s['events'].get('ITEM_MISMATCH', 0)}"),
    ("DESTINATION_OCCUPIED", lambda s: s["events"].get("DESTINATION_OCCUPIED", 0)),
    ("TASK_COMPLETED / TASK_FAILED", lambda s: f"{s['events'].get('TASK_COMPLETED', 0)} / {s['events'].get('TASK_FAILED', 0)}"),
    ("diagnostics", lambda s: ", ".join(f"{k} {v}" for k, v in sorted(s["diagnostics"].items())) or "none"),
    ("robot distance total (m)", lambda s: f"{sum(s['robot_distance_m'].values()):.0f}"),
    ("transport busy share (mean)", None),
]


def read_events(log_dir):
    """Decision counts, and the order releases / truck arrivals the environment produced."""
    c, orders, trucks = Counter(), [], []
    path = Path(log_dir) / "events.jsonl"
    if path.exists():
        with open(path, encoding="utf-8") as f:
            for line in f:
                d = json.loads(line)
                if d.get("kind") == "DECISION":
                    c[d["action"]] += 1
                elif d.get("type") == "ORDER_RELEASED":
                    orders.append((d["t"], d["wave"], json.dumps(d["lines"], sort_keys=True)))
                elif d.get("type") == "TRUCK_ARRIVED":
                    trucks.append((d["t"], d["truck"]))
    return c, orders, trucks


def same_as_first(seqs, k):
    if k == 0:
        return "(base)"
    a, b = seqs[0], seqs[k]
    if a == b:
        return "same"
    n = next((i for i in range(min(len(a), len(b))) if a[i] != b[i]), min(len(a), len(b)))
    return f"differs from #{n + 1}"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("runs", nargs="+", type=Path)
    args = ap.parse_args()
    sums, decs, orders, trucks = [], [], [], []
    for r in args.runs:
        p = r / "summary.json"
        if not p.exists():
            sys.exit(f"{p} not found (did the run reach the end?)")
        sums.append(json.loads(p.read_text(encoding="utf-8")))
        c, o, t = read_events(r)
        decs.append(c)
        orders.append(o)
        trucks.append(t)
    scen = json.loads((Path(__file__).resolve().parents[1] / "scenario.json").read_text(encoding="utf-8"))
    transports = [str(r["id"]) for r in scen["robots"] if r["role"] == "transport"]
    w = 34
    print(f"{'':{w}}" + "".join(f"{s['wes']:>22}" for s in sums))
    for name, fn in ROWS:
        if fn is None:
            vals = []
            for s in sums:
                share = [v for k, v in s["robot_busy_share"].items() if k in transports]
                vals.append(f"{sum(share) / len(share):.2f}" if share else "-")
        else:
            vals = [str(fn(s)) for s in sums]
        print(f"{name:{w}}" + "".join(f"{v:>22}" for v in vals))
    for action in sorted(set().union(*decs)):
        print(f"{'decisions ' + action:{w}}" + "".join(f"{d.get(action, 0):>22}" for d in decs))
    # Same settings and seed should give the same order releases; truck arrivals can shift
    # when a policy leaves the belt full (the next truck waits for the dock).
    print(f"{'order releases vs 1st run':{w}}" + "".join(f"{same_as_first(orders, k):>22}" for k in range(len(orders))))
    print(f"{'truck arrivals vs 1st run':{w}}" + "".join(f"{same_as_first(trucks, k):>22}" for k in range(len(trucks))))


if __name__ == "__main__":
    main()
