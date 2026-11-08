"""Summarize the patrol experiment (tools/run_patrol_experiment.py).

Reads logs/*_wms-verify-*_pt_*_batch/summary.json and writes
  logs/patrol_experiment_runs.csv     one row per run
  logs/patrol_experiment_summary.md   mean (sd) per mix x policy x fault scale, and the
                                      paired difference between mixes (same policy, scale, seed)

Usage (from webots/): python tools/summarize_patrol.py [--logs logs] [--duration 3600]
"""
import argparse
import csv
import json
import re
import statistics
import sys
from collections import defaultdict
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
NAME = re.compile(r"_wms-verify-(?P<policy>\w+?)_seed(?P<seed>\d+)_(?:no)?faults_pt_t(?P<t>\d+)p(?P<p>\d+)_"
                  r"(?P=policy)_f(?P<scale>[\d.]+)_d(?P<duration>[\d.]+)_batch$")

# (column, label, function of the summary) -- higher is better unless the label says otherwise
METRICS = [
    ("units_shipped", "출하 수", lambda s: s["units_shipped"]),
    ("orders_completed", "완료 주문", lambda s: s["orders_completed"]),
    ("missed_cutoffs", "마감 놓침 (낮을수록 좋음)", lambda s: s["missed_cutoffs"]),
    ("mean_order_lead_s", "평균 주문 처리 시간 s (낮을수록 좋음)", lambda s: s.get("mean_order_lead_s")),
    ("short_pick", "SHORT_PICK+ITEM_MISMATCH (낮을수록 좋음)",
     lambda s: s["events"].get("SHORT_PICK", 0) + s["events"].get("ITEM_MISMATCH", 0)),
    ("dest_occupied", "DESTINATION_OCCUPIED (낮을수록 좋음)", lambda s: s["events"].get("DESTINATION_OCCUPIED", 0)),
    ("settled_mismatch_mean", "틀린 기록 시간 평균 (낮을수록 좋음)",
     lambda s: (s.get("record_quality") or {}).get("settled_mismatch_mean")),
    ("mean_time_to_correct_s", "기록 정정까지 평균 s (낮을수록 좋음)",
     lambda s: (s.get("record_quality") or {}).get("mean_time_to_correct_s")),
    ("open_at_end", "끝날 때 미정정 기록 (낮을수록 좋음)", lambda s: (s.get("record_quality") or {}).get("open_at_end")),
    ("verify_done", "VERIFY 완료 수",
     lambda s: sum(v for k, v in ((s.get("wes_stats") or {}).get("tasks") or {}).items() if k == "VERIFY_COMPLETED")),
    ("transport_busy", "운반 로봇 가동률", lambda s: role_busy(s, "transport")),
    ("patrol_busy", "순찰 로봇 가동률", lambda s: role_busy(s, "patrol")),
    ("distance_m", "총 이동 거리 m", lambda s: round(sum(s["robot_distance_m"].values()), 1)),
    ("diagnostics", "진단 (정체 등, 낮을수록 좋음)", lambda s: sum(s["diagnostics"].values())),
]
PAIRED = ["units_shipped", "missed_cutoffs", "short_pick", "settled_mismatch_mean", "open_at_end"]


def role_busy(s, role):
    roles = s.get("robot_roles") or {}
    vals = [b for r, b in s["robot_busy_share"].items() if roles.get(r) == role]
    return round(statistics.mean(vals), 3) if vals else None


def fmt(vals):
    vals = [v for v in vals if v is not None]
    if not vals:
        return "-"
    m = statistics.mean(vals)
    sd = statistics.stdev(vals) if len(vals) > 1 else 0.0
    return f"{m:.3g} ({sd:.2g})"


def main():
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    ap = argparse.ArgumentParser()
    ap.add_argument("--logs", type=Path, default=ROOT / "logs")
    ap.add_argument("--duration", type=float, help="only runs of this length")
    args = ap.parse_args()

    rows = []
    for summ in sorted(args.logs.glob("*_wms-verify-*_pt_*_batch/summary.json")):
        m = NAME.search(summ.parent.name)
        if not m:
            continue
        if args.duration is not None and float(m["duration"]) != args.duration:
            continue
        s = json.loads(summ.read_text(encoding="utf-8"))
        row = {"mix": f"{m['t']}+{m['p']}", "policy": m["policy"], "fault_scale": float(m["scale"]),
               "seed": int(m["seed"]), "duration": float(m["duration"])}
        row.update({col: fn(s) for col, _, fn in METRICS})
        rows.append(row)
    if not rows:
        raise SystemExit("no runs found")

    out_csv = args.logs / "patrol_experiment_runs.csv"
    with open(out_csv, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)

    groups = defaultdict(list)
    for r in rows:
        groups[(r["duration"], r["fault_scale"], r["policy"], r["mix"])].append(r)
    lines = ["# 순찰 로봇 필요성 실험 결과", "",
             "값은 평균 (표준편차), n은 seed 수. 기준 정의는 docs/experiments_patrol.md 참고.", ""]
    cols = [c for c, _, _ in METRICS]
    labels = {c: lab for c, lab, _ in METRICS}
    lines.append("| 시간 | 오류 배율 | 정책 | 구성(운반+순찰) | n | " + " | ".join(labels[c] for c in cols) + " |")
    lines.append("|" + "---|" * (5 + len(cols)))
    for key in sorted(groups):
        g = groups[key]
        dur, scale, policy, mix = key
        lines.append(f"| {dur:g} | {scale:g} | {policy} | {mix} | {len(g)} | "
                     + " | ".join(fmt([r[c] for r in g]) for c in cols) + " |")

    # Paired differences: same duration, scale, policy and seed, each mix against the first mix.
    by = {(r["duration"], r["fault_scale"], r["policy"], r["seed"], r["mix"]): r for r in rows}
    mixes = sorted({r["mix"] for r in rows}, key=lambda m: -int(m.split("+")[0]))
    if len(mixes) > 1:
        base = mixes[0]
        lines += ["", f"## 짝지은 차이 (각 구성 - {base}, 같은 정책·오류 배율·seed)", "",
                  "| 시간 | 오류 배율 | 정책 | 구성 | n | " + " | ".join(labels[c] for c in PAIRED) + " |",
                  "|" + "---|" * (5 + len(PAIRED))]
        cells = sorted({(d, f, p) for d, f, p, _, _ in by})
        for d, f, p in cells:
            for mix in mixes[1:]:
                diffs = defaultdict(list)
                for (d2, f2, p2, seed, m2), r in by.items():
                    if (d2, f2, p2, m2) == (d, f, p, mix) and (d, f, p, seed, base) in by:
                        b = by[(d, f, p, seed, base)]
                        for c in PAIRED:
                            if r[c] is not None and b[c] is not None:
                                diffs[c].append(r[c] - b[c])
                n = max((len(v) for v in diffs.values()), default=0)
                if n:
                    lines.append(f"| {d:g} | {f:g} | {p} | {mix} | {n} | "
                                 + " | ".join(fmt(diffs[c]) for c in PAIRED) + " |")
    out_md = args.logs / "patrol_experiment_summary.md"
    out_md.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print("\n".join(lines))
    print(f"\n{len(rows)} runs -> {out_csv.name}, {out_md.name}")


if __name__ == "__main__":
    main()
