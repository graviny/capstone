"""Webots results on the days30 test scenarios: every policy against team WES and against Hungarian.
Protocol: docs/paper/baselines_days30.md

    python tools/compare_days30.py [--runs "logs/*_d30_batch"] [--group days30_cem=days30_cem_r] [--out file.md]

--group NAME=PREFIX adds a row for the per-scenario mean of all policies whose name starts with PREFIX
(e.g. the three training runs r0, r1, r2). Metrics: tools/validate_lite.py events_metrics. Paired
differences per scenario, 95% bootstrap CI over scenarios (10,000 resamples, seed 20261128).
"""
import argparse
import glob
import json
import random
import statistics as st
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from validate_lite import policy, webots  # noqa: E402

REFS = {"sched_verify": "팀 WES", "sched_hungarian": "헝가리안"}
M = [("order_all", "주문 리드(s)", 1), ("p90", "p90(s)", 0), ("unit", "단위 리드(s)", 1), ("shipped", "출고", 1),
     ("SHORT_PICK", "SHORT_PICK", 1), ("DESTINATION_OCCUPIED", "DEST_OCC", 1), ("busy", "가동률", 3)]


def boot(diffs, n=10000, seed=20261128):
    r = random.Random(seed)
    ms = sorted(st.mean(r.choices(diffs, k=len(diffs))) for _ in range(n))
    return ms[int(0.025 * n)], ms[int(0.975 * n)]


def paired(rows, p, ref):
    common = sorted(set(rows[p]) & set(rows[ref]))
    d = [rows[p][s]["order_all"] - rows[ref][s]["order_all"] for s in common]
    lo, hi = boot(d)
    return f"{st.mean(d):+.1f} [{lo:+.1f}, {hi:+.1f}] ({sum(x < 0 for x in d)}/{len(d)})"


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--runs", default="logs/*_d30_batch")
    ap.add_argument("--group", action="append", default=[], metavar="NAME=PREFIX")
    ap.add_argument("--out", type=Path)
    a = ap.parse_args()
    rows = {}
    for d in map(Path, sorted(glob.glob(a.runs))):
        if not (d / "summary.json").exists() or not (d / "run_env.json").exists():
            continue
        cfg = json.loads((d / "scenario.json").read_text(encoding="utf-8"))
        sc = (cfg["outbound"]["date"], cfg["initial_record_errors_meta"]["level"])
        rows.setdefault(policy(d), {})[sc] = webots(d)
    missing = [r for r in REFS if r not in rows]
    if missing:
        raise SystemExit(f"no runs for {missing}: run tools/webots_days30.sh S H first")
    for g in a.group:
        name, prefix = g.split("=", 1)
        members = [p for p in rows if p.startswith(prefix)]
        common = set.intersection(*(set(rows[p]) for p in members)) if members else set()
        rows[name + f" (평균 {len(members)}회)"] = {s: {m: st.mean(rows[p][s][m] for p in members) for m, _, _ in M}
                                                 for s in common}
    order = list(REFS) + sorted(p for p in rows if p not in REFS)
    n_sc = len(rows["sched_verify"])
    L = [f"## 전체 (Webots, 시나리오 {n_sc}개)", "",
         "| 정책 | n | " + " | ".join(h for _, h, _ in M) + " | − 팀 WES [95% CI] (이긴 수) | − 헝가리안 [95% CI] (이긴 수) |",
         "|---|---|" + "---|" * len(M) + "---|---|"]
    for p in order:
        v = list(rows[p].values())
        cells = " | ".join(f"{st.mean(x[m] for x in v):.{k}f}" for m, _, k in M)
        vs = [("—" if p == ref else paired(rows, p, ref)) for ref in REFS]
        L.append(f"| {REFS.get(p, p)} | {len(v)} | {cells} | {vs[0]} | {vs[1]} |")
    L += ["", "## 시나리오별 주문 리드 (s)", "",
          "| 날짜 | 초기 오류 | " + " | ".join(REFS.get(p, p) for p in order) + " |", "|---|---|" + "---|" * len(order)]
    for s in sorted(rows["sched_verify"]):
        L.append(f"| {s[0]} | {s[1]:g} | " + " | ".join(
            f"{rows[p][s]['order_all']:.0f}" if s in rows[p] else "—" for p in order) + " |")
    text = "\n".join(L) + "\n"
    sys.stdout.reconfigure(encoding="utf-8")
    print(text)
    if a.out:
        a.out.write_text(text, encoding="utf-8")


if __name__ == "__main__":
    main()
