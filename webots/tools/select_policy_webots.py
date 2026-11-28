"""Select a policy only from paired Webots runs on the frozen validation dates.

Example:
  python tools/select_policy_webots.py --split experiments/split_v3.json \
      --policies sched_hungarian sched_verify --levels 0.065 --seeds 42 \
      --record-slip 0.01 --out validation.json "logs/*"

The default requires every validation date in the split. A partial run is a
diagnostic and cannot produce a final policy selection.
"""
import argparse
import glob
import json
import statistics
import sys
from collections import defaultdict
from pathlib import Path

from validate_lite import policy, scenario_key


def evaluate(split, runs, policies, levels, seeds, record_slip, allow_partial=False):
    expected = set(split["validation"])
    forbidden = set(split["train"]) | set(split["test"])
    if expected & forbidden:
        raise ValueError("split has overlapping train/validation/test dates")
    grouped = defaultdict(dict)
    design_cells = {}
    shared_env = defaultdict(dict)
    for folder in runs:
        cfg = json.loads((folder / "scenario.json").read_text(encoding="utf-8"))
        day = cfg["outbound"]["date"]
        if day not in expected:
            raise ValueError(f"{folder}: {day} is not a validation date")
        name = policy(folder)
        if name not in policies:
            raise ValueError(f"{folder}: unexpected policy {name}")
        level = float((cfg.get("initial_record_errors_meta") or {}).get("level", 0))
        slip = float(cfg.get("record_slip_p", 0))
        if level not in levels or cfg["seed"] not in seeds or slip != record_slip:
            raise ValueError(f"{folder}: error level, seed or record-slip differs from declared design")
        key = (day, scenario_key(cfg))
        cell = (day, level, cfg["seed"])
        if cell in design_cells and design_cells[cell] != key:
            raise ValueError(f"{folder}: two different scenarios occupy design cell {cell}")
        design_cells[cell] = key
        if name in grouped[key]:
            raise ValueError(f"duplicate {day} {name} scenario: {folder}")
        env = json.loads((folder / "run_env.json").read_text(encoding="utf-8"))
        common = {k: v for k, v in env.items() if k not in ("WES_SCHEDULER", "ASSIGN_WEIGHTS")}
        if shared_env[key] and common != next(iter(shared_env[key].values())):
            raise ValueError(f"{folder}: non-policy environment differs for paired runs")
        shared_env[key][name] = common
        summary = json.loads((folder / "summary.json").read_text(encoding="utf-8"))
        grouped[key][name] = {"backlog_auc_unit_s": summary["research_metrics"]["unit_backlog_auc_s"],
                              "mis_shipped": summary["mis_shipped"],
                              "released_units": summary["units_released"],
                              "shipped_units": summary["units_shipped"], "run": str(folder)}
    if not grouped:
        raise ValueError("no complete Webots run folders")
    incomplete = {day: sorted(set(policies) - set(rows)) for (day, _), rows in grouped.items()
                  if set(rows) != set(policies)}
    if incomplete:
        raise ValueError(f"unpaired policy runs: {incomplete}")
    covered = {day for day, _ in grouped}
    missing_design = [(day, level, seed) for day in sorted(expected) for level in levels for seed in seeds
                      if (day, level, seed) not in design_cells]
    missing = sorted(expected - covered)
    if missing_design and not allow_partial:
        raise ValueError(f"validation scenario cells missing: {missing_design}")
    scenarios = []
    for (day, key), rows in sorted(grouped.items()):
        scenarios.append({"date": day, "scenario_sha256": key, "policies": rows})
    means = {p: statistics.mean(s["policies"][p]["backlog_auc_unit_s"] for s in scenarios)
             for p in policies}
    eligible = [p for p in policies if all(s["policies"][p]["mis_shipped"] == 0 for s in scenarios)]
    winner = min(eligible, key=means.get) if not missing_design and eligible else None
    return {"metric": "unit_backlog_auc_s", "direction": "lower_is_better",
            "reference_engine": "Webots", "split": "validation", "expected_dates": len(expected),
            "covered_dates": len(covered), "missing_dates": missing,
            "missing_scenario_cells": missing_design,
            "status": "pilot_only" if missing_design else "validation_complete",
            "zero_misship_eligible_policies": eligible,
            "mean_backlog_auc_unit_s": means, "selected_policy": winner,
            "scenarios": scenarios}


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("runs", nargs="+", help="Webots run folders or globs")
    ap.add_argument("--split", type=Path, required=True)
    ap.add_argument("--policies", nargs="+", required=True)
    ap.add_argument("--levels", nargs="+", type=float, required=True)
    ap.add_argument("--seeds", nargs="+", type=int, required=True)
    ap.add_argument("--record-slip", type=float, required=True)
    ap.add_argument("--allow-partial", action="store_true", help="diagnostic only; no policy selection")
    ap.add_argument("--out", type=Path)
    args = ap.parse_args()
    dirs = sorted({Path(p) for pat in args.runs for p in glob.glob(pat)
                   if (Path(p) / "summary.json").exists()})
    try:
        result = evaluate(json.loads(args.split.read_text(encoding="utf-8")), dirs,
                          args.policies, args.levels, args.seeds, args.record_slip, args.allow_partial)
    except ValueError as exc:
        raise SystemExit(str(exc)) from exc
    output = json.dumps(result, ensure_ascii=False, indent=2) + "\n"
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(output, encoding="utf-8")
    sys.stdout.write(output)


if __name__ == "__main__":
    main()
