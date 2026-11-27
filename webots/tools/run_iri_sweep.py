"""Initial record error (IRI) sweep in the lightweight simulator (design: docs/experiments_iri.md).

    python tools/run_iri_sweep.py base      # make logs/iri_v1/base/<date>_s<seed>.json (overwrites tracked
                                            # scenario.json / worlds/warehouse.wbt, then git-restores them)
    python tools/run_iri_sweep.py run [--workers 11]
                                            # -> logs/iri_v1/results.csv, logs/iri_v1/manifest.json
Every run is one row. Level 0 is the same scenario for every mix, so it runs once per base
scenario and policy (mix "none"). A failed run is a manifest entry, never a silent gap.
"""
import argparse
import csv
import json
import os
import shutil
import subprocess
import sys
import tempfile
import time
import traceback
from multiprocessing import Pool
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / "lib"), str(ROOT / "tools")]
OUT = ROOT / "logs" / "iri_v1"
DATES = [("2023-03-09", []), ("2023-03-22", ["--order-sample", "0.5"]), ("2023-08-22", [])]
SEEDS = range(48, 58)
LEVELS = [0.01, 0.03, 0.065, 0.10, 0.20]
MIXES = ["ghost", "hidden", "mixed"]
POLICIES = {"team_wes": None, "hungarian": None,
            "cem_2day": "models/assign_weights_fault_base_cem_r0.json",
            "cem_18day": "models/assign_weights_fault_days18_cem_r0.json"}
FIELDS = ["date", "seed", "level", "mix", "policy", "k", "order_lead_all_s", "unit_lead_s", "order_lead_p90_s",
          "units_shipped", "units_released", "ship_rate", "missed_cutoffs", "SHORT_PICK", "DESTINATION_OCCUPIED",
          "ITEM_MISMATCH", "settled_mismatch_mean", "ghost_mean", "hidden_mean", "swap_mean", "unknown_mean"]


def base_name(date, seed, opts):
    return f"{date}{'half' if opts else ''}_s{seed}"


def make_base():
    (OUT / "base").mkdir(parents=True, exist_ok=True)
    try:
        for date, opts in DATES:
            for seed in SEEDS:
                subprocess.run([sys.executable, "tools/gen_world.py", "--plain", "--date", date, "--seed", str(seed),
                                *opts], cwd=ROOT, check=True, stdout=subprocess.DEVNULL)
                shutil.copy(ROOT / "scenario.json", OUT / "base" / f"{base_name(date, seed, opts)}.json")
    finally:
        for f in ("scenario.json", "worlds/warehouse.wbt"):
            os.chmod(ROOT / f, 0o666)
        subprocess.run(["git", "checkout", "--", "scenario.json", "worlds/warehouse.wbt"], cwd=ROOT, check=True)
    print(f"{len(list((OUT / 'base').glob('*.json')))} base scenarios in {OUT / 'base'}")


_grid = {}


def run_one(job):
    """job = (base file, level, mix, policy) -> row (dict) or {'error': ...}."""
    path, level, mix, policy = job
    name = Path(path).stem
    try:
        os.environ["VERIFY_POLICY"] = "event"
        import sched_hungarian, sched_learned, sched_verify
        from add_record_errors import add_record_errors
        from lite_sim import LiteSim
        from nav import GridMap
        from wes_runner import WES
        cfg = json.loads(Path(path).read_text(encoding="utf-8"))
        if level:
            cfg = add_record_errors(cfg, level, mix, cfg["seed"] + 1000)
        if "g" not in _grid:                           # one layout for every base scenario
            _grid["g"] = (GridMap(cfg), {})
        grid, fields = _grid["g"]
        if policy == "team_wes":
            sched = sched_verify.Scheduler(cfg)
        elif policy == "hungarian":
            sched = sched_hungarian.Scheduler(cfg)
        else:
            sched = sched_learned.Scheduler(cfg, weights=sched_learned.load_weights(ROOT / POLICIES[policy])[0])

        class Counting(LiteSim):
            """Ghost / hidden / swap / unknown rack slots, sampled with record quality (every 10 s)."""
            tally = {"ghost": 0, "hidden": 0, "swap": 0, "unknown": 0, "n": 0}

            def record_quality(self, t):
                super().record_quality(t)
                c = self.tally
                for sid, rec in self.wes.record_snapshot().items():
                    real = self.gt_sku(sid)
                    if rec == "UNKNOWN":
                        c["unknown"] += 1
                    elif rec is not None and real is None:
                        c["ghost"] += 1
                    elif rec is None and real is not None:
                        c["hidden"] += 1
                    elif rec != real:
                        c["swap"] += 1
                c["n"] += 1

        with tempfile.TemporaryDirectory() as tmp:
            sim = Counting(cfg, WES(cfg, scheduler=sched, log_dir=tmp), grid=grid, fields=fields)
            sim.tally = dict(Counting.tally)
            s = sim.run()
        ev, c = s["events"], sim.tally
        return {"date": cfg["outbound"]["date"] + ("half" if "half" in name else ""), "seed": cfg["seed"],
                "level": level, "mix": mix, "policy": policy,
                "k": cfg.get("initial_record_errors_meta", {}).get("k", 0),
                "order_lead_all_s": s["order_lead_all_s"], "unit_lead_s": s["unit_lead_s"],
                "order_lead_p90_s": s["order_lead_p90_s"], "units_shipped": s["units_shipped"],
                "units_released": s["units_released"],
                "ship_rate": s["units_shipped"] / s["units_released"] if s["units_released"] else None,
                "missed_cutoffs": s["missed_cutoffs"], "SHORT_PICK": ev.get("SHORT_PICK", 0),
                "DESTINATION_OCCUPIED": ev.get("DESTINATION_OCCUPIED", 0), "ITEM_MISMATCH": ev.get("ITEM_MISMATCH", 0),
                "settled_mismatch_mean": s["record_quality"]["settled_mismatch_mean"],
                **{f"{x}_mean": c[x] / c["n"] for x in ("ghost", "hidden", "swap", "unknown")}}
    except Exception:
        return {"error": traceback.format_exc(), "job": [name, level, mix, policy]}


def jobs():
    bases = sorted(str(p) for p in (OUT / "base").glob("*.json"))
    if len(bases) != len(DATES) * len(SEEDS):
        raise SystemExit(f"expected {len(DATES) * len(SEEDS)} base scenarios, found {len(bases)}: run 'base' first")
    return [(b, lv, mx, p) for b in bases for p in POLICIES
            for lv, mx in [(0.0, "none")] + [(lv, mx) for lv in LEVELS for mx in MIXES]]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("step", choices=("base", "run"))
    ap.add_argument("--workers", type=int, default=max(1, (os.cpu_count() or 2) - 1))
    a = ap.parse_args()
    if a.step == "base":
        return make_base()
    todo, t0 = jobs(), time.time()
    rows, failed = [], []
    with Pool(a.workers) as pool, open(OUT / "results.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, FIELDS)
        w.writeheader()
        for i, r in enumerate(pool.imap_unordered(run_one, todo, chunksize=4), 1):
            if "error" in r:
                failed.append(r)
            else:
                w.writerow(r)
                rows.append(r)
            if i % 100 == 0:
                f.flush()
                print(f"{i}/{len(todo)} done, {len(failed)} failed, {time.time() - t0:.0f} s", flush=True)
    (OUT / "manifest.json").write_text(json.dumps({
        "design": "docs/experiments_iri.md", "runs": len(todo), "ok": len(rows), "failed": failed,
        "git": subprocess.run(["git", "rev-parse", "HEAD"], cwd=ROOT, capture_output=True, text=True).stdout.strip(),
        "seconds": round(time.time() - t0)}, indent=1), encoding="utf-8")
    print(f"{len(rows)} ok, {len(failed)} failed -> {OUT / 'results.csv'}")


if __name__ == "__main__":
    main()
