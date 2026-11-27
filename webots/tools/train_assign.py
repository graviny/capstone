"""Train the robot assignment weights of lib/sched_learned.py in the lightweight simulator.

Method: cross-entropy method (CEM), a policy-search form of RL. Each iteration samples a
population of weight vectors around the current mean, runs every one of them on the same
batch of episodes (the same scenarios, so they are compared on equal terms), and moves
the mean to the best few. The weights are few (lib/sched_learned.FEATURES) and the reward
comes once per episode, which suits this method; no deep learning library is needed.

Cost of an episode (lower is better): one of COSTS, --cost (default: see COSTS).
The test table always reports the same metrics for every cost, so costs can be compared:
order_all (lead of every released order, unfinished ones counted to the end) is the main one.

Usage (from webots/):
  python tools/train_assign.py --train "logs/train_scenarios_v2/2023-03-09_*.json" \\
         --test "logs/train_scenarios_v2/*.json" --test-seeds 100 101 102 103 104 --robots 8
Writes the weights (--out, after every iteration) and, with --report, the test results as JSON.
"""
import argparse
import glob
import json
import os
import random
import statistics
import sys
import tempfile
import time
from multiprocessing import Pool
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "lib"))
os.environ.setdefault("VERIFY_POLICY", "event")

import sched_learned  # noqa: E402
import sched_verify  # noqa: E402
from lite_sim import LiteSim  # noqa: E402
from nav import GridMap  # noqa: E402
from wes_runner import WES  # noqa: E402

# Episode cost candidates (lower is better), from the episode metrics m (see episode()).
COSTS = {
    "old": lambda m: m["lead"] + 600 * (m["released"] - m["shipped"]) + 300 * m["missed"],
    "order_all": lambda m: m["order_all"],
    "unit": lambda m: m["unit"],
    "order_all_missed": lambda m: m["order_all"] + 300 * m["missed"],
    "unit_missed": lambda m: m["unit"] + 300 * m["missed"],
    "tardy": lambda m: m["tardy"],
    "tardy_unit": lambda m: m["tardy"] + m["unit"],
    "p90": lambda m: m["p90"],
    "order_all_dist": lambda m: m["order_all"] + m["dist"] / 50,
    "mix": lambda m: 0.5 * m["order_all"] + 0.5 * m["unit"] + MISSED_W * m["missed"],
}
MISSED_W = 300.0       # mix's cost per missed cutoff (--missed-w)
DEFAULT_COST = "mix"   # chosen 2026-11-20 by the cost study (docs/lite_sim.md)
METRICS = ("order_all", "unit", "p90", "tardy", "lead", "shipped", "released", "missed", "busy", "dist")
_cfgs, _layouts = {}, {}           # per worker process: scenario path -> cfg; layout -> (grid, fields)


def episode(job):
    """job = (scenario path, seed, robots, weights or None for the stand-in) -> metrics."""
    path, seed, robots, w = job
    window = os.environ.get("ASSIGN_WINDOW", "feasible")
    if path not in _cfgs:
        _cfgs[path] = json.loads(Path(path).read_text(encoding="utf-8"))
    base = _cfgs[path]
    cfg = {**base, "seed": seed, "robots": base["robots"][:robots]}
    key = json.dumps([base["world"], base["racks"], base.get("obstacles")])
    if key not in _layouts:
        _layouts[key] = (GridMap(base), {})
    grid, fields = _layouts[key]
    sched = sched_verify.Scheduler(cfg) if w is None else sched_learned.Scheduler(cfg, weights=w)
    if w is not None:
        sched.window = window
    with tempfile.TemporaryDirectory() as tmp:
        wes = WES(cfg, scheduler=sched, log_dir=tmp)
        s = LiteSim(cfg, wes, grid=grid, fields=fields).run()
    end = cfg["episode"]["duration_s"]
    return {"order_all": s["order_lead_all_s"] or end, "unit": s["unit_lead_s"] or end,
            "p90": s["order_lead_p90_s"] or end, "tardy": s["order_tardiness_s"] or 0.0,
            "lead": s["mean_order_lead_s"] or end, "shipped": s["units_shipped"], "released": s["units_released"],
            "missed": s["missed_cutoffs"], "busy": statistics.mean(s["robot_busy_share"].values()),
            "dist": sum(s["robot_distance_m"].values())}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--train", nargs="+", required=True, help="scenario files (globs ok)")
    ap.add_argument("--test", nargs="+", default=[], help="held-out scenario files for the final table")
    ap.add_argument("--cost", choices=sorted(COSTS), default=DEFAULT_COST)
    ap.add_argument("--robots", type=int, default=8, help="use the first N robots of each scenario")
    ap.add_argument("--iters", type=int, default=15)
    ap.add_argument("--pop", type=int, default=24)
    ap.add_argument("--elite", type=int, default=6)
    ap.add_argument("--batch", type=int, default=6, help="episodes per candidate per iteration "
                    "(0: every training scenario once, same every iteration)")
    ap.add_argument("--missed-w", type=float, default=300.0, help="mix cost per missed cutoff")
    ap.add_argument("--test-seeds", type=int, nargs="+", default=[100, 101, 102, 103, 104])
    ap.add_argument("--sigma", type=float, default=1.0)
    ap.add_argument("--workers", type=int, default=max(1, (os.cpu_count() or 2) - 1))
    ap.add_argument("--out", type=Path, default=sched_learned.DEFAULT_WEIGHTS)
    ap.add_argument("--report", type=Path, help="write the test results here (JSON)")
    ap.add_argument("--rng", type=int, default=0)
    ap.add_argument("--method", choices=("cem", "cma"), default="cem",
                    help="cem: mean of the --elite best; cma: CMA-ES (pip install cma), --elite unused")
    args = ap.parse_args()
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    expand = lambda pats: sorted({p for pat in pats for p in glob.glob(pat)})  # noqa: E731
    train, test = expand(args.train), expand(args.test)
    if not train:
        raise SystemExit("no training scenarios")
    global MISSED_W
    MISSED_W = args.missed_w
    cost = COSTS[args.cost]
    feats = sched_learned.FEATURES
    heur = [sched_learned.HEURISTIC.get(f, 0.0) for f in feats]
    rng = np.random.default_rng(args.rng)
    pick = random.Random(args.rng)
    mean, sd = np.array(heur), np.full(len(feats), args.sigma)
    es = None
    if args.method == "cma":
        import cma  # pip install cma
        es = cma.CMAEvolutionStrategy(heur, args.sigma, {"popsize": args.pop - 1, "seed": args.rng + 1,
                                                         "verbose": -9})
    args.out.parent.mkdir(parents=True, exist_ok=True)
    history = []
    print(f"cost {args.cost}: train {len(train)} scenarios, test {len(test)}, robots {args.robots}, "
          f"workers {args.workers}")

    with Pool(args.workers) as pool:
        t0 = time.time()
        for it in range(1, args.iters + 1):
            batch = ([(p, 0) for p in train] if args.batch == 0
                     else [(pick.choice(train), pick.randrange(1000)) for _ in range(args.batch)])
            if es:  # CMA-ES: same budget, candidate 0 is still the current mean
                asked = es.ask()
                pop = [np.array(es.mean)] + asked
            else:
                pop = [mean] + [mean + sd * rng.standard_normal(len(feats)) for _ in range(args.pop - 1)]
            jobs = [(p, s, args.robots, list(w)) for w in pop for p, s in batch]
            jobs += [(p, s, args.robots, None) for p, s in batch]             # stand-in, for reference
            res = [cost(m) for m in pool.map(episode, jobs)]
            nb = len(batch)
            costs = [statistics.mean(res[k * nb:(k + 1) * nb]) for k in range(len(pop))]
            ref = statistics.mean(res[len(pop) * nb:])
            order = np.argsort(costs)
            elite = np.array([pop[k] for k in order[:args.elite]])
            if es:
                es.tell(asked, costs[1:])
                mean, sd = np.array(es.mean), np.array(es.stds)
            else:
                mean, sd = elite.mean(axis=0), np.maximum(elite.std(axis=0), 0.05)
            history.append({"iter": it, "best": costs[order[0]], "current_mean": costs[0], "standin": ref,
                            "mean": [round(float(v), 4) for v in mean], "sd": [round(float(v), 4) for v in sd]})
            print(f"iter {it:2d}: cost best {costs[order[0]]:.0f}, current {costs[0]:.0f}, stand-in {ref:.0f} "
                  f"| {time.time() - t0:.0f} s", flush=True)
            args.out.write_text(json.dumps({"weights": dict(zip(feats, map(float, mean))), "cost": args.cost,
                                            "robots": args.robots, "train": train, "history": history}, indent=2),
                                encoding="utf-8")

        print("\nweights:", {f: round(float(v), 2) for f, v in zip(feats, mean)})
        if not test:
            return
        eps = [(p, s) for p in test for s in args.test_seeds]
        report = {"cost": args.cost, "robots": args.robots, "episodes": len(eps), "weights": dict(zip(feats, map(float, mean))),
                  "results": {}}
        rows = {}
        for label, w in (("stand-in", None), ("heuristic", heur), ("trained", list(mean))):
            rows[label] = pool.map(episode, [(p, s, args.robots, w) for p, s in eps])
            report["results"][label] = {k: statistics.mean(m[k] for m in rows[label]) for k in METRICS}
        report["paired_order_all_vs_standin"] = {
            label: {"mean": statistics.mean(a["order_all"] - b["order_all"] for a, b in zip(rows[label], rows["stand-in"])),
                    "sd": statistics.pstdev(a["order_all"] - b["order_all"] for a, b in zip(rows[label], rows["stand-in"]))}
            for label in ("heuristic", "trained")}
        print(f"\ntest, {args.robots} robots, {len(eps)} episodes (mean)")
        for label, r in report["results"].items():
            print(f"  {label:9s} order_all {r['order_all']:5.0f} | unit {r['unit']:5.0f} | p90 {r['p90']:5.0f} | "
                  f"tardy {r['tardy']:5.0f} | shipped {r['shipped']:5.1f}/{r['released']:.0f} | missed {r['missed']:.2f} "
                  f"| busy {r['busy']:.3f} | dist {r['dist']:5.0f}")
        if args.report:
            args.report.parent.mkdir(parents=True, exist_ok=True)
            args.report.write_text(json.dumps(report, indent=2), encoding="utf-8")


if __name__ == "__main__":
    main()
