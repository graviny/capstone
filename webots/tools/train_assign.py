"""Train the robot assignment weights of lib/sched_learned.py in the lightweight simulator.

Method: cross-entropy method (CEM), a policy-search form of RL. Each iteration samples a
population of weight vectors around the current mean, runs every one of them on the same
batch of episodes (scenario x fault seed, so they are compared on equal terms), and moves
the mean to the best few. The weights are few (lib/sched_learned.FEATURES) and the reward
comes once per episode, which suits this method; no deep learning library is needed.

Cost of an episode (lower is better):
    mean order lead time (s) + 600 per unit released but not shipped + 300 per missed cutoff

Usage (from webots/):
  python tools/train_assign.py --train logs/train_scenarios/2023-03-09_*.json \\
         --test logs/train_scenarios/2023-03-22_*.json --robots 4
Writes models/assign_weights.json (after every iteration) and prints a test table that
compares the stand-in assignment, the untrained heuristic weights and the trained ones.
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

_cfgs, _layouts = {}, {}           # per worker process: scenario path -> cfg; layout -> (grid, fields)


def episode(job):
    """job = (scenario path, fault seed, robots, weights or None for the stand-in) -> (cost, summary bits)."""
    path, seed, robots, w = job
    if path not in _cfgs:
        _cfgs[path] = json.loads(Path(path).read_text(encoding="utf-8"))
    base = _cfgs[path]
    cfg = {**base, "seed": seed, "robots": base["robots"][:robots]}
    key = json.dumps([base["world"], base["racks"], base.get("obstacles")])
    if key not in _layouts:
        _layouts[key] = (GridMap(base), {})
    grid, fields = _layouts[key]
    sched = sched_verify.Scheduler(cfg) if w is None else sched_learned.Scheduler(cfg, weights=w)
    with tempfile.TemporaryDirectory() as tmp:
        wes = WES(cfg, scheduler=sched, log_dir=tmp)
        s = LiteSim(cfg, wes, grid=grid, fields=fields).run()
    lead = s["mean_order_lead_s"] or cfg["episode"]["duration_s"]
    cost = lead + 600 * (s["units_released"] - s["units_shipped"]) + 300 * s["missed_cutoffs"]
    busy = statistics.mean(s["robot_busy_share"].values())
    return cost, {"lead": lead, "shipped": s["units_shipped"], "released": s["units_released"],
                  "missed": s["missed_cutoffs"], "busy": busy, "dist": sum(s["robot_distance_m"].values())}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--train", nargs="+", required=True, help="scenario files (globs ok)")
    ap.add_argument("--test", nargs="+", default=[], help="held-out scenario files for the final table")
    ap.add_argument("--robots", type=int, default=4, help="use the first N robots of each scenario")
    ap.add_argument("--test-robots", type=int, nargs="+", help="robot counts for the test table (default: --robots)")
    ap.add_argument("--iters", type=int, default=30)
    ap.add_argument("--pop", type=int, default=24)
    ap.add_argument("--elite", type=int, default=6)
    ap.add_argument("--batch", type=int, default=6, help="episodes per candidate per iteration")
    ap.add_argument("--test-seeds", type=int, default=5)
    ap.add_argument("--sigma", type=float, default=1.0)
    ap.add_argument("--workers", type=int, default=max(1, (os.cpu_count() or 2) - 1))
    ap.add_argument("--out", type=Path, default=sched_learned.DEFAULT_WEIGHTS)
    ap.add_argument("--rng", type=int, default=0)
    args = ap.parse_args()
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    expand = lambda pats: sorted({p for pat in pats for p in glob.glob(pat)})  # noqa: E731
    train, test = expand(args.train), expand(args.test)
    if not train:
        raise SystemExit("no training scenarios")
    feats = sched_learned.FEATURES
    heur = [sched_learned.HEURISTIC.get(f, 0.0) for f in feats]
    rng = np.random.default_rng(args.rng)
    pick = random.Random(args.rng)
    mean, sd = np.array(heur), np.full(len(feats), args.sigma)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    history = []
    print(f"train {len(train)} scenarios, test {len(test)}, robots {args.robots}, workers {args.workers}")

    with Pool(args.workers) as pool:
        t0 = time.time()
        for it in range(1, args.iters + 1):
            batch = [(pick.choice(train), pick.randrange(1000)) for _ in range(args.batch)]
            pop = [mean] + [mean + sd * rng.standard_normal(len(feats)) for _ in range(args.pop - 1)]
            jobs = [(p, s, args.robots, list(w)) for w in pop for p, s in batch]
            jobs += [(p, s, args.robots, None) for p, s in batch]             # stand-in, for reference
            res = pool.map(episode, jobs)
            costs = [statistics.mean(c for c, _ in res[k * len(batch):(k + 1) * len(batch)]) for k in range(len(pop))]
            ref = statistics.mean(c for c, _ in res[len(pop) * len(batch):])
            order = np.argsort(costs)
            elite = np.array([pop[k] for k in order[:args.elite]])
            mean, sd = elite.mean(axis=0), np.maximum(elite.std(axis=0), 0.05)
            history.append({"iter": it, "best": costs[order[0]], "current_mean": costs[0], "standin": ref})
            print(f"iter {it:2d}: cost best {costs[order[0]]:.0f}, current {costs[0]:.0f}, stand-in {ref:.0f} "
                  f"| {time.time() - t0:.0f} s", flush=True)
            args.out.write_text(json.dumps({"weights": dict(zip(feats, map(float, mean))), "robots": args.robots,
                                            "train": train, "history": history}, indent=2), encoding="utf-8")

        print("\nweights:", {f: round(float(v), 2) for f, v in zip(feats, mean)})
        if not test:
            return
        table = {}
        for n in args.test_robots or [args.robots]:
            eps = [(p, s) for p in test for s in range(args.test_seeds)]
            for label, w in (("stand-in", None), ("heuristic", heur), ("trained", list(mean))):
                res = pool.map(episode, [(p, s, n, w) for p, s in eps])
                table[(n, label)] = res
            print(f"\ntest, {n} robots, {len(eps)} episodes (mean; paired diff vs stand-in)")
            base = table[(n, "stand-in")]
            for label in ("stand-in", "heuristic", "trained"):
                r = table[(n, label)]
                m = lambda k: statistics.mean(x[1][k] for x in r)  # noqa: E731
                diff = [a[1]["lead"] - b[1]["lead"] for a, b in zip(r, base)]
                print(f"  {label:9s} cost {statistics.mean(x[0] for x in r):6.0f} | lead {m('lead'):5.0f} s "
                      f"(diff {statistics.mean(diff):+5.0f}, sd {statistics.pstdev(diff):4.0f}) | shipped "
                      f"{m('shipped'):5.1f}/{m('released'):.0f} | missed {m('missed'):.1f} | busy {m('busy'):.3f} "
                      f"| dist {m('dist'):5.0f} m")


if __name__ == "__main__":
    main()
