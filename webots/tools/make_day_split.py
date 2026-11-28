"""Make train, validation and test scenarios from a frozen date split; no training.

    python tools/make_day_split.py data/split_days30.json \
        [--train-seeds 42] [--test-seeds 54] [--levels 0.03 0.065 0.10] [--mix mixed]
Defaults are the constants TRAIN_SEEDS, TEST_SEEDS, LEVELS, MIX below; change them there or on the command line.
  1. data/orders_<date>.json for every split day (tools/prep_dataset.py; v3 uses all SKUs)
  2. logs/<name>/{train,validation,test}/<date>_s<seed>_l<level>.json: gen_world.py --plain (8 robots, base layout)
     plus record errors at t=0 (tools/add_record_errors.py, err_seed = seed + 1000) and the
     record slip rate during the episode (record_slip_p = --slip-p)
     train: one level per (day, seed), cycling through --levels in day order (one seed: the days
            share the levels; as many seeds as levels: every day gets every level).
     validation/test: every seed x every level
gen_world.py overwrites the tracked scenario.json and worlds/warehouse.wbt; they are git-restored at
the end. Do not run it while Webots is open. A seed changes the inbound trucks and which slots are
wrong, not the orders: order variety comes from the days.
"""
import argparse
import json
import os
import stat
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
from add_record_errors import MIXES, add_record_errors  # noqa: E402

TRAIN_SEEDS = [42]                 # scenario seeds per training day
VALIDATION_SEEDS = [43]            # validation dates, never used to fit the scheduler
TEST_SEEDS = [54]                  # scenario seeds per test day
LEVELS = [0.03, 0.065, 0.10]       # initial record error levels (share of stocked slots)
MIX = "mixed"                      # ghost, hidden, swap or mixed
SLIP_P = 0.01                      # record slips during the episode: share of completed puts recorded
                                   # one to three slots off (lib/wms.py _slip); 0 = off (ablation)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("split", type=Path)
    ap.add_argument("--train-seeds", type=int, nargs="+", default=TRAIN_SEEDS)
    ap.add_argument("--validation-seeds", type=int, nargs="+", default=VALIDATION_SEEDS)
    ap.add_argument("--test-seeds", type=int, nargs="+", default=TEST_SEEDS)
    ap.add_argument("--levels", type=float, nargs="+", default=LEVELS)
    ap.add_argument("--mix", choices=MIXES, default=MIX)
    ap.add_argument("--slip-p", type=float, default=SLIP_P)
    ap.add_argument("--out", type=Path, help="output folder (default logs/<split name>); e.g. a second folder "
                                              "with --slip-p 0 for the ablation")
    a = ap.parse_args()
    split = json.loads(a.split.read_text(encoding="utf-8"))
    validation = split.get("validation", [])
    groups = [set(split["train"]), set(validation), set(split["test"])]
    if any(groups[i] & groups[j] for i in range(3) for j in range(i + 1, 3)):
        raise SystemExit("train, validation, and test days must not overlap")
    subprocess.run([sys.executable, "tools/prep_dataset.py", "--dates",
                    ",".join(split["train"] + validation + split["test"])],
                   cwd=ROOT, check=True)
    pairs = [(d, s) for d in split["train"] for s in a.train_seeds]
    plan = [("train", d, s, [a.levels[i % len(a.levels)]]) for i, (d, s) in enumerate(pairs)]
    plan += [("validation", d, s, a.levels) for d in validation for s in a.validation_seeds]
    plan += [("test", d, s, a.levels) for d in split["test"] for s in a.test_seeds]
    if not 0 <= a.slip_p <= 1:
        raise SystemExit("--slip-p must be in [0, 1]")
    out = a.out or ROOT / "logs" / split["name"]
    tracked = {f: ((ROOT / f).read_bytes(), stat.S_IMODE((ROOT / f).stat().st_mode))
               for f in ("scenario.json", "worlds/warehouse.wbt")}
    try:
        for part, day, seed, levels in plan:
            (out / part).mkdir(parents=True, exist_ok=True)
            subprocess.run([sys.executable, "tools/gen_world.py", "--plain", "--date", day, "--seed", str(seed)],
                           cwd=ROOT, check=True, stdout=subprocess.DEVNULL)
            base = json.loads((ROOT / "scenario.json").read_text(encoding="utf-8"))
            for lv in levels:
                cfg = {**add_record_errors(base, lv, a.mix, seed + 1000), "record_slip_p": a.slip_p}
                (out / part / f"{day}_s{seed}_l{lv:g}.json").write_text(json.dumps(cfg, indent=1), encoding="utf-8")
    finally:
        for f in ("scenario.json", "worlds/warehouse.wbt"):
            os.chmod(ROOT / f, 0o666)
            (ROOT / f).write_bytes(tracked[f][0])
            os.chmod(ROOT / f, tracked[f][1])
    for part in ("train", "validation", "test"):
        print(f"{part}: {len(list((out / part).glob('*.json')))} scenarios in {out / part}")


if __name__ == "__main__":
    main()
