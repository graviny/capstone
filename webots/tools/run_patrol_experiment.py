"""Run the "do we need a patrol robot?" experiment in batch (docs/experiments_patrol.md).

Every combination of robot mix x verification policy x fault scale x seed: generate the
world (gen_world.py --plain), run Webots in batch, keep the logs. The world is one shared
file, so runs go one after another. A run whose summary.json exists is skipped, so an
interrupted sweep continues where it stopped. At the end the default world is generated
again (python tools/gen_world.py), so the repo is left as it was.

Usage (from webots/):
  python tools/run_patrol_experiment.py                         # the full plan below
  python tools/run_patrol_experiment.py --mixes 8+0 7+1 --policies none event \\
         --fault-scales 1 --seeds 0 --duration 600              # a quick check
  python tools/run_patrol_experiment.py --dry-run               # list the runs only

Then: python tools/summarize_patrol.py
"""
import argparse
import itertools
import os
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_WEBOTS = r"D:\Webots\msys64\mingw64\bin\webots.exe"
DATE = "2023-03-09"


def run_name(mix, policy, scale, duration):
    t, p = mix
    return f"pt_t{t}p{p}_{policy}_f{scale:g}_d{duration:g}"


def log_dir(mix, policy, scale, seed, duration):
    # Same rule as supervisor.py: <date>_<wes name>_seed<n>_<faults>_<run name>_batch
    faults = "faults" if scale > 0 else "nofaults"
    return ROOT / "logs" / f"{DATE}_wms-verify-{policy}_seed{seed}_{faults}_{run_name(mix, policy, scale, duration)}_batch"


def parse_mix(s):
    t, p = s.split("+")
    return int(t), int(p)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--mixes", nargs="+", default=["8+0", "7+1"], help="transports+patrols (same total)")
    ap.add_argument("--policies", nargs="+", default=["none", "event", "periodic"],
                    choices=["none", "event", "periodic", "both"])
    ap.add_argument("--fault-scales", nargs="+", type=float, default=[1.0, 2.0, 4.0])
    ap.add_argument("--seeds", nargs="+", type=int, default=[0, 1, 2, 3, 4])
    ap.add_argument("--duration", type=float, default=3600.0)
    ap.add_argument("--period", type=float, default=60.0, help="VERIFY_PERIOD_S for the periodic policy")
    ap.add_argument("--max-open", type=int, default=4, help="VERIFY_MAX_OPEN")
    ap.add_argument("--webots", default=os.environ.get("WEBOTS_EXE", DEFAULT_WEBOTS))
    ap.add_argument("--port", type=int, default=1300)
    ap.add_argument("--timeout", type=float, default=3 * 3600, help="seconds per run before giving up")
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--keep-world", action="store_true", help="do not regenerate the default world at the end")
    args = ap.parse_args()

    mixes = [parse_mix(m) for m in args.mixes]
    if len({t + p for t, p in mixes}) != 1:
        print("warning: the mixes do not have the same number of robots", file=sys.stderr)
    runs = list(itertools.product(mixes, args.policies, args.fault_scales, args.seeds))
    todo = [r for r in runs if not (log_dir(*r, args.duration) / "summary.json").exists()]
    print(f"{len(runs)} runs, {len(runs) - len(todo)} done already, {len(todo)} to go")
    if args.dry_run:
        for r in todo:
            print("  ", log_dir(*r, args.duration).name)
        return

    failed = []
    started = time.time()
    try:
        for k, (mix, policy, scale, seed) in enumerate(todo, 1):
            name = run_name(mix, policy, scale, args.duration)
            gen = [sys.executable, "tools/gen_world.py", "--plain", "--seed", str(seed),
                   "--transports", str(mix[0]), "--patrols", str(mix[1]), "--duration", str(args.duration),
                   "--run-name", name]
            gen += ["--no-faults"] if scale == 0 else ["--fault-scale", str(scale)]
            subprocess.run(gen, cwd=ROOT, check=True, stdout=subprocess.DEVNULL)
            env = {**os.environ, "QUIT_AT_END": "1", "WES_MODULE": "wes_runner", "WES_SCHEDULER": "sched_verify",
                   "VERIFY_POLICY": policy, "VERIFY_PERIOD_S": str(args.period),
                   "VERIFY_MAX_OPEN": str(args.max_open)}
            out = log_dir(mix, policy, scale, seed, args.duration)
            t0 = time.time()
            print(f"[{k}/{len(todo)}] {out.name} ...", flush=True)
            cmd = [args.webots, "--batch", "--mode=fast", "--minimize", "--no-rendering", "--stdout", "--stderr",
                   f"--port={args.port}", str(ROOT / "worlds" / "warehouse.wbt")]
            log = ROOT / "logs" / f"_webots_{out.name}.txt"
            log.parent.mkdir(exist_ok=True)
            try:
                with open(log, "w", encoding="utf-8", errors="replace") as f:
                    subprocess.run(cmd, cwd=ROOT, env=env, stdout=f, stderr=subprocess.STDOUT,
                                   timeout=args.timeout, check=False)
            except subprocess.TimeoutExpired:
                print("    timeout")
            if (out / "summary.json").exists():
                log.unlink(missing_ok=True)
                print(f"    done in {time.time() - t0:.0f} s", flush=True)
            else:
                failed.append(out.name)
                print(f"    FAILED (no summary.json), Webots output in {log.name}", flush=True)
    finally:
        if not args.keep_world:
            subprocess.run([sys.executable, "tools/gen_world.py"], cwd=ROOT, check=False, stdout=subprocess.DEVNULL)
    print(f"finished in {(time.time() - started) / 60:.0f} min, {len(failed)} failed")
    for n in failed:
        print("  failed:", n)


if __name__ == "__main__":
    main()
