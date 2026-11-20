"""Development runs on scenarios other than the training date/layout (generalization check).

Not confirmatory. Scenario files are copied into the suite with their hashes.
python tools/run_heldout.py --suite logs/heldout_v1 --scenarios a.json b.json --seeds 100 101
"""
import argparse
from concurrent.futures import ProcessPoolExecutor, as_completed
import hashlib
import json
from pathlib import Path
import platform
import sys
from run_research import ROOT, ENV, METHODS, execute, sources


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--suite', type=Path, required=True)
    ap.add_argument('--scenarios', type=Path, nargs='+', required=True)
    ap.add_argument('--methods', nargs='+', default=['baseline', 'heuristic', 'learned30'])
    ap.add_argument('--robots', type=int, nargs='+', default=[4])
    ap.add_argument('--seeds', type=int, nargs='+', default=[100, 101])
    ap.add_argument('--fault-mode', default='mixed')
    ap.add_argument('--fault-scale', type=float, default=1.0)
    ap.add_argument('--workers', type=int, default=3)
    args = ap.parse_args()
    suite = args.suite.resolve()
    suite.mkdir(parents=True, exist_ok=False)
    copies = {}
    for p in args.scenarios:
        dst = suite / 'scenarios' / p.name
        dst.parent.mkdir(exist_ok=True)
        dst.write_bytes(p.read_bytes())
        copies[p.stem] = str(dst)
    code = sources()
    jobs = [(m, n, s, str(suite), {'fault_mode': args.fault_mode, 'scenario': path, 'tag': tag, 'fault_scale': args.fault_scale})
            for tag, path in copies.items() for n in args.robots for s in args.seeds for m in args.methods]
    (suite / 'manifest.json').write_text(json.dumps({
        'purpose': 'development generalization check; not confirmatory', 'planned_jobs': jobs,
        'scenario_sha256': {t: hashlib.sha256(Path(p).read_bytes()).hexdigest() for t, p in copies.items()},
        'source_sha256': code, 'environment': ENV, 'python': sys.version, 'platform': platform.platform()},
        indent=2), encoding='utf-8')
    statuses = []
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        for i, f in enumerate(as_completed([pool.submit(execute, j) for j in jobs]), 1):
            statuses.append(f.result())
            print(i, len(jobs), {k: statuses[-1].get(k) for k in ('method', 'state', 'elapsed_s')}, flush=True)
    (suite / 'completion.json').write_text(json.dumps({
        'complete': all(s['state'] == 'complete' for s in statuses), 'code_unchanged': sources() == code,
        'statuses': statuses}, indent=2), encoding='utf-8')


if __name__ == '__main__':
    main()
