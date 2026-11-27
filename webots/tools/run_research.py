"""Frozen, resumable development matrix. Not a confirmatory experiment.

python tools/run_research.py --suite logs/research_dev_v1 --workers 3
python tools/run_research.py --suite logs/research_dev_v1 --resume
"""
import argparse
from concurrent.futures import ProcessPoolExecutor, as_completed
import copy
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import platform
import subprocess
import sys
import time
import traceback

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'lib'))
METHODS = ['baseline', 'edf', 'heuristic', 'learned30', 'learned_all', 'learned_fw', 'learned_fw_age0', 'learned_fw_cap300', 'learned_fw_cap600', 'learned_fw_cap1200', 'retrain0_fw', 'adapt_fw', 'eta_greedy', 'eta_matching']
ENV = {'VERIFY_POLICY': 'event', 'VERIFY_PERIOD_S': '60', 'VERIFY_MAX_OPEN': '4',
       'VERIFY_RECENT_S': '120', 'VERIFY_NEIGHBORS': '3'}
_grid, _fields = None, {}


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def sources():
    files = [p for folder in ('lib', 'controllers', 'tools', 'tests') for p in (ROOT/folder).rglob('*.py')]
    files += [ROOT/'scenario.json', ROOT/'models/assign_weights.json', ROOT/'models/assign_weights_fw_rng0.json', ROOT/'docs/research_protocol.md']
    return {str(p.relative_to(ROOT)).replace('\\','/'): digest(p) for p in sorted(files)}


def execute(job):
    global _grid
    os.environ.update(ENV)
    import sched_verify
    import sched_learned
    import sched_research
    from nav import GridMap
    from lite_sim import LiteSim
    from wes_runner import WES
    method, n, seed, suite = job[:4]
    full_method = method
    if '@' in method:         # <assignment>@<verify policy>-<tick rule>[-config] e.g. baseline@both-value
        method, spec = method.split('@')
        parts = spec.split('-')
        os.environ.update(VERIFY_POLICY=parts[0], VERIFY_TICK_RULE=parts[1],
                          VERIFY_RATE_SOURCE='config' if 'config' in parts[2:] else 'online')
    else:
        os.environ.pop('VERIFY_TICK_RULE', None)
        os.environ.pop('VERIFY_RATE_SOURCE', None)
    options = job[4] if len(job) > 4 else {}
    mode = options.get('fault_mode', 'mixed')
    scenario = options.get('scenario')
    tag = options.get('tag', '')
    prefix = (tag + '_' if tag else '') + (mode + '_' if options else '')
    folder = Path(suite) / f'{prefix}t{n}_{full_method}_s{seed}'
    folder.mkdir(exist_ok=False)
    started = time.perf_counter()
    status = {'method': full_method, 'robots': n, 'fault_seed': seed, 'state': 'running'}
    wes = None
    try:
        cfg = json.loads((Path(scenario) if scenario else ROOT/'scenario.json').read_text(encoding='utf-8'))
        cfg.update(fault_model='marked-poisson-v1', fault_seed=seed, robots=cfg['robots'][:n])
        cfg['faults'] = dict(cfg['faults'])
        scale = float(options.get('fault_scale', 1.0))
        for key in ('worker_move_per_h', 'worker_remove_per_h', 'dest_taken_p'):
            cfg['faults'][key] = cfg['faults'][key] * scale
        if mode not in ('mixed','none','s1','s2','s3'):
            raise ValueError(mode)
        if mode != 'mixed':
            active = {'s1':'worker_move_per_h','s2':'worker_remove_per_h','s3':'dest_taken_p'}.get(mode)
            for key in ('worker_move_per_h','worker_remove_per_h','dest_taken_p'):
                if key != active:
                    cfg['faults'][key] = 0
        status['fault_mode'] = mode
        if method == 'baseline':
            scheduler = sched_verify.Scheduler(cfg)
        elif method in ('edf', 'eta_greedy', 'eta_matching'):
            scheduler = sched_research.Scheduler(cfg, method=method)
        else:
            weights = (sched_learned.load_weights(ROOT/('models/assign_weights_fw_rng0.json' if method.startswith('retrain0')
                                                        else 'models/assign_weights.json'))[0]
                       if method.startswith(('learned', 'retrain', 'adapt')) else [sched_learned.HEURISTIC.get(f, 0) for f in sched_learned.FEATURES])
            scheduler = sched_learned.Scheduler(cfg, weights=weights)
            scheduler.window = 'raw'   # learned30/learned_all/heuristic keep their original window; *_fw set it below
            if method == 'learned_all':
                scheduler.candidate_limit = None
            if method == 'adapt_fw':
                scheduler.w_low = list(weights)      # deployed weights (age<0): sub-saturated load
                scheduler.w_high = sched_learned.load_weights(ROOT/'models/assign_weights_fw_rng0.json')[0]
                scheduler.window = 'feasible'
            if method.startswith(('learned_fw', 'retrain')):
                scheduler.window = 'feasible'
                if method.endswith('age0'):
                    scheduler.age_mode = 'zero'
                if '_cap' in method:
                    scheduler.max_wait = float(method.rsplit('cap', 1)[1])
        layout = json.dumps([cfg['world'], cfg['racks'], cfg.get('obstacles', [])], sort_keys=True)
        if _grid is None or _grid[0] != layout:
            _grid = (layout, GridMap(cfg))
            _fields.clear()
        wes = WES(cfg, scheduler=scheduler, log_dir=folder)
        timings = []
        plan = scheduler.plan
        def measured(state):
            start = time.perf_counter()
            result = plan(state)
            timings.append(time.perf_counter() - start)
            return result
        scheduler.plan = measured
        with (folder/'events.jsonl').open('w',encoding='utf-8') as trace:
            sim = LiteSim(cfg, wes, grid=_grid[1], fields=_fields,
                          event_sink=lambda event:trace.write(json.dumps(event)+'\n'))
            summary = sim.run()
        import numpy as np
        summary['policy_compute'] = {'calls': len(timings), 'total_s': sum(timings),
            'median_ms': float(np.median(timings)*1000), 'p95_ms': float(np.percentile(timings,95)*1000),
            'max_ms': max(timings)*1000,
            'note': 'Includes cold route cache; wall time measured under concurrent development runs.'}
        counts = summary['wes_stats']['counts']
        bad = {k:v for k,v in counts.items() if v and (k.startswith('anomaly_') or 'reject' in k)}
        sys.path.insert(0, str(ROOT/'tools'))
        from replay_wms import check
        replay = check(cfg,folder)
        replay_ok = replay['same'] and replay['scenario_ok']
        summary['validation'] = {'wms_errors': bad, 'mis_shipped': summary['mis_shipped'],
                                 'replay_ok': replay_ok, 'replay_diff': replay['diff']}
        (folder/'summary.json').write_text(json.dumps(summary, indent=2), encoding='utf-8')
        (folder/'fault_audit.json').write_text(json.dumps(sim.fault_tape.audit, indent=2), encoding='utf-8')
        (folder/'fault_tape.json').write_text(json.dumps(sim.fault_tape.events, indent=2), encoding='utf-8')
        status['state'] = 'complete' if not bad and not summary['mis_shipped'] and replay_ok else 'invalid'
    except Exception:
        status.update(state='failed', error=traceback.format_exc())
        if wes:
            wes.wms.close()
    finally:
        status['elapsed_s'] = time.perf_counter() - started
        (folder/'status.json').write_text(json.dumps(status, indent=2), encoding='utf-8')
    return status


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--suite', type=Path, required=True)
    ap.add_argument('--workers', type=int, default=3)
    ap.add_argument('--seeds', type=int, nargs='+', default=list(range(100,110)))
    ap.add_argument('--robots', type=int, nargs='+', default=[4,8])
    ap.add_argument('--methods', nargs='+', choices=METHODS, default=METHODS)
    ap.add_argument('--resume', action='store_true')
    args = ap.parse_args()
    if hasattr(sys.stdout, 'reconfigure'):
        sys.stdout.reconfigure(encoding='utf-8')
    suite = args.suite.resolve()
    code = sources()
    jobs = [(m,n,s,str(suite)) for n in args.robots for s in args.seeds for m in args.methods]
    manifest = {'purpose': 'development-only; no held-out generalization claim', 'source_sha256': code,
        'commit': subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
        'environment': ENV, 'python': sys.version, 'platform': platform.platform(),
        'planned_jobs': jobs, 'workers': args.workers, 'started_utc': datetime.now(timezone.utc).isoformat()}
    if args.resume:
        old = json.loads((suite/'manifest.json').read_text(encoding='utf-8'))
        if old['source_sha256'] != code or old['planned_jobs'] != [list(j) for j in jobs]:
            raise SystemExit('Resume rejected: code, configuration or planned jobs changed. Use a new suite.')
    else:
        suite.mkdir(parents=True, exist_ok=False)
        (suite/'manifest.json').write_text(json.dumps(manifest, indent=2), encoding='utf-8')
        # Save exact code, not just hashes of files that may later change.
        for name in code:
            target = suite/'source'/name
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes((ROOT/name).read_bytes())
    todo = [j for j in jobs if not (suite/f't{j[1]}_{j[0]}_s{j[2]}'/'status.json').exists()]
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        futures = [pool.submit(execute, j) for j in todo]
        for i, future in enumerate(as_completed(futures), 1):
            status = future.result()
            print(f'{i}/{len(todo)} {status}', flush=True)
    statuses = [json.loads((suite/f't{n}_{m}_s{s}'/'status.json').read_text(encoding='utf-8')) for m,n,s,_ in jobs]
    (suite/'completion.json').write_text(json.dumps({'statuses': statuses,
        'complete': all(s['state']=='complete' for s in statuses), 'code_unchanged': sources()==code,
        'finished_utc': datetime.now(timezone.utc).isoformat()},indent=2), encoding='utf-8')
    if not all(s['state']=='complete' for s in statuses):
        raise SystemExit(1)


if __name__ == '__main__':
    main()
