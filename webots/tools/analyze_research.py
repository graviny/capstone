"""Complete-case gate, paired seed analysis, no significance/winner labels."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import numpy as np

METRICS = {'backlog_mean_units': lambda s:s['research_metrics']['unit_backlog_mean'],
           'restricted_order_flow_s': lambda s:s['research_metrics']['restricted_order_flow_mean_s'],
           'shipped': lambda s:s['units_shipped'],
           'unfinished_orders': lambda s:s['research_metrics']['orders_unfinished'],
           'settled_mismatch': lambda s:s['record_quality']['settled_mismatch_mean'],
           'missed_cutoffs': lambda s:s['missed_cutoffs']}


def paired_interval(values, seed=20261120, samples=10000):
    values = np.asarray(values, dtype=float)
    if len(values)<2 or not np.isfinite(values).all():
        raise ValueError('At least two finite paired observations are required')
    rng = np.random.default_rng(seed)
    draws = values[rng.integers(0,len(values),size=(samples,len(values)))].mean(axis=1)
    low, high = np.quantile(draws,[.025,.975])
    return {'mean': float(values.mean()), 'sd': float(values.std(ddof=1)),
            'ci95': [float(low),float(high)], 'n':len(values)}


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('suite', type=Path)
    args=ap.parse_args()
    suite=args.suite
    manifest=json.loads((suite/'manifest.json').read_text(encoding='utf-8'))
    completion=json.loads((suite/'completion.json').read_text(encoding='utf-8'))
    if not completion['complete'] or not completion['code_unchanged']:
        raise SystemExit('Invalid/incomplete suite or source changed: no pooled comparison produced.')
    rows={}
    hashes={}
    for method,n,seed,_ in manifest['planned_jobs']:
        path=suite/f't{n}_{method}_s{seed}'/'summary.json'
        summary=json.loads(path.read_text(encoding='utf-8'))
        rows[n,seed,method]=summary
        hashes[str(path.relative_to(suite))]=hashlib.sha256(path.read_bytes()).hexdigest()
    ns=sorted({k[0] for k in rows})
    methods=sorted({k[2] for k in rows})
    estimates=[]
    lines=['DEVELOPMENT ONLY — paired marginal 95% bootstrap intervals, not confirmatory claims.',
           'Same initial fixture; only fault seeds vary. No independent training-seed replication.',
           'No multiplicity correction; intervals are descriptive. Positive difference means method minus reference.', '']
    for n in ns:
        seeds=sorted({k[1] for k in rows if k[0]==n})
        for seed in seeds:
            tapes={rows[n,seed,m]['fault_tape']['tape_sha256'] for m in methods}
            if len(tapes)!=1:
                raise ValueError('Exogenous tape mismatch')
        for method in methods:
            means={name:float(np.mean([fun(rows[n,s,method]) for s in seeds])) for name,fun in METRICS.items()}
            lines.append(f'robots={n} method={method} means={json.dumps(means)}')
            for reference in ('baseline','heuristic'):
                if reference not in methods or reference==method:
                    continue
                for name,fun in METRICS.items():
                    diffs=[fun(rows[n,s,method])-fun(rows[n,s,reference]) for s in seeds]
                    estimates.append({'robots':n,'method':method,'reference':reference,'metric':name,
                                      **paired_interval(diffs),'paired_differences':diffs})
    (suite/'analysis.json').write_text(json.dumps({'summary_sha256':hashes,'estimates':estimates},indent=2),encoding='utf-8')
    for row in estimates:
        if row['metric'] in ('backlog_mean_units','restricted_order_flow_s'):
            lines.append(f"{row['robots']} {row['method']} minus {row['reference']} {row['metric']}: "
                         f"{row['mean']:.3f} CI {row['ci95']} n={row['n']}")
    (suite/'analysis.txt').write_text('\n'.join(lines)+'\n',encoding='utf-8')
    print('\n'.join(lines))


if __name__=='__main__':
    main()
