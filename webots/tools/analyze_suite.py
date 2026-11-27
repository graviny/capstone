"""Per-scenario paired analysis of a run_heldout suite. Descriptive only.

Unit of replication = fault seed WITHIN one scenario (initial stock/inbound fixed), so intervals
are conditional on that scenario; scenarios are reported side by side, never pooled as if independent.
python tools/analyze_suite.py logs/heldout_v1 [reference=baseline]
"""
import json
import sys
from pathlib import Path
import numpy as np
from analyze_research import paired_interval


def first_wait(kind):
    return lambda s: s['research_metrics']['tasks'][kind]['max_first_assignment_wait_s'] or 0.0


METRICS = {
    'shipped': lambda s: s['units_shipped'],
    'released': lambda s: s['units_released'],
    'flow_s': lambda s: s['research_metrics']['restricted_order_flow_mean_s'],
    'backlog_units': lambda s: s['research_metrics']['unit_backlog_mean'],
    'unfinished': lambda s: s['research_metrics']['orders_unfinished'],
    'max_wait_pick_s': first_wait('PICK'),
    'max_wait_putaway_s': first_wait('PUTAWAY'),
    'verify_done': lambda s: s['wes_stats']['tasks'].get('VERIFY_COMPLETED', 0),
    'mismatch_mean': lambda s: s['record_quality']['settled_mismatch_mean'],
    'time_to_correct_s': lambda s: s['record_quality']['mean_time_to_correct_s'] or 0.0,
    'missed_cutoffs': lambda s: s['missed_cutoffs'],
}


def load(suite):
    manifest = json.loads((suite / 'manifest.json').read_text(encoding='utf-8'))
    rows = {}
    for method, n, seed, _, opt in manifest['planned_jobs']:
        d = suite / f"{opt['tag']}_{opt['fault_mode']}_t{n}_{method}_s{seed}"
        s = json.loads((d / 'summary.json').read_text(encoding='utf-8'))
        if not s['validation']['replay_ok'] or s['validation']['wms_errors'] or s['mis_shipped']:
            raise SystemExit(f'invalid run {d.name}')
        rows[opt['tag'], n, seed, method] = s
    return rows


def main(suite, ref='baseline'):
    rows = load(Path(suite))
    cells = sorted({k[:2] for k in rows})
    methods = sorted({k[3] for k in rows})
    report = []
    for tag, n in cells:
        seeds = sorted({k[2] for k in rows if k[:2] == (tag, n)})
        print(f'\n== {tag} robots={n} fault seeds={len(seeds)}')
        for m in methods:
            line = {k: float(np.mean([f(rows[tag, n, s, m]) for s in seeds])) for k, f in METRICS.items()}
            print(f'{m:14s}', ' '.join(f'{k}={v:.1f}' for k, v in line.items()))
            entry = {'scenario': tag, 'robots': n, 'method': m, 'means': line, 'vs_' + ref: {}}
            if m != ref:
                for k, f in METRICS.items():
                    d = [f(rows[tag, n, s, m]) - f(rows[tag, n, s, ref]) for s in seeds]
                    entry['vs_' + ref][k] = paired_interval(d) if len(set(d)) > 1 else {'mean': d[0], 'ci95': [d[0], d[0]], 'n': len(d)}
            report.append(entry)
        for m in methods:
            if m == ref:
                continue
            e = [x for x in report if x['scenario'] == tag and x['method'] == m][0]['vs_' + ref]
            print(f'  {m} - {ref}: ' + '; '.join(f"{k} {e[k]['mean']:+.1f} [{e[k]['ci95'][0]:+.1f},{e[k]['ci95'][1]:+.1f}]"
                  for k in ('flow_s', 'shipped', 'max_wait_pick_s', 'verify_done', 'mismatch_mean', 'time_to_correct_s')))
    Path(suite, 'analysis_suite.json').write_text(json.dumps(report, indent=2), encoding='utf-8')


if __name__ == '__main__':
    main(*sys.argv[1:3])
