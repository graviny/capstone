"""Verify-rule analysis: paired vs random and vs oldest, budget check, calibration of p on random runs."""
import json
import sys
from pathlib import Path
import numpy as np

sys.path.insert(0, 'tools')
from analyze_suite import load, METRICS
from analyze_research import paired_interval

suite = Path(sys.argv[1])
rows = load(suite)
cells = sorted({k[:2] for k in rows})
methods = sorted({k[3] for k in rows})
out = []
for tag, n in cells:
    seeds = sorted({k[2] for k in rows if k[:2] == (tag, n)})
    print(f'\n== {tag} seeds={len(seeds)}')
    for m in methods:
        mean = lambda f: np.mean([f(rows[tag, n, s, m]) for s in seeds])
        print(f"{m:28s} mism={mean(METRICS['mismatch_mean']):5.2f} ttc={mean(METRICS['time_to_correct_s']):6.0f} "
              f"verify={mean(METRICS['verify_done']):5.1f} shipped={mean(METRICS['shipped']):6.1f} flow={mean(METRICS['flow_s']):6.0f}")
    for m in methods:
        for ref in ('baseline@both-random', 'baseline@both-oldest'):
            if m == ref or not m.startswith('baseline@both'):
                continue
            d = [METRICS['mismatch_mean'](rows[tag, n, s, m]) - METRICS['mismatch_mean'](rows[tag, n, s, ref]) for s in seeds]
            iv = paired_interval(d) if len(set(d)) > 1 else {'mean': d[0], 'ci95': [d[0], d[0]]}
            vd = np.mean([METRICS['verify_done'](rows[tag, n, s, m]) / max(METRICS['verify_done'](rows[tag, n, s, ref]), 1) for s in seeds])
            print(f"  {m} - {ref.split('@')[1]}: mism {iv['mean']:+.2f} [{iv['ci95'][0]:+.2f},{iv['ci95'][1]:+.2f}]  verify ratio {vd:.2f}")
            out.append({'cell': [tag, n], 'method': m, 'ref': ref, 'mismatch_diff': iv, 'verify_ratio': float(vd)})
# calibration on random runs
cal = {}
for k, s in rows.items():
    if k[3] == 'baseline@both-random':
        cal.setdefault(k[0].split('_')[0], []).extend(s['wes_stats'].get('verify_calibration') or [])
print('\nCalibration of p on random-rule checks (online-rate model), pooled over runs of the suite')
for d, v in cal.items():
    v = np.array(v)
    if len(v) == 0:
        continue
    p, y = v[:, 0], v[:, 1]
    brier = float(np.mean((p - y) ** 2)); base = float(np.mean((y.mean() - y) ** 2))
    print(f'  {d}: n={len(v)} mean p={p.mean():.4f} observed rate={y.mean():.4f} Brier={brier:.4f} (constant-rate Brier={base:.4f})')
Path(suite, 'analysis_verify.json').write_text(json.dumps(out, indent=2), encoding='utf-8')
