import argparse
import json
from pathlib import Path
import numpy as np
from analyze_research import METRICS, paired_interval


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('suite',type=Path)
    args=ap.parse_args()
    p=args.suite
    complete=json.loads((p/'completion.json').read_text(encoding='utf-8'))
    if not complete['complete'] or not complete['code_unchanged']:
        raise SystemExit('Invalid/incomplete suite')
    jobs=json.loads((p/'manifest.json').read_text(encoding='utf-8'))['planned_jobs']
    rows={}
    for method,n,seed,_,options in jobs:
        mode=options['fault_mode']
        s=json.loads((p/f'{mode}_t{n}_{method}_s{seed}'/'summary.json').read_text(encoding='utf-8'))
        if not s['validation']['replay_ok']:
            raise ValueError('Replay failed')
        rows[mode,seed,method]=s
    means=[]
    contrasts=[]
    for mode in sorted({k[0] for k in rows}):
        seeds=sorted({k[1] for k in rows if k[0]==mode})
        for method in ('baseline','heuristic','learned30'):
            mean={'mode':mode,'method':method,'n':len(seeds),
                  **{k:float(np.mean([f(rows[mode,s,method]) for s in seeds])) for k,f in METRICS.items()}}
            means.append(mean)
            print(mean)
            if method=='baseline':
                continue
            for metric,fun in METRICS.items():
                diffs=[fun(rows[mode,s,method])-fun(rows[mode,s,'baseline']) for s in seeds]
                stat=paired_interval(diffs) if len(diffs)>1 else {'mean':diffs[0],'ci95':None,'n':1}
                contrasts.append({'mode':mode,'method':method,'metric':metric,**stat})
    (p/'analysis.json').write_text(json.dumps({'purpose':'descriptive development only',
        'means':means,'contrasts_vs_baseline':contrasts},indent=2),encoding='utf-8')


if __name__=='__main__':
    main()
