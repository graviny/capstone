"""Predeclared development controls: no faults / S1 / S2 / S3.

Separates error mechanisms; does not tune or select a policy.
"""
import argparse
from concurrent.futures import ProcessPoolExecutor, as_completed
import importlib.metadata
import json
from pathlib import Path
import sys
import platform
from run_research import ROOT, ENV, execute, sources


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--suite',type=Path,required=True)
    ap.add_argument('--workers',type=int,default=3)
    ap.add_argument('--seeds',type=int,nargs='+',default=list(range(100,110)))
    ap.add_argument('--modes',nargs='+',default=['none','s1','s2','s3'])
    args=ap.parse_args()
    suite=args.suite.resolve()
    suite.mkdir(parents=True,exist_ok=False)
    code=sources()
    # No-fault runs have no exogenous variation: repeating them 10 times would
    # be pseudo-replication, not 10 independent environmental realizations.
    jobs=[(m,4,s,str(suite),{'fault_mode':mode}) for mode in args.modes
          for s in ([args.seeds[0]] if mode=='none' else args.seeds)
          for m in ['baseline','heuristic','learned30']]
    manifest={'purpose':'development fault mechanism controls; initial fixture fixed',
        'planned_jobs':jobs,'source_sha256':code,'environment':ENV,'python':sys.version,'platform':platform.platform(),
        'dependencies':{p:importlib.metadata.version(p) for p in ('numpy','scipy')},
        'rationale':'4 robots chosen before these controls as the constrained fleet, not a held-out benchmark; no-fault cell is deterministic.'}
    (suite/'manifest.json').write_text(json.dumps(manifest,indent=2),encoding='utf-8')
    for name in code:
        p=suite/'source'/name
        p.parent.mkdir(parents=True,exist_ok=True)
        p.write_bytes((ROOT/name).read_bytes())
    statuses=[]
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        futures=[pool.submit(execute,j) for j in jobs]
        for i,f in enumerate(as_completed(futures),1):
            s=f.result()
            statuses.append(s)
            print(f'{i}/{len(jobs)} {s}',flush=True)
    (suite/'completion.json').write_text(json.dumps({'complete':all(s['state']=='complete' for s in statuses),
        'code_unchanged':sources()==code,'statuses':statuses},indent=2),encoding='utf-8')


if __name__=='__main__':
    main()
