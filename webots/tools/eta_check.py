"""Compare the nominal route/service ETA (lib/travel_time.py) with durations seen in an engine log.

Observed duration = TASK_STATUS COMPLETED time minus the (last) ASSIGNED time. Robot position at
assignment = last pose sample at or before that time (poses.csv). Descriptive only: one run, one
layout; does not calibrate anything. Needs the run's own scenario.json.

python tools/eta_check.py <run log dir> <scenario.json>
"""
import csv
import json
import statistics
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'lib'))
from travel_time import TravelTime  # noqa: E402


def main(run, scenario):
    cfg = json.loads(Path(scenario).read_text(encoding='utf-8'))
    tt = TravelTime(cfg)
    rows = [json.loads(l) for l in (Path(run) / 'wms_log.jsonl').read_text(encoding='utf-8').splitlines()]
    poses = {}
    for r in csv.DictReader((Path(run) / 'poses.csv').open(encoding='utf-8')):
        poses.setdefault(int(r['robot']), []).append((float(r['t']), float(r['x']), float(r['y'])))
    task, assign, done, src, dst = {}, {}, {}, {}, {}
    for r in rows:
        k = r['type']
        if k == 'TASK_CREATED':
            task[r['task_id']] = r['task_type']
            dst[r['task_id']] = r.get('destination')
            src[r['task_id']] = r.get('source')
        elif k == 'TASK_STATUS' and r['new'] == 'ASSIGNED':
            assign[r['task_id']] = (r['t'], r['robot_id'])
        elif k == 'TASK_STATUS' and r['new'] == 'COMPLETED':
            done[r['task_id']] = r['t']
        elif k == 'TASK_PICKED':
            src[r['task_id']] = r['source']
        elif k in ('PUTAWAY_SLOT', 'TASK_REROUTED'):
            dst[r['task_id']] = r['destination']
            if k == 'TASK_REROUTED':
                src[r['task_id']] = r['source']
        elif k == 'SLOT' and r['cause'] == 'TASK_COMPLETED' and r.get('new') and task.get(r['task_id']) == 'PICK':
            dst[r['task_id']] = r['slot']
    out = {}
    for tid, (ta, rid) in assign.items():
        if tid not in done or not dst.get(tid):
            continue
        kind = task[tid]
        if kind != 'VERIFY' and not src.get(tid):
            continue
        samples = [p for p in poses[rid] if p[0] <= ta]
        if not samples:
            continue
        _, x, y = samples[-1]
        eta = tt.completion((x, y), {'task_type': kind}, (src.get(tid), dst[tid]))
        # Straight-line comparator with the same service times (what a distance feature implies).
        import math
        pts = [(x, y)] + ([tt.point(src[tid])] if kind != 'VERIFY' else []) + [tt.point(dst[tid])]
        line = sum(math.dist(a, b) for a, b in zip(pts, pts[1:])) / tt.speed
        line += tt.look if kind == 'VERIFY' else 2 * (tt.look + tt.act)
        out.setdefault(kind, []).append((done[tid] - ta, eta, line))
    result = {}
    for kind, v in sorted(out.items()):
        obs, eta = [a for a, _, _ in v], [b for _, b, _ in v]
        err = [e - o for o, e, _ in v]
        result[kind] = {'n': len(v), 'obs_mean_s': statistics.mean(obs), 'eta_mean_s': statistics.mean(eta),
                        'bias_s': statistics.mean(err), 'mae_s': statistics.mean(abs(e) for e in err),
                        'median_ratio_obs_over_eta': statistics.median(o / e for o, e, _ in v if e > 0),
                        'straight_line_mae_s': statistics.mean(abs(c - o) for o, _, c in v),
                        'straight_line_bias_s': statistics.mean(c - o for o, _, c in v),
                        'p90_abs_err_s': sorted(abs(e) for e in err)[int(.9 * (len(err) - 1))]}
        if len(v) > 2:
            result[kind]['corr'] = statistics.correlation(obs, eta)
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main(*sys.argv[1:3])
