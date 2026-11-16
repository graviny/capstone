"""Check the lightweight simulator (lib/lite_sim.py) against the Webots runs in logs/.

For every Webots batch run with a summary.json and the scenario.json it used, run the same
scenario and WES in the lightweight simulator with several fault seeds, then report
  1. accuracy   per run: Webots value vs lightweight mean, for each metric
  2. ranking    paired differences between runs that differ only in the WES / robot mix:
                does the lightweight simulator give the same direction as Webots?
  3. faults     fault-driven metrics averaged per condition (single runs cannot match:
                the worker faults are drawn per time step, and the steps differ)

Usage (from webots/): python tools/validate_lite.py [--seeds 5]
Writes logs/lite_validation.md.
"""
import argparse
import json
import os
import re
import statistics
import sys
import tempfile
from collections import defaultdict
from multiprocessing import Pool
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "lib"))

NAME = re.compile(r"_wms-(?P<wes>[a-z]+)(?:-(?P<policy>[a-z]+))?_seed(?P<seed>\d+)_[a-z]+_(?P<run>.+)_batch$")
METRICS = ("busy", "lead", "shipped", "dist", "short_pick", "dest_occ", "mismatch", "verify")
FAULTY = ("short_pick", "dest_occ", "mismatch", "verify")          # depend on when faults happen
_layouts = {}


def metrics(s, cfg):
    roles = {str(r["id"]): r["role"] for r in cfg["robots"]}       # older runs have no robot_roles
    busy = [b for r, b in s["robot_busy_share"].items() if roles[str(r)] == "transport"]
    ev = s["events"]
    return {"busy": statistics.mean(busy), "lead": s["mean_order_lead_s"], "shipped": s["units_shipped"],
            "dist": sum(s["robot_distance_m"].values()),
            "short_pick": ev.get("SHORT_PICK", 0) + ev.get("ITEM_MISMATCH", 0),
            "dest_occ": ev.get("DESTINATION_OCCUPIED", 0),
            "mismatch": (s.get("record_quality") or {}).get("settled_mismatch_mean"),
            "verify": (s.get("wes_stats") or {}).get("tasks", {}).get("VERIFY_COMPLETED", 0)}


def scheduler(wes, policy, cfg):
    os.environ["VERIFY_POLICY"] = policy or "event"
    if wes == "standin":
        import sched_standin
        return sched_standin.Scheduler(cfg)
    if wes == "verify":
        import sched_verify
        return sched_verify.Scheduler(cfg)
    import sched_learned
    w = [sched_learned.HEURISTIC.get(f, 0.0) for f in sched_learned.FEATURES] if wes == "heuristic" else None
    return sched_learned.Scheduler(cfg, weights=w)


def lite(job):
    """job = (run dir, wes, policy, fault seed) -> metrics."""
    from lite_sim import LiteSim
    from nav import GridMap
    from wes_runner import WES
    run_dir, wes, policy, seed = job
    base = json.loads((Path(run_dir) / "scenario.json").read_text(encoding="utf-8"))
    cfg = {**base, "seed": seed}
    key = json.dumps([base["world"], base["racks"], base.get("obstacles")])
    if key not in _layouts:
        _layouts[key] = (GridMap(base), {})
    grid, fields = _layouts[key]
    with tempfile.TemporaryDirectory() as tmp:
        w = WES(cfg, scheduler=scheduler(wes, policy, cfg), log_dir=tmp)
        return metrics(LiteSim(cfg, w, grid=grid, fields=fields).run(), cfg)


def mean(xs):
    xs = [x for x in xs if x is not None]
    return statistics.mean(xs) if xs else None


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--logs", type=Path, default=ROOT / "logs")
    ap.add_argument("--seeds", type=int, default=5, help="fault seeds per run in the lightweight simulator")
    ap.add_argument("--workers", type=int, default=max(1, (os.cpu_count() or 2) - 1))
    args = ap.parse_args()
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")

    runs = []
    for d in sorted(args.logs.glob("*_batch")):
        m = NAME.search(d.name)
        if m and (d / "summary.json").exists() and (d / "scenario.json").exists():
            s = json.loads((d / "summary.json").read_text(encoding="utf-8"))
            cfg = json.loads((d / "scenario.json").read_text(encoding="utf-8"))
            mix = f"{sum(r['role'] == 'transport' for r in cfg['robots'])}+{sum(r['role'] == 'patrol' for r in cfg['robots'])}"
            runs.append({"dir": str(d), "wes": m["wes"], "policy": m["policy"], "seed": int(m["seed"]),
                         "run": m["run"], "mix": mix, "speed": cfg["nav"]["speed_mps"],
                         "faults": cfg["faults"]["worker_move_per_h"], "duration": cfg["episode"]["duration_s"],
                         "webots": metrics(s, cfg)})
    print(f"{len(runs)} Webots runs, {args.seeds} lightweight seeds each")
    jobs = [(r["dir"], r["wes"], r["policy"], r["seed"] + 1000 * k) for r in runs for k in range(args.seeds)]
    with Pool(args.workers) as pool:
        res = pool.map(lite, jobs)
    for i, r in enumerate(runs):
        rs = res[i * args.seeds:(i + 1) * args.seeds]
        r["lite"] = {k: mean(x[k] for x in rs) for k in METRICS}

    out = ["# 경량 시뮬레이터 검증", "", f"Webots 실행 {len(runs)}개, 실행마다 경량 시뮬 seed {args.seeds}개 평균.", ""]

    # 1. accuracy
    out += ["## 1. 실행별 정확도 (오류와 무관한 지표)", "",
            "| 지표 | 평균 절대 오차 | 평균 상대 오차 | 최대 상대 오차 | 경량이 더 큼(평균 편향) |", "|---|---|---|---|---|"]
    for k in ("busy", "lead", "shipped", "dist"):
        pairs = [(r["webots"][k], r["lite"][k]) for r in runs if r["webots"][k] is not None and r["lite"][k] is not None]
        ae = [abs(lt - wb) for wb, lt in pairs]
        re_ = [abs(lt - wb) / abs(wb) for wb, lt in pairs if wb]
        bias = statistics.mean((lt - wb) / abs(wb) for wb, lt in pairs if wb)
        out.append(f"| {k} | {statistics.mean(ae):.3g} | {statistics.mean(re_):.1%} | {max(re_):.1%} | {bias:+.1%} |")
    out += ["", "| 실행 | 구성 | 속도 | 운반 가동률 W / L | 처리 시간 W / L | 출하 W / L | 이동 거리 W / L |", "|---|---|---|---|---|---|---|"]
    for r in runs:
        w, lt = r["webots"], r["lite"]
        f = lambda k, p: f"{w[k]:{p}} / {lt[k]:{p}}" if w[k] is not None and lt[k] is not None else "-"  # noqa: E731
        out.append(f"| {r['wes']}-{r['policy'] or ''} s{r['seed']} {r['run']} | {r['mix']} | {r['speed']} | "
                   f"{f('busy', '.3f')} | {f('lead', '.0f')} | {f('shipped', '.0f')} | {f('dist', '.0f')} |")

    # 2. ranking: pairs of runs that differ only in WES / policy / mix
    comps = []
    rl = defaultdict(dict)
    for r in runs:
        if r["run"] in ("rl4", "rl8"):
            rl[r["run"]][r["wes"]] = r
    for run, d in rl.items():
        for a in ("heuristic", "learned"):
            if a in d and "verify" in d:
                comps.append((f"{run}: {a} - stand-in", d[a], d["verify"]))
    pt = {(r["mix"], r["policy"], r["faults"], r["seed"]): r for r in runs if r["run"].startswith("pt_") and r["duration"] == 3600}
    for (mix, pol, fl, seed), r in pt.items():
        if mix == "7+1" and ("8+0", pol, fl, seed) in pt:
            comps.append((f"patrol 7+1 - 8+0, {pol}, faults {fl:g}/h, s{seed}", r, pt[("8+0", pol, fl, seed)]))
        if pol in ("event", "periodic") and (mix, "none", fl, seed) in pt:
            comps.append((f"policy {pol} - none, {mix}, faults {fl:g}/h, s{seed}", r, pt[(mix, "none", fl, seed)]))
    out += ["", "## 2. 순위 일치 (짝지은 차이의 방향)", "",
            "같은 조건에서 WES·정책·구성만 다른 두 실행의 차이(A - B). 차이가 작으면(|W| < 기준) 방향 비교에서 뺐어요.", "",
            "| 지표 | 비교 수 | 방향 일치 | 기준 |", "|---|---|---|---|"]
    tol = {"lead": 15, "shipped": 1.5, "busy": 0.01, "mismatch": 0.3, "short_pick": 1.5, "dest_occ": 2}
    for k, t in tol.items():
        agree = n = 0
        for _, a, b in comps:
            dw = (a["webots"][k] or 0) - (b["webots"][k] or 0)
            dl = (a["lite"][k] or 0) - (b["lite"][k] or 0)
            if abs(dw) >= t:
                n += 1
                agree += (dw > 0) == (dl > 0)
        out.append(f"| {k} | {n} | {agree}/{n} ({agree / n:.0%}) | {t} |" if n else f"| {k} | 0 | - | {t} |")
    # Same comparisons averaged over the seeds of a condition: a single Webots run is noisy.
    cond = defaultdict(list)
    for name, a, b in comps:
        cond[re.sub(r", s\d+$", "", name)].append((a, b))
    out += ["", "조건 단위 (seed 평균 차이):", "", "| 지표 | 비교 수 | 방향 일치 | 기준 |", "|---|---|---|---|"]
    for k, t in tol.items():
        agree = n = 0
        for pairs in cond.values():
            dw = mean((a["webots"][k] or 0) - (b["webots"][k] or 0) for a, b in pairs)
            dl = mean((a["lite"][k] or 0) - (b["lite"][k] or 0) for a, b in pairs)
            if abs(dw) >= t:
                n += 1
                agree += (dw > 0) == (dl > 0)
        out.append(f"| {k} | {n} | {agree}/{n} ({agree / n:.0%}) | {t} |" if n else f"| {k} | 0 | - | {t} |")
    out += ["", "| 조건 | n | 처리 시간 차 W / L | 출하 차 W / L | 가동률 차 W / L | 틀린 기록 차 W / L |",
            "|---|---|---|---|---|---|"]
    for name, pairs in sorted(cond.items()):
        d = lambda k, p: (f"{mean((a['webots'][k] or 0) - (b['webots'][k] or 0) for a, b in pairs):+{p}} / "  # noqa: E731
                          f"{mean((a['lite'][k] or 0) - (b['lite'][k] or 0) for a, b in pairs):+{p}}")
        out.append(f"| {name} | {len(pairs)} | {d('lead', '.0f')} | {d('shipped', '.1f')} | {d('busy', '.3f')} | "
                   f"{d('mismatch', '.2f')} |")
    out += ["", "실행 단위:", "", "| 비교 | 처리 시간 차 W / L | 출하 차 W / L | 가동률 차 W / L | 틀린 기록 차 W / L |",
            "|---|---|---|---|---|"]
    for name, a, b in sorted(comps, key=lambda c: c[0]):
        d = lambda k, p: f"{(a['webots'][k] or 0) - (b['webots'][k] or 0):+{p}} / {(a['lite'][k] or 0) - (b['lite'][k] or 0):+{p}}"  # noqa: E731
        out.append(f"| {name} | {d('lead', '.0f')} | {d('shipped', '.0f')} | {d('busy', '.3f')} | {d('mismatch', '.2f')} |")

    # 3. fault-driven metrics per condition
    groups = defaultdict(list)
    for r in runs:
        if r["run"].startswith("pt_") and r["duration"] == 3600:
            groups[(r["mix"], r["policy"], r["faults"])].append(r)
    out += ["", "## 3. 오류 지표 (조건별 평균, Webots seed 3개 vs 경량 seed 3×N개)", "",
            "| 구성 | 정책 | 작업자 이동/h | SHORT_PICK W / L | DEST_OCC W / L | 틀린 기록 W / L | VERIFY W / L |",
            "|---|---|---|---|---|---|---|"]
    for (mix, pol, fl), g in sorted(groups.items()):
        c = lambda k: f"{mean(r['webots'][k] for r in g):.2f} / {mean(r['lite'][k] for r in g):.2f}"  # noqa: E731
        out.append(f"| {mix} | {pol} | {fl:g} | {c('short_pick')} | {c('dest_occ')} | {c('mismatch')} | {c('verify')} |")

    text = "\n".join(out) + "\n"
    (args.logs / "lite_validation.md").write_text(text, encoding="utf-8")
    print(text)


if __name__ == "__main__":
    main()
