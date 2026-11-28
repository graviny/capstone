"""How closely does the lightweight simulator (lib/lite_sim.py) reproduce Webots?

    python tools/validate_lite.py "logs/*_d30_batch" [--out logs/validate_lite_d30.md]

Each Webots run folder holds scenario.json and run_env.json (written by the supervisor). The same
scenario and policy run once in the lightweight simulator (it is deterministic: one run per scenario),
and both sides' metrics are computed from their events by the same function (events_metrics).
Reported:
  per run       Webots vs lite for each metric
  error         mean |lite - Webots| / Webots per metric (%)
  ranking       share of policy pairs within a scenario ordered the same way by unit backlog AUC
"""
import argparse
import glob
import hashlib
import importlib
import itertools
import json
import os
import statistics as st
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "lib"))
METRICS = ["busy", "backlog_auc", "order_all", "p90", "unit", "shipped", "SHORT_PICK", "DESTINATION_OCCUPIED",
           "ITEM_MISMATCH"]
POLICY_ENV = ("WES_MODULE", "WES_SCHEDULER", "VERIFY_POLICY", "VERIFY_PERIOD_S",
              "VERIFY_MAX_OPEN", "VERIFY_RECENT_S", "VERIFY_NEIGHBORS", "VERIFY_TICK_RULE",
              "ASSIGN_WEIGHTS", "ASSIGN_COMMIT")


def events_metrics(events, summary):
    """Censored order lead (unfinished orders count to the end), its p90, unit lead, shipped units,
    exception counts, mean transport busy share. Same definition for both simulators."""
    end = summary["duration_s"]
    rel, tot, done, shipped, wait, ulead, counts = {}, {}, {}, {}, {}, [], {}
    for e in events:
        if "type" not in e:                      # decisions share the Webots event file
            continue
        counts[e["type"]] = counts.get(e["type"], 0) + 1
        if e["type"] == "ORDER_RELEASED":
            for ln in e["lines"]:
                rel.setdefault(ln["order"], e["t"])
                tot[ln["order"]] = tot.get(ln["order"], 0) + ln["qty"]
                wait.setdefault(ln["order"], []).extend([e["t"]] * ln["qty"])
        elif e["type"] == "OUTBOUND_DEPARTED":
            for c in e["cartons"]:
                o = c.get("order")
                if o in wait and wait[o]:
                    ulead.append(e["t"] - wait[o].pop(0))
                    shipped[o] = shipped.get(o, 0) + 1
                    if shipped[o] == tot[o]:
                        done[o] = e["t"]
    units = ulead + [end - r for rs in wait.values() for r in rs]
    lead = sorted(done.get(o, end) - r for o, r in rel.items())
    roles, busy = summary["robot_roles"], summary["robot_busy_share"]
    return {"busy": st.mean(b for r, b in busy.items() if roles[r] == "transport"),
            "backlog_auc": sum(units),
            "order_all": st.mean(lead), "p90": lead[int(0.9 * (len(lead) - 1))], "unit": st.mean(units),
            "shipped": len(ulead), "SHORT_PICK": counts.get("SHORT_PICK", 0),
            "DESTINATION_OCCUPIED": counts.get("DESTINATION_OCCUPIED", 0),
            "ITEM_MISMATCH": counts.get("ITEM_MISMATCH", 0)}


def webots(d):
    events = [json.loads(line) for line in open(d / "events.jsonl", encoding="utf-8")]
    return events_metrics(events, json.loads((d / "summary.json").read_text(encoding="utf-8")))


def lite(d):
    from lite_sim import LiteSim
    from wes_runner import WES
    cfg = json.loads((d / "scenario.json").read_text(encoding="utf-8"))
    env = json.loads((d / "run_env.json").read_text(encoding="utf-8"))
    for k in POLICY_ENV:
        v = env.get(k)
        os.environ.pop(k, None) if v is None else os.environ.__setitem__(k, v)
    sched = importlib.import_module(env.get("WES_SCHEDULER") or "sched_standin")
    events = []
    with tempfile.TemporaryDirectory() as tmp:
        s = LiteSim(cfg, WES(cfg, scheduler=sched.Scheduler(cfg), log_dir=tmp), event_sink=events.append).run()
    return events_metrics(events, s)


def policy(d):
    env = json.loads((d / "run_env.json").read_text(encoding="utf-8"))
    w = env.get("ASSIGN_WEIGHTS")
    return Path(w).stem.replace("assign_weights_", "") if w else (env.get("WES_SCHEDULER") or "standin")


def scenario_key(cfg):
    """Only compare policies on identical simulator inputs (run label is not an input)."""
    source = {k: v for k, v in cfg.items() if k not in ("run_name", "generator")}
    blob = json.dumps(source, sort_keys=True, separators=(",", ":")).encode("utf-8")
    return hashlib.sha256(blob).hexdigest()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("runs", nargs="+", help="Webots run folders (globs ok)")
    ap.add_argument("--out", type=Path)
    ap.add_argument("--require-ranking-agreement", action="store_true",
                    help="exit nonzero if there are no policy pairs or any pair reverses or ties in LiteSim")
    a = ap.parse_args()
    dirs = sorted({Path(p) for pat in a.runs for p in glob.glob(pat)
                   if (Path(p) / "summary.json").exists() and (Path(p) / "run_env.json").exists()})
    if not dirs:
        raise SystemExit("no Webots runs with summary.json and run_env.json")
    rows = []
    for d in dirs:
        cfg = json.loads((d / "scenario.json").read_text(encoding="utf-8"))
        meta = cfg.get("initial_record_errors_meta") or {}
        rows.append({"scenario": f"{cfg['outbound']['date']}_s{cfg['seed']}_l{meta.get('level', 0):g}"
                                 f"_slip{cfg.get('record_slip_p', 0):g}",
                     "scenario_key": scenario_key(cfg),
                     "policy": policy(d), "webots": webots(d), "lite": lite(d)})
        print(f"{rows[-1]['scenario']} {rows[-1]['policy']}: backlog AUC Webots "
              f"{rows[-1]['webots']['backlog_auc']:.0f}, lite {rows[-1]['lite']['backlog_auc']:.0f}", flush=True)

    err = {m: st.mean(abs(r["lite"][m] - r["webots"][m]) / r["webots"][m] for r in rows if r["webots"][m])
           for m in METRICS if any(r["webots"][m] for r in rows)}
    pair_rows = []
    by = {}
    for r in rows:
        by.setdefault(r["scenario_key"], []).append(r)
    for rs in by.values():
        for x, y in itertools.combinations(rs, 2):
            dw = x["webots"]["backlog_auc"] - y["webots"]["backlog_auc"]
            dl = x["lite"]["backlog_auc"] - y["lite"]["backlog_auc"]
            if dw:
                pair_rows.append((x, y, dw, dl, dw * dl > 0))
    pairs = len(pair_rows)
    agree = sum(ok for _, _, _, _, ok in pair_rows)
    L = ["# Lightweight simulator vs Webots", "",
         f"{len(rows)} Webots runs, {len(by)} scenarios. Lite: dt 0.5 s, slowdown 1, leg 2 s, yield 1 s; one run per "
         "scenario (deterministic). Metrics from events with the same function on both sides.", "",
         "| metric | mean abs error |", "|---|---|"]
    L += [f"| {m} | {100 * e:.1f}% |" for m, e in err.items()]
    ranking = (f"{agree}/{pairs} pairs ({100 * agree / pairs:.0f}%)" if pairs else "not applicable (no policy pairs)")
    L += ["", f"Policy ranking by unit backlog AUC (primary): {ranking}.", "",
          "| scenario | policy A - policy B | Webots difference (unit-s) | Lite difference (unit-s) | agrees |",
          "|---|---|---:|---:|---|"]
    for x, y, dw, dl, ok in pair_rows:
        L.append(f"| {x['scenario']} | {x['policy']} - {y['policy']} | {dw:+.2f} | {dl:+.2f} | "
                 f"{'yes' if ok else 'NO'} |")
    L += ["",
          "| scenario | policy | " + " | ".join(f"{m} W / L" for m in METRICS) + " |",
          "|---|---|" + "---|" * len(METRICS)]
    for r in rows:
        L.append(f"| {r['scenario']} | {r['policy']} | " + " | ".join(
            f"{r['webots'][m]:.3g} / {r['lite'][m]:.3g}" for m in METRICS) + " |")
    text = "\n".join(L) + "\n"
    print(text)
    if a.out:
        a.out.write_text(text, encoding="utf-8")
    if a.require_ranking_agreement and (pairs == 0 or agree != pairs):
        raise SystemExit(f"policy ranking gate failed: {agree}/{pairs} agreeing pairs")


if __name__ == "__main__":
    main()
