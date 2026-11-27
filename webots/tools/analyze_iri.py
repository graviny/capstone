"""Analyse the IRI sweep (docs/experiments_iri.md section 5): logs/iri_v1/results.csv -> report.md, iri_lead.png.

    python tools/analyze_iri.py [--dir logs/iri_v1]
Unit = base scenario (date x seed). Paired differences within a scenario; bootstrap 95% CI over
scenarios (10,000 resamples, RNG seed 20261128). Level 0 is shared by every mix.
"""
import argparse
import csv
import json
from collections import defaultdict
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
POLICIES = ["team_wes", "hungarian", "cem_2day", "cem_18day"]
LABEL = {"team_wes": "Team WES", "hungarian": "Hungarian", "cem_2day": "CEM (2 days)", "cem_18day": "CEM (18 days)"}
MIXES = ["ghost", "hidden", "mixed"]
BOOT, BOOT_SEED = 10_000, 20261128
Y = "order_lead_all_s"


def ci(d):
    """Mean and bootstrap 95% CI of the mean of d (resampling scenarios)."""
    d = np.asarray(d, float)
    rng = np.random.default_rng(BOOT_SEED)
    m = d[rng.integers(0, len(d), (BOOT, len(d)))].mean(1)
    return d.mean(), *np.percentile(m, [2.5, 97.5])


def fmt(t):
    return f"{t[0]:+.1f} [{t[1]:+.1f}, {t[2]:+.1f}]"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", type=Path, default=ROOT / "logs" / "iri_v1")
    a = ap.parse_args()
    rows = list(csv.DictReader(open(a.dir / "results.csv", encoding="utf-8")))
    manifest = json.loads((a.dir / "manifest.json").read_text(encoding="utf-8"))
    val = {}                                    # (scenario, level, mix, policy) -> row
    for r in rows:
        lv = float(r["level"])
        for mix in (MIXES if lv == 0 else [r["mix"]]):
            val[(f"{r['date']}_s{r['seed']}", lv, mix, r["policy"])] = r
    scen = sorted({k[0] for k in val})
    levels = sorted({k[1] for k in val})
    dates = sorted({s.rsplit("_s", 1)[0] for s in scen})

    def series(lv, mix, pol, ref_pol=None, ref_lv=None, subset=None):
        out = []
        for s in subset or scen:
            a_ = val.get((s, lv, mix, pol))
            b_ = val.get((s, lv if ref_lv is None else ref_lv, mix, ref_pol or pol))
            if a_ and b_:
                out.append(float(a_[Y]) - float(b_[Y]))
        return out

    L = ["# IRI sweep report", "",
         f"Runs {manifest['runs']}, ok {manifest['ok']}, failed {len(manifest['failed'])}; git {manifest['git'][:7]}; "
         f"{len(scen)} base scenarios ({', '.join(dates)} x seeds).",
         f"Main metric: {Y} (s). Differences are paired within a base scenario; [95% bootstrap CI over scenarios, "
         f"{BOOT} resamples, seed {BOOT_SEED}]. No significance win/loss table (pre-registered).", ""]

    L += ["## 1. Mean order lead (s)", "", "| level | mix | " + " | ".join(LABEL[p] for p in POLICIES) + " |",
          "|---|---|" + "---|" * len(POLICIES)]
    for lv in levels:
        for mix in (["(all)"] if lv == 0 else MIXES):
            m = mix if lv else "ghost"
            L.append(f"| {lv:g} | {mix} | " + " | ".join(
                f"{np.mean([float(val[(s, lv, m, p)][Y]) for s in scen if (s, lv, m, p) in val]):.0f}" for p in POLICIES) + " |")

    L += ["", "## 2. Policy - Team WES (s, paired)", "", "| level | mix | " +
          " | ".join(LABEL[p] for p in POLICIES[1:]) + " |", "|---|---|" + "---|" * (len(POLICIES) - 1)]
    for lv in levels:
        for mix in (["ghost"] if lv == 0 else MIXES):
            L.append(f"| {lv:g} | {mix if lv else '(all)'} | " +
                     " | ".join(fmt(ci(series(lv, mix, p, ref_pol="team_wes"))) for p in POLICIES[1:]) + " |")

    L += ["", "## 3. Increase over level 0, same policy (s, paired)", "", "| level | mix | " +
          " | ".join(LABEL[p] for p in POLICIES) + " |", "|---|---|" + "---|" * len(POLICIES)]
    for lv in levels[1:]:
        for mix in MIXES:
            L.append(f"| {lv:g} | {mix} | " + " | ".join(fmt(ci(series(lv, mix, p, ref_lv=0.0))) for p in POLICIES) + " |")

    L += ["", "## 4. Ghost - Hidden at the same level, same policy (s, paired; positive = ghost costs more)", "",
          "| level | " + " | ".join(LABEL[p] for p in POLICIES) + " |", "|---|" + "---|" * len(POLICIES)]
    for lv in levels[1:]:
        cells = []
        for p in POLICIES:
            d = [float(val[(s, lv, "ghost", p)][Y]) - float(val[(s, lv, "hidden", p)][Y])
                 for s in scen if (s, lv, "ghost", p) in val and (s, lv, "hidden", p) in val]
            cells.append(fmt(ci(d)))
        L.append(f"| {lv:g} | " + " | ".join(cells) + " |")

    L += ["", "## 5. Policy - Team WES by date, mix = mixed (s, paired, 10 seeds per date)", ""]
    for dt in dates:
        sub = [s for s in scen if s.startswith(dt + "_")]
        L += [f"**{dt}**", "", "| level | " + " | ".join(LABEL[p] for p in POLICIES[1:]) + " |",
              "|---|" + "---|" * (len(POLICIES) - 1)]
        for lv in levels:
            L.append(f"| {lv:g} | " + " | ".join(fmt(ci(series(lv, "mixed", p, ref_pol="team_wes", subset=sub)))
                                               for p in POLICIES[1:]) + " |")
        L.append("")

    L += ["## 6. Errors seen (Team WES, mean per run)", "",
          "| level | mix | k | SHORT_PICK | DEST_OCC | ghost slots | hidden slots | unknown slots | settled mismatch |",
          "|---|---|---|---|---|---|---|---|---|"]
    for lv in levels:
        for mix in (["ghost"] if lv == 0 else MIXES):
            rs = [val[(s, lv, mix, "team_wes")] for s in scen if (s, lv, mix, "team_wes") in val]
            mean = lambda f: np.mean([float(r[f]) for r in rs])  # noqa: E731
            L.append(f"| {lv:g} | {mix if lv else '(all)'} | {mean('k'):.1f} | {mean('SHORT_PICK'):.1f} | "
                     f"{mean('DESTINATION_OCCUPIED'):.1f} | {mean('ghost_mean'):.2f} | {mean('hidden_mean'):.2f} | "
                     f"{mean('unknown_mean'):.2f} | {mean('settled_mismatch_mean'):.2f} |")
    L += ["", "Slot counts are time means over all rack slots every 10 s, including slots under a running task "
          "(a carton on a robot shows as a ghost until the task completes), so level 0 is not exactly 0.", ""]

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(1, 3, figsize=(12, 3.6), sharey=True)
    for ax, mix in zip(axes, MIXES):
        for p in POLICIES[1:]:
            t = [ci(series(lv, mix, p, ref_pol="team_wes")) for lv in levels]
            ax.errorbar([100 * lv for lv in levels], [x[0] for x in t],
                        yerr=[[x[0] - x[1] for x in t], [x[2] - x[0] for x in t]], marker="o", capsize=3, label=LABEL[p])
        ax.axhline(0, color="grey", lw=0.8)
        ax.set_title(mix)
        ax.set_xlabel("initial record errors (% of stocked slots)")
    axes[0].set_ylabel("order lead - Team WES (s)")
    axes[0].legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(a.dir / "iri_lead.png", dpi=150)
    L.append("![order lead vs Team WES](iri_lead.png)")
    (a.dir / "report.md").write_text("\n".join(L) + "\n", encoding="utf-8")
    print(f"wrote {a.dir / 'report.md'} and iri_lead.png")


if __name__ == "__main__":
    main()
