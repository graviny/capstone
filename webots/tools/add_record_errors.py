"""Add record errors at t=0 to a scenario: the WMS record starts out wrong, the physical stock does not change.

    python tools/add_record_errors.py --scenario logs/iri_v1/base/x.json --out logs/iri_v1/x_l0.03_mixed.json \
        --level 0.03 --mix mixed --err-seed 1048

Level: k = round(level x number of stocked rack slots in initial_stock) slots get a wrong record
(the base layout stocks 96 slots, so 1% is about 1 slot). Types (record vs reality):
  ghost   record SKU A, slot empty         -> the robot finds SHORT_PICK
  hidden  record empty, a carton is there  -> DESTINATION_OCCUPIED
  swap    record SKU A, carton is SKU B    -> ITEM_MISMATCH
mix: ghost, hidden, swap, or mixed (ghost:hidden 1:1; ghost gets the extra slot when k is odd).
A slot is used at most once. Candidates are sorted before random.Random(err_seed) draws from them,
so the same input gives the same output. initial_stock (the physical side) is never changed;
the WMS applies "initial_record_errors" when it loads the scenario (lib/wms.py).
Reference range for levels: WERC (2019) inventory count accuracy classes (Destro et al. 2023,
appendix C). This is a sensitivity analysis, not a reproduction of real error rates.
"""
import argparse
import json
import math
import random
from pathlib import Path

DEFINITION = "k = round(level * n_stocked); n_stocked = rack slots in initial_stock"
MIXES = ("ghost", "hidden", "swap", "mixed")


def add_record_errors(cfg, level, mix, err_seed):
    """Return a copy of cfg with initial_record_errors and initial_record_errors_meta added."""
    if mix not in MIXES:
        raise ValueError(f"mix: {mix}")
    if not 0 <= level <= 1:
        raise ValueError("level must be in [0, 1]")
    slots = {s["id"]: s for s in cfg["slots"]}
    sku_at = {st["slot"]: st["sku"] for st in cfg["initial_stock"] if slots[st["slot"]]["kind"] == "rack"}
    stocked = sorted(sku_at)
    empty = sorted(sid for sid, s in slots.items() if s["kind"] == "rack" and sid not in sku_at)
    k = round(level * len(stocked))
    n = {"ghost": k if mix == "ghost" else (k - k // 2 if mix == "mixed" else 0),
         "hidden": k if mix == "hidden" else (k // 2 if mix == "mixed" else 0),
         "swap": k if mix == "swap" else 0}
    if n["ghost"] > len(empty) or n["hidden"] + n["swap"] > len(stocked):
        raise ValueError(f"level {level} needs more slots than the scenario has")
    rng, used, errors = random.Random(err_seed), set(), []

    def dist(a, b):
        return math.dist(slots[a]["pos"], slots[b]["pos"])

    def nearest(sid, pool):
        """Closest slot of pool, same face first; None if pool is empty."""
        return min(pool, key=lambda x: (slots[x]["face"] != slots[sid]["face"], dist(sid, x), x), default=None)

    for _ in range(n["ghost"]):
        # An empty slot next to a slot that holds the SKU (a carton recorded one slot off).
        free = [s for s in empty if s not in used]
        anchor = rng.choice(stocked)
        slot = nearest(anchor, [s for s in free if slots[s]["face"] == slots[anchor]["face"]]) or rng.choice(free)
        used.add(slot)
        errors.append({"slot": slot, "type": "ghost", "record_sku": sku_at[anchor]})
    for kind in ("hidden", "swap"):
        for _ in range(n[kind]):
            slot = rng.choice([s for s in stocked if s not in used])
            used.add(slot)
            if kind == "hidden":
                errors.append({"slot": slot, "type": "hidden", "record_sku": None})
                continue
            others = [s for s in stocked if sku_at[s] != sku_at[slot]]
            same_face = [s for s in others if slots[s]["face"] == slots[slot]["face"]]
            src = nearest(slot, same_face)
            sku = sku_at[src] if src else rng.choice(sorted({c["sku"] for c in cfg["skus"]} - {sku_at[slot]}))
            errors.append({"slot": slot, "type": "swap", "record_sku": sku})
    out = dict(cfg)
    out["initial_record_errors"] = errors
    out["initial_record_errors_meta"] = {"level": level, "mix": mix, "err_seed": err_seed, "k": k,
                                         "n_stocked": len(stocked), "definition": DEFINITION}
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scenario", type=Path, required=True)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--level", type=float, required=True)
    ap.add_argument("--mix", choices=MIXES, required=True)
    ap.add_argument("--err-seed", type=int, required=True)
    a = ap.parse_args()
    cfg = add_record_errors(json.loads(a.scenario.read_text(encoding="utf-8")), a.level, a.mix, a.err_seed)
    a.out.parent.mkdir(parents=True, exist_ok=True)
    a.out.write_text(json.dumps(cfg, indent=1), encoding="utf-8")
    m = cfg["initial_record_errors_meta"]
    print(f"{a.out}: {m['k']} of {m['n_stocked']} stocked slots wrong ({a.mix}, level {a.level:g})")


if __name__ == "__main__":
    main()
