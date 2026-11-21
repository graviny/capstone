"""Robot place failures (Dhanaraj et al., ICRA 2023, extended with retry and takeover).

Shared by the Webots supervisor and the lite sim, so the same attempt has the same outcome in
both engines and under every WES policy: each draw is keyed by (seed, LPN, the carton's place
count, attempt kind). Only rack placements can fail; a failed one puts the carton in a free
neighbor slot of the same face.

scenario.json "faults":
  place_p           success of a robot's first try          (no key: model off)
  retry_q           success of the same robot's retry (2 retries)
  takeover_r        success of another robot's fix (RELOCATE, cause PLACE_FAILED)
  place_fail_alpha  skew of the failure time: skew-normal, location act_s/2, scale act_s/4,
                    clipped to [0.1, act_s]; a success takes act_s
"""
import math
import random

RATE = {"first": "place_p", "retry": "retry_q", "takeover": "takeover_r"}
RETRIES = 2


def draws(seed, lpn, k, kind):
    """The attempt's own random stream. A string seed goes through SHA-512 (deterministic across
    runs and machines); crc32 keys like supervisor.unit() are correlated between sibling keys."""
    return random.Random(f"{seed}-{lpn}-{k}-{kind}")


def enabled(faults):
    return "place_p" in faults


def attempt(seed, lpn, k, kind, faults, act_s):
    """(success, seconds, side) of the k-th placement of carton lpn, try kind first|retry|takeover.
    side (0..1) picks which neighbor a failed placement lands in (landing)."""
    rng = draws(seed, lpn, k, kind)
    ok, side = rng.random() < faults.get(RATE[kind], 1.0), rng.random()
    if ok:
        return True, act_s, side
    a = faults.get("place_fail_alpha", 0.0)
    d = a / math.sqrt(1 + a * a)
    z = d * abs(rng.gauss(0, 1)) + math.sqrt(1 - d * d) * rng.gauss(0, 1)
    return False, min(max(act_s / 2 + act_s / 4 * z, 0.1), act_s), side


def landing(sid, occupant, u):
    """The free neighbor slot (same face, index +-1) a failed placement lands in, or None."""
    face, idx = sid[:3], int(sid[3:])
    for d in ((-1, 1) if u < 0.5 else (1, -1)):
        n = f"{face}{idx + d:02d}"
        if n in occupant and occupant[n] is None:
            return n
    return None
