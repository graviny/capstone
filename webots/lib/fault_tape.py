"""Policy-independent exogenous uniforms; environment-only, never a policy feature.

Version 1 uses marked Poisson attempts for S1/S2 (the continuous-time limit of
the old per-step Bernoulli model). A fixed mark selects a quantile of the current
sorted eligible set: attempts/uniforms match across policies, realized targets
need not. S3 remains conditional on a successful pickup, keyed by physical LPN
and pickup occurrence, not by policy-created task IDs or destinations.
"""
import hashlib
import json
import math


def uniform(seed, *key):
    raw = json.dumps([seed, *key], separators=(',', ':'), ensure_ascii=True).encode()
    # 52 bits keeps endpoints strictly inside (0,1) in IEEE doubles.
    return (int.from_bytes(hashlib.sha256(raw).digest()[:8], 'big') >> 12 | 1) / 2**52


def select(mark, candidates):
    ordered = sorted(candidates)
    return ordered[min(int(mark * len(ordered)), len(ordered) - 1)] if ordered else None


class FaultTape:
    version = 'marked-poisson-v1'

    def __init__(self, cfg):
        self.seed = cfg.get('fault_seed', cfg['seed'])
        self.events = []
        horizon = cfg['episode']['duration_s']
        for kind, rate in [('S1', cfg['faults']['worker_move_per_h']),
                           ('S2', cfg['faults']['worker_remove_per_h'])]:
            if rate < 0:
                raise ValueError('fault rate must be nonnegative')
            if not rate:
                continue
            t, i = 0., 0
            while True:
                t += -math.log(uniform(self.seed, kind, i, 'time')) * 3600 / rate
                if t > horizon:
                    break
                self.events.append({'id': f'{kind}-{i}', 'kind': kind, 'scheduled_t': t,
                                    'target_u': uniform(self.seed, kind, i, 'target'),
                                    'destination_u': uniform(self.seed, kind, i, 'destination')})
                i += 1
        self.events.sort(key=lambda e: (e['scheduled_t'], e['id']))
        self.sha256 = hashlib.sha256(json.dumps(self.events, sort_keys=True).encode()).hexdigest()
        self.index = 0
        self.pickups = {}
        self.audit = []

    def due(self, t):
        out = []
        while self.index < len(self.events) and self.events[self.index]['scheduled_t'] <= t:
            out.append(self.events[self.index])
            self.index += 1
        return out

    def record(self, event, t, outcome, **details):
        self.audit.append({**event, 'applied_t': t, 'outcome': outcome, **details})

    def s3(self, lpn, t, probability):
        k = self.pickups.get(lpn, 0)
        self.pickups[lpn] = k + 1
        u = uniform(self.seed, 'S3', lpn, k)
        event = {'id': f'S3-{lpn}-{k}', 'kind': 'S3', 'scheduled_t': t, 'u': u}
        self.record(event, t, 'triggered' if u < probability else 'not_triggered')
        return event if u < probability else None

    def summary(self):
        counts = {}
        for e in self.audit:
            k = e['kind'] + ':' + e['outcome']
            counts[k] = counts.get(k, 0) + 1
        return {'version': self.version, 'fault_seed': self.seed, 'tape_sha256': self.sha256,
                'counts': counts, 'attempts_due': self.index}


def apply_worker_attempt(event, t, slots, occupant, slot_of, max_slots, take, put, remove, tape):
    """Shared mutation semantics for both engines. Returns details for truth logs."""
    lpn = select(event['target_u'], [l for l, s in slot_of.items() if slots[s]['kind'] == 'rack'])
    if lpn is None:
        tape.record(event, t, 'skipped_no_stock')
        return None
    src = slot_of[lpn]
    if event['kind'] == 'S2':
        remove(lpn)
        details = {'lpn': lpn, 'src': src}
    else:
        idx, face = int(src[3:]), src[:3]
        free = [f'{face}{idx + d:02d}' for d in range(-max_slots, max_slots + 1)
                if d and occupant.get(f'{face}{idx + d:02d}', 'outside') is None]
        dst = select(event['destination_u'], free)
        if dst is None:
            tape.record(event, t, 'skipped_no_destination', lpn=lpn, src=src)
            return None
        take(lpn)
        put(lpn, dst)
        details = {'lpn': lpn, 'src': src, 'dst': dst}
    tape.record(event, t, 'injected', **details)
    return details
