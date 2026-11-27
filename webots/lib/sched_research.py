"""Controlled ablations and transparent comparators, not claimed SOTA policies.

Set RESEARCH_ASSIGNMENT to edf, eta_greedy, or eta_matching. All use the SAME
verification creator, source selection and exception handling. Matching solves a
one-step robot/task matrix; shared slot conflicts are reconciled sequentially,
so it is NOT a globally optimal warehouse schedule.
"""
import math
import os
import sched_learned
import sched_verify
from travel_time import TravelTime


class Assignment(sched_learned.Assign):
    def plan(self, s):
        out, taken = self.reroute_waiting(s)
        robots = {r['id']: r for r in s.idle_robots()}
        pending = s.pending_tasks()
        if self.method == 'edf':
            pending.sort(key=lambda t: (t.get('deadline') if t.get('deadline') is not None else math.inf,
                                        t['priority'], t['created_t'], t['queue_seq']))
        done = set()
        while robots:
            feasible = []
            for task in pending:
                if task['task_id'] in done:
                    continue
                ep = self.endpoints(s, task, taken)
                if ep is None:
                    continue
                role = 'patrol' if task['task_type'] == 'VERIFY' and self.has_patrol else 'transport'
                for rid, robot in robots.items():
                    if robot['role'] != role:
                        continue
                    cost = (s.dist(robot['pos'], s.pos(ep[0] or ep[1])) if self.method == 'edf'
                            else self.travel.completion(robot['pos'], task, ep))
                    if math.isfinite(cost):
                        feasible.append((cost, task, rid, ep))
                if self.method == 'edf' and feasible:
                    break
            if not feasible:
                break
            if self.method == 'eta_matching':
                import numpy as np
                from scipy.optimize import linear_sum_assignment
                rids = sorted(robots)
                tids = sorted({c[1]['task_id'] for c in feasible})
                ri, ti = {v:i for i,v in enumerate(rids)}, {v:i for i,v in enumerate(tids)}
                matrix = np.full((len(rids), len(tids)), 1e12)
                pairs = {}
                for c in feasible:
                    key = ri[c[2]], ti[c[1]['task_id']]
                    matrix[key] = c[0]
                    pairs[key] = c
                rr, tt = linear_sum_assignment(matrix)
                chosen = [pairs[(r,t)] for r,t in zip(rr,tt) if (r,t) in pairs]
                # Apply just one pair and recompute with its reserved slots. This
                # handles SKU/source conflicts without sending invalid decisions.
                best = min(chosen, key=lambda c:(c[0], c[1]['task_id'], c[2])) if chosen else None
            else:
                best = min(feasible, key=lambda c:(c[0], c[1]['task_id'], c[2]))
            if best is None:
                break
            _, task, rid, ep = best
            out.append({'robot_id': rid, 'task_id': task['task_id'], 'action': 'EXECUTE',
                        'source': ep[0], 'destination': ep[1]})
            taken.update(x for x in ep if x)
            done.add(task['task_id'])
            del robots[rid]
        return out


class Scheduler(sched_verify.Scheduler, Assignment):
    def __init__(self, cfg, method=None):
        super().__init__(cfg)
        self.method = method or os.environ.get('RESEARCH_ASSIGNMENT', 'edf')
        if self.method not in ('edf', 'eta_greedy', 'eta_matching'):
            raise ValueError(self.method)
        self.travel = TravelTime(cfg) if self.method != 'edf' else None
        self.name = f'research-{self.method}-{self.policy}'
