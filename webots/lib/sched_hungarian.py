"""Hungarian baseline: each step, idle robots and startable tasks are paired so that the total
robot-to-task distance is least (Kuhn 1955, scipy linear_sum_assignment). No task priority, no
busy robots. VERIFY policy, exceptions and the candidate window (oldest 30 startable tasks) are
sched_learned's, so only the pairing differs.

    WES_MODULE=wes_runner WES_SCHEDULER=sched_hungarian VERIFY_POLICY=event
"""
from scipy.optimize import linear_sum_assignment

import sched_learned


class Hungarian(sched_learned.Assign):
    def plan(self, s):
        out, taken = self.reroute_waiting(s)
        idle = s.idle_robots()
        tasks = [(t, ep) for t in self.candidate_tasks(s, taken) for ep in [self.endpoints(s, t, taken)] if ep]
        pairs = []
        for role in {r["role"] for r in idle}:
            rs = [r for r in idle if r["role"] == role]
            ts = [(t, ep) for t, ep in tasks
                  if ("patrol" if t["task_type"] == "VERIFY" and self.has_patrol else "transport") == role]
            if not rs or not ts:
                continue
            cost = [[s.dist(r["pos"], s.pos(ep[0] or ep[1])) for _, ep in ts] for r in rs]
            for i, j in zip(*linear_sum_assignment(cost)):
                pairs.append((cost[i][j], rs[i]["id"], ts[j][0]))
        for _, rid, task in sorted(pairs, key=lambda p: p[0]):
            ep = self.endpoints(s, task, taken)        # two picks may want the same lane slot or source
            if ep is None:
                continue
            taken.update(x for x in ep if x)
            out.append({"robot_id": rid, "task_id": task["task_id"], "action": "EXECUTE",
                        "source": ep[0], "destination": ep[1]})
        return out


class Scheduler(sched_learned.Scheduler, Hungarian):
    """MRO: Scheduler -> sched_learned.Scheduler -> sched_verify -> Hungarian -> Assign."""

    def __init__(self, cfg):
        super().__init__(cfg, weights=[0.0] * len(sched_learned.FEATURES))
        self.name = f"hungarian-{self.policy}"
