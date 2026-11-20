"""Nominal route/service time from public geometry and hardware settings only.

No simulator future timers or truth. Zero congestion/turning correction until
independently calibrated; therefore this is NOT a validated completion predictor.
"""
import hashlib
import heapq
import json
import math
from nav import GridMap, NEIGHBORS

_CACHE = {}


def distance_field(grid, goal):
    distance, heap = {goal: 0.}, [(0., goal)]
    while heap:
        d, c = heapq.heappop(heap)
        if d > distance[c]:
            continue
        for di, dj, weight in NEIGHBORS:
            n = c[0] + di, c[1] + dj
            if not grid.free(n, set()):
                continue
            if di and dj and not (grid.free((c[0]+di, c[1]), set()) and grid.free((c[0], c[1]+dj), set())):
                continue
            if d + weight < distance.get(n, math.inf):
                distance[n] = d + weight
                heapq.heappush(heap, (d + weight, n))
    return distance


class TravelTime:
    def __init__(self, cfg):
        key = hashlib.sha256(json.dumps([cfg['world'], cfg['racks'], cfg.get('obstacles', [])], sort_keys=True).encode()).hexdigest()
        if key not in _CACHE:
            _CACHE[key] = GridMap(cfg), {}
        self.grid, self.fields = _CACHE[key]
        self.slots = {s['id']: s for s in cfg['slots']}
        self.speed = cfg['nav']['speed_mps']
        self.look = cfg['perception']['look_s']
        self.act = cfg['nav']['act_s']

    def point(self, sid):
        return self.slots[sid]['approach']

    def distance(self, a, b):
        grid = self.grid
        start = grid.nearest_free(grid.to_cell(a), set())
        goal = grid.nearest_free(grid.to_cell(b), set())
        if start is None or goal is None:
            return math.inf
        if goal not in self.fields:
            self.fields[goal] = distance_field(grid, goal)
        return self.fields[goal].get(start, math.inf) * grid.res

    def completion(self, position, task, endpoints):
        src, dst = endpoints
        if task['task_type'] == 'VERIFY':
            return self.distance(position, self.point(dst)) / self.speed + self.look
        return ((self.distance(position, self.point(src)) + self.distance(self.point(src), self.point(dst)))
                / self.speed + 2 * (self.look + self.act))
