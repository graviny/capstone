"""Grid map + A* planner shared by the controllers (and later the real robots)."""
import heapq
import math
from collections import deque

SQRT2 = math.sqrt(2)
NEIGHBORS = [(1, 0, 1), (-1, 0, 1), (0, 1, 1), (0, -1, 1),
             (1, 1, SQRT2), (1, -1, SQRT2), (-1, 1, SQRT2), (-1, -1, SQRT2)]


class GridMap:
    def __init__(self, scenario, res=0.1, inflate=0.25):
        w = scenario["world"]
        self.res = res
        self.x0, self.y0 = w["x_min"], w["y_min"]
        self.nx = int(round((w["x_max"] - w["x_min"]) / res))
        self.ny = int(round((w["y_max"] - w["y_min"]) / res))
        self.occ = [[False] * self.ny for _ in range(self.nx)]
        for i in range(self.nx):
            for j in range(self.ny):
                x, y = self.to_world((i, j))
                if (x < w["x_min"] + inflate or x > w["x_max"] - inflate
                        or y < w["y_min"] + inflate or y > w["y_max"] - inflate):
                    self.occ[i][j] = True
                    continue
                for r in scenario["racks"]:
                    if (r["x_min"] - inflate <= x <= r["x_max"] + inflate
                            and abs(y - r["y"]) <= r["half_depth"] + inflate):
                        self.occ[i][j] = True
                        break
                for o in scenario.get("obstacles", []):
                    if (o["x_min"] - inflate <= x <= o["x_max"] + inflate
                            and o["y_min"] - inflate <= y <= o["y_max"] + inflate):
                        self.occ[i][j] = True

    def to_cell(self, p):
        i = int((p[0] - self.x0) / self.res)
        j = int((p[1] - self.y0) / self.res)
        return min(max(i, 0), self.nx - 1), min(max(j, 0), self.ny - 1)

    def to_world(self, c):
        return self.x0 + (c[0] + 0.5) * self.res, self.y0 + (c[1] + 0.5) * self.res

    def _blocked_cells(self, obstacles, radius, keep_free):
        cells = set()
        r = int(math.ceil(radius / self.res))
        for ox, oy in obstacles:
            ci, cj = self.to_cell((ox, oy))
            for di in range(-r, r + 1):
                for dj in range(-r, r + 1):
                    c = (ci + di, cj + dj)
                    wx, wy = self.to_world(c)
                    if math.hypot(wx - ox, wy - oy) <= radius:
                        cells.add(c)
        return cells - keep_free

    def free(self, c, blocked):
        i, j = c
        return 0 <= i < self.nx and 0 <= j < self.ny and not self.occ[i][j] and c not in blocked

    def nearest_free(self, c, blocked):
        if self.free(c, blocked):
            return c
        seen, q = {c}, deque([c])
        while q:
            cur = q.popleft()
            for di, dj, _ in NEIGHBORS:
                n = (cur[0] + di, cur[1] + dj)
                if n in seen or not (0 <= n[0] < self.nx and 0 <= n[1] < self.ny):
                    continue
                if self.free(n, blocked):
                    return n
                seen.add(n)
                q.append(n)
        return None

    def line_free(self, a, b, blocked):
        steps = max(abs(b[0] - a[0]), abs(b[1] - a[1]))
        for k in range(steps + 1):
            t = k / steps if steps else 0.0
            c = (round(a[0] + (b[0] - a[0]) * t), round(a[1] + (b[1] - a[1]) * t))
            if not self.free(c, blocked):
                return False
        return True

    def plan(self, start, goal, obstacles=(), obstacle_radius=0.35):
        """Return a list of world waypoints from start to goal, or None.

        obstacles are other robots' positions, treated as discs for this plan only.
        """
        s0 = self.to_cell(start)
        blocked = self._blocked_cells(obstacles, obstacle_radius, keep_free={s0})
        s = self.nearest_free(s0, blocked)
        g = self.nearest_free(self.to_cell(goal), blocked)
        if s is None or g is None:
            return None

        def h(c):
            dx, dy = abs(c[0] - g[0]), abs(c[1] - g[1])
            return (dx + dy) + (SQRT2 - 2) * min(dx, dy)

        came, cost = {s: None}, {s: 0.0}
        heap = [(h(s), s)]
        while heap:
            _, cur = heapq.heappop(heap)
            if cur == g:
                break
            for di, dj, w in NEIGHBORS:
                n = (cur[0] + di, cur[1] + dj)
                if not self.free(n, blocked):
                    continue
                # No corner cutting past occupied cells.
                if di and dj and not (self.free((cur[0] + di, cur[1]), blocked)
                                      and self.free((cur[0], cur[1] + dj), blocked)):
                    continue
                nc = cost[cur] + w
                if nc < cost.get(n, float("inf")):
                    cost[n], came[n] = nc, cur
                    heapq.heappush(heap, (nc + h(n), n))
        if g not in came:
            return None

        cells = [g]
        while came[cells[-1]] is not None:
            cells.append(came[cells[-1]])
        cells.reverse()

        # Keep only the cells needed for straight-line segments.
        pruned, k = [cells[0]], 0
        while k < len(cells) - 1:
            nxt = len(cells) - 1
            while nxt > k + 1 and not self.line_free(cells[k], cells[nxt], blocked):
                nxt -= 1
            pruned.append(cells[nxt])
            k = nxt
        path = [self.to_world(c) for c in pruned[1:]] or [self.to_world(g)]
        path[-1] = tuple(goal) if self.free(self.to_cell(goal), blocked) else path[-1]
        return path
