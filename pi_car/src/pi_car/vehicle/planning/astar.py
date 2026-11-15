import heapq
import math


def plan(grid, start_xy, goal_xy):
    """4-connected A*: unknown blocked, no diagonal corner cutting.

    Caller supplies an inflated navigation grid. Waypoints are cell centers,
    with no smoothing that might cut across obstacles.
    """
    start, goal = grid.world_to_cell(*start_xy), grid.world_to_cell(*goal_xy)
    if not grid.is_free(start) or not grid.is_free(goal):
        raise ValueError("Start/goal is outside free navigation space")
    frontier = [(0, start)]
    previous, cost = {}, {start: 0}
    while frontier:
        _, current = heapq.heappop(frontier)
        if current == goal:
            cells = [goal]
            while cells[-1] != start:
                cells.append(previous[cells[-1]])
            return [grid.cell_to_world(*cell) for cell in reversed(cells)]
        x, y = current
        for nxt in ((x+1, y), (x-1, y), (x, y+1), (x, y-1)):
            new_cost = cost[current] + 1
            if grid.is_free(nxt) and new_cost < cost.get(nxt, math.inf):
                cost[nxt], previous[nxt] = new_cost, current
                heuristic = abs(nxt[0]-goal[0]) + abs(nxt[1]-goal[1])
                heapq.heappush(frontier, (new_cost + heuristic, nxt))
    raise ValueError("No reachable path")
