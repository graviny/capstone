import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "lib"))
from nav import GridMap, in_way_of, step_aside  # noqa: E402

OPEN = {"world": {"x_min": 0, "x_max": 6, "y_min": 0, "y_max": 6}, "racks": []}


class StepAsideTests(unittest.TestCase):
    def test_in_way_of_uses_the_blockers_heading(self):
        blocker = (1.0, 1.0, 0.0)                       # facing +x
        self.assertTrue(in_way_of((1.4, 1.05), blocker, 0.6, 0.8))
        self.assertFalse(in_way_of((0.6, 1.0), blocker, 0.6, 0.8))   # behind it
        self.assertFalse(in_way_of((2.0, 1.0), blocker, 0.6, 0.8))   # too far

    def test_point_is_free_and_out_of_the_way(self):
        grid = GridMap(OPEN)
        me, blocker = (3.0, 3.0), (2.6, 3.0, 0.0)       # blocker behind me, facing me
        q = step_aside(grid, me, [blocker], [blocker])
        self.assertIsNotNone(q)
        self.assertFalse(in_way_of(q, blocker, 1.2, 0.8))
        self.assertGreater(math.hypot(q[0] - blocker[0], q[1] - blocker[1]), 0.5)

    def test_avoids_other_robots_and_walls(self):
        grid = GridMap(OPEN)
        me, blocker = (0.5, 3.0), (0.5, 2.6, math.pi / 2)   # against the west wall, blocker below
        others = [blocker, (0.5, 3.8, 0.0)]                  # and a robot above me
        q = step_aside(grid, me, others, [blocker])
        self.assertIsNotNone(q)
        self.assertTrue(grid.free(grid.to_cell(q), set()))
        self.assertTrue(all(math.hypot(q[0] - o[0], q[1] - o[1]) >= 0.5 for o in others))

    def test_none_when_boxed_in(self):
        grid = GridMap(OPEN)
        me = (3.0, 3.0)
        ring = [(3 + 0.6 * math.cos(a), 3 + 0.6 * math.sin(a), a + math.pi)
                for a in [k * math.pi / 4 for k in range(8)]]
        for r in (0.8, 1.2):                                # far points taken too
            ring += [(3 + r * math.cos(a), 3 + r * math.sin(a), 0.0) for a in [k * math.pi / 8 for k in range(16)]]
        self.assertIsNone(step_aside(grid, me, ring, ring[:1]))


if __name__ == "__main__":
    unittest.main()
