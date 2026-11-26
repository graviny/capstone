import unittest
import numpy as np
from pi_car.common.grid import OccupancyGrid,GridMetadata
from pi_car.vehicle.planning.line_of_sight import segment_is_free


class SegmentTests(unittest.TestCase):
    def grid(self):
        return OccupancyGrid(np.zeros((6,6),np.int8),GridMetadata(6,6,1.,(0.,0.),'test','v1','test'))

    def test_clear_segment_and_reverse(self):
        grid=self.grid()
        self.assertTrue(segment_is_free(grid,(.5,.5),(4.5,3.5)))
        self.assertTrue(segment_is_free(grid,(4.5,3.5),(.5,.5)))

    def test_occupied_unknown_and_outside_reject(self):
        grid=self.grid()
        for value in (1,-1):
            grid.cells[2,2]=value
            self.assertFalse(segment_is_free(grid,(.5,.5),(4.5,4.5)))
        self.assertFalse(segment_is_free(grid,(-.1,.5),(4.5,4.5)))

    def test_corner_touch_cannot_cut_between_obstacles(self):
        grid=self.grid();grid.cells[0,1]=1
        self.assertFalse(segment_is_free(grid,(.5,.5),(2.5,2.5)))

    def test_boundary_parallel_segment_checks_both_sides(self):
        grid=self.grid();grid.cells[2,1]=-1
        self.assertFalse(segment_is_free(grid,(2.,.5),(2.,4.5)))

    def test_blocked_cell_in_bounding_box_but_off_segment_is_allowed(self):
        grid=self.grid();grid.cells[3,0]=1
        self.assertTrue(segment_is_free(grid,(.5,.5),(4.5,4.5)))


if __name__=='__main__':unittest.main()
