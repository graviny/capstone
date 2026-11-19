import unittest
from unittest.mock import patch
import numpy as np
import matplotlib
matplotlib.use('Agg')
from pi_car.common.geometry import Intrinsics
from pi_car.pc.camera import RGBDFrame
from pi_car.pc.floor import pick_floor_roi


class FloorSelectionTests(unittest.TestCase):
    def test_two_clicks_in_either_order_produce_builtin_integer_roi(self):
        frame=RGBDFrame(np.zeros((80,100,3),np.uint8),np.ones((80,100)),
                       Intrinsics(100,80,100.,100.,50.,40.),0.,'test')
        with patch('matplotlib.pyplot.ginput',return_value=[(np.float64(90.1),70.2),(10.4,9.9)]):
            self.assertEqual(pick_floor_roi(frame),[10,9,91,71])

    def test_cancel_or_degenerate_selection_is_rejected(self):
        frame=RGBDFrame(np.zeros((80,100,3),np.uint8),np.ones((80,100)),
                       Intrinsics(100,80,100.,100.,50.,40.),0.,'test')
        for points in ([],[(10.,10.)],[(10.,10.),(10.,10.)],[(-1.,0.),(10.,10.)]):
            with patch('matplotlib.pyplot.ginput',return_value=points):
                with self.assertRaises(ValueError):pick_floor_roi(frame)


if __name__=='__main__':unittest.main()
