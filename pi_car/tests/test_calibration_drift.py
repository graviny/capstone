from dataclasses import asdict,replace
from types import SimpleNamespace
import unittest
from pi_car.common.geometry import Intrinsics
from pi_car.pc.calibration import Calibration


class CalibrationDriftTests(unittest.TestCase):
    def test_real_d455_small_focal_update_uses_current_frame_intrinsics(self):
        old=Intrinsics(640,480,385.670166015625,385.2582092285156,314.8418884277344,239.09750366210938)
        new=replace(old,fx=386.8249206542969,fy=386.4117126464844)
        calibration=Calibration([], 'test', 'same-camera', asdict(old),0.)
        frame=SimpleNamespace(serial='same-camera',intrinsics=new)
        calibration.check_frame(frame)
        self.assertEqual(frame.intrinsics.fx,new.fx)  # no replacement with stale K

    def test_changed_device_resolution_large_focal_or_center_still_rejected(self):
        old=Intrinsics(640,480,385.,385.,315.,239.)
        calibration=Calibration([], 'test', 'same',asdict(old),0.)
        for serial,intr in [('different',old),('same',replace(old,width=1280)),
                            ('same',replace(old,fx=400.)),('same',replace(old,cx=317.)),
                            ('same',replace(old,fy=float('nan')))]:
            with self.assertRaises(ValueError):
                calibration.check_frame(SimpleNamespace(serial=serial,intrinsics=intr))


if __name__=='__main__':unittest.main()
