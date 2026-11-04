import math
from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from pallet_pickup_recovery import relative_opening,recovery_issue
from pallet_insertion_core import Opening


class RecoveryTest(unittest.TestCase):
    def test_stopped_frame_matches_new_odom_after_restart(self):
        old=Opening(1.64,-.005,-.005,.245)
        rel=relative_opening(old,(.264,-.001,0.))
        measured=Opening(rel['x'],rel['y'],rel['yaw'],rel['width_m'])
        self.assertEqual(recovery_issue(rel,measured,(0.,0.,0.),0.,0.),'')

    def test_fresh_reacquisition_cannot_waive_movement_slot_or_height_error(self):
        expected=dict(x=1.38,y=0.,yaw=0.,width_m=.245)
        for opening,height in ((Opening(1.42,0.,0.,.245),0.),
                               (Opening(1.38,.03,0.,.245),0.),
                               (Opening(1.38,0.,.05,.245),0.),
                               (Opening(1.38,0.,0.,.20),0.),
                               (Opening(1.38,0.,0.,.245),.01)):
            self.assertTrue(recovery_issue(expected,opening,(0.,0.,0.),height,0.))

    def test_no_blind_near_entry_restart_and_no_relaxed_alignment(self):
        for x,y,yaw in ((.8,0.,0.),(1.38,.025,0.),(1.38,0.,math.radians(2.1))):
            expected=dict(x=x,y=y,yaw=yaw,width_m=.245)
            self.assertTrue(recovery_issue(expected,Opening(**expected),(0.,0.,0.),0.,0.))


if __name__=='__main__':unittest.main()
