"""Quarter-turn correction must preserve standoff and stop before recognition."""
import math
import unittest
from dataclasses import replace
from test_visual_pickup import config,lift
from pallet_checkpoint import quarter_turn_plan,quarter_turn_pose_issue,CheckpointApproach
from pallet_insertion_core import Opening,opening_coordinates
from pallet_visual_pickup import VisualPickupController

class QuarterTurnTest(unittest.TestCase):
    def test_both_sides_and_rotated_world(self):
        for angle in (0.,1.4,-2.9):
            for side in (-1,1):
                c=config();o=Opening(1.8*math.cos(angle)-side*.06*math.sin(angle),
                    1.8*math.sin(angle)+side*.06*math.cos(angle),angle,.255)
                start=(0.,0.,angle+.03);p=quarter_turn_plan(start,o,0.,c)
                self.assertAlmostEqual(p.travel_m,.06)
                self.assertAlmostEqual(opening_coordinates(o,p.goal)[1],0.)
                self.assertAlmostEqual(opening_coordinates(o,start)[0],opening_coordinates(o,p.goal)[0])
                self.assertTrue(all(not quarter_turn_pose_issue(x,o,0.,c) for x in p.poses))
                a=CheckpointApproach(p,c,0.,quarter_turn=True);pose=list(start);v=w=0.
                phases=set()
                for i in range(2200):
                    t=i*.05;a.tick(t,pose,v,w,0.);phases.add(a.phase)
                    self.assertFalse(a.issue,a.issue)
                    v,w=a.v,a.w
                    pose[0]+=.05*v*math.cos(pose[2]);pose[1]+=.05*v*math.sin(pose[2]);pose[2]+=.05*w
                    if a.done:break
                self.assertTrue(a.done)
                self.assertIn('DRIVE',phases)
                self.assertLess(math.hypot(pose[0]-p.goal[0],pose[1]-p.goal[1]),.012)
                self.assertLess(abs(math.atan2(math.sin(pose[2]-angle),math.cos(pose[2]-angle))),math.radians(.3))

    def test_near_pallet_or_excessive_shift_rejected(self):
        for o in (Opening(1.,.06,0.,.255),Opening(1.8,.20,0.,.255)):
            with self.assertRaises(ValueError):quarter_turn_plan((0.,0.,0.),o,0.,config())

    def test_loss_of_operator_stops_and_never_lifts(self):
        c=VisualPickupController(replace(config(),quarter_turn_lateral=True))
        o=Opening(1.8,.06,0.,.255);p=c.preview((0.,0.,0.),o,0.)
        c.start(0.,p.start,o,p.goal,0.,False)
        c.tick(.1,p.start,0.,0.,lift(),operator_alive=False,ros_now=.1)
        self.assertFalse(c.active);self.assertEqual((c.v,c.w),(0.,0.));self.assertIsNone(c.lift_target)
