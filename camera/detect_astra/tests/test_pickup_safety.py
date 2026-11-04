"""The full filled footprint and padding are retained at arbitrary headings."""
import math
from pathlib import Path
import sys
import unittest
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from pallet_pickup_safety import footprint_hit, lock_saved_station_plan, lookup_station_transform
from pallet_insertion_core import Opening, make_plan, terminal_geometry
from pallet_pickup_core import PickupController
from dataclasses import replace


class PickupFootprintTest(unittest.TestCase):
    def test_future_dated_amcl_is_queried_at_current_odometry_time(self):
        from tf2_ros import Buffer
        from geometry_msgs.msg import TransformStamped
        from rclpy.time import Time
        buf=Buffer()
        for stamp in (99.5,100.5):
            tf=TransformStamped();tf.header.frame_id='map';tf.child_frame_id='odom'
            tf.header.stamp=Time(seconds=stamp).to_msg();tf.transform.rotation.w=1.
            tf.transform.translation.x=2.
            buf.set_transform(tf,'amcl_test')
        result=lookup_station_transform(buf,99.95,100.)
        self.assertAlmostEqual(result.header.stamp.sec+result.header.stamp.nanosec*1e-9,99.95)
        self.assertAlmostEqual(result.transform.translation.x,-2.)

    def test_station_transform_still_rejects_old_odometry_or_missing_current_tf(self):
        from tf2_ros import Buffer, TransformException
        from geometry_msgs.msg import TransformStamped
        from rclpy.time import Time
        buf=Buffer();tf=TransformStamped();tf.header.frame_id='map';tf.child_frame_id='odom'
        tf.header.stamp=Time(seconds=99.5).to_msg();tf.transform.rotation.w=1.
        buf.set_transform(tf,'amcl_test')
        with self.assertRaises(ValueError):lookup_station_transform(buf,99.,100.)
        with self.assertRaises(TransformException):lookup_station_transform(buf,99.95,100.)

    def station_plan(self):
        return make_plan((0.,0.,0.),(1.25,0.,0.),Opening(1.6555,0.,0.,.245),0.)

    def test_taught_station_lock_rejects_silent_endpoint_or_heading_correction(self):
        plan=self.station_plan()
        self.assertTrue(lock_saved_station_plan(plan).station_locked)
        self.assertFalse(plan.station_locked)
        for nominal in ((1.26,0.,0.),(1.25,.006,0.),(1.25,0.,math.radians(1.1))):
            with self.subTest(nominal=nominal),self.assertRaises(ValueError):
                lock_saved_station_plan(replace(plan,nominal_goal=nominal))

    def test_lock_checks_wrapped_heading_and_nonfinite_values(self):
        plan=self.station_plan()
        self.assertTrue(lock_saved_station_plan(replace(plan,
            nominal_goal=(1.25,0.,2*math.pi))).station_locked)
        with self.assertRaises(ValueError):
            lock_saved_station_plan(replace(plan,nominal_goal=(float('nan'),0.,0.)))

    def test_no_lift_qualification_outside_taught_station_even_if_opening_fits(self):
        plan=self.station_plan();locked=lock_saved_station_plan(plan)
        self.assertTrue(terminal_geometry((1.25,0.,0.),0.,0.,locked))
        for pose in ((1.25,.011,0.),(1.25,0.,math.radians(1.1))):
            self.assertTrue(terminal_geometry(pose,0.,0.,plan))
            self.assertFalse(terminal_geometry(pose,0.,0.,locked))

    def test_controller_rechecks_taught_station_lock_before_becoming_active(self):
        core=PickupController()
        with self.assertRaises(ValueError):
            core.start(0.,(0.,0.,0.),Opening(1.6555,0.,0.,.245),
                       (1.27,0.,0.),0.,True,station_locked=True)
        self.assertFalse(core.active)
        self.assertEqual(core.state,'IDLE')

    def test_body_fork_and_padding_are_checked_after_rotation(self):
        for yaw in (0.,math.pi/2,-.7):
            c,s=math.cos(yaw),math.sin(yaw)
            def world(p):return np.array([[3+c*p[0]-s*p[1],2+s*p[0]+c*p[1]]])
            for point in ((0.,0.),(.7,.05),(.79,.10),(-.34,0.)):
                self.assertIsNotNone(footprint_hit(world(point),(3.,2.,yaw)))
            for point in ((5.,5.),(.80,.13),(-.4,0.)):
                self.assertIsNone(footprint_hit(world(point),(3.,2.,yaw)))

    def test_empty_filtered_set_does_not_waive_nonfinite_input(self):
        self.assertIsNone(footprint_hit(np.empty((0,2)),(0.,0.,0.)))
        with self.assertRaises(ValueError):
            footprint_hit(np.array([[float('nan'),5.]]),(0.,0.,0.))


if __name__=='__main__':unittest.main()
