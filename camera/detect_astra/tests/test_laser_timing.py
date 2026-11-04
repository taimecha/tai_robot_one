import math
from pathlib import Path
import sys
import time
import unittest
import yaml

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
from pallet_laser_timing import (LASER_TIMEOUT_S, stop_travel,safe_twist_scale,
    validate_stop_polygon,STOP_POLYGON_PADDING_M,STOP_RESERVE_M,measured_velocity_issue,
    COMMAND_TRACKING_FACTOR,COMMAND_ANGULAR_TRACKING_FACTOR,measured_transition_scale)
from pallet_staging_core import StagingConfig,bounded_nav_command
from align_pallet import MotionGuard
from types import SimpleNamespace as NS


class LaserTimingTest(unittest.TestCase):
    def test_actual_stop_polygon_has_required_padding_and_timeout(self):
        config=yaml.safe_load((ROOT/'config/pallet_alignment_collision.yaml').read_text())['pallet_alignment_collision_monitor']['ros__parameters']
        self.assertEqual(config['source_timeout'],LASER_TIMEOUT_S)
        validate_stop_polygon(yaml.safe_load(config['StopZone']['points']))
        with self.assertRaises(ValueError):validate_stop_polygon([[.78,.25],[.78,-.25],[-.33,-.25],[-.33,.25]])

    def test_every_bounded_twist_preserves_curvature_and_fits_delayed_braking(self):
        c=StagingConfig();budget=STOP_POLYGON_PADDING_M-STOP_RESERVE_M
        for v in (0.,.01,.05,.1,.15):
            for w in (-1.,-.24,-.08,0.,.08,.24,1.):
                a,b=bounded_nav_command(v,w,c)
                self.assertLessEqual(stop_travel(a,b),budget+1e-12)
                self.assertLessEqual(stop_travel(a*COMMAND_TRACKING_FACTOR,
                    b*COMMAND_ANGULAR_TRACKING_FACTOR),budget+1e-12)
                if v>0:self.assertAlmostEqual(b/a,w/v)
        self.assertEqual(bounded_nav_command(.1,0.,c),(.1,0.))
        self.assertLess(bounded_nav_command(0.,c.max_angular_rps,c)[1],c.max_angular_rps)
        a,b=bounded_nav_command(.1,.24,c)
        self.assertLess(a,.1);self.assertLess(b,.24)

    def test_real_combined_curve_overshoot_fits_without_disabling_measured_stop(self):
        # Last command before the 17:41 stop; measured v/w overshot it.
        requested=(.0983133,.1007224)
        measured=(.108,.107)
        self.assertTrue(measured_velocity_issue(*measured))
        v,w=bounded_nav_command(*requested,StagingConfig())
        scale=v/requested[0]
        self.assertLess(scale,1.)
        self.assertAlmostEqual(w/requested[1],scale)
        self.assertEqual(measured_velocity_issue(measured[0]*scale,measured[1]*scale),'')
        self.assertLessEqual(stop_travel(v*1.2,w*1.2),.16+1e-12)
        # A real runaway remains blocked even with the new command margin.
        self.assertTrue(measured_velocity_issue(v*2.,w*2.))

    def test_pickup_maximum_twist_fits_same_stop_reserve(self):
        self.assertLessEqual(stop_travel(.1,.08,.15,.4),STOP_POLYGON_PADDING_M-STOP_RESERVE_M)

    def test_real_transition_waits_for_measured_linear_speed_to_fall_before_yaw(self):
        v,w=bounded_nav_command(.0794,.0909,StagingConfig())
        scale=measured_transition_scale(v,w,.1061,.018)
        self.assertLess(scale,1.)
        self.assertLessEqual(stop_travel(max(.1061,v*scale*1.2),max(.018,abs(w)*scale*2.)),.16+1e-12)
        self.assertEqual(measured_transition_scale(.1,.1,.093,.172),0.)
        self.assertTrue(measured_velocity_issue(.093,.172))
        with self.assertRaises(ValueError):measured_transition_scale(.1,0.,float('nan'),0.)

    def test_measured_tracking_overshoot_may_use_reserve_but_not_exceed_polygon(self):
        self.assertGreater(stop_travel(0.,.26),STOP_POLYGON_PADDING_M-STOP_RESERVE_M)
        self.assertEqual(measured_velocity_issue(0.,.26),'')
        self.assertEqual(measured_velocity_issue(.12,0.),'')
        self.assertTrue(measured_velocity_issue(0.,.30))
        self.assertTrue(measured_velocity_issue(.15,0.))
        self.assertIn('w=0.300',measured_velocity_issue(0.,.30))

    def test_guard_accepts_350ms_capture_but_stops_above_400ms_without_restamp(self):
        now=[1.]
        expected={'/pallet/alignment/cmd_vel':'pallet_alignment',
            '/cmd_vel_safe':'pallet_alignment_collision_monitor','/base_controller/cmd_vel':'cmd_vel_stamper'}
        node=NS(now=lambda:now[0],get_publishers_info_by_topic=lambda topic:[NS(node_name=expected[topic],node_namespace='/')])
        wall=time.monotonic()
        g=NS(node=node,scan_stamp=None,scan_wall=None,
             checks={k:(True,wall) for k in ('lifecycle','monitor','stamper')})
        scan=NS(header=NS(stamp=NS(sec=0,nanosec=650000000)),angle_increment=.01,range_min=.05,range_max=10.,ranges=[2.,3.])
        MotionGuard.on_scan(g,scan)
        self.assertAlmostEqual(g.scan_stamp,.65)
        self.assertTrue(MotionGuard.ready(g,1.,wall+.1)[0])
        self.assertFalse(MotionGuard.ready(g,1.051,wall+.1)[0])
        self.assertFalse(MotionGuard.ready(g,1.,wall+.5)[0])
        now[0]=1.1;scan.header.stamp.nanosec=690000000
        MotionGuard.on_scan(g,scan)
        self.assertAlmostEqual(g.scan_stamp,.65)


if __name__=='__main__':unittest.main()
