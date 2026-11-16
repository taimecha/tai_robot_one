"""AMCL/odom frame, correction and health tests without hardware or motion."""
import math
import json
from pathlib import Path
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from r3_amcl_bridge import (AmclTrialBridge, TrialCorrection, GlobalParticleGate,
                            bounded_correction, map_odom_from_capture,
                            particle_cloud_metrics, rotate_covariance)
from r3_core import compose, wrap


def covariance(xy=.03, yaw_deg=1.):
    result=[0.]*36
    result[0]=result[7]=xy**2
    result[35]=math.radians(yaw_deg)**2
    return result


def initialized(baseline=(3.,-2.,.7)):
    correction=TrialCorrection()
    for stamp in (100.,100.5,101.):
        correction.accept(baseline,covariance(),stamp,stamp)
    return correction


class AmclBridgeTest(unittest.TestCase):
    def assertPoseClose(self, a, b):
        self.assertAlmostEqual(a[0],b[0])
        self.assertAlmostEqual(a[1],b[1])
        self.assertAlmostEqual(wrap(a[2]-b[2]),0.)

    def test_capture_transform_removes_same_stamp_robot_motion(self):
        map_odom=(4.,-3.,.8)
        # Robot motion can be large; only the map->odom correction is used.
        for odom_base in ((0.,0.,0.),(.8,.2,-.4),(2.,-1.,2.1)):
            map_base=compose(map_odom,odom_base)
            self.assertPoseClose(map_odom_from_capture(map_base,odom_base),map_odom)
        # Using a newer odom pose would inject robot motion as a correction.
        same_map_base=compose(map_odom,(.8,.2,-.4))
        wrong=map_odom_from_capture(same_map_base,(.9,.2,-.4))
        self.assertGreater(math.dist(wrong[:2],map_odom[:2]),.09)

    def test_global_cloud_rejects_multiple_modes_despite_narrow_strongest_pose(self):
        # A narrow covariance for mode A must not hide 40% of weight at B.
        particles=[(0.,0.,0.,.6),(2.,0.,0.,.4)]
        result=particle_cloud_metrics(particles,(0.,0.,0.))
        self.assertFalse(result['converged'])
        self.assertAlmostEqual(result['dominant_mass'],.6)
        self.assertGreater(result['xy_std_m'],.9)
        # Broad yaw spread is also ambiguous even if XY occupies one cell.
        yaw_modes=[(0.,0.,0.,.5),(0.,0.,math.pi,.5)]
        self.assertFalse(particle_cloud_metrics(yaw_modes,(0.,0.,0.))['converged'])
        for broken in ([],[(0.,0.,0.,0.)],[(0.,0.,0.,-1.)],[(math.nan,0.,0.,1.)]):
            with self.assertRaises(ValueError):particle_cloud_metrics(broken,(0.,0.,0.))

    def test_global_cloud_xy_rms_includes_both_axes_without_changing_gate(self):
        particles=[(x,y,0.,1.) for x in (-.11,.11) for y in (-.11,.11)]
        result=particle_cloud_metrics(particles,(0.,0.,0.))
        self.assertAlmostEqual(result['xy_std_m'],.11)
        self.assertAlmostEqual(result['xy_rms_m'],math.sqrt(2)*.11)
        self.assertTrue(result['converged'])

    def test_particle_diversity_detects_resampling_collapse_despite_tiny_noise(self):
        # Uniform weights and many unequal floating values can still encode
        # one physical pose after repeated stationary no-motion resampling.
        particles=[]
        for index in range(2000):
            noise=((index % 1272)-635)*1e-10
            particles.append((1.+noise,2.+noise/2.,.3+noise/100.,1.))
        result=particle_cloud_metrics(particles,(1.,2.,.3))
        self.assertEqual(result['particle_count'],2000)
        self.assertEqual(len(set(p[:3] for p in particles)),1272)
        self.assertLess(result['xy_rms_m'],1e-6)
        self.assertEqual(result['unique_pose_bins'],1)
        self.assertAlmostEqual(result['largest_pose_bin_mass'],1.)

    def test_particle_diversity_bins_include_position_yaw_and_weighted_mass(self):
        particles=[(0.,0.,0.,6.),(.002,0.,0.,2.),
                   (0.,0.,math.radians(.2),2.)]
        result=particle_cloud_metrics(particles,(0.,0.,0.))
        self.assertEqual(result['unique_pose_bins'],3)
        self.assertAlmostEqual(result['largest_pose_bin_mass'],.6)

    def test_global_cloud_accepts_weighted_wrapped_yaw_and_requires_time(self):
        particles=[(.01,.01,math.radians(179.),2.),(-.01,-.01,math.radians(-179.),1.)]
        reference=(0.,0.,math.pi)
        result=particle_cloud_metrics(particles,reference)
        self.assertTrue(result['converged'])
        self.assertLess(result['yaw_std_deg'],2.)
        gate=GlobalParticleGate()
        self.assertFalse(gate.accept(particles,reference,100.,100.))
        self.assertFalse(gate.accept(particles,reference,100.,100.))
        self.assertFalse(gate.accept(particles,reference,100.5,100.5))
        self.assertTrue(gate.accept(particles,reference,101.,101.))
        self.assertTrue(gate.ready(101.1)[0])
        self.assertFalse(gate.ready(103.1)[0])
        self.assertFalse(gate.accept([(0.,0.,0.,.5),(2.,0.,0.,.5)],reference,103.2,103.2))
        self.assertFalse(gate.ready(103.2)[0])

    def test_global_search_hypothesis_jump_before_baseline_restarts_stability(self):
        c=TrialCorrection()
        self.assertTrue(c.accept((0.,0.,0.),covariance(),100.,100.))
        self.assertTrue(c.accept((2.,-1.,1.),covariance(),100.5,100.5))
        self.assertIsNone(c.baseline)
        self.assertIsNone(c.fault)
        self.assertTrue(c.accept((2.,-1.,1.),covariance(),101.,101.))
        self.assertTrue(c.accept((2.,-1.,1.),covariance(),101.5,101.5))
        self.assertPoseClose(c.baseline,(2.,-1.,1.))

    def test_bad_initial_scan_fit_does_not_freeze_trial_reference(self):
        from geometry_msgs.msg import PoseWithCovarianceStamped
        from builtin_interfaces.msg import Time
        bridge=SimpleNamespace(correction=TrialCorrection(),pending=[],pending_issue='',
                               global_localization=False,global_requested_ros_s=None,
                               stationary_cache_enabled=False)
        transform=SimpleNamespace(transform=SimpleNamespace(
            translation=SimpleNamespace(x=0.,y=0.),
            rotation=SimpleNamespace(x=0.,y=0.,z=0.,w=1.)))
        bridge.tf_buffer=Mock()
        bridge.tf_buffer.lookup_transform.return_value=transform
        bridge.scan_match=None
        bridge.score_scan_fit=Mock(return_value=dict(ready=False,reason='bad initial fit'))
        bridge.stamp=AmclTrialBridge.stamp
        from collections import deque
        for stamp in (100,101,102):
            msg=PoseWithCovarianceStamped()
            msg.header.frame_id='map';msg.header.stamp=Time(sec=stamp)
            msg.pose.pose.orientation.w=1.;msg.pose.covariance=covariance()
            bridge.pending=deque([msg])
            AmclTrialBridge.process_pending(bridge,float(stamp))
        self.assertIsNone(bridge.correction.baseline)
        self.assertIsNone(bridge.correction.fault)
        self.assertFalse(bridge.correction.measurement_ok)
        bridge.score_scan_fit.return_value=dict(ready=True,reason='matching scan')
        for stamp in (103,104,105):
            msg=PoseWithCovarianceStamped()
            msg.header.frame_id='map';msg.header.stamp=Time(sec=stamp)
            msg.pose.pose.position.x=3.
            msg.pose.pose.orientation.w=1.;msg.pose.covariance=covariance()
            bridge.pending=deque([msg])
            AmclTrialBridge.process_pending(bridge,float(stamp))
        self.assertPoseClose(bridge.correction.baseline,(3.,0.,0.))

    def test_one_baseline_and_new_correction_do_not_double_count_motion(self):
        correction=initialized()
        baseline=correction.baseline
        self.assertEqual(baseline,(3.,-2.,.7))
        self.assertPoseClose(correction.applied,(0.,0.,0.))
        raw=(.8,.2,-.1)
        self.assertPoseClose(compose(correction.applied,raw),raw)
        desired=(.04,.03,math.radians(1.))
        correction.accept(compose(baseline,desired),covariance(),101.5,101.5)
        self.assertPoseClose(correction.target,desired)
        self.assertEqual(correction.baseline,baseline)
        for _ in range(30):correction.step(.1)
        self.assertPoseClose(correction.applied,desired)
        self.assertPoseClose(compose(correction.applied,raw),compose(desired,raw))

    def test_smoothing_limits_translation_norm_and_yaw_rate(self):
        moved=bounded_correction((0.,0.,0.),(.10,.10,math.radians(4.)),.1)
        self.assertAlmostEqual(math.hypot(*moved[:2]),.003)
        self.assertAlmostEqual(math.degrees(moved[2]),.05)
        across_wrap=bounded_correction((0.,0.,math.radians(179.99)),
                                      (0.,0.,math.radians(-179.99)),.1)
        self.assertAlmostEqual(math.degrees(across_wrap[2]),-179.99)
        self.assertEqual(bounded_correction((0.,0.,0.),(1.,1.,1.),0.),(0.,0.,0.))

    def test_baseline_requires_distinct_stable_observations_spanning_one_second(self):
        c=TrialCorrection()
        for stamp in (100.,100.,100.1,100.2):
            c.accept((0.,0.,0.),covariance(),stamp,stamp)
        self.assertIsNone(c.baseline)
        self.assertFalse(c.ready(100.2,100.2,100.2)[0])
        c.accept((.06,0.,0.),covariance(),101.,101.)
        self.assertIsNone(c.baseline)
        c.accept((.06,0.,0.),covariance(),102.,102.)
        c.accept((.06,0.,0.),covariance(),102.5,102.5)
        c.accept((.06,0.,0.),covariance(),103.,103.)
        self.assertIsNotNone(c.baseline)

    def test_bad_covariance_blocks_ready_and_never_replaces_with_raw_odom(self):
        c=initialized()
        self.assertTrue(c.ready(101.,101.,101.)[0])
        self.assertFalse(c.accept((3.,-2.,.7),covariance(xy=.2),101.5,101.5))
        self.assertFalse(c.ready(101.5,101.5,101.5)[0])
        c.accept((3.,-2.,.7),covariance(),102.,102.)
        self.assertTrue(c.ready(102.,102.,102.)[0])
        bad=covariance();bad[35]=-1.
        self.assertFalse(c.accept((3.,-2.,.7),bad,102.5,102.5))
        self.assertFalse(c.ready(102.5,102.5,102.5)[0])

    def test_stale_scan_odom_or_amcl_disables_output(self):
        c=initialized()
        self.assertTrue(c.ready(101.1,101.1,101.1)[0])
        self.assertFalse(c.ready(101.6,101.,101.6)[0])
        self.assertFalse(c.ready(101.3,101.3,101.)[0])
        self.assertFalse(c.ready(103.1,103.1,103.1)[0])
        self.assertFalse(c.ready(101.1,None,101.1)[0])

    def test_discrete_amcl_jump_latches_unhealthy_without_new_baseline(self):
        c=initialized((0.,0.,0.))
        self.assertFalse(c.accept((.151,0.,0.),covariance(),101.5,101.5))
        self.assertFalse(c.ready(101.5,101.5,101.5)[0])
        self.assertFalse(c.accept((0.,0.,0.),covariance(),102.,102.))
        self.assertPoseClose(c.baseline,(0.,0.,0.))
        self.assertIn('jumped',c.reason)

    def test_gradual_total_correction_is_still_bounded(self):
        c=initialized((0.,0.,0.))
        for stamp,x in ((101.5,.07),(102.,.14)):
            self.assertTrue(c.accept((x,0.,0.),covariance(),stamp,stamp))
        self.assertFalse(c.accept((.21,0.,0.),covariance(),102.5,102.5))
        self.assertFalse(c.ready(102.5,102.5,102.5)[0])
        self.assertIn('exceeds',c.reason)

    def test_pose_covariance_rotates_with_world_axes(self):
        cov=[0.]*36;cov[0]=.01;cov[7]=.04;cov[35]=.02
        rotated=rotate_covariance(cov,math.pi/2)
        self.assertAlmostEqual(rotated[0],.04)
        self.assertAlmostEqual(rotated[7],.01)
        self.assertAlmostEqual(rotated[35],.02)

    def test_global_scan_seed_keeps_seeded_amcl_pose_and_accepts_later_cloud(self):
        import rclpy
        from builtin_interfaces.msg import Time
        from nav2_msgs.msg import Particle, ParticleCloud
        from rclpy.context import Context
        context=Context()
        rclpy.init(context=context,domain_id=230)
        bridge=None
        try:
            with tempfile.TemporaryDirectory() as tmp:
                args=SimpleNamespace(map=Path(tmp)/'selected.yaml',initial_pose=None,
                                     wait_ready=None,global_seed_ready=True,
                                     global_localization=False,stationary_inspection=True)
                bridge=AmclTrialBridge(args,context=context)
                self.assertTrue(bridge.global_localization)
                self.assertTrue(bridge.global_requested)
                self.assertTrue(bridge.global_initialized)
                seed_stamp=bridge.global_requested_ros_s
                self.assertIsNotNone(seed_stamp)
                bridge.active=bridge.map_received=True
                bridge.next_state=float('inf')
                bridge.global_client=Mock()
                bridge.global_client.service_is_ready.return_value=True
                bridge.poll_services(time.monotonic())
                bridge.global_client.call_async.assert_not_called()

                particle=Particle();particle.pose.orientation.w=1.;particle.weight=1.
                old=ParticleCloud();old.header.frame_id='map'
                old.header.stamp=Time(sec=int(seed_stamp-1),
                    nanosec=int(((seed_stamp-1)%1)*1e9))
                old.particles=[particle]
                bridge.on_particles(old)
                self.assertIsNone(bridge.latest_cloud)
                fresh=ParticleCloud();fresh.header.frame_id='map'
                fresh.header.stamp=bridge.get_clock().now().to_msg()
                fresh.particles=[particle]
                bridge.on_particles(fresh)
                self.assertIsNotNone(bridge.latest_cloud)
                self.assertGreaterEqual(bridge.latest_cloud[0],seed_stamp)
                bridge.global_client.call_async.assert_not_called()
        finally:
            if bridge is not None:bridge.destroy_node()
            context.shutdown()

    def test_stationary_inspection_resets_fault_only_with_fresh_stationary_odom(self):
        import rclpy
        from rclpy.context import Context
        from nav_msgs.msg import Odometry
        context=Context()
        rclpy.init(context=context,domain_id=230)
        cases=[('fresh stationary inspection',True,0.,0.,0.,True,True),
               ('linear moving',True,.005,0.,0.,True,False),
               ('linear reverse',True,-.005,0.,0.,True,False),
               ('angular moving',True,0.,.01,0.,True,False),
               ('stale odom',True,0.,0.,.5,True,False),
               ('missing odom',True,0.,0.,0.,False,False),
               ('normal trial stays latched',False,0.,0.,0.,True,False)]
        try:
            with tempfile.TemporaryDirectory() as tmp:
                for label,inspection,v,w,age,has_odom,reset in cases:
                    with self.subTest(case=label):
                        args=SimpleNamespace(map=Path(tmp)/'selected.yaml',initial_pose=None,
                                             wait_ready=None,stationary_inspection=inspection)
                        bridge=AmclTrialBridge(args,context=context)
                        try:
                            bridge.stationary_inspection=inspection
                            bridge.status_pub=Mock();bridge.marker_pub=Mock();bridge.odom_pub=Mock()
                            now=bridge.get_clock().now().nanoseconds*1e-9
                            raw=Odometry();raw.header.frame_id='odom'
                            raw.child_frame_id='base_footprint'
                            raw.header.stamp=bridge.get_clock().now().to_msg()
                            raw.pose.pose.orientation.w=1.
                            raw.twist.twist.linear.x=v;raw.twist.twist.angular.z=w
                            bridge.on_odom(raw)
                            bridge.odom_stamp=now-age
                            if not has_odom:bridge.raw_odom=None
                            original=initialized((0.,0.,0.))
                            original.fault=original.reason='AMCL correction jumped'
                            bridge.correction=original
                            bridge.last_marker_wall=0.
                            with patch.object(bridge,'poll_services'), \
                                    patch.object(bridge,'process_pending'), \
                                    patch.object(bridge,'check_scan_fit'):
                                bridge.tick()
                            if reset:
                                self.assertIsNot(bridge.correction,original)
                                self.assertIsNone(bridge.correction.fault)
                                self.assertIsNone(bridge.correction.baseline)
                            else:
                                self.assertIs(bridge.correction,original)
                                self.assertEqual(bridge.correction.fault,'AMCL correction jumped')
                            bridge.odom_pub.publish.assert_not_called()
                            bridge.status_pub.publish.assert_called_once()
                            bridge.marker_pub.publish.assert_called_once()
                        finally:
                            bridge.destroy_node()
        finally:
            context.shutdown()

    def test_stationary_inspection_never_publishes_control_odom_even_when_ready(self):
        import rclpy
        from rclpy.context import Context
        from nav_msgs.msg import Odometry
        context=Context()
        rclpy.init(context=context,domain_id=230)
        bridge=None
        try:
            with tempfile.TemporaryDirectory() as tmp:
                args=SimpleNamespace(map=Path(tmp)/'selected.yaml',initial_pose=None,
                                     wait_ready=None,stationary_inspection=True)
                bridge=AmclTrialBridge(args,context=context)
                bridge.stationary_inspection=True
                bridge.status_pub=Mock();bridge.marker_pub=Mock();bridge.odom_pub=Mock()
                now=bridge.get_clock().now().nanoseconds*1e-9
                raw=Odometry();raw.header.frame_id='odom';raw.child_frame_id='base_footprint'
                raw.header.stamp=bridge.get_clock().now().to_msg();raw.pose.pose.orientation.w=1.
                bridge.on_odom(raw);bridge.scan_stamp=now
                bridge.active=bridge.map_received=True
                bridge.scan_match=dict(ready=True,reason='synthetic scan-map fit',
                                       inlier_ratio=.85,median_distance_m=.03)
                for stamp in (now-1.5,now-1.,now-.5):
                    bridge.correction.accept((0.,0.,0.),covariance(),stamp,now)
                bridge.last_marker_wall=0.
                with patch.object(bridge,'poll_services'), \
                        patch.object(bridge,'process_pending'), \
                        patch.object(bridge,'check_scan_fit'):
                    bridge.tick()
                status=json.loads(bridge.status_pub.publish.call_args.args[0].data)
                self.assertTrue(status['ready'])
                bridge.marker_pub.publish.assert_called_once()
                bridge.odom_pub.publish.assert_not_called()
        finally:
            if bridge is not None:bridge.destroy_node()
            context.shutdown()

    def test_inspection_requires_strong_scan_fit_and_valid_full_cloud_spread(self):
        import rclpy
        from rclpy.context import Context
        from nav_msgs.msg import Odometry
        context=Context()
        rclpy.init(context=context,domain_id=230)
        bridge=None
        try:
            with tempfile.TemporaryDirectory() as tmp:
                args=SimpleNamespace(map=Path(tmp)/'selected.yaml',initial_pose=None,
                                     wait_ready=None,stationary_inspection=True)
                bridge=AmclTrialBridge(args,context=context)
                bridge.status_pub=Mock();bridge.marker_pub=Mock();bridge.odom_pub=Mock()
                now=bridge.get_clock().now().nanoseconds*1e-9
                raw=Odometry();raw.header.frame_id='odom';raw.child_frame_id='base_footprint'
                raw.header.stamp=bridge.get_clock().now().to_msg();raw.pose.pose.orientation.w=1.
                bridge.on_odom(raw);bridge.scan_stamp=now
                bridge.active=bridge.map_received=True
                for stamp in (now-1.5,now-1.,now-.5):
                    bridge.correction.accept((0.,0.,0.),covariance(),stamp,now)
                diverse=particle_cloud_metrics(
                    [(index*.002,0.,0.,1.) for index in range(12)],(0.,0.,0.))
                collapsed=particle_cloud_metrics(
                    [(index*1e-10,0.,0.,1.) for index in range(2000)],(0.,0.,0.))
                dominated=particle_cloud_metrics(
                    [(0.,0.,0.,8.)]+[(index*.002,0.,0.,.2) for index in range(1,11)],
                    (0.,0.,0.))
                self.assertGreaterEqual(diverse['unique_pose_bins'],10)
                self.assertEqual(collapsed['unique_pose_bins'],1)
                self.assertGreater(dominated['largest_pose_bin_mass'],.5)
                bridge.last_marker_wall=time.monotonic()
                cases=[('boundary fit',.80,.05,diverse,True),
                       ('70 percent scan fit',.70,.03,diverse,False),
                       ('scan median above 5 cm',.85,.051,diverse,False),
                       ('one verified pose bin',.85,.03,collapsed,True),
                       ('one dominant bin',.85,.03,dominated,True),
                       ('ambiguous full cloud',.85,.03,dict(diverse,converged=False),False),
                       ('wide full cloud',.85,.03,dict(diverse,xy_rms_m=.05),False)]
                for label,inliers,median,metrics,expected in cases:
                    with self.subTest(case=label):
                        bridge.scan_match=dict(ready=True,reason='synthetic fit',
                            inlier_ratio=inliers,median_distance_m=median)
                        bridge.global_gate.metrics=metrics
                        with patch.object(bridge,'poll_services'), \
                                patch.object(bridge,'process_pending'), \
                                patch.object(bridge,'check_scan_fit'):
                            bridge.tick()
                        status=json.loads(bridge.status_pub.publish.call_args.args[0].data)
                        self.assertEqual(status['ready'],expected)
                        bridge.odom_pub.publish.assert_not_called()
                # The stricter fit/spread gate belongs to stationary
                # inspection; a normal trial retains its existing ready rule.
                bridge.stationary_inspection=False
                bridge.scan_match=dict(ready=True,reason='normal trial fit',
                                       inlier_ratio=.70,median_distance_m=.051)
                bridge.global_gate.metrics=collapsed
                with patch.object(bridge,'poll_services'), \
                        patch.object(bridge,'process_pending'), \
                        patch.object(bridge,'check_scan_fit'):
                    bridge.tick()
                status=json.loads(bridge.status_pub.publish.call_args.args[0].data)
                self.assertTrue(status['ready'])
                bridge.odom_pub.publish.assert_called_once()
        finally:
            if bridge is not None:bridge.destroy_node()
            context.shutdown()

    def test_isolated_bridge_publishes_contract_and_withholds_stale_output(self):
        import rclpy
        from rclpy.context import Context
        from nav_msgs.msg import Odometry
        context=Context()
        # Isolate all topic discovery from the real robot's ROS domain.
        rclpy.init(context=context,domain_id=230)
        bridge=None
        clock_patch=None
        try:
            with tempfile.TemporaryDirectory() as tmp:
                args=SimpleNamespace(map=Path(tmp)/'selected.yaml',initial_pose=None,
                                     wait_ready=Path(tmp)/'ready.json')
                bridge=AmclTrialBridge(args,context=context)
                bridge.status_pub=Mock();bridge.odom_pub=Mock();bridge.initial_pub=Mock()
                # Rejected initial map fit must remain observable even before
                # any transform is accepted into the frozen trial baseline.
                bridge.latest_observation=(100.,(1.,2.,.3))
                bridge.scan_stamp=100.
                bad_fit=dict(ready=False,reason='scan/map mismatch',inlier_ratio=.61)
                with patch.object(bridge,'score_scan_fit',return_value=bad_fit):
                    bridge.check_scan_fit(100.)
                self.assertEqual(bridge.scan_match,bad_fit)
                self.assertIsNone(bridge.correction.amcl_stamp)
                bridge.last_fit_key=None
                bridge.latest_observation=None
                now=bridge.get_clock().now().nanoseconds*1e-9
                raw=Odometry()
                raw.header.frame_id='odom';raw.child_frame_id='base_footprint'
                raw.header.stamp=bridge.get_clock().now().to_msg()
                # Advance sensor ages explicitly below. Test execution time
                # under load must not make the synthetic 0.2 s odom stale.
                frozen=SimpleNamespace(nanoseconds=round(now*1e9),to_msg=lambda:raw.header.stamp)
                clock_patch=patch.object(bridge,'get_clock',
                    return_value=SimpleNamespace(now=lambda:frozen))
                clock_patch.start()
                raw.pose.pose.position.x=.8;raw.pose.pose.position.y=.2
                raw.pose.pose.orientation.w=1.
                bridge.on_odom(raw)
                bridge.scan_stamp=now
                baseline=(3.,-2.,.7)
                for stamp in (now-1.5,now-1.,now-.5):
                    bridge.correction.accept(baseline,covariance(),stamp,now)
                bridge.correction.accept(compose(baseline,(.04,.03,math.radians(1.))),
                                         covariance(),now,now)
                bridge.active=bridge.map_received=True
                bridge.scan_match=dict(ready=True,reason='synthetic scan-map fit')
                bridge.state_poll.states['amcl']=dict(id=3,stamp=time.monotonic())
                bridge.next_state=float('inf')
                bridge.poll_services(time.monotonic())
                bridge.initial_pub.publish.assert_not_called()
                bridge.last_tick=time.monotonic()-.05
                with patch.object(bridge,'poll_services'), patch.object(bridge,'check_scan_fit'):
                    bridge.tick()
                output=bridge.odom_pub.publish.call_args.args[0]
                status=json.loads(bridge.status_pub.publish.call_args.args[0].data)
                self.assertTrue(status['ready'])
                self.assertEqual(status['frame_id'],'r3_trial')
                self.assertEqual(status['run_id'],bridge.run_id)
                self.assertEqual(status['baseline_map_odom'],list(baseline))
                self.assertEqual(status['map_file'],str(args.map.resolve()))
                self.assertEqual(status['covariance_model'],'EKF rotated plus AMCL diagonal bound')
                self.assertGreater(status['published_ros_s'],0.)
                self.assertEqual(output.header.frame_id,'r3_trial')
                self.assertEqual(output.header.stamp,raw.header.stamp)
                self.assertEqual(output.child_frame_id,'base_footprint')
                self.assertGreaterEqual(output.pose.covariance[0],.03**2)
                self.assertGreaterEqual(output.pose.covariance[7],.03**2)
                self.assertGreaterEqual(output.pose.covariance[35],math.radians(1.)**2)
                expected=compose(status['correction_xyyaw'],(.8,.2,0.))
                self.assertAlmostEqual(output.pose.pose.position.x,expected[0])
                self.assertAlmostEqual(output.pose.pose.position.y,expected[1])
                self.assertEqual(raw.pose.pose.position.x,.8)
                self.assertEqual(json.loads(args.wait_ready.read_text())['run_id'],bridge.run_id)
                bridge.odom_pub.publish.reset_mock()
                bridge.scan_stamp=now-1.
                with patch.object(bridge,'poll_services'), patch.object(bridge,'check_scan_fit'):
                    bridge.tick()
                bridge.odom_pub.publish.assert_not_called()
                status=json.loads(bridge.status_pub.publish.call_args.args[0].data)
                self.assertFalse(status['ready'])
                self.assertIn('LiDAR',status['reason'])
                # Global mode cannot publish localized odom merely because
                # the usual pose covariance/baseline gate is satisfied.
                bridge.scan_stamp=now
                bridge.global_localization=True
                bridge.global_initialized=True
                bridge.odom_pub.publish.reset_mock()
                with patch.object(bridge,'poll_services'), patch.object(bridge,'check_scan_fit'):
                    bridge.tick()
                bridge.odom_pub.publish.assert_not_called()
                status=json.loads(bridge.status_pub.publish.call_args.args[0].data)
                self.assertFalse(status['ready'])
                self.assertTrue(status['global_localization']['enabled'])
                for stamp in (now-1.5,now-1.,now-.5):
                    bridge.global_gate.accept([(0.,0.,0.,1.)],(0.,0.,0.),stamp,now)
                with patch.object(bridge,'poll_services'), patch.object(bridge,'check_scan_fit'):
                    bridge.tick()
                self.assertTrue(json.loads(bridge.status_pub.publish.call_args.args[0].data)['ready'])
                bridge.odom_pub.publish.assert_called_once()
                bridge.odom_pub.publish.reset_mock()
                bridge.scan_match=dict(ready=False,reason='synthetic selected map does not fit scan')
                with patch.object(bridge,'poll_services'),patch.object(bridge,'check_scan_fit'):
                    bridge.tick()
                bridge.odom_pub.publish.assert_not_called()
                self.assertFalse(json.loads(bridge.status_pub.publish.call_args.args[0].data)['ready'])
                bridge.scan_match=dict(ready=True,reason='synthetic fit')
                # Global service is issued once after active AMCL and map.
                bridge.global_requested=False
                completed=Mock();completed.done.return_value=True;completed.result.return_value=SimpleNamespace()
                client=Mock();client.service_is_ready.return_value=True;client.call_async.return_value=completed
                bridge.global_client=client
                bridge.poll_services(time.monotonic())
                bridge.poll_services(time.monotonic()+.6)
                client.call_async.assert_called_once()
                bridge.initial_pub.publish.assert_not_called()
        finally:
            if clock_patch is not None:clock_patch.stop()
            if bridge is not None:bridge.destroy_node()
            context.shutdown()


if __name__=='__main__':unittest.main()
