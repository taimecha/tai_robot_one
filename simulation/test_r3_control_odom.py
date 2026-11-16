"""Fresh EKF control survives delayed validation without extending its lease."""
import copy
import json
import math
from pathlib import Path
import unittest

from nav_msgs.msg import Odometry
from rclpy.time import Time

from r3_control_odom import ControlOdomGuard,rotate_covariance
from r3_amcl_bridge import rotate_covariance as original_rotate_covariance


MAP=Path('/test/edited.yaml')


def odom(stamp,x=0.,yaw=0.):
    message=Odometry()
    message.header.frame_id='odom';message.child_frame_id='base_footprint'
    message.header.stamp=Time(nanoseconds=round(stamp*1e9)).to_msg()
    message.pose.pose.position.x=x
    message.pose.pose.orientation.z=math.sin(yaw/2.)
    message.pose.pose.orientation.w=math.cos(yaw/2.)
    message.twist.twist.linear.x=.1
    message.pose.covariance[0]=message.pose.covariance[7]=.001
    message.pose.covariance[35]=.0001
    return message


def status(now=100.,mode='prediction'):
    source=now if mode=='live' else now-60.
    covariance=[0.]*36;covariance[0]=covariance[7]=.01**2
    covariance[35]=math.radians(.5)**2
    snapshot=dict(run_id='one-run',map_file=str(MAP),amcl_stamp_s=source,cloud_stamp_s=source+.01,
        map_pose_xyyaw=[3.,1.4,-1.55],odom_anchor_xyyaw=[0.,0.,0.],
        map_odom_xyyaw=[3.,1.4,-1.55],covariance_6x6=covariance,
        particle_metrics=dict(converged=True,xy_rms_m=.014,yaw_std_deg=.5,
                              unique_pose_bins=5,largest_pose_bin_mass=.95))
    return dict(ready=True,published_ros_s=now,frame_id='r3_trial',run_id='one-run',
        map_file=str(MAP),baseline_map_odom=[3.,1.4,-1.55],correction_xyyaw=[.01,-.02,.003],
        external_control_odom=True,role='trial',navigation_monitor=False,stationary_inspection=False,
        localization_fault=None,xy_std_m=.01,yaw_std_deg=.5,amcl_stamp_s=source,
        scan_map_fit=dict(ready=True,scan_stamp_s=now,inlier_ratio=.86,median_distance_m=.014),
        stationary_snapshot=snapshot,live_observation_validated=mode=='live',
        motion_prediction_validated=mode=='prediction',stationary_cache=dict(validated=mode=='parked'),
        motion_prediction=dict(started_ros_s=now-.1,anchor_odom_xyyaw=[0.,0.,0.],
            source_amcl_stamp_s=source,ended=False,max_duration_s=2.,max_distance_m=.08,max_yaw_rad=.08))


class ControlOdomTest(unittest.TestCase):
    def test_350_ms_supervisor_delay_keeps_fresh_control_without_retiming_any_source(self):
        guard=ControlOdomGuard(MAP);state=status();original=copy.deepcopy(state)
        for index in range(19):
            now=100.+index*.02
            raw=odom(now-.01,x=index*.002)
            old=copy.deepcopy(raw)
            result=guard.transform(state,raw,now)
            self.assertEqual(result.header.stamp,raw.header.stamp)
            self.assertEqual(raw,old)
            self.assertEqual(result.header.frame_id,'r3_trial')
            self.assertAlmostEqual(result.pose.covariance[35],raw.pose.covariance[35]+math.radians(.5)**2)
        self.assertEqual(state,original)
        result=guard.transform(state,odom(100.51,x=.04),100.51)
        self.assertEqual(guard.mode,'ekf_tracking')
        self.assertEqual(result.header.stamp,odom(100.51).header.stamp)
        self.assertEqual(guard.reference_state['amcl_stamp_s'],original['amcl_stamp_s'])

    def test_frozen_ekf_bad_map_and_invalid_localization_are_still_rejected(self):
        cases=[('unready',dict(status(),ready=False)),
               ('wrong map',dict(status(),map_file='/wrong.yaml')),
               ('navigation',dict(status(),navigation_monitor=True,role='navigation_monitor')),
               ('no delegate',dict(status(),external_control_odom=False)),
               ('fault',dict(status(),localization_fault='map correction jumped'))]
        for label,state in cases:
            with self.subTest(case=label),self.assertRaises(ValueError):
                ControlOdomGuard(MAP).transform(state,odom(100.),100.)
        with self.assertRaisesRegex(ValueError,'EKF odom is stale'):
            ControlOdomGuard(MAP).transform(status(),odom(99.79),100.)
        state=status();state['scan_map_fit']['scan_stamp_s']=99.49
        with self.assertRaisesRegex(ValueError,'LiDAR scan is stale'):
            ControlOdomGuard(MAP).transform(state,odom(100.),100.)

    def test_latest_own_odom_enforces_motion_onset_distance_yaw_and_time(self):
        for x,yaw,now in ((.08001,0.,100.),(0.,.08001,100.),(0.,0.,102.01)):
            state=status(now);state['motion_prediction']['started_ros_s']=99.9
            with self.subTest(x=x,yaw=yaw,now=now),self.assertRaisesRegex(ValueError,'motion onset exceeded'):
                ControlOdomGuard(MAP).transform(state,odom(now,x=x,yaw=yaw),now)

    def test_live_amcl_cannot_be_extended_by_fresh_status_or_fresh_ekf(self):
        state=status(mode='live')
        state['amcl_stamp_s']=state['stationary_snapshot']['amcl_stamp_s']=97.
        state['stationary_snapshot']['cloud_stamp_s']=97.01
        with self.assertRaisesRegex(ValueError,'live AMCL observation is stale'):
            ControlOdomGuard(MAP).transform(state,odom(100.),100.)

    def test_baseline_change_cannot_reanchor_a_running_trial(self):
        guard=ControlOdomGuard(MAP);state=status()
        guard.transform(state,odom(100.),100.)
        changed=dict(state,baseline_map_odom=[3.01,1.4,-1.55])
        with self.assertRaisesRegex(ValueError,'map/reference changed'):
            guard.transform(changed,odom(100.02),100.02)

    def test_duplicate_or_reversed_capture_is_not_fresh_odometry(self):
        guard=ControlOdomGuard(MAP);state=status()
        guard.transform(state,odom(100.),100.)
        with self.assertRaisesRegex(ValueError,'duplicate'):
            guard.transform(state,odom(100.),100.01)
        with self.assertRaisesRegex(ValueError,'clock reversed'):
            guard.transform(state,odom(99.99),100.01)

    def test_fast_covariance_rotation_matches_existing_full_matrix_operation(self):
        covariance=[math.sin(i)*.001 for i in range(36)]
        for yaw in (0.,.03,-1.5,math.pi):
            actual=rotate_covariance(covariance,yaw)
            expected=original_rotate_covariance(covariance,yaw)
            for a,b in zip(actual,expected):self.assertAlmostEqual(a,b,places=14)

    def test_stale_or_rejected_amcl_keeps_the_last_frame_and_fresh_ekf_moving(self):
        guard=ControlOdomGuard(MAP);state=status(mode='live')
        guard.transform(state,odom(100.),100.)
        old_source=guard.reference_state['amcl_stamp_s']
        # Late/ambiguous map updates do not stop the locked local path and
        # never become a newly accepted correction.
        for index in range(1,36):
            now=100.+index*.1
            candidate=copy.deepcopy(state)
            candidate.update(ready=False,published_ros_s=now,correction_xyyaw=[.15,.1,.02])
            candidate['localization_fault']='AMCL correction jumped'
            output=guard.transform(candidate,odom(now,x=index*.002),now)
            self.assertEqual(guard.mode,'ekf_tracking')
            self.assertAlmostEqual(output.pose.pose.position.x,.01+math.cos(.003)*index*.002)
            self.assertEqual(guard.reference_state['amcl_stamp_s'],old_source)
        with self.assertRaisesRegex(ValueError,'EKF odom is stale'):
            guard.transform(state,odom(103.5,x=.07),103.71)

    def test_valid_new_amcl_can_resume_in_the_same_frame_after_an_update_gap(self):
        guard=ControlOdomGuard(MAP);state=status(mode='live')
        guard.transform(state,odom(100.),100.)
        for index in range(1,8):
            guard.transform(state,odom(100.+index*.1,x=index*.002),100.+index*.1)
        self.assertEqual(guard.mode,'ekf_tracking')
        fresh=status(100.8,mode='live')
        fresh['correction_xyyaw']=[.011,-.02,.003]
        result=guard.transform(fresh,odom(100.8,x=.016),100.8)
        self.assertEqual(guard.mode,'amcl_corrected_ekf')
        self.assertEqual(guard.reference_state['amcl_stamp_s'],100.8)
        self.assertAlmostEqual(result.pose.pose.position.x,.011+math.cos(.003)*.016)

    def test_amcl_recovery_cannot_jump_the_control_frame_after_a_delivery_gap(self):
        guard=ControlOdomGuard(MAP);initial=status(mode='live')
        guard.transform(initial,odom(100.),100.)
        for index in range(1,9):
            guard.transform(initial,odom(100.+index*.1,x=index*.001),100.+index*.1)
        before=guard.applied_correction
        fresh=status(100.9,mode='live')
        fresh['correction_xyyaw']=[.09,-.02,.03]
        guard.transform(fresh,odom(100.9,x=.009),100.9)
        self.assertLessEqual(math.dist(guard.applied_correction[:2],before[:2]),.003+1e-12)
        self.assertLessEqual(abs(guard.applied_correction[2]-before[2]),math.radians(.05)+1e-12)
        self.assertEqual(guard.target_correction,(.09,-.02,.03))


if __name__=='__main__':unittest.main()
