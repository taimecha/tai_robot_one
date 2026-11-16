"""Role-change client checks never publish a goal, transform or velocity."""
import copy
import math
from pathlib import Path
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock

from rclpy.time import Time
from nav_msgs.msg import Odometry

from r3_enable_trial import enable_trial,snapshot_digest,validate_handoff


MAP=Path('/test/map.yaml')


def source(now=100.):
    covariance=[0.]*36;covariance[0]=covariance[7]=.01**2;covariance[35]=math.radians(.5)**2
    snapshot=dict(run_id='same-run',map_file=str(MAP),amcl_stamp_s=75.,cloud_stamp_s=75.1,
        map_pose_xyyaw=[2.,1.,.2],map_odom_xyyaw=[2.,1.,.2],odom_anchor_xyyaw=[0.,0.,0.],
        covariance_6x6=covariance,particle_metrics=dict(converged=True,unique_pose_bins=50,
            largest_pose_bin_mass=.1,xy_rms_m=.014,yaw_std_deg=.5),particle_pairing=dict(method='bounded_timestamps'),
        occupancy_map_signature='same-map')
    return dict(ready=True,reason='parked verified',published_ros_s=now,run_id='same-run',
        frame_id='r3_trial',map_file=str(MAP),continuous_localization=True,navigation_monitor=True,
        robot_stationary=True,stationary_cache=dict(validated=True,nomotion_request_count=1),
        scan_map_fit=dict(ready=True,scan_stamp_s=now,inlier_ratio=.85,median_distance_m=.01),
        stationary_snapshot=snapshot,current_map_odom_xyyaw=[2.,1.,.2],baseline_map_odom=[1.,1.,.1])


def enabled(initial,now=100.1):
    status=copy.deepcopy(initial)
    status.update(published_ros_s=now,navigation_monitor=False,role='trial',baseline_map_odom=[2.,1.,.2])
    status['scan_map_fit']['scan_stamp_s']=now
    status['trial_handoff']=dict(enabled=True,at_ros_s=100.05,enabled_ros_s=100.05,
        source_amcl_stamp_s=75.,source_cloud_stamp_s=75.1,baseline_map_odom=[2.,1.,.2],
        nomotion_request_count=1,previous_role='navigation_monitor',
        source='validated_navigation_snapshot_without_amcl_refresh')
    return status


def odom(now):
    msg=Odometry();msg.header.frame_id='r3_trial';msg.child_frame_id='base_footprint'
    msg.header.stamp=Time(nanoseconds=round(now*1e9)).to_msg();msg.pose.pose.orientation.w=1.
    return msg


class EnableTrialTest(unittest.TestCase):
    def test_role_handoff_keeps_aged_original_stamp_covariance_and_particle_evidence(self):
        before=source();after=enabled(before);original=copy.deepcopy(before)
        result=validate_handoff(before,after,odom(100.1),100.12,MAP)
        self.assertEqual(result['status'],'TRIAL_LOCALIZATION_READY')
        self.assertEqual(result['source_amcl_stamp_s'],75.)
        self.assertEqual(result['original_snapshot_sha256'],snapshot_digest(before['stationary_snapshot']))
        self.assertEqual(result['nomotion_request_count_before'],result['nomotion_request_count_after'])
        self.assertEqual(result['baseline_map_odom'],[2.,1.,.2])
        self.assertEqual(before,original)

    def test_changed_source_run_map_covariance_or_resample_count_is_rejected(self):
        before=source();valid=enabled(before)
        variants=[]
        wrong=copy.deepcopy(valid);wrong['run_id']='restart';variants.append(wrong)
        wrong=copy.deepcopy(valid);wrong['map_file']='/other/map.yaml';variants.append(wrong)
        wrong=copy.deepcopy(valid);wrong['stationary_snapshot']['covariance_6x6'][0]/=2;variants.append(wrong)
        wrong=copy.deepcopy(valid);wrong['stationary_snapshot']['amcl_stamp_s']=99.;variants.append(wrong)
        wrong=copy.deepcopy(valid);wrong['stationary_cache']['nomotion_request_count']=2;variants.append(wrong)
        wrong=copy.deepcopy(valid);wrong['stationary_snapshot']['particle_metrics']['unique_pose_bins']=1;variants.append(wrong)
        for value in variants:
            with self.subTest(value=value),self.assertRaises(ValueError):
                validate_handoff(before,value,odom(100.1),100.12,MAP)

    def test_fresh_role_and_odometry_are_required(self):
        before=source();valid=enabled(before)
        for change in (dict(navigation_monitor=True),dict(role='navigation_monitor'),
                dict(ready=False),dict(published_ros_s=99.),dict(robot_stationary=False),dict(trial_handoff=None)):
            with self.subTest(change=change),self.assertRaises(ValueError):
                validate_handoff(before,dict(valid,**change),odom(100.1),100.12,MAP)
        for message in (None,odom(99.),odom(100.4)):
            with self.subTest(message=message),self.assertRaises(ValueError):
                validate_handoff(before,valid,message,100.12,MAP)
        wrong=odom(100.1);wrong.header.frame_id='odom'
        with self.assertRaises(ValueError):validate_handoff(before,valid,wrong,100.12,MAP)

    def test_idempotent_request_preserves_the_existing_frozen_baseline(self):
        initial=enabled(source());final=copy.deepcopy(initial);final['published_ros_s']=100.2
        self.assertEqual(validate_handoff(initial,final,odom(100.2),100.21,MAP)['status'],'TRIAL_LOCALIZATION_READY')
        final['baseline_map_odom'][0]+=.01;final['trial_handoff']['baseline_map_odom'][0]+=.01
        with self.assertRaisesRegex(ValueError,'already frozen'):
            validate_handoff(initial,final,odom(100.2),100.21,MAP)

    def probe(self,*,reject=False,never_respond=False,owners_until=0.,responses=None):
        wall=[0.];node=NS(status=source(),odom=None,odom_stamps=[])
        initial=copy.deepcopy(node.status)
        node.now=lambda:100.+wall[0]
        node.request_times=[];node.last_response_success=None
        def call(request):
            index=len(node.request_times)
            answer=responses[min(index,len(responses)-1)] if responses is not None else not reject
            node.request_times.append(wall[0]);node.last_response_success=answer
            result=None if answer is None else NS(success=answer,message='enabled' if answer else 'rejected')
            return NS(done=lambda:not never_respond,result=lambda:result)
        node.service=NS(service_is_ready=lambda:True,call_async=Mock(side_effect=call))
        node.velocity_publishers=lambda:({'/cmd_vel_safe':['collision_monitor']} if wall[0]<owners_until else {})
        def spin():
            wall[0]=round(wall[0]+.05,6);now=node.now()
            if node.service.call_async.called and node.last_response_success is True and not never_respond:
                node.status=enabled(initial,now)
                node.status['trial_handoff']['at_ros_s']=100.05+owners_until
                node.status['trial_handoff']['enabled_ros_s']=100.05+owners_until
                node.odom=odom(now);node.odom_stamps.append(now)
            else:
                node.status['published_ros_s']=now
                node.status['scan_map_fit']['scan_stamp_s']=now
        node.spin_once=spin
        return node,wall

    def test_request_waits_for_velocity_owners_then_calls_service_once(self):
        node,wall=self.probe(owners_until=.2)
        result=enable_trial(node,MAP,1.,clock=lambda:wall[0],ok=lambda:True)
        self.assertEqual(result['status'],'TRIAL_LOCALIZATION_READY')
        self.assertGreaterEqual(wall[0],.3)
        self.assertGreaterEqual(result['confirmed_odom_samples'],3)
        node.service.call_async.assert_called_once()

    def test_explicit_rejection_is_retried_at_most_three_times(self):
        node,wall=self.probe(reject=True)
        with self.assertRaisesRegex(RuntimeError,'role change rejected'):
            enable_trial(node,MAP,1.,clock=lambda:wall[0],ok=lambda:True)
        self.assertEqual(node.service.call_async.call_count,3)
        self.assertGreaterEqual(wall[0],.55)
        self.assertTrue(all(b-a>=.25-1e-9 for a,b in zip(node.request_times,node.request_times[1:])))

    def test_explicit_rejection_then_success_changes_role_only_once(self):
        node,wall=self.probe(responses=[False,True])
        result=enable_trial(node,MAP,1.,clock=lambda:wall[0],ok=lambda:True)
        self.assertEqual(result['status'],'TRIAL_LOCALIZATION_READY')
        self.assertEqual(result['role_change_service_calls'],2)
        self.assertEqual(result['successful_role_changes'],1)
        self.assertEqual(result['nomotion_request_count_before'],1)
        self.assertEqual(result['nomotion_request_count_after'],1)
        self.assertGreaterEqual(node.request_times[1]-node.request_times[0],.25-1e-9)

    def test_uncertain_response_never_repeats_request(self):
        node,wall=self.probe(responses=[None])
        with self.assertRaisesRegex(RuntimeError,'uncertain response'):
            enable_trial(node,MAP,1.,clock=lambda:wall[0],ok=lambda:True)
        node.service.call_async.assert_called_once()

    def test_rejected_request_cannot_retry_changed_source_role_or_fault(self):
        for mutation in ('source','role','fault','map'):
            with self.subTest(mutation=mutation):
                node,wall=self.probe(reject=True)
                original_spin=node.spin_once
                def spin():
                    original_spin()
                    if node.service.call_async.called:
                        if mutation=='source':node.status['stationary_snapshot']['covariance_6x6'][0]/=2
                        elif mutation=='role':node.status.update(navigation_monitor=False,role='trial')
                        elif mutation=='fault':node.status['ready']=False
                        elif mutation=='map':node.status['map_file']='/other/map.yaml'
                node.spin_once=spin
                with self.assertRaises(RuntimeError):
                    enable_trial(node,MAP,1.,clock=lambda:wall[0],ok=lambda:True)
                node.service.call_async.assert_called_once()

    def test_quality_precheck_waits_for_current_good_scan_before_request(self):
        node,wall=self.probe()
        original_spin=node.spin_once
        def spin():
            original_spin()
            if not node.service.call_async.called:
                node.status['scan_map_fit']['inlier_ratio']=.64 if wall[0]<.2 else .85
        node.spin_once=spin
        result=enable_trial(node,MAP,1.,clock=lambda:wall[0],ok=lambda:True)
        self.assertEqual(result['status'],'TRIAL_LOCALIZATION_READY')
        self.assertGreaterEqual(node.request_times[0],.2)
        node.service.call_async.assert_called_once()

    def test_wide_covariance_ambiguous_particles_or_stale_scored_scan_prevent_rpc(self):
        for mutation in ('covariance','particles','stale_scan'):
            with self.subTest(mutation=mutation):
                node,wall=self.probe()
                original_spin=node.spin_once
                def spin():
                    original_spin()
                    if mutation=='covariance':node.status['stationary_snapshot']['covariance_6x6'][0]=.06**2
                    elif mutation=='particles':node.status['stationary_snapshot']['particle_metrics']['converged']=False
                    else:node.status['scan_map_fit']['scan_stamp_s']=99.
                node.spin_once=spin
                with self.assertRaisesRegex(RuntimeError,'timeout'):
                    enable_trial(node,MAP,.3,clock=lambda:wall[0],ok=lambda:True)
                node.service.call_async.assert_not_called()

    def test_concentrated_unchanged_posterior_allows_the_role_request(self):
        node,wall=self.probe()
        original_spin=node.spin_once
        def spin():
            original_spin()
            node.status['stationary_snapshot']['particle_metrics'].update(
                unique_pose_bins=5,largest_pose_bin_mass=.95)
        node.spin_once=spin
        result=enable_trial(node,MAP,1.,clock=lambda:wall[0],ok=lambda:True)
        self.assertEqual(result['status'],'TRIAL_LOCALIZATION_READY')
        node.service.call_async.assert_called_once()

    def test_service_response_timeout_is_bounded_without_retry(self):
        node,wall=self.probe(never_respond=True)
        with self.assertRaisesRegex(RuntimeError,'timeout'):
            enable_trial(node,MAP,.3,clock=lambda:wall[0],ok=lambda:True)
        node.service.call_async.assert_called_once()
        self.assertAlmostEqual(wall[0],.3)

    def test_existing_velocity_owner_prevents_even_the_role_change_call(self):
        node,wall=self.probe(owners_until=2.)
        with self.assertRaisesRegex(RuntimeError,'velocity publishers'):
            enable_trial(node,MAP,.3,clock=lambda:wall[0],ok=lambda:True)
        node.service.call_async.assert_not_called()


if __name__=='__main__':unittest.main()
