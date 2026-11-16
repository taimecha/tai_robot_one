"""New map-bound recognition point and Nav2 launch contract; no robot motion."""
import importlib.util
import json
import math
import os
from pathlib import Path
import subprocess
import tempfile
import unittest
from types import SimpleNamespace
from types import MethodType
from unittest.mock import Mock,patch
from builtin_interfaces.msg import Time
from ament_index_python.packages import get_package_prefix
import yaml

from r3_recognition_goal import (map_identity,validate_goal,pose_error,SCHEMA,atomic_json,
                                 RecognitionGoal,validate_map_update,validate_arrival,
                                 validate_a_localization,ARRIVAL_XY_TOLERANCE_M,
                                 ARRIVAL_YAW_TOLERANCE_RAD,A_XY_RMS_LIMIT_M,
                                 A_PREFERRED_XY_M)
from r3_amcl_bridge import particle_cloud_metrics


ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('r3_navigation',ROOT/'camera/detect_astra/launch/r3_navigation.launch.py')
launch=importlib.util.module_from_spec(spec)
spec.loader.exec_module(launch)


class RecognitionGoalTest(unittest.TestCase):
    def map(self,directory):
        directory=Path(directory)
        (directory/'map.pgm').write_bytes(b'P5\n2 2\n255\n\xff\xff\xff\xff')
        path=directory/'map.yaml'
        path.write_text('image: map.pgm\nresolution: 0.05\norigin: [0, 0, 0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
        return path

    def goal(self,identity):
        return dict(schema=SCHEMA,purpose='pallet_recognition_A',frame_id='map',
                    goal_id='new-uuid',saved_at_utc='2026-11-12T00:00:00Z',
                    pose_xyyaw=[1.,2.,.3],map=identity)

    def test_new_goal_binds_to_map_image_and_geometry(self):
        with tempfile.TemporaryDirectory() as tmp:
            path=self.map(tmp);identity=map_identity(path);goal=self.goal(identity)
            self.assertEqual(validate_goal(goal,identity),(1.,2.,.3))
            (Path(tmp)/'map.pgm').write_bytes(b'P5\n2 2\n255\n\xff\x00\xff\xff')
            with self.assertRaisesRegex(ValueError,'different map'):
                validate_goal(goal,map_identity(path))
            self.map(tmp)
            path.write_text(path.read_text().replace('resolution: 0.05','resolution: 0.10'))
            with self.assertRaisesRegex(ValueError,'different map'):
                validate_goal(goal,map_identity(path))

    def test_old_or_invalid_goal_never_becomes_new_A(self):
        identity={'fingerprint_sha256':'maphash'}
        for invalid in ({}, {'pose':[1,2,3]},dict(self.goal(identity),schema='legacy'),
                        dict(self.goal(identity),frame_id='odom'),
                        dict(self.goal(identity),pose_xyyaw=[1.,2.,float('nan')])):
            with self.subTest(goal=invalid),self.assertRaises(ValueError):
                validate_goal(invalid,identity)

    def test_pose_error_wraps_heading_and_save_is_complete(self):
        xy,yaw=pose_error((1.,2.,math.radians(179)),(1.03,2.04,math.radians(-179)))
        self.assertAlmostEqual(xy,.05)
        self.assertAlmostEqual(math.degrees(yaw),2.)
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'new'/'recognition_A.json'
            value=self.goal({'fingerprint_sha256':'maphash'})
            atomic_json(path,value)
            self.assertEqual(json.loads(path.read_text()),value)
            self.assertFalse(path.with_suffix('.json.tmp').exists())

    def test_invalid_goal_is_rejected_before_any_action_or_robot_readiness(self):
        with tempfile.TemporaryDirectory() as tmp:
            map_file=self.map(tmp)
            goal_file=Path(tmp)/'old.json'
            atomic_json(goal_file,{'pose':[1,2,3]})
            probe=SimpleNamespace(wait_pose=Mock(),client=Mock())
            with self.assertRaises(ValueError):
                RecognitionGoal.navigate(probe,map_file,goal_file,10.)
            probe.wait_pose.assert_not_called()
            probe.client.send_goal_async.assert_not_called()

    def test_localization_loss_is_detected_during_action_wait(self):
        future=SimpleNamespace(done=lambda:False)
        probe=SimpleNamespace(spin_once=Mock(),current_pose=Mock(side_effect=ValueError('AMCL lost')))
        with patch('r3_recognition_goal.rclpy.ok',return_value=True), \
                patch('r3_recognition_goal.time.monotonic',return_value=100.):
            with self.assertRaisesRegex(ValueError,'AMCL lost'):
                RecognitionGoal.await_future(probe,future,110.,True)
        probe.spin_once.assert_called_once()
        probe.current_pose.assert_called_once_with(False)

    def test_existing_A_is_never_implicitly_replaced(self):
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'existing.json'
            path.write_text('original A')
            probe=SimpleNamespace(wait_pose=Mock())
            with self.assertRaisesRegex(ValueError,'already exists'):
                RecognitionGoal.save(probe,Path('/not/read/map.yaml'),path,10.)
            probe.wait_pose.assert_not_called()
            self.assertEqual(path.read_text(),'original A')

    def test_accepted_goal_is_canceled_on_localization_loss_or_timeout(self):
        for error in (ValueError('AMCL lost'),RuntimeError('Nav2 action timed out')):
            with self.subTest(error=error),tempfile.TemporaryDirectory() as tmp:
                map_file=self.map(tmp)
                path=Path(tmp)/'new_A.json'
                atomic_json(path,self.goal(map_identity(map_file)))
                result_future=SimpleNamespace(done=lambda:False)
                handle=SimpleNamespace(accepted=True,get_result_async=lambda:result_future,
                    cancel_goal_async=Mock(return_value=SimpleNamespace(done=lambda:True)))
                client=SimpleNamespace(wait_for_server=Mock(return_value=True),send_goal_async=Mock())
                clock=SimpleNamespace(now=lambda:SimpleNamespace(to_msg=lambda:Time(sec=123)))
                probe=SimpleNamespace(wait_pose=Mock(return_value=(.9,2.,.3)),client=client,
                    get_clock=lambda:clock,await_future=Mock(side_effect=[handle,error]),
                    goal_handle=None,result_future=None,localization_evidence=Mock(return_value={}))
                probe.cancel=MethodType(RecognitionGoal.cancel,probe)
                with self.assertRaises(type(error)):
                    RecognitionGoal.navigate(probe,map_file,path,100.)
                handle.cancel_goal_async.assert_called_once()
                client.send_goal_async.assert_called_once()
                sent=client.send_goal_async.call_args.args[0]
                self.assertEqual(sent.pose.header.frame_id,'map')
                self.assertAlmostEqual(sent.pose.pose.position.x,1.)
                self.assertAlmostEqual(sent.pose.pose.position.y,2.)

    def test_map_correction_jump_is_not_confused_with_robot_motion(self):
        validate_map_update((1.,2.,math.radians(179.)),(1.01,2.,math.radians(-179.)))
        with self.assertRaisesRegex(ValueError,'jumped'):
            validate_map_update((1.,2.,0.),(1.2,2.,0.))
        with self.assertRaisesRegex(ValueError,'jumped'):
            validate_map_update((1.,2.,0.),(1.,2.,math.radians(6.)))

    def test_recognition_region_accepts_under_ten_cm_and_five_degrees(self):
        goal=(1.,2.,math.radians(179.))
        # Recognition A is now a coarse region, separate from estimator spread.
        self.assertEqual(A_PREFERRED_XY_M,.03)
        self.assertEqual(ARRIVAL_XY_TOLERANCE_M,.10)
        self.assertEqual(A_XY_RMS_LIMIT_M,.05)
        for distance in (.03,.0518,.099):
            with self.subTest(distance=distance):
                xy,yaw=validate_arrival((1.+distance,2.,math.radians(-176.)),goal)
                self.assertAlmostEqual(xy,distance)
                self.assertAlmostEqual(math.degrees(yaw),5.)
        self.assertAlmostEqual(math.degrees(ARRIVAL_YAW_TOLERANCE_RAD),5.)
        for pose in ((1.10,2.,goal[2]),(1.101,2.,goal[2]),
                     (1.,2.,goal[2]+math.radians(5.1)),
                     (float('nan'),2.,goal[2])):
            with self.subTest(pose=pose),self.assertRaisesRegex(RuntimeError,'A arrival limits'):
                validate_arrival(pose,goal)

    def test_reported_three_point_one_cm_and_point_seven_four_deg_is_accepted(self):
        reference=(0.,0.,0.)
        for distance in (.031,.049):
            with self.subTest(distance=distance):
                covariance=[0.]*36
                covariance[0]=covariance[7]=(distance/math.sqrt(2.))**2
                covariance[35]=math.radians(.74)**2
                particles=dict(xy_rms_m=distance,yaw_std_deg=.74,
                               unique_pose_bins=100,largest_pose_bin_mass=.02)
                recent=[reference,(distance,0.,0.),(0.,0.,math.radians(.74))]
                quality=validate_a_localization(covariance,particles,recent,reference)
                self.assertAlmostEqual(quality['amcl_xy_rms_m'],distance)
                self.assertAlmostEqual(quality['amcl_yaw_std_deg'],.74)
                validate_arrival((distance,0.,math.radians(.74)),reference)

    def test_all_position_checks_reject_five_cm_boundary_and_above(self):
        reference=(0.,0.,0.)
        covariance=[0.]*36
        covariance[0]=covariance[7]=(.03/math.sqrt(2.))**2
        covariance[35]=math.radians(2.)**2
        particles=dict(xy_rms_m=.03,yaw_std_deg=2.,
                       unique_pose_bins=100,largest_pose_bin_mass=.02)
        recent=[reference]*3
        for distance in (.05,.051):
            large_covariance=list(covariance)
            large_covariance[0]=large_covariance[7]=(distance/math.sqrt(2.))**2
            cases=[('radial covariance',large_covariance,particles,recent),
                   ('particle spread',covariance,dict(particles,xy_rms_m=distance),recent),
                   ('recent pose drift',covariance,particles,[reference,reference,(distance,0.,0.)])]
            for name,cov,cloud,poses in cases:
                with self.subTest(distance=distance,check=name),self.assertRaises(ValueError):
                    validate_a_localization(cov,cloud,poses,reference)

    def test_teaching_a_requires_tight_localization_in_all_three_independent_checks(self):
        reference=(1.,2.,0.)
        covariance=[0.]*36
        covariance[0]=covariance[7]=(.03/math.sqrt(2.))**2
        covariance[35]=math.radians(2.)**2
        particles=dict(xy_rms_m=.03,yaw_std_deg=2.,
                       unique_pose_bins=100,largest_pose_bin_mass=.02)
        recent=[reference,(1.03,2.,0.),(1.,2.,math.radians(2.))]
        validate_a_localization(covariance,particles,recent,reference)

        cases=[]
        # Neither axis alone exceeds 5 cm, but the Euclidean XY RMS does.
        too_uncertain_xy=list(covariance)
        too_uncertain_xy[0]=too_uncertain_xy[7]=.04**2
        cases.append(('AMCL combined XY covariance',too_uncertain_xy,particles,recent))
        too_uncertain_yaw=list(covariance);too_uncertain_yaw[35]=math.radians(2.1)**2
        cases.append(('AMCL yaw covariance',too_uncertain_yaw,particles,recent))
        cases.append(('particle radial XY spread',covariance,
                      dict(particles,xy_rms_m=.051,xy_std_m=.04),recent))
        cases.append(('particle yaw spread',covariance,dict(particles,yaw_std_deg=2.1),recent))
        cases.append(('too few distinct particle pose bins',covariance,
                      dict(particles,unique_pose_bins=9),recent))
        cases.append(('one particle pose bin dominates',covariance,
                      dict(particles,largest_pose_bin_mass=.51),recent))
        cases.append(('too few observations',covariance,particles,recent[:2]))
        cases.append(('recent XY drift',covariance,particles,
                      [reference,reference,(1.051,2.,0.)]))
        cases.append(('recent yaw drift',covariance,particles,
                      [reference,reference,(1.,2.,math.radians(2.1))]))
        for name,cov,cloud,poses in cases:
            with self.subTest(name=name),self.assertRaises(ValueError):
                validate_a_localization(cov,cloud,poses,reference)

        # Accept the specified diversity boundary independently of spread.
        validate_a_localization(covariance,
            dict(particles,unique_pose_bins=10,largest_pose_bin_mass=.5),recent,reference)

    def test_teaching_a_rejects_collapsed_cloud_even_with_2000_uniform_particles(self):
        reference=(1.,2.,.3)
        covariance=[0.]*36
        covariance[0]=covariance[7]=1e-8
        covariance[35]=math.radians(.01)**2
        particles=[]
        for index in range(2000):
            noise=((index % 1272)-635)*1e-10
            particles.append((1.+noise,2.+noise/2.,.3+noise/100.,1.))
        metrics=particle_cloud_metrics(particles,reference)
        self.assertLess(metrics['xy_rms_m'],.001)
        self.assertEqual(metrics['unique_pose_bins'],1)
        with self.assertRaises(ValueError):
            validate_a_localization(covariance,metrics,[reference]*3,reference)

    def test_runtime_a_acceptance_allows_concentrated_particles_without_relaxing_pose_limits(self):
        reference=(1.,2.,.3)
        covariance=[0.]*36
        covariance[0]=covariance[7]=(.007/math.sqrt(2.))**2
        covariance[35]=math.radians(.1)**2
        metrics=dict(converged=True,xy_rms_m=.007,yaw_std_deg=.1,
                     unique_pose_bins=5,largest_pose_bin_mass=.95)
        quality=validate_a_localization(covariance,metrics,[reference]*3,reference,teaching=False)
        self.assertAlmostEqual(quality['particle_xy_rms_m'],.007)
        # New-A teaching is a separate operation; this fix never rewrites A.
        with self.assertRaises(ValueError):
            validate_a_localization(covariance,metrics,[reference]*3,reference)
        for changed in (dict(metrics,xy_rms_m=.05),dict(metrics,yaw_std_deg=2.01),
                        dict(metrics,converged=False)):
            with self.subTest(metrics=changed),self.assertRaises(ValueError):
                validate_a_localization(covariance,changed,[reference]*3,reference,teaching=False)

    def test_nav2_reuses_AMCL_hardware_and_preserves_collision_command_chain(self):
        params=launch.navigation_parameters()
        bt=params['bt_navigator']['ros__parameters']
        self.assertNotIn('plugin_lib_names',bt)
        self.assertTrue(Path(bt['default_nav_to_pose_bt_xml']).is_file())
        smoother=params['velocity_smoother']['ros__parameters']
        self.assertEqual(smoother['max_velocity'],[.18,0.,.30])
        self.assertEqual(smoother['min_velocity'][0],-.10)
        hardware=yaml.safe_load((ROOT/'ros2_ws/config/ros2_controllers_real.yaml').read_text())
        drive=hardware['base_controller']['ros__parameters']
        self.assertEqual(smoother['max_decel'][0],drive['linear.x.max_deceleration'])
        self.assertEqual(smoother['max_decel'][1],0.)
        self.assertEqual(smoother['max_decel'][2],drive['angular.z.max_deceleration'])
        monitor=params['collision_monitor']['ros__parameters']
        self.assertEqual(monitor['cmd_vel_out_topic'],'/cmd_vel_safe')
        self.assertEqual(monitor['cmd_vel_in_topic'],'/cmd_vel_smoothed')
        self.assertEqual(monitor['observation_sources'],['scan'])
        self.assertTrue(monitor['VelocityStop']['enabled'])
        self.assertTrue(monitor['FootprintApproach']['enabled'])
        control=params['controller_server']['ros__parameters']
        self.assertEqual(control['FollowPath']['desired_linear_vel'],.18)
        self.assertGreaterEqual(control['FollowPath']['min_approach_linear_velocity'],.10)
        self.assertGreaterEqual(smoother['max_velocity'][2],.22)
        self.assertEqual(control['general_goal_checker']['xy_goal_tolerance'],.08)
        self.assertLess(control['general_goal_checker']['xy_goal_tolerance'],ARRIVAL_XY_TOLERANCE_M)
        self.assertFalse(control['general_goal_checker']['stateful'])
        self.assertGreater(control['general_goal_checker']['yaw_goal_tolerance'],0.)
        self.assertAlmostEqual(control['general_goal_checker']['yaw_goal_tolerance'],math.radians(4.))
        self.assertLess(control['general_goal_checker']['yaw_goal_tolerance'],ARRIVAL_YAW_TOLERANCE_RAD)
        self.assertEqual(control['general_goal_checker']['plugin'],'nav2_controller::StoppedGoalChecker')
        self.assertLessEqual(control['general_goal_checker']['rot_stopped_velocity'],.02)
        self.assertLessEqual(control['general_goal_checker']['trans_stopped_velocity'],.015)
        self.assertTrue(control['FollowPath']['predictive_terminal_braking'])
        self.assertTrue(control['FollowPath']['allow_short_rear_goal_correction'])
        self.assertFalse(control['FollowPath']['allow_reverse_route_tracking'])
        self.assertLessEqual(monitor['VelocityStop']['translation_reverse']['linear_min'],
                             smoother['min_velocity'][0])
        planner=params['planner_server']['ros__parameters']
        for name in ('GridShortest','GridBased'):
            self.assertEqual(planner[name]['plugin'],'tai_robot_one::ExactGoalSmac2D')
            self.assertEqual(planner[name]['tolerance'],0.)
            self.assertFalse(planner[name]['use_final_approach_orientation'])
        nodes=launch.navigation_actions(None)
        packages=[getattr(node,'node_package',None) for node in nodes]
        self.assertNotIn('nav2_amcl',packages)
        self.assertNotIn('nav2_map_server',packages)
        self.assertNotIn('controller_manager',packages)
        self.assertIn('nav2_collision_monitor',packages)
        with patch.object(launch,'Node',side_effect=lambda **kwargs:kwargs):
            definitions=launch.navigation_actions(None)
        smoother=next(node for node in definitions if isinstance(node,dict)
                      and node['package']=='nav2_velocity_smoother')
        self.assertIn(('cmd_vel','cmd_vel_nav'),smoother['remappings'])
        self.assertTrue(all(isinstance(node,dict) for node in definitions))

    def test_serialized_parameters_allow_jazzy_bt_navigator_to_start(self):
        # Construct only the unconfigured BT process in an isolated domain.
        # This exercises ROS YAML type parsing, which rejected the old []
        # plugin_lib_names before any lifecycle service could be created.
        binary=Path(get_package_prefix('nav2_bt_navigator'))/'lib/nav2_bt_navigator/bt_navigator'
        if not binary.is_file():
            self.skipTest('Nav2 bt_navigator executable is not installed')
        with tempfile.TemporaryDirectory() as tmp:
            params=Path(tmp)/'r3_navigation.yaml'
            params.write_text(yaml.safe_dump(launch.navigation_parameters(),sort_keys=False))
            env=dict(os.environ,ROS_DOMAIN_ID='227',RMW_IMPLEMENTATION='rmw_fastrtps_cpp',
                     ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',ROS_LOG_DIR=str(Path(tmp)/'log'))
            env.pop('ROS_LOCALHOST_ONLY',None)
            process=subprocess.Popen([str(binary),'--ros-args','--params-file',str(params)],
                env=env,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
            output=''
            try:
                try:
                    output,_=process.communicate(timeout=2.)
                    self.fail('bt_navigator exited during construction:\n'+output)
                except subprocess.TimeoutExpired:
                    process.terminate()
                    output,_=process.communicate(timeout=5.)
                self.assertNotIn('parameter_value_from failed',output)
                self.assertIn('Waiting on external lifecycle transitions to activate',output)
            finally:
                if process.poll() is None:
                    process.kill()
                    process.communicate(timeout=3.)


if __name__=='__main__':
    unittest.main()
