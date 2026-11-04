"""Cycle ordering/failure boundaries and Linux parent-death handling; no drivers."""
import os
import math
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import unittest
from unittest.mock import patch
from types import SimpleNamespace

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
import run_pallet_cycle as cycle


class Fixture:
    run=cycle.Cycle.run

    def __init__(self,root,failure=None):
        self.root=root;self.failure=failure;self.events=[];self.lease=False

    def event(self,name):
        self.events.append(name)
        if name==self.failure:raise RuntimeError('Injected failure: '+name)

    def ensure_navigation(self):self.event('navigation')
    def verify_localization(self):self.event('localization')
    def auto_localize_map(self):self.event('auto-localization')
    def stop_navigation(self):self.event('nav-off')
    def start(self,name,cmd):self.event('start-'+name)
    def stop_child(self,name):self.event('stop-'+name)
    def operator(self,helper,op,*args,**kwargs):self.event(helper+'-'+op)
    def acquire(self):self.event('acquire')
    def align(self):self.event('align')
    def wait_recognition_station(self):self.event('check-recognition')
    def command_owner(self,name):return True
    def get_publishers_info_by_topic(self,topic):return []
    def wait(self,predicate,*args):
        if not predicate():raise RuntimeError('Checkpoint failed')
    def fresh(self,*args):return dict(state='COMPLETE',active=False)
    def sensors(self,empty=False):return dict(position=0. if 'delivery-lower' in self.events else .08)
    def still(self):return True


class CycleTest(unittest.TestCase):
    def test_disproved_cross_pose_calibration_cannot_authorize_motion(self):
        with tempfile.TemporaryDirectory() as temp:
            p=Path(temp)/'calibration.yaml'
            p.write_text('motion_qualified: false\n')
            with self.assertRaisesRegex(RuntimeError,'Hiệu chuẩn ngang chưa đạt'):
                cycle.require_motion_calibration(p)
            p.write_text('motion_qualified: true\n')
            cycle.require_motion_calibration(p)

    def test_checkpoint_angle_failure_returns_to_recognition_and_restarts(self):
        with tempfile.TemporaryDirectory() as temp:
            f,ctx=self.fixture(temp);f.revalidate_recognition=True
            f.chassis_front_m=.3005;f.fork_root_m=.3655
            f.get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=100_000_000_000))
            orig=f.operator;tries=[0]
            def operator(helper,op,*args,**kwargs):
                if (helper,op)==('pickup','insert-and-lift') and tries[0]==0:
                    tries[0]+=1;f.events.append('pickup-angle-blocked')
                    raise cycle.ReturnToRecognition('Góc pallet đổi quá nhiều ở checkpoint; cần căn lại từ ngoài')
                orig(helper,op,*args,**kwargs)
            f.operator=operator
            with ctx,patch.object(cycle,'recognition_measurement',return_value={'fork_root_plane_gap_m':1.09}):
                f.run(pickup_only=True)
            blocked=f.events.index('pickup-angle-blocked')
            self.assertEqual(f.events[blocked+1:blocked+6],['stop-staging','stop-perception',
                'localization','navigation','delivery-return'])
            self.assertEqual(f.events.count('delivery-return'),2)
            self.assertEqual(f.events.count('acquire'),2)
            self.assertEqual(f.events.count('align'),2)
            self.assertEqual(f.events.count('pickup-insert-and-lift'),1)

    def test_unconfirmed_outside_does_not_start_return_navigation(self):
        with tempfile.TemporaryDirectory() as temp:
            f,ctx=self.fixture(temp);f.chassis_front_m=.3005;f.fork_root_m=.3655
            f.get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=100_000_000_000))
            f.operator=lambda helper,op,*args,**kwargs: (_ for _ in ()).throw(cycle.ReturnToRecognition('angle'))
            with ctx,patch.object(cycle,'recognition_measurement',return_value={'fork_root_plane_gap_m':.48}),self.assertRaisesRegex(RuntimeError,'Checkpoint failed'):
                f.run(from_current_view=True,pickup_only=True)
            self.assertNotIn('navigation',f.events)

    def test_staging_restart_retries_at_same_location_without_nav_goal(self):
        with tempfile.TemporaryDirectory() as temp:
            f,ctx=self.fixture(temp);calls=[0];f.pump=lambda seconds:None
            def align():
                calls[0]+=1;f.events.append('align')
                if calls[0]==1:raise cycle.RestartStaging('Nav2 chưa ACTIVE: controller_server')
            f.align=align
            with ctx:f.run(from_current_view=True,pickup_only=True)
            self.assertEqual(f.events.count('start-staging'),2)
            self.assertEqual(f.events.count('acquire'),2)
            self.assertNotIn('navigation',f.events)
            self.assertNotIn('delivery-return',f.events)
            self.assertIn('pickup-insert-and-lift',f.events)

    def measurement_fixture(self):
        target=SimpleNamespace(valid=True,status='TRACKING target_bias_applied',
            header=SimpleNamespace(stamp=SimpleNamespace(sec=100,nanosec=0),frame_id='camera_color_optical_frame'),
            camera_forward_m=1.5,chassis_front_forward_m=1.4555,opening_lateral_m=.0723,
            entry_heading_rad=math.radians(5.04),face_yaw_rad=-math.radians(5.04),
            rgb_depth_delta_ms=5.,opening_width_m=.255,leg_center_separation_m=.30,
            left_depth_samples=100,right_depth_samples=100)
        state=dict(session=3,started_ros_s=99.,phase='TRACKING',locked=True,enabled=False,
            busy=False,measurement_ready=True,rgbd_health=dict(stable=True))
        return target,state

    def test_recognition_report_uses_fork_root_and_signed_robot_axes(self):
        target,state=self.measurement_fixture()
        result=cycle.recognition_measurement(target,state,100.1,.3005,.3655)
        self.assertAlmostEqual(result['base_forward_m'],1.756)
        self.assertAlmostEqual(result['fork_root_forward_gap_m'],1.3905)
        self.assertAlmostEqual(result['lateral_left_m'],.0723)
        self.assertAlmostEqual(result['entry_heading_deg'],5.04)
        text=cycle.recognition_result_text(result)
        self.assertIn('7.23 cm trái',text);self.assertIn('+5.04° (trái)',text)
        target.opening_lateral_m=-.0723;target.entry_heading_rad=-math.radians(5.04)
        target.face_yaw_rad=-target.entry_heading_rad
        text=cycle.recognition_result_text(cycle.recognition_measurement(target,state,100.1,.3005,.3655))
        self.assertIn('7.23 cm phải',text);self.assertIn('-5.04° (phải)',text)

    def test_recognition_report_cannot_reuse_stale_invalid_or_unbiased_measurement(self):
        for change in ('stale','old_session','invalid','unbiased','busy','none'):
            target,state=self.measurement_fixture();now=100.1
            if change=='stale':now=101.
            if change=='old_session':state['started_ros_s']=100.05
            if change=='invalid':target.valid=False
            if change=='unbiased':target.status='TRACKING'
            if change=='busy':state['busy']=True
            if change=='none':target=None
            self.assertIsNone(cycle.recognition_measurement(target,state,now,.3005,.3655),change)

    def test_tip_origin_translation_preserves_lateral_measured_along_vehicle_axis(self):
        target,state=self.measurement_fixture()
        for angle in (0.,math.radians(6.07),math.radians(-6.07)):
            target.entry_heading_rad=angle;target.face_yaw_rad=-angle
            result=cycle.recognition_measurement(target,state,100.1,.3005,.3655,.390)
            self.assertAlmostEqual(result['fork_tip_lateral_left_m'],target.opening_lateral_m)
            self.assertAlmostEqual(result['fork_tip_forward_gap_m'],result['base_forward_m']-.7555)
            self.assertAlmostEqual(result['fork_tip_plane_gap_m'],
                result['fork_root_plane_gap_m']-.390*math.cos(angle))
            self.assertIn('Từ giữa hai mũi càng',cycle.recognition_result_text(result))

    def test_yaw_correction_alone_cannot_start_insertion(self):
        self.assertIn('YAW_CORRECTED',cycle.FAIL_ALIGNMENT)
        states={name:(value,100.) for name,value in dict(
            handling=dict(state='IDLE',active=False),
            alignment=dict(state='YAW_CORRECTED',active=False,preparing=False),
            recognition=dict(phase='TRACKING',locked=True,enabled=False,busy=False,
                             measurement_ready=True,rgbd_health=dict(stable=True))).items()}
        with self.assertRaisesRegex(RuntimeError,'ALIGNED'):
            cycle.insertion_readiness(states,100.)

    def test_pickup_only_finishes_without_transport_or_lowering(self):
        with tempfile.TemporaryDirectory() as temp:
            f,ctx=self.fixture(temp)
            with ctx:f.run(from_recognition=True,pickup_only=True)
        self.assertIn('check-recognition',f.events)
        self.assertIn('acquire',f.events)
        self.assertIn('align',f.events)
        self.assertIn('pickup-insert-and-lift',f.events)
        self.assertEqual(f.events[-2:],['stop-staging','stop-perception'])
        self.assertNotIn('navigation',f.events)
        self.assertFalse(any(e.startswith('delivery-') for e in f.events))
        self.assertFalse(f.lease)

    def test_pickup_uses_visual_geometry_without_saved_pickup_goal(self):
        self.assertEqual(cycle.required_goal_names(True,True),())
        self.assertEqual(cycle.required_goal_names(True,False),('recognition',))
        self.assertEqual(cycle.required_goal_names(False,True),('recognition','dropoff'))

    def test_local_pickup_needs_no_map_localization_or_navigation_restart(self):
        with tempfile.TemporaryDirectory() as temp:
            f,ctx=self.fixture(temp)
            with ctx:f.run(from_current_view=True,pickup_only=True)
            self.assertNotIn('localization',f.events)
            self.assertNotIn('navigation',f.events)
            self.assertNotIn('check-recognition',f.events)
            self.assertIn('pickup-insert-and-lift',f.events)

    def test_requested_auto_localization_precedes_every_pallet_motion(self):
        with tempfile.TemporaryDirectory() as temp:
            f,ctx=self.fixture(temp);f.auto_localize_requested=True
            with ctx:f.run(from_current_view=True,pickup_only=True)
            self.assertEqual(f.events[0],'auto-localization')
            self.assertLess(f.events.index('auto-localization'),f.events.index('start-staging'))
            self.assertLess(f.events.index('auto-localization'),f.events.index('acquire'))
        with tempfile.TemporaryDirectory() as temp:
            f,ctx=self.fixture(temp,'auto-localization');f.auto_localize_requested=True
            with ctx,self.assertRaises(RuntimeError):f.run(from_current_view=True,pickup_only=True)
            self.assertNotIn('start-staging',f.events)
            self.assertNotIn('acquire',f.events)
            self.assertNotIn('pickup-insert-and-lift',f.events)

    def test_full_pickup_localizes_then_returns_to_recognition_before_perception(self):
        with tempfile.TemporaryDirectory() as temp:
            f,ctx=self.fixture(temp);f.auto_localize_requested=True
            f.revalidate_recognition=True;calls=[]
            def operator(helper,op,*args,**kwargs):
                calls.append((helper,op,kwargs));f.event(helper+'-'+op)
            f.operator=operator
            with ctx:f.run(pickup_only=True)
            self.assertEqual(f.events[:5],['auto-localization','navigation',
                             'delivery-return','localization','nav-off'])
            self.assertEqual(calls[0],('delivery','return',
                             {'extra_args':('--revalidate-recognition',)}))
            self.assertLess(f.events.index('nav-off'),f.events.index('acquire'))
            self.assertLess(f.events.index('acquire'),f.events.index('align'))
            self.assertLess(f.events.index('align'),f.events.index('pickup-insert-and-lift'))
            self.assertEqual(f.events.count('navigation'),1)
            self.assertFalse(f.awaiting_global_localization)
        for failed in ('auto-localization','delivery-return','localization'):
            with tempfile.TemporaryDirectory() as temp:
                f,ctx=self.fixture(temp,failed);f.auto_localize_requested=True
                with ctx,self.assertRaises(RuntimeError):f.run(pickup_only=True)
                self.assertNotIn('start-staging',f.events)
                self.assertNotIn('acquire',f.events)
                self.assertNotIn('pickup-insert-and-lift',f.events)
                self.assertFalse(f.awaiting_global_localization)

    def test_map_tf_is_required_after_stationary_auto_localization_bootstrap(self):
        from unittest.mock import Mock
        wall=cycle.time.monotonic();stamp=SimpleNamespace(sec=100,nanosec=0)
        msg=SimpleNamespace(header=SimpleNamespace(stamp=stamp))
        lift=dict(homed=True,serial_connected=True,fault='NONE',state='IDLE',position=0.,velocity=0.)
        f=SimpleNamespace(states={'odom':(msg,wall),'laser':(msg,wall),'lift':(lift,wall)},
            get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=100_000_000_000)),
            local_pickup=False,awaiting_global_localization=True,
            buffer=SimpleNamespace(lookup_transform=Mock(side_effect=RuntimeError('no map TF'))))
        self.assertEqual(cycle.Cycle.sensors(f,empty=True),lift)
        f.buffer.lookup_transform.assert_not_called()
        f.awaiting_global_localization=False
        with self.assertRaisesRegex(RuntimeError,'no map TF'):cycle.Cycle.sensors(f,empty=True)

    def test_alignment_trial_exits_before_checkpoint_insertion_and_lift(self):
        with tempfile.TemporaryDirectory() as temp:
            f,ctx=self.fixture(temp)
            with ctx:f.run(from_current_view=True,alignment_only=True)
            self.assertIn('align',f.events)
            self.assertNotIn('pickup-insert-and-lift',f.events)
            self.assertNotIn('localization',f.events)
            self.assertNotIn('navigation',f.events)
            self.assertFalse(any(e.startswith('delivery-') for e in f.events))

    def test_pickup_only_never_continues_after_alignment_or_insertion_failure(self):
        for failure,forbidden in [('acquire','align'),('align','pickup-insert-and-lift'),
                                  ('pickup-insert-and-lift','stop-staging')]:
            with tempfile.TemporaryDirectory() as temp:
                f,ctx=self.fixture(temp,failure)
                with ctx,self.assertRaises(RuntimeError):f.run(from_recognition=True,pickup_only=True)
                self.assertNotIn(forbidden,f.events)
                self.assertFalse(any(e.startswith('delivery-') for e in f.events))

    def acquisition_fixture(self, failures=1, active=False, busy=False):
        events=[]
        def once(watch):
            events.append('request')
            if events.count('request') <= failures:
                raise cycle.AcquisitionWindowExpired('explicit session expiration')
        def fresh(key):
            return {'recognition':dict(phase='ACQUIRE_FAILED',enabled=False,busy=busy),
                    'alignment':dict(active=active,preparing=False,nav_snapshot=False),
                    'handling':dict(active=False)}[key]
        f=SimpleNamespace(acquire_once=once,fresh=fresh,still=lambda:True,
            wait=lambda predicate,*a:predicate(),sensors=lambda **kw:events.append(('sensors',kw)))
        return f,events

    def test_expired_acknowledged_session_retries_only_at_rest_with_empty_fork(self):
        f,events=self.acquisition_fixture()
        cycle.Cycle.acquire(f,watch=())
        self.assertEqual(events,['request',('sensors',{'empty':True}),'request'])

    def test_checkpoint_expired_detection_can_retry_only_for_current_stationary_run(self):
        for match in (True,False):
            f,events=self.acquisition_fixture();original=f.fresh;f.pickup_run_id='current'
            def fresh(key):
                if key=='handling':return dict(active=True,phase='CHECKPOINT_RECOGNITION',
                    run_id='current' if match else 'old')
                if key=='alignment':return dict(active=True,preparing=False,perception_refresh_allowed=True)
                return original(key)
            f.fresh=fresh
            if match:
                cycle.Cycle.acquire(f,watch=())
                self.assertEqual(events.count('request'),2)
            else:
                with self.assertRaises(cycle.AcquisitionWindowExpired):cycle.Cycle.acquire(f,watch=())
                self.assertEqual(events.count('request'),1)

    def test_expired_session_retries_until_success_and_never_resume_active_or_busy_control(self):
        f,events=self.acquisition_fixture(failures=4)
        cycle.Cycle.acquire(f,watch=())
        self.assertEqual(events.count('request'),5)
        for active,busy in ((True,False),(False,True)):
            f,events=self.acquisition_fixture(active=active,busy=busy)
            with self.assertRaises(cycle.AcquisitionWindowExpired):cycle.Cycle.acquire(f,watch=())
            self.assertEqual(events,['request'])

    def test_unacknowledged_or_other_acquisition_errors_never_retry(self):
        f,events=self.acquisition_fixture()
        def fail(watch):
            events.append('request');raise RuntimeError('no acknowledgement')
        f.acquire_once=fail
        with self.assertRaisesRegex(RuntimeError,'no acknowledgement'):cycle.Cycle.acquire(f,watch=())
        self.assertEqual(events,['request'])

    def arrival_state(self,**changes):
        return dict(state='REACQUIRE_REQUIRED',strategy='nearest_normal_nav2',
                    stationary_arrival_confirmed=True,nav_snapshot=False,
                    active=False,preparing=False,fork_plane_distance_m=1.326,**changes)

    def test_reacquisition_only_allowed_for_confirmed_safe_stationary_arrival(self):
        state=self.arrival_state()
        self.assertTrue(cycle.can_reacquire_after_arrival(state))
        for key,value in (('state','BLOCKED'),('state','TIMEOUT'),('active',True),
                          ('stationary_arrival_confirmed',False),('nav_snapshot',True),
                          ('preparing',True),('fork_plane_distance_m',1.19),
                          ('fork_plane_distance_m',float('nan'))):
            self.assertFalse(cycle.can_reacquire_after_arrival({**state,key:value}))

    def test_yaw_only_stationary_start_can_reacquire_but_never_skip_new_stillness_checks(self):
        state={**self.arrival_state(),'stationary_arrival_confirmed':False,
               'stationary_trim_start_confirmed':True}
        self.assertTrue(cycle.can_reacquire_after_arrival(state))
        self.assertFalse(cycle.can_reacquire_after_arrival({**state,'nav_snapshot':True}))
        self.assertFalse(cycle.can_reacquire_after_arrival({**state,'state':'BLOCKED'}))
        n,events=self.alignment_fixture([state,dict(state='ALIGNED',active=False,preparing=False)])
        cycle.Cycle.align(n)
        self.assertEqual(events,['align','sensors','acquire','align'])

    def alignment_fixture(self,outcomes):
        events=[];current=[dict(ready=True)];pending=iter(outcomes)
        rec=dict(phase='TRACKING',locked=True,enabled=False,busy=False,
                 measurement_ready=True,rgbd_health=dict(stable=True))
        n=SimpleNamespace(sensor_windows={},
            get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=1000000000)),
            fresh=lambda key,*args:current[0] if key=='alignment' else rec,
            still=lambda:True,sensors=lambda **kw:events.append('sensors'))
        def wait(predicate,*args):
            if not predicate():raise RuntimeError('Checkpoint failed')
        def call(*args):
            events.append('align');current[0]=next(pending)
        def acquire():
            events.append('acquire');current[0]=dict(ready=True)
        n.wait=wait;n.call=call;n.acquire=acquire
        return n,events

    def test_alignment_window_timeout_waits_again_without_sending_motion(self):
        n,events=self.alignment_fixture([dict(state='ALIGNED',active=False,preparing=False)])
        wait=n.wait;timeouts=[0]
        def delayed(predicate,timeout,label,*args):
            if label=='laser/odom/RGB-D và bộ căn ổn định' and timeouts[0]<2:
                timeouts[0]+=1
                self.assertNotIn('align',events)
                raise RuntimeError('Quá thời gian: '+label)
            return wait(predicate,timeout,label,*args)
        n.wait=delayed
        cycle.Cycle.align(n)
        self.assertEqual(timeouts[0],2)
        self.assertEqual(events,['align'])

    def test_inactive_staging_nav_is_restarted_only_when_confirmed_idle(self):
        n,events=self.alignment_fixture([])
        def fresh(key,*args):
            if key=='alignment':return dict(active=False,preparing=False,nav_snapshot=False,
                ready_reason='Nav2 chưa ACTIVE: controller_server')
            return dict(active=False,phase='TRACKING')
        n.fresh=fresh
        def wait(predicate,timeout,label,*args):
            if label=='laser/odom/RGB-D và bộ căn ổn định':raise RuntimeError('Quá thời gian: '+label)
            self.assertTrue(predicate())
        n.wait=wait
        with self.assertRaises(cycle.RestartStaging):cycle.Cycle.align(n)
        self.assertEqual(events,['sensors'])

    def test_arrival_reacquires_once_and_requires_new_alignment_before_pickup(self):
        n,events=self.alignment_fixture([self.arrival_state(),
            dict(state='ALIGNED',active=False,preparing=False)])
        cycle.Cycle.align(n)
        self.assertEqual(events,['align','sensors','acquire','align'])

    def test_other_failures_stop_and_repeated_safe_visual_loss_retries(self):
        bad={**self.arrival_state(),'state':'BLOCKED'}
        n,events=self.alignment_fixture([bad])
        with self.assertRaisesRegex(RuntimeError,'Căn dừng'):cycle.Cycle.align(n)
        self.assertEqual(events,['align'])
        n,events=self.alignment_fixture([self.arrival_state(),self.arrival_state(),self.arrival_state(),
            dict(state='ALIGNED',active=False,preparing=False)])
        cycle.Cycle.align(n)
        self.assertEqual(events,['align','sensors','acquire','align','sensors','acquire','align',
                                 'sensors','acquire','align'])

    def test_two_completed_arrivals_can_reacquire_then_finish_at_one_cm_one_degree(self):
        n,events=self.alignment_fixture([self.arrival_state(),self.arrival_state(),
            dict(state='ALIGNED',active=False,preparing=False)])
        cycle.Cycle.align(n)
        self.assertEqual(events,['align','sensors','acquire','align','sensors','acquire','align'])

    def test_reacquisition_failure_never_sends_second_alignment_request(self):
        n,events=self.alignment_fixture([self.arrival_state()])
        def fail():raise RuntimeError('Nhận diện thất bại')
        n.acquire=fail
        with self.assertRaisesRegex(RuntimeError,'Nhận diện thất bại'):cycle.Cycle.align(n)
        self.assertEqual(events,['align','sensors'])

    def test_reacquisition_needs_actual_stillness_and_empty_fork_sensor_checks(self):
        for failure in ('motion','sensors'):
            n,events=self.alignment_fixture([self.arrival_state()])
            if failure=='motion':
                n.still=lambda:False
            else:
                def invalid(**kwargs):raise RuntimeError('càng/cảm biến không đạt')
                n.sensors=invalid
            with self.assertRaises(RuntimeError):cycle.Cycle.align(n)
            self.assertEqual(events,['align'])

    def test_camera_floor_needs_fresh_qualified_status_before_cycle_start(self):
        f=SimpleNamespace(states={'floor':({'qualified':True},time.monotonic())})
        self.assertEqual(cycle.Cycle.camera_floor_issue(f),'')
        for states in ({},{'floor':({'qualified':True},time.monotonic()-2.)},
                       {'floor':({'qualified':False,'reason':'sàn chưa đủ rộng'},time.monotonic())}):
            f.states=states
            self.assertTrue(cycle.Cycle.camera_floor_issue(f))

    def test_recognition_requalifies_after_callback_gap_without_skipping_checks(self):
        attempts=[]
        def station():
            attempts.append(True)
            if len(attempts)<3:
                raise RuntimeError('Chưa đủ mẫu odometry mới')
        def wait(predicate,timeout,label):
            self.assertEqual(timeout,5.)
            for _ in range(3):
                if predicate():return
            raise RuntimeError('timeout')
        f=SimpleNamespace(recognition_station=station,wait=wait)
        cycle.Cycle.wait_recognition_station(f)
        self.assertEqual(len(attempts),3)

    def test_station_timeout_reports_actual_failed_condition(self):
        def station():raise RuntimeError('Ngoài vùng: 31.2 cm, 22.1°')
        def wait(predicate,*args):
            self.assertFalse(predicate())
            raise RuntimeError('timeout')
        f=SimpleNamespace(recognition_station=station,wait=wait)
        with self.assertRaisesRegex(RuntimeError,'31.2 cm, 22.1°'):
            cycle.Cycle.wait_recognition_station(f)

    def fixture(self,temp,failure=None):
        root=Path(temp)
        return Fixture(root,failure),patch.object(cycle,'ROOT',root)

    def test_success_handoffs_and_lowering_happen_in_required_order(self):
        with tempfile.TemporaryDirectory() as temp:
            f,ctx=self.fixture(temp)
            with ctx:f.run()
        self.assertEqual(f.events,['localization','navigation','delivery-return','localization','nav-off','start-staging',
            'start-perception','acquire','align','pickup-insert-and-lift','stop-staging',
            'stop-perception','navigation','localization','delivery-go','delivery-lower'])
        self.assertFalse(f.lease)

    def test_failure_never_enters_any_later_phase(self):
        for failure,forbidden in [('delivery-return','start-staging'),('acquire','align'),
                ('align','pickup-insert-and-lift'),('pickup-insert-and-lift','delivery-go'),
                ('delivery-go','delivery-lower')]:
            with self.subTest(failure=failure),tempfile.TemporaryDirectory() as temp:
                f,ctx=self.fixture(temp,failure)
                with ctx,self.assertRaisesRegex(RuntimeError,'Injected failure'):f.run()
                self.assertNotIn(forbidden,f.events)

    def test_explicit_station_start_checks_pose_and_reacquires_without_return_navigation(self):
        with tempfile.TemporaryDirectory() as temp:
            f,ctx=self.fixture(temp)
            with ctx:f.run(from_recognition=True)
            self.assertEqual(f.events[:5],['localization','check-recognition','start-staging','start-perception','acquire'])
            self.assertNotIn('delivery-return',f.events)
            self.assertEqual(f.events[-2:],['delivery-go','delivery-lower'])
        with tempfile.TemporaryDirectory() as temp:
            f,ctx=self.fixture(temp,'check-recognition')
            with ctx,self.assertRaisesRegex(RuntimeError,'check-recognition'):f.run(from_recognition=True)
            self.assertNotIn('start-staging',f.events)

    def test_current_view_start_reacquires_without_claiming_recognition_goal_arrival(self):
        with tempfile.TemporaryDirectory() as temp:
            f,ctx=self.fixture(temp)
            with ctx:f.run(from_current_view=True)
            self.assertEqual(f.events[:5],['localization','start-staging','start-perception','acquire','align'])
            self.assertNotIn('delivery-return',f.events)
            self.assertNotIn('check-recognition',f.events)
            self.assertEqual(f.events[-3:],['localization','delivery-go','delivery-lower'])

    def test_recognition_backoff_only_changes_intermediate_return_arguments(self):
        with tempfile.TemporaryDirectory() as temp:
            f,ctx=self.fixture(temp);calls=[]
            original=f.operator
            def operator(helper,op,*a,**kw):
                calls.append((op,kw));original(helper,op,*a,**kw)
            f.operator=operator
            with ctx:f.run(recognition_backoff=.35)
            self.assertEqual(calls[0],('return',{'extra_args':('--recognition-backoff','0.35')}))
            self.assertTrue(all(not kw for op,kw in calls[1:]))

    def test_cycle_bounds_image_work_and_allows_full_alignment_admission_wait(self):
        with tempfile.TemporaryDirectory() as temp:
            f,ctx=self.fixture(temp);commands={}
            def start(name,cmd):commands[name]=cmd;f.event('start-'+name)
            f.start=start
            with ctx:f.run(from_current_view=True,pickup_only=True)
            perception=commands['perception']
            self.assertEqual(perception[perception.index('--track-max-fps')+1],'15')
            self.assertEqual(perception[perception.index('--preview-fps')+1],'2')
        n,events=self.alignment_fixture([dict(state='ALIGNED',active=False,preparing=False)])
        previous=n.call;timeouts=[]
        def call(*args):timeouts.append(args[-1]);previous(*args)
        n.call=call
        cycle.Cycle.align(n)
        self.assertEqual(events,['align'])
        from align_pallet import START_DATA_TIMEOUT_S
        self.assertGreater(timeouts[0],START_DATA_TIMEOUT_S+2.)

    def test_completion_status_and_lift_height_both_gate_transport(self):
        for status,height in [(dict(state='INSERTED',active=False),.08),
                              (dict(state='COMPLETE',active=True),.08),
                              (dict(state='COMPLETE',active=False),.02), (None,.08)]:
            with tempfile.TemporaryDirectory() as temp:
                f,ctx=self.fixture(temp)
                f.fresh=lambda *a:status
                f.sensors=lambda:dict(position=height)
                with ctx,self.assertRaisesRegex(RuntimeError,'Chưa xác nhận'):f.run()
                self.assertNotIn('delivery-go',f.events)

    def test_recognition_needs_fresh_semantic_and_rgbd_gates(self):
        state=dict(phase='TRACKING',locked=True,enabled=False,busy=False,
                   measurement_ready=True,rgbd_health=dict(stable=True))
        self.assertTrue(cycle.recognition_ready(state))
        for key,value in [('phase','LOST'),('locked',False),('enabled',True),('busy',True),
                          ('measurement_ready',False),('rgbd_health',{'stable':False})]:
            self.assertFalse(cycle.recognition_ready({**state,key:value}))

    def test_acquisition_ack_rejects_old_retained_sessions_and_timestamps(self):
        state=dict(session=4,started_ros_s=100.1,phase='ACQUIRING')
        self.assertTrue(cycle.new_recognition_session(state,3,100.))
        for changes in (dict(session=3,phase='TRACKING'),dict(session=5),
                        dict(started_ros_s=99.9),dict(started_ros_s=float('nan')),
                        dict(session=True),dict(started_ros_s=None),dict(phase='PAUSED')):
            self.assertFalse(cycle.new_recognition_session({**state,**changes},3,100.))

    def test_no_alignment_request_before_all_sensors_and_server_qualify(self):
        ready_rec=dict(phase='TRACKING',locked=True,enabled=False,busy=False,
                      measurement_ready=True,rgbd_health=dict(stable=True))
        for blocked in ('laser','odom','server','rgbd'):
            with self.subTest(blocked=blocked):
                n=SimpleNamespace()
                n.sensor_windows={key:SimpleNamespace(ready=lambda *args,k=key:k!=blocked)
                                  for key in ('laser','odom')}
                state=dict(ready=blocked!='server')
                rec={**ready_rec,'rgbd_health':{'stable':blocked!='rgbd'}}
                n.fresh=lambda key,*args:state if key=='alignment' else rec
                n.get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=1000000000))
                calls=[];n.call=lambda *args:calls.append(args)
                def wait(predicate,*args):
                    self.assertFalse(predicate())
                    raise RuntimeError('qualification timeout')
                n.wait=wait
                with self.assertRaisesRegex(RuntimeError,'qualification timeout'):cycle.Cycle.align(n)
                self.assertEqual(calls,[])

    def test_process_selection_excludes_hardware_localization_and_other_namespaces(self):
        args={10:['/opt/ros/jazzy/lib/nav2_controller/controller_server','__node:=controller_server'],
              11:['/opt/ros/jazzy/lib/nav2_controller/controller_server','__node:=controller_server','__ns:=/pallet_nav'],
              12:['/opt/ros/jazzy/lib/nav2_amcl/amcl','__node:=amcl'],
              13:['python3','esp32_bridge','__node:=esp32_imu_bridge'],
              14:['/opt/ros/jazzy/lib/nav2_collision_monitor/collision_monitor','__node:=collision_monitor'],
              15:['/opt/ros/jazzy/lib/nav2_lifecycle_manager/lifecycle_manager','__node:=lifecycle_manager_localization']}
        with patch.object(Path,'iterdir',return_value=[Path(str(p)) for p in args]),\
                patch.object(cycle,'process_domain',return_value=os.environ.get('ROS_DOMAIN_ID','0')),\
                patch.object(cycle,'process',side_effect=lambda p:(args[p],1)),\
                patch.object(cycle,'project_navigation_ancestor',side_effect=lambda p:p!=14):
            self.assertEqual([(pid,name) for pid,name,_ in cycle.navigation_processes()],[(10,'controller_server')])

    def test_other_ros_domains_are_never_stopped(self):
        with patch.object(Path,'iterdir',return_value=[Path('10')]),\
                patch.object(cycle,'process_domain',return_value='not-current-domain'),\
                patch.object(cycle,'process') as inspect:
            self.assertEqual(cycle.navigation_processes(),[])
            inspect.assert_not_called()

    def test_child_receives_stop_when_supervisor_is_killed(self):
        with tempfile.TemporaryDirectory() as temp:
            ready=Path(temp)/'ready';stopped=Path(temp)/'stopped'
            code=('import sys,time,signal\nfrom pathlib import Path\n'
                  'from pallet_process_guard import arm_cycle_parent\n'
                  'arm_cycle_parent()\n'
                  'def stop(*args):\n Path(sys.argv[2]).write_text("stopped");sys.exit(0)\n'
                  'signal.signal(signal.SIGINT,stop)\n'
                  'Path(sys.argv[1]).write_text("ready")\ntime.sleep(30)\n')
            parent_code=('import os,subprocess,sys,time\n'
                'env={**os.environ,"PALLET_CYCLE_PARENT_PID":str(os.getpid())}\n'
                'p=subprocess.Popen([sys.executable,"-c",sys.argv[1],*sys.argv[2:]],env=env)\n'
                'print(p.pid,flush=True)\ntime.sleep(30)')
            parent=subprocess.Popen([sys.executable,'-c',parent_code,code,str(ready),str(stopped)],
                stdout=subprocess.PIPE,text=True,env={**os.environ,'PYTHONPATH':str(ROOT/'scripts')})
            child=int(parent.stdout.readline())
            try:
                end=time.monotonic()+3.
                while not ready.exists() and time.monotonic()<end:time.sleep(.02)
                self.assertTrue(ready.exists())
                parent.kill();parent.wait(timeout=2)
                end=time.monotonic()+3.
                while not stopped.exists() and time.monotonic()<end:time.sleep(.02)
                self.assertTrue(stopped.exists(),'child failed to stop after supervisor SIGKILL')
            finally:
                if parent.poll() is None:parent.kill();parent.wait(timeout=2)
                try:os.kill(child,signal.SIGKILL)
                except ProcessLookupError:pass
                parent.stdout.close()


if __name__=='__main__':unittest.main()
