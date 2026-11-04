"""Warm executor handoff, preflight-before-start and per-run heartbeat; no drivers."""
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
import run_pallet_cycle as cycle


class HandoffFixture:
    pickup_insert_and_lift=cycle.Cycle.pickup_insert_and_lift

    def __init__(self,root,preview_error=False,sensors_ready=True,final_state='INSERTED'):
        self.logs=Path(root);self.lease=True;self.pickup_run_id=None
        self.clock=100.;self.events=[];self.heartbeats=[];self.states={}
        self.pickup_heartbeat=NS(publish=lambda m:self.heartbeats.append(m.data))
        self.sensor_windows={'odom':NS(ready=lambda *a:sensors_ready)}
        self.preview_error=preview_error;self.final_state=final_state;self.started=False

    def refresh(self):
        self.states={
            'alignment':(dict(state='ALIGNED',active=False,preparing=False),self.clock),
            'handling':(dict(state='IDLE',active=False),self.clock),
            'recognition':(dict(phase='TRACKING',locked=True,enabled=False,busy=False,
                 measurement_ready=True,rgbd_health=dict(stable=True)),self.clock)}
        if self.started:
            self.states['handling']=(dict(state=self.final_state,active=False,
                run_id='lift-456' if self.final_state=='COMPLETE' else 'pickup-123',reason='injected stop'),self.clock)

    def confirm_lift(self,watch):self.events.append('confirmed-lift')

    def get_clock(self):return NS(now=lambda:NS(nanoseconds=int(self.clock*1e9)))
    def fresh(self,key,after):
        v,stamp=self.states[key]
        return v if stamp>after else None

    def wait(self,predicate,timeout,label,watch):
        end=self.clock+timeout
        while self.clock<end:
            self.clock+=.1;self.refresh()
            if predicate():return
        raise RuntimeError('timeout: '+label)

    def call(self,typ,name,request,timeout):
        self.events.append(name)
        if name.endswith('/preview'):
            if self.preview_error:raise RuntimeError('Geometry disagrees with saved station')
            return NS(message=json.dumps(dict(plan=dict(goal=[1.,0.,0.]))))
        if name.endswith('/lift'):
            self.final_state='COMPLETE'
            return NS(message='lift-456')
        if name.endswith('/start'):
            if request.data:raise AssertionError('Lift must not be armed by insertion start')
            self.started=True
            return NS(message='pickup-123')
        return NS(message='stopped')


class WarmPickupHandoffTest(unittest.TestCase):
    def test_supervisor_handles_two_refresh_sequences_and_ignores_stale_repeat(self):
        with tempfile.TemporaryDirectory() as root:
            f=HandoffFixture(root);original=f.refresh;original_call=f.call
            done=[0];stale=[0]
            def states():
                original()
                if not f.started:return
                if stale[0]:
                    seq=done[0];stale[0]-=1
                elif done[0]<2:seq=done[0]+1
                else:return
                f.states['handling']=(dict(state='CHECKPOINT_RECOGNITION',phase='CHECKPOINT_RECOGNITION',
                    checkpoint_refresh_seq=seq,active=True,run_id='pickup-123'),f.clock)
            def call(typ,name,request,timeout):
                if name.endswith('/finish_checkpoint_recognition'):
                    done[0]+=1;stale[0]=3;return NS(message='refreshed')
                return original_call(typ,name,request,timeout)
            f.refresh=states;f.call=call;f.still=lambda:True;f.sensors=lambda:dict(position=0.)
            f.acquire=lambda watch:f.events.append('fresh-checkpoint-acquisition')
            with patch.object(cycle.time,'monotonic',lambda:f.clock):f.pickup_insert_and_lift(15.)
            self.assertEqual(f.events.count('fresh-checkpoint-acquisition'),2)
            self.assertEqual(f.events.count('/pallet/handling/start'),1)
            self.assertEqual(f.events.count('/pallet/handling/lift'),1)

    def test_structured_recovery_accepts_final_measurement_only_before_insertion(self):
        for phase in ('FINAL_ALIGN','FINAL_INSERTION'):
            with self.subTest(phase=phase),tempfile.TemporaryDirectory() as root:
                f=HandoffFixture(root,final_state='BLOCKED');original=f.refresh
                def states():
                    original()
                    if f.started:
                        f.states['handling'][0].update(phase=phase,
                            recovery_action='return_to_recognition',
                            reason='Không có đường đưa càng thẳng an toàn: Body is off the opening axis during entry')
                f.refresh=states
                with patch.object(cycle.time,'monotonic',lambda:f.clock),self.assertRaises(RuntimeError) as caught:
                    f.pickup_insert_and_lift(15.)
                self.assertEqual(isinstance(caught.exception,cycle.ReturnToRecognition),phase=='FINAL_ALIGN')
                self.assertEqual(f.events[-1],'/pallet/handling/stop')
                self.assertNotIn('/pallet/handling/lift',f.events)

    def test_only_checkpoint_angle_failure_requests_return(self):
        for phase,expected in [('FINAL_ALIGN',cycle.ReturnToRecognition),('FINAL_INSERTION',RuntimeError)]:
            with self.subTest(phase=phase),tempfile.TemporaryDirectory() as root:
                f=HandoffFixture(root,final_state='BLOCKED');original=f.refresh
                def states():
                    original()
                    if f.started:
                        f.states['handling'][0].update(phase=phase,
                            reason='Góc pallet đổi quá nhiều ở checkpoint; cần căn lại từ ngoài')
                f.refresh=states
                with patch.object(cycle.time,'monotonic',lambda:f.clock),self.assertRaises(expected) as exc:
                    f.pickup_insert_and_lift(15.)
                if phase=='FINAL_INSERTION':self.assertNotIsInstance(exc.exception,cycle.ReturnToRecognition)
                self.assertEqual(f.events[-1],'/pallet/handling/stop')
                self.assertNotIn('/pallet/handling/lift',f.events)

    def test_checkpoint_lateral_failure_requests_return_after_stop(self):
        with tempfile.TemporaryDirectory() as root:
            f=HandoffFixture(root,final_state='BLOCKED');original=f.refresh
            def states():
                original()
                if f.started:
                    f.states['handling'][0].update(phase='FINAL_ALIGN',
                        reason='Lệch ngang tại checkpoint; cần căn lại từ ngoài')
            f.refresh=states
            with patch.object(cycle.time,'monotonic',lambda:f.clock),self.assertRaises(cycle.ReturnToRecognition):
                f.pickup_insert_and_lift(15.)
            self.assertEqual(f.events[-1],'/pallet/handling/stop')
            self.assertNotIn('/pallet/handling/lift',f.events)

    def test_supervisor_refreshes_checkpoint_once_before_inserted_and_lift(self):
        with tempfile.TemporaryDirectory() as root:
            f=HandoffFixture(root);original=f.refresh;refresh_call=f.call
            f.refreshed=False
            def states():
                original()
                if f.started and not f.refreshed:
                    f.states['handling']=(dict(state='CHECKPOINT_RECOGNITION',phase='CHECKPOINT_RECOGNITION',
                        active=True,run_id='pickup-123'),f.clock)
            def call(typ,name,request,timeout):
                if name.endswith('/finish_checkpoint_recognition'):
                    f.events.append(name);f.refreshed=True;return NS(message='refreshed')
                return refresh_call(typ,name,request,timeout)
            f.refresh=states;f.call=call;f.still=lambda:True;f.sensors=lambda:dict(position=0.)
            f.acquire=lambda watch:f.events.append('fresh-checkpoint-acquisition')
            with patch.object(cycle.time,'monotonic',lambda:f.clock):f.pickup_insert_and_lift(15.)
            self.assertEqual(f.events.count('fresh-checkpoint-acquisition'),1)
            self.assertLess(f.events.index('fresh-checkpoint-acquisition'),f.events.index('/pallet/handling/lift'))

    def test_previews_then_starts_once_with_correct_run_heartbeat(self):
        with tempfile.TemporaryDirectory() as root:
            f=HandoffFixture(root)
            with patch.object(cycle.time,'monotonic',lambda:f.clock):
                f.pickup_insert_and_lift(10.)
            self.assertEqual(f.events,['/pallet/handling/preview','/pallet/handling/start','confirmed-lift','/pallet/handling/lift'])
            self.assertEqual(f.heartbeats,['pickup-123','lift-456'])
            self.assertIsNone(f.pickup_run_id)
            self.assertTrue((Path(root)/'pickup_preflight.json').is_file())

    def test_failed_geometry_is_logged_and_never_starts_or_sends_heartbeat(self):
        with tempfile.TemporaryDirectory() as root:
            f=HandoffFixture(root,preview_error=True)
            with patch.object(cycle.time,'monotonic',lambda:f.clock),self.assertRaisesRegex(RuntimeError,'Geometry'):
                f.pickup_insert_and_lift(10.)
            self.assertEqual(f.events,['/pallet/handling/preview'])
            self.assertFalse(f.heartbeats)
            self.assertFalse(json.loads((Path(root)/'pickup_preflight.json').read_text())['passed'])

    def test_unqualified_sensor_window_prevents_even_preview(self):
        with tempfile.TemporaryDirectory() as root:
            f=HandoffFixture(root,sensors_ready=False)
            with patch.object(cycle.time,'monotonic',lambda:f.clock),self.assertRaises(RuntimeError):
                f.pickup_insert_and_lift(10.)
            self.assertFalse(f.events)

    def test_failure_after_acceptance_stops_and_clears_heartbeat(self):
        with tempfile.TemporaryDirectory() as root:
            f=HandoffFixture(root,final_state='BLOCKED')
            with patch.object(cycle.time,'monotonic',lambda:f.clock),self.assertRaisesRegex(RuntimeError,'injected stop'):
                f.pickup_insert_and_lift(10.)
            self.assertEqual(f.events[-1],'/pallet/handling/stop')
            self.assertEqual(f.events.count('/pallet/handling/start'),1)
            self.assertIsNone(f.pickup_run_id)

    def test_no_confirmation_never_calls_lift_and_sends_stop(self):
        with tempfile.TemporaryDirectory() as root:
            f=HandoffFixture(root)
            def cancel(watch):raise RuntimeError('Operator did not confirm')
            f.confirm_lift=cancel
            with patch.object(cycle.time,'monotonic',lambda:f.clock),self.assertRaisesRegex(RuntimeError,'did not confirm'):
                f.pickup_insert_and_lift(10.)
            self.assertNotIn('/pallet/handling/lift',f.events)
            self.assertEqual(f.events[-1],'/pallet/handling/stop')

    def test_unknown_depth_rejected_before_creating_ros_node(self):
        import contextlib
        import io
        for value in (None,'0','nan','100'):
            argv=['run_pallet_cycle.py','run','--from-current-view','--pickup-only']
            if value is not None:argv+=['--usable-depth-cm',value]
            with patch.object(cycle.sys,'argv',argv),patch.object(cycle.rclpy,'init',side_effect=AssertionError('must reject before ROS')):
                with contextlib.redirect_stderr(io.StringIO()),self.assertRaises(SystemExit) as exc:
                    cycle.main()
                self.assertEqual(exc.exception.code,2)

    def test_lift_confirmation_requires_typed_word_and_live_inserted_state(self):
        import io
        f=NS(pickup_run_id='one',fresh=lambda key:dict(run_id='one',state='INSERTED',active=False),
             sensors=lambda:None,still=lambda:True)
        def wait(predicate,*a):
            for _ in range(4):
                if predicate():return
            raise AssertionError('No confirmation')
        f.wait=wait
        with patch.object(cycle.sys,'stdin',io.StringIO('wrong\nnang\n')):
            with patch.object(cycle.select,'select',lambda *a:([cycle.sys.stdin],[],[])):
                cycle.Cycle.confirm_lift(f)
        with patch.object(cycle.sys,'stdin',io.StringIO('')):
            with patch.object(cycle.select,'select',lambda *a:([cycle.sys.stdin],[],[])),self.assertRaisesRegex(RuntimeError,'không có đầu vào'):
                cycle.Cycle.confirm_lift(f)
        f.fresh=lambda key:dict(run_id='one',state='BLOCKED',active=False)
        with self.assertRaisesRegex(RuntimeError,'Mất trạng thái'):
            cycle.Cycle.confirm_lift(f)

    def test_cycle_dispatch_does_not_start_another_python_process(self):
        calls=[]
        f=NS(pickup_insert_and_lift=lambda *a:calls.append(a))
        with patch.object(cycle.subprocess,'Popen',side_effect=AssertionError('must reuse executor')):
            cycle.Cycle.operator(f,'pickup','insert-and-lift',105.,('staging','perception'))
        self.assertEqual(calls,[(105.,('staging','perception'))])

    def test_handling_heartbeat_stops_with_parent_lease(self):
        published=[]
        f=NS(lease=True,token='cycle-token',pickup_run_id='pickup-123',
            heartbeat=NS(publish=lambda m:published.append(m.data)),
            pickup_heartbeat=NS(publish=lambda m:published.append(m.data)))
        cycle.Cycle.beat(f)
        f.lease=False;cycle.Cycle.beat(f)
        self.assertEqual(published,['cycle-token','pickup-123'])


if __name__=='__main__':unittest.main()
