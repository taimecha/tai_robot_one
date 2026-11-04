import importlib.util
import math
from pathlib import Path
import unittest
import sys
from types import SimpleNamespace as NS
from unittest.mock import patch

path=Path(__file__).resolve().parents[1]/'scripts/run_pallet_delivery.py'
sys.path.insert(0,str(path.parent))
spec=importlib.util.spec_from_file_location('delivery_operator',path)
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)


class DeliveryAdmissionTest(unittest.TestCase):
    def test_stationarity_uses_elapsed_window_at_low_and_high_sensor_rates(self):
        for hz in (20,100,500):
            samples=[(10.-i/hz,0.,.002) for i in reversed(range(hz+1))]
            self.assertTrue(module.stationary_report(samples,10.)['stationary'])
        burst=[(10.-i*.0001,0.,0.) for i in reversed(range(100))]
        self.assertFalse(module.stationary_report(burst,10.)['stationary'])
        self.assertFalse(module.stationary_report(samples,12.)['stationary'])
        for v,w in ((.0031,0.),(0.,.0151),(float('nan'),0.)):
            bad=list(samples);bad[-5]=(bad[-5][0],v,w)
            self.assertFalse(module.stationary_report(bad,10.)['stationary'])

    def stationary_wait(self,mode,timeout=3.):
        clock=[0.];samples=[];checks=[]
        def pump(dt):
            clock[0]+=dt
            v=.01 if mode=='moving' or (mode=='transient' and clock[0]<.4) else 0.
            samples.append((clock[0],v,.001))
        lf=dict(state='IDLE' if mode!='lift_busy' else 'MOVING',velocity=0.,position=0.)
        def sensors():
            checks.append(clock[0])
            if mode=='stale':raise RuntimeError('Thời gian dữ liệu quá hạn: odom')
            return lf
        with patch.object(module.time,'monotonic',side_effect=lambda:clock[0]):
            result=module.wait_for_stationary(sensors,lambda:(1.,2.,0.),
                    lambda:module.stationary_report(samples,clock[0]),
                    lambda:mode=='other_goal',pump,timeout)
        return result,clock[0],checks

    def test_short_receipt_window_requalifies_before_admitting_navigation(self):
        result,elapsed,checks=self.stationary_wait('quiet')
        self.assertEqual(result[0]['state'],'IDLE')
        self.assertGreaterEqual(elapsed,.8)
        self.assertGreater(len(checks),10)
        _,elapsed,_=self.stationary_wait('transient')
        self.assertGreaterEqual(elapsed,1.35)

    def test_motion_busy_lift_competing_goal_and_stale_sensors_never_pass(self):
        for mode,reason in (('moving','max_linear_mps'),('lift_busy','lift_state'),
                            ('other_goal','goal khác'),('stale','quá hạn')):
            with self.assertRaisesRegex(RuntimeError,reason):self.stationary_wait(mode)

    def test_only_explicit_recognition_revisit_accepts_pending_visual_validation(self):
        for status in ('recorded','needs_visual_revalidation','pending_reteach','visual_relative_only',None):
            station={'status':status}
            for name in ('recognition','pickup','dropoff'):
                for revalidate in (False,True):
                    expected=(status=='recorded' or (name=='recognition' and revalidate
                              and status=='needs_visual_revalidation'))
                    self.assertEqual(module.station_navigation_ready(station,name,revalidate),expected)
            self.assertEqual(station,{'status':status})

    def test_recognition_backoff_preserves_heading_and_moves_away_along_saved_axis(self):
        for yaw in (-math.pi/2,0.,1.7):
            original=(3.,1.5,yaw)
            goal=module.recognition_backoff_goal(original,.35)
            self.assertAlmostEqual(goal[0],3.-.35*math.cos(yaw))
            self.assertAlmostEqual(goal[1],1.5-.35*math.sin(yaw))
            self.assertEqual(goal[2],yaw)
            self.assertEqual(original,(3.,1.5,yaw))
        for d in (-.1,.61,float('nan'),float('inf')):
            with self.assertRaises(ValueError):module.recognition_backoff_goal((0.,0.,0.),d)

    def endpoint(self,gid=1,namespace='/',name='bt_navigator'):
        return NS(node_name=name,node_namespace=namespace,endpoint_gid=[gid]*16)

    def discover(self,graph,ready=lambda t:True,timeout=3.):
        clock=[0.]
        def pump(seconds):clock[0]+=seconds
        node=NS(get_publishers_info_by_topic=lambda topic:graph(clock[0]))
        client=NS(server_is_ready=lambda:ready(clock[0]))
        with patch.object(module.time,'monotonic',side_effect=lambda:clock[0]):
            owner=module.wait_for_unique_nav2(node,client,pump,timeout=timeout)
        return owner,clock[0]

    def test_cold_discovery_waits_for_one_stable_ready_nav2_server(self):
        endpoint=self.endpoint()
        owner,elapsed=self.discover(lambda t:[] if t<.5 else [endpoint],ready=lambda t:t>=.8)
        self.assertEqual(owner,bytes(endpoint.endpoint_gid))
        self.assertGreaterEqual(elapsed,1.8)

    def test_duplicate_servers_and_wrong_namespace_or_unready_action_never_pass(self):
        graphs=([self.endpoint(),self.endpoint(2)],[self.endpoint(namespace='/other')],[])
        for infos in graphs:
            with self.assertRaisesRegex(RuntimeError,'status_publishers='):
                self.discover(lambda t:infos,timeout=1.5)
        with self.assertRaisesRegex(RuntimeError,'action_ready=False'):
            self.discover(lambda t:[self.endpoint()],ready=lambda t:False,timeout=1.5)

    def test_stale_duplicate_must_disappear_and_replacement_gid_resets_stability(self):
        first,second=self.endpoint(),self.endpoint(2)
        owner,elapsed=self.discover(lambda t:[first,second] if t<.4 else
                                   [first] if t<.9 else [second])
        self.assertEqual(owner,bytes(second.endpoint_gid))
        self.assertGreaterEqual(elapsed,1.9)

    def test_unload_requires_saved_destination_no_active_navigation_and_stillness(self):
        goal=(.201,-.448,-math.pi)
        lift=dict(homed=True,fault='NONE',serial_connected=True)
        self.assertEqual(module.lower_issue(goal,goal,False,True,lift),'')
        self.assertTrue(module.lower_issue((2.96,-.13,-1.54),goal,False,True,lift))
        self.assertTrue(module.lower_issue(goal,goal,True,True,lift))
        self.assertTrue(module.lower_issue(goal,goal,False,False,lift))
        self.assertTrue(module.lower_issue((.201,-.448,0.),goal,False,True,lift))
        self.assertTrue(module.lower_issue(goal,goal,False,True,{**lift,'fault':'SERIAL'}))

    def test_angle_wrap_does_not_reject_equivalent_dropoff_heading(self):
        self.assertTrue(module.at_station((.201,-.448,math.pi),(.201,-.448,-math.pi)))


if __name__=='__main__':unittest.main()
