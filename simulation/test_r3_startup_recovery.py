"""Recover lost lifecycle responses before any Nav2 goal is allowed."""
from pathlib import Path
import subprocess
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

import rclpy
from rclpy.context import Context
from rclpy.task import Future
from rclpy.time import Time
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

from r3_amcl_bridge import AmclTrialBridge


ROOT=Path(__file__).resolve().parents[1]


class BridgeLifecycleRecoveryTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.context=Context()
        rclpy.init(context=cls.context,domain_id=228)

    @classmethod
    def tearDownClass(cls):
        cls.context.shutdown()

    def test_lost_get_state_reply_does_not_suppress_the_only_parked_refresh_forever(self):
        with tempfile.TemporaryDirectory() as tmp:
            args=SimpleNamespace(map=Path(tmp)/'map.yaml',initial_pose=None,
                wait_ready=None,global_seed_ready=True,global_localization=False,
                continuous_localization=True,navigation_monitor=True,stationary_inspection=False)
            bridge=AmclTrialBridge(args,context=self.context)
            try:
                bridge.get_clock=lambda:SimpleNamespace(now=lambda:Time(nanoseconds=100_000_000_000))
                sample=Odometry()
                sample.header.frame_id='odom';sample.child_frame_id='base_footprint'
                sample.header.stamp=Time(nanoseconds=100_000_000_000).to_msg()
                sample.pose.pose.orientation.w=1.
                bridge.on_odom(sample)
                bridge.active=False;bridge.map_received=True
                bridge.parked_since_wall=99.;bridge.nomotion_interfaces_since=99.
                bridge.amcl_subscription=Mock(get_publisher_count=Mock(return_value=1))
                bridge.cloud_subscription=Mock(get_publisher_count=Mock(return_value=1))
                bridge.tf_buffer=Mock(can_transform=Mock(return_value=True))
                bridge.scan_stamp=100.
                bridge.latest_scan=LaserScan();bridge.latest_scan.header.stamp=sample.header.stamp
                missing=Future();active=Future()
                active.set_result(SimpleNamespace(current_state=SimpleNamespace(id=3)))
                client=Mock(service_is_ready=Mock(return_value=True),
                            call_async=Mock(side_effect=[missing,active]))
                bridge.state_client=client
                bridge.nomotion_client=Mock(service_is_ready=Mock(return_value=True),
                    call_async=Mock(return_value=SimpleNamespace(done=lambda:True)))
                bridge.poll_services(100.)
                bridge.poll_services(102.01)
                self.assertFalse(bridge.active)
                self.assertIn('timed out',bridge.last_state_error)
                client.remove_pending_request.assert_called_once_with(missing)
                bridge.nomotion_client.call_async.assert_not_called()
                bridge.poll_services(102.52)
                bridge.poll_services(102.57)
                self.assertTrue(bridge.active)
                self.assertIsNone(bridge.last_state_error)
                self.assertEqual(client.call_async.call_count,2)
                bridge.nomotion_client.call_async.assert_called_once()
                self.assertEqual(bridge.nomotion_request_count,1)
                self.assertFalse(bridge.parking_refresh_armed)
                self.assertTrue(bridge.seed_refresh_requested)
                # A late old response is never processed as a new result or
                # used to issue another AMCL no-motion update.
                bridge.next_state=float('inf')
                bridge.poll_services(102.7)
                self.assertEqual(bridge.nomotion_request_count,1)
            finally:
                bridge.destroy_node()


class NativeNavigationStartupTest(unittest.TestCase):
    def run_startup(self,failures):
        source=(ROOT/'simulation/run_r3_live_trial.sh').read_text()
        start=source.index('start_navigation() {')
        helper=source[start:source.index('\n}\n',start)+3]
        # Execute the actual shell retry path. Only owned Nav2 processes and
        # lifecycle queries are replaced; no ROS or hardware is started.
        prelude='''set -euo pipefail
navigation_pid=''
setsid() { printf 'LAUNCH %s\\n' "$*" >> "$CALL_LOG"; printf 'attempt started\\n'; }
stop_owned() { printf 'STOP %s\\n' "$1" >> "$CALL_LOG"; wait "$1" 2>/dev/null || true; }
sleep() { :; }
python3() {
  local count=0
  [[ ! -f "$COUNT_FILE" ]] || read -r count < "$COUNT_FILE"
  count=$((count+1)); printf '%s\\n' "$count" > "$COUNT_FILE"
  printf 'CHECK %s\\n' "$*" >> "$CALL_LOG"
  if (( count <= FAILURES )); then printf 'Lifecycle startup aborted\\n'; return 2; fi
  printf 'all Nav2 nodes lifecycle ACTIVE\\n'
}
'''
        with tempfile.TemporaryDirectory() as tmp:
            run=Path(tmp)
            env={'PATH':'/usr/bin:/bin','CALL_LOG':str(run/'calls.log'),
                 'COUNT_FILE':str(run/'count'),'FAILURES':str(failures),'run_dir':tmp}
            code=prelude+helper+'\nstart_navigation\nprintf "GOAL_STAGE_ALLOWED\\n" >> "$CALL_LOG"\n'
            result=subprocess.run(['/bin/bash','-c',code],env=env,cwd=ROOT,
                                  capture_output=True,text=True,timeout=5.)
            calls=(run/'calls.log').read_text().splitlines()
            saved_logs=sorted(p.name for p in run.glob('navigation_attempt_*.log'))
            return result,calls,saved_logs,(run/'navigation.log').exists()

    def test_failed_group_is_closed_and_retried_without_resetting_amcl_or_global_search(self):
        result,calls,history,latest=self.run_startup(1)
        self.assertEqual(result.returncode,0,result.stderr)
        launches=[i for i,c in enumerate(calls) if c.startswith('LAUNCH ')]
        checks=[i for i,c in enumerate(calls) if c.startswith('CHECK ')]
        stops=[i for i,c in enumerate(calls) if c.startswith('STOP ')]
        self.assertEqual((len(launches),len(checks),len(stops)),(2,2,1))
        self.assertLess(checks[0],stops[0]);self.assertLess(stops[0],launches[1])
        self.assertIn('GOAL_STAGE_ALLOWED',calls)
        self.assertEqual(history,['navigation_attempt_1.log'])
        self.assertTrue(latest)
        self.assertTrue(all('r3_navigation.launch.py' in calls[i] for i in launches))
        self.assertTrue(all('--startup-log' in calls[i] and 'collision_monitor bt_navigator' in calls[i]
                            for i in checks))
        self.assertFalse(any('r3_localization.launch.py' in c or 'r3_global_localization.py' in c
                             for c in calls))

    def test_repeated_failure_has_a_bound_and_never_reaches_goal_stage(self):
        result,calls,history,latest=self.run_startup(9)
        self.assertEqual(result.returncode,2,result.stderr)
        self.assertEqual(sum(c.startswith('LAUNCH ') for c in calls),3)
        self.assertEqual(sum(c.startswith('STOP ') for c in calls),3)
        self.assertNotIn('GOAL_STAGE_ALLOWED',calls)
        self.assertEqual(len(history),3)
        self.assertFalse(latest)


if __name__=='__main__':
    unittest.main()
