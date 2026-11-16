"""Actual ROS lost-response recovery, isolated from all hardware and goals."""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
from types import SimpleNamespace
import unittest

import rclpy
from lifecycle_msgs.srv import GetState
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from r3_amcl_bridge import AmclTrialBridge


DOMAIN=232
OUTPUT=Path(__file__).resolve().parent/'output/startup_ros_regression'


def delayed_service(ready_path):
    """First two replies arrive after the real client's two-second lease."""
    rclpy.init(domain_id=DOMAIN)
    node=Node('r3_delayed_lifecycle_service')
    count=0

    def reply(request,response):
        nonlocal count
        count+=1
        if count<=2:
            time.sleep(2.3)
        response.current_state.id=3
        response.current_state.label='active'
        return response

    node.create_service(GetState,'/amcl/get_state',reply)
    Path(ready_path).write_text('ready')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


class StartupTransportTest(unittest.TestCase):
    def test_bridge_recovers_two_real_delayed_replies_with_a_new_client(self):
        if os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE')!='LOCALHOST':
            self.skipTest('Run ROS transport regression with LOCALHOST discovery')
        context=Context()
        producer=None
        bridge=None
        executor=None
        OUTPUT.mkdir(parents=True,exist_ok=True)
        with tempfile.TemporaryDirectory() as tmp, (OUTPUT/'service.log').open('w') as log:
            try:
                ready=Path(tmp)/'service_ready'
                producer=subprocess.Popen([sys.executable,__file__,'--delayed-service',str(ready)],
                    stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
                rclpy.init(context=context,domain_id=DOMAIN)
                args=SimpleNamespace(map=Path(tmp)/'selected.yaml',initial_pose=None,
                    wait_ready=Path(tmp)/'ready.json',global_seed_ready=True,global_localization=False,
                    stationary_inspection=False,continuous_localization=True,navigation_monitor=True)
                bridge=AmclTrialBridge(args,context=context)
                executor=SingleThreadedExecutor(context=context)
                executor.add_node(bridge)
                started=time.monotonic()
                while time.monotonic()-started<16.:
                    executor.spin_once(timeout_sec=.02)
                    if producer.poll() is not None:
                        self.fail('Synthetic lifecycle server exited: '+(OUTPUT/'service.log').read_text())
                    state=bridge.state_poll.diagnostics(time.monotonic())['amcl']
                    if (bridge.active and state['request_timeouts']>=2
                            and state['client_recreations']>=1):
                        break
                state=bridge.state_poll.diagnostics(time.monotonic())['amcl']
                report=dict(passed=bridge.active and state.get('id')==3
                    and state['request_timeouts']>=2 and state['client_recreations']>=1,
                    elapsed_s=time.monotonic()-started,lifecycle=state,
                    localization_ready=bridge.ready_file_written,
                    ros_domain=DOMAIN,discovery='LOCALHOST',
                    synthetic_delayed_responses=2,reply_delay_s=2.3,
                    hardware_started=False,navigation_goals=0)
                (OUTPUT/'report.json').write_text(json.dumps(report,indent=2))
                self.assertTrue(report['passed'],report)
                # An ACTIVE service response alone never establishes a valid
                # map pose or bypasses the missing real sensor evidence.
                self.assertFalse(bridge.ready_file_written)
                self.assertIsNone(bridge.correction.baseline)
                self.assertEqual(bridge.nomotion_request_count,0)
            finally:
                if executor is not None:
                    if bridge is not None:
                        executor.remove_node(bridge)
                    executor.shutdown()
                if bridge is not None:
                    bridge.destroy_node()
                if context.ok():
                    context.shutdown()
                if producer is not None:
                    if producer.poll() is None:
                        os.killpg(producer.pid,signal.SIGINT)
                    try:
                        producer.wait(timeout=4.)
                    except subprocess.TimeoutExpired:
                        os.killpg(producer.pid,signal.SIGKILL)
                        producer.wait()


if __name__=='__main__':
    if len(sys.argv)>1 and sys.argv[1]=='--delayed-service':
        delayed_service(sys.argv[2])
    else:
        unittest.main()
