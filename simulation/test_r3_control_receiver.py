"""Fresh queued inputs precede control; receipt never renews old capture stamps."""
from collections import deque
import json
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import rclpy
from rclpy.executors import TimeoutException
from rclpy.time import Time

from r3_core import FrozenPlan
from r3_live_prepare import provisional_geometry
from r3_real_trial import RealTrial, drain_control_inputs


class ReadyTask:
    def __init__(self, callback):
        self.callback = callback

    def __call__(self):
        self.callback()

    def exception(self):
        return None

    def result(self):
        return None


class ReadyExecutor:
    def __init__(self, callbacks):
        self.callbacks = deque(callbacks)

    def wait_for_ready_callbacks(self, timeout_sec):
        if not self.callbacks:
            raise TimeoutException()
        return ReadyTask(self.callbacks.popleft()), None, None


class InputDrainTest(unittest.TestCase):
    def test_discovery_events_do_not_hide_fresh_sensor_waiting_behind_them(self):
        node = SimpleNamespace(input_revision=0, finished=False, pose='old')

        def fresh_input():
            node.input_revision += 1
            node.pose = 'fresh'

        executor = ReadyExecutor([lambda: None, lambda: None, fresh_input])
        count, _ = drain_control_inputs(node, executor)
        self.assertEqual(node.pose, 'fresh')
        self.assertEqual(count, 1)

    def test_busy_inputs_are_bounded_so_control_still_gets_a_turn(self):
        node = SimpleNamespace(input_revision=0, finished=False)

        def input_message():
            node.input_revision += 1

        executor = ReadyExecutor([input_message]*100)
        count, _ = drain_control_inputs(node, executor, max_callbacks=8)
        self.assertEqual(count, 8)
        self.assertEqual(len(executor.callbacks), 92)

    def test_an_exhausted_previous_readiness_snapshot_is_refreshed_before_tick(self):
        node = SimpleNamespace(input_revision=0, finished=False)

        def fresh_input():
            node.input_revision += 1

        executor = ReadyExecutor([fresh_input])
        original = executor.wait_for_ready_callbacks
        calls = 0

        def ready(timeout_sec):
            nonlocal calls
            calls += 1
            if calls == 1:
                raise TimeoutException()  # Cached iterator from an earlier waiting spin.
            return original(timeout_sec)

        executor.wait_for_ready_callbacks = ready
        count, _ = drain_control_inputs(node, executor)
        self.assertEqual(count, 1)
        self.assertGreaterEqual(calls, 4)


class CaptureAgeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init(domain_id=227)

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_transport_delay_cannot_extend_source_freshness_after_receipt(self):
        spec = dict(mode='amcl', run_id='receiver-age-test', map_file='/test/map.yaml',
                    baseline_map_odom=[1., 2., .1], correction_at_capture_xyyaw=[0., 0., 0.])
        target = dict(frame_id='r3_trial', pose=[2., 0., 0.], localization=spec)
        plan = FrozenPlan(json.dumps(target), json.dumps(provisional_geometry()),
                          json.dumps([dict(type='APPROACH', poses=[[0., 0., 0.], [1., 0., 0.]])]),
                          json.dumps(dict(localization=spec)))
        node = RealTrial(plan, Path('/tmp'), control_timer=False)
        try:
            node.armed = node.started = True
            node.localized_pose = (0., 0., 0.)
            node.localized_wall = node.odom_wall = 100.
            node.localized_capture_stamp = 99.81
            node.localization_status = dict(spec, frame_id='r3_trial', ready=True,
                correction_xyyaw=[0., 0., 0.], correction_settled=True, published_ros_s=100.)
            node.get_clock = lambda: SimpleNamespace(
                now=lambda: Time(nanoseconds=100_030_000_000))
            with patch('r3_real_trial.time.monotonic', return_value=100.03), \
                    patch.object(node.follow, 'step') as steer, patch.object(node, 'send') as send:
                node.tick()
            self.assertEqual(node.state, 'STOP_LATCHED')
            self.assertIn('source age 0.220s', node.reason)
            self.assertIn('receive age 0.030s', node.reason)
            steer.assert_not_called()
            send.assert_called_with()
            self.assertEqual(node.localized_capture_stamp, 99.81)
        finally:
            node.destroy_node()


if __name__ == '__main__':
    unittest.main()
