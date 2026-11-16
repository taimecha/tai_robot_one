"""Recorded November 16 lead-in failure can finish C without entering the pallet.

The JSON fixture preserves the actual 92-point plan and 151 control samples
from 20261116_081448_knwung so the regression does not require ignored run logs.
Its plan ID is 64dae1b1b7993dceaacdba806915cc1694106dab4a8f8161fdc7b8a6b1fbf96b.
Kinematic delay simulations exercise control behavior; they prove no hardware
alignment or available motor torque at low speed.
"""
from collections import deque
import contextlib
import io
import json
import math
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import rclpy
from nav2_msgs.msg import CollisionMonitorState
from rclpy.time import Time

from r3_core import FrozenPlan, compose, errors, goal_base, wrap
from r3_real_trial import RealTrial
from r3_sim import FixedPlanFollower


FIXTURE = Path(__file__).with_name('fixtures')/'r3_c_arrival_20261116.json'


def recorded_fixture():
    data = json.loads(FIXTURE.read_text())
    plan = FrozenPlan(*(json.dumps(data[key], sort_keys=True, allow_nan=False)
                        for key in ('target', 'geometry', 'phases', 'config')))
    if plan.plan_id != data['source']['plan_id']:
        raise ValueError('Recorded regression plan changed or fixture is corrupt')
    return dict(plan=vars(plan), telemetry=data['telemetry'], failure_pose=data['failure_pose'])


def replay(approach_only):
    fixture = recorded_fixture()
    plan = FrozenPlan(**fixture['plan'])
    geometry = json.loads(plan.geometry_json)
    target = json.loads(plan.target_json)['pose']
    phases = json.loads(plan.phases_json)
    follower = FixedPlanFollower(phases, target, geometry, approach_only=approach_only)
    follower.start(True)
    previous = fixture['telemetry'][0][0]-.05
    for wall, x, y, yaw, _, _ in fixture['telemetry']:
        follower.step((x, y, yaw), max(.001, min(.10, wall-previous)))
        previous = wall
    return follower, fixture


def continue_with_delay(follower, fixture, delay):
    """Use actual state at failure, then a synthetic delayed unicycle plant."""
    pose = tuple(fixture['failure_pose'])
    count = round(delay/.05)
    queue = deque([tuple(fixture['telemetry'][-1][-2:])]*count)
    plant_v, plant_w = fixture['telemetry'][-1][-2:]
    samples = []
    settled = 0.
    for step in range(600):
        command = follower.step(pose, .05)
        if queue:
            queue.append(command)
            desired_v, desired_w = queue.popleft()
        else:
            desired_v, desired_w = command
        # The synthetic drive cannot stop instantly when a zero command
        # arrives. Use the documented acceleration limits for braking too.
        dv = follower.g['acceleration']*.05
        dw = follower.g['angular_acceleration']*.05
        v = plant_v = max(plant_v-dv, min(plant_v+dv, desired_v))
        w = plant_w = max(plant_w-dw, min(plant_w+dw, desired_w))
        samples.append(dict(pose=pose, state=follower.state, command=command,
                            applied=(v, w), error=errors(pose, follower.target, follower.g)))
        if follower.state == 'STOP_LATCHED':
            break
        if v == w == 0. and follower.state == 'C_REACHED':
            settled += .05
            if settled >= .5:
                break
        else:
            settled = 0.
        if abs(w) < 1e-10:
            pose = compose(pose, (v*.05, 0., 0.))
        else:
            pose = compose(pose, (v/w*math.sin(w*.05),
                                  v/w*(1-math.cos(w*.05)), w*.05))
    return pose, samples


class RecordedCApproachTest(unittest.TestCase):
    def test_real_9mm_lead_in_breach_remains_repairable_without_relaxing_insertion(self):
        strict, fixture = replay(False)
        self.assertEqual(len(fixture['telemetry']), 151)
        pose = fixture['failure_pose']
        self.assertEqual(strict.step(pose), (0., 0.))
        self.assertEqual(strict.reason, 'straight lead-in lateral limit exceeded')
        approach, _ = replay(True)
        self.assertGreater(approach.step(pose)[0], 0.)
        self.assertEqual(approach.state, 'APPROACH')
        self.assertEqual(approach.phase, 0)
        self.assertEqual(approach.g['max_lateral'], .009)
        self.assertEqual(approach.g['max_yaw_deg'], .5)

    def test_recorded_approach_reaches_C_after_zero_to_200ms_command_delay(self):
        for delay in (0., .1, .2):
            with self.subTest(delay=delay):
                follower, fixture = replay(True)
                pose, samples = continue_with_delay(follower, fixture, delay)
                endpoint = follower.phases[0]['poses'][-1]
                self.assertEqual(follower.state, 'C_REACHED', follower.reason)
                self.assertLess(len(samples), 200, 'Approach stalled instead of reaching C')
                self.assertLessEqual(math.dist(pose[:2], endpoint[:2]), .02)
                self.assertLessEqual(abs(math.degrees(wrap(pose[2]-endpoint[2]))), 2.)
                self.assertGreaterEqual(min(s['error']['gap_m']-follower.g['fork_length']
                                            for s in samples), .10)
                self.assertEqual(follower.phase, 0)
                self.assertTrue(all(s['state'] in ('APPROACH', 'C_REACHED') for s in samples))
                self.assertEqual(follower.step(pose, c_verified=True), (0., 0.))
                self.assertEqual(follower.phase, 0, 'C->B must never start after reaching C')
                self.assertLess(abs((endpoint[0]-pose[0])*math.cos(endpoint[2])+
                                    (endpoint[1]-pose[1])*math.sin(endpoint[2])), .015,
                                'The old 5 cm-before-C diagnostic stop is not a C arrival')

    def test_arrival_mode_preserves_obstacle_and_invalid_odom_stops(self):
        for fault, expected in ((dict(obstacle=True), 'obstacle stop'),
                                (dict(odom_valid=False), 'invalid odom')):
            follower, fixture = replay(True)
            self.assertEqual(follower.step(fixture['failure_pose'], **fault), (0., 0.))
            self.assertEqual(follower.state, 'STOP_LATCHED')
            self.assertEqual(follower.reason, expected)

    def test_near_C_lateral_error_does_not_force_endpoint_creep(self):
        follower, _ = replay(True)
        endpoint = follower.phases[0]['poses'][-1]
        pose = compose(endpoint, (-.0055, .006, math.radians(.1)))
        follower.v = follower.follower.v = .020
        follower.w = 0.
        # A low-speed drive may not move during its first few commands.
        # A small residual lateral error must not suppress translation to
        # roughly 1 cm/s while the actual feedback turn remains gentle.
        for _ in range(5):
            v, w = follower.step(pose, .05)
        self.assertEqual(follower.state, 'APPROACH')
        self.assertGreaterEqual(v, .015)
        self.assertLess(v, .025)
        self.assertLess(abs(w), .03)

    def test_arrival_mode_preserves_minimum_fork_clearance(self):
        follower, _ = replay(True)
        unsafe = goal_base(follower.target, follower.g, follower.g['fork_length']+.09)
        self.assertEqual(follower.step(unsafe), (0., 0.))
        self.assertEqual(follower.state, 'STOP_LATCHED')
        self.assertIn('fork-tip clearance', follower.reason)


class SettledCArrivalTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init(domain_id=221)

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        plan = FrozenPlan(**recorded_fixture()['plan'])
        self.node = RealTrial(plan, Path(self.tmp.name), arrive_at_c=True, control_timer=False)
        node = self.node
        end = node.phases[0]['poses'][-1]
        # 12 mm lateral is inside C's staging region while missing the strict
        # 9 mm insertion gate. Arrival must report that difference honestly.
        node.pose = node.localized_pose = compose(end, (0., .012, 0.))
        node.imu_yaw = node.pose[2]
        node.imu_anchor = (node.pose[2], node.pose[2])
        node.twist = (0., 0.)
        node.yolo_enabled = False
        node.armed = node.started = True
        node.armed_at = 99.
        node.c_braking_since = 99.4
        node.follow.start(True)
        node.follow.state = 'C_REACHED'
        node.state = 'C_BRAKING'
        node.last_tick = node.last_progress_wall = 99.95
        self.fresh(100.)

    def tearDown(self):
        self.node.destroy_node()
        self.tmp.cleanup()

    def fresh(self, now):
        node = self.node
        node.localized_wall = node.odom_wall = node.imu_wall = node.yolo_wall = now
        node.localized_capture_stamp = node.odom_capture_stamp = node.imu_capture_stamp = now
        node.scan_wall['lidar'] = now
        node.localization_status = dict(node.localization, frame_id='r3_trial', ready=True,
            correction_xyyaw=[0., 0., 0.], correction_settled=True, published_ros_s=now)
        node.get_clock = lambda: SimpleNamespace(now=lambda: Time(nanoseconds=round(now*1e9)))

    def tick(self, now):
        with patch('r3_real_trial.time.monotonic', return_value=now), \
                patch.object(self.node, 'send') as send, contextlib.redirect_stdout(io.StringIO()):
            self.node.tick()
            send.assert_called_with()

    def test_settled_C_arrival_is_success_and_never_authorizes_insertion(self):
        self.tick(100.)
        node = self.node
        self.assertEqual(node.state, 'C_ARRIVED', node.reason)
        self.assertTrue(node.finished)
        self.assertAlmostEqual(node.c_arrival_report['position_error_m'], .012)
        self.assertFalse(node.c_arrival_report['insertion_alignment_satisfied'])
        self.assertFalse(node.c_arrival_report['insertion_authorized'])
        self.assertFalse(node.c_arrival_report['physical_alignment_verified'])
        self.assertEqual(node.follow.phase, 0)
        self.assertEqual(node.geometry['max_lateral'], .009)

    def test_C_success_waits_for_measured_rest_and_reports_failed_stop(self):
        node = self.node
        node.twist = (.02, 0.)
        self.tick(100.)
        self.assertFalse(node.finished)
        self.assertEqual(node.state, 'C_BRAKING')
        self.fresh(102.)
        self.tick(102.)
        self.assertEqual(node.state, 'STOP_LATCHED')
        self.assertIn('did not settle', node.reason)

    def test_pose_deviation_finishes_with_warning_and_no_insertion_authority(self):
        node = self.node
        node.pose = node.localized_pose = compose(node.phases[0]['poses'][-1], (0., .025, 0.))
        self.tick(100.)
        self.assertEqual(node.state, 'C_ALIGNMENT_WARNING')
        self.assertTrue(node.finished)
        self.assertAlmostEqual(node.c_arrival_report['position_error_m'], .025)
        self.assertFalse(node.c_arrival_report['insertion_authorized'])
        self.assertFalse(node.c_arrival_report['insertion_alignment_satisfied'])
        self.assertEqual(node.follow.phase, 0)

    def test_heading_deviation_is_reported_without_repeated_alignment_motion(self):
        node = self.node
        end = node.phases[0]['poses'][-1]
        node.pose = node.localized_pose = compose(end, (0., 0., math.radians(2.5)))
        node.imu_yaw = node.pose[2]
        node.imu_anchor = (node.pose[2], node.pose[2])
        self.tick(100.)
        self.assertEqual(node.state, 'C_ALIGNMENT_WARNING')
        self.assertTrue(node.finished)
        self.assertAlmostEqual(node.c_arrival_report['heading_error_deg'], 2.5)
        self.assertFalse(node.c_arrival_report['insertion_authorized'])
        self.assertIsNone(node.follow.align_goal)

    def test_actual_localized_source_loss_stops_even_while_braking_at_C(self):
        self.node.localized_wall = self.node.localized_capture_stamp = 99.5
        self.tick(100.)
        self.assertEqual(self.node.state, 'STOP_LATCHED')
        self.assertIn('localized odom unavailable', self.node.reason)

    def test_collision_stops_arrival_without_starting_insertion(self):
        message = CollisionMonitorState()
        message.action_type = CollisionMonitorState.STOP
        message.polygon_name = 'test obstacle'
        with patch.object(self.node, 'send') as send:
            self.node.on_collision_state(message)
            send.assert_called_with()
        self.assertEqual(self.node.state, 'STOP_LATCHED')
        self.assertIn('test obstacle', self.node.reason)
        self.assertEqual(self.node.follow.phase, 0)


if __name__ == '__main__':
    unittest.main()
