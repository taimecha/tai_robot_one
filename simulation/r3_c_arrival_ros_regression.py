#!/usr/bin/env python3
"""Actual ROS C-arrival loop with a private delayed synthetic wheel plant.

No hardware nodes or goals are started. Every publisher and subscription is
confined to LOCALHOST domain229; the production RealTrial drives a separate
process that integrates delayed commands and reports fresh synthetic sensors.
"""
import argparse
from collections import deque
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
PLAN = ROOT/'simulation/fixtures/r3_c_arrival_20261116.json'
OUTPUT = ROOT/'simulation/output/c_arrival_ros_regression'
DOMAIN = 229


def isolated():
    if (os.environ.get('ROS_DOMAIN_ID') != str(DOMAIN)
            or os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE') != 'LOCALHOST'):
        raise RuntimeError('Requires private ROS_DOMAIN_ID=229, discovery=LOCALHOST')


def load_plan():
    from r3_core import FrozenPlan
    saved = json.loads(PLAN.read_text())
    plan = FrozenPlan(*(json.dumps(saved[key], sort_keys=True, allow_nan=False)
                        for key in ('target', 'geometry', 'phases', 'config')))
    if plan.plan_id != saved['source']['plan_id']:
        raise ValueError('Recorded C-arrival fixture plan hash mismatch')
    return plan


def source():
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from nav2_msgs.msg import CollisionMonitorState
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from sensor_msgs.msg import Imu, LaserScan
    from std_msgs.msg import String, Bool
    isolated()
    plan = load_plan()
    spec = json.loads(plan.target_json)['localization']
    assert spec['correction_at_capture_xyyaw'] == [0., 0., 0.]
    pose = list(json.loads(plan.phases_json)[0]['poses'][0])
    rclpy.init(domain_id=DOMAIN)
    node = Node('r3_c_arrival_delayed_synthetic_plant')
    sensor = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
    odom = node.create_publisher(Odometry, '/odom', sensor)
    localized = node.create_publisher(Odometry, '/pallet/r3/localized_odom', sensor)
    imu_pub = node.create_publisher(Imu, '/imu', sensor)
    scan_pub = node.create_publisher(LaserScan, '/pallet/r3/filtered_lidar_scan', sensor)
    enabled = node.create_publisher(Bool, '/pallet/enabled', 1)
    collision = node.create_publisher(CollisionMonitorState, '/pallet/r3/collision_state', 1)
    status = node.create_publisher(String, '/pallet/r3/control_odom/status',
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    pending = deque()
    command = [0., 0.]
    received_commands = 0

    def receive(msg):
        nonlocal received_commands
        pending.append((time.monotonic()+.20, msg.linear.x, msg.angular.z))
        received_commands += 1

    node.create_subscription(Twist, '/pallet/r3/cmd_vel', receive, 10)
    previous = deadline = started = time.monotonic()
    published = 0
    try:
        while time.monotonic()-started < 35. and not (OUTPUT/'stop').exists():
            rclpy.spin_once(node, timeout_sec=.001)
            now = time.monotonic()
            if now < deadline:
                continue
            dt, previous, deadline = min(.05, now-previous), now, now+.02
            while pending and pending[0][0] <= now:
                _, command[0], command[1] = pending.popleft()
            # Integrate the delayed controller command profile;
            # fresh odometry reports this synthetic plant, never the latest command.
            v, w = command
            pose[0] += v*dt*math.cos(pose[2]+w*dt/2.)
            pose[1] += v*dt*math.sin(pose[2]+w*dt/2.)
            pose[2] = math.atan2(math.sin(pose[2]+w*dt), math.cos(pose[2]+w*dt))
            stamp = node.get_clock().now().to_msg()
            sample = Odometry()
            sample.header.stamp = stamp
            sample.header.frame_id = 'odom'
            sample.child_frame_id = 'base_footprint'
            sample.pose.pose.position.x, sample.pose.pose.position.y = pose[:2]
            sample.pose.pose.orientation.z, sample.pose.pose.orientation.w = math.sin(pose[2]/2.), math.cos(pose[2]/2.)
            sample.twist.twist.linear.x, sample.twist.twist.angular.z = v, w
            odom.publish(sample)
            sample.header.frame_id = 'r3_trial'
            localized.publish(sample)
            imu = Imu()
            imu.header.stamp, imu.header.frame_id = stamp, 'imu_link'
            imu.orientation.z, imu.orientation.w = math.sin(pose[2]/2.), math.cos(pose[2]/2.)
            imu.orientation_covariance[0] = .0001
            imu_pub.publish(imu)
            scan = LaserScan()
            scan.header.stamp, scan.header.frame_id = stamp, 'laser_frame'
            scan.range_min, scan.range_max, scan.angle_increment = .05, 8., .1
            scan.ranges = [8., 8., 8.]
            scan_pub.publish(scan)
            enabled.publish(Bool(data=False))
            event = CollisionMonitorState()
            event.action_type = CollisionMonitorState.DO_NOTHING
            collision.publish(event)
            health = dict(spec, ready=True, frame_id='r3_trial', correction_xyyaw=[0.,0.,0.],
                correction_settled=True, published_ros_s=node.get_clock().now().nanoseconds*1e-9,
                control_mode='ekf_tracking')
            status.publish(String(data=json.dumps(health)))
            published += 1
    finally:
        (OUTPUT/'plant.json').write_text(json.dumps(dict(published=published,
            received_commands=received_commands, final_pose=pose, command_delay_s=.2), indent=2))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main():
    isolated()
    OUTPUT.mkdir(parents=True, exist_ok=True)
    if '--source' in sys.argv:
        return source()
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from r3_real_trial import RealTrial, run_control_loop
    from r3_sim import write_csv
    plan = load_plan()
    (OUTPUT/'stop').unlink(missing_ok=True)
    with (OUTPUT/'plant.log').open('w') as stream:
        producer = subprocess.Popen([sys.executable, __file__, '--source'], stdout=stream, stderr=subprocess.STDOUT)
        rclpy.init(domain_id=DOMAIN)
        node = RealTrial(plan, OUTPUT, arrive_at_c=True, control_timer=False)
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        node.armed, node.armed_at = True, time.monotonic()
        original_tick = node.tick
        started = time.monotonic()

        def tick():
            original_tick()
            if time.monotonic()-started > 30. and not node.finished:
                node.fail('synthetic C-arrival regression timeout')

        node.tick = tick
        error = None
        try:
            run_control_loop(node, executor)
        except Exception as exc:
            error = repr(exc)
        finally:
            node.send()
            (OUTPUT/'stop').touch()
            try:
                producer.wait(timeout=5.)
            except subprocess.TimeoutExpired:
                producer.kill()
                producer.wait()
            report = dict(passed=node.state=='C_ARRIVED' and error is None and producer.returncode==0,
                state=node.state, reason=node.reason, error=error, c_arrival=node.c_arrival_report,
                elapsed_s=time.monotonic()-started, control_rows=len(node.rows),
                executed_states=sorted({row['state'] for row in node.rows}),
                command_delay_s=.2, ros_domain=DOMAIN, discovery='LOCALHOST',
                hardware_started=False, physical_robot_commands=0)
            (OUTPUT/'report.json').write_text(json.dumps(report, indent=2))
            if node.rows:
                write_csv(OUTPUT/'executed_odom_base.csv', node.rows)
            (OUTPUT/'frozen_plan.json').write_text(json.dumps(
                dict(vars(node.plan), plan_id=node.plan.plan_id), indent=2))
            (OUTPUT/'mission_result.json').write_text(json.dumps(dict(
                status=node.state, reason=node.reason, execution_mode='arrive_at_c',
                plan_id=node.plan.plan_id, c_arrival=node.c_arrival_report), indent=2))
            node.diagnostics.write(OUTPUT)
            executor.remove_node(node)
            executor.shutdown()
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
    print(json.dumps(report, indent=2))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
