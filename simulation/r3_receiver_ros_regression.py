#!/usr/bin/env python3
"""Isolated ROS transport regression for the real fixed-plan receiver loop.

Run with the robot's ROS/Python overlays sourced. This script refuses domain 0
or network discovery, uses a separate 50 Hz fake-source process, and never
starts hardware. All Twist messages remain in private LOCALHOST domain 231.
The controller and frozen recorded plan are the actual production classes.
"""
import argparse
import json
import math
import os
from pathlib import Path
import statistics
import subprocess
import sys
import time


DOMAIN = '231'
ROOT = Path(__file__).resolve().parents[1]
DEFAULT_PLAN = ROOT/'simulation/output/r3_live/20261115_184346_KBs7lW/plan/frozen_plan.json'
CASES = ('old_timer_pause', 'queued_fresh_pause', 'bad_capture', 'stream_loss', 'collision')


def load_plan(path):
    from r3_core import FrozenPlan
    data = json.loads(path.read_text())
    return FrozenPlan(**{key: data[key] for key in
                         ('target_json', 'geometry_json', 'phases_json', 'config_json')})


def assert_isolated():
    if os.environ.get('ROS_DOMAIN_ID') != DOMAIN:
        raise RuntimeError('Regression requires private ROS_DOMAIN_ID=231')
    if os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE') != 'LOCALHOST':
        raise RuntimeError('Regression requires ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST')


def producer(args):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu, LaserScan
    from std_msgs.msg import Bool, Header, String
    from nav2_msgs.msg import CollisionMonitorState

    assert_isolated()
    plan = load_plan(args.plan)
    target = json.loads(plan.target_json)
    start = json.loads(plan.phases_json)[0]['poses'][0]
    spec = target['localization']
    if any(abs(x) > 1e-9 for x in spec['correction_at_capture_xyyaw']):
        raise RuntimeError('This recorded-plan fixture expects identity capture correction')
    rclpy.init(domain_id=int(DOMAIN))
    node = Node('r3_regression_independent_50hz_sources')
    sensor_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
    pubs = {topic: node.create_publisher(cls, topic, qos) for cls, topic, qos in (
        (Odometry, '/odom', sensor_qos),
        (Odometry, '/base_controller/odom', sensor_qos),
        (Odometry, '/pallet/r3/localized_odom', sensor_qos),
        (Imu, '/imu', sensor_qos),
        (LaserScan, '/pallet/r3/filtered_lidar_scan', sensor_qos),
        (Bool, '/pallet/enabled', 1),
        (CollisionMonitorState, '/pallet/r3/collision_state', 1),
        (String, '/pallet/r3/control_odom/status', QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)),
    )}
    motion_at = None
    commands = []

    def command(msg):
        nonlocal motion_at
        now = time.monotonic()
        if abs(msg.linear.x)+abs(msg.angular.z) > 1e-9 and motion_at is None:
            motion_at = now
        commands.append(dict(wall=now, v=msg.linear.x, w=msg.angular.z))

    node.create_subscription(Twist, '/pallet/r3/cmd_vel', command, 10)
    started = time.monotonic()
    next_publish = started
    published = 0
    publish_walls = []
    faults = 0
    try:
        while time.monotonic()-started < 12. and not (args.output/'stop_requested').exists():
            rclpy.spin_once(node, timeout_sec=.001)
            now = time.monotonic()
            if now < next_publish:
                continue
            next_publish = max(next_publish+.02, now)
            fault = (args.case in ('bad_capture', 'stream_loss', 'collision')
                     and motion_at is not None and now-motion_at > .35)
            faults += int(fault)
            captured = node.get_clock().now().nanoseconds
            if args.case == 'bad_capture' and fault:
                captured -= 500_000_000
            stamp_sec, stamp_ns = divmod(captured, 1_000_000_000)
            odom = Odometry()
            odom.header = Header(frame_id='odom')
            odom.header.stamp.sec, odom.header.stamp.nanosec = stamp_sec, stamp_ns
            odom.child_frame_id = 'base_footprint'
            odom.pose.pose.position.x, odom.pose.pose.position.y = start[:2]
            odom.pose.pose.orientation.z = math.sin(start[2]/2)
            odom.pose.pose.orientation.w = math.cos(start[2]/2)
            imu = Imu()
            imu.header = Header(frame_id='imu_link')
            imu.header.stamp.sec, imu.header.stamp.nanosec = stamp_sec, stamp_ns
            imu.orientation.z, imu.orientation.w = (math.sin(start[2]/2), math.cos(start[2]/2))
            imu.orientation_covariance[0] = .0001
            if not (args.case == 'stream_loss' and fault):
                pubs['/odom'].publish(odom)
                pubs['/base_controller/odom'].publish(odom)
                odom.header.frame_id = 'r3_trial'
                pubs['/pallet/r3/localized_odom'].publish(odom)
                pubs['/imu'].publish(imu)
            scan = LaserScan()
            scan.header.stamp = node.get_clock().now().to_msg()
            scan.header.frame_id = 'laser_frame'
            scan.angle_min, scan.angle_max, scan.angle_increment = -.1, .1, .1
            scan.range_min, scan.range_max, scan.ranges = .05, 8., [8., 8., 8.]
            pubs['/pallet/r3/filtered_lidar_scan'].publish(scan)
            pubs['/pallet/enabled'].publish(Bool(data=False))
            collision = CollisionMonitorState()
            collision.action_type = (CollisionMonitorState.STOP if args.case == 'collision' and fault
                                     else CollisionMonitorState.DO_NOTHING)
            collision.polygon_name = 'regression obstacle' if collision.action_type else ''
            pubs['/pallet/r3/collision_state'].publish(collision)
            status = dict(ready=True, frame_id='r3_trial', run_id=spec['run_id'],
                          map_file=spec['map_file'], baseline_map_odom=spec['baseline_map_odom'],
                          correction_xyyaw=[0., 0., 0.], correction_settled=True,
                          published_ros_s=node.get_clock().now().nanoseconds*1e-9,
                          control_status_topic='/pallet/r3/control_odom/status',
                          control_mode='ekf_tracking')
            pubs['/pallet/r3/control_odom/status'].publish(String(data=json.dumps(status)))
            published += 1
            publish_walls.append(now)
    except KeyboardInterrupt:
        pass
    finally:
        args.output.mkdir(parents=True, exist_ok=True)
        (args.output/'producer.json').write_text(json.dumps(dict(
            rate_hz=50, published=published, faults=faults, motion_at=motion_at,
            commands=commands, publish_walls=publish_walls), indent=2))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def worker(args):
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from r3_real_trial import RealTrial, run_control_loop

    assert_isolated()
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output/'stop_requested').unlink(missing_ok=True)
    producer_log = (args.output/'producer.log').open('w')
    source = subprocess.Popen([sys.executable, __file__, '--producer', '--case', args.case,
                               '--plan', str(args.plan), '--output', str(args.output)],
                              stdout=producer_log, stderr=subprocess.STDOUT)
    rclpy.init(domain_id=int(DOMAIN))
    node = RealTrial(load_plan(args.plan), args.output, stop_at_c=True,
                     control_timer=args.case == 'old_timer_pause')
    node.armed, node.armed_at = True, time.monotonic()
    started_at = None
    pause = {}
    ticks = []
    real_tick = node.tick

    def observe_tick():
        nonlocal started_at
        real_tick()
        now = time.monotonic()
        if node.started and started_at is None:
            started_at = now
        now_ros = node.get_clock().now().nanoseconds*1e-9
        ticks.append(dict(wall=now, state=node.state, finished=node.finished,
                          command=list(node.last_command),
                          localized_source_age=(now_ros-node.localized_capture_stamp
                              if node.localized_capture_stamp else None),
                          localized_receive_age=now-node.localized_wall,
                          input_drain_callbacks=node.input_drain_callbacks,
                          input_drain_duration_s=node.input_drain_duration_s))
        # End the successful synthetic run before progress watchdog is relevant.
        if started_at is not None and now-started_at >= 1.4:
            node.finished = True
        if now-node.armed_at > 9.:
            node.fail('regression fixture timed out before normal completion')

    # Timer callbacks already bind self.tick in __init__; replace their callback too.
    if args.case == 'old_timer_pause':
        for timer in node.timers:
            timer.callback = observe_tick
    node.tick = observe_tick
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    real_spin_once = executor.spin_once

    def pause_when_moving(timeout_sec=None):
        if (args.case in ('old_timer_pause', 'queued_fresh_pause') and not pause
                and started_at is not None and time.monotonic()-started_at >= .2):
            pause['begin'] = time.monotonic()
            pause['state_before'] = node.state
            pause['command_before'] = list(node.last_command)
            pause['old_localized_capture_stamp'] = node.localized_capture_stamp
            time.sleep(.4)
            pause['end'] = time.monotonic()
        return real_spin_once(timeout_sec=timeout_sec)

    executor.spin_once = pause_when_moving
    error = None
    try:
        if args.case == 'old_timer_pause':
            # Old timer scheduling: timer-ready callbacks can run before queued input.
            while rclpy.ok() and not node.finished:
                executor.spin_once(timeout_sec=.01)
        else:
            run_control_loop(node, executor)
    except Exception as exc:
        error = repr(exc)
    finally:
        node.send()
        (args.output/'stop_requested').touch()
        try:
            source.wait(timeout=5.)
        except subprocess.TimeoutExpired:
            source.kill()
            source.wait()
        producer_log.close()
        producer_data = json.loads((args.output/'producer.json').read_text()) if (
            args.output/'producer.json').exists() else {}
        after_pause = [tick for tick in ticks if pause and tick['wall'] >= pause['end']]
        nonzero_after_pause = [tick for tick in after_pause if any(abs(x)>1e-9 for x in tick['command'])]
        publish_walls = producer_data.get('publish_walls', [])
        during_pause = sum(pause['begin'] <= wall <= pause['end'] for wall in publish_walls) if pause else 0
        observed_rate = ((len(publish_walls)-1)/(publish_walls[-1]-publish_walls[0])
                         if len(publish_walls) > 1 else None)
        if args.case == 'old_timer_pause':
            passed = bool(pause) and node.state == 'STOP_LATCHED' and 'localized odom unavailable' in node.reason
        elif args.case == 'queued_fresh_pause':
            passed = (bool(pause) and node.started and node.state == 'APPROACH'
                      and len(nonzero_after_pause) >= 5
                      and after_pause[0]['localized_source_age'] < .2
                      and after_pause[0]['input_drain_callbacks'] > 0
                      and during_pause >= 15)
        elif args.case in ('bad_capture', 'stream_loss'):
            passed = node.started and node.state == 'STOP_LATCHED' and 'localized odom unavailable' in node.reason
        else:
            passed = node.started and node.state == 'STOP_LATCHED' and 'collision monitor stopped' in node.reason
        passed = passed and error is None and source.returncode == 0
        report = dict(case=args.case, passed=passed, error=error, state=node.state,
                      reason=node.reason, started=node.started, pause=pause,
                      first_tick_after_pause=after_pause[0] if after_pause else None,
                      moving_ticks_after_pause=len(nonzero_after_pause), ticks=ticks,
                      fake_source=dict(rate_hz=50, published=producer_data.get('published'),
                                       observed_rate_hz=observed_rate,
                                       publications_during_receiver_pause=during_pause,
                                       faults=producer_data.get('faults'),
                                       returncode=source.returncode),
                      isolated_ros_domain=231, automatic_discovery_range='LOCALHOST',
                      hardware_started=False, physical_robot_commands=0)
        (args.output/'result.json').write_text(json.dumps(report, indent=2))
        node.destroy_node()
        executor.shutdown()
        rclpy.shutdown()
    print(json.dumps({key: value for key, value in report.items() if key != 'ticks'}), flush=True)
    return 0 if passed else 1


def benchmark(plan_path):
    from r3_sim import FixedPlanFollower
    plan = load_plan(plan_path)
    phases, target, geometry = (json.loads(plan.phases_json), json.loads(plan.target_json)['pose'],
                                json.loads(plan.geometry_json))
    samples = []
    first = []
    for _ in range(20):
        follow = FixedPlanFollower(phases, target, geometry)
        follow.start(True)
        for index, pose in enumerate(phases[0]['poses'][:80]):
            begin = time.perf_counter()
            follow.step(pose, dt=.05, c_verified=False)
            elapsed_ms = (time.perf_counter()-begin)*1000
            (first if index == 0 else samples).append(elapsed_ms)
    return dict(approach_pose_count=len(phases[0]['poses']), samples=len(samples),
                steady_median_ms=statistics.median(samples),
                steady_p95_ms=sorted(samples)[int(.95*(len(samples)-1))],
                steady_max_ms=max(samples), first_step_max_ms=max(first))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--plan', type=Path, default=DEFAULT_PLAN)
    parser.add_argument('--output', type=Path, default=ROOT/'simulation/output/control_receiver_ros_regression')
    parser.add_argument('--case', choices=CASES)
    parser.add_argument('--producer', action='store_true')
    parser.add_argument('--worker', action='store_true')
    args = parser.parse_args()
    assert_isolated()
    if args.producer:
        producer(args)
        return 0
    if args.worker:
        return worker(args)
    args.output.mkdir(parents=True, exist_ok=True)
    results = []
    for case in CASES:
        case_output = args.output/case
        case_output.mkdir(parents=True, exist_ok=True)
        with (case_output/'controller.log').open('w') as stream:
            completed = subprocess.run([sys.executable, __file__, '--worker', '--case', case,
                '--plan', str(args.plan), '--output', str(case_output)], stdout=stream,
                stderr=subprocess.STDOUT, timeout=25.)
        path = case_output/'result.json'
        result = json.loads(path.read_text()) if path.exists() else dict(case=case, passed=False)
        result['worker_returncode'] = completed.returncode
        results.append(dict({key: value for key, value in result.items() if key != 'ticks'},
                            trace_result=str(path)))
        print(json.dumps({key: value for key, value in result.items() if key != 'ticks'}), flush=True)
    report = dict(passed=all(item['passed'] for item in results), cases=results,
                  follower_benchmark=benchmark(args.plan), plan=str(args.plan),
                  no_hardware=True, ros_domain=231, discovery='LOCALHOST')
    (args.output/'report.json').write_text(json.dumps(report, indent=2))
    print(json.dumps(dict(passed=report['passed'], follower_benchmark=report['follower_benchmark'],
                          report=str(args.output/'report.json'))), flush=True)
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
