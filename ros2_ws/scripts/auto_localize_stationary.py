#!/usr/bin/env python3
"""Recover AMCL from a saved nearby pose after checking a stationary LiDAR scan."""

import argparse
import json
import math
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty
from tf2_ros import Buffer, TransformListener

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'camera/detect_astra/scripts'))
from pallet_map_alignment import MapAlignment, alignment_ok, proposal_issue


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y * q.y + q.z * q.z))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seed-report', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    saved = json.loads(args.seed_report.read_text())
    seed = [float(saved['x']), float(saved['y']), math.radians(float(saved['yaw_deg']))]
    rclpy.init()
    node = Node('auto_localize_stationary')
    state = {}
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(OccupancyGrid, '/map', lambda m: state.__setitem__('map', m), latched)
    node.create_subscription(LaserScan, '/scan_filtered', lambda m: state.__setitem__('scan', m), qos_profile_sensor_data)
    node.create_subscription(Odometry, '/odom', lambda m: state.__setitem__('odom', m), qos_profile_sensor_data)
    tf_buffer = Buffer()
    listener = TransformListener(tf_buffer, node)
    pub = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
    nomotion = node.create_client(Empty, '/request_nomotion_update')

    def pump(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=min(.1, end - time.monotonic()))

    def ready():
        odom = state.get('odom')
        return (all(key in state for key in ('map', 'scan', 'odom'))
                and pub.get_subscription_count() > 0
                and abs(odom.twist.twist.linear.x) < .015
                and abs(odom.twist.twist.angular.z) < .025)

    try:
        deadline = time.monotonic() + 20
        while not ready() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
        if not ready():
            raise RuntimeError('Map, fresh scan, stationary odometry or AMCL subscriber unavailable')
        pump(2.)
        if not ready():
            raise RuntimeError('Robot not stationary')
        grid = state['map']
        if grid.header.frame_id != 'map':
            raise RuntimeError('Unexpected map frame')
        matcher = MapAlignment(np.asarray(grid.data).reshape(grid.info.height, grid.info.width),
                               grid.info.resolution,
                               (grid.info.origin.position.x, grid.info.origin.position.y))

        def points_from_latest_scan():
            scan = state['scan']
            age = node.get_clock().now().nanoseconds / 1e9 - (
                scan.header.stamp.sec + scan.header.stamp.nanosec * 1e-9)
            if not 0 <= age <= .7:
                raise RuntimeError(f'LiDAR scan stale: {age:.2f} s')
            transform = tf_buffer.lookup_transform('base_footprint', scan.header.frame_id,
                                                   rclpy.time.Time()).transform
            angle = yaw_of(transform.rotation)
            ranges = np.asarray(scan.ranges, float)
            angles = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment
            valid = (np.isfinite(ranges) & (ranges > max(.2, scan.range_min))
                     & (ranges < min(5.5, scan.range_max)))
            x, y = ranges[valid] * np.cos(angles[valid]), ranges[valid] * np.sin(angles[valid])
            return np.column_stack((transform.translation.x + x*np.cos(angle) - y*np.sin(angle),
                                    transform.translation.y + x*np.sin(angle) + y*np.cos(angle)))

        first = points_from_latest_scan()
        before = matcher.metrics(seed, first)
        candidate = seed
        report = {'seed_report': str(args.seed_report), 'seed': seed, 'seed_metrics': before}
        if not alignment_ok(before):
            print(f'Seed scan median {before["median_m"]:.3f} m; searching nearby pose...', flush=True)
            solutions = matcher.propose(seed, first)
            pump(1.)
            holdout = matcher.metrics(solutions[0]['pose'], points_from_latest_scan()) if solutions else {}
            problem = proposal_issue(before, solutions, holdout)
            report.update(solutions=solutions[:3], holdout=holdout, issue=problem)
            if problem:
                raise RuntimeError(problem)
            candidate = solutions[0]['pose']
        pump(.5)
        if not ready():
            raise RuntimeError('Robot moved or AMCL became unavailable before initialization')
        holdout = matcher.metrics(candidate, points_from_latest_scan())
        report['candidate'] = candidate
        report['holdout'] = holdout
        if not alignment_ok(holdout):
            raise RuntimeError('Independent scan does not match map at candidate pose')
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x, msg.pose.pose.position.y = candidate[:2]
        msg.pose.pose.orientation.z = math.sin(candidate[2] / 2)
        msg.pose.pose.orientation.w = math.cos(candidate[2] / 2)
        msg.pose.covariance[0] = msg.pose.covariance[7] = .06**2
        msg.pose.covariance[35] = math.radians(4)**2
        for _ in range(8):
            msg.header.stamp = node.get_clock().now().to_msg()
            pub.publish(msg)
            pump(.3)
        if nomotion.wait_for_service(timeout_sec=2.):
            for _ in range(3):
                future = nomotion.call_async(Empty.Request())
                rclpy.spin_until_future_complete(node, future, timeout_sec=2.)
                pump(.3)
        deadline = time.monotonic() + 12
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
            try:
                transform = tf_buffer.lookup_transform('map', 'base_footprint',
                                                       rclpy.time.Time()).transform
            except Exception:
                continue
            pose = [transform.translation.x, transform.translation.y, yaw_of(transform.rotation)]
            after = matcher.metrics(pose, points_from_latest_scan())
            report.update(amcl_pose=pose, amcl_metrics=after, passed=alignment_ok(after) and ready())
            if report['passed']:
                print(f'AMCL localized: x={pose[0]:.3f} y={pose[1]:.3f} yaw={math.degrees(pose[2]):.2f} deg; scan median={after["median_m"]:.3f} m', flush=True)
                break
        if not report.get('passed'):
            raise RuntimeError('AMCL did not produce a stationary map pose matching LiDAR')
    except Exception as exc:
        report = locals().get('report', {})
        report.update(passed=False, error=str(exc))
        print('Localization failed:', exc, flush=True)
    finally:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
        node.destroy_node()
        rclpy.shutdown()
    return 0 if report.get('passed') else 1


if __name__ == '__main__':
    raise SystemExit(main())
