#!/usr/bin/env python3
"""Filter the one locked pallet from the LiDAR scan for r3 safety monitor.

No wheel/lift publisher. A missing transform or stale scan yields no output,
so the downstream collision monitor must time out and stop the vehicle.
"""

import argparse
import copy
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener

from r3_core import (R3_PLANNED_C_CLEARANCE_M, FrozenPlan, errors,
                     real_geometry_mode, validate_phases, validate_axis_lateral_residual, compose)
from r3_localization_contract import localization_spec, checked_status, CorrectionHistory
from std_msgs.msg import String
from rclpy.qos import QoSProfile, DurabilityPolicy
from r3_pallet_obstacles import classify_ranges


def read_real_target(path, allow_provisional=False, max_age_s=30.):
    data = json.loads(path.read_text())
    plan = FrozenPlan(**{key: data[key] for key in
                         ('target_json', 'geometry_json', 'phases_json', 'config_json')})
    if plan.plan_id != data.get('plan_id'):
        raise ValueError('frozen plan hash mismatch')
    target = json.loads(plan.target_json)
    localization_spec(target)
    geometry = json.loads(plan.geometry_json)
    phases = json.loads(plan.phases_json)
    config = json.loads(plan.config_json)
    validate_phases(phases, target['pose'], geometry)
    clearance = (errors(phases[1]['poses'][0], target['pose'], geometry)['gap_m']
                 - geometry['fork_length'])
    if clearance < R3_PLANNED_C_CLEARANCE_M - 1e-6:
        raise ValueError('C leaves less than 15 cm between fork tips and pallet face')
    if geometry['max_lateral'] > .010:
        raise ValueError('real terminal lateral error must stay under 1 cm')
    refinement = config.get('approach_refinement', {})
    if ([phase.get('type') for phase in phases] != ['APPROACH', 'STRAIGHT']
            or refinement.get('method') != 'bounded fork-root shape search on clamped quintic y(x)'
            or refinement.get('selected_C_unchanged') is not True
            or refinement.get('terminal_line_unchanged') is not True):
        raise ValueError('the reviewed refined A→C and straight C→B plan is required')
    real_geometry_mode(geometry, allow_provisional)
    if (target.get('source') != 'stationary_real_RGBD'
            or not target.get('raw_session_hash')
            or not target.get('yolo_off_ack')
            or geometry.get('provenance') == 'SYNTHETIC_FIXTURE_NOT_REAL_CALIBRATION'):
        raise ValueError('fresh locked real pallet required')
    report = target.get('measurement_report') or {}
    validate_axis_lateral_residual(target, phases[0]['poses'][0])
    raw, offset, corrected = (report.get('camera_lateral_raw_m'),
                              report.get('camera_mount_offset_m'),
                              report.get('fork_lateral_corrected_m'))
    if (not all(isinstance(v, (int, float)) and math.isfinite(v)
                for v in (raw, offset, corrected))
            or not math.isclose(offset, .025, abs_tol=1e-9)
            or not math.isclose(raw + offset, corrected, abs_tol=1e-4)):
        raise ValueError('camera raw + one +2.5 cm correction report required')
    if (report.get('heading_source') != 'floor_referenced_plane'
            or not isinstance(report.get('heading_depth_x_slope'), (int, float))
            or not math.isfinite(report['heading_depth_x_slope'])):
        raise ValueError('fresh floor-referenced pallet heading required')
    if max_age_s is not None and not 0 <= time.time() - target['capture_stamp_s'] <= max_age_s:
        raise ValueError('locked pallet capture is stale; recognize again')
    return target, geometry


class PalletObstacleFilter(Node):
    def __init__(self, target, geometry):
        super().__init__('r3_pallet_obstacle_filter')
        self.target, self.geometry = target['pose'], geometry
        self.localization = localization_spec(target)
        self.localization_status = None
        self.correction_history = CorrectionHistory()
        if self.localization:
            self.create_subscription(String, self.localization.get('control_status_topic','/pallet/r3/localization/status'), self.on_localization,
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.received = {'lidar': 0.}
        self.published = {'lidar': 0.}
        self.last_drop = {'lidar': 'no input'}
        self.outputs = {
            'lidar': self.create_publisher(LaserScan, '/pallet/r3/filtered_lidar_scan', qos_profile_sensor_data),
        }
        self.create_subscription(LaserScan, '/scan_filtered',
                                 lambda msg: self.on_scan('lidar', msg), qos_profile_sensor_data)
        self.create_timer(.5, self.report)

    def on_localization(self, msg):
        try:
            self.localization_status = checked_status(
                msg.data, self.get_clock().now().nanoseconds*1e-9, self.localization)
            self.correction_history.add(self.localization_status)
        except (ValueError, TypeError, KeyError):
            self.localization_status = None
            self.correction_history.clear()

    def on_scan(self, name, msg):
        now = self.get_clock().now().nanoseconds * 1e-9
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.received[name] = time.monotonic()
        if not msg.header.frame_id or not -.05 <= now - stamp <= .3:
            self.last_drop[name] = f'bad frame or stamp (age {now-stamp:.3f} s)'
            return
        try:
            transform = self.tf.lookup_transform('odom', msg.header.frame_id,
                                                 Time.from_msg(msg.header.stamp),
                                                 timeout=Duration(seconds=.03))
        except TransformException as exc:
            self.last_drop[name] = f'odom transform unavailable at scan stamp: {exc}'
            return
        t, q = transform.transform.translation, transform.transform.rotation
        if not all(math.isfinite(v) for v in (t.x, t.y, q.x, q.y, q.z, q.w)):
            self.last_drop[name] = 'nonfinite odom transform'
            return
        yaw = math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))
        scan_pose = (t.x,t.y,yaw)
        if self.localization:
            try:
                state = checked_status(self.localization_status or {}, now, self.localization)
                # Transform the raw scan with the applied correction at its
                # capture stamp, not the newest correction at callback time.
                scan_pose = compose(self.correction_history.at(stamp), scan_pose)
            except (ValueError, TypeError, KeyError) as exc:
                self.last_drop[name] = str(exc)
                return
        try:
            ranges, pallet, other = classify_ranges(
                msg.ranges, msg.angle_min, msg.angle_increment,
                scan_pose, self.target, self.geometry)
        except ValueError:
            self.last_drop[name] = 'invalid scan geometry'
            return
        filtered = copy.copy(msg)
        filtered.ranges = ranges
        self.outputs[name].publish(filtered)
        self.published[name] = time.monotonic()
        self.last_drop[name] = 'none'
        if pallet:
            self.get_logger().debug(f'{name}: ignored {pallet} pallet returns, kept {other} other returns')

    def report(self):
        now = time.monotonic()
        stale = [name for name, stamp in self.published.items() if now - stamp > 1.0]
        if stale:
            detail = ', '.join(f'{name} input_age={now-self.received[name]:.2f}s '
                               f'output_age={now-self.published[name]:.2f}s '
                               f'last_drop={self.last_drop[name]}' for name in stale)
            self.get_logger().warn('No fresh filtered scan: ' + detail, throttle_duration_sec=2.)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--plan', required=True, type=Path)
    parser.add_argument('--allow-provisional', action='store_true')
    args = parser.parse_args()
    target, geometry = read_real_target(args.plan, args.allow_provisional)
    rclpy.init()
    node = PalletObstacleFilter(target, geometry)
    # Scan callbacks wait briefly for the transform at the scan timestamp.
    # A second executor thread lets the TF listener receive it during that wait.
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.remove_node(node)
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
