#!/usr/bin/env python3
"""RGB-D pallet staging alignment. Default mode publishes proposed commands only."""
import argparse
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.task import Future
from rclpy.parameter import parameter_value_to_python
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rcl_interfaces.srv import GetParameters
from lifecycle_msgs.srv import GetState
from geometry_msgs.msg import PoseStamped, Twist, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Bool
from std_srvs.srv import SetBool, Trigger
from visualization_msgs.msg import Marker, MarkerArray
import yaml
from detected_pallets_interfaces.msg import PalletTarget
from pallet_alignment_core import AlignmentConfig, AlignmentController, OdomSample, wrap
from pallet_laser_timing import LASER_TIMEOUT_S, validate_stop_polygon

ROOT = Path(__file__).resolve().parents[1]
FRAME = 'camera_color_optical_frame'
COMMAND = '/pallet/alignment/cmd_vel'
# A rolling five-second qualification window needs time to replace a bad
# sample/gap. This only extends admission waiting, never sensor freshness.
START_DATA_TIMEOUT_S = 20.0


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def rgbd_wait_reason(health):
    reason='Chờ cửa sổ RGB-D ổn định 5 giây trước khi chạy'
    try:
        fps,ratio,gap=(float(health[k]) for k in ('valid_fps','valid_ratio','max_gap_s'))
        if all(math.isfinite(v) for v in (fps,ratio,gap)):
            reason+=f': {fps:.1f} Hz, hợp lệ {ratio*100:.0f}%, ngắt lớn nhất {gap:.2f} s'
    except (TypeError,KeyError,ValueError):
        pass
    return reason


class MotionGuard:
    """Fail closed until lifecycle, configured routing, laser and graph agree."""
    def __init__(self, node):
        self.node = node
        self.checks = {}
        self.pending = {}
        self.scan_stamp = self.scan_wall = None
        config = yaml.safe_load((ROOT / 'config/pallet_alignment_collision.yaml').read_text())
        expected = config['pallet_alignment_collision_monitor']['ros__parameters']
        if expected['source_timeout'] != LASER_TIMEOUT_S:
            raise ValueError('Laser watchdog and Collision Monitor must agree')
        validate_stop_polygon(yaml.safe_load(expected['StopZone']['points']))
        # Verify the actual protection parameters, not just the lifecycle state.
        names = ['base_frame_id', 'odom_frame_id', 'cmd_vel_in_topic', 'cmd_vel_out_topic',
                 'source_timeout', 'transform_tolerance', 'base_shift_correction',
                 'polygons', 'StopZone.type', 'StopZone.points',
                 'StopZone.action_type', 'StopZone.min_points', 'StopZone.enabled',
                 'observation_sources', 'scan.type', 'scan.topic', 'scan.enabled', 'use_sim_time']
        def get_value(key):
            parts = key.split('.')
            return expected[parts[0]][parts[1]] if len(parts) == 2 else expected[key]
        self.expected = {name: get_value(name) for name in names}
        self.params = node.create_client(GetParameters, '/pallet_alignment_collision_monitor/get_parameters')
        self.stamper = node.create_client(GetParameters, '/cmd_vel_stamper/get_parameters')
        self.lifecycle = node.create_client(GetState, '/pallet_alignment_collision_monitor/get_state')
        self.sub = node.create_subscription(LaserScan, '/scan_filtered', self.on_scan,
            QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT))
        self.timer = node.create_timer(.5, self.poll)

    def on_scan(self, msg):
        stamp=stamp_seconds(msg.header.stamp)
        if (not -.05<=self.node.now()-stamp<=LASER_TIMEOUT_S
                or (self.scan_stamp is not None and stamp<=self.scan_stamp)):
            return
        if (math.isfinite(msg.angle_increment) and msg.angle_increment > 0
                and 0 < msg.range_min < msg.range_max
                and sum(math.isfinite(r) and msg.range_min <= r <= msg.range_max for r in msg.ranges) >= 2):
            self.scan_stamp, self.scan_wall = stamp_seconds(msg.header.stamp), time.monotonic()
        else:
            self.scan_stamp = self.scan_wall = None

    def request(self, key, client, request, predicate):
        now = time.monotonic()
        previous = self.pending.get(key)
        if previous and not previous[0].done():
            if now-previous[1] < .8:
                return
            client.remove_pending_request(previous[0])
            previous[0].cancel()
        if not client.service_is_ready():
            self.checks[key] = (False, now)
            return
        future = client.call_async(request)
        self.pending[key] = (future, now)
        def finished(f):
            try:
                valid = not f.cancelled() and predicate(f.result())
            except Exception:
                valid = False
            self.checks[key] = (valid, time.monotonic())
        future.add_done_callback(finished)

    def poll(self):
        self.request('lifecycle', self.lifecycle, GetState.Request(), lambda r: r.current_state.id == 3)
        req = GetParameters.Request(); req.names = list(self.expected)
        self.request('monitor', self.params, req, lambda r: len(r.values) == len(req.names)
                     and all(parameter_value_to_python(v) == self.expected[n] for n, v in zip(req.names, r.values)))
        req2 = GetParameters.Request()
        req2.names = ['input_topic', 'output_topic', 'require_single_publisher', 'use_sim_time']
        self.request('stamper', self.stamper, req2, lambda r:
                     [parameter_value_to_python(v) for v in r.values] ==
                     ['/cmd_vel_safe', '/base_controller/cmd_vel', True, False])

    def ready(self, now, wall):
        for key in ('lifecycle', 'monitor', 'stamper'):
            valid, received = self.checks.get(key, (False, 0))
            if not valid or wall-received > 1.0:
                return False, 'Chưa xác minh lớp tránh vật cản và đường lệnh xe: ' + key
        expected = {COMMAND: 'pallet_alignment', '/cmd_vel_safe': 'pallet_alignment_collision_monitor',
                    '/base_controller/cmd_vel': 'cmd_vel_stamper'}
        for topic, name in expected.items():
            sources = self.node.get_publishers_info_by_topic(topic)
            if len(sources) != 1 or sources[0].node_name != name or sources[0].node_namespace != '/':
                return False, 'Thiếu hoặc có nhiều nguồn lệnh tại ' + topic
        if (self.scan_stamp is None or not -.05 <= now-self.scan_stamp <= LASER_TIMEOUT_S
                or wall-self.scan_wall > LASER_TIMEOUT_S):
            return False, 'Laser /scan_filtered chưa có dữ liệu mới hợp lệ'
        return True, ''


class PalletAlignmentNode(Node):
    def __init__(self, config=AlignmentConfig(), execute=False, controller=None):
        super().__init__('pallet_alignment')
        if execute and config.allow_reverse:
            raise ValueError('Lượt thử thật đầu chưa hỗ trợ lùi; đặt allow_reverse: false')
        self.core = controller if controller is not None else AlignmentController(config)
        self.execute = execute
        self.pending_target = None
        self.last_odom_wall = None
        self.last_publish = 0.0
        self.recognition = None
        self.recognition_received = 0.
        self.yolo_enabled = None
        self.yolo_received = 0.
        self.preparing = False
        self.start_epoch = 0
        self.measure_after_s = 0.
        self.waiters = []
        self.start_waiter = None
        self.proposed = self.create_publisher(Twist, '/pallet/alignment/proposed_cmd_vel', 1)
        # No publisher anywhere in the motor command chain in default mode.
        self.command = self.create_publisher(Twist, COMMAND, 1) if execute else None
        self.status = self.create_publisher(String, '/pallet/alignment/status', 1)
        self.goal_pub = self.create_publisher(PoseStamped, '/pallet/alignment/goal', 1)
        self.marker_pub = self.create_publisher(MarkerArray, '/pallet/alignment/markers', 1)
        self.guard = MotionGuard(self) if execute else None
        self.create_subscription(Odometry, '/odom', self.on_odom,
            QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(PalletTarget, '/pallet/target', self.on_target, 1)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/pallet/recognition/status', self.on_recognition, latched)
        self.create_subscription(Bool, '/pallet/enabled', self.on_yolo_enabled, latched)
        self.prepare_client = self.create_client(Trigger, '/pallet/prepare_alignment')
        self.control_group = ReentrantCallbackGroup()
        self.create_service(SetBool, '/pallet/alignment/set_enabled', self.set_enabled,
                            callback_group=self.control_group)
        self.create_timer(.05, self.tick)
        self.get_logger().info('Mode: ' + ('EXECUTE, chờ bấm bắt đầu' if execute else
                                         'DRY RUN, chỉ xuất lệnh dự kiến; không điều khiển xe'))

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def zero(self):
        self.proposed.publish(Twist())
        if self.command:
            self.command.publish(Twist())

    def reject(self, reason):
        self.pending_target = None
        self.core.invalidate_target(reason)
        self.zero()

    def on_recognition(self, msg):
        try:
            state = json.loads(msg.data)
            if not all(type(state[k]) is bool for k in ('enabled', 'busy', 'locked', 'measurement_ready')):
                raise ValueError('Invalid recognition status')
            if state.get('phase') not in ('ACQUIRING', 'TRACKING', 'PAUSED', 'ACQUIRE_FAILED', 'LOST'):
                raise ValueError('Invalid recognition phase')
            self.recognition = state
            self.recognition_received = time.monotonic()
        except (ValueError, KeyError, TypeError):
            self.recognition = None
        ready, reason = self.recognition_ready(require_measurement=not self.core.active)
        if self.core.active and not ready:
            self.core.stop(reason, 'BLOCKED')
            self.zero()

    def on_yolo_enabled(self, msg):
        self.yolo_enabled, self.yolo_received = msg.data, time.monotonic()
        if msg.data and self.core.active:
            self.core.stop('YOLO được bật; đã dừng căn xe', 'BLOCKED')
            self.zero()

    def recognition_ready(self, require_measurement=True):
        wall = time.monotonic()
        if (self.recognition is None or wall-self.recognition_received > 1.
                or self.yolo_enabled is None or wall-self.yolo_received > 1.):
            return False, 'Chưa có trạng thái mới xác nhận YOLO đã tắt'
        if self.yolo_enabled or self.recognition['enabled'] or self.recognition['busy']:
            return False, 'Chờ YOLO tắt và xử lý xong khung hình đang nhận diện'
        if not self.recognition['locked'] or self.recognition['phase'] != 'TRACKING':
            return False, 'Chưa nhận diện rõ pallet và có phép đo RGB + Depth ổn định'
        if require_measurement and not self.recognition['measurement_ready']:
            return False,rgbd_wait_reason(self.recognition.get('rgbd_health'))
        return True, ''

    def on_odom(self, msg):
        p, q, t = msg.pose.pose.position, msg.pose.pose.orientation, msg.twist.twist
        values = [p.x, p.y, q.x, q.y, q.z, q.w, t.linear.x, t.angular.z]
        stamp = stamp_seconds(msg.header.stamp)
        norm = sum(v*v for v in (q.x, q.y, q.z, q.w))
        age=self.now()-stamp
        odom_issue=''
        if msg.header.frame_id != 'odom' or msg.child_frame_id not in ('base_footprint','base_link'):
            odom_issue=f'Odometry sai hệ tọa độ: {msg.header.frame_id} → {msg.child_frame_id}'
        elif not all(math.isfinite(v) for v in values) or abs(norm-1)>.02:
            odom_issue='Odometry có số không hợp lệ hoặc quaternion sai'
        elif not -.05<=age<=self.core.config.odom_timeout_s:
            odom_issue=f'Odometry quá hạn/lệch thời gian: {age*1000:.0f} ms'
        if odom_issue:
            self.core.stop(odom_issue, 'LOST_DATA')
            self.core.odom.samples.clear()
            self.reject(odom_issue)
            return
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        previous = self.core.odom.samples[-1] if self.core.odom.samples else None
        if previous and (stamp < previous.stamp_s or math.hypot(p.x-previous.x, p.y-previous.y) > .10
                         or abs(wrap(yaw-previous.yaw)) > .15):
            self.core.stop('Odometry bị nhảy hoặc reset; cần bắt đầu lại', 'LOST_DATA')
            self.core.odom.samples.clear()
            self.core.last_seen_stamp = None
            self.reject('Cần phép đo mới sau khi odometry thay đổi')
        self.core.odom.add(OdomSample(stamp, p.x, p.y, yaw, t.linear.x, t.angular.z))
        self.last_odom_wall = time.monotonic()
        self.consume_target()

    def on_target(self, msg):
        if stamp_seconds(msg.header.stamp) < self.measure_after_s:
            return
        if self.execute and 'target_bias_applied' not in msg.status:
            self.reject('Chạy thật cần phép đo đã áp dụng hồ sơ bù khoảng cách/ngang/góc')
            return
        stamp = stamp_seconds(msg.header.stamp)
        c = self.core.config
        values = [msg.chassis_front_forward_m, msg.opening_lateral_m, msg.face_yaw_rad,
                  msg.entry_heading_rad, msg.rgb_depth_delta_ms, msg.opening_width_m,
                  msg.leg_center_separation_m, msg.camera_forward_m]
        if (not msg.valid or msg.header.frame_id != FRAME or not all(math.isfinite(v) for v in values)
                or not -.05 <= self.now()-stamp <= c.target_timeout_s
                or msg.camera_forward_m <= 0 or abs(msg.rgb_depth_delta_ms) > 50
                or min(msg.left_depth_samples, msg.right_depth_samples) < 60
                or not .172 <= msg.opening_width_m <= .332
                or not .15 <= msg.leg_center_separation_m <= .60
                or abs(wrap(msg.entry_heading_rad+msg.face_yaw_rad)) > .001):
            self.reject('RGB + Depth chưa hợp lệ: ' + msg.status)
            return
        # Capture and receipt times stay unchanged if waiting for interpolated odometry.
        self.pending_target = (msg, time.monotonic())
        self.consume_target()

    def consume_target(self):
        if self.pending_target is None:
            return
        msg, received = self.pending_target
        stamp = stamp_seconds(msg.header.stamp)
        if time.monotonic()-received > self.core.config.target_timeout_s:
            self.reject('Phép đo chờ odometry quá hạn')
            return
        if self.core.observe(stamp, received, msg.chassis_front_forward_m+self.core.config.chassis_front_m,
                             msg.opening_lateral_m, msg.entry_heading_rad):
            self.pending_target = None
            if self.core.confirmed < self.core.config.confirm_samples:
                self.zero()

    def interlock(self):
        wall = time.monotonic()
        if self.last_odom_wall is None or wall-self.last_odom_wall > self.core.config.odom_timeout_s:
            return False, 'Chưa có odometry mới'
        ready, reason = self.recognition_ready(require_measurement=not self.core.active)
        if not ready:
            return ready, reason
        return self.guard.ready(self.now(), wall) if self.guard else (True, '')

    async def set_enabled(self, request, response):
        if not request.data:
            self.start_epoch += 1
            self.preparing = False
            if self.start_waiter is not None and not self.start_waiter[0].done():
                self.start_waiter[0].set_result((False, 'Đã hủy bắt đầu căn chỉnh'))
            self.core.stop()
            self.zero()
            response.success, response.message = True, 'Đã dừng căn chỉnh'
        elif self.core.active or self.preparing:
            response.success, response.message = False, 'Căn chỉnh đang chạy'
        else:
            self.start_epoch += 1
            epoch = self.start_epoch
            self.preparing = True
            self.zero()
            self.publish_status()
            try:
                if not self.prepare_client.service_is_ready():
                    response.success, response.message = False, 'Chưa kết nối dịch vụ tắt YOLO'
                else:
                    source = self.prepare_client.call_async(Trigger.Request())
                    completion = Future(executor=self.executor)
                    self.waiters.append((source, completion, time.monotonic()+1.3))
                    result = await completion
                    if epoch != self.start_epoch:
                        response.success, response.message = False, 'Đã hủy yêu cầu căn chỉnh'
                    elif result is None or not result.success:
                        response.success, response.message = False, result.message if result else 'Chưa xác nhận YOLO dừng xong'
                    else:
                        # Require measurements captured AFTER the stop acknowledgement.
                        self.measure_after_s = self.now()
                        self.pending_target = None
                        self.core.invalidate_target('Đã tắt YOLO; chờ các phép đo mới')
                        deadline = time.monotonic()+START_DATA_TIMEOUT_S
                        while True:
                            ready_future = Future(executor=self.executor)
                            self.start_waiter = (ready_future, deadline)
                            ready, reason = await ready_future
                            if epoch != self.start_epoch:
                                ready, reason = False, 'Đã hủy yêu cầu căn chỉnh'
                                break
                            if ready:
                                # A camera callback may invalidate the target
                                # between timer completion and coroutine resume.
                                # Keep the SAME bounded, stationary handshake;
                                # never send another acquisition/start request.
                                ready, reason = self.interlock()
                                if ready:
                                    ready, reason = self.core.readiness(self.now(), time.monotonic())
                            if ready:
                                ready, reason = self.core.start(self.now(), time.monotonic())
                                break
                            if time.monotonic() >= deadline:
                                break
                        response.success, response.message = ready, reason
            finally:
                if epoch == self.start_epoch:
                    self.preparing = False
        self.publish_status()
        return response

    def tick(self):
        wall = time.monotonic()
        remaining = []
        for source, completion, deadline in self.waiters:
            if completion.done():
                continue
            if source.done():
                try:
                    completion.set_result(source.result())
                except Exception:
                    completion.set_result(None)
            elif wall >= deadline:
                self.prepare_client.remove_pending_request(source)
                source.cancel()
                completion.set_result(None)
            else:
                remaining.append((source, completion, deadline))
        self.waiters = remaining
        ready, reason = self.interlock()
        if self.start_waiter is not None:
            future, deadline = self.start_waiter
            data_ready, data_reason = self.core.readiness(self.now(), wall)
            if not future.done() and ((ready and data_ready) or wall >= deadline):
                future.set_result((ready and data_ready, reason if not ready else data_reason))
            if future.done():
                self.start_waiter = None
        # Missing odometry also blocks dry run. No invented fixed pose.
        self.core.tick(self.now(), time.monotonic(), ready, reason)
        self.before_command_publish()
        command = Twist()
        command.linear.x, command.angular.z = self.core.v, self.core.w
        self.proposed.publish(command)
        if self.command:
            self.command.publish(command)
        if time.monotonic()-self.last_publish >= .20:
            self.publish_status()

    def before_command_publish(self):
        """Optional additional command gate, before either publisher sees a command."""

    def status_details(self):
        return {}

    def publish_status(self):
        now, wall = self.now(), time.monotonic()
        self.last_publish = wall
        ready, reason = self.core.readiness(now, wall)
        safe, safety_reason = self.interlock()
        status = self.core.snapshot()
        status.update(self.status_details())
        status.update(execute=self.execute, ready=ready and safe and not self.preparing,
                      preparing=self.preparing,
                      ready_reason=reason if not ready else safety_reason,
                      lateral_tolerance_m=self.core.config.lateral_tolerance_m,
                      distance_tolerance_m=self.core.config.distance_tolerance_m,
                      heading_tolerance_deg=self.core.config.heading_tolerance_deg)
        if not ready or not safe:
            status['errors'] = status['goal'] = None
        self.status.publish(String(data=json.dumps(status, allow_nan=False)))
        markers = MarkerArray()
        if status['goal'] is not None:
            goal = PoseStamped()
            goal.header.frame_id, goal.header.stamp = 'odom', self.get_clock().now().to_msg()
            x, y, yaw = status['goal']
            goal.pose.position.x, goal.pose.position.y = x, y
            goal.pose.orientation.z, goal.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
            self.goal_pub.publish(goal)
            marker = Marker()
            marker.header = goal.header
            marker.ns, marker.id, marker.type, marker.action = 'pallet_alignment', 0, Marker.ARROW, Marker.ADD
            marker.pose.orientation.w = 1.
            # A staging controller can navigate from its odom snapshot while
            # the live camera target is temporarily outside the image.
            cx, cy, _ = self.core.target or getattr(self.core, 'frozen_target', None)
            marker.points = [Point(x=x, y=y, z=.05), Point(x=cx, y=cy, z=.05)]
            marker.scale.x, marker.scale.y, marker.scale.z = .025, .08, .10
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = .2, .9, .7, 1.
            marker.lifetime.sec, marker.lifetime.nanosec = 0, 400_000_000
            markers.markers.append(marker)
        else:
            marker = Marker(); marker.action = Marker.DELETEALL
            markers.markers.append(marker)
        self.marker_pub.publish(markers)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=ROOT/'config/pallet_alignment.yaml')
    parser.add_argument('--execute', action='store_true', help='Cho phép gửi lệnh qua collision monitor; vẫn cần bấm bắt đầu')
    args, ros_args = parser.parse_known_args()
    config = AlignmentConfig(**yaml.safe_load(args.config.read_text()))
    rclpy.init(args=ros_args)
    node = PalletAlignmentNode(config, execute=args.execute)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.core.stop()
        if rclpy.ok():
            node.zero()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
