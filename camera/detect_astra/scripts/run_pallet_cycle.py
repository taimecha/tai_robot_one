#!/usr/bin/env python3
"""One operator-started pallet-1 cycle, keeping hardware and localization alive."""
import argparse
from collections import deque
import fcntl
import json
import itertools
import math
import os
from pathlib import Path
import signal
import select
import sys
import subprocess
import time
import uuid
from concurrent.futures import ThreadPoolExecutor

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from rclpy.signals import SignalHandlerOptions
from action_msgs.msg import GoalStatusArray
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import Twist,PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.srv import ManageLifecycleNodes
from nav_msgs.msg import Odometry,OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Empty, String
from detected_pallets_interfaces.msg import PalletTarget
from std_srvs.srv import SetBool, Trigger
from std_srvs.srv import Empty as EmptyService
from rcl_interfaces.srv import GetParameters
from rclpy.parameter import parameter_value_to_python
from tf2_ros import Buffer, TransformListener, TransformException
import yaml
from pallet_cpu_budget import CpuBudget
from pallet_pickup_admission import SensorWindow
from pallet_laser_timing import LASER_TIMEOUT_S
from run_pallet_pickup import insertion_readiness
from run_pallet_delivery import station_navigation_ready
from pallet_acquisition import valid_target
from pallet_insertion_core import InsertionConfig

ROOT = Path(__file__).resolve().parents[1]
REPO = ROOT.parents[1]
NAV_NAMES = {'controller_server', 'planner_server', 'behavior_server',
             'velocity_smoother', 'collision_monitor', 'bt_navigator',
             'lifecycle_manager_navigation', 'nav_path_display_filter'}
FAIL_ALIGNMENT = {'TOO_CLOSE', 'NO_SAFE_GOAL', 'NAV_FAILED', 'BLOCKED', 'LOST_DATA',
                  'TIMEOUT', 'REACQUIRE_REQUIRED', 'LATERAL_NOT_REACHED', 'STOPPED', 'YAW_CORRECTED'}
ACQUISITION_ACK_TIMEOUT_S = 8.
BIAS_PROFILE = ROOT/'config/calibration/pallet_target_bias_2026_11_04_lateral.yaml'


def require_motion_calibration(path, allow_experimental=False):
    profile=yaml.safe_load(Path(path).read_text())
    if profile.get('motion_qualified') is False and allow_experimental:
        print('HIỆU CHỈNH THỬ NGHIỆM: chưa xác nhận sai số ngang dưới 1 cm; giữ mọi kiểm tra chuyển động.', flush=True)
        return
    if profile.get('motion_qualified') is False:
        raise RuntimeError('Hiệu chuẩn ngang chưa đạt kiểm tra ở tư thế khác; chỉ cho phép đo, chưa chạy xe/càng. '
                           'Cần hoàn tất kiểm tra hiệu chuẩn trước khi chạy chu trình.')


def transient_pickup_preview_issue(message):
    prefix='/pallet/handling/preview: '
    return message.startswith(prefix+'Chờ cửa sổ RGB-D ổn định 5 giây trước khi chạy: ') or message in {
        prefix+'Chưa có phép đo RGB + Depth mới; xe phải dừng',
        prefix+'Cần phép đo mới đã bù RGB-D',
        prefix+'RGB-D chưa ổn định đủ cửa sổ 5 giây',
        prefix+'Đang xác nhận các phép đo mới',
    }


class AcquisitionWindowExpired(RuntimeError):
    """An acknowledged session explicitly ended without a measurement lock."""


class ReturnToRecognition(RuntimeError):
    """A stopped, outside-pallet checkpoint needs a new approach."""


class RestartStaging(RuntimeError):
    """The stationary staging Nav2 stack did not become active."""


def process(pid):
    try:
        root = Path('/proc')/str(pid)
        args = root.joinpath('cmdline').read_bytes().decode().strip('\0').split('\0')
        # /proc/stat's comm may contain spaces or parentheses.
        fields = root.joinpath('stat').read_text().rsplit(')', 1)[1].split()
        return args, int(fields[1])
    except (OSError, ValueError, UnicodeError):
        return [], 0


def project_navigation_ancestor(pid):
    seen = set()
    while pid > 1 and pid not in seen:
        seen.add(pid)
        args, pid = process(pid)
        if ('launch' in args and any(Path(a).name == 'nav_real.launch.py' for a in args)
                and ('tai_robot_one' in args or str(REPO/'ros2_ws/launch/nav_real.launch.py') in args)):
            return True
    return False


def process_domain(pid):
    try:
        values = dict(item.split(b'=', 1) for item in
                      (Path('/proc')/str(pid)/'environ').read_bytes().split(b'\0') if b'=' in item)
        return values.get(b'ROS_DOMAIN_ID', b'0').decode()
    except (OSError, UnicodeError):
        return None


def navigation_processes():
    """Only global Nav2 children of this project's real-navigation launch."""
    found = []
    for path in Path('/proc').iterdir():
        if not path.name.isdecimal():
            continue
        pid = int(path.name)
        if process_domain(pid) != os.environ.get('ROS_DOMAIN_ID', '0'):
            continue
        args, _ = process(pid)
        names = [a.split(':=', 1)[1] for a in args if a.startswith('__node:=')]
        namespaces = [a.split(':=', 1)[1] for a in args if a.startswith('__ns:=')]
        if (len(names) != 1 or names[0] not in NAV_NAMES
                or any(ns != '/' for ns in namespaces)):
            continue
        name = names[0]
        executable = 'lifecycle_manager' if name == 'lifecycle_manager_navigation' else name
        if not any(Path(a).name == executable for a in args):
            continue
        if project_navigation_ancestor(pid):
            found.append((pid, name, args))
    return found


def recognition_ready(state):
    return (state.get('phase') == 'TRACKING' and state.get('locked') is True
            and state.get('enabled') is False and state.get('busy') is False
            and state.get('measurement_ready') is True
            and (state.get('rgbd_health') or {}).get('stable') is True)


def required_goal_names(pickup_only=False, already_at_recognition=False):
    if pickup_only:
        return () if already_at_recognition else ('recognition',)
    return ('recognition', 'dropoff')


def can_reacquire_after_arrival(state):
    distance=state.get('fork_plane_distance_m')
    return (state.get('state') == 'REACQUIRE_REQUIRED'
            and state.get('strategy') == 'nearest_normal_nav2'
            and (state.get('stationary_arrival_confirmed') is True
                 or state.get('stationary_trim_start_confirmed') is True)
            and state.get('nav_snapshot') is False
            and state.get('active') is False and state.get('preparing') is False
            and type(distance) in (int,float) and math.isfinite(distance) and distance >= 1.2)


def new_recognition_session(state, previous_session, requested_ros):
    """A retained old TRACKING sample cannot acknowledge a new request."""
    if not isinstance(state, dict):
        return False
    stamp = state.get('started_ros_s')
    return (type(state.get('session')) is int
            and state['session'] == previous_session+1
            and type(stamp) in (int, float) and math.isfinite(stamp)
            and stamp >= requested_ros
            and state.get('phase') in ('ACQUIRING', 'TRACKING', 'ACQUIRE_FAILED', 'LOST'))


def recognition_measurement(target, state, ros_now, chassis_front_m, fork_root_m,
                            fork_length_m=InsertionConfig.fork_length_m):
    """Report the same fresh, bias-corrected axes used by the alignment owner."""
    if target is None or not recognition_ready(state) or not valid_target(target,ros_now):
        return None
    stamp=target.header.stamp.sec+target.header.stamp.nanosec*1e-9
    started=state.get('started_ros_s')
    if (type(started) not in (int,float) or not math.isfinite(started) or stamp<started
            or 'target_bias_applied' not in target.status):
        return None
    forward=target.chassis_front_forward_m+chassis_front_m
    lateral=target.opening_lateral_m
    heading=target.entry_heading_rad
    tip_forward=forward-fork_root_m-fork_length_m
    return dict(session=state['session'],stamp_s=stamp,base_forward_m=forward,
        lateral_left_m=lateral,entry_heading_deg=math.degrees(heading),
        fork_tip_forward_gap_m=tip_forward,
        fork_tip_lateral_left_m=lateral,
        fork_tip_plane_gap_m=tip_forward*math.cos(heading)+lateral*math.sin(heading),
        fork_root_forward_gap_m=forward-fork_root_m,
        fork_root_plane_gap_m=(forward-fork_root_m)*math.cos(heading)+lateral*math.sin(heading),
        opening_width_m=target.opening_width_m)


def recognition_result_text(result):
    side='trái' if result['lateral_left_m']>0 else 'phải' if result['lateral_left_m']<0 else 'giữa'
    turn='trái' if result['entry_heading_deg']>0 else 'phải' if result['entry_heading_deg']<0 else 'không cần bù'
    return (f"KẾT QUẢ NHẬN DIỆN PALLET XANH — phiên {result['session']} (RGB-D đã bù):\n"
        f"  Gốc càng → mặt pallet: {result['fork_root_plane_gap_m']*100:.2f} cm vuông góc mặt; "
        f"{result['fork_root_forward_gap_m']*100:.2f} cm dọc xe.\n"
        f"  Từ giữa hai mũi càng → tâm lỗ: {result['fork_tip_forward_gap_m']*100:.2f} cm phía trước; "
        f"lệch ngang theo xe {abs(result['fork_tip_lateral_left_m'])*100:.2f} cm {side}.\n"
        f"  Góc cần quay bù: {result['entry_heading_deg']:+.2f}° ({turn}); "
        f"khe rộng đo được {result['opening_width_m']*100:.2f} cm.")


class Cycle(Node):
    def __init__(self):
        super().__init__('pallet_cycle_operator')
        self.states = {}
        self.quiet = deque(maxlen=150)
        self.children = {}
        self.foreground = None
        self.lease = False
        self.token = uuid.uuid4().hex
        self.logs = None
        self.trace_last=0.
        self.recognition_trace_last=0.
        self.recognition_trace_phase=None
        self.trace_error_reported=False
        self.map_matcher=None
        self.map_message=None
        self.localization_checks=0
        self.budget = None
        self.service_clients = {}
        self.pickup_run_id = None
        self.sensor_windows = {'odom': SensorWindow(.20), 'laser': SensorWindow(LASER_TIMEOUT_S)}
        geometry=yaml.safe_load((ROOT/'config/pallet_staging.yaml').read_text())
        self.chassis_front_m=float(geometry['chassis_front_m'])
        self.fork_root_m=float(geometry['fork_root_m'])
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.create_subscription(Odometry, '/odom', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(PalletTarget,'/pallet/target',lambda m:self.receive('target',m),1)
        self.create_subscription(LaserScan, '/scan_filtered',
                                 lambda m: self.receive('laser', m), qos_profile_sensor_data)
        self.create_subscription(String, '/lift/status', lambda m: self.receive('lift', m, True), 1)
        self.create_subscription(String, '/camera/floor_height/status',
                                 lambda m: self.receive('floor',m,True),1)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid,'/map',lambda m:self.receive('map',m),latched)
        self.create_subscription(PoseWithCovarianceStamped,'/amcl_pose',lambda m:self.receive('amcl',m),1)
        self.create_subscription(String,'/pallet/localization/health',lambda m:self.receive('localization',m,True),1)
        for key in ('recognition', 'alignment', 'handling'):
            self.create_subscription(String, f'/pallet/{key}/status',
                                     lambda m, k=key: self.receive(k, m, True),
                                     1 if key == 'alignment' else latched)
        self.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status',
                                 lambda m: self.receive('nav', m), latched)
        self.heartbeat = self.create_publisher(String, '/pallet/cycle/heartbeat', 1)
        self.pickup_heartbeat = self.create_publisher(String, '/pallet/handling/heartbeat', 1)
        self.stop_base = self.create_publisher(Twist, '/cmd_vel_nav', 1)
        self.stop_lift = self.create_publisher(Empty, '/lift/stop', 1)
        self.initial_pose=self.create_publisher(PoseWithCovarianceStamped,'/initialpose',10)
        self.create_timer(.1, self.beat)
        # Discover response readers before the detector creates its services.
        # A just-created FastDDS client may discover requests before replies.
        for typ, name in ((SetBool, '/pallet/set_enabled'),
                          (SetBool, '/pallet/alignment/set_enabled'),
                          (SetBool, '/pallet/handling/start'),
                          (Trigger, '/pallet/handling/preview'),
                          (Trigger, '/pallet/handling/stop'),
                          (GetParameters, '/scan_self_filter/get_parameters'),
                          (ManageLifecycleNodes, '/lifecycle_manager_navigation/manage_nodes')):
            self.service_client(typ, name)

    def beat(self):
        if self.lease:
            self.heartbeat.publish(String(data=self.token))
            if self.pickup_run_id:
                self.pickup_heartbeat.publish(String(data=self.pickup_run_id))

    def receive(self, key, msg, decode=False):
        try:
            value = json.loads(msg.data) if decode else msg
            if decode and not isinstance(value, dict):
                raise ValueError('Status must be an object')
            self.states[key] = (value, time.monotonic())
            if key=='recognition' and self.logs is not None and (
                    time.monotonic()-self.recognition_trace_last>=1.
                    or value.get('phase')!=self.recognition_trace_phase):
                self.recognition_trace_last=time.monotonic()
                self.recognition_trace_phase=value.get('phase')
                try:
                    with (self.logs/'recognition.jsonl').open('a') as output:
                        output.write(json.dumps(dict(ros_s=self.get_clock().now().nanoseconds/1e9,
                            recognition=value),ensure_ascii=False)+'\n')
                except OSError as exc:
                    if not self.trace_error_reported:
                        self.get_logger().warning('Không ghi được trace nhận diện: '+str(exc))
                        self.trace_error_reported=True
            if key=='alignment' and self.logs is not None and time.monotonic()-self.trace_last>=.2:
                self.trace_last=time.monotonic()
                row=dict(ros_s=self.get_clock().now().nanoseconds/1e9,alignment=value)
                if 'odom' in self.states:
                    od=self.states['odom'][0]
                    row['odom']=dict(stamp_s=od.header.stamp.sec+od.header.stamp.nanosec*1e-9,
                        v=od.twist.twist.linear.x,w=od.twist.twist.angular.z,
                        x=od.pose.pose.position.x,y=od.pose.pose.position.y,
                        yaw_rad=2*math.atan2(od.pose.pose.orientation.z,od.pose.pose.orientation.w))
                try:
                    with (self.logs/'motion.jsonl').open('a') as output:
                        output.write(json.dumps(row,ensure_ascii=False)+'\n')
                except OSError as exc:
                    if not self.trace_error_reported:
                        self.get_logger().warning('Không ghi được trace: '+str(exc))
                        self.trace_error_reported=True
            if key in self.sensor_windows:
                stamp = value.header.stamp
                self.sensor_windows[key].observe(stamp.sec+stamp.nanosec*1e-9,
                    time.monotonic(), self.get_clock().now().nanoseconds/1e9)
        except (ValueError, TypeError):
            self.states.pop(key, None)

    def on_odom(self, msg):
        self.receive('odom', msg)
        self.quiet.append((time.monotonic(), abs(msg.twist.twist.linear.x) <= .003
                           and abs(msg.twist.twist.angular.z) <= .015))

    def pump(self, seconds=.05):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=.02)

    def wait(self, predicate, timeout, label, watch=()):
        end = time.monotonic()+timeout
        while time.monotonic() < end:
            self.pump()
            for name in watch:
                if self.children[name].poll() is not None:
                    raise RuntimeError(f'{name} đã thoát; xem log {self.logs}')
            if predicate():
                return
        raise RuntimeError('Quá thời gian: '+label)

    def fresh(self, key, after=0.):
        value, stamp = self.states.get(key, ({}, 0.))
        return value if stamp > after and 0 <= time.monotonic()-stamp <= .8 else None

    def still(self):
        recent = [(t, ok) for t, ok in self.quiet if t >= time.monotonic()-1.]
        return bool(recent and recent[-1][0]-recent[0][0] >= .8 and all(ok for _, ok in recent))

    def camera_floor_issue(self):
        state,received = self.states.get('floor',({},0.))
        if not 0<=time.monotonic()-received<=1.5:
            return 'Chưa có trạng thái mới của bộ xác nhận mặt sàn camera'
        if state.get('qualified') is not True:
            return 'Camera chưa xác nhận mặt sàn: '+str(state.get('reason','đang chờ'))
        return ''

    def sensors(self, empty=False):
        now = self.get_clock().now().nanoseconds/1e9
        for key in ('odom', 'laser', 'lift'):
            value, wall = self.states.get(key, (None, 0.))
            if value is None or not 0 <= time.monotonic()-wall <= .5:
                raise RuntimeError('Mất dữ liệu mới: '+key)
            if key != 'lift':
                stamp = value.header.stamp
                if not -.05 <= now-stamp.sec-stamp.nanosec*1e-9 <= .5:
                    raise RuntimeError('Dữ liệu quá hạn: '+key)
        lift = self.states['lift'][0]
        if (lift.get('homed') is not True or lift.get('serial_connected') is not True
                or lift.get('fault') != 'NONE' or lift.get('state') != 'IDLE'
                or not math.isfinite(lift.get('position', math.nan))
                or not math.isfinite(lift.get('velocity', math.nan))
                or abs(lift['velocity']) > .002):
            raise RuntimeError('Càng chưa đứng yên/sẵn sàng; không tự HOME')
        if empty and not 0 <= lift['position'] <= .01:
            raise RuntimeError('Cần lấy pallet khỏi càng và hạ càng trước khi chạy chu trình mới')
        if (getattr(self,'local_pickup',False)
                or getattr(self,'awaiting_global_localization',False)):return lift
        transform = self.buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
        stamp = transform.header.stamp
        if not -.5 <= now-stamp.sec-stamp.nanosec*1e-9 <= 1.5:
            raise RuntimeError('Định vị bản đồ quá hạn')
        return lift

    def active_nav(self):
        status = self.states.get('nav', (GoalStatusArray(), 0.))[0]
        return any(s.status in (1, 2, 3) for s in status.status_list)

    def auto_localize_map(self):
        from pallet_auto_localization import yaw,summary_observation,PoseStability,navigation_stability_window
        from pallet_map_alignment import MapAlignment,alignment_ok
        import numpy as np
        print('[0/6] Kiểm tra vị trí AMCL hiện tại; chỉ tự định vị lại khi vị trí chưa đạt.',flush=True)
        self.wait(lambda:'map' in self.states and self.still(),25.,'map và xe đứng yên cho AMCL')
        self.sensors(empty=True)
        if self.active_nav():raise RuntimeError('Có goal đang chạy; không reset định vị')
        grid=self.states['map'][0];q=grid.info.origin.orientation
        if grid.header.frame_id!='map' or abs(q.x)+abs(q.y)+abs(q.z)>.001 or abs(q.w-1)>.001:
            raise RuntimeError('Bản đồ không đúng hệ map/phẳng')
        matcher=MapAlignment(np.asarray(grid.data).reshape(grid.info.height,grid.info.width),
                             grid.info.resolution,(grid.info.origin.position.x,grid.info.origin.position.y))
        health_publishers=self.get_publishers_info_by_topic('/pallet/localization/health')
        if len(health_publishers)>1:raise RuntimeError('Có nhiều bộ kiểm tra AMCL cạnh tranh')
        owned_health=not health_publishers
        if owned_health:
            self.start('localization_health',[os.environ.get('PALLET_PYTHON',str(ROOT/'.venv/bin/python')),
                str(ROOT/'scripts/pallet_amcl_health.py')])
        self.wait(lambda:len(self.get_publishers_info_by_topic('/pallet/localization/health'))==1,
                  12.,'bộ giải mã AMCL riêng')
        requested_ros=self.get_clock().now().nanoseconds/1e9
        started=time.monotonic();next_update=[started];last_reason=['AMCL chưa có pose mới'];last_print=[0.]
        report={'method':'amcl_reuse_if_healthy','passed':False,'global_reset':False,'requested_ros_s':requested_ros}
        coarse=PoseStability(2.,.10,math.radians(4),4)
        strict=navigation_stability_window()
        report['stability_limits']={'duration_s':strict.duration,'position_span_m':strict.distance,
            'heading_span_deg':math.degrees(strict.angle),'minimum_samples':strict.minimum_samples}
        def qualify(window,refined=False):
            now=time.monotonic();ros=self.get_clock().now().nanoseconds/1e9
            if now-last_print[0]>=2.:
                prefix='Tự định vị: ' if report['global_reset'] else 'Kiểm tra AMCL: '
                print(prefix+last_reason[0],flush=True);last_print[0]=now
            self.sensors(empty=True)
            odom=self.states['odom'][0].twist.twist
            if (self.active_nav() or abs(odom.linear.x)>.003 or abs(odom.angular.z)>.015
                    or any(not quiet for stamp,quiet in self.quiet if stamp>=now-1.)):
                raise RuntimeError('Xe/goal chuyển động trong khi tự định vị')
            if not self.still():
                window.clear();last_reason[0]='Chờ đủ cửa sổ odom đứng yên mới';return False
            if now>=next_update[0]:
                self.call(EmptyService,'/request_nomotion_update',EmptyService.Request(),3.)
                next_update[0]=time.monotonic()+.25
            summary=self.fresh('localization',started)
            if summary is None:
                window.clear();last_reason[0]='Chờ bản tóm tắt AMCL mới';return False
            # The worker owns the large cloud decode. The supervisor receives
            # only a small packet and keeps servicing laser/odom callbacks.
            ros=self.get_clock().now().nanoseconds/1e9
            stamp,pose,issue=summary_observation(summary,requested_ros,ros,refined)
            if stamp is None:
                window.clear();last_reason[0]=issue;return False
            if stamp<=window.last_stamp:return False
            report['amcl_health_compute_s']=summary['compute_s']
            if not issue and refined:
                scan=self.states['laser'][0]
                tf=self.buffer.lookup_transform('base_footprint',scan.header.frame_id,
                        rclpy.time.Time.from_msg(scan.header.stamp)).transform
                a=yaw(tf.rotation)
                ranges=np.asarray(scan.ranges,float)
                angles=scan.angle_min+np.arange(len(ranges))*scan.angle_increment
                valid=np.isfinite(ranges)&(ranges>max(.2,scan.range_min))&(ranges<min(5.5,scan.range_max))
                x=ranges[valid]*np.cos(angles[valid]);y=ranges[valid]*np.sin(angles[valid])
                points=np.column_stack((tf.translation.x+x*np.cos(a)-y*np.sin(a),
                                        tf.translation.y+x*np.sin(a)+y*np.cos(a)))
                metrics=matcher.metrics(pose,points);report['scan_metrics']=metrics
                mapped=self.buffer.lookup_transform('map','base_footprint',rclpy.time.Time())
                transform=mapped.transform
                tf_age=ros-mapped.header.stamp.sec-mapped.header.stamp.nanosec*1e-9
                mismatch=(math.hypot(transform.translation.x-pose[0],transform.translation.y-pose[1])>.03
                          or abs(math.atan2(math.sin(yaw(transform.rotation)-pose[2]),
                                           math.cos(yaw(transform.rotation)-pose[2])))>math.radians(2))
                if not -.5<=tf_age<=1.5:issue='TF map quá hạn'
                elif not alignment_ok(metrics):issue='Laser chưa khớp bản đồ'
                elif mismatch:issue='TF map chưa khớp pose AMCL mới'
            last_reason[0]=issue or f'Chờ pose/laser ổn định liên tục {window.duration:g} giây'
            report.update(pose=list(pose),reason=last_reason[0])
            return window.observe(stamp,time.monotonic(),pose,not issue)
        try:
            # Request fresh stationary AMCL updates without scattering its
            # particles. No disk flag: a reboot, stale pose, moved robot or
            # scan/map mismatch must qualify again from live data.
            reuse_until=time.monotonic()+12.
            reuse_ok=[False]
            def reuse_ready():
                try:reuse_ok[0]=qualify(strict,True)
                except TransformException:
                    strict.clear();last_reason[0]='Chưa có TF định vị hợp lệ'
                return reuse_ok[0] or time.monotonic()>=reuse_until
            self.wait(reuse_ready,16.,'kiểm tra vị trí AMCL đang dùng')
            if reuse_ok[0]:
                report['passed']=True
                print('GIỮ ĐỊNH VỊ: AMCL/laser vẫn khớp và ổn định; không reset vị trí.',flush=True)
                return
            report['reuse_failure_reason']=last_reason[0]
            print('Vị trí chưa đạt: '+last_reason[0]+'; bắt đầu tự định vị lại.',flush=True)
            requested_ros=self.get_clock().now().nanoseconds/1e9
            self.call(EmptyService,'/reinitialize_global_localization',EmptyService.Request(),15.)
            started=time.monotonic();next_update[0]=started
            coarse.clear();strict=navigation_stability_window()
            report.update(method='amcl_global_stationary',global_reset=True,requested_ros_s=requested_ros)
            self.wait(lambda:qualify(coarse),90.,'AMCL tìm vị trí toàn cục khi đứng yên')
            # Refine only the AMCL-selected basin, then validate independent
            # fresh scans and the global particle distribution before motion.
            self.verify_localization()
            self.wait(lambda:qualify(strict,True),35.,f'AMCL/laser khớp và ổn định {strict.duration:g} giây')
            report['passed']=True
            print('ĐỊNH VỊ ĐẠT: AMCL/laser khớp bản đồ và pose ổn định; tiếp tục chu trình.',flush=True)
        except Exception as exc:
            report.update(error=str(exc),reason=last_reason[0]);raise
        finally:
            try:(self.logs/'auto_localization.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
            finally:
                if owned_health:self.stop_child('localization_health')

    def verify_localization(self):
        """Match fresh laser to the map while stationary, before saved goals."""
        import numpy as np
        from pallet_map_alignment import MapAlignment,alignment_ok,proposal_issue
        self.wait(lambda:'map' in self.states,10.,'bản đồ để kiểm tra định vị')
        self.wait(self.still,8.,'xe đứng yên trước kiểm tra laser/bản đồ')
        self.sensors()
        if self.active_nav():raise RuntimeError('Có goal đang chạy; chưa tự sửa định vị')
        msg=self.states['map'][0]
        q=msg.info.origin.orientation
        if msg.header.frame_id!='map' or abs(q.x)+abs(q.y)+abs(q.z)>.001 or abs(q.w-1)>.001:
            raise RuntimeError('Bản đồ không đúng hệ map/phẳng')
        if msg is not self.map_message:
            self.map_matcher=MapAlignment(np.asarray(msg.data).reshape(msg.info.height,msg.info.width),
                msg.info.resolution,(msg.info.origin.position.x,msg.info.origin.position.y))
            self.map_message=msg
        matcher=self.map_matcher
        def capture():
            self.sensors()
            scan=self.states['laser'][0]
            now=self.get_clock().now().nanoseconds/1e9
            if now-scan.header.stamp.sec-scan.header.stamp.nanosec*1e-9>LASER_TIMEOUT_S:
                raise RuntimeError('Laser quá hạn trước khớp bản đồ')
            tf=self.buffer.lookup_transform('base_footprint',scan.header.frame_id,rclpy.time.Time()).transform
            q=tf.rotation;a=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            r=np.asarray(scan.ranges,float);angles=scan.angle_min+np.arange(len(r))*scan.angle_increment
            valid=np.isfinite(r)&(r>max(.2,scan.range_min))&(r<min(5.5,scan.range_max))
            x=r[valid]*np.cos(angles[valid]);y=r[valid]*np.sin(angles[valid])
            points=np.column_stack((tf.translation.x+x*np.cos(a)-y*np.sin(a),
                                    tf.translation.y+x*np.sin(a)+y*np.cos(a)))
            tf=self.buffer.lookup_transform('map','base_footprint',rclpy.time.Time()).transform
            q=tf.rotation;a=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            return [tf.translation.x,tf.translation.y,a],points
        pose,points=capture();before=matcher.metrics(pose,points)
        self.localization_checks+=1
        report=dict(before=before,original_pose=pose,corrected=False)
        def save():
            if self.logs is not None:
                (self.logs/f'localization_{self.localization_checks}.json').write_text(
                    json.dumps(report,ensure_ascii=False,indent=2))
        if alignment_ok(before):save();return
        if any(self.get_publishers_info_by_topic(topic) for topic in
               ('/pallet/alignment/status','/pallet/handling/status')):
            raise RuntimeError('Định vị lệch khi còn bộ căn/đưa càng; giữ xe đứng yên')
        print(f'Laser/bản đồ lệch (trung vị {before["median_m"]*100:.1f} cm); '
              'giữ xe đứng yên, tính lại pose gần hiện tại.',flush=True)
        pool=ThreadPoolExecutor(max_workers=1)
        future=pool.submit(matcher.propose,pose,points)
        try:
            self.wait(future.done,35.,'tính pose khớp laser/bản đồ')
            solutions=future.result()
        finally:pool.shutdown(wait=False,cancel_futures=True)
        self.wait(self.still,5.,'xe đứng yên sau tính pose')
        self.sensors()
        if self.active_nav():raise RuntimeError('Nav2 có goal mới; không sửa định vị')
        _,holdout=capture()
        metrics=matcher.metrics(solutions[0]['pose'],holdout) if solutions else {}
        issue=proposal_issue(before,solutions,metrics)
        report.update(solutions=solutions,holdout=metrics)
        if issue:save();raise RuntimeError(issue)
        p=solutions[0]['pose'];initial=PoseWithCovarianceStamped();initial.header.frame_id='map'
        initial.pose.pose.position.x,initial.pose.pose.position.y=p[:2]
        initial.pose.pose.orientation.z=math.sin(p[2]/2);initial.pose.pose.orientation.w=math.cos(p[2]/2)
        covariance=[0.]*36;covariance[0]=covariance[7]=.05**2;covariance[35]=math.radians(3)**2
        initial.pose.covariance=covariance
        self.initial_pose.publish(initial)
        self.pump(1.)
        client=self.service_client(EmptyService,'/request_nomotion_update')
        for _ in range(3):
            if client.service_is_ready():self.call(EmptyService,'/request_nomotion_update',EmptyService.Request(),2.)
            self.pump(.3)
        def accepted():
            self.sensors()
            p,pts=capture();report['after_pose']=p;report['after']=matcher.metrics(p,pts)
            return self.still() and alignment_ok(report['after'])
        try:self.wait(accepted,10.,'AMCL nhận pose và scan khớp bản đồ')
        finally:save()
        report['corrected']=True;save()
        print(f'Định vị đã khớp lại, trung vị {report["after"]["median_m"]*100:.1f} cm; xe vẫn đứng yên.',flush=True)

    def service_client(self, typ, name):
        key = (typ, name)
        if key not in self.service_clients:
            self.service_clients[key] = self.create_client(typ, name)
        return self.service_clients[key]

    def call(self, typ, name, request, timeout=8.):
        client = self.service_client(typ, name)
        self.wait(client.service_is_ready, timeout, 'dịch vụ '+name)
        future = client.call_async(request)
        try:
            self.wait(future.done, timeout, 'phản hồi '+name)
            response = future.result()
            if hasattr(response, 'success') and not response.success:
                raise RuntimeError(name+': '+getattr(response, 'message', 'từ chối'))
            return response
        finally:
            if not future.done():
                client.remove_pending_request(future)
                future.cancel()

    def start(self, name, command):
        env = {**os.environ, 'PALLET_CYCLE_TOKEN': self.token, 'PYTHONUNBUFFERED': '1'}
        if self.budget and self.budget.enabled:
            env['PALLET_COMPUTE_CPUS'] = ','.join(map(str, sorted(self.budget.compute)))
        with (self.logs/f'{name}.log').open('a') as output:
            # Staging's handler and Collision Monitor keep access to all CPUs;
            # its two costmap/planner/controller processes use the compute set.
            command = self.budget.command(command) if self.budget and name != 'staging' else command
            self.children[name] = subprocess.Popen(command, cwd=ROOT, env=env,
                start_new_session=True, stdout=output, stderr=subprocess.STDOUT)

    def stop_child(self, name):
        child = self.children.get(name)
        if child is None or child.poll() is not None:
            return
        os.killpg(child.pid, signal.SIGINT)
        try:
            self.wait(lambda: child.poll() is not None, 12., 'dừng '+name)
        except RuntimeError:
            os.killpg(child.pid, signal.SIGKILL)
            self.wait(lambda: child.poll() is not None, 2., 'kết thúc '+name)

    def operator(self, helper, operation, timeout, watch=(), extra_args=()):
        if helper=='pickup' and operation=='insert-and-lift' and not extra_args:
            return self.pickup_insert_and_lift(timeout,watch)
        env = {**os.environ, 'PALLET_CYCLE_PARENT_PID': str(os.getpid()),
               'PYTHONUNBUFFERED': '1'}
        self.foreground = subprocess.Popen(['bash', str(ROOT/f'scripts/open_pallet_{helper}.sh'),
            operation, *extra_args], cwd=ROOT, env=env, start_new_session=True)
        self.wait(lambda: self.foreground.poll() is not None, timeout,
                  helper+' '+operation, watch)
        code = self.foreground.returncode
        self.foreground = None
        if code:
            raise RuntimeError(f'Bước {helper} {operation} lỗi ({code}); không chạy bước kế tiếp')

    def pickup_insert_and_lift(self,timeout,watch=()):
        """Keep the warmed executor/sensor windows across alignment handoff.

        Starting another Python/ROS process here can starve odometry on the
        Pi. The same authoritative server still previews and starts once;
        no alignment certificate or sensor freshness check is waived.
        """
        if not self.lease:
            raise RuntimeError('Chưa có heartbeat chu trình để bàn giao đưa càng')
        ready_since=[None]
        def ready():
            wall=time.monotonic()
            ok,reason=insertion_readiness(self.states,wall)
            if ok:
                ros=self.get_clock().now().nanoseconds/1e9
                ok=all(window.ready(wall,ros) for window in self.sensor_windows.values())
            ready_since[0]=(ready_since[0] if ready_since[0] is not None else wall) if ok else None
            return ready_since[0] is not None and wall-ready_since[0]>=.4
        self.wait(ready,15.,'bàn giao đường checkpoint/ALIGNED và dữ liệu mới trước tiếp cận',watch)
        preflight_path=self.logs/'pickup_preflight.json'
        try:
            while True:
                try:
                    preview=self.call(Trigger,'/pallet/handling/preview',Trigger.Request(),8.)
                    break
                except RuntimeError as exc:
                    # Retry only a confirmed read-only refusal for missing RGB-D.
                    # Never replay the start/lift RPC or hide a geometry failure.
                    if not transient_pickup_preview_issue(str(exc)):
                        raise
                    self.sensors(empty=True)
                    if not self.still():
                        raise RuntimeError('Xe không đứng yên khi chờ phép đo preview mới') from exc
                    print('Preview thiếu RGB-D mới; giữ dừng, chờ dữ liệu và kiểm tra lại tại chỗ. Ctrl+C để hủy.', flush=True)
                    self.wait(lambda: self.still(), 3., 'xe giữ dừng trước kiểm tra lại preview', watch)
                    self.pump(.5)
            preflight_path.write_text(preview.message+'\n')
            try:
                geometry=json.loads(preview.message).get('reference',{}).get('approach_geometry')
                if geometry:
                    goal=geometry['goal_odom']
                    print(f"QUỸ ĐẠO TIẾP CẬN: quay theo xe {geometry['path_turn_deg']:+.2f}°, "
                          f"tiến {geometry['travel_m']*100:.2f} cm; điểm dừng odom "
                          f"({goal[0]:.4f}, {goal[1]:.4f}), "
                          f"lệch trục đích {geometry['goal_axis_cross_m']*1000:.2f} mm.",flush=True)
            except (ValueError,KeyError,TypeError):
                pass  # Diagnostics never authorize or alter a motion plan.

        except Exception as exc:
            preflight_path.write_text(json.dumps(dict(passed=False,reason=str(exc)),ensure_ascii=False)+'\n')
            raise
        # Preview is read-only; recheck fresh inputs before the ONE start RPC.
        self.wait(ready,5.,'dữ liệu mới sau kiểm tra quỹ đạo',watch)
        requested=False;finished=False
        try:
            request=SetBool.Request();request.data=False
            requested=True
            response=self.call(SetBool,'/pallet/handling/start',request,8.)
            self.pickup_run_id=response.message
            self.pickup_heartbeat.publish(String(data=self.pickup_run_id))
            started=time.monotonic()
            checkpoint_refreshed=[None]
            def complete(expected='COMPLETE'):
                status=self.fresh('handling',started)
                if not status or status.get('run_id')!=self.pickup_run_id:
                    if time.monotonic()-started>2.:
                        raise RuntimeError('Mất trạng thái mới/đúng lượt đưa càng; dừng')
                    return False
                if (status.get('active') is True and status.get('phase')=='CHECKPOINT_RECOGNITION'
                        and status.get('checkpoint_refresh_seq',0)!=checkpoint_refreshed[0]):
                    refresh_seq=status.get('checkpoint_refresh_seq',0)
                    self.wait(self.still,5.,'xe dừng tại checkpoint để nhận diện mới',watch)
                    self.sensors()
                    print('Xe đã dừng ngoài pallet; nhận diện MỚI hai chân tại checkpoint rồi tắt YOLO.',flush=True)
                    self.acquire(watch)
                    self.call(Trigger,'/pallet/handling/finish_checkpoint_recognition',Trigger.Request(),8.)
                    checkpoint_refreshed[0]=refresh_seq
                    return False
                if status.get('active') is not False:
                    return False
                if status.get('state')!=expected:
                    if (expected=='INSERTED' and status.get('state')=='BLOCKED'
                            and status.get('phase')=='FINAL_ALIGN'
                            and (status.get('recovery_action')=='return_to_recognition'
                                 or status.get('reason') in (
                                'Góc pallet đổi quá nhiều ở checkpoint; cần căn lại từ ngoài',
                                'Lệch ngang tại checkpoint; cần căn lại từ ngoài'))):
                        raise ReturnToRecognition(status['reason'])
                    raise RuntimeError('Đưa càng/nâng dừng: '+str(status.get('reason')))
                return True
            # Each motion phase owns its timeout/progress checks. Final trim
            # may take longer while still converging; fresh status, heartbeat
            # and child supervision remain mandatory on every iteration.
            self.wait(lambda:complete('INSERTED'),math.inf,'đo lại, chốt đường thẳng và đưa càng',watch)
            self.confirm_lift(watch)
            response=self.call(Trigger,'/pallet/handling/lift',Trigger.Request(),8.)
            self.pickup_run_id=response.message
            self.pickup_heartbeat.publish(String(data=self.pickup_run_id))
            started=time.monotonic()
            self.wait(complete,35.,'nâng đạt COMPLETE',watch)
            finished=True
        finally:
            self.pickup_run_id=None
            if requested and not finished:
                self.call(Trigger,'/pallet/handling/stop',Trigger.Request(),3.)

    def confirm_lift(self,watch=()):
        print('ĐƯA CÀNG XONG, XE ĐÃ DỪNG. Kiểm tra hai càng nằm đúng lỗ, không tì chân/thanh '
              'pallet và vùng nâng trống. Gõ nang rồi Enter để nâng 8 cm; Ctrl+C để dừng.',flush=True)
        def approved():
            status=self.fresh('handling')
            if (not status or status.get('run_id')!=self.pickup_run_id
                    or status.get('state')!='INSERTED' or status.get('active') is not False):
                raise RuntimeError('Mất trạng thái đưa càng đã dừng; không nâng')
            self.sensors()
            if not self.still():raise RuntimeError('Xe đã dịch trong khi chờ kiểm tra; không nâng')
            readable,_,_=select.select([sys.stdin],[],[],0)
            if not readable:return False
            line=sys.stdin.readline()
            if not line:raise RuntimeError('Terminal không có đầu vào xác nhận; không nâng')
            if line.strip().lower()=='nang':return True
            print('Chưa nâng. Gõ nang sau khi kiểm tra, hoặc Ctrl+C để kết thúc.',flush=True)
            return False
        self.wait(approved,180.,'người vận hành kiểm tra trước nâng',watch)

    def command_owner(self, expected):
        infos = self.get_publishers_info_by_topic('/cmd_vel_safe')
        return len(infos) == 1 and infos[0].node_name == expected

    def ensure_navigation(self):
        infos = self.get_publishers_info_by_topic('/cmd_vel_safe')
        if not infos:
            # Only control servers are restarted. Hardware and AMCL must already be live.
            self.start('navigation', ['ros2', 'launch', str(REPO/'ros2_ws/launch/nav_real.launch.py'),
                'reuse_sensors:=true', 'start_localization:=false', 'start_navigation:=true',
                'restore_saved_pose:=false', 'use_camera:=true', 'localization_delay:=0.0',
                'map_file:='+str(REPO/'ros2_ws/maps/tai_map_real_edited.yaml')])
        elif not self.command_owner('collision_monitor'):
            raise RuntimeError('Đang có bộ điều khiển khác; không tranh lệnh xe')
        self.wait(lambda: self.command_owner('collision_monitor'), 25., 'Nav2 collision monitor')
        client = self.create_client(GetState, '/bt_navigator/get_state')
        pending = [None]
        def ready():
            # Keep one client/request alive while lifecycle activation is busy.
            # Recreating clients during activation can lose a late response.
            if pending[0] is not None:
                if not pending[0].done():
                    return False
                response = pending[0].result()
                pending[0] = None
                if response.current_state.id == 3:
                    return True
            if client.service_is_ready():
                pending[0] = client.call_async(GetState.Request())
            return False
        try:
            self.wait(ready, 35., 'Nav2 ACTIVE')
        finally:
            self.destroy_client(client)

    def stop_navigation(self):
        self.sensors()
        if not self.still() or self.active_nav():
            raise RuntimeError('Nav2/xe chưa dừng trước khi chuyển sang căn pallet')
        targets = navigation_processes()
        if len(targets) != len(NAV_NAMES) or {name for _, name, _ in targets} != NAV_NAMES:
            raise RuntimeError('Chưa xác minh đủ các tiến trình Nav2 của dự án; không tắt phần cứng')
        self.call(ManageLifecycleNodes, '/lifecycle_manager_navigation/manage_nodes',
                  ManageLifecycleNodes.Request(command=ManageLifecycleNodes.Request.PAUSE), 12.)
        # Stop only navigation's manager first so it cannot restart control servers.
        for manager in (True, False):
            selected = [p for p in targets if (p[1] == 'lifecycle_manager_navigation') == manager]
            for pid, _, args in selected:
                if process(pid)[0] == args:
                    os.kill(pid, signal.SIGINT)
            self.wait(lambda: all(process(pid)[0] != args for pid, _, args in selected),
                      10., 'dừng máy chủ Nav2')
        if 'navigation' in self.children:
            self.stop_child('navigation')
        self.wait(lambda: not self.get_publishers_info_by_topic('/cmd_vel_safe'),
                  10., 'gỡ nguồn vận tốc Nav2 cũ')

    def acquire(self, watch=('staging', 'perception')):
        attempt=0
        while True:
            try:
                self.acquire_once(watch)
                return
            except AcquisitionWindowExpired:
                attempt+=1
                def paused():
                    status=self.fresh('recognition')
                    return (status and status.get('phase') in ('ACQUIRE_FAILED','TRACKING','LOST')
                            and status.get('enabled') is False and status.get('busy') is False)
                # The final inference can still be draining when the timer
                # reports expiry. Wait for its explicit idle acknowledgement.
                self.wait(paused, 2., 'YOLO kết thúc phiên đã hết hạn', watch)
                state = self.fresh('recognition')
                alignment = self.fresh('alignment')
                handling = self.fresh('handling')
                checkpoint_wait=bool(handling and alignment
                    and getattr(self,'pickup_run_id',None)
                    and handling.get('run_id')==self.pickup_run_id
                    and handling.get('phase')=='CHECKPOINT_RECOGNITION'
                    and handling.get('active') is True
                    and alignment.get('perception_refresh_allowed') is True)
                if (not state or state.get('phase') not in ('ACQUIRE_FAILED','TRACKING','LOST')
                        or state.get('enabled') is not False or state.get('busy') is not False
                        or not alignment or (alignment.get('active') is not False and not checkpoint_wait)
                        or alignment.get('preparing') is not False or alignment.get('nav_snapshot') is True
                        or (handling and handling.get('active') is not False and not checkpoint_wait)):
                    raise
                self.wait(self.still, 5., 'xe đứng yên trước phiên nhận diện mới', watch)
                self.sensors(empty=True)
                print(f'Phiên nhận diện chưa đạt và YOLO đã tắt; thử lại tại chỗ lần {attempt}. '
                      'Chưa gửi chuyển động.', flush=True)

    def acquire_once(self, watch=('staging', 'perception')):
        client = self.service_client(SetBool, '/pallet/set_enabled')
        warm_since = [None]
        def detector_ready():
            state = self.fresh('recognition')
            ready = (client.service_is_ready() and state
                     and state.get('enabled') is False and state.get('busy') is False
                     and state.get('camera_ready') is True and type(state.get('session')) is int)
            if not ready:
                warm_since[0] = None
                return False
            if warm_since[0] is None:
                warm_since[0] = time.monotonic()
            return time.monotonic()-warm_since[0] >= .5
        self.wait(detector_ready, 30., 'model/camera và kênh nhận diện sẵn sàng', watch)
        previous = self.fresh('recognition')['session']
        after = time.monotonic()
        requested_ros = self.get_clock().now().nanoseconds/1e9
        # Send ONCE. A missing RPC reply is never permission to retry.
        future = client.call_async(SetBool.Request(data=True))
        def acknowledged():
            if future.done():
                response = future.result()
                if not response.success:
                    raise RuntimeError('/pallet/set_enabled: '+response.message)
            return new_recognition_session(self.fresh('recognition', after), previous, requested_ros)
        try:
            self.wait(acknowledged, ACQUISITION_ACK_TIMEOUT_S,
                      'xác nhận phiên nhận diện mới; không gửi lặp lệnh bật YOLO', watch)
            print(f'Đã xác nhận phiên nhận diện {previous+1} qua trạng thái trực tiếp.', flush=True)
            def ready():
                state = self.fresh('recognition', after)
                if not new_recognition_session(state, previous, requested_ros):
                    return False
                if future.done():
                    response = future.result()
                    if not response.success:
                        raise RuntimeError('/pallet/set_enabled: '+response.message)
                if state.get('phase') == 'ACQUIRE_FAILED':
                    detail=state.get('last_issue')
                    raise AcquisitionWindowExpired('Nhận diện thất bại: '+str(state.get('reason'))
                        +('; YOLO: '+str(detail) if detail else ''))
                if state.get('phase') == 'LOST':
                    raise AcquisitionWindowExpired('Nhận diện mất dấu: '+str(state.get('reason', state.get('phase'))))
                return recognition_ready(state)
            try:
                self.wait(ready, 30., 'pallet xanh và cửa sổ RGB-D ổn định', watch)
                self.report_recognition(watch)
            except RuntimeError as exc:
                state=self.fresh('recognition',after)
                if (str(exc) in ('Quá thời gian: pallet xanh và cửa sổ RGB-D ổn định',
                                 'Quá thời gian: phép đo mới để in kết quả nhận diện')
                        and new_recognition_session(state,previous,requested_ros)
                        and state.get('enabled') is False and state.get('busy') is False):
                    raise AcquisitionWindowExpired('Phiên đã kết thúc nhưng RGB-D chưa ổn định; nhận diện lại') from exc
                raise
        finally:
            if not future.done():
                client.remove_pending_request(future)
                future.cancel()

    def report_recognition(self, watch):
        result=[None]
        def available():
            result[0]=recognition_measurement(self.fresh('target'),self.fresh('recognition') or {},
                self.get_clock().now().nanoseconds/1e9,self.chassis_front_m,self.fork_root_m)
            return result[0] is not None
        self.wait(available,2.,'phép đo mới để in kết quả nhận diện',watch)
        print(recognition_result_text(result[0]),flush=True)
        if self.logs is not None:
            with (self.logs/'recognition_results.jsonl').open('a') as output:
                output.write(json.dumps(result[0],ensure_ascii=False,allow_nan=False)+'\n')

    def align(self):
        print('Chờ laser/odom và bộ căn ổn định liên tục 5 giây trước khi gửi lệnh.', flush=True)
        def qualified():
            now = self.get_clock().now().nanoseconds/1e9
            windows = all(w.ready(time.monotonic(), now) for w in self.sensor_windows.values())
            status = self.fresh('alignment')
            recognition = self.fresh('recognition')
            return (windows and status and status.get('ready') is True
                    and recognition and recognition_ready(recognition))
        # Repeat only a confirmed stationary visual-loss handoff. Actual fresh
        # stillness and the empty fork are checked before every new session.
        for attempt in itertools.count():
            while True:
                try:
                    self.wait(qualified, 30., 'laser/odom/RGB-D và bộ căn ổn định', ('staging', 'perception'))
                    break
                except RuntimeError as exc:
                    if str(exc)!='Quá thời gian: laser/odom/RGB-D và bộ căn ổn định':
                        raise
                    state=self.fresh('recognition') or {}
                    alignment=self.fresh('alignment') or {}
                    handling=self.fresh('handling') or {}
                    reason=alignment.get('ready_reason') or ''
                    if (reason.startswith('Nav2 chưa ACTIVE:')
                            and alignment.get('active') is False
                            and alignment.get('preparing') is False
                            and alignment.get('nav_snapshot') is False
                            and (not handling or handling.get('active') is False)):
                        self.wait(self.still,5.,'xe đứng yên trước khi khởi động lại bộ căn',('staging','perception'))
                        self.sensors(empty=True)
                        raise RestartStaging(reason)
                    if (state.get('phase') in ('LOST','ACQUIRE_FAILED')
                            and state.get('enabled') is False and state.get('busy') is False
                            and alignment.get('active') is False
                            and alignment.get('preparing') is False
                            and alignment.get('nav_snapshot') is not True
                            and (not handling or handling.get('active') is False)):
                        self.wait(self.still,5.,'xe đứng yên trước nhận diện lại',('staging','perception'))
                        self.sensors(empty=True)
                        print('Nhận diện đã mất; thử nhận diện lại tại chỗ.',flush=True)
                        self.acquire()
                    else:
                        print('Chưa đủ dữ liệu ổn định để căn; giữ xe dừng và tiếp tục chờ. Ctrl+C để hủy.',flush=True)
            after = time.monotonic()
            # One bounded request: server may wait 20 s for a rolling RGB-D
            # window to recover, then check the route. Never retry an unknown
            # service outcome or re-enable motion after a rejected start.
            print('Bật căn một lần; nếu RGB-D ngắt ngắn, xe giữ dừng và chờ phục hồi tối đa 20 giây.',flush=True)
            self.call(SetBool, '/pallet/alignment/set_enabled', SetBool.Request(data=True), 35.)
            # A handler may replace an infeasible staging projection with a
            # checked outside checkpoint route before replying. Read its next
            # fresh status, not the intermediate projection rejection.
            after=time.monotonic()
            reacquire=[False]
            def aligned():
                state = self.fresh('alignment', after)
                if not state:
                    return False
                if state.get('state')=='YAW_CORRECTED':
                    # Handler checks a fresh outside checkpoint route after
                    # yaw trim; this intermediate state is not insertion approval.
                    return False
                if state.get('state') in FAIL_ALIGNMENT:
                    if can_reacquire_after_arrival(state):
                        reacquire[0]=True
                        return True
                    raise RuntimeError('Căn dừng: '+str(state.get('reason')))
                return (state.get('state') in ('ALIGNED','CHECKPOINT_READY') and state.get('active') is False
                        and state.get('preparing') is False)
            self.wait(aligned, 200., 'căn xe đạt ALIGNED', ('staging', 'perception'))
            if not reacquire[0]:
                state=self.fresh('alignment') or {}
                if state.get('state')=='CHECKPOINT_READY':
                    print('Đường tới checkpoint đã kiểm tra; căn ngang/góc khi đầu càng còn ngoài. '
                          'Chưa cho phép đưa càng vào hoặc nâng.',flush=True)
                return
            print(f'Đã dừng tại goal căn nhưng mất dấu khi quay; nhận diện lại pallet xanh lần {attempt+1}, '
                  'chờ YOLO tắt/RGB-D ổn định rồi căn tiếp.',flush=True)
            self.wait(self.still, 5., 'xe đứng yên trước nhận diện lại', ('staging','perception'))
            self.sensors(empty=True)
            self.acquire()

    def halt(self):
        self.lease = False
        if self.foreground is not None and self.foreground.poll() is None:
            os.killpg(self.foreground.pid, signal.SIGINT)
            try:
                self.wait(lambda: self.foreground.poll() is not None, 8., 'dừng lệnh con')
            except RuntimeError:
                os.killpg(self.foreground.pid, signal.SIGKILL)
                self.wait(lambda: self.foreground.poll() is not None, 2., 'kết thúc lệnh con')
        # All service waits are bounded. No next phase or automatic resume.
        for typ, name, request in (
            (Trigger, '/pallet/handling/stop', Trigger.Request()),
            (SetBool, '/pallet/alignment/set_enabled', SetBool.Request(data=False)),
            (CancelGoal, '/navigate_to_pose/_action/cancel_goal', CancelGoal.Request())):
            client = self.create_client(typ, name)
            try:
                if client.service_is_ready():
                    future = client.call_async(request)
                    self.wait(future.done, 2., 'gửi dừng')
            except Exception:
                pass
            finally:
                self.destroy_client(client)
        end = time.monotonic()+1.
        while time.monotonic() < end:
            self.stop_base.publish(Twist())
            self.stop_lift.publish(Empty())
            self.pump()

    def recognition_station(self):
        self.sensors(empty=True)
        t = self.buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time()).transform
        goal = yaml.safe_load((REPO/'ros2_ws/config/pallet_goals.yaml').read_text())['pallets']['pallet_1']['recognition']
        q = t.rotation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        heading = abs(math.atan2(math.sin(yaw-goal['yaw_rad']), math.cos(yaw-goal['yaw_rad'])))
        distance = math.hypot(t.translation.x-goal['position']['x'], t.translation.y-goal['position']['y'])
        stationary = self.still()
        measured = dict(distance_m=distance, heading_error_deg=math.degrees(heading),
                        yaw_deg=math.degrees(yaw), stationary=stationary,
                        x=t.translation.x, y=t.translation.y)
        if self.logs is not None:
            (self.logs/'recognition_station.json').write_text(
                json.dumps(measured, ensure_ascii=False, indent=2)+'\n')
        if distance > .25 or heading > math.radians(20):
            raise RuntimeError(f'Ngoài vùng nhận diện: cách goal {distance*100:.1f} cm '
                f'(tối đa 25 cm), lệch hướng {math.degrees(heading):.1f}° '
                '(tối đa 20°); dùng run thông thường để về goal')
        if not stationary:
            raise RuntimeError(f'Trong vùng nhận diện ({distance*100:.1f} cm, '
                f'{math.degrees(heading):.1f}°), chưa đủ mẫu odometry mới '
                'xác nhận đứng yên liên tục 0,8 giây')
        return measured

    def wait_recognition_station(self):
        # Setup/service discovery can briefly drain the receipt-time window.
        # Pump fresh sensor/TF callbacks and qualify again before any handoff.
        # Keep the same pose, velocity and data-age limits throughout.
        issue = ['Chưa có dữ liệu']
        def ready():
            try:
                self.recognition_station()
                return True
            except Exception as exc:
                issue[0] = str(exc)
                return False
        try:
            self.wait(ready, 5., 'xác nhận vùng nhận diện và xe đứng yên')
        except RuntimeError:
            raise RuntimeError(issue[0])

    def verify_laser_filter(self):
        response=self.call(GetParameters,'/scan_self_filter/get_parameters',
                           GetParameters.Request(names=['max_scan_age_s']),5.)
        if (len(response.values)!=1
                or parameter_value_to_python(response.values[0])!=LASER_TIMEOUT_S):
            raise RuntimeError('Bộ lọc laser chưa nạp cấu hình thời gian quét 0,40 s mới')

    def run(self, from_recognition=False, from_current_view=False, recognition_backoff=0., pickup_only=False, alignment_only=False):
        self.logs = ROOT/'data/diagnostics'/time.strftime('pallet_cycle_%Y%m%d_%H%M%S')
        self.logs.mkdir(parents=True, exist_ok=False)
        print('Log chu trình: '+str(self.logs), flush=True)
        self.local_pickup=from_current_view and (pickup_only or alignment_only)
        if getattr(self,'auto_localize_requested',False):
            # Before AMCL converges, only the stationary sensor checks apply.
            # Restore the map freshness requirement BEFORE any navigation.
            self.awaiting_global_localization=True
            try:self.auto_localize_map()
            finally:self.awaiting_global_localization=False
            self.sensors(empty=True)
        elif not self.local_pickup:self.verify_localization()
        if from_current_view:
            print('[1/6] Nhận diện/căn mới từ vị trí hiện tại; không chạy về goal nhận diện.',flush=True)
            if self.get_publishers_info_by_topic('/cmd_vel_safe'):
                self.stop_navigation()
        elif from_recognition:
            self.wait_recognition_station()
            print('[1/6] Đã ở vùng nhận diện; giữ xe đứng yên, nhận diện và căn lại từ đầu.', flush=True)
            if self.get_publishers_info_by_topic('/cmd_vel_safe'):
                self.stop_navigation()
        else:
            self.ensure_navigation()
            print('[1/6] Về goal nhận diện pallet 1 (càng trống, đã hạ).', flush=True)
            return_args=()
            if recognition_backoff:
                return_args+=('--recognition-backoff',str(recognition_backoff))
            if getattr(self,'revalidate_recognition',False):
                return_args+=('--revalidate-recognition',)
            if return_args:self.operator('delivery','return',205.,extra_args=return_args)
            else:self.operator('delivery','return',205.)
            self.verify_localization()
            self.stop_navigation()
        while True:
            try:
                self.lease = True
                self.start('staging', ['ros2', 'launch', str(ROOT/'launch/pallet_alignment_existing.launch.py')])
                self.wait(lambda: self.command_owner('pallet_alignment_collision_monitor'), 20.,
                          'bộ tránh vật cản cho căn/đưa càng', ('staging',))
                self.start('perception', [os.environ.get('PALLET_PYTHON', str(ROOT/'.venv/bin/python')),
                    str(ROOT/'scripts/run_pallet_perception.py'),
                    '--model', str(ROOT/'data/runtime_models/best_v3_ncnn_model'),
                    '--cpu-threads', '2', '--target-color', 'green',
                    '--track-max-fps', '15', '--preview-fps', '2',
                    '--measurement-bias-yaml', str(BIAS_PROFILE),
                    '--heading-floor-reference', '--start-paused'])
                print('[2/6] Nhận diện pallet xanh; chờ RGB-D ổn định và YOLO tự tắt.', flush=True)
                self.acquire()
                print('[3/6] Căn xe theo ngưỡng trong pallet_staging.yaml; gốc càng cách pallet ≥1,2 m.', flush=True)
                self.align()
                if alignment_only:
                    state=self.fresh('alignment') or {}
                    print(('ĐÃ KIỂM TRA ĐƯỜNG CHECKPOINT; xe còn ở vị trí ban đầu. '
                           if state.get('state')=='CHECKPOINT_READY' else 'ĐÃ CĂN XONG VÀ DỪNG. ')
                          +'Kết thúc bước thử; chưa chạy checkpoint, đưa càng hoặc nâng.',flush=True)
                    return
                reason=str((self.fresh('alignment') or {}).get('reason',''))
                if 'DIRECT_READY' in reason:
                    message='[4/6] Bỏ tiến chéo/checkpoint → đo tại chỗ → quay bù nếu cần → đưa càng thẳng → chờ xác nhận nâng.'
                elif 'QUARTER_TURN_READY' in reason:
                    message='[4/6] Căn hướng → quay 90° → tiến bù ngang → quay lại → nhận diện mới → đưa càng thẳng → chờ xác nhận nâng.'
                else:
                    message='[4/6] Tới checkpoint → đo lại → chốt đường thẳng odom → đưa càng → chờ xác nhận nâng.'
                print(message, flush=True)
                after = time.monotonic()
                self.operator('pickup', 'insert-and-lift', 150., ('staging', 'perception'))
                break
            except RestartStaging as exc:
                print('Bộ căn chưa sẵn sàng: '+str(exc)+'. Khởi động lại bộ căn tại chỗ.',flush=True)
                self.stop_child('staging')
                self.stop_child('perception')
                self.lease=False
                self.wait(lambda: not self.get_publishers_info_by_topic('/cmd_vel_safe')
                          and not self.get_publishers_info_by_topic('/lift_controller/commands'),
                          10.,'gỡ bộ điều khiển trước khi mở lại')
                self.pump(2.)
            except ReturnToRecognition as exc:
                print("Checkpoint: "+str(exc)+". Xe dừng; chuẩn bị về goal nhận diện và làm lại. Ctrl+C để hủy.",flush=True)
                def outside_ready():
                    try:
                        self.sensors(empty=True)
                        if not self.still():return False
                        measurement=recognition_measurement(self.fresh('target'),self.fresh('recognition') or {},
                            self.get_clock().now().nanoseconds/1e9,self.chassis_front_m,self.fork_root_m)
                        return bool(measurement and measurement['fork_root_plane_gap_m']>.49)
                    except Exception:
                        return False
                while True:
                    try:
                        self.wait(outside_ready,30.,'xác nhận càng ở ngoài pallet trước khi về goal',('staging','perception'))
                        break
                    except RuntimeError as wait_error:
                        if str(wait_error)!='Quá thời gian: xác nhận càng ở ngoài pallet trước khi về goal':raise
                        print('Chưa xác nhận được xe đứng yên và càng ở ngoài; tiếp tục giữ dừng và kiểm tra.',flush=True)
                self.stop_child("staging")
                self.stop_child("perception")
                self.lease=False
                self.wait(lambda: not self.get_publishers_info_by_topic("/cmd_vel_safe")
                          and not self.get_publishers_info_by_topic("/lift_controller/commands"),
                          10., "gỡ bộ điều khiển trước khi về goal nhận diện")
                self.verify_localization()
                self.ensure_navigation()
                return_args=()
                if recognition_backoff:return_args+=("--recognition-backoff",str(recognition_backoff))
                if getattr(self,"revalidate_recognition",False):return_args+=("--revalidate-recognition",)
                if return_args:self.operator("delivery","return",205.,extra_args=return_args)
                else:self.operator("delivery","return",205.)
                self.verify_localization()
                self.stop_navigation()
        status = self.fresh('handling', after)
        lift = self.sensors()
        if (not status or status.get('state') != 'COMPLETE' or status.get('active') is not False
                or abs(lift['position']-.08) > .004):
            raise RuntimeError('Chưa xác nhận đưa càng và nâng hoàn tất; chưa giao hàng')
        self.wait(self.still, 4., 'xe dừng sau nâng')
        self.stop_child('staging')
        self.stop_child('perception')
        self.lease = False
        self.wait(lambda: not self.get_publishers_info_by_topic('/cmd_vel_safe')
                  and not self.get_publishers_info_by_topic('/lift_controller/commands'),
                  10., 'gỡ bộ điều khiển căn/đưa càng')
        if pickup_only:
            print('HOÀN TẤT LẤY HÀNG: càng đã nâng 8 cm và xe đứng yên tại điểm lấy. '
                  'Chờ lệnh tiếp theo của người vận hành.', flush=True)
            return
        if not self.local_pickup:self.ensure_navigation()
        print('[5/6] Đưa pallet 1 tới goal giao hàng đã lưu.', flush=True)
        self.verify_localization()
        self.operator('delivery', 'go', 205.)
        print('[6/6] Hạ pallet về 0 cm và kết thúc.', flush=True)
        self.operator('delivery', 'lower', 45.)
        if self.sensors()['position'] > .002 or not self.still():
            raise RuntimeError('Chưa xác nhận hoàn tất hạ pallet')
        print('HOÀN TẤT: đã giao và hạ pallet 1. Xe đứng yên, chưa rút càng.', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('operation', choices=('preview', 'run'))
    parser.add_argument('--auto-localize',action='store_true',
                        help='Giữ AMCL đang tốt; chỉ tìm lại pose toàn cục khi chưa có hoặc mất định vị')
    parser.add_argument('--revalidate-recognition',action='store_true',
                        help='Về goal nhận diện đã lưu cần kiểm chứng, rồi nhận diện/căn mới bằng camera')
    parser.add_argument('--allow-experimental-calibration', action='store_true',
                        help='Chủ động chạy thử hồ sơ hiệu chỉnh chưa đạt kiểm chứng; giữ mọi kiểm tra chuyển động')
    parser.add_argument('--usable-depth-cm',type=float,
                        help='Chiều dài khoang trống giữa các trụ theo hướng đưa càng (cm)')
    end_mode=parser.add_mutually_exclusive_group()
    end_mode.add_argument('--alignment-only',action='store_true',
                          help='Chỉ nhận diện/căn rồi dừng; chưa checkpoint, đưa càng hoặc nâng')
    end_mode.add_argument('--pickup-only', action='store_true',
                        help='Nhận diện/căn/đưa càng/nâng 8 cm rồi kết thúc tại điểm lấy')
    start_mode=parser.add_mutually_exclusive_group()
    start_mode.add_argument('--from-recognition', action='store_true',
                        help='Xe đã ở vùng nhận diện: kiểm tra vị trí, nhận diện/căn mới rồi tiếp tục một lượt')
    start_mode.add_argument('--from-current-view',action='store_true',
                        help='Càng trống/đã hạ: nhận diện và căn mới tại vị trí hiện tại; vẫn kiểm tra đầy đủ trước đưa càng')
    parser.add_argument('--recognition-backoff',type=float,default=0.,
                        help='Điểm nhận diện dự phòng xa hơn goal cũ 0–0,6 m, giữ nguyên goal lưu')
    args = parser.parse_args()
    if args.revalidate_recognition and (args.from_recognition or args.from_current_view):
        parser.error('--revalidate-recognition dùng khi chu trình chạy về goal nhận diện')
    if (not math.isfinite(args.recognition_backoff) or not 0<=args.recognition_backoff<=.6
            or (args.recognition_backoff and (args.from_recognition or args.from_current_view))):
        parser.error('Dự phòng 0–0,6 m chỉ dùng cho chu trình run đầy đủ')
    if args.operation == 'run':
        if args.usable_depth_cm is None:
            if not args.alignment_only:
                parser.error('Cần --usable-depth-cm với chiều dài khoang trống giữa các trụ theo hướng đưa càng')
            os.environ.pop('PALLET_USABLE_DEPTH_M',None)
        else:
            if not math.isfinite(args.usable_depth_cm) or not 10 < args.usable_depth_cm <= 80:
                parser.error('--usable-depth-cm phải là số đo thông thoáng trong khoảng 10–80 cm')
            os.environ['PALLET_USABLE_DEPTH_M']=str(args.usable_depth_cm/100.)
        goals = yaml.safe_load((REPO/'ros2_ws/config/pallet_goals.yaml').read_text())['pallets']['pallet_1']
        pending = [name for name in required_goal_names(args.pickup_only or args.alignment_only,
                   args.from_recognition or args.from_current_view)
                   if not station_navigation_ready(goals.get(name,{}),name,args.revalidate_recognition)]
        if pending:
            print('Chưa lưu goal pallet 1 mới: '+', '.join(pending)+'. Không chạy chu trình.', flush=True)
            return 1
    # Hold the lease before creating any ROS node or command publisher.
    lock = None
    if args.operation == 'run':
        runtime = ROOT/'data/runtime'
        runtime.mkdir(parents=True, exist_ok=True)
        lock = (runtime/'pallet_cycle.lock').open('a+')
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            print('Đang có chu trình khác; không gửi lệnh mới.', flush=True)
            return 1
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Cycle()
    node.auto_localize_requested=args.auto_localize
    node.awaiting_global_localization=args.auto_localize
    node.revalidate_recognition=args.revalidate_recognition
    node.local_pickup=args.from_current_view and (args.pickup_only or args.alignment_only)
    started = False
    def interrupted(*_):
        raise KeyboardInterrupt
    old_term = signal.signal(signal.SIGTERM, interrupted)
    try:
        if args.operation=='run':require_motion_calibration(BIAS_PROFILE, args.allow_experimental_calibration)
        # DDS/TF discovery can first deliver an older retained transform.
        # Wait for fresh inputs using the SAME age limits; do not waive them.
        last_issue = ['Chưa có đủ dữ liệu']
        def ready():
            try:
                node.sensors(empty=True)
                if not node.still():
                    raise RuntimeError('Xe chưa đứng yên liên tục 0,8 giây')
                floor_issue=node.camera_floor_issue()
                if floor_issue:
                    raise RuntimeError(floor_issue)
                return True
            except Exception as exc:
                last_issue[0] = str(exc)
                return False
        while True:
            try:
                node.wait(ready, 12., 'phần cứng/định vị/xe đứng yên')
                break
            except RuntimeError as exc:
                if (args.operation!='run'
                        or str(exc)!='Quá thời gian: phần cứng/định vị/xe đứng yên'):
                    raise RuntimeError('Chưa sẵn sàng: '+last_issue[0]) from exc
                print('Chưa sẵn sàng: '+last_issue[0]+'; giữ xe dừng và tiếp tục kiểm tra. Ctrl+C để hủy.',flush=True)
        if node.active_nav():
            raise RuntimeError('Nav2 đang có goal khác; cần dừng goal trước')
        if any(node.get_publishers_info_by_topic(topic) for topic in
               ('/pallet/alignment/status', '/pallet/handling/status', '/pallet/recognition/status',
                '/lift_controller/commands')):
            raise RuntimeError('Còn bộ căn/nhận diện/điều khiển càng khác đang chạy; không mở trùng')
        if args.from_recognition:
            node.wait_recognition_station()
        node.verify_laser_filter()
        if args.operation == 'preview':
            goals = yaml.safe_load((REPO/'ros2_ws/config/pallet_goals.yaml').read_text())['pallets']['pallet_1']
            print(json.dumps(goals, ensure_ascii=False, indent=2))
            steps=[]
            if args.auto_localize:steps.append('kiểm tra AMCL, chỉ tự định vị lại khi chưa đạt')
            if not (args.from_recognition or args.from_current_view):steps.append('Nav2 về goal nhận diện đã lưu')
            steps+=['nhận diện pallet xanh mới','căn']
            if not args.alignment_only:
                steps+=['checkpoint/đo lại','đưa càng theo odom','chờ gõ nang','nâng 8 cm']
                if not args.pickup_only:steps+=['giao','hạ']
            steps.append('dừng')
            print('Chu trình dự kiến: '+' → '.join(steps)+'. Chỉ kiểm tra; chưa gửi lệnh chuyển động.')
        else:
            started = True
            node.budget = CpuBudget()
            node.budget.apply_existing()
            print('Dành một lõi CPU cho laser/đế xe/TF; tác vụ ảnh/Nav2 dùng các lõi còn lại.'
                  if node.budget.enabled else 'CPU hiện tại không hỗ trợ dành riêng một lõi.', flush=True)
            node.run(args.from_recognition,args.from_current_view,args.recognition_backoff,args.pickup_only,args.alignment_only)
        return 0
    except KeyboardInterrupt:
        print('Đã hủy chu trình; đang dừng xe và càng.', flush=True)
        return 1
    except Exception as exc:
        print('DỪNG CHU TRÌNH: '+str(exc), flush=True)
        return 1
    finally:
        signal.signal(signal.SIGTERM, old_term)
        if started:
            try:
                node.halt()
            except Exception as exc:
                print('Lỗi khi gửi dừng: '+str(exc), flush=True)
            for name in ('staging', 'perception', 'localization_health'):
                try:
                    node.stop_child(name)
                except Exception as exc:
                    print('Kiểm tra tiến trình '+name+': '+str(exc), flush=True)
        if node.budget is not None:
            node.budget.restore()
        node.destroy_node()
        rclpy.shutdown()
        if lock is not None:
            lock.close()


if __name__ == '__main__':
    raise SystemExit(main())
