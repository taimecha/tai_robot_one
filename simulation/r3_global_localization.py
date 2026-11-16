#!/usr/bin/env python3
"""Stationary whole-map scan bootstrap; optionally seed AMCL once, never save A."""
import argparse
from concurrent.futures import Future
import copy
import hashlib
import json
import math
from pathlib import Path
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener

from r3_amcl_bridge import planar_pose
from r3_core import wrap
from r3_recognition_goal import atomic_json, map_identity
# Load native numerical libraries before starting sensor callbacks. A cold
# NumPy/OpenCV import in the worker can hold the interpreter long enough to
# starve odom/LiDAR callbacks and correctly trip the stream-gap guard.
from r3_global_scan_match import GlobalScanMatcher


def fresh(stamp, now, limit):
    return stamp is not None and math.isfinite(stamp) and -.05 <= now-stamp <= limit


def motion_issue(pose, twist, baseline=None):
    if (len(pose)!=3 or len(twist)!=3
            or not all(math.isfinite(v) for v in (*pose,*twist))):
        return 'invalid odometry during stationary search'
    if math.hypot(*twist[:2]) >= .005 or abs(twist[2]) >= .015:
        return 'robot motion detected during stationary search'
    if baseline is not None and (math.dist(pose[:2],baseline[:2]) > .005
            or abs(wrap(pose[2]-baseline[2])) > math.radians(.5)):
        return 'robot pose changed during stationary search'
    return None


def candidate_pose(report):
    pose=(report.get('best') or {}).get('pose_xyyaw')
    if (report.get('accepted') is not True or not isinstance(pose,(list,tuple))
            or len(pose)!=3 or any(isinstance(v,bool) or not isinstance(v,(int,float))
                                   or not math.isfinite(v) for v in pose)):
        raise ValueError('global scan match rejected: '+str(report.get('reason','invalid candidate')))
    return tuple(pose)


def agree_candidates(first, second):
    a,b=candidate_pose(first),candidate_pose(second)
    distance=math.dist(a[:2],b[:2]);angle=abs(wrap(a[2]-b[2]))
    if distance>.05+1e-9 or angle>math.radians(2.)+1e-9:
        raise ValueError('independent fresh scan matches disagree beyond 5 cm / 2 deg')
    return dict(position_difference_m=distance,heading_difference_deg=math.degrees(angle))


def seed_message(pose, stamp):
    msg=PoseWithCovarianceStamped()
    msg.header.frame_id='map';msg.header.stamp=stamp
    msg.pose.pose.position.x,msg.pose.pose.position.y=map(float,pose[:2])
    msg.pose.pose.orientation.z=math.sin(pose[2]/2)
    msg.pose.pose.orientation.w=math.cos(pose[2]/2)
    msg.pose.covariance[0]=msg.pose.covariance[7]=.05**2
    msg.pose.covariance[35]=math.radians(2.)**2
    return msg


class GlobalLocalization(Node):
    def __init__(self):
        super().__init__('r3_global_scan_localization')
        self.grid=self.scan=self.odom=None
        self.grid_key=None;self.odom_stamp=self.scan_stamp=None
        self.pose=self.twist=self.baseline=None
        self.still_since=None;self.started=False;self.fault=None
        self.future=None;self.matcher=None;self.reports=[]
        self.capture_stamp=None;self.after_scan_stamp=-math.inf
        self.active=False;self.active_wall=0.;self.state_future=None;self.state_sent=0.
        self.tf=Buffer();self.listener=TransformListener(self.tf,self)
        self.state_client=self.create_client(GetState,'/amcl/get_state')
        self.publisher=self.create_publisher(PoseWithCovarianceStamped,'/initialpose',10)
        map_qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid,'/map',self.on_map,map_qos)
        self.create_subscription(LaserScan,'/scan_filtered',self.on_scan,qos_profile_sensor_data)
        self.create_subscription(Odometry,'/odom',self.on_odom,qos_profile_sensor_data)

    @staticmethod
    def stamp(msg):
        return msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9

    def now(self):
        return self.get_clock().now().nanoseconds*1e-9

    def on_map(self,msg):
        key=(msg.header.frame_id,msg.info.resolution,msg.info.width,msg.info.height,
             str(msg.info.origin),hashlib.sha256(bytes((v+1)%256 for v in msg.data)).hexdigest())
        if self.started and key!=self.grid_key:
            self.fault='occupancy map changed during global search'
        if msg.header.frame_id!='map':
            self.fault='occupancy map is not in map frame'
        self.grid=msg;self.grid_key=key

    def on_scan(self,msg):
        stamp=self.stamp(msg)
        if self.started and self.scan_stamp is not None and stamp-self.scan_stamp>1.:
            self.fault='LiDAR capture gap exceeds 1 second during search'
        self.scan=msg;self.scan_stamp=stamp

    def on_odom(self,msg):
        try:
            if msg.header.frame_id!='odom' or msg.child_frame_id not in ('base_footprint','base_link'):
                raise ValueError('unexpected odometry frame')
            pose=planar_pose(msg.pose.pose.position,msg.pose.pose.orientation)
            t=msg.twist.twist;twist=(t.linear.x,t.linear.y,t.angular.z)
            issue=motion_issue(pose,twist,self.baseline if self.started else None)
            stamp=self.stamp(msg)
            if self.odom_stamp is not None and stamp-self.odom_stamp>.5:
                self.still_since=None
                if self.started and stamp-self.odom_stamp>1.:
                    self.fault='odometry capture gap exceeds 1 second during search'
            self.odom=msg;self.odom_stamp=stamp;self.pose=pose;self.twist=twist
            if issue:
                self.still_since=None
                if self.started:self.fault=issue
            elif self.still_since is None:
                self.still_since=time.monotonic()
        except ValueError as exc:
            self.odom=None;self.still_since=None
            if self.started:self.fault=str(exc)

    def check_running(self):
        if self.fault:raise ValueError(self.fault)
        if self.started and (not fresh(self.odom_stamp,self.now(),1.)
                             or not fresh(self.scan_stamp,self.now(),1.)):
            raise ValueError('odom or LiDAR gap exceeds 1 second during search')

    def poll_active(self,wall):
        if self.state_future is not None:
            if self.state_future.done():
                try:
                    self.active=self.state_future.result().current_state.id==3
                    self.active_wall=wall
                except Exception:self.active=False
                self.state_future=None
            elif wall-self.state_sent>2.:
                self.state_client.remove_pending_request(self.state_future)
                self.state_future=None;self.active=False
        if self.state_future is None and wall-self.state_sent>=.5 and self.state_client.service_is_ready():
            self.state_future=self.state_client.call_async(GetState.Request());self.state_sent=wall

    def begin_match(self):
        tf=self.tf.lookup_transform('base_footprint',self.scan.header.frame_id,Time())
        base_lidar=planar_pose(tf.transform.translation,tf.transform.rotation)
        grid,scan=copy.deepcopy(self.grid),copy.deepcopy(self.scan)
        self.capture_stamp=self.stamp(scan)
        if not self.started:self.baseline=self.pose;self.started=True
        future=Future();self.future=future
        def work():
            if not future.set_running_or_notify_cancel():return
            try:
                if self.matcher is None:self.matcher=GlobalScanMatcher(grid)
                future.set_result(self.matcher.match(scan,base_lidar))
            except BaseException as exc:future.set_exception(exc)
        # One read-only worker at a time. Daemon exit keeps the total timeout
        # bounded even if a numerical search has not returned yet.
        threading.Thread(target=work,daemon=True,name='r3_global_scan_worker').start()

    def run(self,timeout,apply):
        deadline=time.monotonic()+timeout;issue='waiting for fresh stationary sensors';last_log=0.
        while rclpy.ok() and time.monotonic()<deadline:
            rclpy.spin_once(self,timeout_sec=.05)
            wall=time.monotonic();self.check_running();self.poll_active(wall)
            sensors=(self.grid is not None and self.scan is not None and self.odom is not None
                     and fresh(self.odom_stamp,self.now(),.25) and fresh(self.scan_stamp,self.now(),.5)
                     and self.still_since is not None and wall-self.still_since>=.5)
            if self.future is not None and self.future.done():
                report=self.future.result();self.future=None
                self.reports.append(dict(scan_stamp_ros_s=self.capture_stamp,
                                         completed_ros_s=self.now(),match=report))
                candidate_pose(report)
                self.after_scan_stamp=self.now()
                if len(self.reports)==2:
                    agreement=agree_candidates(self.reports[0]['match'],report)
                    pose=candidate_pose(report)
                    self.get_logger().info('Two fresh global scan matches agree; checking AMCL before seed')
            if len(self.reports)==2:
                if not sensors:issue='waiting for fresh stationary sensors before completing seed'
                elif not apply:return dict(status='CANDIDATE',pose_xyyaw=list(pose),agreement=agreement)
                elif (self.active and wall-self.active_wall<=1.5 and self.publisher.get_subscription_count()>0):
                    self.publisher.publish(seed_message(pose,self.get_clock().now().to_msg()))
                    rclpy.spin_once(self,timeout_sec=.1)
                    return dict(status='SEEDED',pose_xyyaw=list(pose),agreement=agreement,
                                initialpose_publications=1)
                else:issue='waiting for active AMCL and an /initialpose subscriber'
            elif sensors and self.future is None and self.scan_stamp>self.after_scan_stamp:
                try:
                    self.begin_match();issue=f'matching fresh scan {len(self.reports)+1}/2 against whole map'
                except TransformException as exc:issue='waiting for base/LiDAR mounting transform: '+str(exc)
            if wall-last_log>=2.:
                self.get_logger().info(issue);last_log=wall
        raise ValueError('global localization timeout/stopped: '+issue)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--timeout',type=float,default=150.)
    parser.add_argument('--apply',action='store_true',help='Publish one validated /initialpose seed')
    args=parser.parse_args()
    if not args.map.is_absolute() or not args.map.is_file():parser.error('--map must be an existing absolute map YAML')
    if not math.isfinite(args.timeout) or args.timeout<=0:parser.error('--timeout must be finite and positive')
    identity=map_identity(args.map)
    rclpy.init();node=GlobalLocalization();result={}
    try:
        result=node.run(args.timeout,args.apply)
    except (KeyboardInterrupt,Exception) as exc:
        result=dict(status='BLOCKED',reason=str(exc))
    finally:
        result.update(map=identity,scan_reports=node.reports,apply_requested=args.apply,
            physical_accuracy_verified=False,recognition_A_saved=False,
            actuator_commands=0,source='two_fresh_stationary_global_scan_matches')
        atomic_json(args.output,result)
        print(json.dumps(result,indent=2,allow_nan=False),flush=True)
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
    return 0 if result.get('status') in ('SEEDED','CANDIDATE') else 2


if __name__=='__main__':
    raise SystemExit(main())
