#!/usr/bin/env python3
"""Own alignment, operator-started insertion and lift through one command gate."""
import argparse
from pallet_laser_timing import LASER_TIMEOUT_S, stop_travel, STOP_POLYGON_PADDING_M, STOP_RESERVE_M,measured_velocity_issue
from collections import deque
from dataclasses import asdict
import json
import math
from pathlib import Path
import time
import uuid

import rclpy
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Empty, Float64MultiArray
from std_srvs.srv import Trigger, SetBool
from nav_msgs.msg import Path as NavPath
from tf2_ros import Buffer, TransformListener, TransformException
import yaml

from align_pallet import ROOT, stamp_seconds
from stage_pallet import PalletStagingNode
from pallet_staging_core import StagingConfig
from pallet_alignment_core import wrap
from pallet_insertion_core import InsertionConfig, Opening, make_plan, braking_poses, opening_coordinates,terminal_geometry
from pallet_pickup_core import PickupConfig, PickupController
from pallet_visual_pickup import VisualPickupController
from pallet_pickup_safety import footprint_hit,capture_laser_points,LaserTransformPending
from pallet_pickup_recovery import relative_opening
from pallet_checkpoint import checkpoint_plan,checkpoint_pose_issue,pivot_stop_issue

LASER_TRANSFORM_WAIT_S = .12


class PalletHandlingNode(PalletStagingNode):
    def __init__(self,config=StagingConfig(),pickup_config=PickupConfig(),execute=False,recovery_checkpoint=None,cycle_token=None):
        if recovery_checkpoint is not None:
            raise ValueError('Checkpoint phương pháp cũ đã ngừng sử dụng')
        self.cycle_token=cycle_token
        self.cycle_wall=0.
        self.pickup=VisualPickupController(pickup_config)
        self.pickup.require_checkpoint_refresh=bool(cycle_token)
        self.checkpoint_refresh_session=None
        c=pickup_config.insertion
        if stop_travel(c.maximum_speed_mps,c.maximum_yaw_rate_rps,c.braking_mps2,c.angular_braking_rps2)>STOP_POLYGON_PADDING_M-STOP_RESERVE_M:
            raise ValueError('Pickup speed/braking does not fit the laser stop envelope')
        self.pickup_id=None
        self.operator_wall=0.
        self.lease_started=0.
        self.lift=None;self.lift_wall=0.;self.lift_sample=None
        self.pickup_scan=None;self.pickup_scan_wall=0.
        self.pickup_scans=deque(maxlen=6)
        self.laser_hold_since=None
        self.laser_hold_reason=None
        self.scan_pause_started=None
        self.scan_pause_good_since=None
        self.scan_pause_stamp=None
        self.scan_pause_count=0
        self.last_target_msg=None
        self.last_lift_publish=0.
        self.previous_pickup_active=False
        self.pickup_reference=None
        self.recovery_checkpoint=recovery_checkpoint
        self.stopped_opening=None
        self.stopped_height=None
        self.pickup_frame_valid=False
        self.recovery_anchor=None
        self.recovery_anchor_invalid=False
        super().__init__(config,execute)
        self.tf_buffer=Buffer();self.tf_listener=TransformListener(self.tf_buffer,self)
        qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(LaserScan,'/scan_filtered',self.on_pickup_scan,qos)
        self.create_subscription(String,'/lift/status',self.on_lift,1)
        self.create_subscription(String,'/pallet/handling/heartbeat',self.on_operator,1)
        if cycle_token:self.create_subscription(String,'/pallet/cycle/heartbeat',self.on_cycle_heartbeat,1)
        self.handling_status=self.create_publisher(String,'/pallet/handling/status',
            QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.path_pub=self.create_publisher(NavPath,'/pallet/handling/path',1)
        self.lift_command=self.create_publisher(Float64MultiArray,'/lift_controller/commands',1) if execute else None
        self.lift_stop=self.create_publisher(Empty,'/lift/stop',1) if execute else None
        for name,typ,callback in [('preview',Trigger,self.preview_pickup),
                ('start',SetBool,self.start_pickup),('lift',Trigger,self.start_lift),
                ('stop',Trigger,self.stop_pickup),('reset',Trigger,self.reset_pickup),
                ('finish_checkpoint_recognition',Trigger,self.finish_checkpoint_recognition)]:
            self.create_service(typ,'/pallet/handling/'+name,callback)

    def on_pickup_scan(self,msg):
        stamp=stamp_seconds(msg.header.stamp)
        if (not -.05<=self.now()-stamp<=LASER_TIMEOUT_S
                or (self.pickup_scan is not None and stamp<=stamp_seconds(self.pickup_scan.header.stamp))):
            return
        self.pickup_scan=msg;self.pickup_scan_wall=time.monotonic()
        self.pickup_scans.append((msg,self.pickup_scan_wall))

    def on_lift(self,msg):
        try:
            data=json.loads(msg.data)
            if (data.get('serial_connected') is not True or data.get('homed') is not True
                    or data.get('fault')!='NONE' or data.get('state') not in ('IDLE','MOVING')
                    or any(not math.isfinite(data[k]) for k in ('position','target','velocity'))
                    or not 0<=data['position']<=.53 or type(data['sample_ms']) is not int
                    or not 0<=data['sample_ms']<=0xffffffff
                    or not 0<=data['target']<=.53
                    or (data['upper_active'] and data['position']<.50)
                    or type(data['upper_active']) is not bool):
                raise ValueError('Càng chưa HOME, lỗi hoặc phản hồi không hợp lệ')
            if self.lift_sample is not None:
                delta=(data['sample_ms']-self.lift_sample)&0xffffffff
                if not 0<delta<0x80000000:
                    if delta:raise ValueError('ESP32 càng bị reset hoặc thời gian nhảy')
                    return
            self.lift=data;self.lift_wall=time.monotonic();self.lift_sample=data['sample_ms']
        except (ValueError,KeyError,TypeError):
            self.lift=None
            if self.pickup.active:self.abort_pickup('Mất phản hồi càng hợp lệ')

    def on_operator(self,msg):
        if msg.data==self.pickup_id:self.operator_wall=time.monotonic()

    def on_cycle_heartbeat(self,msg):
        if msg.data==self.cycle_token:self.cycle_wall=time.monotonic()

    def cycle_issue(self):
        return 'Mất heartbeat chu trình; phải bắt đầu lại' if self.cycle_token and time.monotonic()-self.cycle_wall>.6 else ''

    def interlock(self):
        issue=self.cycle_issue()
        return (False,issue) if issue else super().interlock()

    def on_target(self,msg):
        self.last_target_msg=msg
        if (self.pickup.state in ('IDLE','BLOCKED','STOPPED') and not self.pickup.active
                or self.pickup.phase in ('REMEASURE','FINAL_ALIGN','CHECKPOINT_RECOGNITION')):
            super().on_target(msg)

    def consume_target(self):
        pending=self.pending_target
        super().consume_target()
        if not pending or self.pickup.phase not in ('REMEASURE','FINAL_ALIGN'):return
        msg,received=pending
        stamp=stamp_seconds(msg.header.stamp)
        if self.core.target is None or self.core.target_stamp!=stamp:return
        odom=self.core.odom.at(stamp)
        if odom is None:return
        forward=msg.chassis_front_forward_m+self.core.config.chassis_front_m
        c,s=math.cos(odom.yaw),math.sin(odom.yaw)
        opening=Opening(odom.x+c*forward-s*msg.opening_lateral_m,
            odom.y+s*forward+c*msg.opening_lateral_m,
            wrap(odom.yaw+msg.entry_heading_rad),msg.opening_width_m)
        health=(self.recognition or {}).get('rgbd_health',{}).get('stable') is True
        self.pickup.observe(stamp,received,opening,health)

    def reject(self,reason):
        camera_reason=reason.startswith(('RGB + Depth','Phép đo chờ','Chạy thật cần phép đo'))
        if self.pickup.active and self.pickup.phase in ('REMEASURE','FINAL_ALIGN','CHECKPOINT_RECOGNITION') and camera_reason:
            self.pickup.samples.clear();self.pickup.observation=None
            self.zero()
        elif self.pickup.active:self.abort_pickup(reason)
        super().reject(reason)

    def on_odom(self,msg):
        previous=self.core.odom.samples[-1] if self.core.odom.samples else None
        p,q=msg.pose.pose.position,msg.pose.pose.orientation
        heading=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        if previous and (stamp_seconds(msg.header.stamp)<previous.stamp_s
                or math.hypot(p.x-previous.x,p.y-previous.y)>.10
                or abs(wrap(heading-previous.yaw))>.15):
            self.pickup_frame_valid=False
            if self.recovery_anchor is not None:self.recovery_anchor_invalid=True
        super().on_odom(msg)
        if (self.recovery_anchor is None and (self.recovery_checkpoint or {}).get('frozen_trajectory') is True
                and self.core.odom.samples and self.core.odom.samples[-1].stamp_s==stamp_seconds(msg.header.stamp)
                and abs(msg.twist.twist.linear.x)<.003 and abs(msg.twist.twist.angular.z)<.015):
            self.recovery_anchor=self.current_pose()[0]
        if self.pickup.active and (not self.core.odom.samples
                or (previous is not None and self.core.odom.samples[-1].stamp_s<previous.stamp_s)):
            self.abort_pickup('Odometry bị reset khi đưa càng')

    async def set_enabled(self,request,response):
        if not request.data:
            self.recovery_checkpoint=None
            self.stopped_opening=self.stopped_height=None
        if self.pickup.state!='IDLE':
            if request.data:
                response.success=False;response.message='Đang ở bước đưa càng/nâng; dùng dừng hoặc reset khi xe ở ngoài pallet'
            else:
                self.abort_pickup('Đã dừng bằng nút Dừng căn xe','STOPPED')
                response.success=True;response.message='Đã dừng xe và càng'
            return response
        if request.data and not self.core.active:
            self.pickup.direct_mode=True
            try:
                plan=self.prepare_plan()
                self.core.stop('DIRECT_READY: ngang dưới 5 cm; bỏ tiến chéo/checkpoint, quay bù nếu cần rồi đi thẳng',
                               'CHECKPOINT_READY')
                self.publish_plan(plan);self.publish_status()
                response.success=True;response.message=self.core.reason
                return response
            except (ValueError,RuntimeError):
                self.pickup.direct_mode=False
        if request.data and self.pickup.visual_config.quarter_turn_lateral and not self.core.active:
            self.pickup.route_requested=True
            try:
                plan=self.prepare_plan()
                self.core.stop('QUARTER_TURN_READY: quay 90°, tiến sửa ngang, quay lại rồi nhận diện mới', 'CHECKPOINT_READY')
                self.publish_plan(plan);self.publish_status()
                response.success=True;response.message=self.core.reason
            except (ValueError,RuntimeError) as exc:
                self.pickup.route_requested=False
                response.success=False;response.message=str(exc)
            return response
        response=await super().set_enabled(request,response)
        if (request.data and not response.success and self.core.state=='NO_SAFE_GOAL'
                and not self.core.candidates and not self.core.heading_only_trim):
            try:
                self.prepare_checkpoint_route()
                response.success=True;response.message=self.core.reason
            except Exception as exc:response.message+='; checkpoint: '+str(exc)
        return response

    def finish_checkpoint_recognition(self,request,response):
        try:
            p=self.pickup
            if not p.active or p.phase!='CHECKPOINT_RECOGNITION':
                raise ValueError('Không ở bước nhận diện lại tại checkpoint')
            issue=self.sensor_issue()
            if issue:raise ValueError(issue)
            pose,v,w=self.current_pose();anchor=p.checkpoint_refresh_pose
            if (abs(v)>.003 or abs(w)>.015 or not anchor
                    or math.hypot(pose[0]-anchor[0],pose[1]-anchor[1])>.005
                    or abs(wrap(pose[2]-anchor[2]))>math.radians(.5)):
                raise ValueError('Xe chưa giữ đúng pose nhận diện tại checkpoint')
            status=self.recognition or {}
            if (self.checkpoint_refresh_session is None or type(status.get('session')) is not int
                    or status['session']<=self.checkpoint_refresh_session
                    or status.get('enabled') is not False or status.get('busy') is not False
                    or status.get('locked') is not True or status.get('measurement_ready') is not True
                    or not status.get('rgbd_health',{}).get('stable')):
                raise ValueError('Cần phiên nhận diện MỚI, đủ hai chân và YOLO đã tắt')
            msg=self.last_target_msg
            if msg is None or not msg.valid or not p.after_stamp<stamp_seconds(msg.header.stamp)<=self.now()+.05:
                raise ValueError('Chưa có phép đo mới sau khi dừng checkpoint')
            ready,reason=self.core.readiness(self.now(),time.monotonic())
            if not ready:raise ValueError(reason)
            odom=self.core.odom.at(stamp_seconds(msg.header.stamp))
            if odom is None or self.now()-stamp_seconds(msg.header.stamp)>.25:
                raise ValueError('Phép đo checkpoint/odom quá hạn')
            forward=msg.chassis_front_forward_m+.3005;c,s=math.cos(odom.yaw),math.sin(odom.yaw)
            opening=Opening(odom.x+c*forward-s*msg.opening_lateral_m,
                odom.y+s*forward+c*msg.opening_lateral_m,wrap(odom.yaw+msg.entry_heading_rad),msg.opening_width_m)
            p.complete_checkpoint_refresh(time.monotonic(),self.now(),opening)
            self.checkpoint_refresh_session=None
            response.success=True;response.message='Đã nhận diện hai chân mới; tiếp tục đo/căn cuối'
        except Exception as exc:response.success=False;response.message=str(exc)
        return response

    def prepare_checkpoint_route(self):
        # A distinct OUTSIDE-only route, never an ALIGNED certificate. The
        # same fresh data, whole-route laser and height checks run again at
        # preview/start; a prior check cannot authorize a changed scene.
        plan=self.prepare_plan(checkpoint_candidate=True)
        checkpoint_plan(plan.start,plan.opening,plan.entry_height_m,self.pickup.visual_config)
        self.core.stop('Đường góc nhỏ tới checkpoint đã kiểm tra; sẽ đo/căn lại trước đưa càng',
                       'CHECKPOINT_READY')
        self.publish_plan(plan);self.publish_status()

    def abort_pickup(self,reason,state='BLOCKED'):
        if (self.pickup.active and self.pickup.plan is not None
                and self.pickup.state in ('INSERTING','STOPPING') and self.core.odom.samples):
            self.stopped_opening=relative_opening(self.pickup.plan.opening,self.current_pose()[0])
            self.stopped_height=self.pickup.plan.entry_height_m
        self.pickup.stop(reason,state);self.zero()
        if self.lift_stop:self.lift_stop.publish(Empty())

    def lift_issue(self):
        if self.lift is None or time.monotonic()-self.lift_wall>.35:
            return 'Chưa có phản hồi càng mới, đã HOME và không lỗi'
        if self.execute:
            publishers=self.get_publishers_info_by_topic('/lift_controller/commands')
            subscribers=self.get_subscriptions_info_by_topic('/lift_controller/commands')
            stop_subscribers=self.get_subscriptions_info_by_topic('/lift/stop')
            if len(publishers)!=1 or publishers[0].node_name!='pallet_alignment':
                return 'Có nguồn nâng càng cạnh tranh'
            if (len(subscribers)!=1 or subscribers[0].node_name!='esp32_imu_bridge'
                    or len(stop_subscribers)!=1 or stop_subscribers[0].node_name!='esp32_imu_bridge'):
                return 'Bridge càng chưa có STOP hoặc có nhiều bộ điều khiển; cần mở stack mới'
        return ''

    def sensor_issue(self):
        issue=self.cycle_issue()
        if issue:return issue
        now,wall=self.now(),time.monotonic()
        if (not self.core.odom.samples or self.last_odom_wall is None
                or wall-self.last_odom_wall>.20
                or not -.05<=now-self.core.odom.samples[-1].stamp_s<=.20):return 'Odometry quá hạn'
        p=self.core.odom.samples[-1];c=self.pickup.config.insertion
        issue=measured_velocity_issue(p.linear,p.angular,c.braking_mps2,c.angular_braking_rps2)
        if issue:return issue
        if (self.recognition is None or wall-self.recognition_received>1.
                or self.yolo_enabled is None or wall-self.yolo_received>1.):return 'Mất heartbeat nhận diện'
        if (self.pickup.phase!='CHECKPOINT_RECOGNITION'
                and (self.yolo_enabled or self.recognition['enabled'] or self.recognition['busy'])):
            return 'YOLO đang bật/xử lý; dừng đưa càng'
        if self.operation is not None or self.cancel_pending or self.cancel_failed:
            return 'Nav2 chưa dừng/hủy xong'
        issue=self.lift_issue()
        if issue:return issue
        if self.execute:
            ready,reason=self.guard.ready(now,wall)
            if not ready:return reason
        if self.pickup_scan is None or wall-self.pickup_scan_wall>LASER_TIMEOUT_S:
            return 'Laser quá hạn'
        if not -.05<=now-stamp_seconds(self.pickup_scan.header.stamp)<=LASER_TIMEOUT_S:return 'Thời gian laser quá hạn'
        return ''

    def hold_checkpoint_scan(self,issue):
        """Zero motion during short scan loss; resume only after fresh recheck.

        This never grants permission to use stale data, insert or lift.
        Existing controller anchors and elapsed-time bounds are preserved.
        """
        laser_errors={'Laser quá hạn','Thời gian laser quá hạn',
                      'Laser /scan_filtered chưa có dữ liệu mới hợp lệ'}
        approach=self.pickup.checkpoint_approach
        if self.pickup.phase!='CHECKPOINT_APPROACH' or approach is None:return False
        if self.scan_pause_started is None and issue not in laser_errors:return False
        now=time.monotonic()
        self.pickup.v=self.pickup.w=0.;self.zero()
        if self.lift_stop:self.lift_stop.publish(Empty())
        if issue and issue not in laser_errors:
            self.abort_pickup(issue);return True
        if now-self.operator_wall>self.pickup.config.operator_timeout_s:
            self.abort_pickup('Mất heartbeat khi chờ laser');return True
        if self.scan_pause_started is None:
            self.scan_pause_count+=1
            self.scan_pause_started=now
            self.scan_pause_stamp=(stamp_seconds(self.pickup_scan.header.stamp) if self.pickup_scan else -math.inf)
            self.get_logger().warning('Laser trễ: giữ dừng, kiểm tra phục hồi tối đa 3 giây')
        if self.scan_pause_count>3 or now-self.scan_pause_started>3.:
            self.abort_pickup('Laser không phục hồi ổn định trong 3 giây hoặc trễ lặp quá 3 lần');return True
        pose,v,w=self.current_pose();c=self.pickup.config.insertion
        bad=approach.pose_issue(pose,approach.plan.opening,self.lift['position'],self.pickup.visual_config)
        if approach.phase.startswith('TURN'):
            bad=bad or pivot_stop_issue(pose,v,w,approach.pivot_anchor,c)
        else:
            cross=(-(pose[0]-approach.plan.start[0])*math.sin(approach.bearing)
                   +(pose[1]-approach.plan.start[1])*math.cos(approach.bearing))
            if abs(cross)>.015:bad='Xe lệch đường checkpoint trong lúc chờ laser'
        if self.lift['state']!='IDLE' or abs(self.lift['position']-approach.plan.entry_height_m)>.003:
            bad='Càng chuyển động khi chờ laser'
        if bad:self.abort_pickup(bad);return True
        good=not issue and abs(v)<=.003 and abs(w)<=.015
        if good:
            try:
                points=self.laser_points()
                good=stamp_seconds(self.pickup_scan.header.stamp)>self.scan_pause_stamp
                if any(footprint_hit(points,p,padding=.21) is not None
                       for p in braking_poses(pose,v,w,c)):
                    self.abort_pickup('Laser chặn xe khi kiểm tra phục hồi');return True
            except LaserTransformPending:good=False
            except Exception as exc:
                self.abort_pickup('Laser phục hồi không hợp lệ: '+str(exc));return True
        self.scan_pause_good_since=(self.scan_pause_good_since if self.scan_pause_good_since is not None else now) if good else None
        if self.scan_pause_good_since is not None and now-self.scan_pause_good_since>=.8:
            self.scan_pause_started=self.scan_pause_good_since=None
            self.pickup.reason='Laser đã mới và xe đứng yên 0,8 giây; kiểm tra lại quỹ đạo trước tiếp tục'
        else:self.pickup.reason='Tạm dừng chờ laser mới ổn định; chưa hủy chu trình'
        self.publish_status()
        return True

    def laser_points(self):
        msg=self.pickup_scan
        if msg is None:raise ValueError('Thiếu laser')
        if (not all(math.isfinite(v) for v in
                    (msg.angle_min,msg.angle_increment,msg.range_min,msg.range_max))
                or msg.angle_increment<=0
                or not 0<msg.range_min<msg.range_max):raise ValueError('Laser không hợp lệ')
        # /odom and /tf use separate DDS queues. Reuse the same capture-time
        # odom history as RGB-D, instead of waiting for its duplicate dynamic
        # odom TF. OdomHistory.at requires bracketing samples <=150 ms apart;
        # it never extrapolates. Sensor-to-base TF is queried at scan time too.
        last_error='Không có laser mới với pose tại thời điểm chụp'
        for candidate,received in reversed(self.pickup_scans):
            if (candidate.header.frame_id!=self.pickup_scan.header.frame_id
                    or time.monotonic()-received>LASER_TIMEOUT_S
                    or not -.05<=self.now()-stamp_seconds(candidate.header.stamp)<=LASER_TIMEOUT_S):continue
            pose=self.core.odom.at(stamp_seconds(candidate.header.stamp))
            if pose is None:
                last_error='Chưa có odom bao quanh thời điểm chụp laser';continue
            try:
                tf=self.tf_buffer.lookup_transform('base_footprint',candidate.header.frame_id,
                        rclpy.time.Time.from_msg(candidate.header.stamp))
                return capture_laser_points(candidate,(pose.x,pose.y,pose.yaw),tf.transform)
            except TransformException as exc:last_error=str(exc)
        raise LaserTransformPending(last_error)

    def current_pose(self):
        p=self.core.odom.samples[-1]
        return (p.x,p.y,p.yaw),p.linear,p.angular

    def prepare_plan(self,checkpoint_candidate=False):
        if (self.pickup.state!='IDLE'
                or self.pickup.active or self.core.active or self.preparing):
            raise ValueError('Dừng/reset bước trước trước khi đưa càng')
        checkpoint_route=(self.core.state=='CHECKPOINT_READY' or checkpoint_candidate or self.pickup.direct_mode or self.pickup.route_requested)
        if self.core.state!='ALIGNED' and not checkpoint_route:
            raise ValueError('Cần ALIGNED hoặc đường checkpoint đã kiểm tra trước')
        yaw_handoff=(self.core.state=='YAW_CORRECTED' and self.core.heading_only_trim)
        if checkpoint_candidate and not yaw_handoff and (self.core.state!='NO_SAFE_GOAL' or self.core.candidates
                                     or self.core.heading_only_trim):
            raise ValueError('Không được chuyển lỗi căn/vật cản sang đường checkpoint')
        ready,reason=self.core.readiness(self.now(),time.monotonic())
        if not ready:raise ValueError(reason)
        ready,reason=self.recognition_ready()
        if not ready:raise ValueError(reason)
        if not self.recognition.get('rgbd_health',{}).get('stable'):
            raise ValueError('RGB-D chưa ổn định đủ cửa sổ 5 giây')
        issue=self.sensor_issue()
        if issue:raise ValueError(issue)
        pose,v,w=self.current_pose()
        if abs(v)>.003 or abs(w)>.015:raise ValueError('Xe phải đứng yên trước khi chốt quỹ đạo')
        target=self.last_target_msg
        if (target is None or not target.valid or self.now()-stamp_seconds(target.header.stamp)>.25
                or 'target_bias_applied' not in target.status):raise ValueError('Cần phép đo mới đã bù RGB-D')
        c=self.pickup.config.insertion
        opening=Opening(*self.core.target,float(target.opening_width_m))
        _,lateral,heading,_=self.core.errors(self.core.odom.samples[-1])
        if not checkpoint_route and (abs(lateral)>=c.lateral_tolerance_m or abs(heading)>=c.heading_tolerance_rad):
            raise ValueError(f'Cần lệch ngang <{c.lateral_tolerance_m*100:g} cm '
                             f'và góc <{math.degrees(c.heading_tolerance_rad):g}° trước checkpoint')
        along,_=opening_coordinates(opening,pose)
        if -along-c.fork_root_m*math.cos(wrap(pose[2]-opening.yaw))<1.20:
            raise ValueError('Bắt đầu từ vị trí nhận diện/căn >=1,20 m')
        if self.lift['state']!='IDLE' or abs(self.lift['velocity'])>.002 or self.lift['position']>.01:
            raise ValueError('Càng cần đứng yên gần HOME (0–1 cm) trước khi đưa vào')
        nominal=(opening.x-(c.fork_root_m+c.root_stop_m)*math.cos(opening.yaw),
                 opening.y-(c.fork_root_m+c.root_stop_m)*math.sin(opening.yaw),opening.yaw)
        reference={'mode':'visual_checkpoint_then_straight_odom',
                   'saved_map_goal_used':False,'preview_scope':'checkpoint_only',
                   'usable_depth_m':self.pickup.visual_config.usable_depth_m,
                   'usable_width_m':self.pickup.visual_config.usable_width_m,
                   'pallet_geometry':'four_aligned_posts_open_rear',
                   'rear_clearance_m':self.pickup.visual_config.rear_clearance_m,
                   'rear_margin_m':self.pickup.visual_config.rear_margin_m}
        plan=self.pickup.preview(pose,opening,self.lift['position'])
        reference['checkpoint_correction_only']=checkpoint_route
        if checkpoint_route and not self.pickup.direct_mode and not self.pickup.visual_config.quarter_turn_lateral:
            plan=checkpoint_plan(pose,opening,self.lift['position'],self.pickup.visual_config)
        points=self.laser_points()
        for p in plan.poses:
            hit=footprint_hit(points,p,padding=.21)
            if hit:raise ValueError('Quỹ đạo đầy đủ thân/càng bị laser chặn: '+str(hit))
        dx,dy=plan.goal[0]-plan.start[0],plan.goal[1]-plan.start[1]
        reference['approach_geometry']={
            'start_odom':list(plan.start),'goal_odom':list(plan.goal),
            'travel_m':plan.travel_m,
            'path_turn_deg':math.degrees(wrap(math.atan2(dy,dx)-plan.start[2])) if plan.travel_m else 0.,
            'goal_axis_cross_m':opening_coordinates(opening,plan.goal)[1],
            'lateral_method':'quarter_turn' if self.pickup.visual_config.quarter_turn_lateral else 'diagonal'}
        self.pickup_reference=reference
        return plan

    def publish_plan(self,plan):
        path=NavPath();path.header.frame_id='odom';path.header.stamp=self.get_clock().now().to_msg()
        path.poses=[self.pose(p) for p in plan.poses];self.path_pub.publish(path)

    def preview_pickup(self,request,response):
        try:
            plan=self.prepare_plan();self.publish_plan(plan)
            response.success=True
            response.message=json.dumps(dict(plan=asdict(plan),reference=self.pickup_reference),allow_nan=False)
        except Exception as exc:response.success=False;response.message=str(exc)
        return response

    def arm_operator(self):
        self.pickup_id=uuid.uuid4().hex
        self.operator_wall=0.;self.lease_started=time.monotonic()
        self.previous_pickup_active=True

    def start_pickup(self,request,response):
        try:
            if not self.execute:raise ValueError('Chế độ xem trước; không gửi lệnh xe/càng')
            if request.data:raise ValueError('Dùng start.data=false; nâng bằng lệnh riêng sau kiểm tra')
            plan=self.prepare_plan()
            self.pickup.start(time.monotonic(),plan.start,plan.opening,plan.nominal_goal,
                plan.entry_height_m,bool(request.data),station_locked=plan.station_locked)
            if self.pickup.direct_mode:self.pickup.after_stamp=self.now()
            self.pending_target=None
            self.pickup_frame_valid=True
            self.arm_operator();self.publish_plan(plan)
            response.success=True;response.message=self.pickup_id
        except Exception as exc:response.success=False;response.message=str(exc)
        return response

    def start_lift(self,request,response):
        try:
            if not self.execute:raise ValueError('Chế độ xem trước')
            issue=self.sensor_issue()
            if issue:raise ValueError(issue)
            pose,v,w=self.current_pose()
            if abs(v)>.003 or abs(w)>.015:raise ValueError('Xe chưa đứng yên')
            if (not self.pickup_frame_valid or self.pickup.phase!='FINAL_INSERTION'
                    or self.pickup.plan is None
                    or not terminal_geometry(pose,v,w,self.pickup.plan,self.pickup.config.insertion)
                    or abs(wrap(pose[2]-self.pickup.plan.start[2]))>self.pickup.visual_config.final_heading_tolerance_rad):
                raise ValueError('Quỹ đạo odom/điểm dừng không còn hợp lệ; không nâng')
            self.pickup.begin_lift(time.monotonic(),pose,self.lift['position'])
            self.arm_operator();response.success=True;response.message=self.pickup_id
        except Exception as exc:response.success=False;response.message=str(exc)
        return response

    def stop_pickup(self,request,response):
        self.core.stop();self.cancel_actions()
        self.abort_pickup('Đã dừng theo yêu cầu','STOPPED')
        self.recovery_checkpoint=None;self.stopped_opening=self.stopped_height=None
        response.success=True;response.message='Đã dừng xe và càng';return response

    def reset_pickup(self,request,response):
        try:
            if self.pickup.active:raise ValueError('Dừng bước đang chạy trước')
            target=self.last_target_msg
            if (target is None or not target.valid or self.now()-stamp_seconds(target.header.stamp)>.25
                    or target.chassis_front_forward_m-.065<1.20):
                raise ValueError('Chỉ reset khi xe ở ngoài pallet >=1,20 m và nhận diện mới hợp lệ')
            pose,v,w=self.current_pose()
            if abs(v)>.003 or abs(w)>.015:raise ValueError('Xe chưa đứng yên')
            if self.lift_issue() or self.lift['position']>.01:raise ValueError('Càng chưa ở cao độ đưa vào')
            self.pickup=VisualPickupController(self.pickup.visual_config)
            self.scan_pause_started=self.scan_pause_good_since=self.scan_pause_stamp=None
            self.scan_pause_count=0
            self.core.invalidate_target('Đã reset; căn chỉnh lại trước lượt tiếp theo')
            self.core.state='IDLE';self.pickup_id=None
            response.success=True;response.message='Đã reset; cần căn chỉnh lại'
        except Exception as exc:response.success=False;response.message=str(exc)
        return response

    def tick(self):
        if not hasattr(self,'handling_status'):return
        if self.cycle_issue() and self.preparing:
            self.start_epoch+=1;self.preparing=False
            if self.start_waiter is not None and not self.start_waiter[0].done():
                self.start_waiter[0].set_result((False,self.cycle_issue()))
            self.core.stop(self.cycle_issue(),'BLOCKED');self.cancel_actions();self.zero()
        if self.pickup.state=='IDLE':
            super().tick()
            if self.core.state=='YAW_CORRECTED' and not self.core.active:
                try:self.prepare_checkpoint_route()
                except ValueError as exc:
                    reason=str(exc)
                    if reason.startswith('Chờ cửa sổ RGB-D ổn định') or reason in (
                            'RGB-D chưa ổn định đủ cửa sổ 5 giây',
                            'Chưa có phép đo RGB + Depth mới; xe phải dừng',
                            'Cần phép đo mới đã bù RGB-D'):
                        # Keep the stationary yaw handoff; retry its read-only
                        # route check next tick without replaying a turn.
                        self.core.reason='Đã quay bù; giữ dừng chờ RGB-D mới để kiểm tra đường tiếp cận: '+reason
                    else:
                        self.core.stop('Chưa có đường căn ngang sau quay an toàn: '+reason,'BLOCKED')
                    self.zero();self.publish_status()
        else:
            if not self.pickup.active:self.zero()
            if self.pickup.active:
                issue=self.sensor_issue();points=None
                if self.hold_checkpoint_scan(issue):return
                if not issue:
                    try:points=self.laser_points()
                    except LaserTransformPending as exc:
                        now=time.monotonic()
                        if self.laser_hold_since is None:
                            self.laser_hold_since=now;self.laser_hold_reason=self.pickup.reason
                            self.get_logger().warning('Tạm dừng chờ pose/TF laser: '+str(exc))
                        self.pickup.v=self.pickup.w=0.;self.zero()
                        if self.lift_stop:self.lift_stop.publish(Empty())
                        if now-self.laser_hold_since<=LASER_TRANSFORM_WAIT_S:
                            self.pickup.reason='Tạm dừng chờ pose/TF đúng thời điểm laser'
                            self.publish_status()
                            return
                        issue='Mất TF/laser quá 120 ms: '+str(exc)
                    except Exception as exc:issue='Mất TF/laser: '+str(exc)
                if points is not None and self.laser_hold_since is not None:
                    self.pickup.reason=self.laser_hold_reason
                    self.laser_hold_since=self.laser_hold_reason=None
                if self.core.odom.samples:
                    pose,v,w=self.current_pose();now=time.monotonic()
                    alive=(now-self.operator_wall<=self.pickup.config.operator_timeout_s)
                    # Brief admission grace permits the service reply and DDS
                    # discovery; motion still waits for the FIRST heartbeat.
                    if self.operator_wall==0 and now-self.lease_started<=1.5:
                        issue=issue or ''
                    else:
                        previous_plan=self.pickup.plan
                        previous_phase=self.pickup.phase
                        self.pickup.tick(now,pose,v,w,self.lift or {},issue,alive,ros_now=self.now())
                        if self.pickup.phase=='CHECKPOINT_RECOGNITION' and previous_phase!=self.pickup.phase:
                            self.checkpoint_refresh_session=(self.recognition or {}).get('session')
                        if self.pickup.plan is not previous_plan:
                            self.publish_plan(self.pickup.plan)
                            self.pickup_reference['preview_scope']='final_straight_odom'
                            self.pickup_reference['root_stop_m']=self.pickup.config.insertion.root_stop_m
                    if self.pickup.active and points is not None:
                        for vel in ((self.pickup.v,self.pickup.w),(v,w)):
                            if any(footprint_hit(points,p,padding=(.21 if self.pickup.phase=='CHECKPOINT_APPROACH'
                                    and self.pickup.checkpoint_approach is not None else .03)) is not None for p in
                                   braking_poses(pose,*vel,self.pickup.config.insertion)):
                                self.abort_pickup('Laser chặn footprint/quỹ đạo phanh');break
                    if self.pickup.active:
                        cmd=self.pose_command(self.pickup.v,self.pickup.w)
                        self.proposed.publish(cmd)
                        if self.command:self.command.publish(cmd)
                        if self.pickup.state in ('LIFTING','LOWERING') and now-self.last_lift_publish>=.09:
                            self.lift_command.publish(Float64MultiArray(data=[self.pickup.lift_target]))
                            self.last_lift_publish=now
                else:self.abort_pickup('Mất odometry')
            if self.previous_pickup_active and not self.pickup.active:
                if (self.pickup.plan is not None and self.pickup.state in ('BLOCKED','STOPPED')
                        and self.pickup.inserted_pose is None and self.core.odom.samples
                        and self.stopped_opening is None):
                    self.stopped_opening=relative_opening(self.pickup.plan.opening,self.current_pose()[0])
                    self.stopped_height=self.pickup.plan.entry_height_m
                if self.lift_stop:self.lift_stop.publish(Empty())
                self.previous_pickup_active=False
            if time.monotonic()-self.last_publish>=.20:self.publish_status()
        p=self.pickup
        data=dict(state=p.state,active=p.active,reason=p.reason,execute=self.execute,
            run_id=self.pickup_id,phase=p.phase,v=p.v,w=p.w,lift_target_m=p.lift_target,
            recovery_action=p.recovery_action,
            checkpoint_refresh_seq=p.checkpoint_refresh_seq,
            lift=self.lift,reference=self.pickup_reference,
            goal=list(p.plan.goal) if p.plan else None,
            opening=asdict(p.plan.opening) if p.plan else None,
            operator_heartbeat_required=p.active,frozen_resume_ready=False)
        self.handling_status.publish(String(data=json.dumps(data,allow_nan=False)))

    def publish_status(self):
        if self.pickup.state=='IDLE':
            return super().publish_status()
        p=self.pickup;errors=None;gap=None
        if p.plan and self.core.odom.samples:
            pose,_,_=self.current_pose()
            along,cross=opening_coordinates(p.plan.opening,pose)
            heading=wrap(pose[2]-p.plan.opening.yaw)
            gap=-along-self.pickup.config.insertion.fork_root_m*math.cos(heading)
            errors=dict(distance_m=gap-self.pickup.config.insertion.root_stop_m,
                        lateral_m=cross,heading_deg=math.degrees(heading))
        self.last_publish=time.monotonic()
        self.status.publish(String(data=json.dumps(dict(execute=self.execute,ready=False,
            active=p.active,preparing=False,state=p.state,reason=p.reason,ready_reason=p.reason,
            strategy='pallet_insertion_odom',standoff_m=self.pickup.config.insertion.root_stop_m,
            perception_refresh_allowed=bool(p.active and p.phase=='CHECKPOINT_RECOGNITION'
                and self.core.odom.samples and abs(self.current_pose()[1])<=.003
                and abs(self.current_pose()[2])<=.015 and not self.sensor_issue()),
            linear_mps=p.v,angular_rps=p.w,errors=errors,fork_plane_distance_m=gap),allow_nan=False)))

    @staticmethod
    def pose_command(v,w):
        from geometry_msgs.msg import Twist
        cmd=Twist();cmd.linear.x=v;cmd.angular.z=w;return cmd


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',type=Path,default=ROOT/'config/pallet_staging.yaml')
    parser.add_argument('--pickup-config',type=Path,default=ROOT/'config/pallet_pickup.yaml')
    parser.add_argument('--execute',action='store_true')
    parser.add_argument('--usable-depth-m',type=float,help='Measured clear hole depth, not pallet length')
    parser.add_argument('--cycle-token',help='Require a live supervisor heartbeat during a continuous cycle')
    parser.add_argument('--recovery-checkpoint',type=Path,
                        help='Nạp checkpoint đã dừng; vẫn chờ lệnh resume và RGB-D mới, không tự chạy')
    args,ros_args=parser.parse_known_args()
    values=yaml.safe_load(args.pickup_config.read_text());values['insertion']=InsertionConfig(**values['insertion'])
    if args.usable_depth_m is not None:values['usable_depth_m']=args.usable_depth_m
    rclpy.init(args=ros_args,signal_handler_options=SignalHandlerOptions.NO)
    if args.recovery_checkpoint:parser.error('Phương pháp cũ đã ngừng; không nạp checkpoint cũ')
    checkpoint=None
    node=PalletHandlingNode(StagingConfig(**yaml.safe_load(args.config.read_text())),PickupConfig(**values),args.execute,checkpoint,args.cycle_token)
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.core.stop();node.abort_pickup('Terminal bộ điều khiển đã đóng','STOPPED')
        if rclpy.ok():
            node.cancel_actions();deadline=time.monotonic()+.5
            while time.monotonic()<deadline:
                node.zero();rclpy.spin_once(node,timeout_sec=.02)
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
