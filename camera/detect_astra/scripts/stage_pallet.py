#!/usr/bin/env python3
"""Nav2 staging on the pallet normal, followed by stationary RGB-D yaw trim."""
import argparse
from concurrent.futures import ThreadPoolExecutor
from copy import copy
from types import SimpleNamespace
import numpy as np
import math
from pathlib import Path
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Path as NavPath
from nav2_msgs.action import ComputePathToPose, FollowPath
from lifecycle_msgs.srv import GetState
import yaml
from align_pallet import PalletAlignmentNode, ROOT, rgbd_wait_reason
from pallet_alignment_core import wrap
from pallet_staging_core import StagingConfig, StagingController, plane_clearance, bounded_nav_command
from pallet_staging_safety import FootprintGrid, approach_route_issue, swept_poses, route_heading_bounds, braking_poses
from pallet_laser_timing import measured_velocity_issue,stop_travel,MEASURED_STOP_LIMIT_M,STOP_POLYGON_PADDING_M,STOP_RESERVE_M,COMMAND_TRACKING_FACTOR,COMMAND_ANGULAR_TRACKING_FACTOR,measured_transition_scale


def route_pose(values,stamp):
    p=PoseStamped();p.header.frame_id='odom';p.header.stamp=stamp
    p.pose.position.x,p.pose.position.y=values[:2]
    p.pose.orientation.z,p.pose.orientation.w=math.sin(values[2]/2),math.cos(values[2]/2)
    return p


def check_route_snapshot(snapshot,path):
    # Only the isolated snapshot is mutable here. No node/core/action client
    # or publishers are accessed by the worker.
    route=PalletStagingNode.planned_route(snapshot,path)
    return SimpleNamespace(path=route,reason=snapshot.last_path_rejection,
        unknown=snapshot.last_path_unknown,bounds=snapshot.checked_heading_bounds,
        grid=snapshot.grid,grid_stamp=snapshot.grid_stamp,start=snapshot.core.odom.samples[-1])


def same_obstacles(a,b):
    if (a is None or b is None or any(getattr(a,k)!=getattr(b,k)
            for k in ('res','width','height','ox','oy'))):return False
    x=np.asarray(a.data,dtype=np.int8);y=np.asarray(b.data,dtype=np.int8)
    return np.array_equal((x<0)|(x>=100),(y<0)|(y>=100))


class PalletStagingNode(PalletAlignmentNode):
    def __init__(self,config=StagingConfig(),execute=False):
        self.grid=None
        self.grid_wall=0.
        self.grid_stamp=0.
        self.nav_checks={}
        self.nav_pending={}
        self.plan_client=None
        self.follow_client=None
        self.operation=None
        self.operation_key=None
        self.handles=[]
        self.future=None
        self.result_future=None
        self.handoff_result_future=None
        self.pending_grid_path=None
        self.grid_retry_stamp=None
        self.action_deadline=0.
        self.cancel_pending=[]
        self.cancel_failed=False
        self.raw_after=math.inf
        self.checked_heading_bounds=None
        self.route_pool=ThreadPoolExecutor(max_workers=1,thread_name_prefix='pallet_route_check')
        self.route_job=None
        self.route_input=None
        self.route_retry_deadline=None
        super().__init__(config,execute,controller=StagingController(config))
        self.plan_client=ActionClient(self,ComputePathToPose,'/pallet_nav/compute_path_to_pose')
        self.follow_client=ActionClient(self,FollowPath,'/pallet_nav/follow_path')
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid,'/pallet_nav/global_costmap/costmap',self.on_grid,qos)
        self.create_subscription(Twist,'/pallet/nav_cmd_vel',self.on_nav_command,1)
        self.nav_lifecycle={name:self.create_client(GetState,f'/pallet_nav/{name}/get_state')
                            for name in ('planner_server','controller_server')}
        self.create_timer(.4,self.poll_nav)

    def on_grid(self,msg):
        try:
            self.grid=FootprintGrid(msg)
            self.grid_wall=time.monotonic()
            self.grid_stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        except ValueError:
            self.grid=None

    def reject(self, reason):
        if self.core.navigating:
            self.pending_target = None
            self.core.invalidate_target(reason)
        else:
            terminal_reason = self.core.reason if not self.core.active and self.core.state != 'IDLE' else None
            super().reject(reason)
            if terminal_reason is not None:
                self.core.reason = terminal_reason

    def poll_nav(self):
        wall=time.monotonic()
        for name,client in self.nav_lifecycle.items():
            previous=self.nav_pending.get(name)
            if previous is not None and not previous[0].done():
                if wall-previous[1]<.8:
                    continue
                client.remove_pending_request(previous[0]); previous[0].cancel()
            if not client.service_is_ready():
                self.nav_checks[name]=(False,wall)
                continue
            future=client.call_async(GetState.Request())
            self.nav_pending[name]=(future,wall)
            def finished(f,key=name):
                try:
                    valid=not f.cancelled() and f.result().current_state.id==3
                except Exception:
                    valid=False
                self.nav_checks[key]=(valid,time.monotonic())
            future.add_done_callback(finished)

    def recognition_ready(self,require_measurement=True):
        if self.core.active:
            wall=time.monotonic()
            if (self.recognition is None or wall-self.recognition_received>1.
                    or self.yolo_enabled is None or wall-self.yolo_received>1.):
                return False,'Mất heartbeat nhận diện; hủy di chuyển'
            if self.yolo_enabled or self.recognition['enabled'] or self.recognition['busy']:
                return False,'YOLO đang chạy; hủy di chuyển'
            return True,''
        ready,reason=super().recognition_ready(require_measurement)
        if ready and not self.recognition.get('rgbd_health',{}).get('stable',False):
            return False,rgbd_wait_reason(self.recognition.get('rgbd_health'))
        return ready,reason

    def interlock(self):
        ready,reason=super().interlock()
        if not ready:
            return ready,reason
        if self.cancel_failed:
            return False,'Chưa xác nhận hủy Nav2; cần khởi động lại bộ căn'
        if self.cancel_pending and self.core.state!='NAV_CANCELING':
            return False,'Đang chờ Nav2 xác nhận hủy lệnh cũ'
        if self.plan_client is None:
            return False,'Đang kết nối Nav2'
        if not self.plan_client.server_is_ready() or not self.follow_client.server_is_ready():
            return False,'Chưa có planner/controller Nav2 riêng cho căn pallet'
        wall=time.monotonic()
        for name in ('planner_server','controller_server'):
            valid,received=self.nav_checks.get(name,(False,0.))
            if not valid or wall-received>1.:
                return False,'Nav2 chưa ACTIVE: '+name
        if self.grid is None or wall-self.grid_wall>.6 or not -.05<=self.now()-self.grid_stamp<=.6:
            return False,'Chưa có costmap odom mới'
        if self.execute:
            sources=self.get_publishers_info_by_topic('/pallet/nav_cmd_vel')
            if len(sources)!=1 or sources[0].node_name!='controller_server' or sources[0].node_namespace!='/pallet_nav':
                return False,'Nguồn vận tốc Nav2 không độc quyền'
        return True,''

    def on_nav_command(self,msg):
        if (not self.core.active or self.core.state!='NAVIGATING'
                or self.operation!='following' or self.now()<self.raw_after):
            return
        values=(msg.linear.x,msg.linear.y,msg.linear.z,msg.angular.x,msg.angular.y,msg.angular.z)
        c=self.core.config
        try:
            if not all(math.isfinite(v) for v in values) or any(abs(v)>1e-6 for v in values[1:5]):
                raise ValueError('Non-planar command')
            p=self.core.odom.samples[-1]
            near_goal=(self.core.goal is not None
                and math.hypot(p.x-self.core.goal[0],p.y-self.core.goal[1])<=.03
                and abs(wrap(p.yaw-self.core.frozen_target[2]))<=math.radians(5)
                and msg.linear.x<=.005)
            if (msg.linear.x<0 or near_goal) and self.result_future is not None and self.core.begin_axis_handoff(time.monotonic()):
                self.handoff_result_future=self.result_future
                self.zero()
                self.cancel_actions()
                return
            v,w=bounded_nav_command(msg.linear.x,msg.angular.z,c)
        except ValueError:
            self.core.stop('Nav2 phát vận tốc ngoài giới hạn; đã dừng: '+str(values),'BLOCKED')
            self.zero()
            return
        self.core.nav_v,self.core.nav_w=v,w
        self.core.nav_command_wall=time.monotonic()

    def cancel_actions(self):
        if self.route_job is not None:self.route_job.cancel()
        self.route_job=None;self.route_input=None;self.route_retry_deadline=None
        for handle in self.handles:
            if handle.status not in (GoalStatus.STATUS_SUCCEEDED,GoalStatus.STATUS_CANCELED,GoalStatus.STATUS_ABORTED):
                self.cancel_pending.append((handle.cancel_goal_async(),time.monotonic()+2.))
        self.handles=[]
        # An acceptance response can arrive after Stop. Cancel that exact goal
        # when it arrives; never let a late accepted FollowPath reopen the gate.
        if self.future is not None:
            def late(f):
                try:
                    handle=f.result()
                    if handle.accepted:
                        self.cancel_pending.append((handle.cancel_goal_async(),time.monotonic()+2.))
                except Exception:
                    self.cancel_failed=True
            if self.future.done():
                late(self.future)
            else:
                self.future.add_done_callback(late)
                self.cancel_pending.append((self.future,time.monotonic()+2.))
        self.future=self.result_future=None
        self.pending_grid_path=None
        self.operation=self.operation_key=None
        self.core.nav_v=self.core.nav_w=0.
        self.checked_heading_bounds=None

    def pose(self,values):
        x,y,yaw=values
        p=PoseStamped(); p.header.frame_id='odom'; p.header.stamp=self.get_clock().now().to_msg()
        p.pose.position.x,p.pose.position.y=x,y
        p.pose.orientation.z,p.pose.orientation.w=math.sin(yaw/2),math.cos(yaw/2)
        return p

    def planned_route(self,path):
        self.last_path_rejection = ''
        self.last_path_unknown = False
        self.checked_heading_bounds=None
        def reject(reason):
            self.last_path_rejection = reason
            return None
        if path.header.frame_id!='odom' or not path.poses:
            return reject('frame/empty: '+path.header.frame_id)
        poses=[]
        for p in path.poses:
            q=p.pose.orientation; x=p.pose.position.x; y=p.pose.position.y
            vals=(x,y,q.x,q.y,q.z,q.w)
            if (p.header.frame_id not in ('','odom') or not all(math.isfinite(v) for v in vals)
                    or abs(sum(v*v for v in vals[2:])-1)>.02 or abs(q.x)+abs(q.y)>.01):
                return reject('invalid pose/frame/quaternion: '+str(vals)+' '+p.header.frame_id)
            yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            poses.append((x,y,yaw))
        p=self.core.odom.samples[-1]
        start=(p.x,p.y,p.yaw)
        goal=self.core.goal
        # The candidate search allocates forward clearance for a shallow
        # diagonal. Check both small pivots and the complete straight corridor.
        # A lattice fallback must obey the same camera heading envelope.
        distance=math.hypot(goal[0]-p.x,goal[1]-p.y)
        travel_yaw=math.atan2(goal[1]-p.y,goal[0]-p.x) if distance>.005 else p.yaw
        corners=[start,(p.x,p.y,travel_yaw),(goal[0],goal[1],travel_yaw),goal]
        direct=[]
        for a,b in zip(corners,corners[1:]):
            direct.extend(swept_poses(a,b))
        direct_issue=approach_route_issue(direct,self.grid,self.core.frozen_target,self.core.config)
        cell=getattr(self.grid,'last_issue',None)
        self.last_path_unknown=bool(direct_issue.startswith('footprint:') and cell and cell[2]<0)
        if not direct_issue:
            shortcut=NavPath(); shortcut.header=self.pose(goal).header
            # Validate both full pivots above, but send RPP the translated
            # segment. Repeated-XY rotation primitives pin a cusp to the old
            # position; real skid drift can make RPP chase that cusp and turn
            # away from the segment. RPP performs its own checked initial and
            # terminal rotations using this segment and the final goal yaw.
            translated=list(swept_poses((p.x,p.y,travel_yaw),
                                       (goal[0],goal[1],travel_yaw)))
            translated[-1]=goal
            shortcut.poses=[self.pose(pose) for pose in translated]
            self.checked_heading_bounds=route_heading_bounds(direct,self.core.frozen_target[2])
            return shortcut
        if math.hypot(poses[0][0]-p.x,poses[0][1]-p.y)>.10:
            return reject('start: '+str(poses[0]))
        # The 5 cm SE2 lattice quantizes endpoints. Connect its final pose to
        # the exact normal-axis goal, and validate that entire connection.
        if math.hypot(poses[-1][0]-goal[0],poses[-1][1]-goal[1])>.075:
            return reject('end: '+str(poses[-1])+' goal='+str(goal))
        # Include the initial and final sweeps, not just the sampled path.
        if sum(math.hypot(b[0]-a[0],b[1]-a[1]) for a,b in zip(poses,poses[1:]))>3.:
            return reject('path length exceeds 3 m')
        checked=[start]+poses+[goal]
        issue=approach_route_issue(checked,self.grid,self.core.frozen_target,self.core.config)
        if issue:
            cell=getattr(self.grid,'last_issue',None)
            self.last_path_unknown |= bool(issue.startswith('footprint:') and cell and cell[2]<0)
            return reject('direct='+direct_issue+'; lattice='+issue)
        path.poses.append(self.pose(goal))
        self.checked_heading_bounds=route_heading_bounds(checked,self.core.frozen_target[2])
        return path

    def schedule_route_validation(self,path,retry_deadline=None):
        if self.route_job is not None:raise ValueError('Đang có kiểm tra quỹ đạo')
        p=self.core.odom.samples[-1]
        snapshot=SimpleNamespace(core=SimpleNamespace(odom=SimpleNamespace(samples=(p,)),
            goal=tuple(self.core.goal),frozen_target=tuple(self.core.frozen_target),config=self.core.config),
            grid=copy(self.grid),grid_stamp=self.grid_stamp)
        stamp=self.get_clock().now().to_msg()
        snapshot.pose=lambda values:route_pose(values,stamp)
        self.route_input=path;self.route_retry_deadline=retry_deadline
        self.route_job=self.route_pool.submit(check_route_snapshot,snapshot,path)
        self.operation='validating'
        self.action_deadline=retry_deadline if retry_deadline is not None else time.monotonic()+6.

    def poll_route_validation(self):
        if self.route_job is None or not self.route_job.done():return
        result=self.route_job.result();self.route_job=None
        path=self.route_input;retry_deadline=self.route_retry_deadline
        self.route_input=None;self.route_retry_deadline=None
        # Admit only fresh sensor/command-chain state on the ROS thread.
        # A completed worker result never owns permission to send FollowPath.
        ready,reason=self.interlock()
        data_ready,data_reason=self.core.readiness(self.now(),time.monotonic())
        if not ready or not data_ready:
            self.core.stop(reason if not ready else data_reason,'BLOCKED');self.cancel_actions();self.zero();return
        p=self.core.odom.samples[-1];start=result.start
        if (math.hypot(p.x-start.x,p.y-start.y)>.005
                or abs(wrap(p.yaw-start.yaw))>math.radians(.5)):
            self.core.stop('Xe đã dịch khi kiểm tra đường; cần căn lại','BLOCKED');self.cancel_actions();self.zero();return
        self.last_path_rejection=result.reason;self.last_path_unknown=result.unknown
        self.checked_heading_bounds=result.bounds
        if result.path is not None:
            if not same_obstacles(result.grid,self.grid):
                # A newer grid can be accepted only if lethal/unknown geometry
                # is unchanged. Otherwise validate against its new snapshot.
                self.schedule_route_validation(path,retry_deadline or self.action_deadline)
                return
            self.follow_checked_route(result.path);return
        if result.unknown:
            self.pending_grid_path=path;self.grid_retry_stamp=result.grid_stamp
            self.operation='waiting_grid'
            self.action_deadline=retry_deadline if retry_deadline is not None else time.monotonic()+2.
            self.get_logger().info('Candidate thiếu phủ laser; chờ costmap mới: '+result.reason)
            return
        self.get_logger().warning('Bỏ candidate '+str(self.core.candidate_index)+': '+result.reason)
        self.operation=self.operation_key=None;self.core.next_candidate()

    def destroy_node(self):
        if self.route_job is not None:self.route_job.cancel()
        self.route_pool.shutdown(wait=False,cancel_futures=True)
        return super().destroy_node()

    def follow_checked_route(self,path):
        self.pending_grid_path=None
        if not self.execute:
            self.core.stop('Goal và đường đã kiểm tra; chế độ xem trước không gửi FollowPath','PREVIEW_READY')
            self.operation=self.operation_key=None
            return
        goal=FollowPath.Goal();goal.path=path;goal.controller_id='FollowPath'
        goal.goal_checker_id='general_goal_checker';goal.progress_checker_id='progress_checker'
        self.future=self.follow_client.send_goal_async(goal)
        self.operation='follow_accept'
        self.action_deadline=time.monotonic()+3.

    def sync_navigation(self):
        pending=[]
        for future,deadline in self.cancel_pending:
            if future.done():
                try:
                    result=future.result()
                    if hasattr(result,'return_code') and result.return_code not in (0,3):
                        self.cancel_failed=True
                except Exception:
                    self.cancel_failed=True
            elif time.monotonic()>deadline:
                self.cancel_failed=True
            else:
                pending.append((future,deadline))
        self.cancel_pending=pending
        if self.core.active and self.core.state=='NAV_CANCELING':
            if self.cancel_failed:
                self.core.stop('Không xác nhận hủy Nav2 tại trục pallet; đã dừng','BLOCKED')
                self.zero()
            elif not self.cancel_pending and self.handoff_result_future is not None and self.handoff_result_future.done():
                try:
                    result=self.handoff_result_future.result()
                    if result.status not in (GoalStatus.STATUS_SUCCEEDED,GoalStatus.STATUS_CANCELED):
                        raise ValueError('terminal status='+str(result.status))
                    self.core.navigation_finished(True,self.now(),time.monotonic())
                    self.core.reason='Đã hủy đường tại trục pallet; chờ xe đứng yên và RGB-D mới'
                except Exception as exc:
                    self.core.stop('Lỗi kết thúc đường Nav2 tại trục pallet: '+str(exc),'BLOCKED')
                self.handoff_result_future=None
                self.zero()
            return
        if not self.core.active:
            self.handoff_result_future=None
        key=(self.core.run_id,self.core.plan_attempt,self.core.candidate_index)
        if not self.core.active or self.core.state not in ('PLANNING','NAVIGATING'):
            if self.operation is not None:
                self.cancel_actions()
            return
        if self.operation_key is not None and self.operation_key!=key:
            self.cancel_actions()
            return
        if self.cancel_pending or self.cancel_failed:
            return
        if self.operation is None and self.core.state=='PLANNING':
            goal=ComputePathToPose.Goal()
            goal.goal=self.pose(self.core.goal); goal.planner_id='StagingLattice'
            goal.use_start=False
            self.future=self.plan_client.send_goal_async(goal)
            self.operation,self.operation_key='plan_accept',key
            self.action_deadline=time.monotonic()+6.
            return
        if self.operation=='waiting_grid':
            # Missing scan coverage is never free space. Keep commands zero
            # and revalidate the SAME candidate only on a newer costmap.
            if time.monotonic()>self.action_deadline:
                self.get_logger().warning('Bỏ candidate '+str(self.core.candidate_index)+
                    ' sau chờ costmap 2 giây: '+self.last_path_rejection)
                self.pending_grid_path=None
                self.operation=self.operation_key=None
                self.core.next_candidate()
            elif self.grid_stamp>self.grid_retry_stamp:
                self.grid_retry_stamp=self.grid_stamp
                self.schedule_route_validation(self.pending_grid_path,self.action_deadline)
            return
        if (self.operation=='validating' and self.route_retry_deadline is not None
                and time.monotonic()>self.action_deadline):
            if self.route_job is not None:self.route_job.cancel()
            self.route_job=None;self.route_input=None;self.route_retry_deadline=None
            self.pending_grid_path=None;self.operation=self.operation_key=None
            self.core.next_candidate();return
        if time.monotonic()>self.action_deadline:
            self.core.stop('Nav2 quá hạn phản hồi; đã hủy lệnh','NAV_FAILED')
            self.cancel_actions(); self.zero()
            return
        if self.operation=='validating':
            try:self.poll_route_validation()
            except Exception as exc:
                self.get_logger().error('Kiểm tra đường: '+str(exc))
                self.core.stop('Lỗi kiểm tra quỹ đạo; đã dừng','NAV_FAILED');self.cancel_actions();self.zero()
            return
        try:
            if self.operation in ('plan_accept','follow_accept') and self.future.done():
                handle=self.future.result(); self.future=None
                if not handle.accepted:
                    if self.operation=='plan_accept':
                        self.operation=self.operation_key=None
                        self.core.next_candidate()
                    else:
                        self.core.navigation_finished(False,self.now(),time.monotonic())
                    return
                self.handles.append(handle)
                self.result_future=handle.get_result_async()
                if self.operation=='plan_accept':
                    self.operation='planning'
                else:
                    self.operation='following'
                    self.core.navigation_started()
                    self.raw_after=self.now()+.10
                    self.action_deadline=time.monotonic()+self.core.config.nav_timeout_s
            if self.operation=='planning' and self.result_future.done():
                result=self.result_future.result()
                self.handles=[]
                valid=(result.status==GoalStatus.STATUS_SUCCEEDED and result.result.error_code==0)
                self.last_path_rejection = ''
                self.last_path_unknown = False
                if valid:
                    self.schedule_route_validation(result.result.path)
                    return
                self.get_logger().warning('Bỏ candidate '+str(self.core.candidate_index)+
                    ': planner='+str(result.result.error_code))
                self.operation=self.operation_key=None;self.core.next_candidate()
            if self.operation=='following' and self.result_future.done():
                result=self.result_future.result()
                self.handles=[]
                self.operation=self.operation_key=None
                self.core.navigation_finished(result.status==GoalStatus.STATUS_SUCCEEDED and result.result.error_code==0,
                                              self.now(),time.monotonic())
                self.zero()
        except Exception as exc:
            self.get_logger().error('Nav2: '+str(exc))
            self.core.stop('Lỗi phản hồi Nav2; đã dừng','NAV_FAILED')
            self.cancel_actions(); self.zero()

    def tick(self):
        if self.plan_client is None:
            return
        self.sync_navigation()
        self.core.post_measurement_stable = bool(
            self.recognition and time.monotonic()-self.recognition_received<=1.
            and self.recognition.get('rgbd_health',{}).get('stable',False)
            and self.core.arrival_wall is not None
            and time.monotonic()-self.core.arrival_wall>=5.)
        super().tick()
        if not self.core.active and self.operation is not None:
            self.cancel_actions(); self.zero()

    def before_command_publish(self):
        # Validate current and braking-horizon footprint for BOTH command
        # sources, including the RGB-D-only final stationary rotation.
        if self.core.active and (self.core.v or self.core.w):
            p=self.core.odom.samples[-1]
            c=self.core.config
            issue=measured_velocity_issue(p.linear,p.angular,c.linear_accel_mps2,c.angular_accel_rps2)
            if issue:
                self.get_logger().warning(issue)
                self.core.stop(issue,'BLOCKED')
                self.cancel_actions();self.zero();self.publish_status();return
            scale=measured_transition_scale(self.core.v,self.core.w,p.linear,p.angular,
                c.linear_accel_mps2,c.angular_accel_rps2)
            self.core.v*=scale;self.core.w*=scale
            target=self.core.frozen_target if self.core.navigating else self.core.target
            x,y,yaw=p.x,p.y,p.yaw
            relative_yaw=wrap(yaw-target[2])
            bounds=(self.checked_heading_bounds if self.core.navigating else
                    (-math.radians(c.max_face_yaw_deg),math.radians(c.max_face_yaw_deg)))
            if bounds is None or not bounds[0] <= relative_yaw <= bounds[1]:
                detail=('chưa có dải hướng' if bounds is None else
                    f'đo {math.degrees(relative_yaw):.2f}°, dải '
                    f'[{math.degrees(bounds[0]):.2f}°, {math.degrees(bounds[1]):.2f}°]')
                self.core.stop('Hướng xe lệch khỏi đường đã kiểm tra; '+detail+'; đã dừng','BLOCKED')
                self.cancel_actions(); self.zero(); self.publish_status()
                return
            safe=True
            # Check both the requested and measured motion through a complete
            # reaction+braking sweep, including the actual fork-root heading.
            # A constant-speed second falsely extrapolated past a safe goal.
            velocities={(self.core.v,self.core.w),(p.linear,p.angular)}
            for v,w in velocities:
                for braking_pose in braking_poses((x,y,yaw),v,w,c):
                    if (plane_clearance(target,braking_pose,c.fork_root_m)<c.standoff_m+.02
                            or self.grid is None or not self.grid.clear(braking_pose)):
                        safe=False; break
                if not safe:break
            if not safe:
                self.core.stop('Quỹ đạo phanh/quay không đủ khoảng cách hoặc footprint bị chặn','BLOCKED')
                self.cancel_actions(); self.zero(); self.publish_status()

    def status_details(self):
        if not self.core.odom.samples:return {}
        p=self.core.odom.samples[-1];c=self.core.config
        return dict(measured_linear_mps=p.linear,measured_angular_rps=p.angular,
            laser_stop_travel_m=stop_travel(p.linear,p.angular,c.linear_accel_mps2,c.angular_accel_rps2),
            laser_command_budget_m=STOP_POLYGON_PADDING_M-STOP_RESERVE_M,
            laser_command_tracking_factor=COMMAND_TRACKING_FACTOR,
            laser_angular_tracking_factor=COMMAND_ANGULAR_TRACKING_FACTOR,
            laser_measured_limit_m=MEASURED_STOP_LIMIT_M)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',type=Path,default=ROOT/'config/pallet_staging.yaml')
    parser.add_argument('--execute',action='store_true')
    args,ros_args=parser.parse_known_args()
    config=StagingConfig(**yaml.safe_load(args.config.read_text()))
    rclpy.init(args=ros_args)
    node=PalletStagingNode(config,args.execute)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.core.stop()
        if rclpy.ok():
            node.zero(); node.cancel_actions()
            deadline=time.monotonic()+.5
            while node.cancel_pending and time.monotonic()<deadline:
                rclpy.spin_once(node,timeout_sec=.02)
                node.zero()
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
