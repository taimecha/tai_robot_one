"""Supervised final approach and lift, using a frozen pallet and live odometry.

No camera or under-fork contact measurement is invented during insertion.
The ROS owner supplies sensor/command-chain checks and an operator heartbeat.
"""
from dataclasses import dataclass, field
import math

from pallet_insertion_core import (InsertionConfig, make_plan, geometry_issue,
    braking_poses, opening_coordinates, terminal_geometry, wrap)


@dataclass(frozen=True)
class PickupConfig:
    insertion: InsertionConfig = field(default_factory=lambda: InsertionConfig(
        lateral_tolerance_m=.01, heading_tolerance_rad=math.radians(1),
        longitudinal_tolerance_m=.04))
    lift_raise_m: float = .08
    settle_s: float = .8
    insertion_timeout_s: float = 35.
    lift_timeout_s: float = 25.
    progress_timeout_s: float = 5.
    operator_timeout_s: float = .6
    tracking_tolerance_m: float = .015
    usable_depth_m: float = 0.  # Unknown until measured; pallet length is NOT hole clearance.
    usable_width_m: float = .255
    rear_clearance_m: float = 0.  # Unknown means no extension past the pallet corridor.
    rear_margin_m: float = .020
    quarter_turn_lateral: bool = False
    checkpoint_root_gap_m: float = 1.10
    remeasure_s: float = 5.
    remeasure_timeout_s: float = 20.
    final_heading_tolerance_rad: float = math.radians(.5)
    final_tracking_tolerance_m: float = .005

    def __post_init__(self):
        if type(self.quarter_turn_lateral) is not bool:raise ValueError("Invalid lateral correction mode")
        if not all(math.isfinite(v) and v>0 for k,v in vars(self).items() if k not in ('insertion','usable_depth_m','rear_clearance_m','quarter_turn_lateral')):
            raise ValueError('Invalid pickup configuration')
        if (not .02 <= self.lift_raise_m <= .12 or self.settle_s<.8
                or self.insertion_timeout_s>60 or self.lift_timeout_s>40
                or self.operator_timeout_s>.6 or self.tracking_tolerance_m>.02):
            raise ValueError('Pickup limits exceed supported range')
        if (not math.isfinite(self.usable_depth_m) or not 0 <= self.usable_depth_m <= .80
                or (self.usable_depth_m != 0 and self.usable_depth_m <= max(.10,self.rear_margin_m))
                or self.rear_margin_m < .02 or not .172 <= self.usable_width_m <= .332
                or not math.isfinite(self.rear_clearance_m) or not 0 <= self.rear_clearance_m <= 1.
                or not .65 <= self.checkpoint_root_gap_m <= 1.20
                or self.remeasure_s < 5 or self.remeasure_timeout_s < self.remeasure_s
                or self.final_heading_tolerance_rad > math.radians(.5)
                or self.final_tracking_tolerance_m > .005):
            raise ValueError('Invalid visual insertion/depth limits')


class PickupController:
    def __init__(self, config=PickupConfig()):
        self.config=config
        self.state='IDLE'
        self.reason='Chờ căn chỉnh xong'
        self.plan=None
        self.active=False
        self.v=self.w=0.
        self.lift_target=None
        self.raise_after=False
        self.started=self.progress_at=self.phase_started=None
        self.best_remaining=math.inf
        self.settle_at=None
        self.inserted_pose=None

    def start(self,now,pose,opening,nominal_goal,height,raise_after,preserve_lateral=False,station_locked=False):
        if self.active or self.state!='IDLE':
            raise ValueError('Bộ đưa càng chưa ở trạng thái IDLE')
        self.plan=make_plan(pose,nominal_goal,opening,height,self.config.insertion,preserve_lateral)
        if station_locked:
            from pallet_pickup_safety import lock_saved_station_plan
            self.plan=lock_saved_station_plan(self.plan)
        self.active=True
        self.state='INSERTING'
        self.reason='Bám đường đã chốt bằng odometry'
        self.raise_after=raise_after
        self.started=self.progress_at=self.phase_started=now
        self.best_remaining=self.plan.travel_m
        self.v=self.w=0.
        return self.plan

    def stop(self,reason='Đã dừng theo yêu cầu',state='STOPPED'):
        self.active=False
        self.state=state
        self.reason=reason
        self.v=self.w=0.
        self.lift_target=None

    def begin_lift(self,now,pose,current_height):
        if self.state!='INSERTED' or self.inserted_pose is None:
            raise ValueError('Chỉ nâng sau khi đưa càng xong và xe đứng yên')
        if math.hypot(pose[0]-self.inserted_pose[0],pose[1]-self.inserted_pose[1])>.005:
            raise ValueError('Xe đã dịch sau khi đưa càng')
        if abs(current_height-self.plan.entry_height_m)>.003:
            raise ValueError('Cao độ càng thay đổi sau khi đưa vào')
        self.lift_target=current_height+self.config.lift_raise_m
        if self.lift_target>.20:
            raise ValueError('Cao độ nâng vượt giới hạn lượt lấy pallet')
        self.active=True
        self.state='LIFTING'
        self.phase_started=now
        self.settle_at=None
        self.reason='Xe đứng yên; nâng càng theo phản hồi hành trình'

    def begin_lower(self,now,pose,current_height):
        if self.state!='COMPLETE' or self.plan is None or self.inserted_pose is None:
            raise ValueError('Chỉ hạ lại pallet sau lượt nâng đã hoàn tất')
        if (math.hypot(pose[0]-self.inserted_pose[0],pose[1]-self.inserted_pose[1])>.005
                or abs(wrap(pose[2]-self.inserted_pose[2]))>math.radians(1)):
            raise ValueError('Xe đã dịch sau khi nâng')
        if not .002<current_height-self.plan.entry_height_m<=.12:
            raise ValueError('Cao độ hạ không phù hợp lượt nâng đã chốt')
        self.lift_target=self.plan.entry_height_m
        self.state='LOWERING';self.active=True;self.phase_started=now;self.settle_at=None
        self.v=self.w=0.;self.reason='Giữ xe đứng yên; hạ pallet về cao độ đưa càng'

    def tick(self,now,pose,linear,angular,lift,issue='',operator_alive=True):
        self.v=self.w=0.
        if not self.active:return
        if issue or not operator_alive:
            self.stop(issue or 'Mất heartbeat terminal người vận hành','BLOCKED');return
        c=self.config.insertion
        if abs(linear)>c.maximum_measured_speed_mps or abs(angular)>.15:
            self.stop('Vận tốc đo vượt giới hạn đưa càng','BLOCKED');return
        if self.state in ('LIFTING','LOWERING'):
            lowering=self.state=='LOWERING'
            if (math.hypot(pose[0]-self.inserted_pose[0],pose[1]-self.inserted_pose[1])>.005
                    or abs(wrap(pose[2]-self.inserted_pose[2]))>math.radians(1)
                    or abs(linear)>.005 or abs(angular)>.025):
                self.stop('Xe dịch chuyển trong lúc nâng','BLOCKED');return
            if lowering:
                if (lift['lower_active'] and lift['position']>self.lift_target+.003
                        or lift['position']<self.lift_target-.004):
                    self.stop('Phản hồi giới hạn/cao độ hạ không khớp','BLOCKED');return
                if now-self.phase_started>self.config.lift_timeout_s:
                    self.stop('Hạ càng quá thời gian','TIMEOUT');return
                at_goal=(abs(lift['position']-self.lift_target)<=.002
                         and abs(lift['velocity'])<.002 and lift['state']=='IDLE')
                self.settle_at=(self.settle_at if self.settle_at is not None else now) if at_goal else None
                if self.settle_at is not None and now-self.settle_at>=self.config.settle_s:
                    self.stop('Đã hạ càng; chờ lệnh tiến lại riêng','LOWERED')
                return
            if lift['upper_active'] and lift['position']<self.lift_target-.003:
                self.stop('Chạm giới hạn trên trước cao độ yêu cầu','BLOCKED');return
            if lift['position']>self.lift_target+.004:
                self.stop('Càng vượt cao độ nâng yêu cầu','BLOCKED');return
            if now-self.phase_started>self.config.lift_timeout_s:
                self.stop('Nâng càng quá thời gian','TIMEOUT');return
            at_goal=(abs(lift['position']-self.lift_target)<=.002
                     and abs(lift['velocity'])<.002 and lift['state']=='IDLE')
            self.settle_at=(self.settle_at if self.settle_at is not None else now) if at_goal else None
            if self.settle_at is not None and now-self.settle_at>=self.config.settle_s:
                self.stop('Đã đạt cao độ càng yêu cầu; kiểm tra pallet thực tế','COMPLETE')
            return
        if abs(lift['position']-self.plan.entry_height_m)>.003 or lift['state']!='IDLE':
            self.stop('Cao độ càng thay đổi trong lúc tiến','BLOCKED');return
        bad=geometry_issue(pose,self.plan.opening,self.plan.entry_height_m,c)
        if bad:
            self.stop(bad,'BLOCKED');return
        bearing=math.atan2(self.plan.goal[1]-self.plan.start[1],self.plan.goal[0]-self.plan.start[0])
        cross=-(pose[0]-self.plan.start[0])*math.sin(bearing)+(pose[1]-self.plan.start[1])*math.cos(bearing)
        if abs(cross)>=self.config.tracking_tolerance_m:
            self.stop('Xe lệch khỏi đường đưa càng đã chốt','BLOCKED');return
        if now-self.started>self.config.insertion_timeout_s:
            self.stop('Đưa càng quá thời gian','TIMEOUT');return
        if self.state=='STOPPING':
            still=abs(linear)<=.003 and abs(angular)<=.015
            self.settle_at=(self.settle_at if self.settle_at is not None else now) if still else None
            if self.settle_at is not None and now-self.settle_at>=self.config.settle_s:
                if not terminal_geometry(pose,linear,angular,self.plan,c):
                    self.stop('Đã dừng nhưng chưa vào đủ vùng kết thúc; không tự nâng','BLOCKED');return
                self.inserted_pose=tuple(pose)
                self.active=False;self.state='INSERTED';self.reason='Đưa càng xong; xe đứng yên'
                if self.raise_after:self.begin_lift(now,pose,lift['position'])
            return
        along,_=opening_coordinates(self.plan.opening,pose)
        heading=wrap(pose[2]-self.plan.opening.yaw)
        gap=-along-c.fork_root_m*math.cos(heading)
        remaining=math.hypot(self.plan.goal[0]-pose[0],self.plan.goal[1]-pose[1])
        if remaining<self.best_remaining-.005:
            self.best_remaining=remaining;self.progress_at=now
        braking=abs(linear)*c.reaction_s+linear**2/(2*c.braking_mps2)
        if gap<=max(c.root_stop_m+min(.002,c.longitudinal_tolerance_m/2),c.minimum_root_gap_m+braking+.005):
            self.state='STOPPING';self.reason='Phanh dừng trước mặt pallet';self.settle_at=None;return
        if now-self.progress_at>self.config.progress_timeout_s:
            self.stop('Xe không tiến; có thể bị LiDAR chặn hoặc truyền động đứng','BLOCKED');return
        w=max(-c.maximum_yaw_rate_rps,min(c.maximum_yaw_rate_rps,
            -1.8*wrap(pose[2]-bearing)-1.8*cross))
        # Approach the goal with enough remaining distance for reaction and
        # braking. A constant 0.10 m/s previously triggered the protection
        # stop early instead of reaching the requested root gap. Leave a
        # 40% speed reserve; still check the complete commanded/measured
        # braking sweeps below, independent of this speed calculation.
        distance=max(0.,gap-c.root_stop_m)
        reaction_velocity=c.braking_mps2*c.reaction_s
        allowed=math.sqrt(reaction_velocity**2+2*c.braking_mps2*distance)-reaction_velocity
        v=min(c.maximum_speed_mps,.6*allowed)
        for velocity in ((v,w),(linear,angular)):
            for p in braking_poses(pose,*velocity,c):
                brake_cross=(-(p[0]-self.plan.start[0])*math.sin(bearing)
                             +(p[1]-self.plan.start[1])*math.cos(bearing))
                if abs(brake_cross)>=self.config.tracking_tolerance_m:
                    self.stop('Quỹ đạo phanh lệch khỏi đường đưa càng đã chốt','BLOCKED');return
                bad=geometry_issue(p,self.plan.opening,self.plan.entry_height_m,c)
                if bad:
                    if 'Body/fork-root' in bad:
                        self.state='STOPPING';self.reason='Dừng theo dự báo phanh';self.settle_at=None
                    else:self.stop('Quỹ đạo phanh không lọt khe: '+bad,'BLOCKED')
                    return
        self.v,self.w=v,w
