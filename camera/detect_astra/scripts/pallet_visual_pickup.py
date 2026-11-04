"""Camera checkpoint followed by one immutable straight path in odom.

No saved map pickup pose is used. Camera capture timestamps, not reception
order, qualify the stationary observation window. Sensor/lease/laser guards
remain owned by handle_pallet.py during every phase.
"""
from collections import deque
from dataclasses import replace
import math

from pallet_pickup_core import PickupController, PickupConfig
from pallet_insertion_core import (Opening, InsertionPlan,
    geometry_issue, opening_coordinates, fork_vertices, braking_poses, wrap)
from pallet_checkpoint import checkpoint_plan,CheckpointApproach,pivot_stop_issue,checkpoint_pose_issue,quarter_turn_plan


def checkpoint_config(config):
    return replace(config, insertion=replace(config.insertion,
        root_stop_m=config.checkpoint_root_gap_m,
        minimum_root_gap_m=config.checkpoint_root_gap_m-.025))


def final_config(config):
    if config.usable_depth_m<=0:
        raise ValueError('Chưa đo độ sâu khoang giữa các trụ theo hướng đưa càng')
    c=config.insertion
    # Four aligned corner posts leave an open rear corridor. Its rear plane
    # is NOT a wall. Bound depth by the outside obstacle only when its clear
    # distance is supplied; otherwise retain the conservative corridor bound.
    # Lateral swept geometry independently keeps both forks between the front
    # and rear post columns, even when the tips pass the rear plane.
    limit=config.usable_depth_m+config.rear_clearance_m-config.rear_margin_m
    minimum=max(c.minimum_root_gap_m, c.fork_length_m+c.fork_half_span_m*math.sin(config.final_heading_tolerance_rad) \
        -limit)
    return replace(config, insertion=replace(c,
        root_stop_m=max(c.root_stop_m,minimum+c.longitudinal_tolerance_m+.001),
        minimum_root_gap_m=max(c.minimum_root_gap_m,minimum),
        heading_tolerance_rad=config.final_heading_tolerance_rad,
        lateral_tolerance_m=min(c.lateral_tolerance_m,.010)),
        tracking_tolerance_m=config.final_tracking_tolerance_m)


def residual_insertion_config(config):
    cfg=final_config(config)
    return replace(cfg,insertion=replace(cfg.insertion,
        heading_tolerance_rad=math.radians(1.5), lateral_tolerance_m=.050))


def accepted_straight_config(pose, opening, height, config):
    """Admit small residuals only when the full straight fork sweep fits."""
    dx,dy=opening.x-pose[0],opening.y-pose[1]
    lateral=-math.sin(pose[2])*dx+math.cos(pose[2])*dy
    if abs(lateral)>=.050 or abs(wrap(pose[2]-opening.yaw))>=math.radians(1.5):
        return None
    cfg=residual_insertion_config(config)
    try:straight_plan(pose,opening,height,cfg)
    except ValueError:return None
    return cfg


def depth_issue(pose, opening, config):
    depth=max(opening_coordinates(opening,p)[0] for p in fork_vertices(pose,config.insertion))
    if depth>config.usable_depth_m+config.rear_clearance_m-config.rear_margin_m+1e-9:
        return 'Đầu càng vượt giới hạn chiều sâu và khoảng trống phía sau đã khai báo'
    return ''


def straight_plan(pose, opening, height, config):
    opening=replace(opening,width_m=min(opening.width_m,config.usable_width_m))
    c=config.insertion
    along,_=opening_coordinates(opening,pose)
    heading=wrap(pose[2]-opening.yaw)
    if abs(heading)>c.heading_tolerance_rad:
        raise ValueError('Cần căn lại góc trước khi chốt đường thẳng')
    distance=(-along-c.fork_root_m*math.cos(heading)-c.root_stop_m)/math.cos(heading)
    if not 0<distance<=c.maximum_travel_m:raise ValueError('Quãng đường thẳng không hợp lệ')
    goal=(pose[0]+distance*math.cos(pose[2]),pose[1]+distance*math.sin(pose[2]),pose[2])
    steps=math.ceil(distance/.005)
    poses=tuple((pose[0]+distance*i/steps*math.cos(pose[2]),
                 pose[1]+distance*i/steps*math.sin(pose[2]),pose[2]) for i in range(steps+1))
    for p in poses:
        issue=geometry_issue(p,opening,height,c) or depth_issue(p,opening,config)
        if issue:raise ValueError('Không có đường đưa càng thẳng an toàn: '+issue)
    return InsertionPlan(tuple(pose),goal,goal,opening,height,0.,distance,poses)


class VisualPickupController(PickupController):
    def __init__(self,config=PickupConfig()):
        self.visual_config=config
        super().__init__(checkpoint_config(config))
        self.phase='IDLE'
        self.samples=deque(maxlen=600)
        self.after_stamp=-math.inf
        self.last_sample_stamp=-math.inf
        self.observation=None
        self.observed_wall=-math.inf
        self.frozen_opening=None
        self.turn_started=None
        self.turning=False
        self.trim_anchor=None
        self.trim_braking=False
        self.trim_settle=None
        self.trim_best_error=math.inf
        self.trim_progress_at=None
        self.checkpoint_approach=None
        self.require_checkpoint_refresh=False
        self.checkpoint_refresh_pose=None
        self.recovery_action=None
        self.checkpoint_refresh_seq=0
        self.refresh_settle=None
        self.measurement_wait_since=None
        self.route_requested=False
        self.direct_mode=False

    def stop(self,reason='Đã dừng theo yêu cầu',state='STOPPED'):
        self.recovery_action=None
        super().stop(reason,state)

    def require_new_approach(self,reason):
        self.stop(reason,'BLOCKED')
        # Only called before admitting the straight insertion, with the
        # outside-pallet clearance check already satisfied in this tick.
        if self.phase=='FINAL_ALIGN':
            self.recovery_action='return_to_recognition'

    def preview(self,pose,opening,height):
        final_config(self.visual_config)  # Fail before any approach when clearance is unknown.
        opening=replace(opening,width_m=min(opening.width_m,self.visual_config.usable_width_m))
        if self.direct_mode:
            dx,dy=opening.x-pose[0],opening.y-pose[1]
            lateral=-math.sin(pose[2])*dx+math.cos(pose[2])*dy
            if abs(lateral)>=.050:raise ValueError('Nhánh trực tiếp cần ngang dưới 5 cm')
            angle=abs(wrap(pose[2]-opening.yaw))
            if angle>=math.radians(12):raise ValueError('Góc quay bù trực tiếp quá lớn')
            cfg=accepted_straight_config(pose,opening,height,self.visual_config)
            end=tuple(pose) if cfg else (pose[0],pose[1],opening.yaw)
            cfg=cfg or residual_insertion_config(self.visual_config)
            # Full future straight sweep must fit even if only pivoting now.
            straight_plan(end,opening,height,cfg)
            from pallet_staging_safety import swept_poses
            poses=tuple(swept_poses(pose,end))
            for point in poses:
                if checkpoint_pose_issue(point,opening,height,self.visual_config):
                    raise ValueError('Thiếu khoảng trống ngoài pallet để quay bù trực tiếp')
            return InsertionPlan(tuple(pose),end,end,opening,height,0.,0.,poses)
        planner=quarter_turn_plan if self.visual_config.quarter_turn_lateral else checkpoint_plan
        return planner(pose,opening,height,self.visual_config)

    def start(self,now,pose,opening,nominal_goal,height,raise_after,**unused):
        self.plan=self.preview(pose,opening,height)
        self.recovery_action=None
        self.config=checkpoint_config(self.visual_config)
        self.active=True;self.state='INSERTING';self.raise_after=False
        self.started=self.progress_at=self.phase_started=now
        self.best_remaining=self.plan.travel_m;self.v=self.w=0.
        if self.direct_mode:
            self.checkpoint_approach=None
            self.phase='REMEASURE';self.state='REMEASURING'
            self.samples.clear();self.observation=None;self.settle_at=None
            self.reason='Ngang nhỏ: đo tại chỗ, quay bù nếu cần rồi đưa càng thẳng; bỏ tiến chéo/checkpoint'
            return self.plan
        self.checkpoint_approach=CheckpointApproach(self.plan,self.visual_config,now,quarter_turn=self.visual_config.quarter_turn_lateral)
        # Lift is always a separate operator-confirmed action in this method.
        self.phase='CHECKPOINT_APPROACH'
        self.reason=('Căn hướng, quay 90°, tiến bù ngang rồi quay lại để nhận diện mới'
                     if self.visual_config.quarter_turn_lateral else 'Tới điểm đo lại; đầu càng còn ở ngoài pallet')
        return self.plan

    def observe(self,stamp,wall,opening,healthy=True):
        if self.phase not in ('REMEASURE','FINAL_ALIGN'):return
        if stamp<=self.after_stamp or stamp<=self.last_sample_stamp:return
        if not healthy:
            self.samples.clear();self.observation=None
            return
        self.last_sample_stamp=stamp
        if self.samples and (stamp-self.samples[-1][0]>.25 or wall-self.samples[-1][1]>.25):
            self.samples.clear()
        self.samples.append((stamp,wall,opening))
        self.observation=opening;self.observed_wall=wall
        while len(self.samples)>1 and stamp-self.samples[1][0]>=self.visual_config.remeasure_s:
            self.samples.popleft()

    def stable_opening(self,now):
        if len(self.samples)<2 or now-self.observed_wall>.25:return None
        if (self.samples[-1][0]-self.samples[0][0]<self.visual_config.remeasure_s
                or self.samples[-1][1]-self.samples[0][1]<self.visual_config.remeasure_s):return None
        if (len(self.samples)-1)/(self.samples[-1][0]-self.samples[0][0])<10:return None
        obs=[p[2] for p in self.samples]
        a=obs[-1].yaw
        if (max(p.x for p in obs)-min(p.x for p in obs)>.02
                or max(p.y for p in obs)-min(p.y for p in obs)>.02
                or max(wrap(p.yaw-a) for p in obs)-min(wrap(p.yaw-a) for p in obs)>math.radians(1)):
            return None
        n=len(obs)
        return Opening(sum(p.x for p in obs)/n,sum(p.y for p in obs)/n,
                       wrap(a+sum(wrap(p.yaw-a) for p in obs)/n),min(p.width_m for p in obs))

    def complete_checkpoint_refresh(self,now,ros,opening):
        if not self.active or self.phase!='CHECKPOINT_RECOGNITION':
            raise ValueError('Không ở bước nhận diện mới tại checkpoint')
        old=self.plan.opening
        if (math.hypot(opening.x-old.x,opening.y-old.y)>.08
                or abs(wrap(opening.yaw-old.yaw))>math.radians(5)):
            raise ValueError('Pallet mới nhận diện khác vị trí/hướng ban đầu; giữ dừng')
        self.phase='REMEASURE';self.state='REMEASURING';self.phase_started=now
        self.after_stamp=ros;self.samples.clear();self.observation=None;self.settle_at=None
        self.measurement_wait_since=None
        self.reason='Hai chân được nhận diện mới; đo lại RGB-D sau khi YOLO tắt'

    def retry_checkpoint_measurement(self,now,ros,pose,linear,angular):
        """Reacquire at rest without restarting an approach or insertion."""
        self.v=self.w=0.
        quiet=abs(linear)<=.003 and abs(angular)<=.003
        self.refresh_settle=(self.refresh_settle if self.refresh_settle is not None else now) if quiet else None
        self.reason='RGB-D chưa ổn định; giữ dừng để nhận diện lại tại checkpoint'
        if (ros is None or self.refresh_settle is None
                or now-self.refresh_settle<self.config.settle_s):return
        self.phase=self.state='CHECKPOINT_RECOGNITION'
        self.checkpoint_refresh_seq+=1
        self.checkpoint_refresh_pose=tuple(pose)
        self.phase_started=now;self.after_stamp=ros
        self.samples.clear();self.observation=None;self.frozen_opening=None
        self.turning=False;self.trim_braking=False;self.trim_settle=None
        self.settle_at=self.refresh_settle=None
        self.measurement_wait_since=None

    def tick(self,now,pose,linear,angular,lift,issue='',operator_alive=True,ros_now=None):
        self.v=self.w=0.
        if not self.active:return
        if issue or not operator_alive:
            self.stop(issue or 'Mất heartbeat terminal người vận hành','BLOCKED');return
        if self.phase=='CHECKPOINT_APPROACH':
            if self.checkpoint_approach is not None:
                if lift['state']!='IDLE':self.stop('Càng chuyển động khi căn checkpoint','BLOCKED');return
                self.checkpoint_approach.tick(now,pose,linear,angular,lift['position'])
                if self.checkpoint_approach.issue:
                    self.stop(self.checkpoint_approach.issue,'BLOCKED');return
                self.v,self.w=self.checkpoint_approach.v,self.checkpoint_approach.w
                if self.checkpoint_approach.done:self.state='INSERTED'
                elif self.checkpoint_approach.pivot_pause=='BRAKE':
                    self.reason='Hãm quay sớm; chờ xe đứng yên trong giới hạn trôi'
                elif self.checkpoint_approach.pivot_pause=='RECENTER':
                    self.reason='Bù tâm về mốc odom ban đầu trước khi quay tiếp'
                else:self.reason='Căn góc nhỏ tới checkpoint: '+self.checkpoint_approach.phase
            else:
                self.stop('Thiếu bộ điều khiển tiếp cận checkpoint','BLOCKED');return
            if self.state=='INSERTED':
                if ros_now is None:
                    self.stop('Thiếu thời gian capture tại checkpoint','BLOCKED');return
                self.phase='CHECKPOINT_RECOGNITION' if self.require_checkpoint_refresh else 'REMEASURE'
                self.state='CHECKPOINT_RECOGNITION' if self.require_checkpoint_refresh else 'REMEASURING'
                self.active=True
                self.phase_started=now;self.after_stamp=ros_now;self.samples.clear()
                self.inserted_pose=None;self.settle_at=None
                self.checkpoint_refresh_pose=tuple(pose)
                self.checkpoint_refresh_seq+=1
                self.reason=('Xe dừng tại điểm còn thấy hai chân; chờ phiên nhận diện mới'
                             if self.require_checkpoint_refresh else
                             'Đứng yên ngoài pallet; đo RGB-D mới liên tục 5 giây')
            return
        if self.phase=='CHECKPOINT_RECOGNITION':
            anchor=self.checkpoint_refresh_pose
            if (abs(linear)>.003 or abs(angular)>.015 or lift['state']!='IDLE'
                    or abs(lift['position']-self.plan.entry_height_m)>.003
                    or math.hypot(pose[0]-anchor[0],pose[1]-anchor[1])>.005
                    or abs(wrap(pose[2]-anchor[2]))>math.radians(.5)):
                self.stop('Xe/càng dịch khi nhận diện mới tại checkpoint','BLOCKED')
            # The supervisor may repeat confirmed idle detection sessions.
            # Keep zero commands while waiting; sensor/lease/pose guards above
            # remain active, with no elapsed-time expiry for this stationary step.
            return
        if self.phase in ('REMEASURE','FINAL_ALIGN'):
            c=self.config.insertion
            if (abs(linear)>.003 or abs(angular)>.08 or lift['state']!='IDLE'
                    or abs(lift['position']-self.plan.entry_height_m)>.003):
                self.stop('Xe/càng chuyển động ngoài dự kiến tại điểm đo lại','BLOCKED');return
            if self.phase=='REMEASURE':
                if abs(angular)>.015:
                    self.samples.clear();self.after_stamp=ros_now;return
                opening=self.stable_opening(now)
                if opening is None:
                    if now-self.phase_started>self.visual_config.remeasure_timeout_s:
                        if self.require_checkpoint_refresh:
                            self.retry_checkpoint_measurement(now,ros_now,pose,linear,angular)
                        else:self.stop('Không đủ RGB-D mới ổn định tại điểm đo lại','BLOCKED')
                    return
                self.frozen_opening=opening;self.phase='FINAL_ALIGN';self.state='FINAL_ALIGN'
                self.turn_started=now;self.settle_at=None
            # During a bounded trim, follow the heading frozen from the
            # stationary window. Camera motion must not repeatedly interrupt
            # the turn to rebuild that same five-second window.
            opening=self.frozen_opening if self.turning else self.stable_opening(now)
            if opening is None:
                self.settle_at=None
                if self.measurement_wait_since is None:self.measurement_wait_since=now
                if now-self.measurement_wait_since>self.visual_config.remeasure_timeout_s:
                    if self.require_checkpoint_refresh:
                        self.retry_checkpoint_measurement(now,ros_now,pose,linear,angular)
                    else:self.stop('Không có RGB-D mới ổn định để căn góc cuối','BLOCKED')
                # Remain stopped while a stationary fresh measurement window rebuilds.
                return
            self.measurement_wait_since=None
            self.refresh_settle=None
            along,_=opening_coordinates(opening,pose)
            if -along-c.fork_root_m<c.fork_length_m+.10:
                self.stop('Đầu càng không còn đủ khoảng trống để căn góc','BLOCKED');return
            error=wrap(pose[2]-opening.yaw)
            accepted_cfg=(accepted_straight_config(pose,opening,self.plan.entry_height_m,
                self.visual_config) if not self.turning else None)
            if abs(error)>math.radians(12 if self.direct_mode else 3):
                self.require_new_approach('Góc pallet đổi quá nhiều ở checkpoint; cần căn lại từ ngoài');return
            if self.turning:
                if abs(error)<self.trim_best_error-math.radians(.05):
                    self.trim_best_error=abs(error);self.trim_progress_at=now
                if not self.trim_braking and now-self.trim_progress_at>20.:
                    self.stop('Đã phát lệnh quay nhưng sai số góc không giảm trong 20 giây; kiểm tra truyền động','BLOCKED');return
                issue=pivot_stop_issue(pose,linear,angular,self.trim_anchor,c)
                if issue:self.stop(issue,'BLOCKED');return
                stopping=c.reaction_s*abs(angular)+angular**2/(2*c.angular_braking_rps2)
                if (abs(error)<=math.radians(.10)
                        or angular*error<0 and abs(error)<=stopping+math.radians(.10)):
                    self.trim_braking=True
                if self.trim_braking:
                    still=abs(angular)<=.003 and abs(linear)<=.003
                    self.trim_settle=(self.trim_settle if self.trim_settle is not None else now) if still else None
                    if self.trim_settle is not None and now-self.trim_settle>=self.config.settle_s:
                        self.turning=False;self.trim_braking=False;self.trim_settle=None
                        self.samples.clear();self.observation=None;self.after_stamp=ros_now
                        self.settle_at=None
                        self.reason='Đã hãm căn góc cuối; đo lại RGB-D khi đứng yên'
                    return
            if accepted_cfg is None and abs(error)>self.visual_config.final_heading_tolerance_rad*.5:
                self.settle_at=None
                if not self.turning:
                    # Before turning, check whether a straight insertion from
                    # the eventual heading can fit the freshly measured axis.
                    # Reacquire from outside when lateral correction is needed.
                    try:
                        straight_plan((pose[0],pose[1],opening.yaw),opening,
                                      self.plan.entry_height_m,residual_insertion_config(self.visual_config))
                    except ValueError as exc:
                        if any(reason in str(exc) for reason in (
                                'Body is off the opening axis during entry',
                                'Fork swept width would contact an opening side')):
                            self.require_new_approach('Đường càng sau quay không lọt khe; cần căn lại từ ngoài: '+str(exc));return
                    self.frozen_opening=opening;self.trim_anchor=tuple(pose[:2])
                    self.turn_started=now;self.trim_braking=False;self.trim_settle=None
                    self.trim_best_error=abs(error);self.trim_progress_at=now
                # Ramp through the observed low-speed dead zone, bounded below
                # the measured-angular-speed guard. Braking above uses actual
                # odometry; commanded stopping geometry is checked below too.
                speed=min(.060,.035+.0125*max(0.,now-self.turn_started))
                w=math.copysign(speed,-error)
                for velocity in ((0.,w),(linear,angular)):
                    for p in braking_poses(pose,*velocity,c):
                        issue=checkpoint_pose_issue(p,opening,self.plan.entry_height_m,self.visual_config)
                        if not issue and max(opening_coordinates(opening,v)[0] for v in fork_vertices(p,c))>=-.10:
                            issue='Đầu càng không còn cách mặt pallet 10 cm'
                        if issue:
                            self.stop('Quỹ đạo xoay cuối không hợp lệ: '+issue,'BLOCKED');return
                self.turning=True
                self.w=w;self.reason='Căn góc khi đầu càng còn ngoài; chưa đưa càng';return
            if self.turning:
                self.trim_braking=True;self.trim_settle=None;return
            still=abs(angular)<=.015
            self.settle_at=(self.settle_at if self.settle_at is not None else now) if still else None
            if self.settle_at is None or now-self.settle_at<self.config.settle_s:return
            cfg=accepted_cfg or residual_insertion_config(self.visual_config)
            try:plan=straight_plan(pose,opening,self.plan.entry_height_m,cfg)
            except ValueError as exc:
                reason=str(exc)
                if any(detail in reason for detail in (
                        'Body is off the opening axis during entry',
                        'Fork swept width would contact an opening side',
                        'Cần căn lại góc trước khi chốt đường thẳng')):
                    self.require_new_approach(reason)
                else:
                    self.stop(reason,'BLOCKED')
                return
            self.config=cfg;self.plan=plan;self.phase='FINAL_INSERTION';self.state='INSERTING'
            self.started=self.progress_at=self.phase_started=now;self.best_remaining=plan.travel_m
            self.settle_at=None;self.frozen_opening=opening
            self.reason='Đã chốt đường thẳng odom; không dùng goal pickup bản đồ'
            return
        if self.phase=='FINAL_INSERTION' and self.state not in ('LIFTING','LOWERING'):
            # Absolute deviation from the frozen LINE heading, independently
            # of the measured pallet yaw. Check actual and braking envelopes.
            for velocity in ((self.v,self.w),(linear,angular)):
                for p in braking_poses(pose,*velocity,self.config.insertion):
                    if (abs(wrap(p[2]-self.plan.start[2]))>self.visual_config.final_heading_tolerance_rad
                            or depth_issue(p,self.plan.opening,self.config)):
                        self.stop('Vượt giới hạn góc đường thẳng hoặc chiều sâu/phanh','BLOCKED');return
        super().tick(now,pose,linear,angular,lift,issue,operator_alive)
        if self.active and self.phase=='FINAL_INSERTION' and self.state in ('INSERTING','STOPPING'):
            # Never rotate without translation once the tips enter. Steering
            # is bounded further as speed approaches zero.
            self.w=max(-abs(self.v)*.20,min(abs(self.v)*.20,self.w))
            for p in braking_poses(pose,self.v,self.w,self.config.insertion):
                if (abs(wrap(p[2]-self.plan.start[2]))>self.visual_config.final_heading_tolerance_rad
                        or depth_issue(p,self.plan.opening,self.config)):
                    self.stop('Lệnh đưa càng vượt bao phanh đường thẳng/chiều sâu','BLOCKED');return
