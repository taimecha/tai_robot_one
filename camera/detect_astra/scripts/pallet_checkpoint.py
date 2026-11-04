"""Small-angle correction entirely outside the pallet, before fresh RGB-D.

This is a checkpoint approach, never permission to insert or lift. Insertion
keeps its separate 0.5-degree / 5-mm frozen-line guards.
"""
import math
from pallet_alignment_core import wrap,clamp
from pallet_insertion_core import InsertionPlan,opening_coordinates,braking_poses,fork_vertices
from pallet_staging_safety import swept_poses
from pallet_staging_core import MAX_STAGING_TURN_RAD
from pallet_laser_timing import (stop_travel,COMMAND_TRACKING_FACTOR,
    COMMAND_ANGULAR_TRACKING_FACTOR,STOP_POLYGON_PADDING_M,STOP_RESERVE_M)

# The checkpoint is 1.10 m out so both front legs remain visible. A small
# lateral correction over the shorter approach can need more than the staging
# limit of 12 degrees. This separate bound applies ONLY with tips outside;
# every swept body/fork pose and braking pose still requires collision checks.
MAX_CHECKPOINT_TURN_RAD=math.radians(30)
MAX_PIVOT_DISPLACEMENT_M=.005
MAX_PIVOT_LINEAR_MPS=.015
MAX_PIVOT_CORRECTION_MPS=.012
MAX_PIVOT_ANGULAR_RPS=.10
PIVOT_BRAKE_RESERVE_M=.004
PIVOT_COMMAND_ACCEL_MPS2=.020


def pivot_stop_distance(pose,linear,angular,anchor,config):
    return max(math.hypot(p[0]-anchor[0],p[1]-anchor[1])
        for p in braking_poses(pose,linear,angular,config))


def pivot_stop_issue(pose,linear,angular,anchor,config):
    actual=math.hypot(pose[0]-anchor[0],pose[1]-anchor[1])
    furthest=pivot_stop_distance(pose,linear,angular,anchor,config)
    if furthest>MAX_PIVOT_DISPLACEMENT_M:
        return ('Tâm xe trôi khi quay vượt 5 mm (gồm quãng phanh): '
            f'hiện {actual*1000:.2f} mm, dự kiến {furthest*1000:.2f} mm')
    return ''


def checkpoint_pose_issue(pose,opening,height,config):
    c=config.insertion
    if opening.width_m<2*(c.fork_half_span_m+c.side_margin_m):
        return 'Khe pallet hẹp hơn bề rộng hai càng và khoảng chừa'
    if not all(math.isfinite(v) for v in (*pose,height)):return 'Pose checkpoint không hợp lệ'
    bottom=c.fork_bottom_at_home_m+height
    if (bottom<c.opening_bottom_m+c.vertical_margin_m
            or bottom+c.fork_thickness_m>c.opening_top_m-c.vertical_margin_m):
        return 'Cao độ càng không lọt khoang pallet'
    along,_=opening_coordinates(opening,pose)
    # Use the worst possible root orientation. Even while turning, tips stay
    # outside the face; the real checkpoint is now 1.10 m from the fork root.
    if -along-c.fork_root_m<config.checkpoint_root_gap_m-.025:
        return 'Checkpoint quá gần pallet; chưa được phép đưa càng'
    if abs(wrap(pose[2]-opening.yaw))>MAX_CHECKPOINT_TURN_RAD+1e-9:
        return 'Góc căn ngoài pallet vượt giới hạn 30°'
    return ''


def checkpoint_plan(start,opening,height,config):
    issue=checkpoint_pose_issue(start,opening,height,config)
    if issue:raise ValueError(issue)
    if abs(wrap(start[2]-opening.yaw))>MAX_STAGING_TURN_RAD:
        raise ValueError('Góc ban đầu vượt giới hạn 30°; cần đặt lại vị trí nhận diện')
    c=config.insertion
    if stop_travel(MAX_PIVOT_CORRECTION_MPS*COMMAND_TRACKING_FACTOR,
            MAX_PIVOT_ANGULAR_RPS*COMMAND_ANGULAR_TRACKING_FACTOR,
            c.braking_mps2,c.angular_braking_rps2)>STOP_POLYGON_PADDING_M-STOP_RESERVE_M:
        raise ValueError('Lệnh bù trôi/quay không nằm trong vùng phanh laser')
    along,cross=opening_coordinates(opening,start)
    if -along-c.fork_root_m<1.20:raise ValueError('Bắt đầu căn checkpoint cần cách pallet ≥1,20 m')
    if abs(cross)>.15:raise ValueError('Lệch trục quá 15 cm; cần đặt lại vị trí nhận diện')
    offset=c.fork_root_m+config.checkpoint_root_gap_m
    goal=(opening.x-offset*math.cos(opening.yaw),opening.y-offset*math.sin(opening.yaw),opening.yaw)
    distance=math.hypot(goal[0]-start[0],goal[1]-start[1])
    # An admitted 1.20 m standoff can be only 0.10 m from the 1.10 m
    # checkpoint. The same angle, clearance and braking checks still apply.
    if not .09<=distance<=config.insertion.maximum_travel_m:
        raise ValueError('Quãng căn tới checkpoint không hợp lệ')
    bearing=math.atan2(goal[1]-start[1],goal[0]-start[0])
    required=max(abs(wrap(bearing-start[2])),abs(wrap(bearing-opening.yaw)))
    if required>MAX_CHECKPOINT_TURN_RAD:
        raise ValueError(f'Đường tới trục pallet cần góc {math.degrees(required):.2f}° >30°; '
                         f'quãng tiến chéo {distance*100:.2f} cm, góc đường theo xe '
                         f'{math.degrees(wrap(bearing-start[2])):+.2f}°. Cần thêm quãng tiếp cận')
    corners=(tuple(start),(start[0],start[1],bearing),(goal[0],goal[1],bearing),goal)
    poses=[]
    for a,b in zip(corners,corners[1:]):poses.extend(swept_poses(a,b))
    for pose in poses:
        issue=checkpoint_pose_issue(pose,opening,height,config)
        if issue:raise ValueError(issue)
    return InsertionPlan(tuple(start),goal,goal,opening,height,0.,distance,tuple(poses))


def quarter_turn_pose_issue(pose,opening,height,config):
    # Reuse height/width/standoff guards with the face-aligned proxy. Actual
    # fork corners, including all intermediate orientations, stay outside.
    issue=checkpoint_pose_issue((pose[0],pose[1],opening.yaw),opening,height,config)
    if issue:return issue
    if max(opening_coordinates(opening,v)[0] for v in fork_vertices(pose,config.insertion))>-.10:
        return 'Đầu càng không còn cách mặt pallet 10 cm khi quay 90°'
    return ''


def quarter_turn_plan(start,opening,height,config):
    issue=quarter_turn_pose_issue(start,opening,height,config)
    if issue:raise ValueError(issue)
    if abs(wrap(start[2]-opening.yaw))>MAX_STAGING_TURN_RAD:
        raise ValueError('Góc ban đầu vượt 30° trước sửa ngang 90°')
    along,cross=opening_coordinates(opening,start)
    if -along-config.insertion.fork_root_m<1.20:
        raise ValueError('Sửa ngang 90° phải bắt đầu ngoài pallet ít nhất 1,20 m')
    if not .005<=abs(cross)<=.15:
        raise ValueError('Sửa ngang 90° cần quãng ngang từ 5 mm tới 15 cm')
    nx,ny=-math.sin(opening.yaw),math.cos(opening.yaw)
    goal=(start[0]-cross*nx,start[1]-cross*ny,opening.yaw)
    bearing=wrap(opening.yaw-math.copysign(math.pi/2,cross))
    corners=(tuple(start),(start[0],start[1],opening.yaw),
        (start[0],start[1],bearing),(goal[0],goal[1],bearing),goal)
    poses=[]
    for a,b in zip(corners,corners[1:]):poses.extend(swept_poses(a,b))
    for pose in poses:
        issue=quarter_turn_pose_issue(pose,opening,height,config)
        if issue:raise ValueError(issue)
    return InsertionPlan(tuple(start),goal,goal,opening,height,abs(cross),abs(cross),tuple(poses))


class CheckpointApproach:
    """Pivot a few degrees, follow the checked diagonal, stop, match face yaw."""
    def __init__(self,plan,config,now,quarter_turn=False):
        self.quarter_turn=quarter_turn
        self.pose_issue=quarter_turn_pose_issue if quarter_turn else checkpoint_pose_issue
        self.timeout_s=100. if quarter_turn else 40.
        self.plan=plan;self.config=config;self.phase='TURN_TO_PATH'
        if quarter_turn:self.phase='TURN_TO_ALIGN'
        self.started=self.progress_at=now;self.best_remaining=plan.travel_m
        self.settle=None;self.braking=False;self.v=self.w=0.
        self.done=False;self.issue=''
        self.pivot_anchor=tuple(plan.start[:2])
        self.pivot_velocity=0.;self.pivot_command=0.;self.last_tick=now
        self.pivot_pause='';self.pause_started=None;self.pause_settle=None
        self.pivot_pauses=0
        self.bearing=math.atan2(plan.goal[1]-plan.start[1],plan.goal[0]-plan.start[0])

    def tick(self,now,pose,linear,angular,height):
        self.v=self.w=0.
        if self.done or self.issue:return
        dt=max(.001,min(.10,now-self.last_tick));self.last_tick=now
        c=self.config.insertion
        if now-self.started>self.timeout_s:
            self.issue='Sửa ngang ngoài pallet quá thời gian cho phép';return
        if (abs(height-self.plan.entry_height_m)>.003 or abs(linear)>c.maximum_measured_speed_mps
                or abs(angular)>.15):
            self.issue='Xe/càng vượt giới hạn căn checkpoint';return
        self.issue=self.pose_issue(pose,self.plan.opening,height,self.config)
        if self.issue:return
        if self.phase in ('TURN_TO_ALIGN','TURN_TO_PATH','TURN_TO_FACE'):
            # A brief wheel/EKF vx transient is not displacement. Bound the
            # actual pivot centre AND its full measured stopping trajectory;
            # never let repeated small samples accumulate unbounded drift.
            if abs(linear)>MAX_PIVOT_LINEAR_MPS:
                self.issue='Tốc độ trôi khi quay vượt 15 mm/s';return
            offset=math.hypot(pose[0]-self.pivot_anchor[0],pose[1]-self.pivot_anchor[1])
            if offset>MAX_PIVOT_DISPLACEMENT_M:
                self.issue='Tâm xe trôi thực khi quay vượt 5 mm';return
            self.pivot_velocity+=dt/(.20+dt)*(linear-self.pivot_velocity)
            projected=pivot_stop_distance(pose,linear,angular,self.pivot_anchor,c)
            if not self.pivot_pause and projected>PIVOT_BRAKE_RESERVE_M:
                self.pivot_pauses+=1
                if self.pivot_pauses>3:
                    self.issue='Bù trôi khi quay chưa ổn định sau 3 lần hãm';return
                self.pivot_pause='BRAKE';self.pause_started=now;self.pause_settle=None
                self.pivot_command=0.;self.settle=None;self.braking=False
            ex=((self.pivot_anchor[0]-pose[0])*math.cos(pose[2])
                +(self.pivot_anchor[1]-pose[1])*math.sin(pose[2]))
            if self.pivot_pause:
                if now-self.pause_started>6.:
                    self.issue='Không giữ lại được tâm xe sau khi hãm quay';return
                still=abs(linear)<=.003 and abs(angular)<=.015
                if self.pivot_pause=='BRAKE':
                    self.pause_settle=(self.pause_settle if self.pause_settle is not None else now) if still else None
                    if self.pause_settle is not None and now-self.pause_settle>=self.config.settle_s:
                        self.pivot_pause='RECENTER';self.pause_settle=None
                        self.pivot_velocity=linear;self.pivot_command=0.
                    return
                if offset<=.0015:
                    self.pivot_command=0.
                    self.pause_settle=(self.pause_settle if self.pause_settle is not None else now) if still else None
                    if self.pause_settle is not None and now-self.pause_settle>=self.config.settle_s:
                        self.pivot_pause='';self.pivot_velocity=linear;self.pause_settle=None
                    return
                self.pause_settle=None
                demand=clamp(2.*ex-.2*self.pivot_velocity,.004)
                self.pivot_command+=clamp(demand-self.pivot_command,PIVOT_COMMAND_ACCEL_MPS2*dt)
                self.v=self.pivot_command
                self.issue=pivot_stop_issue(pose,self.v,0.,self.pivot_anchor,c)
                if self.issue:self.v=0.
                else:self.check_braking(pose,linear,angular,height)
                return
            target=self.bearing if self.phase=='TURN_TO_PATH' else self.plan.opening.yaw
            error=wrap(target-pose[2])
            if abs(error)<math.radians(.3):
                self.pivot_command=0.
                self.settle=self.settle if self.settle is not None else now
                if abs(angular)>.015 or abs(linear)>.003:self.settle=None
                if self.settle is not None and now-self.settle>=self.config.settle_s:
                    self.settle=None;self.braking=False
                    if self.phase=='TURN_TO_ALIGN':
                        self.phase='TURN_TO_PATH';self.pivot_anchor=tuple(pose[:2]);self.pivot_pauses=0
                    elif self.phase=='TURN_TO_PATH':
                        self.phase='DRIVE';self.progress_at=now
                    else:
                        if math.hypot(pose[0]-self.plan.goal[0],pose[1]-self.plan.goal[1])>.012:
                            self.issue='Checkpoint sau quay ngoài sai số 1,2 cm';return
                        self.done=True
                return
            self.settle=None
            # Keep the physical rotation centre at its frozen odom anchor.
            # Zero commanded vx alone did not stop persistent 2--4 mm/s creep
            # on the real base. Correct body-forward position and velocity;
            # lateral skid remains independently bounded by the 5-mm circle.
            demand=clamp(3.*ex-.2*self.pivot_velocity,MAX_PIVOT_CORRECTION_MPS)
            self.pivot_command+=clamp(demand-self.pivot_command,PIVOT_COMMAND_ACCEL_MPS2*dt)
            self.v=self.pivot_command
            if self.braking:
                if abs(angular)<=.015:self.braking=False
            else:
                stopping=c.reaction_s*abs(angular)+angular**2/(2*c.angular_braking_rps2)
                if angular*error>0 and abs(error)<=stopping+math.radians(.10):
                    self.braking=True
                else:self.w=math.copysign(min(MAX_PIVOT_ANGULAR_RPS,max(.07,abs(error)*1.5)),error)
        elif self.phase=='STOP':
            if abs(linear)<=.003 and abs(angular)<=.015:
                self.settle=self.settle if self.settle is not None else now
                if now-self.settle>=self.config.settle_s:
                    if math.hypot(pose[0]-self.plan.goal[0],pose[1]-self.plan.goal[1])>.012:
                        self.issue='Checkpoint dừng ngoài sai số 1,2 cm';return
                    self.phase='TURN_TO_FACE';self.settle=None
                    self.pivot_anchor=tuple(pose[:2])
                    self.pivot_velocity=linear;self.pivot_command=0.;self.pivot_pauses=0
            else:self.settle=None
        else:
            dx,dy=pose[0]-self.plan.start[0],pose[1]-self.plan.start[1]
            cross=-dx*math.sin(self.bearing)+dy*math.cos(self.bearing)
            progress=dx*math.cos(self.bearing)+dy*math.sin(self.bearing)
            remaining=self.plan.travel_m-progress
            if abs(cross)>.015 or abs(wrap(pose[2]-self.bearing))>math.radians(2):
                self.issue='Xe lệch đường căn checkpoint';return
            if remaining<self.best_remaining-.005:
                self.best_remaining=remaining;self.progress_at=now
            if now-self.progress_at>5.:
                self.issue='Xe không tiến tới checkpoint';return
            if remaining<=.003:
                self.phase='STOP';return
            speed=math.sqrt((c.braking_mps2*c.reaction_s)**2+2*c.braking_mps2*remaining)-c.braking_mps2*c.reaction_s
            self.v=min(.08,.6*speed)
            self.w=clamp(-1.8*wrap(pose[2]-self.bearing)-2.*cross,.035)
        # Check measured and requested braking poses, including the full fork
        # clearance to the face. Laser filled-footprint checks stay in owner.
        self.check_braking(pose,linear,angular,height)

    def check_braking(self,pose,linear,angular,height):
        c=self.config.insertion
        for v,w in ((self.v,self.w),(linear,angular)):
            if self.phase in ('TURN_TO_ALIGN','TURN_TO_PATH','TURN_TO_FACE'):
                self.issue=pivot_stop_issue(pose,v,w,self.pivot_anchor,c)
                if self.issue:self.v=self.w=0.;return
            for p in braking_poses(pose,v,w,c):
                self.issue=self.pose_issue(p,self.plan.opening,height,self.config)
                if self.issue:self.v=self.w=0.;return
                cross=(-(p[0]-self.plan.start[0])*math.sin(self.bearing)
                       +(p[1]-self.plan.start[1])*math.cos(self.bearing))
                if abs(cross)>.015:
                    self.issue='Quỹ đạo phanh lệch đường căn checkpoint';self.v=self.w=0.;return
