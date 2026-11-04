"""Collision-checked diagonal staging on the pallet normal, then visual yaw trim.

All geometry is planar odom geometry. No ROS or hardware imports.
"""
from dataclasses import dataclass,replace
import math
from pallet_alignment_core import AlignmentConfig, AlignmentController, wrap, clamp
from pallet_laser_timing import safe_twist_scale

# Nominal staging paths only make small turns. Do not turn a near-centred
# visual observation into a large lateral projection or lattice detour.
MAX_STAGING_TURN_RAD = math.radians(30.)


@dataclass(frozen=True)
class StagingConfig(AlignmentConfig):
    lateral_tolerance_m: float = .01
    heading_tolerance_deg: float = 1.0
    initial_clearance_margin_m: float = .22
    max_linear_mps: float = .10
    max_angular_rps: float = .22
    min_turn_rps: float = .22
    clearance_margin_m: float = .10
    candidate_step_m: float = .10
    candidate_extra_m: float = .80
    max_goal_travel_m: float = 2.0
    max_pallet_range_m: float = 3.0
    nav_timeout_s: float = 60.0
    max_corrections: int = 2

    def __post_init__(self):
        super().__post_init__()
        if (self.standoff_m < 1.2 or self.clearance_margin_m < .08
                or self.min_turn_rps > self.max_angular_rps or self.max_pallet_range_m > 3.5 or self.nav_timeout_s > 90
                or not self.clearance_margin_m <= self.initial_clearance_margin_m <= .40
                or type(self.max_corrections) is not int or self.max_corrections > 3):
            raise ValueError('Invalid staging clearance, limits or yaw trim')


def plane_clearance(target, pose, fork_root_m):
    """Signed perpendicular distance from the fork root to the pallet face."""
    cx, cy, theta = target
    x, y, yaw = pose
    return ((cx-x)*math.cos(theta)+(cy-y)*math.sin(theta)
            - fork_root_m*math.cos(wrap(yaw-theta)))


def bounded_nav_command(v, w, config):
    if not all(math.isfinite(x) for x in (v,w)) or not 0 <= v <= .15 or abs(w) > 1.:
        raise ValueError('Invalid or reversing Nav2 command')
    scale = min(1., config.max_linear_mps/max(v,1e-9),
                config.max_angular_rps/max(abs(w),1e-9))
    # Scale both axes together to preserve the collision-checked curvature.
    v,w=v*scale,w*scale
    scale=safe_twist_scale(v,w,config.linear_accel_mps2,config.angular_accel_rps2)
    return v*scale,w*scale


def planning_clearance_m(config):
    # Preserve the 2 cm braking guard reserve and 2 cm tracking/command-lag
    # reserve, even when the configured nominal goal margin is smaller.
    stopping=.2*config.max_linear_mps+config.max_linear_mps**2/(2*config.linear_accel_mps2)
    return config.standoff_m+max(config.clearance_margin_m,.02+stopping+.02)


def on_approach_axis(target,pose):
    cross=-(target[0]-pose[0])*math.sin(target[2])+(target[1]-pose[1])*math.cos(target[2])
    return abs(cross)<.03


def centered_for_visual_trim(target, pose, config):
    """A small yaw error must not turn a centred opening into a diagonal trip."""
    dx,dy=target[0]-pose[0],target[1]-pose[1]
    lateral=-math.sin(pose[2])*dx+math.cos(pose[2])*dy
    heading=wrap(target[2]-pose[2])
    return (abs(lateral)<config.lateral_tolerance_m
            and abs(heading)<math.radians(2.))


def target_changed_since_staging(old, new):
    return (old is not None and new is not None and
            (math.hypot(new[0]-old[0],new[1]-old[1])>.08
             or abs(wrap(new[2]-old[2]))>math.radians(5.)))


def within_alignment_tolerances(target,pose,config):
    """Use the same vehicle-frame lateral/yaw errors shown to the operator.

    Normal-axis cross-track is a planning criterion, not an additional final
    tolerance. At 1.6 m, an accepted 1-degree angle alone contributes 2.8 cm
    to that cross-track even when the measured opening is centred on the car.
    The insertion planner checks the actual fork/path geometry separately.
    """
    dx,dy=target[0]-pose[0],target[1]-pose[1]
    lateral=-math.sin(pose[2])*dx+math.cos(pose[2])*dy
    heading=wrap(target[2]-pose[2])
    cross=-(target[0]-pose[0])*math.sin(target[2])+(target[1]-pose[1])*math.cos(target[2])
    return (abs(lateral)<config.lateral_tolerance_m
            and abs(heading)<math.radians(config.heading_tolerance_deg))


def staging_candidates(target, pose, config):
    cx, cy, theta = target
    x, y, yaw = pose
    nx, ny = math.cos(theta), math.sin(theta)
    # A pure lateral projection needs a 90-degree pivot on a differential
    # drive. Instead search FORWARD along the normal, using only the measured
    # clearance surplus. Rank by steering effort, then distance: the useful
    # diagonal must fit the small-turn budget as well as collision checks.
    d = (cx-x)*nx+(cy-y)*ny
    minimum = config.fork_root_m+planning_clearance_m(config)
    if d < minimum-1e-9:
        return []
    advance = min(d-minimum, config.candidate_extra_m)
    advances = [i*config.candidate_step_m
                for i in range(int(advance/config.candidate_step_m)+1)]
    if advance-advances[-1] > 1e-9:
        advances.append(advance)
    result = []
    def steering(goal):
        dx,dy=goal[0]-x,goal[1]-y
        if math.hypot(dx,dy) <= .005:return (0.,0.)
        travel=math.atan2(dy,dx)
        return (abs(wrap(travel-yaw))+abs(wrap(travel-theta)),math.hypot(dx,dy))
    for forward in advances:
        distance = d-forward
        goal = (cx-distance*nx, cy-distance*ny, theta)
        dx,dy=goal[0]-x,goal[1]-y
        if math.hypot(dx,dy) > .005:
            travel_yaw=math.atan2(dy,dx)
            if max(abs(wrap(travel_yaw-yaw)),abs(wrap(travel_yaw-theta))) > MAX_STAGING_TURN_RAD:
                continue
        if (distance-config.fork_root_m <= config.max_pallet_range_m
                and math.hypot(goal[0]-x, goal[1]-y) <= config.max_goal_travel_m):
            result.append(goal)
    return sorted(result,key=steering)


class StagingController(AlignmentController):
    NAV_STATES = ('PLANNING', 'NAVIGATING', 'NAV_CANCELING', 'NAV_STOPPING')

    def __init__(self, config=StagingConfig()):
        super().__init__(config)
        self.run_id = 0
        self.frozen_target = None
        self.candidates = []
        self.candidate_index = 0
        self.corrections = 0
        self.nav_started = None
        self.arrival_measure_after = None
        self.arrival_wall = None
        self.nav_v = self.nav_w = 0.
        self.nav_command_wall = None
        self.plan_attempt = 0
        self.post_measurement_stable = False
        self.arrival_validated = False
        self.nav_settle_started = None
        self.stationary_arrival_confirmed = False
        self.stationary_trim_start_confirmed = False
        self.nav_cancel_started = None
        self.visual_hold_since = None
        self.trim_braking_since = None
        self.heading_only_trim = False

    @property
    def navigating(self):
        return self.active and self.state in self.NAV_STATES

    def readiness(self, now_s, wall_s):
        if self.navigating:
            if (not self.odom.samples or not -.05 <= now_s-self.odom.samples[-1].stamp_s <= self.config.odom_timeout_s):
                return False, 'Odometry quá hạn khi đi tới goal'
            return True, ''
        return super().readiness(now_s, wall_s)

    def invalidate_target(self, reason):
        goal = self.goal
        v, w = self.v, self.w
        nav_reason = self.reason
        navigating = self.navigating
        super().invalidate_target(reason)
        if navigating:
            self.goal = goal  # Snapshot goal is valid independently of visibility.
            self.v, self.w = v, w
            self.reason = nav_reason

    def start(self, now_s, wall_s):
        post_arrival_restart=(self.state=='REACQUIRE_REQUIRED'
                              and (self.stationary_arrival_confirmed or self.stationary_trim_start_confirmed))
        if post_arrival_restart and target_changed_since_staging(self.frozen_target,self.target):
            self.stop('Pallet đo lại khác vị trí/hướng trước khi xe chạy; cần kiểm tra trực quan',
                      'TARGET_JUMP')
            return False,self.reason
        if post_arrival_restart and self.corrections>=self.config.max_corrections:
            return False,'Đã hết số lượt sửa ngang; giữ xe đứng yên'
        if not post_arrival_restart:
            self.heading_only_trim=False
        if self.odom.samples:
            p = self.odom.samples[-1]
            if abs(p.linear) > .015 or abs(p.angular) > .035:
                return False, 'Xe phải đứng yên trước khi chốt pallet và chọn goal'
        ready, reason = super().start(now_s, wall_s)
        if not ready:
            return ready, reason
        self.run_id += 1
        self.corrections = self.corrections+1 if post_arrival_restart else 0
        self.nav_settle_started = None
        self.stationary_arrival_confirmed = False
        self.stationary_trim_start_confirmed = False
        self.visual_hold_since = None
        self.trim_braking_since = None
        self.nav_started = wall_s
        self._plan()
        return self.active, self.reason

    def _plan(self):
        p = self.odom.samples[-1]
        self.frozen_target = tuple(self.target)
        clearance = plane_clearance(self.frozen_target, (p.x,p.y,p.yaw), self.config.fork_root_m)
        if clearance < self.config.standoff_m:
            self.stop('Gốc càng đang cách mặt pallet dưới 1,20 m; cần đặt xe xa hơn', 'TOO_CLOSE')
            return
        dx,dy=self.target[0]-p.x,self.target[1]-p.y
        lateral=-math.sin(p.yaw)*dx+math.cos(p.yaw)*dy
        heading=wrap(self.target[2]-p.yaw)
        # Latch the operator's yaw-only request when the INITIAL lateral
        # reading is acceptable. A changed lateral reading after the pivot
        # must never trigger an automatic diagonal translation in this run.
        self.heading_only_trim = self.heading_only_trim or (
            abs(lateral)<self.config.lateral_tolerance_m
            and math.radians(self.config.heading_tolerance_deg)<=abs(heading)<=MAX_STAGING_TURN_RAD)
        if (self.heading_only_trim
                or centered_for_visual_trim(self.frozen_target,(p.x,p.y,p.yaw),self.config)
                or on_approach_axis(self.frozen_target,(p.x,p.y,p.yaw))
                or within_alignment_tolerances(self.frozen_target,(p.x,p.y,p.yaw),self.config)):
            # Longitudinal standoff is a minimum, not an exact docking X.
            # Once already on the normal, do not ask Nav2 to reverse merely
            # because a final skid pivot moved a centimetre past its XY goal.
            self.goal=(p.x,p.y,self.frozen_target[2])
            self.candidates=[]
            self.arrival_wall=self.nav_started
            self.arrival_measure_after=self.target_stamp
            self.arrival_validated=False
            self.post_measurement_stable=False
            self.stationary_trim_start_confirmed=(abs(p.linear)<=.015 and abs(p.angular)<=.035)
            self.v=self.w=0.
            self.state,self.reason='CHECKING',(
                'Ngang ban đầu đã đạt; chỉ quay bù góc, không dịch ngang'
                if self.heading_only_trim else
                'Đã gần trục/đạt ngưỡng căn; giữ xe đứng yên, xác nhận RGB-D mới')
            return
        # Leave space on the first approach for fresh visual measurements
        # and a second shallow diagonal. Subsequent corrections use the normal
        # braking reserve, never reduce the 1.20 m minimum.
        plan_config=(replace(self.config,clearance_margin_m=self.config.initial_clearance_margin_m)
                     if self.corrections==0 else self.config)
        self.candidates = staging_candidates(self.frozen_target, (p.x,p.y,p.yaw), plan_config)
        if plan_config is not self.config:
            # A fresh process can resume from a previously reached staging
            # point, already inside the extra FIRST-approach reserve. Use the
            # normal braking reserve, never demand an impossible 1.42 m stop
            # behind the car or reduce the protected 1.20 m clearance.
            fallback = staging_candidates(self.frozen_target, (p.x,p.y,p.yaw), self.config)
            self.candidates.extend(g for g in fallback if not any(
                math.hypot(g[0]-old[0],g[1]-old[1])<.005 for old in self.candidates))
        if not self.candidates:
            self.stop('Thiếu quãng tiến để căn bằng góc nhỏ ≤30°; cần vị trí nhận diện xa pallet hơn. '
                      'Không tự xoay lớn hoặc đi vòng', 'NO_SAFE_GOAL')
            return
        self.candidate_index = 0
        self.plan_attempt += 1
        self.goal = self.candidates[0]
        self.v = self.w = 0.
        self.state, self.reason = 'PLANNING', 'Đang kiểm tra đường tiến chéo tới trục pallet và khoảng cách gốc càng'

    def next_candidate(self):
        self.candidate_index += 1
        if self.candidate_index >= len(self.candidates):
            self.stop('Không tìm được đường và footprint hợp lệ trên trục tiếp cận', 'NO_SAFE_GOAL')
        else:
            self.goal = self.candidates[self.candidate_index]

    def navigation_started(self):
        self.stationary_arrival_confirmed = False
        self.nav_settle_started = None
        self.state, self.reason = 'NAVIGATING', 'Nav2 đang đưa xe tới trục tiếp cận; theo dõi bằng odom và laser'
        self.nav_command_wall = None
        self.nav_v = self.nav_w = 0.

    def navigation_finished(self, success, now_s, wall_s):
        self.nav_v = self.nav_w = self.v = self.w = 0.
        if not success:
            self.stop('Nav2 không hoàn tất; xe dừng, cần kiểm tra rồi bắt đầu lại', 'NAV_FAILED')
            return
        self.arrival_measure_after = now_s
        self.arrival_wall = wall_s
        self.arrival_validated = False
        self.state, self.reason = 'NAV_STOPPING', 'Đã tới goal; chờ xe đứng yên và phép đo mới'
        self.settle_started = None
        self.nav_settle_started = None

    def begin_axis_handoff(self,wall_s):
        if not self.active or self.state!='NAVIGATING' or not self.odom.samples:
            return False
        p=self.odom.samples[-1];pose=(p.x,p.y,p.yaw)
        if (not on_approach_axis(self.frozen_target,pose)
                or plane_clearance(self.frozen_target,pose,self.config.fork_root_m)<self.config.standoff_m+.02):
            return False
        self.nav_v=self.nav_w=self.v=self.w=0.
        self.nav_cancel_started=wall_s
        self.stationary_arrival_confirmed=False
        self.state,self.reason='NAV_CANCELING','Đã tới trục pallet; chờ hủy đường Nav2, không phát lệnh lùi'
        return True

    def errors(self, pose):
        target = self.target if self.target is not None else self.frozen_target
        if target is None:
            return 0., 0., 0., 0.
        dx, dy = target[0]-pose.x, target[1]-pose.y
        x = math.cos(pose.yaw)*dx+math.sin(pose.yaw)*dy
        y = -math.sin(pose.yaw)*dx+math.cos(pose.yaw)*dy
        distance = plane_clearance(target, (pose.x,pose.y,pose.yaw), self.config.fork_root_m)
        return distance-self.config.standoff_m, y, wrap(target[2]-pose.yaw), x

    def tick(self, now_s, wall_s, interlock=True, interlock_reason=''):
        dt = min(.1,max(.001,wall_s-self.last_tick)) if self.last_tick is not None else .05
        self.last_tick = wall_s
        previous_v, previous_w = self.v, self.w
        self.v = self.w = 0.
        if not self.active:
            return self.snapshot()
        if not interlock:
            self.stop(interlock_reason or 'Mất điều kiện bảo vệ', 'BLOCKED')
            return self.snapshot()
        p = self.odom.samples[-1] if self.odom.samples else None
        if p is None or not -.05 <= now_s-p.stamp_s <= self.config.odom_timeout_s:
            self.stop('Mất odometry; đã hủy di chuyển', 'LOST_DATA')
            return self.snapshot()
        if wall_s-self.started_at > self.config.max_duration_s:
            self.stop('Hết thời gian căn xe', 'TIMEOUT')
            return self.snapshot()
        if self.navigating:
            if self.state=='NAV_CANCELING':
                if plane_clearance(self.frozen_target,(p.x,p.y,p.yaw),self.config.fork_root_m)<self.config.standoff_m:
                    self.stop('Đã chạm giới hạn khoảng cách tối thiểu; dừng xe', 'TOO_CLOSE')
                elif wall_s-self.nav_cancel_started>3.:
                    self.stop('Chưa xác nhận hủy đường Nav2 tại trục pallet; đã dừng','BLOCKED')
            elif self.state != 'NAV_STOPPING' and wall_s-self.nav_started > self.config.nav_timeout_s:
                self.stop('Hết thời gian đi tới goal', 'TIMEOUT')
            elif plane_clearance(self.frozen_target,(p.x,p.y,p.yaw),self.config.fork_root_m) < self.config.standoff_m:
                self.stop('Đã chạm giới hạn khoảng cách tối thiểu; dừng xe', 'TOO_CLOSE')
            elif self.state == 'NAVIGATING':
                if self.nav_command_wall is not None and wall_s-self.nav_command_wall <= .2:
                    # Nav2's stop/pivot requests must not leave 6 cm of slow
                    # software deceleration travel. The base applies its own
                    # braking limit; limit only acceleration here.
                    self.v = (self.nav_v if self.nav_v <= previous_v else
                              previous_v+clamp(self.nav_v-previous_v,self.config.linear_accel_mps2*dt))
                    self.w = (self.nav_w if abs(self.nav_w) <= abs(previous_w) else
                              previous_w+clamp(self.nav_w-previous_w,self.config.angular_accel_rps2*dt))
            elif self.state == 'NAV_STOPPING':
                if wall_s-self.arrival_wall > 5.:
                    self.stop('Xe chưa đứng yên sau khi Nav2 báo tới goal; đã dừng', 'BLOCKED')
                elif abs(p.linear) <= .015 and abs(p.angular) <= .035:
                    # Invalid RGB-D clears the visual settling timer. Physical
                    # arrival uses fresh odometry and must survive that loss.
                    if self.nav_settle_started is None:
                        self.nav_settle_started = wall_s
                    if wall_s-self.nav_settle_started >= self.config.settle_s:
                        self.stationary_arrival_confirmed = True
                        self.state, self.reason = 'CHECKING', 'Xe đã dừng; chờ RGB-D mới để kiểm tra ngang và góc'
                        self.arrival_measure_after = now_s
                        self.arrival_wall = wall_s
                        self.invalidate_target(self.reason)
                else:
                    self.nav_settle_started = None
            return self.snapshot()
        ready, reason = super().readiness(now_s,wall_s)
        if not ready:
            self.settle_started = None
            self.state, self.reason = 'CHECKING', reason
            if self.visual_hold_since is None:
                self.visual_hold_since = wall_s
            # After successful arrival verification, a single bad frame
            # pauses motion immediately but must not spend the old arrival
            # deadline again. Only a continuous loss requests reacquisition.
            expired = (wall_s-self.visual_hold_since > .75 if self.arrival_validated else
                       self.arrival_wall is not None and wall_s-self.arrival_wall > 8.)
            if expired:
                self.stop('Đã dừng tại goal nhưng chưa có RGB-D mới. Nhận diện lại rồi bấm căn tiếp', 'REACQUIRE_REQUIRED')
            return self.snapshot()
        self.visual_hold_since = None
        if target_changed_since_staging(self.frozen_target,self.target):
            self.stop('Pallet đo lại khác vị trí/hướng trước khi xe chạy; cần kiểm tra trực quan',
                      'TARGET_JUMP')
            return self.snapshot()
        if not self.arrival_validated:
            if not self.post_measurement_stable:
                self.settle_started = None
                self.state, self.reason = 'CHECKING', 'Chờ cửa sổ RGB-D ổn định 5 giây sau khi xe dừng'
                if self.arrival_wall is not None and wall_s-self.arrival_wall > 15.:
                    self.stop('RGB-D chưa ổn định sau khi tới goal; cần kiểm tra lại', 'REACQUIRE_REQUIRED')
                return self.snapshot()
            self.arrival_validated = True
        ex, ey, heading, _ = self.errors(p)
        if ex < 0:
            self.stop('Khoảng cách đo mới dưới 1,20 m; không xoay tiếp', 'TOO_CLOSE')
            return self.snapshot()
        theta = self.target[2]
        cross_track = -(self.target[0]-p.x)*math.sin(theta)+(self.target[1]-p.y)*math.cos(theta)
        aligned=within_alignment_tolerances(self.target,(p.x,p.y,p.yaw),self.config)
        if (not self.heading_only_trim and abs(cross_track) >= .03 and not aligned
                and not centered_for_visual_trim(self.target,(p.x,p.y,p.yaw),self.config)):
            if self.corrections >= self.config.max_corrections:
                self.stop('Sai số ngang còn lớn sau các lượt Nav2; cần kiểm tra', 'LATERAL_NOT_REACHED')
            else:
                self.corrections += 1
                self.nav_started = wall_s
                self._plan()
            return self.snapshot()
        settled_heading = abs(heading)<math.radians(.5) if self.heading_only_trim else aligned
        if settled_heading:
            self.state, self.reason = 'SETTLING', (
                'Đã bù góc dưới 0,5°; giữ xe đứng yên và đo lại ngang'
                if self.heading_only_trim else
                f'Đang xác nhận ngang <{self.config.lateral_tolerance_m*100:g} cm, góc <{self.config.heading_tolerance_deg:g}° và khoảng cách ≥1,20 m')
            if abs(p.linear)<=.015 and abs(p.angular)<=.035:
                if self.settle_started is None:
                    self.settle_started = wall_s
                if wall_s-self.settle_started >= self.config.settle_s:
                    if self.heading_only_trim and not aligned:
                        self.stop(f'Đã quay bù góc; còn {math.degrees(heading):+.2f}°, '
                                  f'ngang sau quay {ey*100:+.2f} cm. Xe dừng, chờ kiểm tra đường căn ngang tới checkpoint',
                                  'YAW_CORRECTED')
                    else:self.stop('Đã căn đạt ngang, góc và khoảng cách tối thiểu', 'ALIGNED')
            else:
                self.settle_started = None
        else:
            self.settle_started = None
            self.state, self.reason = 'TURNING', 'Đang sửa góc bằng RGB-D; không tiến vào pallet'
            # A tiny axis offset can be removed on the checked insertion
            # line. Aim the opening centre while staying within 80% of the
            # accepted face angle; otherwise raw Y can remain >1 cm even
            # when the car is parallel. Final errors still use the face yaw.
            bearing=math.atan2(self.target[1]-p.y,self.target[0]-p.x)
            centre_offset=(0. if self.heading_only_trim else
                           clamp(wrap(bearing-theta),math.radians(self.config.heading_tolerance_deg)*.8))
            turn_error=wrap(theta+centre_offset-p.yaw)
            if self.trim_braking_since is not None:
                self.reason='Đang chờ xe dừng sau nhịp quay tinh'
                if abs(p.angular)<=.015 and wall_s-self.trim_braking_since>=.25:
                    self.trim_braking_since=None
                return self.snapshot()
            stopping_angle=.10*abs(p.angular)+p.angular**2/(2*self.config.angular_accel_rps2)
            if p.angular*turn_error>0 and abs(turn_error)<=stopping_angle+math.radians(.15):
                self.trim_braking_since=wall_s
                self.reason='Đang phanh trước hướng mục tiêu, chờ RGB-D mới'
                return self.snapshot()
            # The active 0.22-rad/s pivot is useful for coarse turns but its
            # measured stopping angle can exceed the entire 1-degree target.
            # Fine trim uses a lower request and measured early braking.
            minimum = min(self.config.min_turn_rps,.10) if abs(turn_error)<math.radians(3.) else self.config.min_turn_rps
            desired = math.copysign(max(minimum,
                min(abs(1.2*turn_error),self.config.max_angular_rps)),turn_error)
            _,desired = bounded_nav_command(0.,desired,self.config)
            self.w = previous_w+clamp(desired-previous_w,self.config.angular_accel_rps2*dt)
        return self.snapshot()

    def snapshot(self):
        result = super().snapshot()
        result.update(strategy='nearest_normal_nav2', corrections=self.corrections,
                      heading_only_trim=self.heading_only_trim,
                      max_staging_turn_deg=math.degrees(MAX_STAGING_TURN_RAD),
                      clearance_min_m=self.config.standoff_m,
                      clearance_margin_m=planning_clearance_m(self.config)-self.config.standoff_m,
                      initial_clearance_margin_m=self.config.initial_clearance_margin_m,
                      nav_snapshot=self.navigating,
                      stationary_arrival_confirmed=self.stationary_arrival_confirmed,
                      stationary_trim_start_confirmed=self.stationary_trim_start_confirmed,
                      candidate_index=self.candidate_index,
                      candidate_count=len(self.candidates))
        if self.odom.samples and (self.target is not None or self.frozen_target is not None):
            ex,ey,heading,_=self.errors(self.odom.samples[-1])
            result['errors']=dict(distance_m=ex,lateral_m=ey,heading_deg=math.degrees(heading))
            result['fork_plane_distance_m']=ex+self.config.standoff_m
        else:
            result['fork_plane_distance_m']=None
        return result
