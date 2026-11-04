"""Pallet-relative final insertion geometry. No ROS or actuator dependencies.

The live visual pickup owner freezes its final forward line in odom using
a fresh opening measurement, with independent body, fork and stopping checks.
Nominal-goal helpers remain available for checkpoint geometry and old records;
the active visual pickup owner does not use a saved map pickup goal.
"""
from dataclasses import dataclass
import math

import numpy as np


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass(frozen=True)
class InsertionConfig:
    fork_root_m: float = .3655
    fork_length_m: float = .390
    fork_half_span_m: float = .0775
    fork_inner_edge_m: float = .0245
    fork_bottom_at_home_m: float = .065
    fork_thickness_m: float = .008
    # Operator measurement, 2026-11-01: floor to underside of front beam.
    opening_bottom_m: float = 0.
    opening_top_m: float = .100
    side_margin_m: float = .010
    vertical_margin_m: float = .008
    root_stop_m: float = .040
    minimum_root_gap_m: float = .025
    longitudinal_tolerance_m: float = .025
    lateral_tolerance_m: float = .015
    heading_tolerance_rad: float = math.radians(1.5)
    maximum_goal_correction_m: float = .150
    maximum_goal_heading_correction_rad: float = math.radians(5)
    maximum_travel_m: float = 1.6
    maximum_speed_mps: float = .10
    maximum_measured_speed_mps: float = .14
    maximum_yaw_rate_rps: float = .06
    reaction_s: float = .15
    # Below the measured stopping performance of the bounded 43-cm approach;
    # never use the hardware's configured 0.5 m/s2 as a braking guarantee.
    braking_mps2: float = .15
    angular_braking_rps2: float = .40

    def __post_init__(self):
        if any(not math.isfinite(v) for v in vars(self).values()):
            raise ValueError('Nonfinite insertion configuration')
        if self.opening_bottom_m < 0 or self.opening_top_m <= self.opening_bottom_m:
            raise ValueError('Invalid measured vertical opening')
        if self.opening_top_m-self.vertical_margin_m < self.fork_bottom_at_home_m+self.fork_thickness_m:
            raise ValueError('Opening cannot accept the forks even at HOME')
        if any(v <= 0 for k, v in vars(self).items() if k != 'opening_bottom_m'):
            raise ValueError('Insertion limits must be positive')
        if (self.maximum_speed_mps > .10 or self.maximum_measured_speed_mps > .15
                or self.minimum_root_gap_m < .01 or self.side_margin_m < .01
                or self.vertical_margin_m < .008 or self.braking_mps2 > .15
                or self.reaction_s < .15 or self.maximum_yaw_rate_rps > .08
                or self.maximum_goal_correction_m > .15):
            raise ValueError('Unsupported insertion protection limits')
        if (self.root_stop_m <= self.minimum_root_gap_m
                or self.lateral_tolerance_m > .05
                or self.heading_tolerance_rad > math.radians(2)):
            raise ValueError('Invalid terminal geometry limits')


@dataclass(frozen=True)
class Opening:
    x: float
    y: float
    yaw: float
    width_m: float

    def __post_init__(self):
        if (not all(math.isfinite(v) for v in vars(self).values())
                or not .15 <= self.width_m <= .40):
            raise ValueError('Invalid measured opening')


@dataclass(frozen=True)
class InsertionPlan:
    start: tuple
    nominal_goal: tuple
    goal: tuple
    opening: Opening
    entry_height_m: float
    correction_m: float
    travel_m: float
    poses: tuple
    station_locked: bool = False


def opening_coordinates(opening, point):
    dx, dy = point[0]-opening.x, point[1]-opening.y
    c, s = math.cos(opening.yaw), math.sin(opening.yaw)
    return c*dx+s*dy, -s*dx+c*dy


def fork_vertices(pose, config):
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    return tuple((x+c*a-s*b, y+s*a+c*b)
                 for a in (config.fork_root_m, config.fork_root_m+config.fork_length_m)
                 for b in (-config.fork_half_span_m, config.fork_half_span_m))


def geometry_issue(pose, opening, height, config=InsertionConfig()):
    if not all(math.isfinite(v) for v in (*pose, height)):
        return 'Nonfinite pose/height'
    if not 0 <= height <= .52:
        return 'Entry lift height out of range'
    bottom = config.fork_bottom_at_home_m+height
    if (bottom < config.opening_bottom_m+config.vertical_margin_m
            or bottom+config.fork_thickness_m > config.opening_top_m-config.vertical_margin_m):
        return 'Fork does not fit measured vertical opening'
    heading = wrap(pose[2]-opening.yaw)
    if abs(heading) > config.heading_tolerance_rad:
        return 'Heading outside insertion limit'
    along, cross = opening_coordinates(opening, pose)
    root_gap = -along-config.fork_root_m*math.cos(heading)
    if root_gap < config.minimum_root_gap_m-1e-9:
        return 'Body/fork-root would cross pallet front protection'
    coords = [opening_coordinates(opening, v) for v in fork_vertices(pose, config)]
    # Bound the ENTIRE inserted fork rectangle, including the tip at nonzero
    # yaw, rather than testing only the robot centre or opening midpoint.
    if max(v[0] for v in coords) >= 0:
        if abs(cross) > config.lateral_tolerance_m:
            return 'Body is off the opening axis during entry'
        if max(abs(v[1]) for v in coords) > opening.width_m/2-config.side_margin_m:
            return 'Fork swept width would contact an opening side'
    return ''


def make_plan(start, nominal_goal, opening, entry_height_m, config=InsertionConfig(),preserve_lateral=False):
    if not all(math.isfinite(v) for v in (*start, *nominal_goal)):
        raise ValueError('Nonfinite start/nominal goal')
    issue = geometry_issue(start, opening, entry_height_m, config)
    if issue:
        raise ValueError('ALIGNMENT_REQUIRED: '+issue)
    if abs(wrap(nominal_goal[2]-opening.yaw)) > config.maximum_goal_heading_correction_rad:
        raise ValueError('Saved goal heading disagrees with measured station')
    along, cross = opening_coordinates(opening, start)
    advance = -along-config.fork_root_m-config.root_stop_m
    if not 0 < advance <= config.maximum_travel_m:
        raise ValueError('Invalid insertion travel')
    c, s = math.cos(opening.yaw), math.sin(opening.yaw)
    # Correct enough lateral error to fit the opening, while leaving steering
    # margin for real wheel yaw drift. A centreline goal from the recognition
    # standoff can consume nearly the entire one-degree heading allowance.
    # No lateral manoeuvre or stationary pivot is requested inside the pallet.
    goal = (opening.x-(config.fork_root_m+config.root_stop_m)*c,
            opening.y-(config.fork_root_m+config.root_stop_m)*s, opening.yaw)
    if not preserve_lateral:
        # Prefer the centreline. When joining it would exceed the accepted
        # heading by a tiny amount, use the nearest terminal point still
        # inside the lateral tolerance. Every fork vertex and the entire
        # entry corridor below remain checked; no geometric gate is relaxed.
        maximum_correction=advance*math.tan(config.heading_tolerance_rad*.97)
        # The tips reach the face before the chassis reaches its goal. Keep
        # their lateral position inside the opening at that earlier point.
        entry_fraction=max(0.,min(1.,(-along-config.fork_root_m-config.fork_length_m)/advance))
        entry_residual=(max(0.,config.lateral_tolerance_m*.9
                             -(1.-entry_fraction)*abs(cross))/entry_fraction
                        if entry_fraction>1e-6 else config.lateral_tolerance_m*.5)
        residual=math.copysign(max(min(abs(cross),
                                       config.lateral_tolerance_m*.5,entry_residual),
                                   abs(cross)-maximum_correction,0.),cross)
        if abs(residual)>=config.lateral_tolerance_m:
            raise ValueError('ALIGNMENT_REQUIRED: no insertion line within lateral/heading limits')
        goal=(goal[0]-s*residual,goal[1]+c*residual,goal[2])
    if preserve_lateral:
        # A short depth correction with forks already inside must keep the
        # current heading/lateral offset instead of joining the centreline
        # over just 3 cm, which could demand a large yaw inside the opening.
        heading=wrap(start[2]-opening.yaw)
        gap=-along-config.fork_root_m*math.cos(heading)
        distance=(gap-config.root_stop_m)/math.cos(heading)
        if not 0<distance<=.035:raise ValueError('Short insertion correction exceeds 3.5 cm')
        goal=(start[0]+distance*math.cos(start[2]),start[1]+distance*math.sin(start[2]),start[2])
    travel = math.hypot(goal[0]-start[0],goal[1]-start[1])
    bearing = math.atan2(goal[1]-start[1],goal[0]-start[0])
    if travel > config.maximum_travel_m:
        raise ValueError('Invalid insertion travel')
    correction = math.hypot(goal[0]-nominal_goal[0], goal[1]-nominal_goal[1])
    if correction > config.maximum_goal_correction_m:
        raise ValueError('Saved map goal and measured pallet disagree beyond correction bound')
    steps = max(1, math.ceil(travel/.01))
    poses = (tuple(start),)+tuple((start[0]+(goal[0]-start[0])*i/steps,
                                  start[1]+(goal[1]-start[1])*i/steps, bearing)
                                 for i in range(steps+1))
    for pose in poses:
        issue = geometry_issue(pose, opening, entry_height_m, config)
        if issue:
            raise ValueError(issue)
    return InsertionPlan(tuple(start), tuple(nominal_goal), goal, opening,
                         entry_height_m, correction, travel, poses)


def braking_poses(pose, linear, angular, config=InsertionConfig()):
    if not all(math.isfinite(v) for v in (*pose, linear, angular)):
        raise ValueError('Nonfinite braking state')
    x, y, yaw = pose
    yield pose
    dt = .025
    reaction = config.reaction_s
    for _ in range(300):
        if reaction > 1e-9:
            step = min(dt, reaction)
            next_v, next_w = linear, angular
            reaction -= step
        else:
            if abs(linear)+abs(angular) < 1e-9:
                return
            step = dt
            next_v = math.copysign(max(0., abs(linear)-config.braking_mps2*step), linear)
            next_w = math.copysign(max(0., abs(angular)-config.angular_braking_rps2*step), angular)
        v, w = (linear+next_v)/2, (angular+next_w)/2
        x += v*math.cos(yaw+w*step/2)*step
        y += v*math.sin(yaw+w*step/2)*step
        yaw = wrap(yaw+w*step)
        linear, angular = next_v, next_w
        yield (x, y, yaw)
    raise ValueError('Unbounded braking sweep')


def robot_boxes(entry_height_m, config=InsertionConfig()):
    """Conservative URDF collision boxes, including wheels and complete mast."""
    h = entry_height_m
    boxes = [('chassis', (-.3005,-.145,.125), (.3005,.145,.335)),
             ('mast', (.3005,-.13,.055), (.3305,.13,.735)),
             ('carriage', (.3305,-.13,.069+h), (.3655,.13,.129+h))]
    for x in (-.1775, .1775):
        for y in (-.214, .214):
            boxes.append(('wheel', (x-.0875,y-.0175,0.), (x+.0875,y+.0175,.175)))
    for lo, hi in ((.0245,.0775),(-.0775,-.0245)):
        boxes.append(('fork', (.3655,lo,.065+h), (.7555,hi,.073+h)))
    return boxes


def cloud_collision(points_odom, pose, height, body_margin=.01, fork_margin=.008):
    points = np.asarray(points_odom, float)
    if points.ndim != 2 or points.shape[1] != 3 or not np.isfinite(points).all():
        raise ValueError('Collision scene must contain finite XYZ points')
    p = points.copy()
    c, s = math.cos(pose[2]), math.sin(pose[2])
    p[:, :2] = (p[:, :2]-pose[:2])@np.array([[c,-s],[s,c]])
    for name, lo, hi in robot_boxes(height):
        margin = fork_margin if name == 'fork' else body_margin
        hits = np.all((p >= np.asarray(lo)-margin)&(p <= np.asarray(hi)+margin), axis=1)
        if name == 'wheel':
            # Wheels are cylinders around the lateral axle, not solid boxes
            # down to all four X/Z corners. Preserve the same radial margin.
            cx, cz = (lo[0]+hi[0])/2, (lo[2]+hi[2])/2
            radius = (hi[2]-lo[2])/2
            hits &= (p[:,0]-cx)**2+(p[:,2]-cz)**2 <= (radius+margin)**2
        if hits.any():
            return name, tuple(p[np.flatnonzero(hits)[0]])
    return None


def terminal_geometry(pose, linear, angular, plan, config=InsertionConfig()):
    if abs(linear) > .003 or abs(angular) > .015:
        return False
    if geometry_issue(pose, plan.opening, plan.entry_height_m, config):
        return False
    if plan.station_locked and (
            math.hypot(pose[0]-plan.nominal_goal[0],pose[1]-plan.nominal_goal[1])>.01
            or abs(wrap(pose[2]-plan.nominal_goal[2]))>math.radians(1)):
        return False
    along, _ = opening_coordinates(plan.opening, pose)
    gap = -along-config.fork_root_m*math.cos(wrap(pose[2]-plan.opening.yaw))
    return (gap >= config.minimum_root_gap_m
            and abs(gap-config.root_stop_m) <= config.longitudinal_tolerance_m)
