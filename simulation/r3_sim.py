#!/usr/bin/env python3
"""Run approved r3 Nav2 plugins + existing ShadowFollower in kinematic simulation.
No actuator publishers. Synthetic odom is explicitly separate from Gazebo/real odom.
"""
import argparse
import csv
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import hashlib
import itertools
from datetime import datetime
from zoneinfo import ZoneInfo

from docking_preview import ShadowFollower
from r3_core import (R3_MIN_STRAIGHT_CLEARANCE_M, FrozenPlan, collision, compose, dense, digest, errors, fork_for_base,
                     goal_base, inverse, polygons, rectangle, validate_phases, wrap)

ROOT = Path(__file__).resolve().parents[1]
PRIMITIVES = Path('/opt/ros/jazzy/share/nav2_smac_planner/sample_primitives/5cm_resolution/0.5m_turning_radius/diff/output.json')
sys.path.insert(0, str(ROOT/'camera/detect_astra/scripts'))
from pallet_alignment_core import stationary_heading_command


def parameters():
    return {
        'Lattice.lattice_filepath': str(PRIMITIVES),
        'Lattice.allow_reverse_expansion': False, 'Lattice.smooth_path': False,
        'Lattice.allow_unknown': False, 'Lattice.tolerance': 0.0,
        'Lattice.rotation_penalty': 5.0, 'Lattice.max_planning_time': 5.0,
        'Lattice.lookup_table_size': 10.0,
        'Constrained.minimum_turning_radius': .6,
        'Constrained.reversing_enabled': False,
        'Constrained.keep_start_orientation': True, 'Constrained.keep_goal_orientation': True,
        'Constrained.path_downsampling_factor': 1, 'Constrained.path_upsampling_factor': 5,
        'Constrained.w_smooth': 2000000.0, 'Constrained.w_curve': 30.0,
        'Constrained.w_dist': 0.0, 'Constrained.w_cost': .015,
    }


def build_backend():
    build = Path('/tmp/tai_r3_backend_build')
    for args in (['cmake', '-S', str(ROOT/'simulation/r3_nav2_backend'), '-B', str(build)],
                 ['cmake', '--build', str(build), '-j2']):
        subprocess.run(args, check=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    return build/'r3_nav2_backend'


def plan_once(start, snapshot, g, backend, out):
    """Search valid A's; optimize approach only; reject, never substitute planner."""
    out.mkdir(parents=True, exist_ok=True)
    if (out/'frozen_plan.json').exists():
        raise ValueError('a frozen plan already exists; choose a new output directory')
    target = snapshot['pose']
    # Solve in an odom subframe aligned with the continuous requested BASE yaw.
    # Heading bins are search discretization, never the terminal heading.
    origin = (*target[:2], target[2]-g['base_fork'][2])
    local = inverse(origin)
    start_local = compose(local, start)
    footprint = [[-.303, -.232], [.3305, -.232],
                 [g['base_fork'][0]+g['fork_length'], g['base_fork'][1]-.0775],
                 [g['base_fork'][0]+g['fork_length'], g['base_fork'][1]+.0775],
                 [.3305, .232], [-.303, .232]]
    params = parameters()
    metadata = json.loads(PRIMITIVES.read_text())['lattice_metadata']
    if metadata['motion_model'] != 'diff' or metadata['grid_resolution'] != .05:
        raise ValueError('approved differential primitives at5cm resolution required')
    successes, attempts = [], []
    # A is chosen before either tip crosses the front plane (20mm synthetic
    # geometric margin); terminal line can never collapse to zero length.
    for i, extra in enumerate((.020, .070, .120, .220)):
        gap = g['fork_length']+extra
        a = goal_base(target, g, gap)
        al = compose(local, a)
        # Put A exactly at a cell center to avoid a hidden XY snap at the join.
        margin=2.
        xmin=min(start_local[0],al[0],0.)-margin
        ymin=min(start_local[1],al[1],-g['pallet_width']/2)-margin
        xmax=max(start_local[0],al[0],g['pallet_length'])+margin
        ymax=max(start_local[1],al[1],g['pallet_width']/2)+margin
        ox=al[0]-(math.ceil((al[0]-xmin)/.05)+.5)*.05
        oy=al[1]-(math.ceil((al[1]-ymin)/.05)+.5)*.05
        width=math.ceil((xmax-ox)/.05)+1
        height=math.ceil((ymax-oy)/.05)+1
        # Pallet projection is an actual obstacle, not a fake guide wall.
        # Full 3D tine/deck geometry is checked again after smoothing.
        half = g['pallet_width']/2
        obstacles = [[0., -half, g['pallet_length'], half]]
        req = dict(start=start_local, goal=al, resolution=.05, origin=[ox, oy],size=[width,height],
                   footprint=footprint, obstacles=obstacles, parameters=params)
        request, response = out/f'request_{i}.json', out/f'response_{i}.json'
        request.write_text(json.dumps(req, indent=2))
        response.unlink(missing_ok=True)
        env = dict(os.environ, ROS_DOMAIN_ID='89', ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
        result = subprocess.run([str(backend), str(request), str(response)], env=env,
                                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=20)
        (out/f'nav2_{i}.log').write_text(result.stdout)
        try:
            if result.returncode:
                raise ValueError(f'Nav2 backend exited {result.returncode}; see nav2_{i}.log')
            answer = json.loads(response.read_text())
            if answer['status'] != 'OK':
                raise ValueError(answer.get('reason', 'backend failed'))
            phases = [dict(type=p['type'], poses=[compose(origin, v) for v in p['poses']])
                      for p in answer['phases']]
            if not phases:
                raise ValueError('no approach phases')
            if (math.dist(phases[0]['poses'][0][:2],start[:2])>1e-7
                    or abs(wrap(phases[0]['poses'][0][2]-start[2]))>1e-7):
                raise ValueError('lattice phase does not preserve continuous start pose')
            # Endpoint orientation is exact after constrained optimization;
            # the line is independently constructed and never sent to Ceres.
            end = goal_base(target, g)
            phases.append(dict(type='STRAIGHT', stop_at_join=True,
                               poses=list(dense([a, end], step=.01))))
            validate_phases(phases, target, g)
            length = sum(math.dist(a[:2], b[:2]) for p in phases
                         for a, b in zip(p['poses'], p['poses'][1:]))
            rotations = sum(p['type'] == 'ALIGN_YAW' for p in phases)
            config = dict(parameters=params, primitive_hash=digest(json.loads(PRIMITIVES.read_text())),
                          geometry_hash=digest(g), model='kinematic_unicycle_synthetic_odom',
                          follower='existing ShadowFollower Pure Pursuit + existing stationary_heading_command',
                          speed_profile=dict(omega_reserve_fraction=.8,curvature_preview_samples=80,
                                             max_speed=g['max_speed'],acceleration=g['acceleration'],
                                             max_omega=g['max_omega'],angular_acceleration=g['angular_acceleration'],
                                             join_settle_s=.5,command_stop_numeric_epsilon_m=1e-6),
                          source_hashes={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest()
                                         for p in [ROOT/'simulation/r3_core.py',ROOT/'simulation/r3_sim.py',
                                                   ROOT/'simulation/docking_preview.py',ROOT/'simulation/r3_nav2_backend/main.cpp',
                                                   ROOT/'camera/detect_astra/scripts/pallet_alignment_core.py']},
                          global_optimality_claim=False)
            plan = FrozenPlan.create(snapshot, g, phases, config)
            successes.append((length, rotations, plan, phases))
            attempts.append(dict(A_root_gap_m=gap, status='FEASIBLE', length_m=length, rotations=rotations))
        except (ValueError, KeyError, OSError) as exc:
            attempts.append(dict(A_root_gap_m=gap, status='REJECTED', reason=str(exc)))
    (out/'planner_attempts.json').write_text(json.dumps(attempts, indent=2))
    if not successes:
        raise ValueError('no valid lattice/smoother join: '+json.dumps(attempts))
    length, rotations, plan, phases = min(successes, key=lambda v: (v[0], v[1]))
    (out/'frozen_plan.json').write_text(json.dumps(dict(vars(plan), plan_id=plan.plan_id), indent=2))
    return plan, phases, length, rotations


class FixedPlanFollower:
    """Phase/endpoint adapter; retains ShadowFollower Pure Pursuit steering.

    Rotation uses the existing pallet_alignment_core stationary law. Pure
    Pursuit still steers translating phases; target vision never enters here.
    """
    # The drive cannot resolve a one-micrometre endpoint. Keep this below the
    # real trial's 5 mm terminal gap check and verify the actual odom there.
    endpoint_tolerance_m = .003
    min_straight_entry_clearance_m = R3_MIN_STRAIGHT_CLEARANCE_M
    approach_brake_m = .05
    alignment_done_deg = .25
    alignment_settle_s = .30
    c_position_tolerance_m = .02
    c_heading_tolerance_deg = 2.
    c_stop_trigger_m = .005
    c_command_latency_s = .20

    def __init__(self, phases, target, geometry, approach_only=False):
        self.phases, self.target, self.g = phases, target, geometry
        self.approach_only = approach_only
        if approach_only and (not phases or phases[0]['type'] != 'APPROACH'):
            raise ValueError('arrival at C requires an initial APPROACH phase')
        self.phase = 0
        self.state = 'PREVIEW_WAIT'
        self.follower = None
        self.v = 0.
        self.reason = ''
        self.align_goal = None
        self.w = 0.
        self.join_stopped_s = 0.
        self.align_stable_s = 0.
        self.curvature = []

    def align(self, pose, goal_yaw, dt):
        if errors(pose, self.target, self.g)['gap_m'] <= self.g['fork_length']:
            self.state, self.reason = 'STOP_LATCHED', 'rotation would occur after fork entry'
            return 0., 0.
        w = stationary_heading_command(pose[2], goal_yaw, self.g['max_omega'])
        dw = self.g['angular_acceleration']*dt
        w = max(self.w-dw, min(self.w+dw, w))
        # Check the full predicted sweep, including both tines.
        for p in dense([pose, (*pose[:2], wrap(pose[2]+w*dt))]):
            if collision(p, self.target, self.g):
                self.state, self.reason = 'STOP_LATCHED', 'alignment sweep occupied'
                return 0., 0.
        self.w = w
        return 0., w

    def start(self, operator_start):
        if not operator_start or self.state != 'PREVIEW_WAIT':
            return False
        self.state = self.phases[0]['type']
        return True

    def step(self, pose, dt=.05, obstacle=False, odom_valid=True, c_verified=True):
        if obstacle or not odom_valid:
            self.state = 'STOP_LATCHED'
            self.reason = 'obstacle stop' if obstacle else 'invalid odom'
        if self.state in ('PREVIEW_WAIT', 'STOP_LATCHED', 'STOPPED', 'C_REACHED'):
            return 0., 0.
        if self.align_goal is not None:
            if abs(wrap(self.align_goal-pose[2])) < math.radians(self.alignment_done_deg):
                self.w = 0.
                self.align_stable_s += dt
                if self.align_stable_s < self.alignment_settle_s:
                    return 0., 0.
                e = errors(pose, self.target, self.g)
                if (self.phase+1 < len(self.phases)
                        and self.phases[self.phase+1]['type'] == 'STRAIGHT'
                        and (abs(e['ey_m']) > self.g['max_lateral']
                             or abs(e['yaw_error_rad']) > math.radians(self.g['max_yaw_deg'])
                             or e['gap_m']-self.g['fork_length']
                             < self.min_straight_entry_clearance_m)):
                    self.state, self.reason = 'STOP_LATCHED', 'C alignment or fork-tip clearance failed'
                    return 0., 0.
                self.align_goal = None
                self.align_stable_s = 0.
                self.w = 0.
                if not c_verified:
                    self.state = 'WAIT_C_RGBD'
                    return 0., 0.
                self.phase += 1
                self.follower = None
                self.state = self.phases[self.phase]['type']
                return 0., 0.
            self.align_stable_s = 0.
            return self.align(pose, self.align_goal, dt)
        phase = self.phases[self.phase]
        if phase['type'] == 'ALIGN_YAW':
            goal_yaw = phase['poses'][-1][2]
            if abs(wrap(goal_yaw-pose[2])) < math.radians(self.alignment_done_deg):
                self.phase += 1
                self.follower = None
                self.state = self.phases[self.phase]['type']
                self.w = 0.
                return 0., 0.
            return self.align(pose, goal_yaw, dt)
        if self.follower is None:
            # Uniform spacing preserves the demo's 12-sample lookahead meaning.
            path = [(*p, 0.) for p in dense(phase['poses'], step=.01)]
            self.follower = ShadowFollower(path, max_speed=self.g['max_speed'],
                                           acceleration=self.g['acceleration'], dt=dt,
                                           pre_dock_completion=False)
            self.follower.start(True)
            self.curvature = [abs(wrap(b[2]-a[2]))/max(1e-9, math.dist(a[:2], b[:2]))
                              for a, b in zip(path, path[1:])]
            self.curvature.append(0.)
        e = errors(pose, self.target, self.g)
        end = phase['poses'][-1]
        dx, dy = end[0]-pose[0], end[1]-pose[1]
        remaining = dx*math.cos(end[2])+dy*math.sin(end[2])
        if self.approach_only:
            # C is a staging pose outside the pallet. The insertion-grade
            # lateral gate belongs to C->B, not to the approach's last 12 cm.
            if e['gap_m']-self.g['fork_length'] < self.min_straight_entry_clearance_m:
                self.state, self.reason = 'STOP_LATCHED', 'fork-tip clearance lost approaching C'
                return 0., 0.
            if remaining <= self.c_stop_trigger_m:
                self.v = self.w = self.follower.v = 0.
                self.state = 'C_REACHED'
                return 0., 0.
        if phase['type'] == 'STRAIGHT':
            if abs(e['ey_m']) > self.g['max_lateral'] or abs(e['yaw_error_rad']) > math.radians(self.g['max_yaw_deg']):
                self.state, self.reason = 'STOP_LATCHED', 'terminal yaw/lateral limit exceeded'
                return 0., 0.
        if remaining < -self.endpoint_tolerance_m:
            self.state, self.reason = 'STOP_LATCHED', 'phase endpoint overshot'
            return 0., 0.
        if (not self.approach_only and phase['type'] == 'APPROACH' and self.phase+1 < len(self.phases)
                and self.phases[self.phase+1]['type'] == 'STRAIGHT'
                and remaining <= self.approach_brake_m):
            # Stop before C so Pure Pursuit cannot chase millimetres of
            # lateral error with a large yaw command at the path endpoint.
            self.v = self.w = 0.
            if self.follower is not None:
                self.follower.v = 0.
            if e['gap_m']-self.g['fork_length'] < self.min_straight_entry_clearance_m:
                self.state, self.reason = 'STOP_LATCHED', 'fork-tip clearance lost before C'
                return 0., 0.
            self.join_stopped_s += dt
            if self.join_stopped_s < .5:
                self.state = 'C_SETTLE'
                return 0., 0.
            if abs(wrap(end[2]-pose[2])) >= math.radians(self.g['max_yaw_deg']):
                self.align_goal = end[2]
                self.align_stable_s = 0.
                self.state = 'ALIGN_YAW'
                return 0., 0.
            if abs(e['ey_m']) > self.g['max_lateral']:
                self.state, self.reason = 'STOP_LATCHED', 'C lateral limit exceeded'
                return 0., 0.
            if not c_verified:
                self.state = 'WAIT_C_RGBD'
                return 0., 0.
            self.phase += 1
            self.follower = None
            self.join_stopped_s = 0.
            self.state = self.phases[self.phase]['type']
            return 0., 0.
        if abs(remaining) <= self.endpoint_tolerance_m:
            self.v = 0.
            if self.phase+1 < len(self.phases):
                # Existing settle interval: do not switch curvature/align
                # while the previous angular command is still decelerating.
                self.w = 0.
                self.join_stopped_s += dt
                if self.join_stopped_s < .5:
                    return 0., 0.
                self.join_stopped_s = 0.
                # Correct yaw while the fork tips are still clear of the
                # pallet; yaw error alone can shift the fork-root lateral
                # position past the 9 mm insertion tolerance.
                if abs(wrap(end[2]-pose[2])) >= math.radians(self.g['max_yaw_deg']):
                    self.align_goal = end[2]
                    self.align_stable_s = 0.
                    self.state = 'ALIGN_YAW'
                    return 0., 0.
                if (abs(e['ey_m']) > self.g['max_lateral']
                        or e['gap_m']-self.g['fork_length']
                        < self.min_straight_entry_clearance_m):
                    self.state, self.reason = 'STOP_LATCHED', 'C lateral or fork-tip clearance failed'
                    return 0., 0.
                self.phase += 1
                self.follower = None
                self.state = self.phases[self.phase]['type']
                return 0., 0.
            if abs(e['ey_m']) > self.g['max_lateral']:
                self.state, self.reason = 'STOP_LATCHED', 'phase endpoint lateral limit exceeded'
                return 0., 0.
            if (abs(e['gap_m'] - self.g['stop_gap']) > .005
                    or abs(e['yaw_error_rad']) > math.radians(self.g['max_yaw_deg'])):
                self.state, self.reason = 'STOP_LATCHED', 'terminal odom goal criteria failed'
                return 0., 0.
            self.state = 'STOPPED'
            self.w = 0.
            return 0., 0.
        previous_v = self.v
        v, w = self.follower.step(*pose)
        # Brake by available deceleration. A linear remaining-distance cap
        # drove the real robot below 1 mm/s near C, where its motors stalled.
        # The endpoint check above still sends exact zero at the odom target.
        if self.approach_only:
            # Reserve the measured command pipeline delay as well as braking
            # distance; keep the terminal speed high enough to reach C rather
            # than exponentially creeping toward an unreachable zero.
            a = self.g['acceleration']
            budget = max(0., remaining)
            v = min(v, math.sqrt((a*self.c_command_latency_s)**2+2*a*budget)
                    - a*self.c_command_latency_s)
        else:
            v = min(v, math.sqrt(2*self.g['acceleration']*max(0., remaining)))
        # Same PP curvature, with speed profiling applied consistently to omega.
        tx, ty, _, _ = self.follower.path[min(len(self.follower.path)-1, self.follower.index+12)]
        dist = math.hypot(tx-pose[0], ty-pose[1])
        alpha = math.atan2(ty-pose[1], tx-pose[0])-pose[2]
        kappa = 2*math.sin(alpha)/max(.10, dist)
        # Brake ahead of curvature rather than saturating steering at cruise.
        # 80% of the existing omega limit reserves feedback correction room.
        ahead = 0.
        path = self.follower.path
        for j in range(self.follower.index, min(len(path), self.follower.index+80)):
            if j > self.follower.index:
                ahead += math.dist(path[j-1][:2], path[j][:2])
            curve_speed = min(self.g['max_speed'], .8*self.g['max_omega']/max(self.curvature[j], 1e-9))
            v = min(v, math.sqrt(curve_speed**2+2*self.g['acceleration']*ahead))
        if not (self.approach_only and phase.get('tracking_controller') == 'pose_feedback'):
            # Only Pure Pursuit commands this point-chasing curvature. Near
            # C its endpoint curvature can diverge despite small tangent
            # feedback, unnecessarily starving the actual drive of speed.
            v = min(v, .8*self.g['max_omega']/max(abs(kappa), 1e-9))
        # Profiling may ask for a sharper slowdown than the actuator allows.
        # Honor the existing acceleration/deceleration limit and let steering
        # saturation/error checks report any infeasible tracking transition.
        dv=self.g['acceleration']*dt
        v=max(previous_v-dv,min(previous_v+dv,v))
        v=max(0.,min(self.g['max_speed'],v))
        # Reset speed state to the actual profile output; no minimum-speed floor.
        self.follower.v = self.v = v
        if phase.get('tracking_controller') == 'pose_feedback':
            # Follow the local path tangent rather than a point 12 cm ahead.
            # A fixed-distance Pure Pursuit target cuts across a shallow S and
            # responds late to its change of turn direction at this low speed.
            i = min(self.follower.index, len(path)-2)
            ref, nxt = path[i], path[i+1]
            ds = math.dist(ref[:2], nxt[:2])
            signed_curvature = wrap(nxt[2]-ref[2])/max(ds, 1e-9)
            lateral = (-(pose[0]-ref[0])*math.sin(ref[2])
                       +(pose[1]-ref[1])*math.cos(ref[2]))
            heading = wrap(pose[2]-ref[2])
            desired_w = v*signed_curvature - 2.*math.sin(heading) - 16.*v*lateral
            # The same feedback law runs through the straight lead-in, so
            # lateral feedback is not discarded at its boundary.
            if (not self.approach_only and phase['type'] == 'APPROACH'
                    and remaining <= phase.get('straight_lead_in_m', 0.)
                    and abs(e['ey_m']) > self.g['max_lateral']):
                self.state, self.reason = 'STOP_LATCHED', 'straight lead-in lateral limit exceeded'
                return 0., 0.
        elif (phase['type'] == 'APPROACH' and phase.get('straight_lead_in_m', 0.) > 0
                and remaining <= phase['straight_lead_in_m']):
            # The last approach samples are a straight lead-in. Pure Pursuit
            # can continue steering for a few millimetres of lateral error
            # even after yaw reaches the pallet axis, reversing the final
            # heading. Hold the terminal heading here instead. A large
            # lateral miss cannot be repaired safely beside the pallet.
            if not self.approach_only and abs(e['ey_m']) > self.g['max_lateral']:
                self.state, self.reason = 'STOP_LATCHED', 'straight lead-in lateral limit exceeded'
                return 0., 0.
            desired_w = 1.2*wrap(end[2]-pose[2])
        else:
            desired_w = v*kappa
        w = max(-self.g['max_omega'], min(self.g['max_omega'], desired_w))
        dw = self.g['angular_acceleration']*dt
        w = max(self.w-dw, min(self.w+dw, w))
        self.w = w
        return v, w


def simulate(phases, target, g, start):
    follower = FixedPlanFollower(phases, target, g)
    follower.start(True)  # Operator event is simulated, never sent to a real robot.
    pose = tuple(start)
    rows = []
    for k in itertools.count():
        t = k*.05
        v, w = follower.step(pose)
        hit = next((collision(p, target, g) for p in dense([pose, compose(pose, (v*.05, 0., w*.05))])
                    if collision(p, target, g)), None)
        if hit:
            follower.state, follower.reason = 'STOP_LATCHED', 'executed collision: '+hit
            v = w = 0.
        e = errors(pose, target, g)
        f = fork_for_base(pose, g)
        rows.append(dict(time_s=t, state=follower.state, x_m=pose[0], y_m=pose[1], yaw_rad=pose[2],
                         fork_x_m=f[0], fork_y_m=f[1], fork_yaw_rad=f[2],
                         cmd_v_mps=v, cmd_w_rps=w, lift_m=0., **e))
        if follower.state in ('STOP_LATCHED', 'STOPPED'):
            # Confirm actual synthetic plant rest for the existing settle interval.
            for j in range(1, 11):
                rows.append(dict(rows[-1], time_s=t+j*.05))
            break
        # Exact constant-twist unicycle integration; synthetic odom equals plant
        # by construction. This cannot demonstrate Gazebo slip/contact accuracy.
        if abs(w) < 1e-10:
            pose = (pose[0]+v*.05*math.cos(pose[2]), pose[1]+v*.05*math.sin(pose[2]), pose[2])
        else:
            yaw = pose[2]+w*.05
            pose = (pose[0]+v/w*(math.sin(yaw)-math.sin(pose[2])),
                    pose[1]-v/w*(math.cos(yaw)-math.cos(pose[2])), wrap(yaw))
    return rows, follower


def write_csv(path, rows):
    with path.open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def export(out, plan, phases, length, rotations, rows, follower, g):
    snapshot = json.loads(plan.target_json)
    target = snapshot['pose']
    poses = []
    arc = 0.
    for phase in phases:
        pp = phase['poses']
        speeds = []
        kappas = []
        for a,b in zip(pp,pp[1:]):
            ds=math.dist(a[:2],b[:2]);kappa=wrap(b[2]-a[2])/max(ds,1e-9)
            kappas.append(kappa)
            speeds.append(min(g['max_speed'], .8*g['max_omega']/max(abs(kappa),1e-9)))
        kappas.append(0.);speeds.append(0.);speeds[0]=0.
        for i in range(1,len(pp)):
            ds=math.dist(pp[i-1][:2],pp[i][:2])
            speeds[i]=min(speeds[i],math.sqrt(speeds[i-1]**2+2*g['acceleration']*ds))
        for i in range(len(pp)-2,-1,-1):
            ds=math.dist(pp[i+1][:2],pp[i][:2])
            speeds[i]=min(speeds[i],math.sqrt(speeds[i+1]**2+2*g['acceleration']*ds))
        for i,v in enumerate(pp):
            if i:arc+=math.dist(pp[i-1][:2],v[:2])
            poses.append(dict(phase=phase['type'],arc_length_m=arc,x_m=v[0],y_m=v[1],yaw_rad=v[2],
                              curvature_1pm=kappas[i],reference_v_mps=speeds[i]))
    forks = [dict(phase=v['phase'], **dict(zip(('x_m', 'y_m', 'yaw_rad'), fork_for_base((v['x_m'], v['y_m'], v['yaw_rad']), g)))) for v in poses]
    write_csv(out/'path_planned_base.csv', poses)
    write_csv(out/'path_planned_fork_root.csv', forks)
    write_csv(out/'executed_odom_base.csv', rows)
    write_csv(out/'executed_odom_fork_root.csv', [dict(time_s=r['time_s'], x_m=r['fork_x_m'], y_m=r['fork_y_m'], yaw_rad=r['fork_yaw_rad']) for r in rows])
    (out/'target_snapshot.json').write_text(json.dumps(snapshot, indent=2))
    final = rows[-1]
    line = [r for r in rows if r['state'] == 'STRAIGHT']
    metrics = dict(max_speed_mps=max(r['cmd_v_mps'] for r in rows),
                   line_max_yaw_deg=max((abs(math.degrees(r['yaw_error_rad'])) for r in line), default=None),
                   line_max_lateral_m=max((abs(r['ey_m']) for r in line), default=None),
                   final_gap_m=final['gap_m'], final_gap_error_m=final['gap_m']-.02,
                   final_yaw_deg=math.degrees(final['yaw_error_rad']), final_lateral_m=final['ey_m'],
                   length_m=length, rotation_phases=rotations,
                   executed_alignment_phases=sum(r['state']=='ALIGN_YAW' and (i==0 or rows[i-1]['state']!='ALIGN_YAW') for i,r in enumerate(rows)),
                   command_max_omega_rps=max(abs(r['cmd_w_rps']) for r in rows),
                   max_curvature_1pm=max((abs(wrap(b['yaw_rad']-a['yaw_rad']))/math.dist((a['x_m'], a['y_m']), (b['x_m'], b['y_m']))
                       for a,b in zip(poses, poses[1:]) if math.dist((a['x_m'],a['y_m']), (b['x_m'],b['y_m'])) > 1e-6), default=0.))
    result = dict(status='INCOMPLETE', provenance=g['provenance'], model='kinematic synthetic odom (not Gazebo or hardware)',
                  plan_id=plan.plan_id, transform_hash=digest(g['base_fork']), geometry_hash=digest(g),
                  controller_state=follower.state, controller_reason=follower.reason,
                  metrics=metrics, missing=g['missing_real'],
                  lift_mission_executed=False,
                  lift_reason='mission acceptance requires confirmed real geometry; Gazebo trial is reported separately',
                  real_camera_checked=False, real_robot_commands=0)
    (out/'mission_result.json').write_text(json.dumps(result, indent=2))
    import matplotlib.pyplot as plt
    fig,axs=plt.subplots(2,1,figsize=(10,5),sharex=True)
    axs[0].plot([p['arc_length_m'] for p in poses],[p['reference_v_mps'] for p in poses]);axs[0].set_ylabel('reference v (m/s)')
    axs[1].plot([p['arc_length_m'] for p in poses],[math.degrees(p['yaw_rad']) for p in poses]);axs[1].set_ylabel('reference base yaw (deg)');axs[1].set_xlabel('arc length (m)')
    fig.tight_layout();fig.savefig(out/'reference_speed_heading.png',dpi=150);plt.close(fig)
    plot(out, phases, rows, target, g, metrics)
    return result


def plot(out, phases, rows, target, g, metrics, *, preview_only=False,
         odom_label='synthetic odom'):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon
    poses = [v for p in phases for v in p['poses']]
    forks = [fork_for_base(p, g) for p in poses]
    from r3_plot import save_two_paths
    case_file=out/'case.json'
    scene=json.loads(case_file.read_text()) if case_file.exists() else None
    save_two_paths(out, forks, [], target, g, name='planned',scene=scene)
    if not preview_only:
        actual=[(r['fork_x_m'],r['fork_y_m']) for r in rows]
        save_two_paths(out, forks, actual, target, g, actual_label=odom_label+' · gốc càng',name='overlay_planned_odom',scene=scene)
    if preview_only:
        return
    fig, axes = plt.subplots(5, 1, figsize=(11, 10), sharex=True)
    t = [r['time_s'] for r in rows]
    for ax, key, label, scale in zip(axes, ['cmd_v_mps', 'yaw_error_rad', 'ey_m', 'gap_m', 'lift_m'],
                                    ['v (m/s)', 'yaw F (deg)', 'ey F (mm)', 'root gap (mm)', 'lift (mm)'], [1., 180/math.pi, 1000., 1000., 1000.]):
        ax.plot(t, [r[key]*scale for r in rows]); ax.set_ylabel(label); ax.grid(alpha=.25)
    axes[0].axhline(.15, color='r', ls=':')
    axes[1].axhline(.5, color='r', ls=':'); axes[1].axhline(-.5, color='r', ls=':')
    axes[2].axhline(20., color='r', ls=':'); axes[2].axhline(-20., color='r', ls=':')
    axes[3].axhline(20., color='r', ls=':')
    if not any(r['lift_m'] > .001 for r in rows):
        axes[4].text(.05, .6, 'Mission lift held: real geometry not confirmed', transform=axes[4].transAxes)
    axes[-1].set_xlabel('simulation time (s)'); fig.suptitle(odom_label+' / fixed target; no real robot commands')
    fig.tight_layout(); fig.savefig(out/'heading_lateral_speed_lift.png', dpi=150); plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    run_stamp=datetime.now(ZoneInfo('Asia/Ho_Chi_Minh')).strftime('%Y%m%d_%H%M%S')
    parser.add_argument('--output', type=Path, default=ROOT/'simulation/output/r3_runs'/run_stamp)
    parser.add_argument('--fixture', type=Path, default=ROOT/'simulation/r3/fixture.json')
    args = parser.parse_args()
    g = json.loads(args.fixture.read_text())
    if g['provenance'] != 'SYNTHETIC_FIXTURE_NOT_REAL_CALIBRATION':
        parser.error('This entry point is synthetic simulation only')
    backend = build_backend()
    cases = [('straight', 0., 0.), ('left', .08, 0.), ('right', -.08, 0.),
             ('yaw_left', .05, 8.), ('yaw_right', -.05, -8.)]
    results = []
    for name, lateral, yaw in cases:
        out = args.output/name; out.mkdir(parents=True, exist_ok=True)
        start = (0., lateral, math.radians(yaw))
        snapshot = dict(frame_id='odom', feature='front_entry_corridor_center',
                        pose=[1.8, 0., 0.], applied_bias=[], calibration_hash='synthetic_exact_input',
                        capture_stamp_s=0., source='EXACT_INPUT_SIMULATION (not RGB-D)')
        try:
            plan, phases, length, rotations = plan_once(start, snapshot, g, backend, out)
            rows, follower = simulate(phases, snapshot['pose'], g, start)
            result = export(out, plan, phases, length, rotations, rows, follower, g)
        except (ValueError, subprocess.SubprocessError) as exc:
            result = dict(status='INCOMPLETE', reason=str(exc), real_robot_commands=0)
            (out/'mission_result.json').write_text(json.dumps(result, indent=2))
        results.append(dict(case=name, **result))
        print(name, result.get('controller_state'), result.get('reason', result.get('metrics')), flush=True)
    (args.output/'mission_result.json').write_text(json.dumps(results, indent=2))


if __name__ == '__main__':
    main()
