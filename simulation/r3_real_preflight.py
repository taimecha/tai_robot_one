#!/usr/bin/env python3
"""Read-only readiness report for an r3 frozen plan; never commands the robot."""

import argparse
import json
import math
from pathlib import Path
import time

from r3_core import (R3_PLANNED_C_CLEARANCE_M, FrozenPlan, errors,
                     real_geometry_mode, validate_phases, validate_axis_lateral_residual)
from r3_localization_contract import localization_spec


def inspect_plan(path, allow_provisional=False):
    blockers = []
    try:
        value = json.loads(path.read_text())
        plan = FrozenPlan(**{key: value[key] for key in
                             ('target_json', 'geometry_json', 'phases_json', 'config_json')})
        target = json.loads(plan.target_json)
        geometry = json.loads(plan.geometry_json)
        phases = json.loads(plan.phases_json)
        config = json.loads(plan.config_json)
    except (OSError, ValueError, KeyError, TypeError) as exc:
        return {'status': 'BLOCKED', 'plan': str(path), 'robot_commands': 0,
                'blockers': [f'Cannot read frozen plan: {exc}']}

    if plan.plan_id != value.get('plan_id'):
        blockers.append('Frozen plan hash does not match its contents.')
    try:
        validate_phases(phases, target['pose'], geometry)
        validate_axis_lateral_residual(target, phases[0]['poses'][0])
        spec = localization_spec(target)
        if spec and config.get('localization') != spec:
            raise ValueError('localization metadata differs between target and plan')
    except (ValueError, KeyError, TypeError, IndexError) as exc:
        blockers.append(f'Path/geometry validation failed: {exc}')

    if target.get('source') != 'stationary_real_RGBD':
        blockers.append('Target is not a locked real RGB-D measurement.')
    if not target.get('yolo_off_ack'):
        blockers.append('YOLO-off acknowledgement is missing.')
    report = target.get('measurement_report') or {}
    raw, offset, corrected = (report.get('camera_lateral_raw_m'),
                              report.get('camera_mount_offset_m'),
                              report.get('fork_lateral_corrected_m'))
    if (not all(isinstance(v, (int, float)) and math.isfinite(v)
                for v in (raw, offset, corrected))
            or abs(offset - .025) > 1e-9 or abs(raw + offset - corrected) > 1e-4):
        blockers.append('Displayable camera raw + one +2.5 cm correction report is missing.')
    if (report.get('heading_source') != 'floor_referenced_plane'
            or not isinstance(report.get('heading_depth_x_slope'), (int, float))
            or not math.isfinite(report['heading_depth_x_slope'])):
        blockers.append('Floor-referenced pallet heading measurement is missing.')
    for key in ('raw_session_hash', 'calibration_hash', 'capture_stamp_s'):
        if target.get(key) is None:
            blockers.append(f'Locked target lacks {key}.')
    if target.get('source') == 'stationary_real_RGBD' and target.get('capture_stamp_s') is not None:
        try:
            if not 0 <= time.time() - target['capture_stamp_s'] <= 30.:
                blockers.append('Locked target is older than 30 seconds; recognize again.')
        except TypeError:
            blockers.append('Invalid locked target timestamp.')
    if geometry.get('provenance') == 'SYNTHETIC_FIXTURE_NOT_REAL_CALIBRATION':
        blockers.append('Geometry comes from the synthetic fixture.')
    try:
        geometry_mode = real_geometry_mode(geometry, allow_provisional)
    except ValueError as exc:
        geometry_mode = 'blocked'
        missing = geometry.get('missing_real') or []
        blockers.append(str(exc) +
                        (': ' + '; '.join(map(str, missing)) if missing else ''))
    if geometry.get('max_lateral', float('inf')) > .010:
        blockers.append('Real trial terminal lateral tolerance must be at most 1 cm.')
    refinement = config.get('approach_refinement') or {}
    if (refinement.get('method') != 'bounded fork-root shape search on clamped quintic y(x)'
            or refinement.get('selected_C_unchanged') is not True
            or refinement.get('terminal_line_unchanged') is not True):
        blockers.append('Plan does not contain the reviewed bounded quintic A→C refinement.')
    if [phase.get('type') for phase in phases] != ['APPROACH', 'STRAIGHT']:
        blockers.append('Plan does not contain the reviewed A→C→B phase sequence.')
    else:
        try:
            clearance = (errors(phases[1]['poses'][0], target['pose'], geometry)['gap_m']
                         - geometry['fork_length'])
            if clearance < R3_PLANNED_C_CLEARANCE_M - 1e-6:
                blockers.append('C leaves less than 15 cm between fork tips and pallet face.')
        except (ValueError, KeyError, TypeError, IndexError):
            blockers.append('Cannot verify fork-tip clearance at C.')

    return {'status': 'BLOCKED' if blockers else 'STATIC_PLAN_VALID',
            'plan': str(path), 'plan_id': plan.plan_id,
            'target_source': target.get('source'), 'geometry_mode': geometry_mode,
            'robot_commands': 0,
            'blockers': blockers}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--plan', required=True, type=Path)
    parser.add_argument('--allow-provisional', action='store_true',
                        help='Validate explicit approximate-geometry trial plan up to 0.10 m/s')
    args = parser.parse_args()
    result = inspect_plan(args.plan, args.allow_provisional)
    print(json.dumps(result, indent=2, ensure_ascii=False))
    return 2 if result['blockers'] else 0


if __name__ == '__main__':
    raise SystemExit(main())
