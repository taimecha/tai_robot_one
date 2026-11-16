#!/usr/bin/env python3
"""Render three odom-frame trajectories after an r3 real trial has ended."""
import argparse
import csv
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
from matplotlib.figure import Figure
from matplotlib.patches import Polygon

from r3_core import FrozenPlan, errors, fork_for_base, polygons, rectangle, wrap, compose


def read_rows(path):
    with path.open(newline='') as stream:
        return list(csv.DictReader(stream))


def pallet_view(point, target):
    """Rotate the display frame so the pallet is horizontal; leave odom data intact."""
    dx, dy = point[0]-target[0], point[1]-target[1]
    c, s = math.cos(target[2]), math.sin(target[2])
    return (target[0]+c*dx+s*dy, -s*dx+c*dy)


def configure_view(ax, view):
    """Reflect the entire drawing for the operator's laptop viewpoint."""
    if view not in ('laptop', 'ros'):
        raise ValueError('view must be laptop or ros')
    if ax.yaxis_inverted() != (view == 'laptop'):
        ax.invert_yaxis()
    direction = '+ xuống dưới, − lên trên' if view == 'laptop' else '+ lên trên, − xuống dưới'
    ax.set_ylabel(f'Y ngang pallet (m) · {direction}')


def render(plan_path, trial_dir, output, view='laptop'):
    saved = json.loads(plan_path.read_text())
    plan = FrozenPlan(**{key: saved[key] for key in
                         ('target_json', 'geometry_json', 'phases_json', 'config_json')})
    if saved.get('plan_id') != plan.plan_id:
        raise ValueError('frozen plan hash mismatch')
    mission = json.loads((trial_dir/'mission_result.json').read_text())
    if mission.get('plan_id') != plan.plan_id:
        raise ValueError('odom log belongs to a different plan')
    rows = read_rows(trial_dir/'executed_odom_base.csv')
    if not rows:
        raise ValueError('no executed odom samples; cannot draw the third trajectory')
    geometry = json.loads(plan.geometry_json)
    target = json.loads(plan.target_json)['pose']
    phases = json.loads(plan.phases_json)
    planned_base = [tuple(p) for phase in phases for p in phase['poses']]
    planned_fork = [fork_for_base(p, geometry) for p in planned_base]
    odom = [(float(row['x_m']), float(row['y_m'])) for row in rows]
    final_odom_yaw = float(rows[-1]['yaw_rad'])
    final_imu_yaw = float(rows[-1].get('imu_control_yaw_rad') or final_odom_yaw)
    final_control_yaw = float(rows[-1].get('control_yaw_rad') or final_imu_yaw)
    amcl_control = rows[-1].get('heading_source') == 'amcl_odom_ekf'
    odom_control = rows[-1].get('heading_source') == 'odom_ekf'
    heading_label = 'AMCL + odom EKF' if amcl_control else ('odom EKF' if odom_control else 'IMU')
    final_pose = (float(rows[-1]['x_m']), float(rows[-1]['y_m']), final_control_yaw)
    if (not all(math.isfinite(v) for p in odom for v in p)
            or not all(math.isfinite(v) for v in (final_odom_yaw, final_imu_yaw, final_control_yaw))):
        raise ValueError('executed odom contains nonfinite coordinates')

    # All three paths and all shapes use one rigid view transform. In this
    # frame the pallet is level and the initial robot carries the yaw error.
    planned_base_xy = [pallet_view(p, target) for p in planned_base]
    planned_fork_xy = [pallet_view(p, target) for p in planned_fork]
    odom_xy = [pallet_view(p, target) for p in odom]
    fig = Figure(figsize=(11, 5.2), facecolor='white')
    ax = fig.subplots()
    ax.set_facecolor('#f8fafc')
    deck = [pallet_view(p, target) for p in rectangle(target, (0., geometry['pallet_length'],
                              -geometry['pallet_width']/2, geometry['pallet_width']/2))]
    ax.add_patch(Polygon(deck, facecolor='#25a18e', edgecolor='#0f766e',
                         alpha=.4, linewidth=2, zorder=1))
    body, tines, legs = polygons(planned_base[0], target, geometry)
    for leg in legs:
        ax.add_patch(Polygon([pallet_view(p, target) for p in leg],
                             facecolor='#0f766e', alpha=.7, zorder=2))
    body_xy = [pallet_view(p, target) for p in body]
    ax.add_patch(Polygon(body_xy, facecolor='#64748b', edgecolor='#334155',
                         alpha=.25, linewidth=1.5, zorder=2))
    for tine in tines:
        ax.add_patch(Polygon([pallet_view(p, target) for p in tine],
                             facecolor='#64748b', edgecolor='#334155',
                             alpha=.45, zorder=2))
    final_body, final_tines, _ = polygons(final_pose, target, geometry)
    final_body_xy = [pallet_view(p, target) for p in final_body]
    ax.add_patch(Polygon(final_body_xy, facecolor='none', edgecolor='#b45309',
                         linestyle='--', linewidth=2, zorder=5,
                         label=f'Ước lượng cuối · odom XY + góc {heading_label}'))
    final_tine_xy = []
    for tine in final_tines:
        points = [pallet_view(p, target) for p in tine]
        final_tine_xy.extend(points)
        ax.add_patch(Polygon(points, facecolor='none', edgecolor='#b45309',
                             linestyle='--', linewidth=1.8, zorder=5))

    ax.plot([p[0] for p in planned_base_xy], [p[1] for p in planned_base_xy],
            color='#2563eb', linewidth=2.6, label='Tính toán · tâm xe (base_footprint)', zorder=3)
    ax.plot([p[0] for p in planned_fork_xy], [p[1] for p in planned_fork_xy],
            color='#e11d48', linewidth=2.6, label='Tính toán · tâm gốc càng', zorder=3)
    ax.plot([p[0] for p in odom_xy], [p[1] for p in odom_xy],
            color='#f59e0b', linewidth=2.2, linestyle='--',
            marker='o' if len(odom) == 1 else None,
            label=('AMCL + odom · tâm xe' if amcl_control else 'Odom ghi khi chạy · tâm xe'), zorder=4)
    if amcl_control:
        capture_correction = json.loads(plan.target_json)['localization']['correction_at_capture_xyyaw']
        raw_xy = [pallet_view(compose(capture_correction,
                  (float(row['raw_odom_x_m']),float(row['raw_odom_y_m']),0.)), target) for row in rows]
        ax.plot([p[0] for p in raw_xy],[p[1] for p in raw_xy],color='#64748b',linewidth=1.4,
                linestyle=':',label='Odom encoder + IMU · gắn mốc A',zorder=3)

    a, c, b = planned_base[0], phases[-1]['poses'][0], planned_base[-1]
    for name, pose in (('A', a), ('C', c), ('B', b)):
        display_pose = pallet_view(pose, target)
        ax.scatter(display_pose[0], display_pose[1], s=35, facecolor='#172554', zorder=6)
        ax.annotate(name, display_pose, xytext=(7, 7), textcoords='offset points',
                    fontsize=11, weight='bold', color='#172554')
    ax.scatter(odom_xy[-1][0], odom_xy[-1][1], s=55, facecolor='#d97706',
               edgecolor='white', linewidth=1, zorder=7)
    ax.annotate('Odom cuối', odom_xy[-1], xytext=(9, -17), textcoords='offset points',
                fontsize=9, color='#92400e')
    ax.annotate('Pallet', pallet_view(target, target), xytext=(8, 10), textcoords='offset points',
                fontsize=10, color='#065f46')
    a_xy = pallet_view(a, target)
    a_yaw = a[2]-target[2]
    ax.arrow(a_xy[0], a_xy[1], .12*math.cos(a_yaw), .12*math.sin(a_yaw),
             head_width=.025, head_length=.035, color='#475569', zorder=5,
             length_includes_head=True)

    all_xy = [*planned_base_xy, *planned_fork_xy, *odom_xy, *deck, *body_xy,
              *final_body_xy, *final_tine_xy]
    ax.set_xlim(min(p[0] for p in all_xy)-.18, max(p[0] for p in all_xy)+.18)
    ax.set_ylim(min(p[1] for p in all_xy)-.18, max(p[1] for p in all_xy)+.18)
    ax.set_aspect('equal', adjustable='box')
    ax.set_xlabel('X theo pallet (m)')
    configure_view(ax, view)
    ax.grid(alpha=.22)
    ax.legend(loc='best', framealpha=.95)
    status = mission.get('status', 'UNKNOWN')
    initial_yaw = math.degrees(a_yaw)
    final_error = errors(final_pose, target, geometry)
    final_yaw = math.degrees(final_error['yaw_error_rad'])
    control_turn = math.degrees(wrap(final_control_yaw-a[2]))
    odom_final_yaw = math.degrees(wrap(final_odom_yaw-target[2]))
    tip_clearance_cm = 100*(final_error['gap_m']-geometry['fork_length'])
    status_label = 'ĐÃ DỪNG TRƯỚC C' if status == 'C_DIAGNOSTIC_DONE' else status
    view_label = 'GÓC NHÌN LAPTOP' if view == 'laptop' else 'GÓC NHÌN ROS'
    ax.set_title(f'SƠ ĐỒ ƯỚC LƯỢNG A → C → B · {status_label} · {view_label}\n'
                 f'Pallet: RGB-D khóa tại A  |  Xe: odom X/Y + góc {heading_label}  |  '
                 'không đo lại pallet lúc dừng\n'
                 f'Góc camera tại A {initial_yaw:+.2f}° + quay {heading_label} {control_turn:+.2f}° '
                 f'= góc cuối ước lượng {final_yaw:+.2f}°\n'
                 f'Odom cuối {odom_final_yaw:+.2f}° · đầu càng ước tính còn '
                 f'{tip_clearance_cm:.1f} cm · {len(odom)} mẫu', fontsize=10)
    # Show the signed estimate explicitly. This is a heading vector, never a
    # physical measurement inferred from the operator's photograph.
    final_xy = odom_xy[-1]
    final_display_yaw = wrap(final_control_yaw-target[2])
    ax.annotate('', xy=(final_xy[0]+.30*math.cos(final_display_yaw),
                        final_xy[1]+.30*math.sin(final_display_yaw)),
                xytext=final_xy, arrowprops=dict(arrowstyle='->', color='#b45309', lw=2),
                zorder=8)
    observation_path = trial_dir.parent/'physical_observation.json'
    if observation_path.exists():
        observed = json.loads(observation_path.read_text()).get('display_vi')
        if observed:
            ax.text(.99, .02, observed, transform=ax.transAxes, ha='right', va='bottom',
                    fontsize=9, color='#991b1b',
                    bbox=dict(facecolor='white', edgecolor='#991b1b', alpha=.95))
    fig.tight_layout()
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=170)
    fig.savefig(output.with_suffix('.svg'))
    return {'status': status, 'image': str(output),
            'svg': str(output.with_suffix('.svg')),
            'odom_samples': len(odom), 'plan_id': plan.plan_id, 'view': view}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--plan', required=True, type=Path)
    parser.add_argument('--trial', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--view', choices=('laptop', 'ros'), default='laptop',
                        help='laptop reflects Y for the operator viewpoint; ROS data stays unchanged')
    args = parser.parse_args()
    result = render(args.plan, args.trial, args.output, args.view)
    print(json.dumps(result, indent=2, ensure_ascii=False))


if __name__ == '__main__':
    main()
