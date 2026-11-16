#!/usr/bin/env python3
"""Explain the fresh Nav2 start estimate before the shell asks to move."""
import argparse
import json
import math
from pathlib import Path


def preview_lines(report):
    if (report.get('status') != 'READY_FOR_NAVIGATION'
            or report.get('check_only') is not True
            or report.get('navigation_goal_attempted') is not False):
        raise ValueError('a fresh, no-goal navigation check is required')
    current, goal = report.get('current_pose_xyyaw'), report.get('goal_pose_xyyaw')
    for pose in (current, goal):
        if (not isinstance(pose, list) or len(pose) != 3
                or any(isinstance(v, bool) or not isinstance(v, (int, float))
                       or not math.isfinite(v) for v in pose)):
            raise ValueError('navigation check has no valid current/goal map pose')
    dx, dy = goal[0] - current[0], goal[1] - current[1]
    distance = math.hypot(dx, dy)
    yaw_error = math.remainder(goal[2] - current[2], 2 * math.pi)
    c, s = math.cos(current[2]), math.sin(current[2])
    forward, left = c * dx + s * dy, -s * dx + c * dy
    bearing = math.atan2(left, forward) if distance > 1e-9 else 0.
    lines = [
        f'Định vị hiện tại: X={current[0]:.3f} m, Y={current[1]:.3f} m, '
        f'góc={math.degrees(current[2]):.2f}°.',
        f'So với A đã lưu: cách {100 * distance:.2f} cm; '
        f'góc đích lệch {math.degrees(yaw_error):+.2f}°.',
        f'A theo hướng xe: {100 * abs(forward):.2f} cm '
        f'{"phía trước" if forward >= 0 else "phía sau"}, '
        f'{100 * abs(left):.2f} cm '
        f'{"sang trái" if left >= 0 else "sang phải"}; '
        f'phương tới A lệch {math.degrees(bearing):+.1f}°.',
    ]
    limits=report.get('arrival_limits') or {}
    region_xy=limits.get('position_m',.10)
    region_yaw=math.radians(limits.get('heading_deg',5.))
    if distance<region_xy-1e-12 and abs(yaw_error)<=region_yaw+1e-9:
        lines.append(f'Xe đã trong vùng nhận diện A dưới {region_xy*100:g} cm / '
                     f'{math.degrees(region_yaw):g}°; không chỉnh thêm, chuyển sang nhận diện.')
    elif distance < region_xy-1e-12:
        lines.append(f'Vị trí đã trong vùng A dưới {region_xy*100:g} cm; Nav2 chỉ căn lại hướng '
                     'tại chỗ về góc A, rồi kiểm tra lại trước khi nhận diện.')
    elif distance < .20 and abs(bearing) > math.radians(60):
        lines.append('Điểm A đang được ước lượng ở lệch bên/sau xe; Nav2 có thể phải '
                     'xoay để điều chỉnh vị trí dù sai số khoảng cách nhỏ.')
    lines.append('Nhận diện tự bắt đầu sau khi vị trí A được xác nhận. Tọa độ A đã lưu giữ nguyên.')
    return lines


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--report', type=Path, required=True)
    args = parser.parse_args()
    try:
        print('\n'.join(preview_lines(json.loads(args.report.read_text()))), flush=True)
    except (OSError, ValueError, TypeError) as exc:
        parser.exit(2, f'Chưa có kiểm tra trước khi chạy hợp lệ: {exc}\n')


if __name__ == '__main__':
    main()
