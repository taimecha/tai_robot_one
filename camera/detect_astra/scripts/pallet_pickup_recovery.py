"""Validate a stopped pre-entry approach before an explicit operator resume."""
import math
from pallet_insertion_core import Opening, opening_coordinates, wrap


def relative_opening(opening, pose):
    x,y=opening_coordinates(Opening(pose[0],pose[1],pose[2],opening.width_m),
                            (opening.x,opening.y))
    return dict(x=x,y=y,yaw=wrap(opening.yaw-pose[2]),width_m=opening.width_m)


def recovery_issue(expected, measured, pose, height, expected_height):
    """Require a new RGB-D verification before resuming outside the pallet."""
    values=(*pose,height,expected_height,*expected.values())
    if not all(isinstance(v,(int,float)) and math.isfinite(v) for v in values):
        return 'Checkpoint/phép đo phục hồi không hợp lệ'
    current=relative_opening(measured,pose)
    if (math.hypot(current['x']-expected['x'],current['y']-expected['y'])>.02
            or abs(wrap(current['yaw']-expected['yaw']))>math.radians(2)
            or abs(current['width_m']-expected['width_m'])>.02):
        return 'Pallet/xe không khớp vị trí đã dừng trong 2 cm/2 độ; không tự tiếp tục'
    if abs(height-expected_height)>.003:
        return 'Cao độ càng đã thay đổi sau khi dừng'
    # This recovery requires the pallet to remain fully visible and the forks
    # to remain outside. It cannot reopen a stopped blind insertion near entry.
    root_gap=current['x']-.3655*math.cos(current['yaw'])
    if root_gap<.60:return 'Phục hồi có RGB-D chỉ hỗ trợ khi gốc càng còn cách pallet >=0,60 m'
    if abs(current['y'])>=.02 or abs(current['yaw'])>=math.radians(2):
        return 'Chưa đạt ngang/góc để tiếp tục đưa càng'
    return ''
