"""LiDAR collision checks with the existing full chassis/fork envelope."""
import math
from dataclasses import replace
import numpy as np
from pallet_staging_safety import FOOTPRINT


class LaserTransformPending(ValueError):
    """A fresh scan is waiting for its capture-time pose or sensor mount TF."""


def capture_laser_points(scan,pose,extrinsic):
    """Compose measured capture-time odom with laser-to-base extrinsics.

    The caller must obtain pose by interpolation (never extrapolation) and
    enforce the scan's original source/receipt deadlines.
    """
    q=extrinsic.rotation;t=extrinsic.translation
    values=(*pose,q.x,q.y,q.z,q.w,t.x,t.y,scan.angle_min,
            scan.angle_increment,scan.range_min,scan.range_max)
    if (not all(math.isfinite(v) for v in values)
            or abs(sum(v*v for v in (q.x,q.y,q.z,q.w))-1)>.02
            or abs(q.x)+abs(q.y)>.05):raise ValueError('TF/mặt phẳng laser không hợp lệ')
    if scan.angle_increment<=0 or not 0<scan.range_min<scan.range_max:
        raise ValueError('Laser không hợp lệ')
    c,s=math.cos(pose[2]),math.sin(pose[2])
    x=pose[0]+c*t.x-s*t.y;y=pose[1]+s*t.x+c*t.y
    angle=pose[2]+math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
    ranges=np.asarray(scan.ranges,float)
    valid=np.isfinite(ranges)&(ranges>=scan.range_min)&(ranges<=scan.range_max)
    if valid.sum()<2:raise ValueError('Laser thiếu điểm đo hợp lệ')
    angles=scan.angle_min+np.arange(len(ranges))*scan.angle_increment+angle
    return np.column_stack((x+ranges[valid]*np.cos(angles[valid]),
                            y+ranges[valid]*np.sin(angles[valid])))


def lookup_station_transform(buffer, pose_stamp_s, now_s):
    """Resolve the saved station at the same time as the current odometry pose.

    AMCL postdates map->odom by transform_tolerance. A latest-time lookup
    can therefore have a future header even when localization is healthy.
    Query the measured pose time, keeping the existing freshness bounds.
    """
    from rclpy.time import Time
    if not all(math.isfinite(v) for v in (pose_stamp_s,now_s)) or not -.05<=now_s-pose_stamp_s<=.20:
        raise ValueError('Odometry quá hạn khi chuyển điểm pickup từ map sang odom')
    tf=buffer.lookup_transform('odom','map',Time(seconds=pose_stamp_s))
    stamp=tf.header.stamp.sec+tf.header.stamp.nanosec*1e-9
    if not -.05<=now_s-stamp<=.5:
        raise ValueError('TF map → odom quá hạn tại thời điểm odometry')
    return tf


def lock_saved_station_plan(plan):
    """Reject a perception-derived endpoint that disagrees with the taught station."""
    values=(*plan.goal,*plan.nominal_goal)
    if not all(math.isfinite(v) for v in values):
        raise ValueError('Nonfinite taught station or insertion endpoint')
    distance=math.hypot(plan.goal[0]-plan.nominal_goal[0],plan.goal[1]-plan.nominal_goal[1])
    heading=abs(math.atan2(math.sin(plan.goal[2]-plan.nominal_goal[2]),
                           math.cos(plan.goal[2]-plan.nominal_goal[2])))
    if distance>.005 or heading>math.radians(1):
        raise ValueError(f'Điểm đưa càng tính từ ảnh khác điểm pickup đã dạy '
                         f'{distance*100:.2f} cm / {math.degrees(heading):.2f}°. '
                         'Dừng; giữ nguyên điểm đã lưu, chưa đưa càng/nâng.')
    return replace(plan,station_locked=True)


def footprint_hit(points,pose,padding=.03):
    points=np.asarray(points,float)
    if points.ndim!=2 or points.shape[1]!=2 or not np.isfinite(points).all():
        raise ValueError('Need finite planar laser points')
    c,s=math.cos(pose[2]),math.sin(pose[2])
    p=(points-np.asarray(pose[:2]))@np.array([[c,-s],[s,c]])
    p=p[(p[:,0]>=-.33-padding)&(p[:,0]<=.78+padding)&(abs(p[:,1])<=.25+padding)]
    # No point in the padded bounding box can intersect its enclosed polygon.
    # Avoid six empty edge projections for every pose in each braking sweep.
    if not len(p):return None
    inside=np.zeros(len(p),bool)
    near=np.zeros(len(p),bool)
    for a,b in zip(FOOTPRINT,FOOTPRINT[1:]+FOOTPRINT[:1]):
        a,b=np.asarray(a),np.asarray(b);d=b-a
        t=np.clip((p-a)@d/(d@d),0.,1.)
        near|=np.linalg.norm(p-a-t[:,None]*d,axis=1)<=padding
        if a[1]!=b[1]:
            inside^=((a[1]>p[:,1])!=(b[1]>p[:,1]))&(p[:,0]<(b[0]-a[0])*(p[:,1]-a[1])/(b[1]-a[1])+a[0])
    hits=inside|near
    return tuple(p[np.flatnonzero(hits)[0]]) if hits.any() else None
