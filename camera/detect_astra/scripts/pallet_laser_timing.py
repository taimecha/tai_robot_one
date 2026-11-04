"""Acquisition-aware laser deadline and conservative Collision Monitor envelope.

At 7 Hz a full sweep is ~140 ms old on arrival; just before the next sweep
its first ray is already ~280 ms old. Allow 400 ms, including delivery jitter.
The unchanged stop polygon is >=180 mm outside the vehicle/forks. Reserve
20 mm of that and bound translation + rotation through age, reaction and
conservative braking. Odom and pallet-plane checks keep their own deadlines.
"""
import math

LASER_TIMEOUT_S = .40
CONTROL_REACTION_S = .10
ENVELOPE_RADIUS_M = .80
STOP_POLYGON_PADDING_M = .18
STOP_RESERVE_M = .02
MEASURED_STOP_LIMIT_M = STOP_POLYGON_PADDING_M
# Real base logs show 10–15% linear tracking overshoot. Budget commands for
# 20% overshoot on BOTH axes, leaving the 20 mm reserve in addition. This is
# an allowance, not a physical guarantee: measured motion is still checked.
COMMAND_TRACKING_FACTOR = 1.20
COMMAND_ANGULAR_TRACKING_FACTOR = 2.0
VEHICLE_FOOTPRINT = ((.78,.10),(.78,-.10),(.38,-.25),(-.33,-.25),(-.33,.25),(.38,.25))


def validate_stop_polygon(points):
    if (len(points)<3 or any(len(p)!=2 or not all(math.isfinite(x) for x in p) for p in points)
            or max(math.hypot(*p) for p in VEHICLE_FOOTPRINT)>ENVELOPE_RADIUS_M):
        raise ValueError('Invalid laser stop geometry')
    area=sum(a[0]*b[1]-b[0]*a[1] for a,b in zip(points,points[1:]+points[:1]))
    if abs(area)<1e-8:raise ValueError('Empty stop polygon')
    sign=math.copysign(1.,area)
    for a,b in zip(points,points[1:]+points[:1]):
        dx,dy=b[0]-a[0],b[1]-a[1];length=math.hypot(dx,dy)
        if length<1e-8:raise ValueError('Degenerate stop polygon')
        distance=lambda p:sign*(dx*(p[1]-a[1])-dy*(p[0]-a[0]))/length
        if (min(distance(p) for p in points)<-1e-8
                or min(distance(p) for p in VEHICLE_FOOTPRINT)<STOP_POLYGON_PADDING_M-1e-8):
            raise ValueError('Stop polygon does not contain the required braking reserve')


def stop_travel(v, w, linear_decel=.08, angular_decel=.40):
    if (not all(math.isfinite(x) for x in (v,w,linear_decel,angular_decel))
            or min(linear_decel,angular_decel)<=0):
        raise ValueError('Invalid stopping envelope')
    reaction = LASER_TIMEOUT_S+CONTROL_REACTION_S
    return (abs(v)*reaction+v*v/(2*linear_decel)
            +ENVELOPE_RADIUS_M*(abs(w)*reaction+w*w/(2*angular_decel)))


def safe_twist_scale(v, w, linear_decel=.08, angular_decel=.40):
    stop_travel(v,w,linear_decel,angular_decel)
    v,w=v*COMMAND_TRACKING_FACTOR,w*COMMAND_ANGULAR_TRACKING_FACTOR
    budget = STOP_POLYGON_PADDING_M-STOP_RESERVE_M
    a = v*v/(2*linear_decel)+ENVELOPE_RADIUS_M*w*w/(2*angular_decel)
    b = (abs(v)+ENVELOPE_RADIUS_M*abs(w))*(LASER_TIMEOUT_S+CONTROL_REACTION_S)
    if a+b<=budget:
        return 1.
    # Stable positive root; scaling both axes preserves the checked curvature.
    return 2*budget/(b+math.sqrt(b*b+4*a*budget))


def measured_transition_scale(v,w,measured_v,measured_w,linear_decel=.08,angular_decel=.40):
    """Do not assume the old axis has already slowed when the other starts.

    Worst absolute speeds cover a transition between measured motion and
    the requested command, with observed tracking overshoot allowances.
    The hard measured-motion guard remains independent of this governor.
    """
    budget=STOP_POLYGON_PADDING_M-STOP_RESERVE_M
    stop_travel(v,w,linear_decel,angular_decel)
    if stop_travel(measured_v,measured_w,linear_decel,angular_decel)>budget:
        return 0.
    def fits(scale):
        return stop_travel(max(abs(measured_v),abs(v)*COMMAND_TRACKING_FACTOR*scale),
            max(abs(measured_w),abs(w)*COMMAND_ANGULAR_TRACKING_FACTOR*scale),
            linear_decel,angular_decel)<=budget
    if fits(1.):return 1.
    low,high=0.,1.
    for _ in range(24):
        mid=(low+high)/2
        if fits(mid):low=mid
        else:high=mid
    return low


def measured_velocity_issue(v,w,linear_decel=.08,angular_decel=.40):
    distance=stop_travel(v,w,linear_decel,angular_decel)
    if distance>MEASURED_STOP_LIMIT_M:
        return (f'Vận tốc đo vượt vùng phanh bảo vệ laser: v={v:.3f} m/s, '
                f'w={w:.3f} rad/s, quãng phanh={distance*100:.2f} cm > '
                f'{MEASURED_STOP_LIMIT_M*100:.2f} cm; đã dừng')
    return ''
