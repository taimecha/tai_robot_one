"""Conservative filled-footprint and swept-route validation on an odom costmap."""
import math
import numpy as np
from pallet_alignment_core import wrap
from pallet_staging_core import plane_clearance,MAX_STAGING_TURN_RAD
from pallet_laser_timing import VEHICLE_FOOTPRINT

# Same physical envelope as navigation, plus 3 cm padding and raster reserve.
FOOTPRINT = VEHICLE_FOOTPRINT
ROUTE_HEADING_RESERVE_RAD = .08
STAGING_STOP_POLYGON = ((.96,.28),(.96,-.28),(.56,-.43),(-.51,-.43),(-.51,.43),(.56,.43))


def segment_distance(px,py,a,b):
    dx,dy=b[0]-a[0],b[1]-a[1]
    t=max(0.,min(1.,((px-a[0])*dx+(py-a[1])*dy)/(dx*dx+dy*dy)))
    return math.hypot(px-a[0]-t*dx,py-a[1]-t*dy)


def inside_or_near(x,y,vertices,padding):
    inside=False
    for a,b in zip(vertices,vertices[1:]+vertices[:1]):
        if segment_distance(x,y,a,b)<=padding:
            return True
        if (a[1]>y)!=(b[1]>y) and x<(b[0]-a[0])*(y-a[1])/(b[1]-a[1])+a[0]:
            inside=not inside
    return inside


class FootprintGrid:
    def __init__(self,msg):
        m=msg.info
        q=m.origin.orientation
        if (msg.header.frame_id!='odom' or m.resolution<=0 or m.width<=0 or m.height<=0
                or len(msg.data)!=m.width*m.height
                or any(abs(v)>.0001 for v in (q.x,q.y,q.z)) or abs(q.w-1)>.001):
            raise ValueError('Costmap must be a finite axis-aligned odom grid')
        self.res=m.resolution
        self.width,self.height=m.width,m.height
        self.ox,self.oy=m.origin.position.x,m.origin.position.y
        if not all(math.isfinite(v) for v in (self.res,self.ox,self.oy)):
            raise ValueError('Nonfinite costmap geometry')
        self.data=msg.data
        # ROS OccupancyGrid uses a signed-byte array: retain its live buffer.
        # Synthetic fixtures may use a mutable list, converted on each check.
        self.costs=(np.frombuffer(self.data,dtype=np.int8).reshape(self.height,self.width)
                    if hasattr(self.data,'buffer_info') else None)

    def clear(self,pose,footprint=FOOTPRINT):
        self.last_issue=None
        x,y,yaw=pose
        c,s=math.cos(yaw),math.sin(yaw)
        poly=tuple((x+c*a-s*b,y+s*a+c*b) for a,b in footprint)
        pad=.03+self.res/math.sqrt(2)
        lo_x=math.floor((min(p[0] for p in poly)-pad-self.ox)/self.res)
        hi_x=math.floor((max(p[0] for p in poly)+pad-self.ox)/self.res)
        lo_y=math.floor((min(p[1] for p in poly)-pad-self.oy)/self.res)
        hi_y=math.floor((max(p[1] for p in poly)+pad-self.oy)/self.res)
        if lo_x<0 or lo_y<0 or hi_x>=self.width or hi_y>=self.height:
            return False
        costs=(self.costs if self.costs is not None else
               np.asarray(self.data,dtype=np.int8).reshape(self.height,self.width))
        region=costs[lo_y:hi_y+1,lo_x:hi_x+1]
        # Enumerate only lethal/unknown cells in C. Iterating every free cell
        # for every swept pose can starve laser/odom callbacks on the Pi.
        for row,column in np.argwhere((region<0)|(region>=100)):
            j,i=int(row)+lo_y,int(column)+lo_x
            cost=int(costs[j,i])
            # OccupancyGrid uses 100 for a physical lethal obstacle and
            # 99 for the INSCRIBED_INFLATED_OBSTACLE cost (253 in Nav2).
            # Inflation already accounts for the robot's inscribed
            # radius. Applying it to every cell of the filled footprint
            # would add that radius a second time. Keep the full physical
            # envelope, padding and raster reserve against lethal/unknown
            # cells instead; inflation remains in Nav2's planner/controller.
            if (cost<0 or cost>=100) and inside_or_near(
                    self.ox+(i+.5)*self.res,self.oy+(j+.5)*self.res,poly,pad):
                self.last_issue=(self.ox+(i+.5)*self.res,self.oy+(j+.5)*self.res,cost)
                return False
        return True

    def approach_clear(self,pose):
        """Plan routes that also fit the live laser stop polygon.

        A body-clear route inside Collision Monitor's stop zone cannot be
        followed. Reject it during planning instead of driving there first.
        """
        return self.clear(pose,STAGING_STOP_POLYGON)


def swept_poses(a,b):
    angle=wrap(b[2]-a[2])
    steps=max(1,math.ceil(math.hypot(b[0]-a[0],b[1]-a[1])/.02),
              math.ceil(abs(angle)/math.radians(2)))
    for i in range(steps+1):
        u=i/steps
        yield (a[0]+u*(b[0]-a[0]),a[1]+u*(b[1]-a[1]),wrap(a[2]+u*angle))


def route_issue(poses,grid,target,config):
    if not poses or len(poses)>5000:
        return 'empty or oversized path'
    for a,b in zip(poses,poses[1:]):
        if math.hypot(a[0]-b[0],a[1]-b[1])>.25:
            return 'path discontinuity: '+str((a,b))
    previous=poses[0]
    for pose in poses:
        for p in swept_poses(previous,pose):
            # Worst fork-root position over ALL yaw angles also protects the
            # final precision turn, independently of the planned orientation.
            worst=plane_clearance(target,(p[0],p[1],target[2]),config.fork_root_m)
            if worst<config.standoff_m+.02:
                return 'plane clearance: '+str((p,worst))
            if not grid.clear(p):
                return 'footprint: '+str(p)+' cell='+str(getattr(grid,'last_issue','outside'))
        previous=pose
    return ''


def route_clear(poses,grid,target,config):
    return not route_issue(poses,grid,target,config)


def approach_route_issue(poses,grid,target,config):
    """Validate forward approaches; precision RGB-D limits apply after arrival.

    Navigation uses the frozen pallet in odom and a 12-degree nominal turn
    budget. Tracking reserve is still collision-checked around that route.
    """
    for pose in poses:
        if abs(wrap(pose[2]-target[2])) >= math.pi/2-1e-6:
            return 'approach heading is sideways or backward: '+str(pose)
        if max(abs(wrap(pose[2]-poses[0][2])),abs(wrap(pose[2]-target[2]))) > MAX_STAGING_TURN_RAD+1e-9:
            return 'approach turn exceeds small-turn limit (12 degrees): '+str(pose)
    issue=route_issue(poses,grid,target,config)
    if issue:return issue
    # Approve tracking variation geometrically, not by widening a numerical
    # gate alone. The same full filled footprint, unknown cells and clearance
    # reserve are checked throughout the additional yaw sweep along the route.
    previous=poses[0]
    for pose in poses:
        for p in swept_poses(previous,pose):
            a=(p[0],p[1],p[2]-ROUTE_HEADING_RESERVE_RAD)
            b=(p[0],p[1],p[2]+ROUTE_HEADING_RESERVE_RAD)
            for q in swept_poses(a,b):
                if abs(wrap(q[2]-target[2]))>=math.pi/2-1e-6:
                    return 'reserved approach heading is sideways or backward: '+str(q)
                if not getattr(grid,'approach_clear',grid.clear)(q):
                    return 'footprint: reserved tracking yaw '+str(q)+' cell='+str(grid.last_issue)
        previous=pose
    return ''


def route_heading_bounds(poses,target_yaw):
    """Runtime heading gate follows the exact collision-checked route."""
    if not poses:raise ValueError('Missing checked route')
    angles=[wrap(p[2]-target_yaw) for p in poses]
    return (max(-math.pi/2,min(angles)-ROUTE_HEADING_RESERVE_RAD),
            min(math.pi/2,max(angles)+ROUTE_HEADING_RESERVE_RAD))


def braking_poses(pose,v,w,config):
    """Stopping sweep: 200 ms reaction, then conservative configured braking.

    Robot controller limits are 0.50 m/s² and 1.50 rad/s²; the staging
    limits (0.08 / 0.40 by default) deliberately assume slower braking.
    """
    if not all(math.isfinite(a) for a in (*pose,v,w)):
        raise ValueError('Nonfinite braking state')
    x,y,yaw=pose
    yield (x,y,yaw)
    dt=.05
    for _ in range(4):
        mid=yaw+w*dt/2
        x+=v*math.cos(mid)*dt;y+=v*math.sin(mid)*dt;yaw=wrap(yaw+w*dt)
        yield (x,y,yaw)
    while abs(v)>1e-9 or abs(w)>1e-9:
        next_v=math.copysign(max(0.,abs(v)-config.linear_accel_mps2*dt),v)
        next_w=math.copysign(max(0.,abs(w)-config.angular_accel_rps2*dt),w)
        mid_w=(w+next_w)/2;mid_v=(v+next_v)/2
        mid=yaw+mid_w*dt/2
        x+=mid_v*math.cos(mid)*dt;y+=mid_v*math.sin(mid)*dt;yaw=wrap(yaw+mid_w*dt)
        v,w=next_v,next_w
        yield (x,y,yaw)
