"""Stationary AMCL qualification; pure math, no velocity publishers."""
from collections import deque
import math
import numpy as np


def yaw(q):
    values=(q.x,q.y,q.z,q.w)
    if (not all(math.isfinite(v) for v in values)
            or abs(sum(v*v for v in values)-1)>.02 or abs(q.x)+abs(q.y)>.05):
        raise ValueError('Quaternion AMCL không hợp lệ')
    return math.atan2(2*q.w*q.z,1-2*q.z*q.z)


def particle_issue(pose,covariance,particles,strict=False):
    """Reject dispersed/multiple hypotheses, not just the best cluster covariance."""
    pts=np.asarray(particles,float)
    cov=np.asarray(covariance,float)
    if (len(pose)!=3 or not np.isfinite(pose).all() or cov.shape!=(36,)
            or not np.isfinite(cov).all() or pts.ndim!=2 or pts.shape[1]!=4
            or len(pts)<100 or not np.isfinite(pts).all() or np.any(pts[:,3]<0)):
        return 'Pose/covariance/particle cloud không hợp lệ'
    position_std=.10 if strict else .20
    angle_std=math.radians(5 if strict else 10)
    # AMCL subtracts second moments; a collapsed cloud can produce tiny
    # negative variances from floating-point cancellation.
    if (any(cov[i]<-1e-9 for i in (0,7,35)) or max(cov[0],cov[7])>position_std**2
            or cov[35]>angle_std**2):return 'AMCL chưa hội tụ covariance'
    # A small reported covariance alone can hide a second distant cluster.
    total=float(pts[:,3].sum())
    if total<=0:return 'Particle cloud không có trọng số'
    delta=np.arctan2(np.sin(pts[:,2]-pose[2]),np.cos(pts[:,2]-pose[2]))
    near=(np.hypot(pts[:,0]-pose[0],pts[:,1]-pose[1])<(.25 if strict else .35)) \
        & (np.abs(delta)<math.radians(10 if strict else 15))
    if float(pts[near,3].sum())/total<.90:
        return 'Particle cloud còn phân tán/nhiều giả thuyết vị trí'
    return ''


class PoseStability:
    def __init__(self,duration=5.,distance=.03,angle=math.radians(2),minimum_samples=8):
        self.duration=duration;self.distance=distance;self.angle=angle
        self.minimum_samples=minimum_samples;self.samples=deque();self.last_stamp=-math.inf

    def clear(self):self.samples.clear()

    def observe(self,stamp,wall,pose,qualified):
        if not qualified:self.clear();return False
        if not all(math.isfinite(v) for v in (stamp,wall,*pose)):
            self.clear();return False
        if stamp<=self.last_stamp:return False
        self.last_stamp=stamp
        if self.samples and (stamp-self.samples[-1][0]>1. or wall-self.samples[-1][1]>1.):self.clear()
        self.samples.append((stamp,wall,tuple(pose)))
        while len(self.samples)>1 and stamp-self.samples[1][0]>=self.duration:self.samples.popleft()
        first=self.samples[0]
        if (len(self.samples)<self.minimum_samples or stamp-first[0]<self.duration
                or wall-first[1]<self.duration):return False
        points=np.asarray([p[2] for p in self.samples])
        if np.hypot(np.ptp(points[:,0]),np.ptp(points[:,1]))>self.distance:return False
        angles=np.arctan2(np.sin(points[:,2]-pose[2]),np.cos(points[:,2]-pose[2]))
        return float(np.ptp(angles))<=self.angle


def navigation_stability_window():
    """Operator-requested shorter hold before map navigation.

    Bounds describe pose variation while stationary, not permitted goal error.
    Scan agreement, covariance, particle concentration and source-age checks
    remain independent admission requirements.
    """
    return PoseStability(duration=2.,distance=.05,angle=math.radians(3),minimum_samples=5)


def summary_observation(summary,requested_ros,now,strict=False):
    """Validate original source stamps; worker publication never refreshes them."""
    try:
        if summary.get('valid') is not True:
            return None,None,str(summary.get('reason','Chưa có kiểm tra AMCL hợp lệ'))
        pose=summary['pose'];stamp=summary['pose_stamp_s'];cloud=summary['cloud_stamp_s']
        published=summary['published_ros_s'];compute=summary['compute_s']
        if (len(pose)!=3 or not all(math.isfinite(v) for v in (*pose,stamp,cloud,published,compute))
                or compute<0 or summary['particle_count']<100):
            raise ValueError('Invalid summary')
        if (min(stamp,cloud)<requested_ros or not -.05<=now-stamp<=.8
                or not -.05<=now-cloud<=.8 or not -.05<=now-published<=.8
                or abs(stamp-cloud)>.5):
            return None,None,'Pose/particle AMCL quá hạn'
        issue=summary['strict_issue' if strict else 'coarse_issue']
        if not isinstance(issue,str):raise ValueError('Invalid issue')
        return stamp,tuple(pose),issue
    except (KeyError,TypeError,ValueError,OverflowError):
        return None,None,'Bản tóm tắt AMCL không hợp lệ'
