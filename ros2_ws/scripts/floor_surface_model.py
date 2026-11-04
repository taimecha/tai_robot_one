"""Bounded residual ground estimate, qualified only during stationary views.

No map coordinates, pallet labels or fixed image rectangles are used. A broad
near-ground surface must span both sides and near/far ranges. Raised planes,
small patches and poorly supported fits cannot change the reference. Loss of
support freezes the last qualified model; it never fabricates free rays.
"""
from collections import deque
import numpy as np


ANCHORS = np.array([[1.3,-.6,1.], [1.3,.6,1.], [2.3,-.6,1.], [2.3,.6,1.]])


def estimate_floor_residual(xyz):
    points = np.asarray(xyz).reshape(-1,3)
    valid = (np.isfinite(points).all(1) & (points[:,0]>=1.1)
             & (points[:,0]<=2.8) & (abs(points[:,1])<=1.05)
             & (points[:,2]>=-.08) & (points[:,2]<=.08))
    points = points[valid]
    if len(points)<200:
        return None
    # Give equal weight to occupied XY cells, rather than the image's dense
    # near rows or a vertical face with many returns at the same XY position.
    _,indices = np.unique(np.floor(points[:,:2]/.06),axis=0,return_index=True)
    points = points[indices]
    if len(points)<100:
        return None
    design = np.column_stack((points[:,:2],np.ones(len(points))))
    rng = np.random.default_rng(7421)
    best = None
    for _ in range(64):
        selected = rng.choice(len(points),3,replace=False)
        try:
            plane = np.linalg.solve(design[selected],points[selected,2])
        except np.linalg.LinAlgError:
            continue
        if not admissible(plane):
            continue
        inliers = abs(points[:,2]-design@plane)<=.012
        count = int(inliers.sum())
        if best is None or count>best[0]:
            best = (count,inliers)
    if best is None or best[0]<max(90,.75*len(points)):
        return None
    inliers = best[1]
    for _ in range(2):
        plane = np.linalg.lstsq(design[inliers],points[inliers,2],rcond=None)[0]
        inliers = abs(points[:,2]-design@plane)<=.012
    support = points[inliers]
    if (not admissible(plane) or len(support)<max(90,.75*len(points))
            or np.ptp(support[:,0])<.75 or np.ptp(support[:,1])<1.0
            or any(np.count_nonzero(mask)<12 for mask in
                   (support[:,1]<-.25,support[:,1]>.25,
                    support[:,0]<1.7,support[:,0]>2.0))):
        return None
    error = support[:,2]-np.column_stack((support[:,:2],np.ones(len(support))))@plane
    if np.sqrt(np.mean(error**2))>.008:
        return None
    return plane


def admissible(plane):
    return (np.isfinite(plane).all() and np.linalg.norm(plane[:2])<=.10
            # Absolute height anchor: an elevated flat pallet cannot become
            # the floor even if it occupies most of the image.
            and abs(plane@np.array([1.7,0.,1.]))<=.03
            and np.max(abs(ANCHORS@plane))<=.10)


class FloorSurfaceModel:
    def __init__(self):
        self.plane = None
        self.samples = deque(maxlen=12)
        self.last_attempt = -float('inf')
        self.reason = 'Chờ sàn rộng và odometry đứng yên'

    def observe(self,xyz,now,stationary):
        if not stationary:
            self.samples.clear()
            self.reason = 'Giữ phép bù; xe đang chạy hoặc odometry chưa mới'
            return
        if now-self.last_attempt<.5:
            return
        self.last_attempt = now
        candidate = estimate_floor_residual(xyz)
        if candidate is None:
            self.samples.clear()
            self.reason = 'Giữ phép bù; không đủ mặt sàn được kiểm chứng'
            return
        if self.samples and now-self.samples[-1][0]>.8:
            self.samples.clear()
        self.samples.append((now,candidate))
        if len(self.samples)<8 or now-self.samples[0][0]<3.5:
            self.reason = 'Đang xác nhận mặt sàn liên tục'
            return
        planes = np.stack([p for _,p in self.samples])
        if np.max(np.ptp(planes@ANCHORS.T,axis=0))>.01:
            self.reason = 'Chưa cập nhật: mặt sàn còn biến động'
            return
        self.plane = np.median(planes,axis=0)
        self.reason = 'Đã xác nhận mặt sàn; chỉ cập nhật khi xe đứng yên'

    def apply(self,xyz):
        if self.plane is None:
            return xyz
        result = xyz.copy()
        a,b,c = self.plane
        result[...,2] = (xyz[...,2]-a*xyz[...,0]-b*xyz[...,1]-c)/np.sqrt(1+a*a+b*b)
        return result

    def status(self):
        return dict(qualified=self.plane is not None,reason=self.reason,
                    residual_plane=None if self.plane is None else self.plane.tolist(),
                    qualifying_samples=len(self.samples))
