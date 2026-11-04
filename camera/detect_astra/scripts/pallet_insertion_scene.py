"""Experimental offline 3-D registration; not the supervised pickup interlock.

Real-snapshot ground fitting remains under investigation. The operator-started
pickup mode uses the documented frozen trajectory/odom/laser chain instead.
"""
import math

import numpy as np
from scipy.spatial import cKDTree

from pallet_target_geometry import _leg_components


# Conservative existing navigation envelope, retained for OTHER obstacles.
LEGACY_FOOTPRINT = ((.78,.10),(.78,-.10),(.38,-.25),(-.33,-.25),(-.33,.25),(.38,.25))
# Actual measured narrow fork projection plus conservative body/wheel bounds.
# This is used only together with independent full 3-D body/fork checks.
INSERTION_FOOTPRINT = ((.7555,.0775),(.7555,-.0775),(.3655,-.0775),
    (.3655,-.13),(.3005,-.13),(.3005,-.145),(.265,-.145),(.265,-.25),
    (-.33,-.25),(-.33,.25),(.265,.25),(.265,.145),(.3005,.145),
    (.3005,.13),(.3655,.13),(.3655,.0775))


def transform_xy(points, pose):
    result = np.asarray(points, float).copy()
    c, s = math.cos(pose[2]), math.sin(pose[2])
    result[:, :2] = result[:, :2]@np.array([[c,s],[-s,c]])+pose[:2]
    return result


def voxel_points(points, size=.01):
    points = np.asarray(points, float)
    points = points[np.isfinite(points).all(axis=1)]
    if not len(points):
        return np.empty((0,3))
    _, index = np.unique(np.floor(points/size).astype(np.int64), axis=0, return_index=True)
    return points[index]


def ground_model(points):
    """Recognize the observed floor, including its small residual height bias.

    Fit only near the previously calibrated zero plane. Never classify the
    semantic pallet as ground. The model needs broad support and is bounded
    to 2-cm residual slope/offset; a raised patch remains an obstacle.
    """
    points=np.asarray(points,float)
    candidates=points[(points[:,2]>-.025)&(points[:,2]<.025)]
    if len(candidates)<300 or np.ptp(candidates[:,0])<.5 or np.ptp(candidates[:,1])<.7:
        raise ValueError('Insufficient broad observed floor for wheel contact checks')
    design=np.column_stack((candidates[:,:2],np.ones(len(candidates))))
    keep=np.ones(len(candidates),bool)
    for _ in range(4):
        coef=np.linalg.lstsq(design[keep],candidates[keep,2],rcond=None)[0]
        residual=candidates[:,2]-design@coef
        median=np.median(residual[keep])
        sigma=1.4826*np.median(abs(residual[keep]-median))
        keep=abs(residual-median)<=max(.006,min(.015,3*sigma))
        if keep.sum()<300:raise ValueError('Floor fit has insufficient inlier support')
    if np.linalg.norm(coef[:2])>.02 or abs(coef[2])>.025:
        raise ValueError('Observed floor disagrees with fixed floor calibration')
    tolerance=max(.008,min(.012,3*sigma))
    return coef,tolerance


def ground_points(points,coef,tolerance):
    points=np.asarray(points,float)
    residual=points[:,2]-(points[:,:2]@coef[:2]+coef[2])
    return (abs(residual)<=tolerance)&(points[:,2]<.03)


class RegisteredScene:
    """A semantic pallet receives its own rigid XY correction, not the room.

    The correction registers two measured front-leg surfaces to the calibrated
    opening pose. It is bounded and stored explicitly for replay/review.
    New objects in/near the opening are still tested against the fork volumes.
    """
    def __init__(self, cloud, mask, bbox, opening, cloud_pose):
        cloud = np.asarray(cloud, float)
        if cloud.ndim != 3 or cloud.shape[2] != 3:
            raise ValueError('Need an organized registered XYZ cloud')
        mask = np.asarray(mask)
        if (mask.ndim != 2 or not all(cloud.shape[:2])
                or not all(mask.shape) or not np.isfinite(cloud_pose).all()):
            raise ValueError('Mask/cloud size mismatch')
        sy, sx = math.ceil(mask.shape[0]/cloud.shape[0]), math.ceil(mask.shape[1]/cloud.shape[1])
        if sx != sy or mask[::sy,::sx].shape != cloud.shape[:2]:
            raise ValueError('Unknown organized cloud pixel stride')
        selected = mask[::sy,::sx].astype(bool)
        world = transform_xy(cloud.reshape(-1,3), cloud_pose).reshape(cloud.shape)
        centres = []
        for component in _leg_components(mask, bbox):
            support = np.zeros(mask.shape, bool)
            support[component['v'],component['u']] = True
            points = world[support[::sy,::sx]]
            points = points[np.isfinite(points).all(axis=1)]
            if len(points) < 8:
                raise ValueError('Not enough depth samples on both semantic front legs')
            centres.append(np.median(points,axis=0))
        tangent = centres[0][:2]-centres[1][:2]
        if not .15 < np.linalg.norm(tangent) < .5 or tangent[1]*math.cos(cloud_pose[2])-tangent[0]*math.sin(cloud_pose[2]) < 0:
            raise ValueError('Invalid front leg separation/order')
        raw_yaw = math.atan2(-tangent[0],tangent[1])
        angle = math.atan2(math.sin(opening.yaw-raw_yaw),math.cos(opening.yaw-raw_yaw))
        if abs(angle) > math.radians(8):
            raise ValueError('Pallet cloud/target yaw disagreement exceeds calibration bound')
        c, s = math.cos(angle), math.sin(angle)
        self.rotation = np.array([[c,-s],[s,c]])
        raw_centre = (centres[0][:2]+centres[1][:2])/2
        self.translation = np.array([opening.x,opening.y])-self.rotation@raw_centre
        corrected_centre = self.rotation@raw_centre+self.translation
        if np.linalg.norm(corrected_centre-raw_centre) > .10:
            raise ValueError('Pallet cloud/target translation exceeds calibration bound')
        pallet = world[selected]
        pallet = pallet[np.isfinite(pallet).all(axis=1)]
        if len(pallet) < 100:
            raise ValueError('Insufficient semantic pallet depth coverage')
        self.raw_pallet = voxel_points(pallet)
        self.tree = cKDTree(self.raw_pallet)
        self.opening = opening
        self.metadata = dict(rotation_rad=angle, translation=self.translation.tolist(),
                             raw_front_centre=raw_centre.tolist(), raw_pallet_points=len(self.raw_pallet))
        self.pallet = self.correct(self.raw_pallet)
        flat = world.reshape(-1,3)
        other = flat[~selected.ravel()]
        other = other[np.isfinite(other).all(axis=1)]
        self.floor,self.floor_tolerance=ground_model(other)
        ground=ground_points(other,self.floor,self.floor_tolerance)
        self.metadata.update(floor_coefficients=self.floor.tolist(),
            floor_inlier_tolerance_m=self.floor_tolerance,ground_points=int(ground.sum()))
        self.other = voxel_points(other[~ground])

    def correct(self, points):
        points = np.asarray(points, float).copy()
        points[:, :2] = points[:, :2]@self.rotation.T+self.translation
        return points

    def classify(self, points):
        points = np.asarray(points, float)
        if not np.isfinite(points).all():
            raise ValueError('Nonfinite live points')
        distance, _ = self.tree.query(points, distance_upper_bound=.025)
        target = np.isfinite(distance)
        corrected = points.copy()
        corrected[target] = self.correct(points[target])
        return corrected, target

    def ground(self,points):
        return ground_points(points,self.floor,self.floor_tolerance)

    def static_points(self):
        return np.concatenate((self.pallet,self.other))


def choose_entry_height(points, plan_start, goal, config):
    """Choose an observed-free fork slab inside the operator-measured hole.

    Absence of points alone does not verify a new pallet cavity. This module
    is used only after station/cavity verification and semantic acquisition.
    """
    points = np.asarray(points, float)
    if (points.ndim != 2 or points.shape[1] != 3 or not len(points)
            or not np.isfinite(points).all()):
        raise ValueError('Need finite observed scene before choosing entry height')
    yaw = goal[2]
    p = transform_xy(points,(-math.cos(yaw)*plan_start[0]-math.sin(yaw)*plan_start[1],
                             math.sin(yaw)*plan_start[0]-math.cos(yaw)*plan_start[1],-yaw))
    distance = math.hypot(goal[0]-plan_start[0],goal[1]-plan_start[1])
    span = (((p[:,1]>=.0245-.01)&(p[:,1]<=.0775+.01))
            |((p[:,1]>=-.0775-.01)&(p[:,1]<=-.0245+.01)))
    z = p[(p[:,0]>=.3655-.01)&(p[:,0]<=distance+.7555+.01)&span,2]
    bottom = config.fork_bottom_at_home_m
    top = bottom+config.fork_thickness_m
    lower = max(0.,config.opening_bottom_m+config.vertical_margin_m-bottom)
    upper = min(.05,config.opening_top_m-config.vertical_margin_m-top)
    choices = []
    for h in np.arange(lower,upper+.0001,.001):
        lo, hi = bottom+h, top+h
        clearance = float(np.min(np.maximum(lo-z,z-hi))) if len(z) else .10
        if clearance >= config.vertical_margin_m:
            choices.append((clearance,-abs(h),h))
    if not choices:
        raise ValueError('No collision-free fork height inside measured hole')
    return float(max(choices)[2])
