"""Track a YOLO-seeded pallet in new RGB frames without loading YOLO.

Sparse optical flow transports a semantic mask, with independent support and
appearance checks on BOTH front legs. Depth geometry is checked by the caller
on the current registered frame, never carried forward from the seed.
"""
import math

import cv2
import numpy as np

from pallet_target_geometry import _leg_components


class PalletRGBTracker:
    def __init__(self, allow_gap_recovery=False):
        self.allow_gap_recovery = allow_gap_recovery
        self.recovery_gap_s = 1.5 if allow_gap_recovery else .5
        self.reset()

    def reset(self):
        self.active = False
        self.stamp_ns = 0
        self.frames = 0
        self.reverified = False

    def seed(self, image, mask, bbox, stamp_ns):
        if mask.shape != image.shape[:2] or not mask.any():
            raise ValueError('seed RGB/mask shape mismatch or empty mask')
        self.seed_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        self.seed_mask = mask.astype(np.uint8)
        self.seed_bbox = np.asarray(bbox, np.float32)
        self.seed_legs = []
        for c in _leg_components(mask, bbox):
            leg = np.zeros(mask.shape, np.uint8)
            leg[c['v'], c['u']] = 1
            self.seed_legs.append(leg)
        self.transform = np.eye(3, dtype=np.float64)
        self.gray = self.seed_gray.copy()
        self.mask = self.seed_mask.copy()
        self.bbox = self.seed_bbox.copy()
        self.legs = [leg.copy() for leg in self.seed_legs]
        self._features()
        self.seed_points = self.points.copy()
        self.seed_labels = self.labels.copy()
        self.stamp_ns = stamp_ns
        self.frames = 0
        self.active = True
        return self.mask.astype(bool), self.bbox.copy()

    def _features(self):
        points, labels = [], []
        for index, leg in enumerate(self.legs):
            found = None
            # Smooth plastic legs often have useful corners at the junction
            # with the top bar, a few pixels outside the lower-leg mask.
            # Search at most four pixels around each leg; keep the existing
            # feature-count, two-leg flow and appearance gates unchanged.
            for size in (5,7,9):
                support = cv2.dilate(leg, np.ones((size,size), np.uint8))
                candidate = cv2.goodFeaturesToTrack(
                    self.gray,maxCorners=60,qualityLevel=.01,minDistance=3,
                    mask=support,blockSize=3)
                if candidate is not None and (found is None or len(candidate)>len(found)):
                    found = candidate
                if found is not None and len(found)>=8:
                    break
            if found is None or len(found) < 4:
                raise ValueError('front leg has insufficient RGB tracking features')
            points.append(found)
            labels.extend([index] * len(found))
        self.points = np.concatenate(points).astype(np.float32)
        self.labels = np.array(labels)

    def update(self, image, stamp_ns):
        try:
            return self._update(image, stamp_ns)
        except (ValueError, cv2.error) as exc:
            # A single bad image must invalidate this measurement without
            # permanently discarding a still-recent visual reference. Retry
            # against the same original mask; the last GOOD timestamp stays
            # unchanged. Ordinary flow expires at 500 ms; optional independent
            # two-leg verification has a separate bounded reference lifetime.
            retryable = isinstance(exc, ValueError) and str(exc).startswith((
                'lost RGB support', 'front legs do not share',
                'front-leg RGB appearance lost', 'RGB optical flow unavailable',
                'RGB reverse optical flow unavailable', 'RGB pallet transform unavailable',
                'RGB re-verification pending'))
            recent = self.active and 0 < (stamp_ns-self.stamp_ns)/1e9 <= self.recovery_gap_s
            if not retryable or not recent:
                self.reset()
            raise

    def _update(self, image, stamp_ns):
        if not self.active:
            raise ValueError('no YOLO seed; enable recognition to acquire a pallet')
        dt = (stamp_ns - self.stamp_ns) / 1e9
        if not 0 < dt <= self.recovery_gap_s:
            raise ValueError('RGB tracking gap exceeded 500 ms or timestamp went backwards '
                             f'(dt={dt:.3f} s)')
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        self.reverified = False
        if dt > .5:
            # Optical flow is NOT extrapolated across its 500-ms limit.
            # Independently verify two fresh template matches before any
            # current-depth geometry can be considered again.
            transform = self._reverify_rgb(gray)
            prior = cv2.transform(self.seed_points,self.transform[:2]).reshape(-1,2)
            after = cv2.transform(self.seed_points,transform[:2]).reshape(-1,2)
            labels = self.seed_labels.copy()
            inliers = np.ones(len(labels),bool)
            self.reverified = True
        else:
            try:
                transform, prior, after, labels, inliers = self._flow_transform(
                    self.seed_gray, gray, self.seed_points, self.seed_labels, self.transform)
            except ValueError as original_error:
                # Compensate brightness changes using the last verified leg ROIs.
                # Match the ORIGINAL reference again; never accumulate transforms
                # from a sequence of recent references or relax the flow gap limit.
                try:
                    expected = cv2.warpAffine(self.seed_gray, self.transform[:2],
                        (gray.shape[1], gray.shape[0]), flags=cv2.INTER_LINEAR)
                    support = (self.legs[0] | self.legs[1]).astype(bool)
                    offset = float(np.median(gray[support].astype(float)-expected[support]))
                    reference = np.clip(self.seed_gray.astype(float)+offset,0,255).astype(np.uint8)
                    transform, prior, after, labels, inliers = self._flow_transform(
                        reference, gray, self.seed_points, self.seed_labels, self.transform)
                except (ValueError, cv2.error):
                    raise original_error
        delta = transform @ np.linalg.inv(self.transform)
        scales = np.linalg.svd(delta[:2, :2], compute_uv=False)
        rotation = math.degrees(math.atan2(delta[1, 0], delta[0, 0]))
        if (np.linalg.det(transform[:2, :2]) <= 0 or scales.min() < .85 or scales.max() > 1.18
                or abs(rotation) > 8 or np.max(np.linalg.norm(after - prior, axis=1)) > 60):
            raise ValueError('RGB pallet motion outside tracking range')
        h, w = gray.shape
        warp = lambda src, flags: cv2.warpAffine(src, transform[:2], (w, h), flags=flags)
        mask = warp(self.seed_mask, cv2.INTER_NEAREST)
        legs = [warp(leg, cv2.INTER_NEAREST) for leg in self.seed_legs]
        expected = warp(self.seed_gray, cv2.INTER_LINEAR)
        corners = np.array([[self.seed_bbox[0], self.seed_bbox[1]],
                            [self.seed_bbox[2], self.seed_bbox[1]],
                            [self.seed_bbox[2], self.seed_bbox[3]],
                            [self.seed_bbox[0], self.seed_bbox[3]]], np.float32)
        corners = cv2.transform(corners[None], transform[:2])[0]
        bbox = np.r_[corners.min(axis=0), corners.max(axis=0)]
        if bbox[0] < 2 or bbox[1] < 2 or bbox[2] > w - 2 or bbox[3] > h - 2:
            raise ValueError('pallet leaving camera view; reacquisition required')
        scores = []
        for leg in legs:
            support = cv2.dilate(leg, np.ones((7, 7), np.uint8)).astype(bool)
            if support.sum() < 80:
                raise ValueError('front leg too small for RGB tracking')
            a, b = expected[support].astype(float), gray[support].astype(float)
            a -= a.mean()
            b -= b.mean()
            denom = np.linalg.norm(a) * np.linalg.norm(b)
            score = float(a @ b / denom) if denom > 1 else 0.0
            if score < .75:
                raise ValueError(f'front-leg RGB appearance lost (correlation {score:.2f})')
            scores.append(score)
        self.gray, self.mask, self.bbox, self.legs = gray, mask, bbox, legs
        self.transform = transform
        self.points, self.labels = after[inliers].reshape(-1, 1, 2), labels[inliers]
        self.frames += 1
        self.stamp_ns = stamp_ns
        self.scores = scores
        return mask.astype(bool), bbox.copy()

    def _reverify_rgb(self,gray):
        """Two independent high-NCC local matches, tied to the last reference."""
        height,width = gray.shape
        expected = cv2.warpAffine(self.seed_gray,self.transform[:2],(width,height))
        shifts = []
        for leg in self.legs:
            yy,xx = np.nonzero(leg)
            if len(xx)<80:
                raise ValueError('RGB re-verification pending: insufficient leg support')
            x1,x2 = max(0,int(xx.min())-4),min(width,int(xx.max())+5)
            y1,y2 = max(0,int(yy.min())-4),min(height,int(yy.max())+5)
            template = expected[y1:y2,x1:x2]
            if template.std()<5:
                raise ValueError('RGB re-verification pending: leg template lacks contrast')
            margin = min(24,max(8,int((xx.max()-xx.min()+1)*1.25)))
            sx,sy = max(0,x1-margin),max(0,y1-margin)
            ex,ey = min(width,x2+margin),min(height,y2+margin)
            scores = cv2.matchTemplate(gray[sy:ey,sx:ex],template,cv2.TM_CCOEFF_NORMED)
            _,score,_,point = cv2.minMaxLoc(scores)
            if not math.isfinite(score) or score<.90:
                raise ValueError('RGB re-verification pending: front-leg appearance not confirmed')
            shifts.append(np.array([sx+point[0]-x1,sy+point[1]-y1],float))
        if np.linalg.norm(shifts[0]-shifts[1])>2:
            raise ValueError('RGB re-verification pending: front legs disagree on motion')
        delta = np.eye(3)
        delta[:2,2] = np.mean(shifts,axis=0)
        transform = delta@self.transform
        # Confirm usable CURRENT features on each leg as well as appearance.
        for leg in self.seed_legs:
            current = cv2.warpAffine(leg,transform[:2],(width,height),flags=cv2.INTER_NEAREST)
            support = cv2.dilate(current,np.ones((9,9),np.uint8))
            found = cv2.goodFeaturesToTrack(gray,maxCorners=60,qualityLevel=.01,
                                          minDistance=3,mask=support,blockSize=3)
            if found is None or len(found)<4:
                raise ValueError('RGB re-verification pending: current leg features not confirmed')
        return transform

    def _flow_transform(self, reference, gray, points, labels, guess):
        flow_args = dict(winSize=(15, 15), maxLevel=3,
                         flags=cv2.OPTFLOW_USE_INITIAL_FLOW,
                         criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 25, .01))
        # Register each frame against the original seed, using the previous
        # transform only as an initial guess. Incrementally warping the last
        # mask can accumulate drift even when the robot is stationary.
        predicted = cv2.transform(points, guess[:2]).astype(np.float32)
        # LK needs the pallet neighbourhood, not pyramids of the entire scene
        # twice in each direction. Bound a shared ROI around original AND
        # predicted points. Its origin follows the target; global pixel
        # coordinates and all subsequent motion/appearance gates stay intact.
        locations = np.concatenate((points.reshape(-1,2),predicted.reshape(-1,2)))
        if not np.isfinite(locations).all():
            raise ValueError('RGB optical flow unavailable: nonfinite prediction')
        height,width = gray.shape
        padding = 64  # 15-pixel LK window at pyramid level 3, plus border.
        lower = np.floor((locations.min(axis=0)-padding)/8)*8
        upper = np.ceil((locations.max(axis=0)+padding)/8)*8
        x1,y1 = np.maximum(lower,[0,0]).astype(int)
        x2,y2 = np.minimum(upper,[width,height]).astype(int)
        if x2-x1 < 16 or y2-y1 < 16:
            raise ValueError('RGB optical flow unavailable: target outside image')
        origin = np.array([x1,y1],np.float32)
        local_points = points-origin
        local_prediction = predicted-origin
        reference_roi = np.ascontiguousarray(reference[y1:y2,x1:x2])
        gray_roi = np.ascontiguousarray(gray[y1:y2,x1:x2])
        forward, ok, error = cv2.calcOpticalFlowPyrLK(
            reference_roi, gray_roi, local_points, local_prediction, **flow_args)
        if forward is None or ok is None:
            raise ValueError('RGB optical flow unavailable')
        backward, back_ok, _ = cv2.calcOpticalFlowPyrLK(
            gray_roi, reference_roi, forward, local_points.copy(), **flow_args)
        if backward is None or back_ok is None:
            raise ValueError('RGB reverse optical flow unavailable')
        good = ((ok.ravel() != 0) & (back_ok.ravel() != 0)
                & (error.ravel() < 25)
                & (np.linalg.norm(backward - local_points, axis=2).ravel() <= .8))
        before = points[good].reshape(-1,2)
        after = (forward+origin)[good].reshape(-1,2)
        labels = labels[good]
        if len(before) < 8 or any(np.sum(labels == i) < 3 for i in (0, 1)):
            raise ValueError('lost RGB support on one or both front legs')
        affine, inliers = cv2.estimateAffine2D(before, after, method=cv2.RANSAC,
                                             ransacReprojThreshold=1.2, maxIters=500,
                                             confidence=.99, refineIters=10)
        if affine is None or inliers is None or not np.isfinite(affine).all():
            raise ValueError('RGB pallet transform unavailable')
        inliers = inliers.ravel().astype(bool)
        if inliers.mean() < .7 or any(np.sum(labels[inliers] == i) < 3 for i in (0, 1)):
            raise ValueError('front legs do not share a consistent RGB motion')
        transform = np.vstack((affine, [0, 0, 1]))
        prior = cv2.transform(before[None], guess[:2])[0]
        return transform, prior, after, labels, inliers
