"""Independent live RGB-D pallet measurement after a semantic YOLO seed."""
from collections import deque
import math
import statistics

from pallet_rgb_tracker import PalletRGBTracker
from pallet_target_geometry import measure_pallet_target
from pallet_target_bias import apply_target_bias
from pallet_messages import confirmed


class RGBDTargetEstimator:
    def __init__(self, args):
        self.args = args
        self.tracker = PalletRGBTracker()
        self.heading_filter_frames = max(args.confirm_frames,
                                         getattr(args,'heading_filter_frames',args.confirm_frames))
        self.history = deque(maxlen=self.heading_filter_frames)
        self.last_accepted_stamp_ns = None
        self.seed_stamp_ns = 0
        self.seed_class = ''
        self.seed_confidence = 0.0
        self.depth_failures = 0
        self.last_seed_error = ''
        self.loss_reason = 'no YOLO seed received'
        self.measurement_failure_since_ns = None
        self.measurement_reacquisitions = 0
        # RGB only, never cached Depth/poses. Bridge a delayed semantic mask
        # through observed frames while preserving the 500-ms RGB gap gate.
        self.rgb_history = deque(maxlen=64)

    def remember_rgb(self, image, stamp_ns):
        if self.rgb_history and stamp_ns <= self.rgb_history[-1][0]:
            return
        self.rgb_history.append((stamp_ns, image.copy()))
        age_ns = int((getattr(self.args, 'max_seed_age_ms', 1000.)+500.)*1e6)
        while self.rgb_history and stamp_ns-self.rgb_history[0][0] > age_ns:
            self.rgb_history.popleft()

    def reset(self, reason='tracking reset'):
        self.tracker.reset()
        self.history.clear()
        self.last_accepted_stamp_ns = None
        self.depth_failures = 0
        self.loss_reason = reason
        self.measurement_failure_since_ns = None

    def measurement_failed(self, stamp_ns):
        if self.measurement_failure_since_ns is None:
            self.measurement_failure_since_ns = stamp_ns

    def needs_measurement_reacquisition(self, stamp_ns):
        """A visually coherent mask can still be the wrong depth sample region."""
        return (self.measurement_failure_since_ns is not None
                and stamp_ns-self.measurement_failure_since_ns >= 1_000_000_000)

    def measure(self, mask, bbox, depth, intrinsics):
        a = self.args
        m = measure_pallet_target(
            mask, bbox, depth, intrinsics, pitch_deg=a.pitch_deg,
            inverse_depth_x_slope=a.inverse_depth_x_slope, reference_u=a.reference_u,
            camera_ahead_of_chassis_m=a.camera_ahead_of_chassis_m,
            expected_opening_width_m=a.expected_opening_width_m,
            heading_floor_reference=getattr(a,'heading_floor_reference',False),
            opening_width_tolerance_m=a.opening_width_tolerance_m)
        if a.measurement_bias is not None:
            bias = a.measurement_bias
            # Keep the camera-frame value for an explicit operator report.
            # The configured lateral offset is the total correction, not an
            # extra term added after the old empirical lateral bias.
            m['opening_lateral_camera_raw_m'] = m['opening_lateral_m']
            if getattr(a,'heading_floor_reference',False):
                # The old angular residual was fitted to the old depth skew.
                # Preserve its position offsets, never apply it a second time
                # to the independently floor-referenced plane angle.
                bias = {**bias,'face_yaw_bias_deg':0.}
            m = apply_target_bias(m,bias)
            m['applied_heading_bias_deg'] = bias['face_yaw_bias_deg']
        return m

    def track_rgb(self, image, stamp_ns):
        """Advance only visual state; this never returns a 3-D measurement."""
        try:
            return self.tracker.update(image, stamp_ns)
        except ValueError as exc:
            self.history.clear()
            self.loss_reason = str(exc)
            raise

    def suspend_measurement(self, image, stamp_ns):
        """Bad metadata clears measurements, but fresh RGB can keep identity."""
        self.history.clear()
        self.remember_rgb(image, stamp_ns)
        if self.tracker.active:
            self.track_rgb(image, stamp_ns)

    def process(self, image, depth, intrinsics, stamp_ns, seed=None):
        self.remember_rgb(image, stamp_ns)
        measurement = None
        # Semantic detections acquire a target; do not replace a healthy RGB
        # reference on every inference. Mask jitter would move the depth sample
        # bands even with a stationary pallet. Recovery uses a fresh seed only
        # after visual loss or a sustained lack of valid current geometry.
        # In the latter case a fresh semantic mask is checked independently;
        # an operator must not need to occlude the image to release a bad mask.
        replace_failed_measurement = (self.tracker.active
            and self.needs_measurement_reacquisition(stamp_ns)
            and seed is not None and seed['stamp_ns'] > self.seed_stamp_ns
            and seed['stamp_ns'] >= self.measurement_failure_since_ns)
        if seed is not None and (not self.tracker.active or replace_failed_measurement):
            try:
                max_seed_ns = int(getattr(self.args, 'max_seed_age_ms', 1000.)*1e6)
                if not 0 <= stamp_ns - seed['stamp_ns'] <= max_seed_ns:
                    raise ValueError('YOLO semantic seed outside acquisition age limit')
                candidate = PalletRGBTracker()
                mask, bbox = candidate.seed(seed['rgb'], seed['mask'], seed['bbox'], seed['stamp_ns'])
                for cached_stamp, cached_rgb in self.rgb_history:
                    if (seed['stamp_ns'] < cached_stamp < stamp_ns
                            and cached_stamp-candidate.stamp_ns >= 150_000_000):
                        candidate.update(cached_rgb, cached_stamp)
                if stamp_ns != seed['stamp_ns']:
                    mask, bbox = candidate.update(image, stamp_ns)
                proposed = self.measure(mask, bbox, depth, intrinsics)
                self.tracker = candidate
                if replace_failed_measurement:
                    self.history.clear()
                    self.measurement_reacquisitions += 1
                self.seed_stamp_ns = seed['stamp_ns']
                self.seed_class = seed['class_name']
                self.seed_confidence = seed['confidence']
                measurement = proposed
                self.last_seed_error = ''
                self.loss_reason = ''
            except ValueError as exc:
                self.last_seed_error = str(exc)
        if measurement is None:
            if not self.tracker.active:
                raise ValueError('LOST: enable YOLO to acquire pallet; ' +
                                 (self.last_seed_error or self.loss_reason))
            mask, bbox = self.track_rgb(image, stamp_ns)
            try:
                measurement = self.measure(mask, bbox, depth, intrinsics)
            except ValueError as exc:
                self.history.clear()
                self.depth_failures += 1
                self.measurement_failed(stamp_ns)
                # Invalid Depth cannot produce a pose, but does not mean that
                # the RGB tracker lost the pallet. Reconfirm fresh geometry
                # after Depth returns, without requiring another YOLO seed.
                raise
        self.depth_failures = 0
        if (self.last_accepted_stamp_ns is not None and
                stamp_ns-self.last_accepted_stamp_ns > 500_000_000):
            self.history.clear()
        if self.history and not confirmed(measurement, self.history[-1]):
            previous=self.history[-1]
            position_ok=(abs(measurement['camera_forward_m']-previous['camera_forward_m'])<.06
                         and abs(measurement['opening_lateral_m']-previous['opening_lateral_m'])<.035)
            if position_ok:
                # One noisy depth difference between the two legs must not
                # discard a healthy RGB track or contaminate the median.
                self.measurement_failed(stamp_ns)
                raise ValueError('heading outlier; waiting for fresh RGB-D angle')
            self.reset('RGB-D geometry jumped')
            raise ValueError('LOST: RGB-D geometry jumped; enable YOLO to reacquire')
        self.measurement_failure_since_ns = None
        self.history.append(measurement)
        self.last_accepted_stamp_ns=stamp_ns
        if len(self.history) < self.heading_filter_frames:
            raise ValueError(f'confirming front legs {len(self.history)}/{self.heading_filter_frames}; heading median pending')
        filtered=dict(measurement)
        angles=[m['face_yaw_rad'] for m in self.history]
        median=float(statistics.median(angles))
        inliers=[angle for angle in angles if abs(angle-median)<=math.radians(2.5)]
        if len(inliers)<self.heading_filter_frames-1:
            raise ValueError('heading spread too large; waiting for stable RGB-D')
        filtered['face_yaw_rad']=float(statistics.median(inliers))
        filtered['entry_heading_rad']=-filtered['face_yaw_rad']
        return filtered
