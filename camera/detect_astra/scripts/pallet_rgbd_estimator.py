"""Independent live RGB-D pallet measurement after a semantic YOLO seed."""
from collections import deque

from pallet_rgb_tracker import PalletRGBTracker
from pallet_target_geometry import measure_pallet_target
from pallet_target_bias import apply_target_bias
from pallet_messages import confirmed


class RGBDTargetEstimator:
    def __init__(self, args):
        self.args = args
        self.tracker = PalletRGBTracker(getattr(args,'rgb_gap_recovery',False))
        self.history = deque(maxlen=args.confirm_frames)
        self.seed_stamp_ns = 0
        self.seed_class = ''
        self.seed_confidence = 0.0
        self.depth_failures = 0
        self.last_seed_error = ''
        self.loss_reason = 'no YOLO seed received'
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
        self.depth_failures = 0
        self.loss_reason = reason

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
        # after the visual tracker has actually lost the target.
        if seed is not None and not self.tracker.active:
            try:
                max_seed_ns = int(getattr(self.args, 'max_seed_age_ms', 1000.)*1e6)
                if not 0 <= stamp_ns - seed['stamp_ns'] <= max_seed_ns:
                    raise ValueError('YOLO semantic seed outside acquisition age limit')
                candidate = PalletRGBTracker(getattr(self.args,'rgb_gap_recovery',False))
                mask, bbox = candidate.seed(seed['rgb'], seed['mask'], seed['bbox'], seed['stamp_ns'])
                # Bridge through the furthest cached frame within 400 ms,
                # leaving margin below the unchanged 500-ms flow gap gate.
                # Replaying every 150 ms needlessly delayed the CURRENT pose
                # when inference and navigation shared the Pi CPU.
                while stamp_ns-candidate.stamp_ns > 400_000_000:
                    bridge = [(s,rgb) for s,rgb in self.rgb_history
                              if candidate.stamp_ns < s < stamp_ns and
                              s-candidate.stamp_ns <= 400_000_000]
                    if not bridge:
                        raise ValueError('RGB tracking gap exceeded 500 ms while bridging seed')
                    cached_stamp,cached_rgb = bridge[-1]
                    candidate.update(cached_rgb,cached_stamp)
                if stamp_ns != seed['stamp_ns']:
                    mask, bbox = candidate.update(image, stamp_ns)
                proposed = self.measure(mask, bbox, depth, intrinsics)
                self.tracker = candidate
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
            if self.tracker.reverified:
                self.history.clear()
            try:
                measurement = self.measure(mask, bbox, depth, intrinsics)
            except ValueError as exc:
                self.history.clear()
                self.depth_failures += 1
                # Invalid Depth cannot produce a pose, but does not mean that
                # the RGB tracker lost the pallet. Reconfirm fresh geometry
                # after Depth returns, without requiring another YOLO seed.
                raise
        self.depth_failures = 0
        if self.history and not confirmed(measurement, self.history[-1]):
            self.reset('RGB-D geometry jumped')
            raise ValueError('LOST: RGB-D geometry jumped; enable YOLO to reacquire')
        self.history.append(measurement)
        if len(self.history) < self.args.confirm_frames:
            raise ValueError(f'confirming front legs {len(self.history)}/{self.args.confirm_frames}')
        return measurement
