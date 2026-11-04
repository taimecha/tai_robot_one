"""Lightweight smoothing of pallet heading for the web display."""

import math


HEADING_TOLERANCE_DEG = 1.0


class HeadingDisplayFilter:
    """Time-based EMA: suppress small jitter and follow large changes quickly.

    Use capture timestamps so HTTP polls never count as new measurements.
    Call reset when the measurement becomes invalid or recognition is paused.
    This does not change the ROS measurement or command robot motion.
    """

    def __init__(self):
        self.reset()

    def reset(self):
        self.stamp_ns = None
        self.angle_deg = None

    def update(self, angle_deg, stamp_ns):
        if not math.isfinite(angle_deg):
            self.reset()
            raise ValueError("Nonfinite pallet heading")
        if self.stamp_ns == stamp_ns:
            return self.angle_deg
        dt = (stamp_ns - self.stamp_ns) / 1e9 if self.stamp_ns is not None else None
        if dt is None or not 0 < dt <= 1.0:
            self.angle_deg = angle_deg
        else:
            delta = angle_deg - self.angle_deg
            # At 2 FPS: small changes get alpha ~0.46; large changes ~0.96.
            tau = 0.15 if abs(delta) >= 2.0 else 0.8
            alpha = -math.expm1(-dt / tau)
            self.angle_deg += alpha * delta
        self.stamp_ns = stamp_ns
        return self.angle_deg


def heading_correction_required(angle_deg):
    """The tolerance applies to orientation only; lateral offset is separate."""
    return abs(angle_deg) >= HEADING_TOLERANCE_DEG
