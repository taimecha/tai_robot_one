"""Runtime posterior checks, separate from LiDAR fit and capture freshness.

Resampling can copy one well-supported pose many times. Distinct-pose counts
and the mass of one quantized bin describe that concentration; they do not
establish a localization failure. Consumers still validate the full posterior,
its original covariance, current scan/map agreement and sensor continuity.
Teaching a new recognition A retains its separate conservative checks.
"""
import math


def concentrated_particle_support(metrics):
    """A concentrated cloud needs numerical pose/cloud pairing, not a veto."""
    return (metrics.get('unique_pose_bins',0)<10
            or metrics.get('largest_pose_bin_mass',1.)>.5)


def validate_runtime_particles(metrics):
    """Require unimodal, finite full-cloud spread below 5 cm / within 2 deg."""
    if not isinstance(metrics,dict) or metrics.get('converged') is not True:
        raise ValueError('AMCL full particle cloud is missing/ambiguous')
    for name in ('xy_rms_m','yaw_std_deg'):
        value=metrics.get(name)
        if (isinstance(value,bool) or not isinstance(value,(int,float))
                or not math.isfinite(value) or value<0):
            raise ValueError('invalid AMCL particle spread: '+name)
    if metrics['xy_rms_m']>=.05-1e-12 or metrics['yaw_std_deg']>2.+1e-9:
        raise ValueError('AMCL full particle spread requires <5 cm / 2 deg '
                         f"({metrics['xy_rms_m']*100:.2f} cm / {metrics['yaw_std_deg']:.2f} deg)")
