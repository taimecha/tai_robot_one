"""Shared localization frame/run contract for capture, tracking and scan masking."""
import json
import math
from collections import deque

from r3_core import wrap


def localization_spec(target):
    spec = target.get('localization') or {}
    if target.get('frame_id', 'odom') == 'odom' and not spec:
        return None
    if (target.get('frame_id') != 'r3_trial' or spec.get('mode') != 'amcl'
            or not spec.get('run_id') or not spec.get('map_file')):
        raise ValueError('localized target lacks its AMCL map/run reference')
    for key in ('baseline_map_odom', 'correction_at_capture_xyyaw'):
        value = spec.get(key)
        if not isinstance(value, list) or len(value) != 3 or not all(math.isfinite(v) for v in value):
            raise ValueError('invalid localized target reference: '+key)
    if spec.get('control_status_topic') not in (None,'/pallet/r3/localization/status','/pallet/r3/control_odom/status'):
        raise ValueError('unsupported localization control status topic')
    return spec


def checked_status(data, now_ros, expected=None):
    status = json.loads(data) if isinstance(data, str) else data
    if status.get('ready') is not True:
        raise ValueError('AMCL unavailable: '+str(status.get('reason', 'not ready')))
    if (status.get('frame_id') != 'r3_trial' or not status.get('run_id')
            or not status.get('map_file')
            or not -.05 <= now_ros-float(status.get('published_ros_s', 0)) <= .5):
        raise ValueError('AMCL status is stale or has an invalid frame')
    for key in ('correction_xyyaw', 'baseline_map_odom'):
        value = status.get(key)
        if not isinstance(value, list) or len(value) != 3 or not all(math.isfinite(v) for v in value):
            raise ValueError('AMCL transform is invalid')
    if expected and (status['run_id'] != expected['run_id']
                     or status['map_file'] != expected['map_file']
                     or any(abs(a-b) > 1e-8 for a,b in zip(
                         status['baseline_map_odom'], expected['baseline_map_odom']))):
        raise ValueError('AMCL restarted or its map/reference changed; recognize again')
    return status


class CorrectionHistory:
    """Interpolate the applied trial correction at a sensor capture stamp.

    Bridge updates are normally 20 Hz. Endpoint holds are limited to 50 ms:
    at the bridge limits this contributes at most 1.5 mm translation and
    0.025 degrees rotation, rather than applying today's correction to a
    scan captured 300 ms earlier. No velocity extrapolation is performed.
    Current readiness must still be checked separately before using history.
    """
    def __init__(self, max_age_s=2., endpoint_hold_s=.05, max_gap_s=.2):
        self.entries = deque(maxlen=100)
        self.max_age_s = max_age_s
        self.endpoint_hold_s = endpoint_hold_s
        self.max_gap_s = max_gap_s

    def clear(self):
        self.entries.clear()

    def add(self, status):
        stamp = float(status['published_ros_s'])
        correction = tuple(status['correction_xyyaw'])
        reference = (status['run_id'], status['map_file'], tuple(status['baseline_map_odom']))
        if not math.isfinite(stamp) or not all(math.isfinite(v) for v in correction):
            raise ValueError('AMCL correction history contains nonfinite values')
        if self.entries:
            previous = self.entries[-1]
            if reference != previous[2]:
                self.clear()
                raise ValueError('AMCL correction history reference changed')
            if stamp < previous[0]:
                self.clear()
                raise ValueError('AMCL correction history clock went backwards')
            if stamp == previous[0]:
                if correction != previous[1]:
                    raise ValueError('AMCL correction changed at the same timestamp')
                return
        self.entries.append((stamp, correction, reference))
        while len(self.entries) > 2 and stamp-self.entries[0][0] > self.max_age_s:
            self.entries.popleft()

    def at(self, stamp):
        if not self.entries or not math.isfinite(stamp):
            raise ValueError('AMCL correction history unavailable at scan stamp')
        first, last = self.entries[0], self.entries[-1]
        if stamp <= first[0]:
            if first[0]-stamp <= self.endpoint_hold_s+1e-9:
                return first[1]
            raise ValueError('scan predates AMCL correction history')
        if stamp >= last[0]:
            if stamp-last[0] <= self.endpoint_hold_s+1e-9:
                return last[1]
            raise ValueError('scan is newer than AMCL correction history')
        for before, after in zip(self.entries, list(self.entries)[1:]):
            if before[0] <= stamp <= after[0]:
                span = after[0]-before[0]
                if span > self.max_gap_s+1e-9:
                    raise ValueError('AMCL correction history has a gap at scan stamp')
                fraction = (stamp-before[0])/span
                a, b = before[1], after[1]
                return (a[0]+fraction*(b[0]-a[0]),
                        a[1]+fraction*(b[1]-a[1]),
                        wrap(a[2]+fraction*wrap(b[2]-a[2])))
        raise ValueError('AMCL correction history cannot bracket scan stamp')
