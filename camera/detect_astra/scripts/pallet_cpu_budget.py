"""Leave one Pi CPU available for laser/base/TF while vision and Nav2 run.

This only changes scheduling affinity, never sensor stamps or timeouts.
"""
import os
from pathlib import Path

HEAVY = {'amcl', 'pallet_amcl_health.py', 'astra_camera_node', 'publish_rgb_calibrated_cloud.py',
         'pointcloud_to_laserscan_node', 'publish_pallet_target.py',
         'publish_pallet_detection.py', 'planner_server', 'controller_server',
         'test_pallets.py'}


def identity(tid):
    try:
        return (Path('/proc')/str(tid)/'stat').read_text().rsplit(')', 1)[1].split()[19]
    except (OSError, IndexError):
        return None


class CpuBudget:
    def __init__(self):
        allowed = sorted(os.sched_getaffinity(0))
        self.compute = set(allowed[:-1]) if len(allowed) >= 4 else set(allowed)
        self.enabled = len(allowed) >= 4
        self.saved = {}

    def command(self, command):
        if not self.enabled:
            return command
        return ['taskset', '--cpu-list', ','.join(map(str, sorted(self.compute))), *command]

    def apply_process(self, pid):
        for entry in (Path('/proc')/str(pid)/'task').iterdir():
            tid = int(entry.name)
            try:
                original = os.sched_getaffinity(tid)
                selected = original & self.compute
                if not selected or selected == original:
                    continue
                token = identity(tid)
                if token is None:
                    continue
                self.saved.setdefault(tid, (token, original))
                os.sched_setaffinity(tid, selected)
            except ProcessLookupError:
                pass

    def apply_existing(self):
        if not self.enabled:
            return
        for entry in Path('/proc').iterdir():
            if not entry.name.isdecimal():
                continue
            try:
                args = entry.joinpath('cmdline').read_bytes().decode().strip('\0').split('\0')
                values = dict(item.split(b'=', 1) for item in entry.joinpath('environ').read_bytes().split(b'\0') if b'=' in item)
            except (OSError, UnicodeError):
                continue
            if values.get(b'ROS_DOMAIN_ID', b'0').decode() != os.environ.get('ROS_DOMAIN_ID', '0'):
                continue
            if any(Path(arg).name in HEAVY for arg in args):
                try:
                    self.apply_process(int(entry.name))
                except (FileNotFoundError, ProcessLookupError):
                    pass

    def restore(self):
        for tid, (token, original) in self.saved.items():
            try:
                if identity(tid) == token:
                    os.sched_setaffinity(tid, original)
            except ProcessLookupError:
                pass
        self.saved.clear()
