#!/usr/bin/env python3
"""Run acquisition and RGB-D measurement as separate, supervised processes."""
from pathlib import Path
import os
import signal
import subprocess
import sys
import threading

from pallet_perception_config import parse_args


def main():
    parse_args('both')
    stopped = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stopped.set())
    children = []
    code = 0
    try:
        for name in ('publish_pallet_target.py', 'publish_pallet_detection.py'):
            env = {**os.environ, 'OPENBLAS_NUM_THREADS': '1', 'MKL_NUM_THREADS': '1',
                   'OMP_NUM_THREADS': '1', 'OMP_WAIT_POLICY': 'PASSIVE',
                   'GOMP_SPINCOUNT': '0'}
            # Raw RGB and semantic seeds exceed FastDDS's default SHM segment.
            # Use the same large-sample transport as the live camera processes.
            profile = Path(__file__).resolve().parents[3]/'ros2_ws/config/camera_fastdds.xml'
            if profile.is_file():
                env.setdefault('FASTRTPS_DEFAULT_PROFILES_FILE',str(profile))
                env.setdefault('FASTDDS_DEFAULT_PROFILES_FILE',str(profile))
            children.append(subprocess.Popen(
                [sys.executable, str(Path(__file__).with_name(name)), *sys.argv[1:]],
                start_new_session=True, env=env))
        while not stopped.wait(.2):
            exited = next((child for child in children if child.poll() is not None), None)
            if exited is not None:
                code = exited.returncode or 1
                break
    finally:
        for child in children:
            if child.poll() is None:
                child.send_signal(signal.SIGINT)
        for child in children:
            try:
                child.wait(timeout=6)
            except subprocess.TimeoutExpired:
                child.kill()
                child.wait()
    return code


if __name__ == '__main__':
    raise SystemExit(main())
