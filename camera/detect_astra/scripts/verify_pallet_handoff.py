#!/usr/bin/env python3
"""Read-only live qualification of YOLO -> RGB-D handoff. Never commands motors."""
import argparse
from collections import Counter
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from detected_pallets_interfaces.msg import PalletDetection, PalletTarget
from pallet_acquisition import valid_target


def distribution(values):
    values = sorted(v for v in values if math.isfinite(v))
    if not values:
        return None
    mean = sum(values)/len(values)
    return dict(min=min(values), max=max(values), mean=mean,
                std=math.sqrt(sum((v-mean)**2 for v in values)/len(values)),
                p95=values[int(.95*(len(values)-1))])


class HandoffVerifier(Node):
    def __init__(self, inspect_camera_streams=False):
        super().__init__('verify_pallet_handoff')
        self.state = None
        self.state_at = 0.
        self.last_target_at = 0.
        self.begin = None
        self.rows = []
        self.streams = {'rgb': [], 'depth': []}
        self.detections = []
        self.states = []
        self.wait_statuses = Counter()
        self.create_subscription(String, '/pallet/recognition/status', self.on_state,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(PalletTarget, '/pallet/target', self.on_target, 100)
        self.create_subscription(PalletDetection, '/pallet/detection', self.on_detection, 10)
        if inspect_camera_streams:
            for name, topic in [('rgb', '/camera/color/image_raw'), ('depth', '/camera/depth/image_raw')]:
                self.create_subscription(Image, topic, lambda m, n=name: self.on_image(n, m),
                    QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT), raw=True)

    def on_state(self, msg):
        try:
            self.state = json.loads(msg.data)
            self.state_at = time.monotonic()
            self.states.append((self.state_at, self.state))
        except (ValueError, TypeError):
            self.state = None

    def on_image(self, name, msg):
        self.streams[name].append(time.monotonic())

    def on_detection(self, msg):
        stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        self.detections.append((time.monotonic(), stamp, bool(msg.valid)))

    def on_target(self, msg):
        self.last_target_at = time.monotonic()
        if self.begin is None:
            self.wait_statuses['valid' if msg.valid else msg.status] += 1
            return
        now = self.get_clock().now().nanoseconds/1e9
        stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        self.rows.append(dict(wall=time.monotonic(), stamp=stamp, valid=bool(valid_target(msg, now)),
            status=msg.status, age_ms=(now-stamp)*1000, delta_ms=msg.rgb_depth_delta_ms,
            x=msg.camera_forward_m, y=msg.opening_lateral_m, heading_deg=math.degrees(msg.entry_heading_rad)))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=60.)
    parser.add_argument('--wait-seconds', type=float, default=60.)
    parser.add_argument('--inspect-camera-streams', action='store_true',
                        help='Also subscribe to full-size images; adds DDS image traffic')
    parser.add_argument('--output', type=Path, default=Path('data/diagnostics/pallet_handoff_live.json'))
    args = parser.parse_args()
    if not all(math.isfinite(v) and v >= 5 for v in (args.seconds, args.wait_seconds)):
        parser.error('Durations must be finite and at least 5 seconds')
    rclpy.init(); node = HandoffVerifier(args.inspect_camera_streams)
    report = dict(created_at_utc=datetime.now(timezone.utc).isoformat(), passed=False,
                  camera_streams_inspected=args.inspect_camera_streams,
                  scope='stationary perception only; no motion or navigation commands')
    try:
        deadline = time.monotonic()+args.wait_seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
            state = node.state
            if (state and time.monotonic()-node.state_at < .5
                    and time.monotonic()-node.last_target_at < .10 and state.get('locked')
                    and not state.get('enabled') and not state.get('busy')
                    and state.get('rgbd_health', {}).get('stable')):
                break
        else:
            report.update(reason='No stable post-YOLO RGB-D handoff within wait timeout', last_state=node.state,
                          waiting_target_statuses=dict(node.wait_statuses),
                          waiting_stream_messages={n:len(t) for n,t in node.streams.items()})
            return finish(args.output, report)
        initial = node.state
        node.streams = {'rgb': [], 'depth': []}
        node.begin = time.monotonic()
        barrier_ros = node.get_clock().now().nanoseconds/1e9
        print('Collecting stationary RGB-D with YOLO off...', flush=True)
        while time.monotonic()-node.begin < args.seconds:
            rclpy.spin_once(node, timeout_sec=.03)
        duration = time.monotonic()-node.begin
        rows = node.rows; good = [r for r in rows if r['valid']]
        states = [(w,s) for w,s in node.states if w >= node.begin]
        times = [r['wall'] for r in good]
        gap = max([b-a for a,b in zip(times,times[1:])]
                  + ([times[0]-node.begin, time.monotonic()-times[-1]] if times else [duration]))
        checks = dict(
            valid_ratio=bool(rows) and len(good)/len(rows) >= .98,
            valid_rate=len(good)/duration >= 10,
            max_valid_gap=gap <= .25,
            fresh_unique_targets=bool(rows) and initial.get('yolo_stopped_ros_s') is not None
                and all(r['stamp'] > initial['yolo_stopped_ros_s'] and -.05 <= r['age_ms']/1000 <= .25
                for r in good) and all(b['stamp'] > a['stamp'] for a,b in zip(good,good[1:])),
            yolo_stays_off=bool(states) and all(not s.get('enabled') and not s.get('busy')
                and s.get('session') == initial['session']
                and s.get('inference_attempts') == initial['inference_attempts'] for _,s in states),
            no_new_yolo_seeds=not any(stamp > barrier_ros for _,stamp,_ in node.detections),
            status_fresh=time.monotonic()-node.state_at <= .5 and bool(states)
                and max([b[0]-a[0] for a,b in zip(states,states[1:])]+[states[0][0]-node.begin]) <= .5,
            final_window_stable=bool(node.state and node.state.get('rgbd_health', {}).get('stable')))
        if args.inspect_camera_streams:
            checks.update(rgb_live=len(node.streams['rgb'])/duration >= 10,
                          depth_live=len(node.streams['depth'])/duration >= 10)
        checks['stationary_geometry'] = bool(good) and all(
            max(r[key] for r in good)-min(r[key] for r in good) <= limit
            for key,limit in [('x',.03),('y',.02),('heading_deg',2.)])
        report.update(passed=all(checks.values()), checks=checks, duration_s=duration,
            messages=len(rows), valid=len(good), valid_fps=len(good)/duration,
            valid_ratio=len(good)/len(rows) if rows else 0, max_valid_gap_s=gap,
            stream_fps={n:len(t)/duration for n,t in node.streams.items()} if args.inspect_camera_streams else None,
            invalid=dict(Counter(r['status'] for r in rows if not r['valid'])),
            measurements={k:distribution([r[k] for r in good]) for k in
                ('age_ms','delta_ms','x','y','heading_deg')}, initial_state=initial, final_state=node.state)
        return finish(args.output, report)
    finally:
        node.destroy_node(); rclpy.shutdown()


def finish(path, report):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False)+'\n')
    print(json.dumps(dict(passed=report['passed'], report=str(path), checks=report.get('checks'),
                          reason=report.get('reason')), ensure_ascii=False), flush=True)
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
