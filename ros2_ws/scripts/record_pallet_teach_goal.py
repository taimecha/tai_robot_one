#!/usr/bin/env python3
"""Record RViz teaching clicks without sending a navigation command."""

import argparse
import json
import math
from datetime import datetime, timezone
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)

    rclpy.init()
    node = Node('record_pallet_teach_goal')

    def capture(msg: PoseStamped):
        p = msg.pose.position
        q = msg.pose.orientation
        norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
        if msg.header.frame_id != 'map' or not all(math.isfinite(v) for v in
                (p.x, p.y, p.z, q.x, q.y, q.z, q.w)) or abs(norm - 1.0) > 0.02:
            node.get_logger().warn('Ignored goal: frame must be map and quaternion valid')
            return
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y),
                         1 - 2*(q.y*q.y + q.z*q.z))
        record = {
            'recorded_at_utc': datetime.now(timezone.utc).isoformat(),
            'source_topic': '/pallet/teach_goal',
            'frame_id': 'map',
            'position': {'x': p.x, 'y': p.y, 'z': p.z},
            'orientation': {'x': q.x, 'y': q.y, 'z': q.z, 'w': q.w},
            'yaw_rad': yaw,
            'yaw_deg': math.degrees(yaw),
            'status': 'operator_selected_unverified',
        }
        with args.output.open('a', encoding='utf-8') as stream:
            stream.write(json.dumps(record, ensure_ascii=False) + '\n')
            stream.flush()
        print(json.dumps(record, ensure_ascii=False), flush=True)

    node.create_subscription(PoseStamped, '/pallet/teach_goal', capture, 10)
    print('Waiting for RViz goal on /pallet/teach_goal; vehicle remains stationary.', flush=True)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
