#!/usr/bin/env python3
"""Live stationary A-quality display; never saves A or sends a motion goal."""
import argparse
import copy
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String
from tf2_ros import TransformException
from visualization_msgs.msg import Marker, MarkerArray

from r3_recognition_goal import (RecognitionGoal, atomic_json, map_identity,
                                 validate_a_localization, validate_a_scan_match, DEFAULT_GOAL,
                                 A_XY_RMS_LIMIT_M,A_YAW_STD_LIMIT_DEG,A_PREFERRED_XY_M)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    selected_map=map_identity(args.map)
    rclpy.init()
    node=RecognitionGoal(request_nomotion=False,node_name='r3_a_readiness_watch',
                         selected_map_file=args.map)
    qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,
                   durability=DurabilityPolicy.TRANSIENT_LOCAL)
    status_pub=node.create_publisher(String,'/pallet/r3/localization/a_status',qos)
    markers=node.create_publisher(MarkerArray,'/pallet/r3/localization/a_markers',qos)
    floor_state=[]
    def on_floor(message):
        try:
            floor_state[:]=[json.loads(message.data)]
        except (ValueError,TypeError):
            floor_state.clear()
    node.create_subscription(String,'/camera/depth/nav_floor_alignment/status',on_floor,qos)
    next_report=last_log=0.
    previous=None
    stable_since=None
    pose=None
    try:
        while rclpy.ok():
            node.spin_once()
            wall=time.monotonic()
            if wall<next_report:
                continue
            next_report=wall+.5
            report=dict(ready=False,reason='waiting for localization',published_ros_s=node.now(),
                frame_id='map',map=selected_map,pose_xyyaw=None,
                required_limits=dict(position_rms_m=A_XY_RMS_LIMIT_M,
                    position_comparison='less_than',preferred_position_m=A_PREFERRED_XY_M,
                    yaw_std_deg=A_YAW_STD_LIMIT_DEG),
                physical_alignment_verified=False,recognition_A_saved=DEFAULT_GOAL.exists())
            report['strict_a_scan_duration_s']=node.a_scan_quality_duration()
            try:
                pose=node.current_pose(require_still=True)
                report.update(pose_xyyaw=list(pose),heading_deg=math.degrees(pose[2]),
                    amcl_xy_rms_m=math.sqrt(node.amcl.pose.covariance[0]+node.amcl.pose.covariance[7]),
                    amcl_yaw_std_deg=math.degrees(math.sqrt(node.amcl.pose.covariance[35])),
                    scan_map_match=node.map_match,particle_metrics=node.particle_gate.metrics)
                recent=node.recent_a_poses()
                if node.stationary_mode:
                    report['stationary_snapshot_validation']=node.localization_evidence()[
                        'stationary_snapshot_validation']
                validate_a_scan_match(node.map_match)
                quality=validate_a_localization(node.amcl.pose.covariance,
                    node.particle_gate.metrics,recent,pose)
                if stable_since is None:
                    stable_since=wall
                scan_ready=node.a_scan_quality_ready()
                ready=wall-stable_since>=2. and scan_ready
                report.update(ready=ready,reason=('stationary localization meets A estimate limits'
                    if ready else 'waiting for strict A scan/map agreement on every distinct scan for 2 seconds'
                    if not scan_ready else 'waiting for A quality to remain valid for 2 seconds'),
                    pose_xyyaw=list(pose),heading_deg=math.degrees(pose[2]),quality=quality,
                    strict_a_scan_duration_s=node.a_scan_quality_duration(),
                    scan_map_match=node.map_match,particle_metrics=node.particle_gate.metrics)
            except (ValueError,TransformException) as exc:
                stable_since=None
                report['reason']=str(exc)
            floor=floor_state[-1] if floor_state else {}
            report['camera_floor_alignment']=floor
            atomic_json(args.output,report)
            status_pub.publish(String(data=json.dumps(report,allow_nan=False)))
            marker=Marker()
            marker.header.frame_id='map';marker.header.stamp=node.get_clock().now().to_msg()
            marker.ns='a_readiness';marker.id=0;marker.type=Marker.TEXT_VIEW_FACING
            marker.pose.orientation.w=1.;marker.pose.position.z=1.4
            if pose is not None:
                marker.pose.position.x,marker.pose.position.y=pose[0]+1.,pose[1]+.40
            marker.scale.z=.07;marker.color.a=1.
            marker.color.r=.1 if report['ready'] else 1.
            marker.color.g=.9 if report['ready'] else .2
            marker.text=('A READY TO SAVE' if report['ready'] else 'A NOT READY')+f' | < {A_XY_RMS_LIMIT_M*100:g} cm / {A_YAW_STD_LIMIT_DEG:g} deg'
            floor_marker=copy.deepcopy(marker)
            floor_marker.ns='camera_floor_alignment';floor_marker.pose.position.z=1.65
            floor_marker.pose.position.y-=.15
            floor_ready=(floor.get('ready') is True
                         and -.05<=node.now()-floor.get('published_ros_s',0)<=1.)
            floor_marker.color.r=.1 if floor_ready else 1.
            floor_marker.color.g=.9 if floor_ready else .2
            floor_marker.text='CAMERA FLOOR '+('ALIGNED' if floor_ready else 'NOT READY')
            markers.publish(MarkerArray(markers=[marker,floor_marker]))
            key=(report['ready'],report['reason'])
            if (key!=previous and wall-last_log>=2.) or wall-last_log>=10.:
                print(json.dumps(report,ensure_ascii=False,allow_nan=False),flush=True)
                last_log=wall;previous=key
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__=='__main__':
    main()
