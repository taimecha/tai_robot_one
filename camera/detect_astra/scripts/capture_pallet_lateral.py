#!/usr/bin/env python3
"""Capture stationary lateral-calibration data; perception only, no actuators."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import statistics
import subprocess
import sys
import time

import rclpy
from rclpy.qos import QoSProfile,DurabilityPolicy,qos_profile_sensor_data
from std_msgs.msg import String
from std_srvs.srv import SetBool
from nav_msgs.msg import Odometry
from detected_pallets_interfaces.msg import PalletTarget

ROOT=Path(__file__).resolve().parents[1]


def cleanup(parent):
    owned=[]
    for p in Path('/proc').iterdir():
        if not p.name.isdigit():continue
        try:
            fields=(p/'stat').read_text().rsplit(')',1)[1].split()
            args=(p/'cmdline').read_bytes().decode().split('\0')
            if int(fields[1])==parent.pid and any(Path(a).name in (
                    'publish_pallet_target.py','publish_pallet_detection.py') for a in args):
                owned.append(int(p.name))
        except (OSError,ValueError):pass
    if parent.poll() is None:parent.send_signal(signal.SIGINT)
    try:parent.wait(timeout=10)
    except subprocess.TimeoutExpired:
        for pid in owned:
            try:os.kill(pid,signal.SIGTERM)
            except ProcessLookupError:pass
        parent.terminate()
        try:parent.wait(timeout=3)
        except subprocess.TimeoutExpired:parent.kill();parent.wait()
    for pid in owned:
        try:
            args=(Path('/proc')/str(pid)/'cmdline').read_bytes().decode().split('\0')
            if any(Path(a).name in ('publish_pallet_target.py','publish_pallet_detection.py') for a in args):
                os.kill(pid,signal.SIGKILL)
        except (OSError,ValueError):pass


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bias-profile',type=Path)
    parser.add_argument('--timeout',type=float,default=60.)
    args=parser.parse_args()
    out=ROOT/'data/calibration'/time.strftime('lateral_capture_%Y%m%d_%H%M%S')
    out.mkdir(parents=True)
    rclpy.init();node=rclpy.create_node('pallet_lateral_calibration_capture_only')
    samples=[];recognition={};odometry=[];started=node.get_clock().now().nanoseconds/1e9
    def target(m):
        stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
        age=node.get_clock().now().nanoseconds/1e9-stamp
        if not m.valid or stamp<started or not -.05<=age<=.25:return
        if samples and stamp<=samples[-1]['stamp_s']:return
        row=dict(stamp_s=stamp,lateral_m=m.opening_lateral_m,
            base_forward_m=m.chassis_front_forward_m+.3005,
            heading_deg=math.degrees(m.entry_heading_rad),width_m=m.opening_width_m,
            status=m.status,rgb_depth_delta_ms=m.rgb_depth_delta_ms)
        if args.bias_profile is None:row['lateral_raw_m']=m.opening_lateral_m
        samples.append(row)
    def state(m):
        try:recognition.update(json.loads(m.data))
        except (ValueError,TypeError):pass
    def odom(m):
        p=m.pose.pose.position;q=m.pose.pose.orientation
        odometry.append(dict(stamp_s=m.header.stamp.sec+m.header.stamp.nanosec*1e-9,
            x=p.x,y=p.y,yaw=2*math.atan2(q.z,q.w),v=m.twist.twist.linear.x,w=m.twist.twist.angular.z))
    node.create_subscription(PalletTarget,'/pallet/target',target,1)
    node.create_subscription(String,'/pallet/recognition/status',state,
        QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    node.create_subscription(Odometry,'/odom',odom,qos_profile_sensor_data)
    client=node.create_client(SetBool,'/pallet/set_enabled')
    command=[sys.executable,str(ROOT/'scripts/run_pallet_perception.py'),
        '--model',str(ROOT/'data/runtime_models/best_v3_ncnn_model'),'--cpu-threads','2',
        '--target-color','green','--track-max-fps','15','--preview-fps','2','--heading-floor-reference']
    if args.bias_profile:command+=['--measurement-bias-yaml',str(args.bias_profile.resolve())]
    parent=None
    try:
        end=time.monotonic()+1.5
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)
        if node.get_publishers_info_by_topic('/pallet/target'):
            raise RuntimeError('Existing pallet publisher; do not start a second perception stack')
        with (out/'perception.log').open('w') as log:
            parent=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            end=time.monotonic()+args.timeout;retried=set();pending=None
            while time.monotonic()<end:
                rclpy.spin_once(node,timeout_sec=.04)
                if parent.poll() is not None:raise RuntimeError('Perception exited: '+str(parent.returncode))
                if pending is not None and pending.done():
                    if not pending.result().success:raise RuntimeError(pending.result().message)
                    pending=None
                session=recognition.get('session')
                if (pending is None and recognition.get('phase')=='ACQUIRE_FAILED'
                        and session not in retried and recognition.get('enabled') is False
                        and recognition.get('busy') is False and client.service_is_ready()):
                    retried.add(session);pending=client.call_async(SetBool.Request(data=True))
                if (samples and samples[-1]['stamp_s']-samples[0]['stamp_s']>=12
                        and recognition.get('rgbd_health',{}).get('stable')):break
            now=node.get_clock().now().nanoseconds/1e9
            recent=[s for s in samples if now-s['stamp_s']<=12]
            report=dict(wall_time=time.strftime('%Y-%m-%d %H:%M:%S'),sample_count=len(recent),
                bias_profile=str(args.bias_profile) if args.bias_profile else None,samples=recent,
                odometry=[o for o in odometry if now-o['stamp_s']<=12],recognition=recognition,
                latest_age_s=now-recent[-1]['stamp_s'] if recent else None,log_dir=str(out))
            if recent:
                report['median']={k:statistics.median(s[k] for s in recent) for k in
                    ('lateral_m','base_forward_m','heading_deg','width_m')}
                report['lateral_range_m']=[min(s['lateral_m'] for s in recent),max(s['lateral_m'] for s in recent)]
            path=out/'capture.json';path.write_text(json.dumps(report,ensure_ascii=False,indent=2))
            print(json.dumps({k:v for k,v in report.items() if k not in ('samples','odometry','recognition')},ensure_ascii=False))
            print('Capture:',path,flush=True)
            if not recent:raise RuntimeError('No valid measurements; no calibration may be fitted')
    finally:
        if parent is not None:cleanup(parent)
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
