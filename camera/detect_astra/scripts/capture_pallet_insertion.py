#!/usr/bin/env python3
"""Stationary acquisition and insertion preflight. Never publishes motor/lift commands."""
import argparse
from collections import deque
from dataclasses import asdict
import hashlib
import json
import math
from pathlib import Path
import time

import cv2
import numpy as np
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import SetBool
from detected_pallets_interfaces.msg import PalletDetection, PalletTarget
from tf2_ros import Buffer, TransformListener
import yaml

from pallet_insertion_core import InsertionConfig, Opening, make_plan, cloud_collision
from pallet_insertion_scene import RegisteredScene, choose_entry_height, transform_xy


def stamp(m):
    return m.header.stamp.sec+m.header.stamp.nanosec*1e-9


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))


def xyz(m):
    if m.header.frame_id != 'base_footprint' or m.is_bigendian:
        raise ValueError('Unexpected corrected cloud frame/byte order')
    fields = {f.name:f for f in m.fields}
    if any(k not in fields or fields[k].datatype != 7 for k in ('x','y','z')):
        raise ValueError('Need float32 XYZ cloud')
    return np.stack([np.ndarray((m.height,m.width),dtype='<f4',buffer=m.data,
        offset=fields[k].offset,strides=(m.row_step,m.point_step)) for k in ('x','y','z')],axis=-1)


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--output',type=Path,required=True)
    ap.add_argument('--acquire',action='store_true')
    a=ap.parse_args()
    rclpy.init();n=rclpy.create_node('pallet_insertion_preflight')
    buf=Buffer();listener=TransformListener(buf,n)
    state={};clouds=deque(maxlen=180);odoms=deque(maxlen=1000);subs=[]
    qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
    def record(k,m):state[k]=(m,time.monotonic())
    def cloud(m):clouds.append((stamp(m),m))
    def odom(m):odoms.append((stamp(m),m));record('odom',m)
    subs.append(n.create_subscription(PointCloud2,'/camera/depth/points_nav_corrected',cloud,qos))
    subs.append(n.create_subscription(Odometry,'/odom',odom,qos))
    subs.append(n.create_subscription(PalletTarget,'/pallet/target',lambda m:record('target',m),qos))
    subs.append(n.create_subscription(String,'/lift/status',lambda m:record('lift',m),10))
    subs.append(n.create_subscription(String,'/pallet/recognition/status',lambda m:record('status',m),10))
    subs.append(n.create_subscription(PalletDetection,'/pallet/detection',lambda m:record('seed',m),
        QoSProfile(depth=1)))
    report={'motor_commands_sent':False,'lift_commands_sent':False,'passed':False}
    try:
        if a.acquire:
            client=n.create_client(SetBool,'/pallet/set_enabled')
            if not client.wait_for_service(timeout_sec=5):raise RuntimeError('No recognition service')
            req=SetBool.Request();req.data=True;future=client.call_async(req)
            end=time.monotonic()+5
            while not future.done() and time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.01)
            if not future.done() or not future.result().success:raise RuntimeError('Acquisition request failed')
            print(future.result().message,flush=True)
        start=time.monotonic();last_print=0
        while time.monotonic()-start<45:
            rclpy.spin_once(n,timeout_sec=.01)
            if 'status' not in state:continue
            status=json.loads(state['status'][0].data)
            if time.monotonic()-last_print>3:
                print(json.dumps(status),flush=True);last_print=time.monotonic()
            if ('target' in state and state['target'][0].valid and 'seed' in state
                    and status.get('rgbd_health',{}).get('stable')
                    and not status.get('enabled',True) and not status.get('busy',True)):
                break
        else:raise RuntimeError('No stable complete pallet acquisition')
        now=n.get_clock().now().nanoseconds/1e9
        target=state['target'][0];seed=state['seed'][0]
        if now-stamp(target)>.3 or not seed.valid:
            raise RuntimeError('Stale/invalid semantic target')
        if stamp(seed)<status.get('started_ros_s',float('inf')):
            raise RuntimeError('Semantic seed belongs to an older acquisition session')
        if time.monotonic()-state['status'][1]>.5:raise RuntimeError('Recognition heartbeat stale')
        if not clouds or not odoms:raise RuntimeError('Missing cloud/odom')
        t,cl=min(clouds,key=lambda x:abs(x[0]-stamp(seed)))
        if abs(t-stamp(seed))>.075:raise RuntimeError('No cloud paired to semantic seed')
        _,od=min(odoms,key=lambda x:abs(x[0]-t))
        current=state['odom'][0]
        pose=lambda o:(o.pose.pose.position.x,o.pose.pose.position.y,yaw(o.pose.pose.orientation))
        start_pose=pose(current);cloud_pose=pose(od)
        for _,o in odoms:
            if (abs(o.twist.twist.linear.x)>.005 or abs(o.twist.twist.angular.z)>.025
                    or math.hypot(pose(o)[0]-start_pose[0],pose(o)[1]-start_pose[1])>.004):
                raise RuntimeError('Base was moving during snapshot window')
        _,to=min(odoms,key=lambda x:abs(x[0]-stamp(target)))
        op=pose(to)
        centre=transform_xy(np.array([[target.chassis_front_forward_m+.3005,
                                      target.opening_lateral_m,0.]]),op)[0]
        opening=Opening(centre[0],centre[1],op[2]+target.entry_heading_rad,target.opening_width_m)
        mask=cv2.imdecode(np.frombuffer(seed.mask.data,np.uint8),cv2.IMREAD_GRAYSCALE)
        if mask is None:raise RuntimeError('Invalid semantic mask')
        scene=RegisteredScene(xyz(cl),mask,seed.bbox_xyxy,opening,cloud_pose)
        c=InsertionConfig()
        goal_file=Path('ros2_ws/config/pallet_goals.yaml')
        saved=yaml.safe_load(goal_file.read_text())['pallets']['pallet_1']['pickup']
        if saved['frame_id']!='map' or saved['status']!='recorded':raise RuntimeError('Invalid saved station')
        tf=buf.lookup_transform('odom','map',rclpy.time.Time())
        if now-(tf.header.stamp.sec+tf.header.stamp.nanosec/1e9)>.5:raise RuntimeError('Stale map-to-odom')
        q=tf.transform.rotation;translation=tf.transform.translation
        g=transform_xy(np.array([[saved['position']['x'],saved['position']['y'],0.]]),
                       (translation.x,translation.y,yaw(q)))[0]
        nominal=(g[0],g[1],saved['yaw_rad']+yaw(q))
        initial=make_plan(start_pose,nominal,opening,0.,c)
        height=choose_entry_height(scene.static_points(),start_pose,initial.goal,c)
        plan=make_plan(start_pose,nominal,opening,height,c)
        report.update(config=asdict(c),plan=asdict(plan),registration=scene.metadata,
            goal_sha256=hashlib.sha256(goal_file.read_bytes()).hexdigest(),
            status=status,seed_class=seed.class_name,cloud_stamp=t,seed_stamp=stamp(seed),
            height_proposed_m=height,lift=json.loads(state['lift'][0].data) if 'lift' in state else None)
        a.output.parent.mkdir(parents=True,exist_ok=True)
        np.savez_compressed(a.output.with_suffix('.npz'),cloud=xyz(cl),mask=mask,
            bbox=np.asarray(seed.bbox_xyxy),cloud_pose=np.asarray(cloud_pose),
            points=scene.static_points(),raw_pallet=scene.raw_pallet)
        for p in plan.poses:
            hit=cloud_collision(scene.static_points(),p,height)
            if hit:
                report['collision']={'pose':p,'volume':hit[0],'point_robot':hit[1]}
                raise RuntimeError('Insertion swept volume intersects observed '+hit[0])
        report['passed']=True
    except Exception as e:
        report['reason']=str(e)
    finally:
        a.output.parent.mkdir(parents=True,exist_ok=True)
        a.output.write_text(json.dumps(report,indent=2,allow_nan=False))
        print(json.dumps({k:v for k,v in report.items() if k not in ('plan','status')},indent=2),flush=True)
        n.destroy_node();rclpy.shutdown()
    return 0 if report['passed'] else 1


if __name__=='__main__':
    raise SystemExit(main())
