"""Real AMCL global reset/updates against synthetic stationary sensors; domain 219.

No driver nodes or motor command subscribers are started.
"""
import math
import os
os.environ['ROS_DOMAIN_ID']='219'
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE']='LOCALHOST'
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import threading
import time
import unittest
import numpy as np
from PIL import Image
import yaml

ROOT=Path(__file__).resolve().parents[1]
REPO=ROOT.parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import TransformStamped,Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster
from run_pallet_cycle import Cycle


class AutoLocalizationRosTest(unittest.TestCase):
    def test_real_amcl_localizes_before_any_pallet_or_motion_start(self):
        metadata=yaml.safe_load((REPO/'ros2_ws/maps/tai_map_real_edited.yaml').read_text())
        gray=np.asarray(Image.open(REPO/'ros2_ws/maps'/metadata['image']))[::-1]
        res=metadata['resolution'];origin=np.array(metadata['origin'][:2])
        truth=np.array([2.9406,1.24105,-math.pi/2])
        angles=-math.pi+np.arange(720)*2*math.pi/720
        distances=np.arange(.05,6.,res/2)
        endpoints=truth[:2]+distances[:,None,None]*np.stack((np.cos(angles+truth[2]),np.sin(angles+truth[2])),axis=1)[None,:,:]
        cells=np.floor((endpoints-origin)/res).astype(int)
        inside=(cells[:,:,0]>=0)&(cells[:,:,0]<gray.shape[1])&(cells[:,:,1]>=0)&(cells[:,:,1]<gray.shape[0])
        values=np.full(cells.shape[:2],254,dtype=np.uint8)
        values[inside]=gray[cells[:,:,1][inside],cells[:,:,0][inside]]
        hit=inside&(values<65)
        ranges=[float(distances[np.argmax(hit[:,i])]) if hit[:,i].any() else float('inf') for i in range(len(angles))]
        rclpy.init()
        sensors=Node('stationary_localization_test_inputs');executor=SingleThreadedExecutor();executor.add_node(sensors)
        odom=sensors.create_publisher(Odometry,'/odom',1)
        scan=sensors.create_publisher(LaserScan,'/scan_filtered',1)
        lift=sensors.create_publisher(String,'/lift/status',1)
        tf=TransformBroadcaster(sensors)
        commands=[]
        sensors.create_subscription(Twist,'/cmd_vel_nav',lambda m:commands.append(m),1)
        def emit():
            stamp=sensors.get_clock().now().to_msg()
            msg=Odometry();msg.header.stamp=stamp;msg.header.frame_id='odom';msg.child_frame_id='base_footprint';msg.pose.pose.orientation.w=1.
            odom.publish(msg)
            transform=TransformStamped();transform.header=msg.header;transform.child_frame_id='base_footprint';transform.transform.rotation.w=1.;tf.sendTransform(transform)
            laser=LaserScan();laser.header=msg.header;laser.header.frame_id='base_footprint'
            laser.angle_min=-math.pi;laser.angle_max=math.pi-2*math.pi/720;laser.angle_increment=2*math.pi/720
            laser.range_min=.05;laser.range_max=6.;laser.ranges=ranges;scan.publish(laser)
            lift.publish(String(data='{"homed":true,"serial_connected":true,"fault":"NONE","state":"IDLE","position":0.0,"velocity":0.0}'))
        sensors.create_timer(.05,emit)
        stop=threading.Event()
        def spin():
            while not stop.is_set():executor.spin_once(timeout_sec=.02)
        thread=threading.Thread(target=spin);thread.start()
        logfile=open('/tmp/auto_localization_amcl_test.log','w')
        process=subprocess.Popen(['ros2','launch',str(ROOT/'launch/pallet_pickup_localization.launch.py')],
            stdout=logfile,stderr=subprocess.STDOUT,start_new_session=True)
        cycle=Cycle();cycle.local_pickup=True
        calls=[];original_call=cycle.call
        def record_call(typ,name,*args,**kwargs):
            calls.append(name)
            return original_call(typ,name,*args,**kwargs)
        cycle.call=record_call
        try:
            with tempfile.TemporaryDirectory() as temp:
                cycle.logs=Path(temp)
                try:cycle.auto_localize_map()
                except Exception:
                    report_file=cycle.logs/'auto_localization.json'
                    if report_file.exists():print(report_file.read_text(),flush=True)
                    raise
                self.assertFalse(commands)
                import json
                report=json.loads((cycle.logs/'auto_localization.json').read_text())
                self.assertTrue(report['passed'],report)
                self.assertEqual(report['stability_limits']['duration_s'],2.)
                self.assertEqual(report['stability_limits']['position_span_m'],.05)
                self.assertAlmostEqual(report['stability_limits']['heading_span_deg'],3.)
                self.assertEqual(report['stability_limits']['minimum_samples'],5)
                actual=np.array(report['pose'])
                self.assertLess(np.linalg.norm(actual[:2]-truth[:2]),.12)
                self.assertLess(abs(math.atan2(math.sin(actual[2]-truth[2]),math.cos(actual[2]-truth[2]))),math.radians(5))
                self.assertTrue(report['global_reset'])
                resets=calls.count('/reinitialize_global_localization')
                cycle.auto_localize_map()
                reused=json.loads((cycle.logs/'auto_localization.json').read_text())
                self.assertTrue(reused['passed'],reused)
                self.assertFalse(reused['global_reset'],reused)
                self.assertEqual(calls.count('/reinitialize_global_localization'),resets)
                self.assertFalse(commands)
        finally:
            cycle.stop_child('localization_health')
            cycle.destroy_node()
            os.killpg(process.pid,signal.SIGINT)
            try:process.wait(timeout=8)
            except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait()
            logfile.close();stop.set();executor.wake();thread.join(timeout=3)
            executor.shutdown();sensors.destroy_node();rclpy.shutdown()


if __name__=='__main__':unittest.main()
