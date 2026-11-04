"""Real Nav2 → staging → Nav2 handoff with fake sensors in ROS domain 232.

No hardware launch, serial devices, movement requests or physical ROS domain.
"""
import json
import os
from pathlib import Path
import signal
import tempfile
import time
import unittest

os.environ['ROS_DOMAIN_ID']='232'
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE']='LOCALHOST'
import rclpy
from rclpy.qos import QoSProfile,DurabilityPolicy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid,Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster
from run_pallet_cycle import Cycle,ROOT,navigation_processes,process
from pallet_cpu_budget import CpuBudget


class HandoffTest(unittest.TestCase):
    def test_control_handoff_keeps_existing_sensors_and_localization(self):
        rclpy.init();cycle=Cycle();cycle.budget=CpuBudget()
        latched=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        odom=cycle.create_publisher(Odometry,'/odom',1)
        scan=cycle.create_publisher(LaserScan,'/scan_filtered',1)
        camera=cycle.create_publisher(LaserScan,'/camera/obstacle_scan',1)
        lift=cycle.create_publisher(String,'/lift/status',1)
        grid=cycle.create_publisher(OccupancyGrid,'/map',latched)
        tf=TransformBroadcaster(cycle)
        def sensors():
            stamp=cycle.get_clock().now().to_msg()
            o=Odometry();o.header.stamp=stamp;o.header.frame_id='odom';o.child_frame_id='base_footprint'
            o.pose.pose.orientation.w=1.;odom.publish(o)
            transforms=[]
            for parent,child in [('map','odom'),('odom','base_footprint')]:
                t=TransformStamped();t.header.stamp=stamp;t.header.frame_id=parent;t.child_frame_id=child
                t.transform.rotation.w=1.;transforms.append(t)
            tf.sendTransform(transforms)
            s=LaserScan();s.header=o.header;s.header.frame_id='base_footprint'
            s.angle_min=-3.14159265;s.angle_max=3.14159265;s.angle_increment=.0174532925
            s.range_min=.05;s.range_max=10.;s.ranges=[3.5]*361
            scan.publish(s);camera.publish(s)
            lift.publish(String(data=json.dumps(dict(serial_connected=True,homed=True,fault='NONE',
                state='IDLE',position=0.,velocity=0.,sample_ms=int(time.monotonic()*1000)%2**32,
                target=0.,upper_active=False,lower_active=True))))
        def map_input():
            m=OccupancyGrid();m.header.frame_id='map';m.header.stamp=cycle.get_clock().now().to_msg()
            m.info.resolution=.05;m.info.width=m.info.height=160
            m.info.origin.position.x=m.info.origin.position.y=-4.;m.info.origin.orientation.w=1.
            m.data=[0]*25600;grid.publish(m)
        timers=[cycle.create_timer(.05,sensors),cycle.create_timer(.5,map_input)]
        with tempfile.TemporaryDirectory() as temp:
            cycle.logs=Path(temp)
            try:
                cycle.wait(lambda:cycle.still() and 'lift' in cycle.states,5.,'fake sensors')
                cycle.ensure_navigation()
                nav=navigation_processes()
                self.assertEqual(len(nav),8,nav)
                # Domain filtering must keep every live hardware/Nav2 PID out.
                self.assertTrue(all(pid!=135935 for pid,_,_ in nav))
                cycle.stop_navigation()
                self.assertTrue(all(process(pid)[0]!=args for pid,_,args in nav))
                cycle.sensors(empty=True)
                cycle.lease=True
                cycle.start('staging',['ros2','launch',str(ROOT/'launch/pallet_alignment_existing.launch.py')])
                cycle.wait(lambda:cycle.command_owner('pallet_alignment_collision_monitor'),20.,
                    'dedicated monitor',('staging',))
                cycle.wait(lambda:cycle.fresh('alignment') is not None and
                    cycle.fresh('handling') is not None,12.,'handler status',('staging',))
                if cycle.budget.enabled:
                    stage_nodes=[]
                    for p in Path('/proc').iterdir():
                        if not p.name.isdecimal():continue
                        args,_=process(int(p.name))
                        if '__ns:=/pallet_nav' in args and any('__node:='+name in args for name in ('planner_server','controller_server')):
                            from run_pallet_cycle import process_domain
                            if process_domain(int(p.name))=='232':stage_nodes.append(int(p.name))
                    self.assertEqual(len(stage_nodes),2)
                    self.assertTrue(all(os.sched_getaffinity(pid)==cycle.budget.compute for pid in stage_nodes))
                # Both volatile alignment and retained handling status must arrive.
                self.assertEqual(cycle.fresh('handling')['state'],'IDLE')
                cycle.sensors(empty=True)
                cycle.stop_child('staging');cycle.lease=False
                cycle.wait(lambda:not cycle.get_publishers_info_by_topic('/cmd_vel_safe') and
                    not cycle.get_publishers_info_by_topic('/lift_controller/commands'),10.,'remove stage sources')
                cycle.ensure_navigation()
                cycle.sensors(empty=True)
                names=set(cycle.get_node_names())
                for forbidden in ('ros2_control_node','esp32_imu_bridge','robot_state_publisher',
                                  'spatial_depth_cloud','amcl','map_server','sllidar_node'):
                    self.assertNotIn(forbidden,names)
                self.assertTrue(cycle.command_owner('collision_monitor'))
                self.assertTrue(cycle.still())
            except Exception:
                for log in cycle.logs.glob('*.log'):
                    print(log.name,log.read_text()[-6000:])
                raise
            finally:
                cycle.lease=False
                for name in ('staging','navigation'):
                    try:cycle.stop_child(name)
                    except Exception:pass
                for timer in timers:cycle.destroy_timer(timer)
                cycle.budget.restore()
                cycle.destroy_node();rclpy.shutdown()


if __name__=='__main__':unittest.main()
