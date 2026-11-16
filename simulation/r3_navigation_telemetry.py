#!/usr/bin/env python3
"""Passively record Nav2 commands and measured motion; publishes nothing."""
import argparse
import json
import math
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist,TwistStamped,PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


def finite(value):
    return float(value) if math.isfinite(value) else None


class NavigationTelemetry(Node):
    def __init__(self,output):
        super().__init__('r3_navigation_telemetry')
        output.parent.mkdir(parents=True,exist_ok=True)
        self.file=output.open('w',buffering=65536);self.latest={};self.ticks=0
        self.file.write(json.dumps(dict(schema='r3_navigation_motion_v1',
            observer_only=True,robot_commands_from_this_process=0))+'\n')
        for topic in ['/cmd_vel_nav','/cmd_vel_smoothed','/cmd_vel_safe']:
            self.create_subscription(Twist,topic,
                lambda m,t=topic:self.command(t,m),qos_profile_sensor_data)
        self.create_subscription(TwistStamped,'/base_controller/cmd_vel_out',
            lambda m:self.command('/base_controller/cmd_vel_out',m.twist,self.stamp(m)),qos_profile_sensor_data)
        for topic in ['/odom','/base_controller/odom']:
            self.create_subscription(Odometry,topic,
                lambda m,t=topic:self.odometry(t,m),qos_profile_sensor_data)
        self.create_subscription(Imu,'/imu',self.imu,qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped,'/amcl_pose',self.amcl,qos_profile_sensor_data)
        self.create_timer(.05,self.record)
    def now(self):return self.get_clock().now().nanoseconds*1e-9
    @staticmethod
    def stamp(m):return m.header.stamp.sec+m.header.stamp.nanosec*1e-9
    @staticmethod
    def yaw(q):return finite(math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
    def command(self,topic,msg,stamp=None):
        self.latest[topic]=dict(received_ros_s=self.now(),stamp_ros_s=stamp,
            vx=finite(msg.linear.x),wz=finite(msg.angular.z))
    def odometry(self,topic,msg):
        p=msg.pose.pose;t=msg.twist.twist
        self.latest[topic]=dict(stamp_ros_s=self.stamp(msg),x=finite(p.position.x),
            y=finite(p.position.y),yaw=self.yaw(p.orientation),vx=finite(t.linear.x),wz=finite(t.angular.z))
    def imu(self,msg):
        self.latest['/imu']=dict(stamp_ros_s=self.stamp(msg),yaw=self.yaw(msg.orientation),
            wz=finite(msg.angular_velocity.z))
    def amcl(self,msg):
        p=msg.pose.pose
        self.latest['/amcl_pose']=dict(stamp_ros_s=self.stamp(msg),x=finite(p.position.x),
            y=finite(p.position.y),yaw=self.yaw(p.orientation))
    def record(self):
        self.file.write(json.dumps(dict(ros_s=self.now(),**self.latest),allow_nan=False)+'\n')
        self.ticks+=1
        if self.ticks%20==0:self.file.flush()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True);args=parser.parse_args()
    rclpy.init();node=NavigationTelemetry(args.output)
    try:rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):pass
    finally:
        node.file.flush();node.file.close();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
