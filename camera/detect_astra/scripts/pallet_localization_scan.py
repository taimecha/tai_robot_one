#!/usr/bin/env python3
"""Limit AMCL scan/particle workload without changing capture stamps or safety scan."""
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class LocalizationScan(Node):
    def __init__(self):
        super().__init__('pallet_localization_scan')
        self.last=-math.inf
        self.output=self.create_publisher(LaserScan,'/localization/scan',1)
        self.create_subscription(LaserScan,'/scan_filtered',self.receive,qos_profile_sensor_data)

    def receive(self,msg):
        now=time.monotonic();ros=self.get_clock().now().nanoseconds/1e9
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        if now-self.last<.25 or not -.05<=ros-stamp<=.4:return
        self.last=now
        self.output.publish(msg)


def main():
    rclpy.init();node=LocalizationScan()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
