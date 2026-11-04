#!/usr/bin/env python3
"""Decode large AMCL clouds outside the safety supervisor; publish compact results.

Only localization/status subscriptions and a diagnostic publisher. No motion,
initial pose, lift, or AMCL reset/update commands are owned by this process.
"""
import json
import math
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,ReliabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.msg import ParticleCloud
from std_msgs.msg import String
from pallet_auto_localization import yaw,particle_issue


def seconds(stamp):return stamp.sec+stamp.nanosec*1e-9


class AMCLHealth(Node):
    def __init__(self):
        super().__init__('pallet_amcl_health')
        self.pose=None
        self.output=self.create_publisher(String,'/pallet/localization/health',1)
        self.create_subscription(PoseWithCovarianceStamped,'/amcl_pose',self.on_pose,1)
        self.create_subscription(ParticleCloud,'/particle_cloud',self.on_cloud,
            QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT))

    def on_pose(self,msg):self.pose=msg

    def on_cloud(self,msg):
        start=time.monotonic()
        summary={'valid':False,'reason':'Chờ pose AMCL cùng particle cloud'}
        try:
            pose_msg=self.pose
            if pose_msg is None:raise ValueError(summary['reason'])
            if msg.header.frame_id!='map' or pose_msg.header.frame_id!='map':
                raise ValueError('AMCL sai hệ tọa độ map')
            pose_stamp=seconds(pose_msg.header.stamp);cloud_stamp=seconds(msg.header.stamp)
            ros=self.get_clock().now().nanoseconds/1e9
            if not -.05<=ros-pose_stamp<=.8 or not -.05<=ros-cloud_stamp<=.8:
                raise ValueError('Pose/particle AMCL quá hạn trước giải mã')
            p=pose_msg.pose.pose
            pose=(p.position.x,p.position.y,yaw(p.orientation))
            data=np.empty((len(msg.particles),4),dtype=float)
            for i,v in enumerate(msg.particles):
                data[i]=(v.pose.position.x,v.pose.position.y,yaw(v.pose.orientation),v.weight)
            summary.update(valid=True,pose=list(pose),pose_stamp_s=pose_stamp,
                cloud_stamp_s=cloud_stamp,particle_count=len(data),
                coarse_issue=particle_issue(pose,pose_msg.pose.covariance,data,False),
                strict_issue=particle_issue(pose,pose_msg.pose.covariance,data,True))
        except (ValueError,TypeError,OverflowError) as exc:
            summary={'valid':False,'reason':str(exc)}
        summary.update(published_ros_s=self.get_clock().now().nanoseconds/1e9,
                       compute_s=time.monotonic()-start)
        self.output.publish(String(data=json.dumps(summary,ensure_ascii=False,allow_nan=False)))


def main():
    rclpy.init();node=AMCLHealth()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
