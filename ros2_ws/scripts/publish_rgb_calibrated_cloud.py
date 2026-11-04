#!/usr/bin/env python3
"""Registered Astra depth cloud using calibrated RGB rays.

This assumes depth_registration maps depth pixels to the raw RGB image. It does
not correct an absolute Depth range bias or a remaining RGB/Depth pixel offset.
An optional inverse-depth x slope can preview a spatial sensor-skew correction.
"""

import time
import json
from collections import deque
from copy import deepcopy

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener, TransformException
from floor_surface_model import FloorSurfaceModel


def rotation_matrix(quaternion):
    q = np.asarray(quaternion, dtype=float)
    if q.shape != (4,) or not np.isfinite(q).all() or abs(np.linalg.norm(q)-1) > .001:
        raise ValueError("Camera transform quaternion must be finite and normalized")
    x, y, z, w = q
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ])


def floor_relative_cloud(xyz, rotation, translation, coefficients):
    """Correct height against a fixed surveyed floor; preserve planar X/Y.

    The recorded floor is z=a*x+b*y+c in the reference frame. Signed plane
    distance removes its residual tilt/offset from obstacle heights. No plane
    is learned from incoming obstacles, and robot/camera TF stays unchanged.
    """
    points = xyz @ rotation.T + translation
    a, b, c = coefficients
    points[..., 2] = (points[..., 2] - a*points[..., 0] - b*points[..., 1] - c) / np.sqrt(1+a*a+b*b)
    return points.astype(np.float32)


class CalibratedCloud(Node):
    def __init__(self):
        super().__init__("rgb_calibrated_cloud")
        self.declare_parameter("pixel_stride", 2)
        self.declare_parameter("max_rate_hz", 10.0)
        self.declare_parameter("inverse_depth_x_slope", 0.0)
        self.declare_parameter("reference_u", 370.0)
        self.declare_parameter("output_topic", "/camera/depth/points_rgb_calibrated")
        self.declare_parameter("floor_height_correction_enabled", False)
        self.declare_parameter("height_reference_frame", "base_footprint")
        self.declare_parameter("floor_plane_coefficients", [0.0, 0.0, 0.0])
        self.declare_parameter("floor_surface_tracking_enabled", False)
        self.stride = int(self.get_parameter("pixel_stride").value)
        self.max_rate_hz = float(self.get_parameter("max_rate_hz").value)
        self.inverse_depth_x_slope = float(self.get_parameter("inverse_depth_x_slope").value)
        self.reference_u = float(self.get_parameter("reference_u").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.correct_height = bool(self.get_parameter("floor_height_correction_enabled").value)
        self.height_frame = str(self.get_parameter("height_reference_frame").value)
        self.floor_plane = np.asarray(self.get_parameter("floor_plane_coefficients").value, dtype=float)
        self.floor_surface = (FloorSurfaceModel() if self.correct_height and
            self.get_parameter('floor_surface_tracking_enabled').value else None)
        self.quiet_odom = deque(maxlen=90)
        self.odom_received = 0.
        self.odom_stamp_s = 0.
        if self.floor_surface:
            if self.height_frame != 'base_footprint':
                raise ValueError('Floor surface tracking requires base_footprint')
            self.create_subscription(Odometry,'/odom',self.on_odom,qos_profile_sensor_data)
            self.floor_status = self.create_publisher(String,'/camera/floor_height/status',1)
            self.last_floor_status = 0.
        if self.floor_plane.shape != (3,) or not np.isfinite(self.floor_plane).all():
            raise ValueError("floor_plane_coefficients must contain three finite values")
        if self.correct_height and (not self.height_frame or np.linalg.norm(self.floor_plane[:2]) > .2 or abs(self.floor_plane[2]) > .2):
            raise ValueError("Floor correction exceeds the supported small residual calibration")
        self.tf_buffer = Buffer() if self.correct_height else None
        self.tf_listener = TransformListener(self.tf_buffer, self) if self.correct_height else None
        if self.stride < 1 or self.max_rate_hz <= 0:
            raise ValueError("pixel_stride and max_rate_hz must be positive")
        if not np.isfinite(self.inverse_depth_x_slope) or not np.isfinite(self.reference_u):
            raise ValueError("inverse_depth_x_slope and reference_u must be finite")

        self.rays = None
        self.u_coords = None
        self.info_key = None
        self.frame_id = None
        self.last_publish = 0.0
        self.next_publish = 0.0
        self.publisher = self.create_publisher(
            PointCloud2, self.output_topic, qos_profile_sensor_data
        )
        self.info_sub = self.create_subscription(
            CameraInfo, "/camera/color/camera_info", self.on_info, qos_profile_sensor_data
        )
        self.depth_sub = self.create_subscription(
            Image, "/camera/depth/image_raw", self.on_depth, qos_profile_sensor_data
        )

    def on_info(self, msg):
        key = (msg.width, msg.height, tuple(msg.k), tuple(msg.d), msg.header.frame_id)
        if key == self.info_key:
            return
        if msg.width == 0 or msg.height == 0 or msg.k[0] <= 0 or msg.k[4] <= 0:
            self.get_logger().warn("Waiting for calibrated RGB CameraInfo")
            return
        if self.info_key is not None and self.floor_surface:
            self.floor_surface = FloorSurfaceModel()

        k = np.asarray(msg.k, dtype=np.float64).reshape(3, 3)
        d = np.asarray(msg.d, dtype=np.float64)
        u, v = np.meshgrid(
            np.arange(0, msg.width, self.stride),
            np.arange(0, msg.height, self.stride),
        )
        pixels = np.stack((u, v), axis=-1).astype(np.float32).reshape(-1, 1, 2)
        self.rays = cv2.undistortPoints(pixels, k, d).reshape(u.shape + (2,)).astype(np.float32)
        self.u_coords = u.astype(np.float32)
        self.info_key = key
        self.frame_id = msg.header.frame_id
        self.get_logger().info(
            f"RGB rays ready: {msg.width}x{msg.height}, fx={msg.k[0]:.2f}, "
            f"fy={msg.k[4]:.2f}, stride={self.stride}"
        )

    def on_odom(self,msg):
        now = time.monotonic()
        self.odom_received = now
        self.odom_stamp_s = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        self.quiet_odom.append((now,abs(msg.twist.twist.linear.x)<=.003
                               and abs(msg.twist.twist.angular.z)<=.015))

    def stationary_for_floor(self,now):
        recent = [(t,ok) for t,ok in self.quiet_odom if now-t<=1.2]
        age = self.get_clock().now().nanoseconds/1e9-self.odom_stamp_s
        return (now-self.odom_received<=.2 and -.05<=age<=.2 and bool(recent)
                and recent[-1][0]-recent[0][0]>=.8
                and all(ok for _,ok in recent)
                and all(b[0]-a[0]<=.2 for a,b in zip(recent,recent[1:])))

    def on_depth(self, msg):
        if self.rays is None:
            return
        if (msg.width, msg.height) != self.info_key[:2] or msg.header.frame_id != self.frame_id:
            self.get_logger().warn("Depth size/frame differs from RGB calibration", throttle_duration_sec=5.0)
            return
        now = time.monotonic()
        if now < self.next_publish:
            return

        if msg.encoding == "16UC1":
            dtype, scale = np.dtype("<u2"), 0.001
        elif msg.encoding == "32FC1":
            dtype, scale = np.dtype("<f4"), 1.0
        else:
            self.get_logger().warn(f"Unsupported depth encoding: {msg.encoding}", throttle_duration_sec=5.0)
            return
        row_width = msg.step // dtype.itemsize
        depth = np.frombuffer(msg.data, dtype=dtype).reshape(msg.height, row_width)[:, : msg.width]
        z = depth[:: self.stride, :: self.stride].astype(np.float32) * scale
        if self.inverse_depth_x_slope != 0.0:
            valid = np.isfinite(z) & (z > 0) & (z <= 10)
            with np.errstate(divide="ignore", invalid="ignore"):
                inverse_z = np.where(valid, 1.0 / z, np.nan)
                inverse_z += self.inverse_depth_x_slope * (self.reference_u - self.u_coords)
                z = np.where(inverse_z > 0, 1.0 / inverse_z, np.nan).astype(np.float32)
        xyz = np.empty(z.shape + (3,), dtype=np.float32)
        xyz[:, :, 0] = self.rays[:, :, 0] * z
        xyz[:, :, 1] = self.rays[:, :, 1] * z
        xyz[:, :, 2] = z
        xyz[~np.isfinite(z) | (z <= 0) | (z > 10)] = np.nan

        header = deepcopy(msg.header)
        if self.correct_height:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.height_frame, msg.header.frame_id,
                    rclpy.time.Time.from_msg(msg.header.stamp))
            except TransformException:
                self.get_logger().warn("Waiting for camera-to-height-reference TF; cloud withheld", throttle_duration_sec=5.0)
                return
            t, q = transform.transform.translation, transform.transform.rotation
            xyz = floor_relative_cloud(
                xyz, rotation_matrix([q.x, q.y, q.z, q.w]),
                np.array([t.x, t.y, t.z]), self.floor_plane)
            header.frame_id = self.height_frame
            if self.floor_surface:
                capture_age = self.get_clock().now().nanoseconds/1e9-msg.header.stamp.sec-msg.header.stamp.nanosec*1e-9
                self.floor_surface.observe(xyz,now,
                    self.stationary_for_floor(now) and -.05<=capture_age<=.25)
                xyz = self.floor_surface.apply(xyz)
                if now-self.last_floor_status>=1.:
                    self.floor_status.publish(String(data=json.dumps(self.floor_surface.status(),ensure_ascii=False)))
                    self.last_floor_status = now
                if self.floor_surface.plane is None:
                    # An unqualified baseline must not mark startup floor
                    # ghosts as obstacles. No output also lets the existing
                    # camera-source timeout keep motion stopped until ready.
                    self.next_publish = now+1.0/self.max_rate_hz
                    return

        cloud = PointCloud2()
        cloud.header = header
        cloud.height, cloud.width = z.shape
        cloud.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.is_bigendian = False
        cloud.point_step = 12
        cloud.row_step = cloud.width * cloud.point_step
        cloud.is_dense = False
        cloud.data = xyz.tobytes()
        self.publisher.publish(cloud)
        self.last_publish = now
        interval = 1.0 / self.max_rate_hz
        if self.next_publish == 0.0:
            self.next_publish = now + interval
        else:
            self.next_publish += interval
            if self.next_publish <= now:
                self.next_publish = now + interval


def main():
    rclpy.init()
    node = CalibratedCloud()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
