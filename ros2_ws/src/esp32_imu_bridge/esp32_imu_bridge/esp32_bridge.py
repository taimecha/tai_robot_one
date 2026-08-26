#!/usr/bin/env python3

# Copyright 2026 TAI Robot One contributors
# Licensed under the Apache License, Version 2.0

"""Publish validated BNO055 frames received from an ESP32 over USB."""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
import serial
from serial import SerialException


class FrameError(ValueError):
    """Raised when a complete serial record is not a valid IMU frame."""


class ESP32ImuBridge(Node):
    """Reconnect a configurable serial IMU and publish sensor_msgs/Imu."""

    def __init__(self):
        super().__init__('esp32_imu_bridge')
        self.declare_parameter('port', '/dev/tai_imu')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('topic', '/imu/data')
        self.declare_parameter('reconnect_interval', 2.0)
        self.declare_parameter('diagnostic_interval', 5.0)
        self.declare_parameter('orientation_variance', 0.0004)
        self.declare_parameter('angular_velocity_variance', 0.0004)
        self.declare_parameter('linear_acceleration_variance', 0.04)
        self.declare_parameter('max_lines_per_cycle', 20)

        self.port = str(self.get_parameter('port').value)
        self.baud = int(self.get_parameter('baud').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.reconnect_interval = float(
            self.get_parameter('reconnect_interval').value)
        self.diagnostic_interval = float(
            self.get_parameter('diagnostic_interval').value)
        self.max_lines_per_cycle = int(
            self.get_parameter('max_lines_per_cycle').value)
        topic = str(self.get_parameter('topic').value)

        if self.baud <= 0 or self.reconnect_interval <= 0.0:
            raise ValueError('baud and reconnect_interval must be positive')
        if self.max_lines_per_cycle < 1:
            raise ValueError('max_lines_per_cycle must be at least one')

        self.publisher = self.create_publisher(
            Imu, topic, qos_profile_sensor_data)
        self.serial_port = None
        self.receive_buffer = bytearray()
        self.last_connect_attempt = 0.0
        self.last_diagnostic = 0.0
        self.last_sequence = None
        self.published_count = 0
        self.rejected_count = 0
        self.create_timer(0.005, self.poll_serial)
        self.get_logger().info(
            f'IMU bridge configured: {self.port} at {self.baud} baud -> '
            f'{topic}')

    def close_serial(self):
        """Close the current file descriptor and reset stream state."""
        if self.serial_port is not None:
            try:
                self.serial_port.close()
            except SerialException:
                pass
        self.serial_port = None
        self.receive_buffer.clear()
        self.last_sequence = None

    def try_connect(self):
        """Open the configured port at a bounded retry rate."""
        now = time.monotonic()
        if now - self.last_connect_attempt < self.reconnect_interval:
            return
        self.last_connect_attempt = now
        try:
            self.serial_port = serial.Serial(
                port=self.port,
                baudrate=self.baud,
                timeout=0,
                exclusive=True,
            )
            self.serial_port.reset_input_buffer()
            self.get_logger().info(f'Connected to IMU on {self.port}')
        except (SerialException, OSError) as error:
            self.serial_port = None
            self.log_throttled('warning', f'Cannot open {self.port}: {error}')

    def log_throttled(self, level, message):
        """Avoid filling logs when a cable is absent or data is malformed."""
        now = time.monotonic()
        if now - self.last_diagnostic < self.diagnostic_interval:
            return
        self.last_diagnostic = now
        getattr(self.get_logger(), level)(message)

    @staticmethod
    def parse_float_fields(fields):
        """Convert numeric fields and reject NaN/Inf before publishing."""
        values = [float(value) for value in fields]
        if not all(math.isfinite(value) for value in values):
            raise FrameError('non-finite value')
        return values

    @staticmethod
    def sequence_is_new(sequence, previous):
        """Compare wrapping uint32 sequence numbers."""
        if previous is None:
            return True
        difference = (sequence - previous) & 0xFFFFFFFF
        return 0 < difference < 0x80000000

    def parse_frame(self, line):
        """Parse the tagged v1 protocol and the seven-field legacy format."""
        fields = line.split(',')
        if fields[0] == 'IMU':
            if len(fields) != 18 or fields[1] != '1':
                raise FrameError('unsupported tagged frame')
            try:
                sequence = int(fields[2])
                sample_ms = int(fields[3])
                calibration = [int(value) for value in fields[14:18]]
            except ValueError as error:
                raise FrameError('invalid integer field') from error
            if not 0 <= sequence <= 0xFFFFFFFF or sample_ms < 0:
                raise FrameError('integer outside protocol range')
            if not all(0 <= value <= 3 for value in calibration):
                raise FrameError('calibration outside 0..3')
            if not self.sequence_is_new(sequence, self.last_sequence):
                raise FrameError('stale or repeated sequence')
            values = self.parse_float_fields(fields[4:14])
            self.last_sequence = sequence
            return {
                'orientation': values[0:4],
                'gyro': values[4:7],
                'accel': values[7:10],
                'gyro_available': True,
                'calibration': calibration,
            }

        if len(fields) == 7:
            values = self.parse_float_fields(fields)
            return {
                'orientation': values[0:4],
                'gyro': [0.0, 0.0, 0.0],
                'accel': values[4:7],
                'gyro_available': False,
                'calibration': None,
            }
        raise FrameError('not an IMU record')

    def publish_frame(self, frame):
        """Normalize the quaternion, attach covariances and publish."""
        qx, qy, qz, qw = frame['orientation']
        norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if not 0.5 <= norm <= 1.5:
            raise FrameError(f'invalid quaternion norm {norm:.3f}')

        message = Imu()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.frame_id
        message.orientation.x = qx / norm
        message.orientation.y = qy / norm
        message.orientation.z = qz / norm
        message.orientation.w = qw / norm
        message.angular_velocity.x = frame['gyro'][0]
        message.angular_velocity.y = frame['gyro'][1]
        message.angular_velocity.z = frame['gyro'][2]
        message.linear_acceleration.x = frame['accel'][0]
        message.linear_acceleration.y = frame['accel'][1]
        message.linear_acceleration.z = frame['accel'][2]

        orientation_variance = float(
            self.get_parameter('orientation_variance').value)
        angular_variance = float(
            self.get_parameter('angular_velocity_variance').value)
        acceleration_variance = float(
            self.get_parameter('linear_acceleration_variance').value)
        for covariance, variance in (
                (message.orientation_covariance, orientation_variance),
                (message.linear_acceleration_covariance,
                 acceleration_variance)):
            covariance[0] = variance
            covariance[4] = variance
            covariance[8] = variance
        if frame['gyro_available']:
            message.angular_velocity_covariance[0] = angular_variance
            message.angular_velocity_covariance[4] = angular_variance
            message.angular_velocity_covariance[8] = angular_variance
        else:
            # REP-145: -1 means that this measurement is not provided.
            message.angular_velocity_covariance[0] = -1.0

        self.publisher.publish(message)
        self.published_count += 1

    def handle_line(self, raw_line):
        """Handle diagnostics separately so they can never become IMU data."""
        try:
            line = raw_line.decode('ascii', errors='strict').strip()
        except UnicodeDecodeError as error:
            raise FrameError('non-ASCII record') from error
        if not line:
            return
        if line.startswith(('BOOT,', 'STATUS:')):
            self.log_throttled('info', f'ESP32: {line}')
            return
        self.publish_frame(self.parse_frame(line))

    def poll_serial(self):
        """Read complete newline-delimited frames without blocking ROS."""
        if self.serial_port is None:
            self.try_connect()
            return
        try:
            available = self.serial_port.in_waiting
            if available:
                self.receive_buffer.extend(self.serial_port.read(available))
            if len(self.receive_buffer) > 8192:
                raise FrameError('receive buffer exceeded 8192 bytes')

            processed = 0
            while b'\n' in self.receive_buffer:
                raw_line, _, remainder = self.receive_buffer.partition(b'\n')
                self.receive_buffer = bytearray(remainder)
                try:
                    self.handle_line(raw_line.rstrip(b'\r'))
                except (FrameError, ValueError) as error:
                    self.rejected_count += 1
                    self.log_throttled(
                        'warning',
                        f'Rejected IMU record: {error}; total rejected='
                        f'{self.rejected_count}')
                processed += 1
                if processed >= self.max_lines_per_cycle:
                    break
        except (SerialException, OSError, FrameError) as error:
            self.get_logger().error(f'IMU serial disconnected: {error}')
            self.close_serial()

    def destroy_node(self):
        """Release the serial port before ROS destroys the node."""
        self.close_serial()
        return super().destroy_node()


def main(args=None):
    """Run the ROS 2 IMU serial bridge."""
    rclpy.init(args=args)
    node = ESP32ImuBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
