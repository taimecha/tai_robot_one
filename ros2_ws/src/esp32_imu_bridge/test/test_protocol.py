# Copyright 2026 TAI Robot One contributors
# Licensed under the Apache License, Version 2.0

"""Unit tests for the serial record parser without opening a tty."""

from esp32_imu_bridge.esp32_bridge import ESP32ImuBridge, FrameError
import pytest


def make_parser():
    """Create only the state required by parse_frame."""
    parser = object.__new__(ESP32ImuBridge)
    parser.last_sequence = None
    return parser


def test_tagged_frame_contains_all_imu_channels():
    """The production frame carries quaternion, gyro and acceleration."""
    parser = make_parser()
    frame = parser.parse_frame(
        'IMU,1,42,1000,0,0,0,1,0.1,0.2,0.3,1,2,9.81,3,3,2,1')
    assert frame['orientation'] == [0.0, 0.0, 0.0, 1.0]
    assert frame['gyro'] == [0.1, 0.2, 0.3]
    assert frame['accel'] == [1.0, 2.0, 9.81]
    assert frame['calibration'] == [3, 3, 2, 1]


def test_legacy_frame_marks_gyro_unavailable():
    """Seven-field GitHub input remains readable without inventing gyro."""
    frame = make_parser().parse_frame('0,0,0,1,0,0,9.81')
    assert not frame['gyro_available']
    assert frame['gyro'] == [0.0, 0.0, 0.0]


def test_replayed_sequence_is_rejected():
    """A repeated tagged frame must not refresh downstream sensor data."""
    parser = make_parser()
    line = 'IMU,1,7,1000,0,0,0,1,0,0,0,0,0,9.81,3,3,3,3'
    parser.parse_frame(line)
    with pytest.raises(FrameError, match='stale or repeated'):
        parser.parse_frame(line)


def test_non_finite_value_is_rejected():
    """Non-finite values must never reach sensor_msgs/Imu."""
    with pytest.raises(FrameError, match='non-finite'):
        make_parser().parse_frame('0,0,nan,1,0,0,9.81')
