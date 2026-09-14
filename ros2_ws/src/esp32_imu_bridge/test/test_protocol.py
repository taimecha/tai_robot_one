# Copyright 2026 TAI Robot One contributors
# Licensed under the Apache License, Version 2.0

"""Unit tests for the serial record parser without opening a tty."""

from esp32_imu_bridge.esp32_bridge import ESP32ImuBridge, FrameError
import pytest


def make_parser():
    """Create only the state required by parse_frame."""
    parser = object.__new__(ESP32ImuBridge)
    parser.last_sequence = None
    parser.last_sample_ms = None
    parser.get_logger = lambda: None
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


def test_sequence_restarts_after_esp32_reboot():
    """Low sequence and millis are accepted after both counters roll back."""
    parser = make_parser()
    parser.parse_frame(
        'IMU,1,9000,200000,0,0,0,1,0,0,0,0,0,9.81,3,3,3,3')
    parser.get_logger = lambda: type(
        'Logger', (), {'warning': lambda self, message: None})()
    frame = parser.parse_frame(
        'IMU,1,1,2500,0,0,0,1,0,0,0,0,0,9.81,3,3,3,3')
    assert frame['orientation'] == [0.0, 0.0, 0.0, 1.0]
    assert parser.last_sequence == 1
    assert parser.last_sample_ms == 2500


def test_old_record_is_not_mistaken_for_reboot():
    """An old record outside the bounded boot window remains rejected."""
    parser = make_parser()
    parser.parse_frame(
        'IMU,1,9000,200000,0,0,0,1,0,0,0,0,0,9.81,3,3,3,3')
    with pytest.raises(FrameError, match='stale or repeated'):
        parser.parse_frame(
            'IMU,1,8000,150000,0,0,0,1,0,0,0,0,0,9.81,3,3,3,3')


def test_non_finite_value_is_rejected():
    """Non-finite values must never reach sensor_msgs/Imu."""
    with pytest.raises(FrameError, match='non-finite'):
        make_parser().parse_frame('0,0,nan,1,0,0,9.81')


def test_lift_frame_reports_active_low_home_switch():
    """A debounced LOW lower switch arrives as lower_active and homed."""
    frame = make_parser().parse_lift_frame(
        'LIFT,1,2500,IDLE,1,0.00000,0.00000,0.00000,1,0,NONE')
    assert frame['homed']
    assert frame['lower_active']
    assert not frame['upper_active']
    assert frame['position'] == 0.0


def test_lift_frame_rejects_invalid_switch_flag():
    """Only zero and one are accepted for physical switch telemetry."""
    with pytest.raises(FrameError, match='flag outside'):
        make_parser().parse_lift_frame(
            'LIFT,1,2500,IDLE,1,0,0,0,2,0,NONE')


def test_lift_frame_accepts_temporary_negative_homing_coordinate():
    """Homing remains observable before the lower switch establishes zero."""
    frame = make_parser().parse_lift_frame(
        'LIFT,1,2500,HOMING,0,-0.12000,0,-0.01000,0,0,NONE')
    assert frame['state'] == 'HOMING'
    assert frame['position'] == -0.12
