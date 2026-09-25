"""Smoke tests for the irsim_devices standalone package."""

import pytest


def test_motor_import():
    from irsim_devices.actuators import Motor

    m = Motor(profile="small_dc")
    assert m.command_mode == "pwm"


def test_motor_step():
    from irsim_devices.actuators import Motor

    m = Motor(profile="small_dc")
    omega = m.step(0.5, dt=0.05)
    assert omega >= 0.0


def test_motor_diff_chassis_import():
    from irsim_devices.actuators import MotorDiffChassis

    chassis = MotorDiffChassis(wheel_radius=0.05, wheel_base=0.30)
    assert chassis is not None


def test_chassis_step():
    from irsim_devices.actuators import MotorDiffChassis

    chassis = MotorDiffChassis(wheel_radius=0.05, wheel_base=0.30)
    state = chassis.step([0.5, 0.5], dt=0.05)
    assert state.shape == (3,)


def test_imu_import():
    from irsim_devices.sensors import IMU

    imu = IMU()
    assert imu.sensor_type == "imu"


def test_encoder_import():
    from irsim_devices.sensors import Encoder

    enc = Encoder(profile="dynamixel_xl430")
    assert enc.encoder_cpr == 4096


def test_encoder_reads_chassis():
    from irsim_devices.actuators import MotorDiffChassis
    from irsim_devices.sensors import Encoder

    chassis = MotorDiffChassis(motor_profile="dynamixel_xl430")
    encoder = Encoder(profile="dynamixel_xl430")
    encoder.parent = chassis

    chassis.step([0.6, 0.6], dt=0.01)
    encoder.step(chassis.state)
    readings = encoder.get_measurement()

    assert set(readings) == {"left", "right"}
    assert "ticks" in readings["left"]


def test_geo_utils():
    import math

    from irsim_devices.core import ClipTo2Pi

    assert ClipTo2Pi(math.pi) == pytest.approx(math.pi)
    assert ClipTo2Pi(3 * math.pi) == pytest.approx(2 * math.pi)


def test_random_utils():
    from irsim_devices.core import rng, set_seed

    set_seed(42)
    a = rng.uniform(0, 1)
    set_seed(42)
    b = rng.uniform(0, 1)
    assert a == pytest.approx(b)
