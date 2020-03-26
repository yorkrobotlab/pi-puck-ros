#!/usr/bin/env python
"""Calibrate the magnetometer on the LSM9DS1."""

import math
from argparse import ArgumentDefaultsHelpFormatter, ArgumentParser
from json import dump
from os import path
from sys import exit
from time import sleep, time

from lsm9ds1 import LSM9DS1
from smbus import SMBus

# Constants
I2C_CHANNEL = 4
EPUCK_I2C_ADDR = 0x1e

LEFT_MOTOR_SPEED = 2
RIGHT_MOTOR_SPEED = 3
LEFT_MOTOR_STEPS = 4
RIGHT_MOTOR_STEPS = 5

MAX_SPEED = 500


def convert_speed(x):
    """Convert input speed data into a speed value for the stepper motors."""
    x = float(x)
    if x > 1.0:
        x = 1.0
    elif x < -1.0:
        x = -1.0
    return int(x * MAX_SPEED)


def set_left_speed(bus, speed):
    """Set left motor speed."""
    bus.write_word_data(EPUCK_I2C_ADDR, LEFT_MOTOR_SPEED, convert_speed(speed))


def set_right_speed(bus, speed):
    """Set right motor speed."""
    bus.write_word_data(EPUCK_I2C_ADDR, RIGHT_MOTOR_SPEED, convert_speed(speed))


def get_left_steps(bus, offset=0):
    """Get left motor steps."""
    return int(bus.read_word_data(EPUCK_I2C_ADDR, LEFT_MOTOR_STEPS)) + offset


def calibrate(samples, interval, run_motors=True, just_magnetometer=False):
    """Run calibration."""
    magnetometer_sample_values = []

    if run_motors:
        bus = SMBus(I2C_CHANNEL)
        start_left_steps = get_left_steps(bus)
        right_turn = 1000
        left_turn = 3000

    sensor = LSM9DS1()

    time_of_last_sample = time()

    if run_motors:
        turning_left = False
        turning_right = False

    while len(magnetometer_sample_values) < samples:
        current_time = time()

        # If time to take a sample
        if time_of_last_sample + interval >= current_time:
            time_of_last_sample = current_time
            magnetometer_sample_values.append(sensor.magnetic)

        if run_motors:
            if not turning_left and not turning_right:
                turning_left = True
                set_left_speed(bus, 1)
                set_right_speed(bus, -1)
            elif turning_left and get_left_steps(bus, offset=2000) - start_left_steps > left_turn:
                turning_right = True
                turning_left = False
                set_left_speed(bus, -1)
                set_right_speed(bus, 1)
            elif turning_right and get_left_steps(bus, offset=2000) - start_left_steps < right_turn:
                turning_right = False
                turning_left = True
                set_left_speed(bus, 1)
                set_right_speed(bus, -1)

    if run_motors:
        set_left_speed(bus, 0)
        set_right_speed(bus, 0)

        bus.close()

    magnetometer_x_sum = sum(map(lambda a: a[0], magnetometer_sample_values))
    magnetometer_y_sum = sum(map(lambda a: a[1], magnetometer_sample_values))
    magnetometer_z_sum = sum(map(lambda a: a[2], magnetometer_sample_values))

    magnetometer_count = float(len(magnetometer_sample_values))

    del magnetometer_sample_values

    if just_magnetometer:
        return {
            "magnetometer": {
                "x": magnetometer_x_sum / magnetometer_count,
                "y": magnetometer_y_sum / magnetometer_count,
                "z": magnetometer_z_sum / magnetometer_count
            },
        }

    accelerometer_sample_values = []
    gyro_sample_values = []

    while len(accelerometer_sample_values) < samples:
        accelerometer_sample_values.append(sensor.acceleration)
        gyro_sample_values.append(map(lambda deg: deg * (math.pi / 180.0), sensor.gyro))

    accelerometer_x_sum = sum(map(lambda a: a[0], accelerometer_sample_values))
    accelerometer_y_sum = sum(map(lambda a: a[1], accelerometer_sample_values))
    accelerometer_z_sum = sum(map(lambda a: a[2], accelerometer_sample_values))

    gyro_x_sum = sum(map(lambda a: a[0], gyro_sample_values))
    gyro_y_sum = sum(map(lambda a: a[1], gyro_sample_values))
    gyro_z_sum = sum(map(lambda a: a[2], gyro_sample_values))

    sensor.close()

    samples = float(samples)

    return {
        "magnetometer": {
            "x": magnetometer_x_sum / magnetometer_count,
            "y": magnetometer_y_sum / magnetometer_count,
            "z": magnetometer_z_sum / magnetometer_count
        },
        "accelerometer": {
            "x": accelerometer_x_sum / samples,
            "y": accelerometer_y_sum / samples,
            "z": (accelerometer_z_sum / samples) - 9.80665
        },
        "gyro": {
            "x": gyro_x_sum / samples,
            "y": gyro_y_sum / samples,
            "z": gyro_z_sum / samples
        }
    }


def main():
    """Application entry point."""
    argument_parser = ArgumentParser(formatter_class=ArgumentDefaultsHelpFormatter)

    argument_parser.add_argument("-s",
                                 "--samples",
                                 default=8000,
                                 type=int,
                                 help="Number of samples to take.")
    argument_parser.add_argument("-i",
                                 "--interval",
                                 default=0.05,
                                 type=float,
                                 help="Interval in seconds between samples.")
    argument_parser.add_argument("-o", "--output", default="./", help="Output location/file name.")
    argument_parser.add_argument("-n", "--no-motors", action="store_true", help="Don't run motors.")
    argument_parser.add_argument("-m",
                                 "--only-magnetometer",
                                 action="store_true",
                                 help="Calibrate only the magnetometer.")

    parsed_args = argument_parser.parse_args()

    output_path = path.expanduser(parsed_args.output)
    output_path = path.abspath(output_path)

    if path.isdir(output_path):
        output_path = path.join(output_path, "calibration.json")

    if not path.isdir(path.dirname(output_path)):
        print("Output directory doesn't exist.")
        return 1

    interval = parsed_args.interval

    if interval <= 0:
        print("Interval must be positive and nonzero")
        return 2

    samples = parsed_args.samples

    if samples <= 0:
        print("Samples must be positive and nonzero")
        return 3

    if samples > 100000:
        print("Warning: overly large sample sizes may cause instability")

    print("Calibrating...")

    calibration_result = calibrate(samples,
                                   interval,
                                   run_motors=not parsed_args.no_motors,
                                   just_magnetometer=parsed_args.only_magnetometer)
    with open(output_path, "wb") as output_file_handle:
        dump(calibration_result, output_file_handle)

    print("Done.")

    return 0


if __name__ == "__main__":
    exit(main())
