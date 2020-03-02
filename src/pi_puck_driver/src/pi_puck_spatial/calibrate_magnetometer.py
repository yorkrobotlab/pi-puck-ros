#!/usr/bin/env python
"""Calibrate the magnetometer on the LSM9DS1."""

from argparse import ArgumentParser, ArgumentDefaultsHelpFormatter
from sys import exit
from json import dump
from os import path
from time import time, sleep

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
    """get left motor steps."""
    return int(bus.read_word_data(EPUCK_I2C_ADDR, LEFT_MOTOR_STEPS)) + offset


def calibrate(samples, interval, run_motors=True):
    """Run calibration."""
    sample_values = []

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

    while len(sample_values) < samples:
        current_time = time()

        # If time to take a sample
        if time_of_last_sample + interval >= current_time:
            time_of_last_sample = current_time
            sample_values.append(sensor.magnetic)

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

    sensor.close()
    bus.close()

    x_sum = sum(map(lambda a: a[0], sample_values))
    y_sum = sum(map(lambda a: a[1], sample_values))
    z_sum = sum(map(lambda a: a[2], sample_values))

    count = float(len(sample_values))

    return {"x": x_sum / count, "y": y_sum / count, "z": z_sum / count}


def main():
    """Application entry point."""
    argument_parser = ArgumentParser(formatter_class=ArgumentDefaultsHelpFormatter)

    argument_parser.add_argument("--samples", default=8000, type=int, help="Number of samples to take.")
    argument_parser.add_argument("--interval", default=0.05, type=float, help="Interval in seconds between samples.")
    argument_parser.add_argument("--output", default="./", help="Output location/file name.")

    parsed_args = argument_parser.parse_args()

    output_path = path.expanduser(parsed_args.output)
    output_path = path.abspath(output_path)

    if path.isdir(output_path):
        output_path = path.join(output_path, "magnetometer_calibration.json")

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

    calibration_result = calibrate(samples, interval, run_motors=True)
    with open(output_path, "wb") as output_file_handle:
        dump(calibration_result, output_file_handle)

    print("Done.")

    return 0


if __name__ == "__main__":
    exit(main())
