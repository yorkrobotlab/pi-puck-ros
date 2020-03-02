#!/usr/bin/env python
"""ROS Node to expose a topics for the IMU (LSM9DS1) on the Pi-puck."""

# Base Imports
import math
from os import path
from json import load

# ROS imports
import rospy
from geometry_msgs.msg import Vector3, Quaternion
from sensor_msgs.msg import Imu, Temperature

# Standard imports
from lsm9ds1 import LSM9DS1

UNKNOWN_VARIANCE = 0

# If accuracy is updated these need updating from the
LINEAR_ACCELERATION_COVARIANCE = [0.061, 0, 0, 0, 0.061, 0, 0, 0, 0.061]
ANGULAR_VELOCITY_COVARIANCE = [0.00875, 0, 0, 0, 0.00875, 0, 0, 0, 0.00875]

# Rough estimate based on measurements with Pi-pucks
CALIBRATION_DATA_DEFAULT = {"y": -3.10, "x": 0.15, "z": -1.70}

def calculate_angle()

class PiPuckImuServer:
    """ROS Node to publish data from the Pi-puck's IMU."""

    def __init__(self):
        """Initialise IMU server node."""
        rospy.on_shutdown(self.close_sensor)

        rospy.init_node("imu")

        self._rate = rospy.Rate(rospy.get_param('~rate', 5))

        self._sensor_imu_publisher = rospy.Publisher('imu/imu', Imu, queue_size=10)
        self._sensor_temperature_publisher = rospy.Publisher('imu/temperature', Temperature, queue_size=10)

        self._sensor = None

        if path.isfile("magnetometer_calibration.json"):
            with open("magnetometer_calibration.json", "rb") as calibration_file_handle:
                self._calibration = load(calibration_file_handle)
        else:
            self._calibration = CALIBRATION_DATA_DEFAULT

    def close_sensor(self):
        """Close the sensor after the ROS Node is shutdown."""
        self._sensor.close()

    def open_sensor(self):
        """Open the sensor."""
        self._sensor = LSM9DS1()

    @staticmethod
    def euler_to_quaternion(yaw, pitch, roll):
        double cy = math.cos(yaw * 0.5)
        double sy = math.sin(yaw * 0.5)
        double cp = math.cos(pitch * 0.5)
        double sp = math.sin(pitch * 0.5)
        double cr = math.cos(roll * 0.5)
        double sr = math.sin(roll * 0.5)

        q_w = cy * cp * cr + sy * sp * sr
        q_x = cy * cp * sr - sy * sp * cr
        q_y = sy * cp * sr + cy * sp * cr
        q_z = sy * cp * cr - cy * sp * sr

        return Quaternion(x=q_x, y=q_y, z=q_z, w=q_w)

    @staticmethod
    def calculate_heading_quaternion(magnetometer_reading):
        raw_x, raw_y, raw_z = magnetometer_reading
        cal_x, cal_y, cal_z = self._calibration["x"], self._calibration["y"], self._calibration["z"]
        x, y, z = raw_x - cal_x, raw_y - cal_y, raw_z - cal_z
        if y == 0:
            if x > 0:
                x_y_direction = 180
            else: # x <= 0
                x_y_direction = 0
        elif y > 0:
            x_y_direction = 90 - math.atan(x/y)
        else: # y < 0
            x_y_direction = 270 - math.atan(x/y)

        x_y_direction_rad = x_y_direction * (math.pi / 180.0)

        return euler_to_quaternion(yaw=x_y_direction_rad, 0, 0)


    def run(self):
        """Run the sensor server."""
        self.open_sensor()
        while not rospy.is_shutdown():
            # Temperature is in degrees C so no conversion needed
            temperature_result = self._sensor.temperature
            self._sensor_temperature_publisher.publish(
                Temperature(temperature=temperature_result, variance=UNKNOWN_VARIANCE))

            # Acceleration is already in m/s^2 so no conversion needed,
            # just unpacking
            acceleration_result = self._sensor.acceleration
            acceleration_x, acceleration_y, acceleration_z = acceleration_result

            # Gyro is in degrees/second so conversion to rads/sec is needed,
            # as well as unpacking
            gyro_result = self._sensor.gyro
            gyro_x, gyro_y, gyro_z = map(lambda deg: deg * (math.pi / 180.0), gyro_result)

            magnetometer_result = self._sensor.magnetic
            magnetometer_quaternion = calculate_heading_quaternion(magnetometer_result)

            self._sensor_imu_publisher.publish(
                Imu(linear_acceleration_covariance=LINEAR_ACCELERATION_COVARIANCE,
                    linear_acceleration=Vector3(x=acceleration_x, y=acceleration_y, z=acceleration_z),
                    angular_velocity_covariance=ANGULAR_VELOCITY_COVARIANCE,
                    angular_velocity=Vector3(x=gyro_x, y=gyro_y, z=gyro_z),
                    orientation=magnetometer_quaternion
                    ))

            self._rate.sleep()


if __name__ == "__main__":
    server = PiPuckImuServer()
    server.run()
