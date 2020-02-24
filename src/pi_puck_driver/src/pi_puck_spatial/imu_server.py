#!/usr/bin/env python
"""ROS Node to expose a topics for the IMU (LSM9DS1) on the Pi-puck."""

# Base Imports
import math

# ROS imports
import rospy
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Imu, Temperature

# Standard imports
from lsm9ds1 import LSM9DS1

UNKNOWN_VARIANCE = 0

# If accuracy is updated these need updating from the
LINEAR_ACCELERATION_COVARIANCE = [0.061, 0, 0, 0, 0.061, 0, 0, 0, 0.061]
ANGULAR_VELOCITY_COVARIANCE = [0.00875, 0, 0, 0, 0.00875, 0, 0, 0, 0.00875]


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

    def close_sensor(self):
        """Close the sensor after the ROS Node is shutdown."""
        self._sensor.close()

    def open_sensor(self):
        """Open the sensor."""
        self._sensor = LSM9DS1()

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

            self._sensor_imu_publisher.publish(
                Imu(linear_acceleration_covariance=LINEAR_ACCELERATION_COVARIANCE,
                    linear_acceleration=Vector3(x=acceleration_x, y=acceleration_y, z=acceleration_z),
                    angular_velocity_covariance=ANGULAR_VELOCITY_COVARIANCE,
                    angular_velocity=Vector3(x=gyro_x, y=gyro_y, z=gyro_z)

                    # Our IMU does not provide orientation information so we
                    # can't send that as part of the IMU message

                    # TODO: Implement orentation based on "compass"
                    # We should be able to infer some from using the
                    # magnetometer as a compass, though this might be low
                    # accuracy and would mean making assumptions
                    ))

            self._rate.sleep()


if __name__ == "__main__":
    server = PiPuckImuServer()
    server.run()
