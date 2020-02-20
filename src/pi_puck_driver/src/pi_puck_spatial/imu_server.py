#!/usr/bin/env python
"""ROS Node to expose a topics for the IMU (LSM9DS1) on the Pi-puck."""

# ROS imports
import rospy
from sensor_msgs.msg import Temperature, Imu

# Standard imports
from lsm9ds1 import LSM9DS1

UNKNOWN_VARIANCE = 0


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

    def read_sensor(self):
        """Read the sensor."""
        sensor_reading = self._sensor.get_distance()
        if sensor_reading < MIN_RANGES[self._distance_mode]:
            sensor_reading = -float("inf")
        elif sensor_reading > MAX_RANGES[self._distance_mode]:
            sensor_reading = float("inf")
        sensor_reading = sensor_reading / 1000.0
        return sensor_reading

    def run(self):
        """Run the sensor server."""
        self.open_sensor()
        while not rospy.is_shutdown():
            temperature_result = self._sensor.temperature
            self._sensor_temperature_publisher.publish(
                Temperature(temperature=temperature_result, variance=UNKNOWN_VARIANCE))
            self._rate.sleep()


if __name__ == "__main__":
    server = PiPuckImuServer()
    server.run()
