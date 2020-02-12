#!/usr/bin/env python
"""ROS Node to expose a topics for the short range IR sensors on the Pi-puck."""

# ROS imports
import rospy
from sensor_msgs.msg import Range

# Standard imports
import VL53L1X

TOF_SENSOR_ANGLES = (0, 45, 135, 180, 225, 315)

TOF_I2C_CHANNELS = (5, 6, 7, 8, 9, 10)
TOF_I2C_ADDRESS = 0x29

IR_SENSOR_COUNT = 6

SHORT_RANGE = 1
MEDIUM_RANGE = 2
LONG_RANGE = 3

RANGES = {"short": SHORT_RANGE, "medium": MEDIUM_RANGE, "long": LONG_RANGE}

# In Millimetres
MAX_RANGES = {SHORT_RANGE: 1360, MEDIUM_RANGE: 2900, LONG_RANGE: 3600}

# In Millimetres
MIN_RANGES = {SHORT_RANGE: 4, MEDIUM_RANGE: 4, LONG_RANGE: 4}


class PiPuckTOFSensorServer:
    def __init__(self):
        rospy.on_shutdown(self.close_sensor)

        rospy.init_node("long_range_ir")

        self._rate_raw = float(rospy.get_param('rate', 1))

        self._rate = rospy.Rate(self._rate_raw)

        sensor_index = int(rospy.get_param('~sensor', 0))
        mode = rospy.get_param('~mode', "short")

        if mode in RANGES:
            self._distance_mode = RANGES[mode]
        else:
            self._distance_mode = mode

        self._sensor_publisher = rospy.Publisher('long_range_ir/{}'.format(sensor_index), Range, queue_size=10)

        self._sensor = VL53L1X.VL53L1X(i2c_bus=TOF_I2C_CHANNELS[sensor_index], i2c_address=TOF_I2C_ADDRESS)

    def close_sensor(self):
        """Close the sensor after the ROS Node is shutdown."""
        self._sensor.stop_ranging()
        self._sensor.close()

    def open_sensor(self):
        """Open the sensor and setup timing."""
        self._sensor.open()

        timing_budget_us = int((1.0 / self._rate_raw * 1000000.0) / 12.0)
        inter_measurement_period_ms = int((1.0 / self._rate_raw * 1000.0) / 6.0)

        self._sensor.set_timing(timing_budget=timing_budget_us, inter_measurement_period=inter_measurement_period_ms)

        self._sensor.start_ranging(self._distance_mode)

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
            range_result = Range(radiation_type=Range.INFRARED,
                                 min_range=MIN_RANGES[self._distance_mode] / 1000.0,
                                 max_range=MAX_RANGES[self._distance_mode] / 1000.0,
                                 range=self.read_sensor())
            self._sensor_publisher.publish(range_result)
            self._rate.sleep()


if __name__ == "__main__":
    server = PiPuckTOFSensorServer()
    server.run()
