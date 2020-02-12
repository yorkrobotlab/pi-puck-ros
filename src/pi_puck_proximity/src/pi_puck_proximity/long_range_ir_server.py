#!/usr/bin/env python
"""ROS Node to expose a topics for the short range IR sensors on the Pi-puck."""

# ROS imports
import rospy
from sensor_msgs.msg import Range

# Standard imports
import sys
from importlib import import_module

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
        rospy.on_shutdown(self.close_sensors)

        rospy.init_node("long_range_ir")

        self._ir_sensor_publishers = ir_sensor_publishers = {}

        self._ir_sensors = ir_sensors = {}

        self._rate_raw = float(rospy.get_param('rate', 1))

        self._rate = rospy.Rate(self._rate_raw)

        self._sensor_modules = {}

        mode = rospy.get_param('mode', "short")

        if mode in RANGES:
            self._distance_mode = RANGES[mode]
        else:
            self._distance_mode = mode

        for ir_sensor in range(IR_SENSOR_COUNT):
            ir_sensor_publishers[ir_sensor] = rospy.Publisher('proximity/long_range_ir/{}'.format(ir_sensor),
                                                              Range,
                                                              queue_size=10)

            self._sensor_modules[ir_sensor] = import_module("VL53L1X")

            ir_sensors[ir_sensor] = self._sensor_modules[ir_sensor].VL53L1X(i2c_bus=TOF_I2C_CHANNELS[ir_sensor],
                                                                            i2c_address=TOF_I2C_ADDRESS)

            del sys.modules["VL53L1X"]

    def close_sensors(self):
        """Close the sensors after the ROS Node is shutdown."""
        for sensor in self._ir_sensors.values():
            sensor.stop_ranging()
            sensor.close()

    def open_sensors(self):
        for sensor in self._ir_sensors.values():
            sensor.open()

            timing_budget_us = int((1.0 / self._rate_raw * 1000000.0) / 12.0)
            inter_measurement_period_ms = int((1.0 / self._rate_raw * 1000.0) / 6.0)

            sensor.set_timing(timing_budget=timing_budget_us, inter_measurement_period=inter_measurement_period_ms)

            sensor.start_ranging(self._distance_mode)

    def run(self):
        self.open_sensors()
        while not rospy.is_shutdown():
            for ir_sensor in range(IR_SENSOR_COUNT):
                sensor_reading = self._ir_sensors[ir_sensor].get_distance()
                if sensor_reading < MIN_RANGES[self._distance_mode]:
                    sensor_reading = -float("inf")
                elif sensor_reading > MAX_RANGES[self._distance_mode]:
                    sensor_reading = float("inf")
                sensor_reading = sensor_reading / 1000.0
                range_result = Range(radiation_type=Range.INFRARED, min_range=0, max_range=0.1, range=sensor_reading)
                self._ir_sensor_publishers[ir_sensor].publish(range_result)
            self._rate.sleep()


if __name__ == "__main__":
    server = PiPuckTOFSensorServer()
    server.run()
