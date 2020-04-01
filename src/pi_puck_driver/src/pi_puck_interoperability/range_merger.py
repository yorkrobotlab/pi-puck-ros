#!/usr/bin/env python
"""ROS Node to convert ranges to comprehensive laser scans."""

# Standard Imports

from math import radians
from functools import partial

# ROS imports
import rospy
from sensor_msgs.msg import Range, LaserScan

SENSOR_PREFIX = "navigation/proximity/"

RANGE_SENSORS = {
    "short_range_ir/5": radians(-90),
    "short_range_ir/6": radians(-45),
    "long_range_ir/7": radians(-45),
    "short_range_ir/7": radians(-10),
    "long_range_ir/0": radians(0),
    "short_range_ir/0": radians(10),
    "long_range_ir/1": radians(45),
    "short_range_ir/1": radians(45),
    "short_range_ir/2": radians(90),
}

SCAN_START = radians(-90)
SCAN_END = radians(90)
SCAN_STEP = radians(22.5)
SCAN_STEPS = int((SCAN_END - SCAN_START) / SCAN_STEP)

RELEVANCE_DISTANCE = radians(45)
EFFECT_SMOOTHING = 1.5

INF = float("inf")

MAX_RANGE = 1.2
MIN_RANGE = 0.005

REFERENCE_FRAME_ID = "body_top"
ROBOT_RADIUS = 0.035


class PiPuckRangeMerger(object):
    """ROS Node to convert ranges to comprehensive laser scans."""

    def __init__(self):
        """Initialise node."""
        rospy.init_node("range_merger")

        self._merged_topic = rospy.Publisher("interop/range_merger/scan", LaserScan, queue_size=10)

        robot_root = rospy.get_param("~robot_root", rospy.get_namespace())

        tf_prefix_key = rospy.search_param("tf_prefix")
        if tf_prefix_key:
            tf_prefix = rospy.get_param(tf_prefix_key, None)
        else:
            tf_prefix = None
        if tf_prefix is not None and not tf_prefix.endswith("/"):
            tf_prefix += "/"

        self._tf_reference_frame = REFERENCE_FRAME_ID

        if tf_prefix:
            self._tf_reference_frame = tf_prefix + self._tf_reference_frame

        self._rate = rospy.Rate(rospy.get_param('~rate', 1))

        self._sensor_last_values = {key: INF for key, _ in RANGE_SENSORS}
        self._latest_message = rospy.Time.now()

        for sensor in RANGE_SENSORS:
            rospy.Subscriber(robot_root + SENSOR_PREFIX + sensor, Range,
                             partial(self.range_handler, sensor=sensor))

        self._running = True

    def range_handler(self, data, sensor):
        """Handle new range data."""
        self._latest_message = data.header.stamp
        self._sensor_last_values[sensor] = data.range

    def calculate_reading(self, angle):
        """Calculate a combined reading for a point in the pseudo laser scan."""
        relevant_sensors = (key for key, value in RANGE_SENSORS
                            if abs(value - angle) < RELEVANCE_DISTANCE)

        effect_percents = 0
        negative_infs = 0
        positive_infs = 0
        cumulative_value = 0

        for sensor in relevant_sensors:
            sensor_value = self._sensor_last_values[sensor]
            if sensor_value == INF:
                positive_infs += 1
            elif sensor_value == -INF:
                negative_infs += 1
            else:
                sensor_angle = RANGE_SENSORS[sensor]
                effect_percent = abs(sensor_angle - angle) / RELEVANCE_DISTANCE
                effect_percent = effect_percent**EFFECT_SMOOTHING
                cumulative_value += sensor_value * effect_percent
                effect_percents += effect_percent

        if effect_percents == 0:
            if negative_infs > positive_infs:
                return -INF
            return INF

        final_value = cumulative_value / effect_percents

        if final_value > MAX_RANGE:
            return INF
        if final_value < MIN_RANGE:
            return -INF

        return final_value + ROBOT_RADIUS

    def publish_laser_scan(self):
        """Publish range as laser scan."""
        if not self._running:
            return

        laser_scan_message = LaserScan()
        laser_scan_message.range_max = MAX_RANGE + ROBOT_RADIUS
        laser_scan_message.range_min = MIN_RANGE + ROBOT_RADIUS

        laser_scan_message.angle_min = SCAN_START
        laser_scan_message.angle_max = SCAN_END
        laser_scan_message.angle_increment = SCAN_STEP

        laser_scan_message.ranges = [
            self.calculate_reading(step * SCAN_STEP + SCAN_START) for step in range(SCAN_STEPS + 1)
        ]

        laser_scan_message.header.frame_id = self._tf_reference_frame
        laser_scan_message.header.stamp = self._latest_message

        self._merged_topic.publish(laser_scan_message)

    def run(self):
        """ROS Node server."""
        while not rospy.is_shutdown():
            self.publish_laser_scan()
            self._rate.sleep()
        self._running = False


if __name__ == "__main__":
    PiPuckRangeMerger().run()
