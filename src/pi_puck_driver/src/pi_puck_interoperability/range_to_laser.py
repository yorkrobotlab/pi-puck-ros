#!/usr/bin/env python
"""ROS Node to re-expose range topics as laser scan."""

import posixpath
from functools import partial

# ROS imports
import rospy
from sensor_msgs.msg import Range, LaserScan

PREFIX = "interop/range_to_laser"


class RangeToLaserScan(object):
    """ROS Node to convert ranges to laser scans."""

    def __init__(self):
        """Initialise node."""
        rospy.init_node("range_to_laser")

        robot_root = rospy.get_param("~robot_root", rospy.get_namespace())
        range_publishers = tuple(topic
                                 for topic, topic_type in rospy.get_published_topics(robot_root)
                                 if topic_type == "sensor_msgs/Range")

        for topic in range_publishers:
            new_topic = posixpath.join(PREFIX, posixpath.relpath(topic, robot_root))
            new_topic = posixpath.join(robot_root, new_topic)
            publisher = rospy.Publisher(new_topic, LaserScan, queue_size=10)
            publisher_handler = partial(self.republish_laser_scan, publisher)
            rospy.Subscriber(topic, Range, publisher_handler)

        self._running = True

    def republish_laser_scan(self, publisher, data):
        """Publish range as laser scan."""
        if not self._running:
            return

        laser_scan_message = LaserScan()
        laser_scan_message.range_max = data.max_range
        laser_scan_message.range_min = data.min_range

        laser_scan_message.angle_min = -(data.field_of_view / 2)
        laser_scan_message.angle_max = data.field_of_view / 2
        laser_scan_message.angle_increment = data.field_of_view / 2

        laser_scan_message.ranges = [data.range] * 3

        laser_scan_message.header.frame_id = data.header.frame_id

        publisher.publish(laser_scan_message)

    def run(self):
        """ROS Node server."""
        rospy.spin()
        self._running = False


if __name__ == "__main__":
    RangeToLaserScan().run()
