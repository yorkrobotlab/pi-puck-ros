#!/usr/bin/env python
"""ROS Node to re-expose range topics as laser scan."""

import posixpath
from functools import partial

# ROS imports
import rospy
from sensor_msgs.msg import Range, LaserScan

PREFIX = "interop/range_to_laser"
COMBINED = "interop/range_to_laser/combined"


class RangeToLaserScan(object):
    """ROS Node to convert ranges to laser scans."""

    def __init__(self):
        """Initialise node."""
        rospy.init_node("range_to_laser")

        publish_combined_topic = bool(rospy.get_param('~publish_combined_topic', True))
        publish_separate_topics = bool(rospy.get_param('~publish_separate_topics', False))
        fov_adjustment = float(rospy.get_param('~fov_adjustment', 1.0))
        robot_root = rospy.get_param("~robot_root", rospy.get_namespace())
        range_publishers = tuple(topic
                                 for topic, topic_type in rospy.get_published_topics(robot_root)
                                 if topic_type == "sensor_msgs/Range")

        if publish_combined_topic:
            combined_topic = rospy.Publisher(posixpath.join(robot_root, COMBINED),
                                             LaserScan,
                                             queue_size=10)
        else:
            combined_topic = None

        for topic in range_publishers:
            if publish_separate_topics:
                new_topic = posixpath.join(PREFIX, posixpath.relpath(topic, robot_root))
                new_topic = posixpath.join(robot_root, new_topic)
                publisher = rospy.Publisher(new_topic, LaserScan, queue_size=10)
            else:
                publisher = None
            publisher_handler = partial(self.republish_laser_scan,
                                        fov_adjustment=fov_adjustment,
                                        publisher=publisher,
                                        combined_publisher=combined_topic)
            rospy.Subscriber(topic, Range, publisher_handler)

        self._running = True

    def republish_laser_scan(self,
                             data,
                             fov_adjustment=1.0,
                             publisher=None,
                             combined_publisher=None):
        """Publish range as laser scan."""
        if not self._running:
            return

        laser_scan_message = LaserScan()
        laser_scan_message.range_max = data.max_range
        laser_scan_message.range_min = data.min_range

        laser_scan_message.angle_min = -(data.field_of_view / (2 / fov_adjustment))
        laser_scan_message.angle_max = data.field_of_view / (2 / fov_adjustment)
        laser_scan_message.angle_increment = data.field_of_view / (2 / fov_adjustment)

        laser_scan_message.ranges = [data.range] * 3

        laser_scan_message.header.frame_id = data.header.frame_id
        laser_scan_message.header.stamp = data.header.stamp

        if publisher is not None:
            publisher.publish(laser_scan_message)

        if combined_publisher is not None:
            combined_publisher.publish(laser_scan_message)

    def run(self):
        """ROS Node server."""
        rospy.spin()
        self._running = False


if __name__ == "__main__":
    RangeToLaserScan().run()
