#!/usr/bin/env python
"""ROS Node to do a lot of nothing."""

# ROS imports
import rospy


class DoNothing(object):
    """Node to do nothing."""

    def __init__(self):
        """Initialise node."""
        rospy.init_node("nothing_node")

    def run(self):
        """Run the node."""
        rospy.spin()

if __name__ == "__main__":
    DoNothing().run()
