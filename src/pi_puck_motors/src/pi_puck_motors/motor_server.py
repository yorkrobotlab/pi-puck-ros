#!/usr/bin/env python

# ROS imports
import roslib
roslib.load_manifest('pi_puck_motors')
import rospy

# Standard imports
from pi_puck_base.utils import *


def pi_puck_motor_server():
    rospy.init_node(get_pi_puck_name())
    rospy.spin()


if __name__ == "__main__":
    pi_puck_motor_server()
