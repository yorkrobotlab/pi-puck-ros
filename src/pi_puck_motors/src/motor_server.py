#!/usr/bin/env python
import sys
import roslib
roslib.load_manifest('pi_puck_motors')
print(sys.path)
from pi_puck_base.utils import *
import rospy


def pi_puck_motor_server():
    rospy.init_node(get_pi_puck_name())
    rospy.spin()


if __name__ == "__main__":
    pi_puck_motor_server()
