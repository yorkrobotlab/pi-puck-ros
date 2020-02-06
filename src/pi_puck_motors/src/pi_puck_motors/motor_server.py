#!/usr/bin/env python

# ROS imports
import roslib
roslib.load_manifest('pi_puck_motors')
import rospy

from std_msgs.msg import Int16, Float32

# Other Pi-puck package imports
from pi_puck_base.utils import *

# Standard imports
from smbus import SMBus

# Constants
I2C_CHANNEL = 4
EPUCK_I2C_ADDR = 0x1e

LEFT_MOTOR_SPEED = 2
RIGHT_MOTOR_SPEED = 3
LEFT_MOTOR_STEPS = 4
RIGHT_MOTOR_STEPS = 5

MAX_SPEED = 500


def pi_puck_motor_server():
    bus = SMBus(I2C_CHANNEL)

    def convert_speed(x):
        x = float(x)
        if x > 1.0:
            x = 1.0
        elif x < -1.0:
            x = -1.0
        return int(x * MAX_SPEED)

    def callback_left(data):
        bus.write_word_data(EPUCK_I2C_ADDR, LEFT_MOTOR_SPEED,
                            convert_speed(data))

    def callback_right(data):
        bus.write_word_data(EPUCK_I2C_ADDR, RIGHT_MOTOR_SPEED,
                            convert_speed(data))

    steps_right_pub = rospy.Publisher('motors/steps_right',
                                      Int16,
                                      queue_size=10)
    steps_left_pub = rospy.Publisher('motors/steps_left', Int16, queue_size=10)

    rospy.init_node("motors")

    rospy.Subscriber("motors/speed_right", Float32, callback_right)
    rospy.Subscriber("motors/speed_left", Float32, callback_left)

    rate = rospy.Rate(rospy.get_param('rate', 10))

    rospy.loginfo("Hello")

    while not rospy.is_shutdown():
        print("publishing")
        steps_right_pub.publish(
            int(bus.read_word_data(EPUCK_I2C_ADDR, RIGHT_MOTOR_STEPS)))
        steps_left_pub.publish(
            int(bus.read_word_data(EPUCK_I2C_ADDR, LEFT_MOTOR_STEPS)))
        rate.sleep()


if __name__ == "__main__":
    pi_puck_motor_server()
