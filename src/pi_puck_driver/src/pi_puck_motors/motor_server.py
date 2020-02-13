#!/usr/bin/env python
"""ROS Node to expose topics for motor speed and for motor steps of the Pi-puck."""

# ROS imports
import rospy
# Standard imports
from smbus import SMBus
from std_msgs.msg import Float32, UInt16

# Constants
I2C_CHANNEL = 4
EPUCK_I2C_ADDR = 0x1e

LEFT_MOTOR_SPEED = 2
RIGHT_MOTOR_SPEED = 3
LEFT_MOTOR_STEPS = 4
RIGHT_MOTOR_STEPS = 5

MAX_SPEED = 500

BUS = SMBus(I2C_CHANNEL)


def convert_speed(x):
    """Convert input speed data into a speed value for the stepper motors."""
    x = float(x)
    if x > 1.0:
        x = 1.0
    elif x < -1.0:
        x = -1.0
    return int(x * MAX_SPEED)


def callback_left(data):
    """Handle requests for speed change of left motor."""
    BUS.write_word_data(EPUCK_I2C_ADDR, LEFT_MOTOR_SPEED, convert_speed(data.data))


def callback_right(data):
    """Handle request for speed change of right motor."""
    BUS.write_word_data(EPUCK_I2C_ADDR, RIGHT_MOTOR_SPEED, convert_speed(data.data))


def close_bus():
    """Close the I2C bus after the ROS Node is shutdown."""
    BUS.close()


def pi_puck_motor_server():
    """ROS Node server."""
    rospy.on_shutdown(close_bus)

    steps_right_pub = rospy.Publisher('motors/steps_right', UInt16, queue_size=10)
    steps_left_pub = rospy.Publisher('motors/steps_left', UInt16, queue_size=10)

    rospy.init_node("motors")

    rospy.Subscriber("motors/speed_right", Float32, callback_right)
    rospy.Subscriber("motors/speed_left", Float32, callback_left)

    rate = rospy.Rate(rospy.get_param('rate', 10))

    while not rospy.is_shutdown():
        steps_right_pub.publish(int(BUS.read_word_data(EPUCK_I2C_ADDR, RIGHT_MOTOR_STEPS)))
        steps_left_pub.publish(int(BUS.read_word_data(EPUCK_I2C_ADDR, LEFT_MOTOR_STEPS)))
        rate.sleep()


if __name__ == "__main__":
    pi_puck_motor_server()
