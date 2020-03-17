#!/usr/bin/env python
"""ROS Node to expose topics for motor speed and for motor steps of the Pi-puck."""

# Python libs
from math import cos, pi, sin

# ROS imports
import rospy
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, Int64

# Standard imports
from smbus import SMBus

# Constants
I2C_CHANNEL = 4
EPUCK_I2C_ADDR = 0x1e

LEFT_MOTOR_SPEED = 2
RIGHT_MOTOR_SPEED = 3
LEFT_MOTOR_STEPS = 4
RIGHT_MOTOR_STEPS = 5

WHEEL_DIAMETER = 4.0  # cm
WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * pi / 100.0  # in metres
WHEEL_DISTANCE = 0.053  # the distance between the wheels in metres
MOTOR_STEP_DISTANCE = WHEEL_CIRCUMFERENCE / 1000.0  # 1 turn should be 1000 steps

MAX_MOTOR_STEPS_RAW = 65536

MAX_SPEED = 500

REFERENCE_FRAME_ID = "base_link"


class PiPuckMotorServer(object):
    """ROS Node to expose topics relating to the Pi-puck motors."""

    def __init__(self):
        """Initialise node."""
        self._bus = SMBus(I2C_CHANNEL)

        rospy.on_shutdown(self.close_bus)

        self._steps_right_pub = rospy.Publisher('motors/steps_right', Int64, queue_size=10)
        self._steps_left_pub = rospy.Publisher('motors/steps_left', Int64, queue_size=10)
        self._odometry_pub = rospy.Publisher('motors/odometry', Odometry, queue_size=10)

        rospy.init_node("motors")

        rospy.Subscriber("motors/speed_right", Float32, self.callback_right)
        rospy.Subscriber("motors/speed_left", Float32, self.callback_left)

        self._rate = rospy.Rate(rospy.get_param('rate', 10))

        tf_prefix_key = rospy.search_param("tf_prefix")
        if tf_prefix_key:
            tf_prefix = rospy.get_param(tf_prefix_key, None)
        else:
            tf_prefix = None
        if tf_prefix is not None and not tf_prefix.endswith("/"):
            tf_prefix += "/"

        self._reference_frame = REFERENCE_FRAME_ID
        if tf_prefix:
            self._reference_frame = tf_prefix + self._reference_frame

        self._left_overflows = 0
        self._right_overflows = 0

        self._left_steps_previous = 0
        self._right_steps_previous = 0
        self._real_left_steps_previous = 0
        self._real_right_steps_previous = 0

        self._left_motor_speed = 0
        self._right_motor_speed = 0

        self._estimate_x = 0
        self._estimate_y = 0
        self._estimate_theta = 0

        self._last_measurement_time = rospy.get_time()

    @staticmethod
    def convert_speed(speed):
        """Convert input speed data into a speed value for the stepper motors."""
        speed = float(speed)
        if speed > 1.0:
            speed = 1.0
        elif speed < -1.0:
            speed = -1.0
        return int(speed * MAX_SPEED)

    def callback_left(self, data):
        """Handle requests for speed change of left motor."""
        self._left_motor_speed = data.data
        self._bus.write_word_data(EPUCK_I2C_ADDR, LEFT_MOTOR_SPEED,
                                  PiPuckMotorServer.convert_speed(self._left_motor_speed))

    def callback_right(self, data):
        """Handle request for speed change of right motor."""
        self._right_motor_speed = data.data
        self._bus.write_word_data(EPUCK_I2C_ADDR, RIGHT_MOTOR_SPEED,
                                  PiPuckMotorServer.convert_speed(self._right_motor_speed))

    def close_bus(self):
        """Close the I2C bus after the ROS Node is shutdown."""
        self._bus.write_word_data(EPUCK_I2C_ADDR, LEFT_MOTOR_SPEED, 0)
        self._bus.write_word_data(EPUCK_I2C_ADDR, RIGHT_MOTOR_SPEED, 0)
        self._bus.close()

    @staticmethod
    def euler_to_quaternion(yaw, pitch, roll):
        """Convert euler angles of pitch, roll, and yaw to a quaternion.

        Based on code from
        https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles.
        """
        cy = cos(yaw * 0.5)
        sy = sin(yaw * 0.5)
        cp = cos(pitch * 0.5)
        sp = sin(pitch * 0.5)
        cr = cos(roll * 0.5)
        sr = sin(roll * 0.5)

        q_w = cy * cp * cr + sy * sp * sr
        q_x = cy * cp * sr - sy * sp * cr
        q_y = sy * cp * sr + cy * sp * cr
        q_z = sy * cp * cr - cy * sp * sr

        return Quaternion(x=q_x, y=q_y, z=q_z, w=q_w)

    def run(self):
        """ROS Node server."""
        while not rospy.is_shutdown():
            left_steps = int(self._bus.read_word_data(EPUCK_I2C_ADDR, LEFT_MOTOR_STEPS))
            right_steps = int(self._bus.read_word_data(EPUCK_I2C_ADDR, RIGHT_MOTOR_STEPS))
            measurement_time = rospy.get_time()

            if self._left_motor_speed > 0 and left_steps + (MAX_MOTOR_STEPS_RAW /
                                                            2.0) < self._left_steps_previous:
                self._left_overflows += 1
            elif self._left_motor_speed < 0 and left_steps > self._left_steps_previous + (
                    MAX_MOTOR_STEPS_RAW / 2.0):
                self._left_overflows -= 1

            if self._right_motor_speed > 0 and right_steps + (MAX_MOTOR_STEPS_RAW /
                                                              2.0) < self._right_steps_previous:
                self._right_overflows += 1
            elif self._right_motor_speed < 0 and right_steps > self._right_steps_previous + (
                    MAX_MOTOR_STEPS_RAW / 2.0):
                self._right_overflows -= 1

            real_left_steps = left_steps + self._left_overflows * MAX_MOTOR_STEPS_RAW
            real_right_steps = right_steps + self._right_overflows * MAX_MOTOR_STEPS_RAW

            self._steps_right_pub.publish(right_steps)
            self._steps_left_pub.publish(left_steps)

            delta_left = (real_left_steps - self._real_left_steps_previous) / MOTOR_STEP_DISTANCE
            delta_right = (real_right_steps - self._real_right_steps_previous) / MOTOR_STEP_DISTANCE

            delta_theta = (delta_right - delta_left) / WHEEL_DISTANCE
            delta_steps = (delta_right + delta_left) / 2.0

            self._estimate_x += delta_steps * cos(self._estimate_theta + delta_theta / 2.0)
            self._estimate_y += delta_steps * sin(self._estimate_theta + delta_theta / 2.0)
            self._estimate_theta += delta_theta

            self._left_steps_previous = left_steps
            self._right_steps_previous = right_steps
            self._real_left_steps_previous = real_left_steps
            self._real_right_steps_previous = real_right_steps

            odometry_message = Odometry()

            odometry_message.pose.pose.orientation = PiPuckMotorServer.euler_to_quaternion(
                self._estimate_theta, 0, 0)
            odometry_message.pose.pose.position.x = self._estimate_x
            odometry_message.pose.pose.position.y = self._estimate_y

            odometry_message.twist.twist.linear.x = delta_steps / (measurement_time -
                                                                   self._last_measurement_time)
            odometry_message.twist.twist.angular.z = delta_theta / (measurement_time -
                                                                    self._last_measurement_time)

            odometry_message.header.frame_id = self._reference_frame

            self._odometry_pub.publish(odometry_message)

            self._last_measurement_time = measurement_time

            self._rate.sleep()


if __name__ == "__main__":
    PiPuckMotorServer().run()
