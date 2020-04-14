#!/usr/bin/env python
"""ROS Node to expose topics for motor speed and for motor steps of the Pi-puck."""

# Python libs
from math import cos, pi, sin

# ROS imports
import rospy
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, Int64, UInt16

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

MAX_SPEED = 1000

REFERENCE_FRAME_ID = "base_link"


class PiPuckMotorServer(object):
    """ROS Node to expose topics relating to the Pi-puck motors."""

    def __init__(self):
        """Initialise node."""
        self._bus = SMBus(I2C_CHANNEL)

        rospy.on_shutdown(self.close_bus)

        self._steps_right_pub = rospy.Publisher('motors/steps_right', UInt16, queue_size=10)
        self._steps_left_pub = rospy.Publisher('motors/steps_left', UInt16, queue_size=10)
        self._real_steps_right_pub = rospy.Publisher('motors/real_steps_right',
                                                     Int64,
                                                     queue_size=10)
        self._real_steps_left_pub = rospy.Publisher('motors/real_steps_left', Int64, queue_size=10)
        self._odometry_pub = rospy.Publisher('motors/odometry', Odometry, queue_size=10)

        rospy.init_node("motors")

        rospy.Subscriber("motors/speed_right", Float32, self.callback_right)
        rospy.Subscriber("motors/speed_left", Float32, self.callback_left)

        self._rate = rospy.Rate(rospy.get_param('~rate', 10))

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

        We don't use tf.transforms here to reduce dependence on tf and increase performance on this
        simple task.
        """
        c_y = cos(yaw * 0.5)
        s_y = sin(yaw * 0.5)
        c_p = cos(pitch * 0.5)
        s_p = sin(pitch * 0.5)
        c_r = cos(roll * 0.5)
        s_r = sin(roll * 0.5)

        q_w = c_y * c_p * c_r + s_y * s_p * s_r
        q_x = c_y * c_p * s_r - s_y * s_p * c_r
        q_y = s_y * c_p * s_r + c_y * s_p * c_r
        q_z = s_y * c_p * c_r - c_y * s_p * s_r

        return Quaternion(x=q_x, y=q_y, z=q_z, w=q_w)

    def run(self):
        """ROS Node server."""
        initial_left_steps = int(self._bus.read_word_data(EPUCK_I2C_ADDR, LEFT_MOTOR_STEPS))
        initial_right_steps = int(self._bus.read_word_data(EPUCK_I2C_ADDR, RIGHT_MOTOR_STEPS))

        # Initial steps are set to counter the fact that the node may have previously run and the
        # e-puck firmware will not necessarily have been reset.
        self._left_steps_previous = initial_left_steps
        self._right_steps_previous = initial_right_steps
        self._real_left_steps_previous = initial_left_steps
        self._real_right_steps_previous = initial_right_steps

        while not rospy.is_shutdown():
            # Get current steps
            left_steps = int(self._bus.read_word_data(EPUCK_I2C_ADDR, LEFT_MOTOR_STEPS))
            right_steps = int(self._bus.read_word_data(EPUCK_I2C_ADDR, RIGHT_MOTOR_STEPS))

            # Get step measurement time
            measurement_time = rospy.get_time()

            # Check for overflows and underflows
            if left_steps + (MAX_MOTOR_STEPS_RAW / 2.0) < self._left_steps_previous:
                self._left_overflows += 1
            elif left_steps > self._left_steps_previous + (MAX_MOTOR_STEPS_RAW / 2.0):
                self._left_overflows -= 1

            if right_steps + (MAX_MOTOR_STEPS_RAW / 2.0) < self._right_steps_previous:
                self._right_overflows += 1
            elif right_steps > self._right_steps_previous + (MAX_MOTOR_STEPS_RAW / 2.0):
                self._right_overflows -= 1

            # Calculate the real steps left and right accounting for overflows
            real_left_steps = left_steps + self._left_overflows * MAX_MOTOR_STEPS_RAW
            real_right_steps = right_steps + self._right_overflows * MAX_MOTOR_STEPS_RAW

            # Publish raw and real steps
            self._steps_right_pub.publish(right_steps)
            self._steps_left_pub.publish(left_steps)
            self._real_steps_right_pub.publish(real_right_steps)
            self._real_steps_left_pub.publish(real_left_steps)

            # Calculate step changes
            delta_left = (real_left_steps - self._real_left_steps_previous) * MOTOR_STEP_DISTANCE
            delta_right = (real_right_steps - self._real_right_steps_previous) * MOTOR_STEP_DISTANCE

            #Calculate delta steps and turn
            delta_theta = (delta_right - delta_left) / WHEEL_DISTANCE
            delta_steps = (delta_right + delta_left) / 2.0

            # Update the estimate for x, y, and rotation
            self._estimate_x += delta_steps * cos(self._estimate_theta + delta_theta / 2.0)
            self._estimate_y += delta_steps * sin(self._estimate_theta + delta_theta / 2.0)
            self._estimate_theta += delta_theta

            # Update previous steps to current
            self._left_steps_previous = left_steps
            self._right_steps_previous = right_steps
            self._real_left_steps_previous = real_left_steps
            self._real_right_steps_previous = real_right_steps

            # Send odometry
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
