#!/usr/bin/env python
"""ROS Node to take velocity messages and output motor speeds."""

# Python libs
from math import pi, copysign
from collections import namedtuple

# ROS imports
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist, Vector3

WHEEL_DIAMETER = 4.0  # cm
WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * pi / 100.0  # in metres
WHEEL_SEPARATION = 5.3  # in cm
MOTOR_STEP_DISTANCE = WHEEL_CIRCUMFERENCE / 1000.0  # 1 turn should be 1000 steps

ROBOT_RADIUS = 0.035
ROBOT_CIRCUMFERENCE = (ROBOT_RADIUS * 2 * pi)

MAX_SPEED = 1000

Smoothing = namedtuple("Smoothing", ["angular", "linear"])


def clamp(value, value_max=1, value_min=0):
    """Clamp between values."""
    return max(min(value, value_max), value_min)


class PiPuckBaseController(object):
    """ROS Node to expose topics relating to the Pi-puck motors."""

    def __init__(self):
        """Initialise node."""
        rospy.on_shutdown(self.stop_motors)

        self._speed_right_pub = rospy.Publisher('motors/speed_right', Float32, queue_size=10)
        self._speed_left_pub = rospy.Publisher('motors/speed_left', Float32, queue_size=10)

        rospy.init_node("base_controller")

        rospy.Subscriber("/cmd_vel", Twist, self.callback_velocity)

        self._rate = rospy.Rate(rospy.get_param('~rate', 10))
        self._fixed_rate = bool(rospy.get_param('~fixed_rate', False))
        self._smoothing = Smoothing(float(rospy.get_param('~angular_smoothing', 0.65)),
                                    float(rospy.get_param('~linear_smoothing', 1.25)))
        motor_speed_mode = rospy.get_param('~motor_control_mode', "complex")

        if motor_speed_mode == "simple":
            self._motor_speed_mapper_function = PiPuckBaseController.calculate_motor_speeds_simple
        elif motor_speed_mode == "complex":
            self._motor_speed_mapper_function = self.calculate_motor_speeds_complex
        else:
            raise Exception("Invalid motor control mapping mode")

        self._target_angular = Vector3()
        self._target_linear = Vector3()

    def stop_motors(self):
        """Stop motors on shutdown."""
        self._speed_right_pub.publish(0)
        self._speed_left_pub.publish(0)

    def callback_velocity(self, data):
        """Handle velocity callback."""
        self._target_angular = data.angular
        self._target_linear = data.linear
        if not self._fixed_rate:
            self.update_motor_speeds()

    def update_motor_speeds(self):
        """Send motor speed update messages."""
        motor_left_speed, motor_right_speed = self._motor_speed_mapper_function(
            self._target_linear, self._target_angular)

        self._speed_left_pub.publish(motor_left_speed)
        self._speed_right_pub.publish(motor_right_speed)

    @staticmethod
    def smooth_motor_speed(smoothing, velocity_percent):
        """Apply smoothing."""
        if smoothing == 1.0 or smoothing <= 0:
            return velocity_percent
        return copysign(abs(velocity_percent)**smoothing, velocity_percent)

    @staticmethod
    def calculate_motor_speeds_simple(linear, angular):
        """Calculate motor speed percentages."""
        motor_left_speed = (linear.x - (WHEEL_SEPARATION / 2.0) * angular.z) / WHEEL_DIAMETER
        motor_right_speed = (linear.x + (WHEEL_SEPARATION / 2.0) * angular.z) / WHEEL_DIAMETER

        return motor_left_speed, motor_right_speed

    def calculate_motor_speeds_complex(self, linear, angular):
        """Calculate motor speed percentages."""
        # angular.z is in rads/second
        rotation_speed = angular.z / (2 * pi) * ROBOT_CIRCUMFERENCE  # in metres/second
        rotation_steps = rotation_speed / MOTOR_STEP_DISTANCE  # in steps/second
        rotation_percent = clamp(rotation_steps / MAX_SPEED, 1.0, -1.0)

        rotation_percent = PiPuckBaseController.smooth_motor_speed(self._smoothing.angular,
                                                                   rotation_percent)

        forward_speed = linear.x  # in metres/second
        forward_steps = forward_speed / MOTOR_STEP_DISTANCE  # in steps/second
        forward_percent = clamp(forward_steps / MAX_SPEED, 1.0, -1.0)

        forward_percent = PiPuckBaseController.smooth_motor_speed(self._smoothing.linear,
                                                                  forward_percent)

        magnitude = 1 + abs(rotation_percent)

        motor_right_speed = (forward_percent + rotation_percent) / magnitude
        motor_left_speed = (forward_percent - rotation_percent) / magnitude

        return motor_left_speed, motor_right_speed

    def run(self):
        """Run the base controller."""
        if self._fixed_rate:
            while not rospy.is_shutdown():
                self.update_motor_speeds()
                self._rate.sleep()
        else:
            rospy.spin()


if __name__ == "__main__":
    PiPuckBaseController().run()
