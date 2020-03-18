#!/usr/bin/env python
"""ROS Node to control the Pi-puck with a joystick."""

# ROS imports
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32


class JoystickControl(object):
    """Node to enable joystick control of the pi-puck."""

    def __init__(self):
        """Initialise node."""
        rospy.init_node("joy_control")

        self._turn_left = 0
        self._turn_right = 0
        self._move_forward = 0
        self._move_backwards = 0

        pi_puck_namespace = rospy.get_param('~pi_puck', "/pi_puck")

        if not pi_puck_namespace.endswith("/"):
            pi_puck_namespace += "/"
        motor_node = pi_puck_namespace + "navigation/motors"
        left_motor = motor_node + "/speed_left"
        right_motor = motor_node + "/speed_right"

        self._left_speed_pub = rospy.Publisher(left_motor, Float32, queue_size=10)
        self._right_speed_pub = rospy.Publisher(right_motor, Float32, queue_size=10)

        rospy.on_shutdown(self.handle_close)

    def handle_close(self):
        """Stop motors on close."""
        self._left_speed_pub.publish(0.0)
        self._right_speed_pub.publish(0.0)

    def joy_callback_handler(self, data):
        """Handle joystick data."""
        self._turn_left = max(float(data.axes[0]), 0.0) if data.axes else 0.0
        self._turn_right = abs(min(float(data.axes[0]), 0.0)) if data.axes else 0.0
        self._move_forward = max(float(data.axes[1]), 0.0) if len(data.axes) > 1.0 else 0.0
        self._move_backwards = abs(min(float(data.axes[1]), 0.0)) if len(data.axes) > 1.0 else 0.0

        right_motor_speed = min(
            max(((self._turn_left - self._turn_right) +
                 (self._move_forward - self._move_backwards)), -1), 1)
        left_motor_speed = min(
            max(((-self._turn_left + self._turn_right) +
                 (self._move_forward - self._move_backwards)), -1), 1)

        self._left_speed_pub.publish(left_motor_speed)
        self._right_speed_pub.publish(right_motor_speed)

    def run(self):
        """Run the node."""
        rospy.Subscriber("/joy", Joy, self.joy_callback_handler)
        rospy.spin()


if __name__ == "__main__":
    JoystickControl().run()
