#!/usr/bin/env python
"""ROS Node to publish transforms based on data from sensors."""

from collections import namedtuple
from xml.etree import ElementTree as ETREE
from math import cos, sin, pi

import rospy
import tf
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import Int64
from geometry_msgs.msg import Quaternion

from imu_server import PiPuckImuServer

calculate_heading = PiPuckImuServer.calculate_heading

StaticTransform = namedtuple("StaticTransform", ["parent_link", "child_link", "xyz", "rpy"])

WHEEL_DIAMETER = 4.0  # cm
WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * pi / 100.0  # in metres
WHEEL_DISTANCE = 0.053  # the distance between the wheels in metres
MOTOR_STEP_DISTANCE = WHEEL_CIRCUMFERENCE / 1000.0  # 1 turn should be 1000 steps


class PiPuckTransformServer(object):
    """ROS Node to publish data from the Pi-puck's IMU."""

    def __init__(self):
        """Initialise TF server node."""
        rospy.init_node("tf_broadcaster")

        self._rate = rospy.Rate(rospy.get_param('~rate', 60))
        self._use_imu = bool(rospy.get_param('~use_imu', True))
        self._use_hybrid_position = bool(rospy.get_param('~use_hybrid_position', True))

        self._robot_description_raw = rospy.get_param("robot_description", None)
        self._tf_prefix = rospy.get_param("tf_prefix", None)

        if self._tf_prefix is not None and not self._tf_prefix.endswith("/"):
            self._tf_prefix += "/"

        self._publish_static_transforms = rospy.get_param("publish_static_transforms", True)
        if not isinstance(self._publish_static_transforms, bool):
            self._publish_static_transforms = bool(self._publish_static_transforms)

        if self._robot_description_raw:
            self._robot_description = ETREE.fromstring(self._robot_description_raw)
        else:
            self._robot_description = None

        if self._publish_static_transforms:
            self._static_transforms = []

            self._find_static_transforms()

        self._broadcaster = tf.TransformBroadcaster()

        self._current_xyz = (0, 0, 0)
        self._current_quaternion = tf.transformations.quaternion_from_euler(0, 0, 0)
        self._previous_quaternion = tf.transformations.quaternion_from_euler(0, 0, 0)
        self._steps_right = None
        self._steps_left = None
        self._previous_steps_right = None
        self._previous_steps_left = None
        self._magnetic_orientation = None
        self._previous_theta = None

    def _find_static_transforms(self):

        def _is_fixed(joint_to_check):
            return joint_to_check.get("type").lower() == "fixed"

        for joint in filter(_is_fixed, self._robot_description.findall("joint")):
            parent = joint.find("parent")
            child = joint.find("child")
            origin = joint.find("origin")
            if parent is not None and child is not None and origin is not None:
                parent_link = parent.get("link")
                child_link = child.get("link")
                origin_xyz = origin.get("xyz")
                origin_rpy = origin.get("rpy")
                if parent_link is not None and child_link is not None and origin_xyz is not None:
                    origin_xyz = tuple(map(float, origin_xyz.split(" ")))
                    if origin_rpy is None:
                        origin_rpy = (0, 0, 0)
                    else:
                        origin_rpy = tuple(map(float, origin_rpy.split(" ")))
                    self._static_transforms.append(
                        StaticTransform(parent_link=parent_link,
                                        child_link=child_link,
                                        xyz=origin_xyz,
                                        rpy=origin_rpy))

    def send_static_transforms(self):
        """Send static transforms for fixed joints."""
        for static_transform in self._static_transforms:
            self._broadcaster.sendTransform(
                translation=static_transform.xyz,
                rotation=tf.transformations.quaternion_from_euler(*static_transform.rpy),
                time=rospy.Time.now(),
                child=self._tf_prefix + static_transform.child_link,
                parent=self._tf_prefix + static_transform.parent_link)

    def send_pi_puck_transform(self):
        """Send transform for Pi-puck."""
        self._broadcaster.sendTransform(translation=self._current_xyz,
                                        rotation=self._current_quaternion,
                                        time=rospy.Time.now(),
                                        child=self._tf_prefix + "base_link",
                                        parent="map")

    def imu_data_callback(self, data):
        """IMU data callback handler."""
        self._previous_quaternion = self._current_quaternion
        self._current_quaternion = (data.orientation.x, data.orientation.y, data.orientation.z,
                                    data.orientation.w)

    def motor_odometry_data_callback(self, data):
        """Motor data callback handler."""
        if not self._use_hybrid_position:
            self._current_xyz = (data.pose.pose.position.x, data.pose.pose.position.y,
                                 data.pose.pose.position.z)
        if not self._use_imu:
            self._previous_quaternion = self._current_quaternion
            self._current_quaternion = (data.pose.pose.orientation.x, data.pose.pose.orientation.y,
                                        data.pose.pose.orientation.z, data.pose.pose.orientation.w)

    def magnetometer_callback(self, data):
        """Magnetometer data callback."""
        raw_heading = calculate_heading(
            (data.magnetic_field.x, data.magnetic_field.y, data.magnetic_field.z))
        self._magnetic_orientation = (raw_heading + (pi + pi / 2)) % (2 * pi)

    def motor_steps_left_data_callback(self, data):
        """Motor data callback handler."""
        self._steps_left = data.data

    def motor_steps_right_data_callback(self, data):
        """Motor data callback handler."""
        self._steps_right = data.data

    def calculate_hybrid_position(self):
        """Calculate position using both Odometry and the IMU."""
        steps_left = self._steps_left
        steps_right = self._steps_right
        previous_steps_left = self._previous_steps_left
        previous_steps_right = self._previous_steps_right
        previous_theta = self._previous_theta
        current_theta = None

        if self._magnetic_orientation is not None:
            current_theta = self._magnetic_orientation

        has_magnetic_data = self._magnetic_orientation is not None
        has_theta = self._previous_theta is not None
        has_steps = previous_steps_left is not None and previous_steps_right is not None

        if has_steps and has_theta and has_magnetic_data:
            delta_left = (steps_left - previous_steps_left) * MOTOR_STEP_DISTANCE
            delta_right = (steps_right - previous_steps_right) * MOTOR_STEP_DISTANCE

            delta_theta = current_theta - previous_theta
            delta_steps = (delta_right + delta_left) / 2.0

            delta_estimate_x = delta_steps * cos(previous_theta + delta_theta / 2.0)
            delta_estimate_y = delta_steps * sin(previous_theta + delta_theta / 2.0)

            self._current_xyz = (self._current_xyz[0] + delta_estimate_x,
                                 self._current_xyz[1] + delta_estimate_y, self._current_xyz[2])

        self._previous_steps_left = steps_left
        self._previous_steps_right = steps_right
        self._previous_theta = current_theta

    def run(self):
        """Run the transform server."""
        if self._use_imu:
            rospy.Subscriber("navigation/spatial/imu/imu", Imu, self.imu_data_callback)
        if not self._use_hybrid_position or not self._use_imu:
            rospy.Subscriber("navigation/motors/odometry", Odometry,
                             self.motor_odometry_data_callback)
        if self._use_hybrid_position:
            rospy.Subscriber("navigation/motors/real_steps_left", Int64,
                             self.motor_steps_left_data_callback)
            rospy.Subscriber("navigation/motors/real_steps_right", Int64,
                             self.motor_steps_right_data_callback)
            rospy.Subscriber("navigation/spatial/imu/magnetic", MagneticField,
                             self.magnetometer_callback)

        while not rospy.is_shutdown():
            if self._publish_static_transforms:
                self.send_static_transforms()
            if self._use_hybrid_position:
                self.calculate_hybrid_position()
            self.send_pi_puck_transform()
            self._rate.sleep()


if __name__ == '__main__':
    tf_server = PiPuckTransformServer()
    tf_server.run()
